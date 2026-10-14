/*
 * unlock.c — fingerprint screen unlock (included from gxfp.c).
 *
 * macOS does not call PAM for an empty password field on the lock screen, so:
 *   - `gxfp setpw <user>` (root): verifies the password (OpenDirectory) and stores it
 *     in the System keychain (encrypted, root only), service "gxfp-unlock";
 *   - the daemon exposes /var/run/gxfpd-agent.sock: an UNLOCK request is accepted only if
 *     the client is /usr/local/bin/gxfp-agent, run by the console user, AND that
 *     user's screen is locked. On a fingerprint match -> "PW <password>";
 *   - `gxfp-agent` (LaunchAgent, user session) watches the lock state and types
 *     the password + Return into loginwindow (Accessibility permission).
 * The boot-time login window is deliberately not handled (same as Touch ID after a restart).
 */
#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>
#include <OpenDirectory/OpenDirectory.h>
#include <Security/Security.h>
#include <libproc.h>
#include <pwd.h>
#include <readpassphrase.h>
#include <sys/ucred.h>

#define GXFPD_AGENT_SOCK "/var/run/gxfpd-agent.sock"
#define GXFP_AGENT_BIN   "/usr/local/bin/gxfp-agent"
#define KC_SERVICE       "gxfp-unlock"
#define KC_SYSTEM        "/Library/Keychains/System.keychain"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

/* ---------------- password: verification + System keychain ---------------- */

static gboolean
pw_verify (const char *user, const char *pw)
{
  CFErrorRef err = NULL;
  gboolean ok = FALSE;
  ODNodeRef node = ODNodeCreateWithNodeType (kCFAllocatorDefault, kODSessionDefault,
                                             kODNodeTypeLocalNodes, &err);
  CFStringRef u = CFStringCreateWithCString (NULL, user, kCFStringEncodingUTF8);
  CFStringRef p = CFStringCreateWithCString (NULL, pw, kCFStringEncodingUTF8);
  ODRecordRef rec = node ? ODNodeCopyRecord (node, kODRecordTypeUsers, u, NULL, &err) : NULL;
  if (rec)
    ok = ODRecordVerifyPassword (rec, p, &err);
  if (rec) CFRelease (rec);
  if (node) CFRelease (node);
  CFRelease (u);
  CFRelease (p);
  if (err) CFRelease (err);
  return ok;
}

static SecKeychainRef
kc_system (void)
{
  SecKeychainRef kc = NULL;
  if (SecKeychainOpen (KC_SYSTEM, &kc) != errSecSuccess)
    return NULL;
  return kc;
}

static void
pw_delete (SecKeychainRef kc, const char *user)
{
  SecKeychainItemRef item = NULL;
  if (SecKeychainFindGenericPassword (kc, strlen (KC_SERVICE), KC_SERVICE, strlen (user), user,
                                      NULL, NULL, &item) == errSecSuccess && item)
    {
      SecKeychainItemDelete (item);
      CFRelease (item);
    }
}

static gboolean
pw_store (const char *user, const char *pw)
{
  SecKeychainRef kc = kc_system ();
  SecAccessRef access = NULL;
  SecKeychainItemRef item = NULL;
  gboolean ok = FALSE;

  if (!kc)
    return FALSE;
  pw_delete (kc, user);

  /* Access without an app list: the System keychain can only be decrypted by root,
   * and the ad-hoc signed binary changes signature on every rebuild. */
  if (SecAccessCreate (CFSTR ("gxfp unlock"), NULL, &access) == errSecSuccess)
    {
      CFArrayRef acls = NULL;
      if (SecAccessCopyACLList (access, &acls) == errSecSuccess)
        {
          for (CFIndex i = 0; i < CFArrayGetCount (acls); i++)
            {
              SecACLRef acl = (SecACLRef) CFArrayGetValueAtIndex (acls, i);
              CFArrayRef apps = NULL; CFStringRef desc = NULL;
              SecKeychainPromptSelector sel = 0;
              if (SecACLCopyContents (acl, &apps, &desc, &sel) == errSecSuccess)
                {
                  SecACLSetContents (acl, NULL, desc ? desc : CFSTR ("gxfp"), 0);
                  if (apps) CFRelease (apps);
                  if (desc) CFRelease (desc);
                }
            }
          CFRelease (acls);
        }
    }

  SecKeychainAttribute attrs[] = {
    { kSecServiceItemAttr, (UInt32) strlen (KC_SERVICE), (void *) KC_SERVICE },
    { kSecAccountItemAttr, (UInt32) strlen (user), (void *) user },
    { kSecLabelItemAttr, (UInt32) strlen (KC_SERVICE), (void *) KC_SERVICE },
  };
  SecKeychainAttributeList list = { 3, attrs };
  ok = SecKeychainItemCreateFromContent (kSecGenericPasswordItemClass, &list,
                                         (UInt32) strlen (pw), pw, kc, access, &item) == errSecSuccess;
  if (item) CFRelease (item);
  if (access) CFRelease (access);
  CFRelease (kc);
  return ok;
}

/* Returns a malloc'd buffer (to be wiped + freed) or NULL. */
static char *
pw_load (const char *user)
{
  SecKeychainRef kc = kc_system ();
  UInt32 len = 0;
  void *data = NULL;
  char *out = NULL;

  if (!kc)
    return NULL;
  if (SecKeychainFindGenericPassword (kc, strlen (KC_SERVICE), KC_SERVICE, strlen (user), user,
                                      &len, &data, NULL) == errSecSuccess && data)
    {
      out = g_malloc0 (len + 1);
      memcpy (out, data, len);
      memset (data, 0, len);
      SecKeychainItemFreeContent (NULL, data);
    }
  CFRelease (kc);
  return out;
}

static void
pw_free (char *pw)
{
  if (pw) { memset_s (pw, strlen (pw), 0, strlen (pw)); free (pw); }
}

static int
cmd_setpw (const char *user)
{
  char pw[256];
  struct passwd *p = getpwnam (user);
  if (!p) { fprintf (stderr, "unknown user\n"); return 2; }
  if (!readpassphrase ("Login password (for fingerprint unlock): ",
                       pw, sizeof pw, RPP_REQUIRE_TTY))
    return 2;
  int rc = 1;
  if (!pw_verify (user, pw))
    printf ("Wrong password, nothing saved.\n");
  else if (!pw_store (user, pw))
    printf ("Failed to write to the System keychain.\n");
  else
    { printf ("Password saved in the System keychain (root only).\n"); rc = 0; }
  memset_s (pw, sizeof pw, 0, sizeof pw);
  return rc;
}

/* ---------------- lock state (daemon side, root) ---------------- */

static int
screen_locked_for_uid (uid_t uid)
{
  int locked = -1;
  io_registry_entry_t root = IORegistryGetRootEntry (kIOMainPortDefault);
  CFArrayRef users = IORegistryEntryCreateCFProperty (root, CFSTR ("IOConsoleUsers"), NULL, 0);
  if (users && CFGetTypeID (users) == CFArrayGetTypeID ())
    for (CFIndex i = 0; i < CFArrayGetCount (users); i++)
      {
        CFDictionaryRef d = CFArrayGetValueAtIndex (users, i);
        CFNumberRef u = CFDictionaryGetValue (d, CFSTR ("kCGSSessionUserIDKey"));
        CFBooleanRef oc = CFDictionaryGetValue (d, CFSTR ("kCGSSessionOnConsoleKey"));
        int v = -1;
        if (!u || !CFNumberGetValue (u, kCFNumberIntType, &v) || (uid_t) v != uid)
          continue;
        if (oc && !CFBooleanGetValue (oc))
          continue;
        CFBooleanRef l = CFDictionaryGetValue (d, CFSTR ("CGSSessionScreenIsLocked"));
        locked = (l && CFBooleanGetValue (l)) ? 1 : 0;
      }
  if (users) CFRelease (users);
  return locked;
}

static gboolean
peer_is_installed_agent (int fd, uid_t *out_uid)
{
  uid_t uid; gid_t gid;
  pid_t pid = 0;
  socklen_t len = sizeof pid;
  char path[PROC_PIDPATHINFO_MAXSIZE] = { 0 };

  if (getpeereid (fd, &uid, &gid))
    return FALSE;
  if (getsockopt (fd, SOL_LOCAL, LOCAL_PEERPID, &pid, &len) || pid <= 0)
    return FALSE;
  if (proc_pidpath (pid, path, sizeof path) <= 0 || strcmp (path, GXFP_AGENT_BIN))
    return FALSE;
  *out_uid = uid;
  return TRUE;
}

/* Called by the daemon for a connection on the agent socket. */
static void
handle_agent (int fd)
{
  uid_t uid = 0;
  char buf[64] = { 0 };

  if (!peer_is_installed_agent (fd, &uid)) { say (fd, "ERROR client\n"); return; }
  if (read (fd, buf, sizeof buf - 1) <= 0) { say (fd, "ERROR request\n"); return; }
  if (uid == 0 || strncmp (buf, "UNLOCK", 6)) { say (fd, "ERROR request\n"); return; }
  struct passwd *p = getpwuid (uid);
  if (screen_locked_for_uid (uid) != 1) { say (fd, "NOTLOCKED\n"); return; }
  if (!p || !tmpl_path (p->pw_name)) { say (fd, "ERROR user\n"); return; }

  g_autoptr (GPtrArray) views = load_user_views (p->pw_name);
  if (!views->len) { say (fd, "NOTEMPLATE\n"); return; }
  char *pw = pw_load (p->pw_name);
  if (!pw) { say (fd, "NOPW\n"); return; }
  if (!d_ready ()) { pw_free (pw); say (fd, "ERROR sensor\n"); return; }
  say (fd, "READY\n");
  fp_warn ("agent: screen locked (%s), watching the sensor", p->pw_name);

#define STILL_WAITING() (screen_locked_for_uid (uid) == 1)
  while (!stop_req && !client_gone (fd) && STILL_WAITING ())
    {
      if (!gx_finger_present (D))
        {
          if (!gx_background_is_fresh (D) && !gx_warm_validate (D) && !d_ready ())
            break;
          g_usleep (250000);                    /* 4 Hz: battery vs responsiveness */
          continue;
        }
      GxSiftFeatures *probe = capture_press (D, NULL);
      int score = probe ? gx_score_probe_against_views (views, probe) : 0;
      if (probe) gx_sift_free (probe);
      if (!probe && D->capture_recovery_pending && !d_ready ())
        break;
      fp_warn ("agent: press score %d", score);
      if (score >= GX_MATCH_THRESHOLD && STILL_WAITING ())
        {
          char *line = g_strdup_printf ("PW %s\n", pw);
          say (fd, line);
          memset_s (line, strlen (line), 0, strlen (line));
          free (line);
          break;
        }
      say (fd, "RETRY\n");
      wait_lift (D);
    }
  pw_free (pw);
}

/* ---------------- agent (user session) ---------------- */

static int
agent_locked (void)
{
  CFDictionaryRef d = CGSessionCopyCurrentDictionary ();
  int locked = 0;
  if (d)
    {
      CFBooleanRef l = CFDictionaryGetValue (d, CFSTR ("CGSSessionScreenIsLocked"));
      locked = l && CFBooleanGetValue (l);
      CFRelease (d);
    }
  return locked;
}

static void
post_key (CGEventSourceRef src, CGKeyCode code, UniChar ch, gboolean use_char)
{
  for (int down = 1; down >= 0; down--)
    {
      CGEventRef e = CGEventCreateKeyboardEvent (src, code, down);
      if (use_char)
        CGEventKeyboardSetUnicodeString (e, 1, &ch);
      CGEventPost (kCGHIDEventTap, e);
      CFRelease (e);
      usleep (6000);
    }
}

static void
agent_type (const char *pw)
{
  CGEventSourceRef src = CGEventSourceCreate (kCGEventSourceStateHIDSystemState);
  CFStringRef s = CFStringCreateWithCString (NULL, pw, kCFStringEncodingUTF8);
  CFIndex n = s ? CFStringGetLength (s) : 0;

  post_key (src, 56, 0, FALSE);                 /* Shift: wakes the display / focus */
  usleep (150000);
  for (CFIndex i = 0; i < n; i++)
    post_key (src, 0, CFStringGetCharacterAtIndex (s, i), TRUE);
  post_key (src, 36, 0, FALSE);                 /* Return */
  if (s) CFRelease (s);
  if (src) CFRelease (src);
}

static int
cmd_agent (void)
{
  int failed = 0;

  signal (SIGPIPE, SIG_IGN);
  if (!AXIsProcessTrusted ())
    fp_warn ("agent: missing Accessibility permission for %s — typing will be ignored", GXFP_AGENT_BIN);

  while (!stop_req)
    {
      sleep (1);
      gboolean waiting = agent_locked ();
      if (!waiting) { failed = 0; continue; }
      if (failed) continue;

      int s = socket (AF_UNIX, SOCK_STREAM, 0);
      struct sockaddr_un a = { .sun_family = AF_UNIX };
      strlcpy (a.sun_path, GXFPD_AGENT_SOCK, sizeof a.sun_path);
      if (s < 0 || connect (s, (struct sockaddr *) &a, sizeof a)) { if (s >= 0) close (s); sleep (4); continue; }
      say (s, "UNLOCK\n");

      char line[512];
      size_t n = 0;
      for (;;)
        {
          struct pollfd p = { s, POLLIN, 0 };
          int r = poll (&p, 1, 1000);
          if (r == 0) { if (!agent_locked ()) break; continue; }
          char c;
          if (read (s, &c, 1) != 1) break;
          if (c != '\n' && n + 1 < sizeof line) { line[n++] = c; continue; }
          line[n] = 0;
          n = 0;
          if (!strncmp (line, "PW ", 3))
            {
              agent_type (line + 3);
              memset_s (line, sizeof line, 0, sizeof line);
              sleep (3);
              if (agent_locked ())
                { failed = 1; fp_warn ("agent: still locked after typing (password changed?)"); }
              break;
            }
          if (!strcmp (line, "NOPW") || !strcmp (line, "NOTEMPLATE") || !strncmp (line, "ERROR", 5))
            { fp_warn ("agent: %s", line); failed = 1; break; }
        }
      memset_s (line, sizeof line, 0, sizeof line);
      close (s);
    }
  return 0;
}

#pragma clang diagnostic pop
