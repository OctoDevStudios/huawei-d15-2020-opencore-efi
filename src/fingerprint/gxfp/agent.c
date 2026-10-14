/*
 * gxfp-agent — types the password into loginwindow when gxfpd matches the fingerprint
 * (lock screen of the logged-in user only; the boot-time login window is not handled).
 * Separate, stable binary: the Accessibility permission (tied to the code signature)
 * survives rebuilds of the gxfp driver.
 */
#include <ApplicationServices/ApplicationServices.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

typedef int gboolean;
#define TRUE 1
#define FALSE 0
#define GXFPD_AGENT_SOCK "/var/run/gxfpd-agent.sock"
#define GXFP_INSTALLED "/usr/local/bin/gxfp-agent"
#define fp_warn(...) do { fprintf (stderr, "[warn] " __VA_ARGS__); fputc ('\n', stderr); fflush (stderr); } while (0)
static volatile sig_atomic_t stop_req;
static void on_sig (int s) { (void) s; stop_req = 1; }
static void say (int fd, const char *m) { (void) !write (fd, m, strlen (m)); }

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
    fp_warn ("agent: missing Accessibility permission for %s — typing will be ignored", GXFP_INSTALLED);

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


int
main (int argc, char **argv)
{
  signal (SIGTERM, on_sig);
  (void) argc; (void) argv;
  return cmd_agent ();
}
