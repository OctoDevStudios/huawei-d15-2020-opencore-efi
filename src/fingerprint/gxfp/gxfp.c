/*
 * gxfp — macOS GXFP51A0 driver (port of the libfprint goodix51a0 driver by
 * Benjamin Allègre / GodsQuantum, LGPL-2.1+), on top of MateBookFP.kext.
 *
 *   gxfp init                      reset + PMK + TLS + calibration (diagnostic)
 *   gxfp capture [out.pgm]         wait for a finger, capture, save the image
 *   gxfp enroll <name>             enroll a finger (GX_ENROLL_STAGES presses)
 *   gxfp verify <name> [tries]     verify; exit code 0 = match
 *   gxfp list | delete <name>
 *
 * Templates live in /var/db/gxfp (root, 0700). Option -v / -vv: logs.
 */
#include "src/core_a.c"
#include "src/core_b.c"
#include "src/core_c.c"

#include <dirent.h>
#include <getopt.h>
#include <signal.h>
#include <openssl/sha.h>

int gx_log_level = 0;
int gx_mac_open(void);

#define GXFP_DB_DIR "/var/db/gxfp"
#define GXFP_MAGIC  0x31545847u   /* "GXT1" */

/* ---------------- transport (replaces spidev/udev) ---------------- */

static void
gx_transport_close (FpiDeviceGoodix51A0 *self)
{
  self->spi_fd = -1;
  self->irq_fd = -1;
}

static gboolean
gx_transport_open (FpDevice *dev, GError **error)
{
  FpiDeviceGoodix51A0 *self = FPI_DEVICE_GOODIX51A0 (dev);

  if (gx_mac_open () != 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_FOUND,
                           "MateBookFP.kext unavailable");
      return FALSE;
    }
  self->spi_fd = 3;           /* token: gx_mac_ioctl ignores the fd */
  self->irq_fd = gx51_open_irq_gpio48 ();
  return self->irq_fd >= 0;
}

/* ---------------- matching (taken from gx_score_probe_against_print) ---------------- */

static int
gx_score_probe_against_views (GPtrArray *views, const GxSiftFeatures *probe)
{
  int best = 0;

  if (!probe || !views)
    return 0;

  g_autofree int *scores = g_new0 (int, views->len ? views->len : 1);
  for (guint i = 0; i < views->len; i++)
    {
      scores[i] = gx_sift_match (probe, g_ptr_array_index (views, i));
      if (scores[i] > best)
        best = scores[i];
    }
  fp_info ("best view score = %d (threshold %d, %u views)", best, GX_MATCH_THRESHOLD, views->len);

  /* Strict pixel fallback (rel71.28): only just below the threshold,
   * never merging scores across views. */
  if (best >= GX_PIXEL_RESCUE_CANDIDATE_BEST_MIN && best < GX_MATCH_THRESHOLD)
    for (guint i = 0; i < views->len; i++)
      if (gx_pixel_rescue_view_pass (probe, g_ptr_array_index (views, i), scores[i],
                                     NULL, NULL, NULL, NULL))
        {
          fp_info ("pixel fallback on view %u", i);
          return GX_MATCH_THRESHOLD;
        }
  return best;
}

/* ---------------- template storage ---------------- */

static gchar *
tmpl_path (const char *name)
{
  for (const char *p = name; *p; p++)
    if (!(*p == '-' || *p == '_' || *p == '.' || (*p >= '0' && *p <= '9') ||
          (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z')) || name[0] == '.')
      return NULL;
  return g_strdup_printf ("%s/%s.gxt", GXFP_DB_DIR, name);
}

static gboolean
tmpl_save (const char *name, GPtrArray *views)
{
  g_autofree gchar *path = tmpl_path (name);
  if (!path)
    return FALSE;
  g_mkdir_with_parents (GXFP_DB_DIR, 0700);
  chmod (GXFP_DB_DIR, 0700);

  GByteArray *out = g_byte_array_new ();
  guint32 hdr[2] = { GXFP_MAGIC, views->len };
  g_byte_array_append (out, (guint8 *) hdr, sizeof hdr);
  for (guint i = 0; i < views->len; i++)
    {
      GByteArray *ba = gx_sift_serialize (g_ptr_array_index (views, i));
      guint32 n = ba->len;
      g_byte_array_append (out, (guint8 *) &n, 4);
      g_byte_array_append (out, ba->data, ba->len);
      g_byte_array_unref (ba);
    }
  gboolean ok = g_file_set_contents_full (path, (gchar *) out->data, out->len, 0, 0600, NULL);
  g_byte_array_unref (out);
  return ok;
}

static GPtrArray *
tmpl_load (const char *name)
{
  g_autofree gchar *path = tmpl_path (name);
  g_autofree gchar *data = NULL;
  gsize len = 0;

  if (!path || !g_file_get_contents (path, &data, &len, NULL) || len < 8)
    return NULL;
  const guint32 *hdr = (const guint32 *) data;
  if (hdr[0] != GXFP_MAGIC)
    return NULL;
  GPtrArray *views = g_ptr_array_new_with_free_func ((GDestroyNotify) gx_sift_free);
  gsize off = 8;
  for (guint i = 0; i < hdr[1] && off + 4 <= len; i++)
    {
      guint32 n;
      memcpy (&n, data + off, 4);
      off += 4;
      if (off + n > len)
        break;
      GxSiftFeatures *f = gx_sift_deserialize ((const guint8 *) data + off, n);
      if (f)
        g_ptr_array_add (views, f);
      off += n;
    }
  return views;
}

/* ---------------- session ---------------- */

static FpiDeviceGoodix51A0 *
dev_open (void)
{
  FpiDeviceGoodix51A0 *self = g_new0 (FpiDeviceGoodix51A0, 1);
  GError *err = NULL;

  self->spi_fd = -1;
  self->irq_fd = -1;
  gx_gpio_reset (self);                       /* cold boundary before open */
  if (!gx_transport_open (self, &err))
    {
      fp_warn ("%s", err ? err->message : "transport");
      return NULL;
    }
  if (!gx_cold_prepare (self))
    {
      fp_warn ("cold preparation failed (PMK/TLS/calibration)");
      return NULL;
    }
  return self;
}

static volatile sig_atomic_t stop_req;
static void on_sig (int s) { (void) s; stop_req = 1; }

/* Wait for a finger on the sensor (like gx_poll_on: 100 ms, ~30 s max). */
static gboolean
wait_finger (FpiDeviceGoodix51A0 *self, int timeout_s)
{
  for (int i = 0; i < timeout_s * 10 && !stop_req; i++)
    {
      if (gx_finger_present (self))
        return TRUE;
      g_usleep (100000);
    }
  return FALSE;
}

static void
wait_lift (FpiDeviceGoodix51A0 *self)
{
  for (int i = 0; i < 100 && !stop_req; i++)
    {
      if (!gx_finger_present (self))
        return;
      g_usleep (100000);
    }
}

static GxSiftFeatures *
capture_press (FpiDeviceGoodix51A0 *self, guint16 *px_out)
{
  guint16 px[GOODIX_IMG_PIXELS];

  if (!gx_capture_frame (self, px, FALSE))
    return NULL;
  if (px_out)
    memcpy (px_out, px, sizeof px);
  GxSiftFeatures *f = gx_features_from_pixels (self, px);
  if (f && gx_sift_keypoints (f) < GX_MIN_CAPTURE_KEYPOINTS)
    {
      fp_info ("image rejected: %u keypoints (< %u)", gx_sift_keypoints (f), GX_MIN_CAPTURE_KEYPOINTS);
      gx_sift_free (f);
      return NULL;
    }
  return f;
}

static void
save_pgm (const char *path, FpiDeviceGoodix51A0 *self, const guint16 *px)
{
  g_autofree double *img = gx_preprocess (self, px);
  double mx = 1;
  for (int i = 0; i < GOODIX_IMG_PIXELS; i++)
    if (img[i] > mx) mx = img[i];
  FILE *f = fopen (path, "wb");
  if (!f) return;
  fprintf (f, "P5\n%d %d\n255\n", GOODIX_IMG_WIDTH, GOODIX_IMG_HEIGHT);
  for (int i = 0; i < GOODIX_IMG_PIXELS; i++)
    fputc ((int) (255.0 * img[i] / mx), f);
  fclose (f);
}

/* ---------------- commands ---------------- */

static int
cmd_enroll (const char *name)
{
  if (!tmpl_path (name)) { fprintf (stderr, "invalid name\n"); return 2; }
  FpiDeviceGoodix51A0 *self = dev_open ();
  if (!self) return 1;
  GPtrArray *views = g_ptr_array_new_with_free_func ((GDestroyNotify) gx_sift_free);

  printf ("Enrolling \"%s\": %d presses. Place your finger, lift it at each beep, vary the position slightly.\n",
          name, GX_ENROLL_STAGES);
  int stage = 0, fails = 0;
  while (stage < GX_ENROLL_STAGES && !stop_req && fails < 15)
    {
      printf ("  [%2d/%d] place your finger…\n", stage + 1, GX_ENROLL_STAGES);
      fflush (stdout);
      if (!wait_finger (self, 30)) { printf ("  no finger (30 s)\n"); break; }
      GxSiftFeatures *f = capture_press (self, NULL);
      if (!f)
        { printf ("  unusable image, try again (hold still ~1 s)\n"); fails++; }
      else
        {
          int best = 0; guint weak = 0;
          if (gx_enroll_view_connected (views, f, stage, &best, &weak))
            {
              g_ptr_array_add (views, f);
              stage++;
              printf ("  ok (%u keypoints, link %d)  — lift your finger\a\n", gx_sift_keypoints (f), best);
            }
          else
            {
              gx_sift_free (f);
              fails++;
              printf ("  view too different from the previous ones (link %d), re-center your finger\n", best);
            }
        }
      fflush (stdout);
      wait_lift (self);
    }
  if (stage < GX_ENROLL_STAGES)
    {
      printf ("Enrollment incomplete (%d/%d) — nothing saved.\n", stage, GX_ENROLL_STAGES);
      return 1;
    }
  if (!tmpl_save (name, views)) { printf ("failed to write the template\n"); return 1; }
  printf ("Template \"%s\" saved (%u views).\n", name, views->len);
  return 0;
}

static int
cmd_verify (const char *name, int tries)
{
  g_autoptr (GPtrArray) views = tmpl_load (name);
  if (!views || !views->len) { fprintf (stderr, "template \"%s\" not found\n", name); return 2; }
  FpiDeviceGoodix51A0 *self = dev_open ();
  if (!self) return 3;

  for (int t = 1; t <= tries && !stop_req; t++)
    {
      printf ("Place your finger (try %d/%d)…\n", t, tries);
      fflush (stdout);
      if (!wait_finger (self, 30)) { printf ("no finger\n"); return 1; }
      GxSiftFeatures *probe = capture_press (self, NULL);
      int score = probe ? gx_score_probe_against_views (views, probe) : 0;
      if (probe) gx_sift_free (probe);
      printf ("score %d (threshold %d)\n", score, GX_MATCH_THRESHOLD);
      if (score >= GX_MATCH_THRESHOLD) { printf ("MATCH\n"); return 0; }
      wait_lift (self);
    }
  printf ("NO MATCH\n");
  return 1;
}

static int
cmd_capture (const char *out)
{
  FpiDeviceGoodix51A0 *self = dev_open ();
  if (!self) return 1;
  printf ("Place your finger…\n");
  fflush (stdout);
  if (!wait_finger (self, 30)) { printf ("no finger\n"); return 1; }
  guint16 px[GOODIX_IMG_PIXELS];
  GxSiftFeatures *f = capture_press (self, px);
  printf ("capture: %s, %u keypoints\n", f ? "ok" : "rejected", f ? gx_sift_keypoints (f) : 0);
  save_pgm (out, self, px);
  printf ("image: %s\n", out);
  return f ? 0 : 1;
}


/* ---------------- daemon: sensor kept warm, root socket ---------------- */

#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>

#define GXFPD_SOCK "/var/run/gxfpd.sock"

static FpiDeviceGoodix51A0 *D;

static gboolean
d_ready (void)
{
  if (D && D->production_ready && !gx_warm_crossed_sleep (D))
    {
      if (gx_background_is_fresh (D) || gx_warm_validate (D))
        return gx_wakeup_mcu (D);
      fp_warn ("warm context invalid -> cold preparation");
    }
  if (D)
    gx_warm_abandon (D);
  D = dev_open ();
  return D && gx_wakeup_mcu (D);
}

/* All views of the <user>.gxt and <user>-*.gxt templates */
static GPtrArray *
load_user_views (const char *user)
{
  GPtrArray *all = g_ptr_array_new_with_free_func ((GDestroyNotify) gx_sift_free);
  DIR *d = opendir (GXFP_DB_DIR);
  struct dirent *e;
  size_t ul = strlen (user);
  while (d && (e = readdir (d)))
    {
      size_t n = strlen (e->d_name);
      if (n < 5 || strcmp (e->d_name + n - 4, ".gxt") || strncmp (e->d_name, user, ul) ||
          !(n - 4 == ul || e->d_name[ul] == '-'))
        continue;
      char name[256];
      snprintf (name, sizeof name, "%.*s", (int) (n - 4), e->d_name);
      GPtrArray *v = tmpl_load (name);
      if (!v) continue;
      for (guint i = 0; i < v->len; i++) g_ptr_array_add (all, v->pdata[i]);
      v->free_func = NULL;
      g_ptr_array_unref (v);
    }
  if (d) closedir (d);
  return all;
}

static gboolean
client_gone (int fd)
{
  struct pollfd p = { fd, POLLIN, 0 };
  char c;
  if (poll (&p, 1, 0) > 0 && (p.revents & (POLLHUP | POLLERR | POLLIN)))
    return recv (fd, &c, 1, MSG_PEEK | MSG_DONTWAIT) <= 0;
  return FALSE;
}

static void
say (int fd, const char *m) { (void) !write (fd, m, strlen (m)); }

static void
handle (int fd)
{
  char buf[256] = { 0 }, user[128] = { 0 };
  int timeout = 15;
  ssize_t n = read (fd, buf, sizeof buf - 1);
  if (n <= 0 || sscanf (buf, "VERIFY %127s %d", user, &timeout) < 1 || !tmpl_path (user))
    { say (fd, "ERROR request\n"); return; }

  g_autoptr (GPtrArray) views = load_user_views (user);
  if (!views->len) { say (fd, "NOTEMPLATE\n"); return; }
  if (!d_ready ()) { say (fd, "ERROR sensor\n"); return; }
  say (fd, "READY\n");

  gint64 end = g_get_monotonic_time () + (gint64) timeout * G_USEC_PER_SEC;
  int presses = 0;
  while (g_get_monotonic_time () < end && presses < GX_VERIFY_MAX_ATTEMPTS_CLI)
    {
      if (client_gone (fd)) { fp_info ("client gone"); return; }
      if (!gx_finger_present (D)) { g_usleep (100000); continue; }
      say (fd, "CAPTURING\n");
      GxSiftFeatures *probe = capture_press (D, NULL);
      int score = probe ? gx_score_probe_against_views (views, probe) : 0;
      if (probe) gx_sift_free (probe);
      if (!probe && D->capture_recovery_pending)
        {
          fp_warn ("transport out of sync, recovering");
          if (!d_ready ()) { say (fd, "ERROR sensor\n"); return; }
          continue;
        }
      presses++;
      fp_warn ("verify %s: press %d score %d", user, presses, score);
      if (score >= GX_MATCH_THRESHOLD) { say (fd, "MATCH\n"); wait_lift (D); return; }
      say (fd, probe ? "RETRY\n" : "RETRY quality\n");
      wait_lift (D);
    }
  say (fd, presses ? "NOMATCH\n" : "TIMEOUT\n");
}

#include "unlock.c"

static int
cmd_daemon (void)
{
  signal (SIGPIPE, SIG_IGN);
  unlink (GXFPD_SOCK);
  int s = socket (AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un a = { .sun_family = AF_UNIX };
  strlcpy (a.sun_path, GXFPD_SOCK, sizeof a.sun_path);
  mode_t old = umask (077);
  if (bind (s, (struct sockaddr *) &a, sizeof a) || listen (s, 4)) { perror ("bind"); return 1; }
  unlink (GXFPD_AGENT_SOCK);
  int sa = socket (AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un aa = { .sun_family = AF_UNIX };
  strlcpy (aa.sun_path, GXFPD_AGENT_SOCK, sizeof aa.sun_path);
  if (bind (sa, (struct sockaddr *) &aa, sizeof aa) || listen (sa, 4)) { perror ("bind agent"); return 1; }
  umask (old);
  chmod (GXFPD_SOCK, 0600);
  chmod (GXFPD_AGENT_SOCK, 0666);   /* uid/path/lock-state filtering is done in handle_agent */
  fp_warn ("gxfpd ready on %s", GXFPD_SOCK);
  d_ready ();                                  /* warm-up: TLS + key (background refreshed on demand) */

  while (!stop_req)
    {
      /* Idle: no sensor activity (battery). The background frame is refreshed
       * on demand by d_ready() when the screen locks or a verify starts. */
      struct pollfd pl[2] = { { s, POLLIN, 0 }, { sa, POLLIN, 0 } };
      if (poll (pl, 2, -1) <= 0)
        continue;
      if (pl[1].revents & POLLIN)
        {
          int ca = accept (sa, NULL, NULL);
          if (ca >= 0) { handle_agent (ca); close (ca); }
          continue;
        }
      int c = accept (s, NULL, NULL);
      if (c < 0) { if (errno == EINTR) continue; break; }
      uid_t uid; gid_t gid;
      if (getpeereid (c, &uid, &gid) || uid != 0) { say (c, "ERROR root required\n"); close (c); continue; }
      handle (c);
      close (c);
    }
  unlink (GXFPD_SOCK);
  unlink (GXFPD_AGENT_SOCK);
  return 0;
}

int
main (int argc, char **argv)
{
  int c;
  while ((c = getopt (argc, argv, "v")) != -1)
    if (c == 'v') gx_log_level++;
  argc -= optind; argv += optind;
  if (argc < 1)
    {
      fprintf (stderr, "usage: gxfp [-v] init | capture [out.pgm] | enroll <name> | verify <name> [tries] | list | delete <name>\n");
      return 2;
    }
  if (!strcmp (argv[0], "agent"))
    { signal (SIGTERM, on_sig); return cmd_agent (); }
  if (geteuid () != 0) { fprintf (stderr, "root required (sudo)\n"); return 2; }
  signal (SIGINT, on_sig);
  signal (SIGTERM, on_sig);

  const char *cmd = argv[0];
  if (!strcmp (cmd, "init"))
    return dev_open () ? (printf ("sensor ready (TLS + calibration OK)\n"), 0) : 1;
  if (!strcmp (cmd, "capture"))
    return cmd_capture (argc > 1 ? argv[1] : "/tmp/gxfp.pgm");
  if (!strcmp (cmd, "enroll") && argc > 1)
    return cmd_enroll (argv[1]);
  if (!strcmp (cmd, "verify") && argc > 1)
    return cmd_verify (argv[1], argc > 2 ? atoi (argv[2]) : GX_VERIFY_MAX_ATTEMPTS_CLI);
  if (!strcmp (cmd, "pmk"))
    {
      /* Diagnostic: re-reads the key several times, prints only SHA-256[0..7]. */
      FpiDeviceGoodix51A0 *self = g_new0 (FpiDeviceGoodix51A0, 1);
      self->timing_scale = 100;
      gx_gpio_reset (self);
      if (!gx_transport_open (self, NULL)) return 1;
      for (int i = 0; i < (argc > 1 ? atoi (argv[1]) : 3); i++)
        {
          guint8 pmk[GXFP_FACTORY_PMK_LEN], h[32];
          enum gxfp_factory_staging_diag diag = 0;
          gx_gpio_reset (self);
          gboolean s1 = gxfp_factory_load_pmk_from_single_staging (gx_factory_staging_read_cb, self, pmk);
          if (s1) { SHA256 (pmk, sizeof pmk, h); printf ("single #%d: OK sha=%02x%02x%02x%02x%02x%02x%02x%02x\n", i, h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]); }
          else printf ("single #%d: failed\n", i);
          gx_gpio_reset (self);
          gboolean s2 = gxfp_factory_load_pmk_from_staging_diag (gx_factory_staging_read_cb, self, pmk, &diag);
          if (s2) { SHA256 (pmk, sizeof pmk, h); printf ("pair   #%d: OK sha=%02x%02x%02x%02x%02x%02x%02x%02x\n", i, h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]); }
          else printf ("pair   #%d: %s\n", i, gxfp_factory_staging_diag_name (diag));
          OPENSSL_cleanse (pmk, sizeof pmk);
        }
      return 0;
    }
  if (!strcmp (cmd, "setpw") && argc > 1)
    return cmd_setpw (argv[1]);
  if (!strcmp (cmd, "daemon"))
    return cmd_daemon ();
  if (!strcmp (cmd, "list"))
    {
      DIR *d = opendir (GXFP_DB_DIR);
      struct dirent *e;
      while (d && (e = readdir (d)))
        if (strstr (e->d_name, ".gxt")) printf ("%.*s\n", (int) (strlen (e->d_name) - 4), e->d_name);
      if (d) closedir (d);
      return 0;
    }
  if (!strcmp (cmd, "delete") && argc > 1)
    {
      g_autofree gchar *p = tmpl_path (argv[1]);
      return p && unlink (p) == 0 ? 0 : 1;
    }
  fprintf (stderr, "unknown command\n");
  return 2;
}
