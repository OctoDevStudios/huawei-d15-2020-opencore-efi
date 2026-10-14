/*
 * pam_gxfp — fingerprint authentication (GXFP51A0) through the gxfpd daemon.
 *
 *   auth  sufficient  /usr/local/lib/pam/pam_gxfp.so  [timeout=10] [skip_if_password]
 *
 * Returns PAM_SUCCESS on MATCH, otherwise PAM_AUTHINFO_UNAVAIL / PAM_AUTH_ERR
 * so the stack falls through to the password. Skipped in SSH sessions.
 */
#include <security/pam_appl.h>
#include <security/pam_modules.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define GXFPD_SOCK "/var/run/gxfpd.sock"

static void
info (pam_handle_t *ph, const char *msg)
{
    const struct pam_conv *conv = NULL;
    if (pam_get_item (ph, PAM_CONV, (const void **) &conv) != PAM_SUCCESS || !conv || !conv->conv)
        return;
    struct pam_message m = { PAM_TEXT_INFO, (char *) msg };
    const struct pam_message *mp = &m;
    struct pam_response *resp = NULL;
    conv->conv (1, &mp, &resp, conv->appdata_ptr);
    if (resp) { free (resp->resp); free (resp); }
}

static int
read_line (int fd, char *buf, size_t cap)
{
    size_t n = 0;
    while (n + 1 < cap) {
        ssize_t r = read (fd, buf + n, 1);
        if (r <= 0) return -1;
        if (buf[n] == '\n') break;
        n++;
    }
    buf[n] = 0;
    return (int) n;
}

PAM_EXTERN int
pam_sm_authenticate (pam_handle_t *ph, int flags, int argc, const char **argv)
{
    (void) flags;
    const char *user = NULL;
    int timeout = 10;

    for (int i = 0; i < argc; i++)
        if (!strncmp (argv[i], "timeout=", 8)) timeout = atoi (argv[i] + 8);

    int skip_if_password = 0;
    for (int i = 0; i < argc; i++)
        if (!strcmp (argv[i], "skip_if_password")) skip_if_password = 1;

    if (getenv ("SSH_CONNECTION") || getenv ("SSH_TTY"))
        return PAM_AUTHINFO_UNAVAIL;

    /* Lock screen: a typed password goes straight to the next module;
     * empty field + Return = fingerprint. */
    if (skip_if_password) {
        const void *tok = NULL;
        if (pam_get_item (ph, PAM_AUTHTOK, &tok) == PAM_SUCCESS && tok && *(const char *) tok)
            return PAM_IGNORE;
    }
    if (pam_get_user (ph, &user, NULL) != PAM_SUCCESS || !user || !*user)
        return PAM_AUTHINFO_UNAVAIL;

    int s = socket (AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un a = { .sun_family = AF_UNIX };
    strlcpy (a.sun_path, GXFPD_SOCK, sizeof a.sun_path);
    if (s < 0 || connect (s, (struct sockaddr *) &a, sizeof a) != 0) {
        if (s >= 0) close (s);
        return PAM_AUTHINFO_UNAVAIL;
    }
    struct timeval tv = { timeout + 10, 0 };
    setsockopt (s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    char req[200], line[128];
    snprintf (req, sizeof req, "VERIFY %s %d\n", user, timeout);
    if (write (s, req, strlen (req)) < 0) { close (s); return PAM_AUTHINFO_UNAVAIL; }

    int rc = PAM_AUTHINFO_UNAVAIL;
    while (read_line (s, line, sizeof line) >= 0) {
        if (!strcmp (line, "READY"))
            info (ph, "Touch the power button (or wait to use your password)…");
        else if (!strncmp (line, "RETRY", 5))
            info (ph, "Fingerprint not recognized, try again.");
        else if (!strcmp (line, "MATCH")) { rc = PAM_SUCCESS; break; }
        else if (!strcmp (line, "NOMATCH")) { rc = PAM_AUTH_ERR; break; }
        else if (!strcmp (line, "CAPTURING")) continue;
        else break;   /* TIMEOUT, NOTEMPLATE, ERROR */
    }
    close (s);
    return rc;
}

PAM_EXTERN int
pam_sm_setcred (pam_handle_t *ph, int flags, int argc, const char **argv)
{
    (void) ph; (void) flags; (void) argc; (void) argv;
    return PAM_SUCCESS;
}
