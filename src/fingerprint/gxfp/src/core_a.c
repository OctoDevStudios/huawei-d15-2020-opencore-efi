/*
 * Goodix GXFP51A0 SPI (TLS-PSK) driver for libfprint
 *
 * Copyright (C) 2026 Benjamin Allègre (https://github.com/Sigfrodr)
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Driver candidate for the GXFP51A0 SPI fingerprint sensor used in Huawei
 * MateBook systems. The chain is: split Milan SPI framing, target-specific
 * ChicagoHS configuration, TLS-PSK, and an 80x64 image path (capture
 * integration still under validation).
 * (six bytes carry four interleaved pixels) calibrated by subtracting a
 * background frame.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <https://www.gnu.org/licenses/>.
 */

#define FP_COMPONENT "goodix51a0"
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <errno.h>
#include <glib/gstdio.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <openssl/crypto.h>

#include "goodix51a0.h"
#include "goodix_sift.h"
#include "goodix_tls.h"

#include "gx51_transport.h"
#include "gx51_target.h"
#include "gx51_factory_pmk.h"
#include "gx51_image.h"
#include "gx51_capture_recipe.h"

G_STATIC_ASSERT (GOODIX_PSK_LEN == GXFP_FACTORY_PMK_LEN);


/* Frames averaged for background and finger: a noise/latency trade-off. */
#define GOODIX_BG_FRAMES     1
#define GOODIX_FINGER_FRAMES 3
/* Finger detection threshold on the standard deviation of the
 * background-minus-frame difference. */
#define GOODIX_FINGER_STD    150.0
/* Headroom above the ~10.6 kB target image record and the 22 kB regression case. */
#define GOODIX_RX_MAX        24000
#define GXFP_PMK_ACQUIRE_ATTEMPTS 4
#define GX_TARGET_ACK_ATTEMPTS 2
#define GX_TARGET_ACK_IRQ_TIMEOUT_MS 100
#define GX_PMK_CACHE_DIR  "/var/db/gxfp"
#define GX_PMK_CACHE_FILE "/var/db/gxfp/.pmk"

struct _FpiDeviceGoodix51A0
{


  int           spi_fd;
  int           irq_fd;


  /* TLS stack. Note the sensor is the client and we are the server. */
  GxTls        *tls;
  gboolean      tls_up;

  guint8        psk[GOODIX_PSK_LEN];
  gboolean      psk_ready;
  gboolean      psk_from_cache;
  struct gxfp_target_calibration target_cal;
  gboolean      have_target_cal;

  /* Reassembly buffer for TLS records, drained by the BIO recv callback. */
  guint8        tls_rx[GOODIX_RX_MAX];
  int           tls_rxlen, tls_rxpos;

  guint16      *bg_frame;    /* averaged background, NULL until calibrated */
  guint         poll_id;     /* finger-detection timeout source */
  int           poll_count;  /* poll iterations, bounded to avoid hanging */
  GPtrArray    *enroll_feats;/* descriptor sets accumulated during enrolment */
  int           fdt_base[GXFP_FDT_ZONE_COUNT]; /* FDT baseline (finger absent) */
  gboolean      have_fdt;    /* is fdt_base populated? */
  int           fdt_abs;     /* per-unit absolute floor, derived from baseline */
  int           timing_scale;/* session-local protocol-delay multiplier in % */
  int           capture_gap_scale; /* session-local capture-step gap % */
  guint         capture_clean_streak; /* consecutive complete frames without GET_IMAGE retry */
  gboolean      capture_retry_seen; /* current frame needed a GET_IMAGE/TLS retry */
  gboolean      capture_pacing_suppressed; /* lifecycle loss is not pacing evidence */
  gboolean      capture_recovery_pending; /* transport failed; rebuild between presses */
  gboolean      bg_dirty;    /* background taken with a finger down */
  gboolean      production_ready; /* TLS + fresh background + FDT prepared during open */
  gboolean      warm_valid;       /* TLS/background/FDT retained across fp_device close */
  gint64        warm_last_activity_us; /* monotonic time of last validated sensor activity */
  gint64        bg_last_refresh_us; /* monotonic time of last accepted finger-off ImageBase */
  gint64        warm_sleep_delta_us; /* CLOCK_BOOTTIME-MONOTONIC when warm state was armed */
  gboolean      warm_sleep_clock_valid; /* baseline validity; zero is a legitimate pre-first-suspend value */
  gboolean      force_cold_reset; /* suspend/lifecycle invalidation: never reuse stale sensor state */
};

G_DECLARE_FINAL_TYPE (FpiDeviceGoodix51A0, fpi_device_goodix51a0, FPI,
                      DEVICE_GOODIX51A0, FpDevice)
G_DEFINE_TYPE (FpiDeviceGoodix51A0, fpi_device_goodix51a0, FP_TYPE_DEVICE)

static guint gx_capture_gap_us (FpiDeviceGoodix51A0 *self);
static void gx_capture_transport_desync (FpiDeviceGoodix51A0 *self);
static void gx_capture_pacing_success (FpiDeviceGoodix51A0 *self);
static void gx_protocol_timing_miss (FpiDeviceGoodix51A0 *self,
                                     const gchar          *source);
static gboolean gx_warm_crossed_sleep (FpiDeviceGoodix51A0 *self);
static gboolean gx_warm_validate (FpiDeviceGoodix51A0 *self);
static void gx_warm_abandon (FpiDeviceGoodix51A0 *self);
static gboolean gx_cold_prepare (FpiDeviceGoodix51A0 *self);
static void gx_transport_close (FpiDeviceGoodix51A0 *self);
static gboolean gx_transport_open (FpDevice *dev, GError **error);


/* Fixed startup commands, byte-for-byte as validated against the sensor. */
static const guint8 GX_AMORCE[] = { 0, 5, 0, 0, 0, 0, 0, 0xA5 };
static const guint8 GX_ENABLE[] = { 0x96, 0x03, 0x00, 0x01, 0x00, 0x10 };
static const guint8 GX_REQTLS[] = { 0xd0, 0x03, 0x00, 0x00, 0x00, 0xd7 };
static const guint8 GX_TLS_OK[] = { 0xd4, 0x03, 0x00, 0x00, 0x00, 0xd3 };

#define GX_TLS_IRQ_POLL_MS 20
#define GX_TLS_IRQ_POLLS 200

static gboolean gx_read_fw_version_once (FpiDeviceGoodix51A0 *self,
                                           gchar *out,
                                           gsize cap);
#ifdef GXFP51A0_DEVELOPER
static void gx_dump_capture (FpiDeviceGoodix51A0 *self,
                             const guint16 *px);
#endif

/* ------------------------------------------------------------------ */
/*  SPI transport: one frame is exactly one transfer                   */
/* ------------------------------------------------------------------ */

/* Exact GXFP51A0 Windows 1.1.141.36 WakeupMCU (gfspi.dll SHA-256
 * 4fc5956220cc7bd86d002437e9cae5508d724763a430e4994ba7ce64144a6d59,
 * function 0x1800413e4): one raw four-byte SpbPeripheralWrite
 * {0x0f,0x00,0x00,0x0e}, then Sleep(5).  This is deliberately NOT a Milan
 * protocol frame and expects no sensor ACK. */
static gboolean
gx_wakeup_mcu (FpiDeviceGoodix51A0 *self)
{
  static const guint8 wake[4] = { 0x0f, 0x00, 0x00, 0x0e };
  struct spi_ioc_transfer xfer = { 0 };

  xfer.tx_buf = (unsigned long) wake;
  xfer.len = sizeof wake;
  if (ioctl (self->spi_fd, SPI_IOC_MESSAGE (1), &xfer) < 1)
    {
      fp_warn ("GXFP51A0 AUTH_TRACE WakeupMCU raw SPI write failed");
      return FALSE;
    }

  g_usleep (5000);
  fp_warn ("GXFP51A0 AUTH_TRACE WakeupMCU raw SPI write complete");
  return TRUE;
}

static gboolean
gx_write_frame (FpiDeviceGoodix51A0 *self, guint8 type,
                const guint8 *body, gsize n)
{
  struct gx51_outer hdr;
  struct spi_ioc_transfer xfer = { 0 };

  if (n == 0 || n > 0xffff ||
      gx51_make_outer (type, n, &hdr) < 0)
    return FALSE;

  xfer.tx_buf = (unsigned long) &hdr;
  xfer.len = sizeof hdr;
  if (ioctl (self->spi_fd, SPI_IOC_MESSAGE (1), &xfer) < 1)
    return FALSE;

  g_usleep (2000);

  memset (&xfer, 0, sizeof xfer);
  xfer.tx_buf = (unsigned long) body;
  xfer.len = n;
  return ioctl (self->spi_fd, SPI_IOC_MESSAGE (1), &xfer) >= 1;
}


/* Reads one frame: a 4-byte header then the body. Returns the body length,
 * or -1 on failure. */
static int
gx_read_frame (FpiDeviceGoodix51A0 *self, guint8 *out_type,
               guint8 *rx, gsize rx_cap)
{
  guint8 hdr[4] = { 0 };
  struct spi_ioc_transfer xfer = { 0 };
  guint16 n;

  if (self->irq_fd < 0 ||
      gx51_wait_irq_gpio48 (self->irq_fd, 1200) < 0)
    return -1;

  xfer.rx_buf = (unsigned long) hdr;
  xfer.len = sizeof hdr;
  if (ioctl (self->spi_fd, SPI_IOC_MESSAGE (1), &xfer) < 1)
    return -1;

  if (hdr[0] == 0xff && hdr[1] == 0xff &&
      hdr[2] == 0xff && hdr[3] == 0xff)
    return -1;

  if ((guint8) (hdr[0] + hdr[1] + hdr[2]) != hdr[3])
    return -1;

  if (hdr[0] != GOODIX_PKT_PLAIN && hdr[0] != GOODIX_PKT_TLS)
    return -1;

  *out_type = hdr[0];
  n = hdr[1] | (hdr[2] << 8);
  if (n == 0 || n > rx_cap)
    return -1;

  for (gsize off = 0; off < n; )
    {
      gsize chunk = gx51_read_chunk_size ((gsize) n - off);

      memset (&xfer, 0, sizeof xfer);
      xfer.rx_buf = (unsigned long) (rx + off);
      xfer.len = (guint32) chunk;
      if (ioctl (self->spi_fd, SPI_IOC_MESSAGE (1), &xfer) < 1)
        return -1;
      off += chunk;
    }

  return n;
}


/* Sends the preamble (A0), then a plaintext command (A0). */
static gboolean
gx_send_plain_raw (FpiDeviceGoodix51A0 *self, const guint8 *body, gsize n)
{
  if (!gx_write_frame (self, GOODIX_PKT_PLAIN, GX_AMORCE, sizeof GX_AMORCE))
    return FALSE;
  g_usleep (8000);
  return gx_write_frame (self, GOODIX_PKT_PLAIN, body, n);
}

/* Sends a cleartext command and drains every plain response. A TLS record
 * arriving instead is stashed for the BIO layer to pick up. */
static gboolean
gx_send_plain_drain (FpiDeviceGoodix51A0 *self, const guint8 *body, gsize n,
                     gboolean *out_ack_seen, gboolean *out_tls_seen)
{
/* IRQ-silence polls, 10 ms apart, that count as silence after a response.
 * This is the dominant cost of a capture: each command in the sequence
 * pays it. Cut it too short and frame boundaries desynchronise, leaving the
 * sensor wedged in a state only a hard reset plus an spidev rebind recovers.
 *
 * Measured over 6 to 8 captures per setting:
 *   6 -> 1319 ms, no failures       (original value)
 *   4 -> 1136 ms, no failures       <- kept
 *   3 -> 1062 ms, no failures
 *   2 -> total failure, sensor wedged
 * We keep 4 rather than 3: shipping the value right next to the cliff would
 * leave no margin for load or temperature variation. */
#ifndef GX_DRAIN_SILENCE
#define GX_DRAIN_SILENCE 4
#endif
#ifndef GX_DRAIN_IRQ_POLL_MS
#define GX_DRAIN_IRQ_POLL_MS 10
#endif
/* Windows completes FDT-down asynchronously: first ACK 0x32, then a separate
 * 20-byte FDT/touch report.  The generic post-reply silence budget is only
 * 40 ms, while the already-validated explicit FDT probe allows 100 ms for its
 * second response.  Give FDT-down the same second-response window before
 * advancing to GET_IMAGE. */
#ifndef GX_FDT_DOWN_SECOND_REPLY_TIMEOUT_MS
#define GX_FDT_DOWN_SECOND_REPLY_TIMEOUT_MS 100
#endif
#define GX_FDT_DOWN_SECOND_REPLY_POLLS   (GX_FDT_DOWN_SECOND_REPLY_TIMEOUT_MS / GX_DRAIN_IRQ_POLL_MS)
G_STATIC_ASSERT (GX_FDT_DOWN_SECOND_REPLY_TIMEOUT_MS % GX_DRAIN_IRQ_POLL_MS == 0);
G_STATIC_ASSERT (GX_FDT_DOWN_SECOND_REPLY_POLLS == 10);
#ifndef GX_GET_IMAGE_ATTEMPTS
#define GX_GET_IMAGE_ATTEMPTS 2
#endif
/*
 * Exact Windows GXFP51A0/ST411/14115 WBDI trace calls image SetMode (0x20)
 * with a 1000 ms ACK timeout.  Do not declare a command swallowed after the
 * old ~120 ms no-evidence window: on reference MateBook 13 that caused nearly every first
 * biometric GET_IMAGE to be replayed.  Replaying remains allowed exactly once,
 * but only after the full Windows-equivalent ACK window produced neither a
 * clear ACK nor a TLS image record.
 */
#ifndef GX_GET_IMAGE_ACK_TIMEOUT_MS
#define GX_GET_IMAGE_ACK_TIMEOUT_MS 1000
#endif
#define GX_GET_IMAGE_NO_REPLY_POLLS   (GX_GET_IMAGE_ACK_TIMEOUT_MS / GX_DRAIN_IRQ_POLL_MS)
G_STATIC_ASSERT (GX_GET_IMAGE_ACK_TIMEOUT_MS % GX_DRAIN_IRQ_POLL_MS == 0);
G_STATIC_ASSERT (GX_GET_IMAGE_NO_REPLY_POLLS == 100);

  static guint8 scratch[GOODIX_RX_MAX];
  int expected_plain_replies = 0;
  int attempts = body[0] == 0x20u ? GX_GET_IMAGE_ATTEMPTS : 1;
  int attempt;

  /* Reply counts observed stable across the GXFP51A0 capture path and matching
   * the command semantics. Commands not listed here retain silence-based
   * draining. GET_IMAGE remains special because its ACK/TLS ordering varies. */
  switch (body[0])
    {
    /* Windows ST411/14115 emits two plain responses for FDT-down before
     * image capture: the 0x32 ACK, then the 20-byte FDT data/touch report.
     * Leaving the second response pending contaminates the IRQ/response queue
     * immediately before GET_IMAGE and makes the first 0x20 command appear
     * swallowed. Drain both before advancing the capture recipe. */
    case 0x32u: expected_plain_replies = 2; break; /* FDT-down ACK + data */
    case 0x36u: expected_plain_replies = 2; break; /* FDT manual/data */
    case 0x50u: expected_plain_replies = 2; break; /* NAV */
    case 0x80u: expected_plain_replies = 1; break; /* register write */
    case 0x82u: expected_plain_replies = 2; break; /* register read */
    case 0xaeu: expected_plain_replies = 2; break; /* MCU state */
    default: break;
    }

  if (out_ack_seen)
    *out_ack_seen = FALSE;
  if (out_tls_seen)
    *out_tls_seen = FALSE;

  for (attempt = 1; attempt <= attempts; attempt++)
    {
      guint8 ty;
      gboolean ack_seen = FALSE;
      gboolean tls_seen = FALSE;
      int no_reply_miss_limit =
        body[0] == 0x20u ? GX_GET_IMAGE_NO_REPLY_POLLS :
        body[0] == 0xaeu ? 12 : 25;
      int r, got = 0, i, misses = 0;
      int post_reply_miss_limit =
        body[0] == 0x32u ? GX_FDT_DOWN_SECOND_REPLY_POLLS : GX_DRAIN_SILENCE;

      /* Capture commands are sent directly. The recipe carries explicit NOPs
       * at the positions observed on GXFP51A0; do not prepend one here. */
      if (!gx_write_frame (self, GOODIX_PKT_PLAIN, body, n))
        return FALSE;

      /* Windows sends capture-group NOP without waiting for an ACK.
       * Waiting for silence here costs ~250 ms because NOP intentionally has
       * no response.  The outer capture recipe still keeps its conservative
       * inter-command gap, so this removes only a known-empty drain. */
      if (body[0] == 0x00u)
        {
          fp_dbg ("drain cmd=00: no ACK expected; skip silence wait");
          return TRUE;
        }

      g_usleep (15000);

      for (i = 0; i < 120; i++)
        {
          errno = 0;
          if (gx51_wait_irq_gpio48 (self->irq_fd, GX_DRAIN_IRQ_POLL_MS) < 0)
            {
              if (errno != ETIMEDOUT)
                return FALSE;

              misses++;
              if (got && misses >= post_reply_miss_limit)
                break;
              if (!got && misses >= no_reply_miss_limit)
                break;
              continue;
            }

          r = gx_read_frame (self, &ty, scratch, sizeof scratch);
          if (r > 0 && ty == GOODIX_PKT_PLAIN)
            {
              guint8 ack_status = 0;

              got++;
              misses = 0;
              if (gxfp_parse_ack (scratch, r, body[0], &ack_status) &&
                  gxfp_ack_status_success (ack_status))
                ack_seen = TRUE;
              if (expected_plain_replies > 0 && got >= expected_plain_replies)
                {
                  fp_dbg ("drain cmd=%02x: expected %d replies received; finish",
                          body[0], expected_plain_replies);
                  break;
                }
              g_usleep (5000);
              continue;
            }
          if (r > 0 && ty == GOODIX_PKT_TLS)
            {
              memcpy (self->tls_rx, scratch, r);
              self->tls_rxlen = r;
              self->tls_rxpos = 0;
              tls_seen = TRUE;
              fp_dbg ("drain cmd=%02x -> stashed TLS record, %d bytes",
                      body[0], r);
              break;
            }

          misses++;
          if (got && misses >= post_reply_miss_limit)
            break;
          if (!got && misses >= no_reply_miss_limit)
            break;
        }

      fp_dbg ("drain cmd=%02x: %d cleartext reply/replies ack=%d tls=%d attempt=%d/%d",
              body[0], got, ack_seen, tls_seen, attempt, attempts);
      if (body[0] == 0x32u)
        fp_info ("GXFP51A0 FDT_DOWN_DRAIN replies=%d expected=%d ack=%d",
                 got, expected_plain_replies, ack_seen);

      if (out_ack_seen && ack_seen)
        *out_ack_seen = TRUE;
      if (out_tls_seen && tls_seen)
        *out_tls_seen = TRUE;

      if (body[0] != 0x20u)
        return TRUE;

      /* Windows and the validated Linux background path both receive a
       * cleartext ACK for GET_IMAGE before the oversized TLS record. If neither
       * ACK nor TLS arrived, the command was swallowed. Replaying the identical
       * GET_IMAGE once is safe because FDT remains armed. Never replay once a
       * TLS record is already present: that would advance the client record
       * stream twice and desynchronise AES-GCM sequence numbers. */
      if (tls_seen || ack_seen)
        return TRUE;

      if (attempt < attempts)
        {
          self->capture_retry_seen = TRUE;
          fp_warn ("GXFP51A0 GET_IMAGE had no ACK/TLS after %d ms ACK window; "
                   "retrying attempt=%d/%d",
                   GX_GET_IMAGE_ACK_TIMEOUT_MS, attempt + 1, attempts);
          g_usleep (8000 * self->timing_scale / 100);
        }
    }

  fp_warn ("GXFP51A0 GET_IMAGE failed: no ACK/TLS after %d attempts",
           attempts);
  gx_capture_transport_desync (self);
  return FALSE;
}

/* ------------------------------------------------------------------ */
/*  Transport glue: TLS records travel inside 0xB0 SPI frames          */
/* ------------------------------------------------------------------ */

static int
gx_bio_send (gpointer ctx, const guint8 *b, gsize l)
{
  FpiDeviceGoodix51A0 *self = ctx;

  gsize off = 0;

  /* One TLS record per 0xB0 frame. OpenSSL batches several records into a
   * single write — the ServerHello group comes out as one 121-byte call — but
   * the sensor expects exactly one record per frame and silently ignores a
   * frame carrying two. Split on the record headers rather than forwarding the
   * buffer as it arrives. */
  while (off + 5 <= l)
    {
      gsize rec = 5 + (((gsize) b[off + 3] << 8) | b[off + 4]);

      if (off + rec > l)
        break;                        /* partial record: should not happen */
      if (!gx_write_frame (self, GOODIX_PKT_TLS, b + off, rec))
        return -1;
      off += rec;
    }
  return (int) l;
}

static int
gx_bio_recv (gpointer ctx, guint8 *b, gsize l)
{
  FpiDeviceGoodix51A0 *self = ctx;
  int avail, take, i;
  guint8 ty;

  if (self->tls_rxpos >= self->tls_rxlen)
    {
      /* Buffer empty: pull fresh TLS frames, skipping cleartext ones. The
       * sensor takes a while to produce the image, hence the generous poll. */
      int r = -1;
      self->tls_rxlen = self->tls_rxpos = 0;
      for (i = 0; i < GX_TLS_IRQ_POLLS; i++)
        {
          /* Do not call gx_read_frame() while the line is quiet: its generic
           * 1200 ms wait would turn this intended 20 ms poll loop into a
           * multi-minute block.  TLS handshake records observed on reference MateBook
           * normally arrive in ~10-20 ms, while 4 s remains generous for a
           * genuinely delayed record. */
          errno = 0;
          if (gx51_wait_irq_gpio48 (self->irq_fd, GX_TLS_IRQ_POLL_MS) < 0)
            {
              if (errno != ETIMEDOUT)
                return -1;
              continue;
            }

          r = gx_read_frame (self, &ty, self->tls_rx, GOODIX_RX_MAX);
          if (r > 0 && ty == GOODIX_PKT_TLS)
            {
              fp_dbg ("bio_recv: TLS frame %d bytes (attempt %d)", r, i);
              break;
            }
          r = -1;
        }
      if (r <= 0)
        {
          fp_dbg ("bio_recv: no TLS frame (timed out)");
          return -1;
        }
      self->tls_rxlen = r;
      self->tls_rxpos = 0;
    }
  avail = self->tls_rxlen - self->tls_rxpos;
  take = (int) l < avail ? (int) l : avail;
  memcpy (b, self->tls_rx + self->tls_rxpos, take);
  self->tls_rxpos += take;
  return take;
}

/* ------------------------------------------------------------------ */
/*  Image decoding: six bytes carry four pixels, in interleaved order  */
/* ------------------------------------------------------------------ */


/* ------------------------------------------------------------------ */
/*  Init, handshake and frame capture                                  */
/* ------------------------------------------------------------------ */

/* Hardware reset. GPIO264 is active-HIGH reset on GXFP51A0: HIGH holds the
 * MCU in reset and LOW lets it run. First-contact hardware confirmation uses
 * HIGH for 300 ms, then LOW with a 600 ms settle; final state remains LOW. */
static void
gx_gpio_reset (FpiDeviceGoodix51A0 *self)
{
  /* A cold reset is most reliable while no SPI file descriptor is open.
   * rel71.16 proved that opening spidev first and only then toggling GPIO264
   * can leave ST411 in a half-open TLS state across fprintd restarts. */
  fp_warn ("GXFP51A0 RESET_TRACE GPIO264 300/600 boundary (spi_fd=%d)",
           self ? self->spi_fd : -1);
  if (gx51_reset_gpio264 () < 0)
    fp_warn ("GXFP51A0: reviewed GPIO264 reset failed");
}




static gboolean
gx_target_read_body (FpiDeviceGoodix51A0 *self,
                     guint8                 *body,
                     gsize                   cap,
                     int                    *out_len)
{
  guint8 type = 0;
  int n = gx_read_frame (self, &type, body, cap);

  if (n <= 0 || type != GOODIX_PKT_PLAIN)
    return FALSE;
  if (out_len)
    *out_len = n;
  return TRUE;
}

static gboolean
gx_target_read_response (FpiDeviceGoodix51A0 *self,
                         guint8               *body,
                         gsize                 cap,
                         int                  *out_len)
{
  /* Proven on reference MateBook: response-bearing commands need the ACK IRQ cycle to
   * deassert before the response is armed. The validated probes use 8 ms. */
  g_usleep (8000 * self->timing_scale / 100);
  return gx_target_read_body (self, body, cap, out_len);
}

static gboolean
gx_target_send_ack (FpiDeviceGoodix51A0       *self,
                    const struct gxfp_target_packet *packet,
                    guint8                           command,
                    guint8                          *status)
{
  guint8 rx[128];
  guint8 type = 0;
  guint8 ack_status = 0;
  int attempt;

  for (attempt = 1; attempt <= GX_TARGET_ACK_ATTEMPTS; attempt++)
    {
      int n;

      /* GPIO48 is level-high.  Never start a new command while a previous
       * response still owns the line. */
      if (gx51_wait_irq_gpio48_low (self->irq_fd, 50) < 0)
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: pre-write-irq-high cmd=0x%02x",
                   command);
          return FALSE;
        }

      if (!gx_write_frame (self, GOODIX_PKT_PLAIN,
                           packet->inner, packet->inner_len))
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: write-failed cmd=0x%02x",
                   command);
          return FALSE;
        }

      /* Distinguish a completely swallowed command (no IRQ edge/level at all)
       * from a protocol error.  Only the former is safe to retry: register
       * writes and idle/config commands are idempotent when replayed with the
       * same bytes.  If any packet arrives, parsing/status errors are final. */
      errno = 0;
      if (gx51_wait_irq_gpio48 (self->irq_fd, GX_TARGET_ACK_IRQ_TIMEOUT_MS) < 0)
        {
          if (errno == ETIMEDOUT && attempt < GX_TARGET_ACK_ATTEMPTS)
            {
              fp_warn ("GXFP51A0 target ACK diagnostic: no-irq retry cmd=0x%02x attempt=%d/%d",
                       command, attempt, GX_TARGET_ACK_ATTEMPTS);
              gx_protocol_timing_miss (self, "target-ack");
              g_usleep (8000 * self->timing_scale / 100);
              continue;
            }

          fp_warn ("GXFP51A0 target ACK diagnostic: ack-read-failed cmd=0x%02x",
                   command);
          return FALSE;
        }

      n = gx_read_frame (self, &type, rx, sizeof rx);
      if (n <= 0)
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: ack-read-failed cmd=0x%02x",
                   command);
          return FALSE;
        }
      if (type != GOODIX_PKT_PLAIN)
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: ack-type-failed cmd=0x%02x type=0x%02x",
                   command, type);
          return FALSE;
        }
      if (!gxfp_parse_ack (rx, n, command, &ack_status))
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: ack-parse-failed cmd=0x%02x len=%d first=0x%02x",
                   command, n, n > 0 ? rx[0] : 0);
          return FALSE;
        }
      if (!gxfp_ack_status_success (ack_status))
        {
          fp_warn ("GXFP51A0 target ACK diagnostic: status-failed cmd=0x%02x status=0x%02x",
                   command, ack_status);
          return FALSE;
        }

      if (status)
        *status = ack_status;
      return TRUE;
    }

  return FALSE;
}

static bool
gx_factory_staging_read_cb (void *user, uint32_t selector,
                            uint32_t request_len, uint8_t *out,
                            size_t out_cap, size_t *out_len)
{
  FpiDeviceGoodix51A0 *self = user;
  struct gxfp_target_packet packet;
  guint8 rx[GXFP_MEM_READ_MAX + 16];
  guint8 discard[GXFP_MEM_READ_MAX];
  guint8 type = 0, status = 0;
  int n;

  if (!out || !out_len || request_len != GXFP_FACTORY_BODY_LEN ||
      out_cap < GXFP_FACTORY_STAGING_LEN ||
      !gxfp_build_mem_read (selector, request_len, &packet) ||
      !gx_write_frame (self, GOODIX_PKT_PLAIN,
                       packet.inner, packet.inner_len))
    return false;

  n = gx_read_frame (self, &type, rx, sizeof rx);
  if (n <= 0 || type != GOODIX_PKT_PLAIN ||
      !gxfp_parse_ack (rx, n, 0xf2, &status) ||
      !gxfp_ack_status_success (status))
    return false;

  for (unsigned int i = 0; i < 3; i++)
    {
      enum gxfp_mem_read_result result;

      g_usleep ((i == 0 ? 8000 : 5000) * self->timing_scale / 100);
      n = gx_read_frame (self, &type, rx, sizeof rx);
      if (n <= 0 || type != GOODIX_PKT_PLAIN)
        return false;
      if (gxfp_14115_parse_rejected_staging_response (
            rx, n, selector, request_len, out, out_cap, out_len))
        return *out_len == GXFP_FACTORY_STAGING_LEN;

      result = gxfp_classify_mem_read_response (
        rx, n, selector, request_len, discard);
      if (result != GXFP_MEM_READ_ECHO_ONLY)
        return false;
    }
  return false;
}

static bool
gx_factory_e4_sanity (FpiDeviceGoodix51A0 *self)
{
  struct gxfp_target_packet packet;
  guint8 rx[128], value[32];
  uint32_t dtype = 0;
  int n = 0;
  bool ok;

  ok = gxfp_build_factory_hash_read (&packet) &&
       gx_target_send_ack (self, &packet, 0xe4, NULL) &&
       gx_target_read_response (self, rx, sizeof rx, &n) &&
       gxfp_parse_factory_hash_response (rx, n, &dtype, value) &&
       dtype == 0x0000aaaau;
  OPENSSL_cleanse (value, sizeof value);
  return ok;
}

static void
gx_pmk_clear (FpiDeviceGoodix51A0 *self)
{
  OPENSSL_cleanse (self->psk, sizeof self->psk);
  self->psk_ready = FALSE;
  self->psk_from_cache = FALSE;
}


static gboolean
gx_pmk_cache_load (FpiDeviceGoodix51A0 *self)
{
  struct stat st;
  g_autofree gchar *data = NULL;
  gsize len = 0;
  g_autoptr(GError) error = NULL;

  if (g_stat (GX_PMK_CACHE_FILE, &st) != 0)
    return FALSE;

  if (!S_ISREG (st.st_mode) ||
      st.st_uid != geteuid () ||
      (st.st_mode & 0077) != 0)
    {
      fp_warn ("GXFP51A0 PMK cache rejected: owner/permissions are unsafe");
      return FALSE;
    }

  if (!g_file_get_contents (GX_PMK_CACHE_FILE, &data, &len, &error))
    {
      fp_warn ("GXFP51A0 PMK cache read failed: %s", error->message);
      return FALSE;
    }

  if (len != GOODIX_PSK_LEN)
    {
      fp_warn ("GXFP51A0 PMK cache rejected: invalid length");
      OPENSSL_cleanse (data, len);
      return FALSE;
    }

  memcpy (self->psk, data, GOODIX_PSK_LEN);
  OPENSSL_cleanse (data, len);
  self->psk_ready = TRUE;
  self->psk_from_cache = TRUE;
  fp_info ("GXFP51A0 PMK cache candidate loaded; TLS validation pending");
  return TRUE;
}


static gboolean
gx_pmk_cache_save (FpiDeviceGoodix51A0 *self)
{
  g_autoptr(GError) error = NULL;

  if (!self->psk_ready || !self->tls_up)
    return FALSE;

  if (!g_file_test (GX_PMK_CACHE_DIR, G_FILE_TEST_IS_DIR) &&
      g_mkdir_with_parents (GX_PMK_CACHE_DIR, 0700) != 0)
    {
      fp_warn ("GXFP51A0 PMK cache directory creation failed: %s",
               g_strerror (errno));
      return FALSE;
    }

  if (!g_file_set_contents_full (
        GX_PMK_CACHE_FILE,
        (const gchar *) self->psk,
        GOODIX_PSK_LEN,
        G_FILE_SET_CONTENTS_CONSISTENT | G_FILE_SET_CONTENTS_DURABLE,
        0600,
        &error))
    {
      fp_warn ("GXFP51A0 PMK cache save failed: %s", error->message);
      return FALSE;
    }

  if (g_chmod (GX_PMK_CACHE_FILE, 0600) != 0)
    {
      fp_warn ("GXFP51A0 PMK cache chmod failed: %s", g_strerror (errno));
      (void) g_unlink (GX_PMK_CACHE_FILE);
      return FALSE;
    }

  fp_info ("GXFP51A0 PMK cache saved after live TLS validation");
  return TRUE;
}


static gboolean
gx_factory_load_staging_pmk (FpiDeviceGoodix51A0 *self)
{
  guint8 pmk[GXFP_FACTORY_PMK_LEN];
  gboolean ok = FALSE;

  /* On 14115 the rejected F2 path can expose one boot-staged factory record.
   * A second selector is not required at runtime: the first-byte brute force
   * must produce exactly one marker=0x000d,len=0x30 candidate, and that
   * candidate is then validated by the real TLS handshake.  Keeping the
   * two-selector consensus code as offline research avoids coupling runtime
   * success to the IRQ/reply behavior of a second rejected F2 command. */
  if (!gxfp_factory_load_pmk_from_single_staging (
        gx_factory_staging_read_cb, self, pmk))
    {
      fp_warn ("GXFP51A0 PMK single-staging candidate invalid for this boot session");
      goto out;
    }

  /* Run E4 only after extraction: any earlier command can reuse staging. */
  if (!gx_factory_e4_sanity (self))
    {
      fp_warn ("GXFP51A0 PMK candidate recovered but E4 sanity failed");
      goto out;
    }

  memcpy (self->psk, pmk, sizeof pmk);
  self->psk_ready = TRUE;
  self->psk_from_cache = FALSE;
  fp_info ("GXFP51A0 PMK recovered from single rejected-read boot staging; TLS validation pending");
  ok = TRUE;

out:
  if (!ok)
    {
      gx_pmk_clear (self);
    }
  OPENSSL_cleanse (pmk, sizeof pmk);
  return ok;
}


static gboolean
gx_factory_acquire_staging_pmk (FpiDeviceGoodix51A0 *self, gboolean allow_cache)
{
  int attempt;

  if (allow_cache && gx_pmk_cache_load (self))
    return TRUE;

  for (attempt = 1; attempt <= GXFP_PMK_ACQUIRE_ATTEMPTS; attempt++)
    {
      gx_gpio_reset (self);
      fp_info ("GXFP51A0 PMK staging attempt %d/%d after hardware reset",
               attempt, GXFP_PMK_ACQUIRE_ATTEMPTS);

      if (gx_factory_load_staging_pmk (self))
        {
          if (attempt > 1)
            fp_info ("GXFP51A0 PMK staging recovered on attempt %d", attempt);
          return TRUE;
        }

      if (attempt < GXFP_PMK_ACQUIRE_ATTEMPTS)
        g_usleep (50000);
    }

  fp_warn ("GXFP51A0 PMK staging acquisition exhausted after %d boot sessions",
           GXFP_PMK_ACQUIRE_ATTEMPTS);
  return FALSE;
}

static gboolean
gx_read_fw_version_stage2e (FpiDeviceGoodix51A0 *self, gchar *out, gsize cap)
{
  static const guint8 a8[] = {
    GOODIX_CMD_FW_VERSION, 0x03, 0x00, 0x00, 0x00, 0xFF
  };
  guint8 rx[128], type = 0, status = 0;
  int n = 0;

  /* Exact post-reset boundary proven by reference MateBook Stage2E:
   * A8 -> ACK -> 8 ms -> A8 response.  No preceding NOP here. */
  if (!gx_write_frame (self, GOODIX_PKT_PLAIN, a8, sizeof a8))
    {
      fp_warn ("GXFP51A0 Stage2E A8 write failed");
      return FALSE;
    }

  n = gx_read_frame (self, &type, rx, sizeof rx);
  if (n <= 0 || type != GOODIX_PKT_PLAIN ||
      !gxfp_parse_ack (rx, n, GOODIX_CMD_FW_VERSION, &status) ||
      !gxfp_ack_status_success (status))
    {
      fp_warn ("GXFP51A0 Stage2E A8 ACK failed");
      return FALSE;
    }
  fp_info ("GXFP51A0 Stage2E A8 ACK accepted");

  g_usleep (8000 * self->timing_scale / 100);

  n = gx_read_frame (self, &type, rx, sizeof rx);
  if (n <= 3 || type != GOODIX_PKT_PLAIN ||
      rx[0] != GOODIX_CMD_FW_VERSION)
    {
      fp_warn ("GXFP51A0 Stage2E A8 response failed");
      return FALSE;
    }

  g_strlcpy (out, (const gchar *) rx + 3,
             MIN ((gsize) (n - 3) + 1, cap));
  fp_info ("GXFP51A0 Stage2E A8 response accepted");
  return TRUE;
}


static gboolean
gx_target_soft_reset (FpiDeviceGoodix51A0 *self)
{
  struct gxfp_target_packet packet;
  guint8 rx[128];
  uint32_t code = 0;
  int n = 0;

  if (!gxfp_build_soft_reset (&packet))
    {
      fp_warn ("GXFP51A0 A2 diagnostic: build-failed");
      return FALSE;
    }
  if (!gx_target_send_ack (self, &packet, 0xa2, NULL))
    {
      fp_warn ("GXFP51A0 A2 diagnostic: ack-failed");
      return FALSE;
    }
  fp_info ("GXFP51A0 A2 ACK accepted");

  if (!gx_target_read_response (self, rx, sizeof rx, &n))
    {
      fp_warn ("GXFP51A0 A2 diagnostic: response-read-failed");
      return FALSE;
    }
  if (!gxfp_parse_soft_reset_response (rx, n, &code))
    {
      fp_warn ("GXFP51A0 A2 diagnostic: response-parse-failed len=%d", n);
      return FALSE;
    }
  if (code != 0x010008u)
    {
      fp_warn ("GXFP51A0 A2 diagnostic: unexpected-code=0x%06x", code);
      return FALSE;
    }

  fp_info ("GXFP51A0 A2 reset code accepted: 0x%06x", code);
  return TRUE;
}

static gboolean
gx_target_configure (FpiDeviceGoodix51A0 *self)
{
  struct gxfp_target_packet packet;
  struct gxfp_target_calibration cal;
  guint8 rx[512];
  guint8 otp[64];
  guint8 config[GXFP_CONFIG_LEN];
  guint8 status = 0;
  uint16_t chip_id = 0;
  int n = 0;

  if (!gx_target_soft_reset (self))
    {
      fp_warn ("GXFP51A0 target init failed: soft-reset-1");
      return FALSE;
    }

  if (!gxfp_build_chip_id (&packet) ||
      !gx_target_send_ack (self, &packet, 0x82, NULL) ||
      !gx_target_read_response (self, rx, sizeof rx, &n) ||
      !gxfp_parse_chip_id_response (rx, n, &chip_id) ||
      chip_id != 0x2504u)
    {
      fp_warn ("GXFP51A0 target init failed: chip-id");
      return FALSE;
    }
  fp_info ("GXFP51A0 chip-id accepted: 0x%04x", chip_id);

  if (!gxfp_build_read_otp (&packet))
    {
      fp_warn ("GXFP51A0 target init failed: otp-build");
      return FALSE;
    }
  if (!gx_target_send_ack (self, &packet, 0xa6, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: otp-ack");
      return FALSE;
    }
  fp_info ("GXFP51A0 OTP ACK accepted");
  if (!gx_target_read_response (self, rx, sizeof rx, &n))
    {
      fp_warn ("GXFP51A0 target init failed: otp-response-read");
      return FALSE;
    }
  if (!gxfp_parse_otp_response (rx, n, otp))
    {
      fp_warn ("GXFP51A0 target init failed: otp-response-parse len=%d", n);
      return FALSE;
    }
  if (!gxfp_derive_calibration (otp, &cal))
    {
      fp_warn ("GXFP51A0 target init failed: otp-validation");
      return FALSE;
    }
  fp_info ("GXFP51A0 OTP validated: len=%d tcode=%u fdt_delta=%u",
           n, cal.tcode, cal.fdt_delta);

  memcpy (config, GXFP_TARGET_BASE_CONFIG, sizeof config);
  if (!gxfp_patch_config (config, &cal) ||
      !gxfp_config_checksum_valid (config))
    {
      fp_warn ("GXFP51A0 target init failed: config-patch");
      return FALSE;
    }

  if (!gx_target_soft_reset (self))
    {
      fp_warn ("GXFP51A0 target init failed: soft-reset-2");
      return FALSE;
    }

  if (!gxfp_build_idle (&packet) ||
      !gx_target_send_ack (self, &packet, 0x70, &status))
    {
      fp_warn ("GXFP51A0 target init failed: idle");
      return FALSE;
    }

  if (!gxfp_build_reg_write (0x0220u, cal.dac_main, &packet) ||
      !gx_target_send_ack (self, &packet, 0x80, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: dac-main");
      return FALSE;
    }
  if (!gxfp_build_reg_write (0x0236u, cal.dac1, &packet) ||
      !gx_target_send_ack (self, &packet, 0x80, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: dac1");
      return FALSE;
    }
  if (!gxfp_build_reg_write (0x0238u, cal.dac2, &packet) ||
      !gx_target_send_ack (self, &packet, 0x80, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: dac2");
      return FALSE;
    }
  if (!gxfp_build_reg_write (0x023au, cal.dac3, &packet) ||
      !gx_target_send_ack (self, &packet, 0x80, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: dac3");
      return FALSE;
    }

  if (!gxfp_build_upload_config (config, &packet) ||
      !gx_target_send_ack (self, &packet, 0x90, NULL))
    {
      fp_warn ("GXFP51A0 target init failed: config-ack");
      return FALSE;
    }
  if (!gx_target_read_response (self, rx, sizeof rx, &n) ||
      !gxfp_parse_config_response (rx, n, &status) || status != 0x01u)
    {
      fp_warn ("GXFP51A0 target init failed: config-response");
      return FALSE;
    }

  self->target_cal = cal;
  self->have_target_cal = TRUE;
  fp_info ("GXFP51A0 target config accepted: chip=0x%04x tcode=%u "
           "fdt_delta=%u dac=0x%03x/%02x/%02x/%02x",
           chip_id, cal.tcode, cal.fdt_delta, cal.dac_main,
           cal.dac1, cal.dac2, cal.dac3);
  return TRUE;
}


/* reference MateBook now validates the 14115 rejected-read staging provider. Genuine
 * private-flash/RAM F2 remains closed; the reset is required so extraction
 * happens before another command can reuse the boot staging area. */
static gboolean
gx_upload_config_and_reqtls (FpiDeviceGoodix51A0 *self, gboolean allow_cache)
{
  guint8 ack[128], status = 0, type = 0;
  int n;

  if (!self->psk_ready)
    {
      gchar fw[64] = { 0 };

      if (!gx_factory_acquire_staging_pmk (self, allow_cache))
        return FALSE;

      /* Rejected F2 staging reads are intentionally outside the normal 14115
       * command flow. Keep the recovered PMK host-side, but return the MCU to a
       * pristine boot state before target init. The exact reference MateBook config probe
       * that passed starts a fresh hardware reset with A8 -> A2 -> chip-id ->
       * OTP, so reproduce that boundary here instead of continuing after F2/E4. */
      fp_info ("GXFP51A0 PMK retained; performing post-PMK reset before target init");
      gx_gpio_reset (self);
      if (!gx_read_fw_version_stage2e (self, fw, sizeof fw) ||
          g_strcmp0 (fw, "GF_ST411SEC_APP_14115") != 0)
        {
          fp_warn ("GXFP51A0 target init failed: post-PMK reset/A8");
          return FALSE;
        }
      fp_info ("GXFP51A0 post-PMK reset/A8 confirmed: %s", fw);
    }

  if (!gx_target_configure (self))
    {
      fp_warn ("GXFP51A0: target initialization/configuration failed");
      return FALSE;
    }

  if (!gx_send_plain_raw (self, GX_REQTLS, sizeof GX_REQTLS))
    return FALSE;
  n = gx_read_frame (self, &type, ack, sizeof ack);
  if (n <= 0 || type != GOODIX_PKT_PLAIN ||
      !gxfp_parse_ack (ack, n, 0xd0, &status) ||
      !gxfp_ack_status_success (status))
    {
      fp_warn ("GXFP51A0: D0 TLS request was not accepted");
      return FALSE;
    }

  return TRUE;
}


/* Establishes the TLS-PSK channel, with us as the server. */
static gboolean
gx_tls_handshake (FpiDeviceGoodix51A0 *self)
{
  g_autoptr(GError) err = NULL;
  guint8 ack[128], status = 0, type = 0;
  int n;

  if (!self->psk_ready)
    return FALSE;
  self->tls_rxlen = self->tls_rxpos = 0;
  g_clear_pointer (&self->tls, gx_tls_free);
  self->tls = gx_tls_new (self->psk, GOODIX_PSK_LEN, GOODIX_TLS_IDENTITY,
                          gx_bio_send, gx_bio_recv, self);
  if (!self->tls)
    {
      fp_warn ("cannot set up the TLS channel");
      return FALSE;
    }
  if (!gx_tls_handshake_run (self->tls, &err))
    {
      fp_warn ("%s", err->message);
      return FALSE;
    }

  /* Windows and the validated Stage3C path both confirm the completed
   * handshake back to the MCU with NOP -> D4. */
  if (!gx_send_plain_raw (self, GX_TLS_OK, sizeof GX_TLS_OK))
    {
      fp_warn ("GXFP51A0: failed to send TLS established D4");
      return FALSE;
    }
  n = gx_read_frame (self, &type, ack, sizeof ack);
  if (n <= 0 || type != GOODIX_PKT_PLAIN ||
      !gxfp_parse_ack (ack, n, 0xd4, &status) ||
      !gxfp_ack_status_success (status))
    {
      fp_warn ("GXFP51A0: TLS established D4 was not accepted");
      return FALSE;
    }

  fp_info ("GXFP51A0 TLS established ACK D4 accepted");
  fp_info ("GXFP51A0 PMK candidate validated by live TLS handshake");
  fp_info ("TLS-PSK session up (%s)", gx_tls_ciphersuite (self->tls));
  self->tls_up = TRUE;
  if (self->psk_from_cache)
    fp_info ("GXFP51A0 cached PMK validated by live TLS handshake");
  else
    (void) gx_pmk_cache_save (self);
  return TRUE;
}


static void
gx_tls_teardown (FpiDeviceGoodix51A0 *self)
{
  if (self->tls_up)
    {
      gx_tls_close (self->tls);
      self->tls_up = FALSE;
    }
  g_clear_pointer (&self->tls, gx_tls_free);
}

/* Exact 14115/51x7 capture recipes, as validated by the Linux imaging path.
 * 30 ms remains the default on the reference 2021 unit. Slower controllers can
 * learn a capture-only pacing multiplier after a real GET_IMAGE transport
 * desync; this is deliberately separate from TLS/init timing. */
#ifndef GX_SEQ_GAP_US
#define GX_SEQ_GAP_US 30000
#endif

typedef enum
{
  GX_CAPTURE_FINGER,
  GX_CAPTURE_BACKGROUND_COLD,
  GX_CAPTURE_BACKGROUND_WARM
} GxCaptureKind;

static gboolean
gx_send_capture_recipe (FpiDeviceGoodix51A0 *self, GxCaptureKind kind)
{
  struct gxfp_capture_recipe recipe;
  gboolean ok;
  gsize i;

  if (kind == GX_CAPTURE_BACKGROUND_COLD)
    ok = self->have_target_cal &&
         gxfp_build_background_capture_recipe (&self->target_cal, &recipe);
  else if (kind == GX_CAPTURE_BACKGROUND_WARM)
    ok = gxfp_build_warm_background_capture_recipe (&recipe);
  else
    ok = gxfp_build_finger_capture_recipe (&recipe);
  if (!ok)
    return FALSE;

  for (i = 0; i < recipe.count; i++)
    {
      const struct gxfp_target_packet *packet = &recipe.steps[i];
      gint64 t0 = g_get_monotonic_time ();

      if (!gx_send_plain_drain (self, packet->inner, packet->inner_len, NULL, NULL))
        return FALSE;
      fp_dbg ("chrono cmd=%02x drain=%ld us", packet->inner[0],
              (long) (g_get_monotonic_time () - t0));
      g_usleep (gx_capture_gap_us (self));
    }
  return TRUE;
}


/* Takes exactly one complete sensor->host TLS transport frame. A frame may
 * already have been stashed by gx_send_plain_drain(); otherwise wait for it. */
static int
gx_take_tls_frame (FpiDeviceGoodix51A0 *self, guint8 *rec, gsize cap)
{
  int raw, k;

  if (self->tls_rxlen > self->tls_rxpos)
    {
      raw = self->tls_rxlen - self->tls_rxpos;
      if ((gsize) raw > cap)
        return -1;
      memcpy (rec, self->tls_rx + self->tls_rxpos, raw);
      self->tls_rxlen = self->tls_rxpos = 0;
      return raw;
    }

  for (k = 0; k < GX_TLS_IRQ_POLLS; k++)
    {
      guint8 ty;

      errno = 0;
      if (gx51_wait_irq_gpio48 (self->irq_fd, GX_TLS_IRQ_POLL_MS) < 0)
        {
          if (errno != ETIMEDOUT)
            return -1;
          continue;
        }

      raw = gx_read_frame (self, &ty, rec, cap);
      if (raw > 0 && ty == GOODIX_PKT_TLS)
        return raw;
    }
  return -1;
}


static int
gx_fail_get_image_after_tls_timeout (FpiDeviceGoodix51A0 *self)
{
  /* Reaching this point means gx_send_plain_drain() already observed evidence
   * that GET_IMAGE was accepted (normally its cleartext ACK), then no TLS image
   * arrived during the full bounded TLS wait.  The command may still complete
   * late.  Replaying it here is unsafe: two accepted GET_IMAGE commands can
   * leave two application records in flight, so the next semantic capture can
   * consume a stale record even when AES-GCM sequence authentication itself
   * still succeeds.  The only unambiguous recovery boundary is a fresh MCU/TLS
   * session.  By contrast, gx_send_plain_drain() still retries once when
   * neither ACK nor TLS was observed, because there is no evidence that command
   * reached the sensor. */
  fp_warn ("GXFP51A0 GET_IMAGE was accepted but TLS image timed out; "
           "not replaying accepted command; forcing full session recovery");

  if (!self->capture_recovery_pending)
    gx_capture_transport_desync (self);
  return -1;
}


/* The 51x7 reference requires an FDT-up image, NAV and final FDT-down after a
 * finger frame. The intermediate image record must be AUTHENTICATED/decrypted,
 * not merely discarded: AES-GCM record sequence numbers advance for every
 * sensor record, and skipping it would desynchronise the following capture. */
static gboolean
gx_send_capture_cleanup (FpiDeviceGoodix51A0 *self)
{
  struct gxfp_capture_recipe recipe;
  g_autofree guint8 *rec = g_malloc0 (GOODIX_RX_MAX);
  g_autofree guint8 *plain = g_malloc0 (GOODIX_RX_MAX);
  gboolean ok = FALSE;
  gsize i;

  if (!gxfp_build_capture_cleanup_recipe (&recipe))
    goto out;

  for (i = 0; i < recipe.count; i++)
    {
      const struct gxfp_target_packet *packet = &recipe.steps[i];

      if (!gx_send_plain_drain (self, packet->inner, packet->inner_len, NULL, NULL))
        goto out;

      if (packet->inner[0] == 0x20u)
        {
          int raw = gx_take_tls_frame (self, rec, GOODIX_RX_MAX);
          gssize got;

          if (raw < 0)
            raw = gx_fail_get_image_after_tls_timeout (self);

          got = raw > 0
                  ? gx_tls_decrypt_record (self->tls, rec, (gsize) raw,
                                           plain, GOODIX_RX_MAX)
                  : -1;
          if (got != (gssize) GXFP_IMAGE_PLAINTEXT_LEN)
            {
              fp_warn ("capture cleanup image record failed (%d raw, %ld plain)",
                       raw, (long) got);
              if (!self->capture_recovery_pending)
                gx_capture_transport_desync (self);
              goto out;
            }
        }

      g_usleep (gx_capture_gap_us (self));
    }

  ok = TRUE;
out:
  OPENSSL_cleanse (rec, GOODIX_RX_MAX);
  OPENSSL_cleanse (plain, GOODIX_RX_MAX);
  return ok;
}


/* Captures one full image, assuming a TLS session is already up, and fills
 * px[GOODIX_IMG_PIXELS] with 12-bit samples. */
static gboolean
gx_capture_frame_ex (FpiDeviceGoodix51A0 *self,
                     guint16             *px,
                     GxCaptureKind        kind,
                     gboolean             cleanup)
{
  gboolean background = kind != GX_CAPTURE_FINGER;
  g_autofree guint8 *img = g_malloc (GOODIX_RX_MAX);
  int total = 0;

  self->capture_retry_seen = FALSE;
  gint64 tA = g_get_monotonic_time ();
  if (!gx_send_capture_recipe (self, kind))
    return FALSE;
  fp_dbg ("timing: capture sequence %ld us",
           (long) (g_get_monotonic_time () - tA));

  /* The image arrives as one complete sensor application-data record. Keep
   * explicit record decryption here because capture drains/stashes the already
   * framed transport record and the client-read sequence is tracked explicitly.
   * The record has usually already been picked up while draining the capture
   * sequence; otherwise wait for it. */
  {
    g_autofree guint8 *rec = g_malloc (GOODIX_RX_MAX);
    int raw = gx_take_tls_frame (self, rec, GOODIX_RX_MAX);
    gssize got;

    if (raw < 0)
      raw = gx_fail_get_image_after_tls_timeout (self);

    got = raw > 0 ? gx_tls_decrypt_record (self->tls, rec, (gsize) raw,
                                           img, GOODIX_RX_MAX) : -1;
    if (got < 0)
      {
        fp_warn ("cannot authenticate/decrypt the image record (%d raw bytes); "
                 "forcing full session recovery", raw);
        if (!self->capture_recovery_pending)
          gx_capture_transport_desync (self);
        return FALSE;
      }
    total = (int) got;
  }
  fp_dbg ("timing: whole capture %ld us (image %d bytes)",
           (long) (g_get_monotonic_time () - tA), total);
  if (total != (int) GXFP_IMAGE_PLAINTEXT_LEN)
    {
      fp_warn ("unexpected GXFP51A0 image plaintext: %d bytes (expected %u)",
               total, (unsigned) GXFP_IMAGE_PLAINTEXT_LEN);
      return FALSE;
    }
  if (!gxfp_decode_image_plaintext (img, (gsize) total, px))
    {
      fp_warn ("cannot decode GXFP51A0 88x80 transport raster");
      return FALSE;
    }

  /* Preserve a successfully decoded finger frame before cleanup.  The cleanup
   * is required to keep TLS record sequencing healthy for subsequent captures,
   * but a cleanup failure must not destroy the diagnostic evidence from the
   * frame we already authenticated and decoded.  gx_dump_capture() is inert
   * unless the private dump directory exists. */
#ifdef GXFP51A0_DEVELOPER
  if (!background)
    gx_dump_capture (self, px);
#endif

  if (!background && cleanup)
    {
      if (!gx_send_capture_cleanup (self))
        {
          fp_warn ("GXFP51A0 post-capture cleanup failed");
          return FALSE;
        }
      gx_capture_pacing_success (self);
    }
  return TRUE;
}

static gboolean
gx_capture_frame (FpiDeviceGoodix51A0 *self, guint16 *px, gboolean background)
{
  return gx_capture_frame_ex (self, px,
                              background ? GX_CAPTURE_BACKGROUND_COLD
                                         : GX_CAPTURE_FINGER,
                              TRUE);
}

static gboolean
gx_capture_warm_background_frame (FpiDeviceGoodix51A0 *self, guint16 *px)
{
  return gx_capture_frame_ex (self, px, GX_CAPTURE_BACKGROUND_WARM, TRUE);
}


static int
gx_cmp_dbl (const void *a, const void *b)
{
  double x = *(const double *) a - *(const double *) b;
  return (x > 0) - (x < 0);
}






/* Average n frames into one pixel array. */
static gboolean
gx_capture_avg (FpiDeviceGoodix51A0 *self, int nframes, guint16 *avg)
{
  g_autofree guint32 *acc = g_malloc0 (sizeof (guint32) * GOODIX_IMG_PIXELS);
  guint16 px[GOODIX_IMG_PIXELS];
  int got = 0, f, i;

  for (f = 0; f < nframes; f++)
    {
      if (!gx_capture_frame (self, px, TRUE))
        {
          if (self->capture_recovery_pending)
            return FALSE;
          continue;
        }
      for (i = 0; i < GOODIX_IMG_PIXELS; i++)
        acc[i] += px[i];
      got++;
    }
  if (!got)
    return FALSE;
  for (i = 0; i < GOODIX_IMG_PIXELS; i++)
    avg[i] = acc[i] / got;
  return TRUE;
}

/* ------------------------------------------------------------------ */
/*  Device life cycle: open / close                                    */
/* ------------------------------------------------------------------ */



/* Native finger detection: six 16-bit values, one per sensor zone, in about
 * 33 ms — some 70 times faster than capturing an image. A finger pulls those
 * values DOWN by roughly 100 counts (around 335/371/361 idle against
 * 229/269/275 with a finger), which is what makes cheap polling possible. */
#define GX_FDT_PROBE_ATTEMPTS 3
#define GX_FDT_IRQ_TIMEOUT_MS 100

static int
gx_fdt_probe_ex (FpiDeviceGoodix51A0 *self,
                 int                 *out_zones,
                 guint8              *out_touchflag)
{
  struct gxfp_target_packet packet;
  guint8 rx[256], ty, ack_status, touchflag;
  guint16 zones[GXFP_FDT_ZONE_COUNT];
  int attempt, n, k;

  if (!gxfp_build_fdt_probe (&packet))
    return -1;

  for (attempt = 1; attempt <= GX_FDT_PROBE_ATTEMPTS; attempt++)
    {
      const gchar *stage = "send";

      /* FDT manual is idempotent and Windows reissues it repeatedly while
       * building the baseline. A missing IRQ here is therefore recoverable:
       * retry the complete command, but never spin indefinitely. */
      if (!gx_send_plain_raw (self, packet.inner, packet.inner_len))
        return -1;

      g_usleep (15000 * self->timing_scale / 100);

      stage = "ack";
      errno = 0;
      if (gx51_wait_irq_gpio48 (self->irq_fd, GX_FDT_IRQ_TIMEOUT_MS) < 0)
        goto retry;

      n = gx_read_frame (self, &ty, rx, sizeof rx);
      if (n <= 0 || ty != GOODIX_PKT_PLAIN ||
          !gxfp_parse_ack (rx, (size_t) n, 0x36u, &ack_status) ||
          !gxfp_ack_status_success (ack_status))
        goto retry;

      g_usleep (8000 * self->timing_scale / 100);

      stage = "response";
      errno = 0;
      if (gx51_wait_irq_gpio48 (self->irq_fd, GX_FDT_IRQ_TIMEOUT_MS) < 0)
        goto retry;

      n = gx_read_frame (self, &ty, rx, sizeof rx);
      if (n <= 0 || ty != GOODIX_PKT_PLAIN ||
          !gxfp_parse_fdt_response (rx, (size_t) n, &touchflag, zones))
        goto retry;

      for (k = 0; k < (int) GXFP_FDT_ZONE_COUNT; k++)
        out_zones[k] = (int) zones[k];
      if (out_touchflag)
        *out_touchflag = touchflag;

      return 0;

retry:
      fp_warn ("GXFP51A0 FDT probe retry: stage=%s attempt=%d/%d",
               stage, attempt, GX_FDT_PROBE_ATTEMPTS);
      if (attempt == 1)
        gx_protocol_timing_miss (self, stage);
      if (attempt < GX_FDT_PROBE_ATTEMPTS)
        {
          /* Give a late/stale transaction a small quiesce window before
           * reissuing the same idempotent FDT-manual command. */
          g_usleep (10000 * self->timing_scale / 100);
        }
    }

  return -1;
}

static guint
gx_fdt_touch_count (guint8 touchflag)
{
  guint count = 0;

  for (guint8 zones = touchflag & 0x3fu; zones; zones >>= 1)
    count += zones & 1u;
  return count;
}

#define GX_FDT_TOUCH_MIN_ZONES 5

static gboolean
gx_fdt_touch_is_finger (guint8 touchflag)
{
  return gx_fdt_touch_count (touchflag) >= GX_FDT_TOUCH_MIN_ZONES;
}

/* Mean of the six exact-target FDT zones: an absolute test needing no baseline. */
static int
gx_fdt_mean (const int *v)
{
  int k, s = 0;
  for (k = 0; k < (int) GXFP_FDT_ZONE_COUNT; k++)
    s += v[k];
  return s / (int) GXFP_FDT_ZONE_COUNT;
}

/* Total drop against the baseline; above the threshold means a finger. */
static int
gx_fdt_drop (const int *base, const int *cur)
{
  int k, d = 0;
  for (k = 0; k < (int) GXFP_FDT_ZONE_COUNT; k++)
    d += base[k] - cur[k];
  return d / (int) GXFP_FDT_ZONE_COUNT;
}

/* ------------------------------------------------------------------ */
/*  Enrolment and verification, using local descriptors instead of the */
/*  NBIS pipeline.                                                     */
/*                                                                     */
/*  FpImageDevice mandates NBIS (minutiae plus bozorth3), which this    */
/*  tiny active area cannot reliably feed the stock NBIS pipeline.     */
/*  far below what reliable matching needs, and bozorth3 scored zero    */
/*  every time. Hence deriving straight from FpDevice and implementing  */
/*  enrol and verify against the descriptor matcher in goodix_sift.c.   */
/* ------------------------------------------------------------------ */

/* SIGFM score gates.
 *
 * These are candidate values, not security claims.  The release is only
 * promotable after a labelled genuine/impostor corpus shows a clear margin.
 * External small-sensor SIGFM drivers use runtime gates around 100-150; start
 * at the conservative low end until this exact 80x64 target is measured. */
#define GX_MATCH_THRESHOLD            7
#define GX_MIN_CAPTURE_KEYPOINTS     25

/* rel71.30 candidate: conservative per-view photometric rescue.
 *
 * The primary SIGFM acceptance threshold remains 7.  Only a weak-but-usable
 * per-view score in [3,6] may be promoted, and only when the already-enrolled
 * normalized raster independently clears every strict alignment gate below.
 * This is not score fusion and does not add another biometric draw: one
 * physical pose still produces exactly one authentication decision.
 *
 * Gate A from the local aggregate screen remains selected for rel71.30:
 *   genuine enrolled-view pairs rescued: 5
 *   inter-finger enrolled-view pairs rescued: 0 / 1200
 * The primary candidate's best SIGFM score must still be at least 4, so a
 * photometric match can never promote a completely weak gallery result.
 * Historical inter-finger maxima were ZNCC=.634 and agreement=.800 versus
 * the rescue minima .750/.845.  This remains local validation evidence,
 * not a population FAR claim, so rel71.30 stays private until field-tested. */
#define GX_PIXEL_RESCUE_CANDIDATE_BEST_MIN 4
#define GX_PIXEL_RESCUE_SCORE_MIN       3
#define GX_PIXEL_RESCUE_SCORE_MAX       6
#define GX_PIXEL_RESCUE_MIN_OVERLAP  1500
#define GX_PIXEL_RESCUE_MIN_ZNCC_MILLI 750
#define GX_PIXEL_RESCUE_MIN_AGREE_PERMILLE 845

/* One user-visible press is one biometric sample.  Hidden second captures made
 * the previous enrollment slow and did not add controlled diversity. */
#define GX_ENROLL_STAGES             20
#define GX_VIEWS_PER_STAGE            1
#define GX_ENROLL_VIEWS              GX_ENROLL_STAGES

/* Enrollment topology policy for a tiny partial sensor.
 *
 * The first five accepted views form a redundant anchor around the user's
 * natural placement. Later views may extend coverage, but must remain connected
 * to the accepted template graph. This prevents a valid-looking yet effectively
 * isolated capture from consuming one of the fixed 20 enrollment slots.
 *
 * Authentication threshold remains independent at 7; these much lower scores
 * are used only while the user is deliberately enrolling the same finger. */
#define GX_ENROLL_ANCHOR_STAGES              5
#define GX_ENROLL_ANCHOR_MIN_SCORE           3
#define GX_ENROLL_EXTENSION_MIN_SCORE        3
#define GX_ENROLL_EXTENSION_WEAK_SCORE       2
#define GX_ENROLL_EXTENSION_WEAK_LINKS       2

/* Reads the firmware version, which also proves the SPI dialogue works. */
static gboolean
gx_wait_plain_ack_for (FpiDeviceGoodix51A0 *self, guint8 command)
{
  guint8 rx[128], type = 0;
  int n = gx_read_frame (self, &type, rx, sizeof rx);

  return n >= 5 &&
         type == GOODIX_PKT_PLAIN &&
         rx[0] == 0xB0 &&
         rx[3] == command;
}

static gboolean
gx_driverstate_install_windows (FpiDeviceGoodix51A0 *self)
{
  int wrapper, attempt;

  /* Windows GF3658 first-contact: NOP -> 5 ms -> DriverState Install. */
  if (!gx_write_frame (self, GOODIX_PKT_PLAIN,
                       GX_AMORCE, sizeof GX_AMORCE))
    return FALSE;

  g_usleep (5000);

  for (wrapper = 0; wrapper < 2; wrapper++)
    for (attempt = 0; attempt < 2; attempt++)
      {
        if (!gx_write_frame (self, GOODIX_PKT_PLAIN,
                             GX_ENABLE, sizeof GX_ENABLE))
          return FALSE;
        if (gx_wait_plain_ack_for (self, 0x96))
          return TRUE;
      }

  return FALSE;
}

static gboolean
gx_read_fw_version_once (FpiDeviceGoodix51A0 *self, gchar *out, gsize cap)
{
  static const guint8 nop[] = {
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5
  };
  static const guint8 a8[] = {
    GOODIX_CMD_FW_VERSION, 0x03, 0x00, 0x00, 0x00, 0xFF
  };
  guint8 rx[128], cached[128];
  guint8 ty = 0, cached_ty = 0;
  int n, cached_n = 0;
  int send_index, read_index;
  gboolean acked = FALSE;

  if (!gx_write_frame (self, GOODIX_PKT_PLAIN, nop, sizeof nop))
    return FALSE;
  g_usleep (5000);

  for (send_index = 0; send_index < 2 && !acked; send_index++)
    {
      cached_n = 0;
      cached_ty = 0;

      if (!gx_write_frame (self, GOODIX_PKT_PLAIN, a8, sizeof a8))
        return FALSE;

      for (read_index = 0; read_index < 2; read_index++)
        {
          n = gx_read_frame (self, &ty, rx, sizeof rx);
          if (n <= 0)
            break;
          if (ty != GOODIX_PKT_PLAIN)
            continue;

          if (n >= 5 && rx[0] == 0xB0 && rx[3] == GOODIX_CMD_FW_VERSION)
            {
              acked = TRUE;
              break;
            }

          if (n > 3 && rx[0] != 0xB0)
            {
              memcpy (cached, rx, n);
              cached_n = n;
              cached_ty = ty;
            }
        }
    }

  if (!acked)
    return FALSE;

  if (cached_n > 3 && cached_ty == GOODIX_PKT_PLAIN)
    {
      g_strlcpy (out, (const gchar *) cached + 3,
                 MIN ((gsize) (cached_n - 3) + 1, cap));
      return TRUE;
    }

  n = gx_read_frame (self, &ty, rx, sizeof rx);
  if (n <= 3 || ty != GOODIX_PKT_PLAIN || rx[0] == 0xB0)
    return FALSE;

  g_strlcpy (out, (const gchar *) rx + 3,
             MIN ((gsize) (n - 3) + 1, cap));
  return TRUE;
}

static gboolean
gx_read_fw_version (FpiDeviceGoodix51A0 *self, gchar *out, gsize cap)
{
  int attempt;

  /* GetEvkVersionWithRetry: three initial tries. The caller owns the
   * reviewed HardResetMcu fallback and one final try. */
  for (attempt = 0; attempt < 3; attempt++)
    if (gx_read_fw_version_once (self, out, cap))
      return TRUE;

  return FALSE;
}


/* ------------------------------------------------------------------ */
/*  Session: TLS setup, background frame, detection baseline           */
/* ------------------------------------------------------------------ */

/* Runtime timing adaptation.
 *
 * rel24-rel40 persisted protocol/capture timing under /var/lib/fprint.  That
 * looked useful for slow controllers, but deep-S3/prewarm transport loss could
 * be mis-attributed as a speed problem and ratchet 100 -> 150 -> ... -> 300
 * across reboots.  rel42 deliberately keeps both scales process-local.
 *
 * Every fresh/cold lifecycle starts at the Windows-validated nominal timing.
 * A real runtime failure may loosen timings for the current fprintd lifetime,
 * while clean captures decay back toward nominal.  No lifecycle/prewarm event
 * can poison a future boot through persistent tuning state. */
#define GX_TIMING_SCALE_MIN 100
#define GX_TIMING_SCALE_MAX 300
#define GX_TIMING_SCALE_STEP 50

#define GX_CAPTURE_SCALE_MIN 100
#define GX_CAPTURE_SCALE_MAX 300
#define GX_CAPTURE_SCALE_STEP 50
#define GX_CAPTURE_CLEAN_DECAY_STREAK 8

/* Protocol timing is a controller/session characteristic, not biometric
 * evidence. Start every fresh fprintd process at Windows-nominal 100%, then
 * widen only after an observed missed ACK/FDT response. Keep the result in RAM
 * for the daemon lifetime so a recovery does not relearn the same controller.
 * Nothing is persisted across daemon restarts or boots. */
static void
gx_protocol_timing_miss (FpiDeviceGoodix51A0 *self,
                         const gchar          *source)
{
  int previous = MAX (self->timing_scale, GX_TIMING_SCALE_MIN);

  if (previous >= GX_TIMING_SCALE_MAX)
    return;

  self->timing_scale =
    MIN (previous + GX_TIMING_SCALE_STEP, GX_TIMING_SCALE_MAX);

  fp_warn ("GXFP51A0 protocol timing auto-calibration: %d%% -> %d%% "
           "after %s miss; session-local only",
           previous, self->timing_scale, source ? source : "protocol");
}

static guint
gx_capture_gap_us (FpiDeviceGoodix51A0 *self)
{
  int scale = CLAMP (self->capture_gap_scale,
                     GX_CAPTURE_SCALE_MIN, GX_CAPTURE_SCALE_MAX);

  return (guint) ((guint64) GX_SEQ_GAP_US * (guint) scale / 100u);
}

static void
gx_capture_transport_desync (FpiDeviceGoodix51A0 *self)
{
  int previous = MAX (self->capture_gap_scale, GX_CAPTURE_SCALE_MIN);

  self->capture_clean_streak = 0;
  self->capture_recovery_pending = TRUE;

  /* Lifecycle/prewarm failures say nothing about the steady-state capture
   * gap. They request a session rebuild only and must never change pacing. */
  if (self->capture_pacing_suppressed)
    {
      fp_warn ("GXFP51A0 lifecycle recovery desync: pacing remains %d%% "
               "(%u us gap); full MCU/session recovery required",
               previous, gx_capture_gap_us (self));
      return;
    }

  self->capture_gap_scale =
    MIN (previous + GX_CAPTURE_SCALE_STEP, GX_CAPTURE_SCALE_MAX);

  fp_warn ("GXFP51A0 session capture desync: pacing %d%% -> %d%% "
           "(%u us gap); full MCU/session recovery required; not persisted",
           previous, self->capture_gap_scale, gx_capture_gap_us (self));
}

static void
gx_capture_pacing_success (FpiDeviceGoodix51A0 *self)
{
  if (self->capture_retry_seen)
    {
      int previous = MAX (self->capture_gap_scale, GX_CAPTURE_SCALE_MIN);
      int protocol_floor =
        CLAMP (self->timing_scale - GX_CAPTURE_SCALE_STEP,
               GX_CAPTURE_SCALE_MIN, 250);
      int target = MIN (GX_CAPTURE_SCALE_MAX,
                        MAX (previous + GX_CAPTURE_SCALE_STEP, protocol_floor));

      self->capture_clean_streak = 0;

      /* A successfully decoded real finger frame that needed the transport
       * retry is direct evidence that nominal capture pacing was too tight for
       * this session.  Do not wait for three failed user presses: calibrate the
       * NEXT physical press immediately.  The already-observed protocol timing
       * provides a conservative floor (300% protocol -> 250% capture on the
       * reference MateBook), while controllers with nominal protocol timing
       * still move only one 50-point step.  Nothing is persisted. */
      if (target > previous)
        {
          self->capture_gap_scale = target;
          fp_info ("GXFP51A0 session capture pacing calibrated by retry-assisted "
                   "finger frame: %d%% -> %d%% (%u us gap, protocol=%d%%); "
                   "not persisted",
                   previous, self->capture_gap_scale,
                   gx_capture_gap_us (self), self->timing_scale);
        }

      self->capture_retry_seen = FALSE;
      self->warm_last_activity_us = g_get_monotonic_time ();
      return;
    }


  if (self->capture_gap_scale > GX_CAPTURE_SCALE_MIN)
    {
      self->capture_clean_streak++;
      if (self->capture_clean_streak >= GX_CAPTURE_CLEAN_DECAY_STREAK)
        {
          int previous = self->capture_gap_scale;

          self->capture_gap_scale =
            MAX (GX_CAPTURE_SCALE_MIN,
                 previous - GX_CAPTURE_SCALE_STEP);
          self->capture_clean_streak = 0;
          fp_info ("GXFP51A0 session capture pacing decayed after %d clean "
                   "captures: %d%% -> %d%% (%u us gap)",
                   GX_CAPTURE_CLEAN_DECAY_STREAK, previous,
                   self->capture_gap_scale, gx_capture_gap_us (self));
        }
    }
  else
    self->capture_clean_streak = 0;

  self->capture_retry_seen = FALSE;
  self->warm_last_activity_us = g_get_monotonic_time ();
}

/* Establishes the TLS channel. This is the expensive step, about half a
 * second, and it does not depend on when the finger arrives — so it can be
 * done once and kept. */
static gboolean
gx_tls_session (FpiDeviceGoodix51A0 *self)
{
  int att;
  gboolean diagnostic = g_getenv ("GXFP_DIAGNOSTIC_ONESHOT") != NULL;
  gboolean capture_diagnostic = g_getenv ("GXFP_DIAGNOSTIC_CAPTURE_ONCE") != NULL;
  int max_attempts = capture_diagnostic ? 2 : (diagnostic ? 1 : 5);

  if (self->tls_up)
    return TRUE;

  for (att = 1; att <= max_attempts && !self->tls_up; att++)
    {
      GCancellable *c = fpi_device_get_cancellable (FP_DEVICE (self));

      if (c && g_cancellable_is_cancelled (c))
        {
          fp_info ("TLS setup cancelled by the caller");
          break;
        }

      if (gx_upload_config_and_reqtls (self, TRUE) &&
          gx_tls_handshake (self))
        break;

      gx_tls_teardown (self);

      /* A PMK that reached D4 in an earlier session is durable trusted state.
       * A transport/configuration/handshake failure later does NOT prove that
       * the key changed, so never unlink the cache on a transient failure. */
      if (self->psk_from_cache && diagnostic)
        {
          if (att >= max_attempts)
            {
              fp_warn ("GXFP51A0 cached PMK retained after failed session; diagnostic mode will not fall back to staging");
              break;
            }
          fp_warn ("GXFP51A0 cached PMK retained after failed diagnostic TLS attempt; retrying clean target init");
        }

      /* Self-healing on desync. The protocol delays are tuned to the author's
       * unit and sit right at the edge; a different SPI controller can be
       * slower and lose sync. Adaptation is session-local and never persisted. */
      gx_protocol_timing_miss (self, "tls-handshake");

      {
        gchar fw[64] = { 0 };

        gx_gpio_reset (self);
        if (!gx_read_fw_version_stage2e (self, fw, sizeof fw) ||
            g_strcmp0 (fw, "GF_ST411SEC_APP_14115") != 0)
          {
            fp_warn ("GXFP51A0 TLS retry reset/A8 failed");
            break;
          }
        fp_info ("GXFP51A0 TLS retry reset/A8 confirmed: %s", fw);
      }
    }

  /* Production-only stale-cache recovery. Keep the validated cache file on
   * disk while probing fresh boot staging. If the newly recovered candidate
   * reaches D4, gx_pmk_cache_save() atomically replaces the old file. If it
   * does not, the last-known-good cache remains available for a later boot. */
  if (!self->tls_up && !diagnostic && self->psk_from_cache)
    {
      fp_warn ("GXFP51A0 cached PMK could not establish TLS; trying fresh staging without deleting validated cache");
      gx_pmk_clear (self);

      if (gx_upload_config_and_reqtls (self, FALSE) &&
          gx_tls_handshake (self))
        {
          fp_info ("GXFP51A0 fresh staging replaced stale cache after live TLS validation");
        }
      else
        {
          gx_tls_teardown (self);
          fp_warn ("GXFP51A0 fresh staging fallback failed; validated cache file retained");
        }
    }

  if (!self->tls_up)
    fp_warn ("no TLS session after %d/%d attempts; if this persists the sensor "
             "needs a full recovery (long reset plus an spidev rebind)",
             att - 1, max_attempts);
  else if (self->timing_scale > GX_TIMING_SCALE_MIN)
    fp_info ("GXFP51A0 session protocol timing settled at %d%%; not persisted",
             self->timing_scale);

  return self->tls_up;
}

/* Refreshes the background frame and the detection baseline.
 *
 * This must happen JUST BEFORE each operation, unlike the TLS setup above: the
 * sensor's baseline level drifts, and the whole chain rests on the
 * background-minus-finger difference. A background captured too early yields a
 * distorted image and collapses the score — measured at 11 to 28 matches with
 * a fresh background against 2 to 4 with one a few tens of seconds old. */

/* How long to wait for the finger to be lifted before taking the background. */
#define GX_BG_WAIT_MS 2000

/* One-shot diagnostic UX: give the operator time to react to the explicit
 * "sensor clear" prompt before measuring anything. Then reject a calibration
 * whose mean moved too far from the just-observed no-finger anchor. */
#define GX_DIAG_CLEAR_GRACE_SEC 3
#define GX_DIAG_BASELINE_MAX_DRIFT 20
#define GX_DIAG_BASELINE_ATTEMPTS 4
#define GX_CLEAR_WAIT_MS 15000

static gboolean
gx_wait_clean_anchor (FpiDeviceGoodix51A0 *self, int *out_mean)
{
  int cur[GXFP_FDT_ZONE_COUNT];
  int waited = 0;
  gboolean warned = FALSE;

  while (waited <= GX_CLEAR_WAIT_MS)
    {
      GCancellable *c = fpi_device_get_cancellable (FP_DEVICE (self));

      if (c && g_cancellable_is_cancelled (c))
        return FALSE;

      guint8 touchflag = 0;

      if (gx_fdt_probe_ex (self, cur, &touchflag) == 0)
        {
          int mean = gx_fdt_mean (cur);

          if (!gx_fdt_touch_is_finger (touchflag) &&
              mean >= GOODIX_FDT_ABS)
            {
              *out_mean = mean;
              fp_info ("GXFP51A0 clean calibration anchor: mean=%d", mean);
              return TRUE;
            }

          if (!warned)
            {
              fp_warn ("GXFP51A0 calibration waiting for sensor clear: mean=%d floor=%d",
                       mean, GOODIX_FDT_ABS);
              warned = TRUE;
            }
        }

      g_usleep (200 * 1000);
      waited += 200;
    }

  fp_warn ("GXFP51A0 calibration could not find a clean sensor within %d ms",
           GX_CLEAR_WAIT_MS);
  return FALSE;
}

static gboolean
gx_wait_sensor_clear (FpiDeviceGoodix51A0 *self,
                      int off_anchor_mean,
                      int *out_mean)
{
  int cur[GXFP_FDT_ZONE_COUNT];
  int waited = 0;
  gboolean warned = FALSE;

  while (waited <= GX_CLEAR_WAIT_MS)
    {
      guint8 touchflag = 0;

      if (gx_fdt_probe_ex (self, cur, &touchflag) == 0)
        {
          int mean = gx_fdt_mean (cur);

          if (!gx_fdt_touch_is_finger (touchflag) &&
              off_anchor_mean - mean <= GX_DIAG_BASELINE_MAX_DRIFT)
            {
              if (out_mean)
                *out_mean = mean;
              if (warned)
                fp_info ("GXFP51A0 sensor clear again: mean=%d anchor=%d",
                         mean, off_anchor_mean);
              return TRUE;
            }

          if (!warned)
            {
              fp_warn ("GXFP51A0 calibration waiting for sensor clear: mean=%d anchor=%d",
                       mean, off_anchor_mean);
              warned = TRUE;
            }
        }

      g_usleep (200 * 1000);
      waited += 200;
    }

  fp_warn ("GXFP51A0 sensor still not clear after %d ms",
           GX_CLEAR_WAIT_MS);
  return FALSE;
}

static gboolean
gx_diag_press_countdown (FpiDeviceGoodix51A0 *self, int off_anchor_mean)
{
  int cur[GXFP_FDT_ZONE_COUNT];
  int q;

  for (;;)
    {
      gboolean restart = FALSE;

      fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_GET_READY: keep finger OFF until zero");
      for (q = 3; q > 0; q--)
        {
          fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_PRESS_IN_%d", q);
          g_usleep (G_USEC_PER_SEC);

          guint8 touchflag = 0;

          if (gx_fdt_probe_ex (self, cur, &touchflag) == 0)
            {
              int mean = gx_fdt_mean (cur);

              if (gx_fdt_touch_is_finger (touchflag) ||
                  off_anchor_mean - mean > GX_DIAG_BASELINE_MAX_DRIFT)
                {
                  fp_warn ("GXFP51A0 CAPTURE_DIAGNOSTIC_COUNTDOWN_RESTART: REMOVE_FINGER mean=%d anchor=%d",
                           mean, off_anchor_mean);
                  if (!gx_wait_sensor_clear (self, off_anchor_mean, NULL))
                    return FALSE;
                  restart = TRUE;
                  break;
                }
            }
        }

      if (restart)
        continue;

      fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_PRESS_NOW: PRESS_AND_HOLD_NOW");
      return TRUE;
    }
}

static gboolean
gx_prepare_capture_context_once (FpiDeviceGoodix51A0 *self,
                                 gboolean capture_diagnostic)
{
  int q, k;
  int off_anchor_mean = -1;

  if (!gx_tls_session (self))
    return FALSE;


  /* The sensor drives GPIO48 level-high while a response is pending. The
   * first background capture must not race the tail of the TLS handshake:
   * wait for a proven idle-low boundary, then give the MCU a short quiesce
   * window before starting the capture recipe. This only runs on cold/context
   * preparation; the normal warm Claim path never pays this delay. */
  if (gx51_wait_irq_gpio48_low (self->irq_fd, 250) < 0)
    {
      fp_warn ("GXFP51A0 TLS settled but IRQ did not return low before calibration");
      return FALSE;
    }
  g_usleep (100 * 1000);

  if (capture_diagnostic)
    {
      fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_PREPARING_BACKGROUND: KEEP_FINGER_OFF_SENSOR");
      fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_CLEAR_SENSOR: REMOVE_FINGER_NOW");
      for (q = GX_DIAG_CLEAR_GRACE_SEC; q > 0; q--)
        {
          fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_CALIBRATION_STARTS_IN_%d", q);
          g_usleep (G_USEC_PER_SEC);
        }
    }

  self->bg_dirty = FALSE;

  /* Both the background and the detection baseline are only meaningful with
   * the finger LIFTED. A finger left resting on the sensor when the operation
   * starts ends up in the background, the difference then cancels out, and the
   * capture is worthless — the observed symptom being verifications collapsing
   * from about 100 matches to 6 with no change in how the finger was placed.
   *
   * So take the background, ask the sensor whether it saw a finger, and redo it
   * if so. Background first, detection second, matching the order the rest of
   * the driver uses.
   *
   * The presence test here is the ABSOLUTE one, the only usable choice, since
   * the relative baseline is precisely what we are about to establish. */
  if (!self->bg_frame)
    self->bg_frame = g_malloc (sizeof (guint16) * GOODIX_IMG_PIXELS);

  {
    int cur[GXFP_FDT_ZONE_COUNT];
    gint64 t0 = g_get_monotonic_time ();

    /* Production and diagnostics use the same invariant: calibration starts
     * only from a proven finger-off sample. The previous production path used
     * a 2 s timeout and then accepted a contaminated background anyway; on
     * reference MateBook that learned idle≈211 while the true finger-off level is≈353,
     * making every subsequent press invisible. Never do that. */
    if (!gx_wait_clean_anchor (self, &off_anchor_mean))
      return FALSE;

    if (capture_diagnostic)
      fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_CLEAR_SENSOR_ANCHOR: mean=%d",
               off_anchor_mean);

    for (;;)
      {
        int clear_mean = off_anchor_mean;
        int mean;

        if (!gx_wait_sensor_clear (self, off_anchor_mean, &clear_mean))
          return FALSE;
        if (clear_mean > off_anchor_mean)
          off_anchor_mean = clear_mean;

        if (!gx_capture_avg (self, GOODIX_BG_FRAMES, self->bg_frame))
          return FALSE;

        /* A finger may land during the ~2 s background capture. Reject that
         * frame and wait for release rather than baking the finger into the
         * reference image. */
        guint8 touchflag = 0;

        if (gx_fdt_probe_ex (self, cur, &touchflag) != 0)
          return FALSE;

        mean = gx_fdt_mean (cur);
        if (mean > off_anchor_mean)
          off_anchor_mean = mean;

        if (!gx_fdt_touch_is_finger (touchflag) &&
            off_anchor_mean - mean <= GX_DIAG_BASELINE_MAX_DRIFT)
          break;

        fp_warn ("GXFP51A0 background contaminated by touch: mean=%d anchor=%d touch=0x%02x; discarding",
                 mean, off_anchor_mean, touchflag);
        if (!gx_wait_sensor_clear (self, off_anchor_mean, NULL))
          return FALSE;
      }

    self->bg_last_refresh_us = g_get_monotonic_time ();
    fp_info ("session ready in %d ms; fresh ImageBase armed",
             (int) ((self->bg_last_refresh_us - t0) / 1000));
  }

  /* Detection baseline. In one-shot diagnostic mode, validate it against the
   * no-finger anchor. If the user touched while those four probes were being
   * collected, discard them and retry instead of learning the finger as idle. */
  {
    int attempt;

    self->have_fdt = FALSE;
    for (attempt = 1; attempt <= GX_DIAG_BASELINE_ATTEMPTS; attempt++)
      {
        int t[GXFP_FDT_ZONE_COUNT];
        int acc[GXFP_FDT_ZONE_COUNT] = { 0 };
        int nb = 0;
        int baseline_mean;

        for (q = 0; q < 4; q++)
          {
            guint8 touchflag = 0;

            if (gx_fdt_probe_ex (self, t, &touchflag) == 0 &&
                !gx_fdt_touch_is_finger (touchflag))
              {
                for (k = 0; k < (int) GXFP_FDT_ZONE_COUNT; k++)
                  acc[k] += t[k];
                nb++;
              }
          }

        if (!nb)
          continue;

        for (k = 0; k < (int) GXFP_FDT_ZONE_COUNT; k++)
          self->fdt_base[k] = acc[k] / nb;
        baseline_mean = gx_fdt_mean (self->fdt_base);

        if (off_anchor_mean >= 0 &&
            off_anchor_mean - baseline_mean > GX_DIAG_BASELINE_MAX_DRIFT)
          {
              fp_warn ("GXFP51A0 FDT baseline contaminated by touch: mean=%d anchor=%d attempt=%d/%d",
                     baseline_mean, off_anchor_mean, attempt,
                     GX_DIAG_BASELINE_ATTEMPTS);
            if (!gx_wait_sensor_clear (self, off_anchor_mean, NULL))
              return FALSE;
            continue;
          }

        if (baseline_mean > off_anchor_mean)
          off_anchor_mean = baseline_mean;

        self->have_fdt = TRUE;
        self->fdt_abs = baseline_mean - GOODIX_FDT_ABS_MARGIN;
        fp_info ("FDT auto-calibrated: idle mean=%d -> floor=%d (drop>%d)",
                 baseline_mean, self->fdt_abs, GOODIX_FDT_DROP);
        if (capture_diagnostic)
          fp_info ("GXFP51A0 CAPTURE_DIAGNOSTIC_BASELINE_STABLE: mean=%d anchor=%d",
                   baseline_mean, off_anchor_mean);
        break;
      }
  }

  if (capture_diagnostic && self->have_fdt &&
      !gx_diag_press_countdown (self, off_anchor_mean))
    return FALSE;

  return self->have_fdt;
}

#define GX_PREPARE_ATTEMPTS 2
#define GX_BG_FRESH_MAX_US (G_GINT64_CONSTANT(20) * G_USEC_PER_SEC)
G_STATIC_ASSERT (GX_BG_FRESH_MAX_US == G_GINT64_CONSTANT(20000000));

static gint64
gx_background_age_us (FpiDeviceGoodix51A0 *self)
{
  gint64 now;

  if (!self->bg_frame || self->bg_last_refresh_us <= 0)
    return G_MAXINT64;
  now = g_get_monotonic_time ();
  return MAX (G_GINT64_CONSTANT(0), now - self->bg_last_refresh_us);
}

static gboolean
gx_background_is_fresh (FpiDeviceGoodix51A0 *self)
{
  return gx_background_age_us (self) <= GX_BG_FRESH_MAX_US;
}

static void
gx_invalidate_capture_context (FpiDeviceGoodix51A0 *self)
{
  self->production_ready = FALSE;
  self->warm_valid = FALSE;
  self->have_fdt = FALSE;
  self->bg_dirty = FALSE;
  self->bg_last_refresh_us = 0;
  self->tls_rxlen = 0;
  self->tls_rxpos = 0;
}

static gboolean
gx_recover_capture_context (FpiDeviceGoodix51A0 *self)
{
  gchar fw[64] = { 0 };
  g_autoptr(GError) reopen_error = NULL;

  gx_invalidate_capture_context (self);
  gx_tls_teardown (self);

  /* A timed-out TLS/image transaction can leave ST411 state coupled to the
   * currently open spidev file.  Do not reset through an already-open SPI
   * transport.  Close both SPI/IRQ handles, pulse the reviewed GPIO264 reset,
   * then open a completely fresh transport before proving the A8 boundary.
   *
   * This is deliberately NOT a sysfs unbind/rebind and needs no kernel,
   * distro, udev or desktop-specific helper.  It is the narrowest native
   * userspace equivalent to a cold device boundary and is safe to use inside
   * a live libfprint operation. */
  gx_transport_close (self);
  gx_gpio_reset (self);

  if (!gx_transport_open (FP_DEVICE (self), &reopen_error))
    {
      fp_warn ("GXFP51A0 capture-context recovery could not reopen transport: %s",
               reopen_error ? reopen_error->message : "unknown error");
      return FALSE;
    }

  fp_warn ("GXFP51A0 RESET_TRACE transport reopened after detached GPIO264 reset");

  if (!gx_read_fw_version_stage2e (self, fw, sizeof fw) ||
      g_strcmp0 (fw, "GF_ST411SEC_APP_14115") != 0)
    {
      fp_warn ("GXFP51A0 capture-context recovery could not re-establish the firmware boundary");
      return FALSE;
    }

  fp_info ("GXFP51A0 capture-context recovery reset/A8 confirmed: %s", fw);
  return TRUE;
}

static gboolean
gx_prepare_capture_context (FpiDeviceGoodix51A0 *self,
                            gboolean capture_diagnostic)
{
  int attempt;
  gboolean previous_pacing_suppression;
  gboolean ok = FALSE;

  /* Diagnostics are intentionally single-shot so protocol failures remain
   * visible to the research harness.  Production operations get a bounded
   * whole-session retry; each retry starts after a hard reset because a
   * missing TLS image after ACK may have advanced unknown sensor state. */
  if (capture_diagnostic ||
      g_getenv ("GXFP_DIAGNOSTIC_ONESHOT") ||
      g_getenv ("GXFP_DIAGNOSTIC_TLS_ONLY"))
    return gx_prepare_capture_context_once (self, capture_diagnostic);

  /* Context preparation is lifecycle/calibration work, not biometric
   * performance evidence.  A GET_IMAGE miss while building background/FDT
   * must request recovery but must never slow the capture gap used by the
   * first real finger.  Preserve an already-adapted scale from an earlier
   * genuine biometric capture; only suppress further adaptation here. */
  previous_pacing_suppression = self->capture_pacing_suppressed;
  self->capture_pacing_suppressed = TRUE;

  for (attempt = 1; attempt <= GX_PREPARE_ATTEMPTS; attempt++)
    {
      GCancellable *c = fpi_device_get_cancellable (FP_DEVICE (self));

      if (c && g_cancellable_is_cancelled (c))
        goto out;

      if (gx_prepare_capture_context_once (self, FALSE))
        {
          self->capture_recovery_pending = FALSE;
          ok = TRUE;
          goto out;
        }

      fp_warn ("GXFP51A0 capture-context preparation failed attempt=%d/%d",
               attempt, GX_PREPARE_ATTEMPTS);

      if (!gx_recover_capture_context (self))
        break;
    }

out:
  self->capture_pacing_suppressed = previous_pacing_suppression;
  return ok;
}

static gboolean
gx_session_start (FpiDeviceGoodix51A0 *self)
{
  gboolean capture_diagnostic =
    g_getenv ("GXFP_DIAGNOSTIC_CAPTURE_ONCE") != NULL;

  /* A Verify/Identify operation may already be open when the machine enters
   * deep S3. In that case fprintd does not necessarily close/reopen the device,
   * so gx_dev_open() never gets a chance to invalidate stale TLS/FDT state.
   * Recover natively inside the existing operation, before any post-resume FDT
   * or GET_IMAGE command can hit the dead MCU session. */
  if (self->force_cold_reset)
    {
      fp_warn ("GXFP51A0 active resume recovery: rebuilding cold sensor "
               "context before continuing authentication");
      gx_warm_abandon (self);
      self->capture_recovery_pending = FALSE;
      self->capture_gap_scale = 0;
      self->capture_clean_streak = 0;
      self->capture_retry_seen = FALSE;
      self->capture_pacing_suppressed = TRUE;

      gx_gpio_reset (self);
      self->force_cold_reset = FALSE;

      if (!gx_cold_prepare (self))
        return FALSE;

      return gx_wakeup_mcu (self);
    }

  if (self->capture_recovery_pending)
    {
      fp_info ("GXFP51A0 rebuilding capture context after transport desync");
      if (!gx_recover_capture_context (self))
        return FALSE;
    }

  /* Diagnostics deliberately retain their interactive preparation so the
   * research harness can observe each boundary. Normal fprintd clients are
   * prepared during device open/Claim, before EnrollStart or VerifyStart is
   * returned to the desktop. */
  if (capture_diagnostic || g_getenv ("GXFP_DIAGNOSTIC_ONESHOT") ||
      g_getenv ("GXFP_DIAGNOSTIC_TLS_ONLY"))
    {
      if (!gx_prepare_capture_context (self, capture_diagnostic))
        return FALSE;
      return g_getenv ("GXFP_DIAGNOSTIC_TLS_ONLY") != NULL ||
             gx_wakeup_mcu (self);
    }

  if (self->production_ready && self->tls_up && self->have_fdt &&
      self->bg_frame)
    {
      /* FAST_READY proves that the retained TLS/FDT transport is alive, but
       * ImageBase drifts much faster than the TLS lifecycle. reference MateBook 13 traces
       * show that a background only tens of seconds old can collapse genuine
       * scores from double digits to 2-4. Keep Claim sub-second, then refresh
       * the imaging reference here on the existing SESSION worker before
       * biometric READY. If the user is already touching the sensor,
       * gx_warm_validate() safely defers the refresh and the bounded reposition
       * path remains available. */
      if (!gx_background_is_fresh (self))
        {
          gint64 age_us = gx_background_age_us (self);
          gint64 previous_refresh = self->bg_last_refresh_us;

          fp_warn ("GXFP51A0 AUTH_TRACE ImageBase age=%d ms exceeds %d ms; "
                   "refreshing before biometric READY",
                   age_us == G_MAXINT64 ? -1 : (int) (age_us / 1000),
                   (int) (GX_BG_FRESH_MAX_US / 1000));

          if (!gx_warm_validate (self))
            {
              fp_warn ("GXFP51A0 AUTH_TRACE stale ImageBase refresh failed; "
                       "rebuilding cold capture context");
              gx_warm_abandon (self);
              gx_gpio_reset (self);
              if (!gx_cold_prepare (self))
                return FALSE;
            }
          else if (self->bg_last_refresh_us > previous_refresh)
            fp_warn ("GXFP51A0 AUTH_TRACE fresh ImageBase ready before biometric READY");
          else
            fp_warn ("GXFP51A0 AUTH_TRACE ImageBase refresh deferred by touch; "
                     "using retained reference for this pose");
        }

      fp_info ("GXFP51A0 reusing capture context prepared during device open");
      return gx_wakeup_mcu (self);
    }

  /* Defensive fallback for non-fprintd clients that may reach an operation
   * without a normal Claim/Open lifecycle. */
  if (!gx_prepare_capture_context (self, FALSE))
    return FALSE;

  self->capture_recovery_pending = FALSE;
  self->production_ready = TRUE;
  return gx_wakeup_mcu (self);
}



/* Pre-processing: subtract the background, then remove row and column
 * banding with a median. */

static double *
gx_preprocess (FpiDeviceGoodix51A0 *self, const guint16 *px)
{
  const int n = GOODIX_IMG_PIXELS;
  double *out = g_new (double, n);
  g_autofree double *sorted = g_new (double, n);
  double pos, frac, p54;
  int lo, hi;

  /* Exact-target GXFP51A0 imaging: frame-background has a drifting DC
   * component.  Recenter at the ~54th percentile, clip negatives, and keep
   * one physical frame per press.  The matcher wrapper performs the final
   * percentile stretch and unsharp mask. */
  for (int i = 0; i < n; i++)
    {
      out[i] = (double) px[i] - self->bg_frame[i];
      sorted[i] = out[i];
    }

  qsort (sorted, n, sizeof *sorted, gx_cmp_dbl);
  pos = 0.54 * (double) (n - 1);
  lo = (int) floor (pos);
  hi = (int) ceil (pos);
  frac = pos - lo;
  p54 = sorted[lo] * (1.0 - frac) + sorted[hi] * frac;

  for (int i = 0; i < n; i++)
    out[i] = MAX (0.0, out[i] - p54);

  return out;
}

/* Is a finger actually down? Costs about 33 ms. */
static gboolean
gx_finger_present (FpiDeviceGoodix51A0 *self)
{
  int cur[GXFP_FDT_ZONE_COUNT];
  guint8 touchflag = 0;

  if (gx_fdt_probe_ex (self, cur, &touchflag) != 0)
    return FALSE;
  return gx_fdt_touch_is_finger (touchflag) ||
         gx_fdt_drop (self->fdt_base, cur) > GOODIX_FDT_DROP ||
         gx_fdt_mean (cur) < self->fdt_abs;
}

/* Windows OnRetryCaptureIMG keeps the current physical press alive:
 * FDT-manual first confirms that the finger is still down, then another
 * image command 0x20 is issued without waiting for a lift/new FDT-down IRQ.
 * This is deliberately a new biometric image, not the transport-level
 * resend performed inside gx_send_plain_drain(). */
static gboolean
gx_capture_retry_same_press_frame (FpiDeviceGoodix51A0 *self,
                                   guint16             *px,
                                   gboolean            *finger_still_down)
{
  struct gxfp_target_packet packet;
  g_autofree guint8 *rec = g_malloc0 (GOODIX_RX_MAX);
  g_autofree guint8 *plain = g_malloc0 (GOODIX_RX_MAX);
  int cur[GXFP_FDT_ZONE_COUNT];
  guint8 touchflag = 0;
  int raw;
  gssize got;

  g_return_val_if_fail (finger_still_down != NULL, FALSE);
  *finger_still_down = FALSE;

  if (gx_fdt_probe_ex (self, cur, &touchflag) != 0)
    {
      fp_warn ("same-press RetryCaptureIMG FDT-manual failed; stopping retries");
      return FALSE;
    }

  {
    int mean = gx_fdt_mean (cur);
    int drop = gx_fdt_drop (self->fdt_base, cur);

    fp_warn ("GXFP51A0 VERIFY_TRACE same-press FDT mean=%d drop=%d "
             "floor=%d threshold_drop=%d touch=0x%02x zones=%u",
             mean, drop, self->fdt_abs, GOODIX_FDT_DROP,
             touchflag, gx_fdt_touch_count (touchflag));
    if (!(gx_fdt_touch_is_finger (touchflag) ||
          drop > GOODIX_FDT_DROP || mean < self->fdt_abs))
      {
        fp_warn ("GXFP51A0 VERIFY_TRACE RetryCaptureIMG stopped: "
                 "finger not detected on current press");
        return TRUE;
      }
  }

  *finger_still_down = TRUE;

  /* The Windows-style RetryCaptureIMG path is a fresh sensor image request,
   * not a transport replay.  FDT-manual has just completed a separate MCU
   * transaction; give the sensor the same session-calibrated inter-command
   * settling window used by the normal capture recipe before issuing 0x20.
   *
   * On the reference MateBook, successful rel56 traces had later same-press
   * images accepted on their first GET_IMAGE, while failing rel58 traces sent
   * the retry immediately after FDT and every image needed the no-evidence
   * transport replay.  This gap is process-local, bounded to 30..90 ms and
   * never changes biometric scoring. */
  fp_warn ("GXFP51A0 VERIFY_TRACE RetryCaptureIMG pacing barrier=%u us",
           gx_capture_gap_us (self));
  g_usleep (gx_capture_gap_us (self));

  if (!gxfp_build_get_image (&packet))
    return FALSE;

  self->capture_retry_seen = FALSE;
  if (!gx_send_plain_drain (self, packet.inner, packet.inner_len, NULL, NULL))
    return FALSE;

  raw = gx_take_tls_frame (self, rec, GOODIX_RX_MAX);
  if (raw < 0)
    raw = gx_fail_get_image_after_tls_timeout (self);

  got = raw > 0
          ? gx_tls_decrypt_record (self->tls, rec, (gsize) raw,
                                   plain, GOODIX_RX_MAX)
          : -1;
  if (got != (gssize) GXFP_IMAGE_PLAINTEXT_LEN)
    {
      fp_warn ("same-press RetryCaptureIMG failed (%d raw, %ld plain); "
               "forcing full session recovery",
               raw, (long) got);
      if (!self->capture_recovery_pending)
        gx_capture_transport_desync (self);
      return FALSE;
    }

  if (!gxfp_decode_image_plaintext (plain, (gsize) got, px))
    {
      fp_warn ("same-press RetryCaptureIMG could not decode transport raster");
      return FALSE;
    }

  return TRUE;
}


#ifdef GXFP51A0_DEVELOPER
/* Dumps captures for offline evaluation. Enabled simply by the dump directory
 * existing, rather than by an environment variable, which cannot conveniently
 * be set on an already-running systemd service.
 *
 * pN.bin holds the raw frame as 16-bit samples and pN.bin.bg the matching
 * background, so that recorded sets can be replayed through the offline
 * matcher bench. */
#define GX_DUMP_DIR "/run/goodix51a0/dump"

static void
gx_dump_capture (FpiDeviceGoodix51A0 *self, const guint16 *px)
{
  static int seq = 0;
  g_autofree gchar *p = NULL, *b = NULL;

  if (!g_file_test (GX_DUMP_DIR, G_FILE_TEST_IS_DIR) || !self->bg_frame)
    return;

  /* Continue past captures already there, so a restart does not overwrite
   * an earlier collection run. */
  do
    {
      g_free (p);
      p = g_strdup_printf ("%s/p%03d.bin", GX_DUMP_DIR, ++seq);
    }
  while (g_file_test (p, G_FILE_TEST_EXISTS) && seq < 9999);

  b = g_strdup_printf ("%s.bg", p);
  if (g_file_set_contents_full (
        p, (const gchar *) px,
        sizeof (guint16) * GOODIX_IMG_PIXELS,
        G_FILE_SET_CONTENTS_CONSISTENT, 0600, NULL) &&
      g_file_set_contents_full (
        b, (const gchar *) self->bg_frame,
        sizeof (guint16) * GOODIX_IMG_PIXELS,
        G_FILE_SET_CONTENTS_CONSISTENT, 0600, NULL))
    fp_info ("capture saved: %s", p);
}

#endif

static GxSiftFeatures *
gx_features_from_pixels (FpiDeviceGoodix51A0 *self, const guint16 *px)
{
  g_autofree double *img = NULL;
  double m = 0, v = 0;
  int i;

  img = gx_preprocess (self, px);

  /* This quality gate is ESSENTIAL. The keypoint detector always returns
   * maxima, even on pure noise, so the number of points says nothing about
   * whether a capture is usable. What separates them is contrast: about 5
   * with no finger against about 200 with one. */
  for (i = 0; i < GOODIX_IMG_PIXELS; i++)
    m += img[i];
  m /= GOODIX_IMG_PIXELS;
  for (i = 0; i < GOODIX_IMG_PIXELS; i++)
    { double d = img[i] - m; v += d * d; }
  v = sqrt (v / GOODIX_IMG_PIXELS);
  if (v < GOODIX_FINGER_STD / 3.0)
    {
      fp_info ("capture rejected: contrast %.0f (no finger?)", v);
      return NULL;
    }

  return gx_sift_extract (img, GOODIX_IMG_WIDTH, GOODIX_IMG_HEIGHT);
}

/* Captures one press and extracts its descriptors. */
static GxSiftFeatures *
gx_capture_features (FpiDeviceGoodix51A0 *self)
{
  guint16 px[GOODIX_IMG_PIXELS];

  if (!gx_capture_frame (self, px, FALSE))
    return NULL;
  return gx_features_from_pixels (self, px);
}
