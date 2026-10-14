/*
 * Excerpt of fingerprint/driver/goodix51a0/goodix51a0.c
 * (GodsQuantum/Huawei-Matebook-13-Linux, Copyright (C) 2026 Benjamin Allègre / Sigfrodr).
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#define GX_SLEEP_DELTA_STALE_US (250 * 1000)
/* Keep a healthy same-process context across an awake workday.  This is not a
 * blind lease: every new Claim still performs a cheap FDT liveness probe, and
 * suspend or any failed liveness check forces the full recovery path. */
#define GX_WARM_IDLE_TTL_US (G_GINT64_CONSTANT(12) * 60 * 60 * G_USEC_PER_SEC)
G_STATIC_ASSERT (GX_WARM_IDLE_TTL_US == G_GINT64_CONSTANT(43200000000));

static gboolean
gx_sleep_delta_us (gint64 *out)
{
  struct timespec boot = { 0, }, mono = { 0, };

  g_return_val_if_fail (out != NULL, FALSE);

  if (clock_gettime (CLOCK_MONOTONIC, &boot) != 0 ||
      clock_gettime (CLOCK_UPTIME_RAW, &mono) != 0)
    return FALSE;

  *out = ((gint64) boot.tv_sec - (gint64) mono.tv_sec) * G_USEC_PER_SEC +
         ((gint64) boot.tv_nsec - (gint64) mono.tv_nsec) / 1000;
  return TRUE;
}

static gboolean
gx_warm_crossed_sleep (FpiDeviceGoodix51A0 *self)
{
  gint64 now = 0;

  if (!self->warm_sleep_clock_valid)
    return FALSE;

  if (!gx_sleep_delta_us (&now))
    return FALSE;

  if (now - self->warm_sleep_delta_us > GX_SLEEP_DELTA_STALE_US)
    {
      fp_info ("GXFP51A0 sleep boundary detected: boottime-monotonic advanced by %d ms",
               (int) ((now - self->warm_sleep_delta_us) / 1000));
      return TRUE;
    }

  return FALSE;
}

static gboolean
gx_warm_idle_expired (FpiDeviceGoodix51A0 *self)
{
  gint64 now;

  if (!self->warm_valid || self->warm_last_activity_us <= 0)
    return FALSE;

  now = g_get_monotonic_time ();
  if (now - self->warm_last_activity_us > GX_WARM_IDLE_TTL_US)
    {
      fp_info ("GXFP51A0 warm context idle for %d s; forcing cold rebuild",
               (int) ((now - self->warm_last_activity_us) / G_USEC_PER_SEC));
      return TRUE;
    }

  return FALSE;
}

static void
gx_warm_abandon (FpiDeviceGoodix51A0 *self)
{
  /* Host-only invalidation.  Use this after suspend or another lifecycle
   * boundary where sensor-side TLS state cannot be trusted: never send a
   * close_notify over a session that may no longer exist. */
  self->warm_valid = FALSE;
  self->production_ready = FALSE;
  self->have_fdt = FALSE;
  self->bg_dirty = FALSE;
  self->bg_last_refresh_us = 0;
  self->tls_rxlen = 0;
  self->tls_rxpos = 0;
  self->tls_up = FALSE;
  self->warm_last_activity_us = 0;
  self->warm_sleep_delta_us = 0;
  self->warm_sleep_clock_valid = FALSE;
  g_clear_pointer (&self->tls, gx_tls_free);
  gx_pmk_clear (self);
  g_clear_pointer (&self->bg_frame, g_free);
}

static void
gx_warm_discard (FpiDeviceGoodix51A0 *self)
{
  if (self->tls_up && self->spi_fd >= 0 && self->irq_fd >= 0)
    gx_tls_teardown (self);
  gx_warm_abandon (self);
}

static gboolean
gx_warm_available (FpiDeviceGoodix51A0 *self)
{
  return self->warm_valid && self->tls_up && self->tls &&
         self->bg_frame && self->have_fdt;
}

static gboolean
gx_warm_fast_ready (FpiDeviceGoodix51A0 *self)
{
  int cur[GXFP_FDT_ZONE_COUNT];
  guint8 touchflag = 0;
  int mean;
  gint64 t0 = g_get_monotonic_time ();

  /* Fast path for an already prepared sensor: prove only that the FDT path is
   * alive.  Do not take a new background image before telling the desktop the
   * reader is ready; that old rebase made every normal lock pay 1-20 seconds
   * before the user could even be detected.  Sleep is rejected before this
   * function and any failed FDT probe falls back to the full validated path. */
  if (gx_fdt_probe_ex (self, cur, &touchflag) != 0)
    return FALSE;

  mean = gx_fdt_mean (cur);
  self->warm_last_activity_us = g_get_monotonic_time ();

  if (!gx_fdt_touch_is_finger (touchflag) && mean >= GOODIX_FDT_ABS)
    {
      memcpy (self->fdt_base, cur, sizeof self->fdt_base);
      self->fdt_abs = mean - GOODIX_FDT_ABS_MARGIN;
      self->have_fdt = TRUE;
    }

  fp_warn ("GXFP51A0 FAST_READY in %d ms: FDT alive touch=0x%02x mean=%d "
           "floor=%d; reusing prepared TLS/background",
           (int) ((self->warm_last_activity_us - t0) / 1000),
           touchflag, mean, self->fdt_abs);
  return TRUE;
}

static gboolean
gx_warm_validate (FpiDeviceGoodix51A0 *self)
{
  g_autofree guint16 *fresh_bg = g_new (guint16, GOODIX_IMG_PIXELS);
  gboolean previous_pacing_suppression = self->capture_pacing_suppressed;
  gboolean ok = FALSE;
  int cur[GXFP_FDT_ZONE_COUNT];
  int after[GXFP_FDT_ZONE_COUNT];
  guint8 before_touchflag = 0;
  guint8 after_touchflag = 0;
  int before_mean;
  int after_mean;
  gint64 t0 = g_get_monotonic_time ();

  /* Failure while validating a retained lifecycle context is evidence that the
   * session went stale, not that steady-state capture pacing is too fast. */
  self->capture_pacing_suppressed = TRUE;

  if (gx_fdt_probe_ex (self, cur, &before_touchflag) != 0)
    goto out;

  before_mean = gx_fdt_mean (cur);

  /* A user may already be holding the power-button sensor when Claim/Open runs.
   * Never overwrite ImageBase/background with a finger frame.  Sleep and idle
   * boundaries were handled before this function, so an otherwise warm
   * context with a responsive FDT may proceed and let the real Verify capture
   * exercise TLS.  The previous clean background is retained for this press. */
  if (gx_fdt_touch_is_finger (before_touchflag) ||
      before_mean < GOODIX_FDT_ABS)
    {
      self->warm_last_activity_us = g_get_monotonic_time ();
      fp_warn ("GXFP51A0 WARM_REBASE deferred: finger already present "
               "(FDT mean=%d floor=%d); retaining previous background",
               before_mean, GOODIX_FDT_ABS);
      ok = TRUE;
      goto out;
    }

  /* Exact-target imaging depends on a no-finger background. Refresh only
   * the ImageBase plane here: cold preparation owns OTP-derived DAC writes.
   * Rewriting those DACs during a warm NeedUpdateImageBase-style transition
   * conflates calibration with baseline refresh and can reset useful sensor
   * state. Reuse the warm T0 validation image as the new background, then
   * prove the sensor stayed clear before adopting it. */
  if (!gx_capture_warm_background_frame (self, fresh_bg))
    {
      fp_warn ("GXFP51A0 warm FDT answered but GET_IMAGE/TLS validation failed");
      goto out;
    }

  if (gx_fdt_probe_ex (self, after, &after_touchflag) != 0)
    goto out;

  after_mean = gx_fdt_mean (after);
  if (gx_fdt_touch_is_finger (after_touchflag) ||
      after_mean < GOODIX_FDT_ABS)
    {
      self->warm_last_activity_us = g_get_monotonic_time ();
      fp_warn ("GXFP51A0 WARM_REBASE discarded: finger landed during "
               "background capture (FDT mean=%d floor=%d)",
               after_mean, GOODIX_FDT_ABS);
      ok = TRUE;
      goto out;
    }

  memcpy (self->bg_frame, fresh_bg,
          sizeof (guint16) * GOODIX_IMG_PIXELS);
  self->bg_last_refresh_us = g_get_monotonic_time ();
  memcpy (self->fdt_base, after, sizeof self->fdt_base);
  self->fdt_abs = after_mean - GOODIX_FDT_ABS_MARGIN;
  self->have_fdt = TRUE;
  self->warm_last_activity_us = g_get_monotonic_time ();

  fp_warn ("GXFP51A0 WARM_REBASE refreshed background+FDT in %d ms "
           "(idle=%d floor=%d)",
           (int) ((self->warm_last_activity_us - t0) / 1000),
           after_mean, self->fdt_abs);
  ok = TRUE;

out:
  self->capture_pacing_suppressed = previous_pacing_suppression;
  return ok;
}

static gboolean
gx_cold_prepare (FpiDeviceGoodix51A0 *self)
{
  gchar fw[64] = "";

  self->fdt_abs = GOODIX_FDT_ABS;

  /* A fresh fprintd process starts at nominal timing, but a same-process cold
   * recovery keeps any protocol timing already proven necessary by missed
   * ACK/FDT responses. This is RAM-only and therefore cannot ratchet across
   * boots. Capture pacing remains an independent biometric-path setting. */
  if (self->timing_scale < GX_TIMING_SCALE_MIN)
    self->timing_scale = GX_TIMING_SCALE_MIN;
  if (self->capture_gap_scale < GX_CAPTURE_SCALE_MIN)
    self->capture_gap_scale = GX_CAPTURE_SCALE_MIN;

  if (!gx_driverstate_install_windows (self))
    {
      fp_info ("GXFP51A0: DriverState silent; applying reviewed Windows fallback reset");
      gx_gpio_reset (self);
      fp_warn ("GXFP51A0: continuing to init_MCU after DriverState fallback as Windows does");
    }

  if (gx_read_fw_version (self, fw, sizeof fw))
    fp_info ("GXFP51A0 firmware: %s", fw);
  else
    {
      fp_info ("GXFP51A0: three A8 attempts silent; applying common-init reset fallback");
      gx_gpio_reset (self);
      if (gx_read_fw_version_once (self, fw, sizeof fw))
        fp_info ("GXFP51A0 firmware after reset fallback: %s", fw);
      else
        fp_warn ("GXFP51A0: no A8 firmware response after final fallback");
    }

  fp_info ("GXFP51A0 opened; experimental PMK/TLS/capture path enabled");

  if (g_getenv ("GXFP_DIAGNOSTIC_CAPTURE_ONCE") ||
      g_getenv ("GXFP_DIAGNOSTIC_ONESHOT") ||
      g_getenv ("GXFP_DIAGNOSTIC_TLS_ONLY"))
    return FALSE;

  /* Claim/open is the authoritative preparation boundary.  Unlike rel24/25
   * probe prewarm, this uses the bounded recovery wrapper, including a fresh
   * MCU boundary between attempts and the production PMK fallback. */
  if (!gx_prepare_capture_context (self, FALSE))
    {
      fp_warn ("GXFP51A0 cold preparation failed after bounded recovery");
      gx_invalidate_capture_context (self);
      gx_tls_teardown (self);
      gx_pmk_clear (self);
      g_clear_pointer (&self->bg_frame, g_free);
      return FALSE;
    }

  /* Protocol ACK/FDT misses measured during this cold preparation are useful
   * controller-speed evidence before the user ever touches the sensor.  Seed
   * the capture gap conservatively from that evidence so the first biometric
   * GET_IMAGE does not have to fail once merely to learn the same fact.
   * 300% protocol -> 250% capture on the reference MateBook; a nominal/fast
   * controller stays at 100%.  This remains process-local and non-persistent. */
  {
    int protocol_floor =
      CLAMP (self->timing_scale - GX_CAPTURE_SCALE_STEP,
             GX_CAPTURE_SCALE_MIN, 250);
    int previous = MAX (self->capture_gap_scale, GX_CAPTURE_SCALE_MIN);

    if (protocol_floor > previous)
      {
        self->capture_gap_scale = protocol_floor;
        fp_info ("GXFP51A0 capture pacing seeded from protocol calibration: "
                 "%d%% -> %d%% (%u us gap, protocol=%d%%); not persisted",
                 previous, self->capture_gap_scale,
                 gx_capture_gap_us (self), self->timing_scale);
      }
  }

  self->capture_recovery_pending = FALSE;
  self->capture_pacing_suppressed = FALSE;
  self->production_ready = TRUE;
  self->warm_valid = TRUE;
  self->warm_last_activity_us = g_get_monotonic_time ();
  self->warm_sleep_clock_valid =
    gx_sleep_delta_us (&self->warm_sleep_delta_us);
  fp_info ("GXFP51A0 production capture context ready; native warm state armed "
           "(capture pacing=%d%% gap=%u us)",
           self->capture_gap_scale, gx_capture_gap_us (self));
  return TRUE;
}

