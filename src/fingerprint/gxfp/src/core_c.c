/*
 * Excerpt of fingerprint/driver/goodix51a0/goodix51a0.c
 * (GodsQuantum/Huawei-Matebook-13-Linux, Copyright (C) 2026 Benjamin Allègre / Sigfrodr).
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
/* Pure excerpts of goodix51a0.c (matching / enrollment) */
#define GX_VERIFY_MAX_ATTEMPTS_CLI 3
static gboolean
gx_pixel_rescue_view_pass (const GxSiftFeatures *probe,
                           const GxSiftFeatures *enrolled,
                           int                   score,
                           int                  *out_inliers,
                           int                  *out_overlap,
                           int                  *out_zncc,
                           int                  *out_agree)
{
  int inliers = 0, overlap = 0, zncc = 0, agree = 0;

  if (!probe || !enrolled ||
      score < GX_PIXEL_RESCUE_SCORE_MIN ||
      score > GX_PIXEL_RESCUE_SCORE_MAX)
    return FALSE;

  if (!gx_sift_pixel_overlap_metrics (probe, enrolled,
                                      &inliers, &overlap,
                                      &zncc, &agree))
    return FALSE;

  if (out_inliers)
    *out_inliers = inliers;
  if (out_overlap)
    *out_overlap = overlap;
  if (out_zncc)
    *out_zncc = zncc;
  if (out_agree)
    *out_agree = agree;

  return overlap >= GX_PIXEL_RESCUE_MIN_OVERLAP &&
         zncc >= GX_PIXEL_RESCUE_MIN_ZNCC_MILLI &&
         agree >= GX_PIXEL_RESCUE_MIN_AGREE_PERMILLE;
}
static gboolean
gx_enroll_view_connected (GPtrArray             *accepted,
                          const GxSiftFeatures  *candidate,
                          int                    stage,
                          int                   *out_best,
                          guint                 *out_weak_links)
{
  int best = 0;
  guint weak_links = 0;

  if (out_best)
    *out_best = 0;
  if (out_weak_links)
    *out_weak_links = 0;

  if (!candidate)
    return FALSE;

  if (!accepted || accepted->len == 0)
    return TRUE;

  for (guint i = 0; i < accepted->len; i++)
    {
      int score = gx_sift_match (candidate, g_ptr_array_index (accepted, i));

      best = MAX (best, score);
      if (score >= GX_ENROLL_EXTENSION_WEAK_SCORE)
        weak_links++;
    }

  if (out_best)
    *out_best = best;
  if (out_weak_links)
    *out_weak_links = weak_links;

  if (stage < GX_ENROLL_ANCHOR_STAGES)
    return best >= GX_ENROLL_ANCHOR_MIN_SCORE;

  return best >= GX_ENROLL_EXTENSION_MIN_SCORE ||
         weak_links >= GX_ENROLL_EXTENSION_WEAK_LINKS;
}

