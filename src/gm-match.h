/*
 * gm-match: fingerprint matching for small sensors (Goodix Milan, 108x88).
 * Single-scale ORB (FAST-9 + intensity-centroid orientation + steered BRIEF)
 * with RANSAC verification of a similarity transform. Plain C99, depends only on libm.
 *
 * Copyright (C) 2026 enBeg
 * Developed with the assistance of Claude (Anthropic), see README.md.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#pragma once

#include <stdint.h>

#ifndef GM_BITS
#define GM_BITS 256
#endif
#define GM_DESC_BYTES (GM_BITS / 8)

/* Bump on any change that alters the extracted keypoints or descriptors:
 * features stored with a different version are recomputed from the stored images. */
#define GM_ALGO_VERSION 2

typedef struct
{
  int      n;
  float   *x;
  float   *y;
  uint8_t *desc;     /* n * GM_DESC_BYTES */
} GmFeatures;

/* Raw difference image (background - frame, float) w x h  ->  normalized 8 bit image (w-4) x (h-4). */
void  gm_prepare (const float *diff, int w, int h, uint8_t *out);

/* Extracts features from a normalized image. Returns the number of keypoints. */
int   gm_extract (const uint8_t *img, int w, int h, GmFeatures *out);

/* Number of geometrically consistent correspondences between two feature sets. */
int   gm_match (const GmFeatures *a, const GmFeatures *b);

void  gm_features_clear (GmFeatures *f);

/* Ridge energy of a raw frame: ~31 with an empty sensor, >50 with a finger. */
float gm_ridge_energy (const float *frame, int w, int h);
