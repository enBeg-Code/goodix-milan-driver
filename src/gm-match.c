/*
 * gm-match: fingerprint matching for small sensors (Goodix Milan, 108x88).
 * See gm-match.h.
 *
 * Copyright (C) 2026 enBeg
 * Developed with the assistance of Claude (Anthropic), see README.md.
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "gm-match.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef SCALE
#define SCALE 3       /* upscaling factor before keypoint extraction */
#endif
#ifndef FAST_T
#define FAST_T 5       /* FAST threshold */
#endif
#ifndef MAX_KP
#define MAX_KP 1000
#endif
#ifndef PATCH_R
#define PATCH_R        24      /* patch radius (tuned on real captures; ORB uses 15) */
#endif
#ifndef BRIEF_SIGMA
#define BRIEF_SIGMA    2.0f
#endif
#ifndef PATTERN_SIGMA
#define PATTERN_SIGMA  (PATCH_R * 31.f / 15.f / 5.f)
#endif
#define BORDER         (PATCH_R + 4)
#define HARRIS_BLOCK   7
#ifndef RATIO
#define RATIO 0.85f
#endif
#ifndef RANSAC_ITERS
#define RANSAC_ITERS 2000
#endif
#ifndef RANSAC_THR
#define RANSAC_THR 6.0f
#endif
#define SCALE_MIN      0.85f
#define SCALE_MAX      1.18f

/* ------------------------------------------------------------------ helpers */

static inline int
reflect101 (int i, int n)
{
  if (n == 1)
    return 0;
  while (i < 0 || i >= n)
    i = i < 0 ? -i : 2 * n - 2 - i;
  return i;
}

/* separable gaussian blur on floats, REFLECT_101 border */
static void
gauss_blur_f (const float *src, float *dst, int w, int h, float sigma, int radius)
{
  float *k = malloc (sizeof (float) * (2 * radius + 1));
  float *tmp = malloc (sizeof (float) * w * h);
  float sum = 0;

  for (int i = -radius; i <= radius; i++)
    sum += k[i + radius] = expf (-(float) (i * i) / (2 * sigma * sigma));
  for (int i = 0; i <= 2 * radius; i++)
    k[i] /= sum;

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int i = -radius; i <= radius; i++)
          s += k[i + radius] * src[y * w + reflect101 (x + i, w)];
        tmp[y * w + x] = s;
      }
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int i = -radius; i <= radius; i++)
          s += k[i + radius] * tmp[reflect101 (y + i, h) * w + x];
        dst[y * w + x] = s;
      }
  free (tmp);
  free (k);
}

static void
gauss_blur_u8 (const uint8_t *src, uint8_t *dst, int w, int h, float sigma, int radius)
{
  float *f = calloc ((size_t) w * h, sizeof (float));

  for (int i = 0; i < w * h; i++)
    f[i] = src[i];
  gauss_blur_f (f, f, w, h, sigma, radius);
  for (int i = 0; i < w * h; i++)
    dst[i] = (uint8_t) fminf (255.f, fmaxf (0.f, roundf (f[i])));
  free (f);
}

static int
cmp_float (const void *a, const void *b)
{
  float x = *(const float *) a, y = *(const float *) b;

  return (x > y) - (x < y);
}

static float
percentile_sorted (const float *s, int n, float p)
{
  float pos = p / 100.f * (n - 1);
  int i = (int) pos;
  float f = pos - i;

  return i + 1 < n ? s[i] * (1 - f) + s[i + 1] * f : s[n - 1];
}

/* ------------------------------------------------------------------ preparation */

void
gm_prepare (const float *diff, int w, int h, uint8_t *out)
{
  int cw = w - 4, ch = h - 4, n = cw * ch;
  float *a = calloc (n, sizeof (float));
  float *b = malloc (sizeof (float) * n);
  float lo, hi;

  for (int y = 0; y < ch; y++)
    for (int x = 0; x < cw; x++)
      a[y * cw + x] = diff[(y + 2) * w + x + 2];

  /* high-pass: removes slow variations caused by finger pressure */
  gauss_blur_f (a, b, cw, ch, 6.0f, 24);
  for (int i = 0; i < n; i++)
    a[i] -= b[i];

  memcpy (b, a, sizeof (float) * n);
  qsort (b, n, sizeof (float), cmp_float);
  lo = percentile_sorted (b, n, 1);
  hi = percentile_sorted (b, n, 99);
  if (hi - lo < 1e-6f)
    hi = lo + 1e-6f;
  for (int i = 0; i < n; i++)
    {
      float v = (a[i] - lo) / (hi - lo) * 255.f;
      out[i] = (uint8_t) fminf (255.f, fmaxf (0.f, v));
    }
  free (a);
  free (b);
}

/* ------------------------------------------------------------------ BRIEF pattern */

static int8_t pattern[GM_BITS][4];
static int pattern_ready;

static void
init_pattern (void)
{
  /* point pairs drawn from an isotropic gaussian, inside the patch circle;
   * deterministic generator, so stored templates and probes always use the same pattern */
  uint64_t s = 0x9E3779B97F4A7C15ull;
  int i = 0, c = 0;
  float pt[2];

  while (i < GM_BITS)
    {
      float u1, u2, r, g0, g1;
      s ^= s << 13; s ^= s >> 7; s ^= s << 17;
      u1 = ((s >> 11) + 1) * (1.0f / 9007199254740993.0f);
      s ^= s << 13; s ^= s >> 7; s ^= s << 17;
      u2 = (s >> 11) * (1.0f / 9007199254740992.0f);
      r = sqrtf (-2.f * logf (u1)) * PATTERN_SIGMA;
      g0 = r * cosf (2 * (float) M_PI * u2);
      g1 = r * sinf (2 * (float) M_PI * u2);
      if (g0 * g0 + g1 * g1 > PATCH_R * PATCH_R)
        continue;
      pt[0] = g0;
      pt[1] = g1;
      pattern[i][c * 2] = (int8_t) lroundf (pt[0]);
      pattern[i][c * 2 + 1] = (int8_t) lroundf (pt[1]);
      if (++c == 2)
        {
          if (pattern[i][0] != pattern[i][2] || pattern[i][1] != pattern[i][3])
            i++;
          c = 0;
        }
    }
  pattern_ready = 1;
}

/* ------------------------------------------------------------------ extraction */

typedef struct
{
  int   x, y;
  float fast;
  float harris;
} Kp;

static const int circle[16][2] = {
  {0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
  {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3},
};

static float
fast_score (const uint8_t *img, int w, int x, int y)
{
  int p = img[y * w + x];
  int d[16], brighter = 0, darker = 0;
  float sb = 0, sd = 0;

  for (int i = 0; i < 16; i++)
    d[i] = img[(y + circle[i][1]) * w + x + circle[i][0]] - p;

  /* look for 9 contiguous pixels all brighter or all darker than the threshold */
  for (int i = 0; i < 16 + 9; i++)
    {
      int v = d[i % 16];
      brighter = v > FAST_T ? brighter + 1 : 0;
      darker = v < -FAST_T ? darker + 1 : 0;
      if (brighter >= 9 || darker >= 9)
        break;
    }
  if (brighter < 9 && darker < 9)
    return 0;
  for (int i = 0; i < 16; i++)
    {
      if (d[i] > FAST_T)
        sb += d[i] - FAST_T;
      else if (d[i] < -FAST_T)
        sd += -d[i] - FAST_T;
    }
  return sb > sd ? sb : sd;
}

static float
harris (const uint8_t *img, int w, int x, int y)
{
  float a = 0, b = 0, c = 0;
  int r = HARRIS_BLOCK / 2;

  for (int v = -r; v <= r; v++)
    for (int u = -r; u <= r; u++)
      {
        const uint8_t *p = img + (y + v) * w + x + u;
        float dx = (p[-w + 1] + 2 * p[1] + p[w + 1]) - (p[-w - 1] + 2 * p[-1] + p[w - 1]);
        float dy = (p[w - 1] + 2 * p[w] + p[w + 1]) - (p[-w - 1] + 2 * p[-w] + p[-w + 1]);
        a += dx * dx;
        b += dy * dy;
        c += dx * dy;
      }
  return a * b - c * c - 0.04f * (a + b) * (a + b);
}

static int
cmp_kp_fast (const void *pa, const void *pb)
{
  const Kp *a = pa, *b = pb;

  return (a->fast < b->fast) - (a->fast > b->fast);
}

static int
cmp_kp_harris (const void *pa, const void *pb)
{
  const Kp *a = pa, *b = pb;

  return (a->harris < b->harris) - (a->harris > b->harris);
}

int
gm_extract (const uint8_t *img8, int w0, int h0, GmFeatures *out)
{
  int W = w0 * SCALE, H = h0 * SCALE, nkp = 0, cap = 4096;
  uint8_t *up = malloc (W * H), *bl = malloc (W * H), *sm = malloc (W * H);
  float *score = calloc (W * H, sizeof (float));
  Kp *kp = malloc (sizeof (Kp) * cap);
  int umax[PATCH_R + 1];

  if (!pattern_ready)
    init_pattern ();
  memset (out, 0, sizeof (*out));

  /* bilinear upscaling (pixel centres aligned like OpenCV INTER_LINEAR) */
  for (int y = 0; y < H; y++)
    {
      float sy = (y + 0.5f) / SCALE - 0.5f;
      int y0 = (int) floorf (sy);
      float fy = sy - y0;
      int ya = y0 < 0 ? 0 : (y0 >= h0 ? h0 - 1 : y0);
      int yb = y0 + 1 >= h0 ? h0 - 1 : (y0 + 1 < 0 ? 0 : y0 + 1);
      if (y0 < 0)
        fy = 0;
      for (int x = 0; x < W; x++)
        {
          float sx = (x + 0.5f) / SCALE - 0.5f;
          int x0 = (int) floorf (sx);
          float fx = sx - x0;
          int xa = x0 < 0 ? 0 : (x0 >= w0 ? w0 - 1 : x0);
          int xb = x0 + 1 >= w0 ? w0 - 1 : (x0 + 1 < 0 ? 0 : x0 + 1);
          float v;
          if (x0 < 0)
            fx = 0;
          v = (1 - fy) * ((1 - fx) * img8[ya * w0 + xa] + fx * img8[ya * w0 + xb]) +
              fy * ((1 - fx) * img8[yb * w0 + xa] + fx * img8[yb * w0 + xb]);
          up[y * W + x] = (uint8_t) lroundf (v);
        }
    }

  /* 3x3 [1 2 1]/4 blur, then FAST detection with non-maximum suppression */
  {
    float *f = malloc (sizeof (float) * W * H);
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++)
        {
          int xm = reflect101 (x - 1, W), xp = reflect101 (x + 1, W);
          f[y * W + x] = 0.25f * up[y * W + xm] + 0.5f * up[y * W + x] + 0.25f * up[y * W + xp];
        }
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++)
        {
          int ym = reflect101 (y - 1, H), yp = reflect101 (y + 1, H);
          float v = 0.25f * f[ym * W + x] + 0.5f * f[y * W + x] + 0.25f * f[yp * W + x];
          bl[y * W + x] = (uint8_t) lroundf (v);
        }
    free (f);
  }

  for (int y = 3; y < H - 3; y++)
    for (int x = 3; x < W - 3; x++)
      score[y * W + x] = fast_score (bl, W, x, y);

  for (int y = BORDER; y < H - BORDER; y++)
    for (int x = BORDER; x < W - BORDER; x++)
      {
        float s = score[y * W + x];
        int is_max = s > 0;
        for (int v = -1; v <= 1 && is_max; v++)
          for (int u = -1; u <= 1; u++)
            if ((u || v) && (score[(y + v) * W + x + u] > s ||
                             (score[(y + v) * W + x + u] == s && (v < 0 || (v == 0 && u < 0)))))
              {
                is_max = 0;
                break;
              }
        if (!is_max)
          continue;
        if (nkp == cap)
          kp = realloc (kp, sizeof (Kp) * (cap *= 2));
        kp[nkp++] = (Kp) { x, y, s, 0 };
      }

  /* like ORB: keep the best 2N by FAST score, then the best N by Harris response */
  qsort (kp, nkp, sizeof (Kp), cmp_kp_fast);
  if (nkp > 2 * MAX_KP)
    nkp = 2 * MAX_KP;
  for (int i = 0; i < nkp; i++)
    kp[i].harris = harris (bl, W, kp[i].x, kp[i].y);
  qsort (kp, nkp, sizeof (Kp), cmp_kp_harris);
  if (nkp > MAX_KP)
    nkp = MAX_KP;

  /* smoothed image for the BRIEF tests */
  gauss_blur_u8 (bl, sm, W, H, BRIEF_SIGMA, (int) ceilf (BRIEF_SIGMA * 1.5f));

  for (int v = 0; v <= PATCH_R; v++)
    umax[v] = (int) floorf (sqrtf ((float) (PATCH_R * PATCH_R - v * v)) + 0.5f);

  out->n = nkp;
  out->x = malloc (sizeof (float) * (nkp ? nkp : 1));
  out->y = malloc (sizeof (float) * (nkp ? nkp : 1));
  out->desc = calloc (nkp ? nkp : 1, GM_DESC_BYTES);

  for (int i = 0; i < nkp; i++)
    {
      int cx = kp[i].x, cy = kp[i].y;
      long m01 = 0, m10 = 0;
      float ang, ca, sa;
      uint8_t *d = out->desc + i * GM_DESC_BYTES;

      /* orientation: intensity centroid in the circular patch */
      for (int v = -PATCH_R; v <= PATCH_R; v++)
        {
          int r = umax[abs (v)];
          for (int u = -r; u <= r; u++)
            {
              int val = bl[(cy + v) * W + cx + u];
              m10 += (long) u * val;
              m01 += (long) v * val;
            }
        }
      ang = atan2f ((float) m01, (float) m10);
      ca = cosf (ang);
      sa = sinf (ang);

      for (int b = 0; b < GM_BITS; b++)
        {
          const int8_t *p = pattern[b];
          int x1 = cx + (int) lroundf (p[0] * ca - p[1] * sa);
          int y1 = cy + (int) lroundf (p[0] * sa + p[1] * ca);
          int x2 = cx + (int) lroundf (p[2] * ca - p[3] * sa);
          int y2 = cy + (int) lroundf (p[2] * sa + p[3] * ca);
          if (sm[y1 * W + x1] < sm[y2 * W + x2])
            d[b >> 3] |= 1 << (b & 7);
        }
      out->x[i] = (float) cx;
      out->y[i] = (float) cy;
    }

  free (up);
  free (bl);
  free (sm);
  free (score);
  free (kp);
  return nkp;
}

/* ------------------------------------------------------------------ matching */

static inline int
hamming (const uint8_t *a, const uint8_t *b)
{
  int d = 0;

  for (int i = 0; i < GM_DESC_BYTES; i += 8)
    {
      uint64_t x, y;
      memcpy (&x, a + i, 8);
      memcpy (&y, b + i, 8);
      d += __builtin_popcountll (x ^ y);
    }
  return d;
}

int
gm_match (const GmFeatures *a, const GmFeatures *b)
{
  int *qa, *qb, ng = 0, best = 0;
  float br = 1, bi = 0, btx = 0, bty = 0;
  uint64_t s = 0x2545F4914F6CDD1Dull;

  if (a->n < 3 || b->n < 3)
    return 0;
  qa = malloc (sizeof (int) * a->n);
  qb = malloc (sizeof (int) * a->n);

  /* correspondences: Lowe ratio test + mutual check, computing the distances only once.
   * a[i] is kept if b[j1] is its nearest neighbour, clearly better than the second one,
   * and no other point of a is closer to b[j1] than a[i]. */
  {
    int *d1 = malloc (sizeof (int) * a->n), *d2 = malloc (sizeof (int) * a->n);
    int *j1 = malloc (sizeof (int) * a->n), *colmin = malloc (sizeof (int) * b->n);

    for (int j = 0; j < b->n; j++)
      colmin[j] = 1 << 30;
    for (int i = 0; i < a->n; i++)
      {
        const uint8_t *da = a->desc + i * GM_DESC_BYTES;
        d1[i] = d2[i] = 1 << 30;
        j1[i] = -1;
        for (int j = 0; j < b->n; j++)
          {
            int d = hamming (da, b->desc + j * GM_DESC_BYTES);
            if (d < d1[i])
              {
                d2[i] = d1[i];
                d1[i] = d;
                j1[i] = j;
              }
            else if (d < d2[i])
              {
                d2[i] = d;
              }
            if (d < colmin[j])
              colmin[j] = d;
          }
      }
    for (int i = 0; i < a->n; i++)
      {
        if (j1[i] < 0 || !(d1[i] < RATIO * d2[i]))
          continue;
#ifndef NO_CROSS_CHECK
        if (colmin[j1[i]] < d1[i])
          continue;
#endif
        qa[ng] = i;
        qb[ng] = j1[i];
        ng++;
      }
    free (d1);
    free (d2);
    free (j1);
    free (colmin);
  }

  /* RANSAC of a similarity transform (rotation + scale + translation) */
  if (ng >= 4)
    for (int it = 0; it < RANSAC_ITERS; it++)
      {
        int i1, i2, inl = 0;
        float ax, ay, bx, by, dax, day, dbx, dby, den, re, im, sc, tx, ty;

        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        i1 = (int) (s % ng);
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        i2 = (int) (s % ng);
        if (i1 == i2)
          continue;
        ax = a->x[qa[i1]]; ay = a->y[qa[i1]];
        bx = b->x[qb[i1]]; by = b->y[qb[i1]];
        dax = a->x[qa[i2]] - ax; day = a->y[qa[i2]] - ay;
        dbx = b->x[qb[i2]] - bx; dby = b->y[qb[i2]] - by;
        den = dax * dax + day * day;
        if (den < 1.0f)
          continue;
        /* q = db / da as complex numbers */
        re = (dbx * dax + dby * day) / den;
        im = (dby * dax - dbx * day) / den;
        sc = sqrtf (re * re + im * im);
        if (sc < SCALE_MIN || sc > SCALE_MAX)
          continue;
        tx = bx - (re * ax - im * ay);
        ty = by - (im * ax + re * ay);
        for (int k = 0; k < ng; k++)
          {
            float px = a->x[qa[k]], py = a->y[qa[k]];
            float ex = re * px - im * py + tx - b->x[qb[k]];
            float ey = im * px + re * py + ty - b->y[qb[k]];
            if (ex * ex + ey * ey < RANSAC_THR * RANSAC_THR)
              inl++;
          }
        if (inl > best)
          {
            best = inl;
            br = re; bi = im; btx = tx; bty = ty;
          }
      }

#ifndef NO_REFINE
  /* refinement: least-squares similarity on the inliers, then recount (2 passes) */
  for (int pass = 0; pass < 2 && best >= 3; pass++)
    {
      double sax = 0, say = 0, sbx = 0, sby = 0, sxx = 0, sre = 0, sim = 0;
      int m = 0, inl = 0;
      float re, im, tx, ty, sc;
      for (int k = 0; k < ng; k++)
        {
          float px = a->x[qa[k]], py = a->y[qa[k]];
          float ex = br * px - bi * py + btx - b->x[qb[k]];
          float ey = bi * px + br * py + bty - b->y[qb[k]];
          if (ex * ex + ey * ey < RANSAC_THR * RANSAC_THR)
            {
              sax += px; say += py; sbx += b->x[qb[k]]; sby += b->y[qb[k]]; m++;
            }
        }
      sax /= m; say /= m; sbx /= m; sby /= m;
      for (int k = 0; k < ng; k++)
        {
          float px = a->x[qa[k]], py = a->y[qa[k]];
          float ex = br * px - bi * py + btx - b->x[qb[k]];
          float ey = bi * px + br * py + bty - b->y[qb[k]];
          if (ex * ex + ey * ey < RANSAC_THR * RANSAC_THR)
            {
              double ux = px - sax, uy = py - say, vx = b->x[qb[k]] - sbx, vy = b->y[qb[k]] - sby;
              sxx += ux * ux + uy * uy;
              sre += ux * vx + uy * vy;
              sim += ux * vy - uy * vx;
            }
        }
      if (sxx < 1e-6)
        break;
      re = (float) (sre / sxx); im = (float) (sim / sxx);
      sc = sqrtf (re * re + im * im);
      if (sc < SCALE_MIN || sc > SCALE_MAX)
        break;
      tx = (float) (sbx - (re * sax - im * say));
      ty = (float) (sby - (im * sax + re * say));
      for (int k = 0; k < ng; k++)
        {
          float px = a->x[qa[k]], py = a->y[qa[k]];
          float ex = re * px - im * py + tx - b->x[qb[k]];
          float ey = im * px + re * py + ty - b->y[qb[k]];
          if (ex * ex + ey * ey < RANSAC_THR * RANSAC_THR)
            inl++;
        }
      if (inl < best)
        break;
      best = inl; br = re; bi = im; btx = tx; bty = ty;
    }
#endif

  free (qa);
  free (qb);
  return best;
}

void
gm_features_clear (GmFeatures *f)
{
  free (f->x);
  free (f->y);
  free (f->desc);
  memset (f, 0, sizeof (*f));
}

/* ------------------------------------------------------------------ finger detection */

float
gm_ridge_energy (const float *frame, int w, int h)
{
  float *a = calloc ((size_t) w * h, sizeof (float));
  float *b = malloc (sizeof (float) * w * h);
  double s = 0, s2 = 0;
  int n = 0;

  /* remove each row's mean (the sensor's horizontal banding), then band-pass */
  for (int y = 0; y < h; y++)
    {
      double m = 0;
      for (int x = 0; x < w; x++)
        m += frame[y * w + x];
      m /= w;
      for (int x = 0; x < w; x++)
        a[y * w + x] = frame[y * w + x] - (float) m;
    }
  gauss_blur_f (a, b, w, h, 2.0f, 8);
  for (int i = 0; i < w * h; i++)
    a[i] -= b[i];
  gauss_blur_f (a, b, w, h, 0.8f, 3);
  for (int y = 3; y < h - 3; y++)
    for (int x = 3; x < w - 3; x++)
      {
        s += b[y * w + x];
        s2 += (double) b[y * w + x] * b[y * w + x];
        n++;
      }
  free (a);
  free (b);
  return (float) sqrt (s2 / n - (s / n) * (s / n));
}
