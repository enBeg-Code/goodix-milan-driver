/*
 * Goodix Milan F (GXFP3200) SPI fingerprint sensor driver for libfprint
 *
 * Found in ASUSPRO P5440FA laptops (ACPI \_SB.PCI0.SPI0.FPRT, HID GXFP3200).
 * 108x88 pixel, 12 bit, match-on-host. Protocol reverse engineered from the
 * ASUS Windows UMDF driver (gfspi.dll 1.0.20.800):
 *
 *   read(addr, n):     CS[F0 aH aL]   CS[F1 | read n]
 *   write16(addr, v):  CS[F0 aH aL 00 01 vH vL]
 *   cmd(c):            CS[c]             C0 = idle, C2 = start image capture
 *
 *   Capture: C0, clear IRQ (0x124), mode registers, C2, 0x54 = 0x0110,
 *   poll IRQ status 0x126 for bit 0x08, clear, read 14260 bytes from the
 *   0xAAAA FIFO (readable in chunks): 6 bytes -> 4 pixels, CRC-32/MPEG-2.
 *
 * The sensor is too small for NBIS minutiae matching, so matching is done by
 * gm-match.c (ORB-like keypoints + RANSAC).
 *
 * Copyright (C) 2026 enBeg
 * Developed with the assistance of Claude (Anthropic), see README.md.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "goodixmilan"

#include "drivers_api.h"
#include "gm-match.h"

#include <errno.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>

#define GM_W               108
#define GM_H               88
#define GM_PREP_W          (GM_W - 4)
#define GM_PREP_H          (GM_H - 4)
#define GM_PREP_SIZE       (GM_PREP_W * GM_PREP_H)
#define GM_RAWLEN          0x37B4
#define GM_FIFO_CHUNK      4092           /* fits spidev default bufsiz */
#define GM_CHIP_ID         0x2202

#define GM_REG_IRQ_CLEAR   0x0124
#define GM_REG_IRQ_STATUS  0x0126
#define GM_IRQ_IMAGE       0x0008

#define GM_RIDGE_PRESENT   40.0f
#define GM_RIDGE_ABSENT    35.0f
#define GM_QUALITY_MIN     300.0f
#define GM_MATCH_THRESHOLD 12
#define GM_ENROLL_STAGES   12

static const guint16 gm_mode_capture[][2] = {
  { 0x005C, 0x0080 }, { 0x0120, 0x0408 }, { 0x0052, 0x0008 }, { 0x0050, 0x0501 },
};

struct _FpiDeviceGoodixMilan
{
  FpDevice   parent;

  int        fd;
  float     *bg;                /* empty sensor frame, GM_W x GM_H */
  GPtrArray *enroll_imgs;       /* GBytes, prepared GM_PREP_W x GM_PREP_H */
};

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixMilan, fpi_device_goodixmilan, FPI, DEVICE_GOODIXMILAN, FpDevice);
G_DEFINE_TYPE (FpiDeviceGoodixMilan, fpi_device_goodixmilan, FP_TYPE_DEVICE);

/* ======================================================================== SPI */

static gboolean
gm_xfer (FpiDeviceGoodixMilan *self, const guint8 *wr, gsize wlen,
         guint8 *rd, gsize rlen, GError **error)
{
  /* All sensor I/O happens in a worker thread, so the spidev ioctl is issued
   * directly (fpi_spi_transfer_submit_sync() would spawn yet another thread).
   * Write and read share one message so that CS stays asserted in between. */
  struct spi_ioc_transfer x[2] = {
    { .tx_buf = (guintptr) wr, .len = wlen, .bits_per_word = 8 },
    { .rx_buf = (guintptr) rd, .len = rlen, .bits_per_word = 8 },
  };

  if (ioctl (self->fd, SPI_IOC_MESSAGE (rlen ? 2 : 1), x) < 0)
    {
      int e = errno;
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (e), "SPI transfer failed: %s", g_strerror (e));
      return FALSE;
    }
  return TRUE;
}

static gboolean
gm_cmd (FpiDeviceGoodixMilan *self, guint8 c, GError **error)
{
  return gm_xfer (self, &c, 1, NULL, 0, error);
}

static gboolean
gm_set_addr (FpiDeviceGoodixMilan *self, guint16 addr, GError **error)
{
  guint8 b[3] = { 0xF0, addr >> 8, addr & 0xFF };

  return gm_xfer (self, b, 3, NULL, 0, error);
}

static gboolean
gm_read (FpiDeviceGoodixMilan *self, guint16 addr, guint8 *buf, gsize n, GError **error)
{
  const guint8 f1 = 0xF1;

  return gm_set_addr (self, addr, error) && gm_xfer (self, &f1, 1, buf, n, error);
}

static gboolean
gm_read16 (FpiDeviceGoodixMilan *self, guint16 addr, guint16 *val, GError **error)
{
  guint8 b[2];

  if (!gm_read (self, addr, b, 2, error))
    return FALSE;
  *val = b[0] << 8 | b[1];
  return TRUE;
}

static gboolean
gm_write16 (FpiDeviceGoodixMilan *self, guint16 addr, guint16 val, GError **error)
{
  guint8 b[7] = { 0xF0, addr >> 8, addr & 0xFF, 0x00, 0x01, val >> 8, val & 0xFF };

  return gm_xfer (self, b, 7, NULL, 0, error);
}

static gboolean
gm_read_chip_id (FpiDeviceGoodixMilan *self, guint32 *id, GError **error)
{
  guint8 b[4];

  if (!gm_read (self, 0x0000, b, 4, error))
    return FALSE;
  *id = (guint32) b[2] << 24 | (guint32) b[3] << 16 | b[0] << 8 | b[1];
  return TRUE;
}

static guint32
gm_crc32_mpeg2 (const guint8 *d, gsize n)
{
  guint32 c = 0xFFFFFFFF;

  for (gsize i = 0; i < n; i++)
    {
      c ^= (guint32) d[i] << 24;
      for (int k = 0; k < 8; k++)
        c = c & 0x80000000 ? (c << 1) ^ 0x04C11DB7 : c << 1;
    }
  return c;
}

/* Capture one raw frame. Runs in a worker thread. Returns GM_W x GM_H floats. */
static float *
gm_capture_frame_once (FpiDeviceGoodixMilan *self, GError **error)
{
  g_autofree guint8 *raw = g_malloc (GM_RAWLEN);
  const guint8 f1 = 0xF1;
  guint16 st = 0, seed_hi, seed_lo;
  gint64 t0;
  guint32 crc;
  float *frame;

  if (!gm_cmd (self, 0xC0, error))
    return NULL;
  g_usleep (1000);
  if (!gm_read16 (self, GM_REG_IRQ_STATUS, &st, error))
    return NULL;
  if (st && !gm_write16 (self, GM_REG_IRQ_CLEAR, st, error))
    return NULL;
  if (!gm_write16 (self, GM_REG_IRQ_CLEAR, 0x00FF, error))
    return NULL;
  for (guint i = 0; i < G_N_ELEMENTS (gm_mode_capture); i++)
    if (!gm_write16 (self, gm_mode_capture[i][0], gm_mode_capture[i][1], error))
      return NULL;
  if (!gm_cmd (self, 0xC2, error))
    return NULL;
  g_usleep (1000);
  if (!gm_write16 (self, 0x0054, 0x0110, error))
    return NULL;

  t0 = g_get_monotonic_time ();
  for (;;)
    {
      if (!gm_read16 (self, GM_REG_IRQ_STATUS, &st, error))
        return NULL;
      if (st & GM_IRQ_IMAGE)
        break;
      if (g_get_monotonic_time () - t0 > G_USEC_PER_SEC)
        {
          gm_cmd (self, 0xC0, NULL);
          g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                       "Image capture timed out (status 0x%04x)", st);
          return NULL;
        }
      g_usleep (2000);
    }
  if (!gm_write16 (self, GM_REG_IRQ_CLEAR, st, error))
    return NULL;

  /* the 0xAAAA FIFO can be drained in chunks after a single address write */
  if (!gm_set_addr (self, 0xAAAA, error))
    return NULL;
  for (gsize off = 0; off < GM_RAWLEN; off += GM_FIFO_CHUNK)
    if (!gm_xfer (self, &f1, 1, raw + off, MIN (GM_FIFO_CHUNK, GM_RAWLEN - off), error))
      return NULL;

  if (!gm_read16 (self, 0x0074, &seed_hi, error) || !gm_read16 (self, 0x0072, &seed_lo, error))
    return NULL;
  gm_cmd (self, 0xC0, NULL);

  crc = gm_crc32_mpeg2 (raw, GM_RAWLEN - 4);
  if (raw[GM_RAWLEN - 4] != ((crc >> 8) & 0xFF) || raw[GM_RAWLEN - 3] != (crc & 0xFF) ||
      raw[GM_RAWLEN - 2] != (crc >> 24) || raw[GM_RAWLEN - 1] != ((crc >> 16) & 0xFF))
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO, "Frame CRC mismatch");
      return NULL;
    }
  if (seed_hi || seed_lo)
    {
      /* the Windows driver supports XOR-scrambled frames; never seen on this hardware */
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED,
                   "Scrambled image data (seed 0x%04x%04x) not supported", seed_hi, seed_lo);
      return NULL;
    }

  /* 12 bit unpacking: 6 bytes -> 4 pixels */
  frame = g_new (float, GM_W * GM_H);
  for (int i = 0, p = 0; p < GM_W * GM_H; i += 6, p += 4)
    {
      const guint8 *b = raw + i;
      frame[p + 0] = b[1] + ((b[0] & 0x0F) << 8);
      frame[p + 1] = (b[3] << 4) + (b[0] >> 4);
      frame[p + 2] = b[2] + ((b[5] & 0x0F) << 8);
      frame[p + 3] = (b[4] << 4) + (b[5] >> 4);
    }
  return frame;
}

static float *
gm_capture_frame (FpiDeviceGoodixMilan *self, GError **error)
{
  for (int attempt = 0;; attempt++)
    {
      g_autoptr(GError) local = NULL;
      float *f = gm_capture_frame_once (self, &local);

      if (f)
        return f;
      if (attempt == 2 || !g_error_matches (local, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO))
        {
          g_propagate_error (error, g_steal_pointer (&local));
          return NULL;
        }
      fp_dbg ("Retrying capture: %s", local->message);
      g_usleep (20000);
    }
}

/* ======================================================================== scanning (worker thread) */

typedef enum {
  SCAN_WAIT_LIFT,
  SCAN_WAIT_FINGER,
} ScanMode;

typedef struct
{
  ScanMode mode;
  float    quality;
  guint8  *prepared;           /* GM_PREP_SIZE, only for SCAN_WAIT_FINGER */
} ScanJob;

static void
scan_job_free (ScanJob *job)
{
  g_free (job->prepared);
  g_free (job);
}

static void
update_background (FpiDeviceGoodixMilan *self, const float *a, const float *b)
{
  if (!self->bg)
    {
      self->bg = g_new (float, GM_W * GM_H);
      for (int i = 0; i < GM_W * GM_H; i++)
        self->bg[i] = 0.5f * (a[i] + b[i]);
    }
  else
    {
      for (int i = 0; i < GM_W * GM_H; i++)
        self->bg[i] = 0.7f * self->bg[i] + 0.15f * (a[i] + b[i]);
    }
}

static float
diff_quality (const float *bg, const float *f)
{
  double s = 0;

  for (int i = 0; i < GM_W * GM_H; i++)
    s += fabsf (bg[i] - f[i]);
  return (float) (s / (GM_W * GM_H));
}

static void
scan_thread (GTask *task, gpointer source, gpointer task_data, GCancellable *cancellable)
{
  FpiDeviceGoodixMilan *self = source;
  ScanJob *job = task_data;
  g_autofree float *prev_empty = NULL;
  int hits = 0, calm = 0;
  GError *error = NULL;

  while (!g_cancellable_set_error_if_cancelled (cancellable, &error))
    {
      g_autofree float *f = gm_capture_frame (self, &error);
      float r;

      if (!f)
        break;
      r = gm_ridge_energy (f, GM_W, GM_H);

      if (r < GM_RIDGE_ABSENT)
        {
          hits = 0;
          if (prev_empty)
            update_background (self, prev_empty, f);
          g_free (prev_empty);
          prev_empty = g_steal_pointer (&f);
          if (job->mode == SCAN_WAIT_LIFT && ++calm >= 2)
            {
              g_task_return_boolean (task, TRUE);
              return;
            }
          g_usleep (50000);
          continue;
        }
      calm = 0;

      if (job->mode == SCAN_WAIT_FINGER && r > GM_RIDGE_PRESENT && self->bg && ++hits >= 2)
        {
          g_autofree float *best = NULL;
          g_autofree float *diff = g_new (float, GM_W * GM_H);
          float best_q = -1;

          g_usleep (250000);      /* let the finger settle */
          for (int k = 0; k < 2; k++)
            {
              g_autofree float *c = gm_capture_frame (self, &error);
              float q;
              if (!c)
                {
                  g_task_return_error (task, error);
                  return;
                }
              q = diff_quality (self->bg, c);
              if (q > best_q)
                {
                  best_q = q;
                  g_free (best);
                  best = g_steal_pointer (&c);
                }
            }
          for (int i = 0; i < GM_W * GM_H; i++)
            diff[i] = self->bg[i] - best[i];
          job->quality = best_q;
          job->prepared = g_malloc (GM_PREP_SIZE);
          gm_prepare (diff, GM_W, GM_H, job->prepared);
          fp_dbg ("Finger captured, quality %.0f", best_q);
          g_task_return_boolean (task, TRUE);
          return;
        }
      g_usleep (30000);
    }
  g_task_return_error (task, error);
}

static void
start_scan (FpiDeviceGoodixMilan *self, ScanMode mode, GAsyncReadyCallback cb)
{
  g_autoptr(GTask) task = g_task_new (self, fpi_device_get_cancellable (FP_DEVICE (self)), cb, NULL);
  ScanJob *job = g_new0 (ScanJob, 1);

  job->mode = mode;
  g_task_set_task_data (task, job, (GDestroyNotify) scan_job_free);
  if (mode == SCAN_WAIT_FINGER)
    fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NEEDED);
  g_task_run_in_thread (task, scan_thread);
}

/* ======================================================================== print data
 *
 * fpi-data = (u algo, q w, q h, ay images, a(aqaqay) features)
 *   images:   the prepared GM_PREP_W x GM_PREP_H images, concatenated
 *   features: keypoints (x, y) and descriptors per image, valid only if
 *             algo == GM_ALGO_VERSION; otherwise recomputed from the images.
 */
#define GM_PRINT_FORMAT "(uqq@ay@a(aqaqay))"

static GVariant *
features_to_variant (const GmFeatures *f)
{
  g_autofree guint16 *x = g_new (guint16, MAX (f->n, 1));
  g_autofree guint16 *y = g_new (guint16, MAX (f->n, 1));

  for (int i = 0; i < f->n; i++)
    {
      x[i] = (guint16) f->x[i];
      y[i] = (guint16) f->y[i];
    }
  return g_variant_new ("(@aq@aq@ay)",
                        g_variant_new_fixed_array (G_VARIANT_TYPE_UINT16, x, f->n, sizeof (guint16)),
                        g_variant_new_fixed_array (G_VARIANT_TYPE_UINT16, y, f->n, sizeof (guint16)),
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, f->desc,
                                                   (gsize) f->n * GM_DESC_BYTES, 1));
}

static gboolean
variant_to_features (GVariant *v, GmFeatures *f)
{
  g_autoptr(GVariant) vx = NULL, vy = NULL, vd = NULL;
  const guint16 *x, *y;
  const guint8 *d;
  gsize nx, ny, nd;

  g_variant_get (v, "(@aq@aq@ay)", &vx, &vy, &vd);
  x = g_variant_get_fixed_array (vx, &nx, sizeof (guint16));
  y = g_variant_get_fixed_array (vy, &ny, sizeof (guint16));
  d = g_variant_get_fixed_array (vd, &nd, 1);
  if (nx != ny || nd != nx * GM_DESC_BYTES)
    return FALSE;
  f->n = nx;
  f->x = malloc (sizeof (float) * MAX (nx, 1));
  f->y = malloc (sizeof (float) * MAX (nx, 1));
  f->desc = malloc (MAX (nd, 1));
  for (gsize i = 0; i < nx; i++)
    {
      f->x[i] = x[i];
      f->y[i] = y[i];
    }
  memcpy (f->desc, d, nd);
  return TRUE;
}

/* Builds the print data, extracting the features (slow: call from a worker thread). */
static GVariant *
build_print_data (GPtrArray *imgs)
{
  g_autoptr(GByteArray) all = g_byte_array_new ();
  GVariantBuilder feats;

  g_variant_builder_init (&feats, G_VARIANT_TYPE ("a(aqaqay)"));
  for (guint i = 0; i < imgs->len; i++)
    {
      gsize n;
      const guint8 *d = g_bytes_get_data (g_ptr_array_index (imgs, i), &n);
      GmFeatures f;

      g_byte_array_append (all, d, n);
      gm_extract (d, GM_PREP_W, GM_PREP_H, &f);
      g_variant_builder_add_value (&feats, features_to_variant (&f));
      gm_features_clear (&f);
    }
  return g_variant_new (GM_PRINT_FORMAT, (guint32) GM_ALGO_VERSION, GM_PREP_W, GM_PREP_H,
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, all->data, all->len, 1),
                        g_variant_builder_end (&feats));
}

typedef struct
{
  GVariant     *holder;
  GVariant     *images;
  GVariant     *features;       /* NULL if stale */
  const guint8 *img_data;
  gsize         n;
} PrintData;

static void
print_data_clear (PrintData *pd)
{
  g_clear_pointer (&pd->features, g_variant_unref);
  g_clear_pointer (&pd->images, g_variant_unref);
  g_clear_pointer (&pd->holder, g_variant_unref);
}

static gboolean
print_data_load (FpPrint *print, PrintData *pd)
{
  guint32 algo;
  guint16 w, h;
  gsize len;

  memset (pd, 0, sizeof (*pd));
  g_object_get (print, "fpi-data", &pd->holder, NULL);
  if (!pd->holder || !g_variant_check_format_string (pd->holder, GM_PRINT_FORMAT, FALSE))
    return FALSE;
  g_variant_get (pd->holder, GM_PRINT_FORMAT, &algo, &w, &h, &pd->images, &pd->features);
  if (w != GM_PREP_W || h != GM_PREP_H)
    return FALSE;
  pd->img_data = g_variant_get_fixed_array (pd->images, &len, 1);
  pd->n = len / GM_PREP_SIZE;
  if (algo != GM_ALGO_VERSION || g_variant_n_children (pd->features) != pd->n)
    g_clear_pointer (&pd->features, g_variant_unref);
  return TRUE;
}

static void
print_data_features (PrintData *pd, gsize i, GmFeatures *f)
{
  if (pd->features)
    {
      g_autoptr(GVariant) v = g_variant_get_child_value (pd->features, i);
      if (variant_to_features (v, f))
        return;
    }
  gm_extract (pd->img_data + i * GM_PREP_SIZE, GM_PREP_W, GM_PREP_H, f);
}

/* ======================================================================== matching (worker thread) */

typedef struct
{
  guint8    *probe;            /* GM_PREP_SIZE */
  GPtrArray *prints;           /* FpPrint, candidates */
  gint       best_index;
  gint       best_score;
} MatchJob;

static void
match_job_free (MatchJob *job)
{
  g_free (job->probe);
  g_ptr_array_unref (job->prints);
  g_free (job);
}

static void
match_thread (GTask *task, gpointer source, gpointer task_data, GCancellable *cancellable)
{
  MatchJob *job = task_data;
  GmFeatures probe;
  gint64 t0 = g_get_monotonic_time ();

  gm_extract (job->probe, GM_PREP_W, GM_PREP_H, &probe);
  job->best_index = -1;
  job->best_score = 0;

  for (guint p = 0; p < job->prints->len && !g_cancellable_is_cancelled (cancellable); p++)
    {
      PrintData pd;

      if (!print_data_load (g_ptr_array_index (job->prints, p), &pd))
        {
          fp_warn ("Ignoring print %u with unknown data format", p);
          print_data_clear (&pd);
          continue;
        }
      for (gsize i = 0; i < pd.n; i++)
        {
          GmFeatures t;
          int s;

          print_data_features (&pd, i, &t);
          s = gm_match (&probe, &t);
          gm_features_clear (&t);
          if (s > job->best_score)
            {
              job->best_score = s;
              job->best_index = p;
            }
        }
      print_data_clear (&pd);
    }
  gm_features_clear (&probe);
  fp_dbg ("Best match score %d (threshold %d), %" G_GINT64_FORMAT " ms",
          job->best_score, GM_MATCH_THRESHOLD, (g_get_monotonic_time () - t0) / 1000);
  g_task_return_boolean (task, TRUE);
}

/* ======================================================================== enroll */

static void enroll_lift_done (GObject *src, GAsyncResult *res, gpointer user_data);

static void
enroll_finalize_thread (GTask *task, gpointer source, gpointer task_data, GCancellable *cancellable)
{
  g_task_return_pointer (task, g_variant_ref_sink (build_print_data (task_data)),
                         (GDestroyNotify) g_variant_unref);
}

static void
enroll_finalize_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpDevice *dev = FP_DEVICE (src);
  g_autoptr(GVariant) data = g_task_propagate_pointer (G_TASK (res), NULL);
  FpPrint *print = NULL;

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, FALSE);
  g_object_set (print, "fpi-data", data, NULL);
  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
enroll_finger_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (src);
  FpDevice *dev = FP_DEVICE (self);
  ScanJob *job = g_task_get_task_data (G_TASK (res));
  GError *error = NULL;

  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }
  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_PRESENT);

  if (job->quality < GM_QUALITY_MIN)
    {
      fpi_device_enroll_progress (dev, self->enroll_imgs->len, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
    }
  else
    {
      g_ptr_array_add (self->enroll_imgs, g_bytes_new (job->prepared, GM_PREP_SIZE));
      fpi_device_enroll_progress (dev, self->enroll_imgs->len, NULL, NULL);
    }

  if (self->enroll_imgs->len >= GM_ENROLL_STAGES)
    {
      g_autoptr(GTask) task = g_task_new (self, NULL, enroll_finalize_done, NULL);

      g_task_set_task_data (task, g_steal_pointer (&self->enroll_imgs), (GDestroyNotify) g_ptr_array_unref);
      g_task_run_in_thread (task, enroll_finalize_thread);
      return;
    }
  start_scan (self, SCAN_WAIT_LIFT, enroll_lift_done);
}

static void
enroll_lift_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (src);
  GError *error = NULL;

  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      g_clear_pointer (&self->enroll_imgs, g_ptr_array_unref);
      fpi_device_enroll_complete (FP_DEVICE (self), NULL, error);
      return;
    }
  fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NONE);
  start_scan (self, SCAN_WAIT_FINGER, enroll_finger_done);
}

static void
gm_enroll (FpDevice *dev)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (dev);

  g_clear_pointer (&self->enroll_imgs, g_ptr_array_unref);
  self->enroll_imgs = g_ptr_array_new_with_free_func ((GDestroyNotify) g_bytes_unref);
  start_scan (self, SCAN_WAIT_LIFT, enroll_lift_done);
}

/* ======================================================================== verify / identify */

static void
complete_match_action (FpDevice *dev, GError *error)
{
  if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
    fpi_device_verify_complete (dev, error);
  else
    fpi_device_identify_complete (dev, error);
}

static void
match_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpDevice *dev = FP_DEVICE (src);
  MatchJob *job = g_task_get_task_data (G_TASK (res));
  GError *error = NULL;
  gboolean ok;

  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      complete_match_action (dev, error);
      return;
    }
  ok = job->best_score >= GM_MATCH_THRESHOLD;

  if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
    fpi_device_verify_report (dev, ok ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL, NULL, NULL);
  else
    fpi_device_identify_report (dev, ok ? g_ptr_array_index (job->prints, job->best_index) : NULL,
                                NULL, NULL);
  complete_match_action (dev, NULL);
}

static void
match_finger_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (src);
  FpDevice *dev = FP_DEVICE (self);
  ScanJob *scan = g_task_get_task_data (G_TASK (res));
  g_autoptr(GTask) task = NULL;
  GError *error = NULL;
  MatchJob *job;

  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      complete_match_action (dev, error);
      return;
    }
  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_PRESENT);

  if (scan->quality < GM_QUALITY_MIN)
    {
      error = fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER);
      if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
        fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, error);
      else
        fpi_device_identify_report (dev, NULL, NULL, error);
      complete_match_action (dev, NULL);
      return;
    }

  job = g_new0 (MatchJob, 1);
  job->probe = g_steal_pointer (&scan->prepared);
  if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *print = NULL;
      fpi_device_get_verify_data (dev, &print);
      job->prints = g_ptr_array_new_with_free_func (g_object_unref);
      g_ptr_array_add (job->prints, g_object_ref (print));
    }
  else
    {
      GPtrArray *gallery = NULL;
      fpi_device_get_identify_data (dev, &gallery);
      job->prints = g_ptr_array_ref (gallery);
    }

  task = g_task_new (self, fpi_device_get_cancellable (dev), match_done, NULL);
  g_task_set_task_data (task, job, (GDestroyNotify) match_job_free);
  g_task_run_in_thread (task, match_thread);
}

static void
match_lift_done (GObject *src, GAsyncResult *res, gpointer user_data)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (src);
  GError *error = NULL;

  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      complete_match_action (FP_DEVICE (self), error);
      return;
    }
  fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NONE);
  start_scan (self, SCAN_WAIT_FINGER, match_finger_done);
}

static void
gm_verify_identify (FpDevice *dev)
{
  /* a finger still resting from a previous attempt must be lifted first */
  start_scan (FPI_DEVICE_GOODIXMILAN (dev), SCAN_WAIT_LIFT, match_lift_done);
}

/* ======================================================================== open / close */

static void
gm_open (FpDevice *dev)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (dev);
  const char *path = fpi_device_get_udev_data (dev, FPI_DEVICE_UDEV_SUBTYPE_SPIDEV);
  GError *error = NULL;
  guint32 id = 0;

  self->fd = open (path, O_RDWR | O_CLOEXEC);
  if (self->fd < 0)
    {
      fpi_device_open_complete (dev, g_error_new (G_IO_ERROR, g_io_error_from_errno (errno),
                                                  "Cannot open %s: %s", path, g_strerror (errno)));
      return;
    }
  if (!gm_read_chip_id (self, &id, &error))
    goto fail;
  fp_dbg ("Chip ID 0x%08x", id);
  if (id >> 8 != GM_CHIP_ID)
    {
      error = fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED, "Unknown chip ID 0x%08x", id);
      goto fail;
    }
  if (!gm_cmd (self, 0xC0, &error))
    goto fail;
  fpi_device_open_complete (dev, NULL);
  return;

fail:
  close (self->fd);
  self->fd = -1;
  fpi_device_open_complete (dev, error);
}

static void
gm_close (FpDevice *dev)
{
  FpiDeviceGoodixMilan *self = FPI_DEVICE_GOODIXMILAN (dev);

  if (self->fd >= 0)
    {
      gm_cmd (self, 0xC0, NULL);
      close (self->fd);
      self->fd = -1;
    }
  g_clear_pointer (&self->bg, g_free);
  g_clear_pointer (&self->enroll_imgs, g_ptr_array_unref);
  fpi_device_close_complete (dev, NULL);
}

/* ======================================================================== class */

static const FpIdEntry id_table[] = {
  { .udev_types = FPI_DEVICE_UDEV_SUBTYPE_SPIDEV, .spi_acpi_id = "GXFP3200" },
  { .udev_types = 0 }
};

static void
fpi_device_goodixmilan_init (FpiDeviceGoodixMilan *self)
{
  self->fd = -1;
}

static void
fpi_device_goodixmilan_class_init (FpiDeviceGoodixMilanClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = "goodixmilan";
  dev_class->full_name = "Goodix Milan F SPI Fingerprint Sensor (GXFP3200)";
  dev_class->type = FP_DEVICE_TYPE_UDEV;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = GM_ENROLL_STAGES;
  dev_class->temp_hot_seconds = -1;

  dev_class->open = gm_open;
  dev_class->close = gm_close;
  dev_class->enroll = gm_enroll;
  dev_class->verify = gm_verify_identify;
  dev_class->identify = gm_verify_identify;

  fpi_device_class_auto_initialize_features (dev_class);
}

#ifdef GOODIXMILAN_TOD
/* entry point looked up by Ubuntu's libfprint TOD loader */
G_MODULE_EXPORT GType
fpi_tod_shared_driver_get_type (void)
{
  return fpi_device_goodixmilan_get_type ();
}
#endif
