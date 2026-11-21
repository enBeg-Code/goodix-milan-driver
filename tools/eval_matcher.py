#!/usr/bin/env python3
"""Evaluates src/gm-match.c on a dataset collected with collect.py.

usage: make libgmmatch.so && tools/eval_matcher.py [DATASET_DIR] [--threshold 12]

Every sub-directory of DATASET_DIR is one finger. For each finger, random templates of
N images (like an enrollment) are matched against the remaining images of the same finger
(genuine attempts) and against all images of the other fingers (impostor attempts).
"""
import argparse
import ctypes
import glob
import os
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class GmFeatures(ctypes.Structure):
    _fields_ = [('n', ctypes.c_int), ('x', ctypes.POINTER(ctypes.c_float)),
                ('y', ctypes.POINTER(ctypes.c_float)), ('desc', ctypes.POINTER(ctypes.c_uint8))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset', nargs='?', default='dataset')
    ap.add_argument('--threshold', type=int, default=12)
    ap.add_argument('--lib', default=os.path.join(ROOT, 'libgmmatch.so'))
    args = ap.parse_args()

    lib = ctypes.CDLL(args.lib)
    lib.gm_prepare.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
    lib.gm_extract.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(GmFeatures)]
    lib.gm_match.argtypes = [ctypes.POINTER(GmFeatures), ctypes.POINTER(GmFeatures)]

    labels = sorted(d for d in os.listdir(args.dataset) if os.path.isdir(os.path.join(args.dataset, d)))
    files = [(l, p) for l in labels for p in sorted(glob.glob(os.path.join(args.dataset, l, '*.npy')))]
    if len(labels) < 2:
        raise SystemExit('need at least two fingers (sub-directories) in the dataset')
    labs = np.array([l for l, _ in files])

    feats = []
    t0 = time.time()
    for _, p in files:
        diff = np.ascontiguousarray(np.load(p), dtype=np.float32)
        h, w = diff.shape
        img = np.zeros((h - 4, w - 4), np.uint8)
        lib.gm_prepare(diff.ctypes.data, w, h, img.ctypes.data)
        f = GmFeatures()
        lib.gm_extract(img.ctypes.data, w - 4, h - 4, ctypes.byref(f))
        feats.append(f)
    t_ext = (time.time() - t0) / len(files)

    n = len(feats)
    S = np.zeros((n, n), int)
    t0 = time.time()
    for i in range(n):
        for j in range(i + 1, n):
            S[i, j] = S[j, i] = lib.gm_match(ctypes.byref(feats[i]), ctypes.byref(feats[j]))
    t_match = (time.time() - t0) / max(1, n * (n - 1) // 2)

    print(f'{n} images, {len(labels)} fingers, {np.mean([f.n for f in feats]):.0f} keypoints/image, '
          f'extraction {t_ext * 1000:.0f} ms, match {t_match * 1000:.1f} ms')
    rng = np.random.default_rng(1)
    idx = {l: np.where(labs == l)[0] for l in labels}
    for N in (8, 12):
        gen, imp = [], []
        for l in labels:
            if len(idx[l]) < N + 2:
                continue
            for _ in range(20):
                perm = rng.permutation(idx[l])
                tmpl, probes = perm[:N], perm[N:]
                gen += [S[p, tmpl].max() for p in probes]
                imp += [S[p, tmpl].max() for o in labels if o != l for p in idx[o]]
        if not gen:
            continue
        gen, imp = np.array(gen), np.array(imp)
        t = args.threshold
        print(f'template of {N:2d} images: genuine median {np.median(gen):.0f}, impostor max {imp.max()} | '
              f'threshold {t}: false rejects {np.mean(gen < t):.1%}, false accepts {np.mean(imp >= t):.2%}')


if __name__ == '__main__':
    main()
