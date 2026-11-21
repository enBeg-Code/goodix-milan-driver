#!/usr/bin/env python3
"""Guided collection of fingerprint images, to evaluate or tune the matcher.

usage: collect.py <label> <count> [--out DIR]
       e.g. collect.py right-index 20 ; collect.py right-middle 10

Images are saved as DIR/<label>/NN.npy (float32 background - frame) and NN.png.
They are biometric data: keep them private, do not commit or upload them.
"""
import argparse
import os
import sys
import time

import numpy as np
from PIL import Image

from gmsensor import QUALITY_MIN, W, H, Sensor


def banner(msg):
    print('\n' + '#' * 50 + f'\n   {msg}\n' + '#' * 50 + '\a', flush=True)


def save_png(img, path):
    lo, hi = np.percentile(img, 1), np.percentile(img, 99)
    g = np.clip((img - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
    Image.fromarray(g).resize((W * 3, H * 3), Image.BICUBIC).save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label')
    ap.add_argument('count', type=int)
    ap.add_argument('--out', default='dataset')
    args = ap.parse_args()
    d = os.path.join(args.out, args.label)
    os.makedirs(d, exist_ok=True)
    start = len([f for f in os.listdir(d) if f.endswith('.npy')])

    s = Sensor()
    print(f'Collecting {args.count} images of "{args.label}". Move the finger a little every time.')
    input('Keep the finger OFF the sensor and press ENTER... ')
    s.wait_lift()
    i = 0
    while i < args.count:
        banner(f'PLACE the finger and keep it still  ({i + 1}/{args.count})')
        img, q = s.wait_finger()
        if q < QUALITY_MIN:
            print(f'   x unusable image (quality {q:.0f}), lift and try again')
        else:
            n = start + i
            np.save(os.path.join(d, f'{n:02d}.npy'), img.astype(np.float32))
            save_png(img, os.path.join(d, f'{n:02d}.png'))
            print(f'   ok, saved {n:02d} (quality {q:.0f})')
            i += 1
        banner('LIFT the finger')
        s.wait_lift()
        time.sleep(0.4)
    s.close()
    print('Done.')


if __name__ == '__main__':
    sys.exit(main())
