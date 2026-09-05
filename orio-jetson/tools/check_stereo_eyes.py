#!/usr/bin/env python3
"""Measure which Argus sensor is physically the LEFT eye.

    uv run python tools/check_stereo_eyes.py

Run this after ANY change to the CSI cabling. The eye/sensor mapping is a fact
about which ribbon is in which port, not a constant, and getting it backwards
is silent: disparity comes out negative, `DepthEstimator.depth()` discards
every negative disparity as unknown, and the depth map goes blank instead of
raising. The same run also re-measures the vertical offset.

Grabs pairs from sensor 0 and sensor 1, matches ORB features, and reports the
median horizontal shift dx = x(sensor0) - x(sensor1). A feature sits further
RIGHT in the LEFT camera's image, so disparity = x_left - x_right > 0, hence:

    dx > 0  -> sensor 0 is the LEFT eye
    dx < 0  -> sensor 1 is the LEFT eye

Point the cameras at a textured, reasonably lit scene first — a blank wall or a
dark close-up yields too few matches and the run says so rather than guessing.
Holds both sensors, so stop the vision tool and the main app before running.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from orio import config
from orio.stereo import _argus_pipeline

W, H, FPS = 640, 360, 30
WARMUP = 8


def open_cap(sid):
    cap = cv2.VideoCapture(_argus_pipeline(sid, W, H, FPS), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(f"could not open Argus sensor {sid}")
    return cap


def robust_median(v, k=2.5):
    v = np.asarray(v, float)
    m = np.median(v)
    mad = np.median(np.abs(v - m)) or 1e-6
    keep = np.abs(v - m) < k * 1.4826 * mad
    return float(np.median(v[keep])), int(keep.sum())


def main():
    caps = [open_cap(0), open_cap(1)]
    try:
        for _ in range(WARMUP):          # let auto-exposure settle
            for c in caps:
                c.grab()
            for c in caps:
                c.retrieve()

        dxs, dys, per_frame = [], [], []
        orb = cv2.ORB_create(nfeatures=3000)
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)

        for trial in range(5):
            for c in caps:
                c.grab()
            frames = []
            for sid, c in zip((0, 1), caps):
                ok, f = c.retrieve()
                if not ok:
                    raise SystemExit(f"no frame from sensor {sid}")
                frames.append(f)
            g0 = cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY)
            g1 = cv2.cvtColor(frames[1], cv2.COLOR_BGR2GRAY)
            k0, d0 = orb.detectAndCompute(g0, None)
            k1, d1 = orb.detectAndCompute(g1, None)
            if d0 is None or d1 is None or len(k0) < 10 or len(k1) < 10:
                print(f"  trial {trial}: too few features ({len(k0)}/{len(k1)})")
                continue
            good = [m for m, n in bf.knnMatch(d0, d1, k=2) if m.distance < 0.75 * n.distance]
            if len(good) < 10:
                print(f"  trial {trial}: too few matches ({len(good)})")
                continue
            fdx = [k0[m.queryIdx].pt[0] - k1[m.trainIdx].pt[0] for m in good]
            fdy = [k0[m.queryIdx].pt[1] - k1[m.trainIdx].pt[1] for m in good]
            mdx, n = robust_median(fdx)
            mdy, _ = robust_median(fdy)
            per_frame.append(mdx)
            print(f"  trial {trial}: {len(good):4d} matches, dx={mdx:+7.2f} px, dy={mdy:+6.2f} px (inliers {n})")
            dxs += fdx
            dys += fdy

        if not per_frame:
            raise SystemExit("no usable matches — point the cameras at a textured scene and retry")

        dx, nx = robust_median(dxs)
        dy, _ = robust_median(dys)
        print(f"\nPooled over {len(per_frame)} frames at {W}x{H}:")
        print(f"  dx = x(sensor0) - x(sensor1) = {dx:+.2f} px   ({nx} inliers)")
        print(f"  dy = y(sensor0) - y(sensor1) = {dy:+.2f} px")
        left = 0 if dx > 0 else 1
        right = 1 - left
        print(f"\n  => sensor {left} is the LEFT eye, sensor {right} is the RIGHT eye")
        print(f"  config currently says LEFT={config.STEREO_LEFT_SENSOR_ID}, "
              f"RIGHT={config.STEREO_RIGHT_SENSOR_ID}")
        ok = (config.STEREO_LEFT_SENSOR_ID == left)
        print("  => disparity with the current config would be "
              + ("POSITIVE (correct)" if ok else "NEGATIVE (eyes swapped in config)"))
        vfrac = dy / H
        print(f"\n  vertical offset as a fraction of height: {abs(vfrac):.5f} "
              f"(config STEREO_FALLBACK_VSHIFT_FRAC = {config.STEREO_FALLBACK_VSHIFT_FRAC:.5f})")
    finally:
        for c in caps:
            c.release()


if __name__ == "__main__":
    main()
