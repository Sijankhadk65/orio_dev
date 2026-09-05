#!/usr/bin/env python3
"""Stereo-calibrate the IMX219-83 pair from checkerboard views.

Without calibration, `orio.stereo` falls back to published optics and a
measured row offset: obstacles rank correctly but the metres carry real error.
This produces the real thing — per-camera intrinsics and distortion, plus the
rotation and translation between the eyes — and writes it where
`config.STEREO_CALIBRATION` expects it.

    uv run python tools/calibrate_stereo.py

You need a printed checkerboard, rigid and flat: tape it to card or a clipboard,
because a bowed sheet quietly poisons the result. Default target is a 9x6 grid
of *inner corners* (a 10x7 grid of squares). Measure a square with calipers and
pass `--square-mm`; the value sets the scale of the whole calibration, so
guessing it makes every distance wrong by the same factor.

Aim for 20+ pairs with the board at varied distances, angles and corners of the
frame — tilted views constrain the lens model that flat-on views cannot. Both
cameras must see the whole board for a pair to count.

Keys: SPACE captures, C calibrates and saves, Q quits without saving.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from orio import config
from orio.stereo import StereoCamera

# Corner refinement and the calibration solver both stop on this.
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)


def find_corners(gray, pattern):
    """Locate the checkerboard, refined to sub-pixel. None if not fully visible."""
    ok, corners = cv2.findChessboardCorners(
        gray, pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK,
    )
    if not ok:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)


def calibrate(obj_points, left_points, right_points, size, out_path: Path) -> int:
    """Solve intrinsics then extrinsics, report error, and save."""
    print(f"\ncalibrating on {len(obj_points)} pairs at {size[0]}x{size[1]}...")

    err1, K1, D1, *_ = cv2.calibrateCamera(obj_points, left_points, size, None, None)
    err2, K2, D2, *_ = cv2.calibrateCamera(obj_points, right_points, size, None, None)
    print(f"  intrinsics RMS: left {err1:.3f} px, right {err2:.3f} px")

    # Intrinsics are already solved above, so only the pose between the cameras
    # is left free here — refining all of it at once from a handful of views is
    # what produces plausible-looking but subtly wrong extrinsics.
    err, K1, D1, K2, D2, R, T, *_ = cv2.stereoCalibrate(
        obj_points, left_points, right_points, K1, D1, K2, D2, size,
        criteria=CRITERIA, flags=cv2.CALIB_FIX_INTRINSIC,
    )
    baseline_mm = float(np.linalg.norm(T))
    print(f"  stereo RMS:     {err:.3f} px")
    print(f"  baseline:       {baseline_mm:.1f} mm (published: 60.0 mm)")

    if err > 1.0:
        print("  ⚠ RMS above 1 px — usually a bowed board, too few angles, or a")
        print("    wrong --square-mm. Consider recapturing before trusting this.")
    if abs(baseline_mm - 60.0) > 6.0:
        print("  ⚠ baseline is >10% off the published 60 mm, which usually means")
        print("    --square-mm is wrong. Every distance scales with it.")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        out_path,
        K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T,
        image_width=size[0], image_height=size[1],
        rms=err, baseline_mm=baseline_mm,
    )
    print(f"\n✓ saved {out_path}")
    print("  orio.stereo picks this up automatically on next start.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=9, help="inner corners across (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down (default 6)")
    ap.add_argument("--square-mm", type=float, default=25.0, help="printed square size, measured")
    # 1280x960 is an exact 4x of the 320x240 matching resolution. That matters:
    # DepthEstimator rescales the stored intrinsics by a SINGLE factor
    # (width_now / image_width), which is only valid if the calibration and the
    # matching resolution share an aspect ratio. Calibrating at 16:9 against a
    # 4:3 matching size would skew fy against fx and quietly bend every depth.
    ap.add_argument("--width", type=int, default=1280, help="capture width (higher = better corners)")
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--min-pairs", type=int, default=12)
    ap.add_argument("--out", type=Path, default=config.STEREO_CALIBRATION)
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    size = (args.width, args.height)

    # Board model in millimetres, so T comes out in mm and the baseline is
    # directly comparable to the published 60 mm as a sanity check.
    grid = np.zeros((args.rows * args.cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    grid *= args.square_mm

    obj_points, left_points, right_points = [], [], []

    camera = StereoCamera(width=args.width, height=args.height)
    print(f"{pattern[0]}x{pattern[1]} inner corners, {args.square_mm} mm squares")
    print("SPACE capture · C calibrate & save · Q quit\n")

    try:
        while True:
            left, right = camera.read()
            gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            cl, cr = find_corners(gl, pattern), find_corners(gr, pattern)
            both = cl is not None and cr is not None

            view = np.hstack([left.copy(), right.copy()])
            if both:
                cv2.drawChessboardCorners(view[:, : args.width], pattern, cl, True)
                cv2.drawChessboardCorners(view[:, args.width :], pattern, cr, True)
            cv2.putText(
                view, f"pairs: {len(obj_points)}  {'BOARD OK' if both else 'searching...'}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 0) if both else (0, 0, 255), 2, cv2.LINE_AA,
            )
            cv2.imshow("stereo calibration (left | right)", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" ") and both:
                obj_points.append(grid.copy())
                left_points.append(cl)
                right_points.append(cr)
                print(f"  captured pair {len(obj_points)}")
            elif key == ord(" "):
                print("  ✗ board not fully visible in BOTH eyes — not captured")
            elif key in (ord("c"), ord("C")):
                if len(obj_points) < args.min_pairs:
                    print(f"  need at least {args.min_pairs} pairs, have {len(obj_points)}")
                    continue
                return calibrate(obj_points, left_points, right_points, size, args.out)
            elif key in (ord("q"), ord("Q"), 27):
                print("quit without saving")
                return 1
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
