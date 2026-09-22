#!/usr/bin/env python3
"""Live depth + obstacle-sector view. The stereo counterpart to ORIO_VISION_DEBUG.

    uv run python tools/stereo_debug.py

Left pane is the RECTIFIED left camera with sector distances drawn over it;
right pane is the colourised depth map (warm = near, cool = far, black =
unknown), or — press G — the HEIGHT CLASSIFICATION that ORIO_STEREO_GROUND_PLANE
steers on: grey floor, red obstacle, blue driven-under, black unknown.

That second view is the one to tune STEREO_CAM_HEIGHT_M and
STEREO_CAM_PITCH_DEG against, and doing it by eye is not a shortcut: a degree
or two of pitch error tilts the fitted plane enough that distant floor turns
red, and the robot then refuses to leave `steer` for `cruise` in an empty room.
Red creeping up the floor toward the horizon means the pitch is too shallow;
a box that stays grey means it is too steep. Rectified, because that is the frame the depth map is computed in —
the raw capture is displaced from it by enough to make the comparison useless.
Black borders in the left pane are therefore real: they are the part of the
output frame that rectification leaves undefined, and no depth can exist there. The bar
under each sector shows its distance, and a sector with too few valid pixels
reads UNKNOWN rather than a number — the distinction that matters, since
"unknown" must never be acted on as "clear". The tag above each bar names the
sensor that produced it, which is how a disagreement between the cameras and
the ToF fan becomes visible rather than arguable.

A dev aid only: it reads the cameras and prints, and drives nothing. Q or Esc
quits. Note it holds both sensors, so nothing else can capture while it runs.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from orio import config
from orio.stereo import ObstacleDetector

NEAR_M, FAR_M = config.STEREO_MIN_RANGE_M, config.STEREO_MAX_RANGE_M


def colourise(depth):
    """Depth in metres -> BGR. Near is warm, far is cool, unknown is black."""
    valid = np.isfinite(depth)
    norm = np.zeros(depth.shape, np.uint8)
    if valid.any():
        clipped = np.clip(depth[valid], NEAR_M, FAR_M)
        norm[valid] = (255 * (1 - (clipped - NEAR_M) / (FAR_M - NEAR_M))).astype(np.uint8)
    out = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    out[~valid] = 0
    return out


# Floor / obstacle / above-the-robot / unknown, in the order sectors.classify
# numbers them. Deliberately not a pretty palette: the only question this view
# answers is which pixels the policy is about to act on.
CLASS_COLOURS = {
    0: (0, 0, 0),         # unknown — no depth
    1: (70, 70, 70),      # floor — eliminated by geometry, not by cropping
    2: (40, 40, 235),     # obstacle — between the floor tolerance and the robot
    3: (150, 90, 0),      # above the robot — driven under
}


def colourise_classes(classes, stride, shape):
    """Per-pixel class -> BGR, upscaled back to the depth map's size."""
    out = np.zeros(classes.shape + (3,), np.uint8)
    for value, colour in CLASS_COLOURS.items():
        out[classes == value] = colour
    h, w = shape
    return cv2.resize(out, (w, h), interpolation=cv2.INTER_NEAREST)


def draw_sectors(frame, omap):
    """Overlay sector boundaries, distances and validity onto the left view."""
    h, w = frame.shape[:2]
    n = len(omap.sectors)
    top, bottom = int(config.STEREO_BAND_TOP * h), int(config.STEREO_BAND_BOTTOM * h)
    cv2.rectangle(frame, (0, top), (w - 1, bottom), (90, 90, 90), 1)

    # Bars live in a strip along the bottom rather than filling the band: at
    # typical indoor ranges every sector is near, so full-height bars cover the
    # very image you are trying to check them against.
    strip = max(18, int(0.18 * h))
    y1 = bottom
    y0 = bottom - strip
    overlay = frame.copy()

    for s in omap.sectors:
        x0, x1 = int(s.index * w / n), int((s.index + 1) * w / n)
        cv2.line(frame, (x0, top), (x0, bottom), (90, 90, 90), 1)
        if s.known:
            # Nearer reads hotter, so the eye finds the hazard without reading
            # numbers. Bar length is scaled over the near half of the range,
            # where obstacle decisions actually get made.
            span = max(FAR_M / 2 - NEAR_M, 1e-6)
            frac = 1 - min(max(s.distance_m - NEAR_M, 0) / span, 1)
            colour = (0, int(255 * (1 - frac)), int(255 * frac))
            label = f"{s.distance_m:.2f}"
            cv2.rectangle(overlay, (x0 + 2, y1 - int(strip * frac)), (x1 - 2, y1), colour, -1)
        else:
            colour, label = (150, 150, 150), "UNK"
            cv2.rectangle(overlay, (x0 + 2, y0), (x1 - 2, y1), (60, 60, 60), -1)
        cv2.putText(frame, label, (x0 + 4, top - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)
        cv2.putText(frame, f"{s.valid_frac:.0%}", (x0 + 4, y0 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1, cv2.LINE_AA)
        if s.source:
            # Which sensor won this sector. "band" is the row crop, "ground" the
            # height classification, anything else is a ToF array.
            tag = s.source.replace("stereo-", "")
            cv2.putText(frame, tag, (x0 + 4, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.32, (210, 210, 210), 1, cv2.LINE_AA)

    # Blend so the scene stays readable underneath the bars.
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    return frame


def main() -> int:
    det = ObstacleDetector()
    if not det._estimator.calibrated:
        print("⚠ uncalibrated — distances are approximate. See tools/calibrate_stereo.py")

    window = "Orio stereo debug — left + sectors | depth"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    last = time.monotonic()
    fps = 0.0
    show_classes = config.STEREO_GROUND_PLANE
    if not config.STEREO_GROUND_PLANE:
        print("ground-plane classification is OFF (ORIO_STEREO_GROUND_PLANE=0) — "
              "G still shows what it WOULD classify, using STEREO_CAM_HEIGHT_M "
              f"{config.STEREO_CAM_HEIGHT_M:.2f} m / pitch "
              f"{config.STEREO_CAM_PITCH_DEG:g} deg")

    try:
        while True:
            omap, left, depth = det.sense_with_frames()
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - last, 1e-6))
            last = now

            if show_classes:
                classes, stride = det._estimator.classify(depth)
                right_pane = colourise_classes(classes, stride, depth.shape)
            else:
                right_pane = colourise(depth)
            view = np.hstack([draw_sectors(left.copy(), omap), right_pane])
            # Upscale for viewing: matching runs at 320x180 by design, which is
            # unreadably small on screen but is the resolution that actually
            # gives both speed and the most valid pixels.
            if view.shape[1] < 960:
                view = cv2.resize(view, None, fx=960 / view.shape[1], fy=960 / view.shape[1],
                                  interpolation=cv2.INTER_NEAREST)
            mode = "height classes" if show_classes else "depth"
            cv2.putText(view, f"{fps:4.1f} fps   {omap.describe()}   [{mode}, G toggles]",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.imshow(window, view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("g"), ord("G")):
                show_classes = not show_classes
            elif key in (ord("q"), ord("Q"), 27):
                return 0
    finally:
        det.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
