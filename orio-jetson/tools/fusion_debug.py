#!/usr/bin/env python3
"""Stereo, the ToF fan, and what the avoider gets — per sector, side by side.

    uv run python tools/fusion_debug.py              # window: camera over the table
    uv run python tools/fusion_debug.py --no-window  # terminal only, over ssh

`stereo_debug.py` shows the cameras and `tof_debug.py` shows the fan; neither
shows the one map the policy actually acts on, or which sensor put each number
into it. This does. It drives nothing: it opens the cameras and the fan, prints,
and quits. Q or Esc closes the window.

## What the rows are

One column per sector, left to right, lined up under the rectified left eye so
a column can be checked against what is in the picture above it.

* **stereo** — `ObstacleDetector`'s map, exactly as `avoid.Sensor` receives it.
* **tof-left / tof-right** — each sensor's own latest map, with its age.
* **ToF fan** — the pair fused by `ToFDetector`, or STALE once it has gone
  quiet and dropped out of the fusion (stereo then decides alone).
* **FUSED** — `fuse(stereo, fan)`, the same call `avoid.Sensor._publish` makes,
  and the map the avoider steers on. The tag under each cell is the sensor that
  won it; the cell it came from is outlined in the rows above.

Cells are coloured by distance on one scale for every row (warm is near), so a
sector where the colours disagree is a sector where the sensors do. `UNK` is
unknown, which the policy treats as blocked. A value marked `~` is not an
obstacle at all: it is ground seen free out to that distance (`clear_m`, the
answer of last resort), and it is worth watching most closely — see the
`TOF_HEIGHTS_M` note in config for how a high mount turns floor into "clear".

## Before reading anything into it

* **Face the head forward.** The cameras ride the neck and the fan is bolted to
  the chassis; with the head turned, the same column is two different
  directions and the comparison means nothing.
* The bump memory is left out on purpose: it only speaks when the robot is
  stalled against something, and this tool never drives.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.sectors import fuse
from orio.stereo import ObstacleDetector

NEAR_M = 0.0
FAR_M = max(config.STEREO_MAX_RANGE_M, config.TOF_MAX_RANGE_M)

GUTTER = 120  # row labels, left of both the image and the table
ROW_H = 44
VIEW_W = 960


def colour_for(distance_m):
    """Distance -> BGR on the turbo scale; near is warm. `None` -> dark grey."""
    import cv2
    import numpy as np

    if distance_m is None:
        return (45, 45, 45)
    frac = min(max((distance_m - NEAR_M) / (FAR_M - NEAR_M), 0.0), 1.0)
    px = cv2.applyColorMap(np.array([[int(255 * (1 - frac))]], np.uint8), cv2.COLORMAP_TURBO)
    return tuple(int(v) for v in px[0, 0])


def cell_text(sector) -> str:
    if sector is None or not sector.known:
        return "UNK"
    mark = "~" if sector.source.endswith("-clear") else ""
    return f"{mark}{sector.distance_m:.2f}"


def short_source(source: str) -> str:
    """`stereo-ground-clear` -> `ground~`, `tof-left` -> `tof-left`."""
    tag = source.replace("stereo-", "")
    return tag.replace("-clear", "~")


def won(sector, fused_sector) -> bool:
    """Did this row's sector supply the fused value?"""
    return (sector is not None and sector.known and fused_sector.known
            and sector.source == fused_sector.source
            and abs(sector.distance_m - fused_sector.distance_m) < 1e-9)


def build_rows(stereo_map, per_sensor, fan_map, fused):
    """(label, note, map-or-None) for each table row, top to bottom."""
    now = time.time()
    rows = [("stereo", "", stereo_map)]
    for name, omap in sorted(per_sensor.items()):
        rows.append((name, f"{now - omap.timestamp:4.2f}s", omap))
    rows.append(("ToF fan", "" if fan_map is not None else "STALE", fan_map))
    rows.append(("FUSED", "", fused))
    return rows


def draw_window(left, rows, fused, header: str):
    import cv2
    import numpy as np

    n = len(fused.sectors)
    img_w = VIEW_W - GUTTER
    scale = img_w / left.shape[1]
    img = cv2.resize(left, (img_w, int(left.shape[0] * scale)), interpolation=cv2.INTER_LINEAR)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    col = [int(i * img_w / n) for i in range(n + 1)]
    for x in col[1:-1]:
        cv2.line(img, (x, 0), (x, img.shape[0] - 1), (200, 200, 200), 1)
    for s in fused.sectors:
        cv2.putText(img, f"s{s.index} {round(s.angle_deg):+d}", (col[s.index] + 4, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    top = np.hstack([np.zeros((img.shape[0], GUTTER, 3), np.uint8), img])

    table = np.zeros((ROW_H * len(rows), VIEW_W, 3), np.uint8)
    for r, (label, note, omap) in enumerate(rows):
        y0 = r * ROW_H
        is_fused = label == "FUSED"
        cv2.putText(table, label, (6, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255) if is_fused else (210, 210, 210), 1, cv2.LINE_AA)
        if note:
            cv2.putText(table, note, (6, y0 + 37), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (80, 80, 255) if note == "STALE" else (160, 160, 160), 1, cv2.LINE_AA)
        for i, fs in enumerate(fused.sectors):
            x0, x1 = GUTTER + col[i] + 2, GUTTER + col[i + 1] - 2
            s = omap.sectors[i] if omap is not None else None
            colour = colour_for(s.distance_m if s is not None and s.known else None)
            cv2.rectangle(table, (x0, y0 + 2), (x1, y0 + ROW_H - 3), colour, -1)
            if not is_fused and won(s, fs):
                cv2.rectangle(table, (x0, y0 + 2), (x1, y0 + ROW_H - 3), (255, 255, 255), 2)
            ink = (0, 0, 0) if sum(colour) > 380 else (255, 255, 255)
            cv2.putText(table, cell_text(s), (x0 + 6, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, ink, 1, cv2.LINE_AA)
            if is_fused and s is not None and s.known and s.source:
                cv2.putText(table, short_source(s.source), (x0 + 6, y0 + 37),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, ink, 1, cv2.LINE_AA)
        if is_fused:
            cv2.line(table, (0, y0), (VIEW_W - 1, y0), (255, 255, 255), 1)

    bar = np.zeros((26, VIEW_W, 3), np.uint8)
    cv2.putText(bar, header, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return np.vstack([bar, top, table])


def print_rows(rows, fused, header: str) -> None:
    print("\033[2J\033[H", end="")
    print(header + "\n")
    print(" " * 20 + "".join(f"{f's{s.index} {round(s.angle_deg):+d}':>11}" for s in fused.sectors))
    for label, note, omap in rows:
        cells = []
        for i, fs in enumerate(fused.sectors):
            s = omap.sectors[i] if omap is not None else None
            text = cell_text(s)
            if label != "FUSED" and won(s, fs):
                text = f"[{text}]"
            cells.append(f"{text:>11}")
        print(f"{label:>12} {note:>6} " + "".join(cells))
    print(" " * 20 + "".join(f"{short_source(s.source) if s.known else '':>11}"
                             for s in fused.sectors))
    print("\n[x] = the row that supplied the fused value; ~ = ground seen clear, not an obstacle")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-window", action="store_true", help="terminal only")
    ap.add_argument("--hz", type=float, default=4.0, help="terminal refresh rate")
    args = ap.parse_args()

    # The fan first, as avoid.Sensor does: its firmware uploads are seconds of
    # I2C, and failing to open it degrades to stereo-only rather than quitting.
    from orio.tof import ToFDetector

    tof: ToFDetector | None = ToFDetector()
    print(f"--- opening the ToF fan ({', '.join(tof.names)}); the firmware upload "
          "takes ~11 s for the pair ---")
    try:
        tof.start()
        if tof.error:
            print(f"ToF fan partly open: {tof.error}")
    except Exception as exc:
        print(f"ToF fan did not open — showing stereo alone. {type(exc).__name__}: {exc}")
        tof.close()
        tof = None

    print("--- opening the cameras ---")
    det = ObstacleDetector()
    if not det._estimator.calibrated:
        print("⚠ stereo uncalibrated — distances are approximate. See tools/calibrate_stereo.py")

    window = "Orio fusion debug — stereo | ToF | fused"
    if not args.no_window:
        import cv2
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    last = time.monotonic()
    last_print = 0.0
    fps = 0.0
    try:
        while True:
            stereo_map, left, _depth = det.sense_with_frames()
            fan_map = tof.fresh_map() if tof is not None else None
            per_sensor = tof.latest if tof is not None else {}
            # The same call avoid.Sensor._publish makes; `None` is skipped.
            fused = fuse(stereo_map, fan_map)

            now = time.monotonic()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - last, 1e-6))
            last = now

            ahead_s = stereo_map.clearance_ahead()
            ahead_f = fused.clearance_ahead()
            fmt = lambda v: "UNK" if v is None else f"{v:.2f} m"  # noqa: E731
            header = (f"stereo {fps:4.1f} fps   ahead: stereo {fmt(ahead_s)} -> fused "
                      f"{fmt(ahead_f)}   {fused.describe()}")
            if tof is None:
                header += "   [ToF OFF]"
            elif tof.error:
                header += f"   [ToF: {tof.error}]"

            rows = build_rows(stereo_map, per_sensor, fan_map, fused)
            if args.no_window:
                if now - last_print >= 1.0 / max(args.hz, 0.1):
                    last_print = now
                    print_rows(rows, fused, header)
            else:
                import cv2
                cv2.imshow(window, draw_window(left, rows, fused, header))
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    return 0
    except KeyboardInterrupt:
        return 0
    finally:
        det.close()
        if tof is not None:
            tof.close()
        if not args.no_window:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
