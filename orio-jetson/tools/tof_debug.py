#!/usr/bin/env python3
"""Live view of the ToF fan — the 8x8 grids, their classes, and the sector map.

    uv run python tools/tof_debug.py              # both sensors, per config
    uv run python tools/tof_debug.py --bus 7      # just one, for bench bring-up
    uv run python tools/tof_debug.py --no-window  # terminal only, over ssh

The counterpart to `tools/stereo_debug.py`, and the tool the plan's Phase 2 is
run through. It drives nothing: it opens the sensors, prints, and quits. Q or
Esc closes the window.

## Bring-up order, when nothing works yet

1. `i2cdetect -l` — which bus is actually which. The numbering varies by Jetson
   model and JetPack version, and the config's 7 and 1 are this Orin Nano's.
2. `i2cdetect -y -r 7` — a VL53L5CX answers at **0x29**. Nothing there means
   wiring, not software: check 3.3 V (pin 1/17, never pin 2/4 — those are 5 V
   and this is a 3.3 V part), ground, and that SDA and SCL are not swapped.
3. This tool. The first open takes SECONDS, not milliseconds: ST's ULD pushes an
   ~84 KB firmware blob over I2C at every power-on, about 2 s at 400 kHz and
   nearer 8 s at the 100 kHz default. It is not a hang.
4. Wave a hand at it. If the grid lights up on the wrong side, or upside down,
   that is `ORIO_TOF_FLIP_H` / `ORIO_TOF_FLIP_V` — zone 0 is the sensor's own
   top-left and which physical corner that is depends on how the breakout sits
   on the bracket. Fix it here, not by rotating the bracket.

## What to look at

The grid pane shows each zone's range, coloured by what the height
classification made of it — grey floor, red obstacle, blue above the robot,
black unknown. A LEVEL mount is supposed to see floor in its lower rows: that
is the design, and the height test is what removes it. If the lower rows come
back red instead, the mount pose in config does not match the bracket, and
every number downstream is describing somewhere else.

Untrusted zones are black, never a distance. `target_status` 5 is a trusted
range, 6 and 9 are usable with caveats, and everything else — 255 "no target"
most of all — means UNKNOWN. A low-confidence zone flattened to max range reads
to the policy as clear road, and that failure is silent.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from orio import config
from orio.sectors import fill_clear, fuse
from orio.tof import ToFArray, ToFSensor, arrays_from_config

CLASS_NAMES = {0: "unknown", 1: "floor", 2: "OBSTACLE", 3: "above"}
# Matches tools/stereo_debug.py, so the two views read the same way.
CLASS_COLOURS = {0: (0, 0, 0), 1: (70, 70, 70), 2: (40, 40, 235), 3: (150, 90, 0)}
# ANSI, for the terminal view: dim, grey, red, blue.
CLASS_ANSI = {0: "\033[90m", 1: "\033[37m", 2: "\033[91m", 3: "\033[94m"}


def build_arrays(args) -> list[ToFArray]:
    """Either the configured pair, or one sensor named on the command line.

    The single-sensor path exists for the bench: Phase 2 is "one sensor, before
    anything is mounted", and it should not need the second one wired or the
    mount poses measured to say whether the part works at all.
    """
    if args.bus is None:
        return arrays_from_config()
    return [
        ToFArray(
            ToFSensor(bus=args.bus, address=args.address, name=f"tof-bus{args.bus}"),
            height_m=args.height, pitch_deg=args.pitch, yaw_deg=args.yaw,
        )
    ]


def print_grid(array: ToFArray, ranges, status) -> None:
    """One sensor's 8x8, as metres, coloured by class."""
    import numpy as np

    classes = array.geometry.classify(
        ranges, floor_tol_m=config.STEREO_FLOOR_TOL_M, ceiling_m=config.ROBOT_HEIGHT_M
    )
    rows, cols = array.sensor.rows, array.sensor.cols
    r = np.asarray(ranges).reshape(rows, cols)
    c = np.asarray(classes).reshape(rows, cols)
    st = np.asarray(status).reshape(rows, cols)
    print(f"  {array.name}  (h {array.height_m:.3f} m, pitch {array.pitch_deg:+g}, "
          f"yaw {array.yaw_deg:+g})")
    for i in range(rows):
        cells = []
        for j in range(cols):
            if np.isfinite(r[i, j]):
                cells.append(f"{CLASS_ANSI[c[i, j]]}{r[i, j]:5.2f}\033[0m")
            else:
                # Show the status that rejected it: "no target" and "sigma high"
                # are different problems and only one of them is the geometry's.
                cells.append(f"\033[90m  s{st[i, j]:<2d}\033[0m")
        print("    " + " ".join(cells))


def print_sectors(omap) -> None:
    for s in omap.sectors:
        if s.known:
            bar = "#" * min(40, int(s.distance_m * 20))
            note = " (clear ground, not an obstacle)" if s.source.endswith("-clear") else ""
            print(f"    s{s.index} {s.angle_deg:+6.1f} deg  {s.distance_m:5.2f} m  "
                  f"{s.valid_frac:4.0%}  {bar}{note}")
        else:
            print(f"    s{s.index} {s.angle_deg:+6.1f} deg  UNKNOWN       "
                  f"{s.valid_frac:4.0%}")


def draw_window(arrays, grids, omap):
    """Grids on the left, the derived sector map on the right."""
    import cv2
    import numpy as np

    cell, pad = 48, 8
    panes = []
    for array in arrays:
        frame = grids.get(array.name)
        rows, cols = array.sensor.rows, array.sensor.cols
        pane = np.zeros((rows * cell, cols * cell, 3), np.uint8)
        if frame is not None:
            ranges, _status = frame
            classes = np.asarray(
                array.geometry.classify(ranges, floor_tol_m=config.STEREO_FLOOR_TOL_M,
                                        ceiling_m=config.ROBOT_HEIGHT_M)
            ).reshape(rows, cols)
            r = np.asarray(ranges).reshape(rows, cols)
            for i in range(rows):
                for j in range(cols):
                    y, x = i * cell, j * cell
                    pane[y:y + cell, x:x + cell] = CLASS_COLOURS[int(classes[i, j])]
                    cv2.rectangle(pane, (x, y), (x + cell - 1, y + cell - 1), (30, 30, 30), 1)
                    if np.isfinite(r[i, j]):
                        cv2.putText(pane, f"{r[i, j]:.2f}", (x + 3, y + cell // 2 + 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
                                    cv2.LINE_AA)
        label = np.zeros((22, pane.shape[1], 3), np.uint8)
        cv2.putText(label, array.name, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (220, 220, 220), 1, cv2.LINE_AA)
        panes.append(np.vstack([label, pane]))

    grid_view = np.hstack([
        np.hstack([p, np.zeros((p.shape[0], pad, 3), np.uint8)]) for p in panes
    ])

    n = len(omap.sectors)
    sectors_view = np.zeros((grid_view.shape[0], max(240, n * 52), 3), np.uint8)
    h = sectors_view.shape[0]
    for s in omap.sectors:
        x0, x1 = int(s.index * sectors_view.shape[1] / n) + 2, \
                 int((s.index + 1) * sectors_view.shape[1] / n) - 2
        if s.known:
            frac = 1 - min(s.distance_m / max(config.TOF_MAX_RANGE_M, 1e-6), 1.0)
            cv2.rectangle(sectors_view, (x0, h - int((h - 30) * frac)), (x1, h - 4),
                          (0, int(255 * (1 - frac)), int(255 * frac)), -1)
            text = f"{s.distance_m:.2f}"
        else:
            text = "UNK"
        cv2.putText(sectors_view, text, (x0, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (230, 230, 230), 1, cv2.LINE_AA)
    return np.hstack([grid_view, sectors_view])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=None,
                    help="bench mode: one sensor on this I2C bus, ignoring the configured pair")
    ap.add_argument("--address", type=lambda v: int(v, 0), default=0x29)
    ap.add_argument("--height", type=float, default=config.TOF_HEIGHTS_M[0],
                    help="bench mode: mount height in metres")
    ap.add_argument("--pitch", type=float, default=0.0, help="bench mode: pitch, + is down")
    ap.add_argument("--yaw", type=float, default=0.0, help="bench mode: yaw, + is right")
    ap.add_argument("--no-window", action="store_true", help="terminal only")
    ap.add_argument("--hz", type=float, default=4.0, help="terminal refresh rate")
    args = ap.parse_args()

    print(__doc__)
    arrays = build_arrays(args)
    print(f"--- opening {len(arrays)} sensor(s); the firmware upload takes seconds ---")
    opened = []
    for array in arrays:
        try:
            array.sensor.open()
            opened.append(array)
        except Exception as exc:
            print(f"\n{array.name} on /dev/i2c-{array.sensor.bus} at "
                  f"0x{array.sensor.address:02x}: {type(exc).__name__}: {exc}")
    if not opened:
        print("\nNo sensor opened. Work down the bring-up list at the top of this file — "
              "`i2cdetect -y -r <bus>` first, and expect 0x29.")
        return 1

    window = "Orio ToF debug — grids | sectors"
    if not args.no_window:
        import cv2
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    grids: dict = {}
    maps: dict = {}
    last_print = 0.0
    try:
        while True:
            for array in opened:
                frame = array.sensor.read()
                if frame is None:
                    continue
                grids[array.name] = frame
                maps[array.name] = array.reduce(frame[0])

            if not maps:
                time.sleep(0.02)
                continue
            omap = fill_clear(fuse(*maps.values()))

            now = time.monotonic()
            if now - last_print >= 1.0 / max(args.hz, 0.1):
                last_print = now
                print("\033[2J\033[H", end="")
                print(f"ToF fan — {len(opened)} sensor(s) @ {config.TOF_FREQ_HZ} Hz, "
                      f"trusted status {sorted(config.TOF_TRUSTED_STATUS)}\n")
                for array in opened:
                    if array.name in grids:
                        print_grid(array, *grids[array.name])
                        print()
                print(f"  fused sector map ({omap.describe()})")
                print_sectors(omap)

            if not args.no_window:
                import cv2
                cv2.imshow(window, draw_window(opened, grids, omap))
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    return 0
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        return 0
    finally:
        for array in opened:
            array.sensor.close()
        if not args.no_window:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
