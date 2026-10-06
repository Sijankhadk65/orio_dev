#!/usr/bin/env python3
"""Bring-up check for the Gemini 336L: is it there, what does it offer, does depth flow.

    uv run python tools/gemini_check.py              # enumerate, open, sample 60 frames
    uv run python tools/gemini_check.py --list       # also list every stream profile
    uv run python tools/gemini_check.py --frames 300 # longer sample

Run this first after plugging the camera in, and after any change to
ORIO_GEMINI_*. It drives nothing and opens no window, so it works over ssh.

## Bring-up order, when nothing works yet

1. `lsusb | grep -i 2bc5` — Orbbec's vendor id. Nothing there is a cable or port
   problem, not software. It wants a USB3 port: on USB2 the SDK still opens it
   but the high-bandwidth modes are missing, and 640x400 depth + colour at 30 fps
   may not be offered at all.
2. Present in lsusb but "no Orbbec camera found" here means permissions: install
   the udev rule (`sudo cp udev/99-orbbec.rules /etc/udev/rules.d/ && sudo
   udevadm control --reload && sudo udevadm trigger`), then replug.
3. This tool. It prints the device, the intrinsics the obstacle map will use,
   whether depth-to-colour alignment ran in hardware, and what fraction of the
   frame carries valid depth. Point it at a room, not a blank wall at 10 cm.

## What to look at

* **hfov** — must be close to `STEREO_HFOV_DEG` (94 by default), which the ToF
  fan and the bump memory lay their sectors out with. If it is not, set
  `ORIO_STEREO_HFOV_DEG` to the printed value.
* **valid depth** — a lit room should be well above half. Very low with a
  sensible scene means the stream is not what it claims; the per-sector line
  shows whether it is everywhere or just the frame edges (the colour camera
  sees slightly wider than the depth pair, so the outermost columns are thin).
* **fps** — should sit at `GEMINI_FPS`. Much lower usually means USB2.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from orio import config
from orio.gemini import GeminiCamera, route_sdk_log
from orio.stereo import DepthReducer


def list_profiles() -> None:
    import pyorbbecsdk as ob

    pipeline = ob.Pipeline()
    for label, sensor in (("colour", ob.OBSensorType.COLOR_SENSOR),
                          ("depth", ob.OBSensorType.DEPTH_SENSOR)):
        profiles = pipeline.get_stream_profile_list(sensor)
        print(f"{label} profiles:")
        for i in range(profiles.get_count()):
            p = profiles.get_stream_profile_by_index(i).as_video_stream_profile()
            print(f"    {p.get_width()}x{p.get_height()} @ {p.get_fps()} fps  {p.get_format()}")
    del pipeline  # release the device before GeminiCamera opens it


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--list", action="store_true", help="list every stream profile")
    parser.add_argument("--frames", type=int, default=60, help="frames to sample")
    args = parser.parse_args()
    # gemini.py reports the negotiated streams and D2C mode at INFO.
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    try:
        import pyorbbecsdk as ob
    except ImportError:
        print("pyorbbecsdk is not installed — run `uv sync`")
        return 1

    # Keep the context in a variable: the device list borrows its device
    # manager, and a temporary Context is freed before the list is read
    # ("NULL pointer passed for argument deviceMgr").
    route_sdk_log(ob)  # before the first Context, or the SDK logs into the cwd
    ctx = ob.Context()
    devices = ctx.query_devices()
    if devices.get_count() == 0:
        print("no Orbbec camera found — see step 1 and 2 in this tool's docstring")
        return 1
    # List-level getters only: get_device_by_index() would OPEN the device, and
    # the pipeline below needs to open it itself.
    for i in range(devices.get_count()):
        print(f"found: {devices.get_device_name_by_index(i)}  "
              f"serial {devices.get_device_serial_number_by_index(i)}  "
              f"{devices.get_device_connection_type_by_index(i)}")
    del devices, ctx

    if args.list:
        list_profiles()

    cam = GeminiCamera()
    try:
        cam.start()
        k = cam.intrinsics
        print(f"opened: {cam.device_name}")
        print(f"intrinsics @ {k.width}x{k.height}: fx {k.fx:.1f} fy {k.fy:.1f} "
              f"cx {k.cx:.1f} cy {k.cy:.1f}  ->  hfov {k.hfov_deg:.1f} deg, "
              f"vfov {k.vfov_deg:.1f} deg  (STEREO_HFOV_DEG = {config.STEREO_HFOV_DEG:g})")

        reducer = DepthReducer(k)
        seq, valid, t0 = 0, [], None
        omap = None
        for _ in range(args.frames):
            frames = cam.read(after=seq)
            seq = frames.seq
            if t0 is None:
                t0 = time.monotonic()  # time from the first frame, not the open
            valid.append(float(np.isfinite(frames.depth).mean()))
            omap = reducer.obstacles(frames.depth)
        elapsed = time.monotonic() - t0
        fps = (len(valid) - 1) / elapsed if elapsed > 0 else 0.0

        print(f"{len(valid)} frames at {fps:.1f} fps (GEMINI_FPS = {config.GEMINI_FPS})")
        print(f"valid depth: {np.mean(valid):.0%} of the frame "
              f"(range gate {config.STEREO_MIN_RANGE_M:.2f}-{config.STEREO_MAX_RANGE_M:.1f} m)")
        print("last sector map: " + omap.describe())
        print("per sector, valid fraction in the band: "
              + "  ".join(f"{s.angle_deg:+.0f}:{s.valid_frac:.0%}" for s in omap.sectors))
        return 0
    finally:
        cam.close()


if __name__ == "__main__":
    sys.exit(main())
