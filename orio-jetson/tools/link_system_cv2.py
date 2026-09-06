#!/usr/bin/env python3
"""Link JetPack's system OpenCV into the project venv. Idempotent.

Orio needs an OpenCV built with GStreamer, because the IMX219 CSI cameras can
only be debayered by the Jetson ISP, reached through an `nvarguscamerasrc`
pipeline. PyPI's `opencv-python` wheels are built *without* GStreamer, so on
them the pipeline never opens and a plain V4L2 fallback yields a solid green
frame while reporting success. JetPack ships a GStreamer-enabled build, but
only for the system interpreter — hence this symlink, and hence the
`requires-python = ">=3.12,<3.13"` pin in pyproject.toml.

`opencv-python` is kept out of the venv by `[tool.uv] override-dependencies`,
so nothing shadows the link. Re-run this after recreating the venv:

    uv run python tools/link_system_cv2.py
"""

from __future__ import annotations

import sys
import sysconfig
from pathlib import Path

SYSTEM_CV2 = Path("/usr/lib/python3.12/dist-packages/cv2")


def main() -> int:
    if not SYSTEM_CV2.is_dir():
        print(f"✗ system OpenCV not found at {SYSTEM_CV2}")
        print("  Install it with: sudo apt install python3-opencv")
        return 1

    site_packages = Path(sysconfig.get_paths()["purelib"])
    link = site_packages / "cv2"

    if link.is_symlink():
        if link.readlink() == SYSTEM_CV2:
            print(f"✓ already linked: {link} -> {SYSTEM_CV2}")
            return check_import()
        link.unlink()
    elif link.exists():
        # A real directory here means the pip wheel got installed after all,
        # which would shadow the system build. Say so rather than delete it.
        print(f"✗ {link} is a real directory, not a link — a PyPI opencv-python")
        print("  wheel is installed and would shadow the system build.")
        print("  Remove it (`uv pip uninstall opencv-python`) and re-run.")
        return 1

    link.symlink_to(SYSTEM_CV2)
    print(f"✓ linked {link} -> {SYSTEM_CV2}")
    return check_import()


def check_import() -> int:
    try:
        import cv2
    except Exception as exc:  # numpy ABI mismatch shows up here
        print(f"✗ import cv2 failed: {type(exc).__name__}: {exc}")
        print("  The system build is compiled against numpy 1.x; the venv must")
        print("  keep `numpy<2` (see pyproject.toml) or the import breaks.")
        return 1

    gst = [l for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
    has_gst = bool(gst) and "YES" in gst[0]
    print(f"  cv2 {cv2.__version__}, GStreamer: {'YES' if has_gst else 'NO'}")
    if not has_gst:
        print("✗ this OpenCV has no GStreamer support — CSI capture will not work.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
