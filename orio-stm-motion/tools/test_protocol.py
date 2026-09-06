#!/usr/bin/env python3
"""
Runs every orio-stm-motion protocol test in sequence, against one open
serial connection. Each subsystem also has its own standalone script for
testing it in isolation:
    test_heartbeat.py -- link handshake, malformed frames, watchdog timeout
    test_servo.py     -- joint movement, range checks, reset pose
    test_fan.py        -- fan speed ramp, range checks
    test_rgb.py        -- ARGB color cycle

Usage:
    pip install pyserial
    python tools/test_protocol.py COM5
"""
import sys

import test_fan
import test_heartbeat
import test_rgb
import test_servo
from proto_client import open_port, reset_results, results_exit_code


def main():
    # One tally spanning all four subsystems: each module's run() records its
    # own checks, and only the aggregate decides the exit code. The per-module
    # main() functions are bypassed here, so their individual summaries and
    # sys.exit() calls don't fire.
    reset_results()
    ser = open_port()
    try:
        for label, module in (
            ("heartbeat / e-stop", test_heartbeat),
            ("servo / joints", test_servo),
            ("fan", test_fan),
            ("RGB", test_rgb),
        ):
            print(f"\n{'=' * 20} {label} {'=' * 20}")
            module.run(ser)
    finally:
        ser.close()
    sys.exit(results_exit_code("ALL SUBSYSTEMS"))


if __name__ == "__main__":
    main()
