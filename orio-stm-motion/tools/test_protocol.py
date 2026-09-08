#!/usr/bin/env python3
"""
Runs every orio-stm-motion protocol test in sequence, against one open
serial connection. Each subsystem also has its own standalone script for
testing it in isolation:
    test_whoami.py    -- board identity: role, and that WHOAMI neither needs
                         an armed link nor feeds the watchdog
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
import test_whoami
from proto_client import open_port, reset_results, results_exit_code


def main():
    # One tally spanning every subsystem: each module's run() records its own
    # checks, and only the aggregate decides the exit code. The per-module
    # main() functions are bypassed here, so their individual summaries and
    # sys.exit() calls don't fire.
    #
    # Identity goes first on purpose. Every check after it assumes the board
    # answering is the motion board, and on the wrong board most of them would
    # still pass -- the framing and CRC are shared, so a wheel-side board ACKs
    # the link-level traffic quite happily. A run that opened the wrong port
    # should say so up front rather than fail obscurely somewhere in the servo
    # section.
    reset_results()
    ser = open_port()
    try:
        for label, module in (
            ("identity handshake", test_whoami),
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
