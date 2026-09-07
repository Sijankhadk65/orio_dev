#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial"]
# ///
"""
Live WASD teleop for bench-testing the drivetrain. Cross-platform (Windows
and Linux/macOS) -- stdlib only, no extra dependency for keyboard input.

There's no "move forward"/"turn left" function anywhere in the firmware --
the only primitive it exposes is Wheel_SetSpeeds(left_permille,
right_permille) in Core/Src/wheel.c. Every direction below is just a
specific (left, right) pair of that one function, computed here on the
host side and sent as a CMD_SET_DRIVE frame.

CMD_SET_DRIVE frames do NOT reset the firmware's heartbeat watchdog --
only CMD_HEARTBEAT does (see protocol.c). So this sends heartbeats on its
own timer in the background, independent of whatever WASD is doing;
otherwise the board would e-stop every 500ms regardless of input.

Controls (latched, not held -- tap a direction once and it keeps going):
  W          forward
  S          reverse
  A          turn left  (in place: left reverse, right forward)
  D          turn right (in place: left forward, right reverse)
  [ / ]      decrease / increase duty by 5%
  space      stop
  q / Esc    quit (stops first)
  Ctrl+C     also quits (stops first) -- works even on the raw-mode Linux
             terminal, since cbreak mode leaves signal generation enabled

Safety: there's no auto-stop on its own here -- once latched, a direction
keeps going until you press space, a different direction, or quit. The
firmware's heartbeat watchdog is still the backstop if this script itself
hangs or is killed outright (no more heartbeats -> e-stop within 500ms),
but walking away from the keyboard without pressing space will NOT stop
the robot on its own the way it used to.

Usage:
    pip install pyserial
    python tools/teleop_wasd.py COM5                    # Windows
    python tools/teleop_wasd.py /dev/ttyACM0             # Linux/Jetson
    python tools/teleop_wasd.py COM5 --duty 50           # start at 50% instead of the default
"""
import argparse
import sys
import time

from proto_client import (
    CMD_HEARTBEAT,
    CMD_SET_DRIVE,
    arm,
    build_frame,
    open_serial,
    set_drive_payload,
)

DEFAULT_DUTY_PERCENT = 30.0
DUTY_STEP_PERCENT = 5.0
HEARTBEAT_INTERVAL_S = 0.2
LOOP_TICK_S = 0.03

# (left_sign, right_sign)
DIRECTION_KEYS = {
    b"w": (1, 1),
    b"s": (-1, -1),
    b"a": (-1, 1),
    b"d": (1, -1),
}

if sys.platform == "win32":
    import msvcrt

    class KeyReader:
        """Non-blocking single-key reads via msvcrt -- no terminal mode to
        change, so entering/exiting this is a no-op."""

        def __enter__(self):
            return self

        def __exit__(self, *exc_info):
            return False

        def kbhit(self):
            return msvcrt.kbhit()

        def getch(self):
            return msvcrt.getch()

else:
    import select
    import termios
    import tty

    class KeyReader:
        """Non-blocking single-key reads on POSIX by putting the terminal in
        cbreak mode (unbuffered, one keystroke at a time) and polling stdin
        with a zero-timeout select(). Restores the original terminal
        settings on exit -- without that, the shell would be left unusable
        after the script quits."""

        def __enter__(self):
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)  # keeps ISIG on, so Ctrl+C still works
            return self

        def __exit__(self, *exc_info):
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
            return False

        def kbhit(self):
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            return bool(ready)

        def getch(self):
            return sys.stdin.read(1).encode()


def send_drive(ser, left_permille, right_permille):
    ser.write(build_frame(CMD_SET_DRIVE, set_drive_payload(left_permille, right_permille)))


def drain(ser):
    """Discards whatever ACK/STATUS bytes have piled up. This loop fires
    frames faster than it's worth reading responses for, but the input
    buffer still needs emptying so it doesn't grow unbounded over a long
    session."""
    if ser.in_waiting:
        ser.read(ser.in_waiting)


def parse_args():
    parser = argparse.ArgumentParser(description="Live WASD teleop for bench-testing the drivetrain.")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument(
        "--duty",
        type=float,
        default=DEFAULT_DUTY_PERCENT,
        help=f"starting duty cycle percent, 0-100 (default {DEFAULT_DUTY_PERCENT:g}); adjustable live with [ and ]",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not (0.0 <= args.duty <= 100.0):
        print("--duty must be between 0 and 100")
        sys.exit(1)

    ser = open_serial(args.port)
    duty_percent = args.duty
    last_sent = (0, 0)
    current_direction = None  # (left_sign, right_sign) of the active latch, or None once stopped
    last_heartbeat_time = 0.0

    def apply_direction(signs):
        nonlocal last_sent, current_direction
        left_sign, right_sign = signs
        permille = round(duty_percent * 10)
        left = left_sign * permille
        right = right_sign * permille
        if (left, right) != last_sent:
            send_drive(ser, left, right)
            last_sent = (left, right)
            print(f"left={left} right={right}")
        current_direction = signs

    print(__doc__)
    try:
        arm(ser)
        print(f"\nduty={duty_percent:g}% -- tap W/A/S/D to drive, space to stop, q to quit\n")

        with KeyReader() as keys:
            while True:
                now = time.monotonic()

                while keys.kbhit():
                    key = keys.getch().lower()
                    if key in (b"q", b"\x1b"):  # q or Esc
                        raise KeyboardInterrupt
                    if key == b"[":
                        duty_percent = max(0.0, duty_percent - DUTY_STEP_PERCENT)
                        print(f"duty={duty_percent:g}%")
                        if current_direction is not None:
                            apply_direction(current_direction)
                    elif key == b"]":
                        duty_percent = min(100.0, duty_percent + DUTY_STEP_PERCENT)
                        print(f"duty={duty_percent:g}%")
                        if current_direction is not None:
                            apply_direction(current_direction)
                    elif key == b" ":
                        last_sent = (0, 0)
                        current_direction = None
                        send_drive(ser, 0, 0)
                        print("stop")
                    elif key in DIRECTION_KEYS:
                        apply_direction(DIRECTION_KEYS[key])

                if (now - last_heartbeat_time) > HEARTBEAT_INTERVAL_S:
                    ser.write(build_frame(CMD_HEARTBEAT))
                    last_heartbeat_time = now

                drain(ser)
                time.sleep(LOOP_TICK_S)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n--- stopping ---")
        send_drive(ser, 0, 0)
        time.sleep(0.05)
        drain(ser)
        ser.close()


if __name__ == "__main__":
    main()
