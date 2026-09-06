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

Controls:
  W          forward
  S          reverse
  A          turn left  (in place: left reverse, right forward)
  D          turn right (in place: left forward, right reverse)
  [ / ]      decrease / increase duty by 5%
  space      stop
  q / Esc    quit (stops first)
  Ctrl+C     also quits (stops first) -- works even on the raw-mode Linux
             terminal, since cbreak mode leaves signal generation enabled

Safety: releasing all movement keys auto-stops after a short delay (handles
OS key-repeat gaps without needing space every time); the firmware's own
heartbeat watchdog is still the backstop of last resort if this script
itself hangs or is killed outright.

Usage:
    pip install pyserial
    python tools/teleop_wasd.py COM5           # Windows
    python tools/teleop_wasd.py /dev/ttyACM0   # Linux/Jetson
"""
import sys
import time

from proto_client import (
    CMD_HEARTBEAT,
    CMD_SET_DRIVE,
    arm,
    build_frame,
    open_port,
    set_drive_payload,
)

DEFAULT_DUTY_PERCENT = 30.0
DUTY_STEP_PERCENT = 5.0
HEARTBEAT_INTERVAL_S = 0.2
# Comfortably longer than a typical OS's initial key-repeat delay (often
# ~0.5s before repeat kicks in), so releasing a key doesn't get confused
# with the normal pause before autorepeat starts.
DEADMAN_TIMEOUT_S = 0.8
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


def main():
    ser = open_port()
    duty_percent = DEFAULT_DUTY_PERCENT
    last_sent = (0, 0)
    last_movement_time = 0.0
    last_heartbeat_time = 0.0

    print(__doc__)
    try:
        arm(ser)
        print(f"\nduty={duty_percent:g}% -- hold W/A/S/D to drive, space to stop, q to quit\n")

        with KeyReader() as keys:
            while True:
                now = time.monotonic()
                moved_this_tick = False

                while keys.kbhit():
                    key = keys.getch().lower()
                    if key in (b"q", b"\x1b"):  # q or Esc
                        raise KeyboardInterrupt
                    if key == b"[":
                        duty_percent = max(0.0, duty_percent - DUTY_STEP_PERCENT)
                        print(f"duty={duty_percent:g}%")
                    elif key == b"]":
                        duty_percent = min(100.0, duty_percent + DUTY_STEP_PERCENT)
                        print(f"duty={duty_percent:g}%")
                    elif key == b" ":
                        last_sent = (0, 0)
                        send_drive(ser, 0, 0)
                        print("stop")
                    elif key in DIRECTION_KEYS:
                        left_sign, right_sign = DIRECTION_KEYS[key]
                        permille = round(duty_percent * 10)
                        left = left_sign * permille
                        right = right_sign * permille
                        if (left, right) != last_sent:
                            send_drive(ser, left, right)
                            last_sent = (left, right)
                            print(f"left={left} right={right}")
                        last_movement_time = now
                        moved_this_tick = True

                if not moved_this_tick and last_sent != (0, 0) and (now - last_movement_time) > DEADMAN_TIMEOUT_S:
                    send_drive(ser, 0, 0)
                    last_sent = (0, 0)
                    print("stop (key released)")

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
