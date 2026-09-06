#!/usr/bin/env python3
"""
Steps one axis of one joint outward in small increments so you can watch for
the point where it stops moving freely -- the mechanical limit of that
bracket. Use it to MEASURE a joint's usable window before writing the result
into kJointLimits[] in Core/Src/servo_joint.c.

This is a calibration tool, not a test. It deliberately walks a servo toward
somewhere it may not be able to go, so read the safety notes below before
running it on an assembled joint.

    SAFETY
    - Unload the axis first where you can: take the horn off the bracket, or
      unbolt the head. An unloaded servo that reaches its end stop just sits
      there; a loaded one levers against whatever it is carrying.
    - Keep a hand on the power. A servo commanded past a mechanical stop
      stalls: it buzzes or hums, gets hot, and draws locked-rotor current
      until something gives. Cut power the moment you hear it.
    - Step outward from the middle, never inward from an end. The middle of
      a servo's travel is the one place it is definitely not against a stop.
    - Stop at the last angle that moved cleanly AND held quietly. The first
      angle that buzzes is already past the limit, not at it.

The firmware clamps every command to the joint's CURRENT window, so a sweep
that runs past it comes back NACK OUT_OF_RANGE rather than moving. To
measure a window wider than the one already configured you must temporarily
widen that row in kJointLimits[] to the servo's full travel, flash, measure,
then write the measured numbers back. This script detects the clamp and says
so rather than leaving you wondering why the servo stopped.

Usage:
    python tools/sweep_axis.py <port> <joint> <pan|tilt> <from> <to> [step]

Examples:
    # neck pan, from mid-travel outward toward the 270 end, 5 deg at a time
    python tools/sweep_axis.py COM3 neck pan 135 270 5

    # neck tilt, from level downward, 2 deg at a time
    python tools/sweep_axis.py COM3 neck tilt 90 60 2
"""
import argparse
import sys
import threading

from proto_client import (
    CMD_ACK,
    CMD_GET_STATUS,
    CMD_HEARTBEAT,
    CMD_MOVE_JOINT_TO,
    CMD_NACK,
    HEARTBEAT_INTERVAL_S,
    JOINT_POS_NAMES,
    NACK_ESTOPPED,
    NACK_OUT_OF_RANGE,
    NACK_REASONS,
    PAN_SERVO_CALIB,
    TILT_SERVO_CALIB,
    arm,
    axis_limit,
    build_frame,
    hold_alive,
    move_joint_payload,
    open_serial,
    read_frame,
)

JOINT_NAME_TO_POS = {name: pos for pos, name in JOINT_POS_NAMES.items()}


class Keepalive:
    """Sends heartbeats from a background thread while the main thread sits
    blocked on input().

    The board e-stops after PROTO_HEARTBEAT_TIMEOUT_MS (500 ms) of silence,
    which is far shorter than a human takes to read a prompt. Without this the
    first pause would disarm the joint -- cutting PWM, letting the axis go
    limp -- and every step after it would come back NACK ESTOPPED.

    Only one thread touches the port at a time: this one runs solely while the
    main thread is inside input(), and is stopped before the next command."""

    def __init__(self, ser):
        self.ser = ser
        self._stop = threading.Event()
        self._thread = None

    def __enter__(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.wait(HEARTBEAT_INTERVAL_S):
            try:
                self.ser.write(build_frame(CMD_HEARTBEAT))
                read_frame(self.ser, timeout_s=0.2)  # drain the ACK
            except Exception:
                return

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        return False


def parse_args():
    p = argparse.ArgumentParser(
        description="Step one axis outward to find where it stops moving freely.",
    )
    p.add_argument("port", help="serial port, e.g. COM3")
    p.add_argument("joint", choices=sorted(JOINT_NAME_TO_POS))
    p.add_argument("axis", choices=("pan", "tilt"))
    p.add_argument("start_deg", type=float, help="angle to begin at (use mid-travel)")
    p.add_argument("end_deg", type=float, help="angle to walk toward")
    p.add_argument("step_deg", type=float, nargs="?", default=5.0,
                   help="increment per press, default 5")
    p.add_argument("--yes", action="store_true",
                   help="skip the per-step prompt and pause 1.5 s instead")
    return p.parse_args()


def pulse_us(axis: str, deg: float) -> int:
    """The pulse width the firmware will produce for this angle -- same integer
    arithmetic as pan_angle_to_pulse_us() / tilt_angle_to_pulse_us()."""
    lo_cdeg, hi_cdeg, lo_us, hi_us = PAN_SERVO_CALIB if axis == "pan" else TILT_SERVO_CALIB
    cdeg = round(deg * 100)
    return lo_us + ((cdeg - lo_cdeg) * (hi_us - lo_us)) // (hi_cdeg - lo_cdeg)


def read_angles(ser, position):
    ser.write(build_frame(CMD_GET_STATUS))
    resp = read_frame(ser)
    if resp is None or resp[0] != 0x82:
        print("no STATUS response -- is the board powered and on the right port?")
        sys.exit(1)
    body = resp[1][1:]
    off = position * 4
    pan = int.from_bytes(body[off:off + 2], "little", signed=True) / 100.0
    tilt = int.from_bytes(body[off + 2:off + 4], "little", signed=True) / 100.0
    return pan, tilt


def steps(start, end, step):
    """Angles from start toward end inclusive, always stepping the right way."""
    step = abs(step) * (1 if end >= start else -1)
    n = int(abs(end - start) / abs(step))
    out = [start + i * step for i in range(n + 1)]
    if out[-1] != end:
        out.append(end)
    return out


def main():
    args = parse_args()
    position = JOINT_NAME_TO_POS[args.joint]
    lo, hi = axis_limit(position, args.axis)

    print(f"--- sweeping {args.joint} {args.axis}: "
          f"{args.start_deg:g} -> {args.end_deg:g} deg in {args.step_deg:g} deg steps ---")
    print(f"    window per this checkout's JOINT_LIMITS_DEG: {lo:g} .. {hi:g} deg")
    print("    (a local mirror of kJointLimits[] -- the BOARD is the authority,")
    print("     and only agrees if the firmware on it was built from this checkout)")
    outside = [a for a in (args.start_deg, args.end_deg) if not (lo <= a <= hi)]
    if outside:
        print(f"    NOTE: {', '.join(f'{a:g}' for a in outside)} lies outside that window.")
        print("    The firmware will NACK those steps. Widen this row in kJointLimits[]")
        print("    (Core/Src/servo_joint.c) and reflash to measure past it.")
    if abs(args.step_deg) >= abs(args.end_deg - args.start_deg):
        print(f"    WARNING: a {args.step_deg:g} deg step spans the whole sweep, so the")
        print("    very first command jumps straight to the far end at full slew. Use a")
        print("    small step (5 deg) from mid-travel so you approach a stop gradually.")
    print("    Ctrl-C at the first sign of buzzing or straining.\n")

    ser = open_serial(args.port)
    last_good = None
    try:
        arm(ser)
        pan_deg, tilt_deg = read_angles(ser, position)
        print(f"    starting from: pan={pan_deg:g} tilt={tilt_deg:g}\n")

        for angle in steps(args.start_deg, args.end_deg, args.step_deg):
            if args.axis == "pan":
                pan_deg = angle
            else:
                tilt_deg = angle

            ser.write(build_frame(CMD_MOVE_JOINT_TO,
                                  move_joint_payload(position, pan_deg, tilt_deg)))
            resp = read_frame(ser)

            tag = f"{args.axis} {angle:7.2f} deg -> {pulse_us(args.axis, angle):4d} us"
            if resp is None:
                print(f"  {tag}   no response")
                break
            if resp[0] == CMD_NACK:
                code = resp[1][1] if len(resp[1]) > 1 else None
                print(f"  {tag}   NACK {NACK_REASONS.get(code, hex(code) if code else '?')}"
                      f" -- stopping")
                if code == NACK_OUT_OF_RANGE:
                    if lo <= angle <= hi:
                        # This checkout says the angle is legal and the board
                        # disagrees, so the two are out of sync. Nearly always
                        # means kJointLimits[] was edited but not reflashed.
                        print("      This checkout says that angle IS legal, so the board")
                        print("      disagrees with it -- the firmware running on the board")
                        print("      predates your latest kJointLimits[] edit.")
                        print("      Rebuild, flash, then re-run this sweep.")
                    else:
                        print("      That is the configured window clamping you, not a")
                        print("      mechanical limit. Widen this row in kJointLimits[],")
                        print("      then REBUILD AND FLASH before re-running.")
                elif code == NACK_ESTOPPED:
                    print("      The heartbeat watchdog disarmed the board mid-sweep.")
                    print("      This should not happen -- please report it.")
                break
            if resp[0] != CMD_ACK:
                print(f"  {tag}   unexpected reply -- stopping")
                break

            print(f"  {tag}   ok")
            last_good = angle

            if args.yes:
                hold_alive(ser, 1.5)
            else:
                hold_alive(ser, 0.4)
                try:
                    # Heartbeats must keep flowing while this blocks, or the
                    # watchdog e-stops the board before the next step.
                    with Keepalive(ser):
                        input("      [Enter] next step, Ctrl-C to stop here: ")
                except (KeyboardInterrupt, EOFError):
                    print()
                    break
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        ser.close()

    print("\n" + "=" * 60)
    if last_good is None:
        print("  nothing moved -- no usable limit measured")
    else:
        print(f"  last angle that moved cleanly: {last_good:g} deg "
              f"({pulse_us(args.axis, last_good)} us)")
        print(f"  if it was still quiet there, that is this bracket's {args.axis} limit")
        print(f"  -> write {round(last_good * 100)} into the {args.joint} row of kJointLimits[]")
    print("=" * 60)


if __name__ == "__main__":
    main()
