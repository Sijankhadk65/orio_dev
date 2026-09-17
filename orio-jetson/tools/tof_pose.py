#!/usr/bin/env python3
"""Solve each ToF array's mount pose from the floor it can already see.

    uv run python tools/tof_pose.py                 # both sensors, floor only
    uv run python tools/tof_pose.py --bus 7         # one sensor
    uv run python tools/tof_pose.py --wall          # squared to a wall: adds yaw
    uv run python tools/tof_pose.py --selftest      # no hardware; checks the maths

Phase 3 step 3 of docs/avoidance-plan.md asks for each sensor's real mount
height, pitch and yaw, on the grounds that a sensor pointing somewhere else
measures somewhere else while reporting the same numbers. This is the way to
get them without a protractor on a bracket that is already bolted down: the
sensor is looking at the floor, the floor is a plane, and a plane seen from a
known set of angles determines the height and pitch of whatever is looking at
it.

That is the same trick Phase 0 uses on the cameras — two known off-axis angles
and two floor distances — run over all 64 zones at once and fitted by RANSAC
rather than by reading two numbers off a screen, because the zones disagree and
the disagreement is the error bar.

## What it can and cannot tell you

* **Height and pitch: yes**, from the floor alone, and they are the two numbers
  the height classification actually consumes.
* **Roll: as a warning.** It should be zero. If it is not, the bracket is
  twisted and every zone's elevation is wrong by a different amount.
* **Yaw: only with `--wall`**, and only because a floor plane is the same plane
  at every yaw — there is nothing in it to measure a rotation about the
  vertical against. Square the robot to a flat wall, run `--wall`, and the wall
  supplies what the floor cannot. Eyeballing the squaring is the dominant error
  here, so treat the result as a check on the intended splay, not a revelation.

## If the sensors look forward rather than down

A sensor bolted to the front of the robot looking forward never sees a floor,
and everything above that depends on one is unavailable. `--wall` is then the
whole tool rather than an extra: squared to a flat wall, the wall's normal
gives **pitch and yaw together**, each from a different component of it, with
no floor involved.

Height it cannot give — a vertical plane looks the same from every height — and
does not need to: the height of a forward-looking bracket is a tape measure
away and better measured that way. Note also that the fan was planned low and
level precisely to cover what the cameras cannot, so a forward-looking pair is
a different sensor answering a different question. Worth settling before the
poses are written down.
* **TOF_FLIP_H: no.** A floor is symmetric left to right and so is a wall you
  are square to. That one still wants a hand in front of `tools/tof_debug.py`.
  A wrong TOF_FLIP_V, on the other hand, this will catch: the floor lands in
  the rows that should be looking up and the fit comes back with the plane on
  the wrong side, which is reported rather than quietly negated.

## Frames

Robot frame is x right, y up, z forward, origin under the sensor on the floor.
Pitch is positive DOWN and yaw positive RIGHT, matching `config.py`. In the
sensor's own frame the floor plane works out to

    p . (0, cos pitch, -sin pitch) = -height

which is independent of yaw, as it has to be, and is the whole reason yaw needs
the wall.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from orio import config
from orio.tof import ToFSensor, zone_angles


def ray_directions(az_deg, el_deg) -> np.ndarray:
    """Unit direction per zone in the sensor's own frame, as (N, 3)."""
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.stack([np.cos(el) * np.sin(az), np.sin(el), np.cos(el) * np.cos(az)], axis=1)


# Inlier tolerance, and it is NOT slack for sensor noise — it is the width of
# the ambiguity at the floor-to-wall junction. The bottom row of zones on a wall
# is only centimetres above the floor, so a plane tilted to catch both has MORE
# inliers than the true floor and honest RANSAC picks it: at 20 mm the selftest
# recovers 8 deg of pitch as 10.4 deg and never notices. 10 mm is below the
# junction's width and still far above the residual of a real fit, and it holds
# up in the selftest against 10 mm of simulated range noise — four times the
# noise these parts actually show. Widen it only with the selftest running.
PLANE_TOL_M = 0.010


def fit_plane_ransac(pts: np.ndarray, tol: float = PLANE_TOL_M, iters: int = 800,
                     accept=None, rng=None):
    """Largest plane in `pts` as `(normal, offset, inlier_mask)`, p . n = offset.

    RANSAC rather than least squares because most of what a low sensor sees is
    not the plane being fitted — the floor is one surface among the furniture,
    and a least-squares fit over everything returns a plane through none of it.

    `accept(normal, offset)` filters candidates, which is how the floor is told
    from the wall behind it: both are planes and both are large, and the only
    thing that distinguishes them is which way they face.
    """
    rng = rng or np.random.default_rng(0)
    n_pts = len(pts)
    if n_pts < 12:
        return None, None, None

    best = (0, None, None)
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        a, b, c = pts[idx]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:          # three collinear zones say nothing
            continue
        normal = normal / norm
        offset = float(normal @ a)
        if accept is not None and not accept(normal, offset):
            normal, offset = -normal, -offset       # a plane has two faces
            if not accept(normal, offset):
                continue
        inliers = np.abs(pts @ normal - offset) < tol
        count = int(inliers.sum())
        if count > best[0]:
            best = (count, normal, offset)

    count, normal, offset = best
    if normal is None or count < 12:
        return None, None, None

    # Refit on the inliers: RANSAC picks the plane, three points do not define
    # it well enough to report to two decimal places. Twice, because the first
    # refit moves the plane and the point set that belongs to it moves with it
    # — at the floor-to-wall junction the two surfaces are within `tol` of each
    # other, and the corner's points are what bias a single pass.
    for _ in range(2):
        inliers = np.abs(pts @ normal - offset) < tol
        if inliers.sum() < 6:
            break
        sel = pts[inliers]
        centroid = sel.mean(axis=0)
        _, _, vt = np.linalg.svd(sel - centroid)
        refit = vt[-1] / np.linalg.norm(vt[-1])
        refit_offset = float(refit @ centroid)
        if accept is not None and not accept(refit, refit_offset):
            refit, refit_offset = -refit, -refit_offset
            if not accept(refit, refit_offset):
                break
        normal, offset = refit, refit_offset
    return normal, offset, np.abs(pts @ normal - offset) < tol


# A floor plane's normal in the sensor frame is (0, cos pitch, -sin pitch), so
# its upward component IS cos(pitch): 0.85 admits any mount within about 31 deg
# of level and excludes the merely floor-ish. Loosen it and the fit will happily
# return the ramp made of the floor and the wall it runs into.
FLOOR_MIN_UP = 0.85


def floor_accept(normal, offset) -> bool:
    """A floor faces up, from a sensor mounted a few centimetres above it."""
    height = -offset
    return normal[1] > FLOOR_MIN_UP and 0.005 < height < 0.60


# A wall nearer than this is reported but flagged: see `angle_sigma_deg` for
# why close walls make bad references. Below 10 cm it is not a reference at all,
# it is a sensor pressed against something.
WALL_MIN_M = 0.10
# Under this, the angular error bar is wide enough that the number should not be
# copied into config without backing the robot up first.
WALL_COMFORTABLE_M = 0.50


def wall_accept(normal, offset) -> bool:
    """A wall faces back at the sensor, from somewhere ahead of it."""
    return normal[2] > 0.5 and WALL_MIN_M < offset < config.TOF_MAX_RANGE_M


def angle_sigma_deg(pts, normal, offset) -> float | None:
    """How well this point set actually pins the plane's normal down, in degrees.

    The zones span a fixed 45 deg whatever the range, so it is tempting to think
    distance does not matter. It does, and this is the number that says so: the
    fit is a lever, its arm is how far the points spread ACROSS the plane, and
    range noise at the ends of that arm is what tilts it. The arm grows with
    distance while the noise barely does, so

        angular error ~ (residual scatter) / (in-plane radius)

    and a wall at 0.15 m pins the normal roughly four times worse than the same
    wall at 0.6 m. That is not a rounding detail — it is the difference between
    a pose worth writing into config and one that moves a degree every time it
    is measured.
    """
    if len(pts) < 6:
        return None
    centroid = pts.mean(axis=0)
    delta = pts - centroid
    in_plane = delta - np.outer(delta @ normal, normal)
    radius = float(np.sqrt((in_plane ** 2).sum(axis=1).mean()))
    if radius < 1e-6:
        return None
    scatter = float(np.abs(pts @ normal - offset).std())
    return math.degrees(scatter / radius)


def pose_from_floor(normal, offset):
    """`(height_m, pitch_deg, roll_deg)` from the fitted floor plane."""
    height = -offset
    pitch = math.degrees(math.atan2(-normal[2], normal[1]))
    roll = math.degrees(math.atan2(normal[0], normal[1]))
    return height, pitch, roll


def pose_from_wall(normal):
    """`(pitch_deg, yaw_deg)` from a wall the robot is squared to.

    A wall perpendicular to the robot's forward axis has, in the sensor frame,
    the normal

        (-sin yaw, cos yaw . sin pitch, cos yaw . cos pitch)

    so yaw comes from the x component alone and pitch from the ratio of the
    other two — independent of each other, and neither needing a floor. That is
    the whole reason this path exists: a sensor bolted to the front of the robot
    looking forward never sees a floor to fit, and its pitch and yaw are exactly
    the two numbers the height classification cannot do without.

    What a wall CANNOT give is height: a vertical plane looks identical from
    every height, so there is nothing in it to measure one against. For a
    forward-looking mount that is fine — the height is where the bracket is, and
    a tape measure reads it directly and better.
    """
    yaw = math.degrees(math.asin(max(-1.0, min(1.0, -normal[0]))))
    pitch = math.degrees(math.atan2(normal[1], normal[2]))
    return pitch, yaw


def median_frame(sensor: ToFSensor, frames: int = 30, timeout_s: float = 20.0):
    """Per-zone median over `frames` readings, NaN where the sensor never
    vouched for the zone.

    The median and not the mean: a zone that flickers between the floor and the
    chair leg above it should report one of them, not the midpoint of the two,
    and the midpoint is a range at which there is nothing at all.
    """
    stack = []
    deadline = time.monotonic() + timeout_s
    while len(stack) < frames and time.monotonic() < deadline:
        got = sensor.read()
        if got is not None:
            stack.append(got[0])
        else:
            time.sleep(0.005)
    if not stack:
        return None
    arr = np.array(stack, dtype=np.float32)
    with np.errstate(invalid="ignore"):
        return np.nanmedian(arr, axis=0)


def solve(name: str, ranges, want_wall: bool, tol: float = PLANE_TOL_M) -> dict:
    """Everything this tool can say about one sensor, as a dict."""
    az, el = zone_angles(8, 8, config.TOF_FOV_DEG, config.TOF_FLIP_H, config.TOF_FLIP_V)
    valid = np.isfinite(ranges)
    pts = ray_directions(az, el)[valid] * ranges[valid, None]
    out = {"name": name, "valid": int(valid.sum()), "total": int(valid.size)}

    # The floor and the wall are fitted independently, because a sensor may
    # legitimately see one and not the other. A low level mount sees floor and
    # no wall worth having; one on the front of the robot looking forward sees
    # a wall and no floor at all, and used to get nothing but a shrug here.
    normal, offset, inliers = fit_plane_ransac(pts, tol=tol, accept=floor_accept)
    if normal is None:
        out["floor"] = None
        # Was there a plane at all, facing anywhere? Saying "no floor" is much
        # more useful with "but there is a big plane 2.8 m ahead" attached.
        any_n, any_c, any_in = fit_plane_ransac(pts, tol=tol, accept=None)
        out["any_plane"] = None if any_n is None else (any_n, any_c, int(any_in.sum()))
        rest = pts
    else:
        height, pitch, roll = pose_from_floor(normal, offset)
        residual = float(np.abs(pts[inliers] @ normal - offset).std())
        out["floor"] = {"height_m": height, "pitch_deg": pitch, "roll_deg": roll,
                        "inliers": int(inliers.sum()), "residual_m": residual,
                        "sigma_deg": angle_sigma_deg(pts[inliers], normal, offset)}
        rest = pts[~inliers]

    if want_wall:
        w_n, w_c, w_in = fit_plane_ransac(rest, tol=tol, accept=wall_accept)
        if w_n is None:
            out["wall"] = None
        else:
            w_pitch, w_yaw = pose_from_wall(w_n)
            out["wall"] = {"pitch_deg": w_pitch, "yaw_deg": w_yaw,
                           "distance_m": w_c, "inliers": int(w_in.sum()),
                           "sigma_deg": angle_sigma_deg(rest[w_in], w_n, w_c)}
    return out


def report(res: dict) -> None:
    name = res["name"]
    print(f"\n── {name} ── {res['valid']}/{res['total']} zones vouched for")
    floor = res.get("floor")
    wall = res.get("wall")
    if floor is None and wall:
        sigma = wall.get("sigma_deg")
        pm = "" if sigma is None else f" +/- {sigma:.1f}"
        print(f"  no floor in view — pose taken from the wall {wall['distance_m']:.2f} m "
              f"ahead ({wall['inliers']} zones)")
        print(f"  pitch {wall['pitch_deg']:+.1f}{pm} deg   yaw {wall['yaw_deg']:+.1f}{pm} deg"
              "   height: not measurable from a wall, use a tape measure")
        print("    — only as square as the robot was to the wall.")
        if wall["distance_m"] < WALL_COMFORTABLE_M:
            print(f"  ⚠ that wall is {wall['distance_m'] * 100:.0f} cm away, which is too close to")
            print(f"    measure against: the fit's lever arm is short and the error bar above")
            print(f"    shows it. Back the robot up to {WALL_COMFORTABLE_M:.1f}-1.5 m from a flat")
            print("    wall and run this again before writing anything into config.")
        return
    if floor is None:
        print("  NO FLOOR PLANE IN VIEW.")
        any_plane = res.get("any_plane")
        if any_plane is None:
            print("  No large plane at all — nothing here is a flat surface.")
        else:
            n, c, k = any_plane
            facing = ("up" if n[1] > 0.5 else "down" if n[1] < -0.5 else
                      "forward" if abs(n[2]) > 0.5 else "sideways")
            print(f"  There IS a plane ({k} zones) {c:.2f} m away, facing {facing} "
                  f"— normal ({n[0]:+.2f}, {n[1]:+.2f}, {n[2]:+.2f}).")
        print("  A sensor mounted low and level sees floor in its lower rows. If it")
        print("  does not, check in this order: is the bracket actually level, is")
        print("  ORIO_TOF_FLIP_V the right way round, and is the floor dark enough")
        print("  to return 255 'no target' at grazing incidence (tools/tof_debug.py")
        print("  shows the statuses).")
        return

    sigma = floor.get("sigma_deg")
    pm = "" if sigma is None else f" +/- {sigma:.1f}"
    print(f"  height {floor['height_m'] * 100:.1f} cm   pitch {floor['pitch_deg']:+.1f}{pm} deg"
          f"   roll {floor['roll_deg']:+.1f}{pm} deg")
    print(f"  fitted on {floor['inliers']} floor zones, residual {floor['residual_m'] * 1000:.0f} mm")
    if floor["inliers"] < 20:
        print(f"  ⚠ only {floor['inliers']} zones — a floor filling the lower rows gives many")
        print("    more than that. This is more likely a patch of something flat than a floor;")
        print("    check it in tools/tof_debug.py before believing the height.")
    if abs(floor["roll_deg"]) > 3:
        print(f"  ⚠ roll {floor['roll_deg']:+.1f} deg — the bracket is twisted. Every zone's")
        print("    elevation is then wrong by a different amount; fix the mount, not the config.")
    if floor["residual_m"] > 0.02:
        print(f"  ⚠ residual {floor['residual_m'] * 1000:.0f} mm is high for a flat floor —")
        print("    fitted something that is not one, or the floor is carpet-deep.")
    if wall is not None:
        print(f"  yaw {wall['yaw_deg']:+.1f} deg, from a wall {wall['distance_m']:.2f} m away "
              f"({wall['inliers']} zones)")
        print("    — only as square as the robot was to the wall.")
        disagree = abs(wall["pitch_deg"] - floor["pitch_deg"])
        if disagree > 2.0:
            print(f"  ⚠ the wall makes the pitch {wall['pitch_deg']:+.1f} deg against the "
                  f"floor's {floor['pitch_deg']:+.1f} — {disagree:.1f} deg apart.")
            print("    One of the two surfaces is not what it was assumed to be, or the")
            print("    robot is not square to the wall. Do not average them.")
    elif "wall" in res:
        print("  no wall plane found: square the robot to a flat wall within "
              f"{config.TOF_MAX_RANGE_M:.1f} m, or drop --wall.")


def selftest() -> int:
    """Recover known poses from synthetic zones. No hardware, no floor, no faith."""
    print("tof_pose selftest — recovering known poses from synthetic grids\n")
    az, el = zone_angles(8, 8, config.TOF_FOV_DEG)
    dirs = ray_directions(az, el)
    failures = 0

    # The last case carries 10 mm of range noise — four times what these parts
    # actually show — because a pose solver that only works on clean data is a
    # pose solver that will be believed on a carpet.
    for height, pitch, yaw, noise in [(0.045, 0.0, 0.0, 0.003),
                                      (0.060, 8.0, -22.5, 0.003),
                                      (0.030, -5.0, 22.5, 0.003),
                                      (0.120, 15.0, 10.0, 0.003),
                                      (0.045, 6.0, -22.5, 0.010)]:
        p = math.radians(pitch)
        n_floor = np.array([0.0, math.cos(p), -math.sin(p)])
        denom = dirs @ n_floor
        with np.errstate(divide="ignore", invalid="ignore"):
            r_floor = -height / denom
        r_floor[(denom >= -1e-6) | (r_floor > config.TOF_MAX_RANGE_M)] = np.nan

        w = math.radians(yaw)
        n_wall = np.array([-math.sin(w), math.cos(w) * math.sin(p), math.cos(w) * math.cos(p)])
        dist = 2.0
        denom_w = dirs @ n_wall
        with np.errstate(divide="ignore", invalid="ignore"):
            r_wall = dist / denom_w
        r_wall[(denom_w <= 1e-6) | (r_wall > config.TOF_MAX_RANGE_M)] = np.nan

        # Nearest surface wins, which is what TARGET_ORDER_CLOSEST does.
        ranges = np.fmin(np.nan_to_num(r_floor, nan=np.inf),
                         np.nan_to_num(r_wall, nan=np.inf))
        ranges[np.isinf(ranges)] = np.nan
        rng = np.random.default_rng(1)
        ranges = ranges + rng.normal(0, noise, ranges.shape)

        res = solve("synthetic", ranges, want_wall=True)
        floor = res.get("floor")
        if floor is None:
            print(f"  FAIL  h={height} p={pitch} w={yaw}: no floor plane found")
            failures += 1
            continue
        dh = abs(floor["height_m"] - height)
        dp = abs(floor["pitch_deg"] - pitch)
        dr = abs(floor["roll_deg"])
        wall = res.get("wall")
        dy = None if wall is None else abs(wall["yaw_deg"] - yaw)
        ok = dh < 0.005 and dp < 1.0 and dr < 1.0 and (dy is not None and dy < 1.5)
        failures += 0 if ok else 1
        got_yaw = "none " if wall is None else f"{wall['yaw_deg']:+5.1f}"
        yaw_err = "  -  " if dy is None else f"{dy:.2f}"
        verdict = "ok  " if ok else "FAIL"
        print(f"  {verdict}  h {height * 100:5.1f} -> {floor['height_m'] * 100:5.1f} cm "
              f"({dh * 1000:4.1f} mm)   pitch {pitch:+5.1f} -> {floor['pitch_deg']:+5.1f} "
              f"({dp:.2f} deg)   yaw {yaw:+5.1f} -> {got_yaw} ({yaw_err} deg)")

    # A grid with no floor in it must say so rather than invent one.
    res = solve("wall only", np.full(64, 2.5, dtype=np.float32), want_wall=False)
    no_floor_ok = res.get("floor") is None
    failures += 0 if no_floor_ok else 1
    print(f"  {'ok  ' if no_floor_ok else 'FAIL'}  a grid with no floor in it reports no floor")

    # Forward-looking mount: bolted to the front of the robot, seeing a wall and
    # no floor whatsoever. This is the real mount on Orio, and the floor path
    # has nothing to say about it.
    print("\n  forward-looking mount — a wall, no floor:")
    for pitch, yaw, dist in [(0.0, 0.0, 2.0), (-6.0, -22.5, 2.5),
                             (4.0, 22.5, 1.5), (-12.0, 8.0, 2.8)]:
        pr, wr = math.radians(pitch), math.radians(yaw)
        n_wall = np.array([-math.sin(wr), math.cos(wr) * math.sin(pr),
                           math.cos(wr) * math.cos(pr)])
        denom = dirs @ n_wall
        with np.errstate(divide="ignore", invalid="ignore"):
            ranges = dist / denom
        ranges[(denom <= 1e-6) | (ranges > config.TOF_MAX_RANGE_M)] = np.nan
        ranges = ranges + np.random.default_rng(2).normal(0, 0.005, ranges.shape)

        res = solve("synthetic wall", ranges, want_wall=True)
        wall = res.get("wall")
        if wall is None:
            print(f"    FAIL  pitch={pitch} yaw={yaw}: no wall plane found")
            failures += 1
            continue
        dp, dy = abs(wall["pitch_deg"] - pitch), abs(wall["yaw_deg"] - yaw)
        dd = abs(wall["distance_m"] - dist)
        ok = dp < 1.0 and dy < 1.0 and dd < 0.02 and res.get("floor") is None
        failures += 0 if ok else 1
        print(f"    {'ok  ' if ok else 'FAIL'}  pitch {pitch:+5.1f} -> {wall['pitch_deg']:+5.1f} "
              f"({dp:.2f} deg)   yaw {yaw:+5.1f} -> {wall['yaw_deg']:+5.1f} ({dy:.2f} deg)   "
              f"wall {dist:.2f} -> {wall['distance_m']:.2f} m"
              f"{'' if res.get('floor') is None else '   BUT INVENTED A FLOOR'}")

    print("\nall passed" if not failures else f"\n{failures} FAILED")
    return 0 if not failures else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=None, help="one sensor on this bus")
    ap.add_argument("--address", type=lambda v: int(v, 0), default=0x29)
    ap.add_argument("--frames", type=int, default=30, help="frames to median over")
    ap.add_argument("--tol", type=float, default=PLANE_TOL_M,
                    help="plane inlier tolerance in metres (see PLANE_TOL_M)")
    ap.add_argument("--wall", action="store_true",
                    help="also solve yaw from a wall the robot is squared to")
    ap.add_argument("--selftest", action="store_true", help="check the maths, no hardware")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.bus is not None:
        pairs = [(args.bus, args.address, f"tof-bus{args.bus}")]
    else:
        pairs = list(zip(config.TOF_BUSES, config.TOF_ADDRESSES, config.TOF_NAMES))

    print(f"Opening {len(pairs)} sensor(s) — seconds each, the firmware upload is slow.")
    results = []
    for bus, addr, name in pairs:
        sensor = ToFSensor(bus=bus, address=addr, name=name)
        try:
            sensor.open()
        except Exception as exc:
            print(f"  {name} on /dev/i2c-{bus}: {type(exc).__name__}: {exc}")
            continue
        try:
            ranges = median_frame(sensor, args.frames)
            if ranges is None:
                print(f"  {name}: opened but never delivered a frame")
                continue
            results.append(solve(name, ranges, args.wall, args.tol))
        finally:
            sensor.close()

    if not results:
        print("\nNothing measured.")
        return 1
    for res in results:
        report(res)

    solved = [r for r in results if r.get("floor") or r.get("wall")]
    if solved:
        print("\n── for orio/config.py, once you believe them ──")
        heights = ["%.3f" % r["floor"]["height_m"] if r.get("floor") else "?"
                   for r in solved]
        pitches = ["%.1f" % (r["floor"]["pitch_deg"] if r.get("floor")
                             else r["wall"]["pitch_deg"]) for r in solved]
        yaws = ["%.1f" % r["wall"]["yaw_deg"] if r.get("wall") else "?" for r in solved]
        print("TOF_HEIGHTS_M   = " + ",".join(heights))
        print("TOF_PITCHES_DEG = " + ",".join(pitches))
        print("TOF_YAWS_DEG    = " + ",".join(yaws))
        if "?" in heights:
            print("\n'?' height: a wall is the same wall at every height, so it cannot")
            print("give one. Tape-measure the bracket to the floor and fill it in.")
        if "?" in yaws:
            print("\n'?' yaw: rerun with --wall, squared to a flat wall.")
        print("\nOrder follows ORIO_TOF_BUSES. Measure twice: the pose is what turns a")
        print("range into a height above the floor, and a wrong one is silent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
