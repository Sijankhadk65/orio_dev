# Low-obstacle sensing — seeing what the stereo pair cannot

The stereo pair misses obstacles that are real but short. A box, a shoe, a cable
spool, a door threshold: tall enough to stop the wheels, too low to enter the
band of the frame `orio/stereo.py` actually reads. The robot drives into them at
full cruise duty, because `Avoider` is told the way is clear and has no reason
to doubt it.

This plan closes that gap in two independent steps — one free and in software,
one with the pair of **VL53L5CX** time-of-flight arrays now in hand — and sets
out how to prove each one worked.

> Status: **not started.** Branched off `develop` at `75dc722`. Written on a dev
> box from the code and the committed calibration file; every number below
> marked *(predicted)* is arithmetic, not a measurement, and Phase 0 exists to
> replace them with real ones. Expect some of them to be wrong.

## 1. Why it happens

Two separate blind zones stack, and they have different cures.

**The band crop.** `obstacles()` in `orio/stereo.py:454` reduces the depth map
using only rows `STEREO_BAND_TOP`..`STEREO_BAND_BOTTOM` (0.35..0.71 of frame
height). Everything below row 0.71 is discarded before the sector map is built.
Computed from the committed `models/stereo/calibration.npz` at 320x240
(rectified focal 216.0 px, cy 117.1, hfov 73.1 deg, vfov 58.1 deg):

| edge | angle relative to the optical axis |
|---|---|
| band top (0.35) | 8.7 deg **up** |
| band bottom (0.71) | 13.9 deg **down** |
| bottom of the frame (1.00) | 29.6 deg **down** |

So the cameras deliver 15.7 degrees of view below the band that nothing ever
looks at. That is not a sensor limitation. It is a crop, and it is the larger
half of the problem.

The crop is there for a good reason — the floor is always "close", and without
it the robot believes it is permanently blocked. The fix is not to widen the
band. It is to stop deciding by row.

**The neck tilt interacts with all of this.** The cameras are not fixed: they
ride the motion board's neck at `NECK_TILT_DEG` (40), and `config.py` already
records what that costs, measured on the robot 2026-09-09 —

    tilt 30   no floor in frame at all; a thing on the ground is invisible
    tilt 40   floor plus the wall beyond; the driving pose
    tilt 45   straight ahead 1.06 m — mostly floor, below AVOID_CLEAR_M

Tilt 40 is the middle of a window roughly 35..45 wide, chosen so the robot can
both cruise and see the ground. Every angle in the table above is relative to
the optical axis, which at tilt 40 is pitched down by some amount **nobody has
measured**. That unknown is why Phase 0 comes first: the depression angle of the
band edge, not its angle off-axis, is what decides which obstacles are lost.

Rough shape of it *(predicted)*: if the band's lower edge meets the floor around
2.3 m — consistent with the tilt-40 row above — and the cameras sit near 0.20 m,
then an obstacle at 0.5 m has to be about **16 cm tall** to enter the band at
all. Using the full frame instead would bring that to roughly zero at 0.5 m,
with the floor first entering view near 0.53 m. Both numbers are arithmetic on
an unmeasured camera height. Treat them as the hypothesis Phase 0 tests, not as
findings.

**The near gate.** Separately, `STEREO_MIN_RANGE_M` is 0.25 m and a 60 mm
baseline genuinely cannot triangulate much closer. Nothing in software recovers
the last 25 cm. That is the part the ToF sensors are for.

**Depth noise bounds the software fix.** From the same calibration, one pixel of
disparity error is worth:

| range | range error per 1 px disparity |
|---|---|
| 0.3 m | 0.7 cm |
| 0.5 m | 1.8 cm |
| 1.0 m | 7.3 cm |
| 1.5 m | 16.5 cm |
| 2.0 m | 29.3 cm |

Classifying a 5 cm object by its height above the floor is comfortable at 0.5 m
and meaningless at 1.5 m. That is fine — low obstacles only matter close — but
it means the software fix must **degrade to the current behaviour with range**
rather than pretending to measure heights out to 4 m. Do not skip this.

## 2. The two fixes

**Fix A — classify by height above the ground plane, not by image row.**
Reproject each depth pixel to a 3D point in the robot frame and keep it as an
obstacle if its height above the floor falls between a floor tolerance and the
robot's own height. The floor then eliminates itself by geometry instead of by
cropping, the whole lower frame becomes usable, and a 5 cm box at 0.5 m becomes
a legitimate reading rather than a discarded row. Costs nothing but code.

**Fix B — a low ToF fan for what is below the frame and inside 0.25 m.** Two
VL53L5CX arrays mounted at wheel height, looking forward. They cover exactly the
wedge the cameras cannot reach: below the frame edge, and closer than the stereo
near gate. They also work in the dark and on blank walls, which is where SGBM is
weakest.

Neither fix subsumes the other. A is for the 0.3–1.0 m band the cameras already
image and throw away; B is for the volume no camera pixel ever covers.

## 3. Architecture — fuse at `ObstacleMap`, not at the pixels

`orio/stereo.py` ends at a clean contract, and the module docstring is explicit
that this is the handoff point: `ObstacleMap` carries `Sector(angle_deg,
distance_m, valid_frac)` with `distance_m = None` meaning *unknown, which is not
clear*. `orio/avoid.py` consumes only that.

**Decision: a ToF source emits the same `ObstacleMap`, over the same sector
grid, and a `fuse()` combines them per sector.** `Avoider` is not touched.

Rejected: fusing raw depth and ToF into a shared point cloud before sectoring.
It is the better long-term answer and it is the wrong thing to build now — the
two sensors have different frames, different rates (30 Hz vs 15 Hz), different
latencies and different failure modes, and a bug anywhere in that stack presents
as bad steering with no way to tell which sensor lied. Fusing at the sector
level keeps provenance: every sector can say which sensor produced it.

**Sector grid, v1:** keep the existing 7 sectors across stereo's 73.1 deg.
Project ToF zones into that grid and discard zones beyond ±36.5 deg. This throws
away some of the ToF pair's periphery (they span about 90 deg together) and buys
a change that touches `avoid.py` not at all. Widening the grid to the union FOV
is a follow-up, and a real one — `AVOID_STOP_M`'s comment already notes that the
outermost sector centre at 31.8 deg is what forces the blind back-off.

**Fusion rule:** per sector, `min()` of the known distances; known if either
source is known; unknown only if both are. `min()` is the conservative choice
and conservative is correct for a collision guard. Record which source won, for
the debug view and for the inevitable argument about which sensor is lying.

**Shared code:** Fix A and the ToF projection both turn (range, azimuth,
elevation) into (height above floor, azimuth, ground distance). Write that once
— `points_to_sectors()` — and feed it from both. If it ends up written twice,
the two will drift and only one will get the bug fix.

## 4. Phases

Each phase ends with something demonstrable. Do not start Phase 2 before Phase 1
has numbers, or there will be no way to tell which fix did what.

### Phase 0 — measure the rig and the current blind zone

Nothing here is code. Everything downstream depends on it.

1. **Camera height and true pitch at `NECK_TILT_DEG=40`.** Run a tape measure
   along the floor away from the robot. In `tools/stereo_debug.py`'s rectified
   left pane, read the floor distance at the bottom row of the frame and at the
   band-bottom row. Two rows, two known depression angles off-axis (29.6 and
   13.9 deg), two floor distances — solve for camera height `h` and neck pitch.
   Cross-check `h` against the tape measure directly.
2. **Blind-zone baseline.** Objects of known height — 2, 5, 10, 15, 20 cm — at
   0.25, 0.4, 0.6, 0.8, 1.2, 1.6 m, centred and at ±25 deg. For each, record
   whether the sector map reports it. This table is the deliverable of every
   later phase; without it "better" is an opinion.
3. Confirm or refute the 16 cm-at-0.5 m prediction. If it is badly off, stop and
   find out why before writing code against the model that produced it.

Record all of it in this file, in the style of the tilt table in `config.py` —
numbers, date, method.

### Phase 1 — ground-plane classification in `stereo.py` (no new hardware)

1. Add `STEREO_CAM_HEIGHT_M` and `STEREO_CAM_PITCH_DEG` to `config.py`, from
   Phase 0, with the tilt-dependency written down beside them.
2. Reproject depth to robot-frame points using the rectified intrinsics already
   loaded in `DepthEstimator` (`_focal_px`, and `cy` from `P1` — note
   `obstacles()` currently has access to neither; either pass them in, or move
   the reduction onto the estimator).
3. `points_to_sectors()`: keep points with `STEREO_FLOOR_TOL_M` (start 0.03) <
   height < `ROBOT_HEIGHT_M`. Per sector, the 10th percentile of ground-range
   over the kept points, and the existing `STEREO_MIN_VALID_FRAC` gate. Preserve
   the doctrine: too few points means `None`, never a distance.
4. **Range-dependent trust.** Beyond roughly 1.2 m the height estimate is worth
   less than the noise table above allows; fall back to the row band there. One
   `STEREO_HEIGHT_TRUST_M` knob, and a docstring saying why it exists.
5. Keep the row band available as `ORIO_STEREO_GROUND_PLANE=0`, so a regression
   is one env var away from being confirmed rather than argued about.
6. Extend `tools/stereo_debug.py` to colour points by height class — floor /
   obstacle / above / unknown. Tuning this blind is not worth attempting.
7. Re-run the Phase 0 table. Expect the 0.3–1.0 m column to improve
   substantially and the 1.5 m+ column not to change.

**Watch for:** a false floor. Pitch error of a degree or two tilts the fitted
plane enough that distant floor reads as an obstacle and the robot never
cruises — exactly the tilt-45 failure already recorded in `config.py`. If
hand-measured pitch proves too fragile, fit the plane by RANSAC over the lower
frame on flat ground and keep the measured value as the prior. Decide with data,
not upfront.

### Phase 2 — VL53L5CX bring-up

One sensor first, on the bench, before anything is mounted.

1. **Wire one.** 3.3 V (40-pin pin 1), GND, SDA/SCL on pins 3/5 — that is
   `/dev/i2c-7` on the Orin Nano; pins 27/28 are `/dev/i2c-1`. Confirm with
   `i2cdetect -y -r 7` and expect `0x29`. Add the user to the `i2c` group. Keep
   the runs under about 20 cm.
2. **Library.** Pimoroni's `vl53l5cx-ctypes` on PyPI wraps ST's ULD driver and
   is the least work. It will likely build from source on aarch64 — fine,
   JetPack has the toolchain. If it fights back, the fallback is ST's ULD C
   driver plus a small `ctypes` shim; the API is small and the vendor sample is
   close to what we need. **Verify this before mounting anything.**
3. **Firmware upload.** The part has no flash: roughly 84 KB is pushed over I2C
   at every init. About 2 s at 400 kHz, nearer 8 s at the 100 kHz default. Raise
   the bus clock if it is easy; otherwise budget the delay in startup and do not
   mistake it for a hang.
4. **Second sensor, address conflict.** Both ship as 0x29. `LPn` gates a
   sensor's I2C interface, so: hold both `LPn` low, raise A, move A to 0x2A with
   `vl53l5cx_set_i2c_address()`, then raise B and leave it at 0x29. **The new
   address is volatile** — lost on every power cycle, and must be redone at every
   start. Needs two Jetson GPIOs. A TCA9548A mux is the alternative: more parts,
   no GPIO sequencing, both sensors stay 0x29. Prefer LPn; fall back to the mux
   if the sequencing proves flaky.
5. **Bus health.** Jetson's 40-pin I2C carries its own pull-ups and most
   breakouts add more. Two breakouts plus the board can over-pull the bus. If
   edges look bad or transfers fail intermittently, remove the breakouts'
   pull-ups before suspecting anything else.
6. Sanity: 8x8 at 15 Hz, print the grid, wave a hand. Check `target_status` per
   zone — 5 is a trusted range, 6 and 9 are usable with caveats, everything else
   is unknown. That maps straight onto the existing "unknown is not clear" rule;
   do not flatten it to a distance.

### Phase 3 — mounting and `orio/tof.py`

1. **Mount low and level**, 3–6 cm above the floor, one sensor splayed left and
   one right by about 22.5 deg so their 45 deg squares abut into ~90 deg. Level,
   not pitched up: the array is square, so a level mount sees floor in its lower
   rows, and the answer to that is the same height classification as Phase 1 —
   not a mechanical dodge that also throws away the lowest obstacles.
   *The mount is fixed to the chassis, not the neck. That is a feature: unlike
   the cameras, the ToF fan does not move when the head looks around.*
2. `orio/tof.py`, mirroring `stereo.py`'s shape — lazy open, a lock, explicit
   `close()`, perception only, no motion. Each zone becomes (range, azimuth,
   elevation) from its row/column, then a robot-frame point, then through the
   *same* `points_to_sectors()`.
3. Measure and record each sensor's real mount height, yaw and pitch. Same
   discipline as Phase 0: a sensor pointing somewhere else measures somewhere
   else while reporting the same numbers.
4. `tools/tof_debug.py`, the counterpart to `stereo_debug.py`: the 8x8 grids,
   the derived sector map, and the height classification.

### Phase 4 — fusion and integration

1. `fuse(stereo_map, tof_map) -> ObstacleMap`, per section 3, with provenance
   per sector.
2. `orio/avoid.py`'s `Sensor` gains an optional ToF source. **A ToF failure must
   degrade to stereo-only, not halt** — it is an addition to a working guard, and
   a new sensor that can stop the robot is a new way for the robot to be stopped.
   A stereo failure keeps halting exactly as it does today.
3. Staleness per source, against the existing `AVOID_STALE_S` (0.5 s). ToF at
   15 Hz has 67 ms between frames and will not trip it; say so in a comment so
   nobody later "fixes" the threshold.
4. `tools/teleop_guarded.py`: a `--no-tof` flag, and show the fused sectors with
   their source in the status line.

### Phase 5 — prove it

1. Re-run the Phase 0 table for all four configurations: baseline, Phase 1 only,
   ToF only, fused. One table, four columns, in this file.
2. Drive the corridor test from the `Avoider` docstring with low obstacles
   added — the 5 and 10 cm rows that used to be invisible.
3. Confirm no regression in open-room cruising. The specific risk is the floor
   reading as an obstacle and the robot never leaving `steer` for `cruise`.
4. Update `README.md`'s env-var table and the `stereo.py` module docstring —
   which currently says the band exists to exclude the floor, and will be wrong.

## 5. Deferred, deliberately

**Bumper microswitches.** Not yet in hand. They remain the right backstop: every
ranging sensor has a failure mode and contact has none. Leave the seam for them
in `fuse()`.

**VESC stall detection, as the stand-in.** `WheelTelemetry` in
`orio/drivetrain.py` already decodes `erpm`, `current_a` and `fault_code` from
`COMM_GET_VALUES`. Commanded duty with near-zero erpm and rising current means
the robot is pushing against something. No new hardware, one link message, and
it catches the case every forward-looking sensor misses. Worth doing early if
Phase 2 stalls on parts.

**IMU.** Does not detect obstacles and does not belong in this plan. It belongs
to the *next* one: the `Avoider` docstring's admission that it "wanders rather
than travels" is a missing heading reference, and a gyro is what fixes it.

**2D LiDAR.** The single biggest capability jump — it would also end "there is no
rear or side sensing… the back-off reverses blind" — and out of scope here. Note
it scans one plane, so it complements stereo rather than replacing it: it would
miss a tabletop overhang the cameras see easily.

## 6. What will probably go wrong

- **Camera pitch is not accurately known and the fitted floor is wrong.** The
  most likely single failure, and its signature is distinctive: the robot
  refuses to cruise in an empty room. RANSAC is the escape hatch.
- **The ToF library does not build cleanly on JetPack.** Budget a day. Find out
  in Phase 2, on the bench, before anything is glued to the chassis.
- **The volatile I2C address bites after a brown-out.** The sensors come back at
  0x29 and the second one is simply missing. Re-run the LPn sequence on every
  open, and fail loudly rather than silently ranging with one sensor.
- **The two sensors disagree at the overlap.** `min()` means the pessimist wins,
  so a single bad ToF zone stops the robot. Apply the same robust low percentile
  across zones that `obstacles()` already applies across pixels — one hot zone
  must not be a wall.
- **Bright sunlight** shortens ToF range sharply. Indoors this is fine; worth
  knowing before someone tests on a patio and concludes the parts are bad.

## 7. Getting started on the Jetson

```bash
git clone git@github.com:Sijankhadk65/orio_dev.git
cd orio_dev && git checkout feature/low-obstacle-sensing
cd orio-jetson && uv sync
```

Phase 0 needs no new dependencies — `tools/stereo_debug.py`, a tape measure and
some boxes of known height. Stop the main app first: stereo needs both sensors
and nothing else may hold a camera.
