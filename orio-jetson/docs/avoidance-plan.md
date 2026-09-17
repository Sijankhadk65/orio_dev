# Extending avoidance — seeing the obstacles the stereo pair cannot

The robot drives into things that are real but short. A box, a shoe, a cable
spool, a door threshold: tall enough to stop the wheels, too low to enter the
band of the frame `orio/stereo.py` actually reads. `Avoider` is told the way is
clear and has no reason to doubt it, so it cruises at full duty into something
it never saw.

This plan closes that gap. It has three parts, and they are independent enough
that each can be proven on its own: **aim the head where the policy thinks it is
aimed**, **classify by height above the floor instead of by image row**, and
**add the pair of VL53L5CX time-of-flight arrays** for the volume no camera
pixel covers at all.

> Status: **not started.** Branched off `develop` at `7577851`. Written from the
> code and the committed calibration; every figure marked *(predicted)* is
> arithmetic rather than a measurement, and Phase 0 exists to replace those with
> real numbers. Expect some of them to be wrong.

## 1. Three causes, not one

### 1a. The head may not be aimed where the thresholds assume

Read this before anything else, because it can invalidate the rest.

`config.py` sets `NECK_TILT_DEG = 30` and, in the same comment block, records a
sweep from 2026-09-09 saying exactly this about tilt 30:

    tilt 30   straight ahead 2.87 m   the UPPER WALL and ceiling. No floor in
                                      frame at all, so an obstacle standing on
                                      the ground is not merely far, it is
                                      invisible.

The default was moved to 30 the following day because the neck bracket sits on
an **inclined** body rather than a square one, so the servo's own scale does not
read level where the sweep assumed. The firmware window was narrowed to match
(tilt 20..40). The config is explicit that **the sweep has not been re-run
against the incline**, and warns that until it is, every `AVOID_*` threshold
describes ground the camera may no longer be looking at.

So one live hypothesis is that a good share of the current blindness is aim, not
optics. If the cameras really are pointed at the upper wall, no amount of
ground-plane classification will help — there are no floor pixels to classify —
and bolting ToF sensors on would paper over a head that is looking at the
ceiling. **Phase 0 resolves this first.** It may turn out to be cheap.

### 1b. The band crop throws away view the cameras already deliver

`obstacles()` in `orio/stereo.py:454` reduces the depth map using only rows
`STEREO_BAND_TOP`..`STEREO_BAND_BOTTOM` (0.35..0.71 of frame height). Everything
below row 0.71 is discarded before the sector map is built. Computed from the
committed `models/stereo/calibration.npz` at 320x240 — rectified focal 216.0 px,
cy 117.1, hfov 73.1 deg, vfov 58.1 deg:

| edge | angle relative to the optical axis |
|---|---|
| band top (0.35) | 8.7 deg **up** |
| band bottom (0.71) | 13.9 deg **down** |
| bottom of the frame (1.00) | 29.6 deg **down** |

**Fifteen point seven degrees of delivered view that nothing ever looks at.**
That is not a sensor limitation, it is a crop, and it is free to recover.

The crop exists for a good reason — the floor is always "close", and without it
the robot believes it is permanently blocked. The fix is not to widen the band.
It is to stop deciding by row.

Note these are angles off the **optical axis**, and the optical axis is not
horizontal: it rides the neck, per 1a. The depression angle of the band edge is
what decides which obstacles are lost, and that depends on a neck pitch nobody
has measured.

### 1c. The near gate is real and nothing in software fixes it

`STEREO_MIN_RANGE_M` is 0.25 m and a 60 mm baseline genuinely cannot triangulate
much closer. That last 25 cm is what the ToF sensors are for.

### Depth noise bounds what 1b can achieve

From the same calibration, one pixel of disparity error is worth:

| range | range error per 1 px disparity |
|---|---|
| 0.3 m | 0.7 cm |
| 0.5 m | 1.8 cm |
| 1.0 m | 7.3 cm |
| 1.5 m | 16.5 cm |
| 2.0 m | 29.3 cm |

Classifying a 5 cm object by its height above the floor is comfortable at 0.5 m
and meaningless at 1.5 m. That is fine — low obstacles only matter close — but
it means the height test must **degrade back to the row band with range** rather
than pretending to measure heights out to 4 m. Do not skip this.

## 2. The three fixes

**Fix 0 — re-aim the head.** Re-run the tilt sweep against the inclined bracket
and set `NECK_TILT_DEG` to what actually sees the ground. Costs an afternoon and
possibly fixes a large fraction of the problem. Everything else is tuned through
the head, so this goes first whatever else happens.

**Fix A — classify by height above the ground plane, not by image row.**
Reproject each depth pixel into the robot frame and keep it as an obstacle if
its height above the floor is between a floor tolerance and the robot's own
height. The floor then eliminates itself by geometry instead of by cropping, the
whole lower frame becomes usable, and a 5 cm box at 0.5 m becomes a legitimate
reading rather than a discarded row. Costs nothing but code.

**Fix B — a low ToF fan.** Two VL53L5CX arrays at wheel height, looking forward,
covering what the cameras cannot reach: below the frame edge, and closer than
the 0.25 m stereo near gate. They also work in the dark and against blank walls,
which is exactly where SGBM is weakest.

None of the three subsumes another. Fix 0 is aim, A is the 0.3–1.0 m band the
cameras already image and throw away, B is the volume no pixel covers.

## 3. Architecture

### Fuse at `ObstacleMap`, not at the pixels

`orio/stereo.py` ends at a clean contract and its docstring says so explicitly:
`ObstacleMap` carries `Sector(angle_deg, distance_m, valid_frac)`, with
`distance_m = None` meaning *unknown, which is not the same as clear*.
`orio/avoid.py` consumes only that.

**Decision: a ToF source emits the same `ObstacleMap`, over the same sector
grid, and a `fuse()` combines them per sector.** `Avoider` is not touched.

Rejected: fusing raw depth and ToF into a shared point cloud before sectoring.
It is the better long-term answer and the wrong thing to build now — the two
sensors have different frames, rates (30 Hz vs 15 Hz), latencies and failure
modes, and a bug anywhere in that stack presents as bad steering with no way to
tell which sensor lied. Sector-level fusion keeps provenance: every sector can
say which sensor produced it.

**Sector grid, v1:** keep the existing 7 sectors across stereo's 73.1 deg.
Project ToF zones into that grid and discard zones beyond ±36.5 deg. This throws
away some of the pair's periphery (together they span about 90 deg) and buys a
change that touches `avoid.py` not at all. Widening to the union FOV is a real
follow-up — `AVOID_STOP_M`'s comment already notes that the outermost sector
centre at 31.8 deg is what forces the blind back-off.

**Fusion rule:** per sector, `min()` of the known distances; known if either
source is known; unknown only if both are. `min()` is the conservative choice
and conservative is correct for a collision guard. Record which source won, for
the debug view and for the inevitable argument about which sensor is lying.

**Shared code:** Fix A and the ToF projection both turn (range, azimuth,
elevation) into (height above floor, azimuth, ground distance). Write it once —
`points_to_sectors()` — and feed it from both. Written twice, the two drift and
only one gets the bug fix.

### Where the sensors connect: the Jetson, not either STM32

Asked and settled 2026-09-16. **The pair hangs off the Jetson's 40-pin I2C.**
Neither microcontroller gets them, and the reasoning is worth keeping because
the question will come back.

| | drivetrain | motion |
|---|---|---|
| MCU | STM32L152RET6 | STM32C031C6T3 |
| Flash | 512 KB | 32 KB |
| RAM | 80 KB | 12 KB |
| in use | USART1/2/3 (both VESCs + VCP) | TIM1/3/14/16 — six servos, fan, ARGB, DMA |
| free I2C | PB6/PB7 (I2C1) | none spare |

**The motion board cannot host it at all.** The part has no flash of its own, so
ST's ULD driver carries an ~84 KB firmware blob as `const` data and pushes it
over I2C at every power-on. 84 KB does not fit in 32 KB, and ST's documented
~16 KB RAM for 8x8 mode does not fit in 12 KB. Two independent failures, each
off by roughly 3x. (Both figures are ST's, from the ULD documentation, and have
not been verified against a build here — they do not need to be at this margin.)

**The drivetrain board has the silicon and still should not have it.** The
blocker is the wire:

* `PROTO_MAX_PAYLOAD` is **24 bytes** (`orio-stm-drivetrain/Core/Inc/protocol.h`).
  One 8x8 frame is 64 zones x (int16 distance + uint8 status) = 192 bytes per
  sensor, 384 for the pair. Fragmenting is ~17 frames per sensor-reading, about
  62% of the 115200 baud link at 15 Hz — on the same line as the 5 Hz e-stop
  heartbeat and the ~33 Hz drive commands.
* Raising the cap is a wire-format change, and `protocol.h` is explicit that
  `PROTO_VERSION` is deliberately the same value on **both** boards. It would
  move the motion firmware and `drivetrain.py` with it.
* Reducing to a sector map on the MCU does fit — 7 sectors x (uint16 + uint8) =
  21 bytes — but puts the geometry and height classification in C on the
  microcontroller: a second copy of `points_to_sectors()`, which the paragraph
  above rules out.
* The ULD's 84 KB upload is a blocking transfer of seconds. On the board that
  owns the e-stop and the watchdog, that is a stall to sequence around arming
  rather than a detail.

On the Jetson none of this exists: no firmware, no opcodes, no version bump, and
the ToF points arrive in the same process as the depth map, which is where the
fusion happens anyway.

**The one surviving argument** is e-stop locality — a guard on the MCU that
commands the wheels could cut the motors without a round trip through Linux, the
GIL and `AVOID_STALE_S`. That is a real safety property, and a *different
feature*: it does not need the ranging sensors to live there. Section 6.

### Wiring: one sensor per I2C bus

The 40-pin header exposes **two** I2C buses. One sensor on each and the shared
0x29 address stops being a problem at all: no LPn sequencing, no GPIO, no mux,
and the volatile-address failure mode disappears.

| | Sensor A (left) | Sensor B (right) |
|---|---|---|
| VIN / 3V3 | pin **1** (3.3 V) | pin **17** (3.3 V) |
| GND | pin **6** | pin **25** |
| SDA | pin **3** | pin **27** |
| SCL | pin **5** | pin **28** |
| bus | `/dev/i2c-7` | `/dev/i2c-1` |
| address | 0x29 | 0x29 |

Four wires each. `LPn`, `PWREN` and `I2C_RST` stay at their breakout defaults
(normally pulled to "enabled"); if the breakout needs `LPn` high to run, tie it
to 3.3 V. `INT`/`GPIO1` is optional — it signals data-ready, but polling at
15 Hz is fine and saves two GPIOs. Two independent buses also means the two
84 KB firmware uploads can run in parallel threads rather than back to back.

**Verify the bus numbers before wiring.** Numbering varies by Jetson model and
JetPack version; the table assumes an Orin Nano. `i2cdetect -l` is the authority:

```bash
sudo usermod -aG i2c $USER    # then log out and back in
i2cdetect -l                  # which bus is actually which
i2cdetect -y -r 7             # expect 0x29
i2cdetect -y -r 1             # expect 0x29 — and see what else already lives here
```

Bus 1 may already carry carrier-board devices. If it is occupied or unusable,
fall back to both sensors on bus 7 with LPn gating: hold both `LPn` low, raise
A, move it to 0x2A with `vl53l5cx_set_i2c_address()`, then raise B at 0x29. That
address is **volatile** — lost on every power cycle and re-applied at every
open. A TCA9548A mux is the second fallback.

**Voltage.** The VL53L5CX is a 3.3 V part and the 40-pin I2C is 3.3 V logic.
Never put 5 V on SDA/SCL. Power from pin 1/17, not 2/4, unless the breakout has
its own regulator *and* level shifting — and 3.3 V is still the safe choice.

**Cable length.** Keep runs under ~30 cm at 400 kHz. This one is pointed: the
sensors mount at wheel height on the front chassis while the Jetson sits in the
body. If that run is long, either drop to 100 kHz (pushing each firmware upload
to ~8 s) or add a P82B715/PCA9600 bus extender. Twisted pair with a ground
return helps. **Measure the actual distance before soldering.**

**Current.** The VCSEL fires in bursts; keep decoupling close to each sensor
(most breakouts have it). On brownouts or bus errors under ranging, feed the
sensors from 5 V through their own 3.3 V LDO and keep only SDA/SCL/GND common
with the Jetson.

## 4. Phases

Each phase ends in something demonstrable. Do not start a phase before the
previous one has numbers, or there will be no way to tell which fix did what.

### Phase 0 — measure the rig (no code)

Everything downstream depends on this, and the first item may shrink the rest of
the plan considerably.

1. **Re-run the tilt sweep against the inclined bracket.** Sweep the joint
   across the firmware window (tilt 20..40) and read the sector map and the
   rectified frame at each stop, exactly as the 2026-09-09 sweep did. Find the
   tilt that puts the floor ahead in frame with the wall beyond it. Update
   `NECK_TILT_DEG` and **replace the stale sweep table in `config.py`** — the
   comment there is currently a warning about itself. If the window will not
   reach a usable aim, that is a mechanical finding and it changes the plan.
2. **Camera height and true pitch at the corrected tilt.** Run a tape measure
   along the floor away from the robot. In `tools/stereo_debug.py`'s rectified
   left pane, read the floor distance at the bottom row of the frame and at the
   band-bottom row. Two rows, two known off-axis angles (29.6 and 13.9 deg), two
   floor distances — solve for camera height `h` and pitch. Cross-check `h` with
   the tape directly.
3. **Blind-zone baseline.** Objects of known height — 2, 5, 10, 15, 20 cm — at
   0.25, 0.4, 0.6, 0.8, 1.2, 1.6 m, centred and at ±25 deg. Record whether the
   sector map reports each one. This table is the deliverable of every later
   phase; without it "better" is an opinion.
4. **Re-check every `AVOID_*` threshold** against the corrected aim. They were
   measured through a head pointing somewhere else.

Record all of it in this file, in the style of the tilt table in `config.py` —
numbers, date, method.

### Phase 1 — ground-plane classification in `stereo.py` (no new hardware)

1. Add `STEREO_CAM_HEIGHT_M` and `STEREO_CAM_PITCH_DEG` to `config.py` from
   Phase 0, with the neck-tilt dependency written down beside them.
2. Reproject depth into robot-frame points using the rectified intrinsics
   already loaded in `DepthEstimator` (`_focal_px`, and `cy` from `P1` — note
   `obstacles()` currently has access to neither, so either pass them in or move
   the reduction onto the estimator).
3. `points_to_sectors()`: keep points with `STEREO_FLOOR_TOL_M` (start 0.03) <
   height < `ROBOT_HEIGHT_M`. Per sector, the 10th percentile of ground-range
   over the kept points, and the existing `STEREO_MIN_VALID_FRAC` gate. Preserve
   the doctrine: too few points means `None`, never a distance.
4. **Range-dependent trust.** Beyond roughly 1.2 m the height estimate is worth
   less than the noise table allows; fall back to the row band there. One
   `STEREO_HEIGHT_TRUST_M` knob, with a docstring saying why it exists.
5. Keep the row band reachable as `ORIO_STEREO_GROUND_PLANE=0`, so a regression
   is one env var away from being confirmed rather than argued about.
6. Extend `tools/stereo_debug.py` to colour points by class — floor / obstacle /
   above / unknown. Tuning this blind is not worth attempting.
7. Re-run the Phase 0 table. Expect 0.3–1.0 m to improve substantially and
   1.5 m+ not to change.

**Watch for a false floor.** A degree or two of pitch error tilts the fitted
plane enough that distant floor reads as an obstacle and the robot never
cruises — precisely the tilt-45 failure already recorded in `config.py`. Its
signature is distinctive: the robot refuses to leave `steer` for `cruise` in an
empty room. If hand-measured pitch proves too fragile, fit the plane by RANSAC
over the lower frame on flat ground and keep the measured value as the prior.
Decide with data, not upfront.

### Phase 2 — VL53L5CX bring-up

One sensor, on the bench, before anything is mounted.

1. **Wire one** per section 3. Confirm with `i2cdetect` and expect 0x29.
2. **Library.** Pimoroni's `vl53l5cx-ctypes` on PyPI wraps ST's ULD and is the
   least work. It will likely build from source on aarch64 — fine, JetPack has
   the toolchain. If it fights back, the fallback is ST's ULD C driver plus a
   small `ctypes` shim; the API is small and the vendor sample is close to what
   we need. **Verify this before mounting anything** — it is the single item
   most likely to cost a day.
3. **Firmware upload.** ~84 KB pushed over I2C at every init: about 2 s at
   400 kHz, nearer 8 s at the 100 kHz default. Raise the bus clock if it is
   easy; otherwise budget the delay at startup and do not mistake it for a hang.
4. **Second sensor** on the second bus. Confirm both range independently.
5. **Bus health.** The 40-pin I2C carries its own pull-ups and most breakouts
   add more. If edges look bad or transfers fail intermittently, remove the
   breakouts' pull-ups before suspecting anything else.
6. Sanity: 8x8 at 15 Hz, print the grid, wave a hand. Check `target_status` per
   zone — 5 is a trusted range, 6 and 9 are usable with caveats, everything else
   is unknown. That maps straight onto the existing "unknown is not clear" rule.
   Do not flatten it to a distance.

### Phase 3 — mounting and `orio/tof.py`

1. **Mount low and level**, 3–6 cm above the floor, one splayed left and one
   right by about 22.5 deg so their 45 deg squares abut into ~90 deg. Level, not
   pitched up: the array is square, so a level mount sees floor in its lower
   rows, and the answer is the same height classification as Phase 1 — not a
   mechanical dodge that also throws away the lowest obstacles.
   *The mount is fixed to the chassis, not the neck. That is a feature: unlike
   the cameras, the ToF fan does not move when the head looks around, and it is
   immune to the whole of section 1a.*
2. `orio/tof.py`, mirroring `stereo.py`'s shape — lazy open, a lock, explicit
   `close()`, perception only, no motion. Each zone becomes (range, azimuth,
   elevation) from its row and column, then a robot-frame point, then through
   the *same* `points_to_sectors()`.
3. Measure and record each sensor's real mount height, yaw and pitch. Same
   discipline as Phase 0: a sensor pointing somewhere else measures somewhere
   else while reporting the same numbers.
4. `tools/tof_debug.py`, the counterpart to `stereo_debug.py`: the 8x8 grids,
   the derived sector map, and the height classification.

### Phase 4 — fusion and integration

1. `fuse(stereo_map, tof_map) -> ObstacleMap` per section 3, with provenance.
2. `orio/avoid.py`'s `Sensor` (line 78, which constructs `ObstacleDetector()`
   directly) gains an optional ToF source. **A ToF failure must degrade to
   stereo-only, not halt** — it is an addition to a working guard, and a new
   sensor that can stop the robot is a new way for the robot to be stopped. A
   stereo failure keeps halting exactly as it does today.
3. `orio/body.py:263` constructs the `Sensor`; thread the option through without
   making ToF mandatory for `Body` to come up.
4. Staleness per source against `AVOID_STALE_S` (0.5 s). ToF at 15 Hz has 67 ms
   between frames and will not trip it — say so in a comment, so nobody later
   "fixes" the threshold.
5. `tools/teleop_guarded.py`: a `--no-tof` flag, and the fused sectors with their
   source in the status line.
6. **Extend `tools/test_cruise.py`.** It already has a `FakeSensor` (line 110)
   and runs with no hardware, cameras or serial ports. `fuse()` is pure and
   sector-shaped, so it is exactly the kind of logic that belongs there —
   disagreement between sources, one source unknown, both unknown, one stale.

### Phase 5 — prove it

1. Re-run the Phase 0 table for five configurations: baseline, re-aimed head
   only, + Phase 1, ToF only, fused. One table, five columns, in this file.
2. Drive the corridor test from the `Avoider` docstring with low obstacles
   added — the 5 and 10 cm rows that used to be invisible.
3. Confirm no regression in open-room cruising. The risk is the floor reading as
   an obstacle and the robot never leaving `steer` for `cruise`.
4. Update `README.md`'s env-var table, the `stereo.py` module docstring (which
   says the band exists to exclude the floor, and will be wrong), and the
   `config.py` tilt block.

## 5. What will probably go wrong

- **The neck aim cannot be fixed within the firmware window.** `kJointLimits[]`
  now allows tilt 20..40 only. If the incline means a usable aim sits outside
  that, it is a firmware change or a bracket change, and it blocks Phase 0.
- **Camera pitch is not accurately known and the fitted floor is wrong.** The
  likeliest software failure; the robot refuses to cruise in an empty room.
  RANSAC is the escape hatch.
- **The ToF library does not build cleanly on JetPack.** Budget a day, and find
  out in Phase 2 on the bench, before anything is glued to the chassis.
- **The I2C run to the front of the chassis is too long.** Measure first;
  100 kHz or a bus extender are the answers, and both are easier before mounting.
- **The two sensors disagree with stereo at the overlap.** `min()` means the
  pessimist wins, so one bad ToF zone stops the robot. Apply the same robust low
  percentile across zones that `obstacles()` already applies across pixels — one
  hot zone must not read as a wall.
- **Bright sunlight** shortens ToF range sharply. Indoors this is fine; worth
  knowing before someone tests on a patio and concludes the parts are faulty.

## 6. Deferred, deliberately

**Bumper microswitches.** Not yet in hand. Still the right backstop: every
ranging sensor has a failure mode and contact has none. Leave the seam in
`fuse()`.

**VESC stall detection, as the stand-in.** `WheelTelemetry` in
`orio/drivetrain.py` already decodes `erpm`, `current_a` and `fault_code` from
`COMM_GET_VALUES`. Commanded duty with near-zero erpm and rising current means
the robot is pushing against something. No new hardware, one link message, and
it catches the case every forward-looking sensor misses. Worth doing early if
Phase 2 stalls on parts.

**An MCU-side collision cutout.** The e-stop locality argument from section 3,
kept separate because it is a safety feature rather than a perception one. The
drivetrain board already owns the e-stop and the watchdog; a cutout there needs
a reason to fire, not the ToF sensors. Revisit once the Jetson-side guard has
shown what it does and does not catch.

**IMU.** Does not detect obstacles and does not belong in this plan. It belongs
to the next one: the `Avoider` docstring's admission that it "wanders rather
than travels" is a missing heading reference, and a gyro is what fixes it.

**2D LiDAR.** The biggest single capability jump — it would also end "there is
no rear or side sensing… the back-off reverses blind" — and out of scope here.
It scans one plane, so it complements stereo rather than replacing it: it would
miss a tabletop overhang the cameras see easily.

## 7. Starting on the Jetson

```bash
cd orio-jetson && uv sync
```

Phase 0 needs no new dependencies — `tools/stereo_debug.py`, a tape measure and
some boxes of known height. Stop the main app first: stereo needs both sensors
and nothing else may hold a camera.

Phase 2 step 2 (does `vl53l5cx-ctypes` build?) is independent of everything else
and is the item most likely to surprise. It can run in parallel with Phase 0.
