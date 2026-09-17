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

> Status: **the software is built, the parts are wired and ranging, and the rig
> is still not measured.** Branched off `develop` at `7577851`. Written from the code and the
> committed calibration; every figure marked *(predicted)* is arithmetic rather
> than a measurement, and Phase 0 exists to replace those with real numbers.
> Expect some of them to be wrong. See **Progress** below for what has actually
> been done, and what each remaining step is waiting on.

## Progress — 2026-09-17

Phases 1, 3 and 4 are implemented and the pure logic is tested. **Phase 2 is
now done on the real parts**: both sensors answer at 0x29, one per bus, and the
pair opens, ranges and fuses through `orio/tof.py` — see *Bring-up on the wired
pair* below, which cost two real bugs. Phase 0 and the bracket are untouched,
because neither can be done from a keyboard: one needs a tape measure and the
other needs a bracket.

**Everything new is OFF by default** (`ORIO_STEREO_GROUND_PLANE=0`, `ORIO_TOF=0`)
and nothing in the existing path changed behaviour. That is not caution for its
own sake — Phase 0's measurements are the inputs both features need, and
switching either on against a guessed camera pitch is precisely how the robot
ends up refusing to cruise in an empty room.

| | what exists | what it is waiting on |
|---|---|---|
| Phase 0 | nothing | a tape measure, boxes, and the tilt sweep. Blocks everything below from being *trusted*, not from being *run* |
| Phase 1 | `sectors.SectorGeometry` / `reduce()`, `DepthEstimator.obstacles()`, the class view in `stereo_debug.py` (press **G**) | `STEREO_CAM_HEIGHT_M` and `STEREO_CAM_PITCH_DEG`, which are placeholders |
| Phase 2 | **done** — both sensors wired, opening, ranging and fusing on the robot | nothing |
| Phase 3 | `orio/tof.py`, `tools/tof_debug.py` | the bracket, the measured mount pose per sensor, and `TOF_FLIP_H/V` from a hand in front of the grid |
| Phase 4 | `sectors.fuse()`, `avoid.Sensor(use_tof=)`, `body.py` notes, `teleop_guarded.py --no-tof`, five new cases in `test_cruise.py` | nothing — it runs today, with one source |
| Phase 5 | nothing | all of the above |

### Phase 2 step 2 is answered: the library builds, and it is quick

The item flagged as "the single item most likely to cost a day" cost seven
seconds. `vl53l5cx-ctypes` **0.0.3** builds from source on this aarch64 JetPack
with no coaxing — `uv pip install vl53l5cx-ctypes`, 6.9 s, and it pulls in
`smbus2` and nothing else. No vendor SDK, no torch. It is now a normal project
dependency; `uv sync` brings it in.

Its API is small and it is ST's ULD underneath, as advertised:
`VL53L5CX(i2c_addr=0x29, i2c_dev=SMBus(bus))` — the constructor is what performs
the ~84 KB firmware upload and it raises rather than returning a dud handle —
then `set_resolution(64)`, `set_ranging_frequency_hz(15)`,
`set_target_order(TARGET_ORDER_CLOSEST)`, `start_ranging()`, and
`data_ready()` / `get_data()` per frame. `get_data()` returns `distance_mm` and
`target_status` as `[target][zone]`, one target per zone. There was no need for
the ctypes-shim fallback.

### Bring-up on the wired pair — 2026-09-17

Both parts answer at 0x29, one per bus, exactly as planned. Getting from there
to a fan that stays in the fusion took two fixes, and neither was predicted:

**1. Opening the two sensors in parallel SEGFAULTS the interpreter.** Not a
race that corrupts a reading — the process dies. Reproduced 3/3, both threads
in `is_alive()`, which is the first call that drives the I2C callbacks and
comes *before* the firmware upload. Locking only the constructor did not help,
and neither did deferring `init()` with `skip_init=True`: the upload crashes in
parallel too. The ctypes wrapper is simply not reentrant while a device is
being brought up.

This mattered more than an ordinary bug because Phase 4 step 2 says a ToF
failure must degrade to stereo rather than halt — and `avoid.Sensor.start()`
does wrap the open in `try/except` to guarantee exactly that. **A SIGSEGV is
not an exception.** The one guard written to stop the fan taking the robot down
was the one thing that could not catch it. Opens are now serialised under a
module-level `_OPEN_LOCK`; steady-state reads were measured safe from separate
threads, so only the open is serialised.

**2. The two buses are not equals, and it shows in the data.** From the device
tree, bus 7 runs at **400 kHz** and bus 1 at **100 kHz**:

| | bus 7 (pins 3/5) | bus 1 (pins 27/28) |
|---|---|---|
| clock | 400 kHz | 100 kHz |
| firmware upload at open | 2.73 s | 8.78 s |
| frame rate, own thread | 15.3 Hz | 4.7 Hz |
| frame gap, median / max | 46 / 136 ms | 196 / 329 ms |

Polled from one thread the fast sensor is dragged down to the slow one and
**both** fall to 4 Hz, because the thread sits inside the slow sensor's ~1 KB
`get_data()`. Each sensor now gets its own reader thread.

Worse, under the detector's own load the bus-1 sensor is GIL-starved by its
neighbour — the ULD calls back into Python for every I2C chunk — and its frame
gap stretched to **1118 ms**. Since `fuse()` stamps with its oldest
contributor, that dragged the whole fan past `TOF_STALE_S` and `fresh_map()`
withheld *both* sensors: avoidance silently fell back to stereo, blind low and
blind inside 0.25 m, in stretches, exactly where these parts were added to see.
Two changes fix it, and both are the same principle the rest of the plan runs
on — degrade where the fault is:

* **Staleness is applied per sensor**, in `_republish`. A starved sensor drops
  out and its sectors go unknown; its healthy neighbour keeps publishing.
* **Liveness asks whether the fan is still publishing**, not how old the oldest
  reading in the map is. A map fused from a 0.49 s reading is itself stamped
  0.49 s old, so the old rule blinked the fan out on arithmetic alone while
  both sensors were ranging happily. The map keeps its honest
  oldest-contributor timestamp for anyone fusing it further.

Measured after: **0 dropouts in 989 samples over 20 s**, both sensors
contributing sectors, against intermittent whole-fan dropouts before.

**Startup is now 11.5 s for the pair** (2.73 + 8.78, serialised), and it is not
a hang. Raising bus 1 to 400 kHz would collapse the upload time, the frame rate
and the starvation at once — it is a device-tree change on a bus the carrier
board's USB-C PD controller and power monitor already own, so it is a decision
to take deliberately. It is also worth asking which arc deserves the fast bus.

### The as-built mount is forward-looking, not low and level — 2026-09-17

Phase 3 step 1 specifies the pair mounted 3-6 cm off the floor and **level**,
so that each array sees floor in its lower rows and the height classification
removes it. What is actually on the robot is a pair on the **front, looking
forward**. The sensors confirm it: neither finds a floor plane anywhere in its
64 zones, and the lower rows return ~2.9 m — the far wall. A ray leaving a
level sensor 4.5 cm up and 19.7 deg down meets the floor at 13 cm and cannot
reach 2.9 m, so whatever the bracket is, it is not that.

This is worth settling before any pose is written into config, because the
mount is what decides whether the fan answers the question it was added for.
The two volumes in section 1 are *below the camera band* and *inside 0.25 m*,
and both are close to the floor and close to the robot. A forward-looking pair
covers the second but reaches over the first, which is the one that motivated
the whole plan: the box, the shoe, the door threshold.

It may still be the right mount — a forward fan covers the near gate and works
in the dark against blank walls, which stereo does not. But it is a different
sensor answering a different question, and the plan should say which one is
intended rather than discover it from a grid of numbers later.

`tools/tof_pose.py` handles both mounts. From a floor it solves height, pitch
and roll; from a wall the robot is squared to it solves pitch and yaw, which is
the only reference a forward-looking sensor has. It cannot get height from a
wall — a vertical plane looks identical from every height — and for this mount
that is a tape measure anyway. Its `--selftest` recovers ten known poses from
synthetic grids, including 10 mm of simulated range noise, so the maths can be
trusted before the bracket is.

### The ULD reports AXIAL distance, and that was worth a wall to find out

`distance_mm` is the distance along the sensor's **optical axis**, not along
each zone's own line of sight, and the zone grid behaves as a pinhole array
rather than a fan of equally-spaced rays. Both were measured against a flat
wall at 0.86 m, and the evidence is not subtle:

| read as | plane residual RMS | centre-minus-edge |
|---|---|---|
| line-of-sight range (the old code) | 25 mm | **+44 mm** |
| axial distance through a pinhole | **3.3 mm** | +0.4 mm |

A flat wall read the wrong way comes back as a bowl, and the residual map is a
textbook bullseye: flat around the border, 44 mm proud in the middle. Read the
right way the structure vanishes into 3.3 mm of sensor noise.

Uncorrected, a corner zone's range is **12% short** and its elevation 1.1 deg
too high — `sin` where the geometry wants `tan`. Twelve per cent reads as an
obstacle nearer than it is, which sounds like the safe direction and is not:
this feeds the height-above-floor classification, where a misplaced floor point
at the edge of the grid becomes an obstacle and the robot refuses to cruise
down an empty corridor. That is the Phase 5 regression, arrived at through a
datasheet figure rather than a mistake in the policy.

`stereo.py:_ground_geometry` had this right from the beginning — pinhole ray,
then `depth * norm` for the line-of-sight range. The ToF path now does the same
two things, so both sensors finally agree on what a point is.

**The fix is independently confirmed by the sensors themselves.** Before it,
the pair's fitted plane held 22 and 20 zones of 64 and their pitches disagreed
by 7 deg. After it: 55-63 zones, and two physically separate parts on the same
bracket agree on pitch to 0.2 deg. Repeatability over five solves went from
+/-1.45 deg to +/-0.08.

### The measured mount — 2026-09-17, against a wall at 0.86 m

| | pitch (down) | yaw (right) | zones on the plane |
|---|---|---|---|
| `tof-left` (bus 7) | +8.13 +/- 0.08 deg | -5.03 +/- 0.21 deg | 55-57 / 64 |
| `tof-right` (bus 1) | +7.80 +/- 0.05 deg | +2.07 +/- 0.03 deg | 62-63 / 64 |

Two things in this table are not what config assumes.

**The splay is 7.1 deg, not 45.** Config carries `-22.5,+22.5` — the plan's
intent of two 45 deg squares abutting into ~90 deg of cover. The bracket holds
them 7.1 deg apart, so the fields overlap almost entirely and the pair spans
about 52 deg rather than 90. Against the 73.1 deg sector grid that leaves the
outer sectors with no ToF in them at all, and the middle sectors covered twice.
Note the splay is the DIFFERENCE between the two yaws, so it survives whatever
error there was in squaring the robot to the wall — only the absolute yaws move
with that. The bisector sits at -1.5 deg, which is either the robot being that
far off square or the bracket being that far off centre, and this measurement
cannot tell which.

**Both sensors look down about 8 deg**, where config has 0.

Heights are still unmeasured and a wall cannot give them: a vertical plane
looks identical from every height. Tape measure.

### The I2C survey, before anything is soldered

`i2cdetect -l` on this Orin Nano, confirmed against the device tree:

| bus | controller | 40-pin | what is on it |
|---|---|---|---|
| `/dev/i2c-7` | `c250000.i2c` | pins 3/5 | **empty** |
| `/dev/i2c-1` | `c240000.i2c` | pins 27/28 | `0x25` and `0x40`, both driver-claimed (`UU`) |

So the plan's bus table holds, with one correction worth having: **bus 1 is not
private.** Two carrier-board devices already live there. Neither is at 0x29 so
there is no collision, but it does mean the ToF is sharing a bus with something
whose driver owns it, and a bus error there is not necessarily the ToF's fault.
Bus 7 is completely clear — note that the stereo camera's ICM20948 IMU would
also land there at 0x68 if it is ever wired (it is not, today).

The user account is already in the `i2c` group, so no `usermod` is needed.

### Two decisions the plan did not make, made here

**1. "Nothing there" needed a third state.** Classifying by height creates
sectors that are unknown for a new reason: not "I cannot see" but "I can see the
ground and there is nothing standing on it". Reported as `None` — which is what
Phase 1 step 3 literally says — the policy treats them as blocked, and an empty
room stops the robot dead. That is the Phase 5 regression, reached by following
the plan rather than by deviating from it.

So `Sector` gained `clear_m`: *the ground under this sector was verified free
out to here*. It is not an obstacle distance, it never competes with one, and it
is applied only where no sensor saw anything at all (`sectors.fill_clear`, run
after fusion). It is also more honest than it first looks — an obstacle too dark
to match also occludes the floor behind it, so the observed ground stops at the
obstacle rather than past it. The residual risk is real and is now the main
thing Phase 5 should hunt for: a small untextured obstacle with visible floor to
either side of it inside the same sector.

**2. The band and the height test fuse by `min()`, they do not switch.** Phase 1
step 4 asks the height test to "degrade back to the row band" past
`STEREO_HEIGHT_TRUST_M`. Implemented as a hand-off, the two would disagree at
the boundary and the nearer reading could be the one discarded. Implemented as
`fuse()` — the same function the ToF uses — each is authoritative over its own
volume and the pessimist wins where they overlap. Past the trust range a point
can still count as floor (good evidence, cheap to get right) but is no longer
allowed to *claim* an obstacle (the part the noise ruins).

### Measured incidentally

* The ground-plane reduction costs **5.7 ms** against the row band's **2.2 ms**,
  at 320x240 with `STEREO_GROUND_STRIDE=2` on this Jetson — 3.5 ms added to a
  33 ms frame budget. The trigonometry is cached at open (`SectorGeometry`),
  which is what keeps it there; rebuilt per frame it is several times that.
* Each ToF array loses its outer two columns to the 7-sector grid: 48 of 64
  zones land inside ±36.55 deg, 16 fall outside and are discarded, exactly as
  section 3 predicted. The pair still covers all seven sectors between them.

### What to do next, in order

1. **Phase 0.** Nothing below is trustworthy without it, and its first item may
   shrink the rest considerably. It needs no code and no parts.
2. Fill in `STEREO_CAM_HEIGHT_M` / `STEREO_CAM_PITCH_DEG`, set
   `ORIO_STEREO_GROUND_PLANE=1`, and watch `tools/stereo_debug.py` with **G**
   held on an empty floor. Red creeping toward the horizon means the pitch is
   too shallow.
3. ~~Wire one sensor, run `tools/tof_debug.py --bus 7`, wave a hand at it.~~
   Done — both sensors, see *Bring-up on the wired pair*. The hand wave is
   still owed, and it is what sets `TOF_FLIP_H` / `TOF_FLIP_V`: run
   `tools/tof_debug.py` and check the grid lights up on the side the hand is
   actually on, and in the row it is actually in.
4. **Settle the mount**, then measure it. The bracket exists and points
   forward rather than down (see *The as-built mount*), so decide whether that
   is the intent before recording it. Then square the robot to a flat wall,
   run `tools/tof_pose.py --wall`, tape-measure the height, fill in
   `TOF_HEIGHTS_M` / `TOF_PITCHES_DEG` / `TOF_YAWS_DEG`, and set `ORIO_TOF=1`.
   Until then the fan stays off and the placeholders are the plan's intent,
   not a measurement.

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
