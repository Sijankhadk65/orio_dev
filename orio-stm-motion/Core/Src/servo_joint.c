/**
  * @file    servo_joint.c
  * @brief   2-DOF pan/tilt servo joint pair implementation (see servo_joint.h).
  */
#include "servo_joint.h"

/* ServoJoint_Update() works in integer arithmetic at the caller's
 * millisecond resolution, and the profile has two per-call increments that
 * both divide by 1000: SERVO_JOINT_ACCEL_CDEG_PER_S2 * elapsed_ms / 1000
 * for the change in speed, and speed * elapsed_ms / 1000 for the travel
 * that speed buys. Either one set below 1000 truncates to zero for a 1 ms
 * tick, so the rate would be entirely below the resolution it is applied
 * at: the axis would crawl at axis_step()'s one-cdeg progress floor and the
 * configured number would be silently doing nothing. Reject both at build
 * time rather than ship a constant that does not mean what it says. */
#if SERVO_JOINT_SLEW_CDEG_PER_S < 1000
#error "SERVO_JOINT_SLEW_CDEG_PER_S below 1000 truncates to a 0 cdeg step at 1 ms tick resolution"
#endif
#if SERVO_JOINT_ACCEL_CDEG_PER_S2 < 1000
#error "SERVO_JOINT_ACCEL_CDEG_PER_S2 below 1000 truncates to a 0 cdeg/s speed change at 1 ms tick resolution"
#endif

/* Upper bound on the elapsed_ms one ServoJoint_Update() call will act on. A
 * long stall (debugger halt, a blocked main loop) would otherwise authorize
 * one step big enough to cover the whole travel, and one speed change big
 * enough to jump straight to the cruise rate -- exactly the lurch the
 * profile exists to prevent -- and a large enough value would overflow the
 * int32 multiplies outright. Capping means the joint resumes its profile
 * from where it was instead of catching up all at once. */
#define SERVO_JOINT_MAX_STEP_MS 100u

/**
  * @brief  Per-joint commandable limits, indexed by ServoJointPosition_t.
  * @note   All angles are on the vendor scale (see servo_joint.h): pan
  *         0..270, tilt 0..180, both from the servo's own zero end. Tilt
  *         mid-travel -- level, for a bracket mounted square -- is 90.00.
  *
  *         Pan is the pan servo's full 270 deg travel on every joint.
  *         Tilt differs by joint and is always a restricted window:
  *           neck:      85.00 .. 95.00 deg -- a 10 deg band centred on
  *                      level. The head is bolted to this axis, so the
  *                      window is what the head clears, not what the servo
  *                      can reach.
  *           arms:      30.00 .. 90.00 deg -- from well below level up to
  *                      level. Untested against real brackets; nothing is
  *                      mounted yet.
  *         These are the numbers this table has always held. They were
  *         being read as offsets from mid-travel rather than vendor
  *         angles, which put the neck's window at 175..185 -- against the
  *         servo's top end stop -- and is what made the head stall.
  *         Edit a row here if one joint's bracket needs a different limit.
  */
/* !! TEMPORARY CALIBRATION BUILD -- DO NOT SHIP !!
 * The neck's tilt row is opened to the tilt servo's FULL 0..180 travel so
 * tools/sweep_axis.py can walk the axis out to its real mechanical stops and
 * measure them. That removes the very protection the row exists to provide:
 * with this flashed, a tilt command drives the axis wherever it is told,
 * including into whatever the head would otherwise foul on.
 *
 * Only run this with the head unbolted. Once the stops are measured, put the
 * measured numbers back in place of { 0, 18000 } and reflash before mounting
 * anything to the axis. The previous value was { 8500, 9500 } -- a 10 deg
 * band centred on level -- if you need to restore it unchanged.
 *
 * Nothing else is affected: the axis still parks at 9000 (90.00 deg, level,
 * 1500 us) because that is the midpoint of this wider window too. */
static const ServoJointLimits_t kJointLimits[SERVO_JOINT_POSITION_COUNT] =
{
  /* JOINT_POS_NECK */      { .pan = { 0, 27000 }, .tilt = { 0, 18000 } }, /* TEMPORARY -- see below */
  /* JOINT_POS_LEFT_ARM */  { .pan = { 0, 27000 }, .tilt = { 3000, 9000 } },
  /* JOINT_POS_RIGHT_ARM */ { .pan = { 0, 27000 }, .tilt = { 3000, 9000 } },
};

/* A commandable window is only meaningful as a SUBSET of the servo model's
 * rated travel (see servo_joint.h). A row reaching outside it does not get
 * caught anywhere at runtime -- clamp_cdeg() clamps commands to the window,
 * not the window to the servo -- so the axis would just be driven against
 * its own end stop. That is exactly how the neck's old 95 deg ceiling came
 * to be papered over by re-calibrating the servo model itself; assert it
 * here so the next such row fails the build instead. */
_Static_assert(PAN_SERVO_CALIB_MIN_CDEG <= 0 && 27000 <= PAN_SERVO_CALIB_MAX_CDEG,
               "a joint's pan window reaches outside the pan servo's rated travel");
_Static_assert(TILT_SERVO_CALIB_MIN_CDEG <= 0 && 18000 <= TILT_SERVO_CALIB_MAX_CDEG,
               "a joint's tilt window reaches outside the tilt servo's rated travel");

/**
  * @brief  Maps a commanded pan angle to a pulse width, against the pan
  *         servo model's fixed full-travel calibration (not the joint's
  *         commandable limit -- see servo_joint.h).
  * @param  angle_cdeg Angle in hundredths of a degree.
  * @retval Pulse width in microseconds.
  */
static uint16_t pan_angle_to_pulse_us(int16_t angle_cdeg)
{
  int32_t span_cdeg = PAN_SERVO_CALIB_MAX_CDEG - PAN_SERVO_CALIB_MIN_CDEG;
  int32_t span_us = PAN_SERVO_PULSE_MAX_US - PAN_SERVO_PULSE_MIN_US;
  int32_t offset_cdeg = angle_cdeg - PAN_SERVO_CALIB_MIN_CDEG;

  return (uint16_t)(PAN_SERVO_PULSE_MIN_US + (offset_cdeg * span_us) / span_cdeg);
}

/**
  * @brief  Maps a commanded tilt angle to a pulse width, against the tilt
  *         servo model's fixed full-travel calibration.
  * @param  angle_cdeg Angle in hundredths of a degree.
  * @retval Pulse width in microseconds.
  */
static uint16_t tilt_angle_to_pulse_us(int16_t angle_cdeg)
{
  int32_t span_cdeg = TILT_SERVO_CALIB_MAX_CDEG - TILT_SERVO_CALIB_MIN_CDEG;
  int32_t span_us = TILT_SERVO_PULSE_MAX_US - TILT_SERVO_PULSE_MIN_US;
  int32_t offset_cdeg = angle_cdeg - TILT_SERVO_CALIB_MIN_CDEG;

  return (uint16_t)(TILT_SERVO_PULSE_MIN_US + (offset_cdeg * span_us) / span_cdeg);
}

static int16_t clamp_cdeg(int16_t value, int16_t min_cdeg, int16_t max_cdeg)
{
  if (value < min_cdeg)
  {
    return min_cdeg;
  }
  if (value > max_cdeg)
  {
    return max_cdeg;
  }
  return value;
}

/**
  * @brief  Moves value toward target by at most max_step (either direction),
  *         landing exactly on target rather than overshooting it.
  */
static int32_t step_toward(int32_t value, int32_t target, int32_t max_step)
{
  int32_t diff = target - value;

  if (diff > max_step)
  {
    diff = max_step;
  }
  else if (diff < -max_step)
  {
    diff = -max_step;
  }
  return value + diff;
}

/**
  * @brief  Advances one axis along the trapezoidal profile for elapsed_ms:
  *         picks the speed to head for -- cruise, or zero once the target
  *         is within braking distance -- moves the axis's carried speed
  *         that way by at most one tick's worth of acceleration, then
  *         travels at the result.
  * @param  current        Angle the axis is at now, hundredths of a degree.
  * @param  target         Angle it is heading for, already clamped to limit.
  * @param  limit          The axis's commandable window.
  * @param  vel_cdeg_per_s In/out: the axis's signed speed, carried between calls.
  * @param  elapsed_ms     Milliseconds this call covers; must be > 0.
  * @retval The angle the axis has reached.
  */
static int16_t axis_step(int16_t current, int16_t target, const ServoAxisLimit_t *limit,
                         int32_t *vel_cdeg_per_s, int32_t elapsed_ms)
{
  int32_t err = (int32_t)target - (int32_t)current;
  int32_t abs_err = (err < 0) ? -err : err;
  int32_t vel = *vel_cdeg_per_s;
  int32_t abs_vel = (vel < 0) ? -vel : vel;

  if (err == 0)
  {
    *vel_cdeg_per_s = 0;
    return target;
  }

  /* Travel it would still take to bring the current speed back to zero.
   * Once that is all the room left, braking is the only thing that lands
   * the axis on the target instead of past it -- the trapezoid's falling
   * edge. Below cruise the axis brakes later, which is why a short move
   * never cruises yet still arrives at rest. */
  int32_t brake_cdeg = (abs_vel * abs_vel) / (2 * SERVO_JOINT_ACCEL_CDEG_PER_S2);
  int32_t goal_vel;

  if (err > 0)
  {
    goal_vel = (vel > 0 && abs_err <= brake_cdeg) ? 0 : SERVO_JOINT_SLEW_CDEG_PER_S;
  }
  else
  {
    goal_vel = (vel < 0 && abs_err <= brake_cdeg) ? 0 : -SERVO_JOINT_SLEW_CDEG_PER_S;
  }

  /* The speed is what is rate-limited; the angle just follows it. That is
   * the whole difference from stepping at a flat speed -- every move now
   * starts and ends at rest instead of switching between stopped and cruise
   * between two ticks. A target reversed mid-move therefore brakes down
   * through zero and back up the other way rather than flipping sign. */
  vel = step_toward(vel, goal_vel, (SERVO_JOINT_ACCEL_CDEG_PER_S2 * elapsed_ms) / 1000);

  int32_t step = (vel * elapsed_ms) / 1000;

  /* Integer truncation discards the sub-cdeg part of each tick's travel,
   * and at the end of a move -- where the profile is deliberately slow --
   * that is the whole of it: the axis brakes to a speed too small to buy
   * even one cdeg per tick, whereupon the accelerate/brake decision above
   * just oscillates around a target a tenth of a degree away and the axis
   * never arrives. Guarantee one cdeg of progress whenever it is closing on
   * a target it has not reached, which converges. The floor cannot be seen:
   * one cdeg is a thirteenth of a microsecond of pan pulse. */
  if (step == 0)
  {
    step = (err > 0) ? 1 : -1;
  }

  /* Don't step past the target while closing on it. A step moving AWAY from
   * the target belongs to a reversal still braking off its old speed, so it
   * is left alone: clamping that one would be the instant stop this profile
   * exists to avoid. */
  if ((err > 0 && step > err) || (err < 0 && step < err))
  {
    step = err;
  }

  int32_t next = (int32_t)current + step;

  /* That braking-away case travels in the direction the axis was already
   * going, so it can carry the axis past the end of the commandable window
   * if the interrupted move was headed for the end of it. The window stands
   * in for a mechanical limit, so the axis stops against it and gives up
   * the rest of the brake rather than coasting through. */
  if (next < limit->min_cdeg)
  {
    next = limit->min_cdeg;
    vel = 0;
  }
  else if (next > limit->max_cdeg)
  {
    next = limit->max_cdeg;
    vel = 0;
  }

  *vel_cdeg_per_s = vel;
  return (int16_t)next;
}

const ServoAxisLimit_t *ServoJoint_PanLimit(ServoJointPosition_t position)
{
  return &kJointLimits[position].pan;
}

const ServoAxisLimit_t *ServoJoint_TiltLimit(ServoJointPosition_t position)
{
  return &kJointLimits[position].tilt;
}

void ServoJoint_NeutralAngles(ServoJointPosition_t position, int16_t *pan_cdeg, int16_t *tilt_cdeg)
{
  const ServoAxisLimit_t *pan_limit = &kJointLimits[position].pan;
  const ServoAxisLimit_t *tilt_limit = &kJointLimits[position].tilt;

  /* Midpoint of the commandable window, not of the servo's mechanical
   * travel: it is legal by construction, and sits as far from either end
   * stop as the window allows, so parking here can never rest the joint
   * against a stop. */
  *pan_cdeg = (int16_t)(((int32_t)pan_limit->min_cdeg + (int32_t)pan_limit->max_cdeg) / 2);
  *tilt_cdeg = (int16_t)(((int32_t)tilt_limit->min_cdeg + (int32_t)tilt_limit->max_cdeg) / 2);
}

void ServoJoint_Init(ServoJoint_t *joint, ServoJointPosition_t position,
                      TIM_HandleTypeDef *pan_htim, uint32_t pan_channel,
                      TIM_HandleTypeDef *tilt_htim, uint32_t tilt_channel)
{
  joint->position = position;
  Servo_Init(&joint->pan, pan_htim, pan_channel, PAN_SERVO_PULSE_MIN_US, PAN_SERVO_PULSE_MAX_US);
  Servo_Init(&joint->tilt, tilt_htim, tilt_channel, TILT_SERVO_PULSE_MIN_US, TILT_SERVO_PULSE_MAX_US);

  /* Servo_Init() parks each output at the midpoint of its PULSE range,
   * i.e. the servo's mechanical centre. For an axis whose commandable
   * window excludes that centre this is an illegal pose that would be
   * driven the moment the outputs go live: the arms' tilt window is
   * 30..90 deg, so pan's mid-travel would be fine but a joint whose
   * window excluded mid-travel would be commanded outside it, into
   * whatever the window exists to keep it off.
   * Override with the joint's own neutral -- which is inside the window by
   * construction -- before anything can resume the outputs.
   *
   * Seeding target == current also means an Update() before the first
   * SetAngles() is a no-op, so the joint holds here rather than ramping. */
  ServoJoint_NeutralAngles(position, &joint->current_pan_cdeg, &joint->current_tilt_cdeg);
  joint->target_pan_cdeg = joint->current_pan_cdeg;
  joint->target_tilt_cdeg = joint->current_tilt_cdeg;
  joint->pan_vel_cdeg_per_s = 0;
  joint->tilt_vel_cdeg_per_s = 0;

  Servo_SetPulseWidthUs(&joint->pan, pan_angle_to_pulse_us(joint->current_pan_cdeg));
  Servo_SetPulseWidthUs(&joint->tilt, tilt_angle_to_pulse_us(joint->current_tilt_cdeg));
}

void ServoJoint_SetAngles(ServoJoint_t *joint, int16_t pan_cdeg, int16_t tilt_cdeg)
{
  const ServoAxisLimit_t *pan_limit = ServoJoint_PanLimit(joint->position);
  const ServoAxisLimit_t *tilt_limit = ServoJoint_TiltLimit(joint->position);

  /* Target only -- ServoJoint_Update() walks the outputs there over time. */
  joint->target_pan_cdeg = clamp_cdeg(pan_cdeg, pan_limit->min_cdeg, pan_limit->max_cdeg);
  joint->target_tilt_cdeg = clamp_cdeg(tilt_cdeg, tilt_limit->min_cdeg, tilt_limit->max_cdeg);
}

void ServoJoint_Update(ServoJoint_t *joint, uint32_t elapsed_ms)
{
  if (elapsed_ms == 0u)
  {
    return;
  }
  if (elapsed_ms > SERVO_JOINT_MAX_STEP_MS)
  {
    elapsed_ms = SERVO_JOINT_MAX_STEP_MS;
  }

  int16_t next_pan_cdeg = axis_step(joint->current_pan_cdeg, joint->target_pan_cdeg,
                                    ServoJoint_PanLimit(joint->position),
                                    &joint->pan_vel_cdeg_per_s, (int32_t)elapsed_ms);

  if (next_pan_cdeg != joint->current_pan_cdeg)
  {
    joint->current_pan_cdeg = next_pan_cdeg;
    Servo_SetPulseWidthUs(&joint->pan, pan_angle_to_pulse_us(next_pan_cdeg));
  }

  int16_t next_tilt_cdeg = axis_step(joint->current_tilt_cdeg, joint->target_tilt_cdeg,
                                     ServoJoint_TiltLimit(joint->position),
                                     &joint->tilt_vel_cdeg_per_s, (int32_t)elapsed_ms);

  if (next_tilt_cdeg != joint->current_tilt_cdeg)
  {
    joint->current_tilt_cdeg = next_tilt_cdeg;
    Servo_SetPulseWidthUs(&joint->tilt, tilt_angle_to_pulse_us(next_tilt_cdeg));
  }
}

void ServoJoint_Stop(ServoJoint_t *joint)
{
  /* Abandon the rest of any in-flight move. Without this the target would
   * outlive the e-stop and ServoJoint_Resume() would restart the servos
   * mid-travel, driving on to a destination commanded before the stop --
   * the opposite of what stopping is for. The speed goes with it: a joint
   * resumed still carrying the speed it had when the PWM was cut would take
   * its first step at that speed, having spent the stop standing still. */
  joint->target_pan_cdeg = joint->current_pan_cdeg;
  joint->target_tilt_cdeg = joint->current_tilt_cdeg;
  joint->pan_vel_cdeg_per_s = 0;
  joint->tilt_vel_cdeg_per_s = 0;

  Servo_Stop(&joint->pan);
  Servo_Stop(&joint->tilt);
}

void ServoJoint_Resume(ServoJoint_t *joint)
{
  Servo_Resume(&joint->pan);
  Servo_Resume(&joint->tilt);
}
