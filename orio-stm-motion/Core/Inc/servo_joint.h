/**
  * @file    servo_joint.h
  * @brief   Driver for one 2-DOF joint (neck or arm) built from a pair of
  *          RC servos on an aluminum-alloy bracket:
  *            - pan  (bottom): 20 kg-cm, 270 deg servo -- left/right travel.
  *            - tilt (top):    25 kg-cm, 180 deg servo -- up/down travel.
  *          Both axes are driven through servo.c; this module just pairs two
  *          Servo_t instances, tags them with which body position they are,
  *          and moves them as one logical joint.
  * @note    Two different things are deliberately kept separate here:
  *            - PAN/TILT_SERVO_* below: the servo MODEL's physical
  *              calibration (pulse width at its true mechanical extremes).
  *              Fixed -- every joint uses the same servo hardware, so this
  *              doesn't vary per joint. Changing it means the servo model
  *              itself changed, not that one joint's bracket is different.
  *            - Per-joint commandable LIMIT (min/max angle actually
  *              allowed): a safety-relevant SUBSET of the servo's full
  *              travel, e.g. because a specific bracket hits a mechanical
  *              stop before the servo's own electrical limit, or because
  *              only part of the travel is a useful pose for that joint.
  *              This is what varies per joint, and lives in the
  *              position-indexed kJointLimits[] table in servo_joint.c.
  *          Conflating the two would re-slope the pulse mapping every time
  *          a joint's limit narrows -- e.g. restricting tilt to a 60 deg
  *          window out of its 180 deg travel would stretch that one pulse
  *          range across the whole window, so a 1 deg command near one end
  *          would swing the servo across most of its real travel.
  */
#ifndef SERVO_JOINT_H
#define SERVO_JOINT_H

#include "servo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Both servo models below are calibrated straight from the vendor's own
 * reference drivers for these two parts (the 2-DOF gimbal firmware, and
 * the Arduino / Pico / Raspberry Pi tutorials): a 500..2500 us pulse
 * sweeps the part's full rated travel, 50 Hz frame, no wider on either
 * end. The two parts differ only in how much travel that same pulse range
 * sweeps -- 270 deg for pan, 180 deg for tilt.
 *
 * Angles throughout this firmware are on that SAME vendor scale: pan runs
 * 0.00..270.00, tilt runs 0.00..180.00, both measured from the servo's own
 * zero end. Mid-travel is therefore 135.00 for pan and 90.00 for tilt, not
 * 0. An earlier revision re-zeroed both axes on mid-travel; that made every
 * angle in the codebase differ from every angle in the vendor's own
 * documentation and examples by a constant, and the per-joint limit table
 * was in fact written in vendor numbers and then read as re-zeroed ones,
 * which silently aimed the neck's tilt window at the top 5 deg of the
 * servo's travel instead of a band around level.
 *
 * NOTE: the 270 deg pan servo needs a 6..7.4 V supply. Below that it
 * misbehaves rather than simply running weak, so a pan axis that jitters
 * or fails to reach its commanded angle is worth checking against the
 * rail before it is treated as a calibration problem. */

/* Pan servo model: full 270 deg mechanical travel, 0 at one end stop.
 * Pulse width is calibrated against this FULL range regardless of what
 * subset any given joint is allowed to command. Mid-travel is 135.00. */
#define PAN_SERVO_PULSE_MIN_US   500u
#define PAN_SERVO_PULSE_MAX_US   2500u
#define PAN_SERVO_CALIB_MIN_CDEG (0)     /*   0.00 deg -> 500 us */
#define PAN_SERVO_CALIB_MAX_CDEG (27000) /* 270.00 deg -> 2500 us */

/* Tilt servo model: full 180 deg mechanical travel, 0 at one end stop.
 * Same 500..2500 us pulse range as pan -- the two servo models differ in
 * how much travel that range sweeps (180 vs 270 deg), not in the pulse
 * widths that sweep it. Mid-travel is 90.00.
 *
 * An earlier revision stretched this to 2556 us / +95.00 deg because the
 * unit was seen to keep moving past its nominal top end. Every vendor
 * reference for this part maps 0..180 deg onto exactly 500..2500 us and
 * goes no wider, so that extra span is travel past the servo's rated
 * limit, not rated travel that had been left out. Commanding into it
 * drives the servo against its own end stop, where it stalls and heats
 * under load instead of holding an angle. Restored to the rated range. */
#define TILT_SERVO_PULSE_MIN_US   500u
#define TILT_SERVO_PULSE_MAX_US   2500u
#define TILT_SERVO_CALIB_MIN_CDEG (0)     /*   0.00 deg -> 500 us */
#define TILT_SERVO_CALIB_MAX_CDEG (18000) /* 180.00 deg -> 2500 us */

/* The motion profile ServoJoint_Update() runs both axes of every joint
 * along: a trapezoid, accelerating at SERVO_JOINT_ACCEL_CDEG_PER_S2 up to a
 * cruise of SERVO_JOINT_SLEW_CDEG_PER_S, then braking back down so the axis
 * arrives at rest. A move too short to reach cruise is all ramp and the
 * trapezoid degenerates to a triangle.
 *
 * The acceleration limit is what makes a move look smooth, and it is the
 * half that used to be missing: an earlier revision stepped at the cruise
 * speed flat, which means the axis went from stopped to 120 deg/s between
 * two consecutive 1 ms ticks and back to stopped the instant it arrived --
 * unbounded acceleration at both ends of every move. The neck is where that
 * showed worst, since the head is the heaviest thing hung on any of these
 * joints.
 *
 * At the values below a move spends 0.30 s and 18.00 deg on each ramp, so
 * anything shorter than 36.00 deg never reaches cruise. Both constants are
 * fixed and global rather than per-joint or per-command: no bracket
 * currently needs a different profile, and nothing yet exposes a per-move
 * speed over the wire (CMD_MOVE_JOINT_TO's payload has no speed field).
 * Lower the cruise to make a joint travel slower; lower the acceleration to
 * make it start and stop more gently at the same travel speed. */
#define SERVO_JOINT_SLEW_CDEG_PER_S   12000 /* cruise, 120.00 deg/s */
#define SERVO_JOINT_ACCEL_CDEG_PER_S2 40000 /* 400.00 deg/s^2 */

/**
  * @brief  Which body position a ServoJoint_t occupies. The robot has (or
  *         will have) exactly one physical joint pair per value.
  */
typedef enum
{
  JOINT_POS_NECK,
  JOINT_POS_LEFT_ARM,
  JOINT_POS_RIGHT_ARM,
} ServoJointPosition_t;

#define SERVO_JOINT_POSITION_COUNT 3u /* one per ServoJointPosition_t value */

/**
  * @brief  Commandable angle limit for one servo axis -- a subset of (or
  *         equal to) that servo model's full calibrated travel.
  */
typedef struct
{
  int16_t min_cdeg; /* minimum commandable angle, hundredths of a degree */
  int16_t max_cdeg; /* maximum commandable angle, hundredths of a degree */
} ServoAxisLimit_t;

/**
  * @brief  Pan + tilt commandable limits for one joint position. One
  *         instance per ServoJointPosition_t value lives in the generated,
  *         position-indexed kJointLimits[] table in servo_joint.c.
  */
typedef struct
{
  ServoAxisLimit_t pan;
  ServoAxisLimit_t tilt;
} ServoJointLimits_t;

/**
  * @brief  One 2-DOF joint: a pan (bottom) and tilt (top) servo moved together.
  * @note   target_*_cdeg is where ServoJoint_SetAngles() wants the joint;
  *         current_*_cdeg is where the PWM outputs are actually set right
  *         now. ServoJoint_Update() walks current toward target over time
  *         instead of jumping there in one call -- see servo_joint.c.
  *         *_vel_cdeg_per_s is the speed each axis is travelling at, and is
  *         state rather than something derivable from the two angles: the
  *         profile's next step depends on the speed the axis is already
  *         carrying, which is exactly what lets it ramp instead of switch.
  */
typedef struct
{
  Servo_t pan;
  Servo_t tilt;
  ServoJointPosition_t position;
  int16_t target_pan_cdeg;
  int16_t target_tilt_cdeg;
  int16_t current_pan_cdeg;
  int16_t current_tilt_cdeg;
  int32_t pan_vel_cdeg_per_s;  /* signed: negative travels toward 0 deg */
  int32_t tilt_vel_cdeg_per_s;
} ServoJoint_t;

/**
  * @brief  Binds the pan and tilt servos of a joint pair to their
  *         timer/channels using the fixed servo-model pulse calibration
  *         (PAN/TILT_SERVO_PULSE_*_US), tags the pair with its body
  *         position, and starts both PWM outputs at neutral (mid-travel).
  * @param  joint        Instance to initialize.
  * @param  position     Which body position this joint pair is.
  * @param  pan_htim     Timer handle for the bottom (pan) servo, 50 Hz / 1 us ticks.
  * @param  pan_channel  Timer channel the pan servo signal line is wired to.
  * @param  tilt_htim    Timer handle for the top (tilt) servo, 50 Hz / 1 us ticks.
  * @param  tilt_channel Timer channel the tilt servo signal line is wired to.
  * @retval None
  */
void ServoJoint_Init(ServoJoint_t *joint, ServoJointPosition_t position,
                      TIM_HandleTypeDef *pan_htim, uint32_t pan_channel,
                      TIM_HandleTypeDef *tilt_htim, uint32_t tilt_channel);

/**
  * @brief  Sets the target angles for both axes of the joint. Each angle is
  *         clamped to that joint's own commandable limit (see
  *         ServoJoint_PanLimit() / ServoJoint_TiltLimit()) first.
  * @note   Does not move the servos itself -- it only sets where
  *         ServoJoint_Update() should steer them. Call ServoJoint_Update()
  *         periodically to actually approach this target.
  * @param  joint     Instance, already initialized with ServoJoint_Init().
  * @param  pan_cdeg  Pan target angle in hundredths of a degree.
  * @param  tilt_cdeg Tilt target angle in hundredths of a degree.
  * @retval None
  */
void ServoJoint_SetAngles(ServoJoint_t *joint, int16_t pan_cdeg, int16_t tilt_cdeg);

/**
  * @brief  Advances both axes along their motion profile by elapsed_ms:
  *         each axis's speed moves toward the cruise rate -- or toward zero
  *         once the target is close enough that it needs the room to brake
  *         -- by at most SERVO_JOINT_ACCEL_CDEG_PER_S2 * elapsed_ms / 1000,
  *         and the axis then travels at whatever speed that leaves it with.
  *         Whichever axes moved are re-mapped to a pulse width and applied.
  *         A no-op for any axis already stopped on its target.
  * @param  joint      Instance, already initialized with ServoJoint_Init().
  * @param  elapsed_ms Milliseconds since the last call (0 is a safe no-op).
  * @retval None
  */
void ServoJoint_Update(ServoJoint_t *joint, uint32_t elapsed_ms);

/**
  * @brief  Disables both PWM outputs (signal lines go idle/low) and
  *         abandons the rest of any in-flight move -- target and speed both
  *         reset to a joint standing still wherever it had actually
  *         travelled to when it stopped.
  * @note   Use for e-stop; see Servo_Stop().
  * @param  joint Instance, already initialized with ServoJoint_Init().
  * @retval None
  */
void ServoJoint_Stop(ServoJoint_t *joint);

/**
  * @brief  Re-enables both PWM outputs at the angles the joint was stopped
  *         at. A move interrupted by ServoJoint_Stop() does NOT continue --
  *         the controller must re-command it.
  * @param  joint Instance, already initialized with ServoJoint_Init().
  * @retval None
  */
void ServoJoint_Resume(ServoJoint_t *joint);

/**
  * @brief  Returns the pose a joint parks at with nothing commanded: the
  *         midpoint of each axis's commandable window.
  * @note   NOT necessarily the servo's mechanical centre. Where a window
  *         excludes that centre -- the arms' 30..90 deg tilt does -- the
  *         centre is an angle the joint must never be driven to, so it
  *         cannot serve as a rest pose. Callers needing a safe default
  *         angle (ServoJoint_Init() parking the outputs, the protocol
  *         layer seeding the angles it reports before anything has been
  *         commanded) must use this rather than assume any fixed value.
  * @param  position  Body position to look up (must be < SERVO_JOINT_POSITION_COUNT).
  * @param  pan_cdeg  Out: pan neutral, hundredths of a degree.
  * @param  tilt_cdeg Out: tilt neutral, hundredths of a degree.
  * @retval None
  */
void ServoJoint_NeutralAngles(ServoJointPosition_t position, int16_t *pan_cdeg, int16_t *tilt_cdeg);

/**
  * @brief  Returns the pan axis's commandable limit for a joint position.
  * @param  position Body position to look up (must be < SERVO_JOINT_POSITION_COUNT).
  * @retval Pointer into the generated, position-indexed limit table.
  */
const ServoAxisLimit_t *ServoJoint_PanLimit(ServoJointPosition_t position);

/**
  * @brief  Returns the tilt axis's commandable limit for a joint position.
  * @param  position Body position to look up (must be < SERVO_JOINT_POSITION_COUNT).
  * @retval Pointer into the generated, position-indexed limit table.
  */
const ServoAxisLimit_t *ServoJoint_TiltLimit(ServoJointPosition_t position);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_JOINT_H */
