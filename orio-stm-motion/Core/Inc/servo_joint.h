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

/* Pan servo model: full 270 deg mechanical travel, centered at 0. Pulse
 * width is calibrated against this FULL range regardless of what subset
 * any given joint is allowed to command. */
#define PAN_SERVO_PULSE_MIN_US   500u
#define PAN_SERVO_PULSE_MAX_US   2500u
#define PAN_SERVO_CALIB_MIN_CDEG (-13500) /* -135.00 deg */
#define PAN_SERVO_CALIB_MAX_CDEG (13500)  /*  135.00 deg */

/* Tilt servo model: full 180 deg mechanical travel, centered at 0. */
#define TILT_SERVO_PULSE_MIN_US   500u
#define TILT_SERVO_PULSE_MAX_US   2500u
#define TILT_SERVO_CALIB_MIN_CDEG (-9000) /* -90.00 deg */
#define TILT_SERVO_CALIB_MAX_CDEG (9000)  /*  90.00 deg */

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
  */
typedef struct
{
  Servo_t pan;
  Servo_t tilt;
  ServoJointPosition_t position;
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
  * @brief  Moves both axes of the joint together. Each angle is clamped to
  *         that joint's own commandable limit (see ServoJoint_PanLimit() /
  *         ServoJoint_TiltLimit()) before being mapped to a pulse width
  *         using the servo model's fixed full-travel calibration.
  * @param  joint     Instance, already initialized with ServoJoint_Init().
  * @param  pan_cdeg  Pan angle in hundredths of a degree.
  * @param  tilt_cdeg Tilt angle in hundredths of a degree.
  * @retval None
  */
void ServoJoint_SetAngles(ServoJoint_t *joint, int16_t pan_cdeg, int16_t tilt_cdeg);

/**
  * @brief  Disables both PWM outputs (signal lines go idle/low).
  * @note   Use for e-stop; see Servo_Stop().
  * @param  joint Instance, already initialized with ServoJoint_Init().
  * @retval None
  */
void ServoJoint_Stop(ServoJoint_t *joint);

/**
  * @brief  Re-enables both PWM outputs at their last commanded angles.
  * @param  joint Instance, already initialized with ServoJoint_Init().
  * @retval None
  */
void ServoJoint_Resume(ServoJoint_t *joint);

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
