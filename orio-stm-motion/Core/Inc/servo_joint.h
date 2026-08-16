/**
  * @file    servo_joint.h
  * @brief   Driver for one 2-DOF joint (neck or arm) built from a pair of
  *          RC servos on an aluminum-alloy bracket:
  *            - pan  (bottom): 20 kg-cm, 270 deg servo -- left/right travel.
  *            - tilt (top):    25 kg-cm, 180 deg servo -- up/down travel.
  *          Both axes are driven through servo.c; this module just pairs two
  *          Servo_t instances, tags them with which body position they are,
  *          and moves them as one logical joint.
  * @note    Pulse-width and angle ranges below are typical for this class of
  *          270/180 deg metal-gear servo (500-2500 us full travel). Confirm
  *          against the actual datasheets and adjust if the bracket's
  *          mechanical stops are narrower than the servos' electrical range.
  */
#ifndef SERVO_JOINT_H
#define SERVO_JOINT_H

#include "servo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pan (bottom, 20 kg, 270 deg) pulse width and angle range. */
#define SERVO_JOINT_PAN_PULSE_MIN_US 500u
#define SERVO_JOINT_PAN_PULSE_MAX_US 2500u
#define SERVO_JOINT_PAN_MIN_CDEG     (-13500) /* -135.00 deg */
#define SERVO_JOINT_PAN_MAX_CDEG     (13500)  /*  135.00 deg (270 deg of travel) */

/* Tilt (top, 25 kg, 180 deg) pulse width and angle range. */
#define SERVO_JOINT_TILT_PULSE_MIN_US 500u
#define SERVO_JOINT_TILT_PULSE_MAX_US 2500u
#define SERVO_JOINT_TILT_MIN_CDEG     (-9000) /* -90.00 deg */
#define SERVO_JOINT_TILT_MAX_CDEG     (9000)  /*  90.00 deg (180 deg of travel) */

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
  *         timer/channels, tags the pair with its body position, and starts
  *         both PWM outputs at neutral (mid-travel).
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
  * @brief  Moves both axes of the joint together.
  * @param  joint     Instance, already initialized with ServoJoint_Init().
  * @param  pan_cdeg  Pan angle in hundredths of a degree, clamped to
  *                    [SERVO_JOINT_PAN_MIN_CDEG, SERVO_JOINT_PAN_MAX_CDEG].
  * @param  tilt_cdeg Tilt angle in hundredths of a degree, clamped to
  *                    [SERVO_JOINT_TILT_MIN_CDEG, SERVO_JOINT_TILT_MAX_CDEG].
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

#ifdef __cplusplus
}
#endif

#endif /* SERVO_JOINT_H */
