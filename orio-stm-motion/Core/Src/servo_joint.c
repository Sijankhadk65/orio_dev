/**
  * @file    servo_joint.c
  * @brief   2-DOF pan/tilt servo joint pair implementation (see servo_joint.h).
  */
#include "servo_joint.h"

/**
  * @brief  Per-joint commandable limits, indexed by ServoJointPosition_t.
  * @note   All three joints currently share the same limits:
  *           pan:  -135.00 .. 135.00 deg (the pan servo's full 270 deg travel)
  *           tilt:   30.00 ..  90.00 deg (a restricted window, not the
  *                   tilt servo's full 180 deg travel)
  *         Edit a row here if one joint's bracket needs a different limit
  *         from the others.
  */
static const ServoJointLimits_t kJointLimits[SERVO_JOINT_POSITION_COUNT] =
{
  /* JOINT_POS_NECK */      { .pan = { -13500, 13500 }, .tilt = { 3000, 9000 } },
  /* JOINT_POS_LEFT_ARM */  { .pan = { -13500, 13500 }, .tilt = { 3000, 9000 } },
  /* JOINT_POS_RIGHT_ARM */ { .pan = { -13500, 13500 }, .tilt = { 3000, 9000 } },
};

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

const ServoAxisLimit_t *ServoJoint_PanLimit(ServoJointPosition_t position)
{
  return &kJointLimits[position].pan;
}

const ServoAxisLimit_t *ServoJoint_TiltLimit(ServoJointPosition_t position)
{
  return &kJointLimits[position].tilt;
}

void ServoJoint_Init(ServoJoint_t *joint, ServoJointPosition_t position,
                      TIM_HandleTypeDef *pan_htim, uint32_t pan_channel,
                      TIM_HandleTypeDef *tilt_htim, uint32_t tilt_channel)
{
  joint->position = position;
  Servo_Init(&joint->pan, pan_htim, pan_channel, PAN_SERVO_PULSE_MIN_US, PAN_SERVO_PULSE_MAX_US);
  Servo_Init(&joint->tilt, tilt_htim, tilt_channel, TILT_SERVO_PULSE_MIN_US, TILT_SERVO_PULSE_MAX_US);
}

void ServoJoint_SetAngles(ServoJoint_t *joint, int16_t pan_cdeg, int16_t tilt_cdeg)
{
  const ServoAxisLimit_t *pan_limit = ServoJoint_PanLimit(joint->position);
  const ServoAxisLimit_t *tilt_limit = ServoJoint_TiltLimit(joint->position);

  pan_cdeg = clamp_cdeg(pan_cdeg, pan_limit->min_cdeg, pan_limit->max_cdeg);
  tilt_cdeg = clamp_cdeg(tilt_cdeg, tilt_limit->min_cdeg, tilt_limit->max_cdeg);

  Servo_SetPulseWidthUs(&joint->pan, pan_angle_to_pulse_us(pan_cdeg));
  Servo_SetPulseWidthUs(&joint->tilt, tilt_angle_to_pulse_us(tilt_cdeg));
}

void ServoJoint_Stop(ServoJoint_t *joint)
{
  Servo_Stop(&joint->pan);
  Servo_Stop(&joint->tilt);
}

void ServoJoint_Resume(ServoJoint_t *joint)
{
  Servo_Resume(&joint->pan);
  Servo_Resume(&joint->tilt);
}
