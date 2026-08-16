/**
  * @file    servo_joint.c
  * @brief   2-DOF pan/tilt servo joint pair implementation (see servo_joint.h).
  */
#include "servo_joint.h"

/**
  * @brief  Maps a commanded angle to a servo PWM pulse width.
  * @param  angle_cdeg   Angle in hundredths of a degree, within [min_cdeg, max_cdeg].
  * @param  min_cdeg     Angle at pulse_min_us.
  * @param  max_cdeg     Angle at pulse_max_us.
  * @param  pulse_min_us Pulse width at min_cdeg.
  * @param  pulse_max_us Pulse width at max_cdeg.
  * @retval Pulse width in microseconds, within [pulse_min_us, pulse_max_us].
  */
static uint16_t angle_to_pulse_us(int16_t angle_cdeg, int16_t min_cdeg, int16_t max_cdeg,
                                   uint16_t pulse_min_us, uint16_t pulse_max_us)
{
  int32_t span_cdeg = max_cdeg - min_cdeg;
  int32_t span_us = pulse_max_us - pulse_min_us;
  int32_t offset_cdeg = angle_cdeg - min_cdeg;

  return (uint16_t)(pulse_min_us + (offset_cdeg * span_us) / span_cdeg);
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

void ServoJoint_Init(ServoJoint_t *joint, ServoJointPosition_t position,
                      TIM_HandleTypeDef *pan_htim, uint32_t pan_channel,
                      TIM_HandleTypeDef *tilt_htim, uint32_t tilt_channel)
{
  joint->position = position;
  Servo_Init(&joint->pan, pan_htim, pan_channel,
             SERVO_JOINT_PAN_PULSE_MIN_US, SERVO_JOINT_PAN_PULSE_MAX_US);
  Servo_Init(&joint->tilt, tilt_htim, tilt_channel,
             SERVO_JOINT_TILT_PULSE_MIN_US, SERVO_JOINT_TILT_PULSE_MAX_US);
}

void ServoJoint_SetAngles(ServoJoint_t *joint, int16_t pan_cdeg, int16_t tilt_cdeg)
{
  pan_cdeg = clamp_cdeg(pan_cdeg, SERVO_JOINT_PAN_MIN_CDEG, SERVO_JOINT_PAN_MAX_CDEG);
  tilt_cdeg = clamp_cdeg(tilt_cdeg, SERVO_JOINT_TILT_MIN_CDEG, SERVO_JOINT_TILT_MAX_CDEG);

  Servo_SetPulseWidthUs(&joint->pan,
                         angle_to_pulse_us(pan_cdeg, SERVO_JOINT_PAN_MIN_CDEG, SERVO_JOINT_PAN_MAX_CDEG,
                                            SERVO_JOINT_PAN_PULSE_MIN_US, SERVO_JOINT_PAN_PULSE_MAX_US));
  Servo_SetPulseWidthUs(&joint->tilt,
                         angle_to_pulse_us(tilt_cdeg, SERVO_JOINT_TILT_MIN_CDEG, SERVO_JOINT_TILT_MAX_CDEG,
                                            SERVO_JOINT_TILT_PULSE_MIN_US, SERVO_JOINT_TILT_PULSE_MAX_US));
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
