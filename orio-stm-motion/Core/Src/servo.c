/**
  * @file    servo.c
  * @brief   Minimal HAL-timer-PWM servo driver implementation (see servo.h).
  */
#include "servo.h"

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint16_t pulse_min_us, uint16_t pulse_max_us)
{
  servo->htim = htim;
  servo->channel = channel;
  servo->pulse_min_us = pulse_min_us;
  servo->pulse_max_us = pulse_max_us;
  servo->pulse_us = (uint16_t)((pulse_min_us + pulse_max_us) / 2u);

  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
  HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

void Servo_SetPulseWidthUs(Servo_t *servo, uint16_t pulse_us)
{
  if (pulse_us < servo->pulse_min_us)
  {
    pulse_us = servo->pulse_min_us;
  }
  else if (pulse_us > servo->pulse_max_us)
  {
    pulse_us = servo->pulse_max_us;
  }

  servo->pulse_us = pulse_us;
  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
}

void Servo_Stop(Servo_t *servo)
{
  HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}

void Servo_Resume(Servo_t *servo)
{
  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
  HAL_TIM_PWM_Start(servo->htim, servo->channel);
}
