/**
  * @file    servo.c
  * @brief   Minimal HAL-timer-PWM servo driver implementation (see servo.h).
  */
#include "servo.h"

static TIM_HandleTypeDef *s_htim;
static uint32_t s_channel;
static uint16_t s_pulse_us = (SERVO_PULSE_MIN_US + SERVO_PULSE_MAX_US) / 2u;

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  s_htim = htim;
  s_channel = channel;

  __HAL_TIM_SET_COMPARE(s_htim, s_channel, s_pulse_us);
  HAL_TIM_PWM_Start(s_htim, s_channel);
}

void Servo_SetPulseWidthUs(uint16_t pulse_us)
{
  if (pulse_us < SERVO_PULSE_MIN_US)
  {
    pulse_us = SERVO_PULSE_MIN_US;
  }
  else if (pulse_us > SERVO_PULSE_MAX_US)
  {
    pulse_us = SERVO_PULSE_MAX_US;
  }

  s_pulse_us = pulse_us;
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, s_pulse_us);
}

void Servo_Stop(void)
{
  HAL_TIM_PWM_Stop(s_htim, s_channel);
}

void Servo_Resume(void)
{
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, s_pulse_us);
  HAL_TIM_PWM_Start(s_htim, s_channel);
}
