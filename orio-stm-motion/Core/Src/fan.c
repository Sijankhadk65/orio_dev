/**
  * @file    fan.c
  * @brief   4-pin PWM fan driver implementation (see fan.h).
  */
#include "fan.h"

static TIM_HandleTypeDef *s_htim;
static uint32_t s_channel;
static uint8_t s_percent;

static void apply_percent(uint8_t percent)
{
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, ((uint32_t)percent * FAN_PWM_PERIOD_TICKS) / 100u);
}

void Fan_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  s_htim = htim;
  s_channel = channel;
  s_percent = 0u;

  apply_percent(s_percent);
  HAL_TIM_PWM_Start(s_htim, s_channel);
}

void Fan_SetSpeedPercent(uint8_t percent)
{
  if (percent > 100u)
  {
    percent = 100u;
  }

  s_percent = percent;
  apply_percent(s_percent);
}

void Fan_Stop(void)
{
  apply_percent(0u);
}

void Fan_Resume(void)
{
  apply_percent(s_percent);
}
