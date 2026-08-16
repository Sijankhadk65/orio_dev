/**
  * @file    fan.h
  * @brief   Driver for a 4-pin PC-style PWM fan used as chassis exhaust
  *          (TIM14_CH1 / PA4 "FAN_PWM" on this board).
  * @note    The bound timer must already be configured for the standard
  *          25 kHz fan PWM frequency (Prescaler = 0, Period = 1919 on the
  *          48 MHz APB timer clock -> 48 MHz / 1920 = 25 kHz), so CCR is
  *          just a fraction of FAN_PWM_PERIOD_TICKS.
  * @note    Most 4-pin fans won't spin below roughly 20-30% duty and some
  *          treat very low nonzero duty as "stop" -- check your fan's specs
  *          if it stalls at low commanded speeds.
  */
#ifndef FAN_H
#define FAN_H

#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FAN_PWM_PERIOD_TICKS 1920u

/**
  * @brief  Binds the driver to its timer/channel and starts the PWM output
  *         stopped (0% duty).
  * @param  htim    Timer handle already configured for 25 kHz PWM.
  * @param  channel Timer channel the fan's PWM control line is wired to.
  * @retval None
  */
void Fan_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/**
  * @brief  Sets the fan speed as a PWM duty cycle.
  * @param  percent Desired duty cycle, clamped to [0, 100].
  * @retval None
  */
void Fan_SetSpeedPercent(uint8_t percent);

/**
  * @brief  Forces the fan output to 0% duty without forgetting the last
  *         commanded speed.
  * @note   Use for e-stop.
  * @retval None
  */
void Fan_Stop(void);

/**
  * @brief  Restores the fan output to the last speed set via
  *         Fan_SetSpeedPercent() (0% if none was ever set).
  * @retval None
  */
void Fan_Resume(void);

#ifdef __cplusplus
}
#endif

#endif /* FAN_H */
