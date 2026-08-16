/**
  * @file    servo.h
  * @brief   Minimal driver for a hobby RC servo driven by a HAL timer PWM
  *          channel (TIM3_CH1 / PA6 "SERVO_PWM" on this board).
  * @note    The bound timer must already be configured for a 50 Hz period
  *          with the counter clocked at 1 MHz (1 tick = 1 us), e.g. via
  *          MX_TIM3_Init(): Prescaler = 47, Period = 19999 on the 48 MHz
  *          APB timer clock. CCR is then just the pulse width in microseconds.
  */
#ifndef SERVO_H
#define SERVO_H

#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_PULSE_MIN_US 1000u /* pulse width at the minimum commanded angle */
#define SERVO_PULSE_MAX_US 2000u /* pulse width at the maximum commanded angle */

/**
  * @brief  Binds the driver to its timer/channel and starts the PWM output
  *         at the neutral (mid-travel) pulse width.
  * @param  htim    Timer handle already configured for 50 Hz / 1 us ticks.
  * @param  channel Timer channel the servo signal line is wired to.
  * @retval None
  */
void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/**
  * @brief  Sets the PWM pulse width, clamped to [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US].
  * @param  pulse_us Desired pulse width in microseconds.
  * @retval None
  */
void Servo_SetPulseWidthUs(uint16_t pulse_us);

/**
  * @brief  Disables the PWM output (signal line goes idle/low).
  * @note   Use for e-stop; most servos will hold their last position with
  *         no signal present, but no longer resist an external load.
  * @retval None
  */
void Servo_Stop(void);

/**
  * @brief  Re-enables the PWM output at the last commanded pulse width.
  * @retval None
  */
void Servo_Resume(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
