/**
  * @file    servo.h
  * @brief   Minimal driver for a hobby RC servo driven by a HAL timer PWM
  *          channel.
  * @note    The bound timer must already be configured for a 50 Hz period
  *          with the counter clocked at 1 MHz (1 tick = 1 us), e.g. via
  *          MX_TIM3_Init(): Prescaler = 47, Period = 19999 on the 48 MHz
  *          APB timer clock. CCR is then just the pulse width in microseconds.
  * @note    Instance-based: each Servo_t is one physical servo on one timer
  *          channel, so several can share a timer on different channels (see
  *          servo_joint.h, which drives a pan/tilt pair this way) or be
  *          spread across different timers entirely.
  */
#ifndef SERVO_H
#define SERVO_H

#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_PULSE_MIN_US 1000u /* typical hobby-servo pulse width at min angle */
#define SERVO_PULSE_MAX_US 2000u /* typical hobby-servo pulse width at max angle */

/**
  * @brief  Driver state for one servo. Zero-initialize (or don't bother --
  *         Servo_Init() sets every field) and pass by pointer to the rest
  *         of the API.
  */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  uint16_t pulse_min_us; /* pulse width at this servo's minimum commanded angle */
  uint16_t pulse_max_us; /* pulse width at this servo's maximum commanded angle */
  uint16_t pulse_us;     /* last commanded pulse width, for Servo_Resume() */
} Servo_t;

/**
  * @brief  Binds a driver instance to its timer/channel and starts the PWM
  *         output at the neutral (mid-travel) pulse width.
  * @param  servo        Instance to initialize.
  * @param  htim         Timer handle already configured for 50 Hz / 1 us ticks.
  * @param  channel      Timer channel the servo signal line is wired to.
  * @param  pulse_min_us Pulse width, in microseconds, at this servo's
  *                       minimum commanded angle (its datasheet range, not
  *                       necessarily SERVO_PULSE_MIN_US).
  * @param  pulse_max_us Pulse width, in microseconds, at this servo's
  *                       maximum commanded angle.
  * @retval None
  */
void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint16_t pulse_min_us, uint16_t pulse_max_us);

/**
  * @brief  Sets the PWM pulse width, clamped to
  *         [servo->pulse_min_us, servo->pulse_max_us].
  * @param  servo    Instance, already initialized with Servo_Init().
  * @param  pulse_us Desired pulse width in microseconds.
  * @retval None
  */
void Servo_SetPulseWidthUs(Servo_t *servo, uint16_t pulse_us);

/**
  * @brief  Disables the PWM output (signal line goes idle/low).
  * @note   Use for e-stop; most servos will hold their last position with
  *         no signal present, but no longer resist an external load.
  * @param  servo Instance, already initialized with Servo_Init().
  * @retval None
  */
void Servo_Stop(Servo_t *servo);

/**
  * @brief  Re-enables the PWM output at the last commanded pulse width.
  * @param  servo Instance, already initialized with Servo_Init().
  * @retval None
  */
void Servo_Resume(Servo_t *servo);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
