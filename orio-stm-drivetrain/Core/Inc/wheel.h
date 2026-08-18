/**
  * @file    wheel.h
  * @brief   Differential-drive wrapper over two Vesc_t instances (left/right
  *          hub motor FSESCs), mirroring servo_joint.c's stop/resume pattern
  *          from the motion board so e-stop handling reads the same way.
  */
#ifndef WHEEL_H
#define WHEEL_H

#include "vesc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Binds both FSESCs through the mux. Call after UsartMux_Init().
  * @retval None
  */
void Wheel_Init(void);

/**
  * @brief  Commands both wheels' duty cycle.
  * @note   Latches the commanded values so Wheel_Resume() can reapply them
  *         after a Wheel_Stop(). Does not itself check e-stop state --
  *         callers (protocol.c) are responsible for gating this on it.
  * @param  left_permille  Left duty, -1000..1000 for -100.0%..100.0%.
  * @param  right_permille Right duty, -1000..1000 for -100.0%..100.0%.
  * @retval None
  */
void Wheel_SetSpeeds(int16_t left_permille, int16_t right_permille);

/**
  * @brief  Immediately commands both wheels to 0 duty (active brake to stop,
  *         not just "stop sending"). Last-commanded speeds are preserved for
  *         Wheel_Resume().
  * @retval None
  */
void Wheel_Stop(void);

/**
  * @brief  Re-sends the last-commanded speeds (0 if none were ever set).
  * @retval None
  */
void Wheel_Resume(void);

/**
  * @brief  Polls fresh telemetry from both FSESCs (two blocking VESC UART
  *         transactions) and caches it for Wheel_GetTelemetry().
  * @note   Call periodically from the main loop, not from an ISR.
  * @retval None
  */
void Wheel_PollTelemetry(void);

/**
  * @brief  Returns the telemetry from the most recent Wheel_PollTelemetry().
  * @param  side Which wheel.
  * @retval Pointer to the cached VescTelemetry_t (owned by wheel.c, valid
  *         until the next Wheel_PollTelemetry() call).
  */
const VescTelemetry_t *Wheel_GetTelemetry(DriveSide_t side);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_H */
