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
  * @brief  Which wheel -- and which dedicated UART/FSESC -- an index refers to.
  */
typedef enum
{
  WHEEL_LEFT = 0,
  WHEEL_RIGHT = 1,
  WHEEL_SIDE_COUNT,
} WheelSide_t;

/**
  * @brief  Binds both FSESCs to their own dedicated UARTs.
  * @param  huart_left  UART handle wired to the left FSESC's COMM port.
  * @param  huart_right UART handle wired to the right FSESC's COMM port.
  * @retval None
  */
void Wheel_Init(UART_HandleTypeDef *huart_left, UART_HandleTypeDef *huart_right);

/**
  * @brief  Commands both wheels' duty cycle.
  * @note   Latches the commanded values so Wheel_Resume() can reapply them
  *         if the link is merely re-armed without an intervening stop (e.g.
  *         a routine heartbeat that never actually lapsed). Does not itself
  *         check e-stop state -- callers (protocol.c) are responsible for
  *         gating this on it.
  * @param  left_permille  Left duty, -1000..1000 for -100.0%..100.0%.
  * @param  right_permille Right duty, -1000..1000 for -100.0%..100.0%.
  * @retval None
  */
void Wheel_SetSpeeds(int16_t left_permille, int16_t right_permille);

/**
  * @brief  Immediately commands both wheels to 0 duty (active brake to stop,
  *         not just "stop sending"), and clears the latched commanded
  *         speeds to 0 as well.
  * @note   That second part matters: it's what makes Wheel_Resume() come
  *         back stopped and wait for a fresh command after any stop
  *         (CMD_STOP or a heartbeat-timeout e-stop), instead of silently
  *         resuming whatever motion was in progress when the stop happened.
  * @retval None
  */
void Wheel_Stop(void);

/**
  * @brief  Re-sends the last-commanded speeds (0 if none were ever set, or
  *         if the most recent event was a Wheel_Stop()).
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
const VescTelemetry_t *Wheel_GetTelemetry(WheelSide_t side);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_H */
