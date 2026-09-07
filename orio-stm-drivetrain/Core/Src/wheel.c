/**
  * @file    wheel.c
  * @brief   Differential-drive wrapper implementation (see wheel.h).
  */
#include "wheel.h"

static Vesc_t s_escs[WHEEL_SIDE_COUNT];
static int16_t s_commanded_permille[WHEEL_SIDE_COUNT];
static VescTelemetry_t s_telemetry[WHEEL_SIDE_COUNT];
static WheelSide_t s_next_poll_side = WHEEL_LEFT;

void Wheel_Init(UART_HandleTypeDef *huart_left, UART_HandleTypeDef *huart_right)
{
  Vesc_Init(&s_escs[WHEEL_LEFT], huart_left);
  Vesc_Init(&s_escs[WHEEL_RIGHT], huart_right);
  s_commanded_permille[WHEEL_LEFT] = 0;
  s_commanded_permille[WHEEL_RIGHT] = 0;
}

void Wheel_SetSpeeds(int16_t left_permille, int16_t right_permille)
{
  s_commanded_permille[WHEEL_LEFT] = left_permille;
  s_commanded_permille[WHEEL_RIGHT] = right_permille;
  Vesc_SetDuty(&s_escs[WHEEL_LEFT], left_permille);
  Vesc_SetDuty(&s_escs[WHEEL_RIGHT], right_permille);
}

void Wheel_Stop(void)
{
  /* Clears the commanded speeds too, not just the ESC outputs -- so a
   * later Wheel_Resume() (from a heartbeat clearing the e-stop) comes back
   * stopped and waits for a fresh command, instead of silently resuming
   * whatever motion was in progress when the stop happened. */
  s_commanded_permille[WHEEL_LEFT] = 0;
  s_commanded_permille[WHEEL_RIGHT] = 0;
  Vesc_SetDuty(&s_escs[WHEEL_LEFT], 0);
  Vesc_SetDuty(&s_escs[WHEEL_RIGHT], 0);
}

void Wheel_Resume(void)
{
  Vesc_SetDuty(&s_escs[WHEEL_LEFT], s_commanded_permille[WHEEL_LEFT]);
  Vesc_SetDuty(&s_escs[WHEEL_RIGHT], s_commanded_permille[WHEEL_RIGHT]);
}

void Wheel_PollTelemetry(void)
{
  /* One side per call, alternating -- polling both back to back doubles
   * the worst-case block on the main loop (which also has to service the
   * Jetson-link heartbeat watchdog), for no benefit callers actually need.
   * See the worst-case-timing note on VESC_UART_TIMEOUT_MS in vesc.c. */
  Vesc_PollValues(&s_escs[s_next_poll_side], &s_telemetry[s_next_poll_side]);
  s_next_poll_side = (s_next_poll_side == WHEEL_LEFT) ? WHEEL_RIGHT : WHEEL_LEFT;
}

const VescTelemetry_t *Wheel_GetTelemetry(WheelSide_t side)
{
  return &s_telemetry[side];
}
