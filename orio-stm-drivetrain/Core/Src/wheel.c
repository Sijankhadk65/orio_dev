/**
  * @file    wheel.c
  * @brief   Differential-drive wrapper implementation (see wheel.h).
  */
#include "wheel.h"

static Vesc_t s_escs[DRIVE_SIDE_COUNT];
static int16_t s_commanded_permille[DRIVE_SIDE_COUNT];
static VescTelemetry_t s_telemetry[DRIVE_SIDE_COUNT];

void Wheel_Init(void)
{
  Vesc_Init(&s_escs[DRIVE_LEFT], DRIVE_LEFT);
  Vesc_Init(&s_escs[DRIVE_RIGHT], DRIVE_RIGHT);
  s_commanded_permille[DRIVE_LEFT] = 0;
  s_commanded_permille[DRIVE_RIGHT] = 0;
}

void Wheel_SetSpeeds(int16_t left_permille, int16_t right_permille)
{
  s_commanded_permille[DRIVE_LEFT] = left_permille;
  s_commanded_permille[DRIVE_RIGHT] = right_permille;
  Vesc_SetDuty(&s_escs[DRIVE_LEFT], left_permille);
  Vesc_SetDuty(&s_escs[DRIVE_RIGHT], right_permille);
}

void Wheel_Stop(void)
{
  Vesc_SetDuty(&s_escs[DRIVE_LEFT], 0);
  Vesc_SetDuty(&s_escs[DRIVE_RIGHT], 0);
}

void Wheel_Resume(void)
{
  Vesc_SetDuty(&s_escs[DRIVE_LEFT], s_commanded_permille[DRIVE_LEFT]);
  Vesc_SetDuty(&s_escs[DRIVE_RIGHT], s_commanded_permille[DRIVE_RIGHT]);
}

void Wheel_PollTelemetry(void)
{
  Vesc_PollValues(&s_escs[DRIVE_LEFT], &s_telemetry[DRIVE_LEFT]);
  Vesc_PollValues(&s_escs[DRIVE_RIGHT], &s_telemetry[DRIVE_RIGHT]);
}

const VescTelemetry_t *Wheel_GetTelemetry(DriveSide_t side)
{
  return &s_telemetry[side];
}
