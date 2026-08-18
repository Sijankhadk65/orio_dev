/**
  * @file    vesc.h
  * @brief   Minimal VESC UART driver for one FSESC: enough to command duty
  *          cycle and poll basic telemetry over usart_mux.h's shared link.
  *
  * Packet layout (short packets only -- our payloads are always <256 bytes):
  *   [0x02][LEN][CMD][PAYLOAD 0..LEN-2][CRC16_HI][CRC16_LO][0x03]
  *   - LEN   : 1 byte, number of bytes in CMD+PAYLOAD (i.e. 1 + payload length)
  *   - CRC16 : CRC-16/XMODEM (poly 0x1021, init 0x0000) over CMD,PAYLOAD
  * This is VESC's own wire format (distinct from protocol.h's Jetson-facing
  * framing, which uses CRC-16/CCITT-FALSE with a 0xFFFF init).
  */
#ifndef VESC_H
#define VESC_H

#include <stdint.h>
#include "usart_mux.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  One FSESC, addressed through the shared USART1 mux.
  */
typedef struct
{
  DriveSide_t side;
} Vesc_t;

/**
  * @brief  Telemetry decoded from a COMM_GET_VALUES reply.
  * @note   Only the fields present in every VESC firmware version since the
  *         command was introduced are parsed (temps/currents/duty/rpm/v_in/
  *         fault_code) -- everything COMM_GET_VALUES reports past fault_code
  *         (pid_pos, controller_id, extra NTC channels, ...) has shifted
  *         across firmware versions, so it's deliberately left unread.
  */
typedef struct
{
  int32_t erpm;         /* electrical RPM, divide by motor pole pairs for mechanical RPM */
  int16_t current_ca;   /* motor current, centi-amps (actual A = value / 100) */
  int16_t v_in_dv;      /* input voltage, deci-volts (actual V = value / 10) */
  uint8_t fault_code;   /* raw mc_fault_code from the FSESC */
  uint8_t valid;        /* 1 if this poll got a CRC-valid reply, 0 on timeout/garbage */
} VescTelemetry_t;

/**
  * @brief  Binds a Vesc_t to a mux side. Call after UsartMux_Init().
  * @param  esc  Instance to initialize.
  * @param  side Which mux side (and therefore which physical FSESC) this is.
  * @retval None
  */
void Vesc_Init(Vesc_t *esc, DriveSide_t side);

/**
  * @brief  Sends a COMM_SET_DUTY command.
  * @note   Fire-and-forget -- FSESC does not ack a SET_DUTY. If the mux link
  *         is down this silently does nothing; rely on the FSESC's own
  *         UART-timeout safety cutoff (~1s) as the last-resort backstop, and
  *         on Wheel_Stop()/the Jetson heartbeat as the primary one.
  * @param  esc            Instance, already initialized with Vesc_Init().
  * @param  duty_permille  Signed duty, -1000..1000 for -100.0%..100.0%.
  * @retval None
  */
void Vesc_SetDuty(Vesc_t *esc, int16_t duty_permille);

/**
  * @brief  Sends a COMM_GET_VALUES request and blocks for the reply.
  * @param  esc Instance, already initialized with Vesc_Init().
  * @param  out Filled with the decoded telemetry; out->valid is 0 if the
  *             request timed out or the reply failed its CRC check.
  * @retval None
  */
void Vesc_PollValues(Vesc_t *esc, VescTelemetry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* VESC_H */
