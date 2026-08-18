/**
  * @file    vesc.c
  * @brief   VESC UART driver implementation (see vesc.h).
  */
#include <string.h>
#include "vesc.h"

#define VESC_PKT_START        0x02u
#define VESC_PKT_END          0x03u
#define VESC_UART_TIMEOUT_MS  25u

/* COMM_PACKET_ID values this driver needs, per VESC's datatypes.h. */
typedef enum
{
  VESC_COMM_GET_VALUES = 4,
  VESC_COMM_SET_DUTY   = 5,
} VescCommPacketId;

/* Byte offsets into a COMM_GET_VALUES reply payload (payload[0] is the
 * COMM_GET_VALUES id itself; values start at payload[1]: temp_mos(i16),
 * temp_motor(i16), then the fields below). Stable across firmware versions
 * -- see the @note in vesc.h. */
#define VESC_VAL_OFF_CURRENT_MOTOR 5u  /* int32, x100 A */
#define VESC_VAL_OFF_DUTY_NOW      21u /* int16, x1000 */
#define VESC_VAL_OFF_RPM           23u /* int32 (erpm) */
#define VESC_VAL_OFF_V_IN          27u /* int16, x10 V */
#define VESC_VAL_OFF_FAULT_CODE    53u /* uint8, after amp/watt-hours + tachometer counters */
#define VESC_VAL_MIN_PAYLOAD_LEN   54u /* payload[0..53] must be present to read fault_code */

#define VESC_RX_MAX_PAYLOAD 128u

/**
  * @brief  Computes CRC-16/XMODEM (poly 0x1021, init 0x0000) -- VESC's own
  *         packet checksum, distinct from protocol.c's CCITT-FALSE variant.
  * @param  data Pointer to the bytes to checksum.
  * @param  len  Number of bytes in data.
  * @retval The 16-bit CRC.
  */
static uint16_t vesc_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0x0000u;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8u; b++)
    {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/**
  * @brief  Builds a short VESC packet ([0x02][LEN][CMD][PAYLOAD][CRC16][0x03]).
  * @param  frame       Output buffer, must hold payload_len + 6 bytes.
  * @param  cmd         COMM_PACKET_ID to send.
  * @param  payload     Payload bytes (may be NULL if payload_len is 0).
  * @param  payload_len Number of payload bytes.
  * @retval Total number of bytes written to frame.
  */
static uint16_t vesc_build_packet(uint8_t *frame, uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t crc_input[1u + 8u]; /* cmd + payload; 8 bytes covers every payload this driver sends */
  crc_input[0] = cmd;
  if (payload_len > 0u)
  {
    memcpy(&crc_input[1], payload, payload_len);
  }
  uint16_t crc = vesc_crc16(crc_input, (uint16_t)(1u + payload_len));

  uint16_t idx = 0;
  frame[idx++] = VESC_PKT_START;
  frame[idx++] = (uint8_t)(1u + payload_len);
  frame[idx++] = cmd;
  if (payload_len > 0u)
  {
    memcpy(&frame[idx], payload, payload_len);
    idx = (uint16_t)(idx + payload_len);
  }
  frame[idx++] = (uint8_t)(crc >> 8);
  frame[idx++] = (uint8_t)(crc & 0xFFu);
  frame[idx++] = VESC_PKT_END;
  return idx;
}

void Vesc_Init(Vesc_t *esc, DriveSide_t side)
{
  esc->side = side;
}

void Vesc_SetDuty(Vesc_t *esc, int16_t duty_permille)
{
  if (duty_permille > 1000)
  {
    duty_permille = 1000;
  }
  else if (duty_permille < -1000)
  {
    duty_permille = -1000;
  }

  /* VESC scales duty by 100000 for the full -1.0..1.0 range; our permille
   * input is already duty_fraction * 1000, so multiply by 100 more. */
  int32_t raw = (int32_t)duty_permille * 100;
  uint8_t payload[4] = {
    (uint8_t)((uint32_t)raw >> 24), (uint8_t)((uint32_t)raw >> 16),
    (uint8_t)((uint32_t)raw >> 8), (uint8_t)((uint32_t)raw)
  };

  uint8_t frame[4u + 6u];
  uint16_t len = vesc_build_packet(frame, VESC_COMM_SET_DUTY, payload, sizeof(payload));
  UsartMux_Transmit(esc->side, frame, len, VESC_UART_TIMEOUT_MS);
}

void Vesc_PollValues(Vesc_t *esc, VescTelemetry_t *out)
{
  memset(out, 0, sizeof(*out));

  uint8_t frame[0u + 6u];
  uint16_t len = vesc_build_packet(frame, VESC_COMM_GET_VALUES, NULL, 0u);
  if (UsartMux_Transmit(esc->side, frame, len, VESC_UART_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  uint8_t start;
  uint8_t payload_len;
  uint8_t payload[VESC_RX_MAX_PAYLOAD];
  uint8_t crc_bytes[2];
  uint8_t end;

  if (UsartMux_Receive(esc->side, &start, 1u, VESC_UART_TIMEOUT_MS) != HAL_OK || start != VESC_PKT_START)
  {
    return;
  }
  if (UsartMux_Receive(esc->side, &payload_len, 1u, VESC_UART_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }
  if (payload_len < VESC_VAL_MIN_PAYLOAD_LEN || payload_len > sizeof(payload))
  {
    return;
  }
  if (UsartMux_Receive(esc->side, payload, payload_len, VESC_UART_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }
  if (UsartMux_Receive(esc->side, crc_bytes, 2u, VESC_UART_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }
  if (UsartMux_Receive(esc->side, &end, 1u, VESC_UART_TIMEOUT_MS) != HAL_OK || end != VESC_PKT_END)
  {
    return;
  }

  uint16_t expected_crc = (uint16_t)(((uint16_t)crc_bytes[0] << 8) | crc_bytes[1]);
  if (vesc_crc16(payload, payload_len) != expected_crc)
  {
    return;
  }
  if (payload[0] != VESC_COMM_GET_VALUES)
  {
    return;
  }

  out->current_ca = (int16_t)(((uint32_t)payload[VESC_VAL_OFF_CURRENT_MOTOR] << 24)
                             | ((uint32_t)payload[VESC_VAL_OFF_CURRENT_MOTOR + 1] << 16)
                             | ((uint32_t)payload[VESC_VAL_OFF_CURRENT_MOTOR + 2] << 8)
                             | (uint32_t)payload[VESC_VAL_OFF_CURRENT_MOTOR + 3]) / 100;
  (void)VESC_VAL_OFF_DUTY_NOW; /* duty_now not currently surfaced; offset kept for reference */
  out->erpm = (int32_t)(((uint32_t)payload[VESC_VAL_OFF_RPM] << 24)
                       | ((uint32_t)payload[VESC_VAL_OFF_RPM + 1] << 16)
                       | ((uint32_t)payload[VESC_VAL_OFF_RPM + 2] << 8)
                       | (uint32_t)payload[VESC_VAL_OFF_RPM + 3]);
  out->v_in_dv = (int16_t)(((uint16_t)payload[VESC_VAL_OFF_V_IN] << 8) | payload[VESC_VAL_OFF_V_IN + 1]);
  out->fault_code = payload[VESC_VAL_OFF_FAULT_CODE];
  out->valid = 1u;
}
