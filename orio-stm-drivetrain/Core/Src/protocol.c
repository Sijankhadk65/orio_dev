/**
  * @file    protocol.c
  * @brief   Framed binary UART protocol implementation (see protocol.h).
  */
#include <string.h>
#include "protocol.h"
#include "wheel.h"

typedef enum
{
  RX_WAIT_STX,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_CRC_LO,
  RX_WAIT_CRC_HI,
} RxState;

static UART_HandleTypeDef *s_huart;
static uint8_t s_rx_byte;

static RxState s_rx_state = RX_WAIT_STX;
static uint8_t s_rx_len;
static uint8_t s_rx_idx;
static uint8_t s_rx_buf[1u + PROTO_MAX_PAYLOAD]; /* CMD + PAYLOAD */
static uint16_t s_rx_crc;

/* Latched frame, filled from ISR context, drained by Protocol_Process(). */
static volatile uint8_t s_frame_ready;
static uint8_t s_frame_cmd;
static uint8_t s_frame_payload[PROTO_MAX_PAYLOAD];
static uint8_t s_frame_payload_len;

static volatile uint32_t s_last_heartbeat_tick;
static volatile uint8_t s_estopped;

/**
  * @brief  Computes CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over a buffer.
  * @param  data Pointer to the bytes to checksum.
  * @param  len  Number of bytes in data.
  * @retval The 16-bit CRC.
  */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
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
  * @brief  Builds a full [STX][LEN][CMD][PAYLOAD][CRC16] frame and transmits it.
  * @param  cmd         Command/response opcode to place in the frame.
  * @param  payload     Pointer to the payload bytes (may be NULL if payload_len is 0).
  * @param  payload_len Number of payload bytes (must be <= PROTO_MAX_PAYLOAD).
  * @retval None
  */
static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t frame[3u + PROTO_MAX_PAYLOAD + 2u];
  uint8_t len = (uint8_t)(1u + payload_len);

  frame[0] = PROTO_STX;
  frame[1] = len;
  frame[2] = cmd;
  if (payload_len > 0u)
  {
    memcpy(&frame[3], payload, payload_len);
  }

  uint16_t crc = crc16_ccitt(&frame[1], (uint16_t)(1u + len));
  frame[3u + payload_len] = (uint8_t)(crc & 0xFFu);
  frame[3u + payload_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

  HAL_UART_Transmit(s_huart, frame, (uint16_t)(3u + payload_len + 2u), HAL_MAX_DELAY);
}

/**
  * @brief  Sends a CMD_ACK frame acknowledging a successfully handled command.
  * @param  orig_cmd Opcode of the command being acknowledged.
  * @retval None
  */
static void send_ack(uint8_t orig_cmd)
{
  send_frame(CMD_ACK, &orig_cmd, 1u);
}

/**
  * @brief  Sends a CMD_NACK frame rejecting a command.
  * @param  orig_cmd Opcode of the command being rejected.
  * @param  reason   One of the ProtoNackReason values explaining the rejection.
  * @retval None
  */
static void send_nack(uint8_t orig_cmd, uint8_t reason)
{
  uint8_t p[2] = { orig_cmd, reason };
  send_frame(CMD_NACK, p, sizeof(p));
}

/**
  * @brief  Appends one side's cached telemetry to a CMD_STATUS payload.
  * @param  dst  Write position for this side's 9-byte block.
  * @param  side Which side's telemetry to append.
  * @retval None
  */
static void append_status_side(uint8_t *dst, DriveSide_t side)
{
  const VescTelemetry_t *t = Wheel_GetTelemetry(side);
  dst[0] = (uint8_t)(t->erpm & 0xFF);
  dst[1] = (uint8_t)((t->erpm >> 8) & 0xFF);
  dst[2] = (uint8_t)((t->erpm >> 16) & 0xFF);
  dst[3] = (uint8_t)((t->erpm >> 24) & 0xFF);
  dst[4] = (uint8_t)(t->current_ca & 0xFF);
  dst[5] = (uint8_t)((t->current_ca >> 8) & 0xFF);
  dst[6] = (uint8_t)(t->v_in_dv & 0xFF);
  dst[7] = (uint8_t)((t->v_in_dv >> 8) & 0xFF);
  dst[8] = t->fault_code;
  dst[9] = t->valid;
}

/**
  * @brief  Sends a CMD_STATUS frame with the current e-stop state and each
  *         wheel's last-polled telemetry.
  * @retval None
  */
static void send_status(void)
{
  uint8_t p[1u + (DRIVE_SIDE_COUNT * 10u)];

  p[0] = s_estopped;
  append_status_side(&p[1], DRIVE_LEFT);
  append_status_side(&p[11], DRIVE_RIGHT);
  send_frame(CMD_STATUS, p, sizeof(p));
}

/**
  * @brief  Validates and applies a CMD_SET_DRIVE command.
  * @param  payload Payload bytes: [left_lo][left_hi][right_lo][right_hi].
  * @param  len     Number of bytes in payload (must be exactly 4).
  * @retval None
  */
static void handle_set_drive(const uint8_t *payload, uint8_t len)
{
  if (len != 4u)
  {
    send_nack(CMD_SET_DRIVE, NACK_BAD_LENGTH);
    return;
  }

  int16_t left = (int16_t)(payload[0] | ((uint16_t)payload[1] << 8));
  int16_t right = (int16_t)(payload[2] | ((uint16_t)payload[3] << 8));

  if ((left < -1000) || (left > 1000) || (right < -1000) || (right > 1000))
  {
    send_nack(CMD_SET_DRIVE, NACK_OUT_OF_RANGE);
    return;
  }
  if (s_estopped)
  {
    send_nack(CMD_SET_DRIVE, NACK_ESTOPPED);
    return;
  }

  Wheel_SetSpeeds(left, right);
  send_ack(CMD_SET_DRIVE);
}

/**
  * @brief  Dispatches a fully received, CRC-valid frame to its command handler.
  * @param  cmd         Command opcode.
  * @param  payload     Pointer to the command's payload bytes.
  * @param  payload_len Number of payload bytes.
  * @retval None
  */
static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  switch (cmd)
  {
    case CMD_HEARTBEAT:
      s_last_heartbeat_tick = HAL_GetTick();
      if (s_estopped)
      {
        s_estopped = 0u;
        Wheel_Resume();
      }
      send_ack(cmd);
      break;

    case CMD_SET_DRIVE:
      handle_set_drive(payload, payload_len);
      break;

    case CMD_STOP:
      s_estopped = 1u;
      Wheel_Stop();
      send_ack(cmd);
      break;

    case CMD_GET_STATUS:
      send_status();
      break;

    default:
      send_nack(cmd, NACK_UNKNOWN_CMD);
      break;
  }
}

/**
  * @brief  Advances the frame-parser state machine by one received byte.
  * @note   Runs in ISR context (called from HAL_UART_RxCpltCallback) -- keep it
  *         short, no blocking calls. Malformed/bad-CRC frames are silently
  *         dropped and the parser resyncs on the next STX byte. Latches the
  *         decoded frame into the module-level "ready" buffer for
  *         Protocol_Process() to consume.
  * @param  byte The newly received byte.
  * @retval None
  */
static void on_byte_received(uint8_t byte)
{
  switch (s_rx_state)
  {
    case RX_WAIT_STX:
      if (byte == PROTO_STX)
      {
        s_rx_state = RX_WAIT_LEN;
      }
      break;

    case RX_WAIT_LEN:
      s_rx_len = byte;
      if ((s_rx_len == 0u) || (s_rx_len > sizeof(s_rx_buf)))
      {
        s_rx_state = RX_WAIT_STX;
        break;
      }
      s_rx_idx = 0u;
      s_rx_state = RX_WAIT_DATA;
      break;

    case RX_WAIT_DATA:
      s_rx_buf[s_rx_idx++] = byte;
      if (s_rx_idx == s_rx_len)
      {
        s_rx_state = RX_WAIT_CRC_LO;
      }
      break;

    case RX_WAIT_CRC_LO:
      s_rx_crc = byte;
      s_rx_state = RX_WAIT_CRC_HI;
      break;

    case RX_WAIT_CRC_HI:
      s_rx_crc |= (uint16_t)((uint16_t)byte << 8);
      s_rx_state = RX_WAIT_STX;

      {
        uint8_t crc_input[1u + sizeof(s_rx_buf)];
        crc_input[0] = s_rx_len;
        memcpy(&crc_input[1], s_rx_buf, s_rx_len);

        if ((crc16_ccitt(crc_input, (uint16_t)(1u + s_rx_len)) == s_rx_crc) && !s_frame_ready)
        {
          s_frame_cmd = s_rx_buf[0];
          s_frame_payload_len = (uint8_t)(s_rx_len - 1u);
          memcpy(s_frame_payload, &s_rx_buf[1], s_frame_payload_len);
          s_frame_ready = 1u;
        }
      }
      break;

    default:
      s_rx_state = RX_WAIT_STX;
      break;
  }
}

/**
  * @brief  Initializes the protocol layer and arms the first byte-wise UART receive.
  * @param  huart Pointer to the UART handle the protocol will run on.
  * @retval None
  */
void Protocol_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_rx_state = RX_WAIT_STX;
  s_frame_ready = 0u;
  s_estopped = 1u; /* stay stopped until the first heartbeat arrives */
  s_last_heartbeat_tick = HAL_GetTick();
  Wheel_Stop();

  HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

/**
  * @brief  Feeds one received byte into the frame parser and re-arms the next receive.
  * @param  huart UART handle passed through from HAL_UART_RxCpltCallback.
  * @retval None
  */
void Protocol_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != s_huart->Instance)
  {
    return;
  }
  on_byte_received(s_rx_byte);
  HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

void Protocol_Process(void)
{
  if (!s_estopped && ((HAL_GetTick() - s_last_heartbeat_tick) > PROTO_HEARTBEAT_TIMEOUT_MS))
  {
    s_estopped = 1u;
    Wheel_Stop();
  }

  if (!s_frame_ready)
  {
    return;
  }

  handle_frame(s_frame_cmd, s_frame_payload, s_frame_payload_len);
  s_frame_ready = 0u;
}
