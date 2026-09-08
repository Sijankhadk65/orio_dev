/**
  * @file    protocol.c
  * @brief   Framed binary UART protocol implementation (see protocol.h).
  */
#include <string.h>
#include "protocol.h"
#include "servo_joint.h"
#include "fan.h"
#include "argb.h"

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

/* Physical joints, and their last-commanded angles, indexed by
 * ServoJointPosition_t. s_joints[] holds NULL for any position without
 * hardware bound yet (Protocol_BindJoint() never called for it), which is
 * still latched here for status reporting. */
static ServoJoint_t *s_joints[PROTO_JOINT_COUNT];
static int16_t s_pan_cdeg[PROTO_JOINT_COUNT];
static int16_t s_tilt_cdeg[PROTO_JOINT_COUNT];

/* Reset ("home") pose per joint position, on the same centered command
 * scale as CMD_MOVE_JOINT_TO: the vendor scale, pan 0..270 and tilt
 * 0..180, measured from each servo's own zero end (see servo_joint.h).
 * Each joint's tilt homes to the midpoint of its own window: 9000 cdeg
 * (90.00 deg -- level) for the neck's 85..95 window, 6000 cdeg (60.00
 * deg) for the arms' 30..90 one. Keep these in step with kJointLimits --
 * a stale value here is silently clamped, not flagged, so the joint would
 * just quietly home somewhere other than its midpoint.
 *   neck pan: 180.00 deg on the pan servo's 0..270 scale -> 18000 cdeg.
 * left-arm/right-arm pan default to pan mid-travel (13500 cdeg = 135.00
 * deg) until their actual reset pose is measured on the physical
 * brackets and tuned like neck's.
 * handle_reset_joints() still clamps these against each joint's live
 * limit before latching/applying, so a future limit change can't desync
 * CMD_GET_STATUS from what's actually commanded to hardware. */
static const int16_t s_reset_pan_cdeg[PROTO_JOINT_COUNT]  = { 18000, 13500, 13500 }; /* neck, left-arm, right-arm */
static const int16_t s_reset_tilt_cdeg[PROTO_JOINT_COUNT] = { 9000, 6000, 6000 };

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
  * @param  payload_len Number of payload bytes (must leave room for STX/LEN/CMD/CRC
  *                     within the frame buffer, i.e. <= PROTO_MAX_PAYLOAD).
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
  * @brief  Sends a CMD_STATUS frame with the current e-stop state and every
  *         joint's last-commanded pan/tilt angles.
  * @note   These are the commanded TARGET angles, not live positions -- a
  *         joint polled mid-ramp reports where it is heading, not where it
  *         has got to. They coincide once the ramp finishes, which is the
  *         only time the two can differ observably.
  * @retval None
  */
static void send_status(void)
{
  uint8_t p[1u + (PROTO_JOINT_COUNT * 4u)];

  p[0] = s_estopped;
  for (uint32_t i = 0; i < PROTO_JOINT_COUNT; i++)
  {
    uint8_t *e = &p[1u + (i * 4u)];
    e[0] = (uint8_t)(s_pan_cdeg[i] & 0xFF);
    e[1] = (uint8_t)((s_pan_cdeg[i] >> 8) & 0xFF);
    e[2] = (uint8_t)(s_tilt_cdeg[i] & 0xFF);
    e[3] = (uint8_t)((s_tilt_cdeg[i] >> 8) & 0xFF);
  }
  send_frame(CMD_STATUS, p, sizeof(p));
}

/**
  * @brief  Sends a CMD_IDENTITY frame naming this board's role, firmware
  *         version and wire-protocol version.
  * @note   Answers unconditionally, e-stopped or not -- modelled on
  *         send_status() rather than on the gated command handlers. The
  *         Jetson asks WHOAMI before its first heartbeat, so a reply that
  *         needed the link armed would deadlock startup.
  * @note   Deliberately does NOT touch s_last_heartbeat_tick. Only
  *         CMD_HEARTBEAT may feed the watchdog; otherwise a controller
  *         polling identity could hold the board armed indefinitely without
  *         ever proving the link healthy.
  * @retval None
  */
static void send_identity(void)
{
  uint8_t p[5];

  p[0] = (uint8_t)PROTO_SELF_ROLE;
  p[1] = FW_VERSION_MAJOR;
  p[2] = FW_VERSION_MINOR;
  p[3] = FW_VERSION_PATCH;
  p[4] = PROTO_VERSION;
  send_frame(CMD_IDENTITY, p, sizeof(p));
}

/**
  * @brief  Validates and applies a CMD_MOVE_JOINT_TO command.
  * @note   Latches both angles for status reporting and, if a physical
  *         ServoJoint_t is bound at that position, drives its pan and tilt
  *         servos together via a single ServoJoint_SetAngles() call.
  * @param  payload Payload bytes: [position][pan_cdeg_lo][pan_cdeg_hi][tilt_cdeg_lo][tilt_cdeg_hi].
  * @param  len     Number of bytes in payload (must be exactly 5).
  * @retval None
  */
static void handle_move_joint_to(const uint8_t *payload, uint8_t len)
{
  if (len != 5u)
  {
    send_nack(CMD_MOVE_JOINT_TO, NACK_BAD_LENGTH);
    return;
  }

  uint8_t position = payload[0];
  int16_t pan_cdeg = (int16_t)(payload[1] | ((uint16_t)payload[2] << 8));
  int16_t tilt_cdeg = (int16_t)(payload[3] | ((uint16_t)payload[4] << 8));

  if (position >= PROTO_JOINT_COUNT)
  {
    send_nack(CMD_MOVE_JOINT_TO, NACK_OUT_OF_RANGE);
    return;
  }

  const ServoAxisLimit_t *pan_limit = ServoJoint_PanLimit((ServoJointPosition_t)position);
  const ServoAxisLimit_t *tilt_limit = ServoJoint_TiltLimit((ServoJointPosition_t)position);
  if ((pan_cdeg < pan_limit->min_cdeg) || (pan_cdeg > pan_limit->max_cdeg)
      || (tilt_cdeg < tilt_limit->min_cdeg) || (tilt_cdeg > tilt_limit->max_cdeg))
  {
    send_nack(CMD_MOVE_JOINT_TO, NACK_OUT_OF_RANGE);
    return;
  }
  if (s_estopped)
  {
    send_nack(CMD_MOVE_JOINT_TO, NACK_ESTOPPED);
    return;
  }

  s_pan_cdeg[position] = pan_cdeg;
  s_tilt_cdeg[position] = tilt_cdeg;
  if (s_joints[position] != NULL)
  {
    ServoJoint_SetAngles(s_joints[position], pan_cdeg, tilt_cdeg);
  }
  send_ack(CMD_MOVE_JOINT_TO);
}

static int16_t clamp_cdeg(int16_t value, int16_t min_cdeg, int16_t max_cdeg)
{
  if (value < min_cdeg)
  {
    return min_cdeg;
  }
  if (value > max_cdeg)
  {
    return max_cdeg;
  }
  return value;
}

/**
  * @brief  Validates and applies a CMD_RESET_JOINTS command: moves every
  *         joint (bound or not) to its predefined reset pose in one shot.
  * @note   Latches every position's reset angles for status reporting even
  *         if no physical ServoJoint_t is bound there yet. Clamps each
  *         reset angle against that position's live limit before latching,
  *         so CMD_GET_STATUS can never report a value different from what
  *         ServoJoint_SetAngles() would actually apply -- protects against
  *         the reset table and a joint's limit drifting out of sync (e.g.
  *         after a limit change) without anyone noticing.
  * @retval None
  */
static void handle_reset_joints(void)
{
  if (s_estopped)
  {
    send_nack(CMD_RESET_JOINTS, NACK_ESTOPPED);
    return;
  }

  for (uint32_t i = 0; i < PROTO_JOINT_COUNT; i++)
  {
    const ServoAxisLimit_t *pan_limit = ServoJoint_PanLimit((ServoJointPosition_t)i);
    const ServoAxisLimit_t *tilt_limit = ServoJoint_TiltLimit((ServoJointPosition_t)i);
    int16_t pan_cdeg = clamp_cdeg(s_reset_pan_cdeg[i], pan_limit->min_cdeg, pan_limit->max_cdeg);
    int16_t tilt_cdeg = clamp_cdeg(s_reset_tilt_cdeg[i], tilt_limit->min_cdeg, tilt_limit->max_cdeg);

    s_pan_cdeg[i] = pan_cdeg;
    s_tilt_cdeg[i] = tilt_cdeg;
    if (s_joints[i] != NULL)
    {
      ServoJoint_SetAngles(s_joints[i], pan_cdeg, tilt_cdeg);
    }
  }
  send_ack(CMD_RESET_JOINTS);
}

/**
  * @brief  Validates and applies a CMD_SET_FAN_SPEED command.
  * @param  payload Payload bytes: [percent].
  * @param  len     Number of bytes in payload (must be exactly 1).
  * @retval None
  */
static void handle_set_fan_speed(const uint8_t *payload, uint8_t len)
{
  if (len != 1u)
  {
    send_nack(CMD_SET_FAN_SPEED, NACK_BAD_LENGTH);
    return;
  }
  if (payload[0] > 100u)
  {
    send_nack(CMD_SET_FAN_SPEED, NACK_OUT_OF_RANGE);
    return;
  }
  if (s_estopped)
  {
    send_nack(CMD_SET_FAN_SPEED, NACK_ESTOPPED);
    return;
  }

  Fan_SetSpeedPercent(payload[0]);
  send_ack(CMD_SET_FAN_SPEED);
}

/**
  * @brief  Validates and applies a CMD_SET_FAN_RGB command.
  * @note   Not gated by e-stop: lighting isn't a motion-safety concern.
  * @param  payload Payload bytes: [r][g][b], applied to every LED.
  * @param  len     Number of bytes in payload (must be exactly 3).
  * @retval None
  */
static void handle_set_fan_rgb(const uint8_t *payload, uint8_t len)
{
  if (len != 3u)
  {
    send_nack(CMD_SET_FAN_RGB, NACK_BAD_LENGTH);
    return;
  }

  ARGB_SetAll(payload[0], payload[1], payload[2]);
  ARGB_Show();
  send_ack(CMD_SET_FAN_RGB);
}

/**
  * @brief  Stops every bound joint's servos.
  * @retval None
  */
static void stop_all_joints(void)
{
  for (uint32_t i = 0; i < PROTO_JOINT_COUNT; i++)
  {
    if (s_joints[i] != NULL)
    {
      ServoJoint_Stop(s_joints[i]);
    }
  }
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
        for (uint32_t i = 0; i < PROTO_JOINT_COUNT; i++)
        {
          if (s_joints[i] != NULL)
          {
            ServoJoint_Resume(s_joints[i]);
          }
        }
        Fan_Resume();
      }
      send_ack(cmd);
      break;

    case CMD_MOVE_JOINT_TO:
      handle_move_joint_to(payload, payload_len);
      break;

    case CMD_RESET_JOINTS:
      handle_reset_joints();
      break;

    case CMD_STOP:
      s_estopped = 1u;
      stop_all_joints();
      Fan_Stop();
      send_ack(cmd);
      break;

    case CMD_GET_STATUS:
      send_status();
      break;

    case CMD_WHOAMI:
      send_identity();
      break;

    case CMD_SET_FAN_SPEED:
      handle_set_fan_speed(payload, payload_len);
      break;

    case CMD_SET_FAN_RGB:
      handle_set_fan_rgb(payload, payload_len);
      break;

    default:
      send_nack(cmd, NACK_UNKNOWN_CMD);
      break;
  }
}

/**
  * @brief  Advances the frame-parser state machine by one received byte.
  * @note   Runs in ISR context (called from HAL_UART_RxCpltCallback) — keep it
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

void Protocol_BindJoint(ServoJointPosition_t position, ServoJoint_t *joint)
{
  s_joints[position] = joint;
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
  /* Report each joint's neutral -- the pose ServoJoint_Init() actually
   * parked the outputs at -- rather than zeroing. Zero is below every
   * joint's tilt floor, so a controller doing read-modify-write
   * against CMD_GET_STATUS (move one axis, echo the other back unchanged)
   * would resend that illegal tilt and have the whole move bounced as
   * OUT_OF_RANGE, including the axis it did mean to move. */
  for (uint32_t i = 0; i < PROTO_JOINT_COUNT; i++)
  {
    ServoJoint_NeutralAngles((ServoJointPosition_t)i, &s_pan_cdeg[i], &s_tilt_cdeg[i]);
  }
  stop_all_joints(); /* ServoJoint_Init() left PWM running; hold off until armed */

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
    stop_all_joints();
    Fan_Stop();
  }

  if (!s_frame_ready)
  {
    return;
  }

  handle_frame(s_frame_cmd, s_frame_payload, s_frame_payload_len);
  s_frame_ready = 0u;
}
