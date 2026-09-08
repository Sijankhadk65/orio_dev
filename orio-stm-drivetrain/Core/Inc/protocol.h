/**
  * @file    protocol.h
  * @brief   Framed binary UART protocol between the Jetson brain and this
  *          STM32 drivetrain controller (see root README.md, "Architecture").
  *          Same wire framing and opcode numbering convention as the motion
  *          board's protocol.h -- CMD_SET_DRIVE takes CMD_MOVE_JOINT_TO's
  *          0x02 slot since each board only has one "move" verb -- but the
  *          two links are independent connections with independent e-stop.
  *          CMD_WHOAMI / CMD_IDENTITY go past mere convention: they are
  *          byte-for-byte identical on both boards, since a client has to be
  *          able to ask "which board are you?" without already knowing the
  *          answer.
  *
  * Frame layout (little-endian):
  *   [STX][LEN][CMD][PAYLOAD 0..LEN-1][CRC16_LO][CRC16_HI]
  *   - STX  : 1 byte, always PROTO_STX
  *   - LEN  : 1 byte, number of bytes in CMD+PAYLOAD (i.e. 1 + payload length)
  *   - CRC16: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over LEN,CMD,PAYLOAD
  */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "stm32l1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_STX                  0xAAu
#define PROTO_MAX_PAYLOAD          24u
#define PROTO_HEARTBEAT_TIMEOUT_MS 500u

/* Firmware and wire-protocol versions, reported by CMD_WHOAMI.
 *
 * FW_VERSION_* is this board's own build and moves independently of the
 * motion board's. PROTO_VERSION describes the framing and payload layouts
 * instead, so it is deliberately the SAME value on both boards -- bump it
 * only when the wire format changes in a way an existing client cannot
 * parse, never for a firmware change that merely adds behaviour. */
#define FW_VERSION_MAJOR 1u
#define FW_VERSION_MINOR 0u
#define FW_VERSION_PATCH 0u
#define PROTO_VERSION    1u

/* Which subsystem a board is, as carried in a CMD_IDENTITY payload. Shares
 * its numbering with the motion board's protocol.h: telling the two apart
 * is the entire purpose of the value, so it can never be board-local. */
typedef enum
{
  ROLE_DRIVETRAIN = 0x01,
  ROLE_MOTION     = 0x02,
} ProtoRole;

/* What this firmware answers CMD_WHOAMI with -- the one identity definition
 * that intentionally differs between the two boards. */
#define PROTO_SELF_ROLE ROLE_DRIVETRAIN

typedef enum
{
  CMD_HEARTBEAT   = 0x01,
  CMD_SET_DRIVE   = 0x02, /* payload: [left_permille_lo][left_permille_hi][right_permille_lo][right_permille_hi] */
  CMD_STOP        = 0x03,
  CMD_GET_STATUS  = 0x04,
  /* payload: none. Answered with CMD_IDENTITY rather than an ACK, so the
   * Jetson can confirm WHICH board is on a port before it arms anything.
   * Both Nucleos present their ST-LINK VCP as USB 0483:374b and share this
   * framing and CRC, so a CMD_SET_DRIVE that reaches the motion board
   * passes CRC and decodes as a valid frame of the wrong kind -- servos
   * slew to whatever a wheel duty pair happens to mean, with nothing
   * reporting an error. A udev symlink cannot catch a board swap or the
   * wrong firmware flashed onto a board; only an answer on the wire can.
   * Answers while e-stopped, exactly like CMD_GET_STATUS: the check happens
   * before arming, so gating it on the e-stop would deadlock startup.
   * 0x08 is free on both boards (drivetrain uses 0x01-0x04, motion 0x01-0x07). */
  CMD_WHOAMI      = 0x08,

  CMD_ACK         = 0x80,
  CMD_NACK        = 0x81,
  CMD_STATUS      = 0x82, /* payload: [estopped] then per side (left,right):
                            *   [erpm 4B LE][current_ca 2B LE][v_in_dv 2B LE][fault_code][valid] */
  CMD_IDENTITY    = 0x83, /* payload: [role][fw_major][fw_minor][fw_patch][proto_version],
                            * role being a ProtoRole value. Identical layout on both boards. */
} ProtoCmd;

typedef enum
{
  NACK_BAD_CRC      = 0x01,
  NACK_BAD_LENGTH   = 0x02,
  NACK_UNKNOWN_CMD  = 0x03,
  NACK_OUT_OF_RANGE = 0x04,
  NACK_ESTOPPED     = 0x05,
} ProtoNackReason;

/**
  * @brief  Initializes the protocol layer and arms the first byte-wise UART receive.
  * @note   Call once at startup, after the UART peripheral and Wheel_Init()
  *         have run. The link starts e-stopped and stays that way until the
  *         first CMD_HEARTBEAT frame is received.
  * @param  huart Pointer to the UART handle the protocol will run on (e.g. &huart2).
  * @retval None
  */
void Protocol_Init(UART_HandleTypeDef *huart);

/**
  * @brief  Feeds one received byte into the frame parser and re-arms the next receive.
  * @note   Call from HAL_UART_RxCpltCallback for every byte received on the
  *         linked UART. Runs in ISR context, so it only touches the internal
  *         parser state and never blocks.
  * @param  huart UART handle passed through from HAL_UART_RxCpltCallback.
  * @retval None
  */
void Protocol_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
  * @brief  Services the protocol: checks the heartbeat watchdog and, if a
  *         complete frame has been received, dispatches it to its handler.
  * @note   Call repeatedly from the main loop (not from an ISR).
  * @retval None
  */
void Protocol_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
