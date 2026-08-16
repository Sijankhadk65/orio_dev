/**
  * @file    protocol.h
  * @brief   Framed binary UART protocol between the Jetson brain and this
  *          STM32 motion controller (see root README.md, "Architecture").
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
#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_STX                  0xAAu
#define PROTO_MAX_PAYLOAD          16u
#define PROTO_HEARTBEAT_TIMEOUT_MS 500u
#define PROTO_ARM_JOINT_COUNT      4u
#define PROTO_JOINT_ANGLE_MIN_CDEG (-9000)  /* -90.00 deg, in hundredths of a degree */
#define PROTO_JOINT_ANGLE_MAX_CDEG (9000)   /*  90.00 deg */

typedef enum
{
  CMD_HEARTBEAT    = 0x01,
  CMD_MOVE_ARM_TO  = 0x02,
  CMD_STOP         = 0x03,
  CMD_GET_STATUS   = 0x04,
  CMD_SET_FAN_SPEED = 0x05, /* payload: [percent 0-100] */
  CMD_SET_FAN_RGB   = 0x06, /* payload: [r][g][b], applied to every LED */

  CMD_ACK         = 0x80,
  CMD_NACK        = 0x81,
  CMD_STATUS      = 0x82,
} ProtoCmd;

typedef enum
{
  NACK_BAD_CRC        = 0x01,
  NACK_BAD_LENGTH      = 0x02,
  NACK_UNKNOWN_CMD     = 0x03,
  NACK_OUT_OF_RANGE    = 0x04,
  NACK_ESTOPPED        = 0x05,
} ProtoNackReason;

/**
  * @brief  Initializes the protocol layer and arms the first byte-wise UART receive.
  * @note   Call once at startup, after the UART peripheral has been initialized.
  *         The link starts e-stopped and stays that way until the first
  *         CMD_HEARTBEAT frame is received.
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
