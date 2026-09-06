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
#include "servo_joint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_STX                  0xAAu
#define PROTO_MAX_PAYLOAD          16u
#define PROTO_HEARTBEAT_TIMEOUT_MS 500u

/* One slot per ServoJointPosition_t value (neck, left-arm, right-arm). Every
 * joint is a pan+tilt pair -- see servo_joint.h -- so CMD_MOVE_JOINT_TO
 * addresses a whole joint by position and moves both servos in one frame. */
#define PROTO_JOINT_COUNT SERVO_JOINT_POSITION_COUNT

typedef enum
{
  CMD_HEARTBEAT    = 0x01,
  /* payload: [position][pan_cdeg_lo][pan_cdeg_hi][tilt_cdeg_lo][tilt_cdeg_hi].
   * Angles are hundredths of a degree on the VENDOR scale -- pan 0..27000
   * (0..270.00 deg), tilt 0..18000 (0..180.00 deg), each measured from
   * that servo's own zero end, matching the vendor's datasheet and example
   * code. Mid-travel is 13500 for pan and 9000 for tilt.
   * Each joint additionally restricts these; see kJointLimits in servo_joint.c.
   * The joint ramps to these angles at SERVO_JOINT_SLEW_CDEG_PER_S rather
   * than snapping to them, so the ACK means "accepted and under way", not
   * "arrived" -- travel takes roughly (angle delta / slew rate). Nothing
   * reports arrival yet; a controller that needs to know must wait out the
   * ramp itself. */
  CMD_MOVE_JOINT_TO = 0x02,
  CMD_STOP         = 0x03,
  CMD_GET_STATUS   = 0x04,
  CMD_SET_FAN_SPEED = 0x05, /* payload: [percent 0-100] */
  CMD_SET_FAN_RGB   = 0x06, /* payload: [r][g][b], applied to every LED */
  CMD_RESET_JOINTS  = 0x07, /* payload: none; moves every joint to its predefined home pose */

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
  * @brief  Binds the physical ServoJoint_t at a body position so
  *         CMD_MOVE_JOINT_TO can drive it and CMD_GET_STATUS can report it.
  * @note   Call before Protocol_Init() for any joint that must start held
  *         stopped (Protocol_Init()'s e-stop-on-boot handling only stops
  *         joints already bound at that point). Passing NULL leaves that
  *         position latched for status only, same as a position never bound.
  * @param  position Which body position joint is being bound.
  * @param  joint    Pointer to a ServoJoint_t already initialized with
  *                  ServoJoint_Init() (must outlive the protocol layer), or
  *                  NULL to unbind.
  * @retval None
  */
void Protocol_BindJoint(ServoJointPosition_t position, ServoJoint_t *joint);

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
