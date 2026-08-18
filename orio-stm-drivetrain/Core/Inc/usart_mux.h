/**
  * @file    usart_mux.h
  * @brief   Shares USART1 (PB6/PB7) between the two FSESCs through a
  *          GPIO-selected 2:1 analog switch on ESC_MUX_SEL (PA1): low routes
  *          USART1 to the left FSESC, high routes it to the right FSESC.
  *          Only one side can be talked to at a time -- callers must not
  *          interleave transfers to the two sides from an ISR.
  */
#ifndef USART_MUX_H
#define USART_MUX_H

#include <stdint.h>
#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Which physical side of the robot -- and which mux position,
  *         FSESC, and wheel -- an index refers to. Shared by usart_mux.h,
  *         vesc.h and wheel.h since the mapping is 1:1 all the way through.
  */
typedef enum
{
  DRIVE_LEFT = 0,
  DRIVE_RIGHT = 1,
  DRIVE_SIDE_COUNT,
} DriveSide_t;

/**
  * @brief  Binds the shared UART handle for later Select/Transmit/Receive calls.
  * @param  huart Pointer to the USART1 handle (must already be HAL_UART_Init'd).
  * @retval None
  */
void UsartMux_Init(UART_HandleTypeDef *huart);

/**
  * @brief  Points ESC_MUX_SEL at the requested side. No-op if already selected.
  * @param  side Which FSESC USART1 should be connected to.
  * @retval None
  */
void UsartMux_Select(DriveSide_t side);

/**
  * @brief  Selects a side, then blocks transmitting to it.
  * @param  side    Which FSESC to send to.
  * @param  data    Bytes to transmit.
  * @param  len     Number of bytes in data.
  * @param  timeout_ms Timeout in milliseconds passed to HAL_UART_Transmit.
  * @retval HAL status from HAL_UART_Transmit.
  */
HAL_StatusTypeDef UsartMux_Transmit(DriveSide_t side, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
  * @brief  Selects a side, then blocks receiving from it.
  * @note   Does not re-select if UsartMux_Transmit() to the same side was
  *         just called -- caller is expected to pair a Transmit then Receive
  *         to the same side for one request/response transaction.
  * @param  side    Which FSESC to receive from.
  * @param  data    Buffer to receive into.
  * @param  len     Number of bytes to receive.
  * @param  timeout_ms Timeout in milliseconds passed to HAL_UART_Receive.
  * @retval HAL status from HAL_UART_Receive.
  */
HAL_StatusTypeDef UsartMux_Receive(DriveSide_t side, uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* USART_MUX_H */
