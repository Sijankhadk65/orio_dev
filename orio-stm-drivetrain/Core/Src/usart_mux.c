/**
  * @file    usart_mux.c
  * @brief   USART1/ESC_MUX_SEL switch implementation (see usart_mux.h).
  */
#include "usart_mux.h"
#include "main.h"

static UART_HandleTypeDef *s_huart;
static DriveSide_t s_selected = DRIVE_SIDE_COUNT; /* invalid -- forces the first select */

void UsartMux_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_selected = DRIVE_SIDE_COUNT;
}

void UsartMux_Select(DriveSide_t side)
{
  if (side == s_selected)
  {
    return;
  }

  HAL_GPIO_WritePin(ESC_MUX_SEL_GPIO_Port, ESC_MUX_SEL_Pin, (side == DRIVE_RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  s_selected = side;
}

HAL_StatusTypeDef UsartMux_Transmit(DriveSide_t side, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  UsartMux_Select(side);
  return HAL_UART_Transmit(s_huart, (uint8_t *)data, len, timeout_ms);
}

HAL_StatusTypeDef UsartMux_Receive(DriveSide_t side, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  UsartMux_Select(side);
  return HAL_UART_Receive(s_huart, data, len, timeout_ms);
}
