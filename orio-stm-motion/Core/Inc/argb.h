/**
  * @file    argb.h
  * @brief   Driver for a WS2812B/SK6812-style single-wire addressable RGB LED
  *          chain, as used on ARGB case fans (TIM16_CH1 / PA0 "ARGB_DATA" on
  *          this board, pushed out via DMA so bit timing survives interrupts
  *          elsewhere in the app).
  * @note    The bound timer must already be configured for an 800 kHz PWM
  *          period (Prescaler = 0, Period = 59 on the 48 MHz APB timer clock
  *          -> 48 MHz / 60 = 800 kHz), with DMA linked to the PWM channel's
  *          capture/compare event.
  * @note    ARGB_LED_COUNT defaults to 8, a common count for ARGB case fans;
  *          set it to match your fan's actual LED count. Byte order is GRB,
  *          the standard for WS2812B/SK6812 -- if colors come out swapped,
  *          your chain uses a different order.
  */
#ifndef ARGB_H
#define ARGB_H

#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ARGB_LED_COUNT 8u

/**
  * @brief  Binds the driver to its timer/channel. Does not transmit
  *         anything -- call ARGB_Show() (or ARGB_Off()) to push the first
  *         frame once colors are set, otherwise the chain stays whatever it
  *         powered on with.
  * @param  htim    Timer handle already configured for 800 kHz PWM with DMA.
  * @param  channel Timer channel the LED data line is wired to.
  * @retval None
  */
void ARGB_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/**
  * @brief  Sets one LED's color in the local pixel buffer (not yet sent).
  * @param  index LED position in the chain, 0-based; out-of-range is ignored.
  * @param  r, g, b 8-bit color channel values.
  * @retval None
  */
void ARGB_SetLed(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
  * @brief  Sets every LED in the local pixel buffer to the same color (not
  *         yet sent).
  * @param  r, g, b 8-bit color channel values.
  * @retval None
  */
void ARGB_SetAll(uint8_t r, uint8_t g, uint8_t b);

/**
  * @brief  Encodes the local pixel buffer into the WS2812 bit pattern and
  *         blocks (briefly, well under 1 ms for ARGB_LED_COUNT in the tens)
  *         while DMA shifts it out over the data line.
  * @retval None
  */
void ARGB_Show(void);

/**
  * @brief  Sets every LED to black and shows it, turning the chain off.
  * @retval None
  */
void ARGB_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* ARGB_H */
