/**
  * @file    argb.c
  * @brief   WS2812B/SK6812-style ARGB chain driver implementation (see argb.h).
  */
#include <string.h>
#include "argb.h"

#define ARGB_BITS_PER_LED  24u
#define ARGB_RESET_SLOTS   60u /* >=75us of low, comfortably above the >=50us most WS2812-family chips need to latch */
#define ARGB_BUFFER_LEN    ((ARGB_LED_COUNT * ARGB_BITS_PER_LED) + ARGB_RESET_SLOTS)

#define ARGB_DUTY_0 19u /* ~0.40us high of a 1.25us bit period -> logic 0 */
#define ARGB_DUTY_1 38u /* ~0.79us high of a 1.25us bit period -> logic 1 */

#define ARGB_SHOW_TIMEOUT_MS 5u

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} ArgbColor;

static TIM_HandleTypeDef *s_htim;
static uint32_t s_channel;
static ArgbColor s_pixels[ARGB_LED_COUNT];
static uint32_t s_dma_buf[ARGB_BUFFER_LEN];
static volatile uint8_t s_tx_busy;

static void encode_byte(uint8_t value, uint32_t *out)
{
  for (uint8_t bit = 0u; bit < 8u; bit++)
  {
    out[bit] = ((value & (0x80u >> bit)) != 0u) ? ARGB_DUTY_1 : ARGB_DUTY_0;
  }
}

void ARGB_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  s_htim = htim;
  s_channel = channel;
  s_tx_busy = 0u;
  memset(s_pixels, 0, sizeof(s_pixels));
  memset(s_dma_buf, 0, sizeof(s_dma_buf));
}

void ARGB_SetLed(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  if (index >= ARGB_LED_COUNT)
  {
    return;
  }

  s_pixels[index].r = r;
  s_pixels[index].g = g;
  s_pixels[index].b = b;
}

void ARGB_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint16_t i = 0u; i < ARGB_LED_COUNT; i++)
  {
    s_pixels[i].r = r;
    s_pixels[i].g = g;
    s_pixels[i].b = b;
  }
}

void ARGB_Show(void)
{
  uint32_t idx = 0u;

  for (uint16_t i = 0u; i < ARGB_LED_COUNT; i++)
  {
    encode_byte(s_pixels[i].g, &s_dma_buf[idx]);
    idx += 8u;
    encode_byte(s_pixels[i].r, &s_dma_buf[idx]);
    idx += 8u;
    encode_byte(s_pixels[i].b, &s_dma_buf[idx]);
    idx += 8u;
  }
  while (idx < ARGB_BUFFER_LEN)
  {
    s_dma_buf[idx++] = 0u;
  }

  s_tx_busy = 1u;
  HAL_TIM_PWM_Start_DMA(s_htim, s_channel, s_dma_buf, ARGB_BUFFER_LEN);

  uint32_t start_tick = HAL_GetTick();
  while (s_tx_busy && ((HAL_GetTick() - start_tick) < ARGB_SHOW_TIMEOUT_MS))
  {
  }
  HAL_TIM_PWM_Stop_DMA(s_htim, s_channel);
}

void ARGB_Off(void)
{
  ARGB_SetAll(0u, 0u, 0u);
  ARGB_Show();
}

/**
  * @brief  DMA transfer-complete callback for PWM channels; flags the ARGB
  *         chain's frame as fully shifted out.
  * @param  htim Timer handle for which the DMA-driven PWM stream completed.
  * @retval None
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16)
  {
    s_tx_busy = 0u;
  }
}
