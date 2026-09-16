/**
  * @file    lights.c
  * @brief   Four-channel on/off light driver implementation (see lights.h).
  */
#include "lights.h"

/* Which pin switches which light.
 *
 * UNVERIFIED -- these four assignments are the one thing in this module that
 * was not read off a datasheet. They were picked as ports this firmware's
 * .ioc leaves unused (PA0/2/3/4/6/7/8/9/13/14, PB0/1, PC14/15 and PF0/1 are
 * all taken), NOT confirmed against the LQFP48 bonding in DS13866 or against
 * the Nucleo-C031C6's Morpho headers in UM2953. A pin that is not bonded out
 * on this package silently does nothing.
 *
 * Two checks settle it. Opening the .ioc in CubeMX proves the pins EXIST on
 * this package -- they are registered there as LIGHT_* GPIO_Output, and
 * CubeMX rejects a pin the LQFP48 does not have. Finding which Morpho pad
 * each one lands on is the other half, and the firmware itself answers that:
 * send CMD_SET_LIGHTS with one bit set and probe the headers for the pad
 * that went to 3.3 V.
 *
 * PB6 and PB7 are USART1's default TX/RX. Nothing uses USART1 today -- the
 * Jetson link is USART2 -- but a second serial port later would want them
 * back, and PB2/PB3 are free to swap in if so.
 *
 * Changing a light's pin is a one-line edit here and nothing else. The same
 * is not true of the ORDER: the row index is LightId_t, which is the bit
 * position in a CMD_SET_LIGHTS mask, so reordering rows silently repoints
 * every mask an existing client sends. */
static const struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} kLightPins[LIGHT_COUNT] = {
  { GPIOB, GPIO_PIN_4 }, /* LIGHT_TRAILER   */
  { GPIOB, GPIO_PIN_5 }, /* LIGHT_DRL_LEFT  */
  { GPIOB, GPIO_PIN_6 }, /* LIGHT_DRL_RIGHT */
  { GPIOB, GPIO_PIN_7 }, /* LIGHT_CHEST     */
};

static uint8_t s_mask;

/**
  * @brief  Enables the AHB clock for one GPIO port.
  * @note   Exists so the pin table above can name any port without
  *         Lights_Init() growing a hidden dependency on MX_GPIO_Init()
  *         having already enabled that particular one. The macros are
  *         idempotent, so enabling a port twice is free.
  * @param  port GPIO port to clock.
  * @retval None
  */
static void enable_port_clock(GPIO_TypeDef *port)
{
  if (port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (port == GPIOF)
  {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  }
  else
  {
    /* Not a port this part has; the pin table is wrong. */
  }
}

/**
  * @brief  Drives one light's pin to match a bit of the cached mask.
  * @param  id Which light (already known to be in range).
  * @retval None
  */
static void apply_one(uint32_t id)
{
  GPIO_PinState state = ((s_mask >> id) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(kLightPins[id].port, kLightPins[id].pin, state);
}

void Lights_Init(void)
{
  GPIO_InitTypeDef init = {0};

  s_mask = 0u;

  for (uint32_t i = 0; i < LIGHT_COUNT; i++)
  {
    enable_port_clock(kLightPins[i].port);

    /* Drive the pad low BEFORE switching it to an output, so it never
     * briefly presents whatever was in ODR as a flash on the lamp. */
    HAL_GPIO_WritePin(kLightPins[i].port, kLightPins[i].pin, GPIO_PIN_RESET);

    init.Pin = kLightPins[i].pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    /* A lamp does not care about edge rate and a slow edge radiates less;
     * nothing here is switching fast enough for LOW to cost anything. */
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(kLightPins[i].port, &init);
  }
}

void Lights_Set(LightId_t id, uint8_t on)
{
  if ((uint32_t)id >= LIGHT_COUNT)
  {
    return;
  }

  if (on)
  {
    s_mask |= (uint8_t)(1u << (uint32_t)id);
  }
  else
  {
    s_mask &= (uint8_t)~(1u << (uint32_t)id);
  }
  apply_one((uint32_t)id);
}

void Lights_SetMask(uint8_t mask)
{
  s_mask = (uint8_t)(mask & LIGHTS_ALL_MASK);

  for (uint32_t i = 0; i < LIGHT_COUNT; i++)
  {
    apply_one(i);
  }
}

uint8_t Lights_GetMask(void)
{
  return s_mask;
}
