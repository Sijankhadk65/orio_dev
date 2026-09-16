/**
  * @file    lights.h
  * @brief   On/off driver for the four chassis lights, each switched on the
  *          LOW side -- the switching device sits between the lamp and
  *          ground, so every pin is active HIGH and the lamp's own rail
  *          never reaches the MCU. That last point is why a 5 V lamp and a
  *          12 V one need no difference here, in the firmware or in the
  *          drive circuit.
  *
  *          Three of the four are S8050 NPN transistors:
  *
  *            MCU pin --[R_B]--> base
  *            emitter ---------> GND (common with the MCU's)
  *            collector -------> lamp's negative terminal
  *            lamp's positive -> +12 V rail
  *
  *          LIGHT_CHEST is NOT. At 820 mA it needs roughly 82 mA of base
  *          current to saturate an S8050, which is the MCU's entire I/O
  *          budget through one pin rated for 20 mA. Underdriven, the
  *          transistor leaves saturation and burns the difference as heat in
  *          a 1 W TO-92 package. That channel uses a logic-level N-MOSFET
  *          instead -- gate where the base was, source to ground, drain to
  *          the lamp -- which the firmware drives identically because it is
  *          still an active-high low-side switch.
  *
  * @note    Each lamp's negative terminal must be FREE. Low-side switching
  *          interrupts the ground side, so a fixture whose negative lead is
  *          bonded to a shared chassis ground is permanently on no matter
  *          what this driver does. If a light turns out to be wired that
  *          way it needs a high-side switch (PNP + NPN pair) instead, and
  *          this module is the wrong driver for it.
  *
  * @note    Measured lamp currents, meter in series -- NOT the listings'
  *          figures, which are wrong for the trailer bar:
  *            LIGHT_TRAILER            12 V     30 mA  (0.36 W)
  *            LIGHT_DRL_LEFT/_RIGHT    12 V     70 mA  (0.84 W) each
  *            LIGHT_CHEST (LED ring)    5 V    820 mA  (4.1 W)
  *          Two rails, not one, and the chest ring alone pulls nearly five
  *          times the other three put together. Whatever supplies 5 V has to
  *          carry that on top of everything else already on it -- it is far
  *          more than a Nucleo's own 5 V pin can source.
  *
  * @note    470 ohm base resistors on the three S8050 channels. At 3.3 V
  *          with V_BE(sat) near 0.8 V that is ~5.3 mA of base current, a
  *          forced beta of 13 on the heaviest of them (70 mA) against an
  *          S8050 whose worst-case h_FE is 85 -- roughly 6x more drive than
  *          saturation needs, for 16 mA of I/O budget across all three.
  *          Re-check that if a lamp is ever swapped for a brighter one: past
  *          about 250 mA a 470 ohm resistor stops supplying enough base
  *          current to saturate, and V_CE(sat) climbs instead of the
  *          transistor simply switching harder. That is exactly the wall
  *          LIGHT_CHEST ran into.
  *
  * @note    LIGHT_CHEST's MOSFET needs a gate PULLDOWN (100k, gate to
  *          source) that the BJT channels do not. Every pin on this part
  *          comes out of reset as a high-impedance input and stays that way
  *          until Lights_Init() runs. A floating base simply passes no
  *          current, so a BJT channel is dark by default -- but a floating
  *          gate holds whatever charge it finds and can switch the FET
  *          partly on, which at 820 mA means a hot transistor and a chest
  *          light glowing through reset, programming and every brownout.
  *          A ~100 ohm series gate resistor is also worth fitting, to damp
  *          the gate-capacitance current spike at each switching edge.
  *
  * @note    On/off only, deliberately. The S8050 switches fine, but dimming
  *          would mean PWM, and every timer channel on this part is already
  *          spoken for (TIM1/TIM3 servos, TIM14 fan, TIM16 ARGB). Adding
  *          brightness control means finding a timer first, not editing here.
  */
#ifndef LIGHTS_H
#define LIGHTS_H

#include "stm32c0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Which light, as an index into the pin table in lights.c.
 *
 * This order IS the wire protocol's bit order: LIGHT_TRAILER is bit 0 of a
 * CMD_SET_LIGHTS mask and LIGHT_CHEST is bit 3. Renaming a value costs
 * nothing, but REORDERING or inserting one silently repoints every mask an
 * existing client already sends -- append a fifth light rather than slotting
 * it in where it belongs anatomically. */
typedef enum
{
  LIGHT_TRAILER,   /* Greluma 12-24 V LED light bar, rear */
  LIGHT_DRL_LEFT,  /* X SIM FITNESSX 8-LED daytime running light */
  LIGHT_DRL_RIGHT, /* the other half of that same pair */
  LIGHT_CHEST,     /* front chest light -- part not chosen yet */
} LightId_t;

#define LIGHT_COUNT      4u    /* one per LightId_t value */
#define LIGHTS_ALL_MASK  0x0Fu /* every valid bit in a Lights_SetMask() mask */

/**
  * @brief  Configures all four light pins as push-pull outputs and leaves
  *         every light OFF.
  * @note   Call once at startup. Safe to call before or after
  *         MX_GPIO_Init() -- it enables its own port clocks rather than
  *         relying on that one having run first.
  * @note   The four pins are configured in BOTH places, deliberately. They
  *         are registered in the .ioc as locked GPIO_Output, so CubeMX
  *         reserves them and will not hand one to a peripheral behind your
  *         back -- and they are configured again here, so this module stands
  *         on its own and can be dropped into a project whose .ioc knows
  *         nothing about it. HAL_GPIO_Init() is idempotent and CubeMX's
  *         MX_GPIO_Init() runs first, so the second pass costs a few cycles
  *         at boot and nothing else.
  * @retval None
  */
void Lights_Init(void);

/**
  * @brief  Turns one light on or off.
  * @param  id Which light. Out-of-range values are ignored.
  * @param  on Nonzero lights the lamp, zero extinguishes it.
  * @retval None
  */
void Lights_Set(LightId_t id, uint8_t on);

/**
  * @brief  Sets all four lights at once from a bitmask.
  * @note   Not atomic at the hardware level -- the pins are written one at a
  *         time, microseconds apart. That is invisible on a lamp but would
  *         matter if these pins ever drove something timing-sensitive.
  * @param  mask Bit N (N = LightId_t) set lights that lamp. Bits above
  *              LIGHTS_ALL_MASK are ignored.
  * @retval None
  */
void Lights_SetMask(uint8_t mask);

/**
  * @brief  Returns the currently applied mask, in the same bit order
  *         Lights_SetMask() takes.
  * @retval Bitmask of which lights are lit.
  */
uint8_t Lights_GetMask(void);

#ifdef __cplusplus
}
#endif

#endif /* LIGHTS_H */
