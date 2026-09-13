"""Nucleo-64 Morpho header pin maps -- THE ONE UNVERIFIED TABLE IN THIS PROJECT.

Every other number in the design comes from a datasheet, a calculation, or the
firmware's own .ioc files. These do not: they map an MCU port name to the
physical pin index on the Nucleo-64's CN7/CN10 2x19 Morpho headers, and that
mapping differs per MCU variant even though the connector is mechanically the
same. It has to be read off ST's board documentation:

  NUCLEO-C031C6  -> UM2953, "Extension connectors" (board MB1717)
  NUCLEO-L152RE  -> UM1724, "Extension connectors" (board MB1136)

Until VERIFIED is True the numbers below are sequential placeholders. The
*netlist* is correct regardless -- every signal is wired to the right MCU port
name -- but the physical pad each one lands on is not, so the board must not be
fabricated. generate.py refuses to drop the warning banner while VERIFIED is
False.

To fill this in: replace each value with ("CN7"|"CN10", pin_number) and flip
VERIFIED. Then re-run `python tools/generate.py` -- nothing else changes.
"""

VERIFIED = False

# --- NUCLEO-C031C6 (motion board) -----------------------------------------
# Ports used by orio-stm-motion, per orio-stm-motion.ioc.
C031C6 = {
    "PA6":  ("CN10", 1),   # TIM3_CH1  neck pan servo
    "PA7":  ("CN10", 2),   # TIM3_CH2  neck tilt servo
    "PB0":  ("CN10", 3),   # TIM3_CH3  left arm pan servo
    "PB1":  ("CN10", 4),   # TIM3_CH4  left arm tilt servo
    "PA8":  ("CN10", 5),   # TIM1_CH1  right arm pan servo
    "PA9":  ("CN10", 6),   # TIM1_CH2  right arm tilt servo
    "PA0":  ("CN10", 7),   # TIM16_CH1 ARGB data (DMA)
    "PA4":  ("CN10", 8),   # TIM14_CH1 fan PWM, 25 kHz
    "PA1":  ("CN10", 9),   # TIM2_CH2  fan tach capture (spare)
    "3V3":  ("CN7",  1),
    "5V":   ("CN7",  2),
    "GND1": ("CN7",  3),
    "GND2": ("CN7",  4),
}

# --- NUCLEO-L152RE (drivetrain board) -------------------------------------
# Ports used by orio-stm-drivetrain, per orio-stm-drivetrain.ioc.
L152RE = {
    "PA9":  ("CN10", 1),   # USART1_TX  ESC left
    "PA10": ("CN10", 2),   # USART1_RX  ESC left
    "PB10": ("CN10", 3),   # USART3_TX  ESC right
    "PB11": ("CN10", 4),   # USART3_RX  ESC right
    "PC13": ("CN10", 5),   # EXTI13     e-stop (shares the user-button pin)
    "3V3":  ("CN7",  1),
    "5V":   ("CN7",  2),
    "GND1": ("CN7",  3),
    "GND2": ("CN7",  4),
}


def pin(table, port):
    """Return the flat 1..38 pin number for `port`, plus which header it is on."""
    header, number = table[port]
    return header, number
