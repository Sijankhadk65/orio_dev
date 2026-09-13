# Orio Carrier Board

Power distribution and Nucleo motherboard for Orio. One 19–20 V input becomes
four rails; both Nucleo-64 modules plug in through their Morpho headers; every
servo, fan, ARGB and FSESC connector lands on a board edge.

The full design rationale — power tree, budget, fuse sizing, ground topology,
floorplan — lives in the design package artifact. This directory is the KiCad
project that implements it.

> **Do not fabricate yet.** `UNVERIFIED.md` lists the pin tables that still need
> checking against datasheets. The netlist is correct by *signal name*; the
> physical pin numbers are a transcription step that hasn't been done.

## Layout

```
hardware/orio-carrier/
  tools/design.py        the design -- parts, nets, values. Edit this.
  tools/morpho_map.py    Nucleo Morpho pin maps      <- UNVERIFIED
  tools/part_pinouts.py  IC pinouts from datasheets  <- UNVERIFIED
  tools/generate.py      emits everything below
  orio-carrier.kicad_pro / .kicad_sch / .kicad_sym / .kicad_pcb
  sym-lib-table          registers the project symbol library
  power / conv7v4 / conv12_5 / motion / drivetrain .kicad_sch
  bom.csv
```

Regenerate after any change:

```bash
python tools/generate.py
```

**Edit `tools/design.py`, never the `.kicad_sch` files** — they are overwritten
on every run. Once the pin tables are verified and you start laying out the
board in KiCad, the `.kicad_pcb` becomes the thing you edit by hand and the
generator's job is finished.

## How the netlist is built

Every pin gets a 2.54 mm wire stub ending in a global label carrying its net
name, at a coordinate computed from the same symbol geometry the generator
emits. There is no hand-placed wire that can be one grid step off a pin, and no
net that exists only because two things happen to touch. The generator refuses
to finish if a part references a pin its symbol doesn't have, and reports any
net with fewer than two connections.

Current state: **151 parts, 93 nets, 0 floating**.

The schematic is machine-tidy, not human-pretty — symbols column-packed, labels
everywhere. It is meant to be read as a correct netlist. Rearranging it into
something nice to look at is a job for KiCad, after verification.

## Before fabricating

1. **Fill in `tools/morpho_map.py`.** Map each MCU port to its CN7/CN10 pin from
   ST's extension-connector tables — UM2953 for the NUCLEO-C031C6 (MB1717),
   UM1724 for the NUCLEO-L152RE (MB1136). Set `VERIFIED = True`.
2. **Check every entry in `tools/part_pinouts.py`** against the package drawing
   in each linked datasheet, then flip its flag in `VERIFIED`.
3. **Confirm the Jetson's DC input range** for your Orin Nano carrier revision.
   The board feeds it directly from the protected bus with no conversion stage,
   which is only sound if 19.5 V sits comfortably inside that window.
4. **Resolve footprints in KiCad.** The references in `design.py`'s `FP` table
   are standard-library names chosen by inspection; KiCad will flag any that
   don't resolve when you open the project. The XT60 is mapped to solder wire
   pads rather than a connector footprint — KiCad has no XT60.
5. **Confirm the FSESC UART connector** matches `J20`/`J21` (currently JST-XH
   4-pin). On those headers pin 2 is the ESC's *RX* — we drive it — and pin 3 is
   the ESC's *TX*.
6. **Servo class.** The 7.4 V rail is sized for 25 kg·cm digital servos at
   ~2.7 A stall. Going to the 35–60 kg·cm class roughly doubles it and changes
   `L1`, `Q2`/`Q3` and the PTCs.

Then run the generator once more, watch `UNVERIFIED.md` disappear, and open the
project in KiCad to place and route.

## Things the schematic encodes that are easy to undo by accident

- **`JP1` / `JP2` are DNP on purpose.** Both Nucleos take their 5 V from the
  Jetson's USB port. Fitting these jumpers without also moving the Nucleo's own
  power-source jumper to E5V back-feeds the ST-Link's regulator.
- **The ESC links are isolated.** `U9`/`U10` plus `PS1`/`PS2` exist so the FSESC
  ground — bolted to pack negative, carrying motor return current — never
  bridges to logic ground. Replacing them with 0 Ω links to "simplify" closes a
  loop around the motor leads. The `.kicad_pcb` needs a plane slot on every
  layer under the barrier, running out to the board edge.
- **`EN_SYS` is shared** by all three converters, so the rails come up and drop
  together at 16.0 V instead of chattering independently through a brownout.
- **The star ground origin is `J1`.** All high-current returns converge there,
  and the six servo grounds get their own local star first.

## Relationship to the firmware

Nothing here requires a firmware change. Pin assignments come from
`orio-stm-motion.ioc` and `orio-stm-drivetrain.ioc` and are unchanged; routing
fan PWM through the 5 V buffer rather than an open-drain FET was chosen partly
so `fan.c`'s duty stays non-inverted.

Two capabilities become possible once the board exists, neither wired up in
software yet: fan tachometer capture on `PA1` (TIM2_CH2), and a hardware e-stop
on `PC13`, which the drivetrain firmware already configures as EXTI13 for the
Nucleo user button.
