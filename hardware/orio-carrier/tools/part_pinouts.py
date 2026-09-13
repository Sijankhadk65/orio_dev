"""IC pinouts, transcribed from datasheets -- the second unverified table.

Same rule as morpho_map.py: the pin *names* here are the design (they say what
each connection is for, and the netlist in design.py is written against them),
but the pin *numbers* are a mechanical transcription from each part's datasheet
and are marked for checking before fab. A wrong number here is a dead board
that looks perfectly fine in the schematic.

Check each against the package drawing in the linked datasheet, then set its
entry in VERIFIED to True. generate.py lists everything still False.

Sides: "L" puts the pin on the left of the symbol body, "R" on the right.
Types are KiCad electrical types and only affect ERC.
"""

VERIFIED = {
    "LM74700": False,   # https://www.ti.com/lit/ds/symlink/lm74700-q1.pdf   SOT-23-6 (DBV)
    "LM5145":  False,   # https://www.ti.com/lit/ds/symlink/lm5145.pdf       VQFN-24 (RGY)
    "TPS54360B": False, # https://www.ti.com/lit/ds/symlink/tps54360b.pdf    SO PowerPAD-8 (DDA)
    "AHCT244": False,   # https://www.ti.com/lit/ds/symlink/sn74ahct244.pdf  TSSOP-20 (PW)
    "ISO7721": False,   # https://www.ti.com/lit/ds/symlink/iso7721.pdf      SOIC-8 (D)
    "B0505S":  False,   # MORNSUN B0505S-1WR3                               SIP-4
    "AP2112K": False,   # https://www.diodes.com/assets/Datasheets/AP2112.pdf SOT-23-5
}

PINOUTS = {
    # --- input protection -------------------------------------------------
    "LM74700": [
        ("VCAP",  "1", "L", "passive"),
        ("VS",    "2", "L", "input"),
        ("EN",    "3", "L", "input"),
        ("GND",   "4", "L", "power_in"),
        ("GATE",  "5", "R", "output"),
        ("VIN",   "6", "R", "power_in"),
    ],

    # --- 7.4 V synchronous buck controller --------------------------------
    "LM5145": [
        ("VIN",   "1",  "L", "power_in"),
        ("EN",    "2",  "L", "input"),
        ("RT",    "3",  "L", "passive"),
        ("SS",    "4",  "L", "passive"),
        ("FB",    "5",  "L", "input"),
        ("COMP",  "6",  "L", "passive"),
        ("AGND",  "7",  "L", "power_in"),
        ("ILIM",  "8",  "L", "passive"),
        ("VCCX",  "9",  "L", "passive"),
        ("DEMB",  "10", "L", "input"),
        ("SYNCIN","11", "L", "input"),
        ("PGOOD", "12", "L", "output"),
        ("HB",    "13", "R", "passive"),
        ("HO",    "14", "R", "output"),
        ("SW",    "15", "R", "passive"),
        ("LO",    "16", "R", "output"),
        ("PGND",  "17", "R", "power_in"),
        ("VCC",   "18", "R", "power_out"),
        ("VDDA",  "19", "R", "power_in"),
        ("EP",    "20", "R", "power_in"),
    ],

    # --- 12 V and 5 V step-down (internal HS FET, external catch diode) ---
    "TPS54360B": [
        ("BOOT",  "1", "L", "passive"),
        ("VIN",   "2", "L", "power_in"),
        ("EN",    "3", "L", "input"),
        ("RT/CLK","4", "L", "input"),
        ("FB",    "5", "R", "input"),
        ("COMP",  "6", "R", "passive"),
        ("GND",   "7", "R", "power_in"),
        ("SW",    "8", "R", "output"),
        ("EP",    "9", "R", "power_in"),
    ],

    # --- 3.3 V -> 5 V octal buffer, TTL input thresholds ------------------
    "AHCT244": [
        ("~{OE1}", "1",  "L", "input"),
        ("A1_1",   "2",  "L", "input"),
        ("A2_4",   "3",  "L", "input"),
        ("A1_2",   "4",  "L", "input"),
        ("A2_3",   "5",  "L", "input"),
        ("A1_3",   "6",  "L", "input"),
        ("A2_2",   "7",  "L", "input"),
        ("A1_4",   "8",  "L", "input"),
        ("A2_1",   "9",  "L", "input"),
        ("GND",    "10", "L", "power_in"),
        ("Y2_1",   "11", "R", "tri_state"),
        ("Y1_4",   "12", "R", "tri_state"),
        ("Y2_2",   "13", "R", "tri_state"),
        ("Y1_3",   "14", "R", "tri_state"),
        ("Y2_3",   "15", "R", "tri_state"),
        ("Y1_2",   "16", "R", "tri_state"),
        ("Y2_4",   "17", "R", "tri_state"),
        ("Y1_1",   "18", "R", "tri_state"),
        ("~{OE2}", "19", "R", "input"),
        ("VCC",    "20", "R", "power_in"),
    ],

    # --- reinforced digital isolator, 2 channel, one each direction -------
    "ISO7721": [
        ("VCC1",  "1", "L", "power_in"),
        ("GND1",  "2", "L", "power_in"),
        ("INA",   "3", "L", "input"),
        ("OUTB",  "4", "L", "output"),
        ("OUTA",  "5", "R", "output"),
        ("INB",   "6", "R", "input"),
        ("GND2",  "7", "R", "power_in"),
        ("VCC2",  "8", "R", "power_in"),
    ],

    # --- isolated 5 V -> 5 V, 1 W ----------------------------------------
    "B0505S": [
        ("-VIN",  "1", "L", "power_in"),
        ("+VIN",  "2", "L", "power_in"),
        ("-VOUT", "3", "R", "power_out"),
        ("+VOUT", "4", "R", "power_out"),
    ],

    # --- isolated-side 3.3 V LDO for the VESC UART level ------------------
    "AP2112K": [
        ("VIN",   "1", "L", "power_in"),
        ("GND",   "2", "L", "power_in"),
        ("EN",    "3", "L", "input"),
        ("NC",    "4", "R", "no_connect"),
        ("VOUT",  "5", "R", "power_out"),
    ],
}


def unverified():
    return sorted(k for k, v in VERIFIED.items() if not v)
