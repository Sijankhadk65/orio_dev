"""The Orio carrier board, as data.

This module *is* the design. generate.py turns it into a KiCad project; nothing
downstream carries information that isn't here, so a change lands by editing
this file and re-running the generator, never by hand-editing the .kicad_sch.

Every part is a dict:
    ref    schematic reference designator
    sym    symbol name (a generic from generate.py, or a key in part_pinouts)
    value  what goes on the BOM
    fp     KiCad footprint library reference
    sheet  which schematic sheet it lands on
    conn   {pin number or pin name: net name}
    dnp    optional, True for do-not-populate

Pins are addressed by number for generic two-pin parts and by *name* for ICs,
because a name is design intent and a number is a datasheet transcription that
part_pinouts.py flags as unverified.
"""

from morpho_map import C031C6, L152RE

SHEETS = [
    ("power",      "Input Protection and Jetson Feed"),
    ("conv7v4",    "7.4 V Servo Rail"),
    ("conv12_5",   "12 V and 5 V Rails"),
    ("motion",     "Motion I/O -- Servos, ARGB, Fan"),
    ("drivetrain", "Drivetrain I/O -- Isolated ESC Links, E-stop"),
]

# Nets carrying more than ~1 A, given their own net class so the router widens
# them without being asked. V7V4 at 12 A is the one that actually matters.
NET_CLASSES = {
    "HighCurrent": dict(width=3.0, clearance=0.4,
                        nets=["VIN_RAW", "VIN_FUSED", "V19V5", "V19V5_JETSON",
                              "V19V5_J2", "V7V4", "V7V4_FUSED", "SW_7V4", "GND"]),
    "Power":       dict(width=1.0, clearance=0.3,
                        nets=["V12", "V12_FUSED", "V5", "V5_FUSED", "SW_12", "SW_5",
                              "SERVO1_V", "SERVO2_V", "SERVO3_V",
                              "SERVO4_V", "SERVO5_V", "SERVO6_V"]),
}

FP = dict(
    r="Resistor_SMD:R_0805_2012Metric",
    c="Capacitor_SMD:C_0805_2012Metric",
    c1210="Capacitor_SMD:C_1210_3225Metric",
    cp="Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    cp_big="Capacitor_THT:CP_Radial_D12.5mm_P5.00mm",
    ptc="Resistor_SMD:R_1812_4532Metric",
    fuse_smd="Fuse:Fuse_1812_4532Metric",
    fuse_ato="Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    # SRP1245A and the SRP1265A this design calls for share the 12.5x12.5 mm
    # SRP12xx land pattern; only height differs. Verify before fab.
    l_big="Inductor_SMD:L_Bourns_SRP1245A",
    l_sml="Inductor_SMD:L_Bourns_SRN6045TA",
    ferrite="Inductor_SMD:L_1812_4532Metric",
    sod123="Diode_SMD:D_SOD-123",
    smc="Diode_SMD:D_SMC",
    smb="Diode_SMD:D_SMB",
    sot23_6="Package_TO_SOT_SMD:SOT-23-6",
    sot23_5="Package_TO_SOT_SMD:SOT-23-5",
    sot23="Package_TO_SOT_SMD:SOT-23",
    dpak="Package_TO_SOT_SMD:TO-263-2",
    qfn24="Package_DFN_QFN:HVQFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm_ThermalVias",
    soic8ep="Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm_ThermalVias",
    soic8="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
    tssop20="Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm",
    # PLACEHOLDER: KiCad ships no MORNSUN footprint and the B0505S-1WR3 is a
    # SIP-4 whose pin arrangement is not a 2.54 mm inline row. Draw the real
    # land pattern from the datasheet before fab -- see FP_PLACEHOLDER below.
    sip4="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
    hdr2="Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    hdr3="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    hdr4="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
    xh4="Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
    # XT60 pigtail soldered to wire pads: 4 sqmm is ~12 AWG, what XT60 leads use.
    xt60="Connector_Wire:SolderWire-4sqmm_1x02_P12mm_D3mm_OD6mm",
    barrel="Connector_BarrelJack:BarrelJack_Horizontal",
    screw2="TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal",
    morpho="Connector_PinSocket_2.54mm:PinSocket_2x19_P2.54mm_Vertical",
    tp="TestPoint:TestPoint_Pad_D2.0mm",
)

# Footprints that are deliberate stand-ins, not choices. generate.py reports
# these alongside the unverified pin tables so they cannot be forgotten.
FP_PLACEHOLDER = {
    FP["sip4"]: "B0505S-1WR3 needs a custom SIP-4 land pattern",
}

PARTS = []


def add(ref, sym, value, fp, sheet, conn, dnp=False, note=""):
    PARTS.append(dict(ref=ref, sym=sym, value=value, fp=fp, sheet=sheet,
                      conn=conn, dnp=dnp, note=note))


def morpho(prefix, table, port_nets, sheet, board):
    """Emit the two Morpho socket rows for one Nucleo, wired by MCU port name."""
    rows = {"CN7": {}, "CN10": {}}
    for port, net in port_nets.items():
        header, number = table[port]
        rows[header][str(number)] = net
    for header, conn in rows.items():
        add(f"{prefix}{1 if header == 'CN7' else 2}", "Morpho2x19",
            f"{board} {header}", FP["morpho"], sheet, conn,
            note=f"Nucleo-64 Morpho {header}; pin numbers pending verification")


# ===========================================================================
# Sheet: power -- input protection, star ground origin, Jetson feed
# ===========================================================================
S = "power"
add("J1", "Conn_2P", "XT60 20 V in", FP["xt60"], S, {"1": "VIN_RAW", "2": "GND"})
add("F1", "Fuse", "ATO 15 A", FP["fuse_ato"], S, {"1": "VIN_RAW", "2": "VIN_FUSED"})

# Reverse polarity + overvoltage lockout. The FET is the only thing between a
# failed upstream module and the Jetson, which has no regulator of its own.
add("U1", "LM74700", "LM74700-Q1", FP["sot23_6"], S,
    {"VIN": "VIN_FUSED", "VS": "V19V5", "GATE": "GATE_ID",
     "GND": "GND", "EN": "OVLO_EN", "VCAP": "VCAP_ID"})
add("Q1", "Q_NMOS", "IPB057N06N", FP["dpak"], S,
    {"G": "GATE_ID", "D": "V19V5", "S": "VIN_FUSED"})
add("C1", "C", "100nF/50V", FP["c"], S, {"1": "VCAP_ID", "2": "VIN_FUSED"})
# 21.0 V trip: 1.59 V at the 1.6 V EN threshold.
add("R1", "R", "100k", FP["r"], S, {"1": "VIN_FUSED", "2": "OVLO_EN"})
add("R2", "R", "8k06", FP["r"], S, {"1": "OVLO_EN", "2": "GND"})
add("D1", "D_TVS", "SMCJ22A", FP["smc"], S, {"1": "V19V5", "2": "GND"})

for i, (ref, val, f) in enumerate([("C2", "470uF/35V", "cp_big"), ("C3", "470uF/35V", "cp_big"),
                                   ("C4", "10uF/50V", "c1210"), ("C5", "10uF/50V", "c1210"),
                                   ("C6", "10uF/50V", "c1210"), ("C7", "10uF/50V", "c1210")]):
    add(ref, "C_POL" if "470" in val else "C", val, FP[f], S, {"1": "V19V5", "2": "GND"})

add("F2", "Fuse", "ATO 5 A", FP["fuse_ato"], S, {"1": "V19V5", "2": "V19V5_JETSON"})
add("FB1", "L", "600R@100MHz 6A", FP["ferrite"], S, {"1": "V19V5_JETSON", "2": "V19V5_J2"})
add("C8", "C_POL", "100uF/35V", FP["cp"], S, {"1": "V19V5_J2", "2": "GND"})
add("C9", "C", "100nF/50V", FP["c"], S, {"1": "V19V5_J2", "2": "GND"})
add("J2", "Conn_2P", "Barrel 5.5x2.5 to Jetson", FP["barrel"], S,
    {"1": "V19V5_J2", "2": "GND"})

# One shared undervoltage lockout so all three rails come up and drop together
# rather than chattering independently through a brownout. 16.0 V turn-on.
add("R3", "R", "100k", FP["r"], S, {"1": "V19V5", "2": "EN_SYS"})
add("R4", "R", "8k06", FP["r"], S, {"1": "EN_SYS", "2": "GND"})

for ref, net in [("TP1", "V19V5"), ("TP2", "GND")]:
    add(ref, "TestPoint", net, FP["tp"], S, {"1": net})

for ref, net in [("PWR1", "GND"), ("PWR2", "V19V5")]:
    add(ref, "PWR_FLAG", "PWR_FLAG", "", S, {"1": net})


# ===========================================================================
# Sheet: conv7v4 -- the only rail that needs a controller, not a converter
# ===========================================================================
S = "conv7v4"
add("U2", "LM5145", "LM5145", FP["qfn24"], S,
    {"VIN": "V19V5", "EN": "EN_SYS", "RT": "RT_7V4", "SS": "SS_7V4",
     "FB": "FB_7V4", "COMP": "COMP_7V4", "AGND": "GND", "ILIM": "ILIM_7V4",
     "VCCX": "VCC_7V4", "DEMB": "GND", "SYNCIN": "GND", "PGOOD": "PG_7V4",
     "HB": "HB_7V4", "HO": "HO_7V4", "SW": "SW_7V4", "LO": "LO_7V4",
     "PGND": "GND", "VCC": "VCC_7V4", "VDDA": "VCC_7V4", "EP": "GND"})
add("Q2", "Q_NMOS", "CSD18540Q5B", FP["dpak"], S,
    {"G": "HO_7V4", "D": "V19V5", "S": "SW_7V4"}, note="high side")
add("Q3", "Q_NMOS", "CSD18540Q5B", FP["dpak"], S,
    {"G": "LO_7V4", "D": "SW_7V4", "S": "GND"}, note="low side")
add("L1", "L", "3.3uH 20A", FP["l_big"], S, {"1": "SW_7V4", "2": "V7V4"})
add("C10", "C", "100nF/25V", FP["c"], S, {"1": "HB_7V4", "2": "SW_7V4"})
add("C11", "C", "2.2uF/25V", FP["c"], S, {"1": "VCC_7V4", "2": "GND"})
add("C12", "C", "22nF/25V", FP["c"], S, {"1": "SS_7V4", "2": "GND"})
add("R5", "R", "25k", FP["r"], S, {"1": "RT_7V4", "2": "GND"}, note="400 kHz")
add("R6", "R", "10k", FP["r"], S, {"1": "ILIM_7V4", "2": "GND"})
# 0.8 V reference -> 7.41 V out.
add("R7", "R", "49k9", FP["r"], S, {"1": "V7V4", "2": "FB_7V4"})
add("R8", "R", "6k04", FP["r"], S, {"1": "FB_7V4", "2": "GND"})
add("R9", "R", "10k", FP["r"], S, {"1": "COMP_7V4", "2": "COMP_7V4_C"})
add("C13", "C", "1nF/25V", FP["c"], S, {"1": "COMP_7V4_C", "2": "GND"})
add("R10", "R", "100k", FP["r"], S, {"1": "PG_7V4", "2": "VCC_7V4"})

for ref in ("C14", "C15", "C16", "C17"):
    add(ref, "C", "10uF/50V", FP["c1210"], S, {"1": "V19V5", "2": "GND"})
for ref in ("C18", "C19"):
    add(ref, "C_POL", "1000uF/16V", FP["cp_big"], S, {"1": "V7V4", "2": "GND"})
for ref in ("C20", "C21", "C22", "C23"):
    add(ref, "C", "22uF/16V", FP["c1210"], S, {"1": "V7V4", "2": "GND"})

add("F3", "Fuse", "ATO 15 A", FP["fuse_ato"], S, {"1": "V7V4", "2": "V7V4_FUSED"})
add("TP3", "TestPoint", "V7V4", FP["tp"], S, {"1": "V7V4"})
add("PWR3", "PWR_FLAG", "PWR_FLAG", "", S, {"1": "V7V4"})


# ===========================================================================
# Sheet: conv12_5 -- two instances of the same part, different dividers
# ===========================================================================
S = "conv12_5"


def tps54360(ref, sw, boot, fb, comp, rt, lval, lfp, out, rtop, rbot, dref, lref,
             misc, cin, cout):
    add(ref, "TPS54360B", "TPS54360B", FP["soic8ep"], S,
        {"BOOT": boot, "VIN": "V19V5", "EN": "EN_SYS", "RT/CLK": rt,
         "FB": fb, "COMP": comp, "GND": "GND", "SW": sw, "EP": "GND"})
    add(misc[0], "C", "100nF/25V", FP["c"], S, {"1": boot, "2": sw})
    add(dref, "D_Schottky", "SS5H10 5A/100V", FP["smc"], S, {"A": "GND", "K": sw})
    add(lref, "L", lval, FP[lfp], S, {"1": sw, "2": out})
    add(rtop[0], "R", rtop[1], FP["r"], S, {"1": out, "2": fb})
    add(rbot[0], "R", rbot[1], FP["r"], S, {"1": fb, "2": "GND"})
    add(misc[1], "R", "100k", FP["r"], S, {"1": rt, "2": "GND"})
    add(misc[2], "R", "10k", FP["r"], S, {"1": comp, "2": comp + "_C"})
    add(misc[3], "C", "3.3nF/25V", FP["c"], S, {"1": comp + "_C", "2": "GND"})
    for r in cin:
        add(r, "C", "10uF/50V", FP["c1210"], S, {"1": "V19V5", "2": "GND"})
    for r in cout:
        add(r, "C", "47uF/25V", FP["c1210"], S, {"1": out, "2": "GND"})


# 0.8 V reference -> 12.0 V and 4.98 V.
tps54360("U3", "SW_12", "BOOT_12", "FB_12", "COMP_12", "RT_12",
         "15uH 4A", "l_sml", "V12", ("R11", "140k"), ("R12", "10k"),
         "D2", "L2", ("C24", "R13", "R14", "C25"), ("C26", "C27"), ("C28", "C29"))
tps54360("U4", "SW_5", "BOOT_5", "FB_5", "COMP_5", "RT_5",
         "6.8uH 5A", "l_sml", "V5", ("R15", "52k3"), ("R16", "10k"),
         "D3", "L3", ("C30", "R17", "R18", "C31"), ("C32", "C33"), ("C34", "C35"))

add("F4", "Fuse", "3 A slow", FP["fuse_smd"], S, {"1": "V12", "2": "V12_FUSED"})
add("F5", "Fuse", "3 A slow", FP["fuse_smd"], S, {"1": "V5", "2": "V5_FUSED"})
add("J3", "Conn_2P", "12 V aux / lights", FP["screw2"], S,
    {"1": "V12_FUSED", "2": "GND"})
for ref, net in [("TP4", "V12"), ("TP5", "V5")]:
    add(ref, "TestPoint", net, FP["tp"], S, {"1": net})
for ref, net in [("PWR4", "V12"), ("PWR5", "V5")]:
    add(ref, "PWR_FLAG", "PWR_FLAG", "", S, {"1": net})


# ===========================================================================
# Sheet: motion -- STM32C031C6 domain
# ===========================================================================
S = "motion"
SERVOS = [
    ("PA6", "neck pan"), ("PA7", "neck tilt"),
    ("PB0", "left arm pan"), ("PB1", "left arm tilt"),
    ("PA8", "right arm pan"), ("PA9", "right arm tilt"),
]

motion_nets = {port: f"SERVO{i+1}_3V3" for i, (port, _) in enumerate(SERVOS)}
motion_nets.update({
    "PA0": "ARGB_3V3", "PA4": "FAN_PWM_3V3", "PA1": "FAN_TACH",
    "3V3": "V3V3_M", "5V": "M5V_JP", "GND1": "GND", "GND2": "GND",
})
morpho("JM", C031C6, motion_nets, S, "NUCLEO-C031C6")

# Default build leaves this open: the module takes its 5 V from the Jetson's USB
# port. Fitting it without also moving the Nucleo's own jumper to E5V back-feeds
# the ST-Link regulator.
add("JP1", "Conn_2P", "E5V ONLY - DNP", FP["hdr2"], S,
    {"1": "V5_FUSED", "2": "M5V_JP"}, dnp=True)

# Eight 3.3 V signals out, eight 5 V signals in. 8/8 channels used.
add("U7", "AHCT244", "SN74AHCT244PWR", FP["tssop20"], S, {
    "~{OE1}": "GND", "~{OE2}": "GND", "VCC": "V5_FUSED", "GND": "GND",
    "A1_1": "SERVO1_3V3", "Y1_1": "SERVO1_5V",
    "A1_2": "SERVO2_3V3", "Y1_2": "SERVO2_5V",
    "A1_3": "SERVO3_3V3", "Y1_3": "SERVO3_5V",
    "A1_4": "SERVO4_3V3", "Y1_4": "SERVO4_5V",
    "A2_1": "SERVO5_3V3", "Y2_1": "SERVO5_5V",
    "A2_2": "SERVO6_3V3", "Y2_2": "SERVO6_5V",
    "A2_3": "ARGB_3V3",   "Y2_3": "ARGB_5V",
    "A2_4": "FAN_PWM_3V3", "Y2_4": "FAN_PWM_5V",
})
add("C36", "C", "100nF/25V", FP["c"], S, {"1": "V5_FUSED", "2": "GND"})
add("C37", "C", "100nF/25V", FP["c"], S, {"1": "V5_FUSED", "2": "GND"})

for i, (port, what) in enumerate(SERVOS, start=1):
    add(f"R{19+i}", "R", "100R", FP["r"], S,
        {"1": f"SERVO{i}_5V", "2": f"SERVO{i}_SIG"})
    add(f"PTC{i}", "PTC", "1812L300/16", FP["ptc"], S,
        {"1": "V7V4_FUSED", "2": f"SERVO{i}_V"})
    add(f"J{9+i}", "Conn_Servo", f"{what} servo", FP["hdr3"], S,
        {"1": f"SERVO{i}_SIG", "2": f"SERVO{i}_V", "3": "GND"})

add("C38", "C_POL", "470uF/16V", FP["cp"], S, {"1": "V7V4_FUSED", "2": "GND"})
add("R26", "R", "330R", FP["r"], S, {"1": "ARGB_5V", "2": "ARGB_DATA"})
add("C39", "C_POL", "1000uF/10V", FP["cp_big"], S, {"1": "V5_FUSED", "2": "GND"})
add("J16", "Conn_Servo", "ARGB chain (16 LED)", FP["hdr3"], S,
    {"1": "V5_FUSED", "2": "ARGB_DATA", "3": "GND"})
# One header, both fans daisy-chained on the harness; only the first returns tach.
add("J17", "Conn_Fan", "PC fan x2 daisy", FP["hdr4"], S,
    {"1": "GND", "2": "V12_FUSED", "3": "FAN_TACH", "4": "FAN_PWM_5V"})
add("R27", "R", "10k", FP["r"], S, {"1": "FAN_TACH", "2": "V3V3_M"})
add("C40", "C", "100pF/25V", FP["c"], S, {"1": "FAN_TACH", "2": "GND"})
add("TP6", "TestPoint", "GND", FP["tp"], S, {"1": "GND"})
add("PWR6", "PWR_FLAG", "PWR_FLAG", "", S, {"1": "V3V3_M"})


# ===========================================================================
# Sheet: drivetrain -- STM32L152RE domain, isolated ESC links
# ===========================================================================
S = "drivetrain"
drive_nets = {
    "PA9": "ESCL_TX_MCU", "PA10": "ESCL_RX_MCU",
    "PB10": "ESCR_TX_MCU", "PB11": "ESCR_RX_MCU",
    "PC13": "ESTOP_N",
    "3V3": "V3V3_D", "5V": "D5V_JP", "GND1": "GND", "GND2": "GND",
}
morpho("JD", L152RE, drive_nets, S, "NUCLEO-L152RE")
add("JP2", "Conn_2P", "E5V ONLY - DNP", FP["hdr2"], S,
    {"1": "V5_FUSED", "2": "D5V_JP"}, dnp=True)


def esc_channel(side, u, ps, ldo, dref, j, rrefs, crefs):
    """One galvanically isolated VESC UART link.

    The barrier is the point: the FSESC ground is bolted to pack negative and
    carries motor return current. Bridging it to logic ground with a UART ground
    wire closes a loop around the motor leads.
    """
    iso_g, iso_5, iso_3 = f"ISO_GND_{side}", f"ISO_5V_{side}", f"ISO_3V3_{side}"
    tx_mcu, rx_mcu = f"ESC{side}_TX_MCU", f"ESC{side}_RX_MCU"
    tx_iso, rx_iso = f"ESC{side}_TX_ISO", f"ESC{side}_RX_ISO"
    tx_out, rx_in = f"ESC{side}_TX_OUT", f"ESC{side}_RX_IN"

    add(u, "ISO7721", "ISO7721DR", FP["soic8"], S,
        {"VCC1": "V3V3_D", "GND1": "GND", "INA": tx_mcu, "OUTB": rx_mcu,
         "OUTA": tx_iso, "INB": rx_iso, "GND2": iso_g, "VCC2": iso_3})
    add(ps, "B0505S", "B0505S-1WR3", FP["sip4"], S,
        {"+VIN": "V5_FUSED", "-VIN": "GND", "+VOUT": iso_5, "-VOUT": iso_g})
    add(ldo, "AP2112K", "AP2112K-3.3", FP["sot23_5"], S,
        {"VIN": iso_5, "EN": iso_5, "GND": iso_g, "VOUT": iso_3, "NC": ""})
    add(rrefs[0], "R", "100R", FP["r"], S, {"1": tx_iso, "2": tx_out})
    add(rrefs[1], "R", "100R", FP["r"], S, {"1": rx_in, "2": rx_iso})
    add(crefs[0], "C", "100pF/25V", FP["c"], S, {"1": tx_out, "2": iso_g})
    add(crefs[1], "C", "100pF/25V", FP["c"], S, {"1": rx_in, "2": iso_g})
    add(crefs[2], "C", "4.7uF/16V", FP["c"], S, {"1": iso_5, "2": iso_g})
    add(crefs[3], "C", "1uF/16V", FP["c"], S, {"1": iso_3, "2": iso_g})
    add(crefs[4], "C", "100nF/25V", FP["c"], S, {"1": "V3V3_D", "2": "GND"})
    add(dref, "D_ESD4", "PESD3V3L4UG", FP["sot23_6"], S,
        {"IO1": tx_out, "IO2": rx_in, "IO3": "", "IO4": "", "GND": iso_g})
    # J pin 2 is the ESC's RX (we drive it); pin 3 is the ESC's TX (it drives us).
    add(j, "Conn_ESC", f"FSESC {side}", FP["xh4"], S,
        {"1": iso_g, "2": tx_out, "3": rx_in, "4": ""})
    add(f"PWR_{side}", "PWR_FLAG", "PWR_FLAG", "", S, {"1": iso_3})


esc_channel("L", "U9", "PS1", "U11", "D4", "J20",
            ("R28", "R29"), ("C41", "C42", "C43", "C44", "C45"))
esc_channel("R", "U10", "PS2", "U12", "D5", "J21",
            ("R30", "R31"), ("C46", "C47", "C48", "C49", "C50"))

# Normally-closed loop on PC13, which the drivetrain firmware already has
# configured as EXTI13 for the Nucleo user button.
add("J22", "Conn_2P", "E-stop (NC)", FP["hdr2"], S, {"1": "ESTOP_N", "2": "GND"})
add("R32", "R", "10k", FP["r"], S, {"1": "ESTOP_N", "2": "V3V3_D"})
add("C51", "C", "100nF/25V", FP["c"], S, {"1": "ESTOP_N", "2": "GND"})
add("PWR7", "PWR_FLAG", "PWR_FLAG", "", S, {"1": "V3V3_D"})
