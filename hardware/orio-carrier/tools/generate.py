#!/usr/bin/env python3
"""Turn design.py into a KiCad 8 project.

Run from this directory:  python generate.py

Connectivity is built by construction rather than by drawing: every pin gets a
2.54 mm wire stub ending in a global label carrying its net name, at a
coordinate computed from the symbol geometry this script also emits. There is
no hand-placed wire to be one grid step off. The layout is machine-tidy rather
than human-pretty -- the schematic is a correct netlist you can read, and the
place to make it pretty is KiCad, after the pin tables below are verified.

Outputs, all overwritten on every run:
    orio-carrier.kicad_pro      project
    orio-carrier.kicad_sym      project symbol library
    orio-carrier.kicad_sch      root sheet
    <sheet>.kicad_sch           x5
    orio-carrier.kicad_pcb      board outline, stackup, net classes
    bom.csv
"""

import csv
import datetime
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import design
import morpho_map
import part_pinouts

OUT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PROJECT = "orio-carrier"
NS = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")
SCH_VERSION = 20231120
PCB_VERSION = 20240108
GRID = 2.54
STUB = 2.54
PIN_LEN = 5.08
PAPER = "A1"
PAPER_W, PAPER_H = 841.0, 594.0

BOARD_W, BOARD_H = 220.0, 160.0
BOARD_X, BOARD_Y = 40.0, 40.0


def uid(key):
    return str(uuid.uuid5(NS, key))


def n(v):
    """KiCad tolerates trailing zeros; keep the file diffable instead."""
    return f"{v:.4f}".rstrip("0").rstrip(".") or "0"


# ---------------------------------------------------------------------------
# Symbol definitions
# ---------------------------------------------------------------------------
# (name, number, side, electrical type)
GENERIC = {
    "R":          (dict(w=7.62, ref="R"),  [("1", "1", "L", "passive"), ("2", "2", "R", "passive")]),
    "C":          (dict(w=7.62, ref="C"),  [("1", "1", "L", "passive"), ("2", "2", "R", "passive")]),
    "C_POL":      (dict(w=7.62, ref="C"),  [("+", "1", "L", "passive"), ("-", "2", "R", "passive")]),
    "L":          (dict(w=7.62, ref="L"),  [("1", "1", "L", "passive"), ("2", "2", "R", "passive")]),
    "Fuse":       (dict(w=10.16, ref="F"), [("1", "1", "L", "passive"), ("2", "2", "R", "passive")]),
    "PTC":        (dict(w=10.16, ref="PTC"), [("1", "1", "L", "passive"), ("2", "2", "R", "passive")]),
    "D_TVS":      (dict(w=7.62, ref="D"),  [("A", "1", "L", "passive"), ("K", "2", "R", "passive")]),
    "D_Schottky": (dict(w=7.62, ref="D"),  [("A", "A", "L", "passive"), ("K", "K", "R", "passive")]),
    "D_ESD4":     (dict(w=15.24, ref="D"), [("IO1", "1", "L", "passive"), ("IO2", "2", "L", "passive"),
                                            ("IO3", "3", "L", "passive"), ("IO4", "4", "L", "passive"),
                                            ("GND", "5", "R", "passive")]),
    "Q_NMOS":     (dict(w=12.7, ref="Q"),  [("G", "G", "L", "input"), ("D", "D", "R", "passive"),
                                            ("S", "S", "R", "passive")]),
    "TestPoint":  (dict(w=5.08, ref="TP"), [("1", "1", "L", "passive")]),
    "PWR_FLAG":   (dict(w=5.08, ref="PWR"), [("1", "1", "L", "power_out")]),
    "Conn_2P":    (dict(w=10.16, ref="J"), [("1", "1", "L", "passive"), ("2", "2", "L", "passive")]),
    "Conn_Servo": (dict(w=12.7, ref="J"),  [("SIG", "1", "L", "passive"), ("V+", "2", "L", "passive"),
                                            ("GND", "3", "L", "passive")]),
    "Conn_Fan":   (dict(w=12.7, ref="J"),  [("GND", "1", "L", "passive"), ("+12V", "2", "L", "passive"),
                                            ("TACH", "3", "L", "passive"), ("PWM", "4", "L", "passive")]),
    "Conn_ESC":   (dict(w=12.7, ref="J"),  [("GND", "1", "L", "passive"), ("ESC_RX", "2", "L", "passive"),
                                            ("ESC_TX", "3", "L", "passive"), ("NC", "4", "L", "passive")]),
    "Morpho2x19": (dict(w=17.78, ref="J"),
                   [(str(i), str(i), "L" if i <= 19 else "R", "passive") for i in range(1, 39)]),
}


def build_symbols():
    syms = {}
    for name, (meta, pins) in GENERIC.items():
        syms[name] = dict(w=meta["w"], ref=meta["ref"], pins=pins)
    for name, pins in part_pinouts.PINOUTS.items():
        syms[name] = dict(w=35.56, ref="U",
                          pins=[(pn, num, side, et) for pn, num, side, et in pins])
    return syms


SYMS = build_symbols()


def geometry(sym):
    """Absolute pin coordinates in library space (Y up), plus the body rect."""
    pins = SYMS[sym]["pins"]
    w = SYMS[sym]["w"]
    left = [p for p in pins if p[2] == "L"]
    right = [p for p in pins if p[2] == "R"]
    rows = max(len(left), len(right))
    top = (rows - 1) * GRID / 2.0
    out = {}
    for i, p in enumerate(left):
        out[p[1]] = (-w / 2 - PIN_LEN, top - i * GRID, "L", p[0], p[3])
    for i, p in enumerate(right):
        out[p[1]] = (w / 2 + PIN_LEN, top - i * GRID, "R", p[0], p[3])
    body = (-w / 2, top + GRID, w / 2, top - (rows - 1) * GRID - GRID)
    return out, body, rows


def resolve(sym, key):
    """design.py addresses IC pins by name and passives by number; accept both."""
    pins, _, _ = geometry(sym)
    if key in pins:
        return key
    for num, (_, _, _, pname, _) in pins.items():
        if pname == key:
            return num
    raise KeyError(f'symbol {sym} has no pin "{key}"')


def emit_symbol_lib():
    lines = [f'(kicad_symbol_lib (version {SCH_VERSION}) (generator "orio-carrier-gen")']
    for name in sorted(SYMS):
        lines.append(symbol_def(name))
    lines.append(")")
    write(f"{PROJECT}.kicad_sym", "\n".join(lines))


def symbol_def(name, indent="  "):
    pins, body, _ = geometry(name)
    ref = SYMS[name]["ref"]
    o = [f'{indent}(symbol "{name}" (pin_names (offset 0.254)) (in_bom yes) (on_board yes)']
    o.append(f'{indent}  (property "Reference" "{ref}" (at 0 {n(body[1] + 2.54)} 0) '
             f'(effects (font (size 1.27 1.27)) (justify left)))')
    o.append(f'{indent}  (property "Value" "{name}" (at 0 {n(body[3] - 2.54)} 0) '
             f'(effects (font (size 1.27 1.27)) (justify left)))')
    for prop in ("Footprint", "Datasheet", "Description"):
        o.append(f'{indent}  (property "{prop}" "" (at 0 0 0) '
                 f'(effects (font (size 1.27 1.27)) (hide yes)))')
    o.append(f'{indent}  (symbol "{name}_0_1"')
    o.append(f'{indent}    (rectangle (start {n(body[0])} {n(body[1])}) (end {n(body[2])} {n(body[3])})')
    o.append(f'{indent}      (stroke (width 0.254) (type default)) (fill (type background)))')
    o.append(f'{indent}  )')
    o.append(f'{indent}  (symbol "{name}_1_1"')
    for num, (x, y, side, pname, etype) in pins.items():
        angle = 0 if side == "L" else 180
        o.append(f'{indent}    (pin {etype} line (at {n(x)} {n(y)} {angle}) (length {n(PIN_LEN)})')
        o.append(f'{indent}      (name "{pname}" (effects (font (size 1.27 1.27))))')
        o.append(f'{indent}      (number "{num}" (effects (font (size 1.27 1.27)))))')
    o.append(f'{indent}  )')
    o.append(f'{indent})')
    return "\n".join(o)


# ---------------------------------------------------------------------------
# Schematic
# ---------------------------------------------------------------------------
ROOT_UUID = uid("root-sheet")


def place(parts):
    """Column-pack parts onto the sheet, returning (part, x, y)."""
    col_w, x0, y0 = 104.0, 66.0, 48.0
    placed, col, y = [], 0, y0
    for p in parts:
        _, body, rows = geometry(p["sym"])
        h = (rows + 1) * GRID
        if y + h > PAPER_H - 40.0:
            col += 1
            y = y0
        x = x0 + col * col_w
        placed.append((p, round(x / GRID) * GRID, round(y / GRID) * GRID))
        y += h + 10.16
    return placed


def emit_sheet(key, title, parts, page):
    sheet_uuid = uid(f"sheet-{key}")
    path = f"/{ROOT_UUID}/{sheet_uuid}"
    used = sorted({p["sym"] for p in parts})

    o = [f"(kicad_sch (version {SCH_VERSION}) (generator \"orio-carrier-gen\")",
         f'  (uuid "{uid("sch-" + key)}")',
         f'  (paper "{PAPER}")',
         "  (title_block",
         f'    (title "Orio Carrier -- {title}")',
         f'    (date "{datetime.date.today().isoformat()}")',
         '    (rev "A")',
         '    (company "Orio")',
         f'    (comment 1 "{banner()}")',
         '    (comment 2 "Generated by tools/generate.py -- edit design.py, not this file")',
         "  )",
         "  (lib_symbols"]
    for s in used:
        o.append(symbol_def(s, indent="    ").replace(f'(symbol "{s}" ', f'(symbol "{PROJECT}:{s}" ', 1))
    o.append("  )")

    nets_here = {}
    for p, x, y in place(parts):
        pins, _, _ = geometry(p["sym"])
        o.append(f'  (symbol (lib_id "{PROJECT}:{p["sym"]}") (at {n(x)} {n(y)} 0) (unit 1)')
        o.append(f'    (exclude_from_sim no) (in_bom yes) (on_board yes) '
                 f'(dnp {"yes" if p["dnp"] else "no"})')
        o.append(f'    (uuid "{uid("sym-" + p["ref"])}")')
        _, body, rows = geometry(p["sym"])
        o.append(f'    (property "Reference" "{p["ref"]}" (at {n(x)} {n(y - body[1] - 2.54)} 0) '
                 f'(effects (font (size 1.27 1.27)) (justify left)))')
        o.append(f'    (property "Value" "{esc(p["value"])}" (at {n(x)} {n(y - body[3] + 2.54)} 0) '
                 f'(effects (font (size 1.27 1.27)) (justify left)))')
        o.append(f'    (property "Footprint" "{p["fp"]}" (at {n(x)} {n(y)} 0) '
                 f'(effects (font (size 1.27 1.27)) (hide yes)))')
        if p["note"]:
            o.append(f'    (property "Note" "{esc(p["note"])}" (at {n(x)} {n(y)} 0) '
                     f'(effects (font (size 1.27 1.27)) (hide yes)))')
        for num in pins:
            o.append(f'    (pin "{num}" (uuid "{uid("pin-" + p["ref"] + "-" + num)}"))')
        o.append(f'    (instances (project "{PROJECT}" (path "{path}" '
                 f'(reference "{p["ref"]}") (unit 1))))')
        o.append("  )")

        for pin_key, net in p["conn"].items():
            if not net:
                continue
            try:
                num = resolve(p["sym"], pin_key)
            except KeyError as exc:
                raise KeyError(f'{p["ref"]}: {exc}') from None
            lx, ly, side, _, _ = pins[num]
            px, py = x + lx, y - ly
            sx = px - STUB if side == "L" else px + STUB
            o.append(f'  (wire (pts (xy {n(px)} {n(py)}) (xy {n(sx)} {n(py)})) '
                     f'(stroke (width 0) (type default)) '
                     f'(uuid "{uid("w-" + p["ref"] + "-" + num)}"))')
            ang = 180 if side == "L" else 0
            o.append(f'  (global_label "{net}" (shape passive) (at {n(sx)} {n(py)} {ang}) '
                     f'(fields_autoplaced yes)')
            o.append(f'    (effects (font (size 1.27 1.27)) (justify {"right" if side == "L" else "left"}))')
            o.append(f'    (uuid "{uid("gl-" + p["ref"] + "-" + num)}")')
            o.append(f'    (property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {n(sx)} {n(py)} 0) '
                     f'(effects (font (size 1.27 1.27)) (hide yes)))')
            o.append("  )")
            nets_here.setdefault(net, []).append(f'{p["ref"]}.{num}')

    o.append(f'  (sheet_instances (path "/" (page "{page}")))')
    o.append(")")
    write(f"{key}.kicad_sch", "\n".join(o))
    return nets_here


def emit_root():
    o = [f"(kicad_sch (version {SCH_VERSION}) (generator \"orio-carrier-gen\")",
         f'  (uuid "{ROOT_UUID}")',
         '  (paper "A4")',
         "  (title_block",
         '    (title "Orio Carrier Board")',
         f'    (date "{datetime.date.today().isoformat()}")',
         '    (rev "A")',
         '    (company "Orio")',
         f'    (comment 1 "{banner()}")',
         "  )",
         "  (lib_symbols)"]
    for i, (key, title) in enumerate(design.SHEETS):
        x, y = 30.0, 30.0 + i * 34.0
        su = uid(f"sheet-{key}")
        o.append(f'  (sheet (at {n(x)} {n(y)}) (size 120 22)')
        o.append('    (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)')
        o.append('    (fields_autoplaced yes)')
        o.append('    (stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0000))')
        o.append(f'    (uuid "{su}")')
        o.append(f'    (property "Sheetname" "{esc(title)}" (at {n(x)} {n(y - 0.7)} 0) '
                 f'(effects (font (size 1.27 1.27)) (justify left bottom)))')
        o.append(f'    (property "Sheetfile" "{key}.kicad_sch" (at {n(x)} {n(y + 22.6)} 0) '
                 f'(effects (font (size 1.27 1.27)) (justify left top)))')
        o.append(f'    (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (page "{i + 2}"))))')
        o.append("  )")
    o.append('  (sheet_instances (path "/" (page "1")))')
    o.append(")")
    write(f"{PROJECT}.kicad_sch", "\n".join(o))


# ---------------------------------------------------------------------------
# Board: outline, stackup, rules. Footprints arrive via Update PCB from Schematic.
# ---------------------------------------------------------------------------
def emit_pcb(nets):
    order = ["", *sorted(nets)]
    o = [f"(kicad_pcb (version {PCB_VERSION}) (generator \"orio-carrier-gen\") "
         f'(generator_version "8.0")',
         "  (general (thickness 1.6) (legacy_teardrops no))",
         f'  (paper "{PAPER}")',
         "  (layers",
         '    (0 "F.Cu" signal)',
         '    (1 "In1.Cu" signal "GND plane")',
         '    (2 "In2.Cu" signal "Rail distribution")',
         '    (31 "B.Cu" signal)',
         '    (34 "B.Paste" user) (35 "F.Paste" user)',
         '    (36 "B.SilkS" user) (37 "F.SilkS" user)',
         '    (38 "B.Mask" user) (39 "F.Mask" user)',
         '    (44 "Edge.Cuts" user) (45 "Margin" user)',
         '    (46 "B.CrtYd" user) (47 "F.CrtYd" user)',
         '    (48 "B.Fab" user) (49 "F.Fab" user)',
         "  )",
         "  (setup",
         "    (stackup",
         '      (layer "F.Cu" (type "copper") (thickness 0.07))',
         '      (layer "dielectric 1" (type "prepreg") (thickness 0.2104) (material "FR4"))',
         '      (layer "In1.Cu" (type "copper") (thickness 0.0356))',
         '      (layer "dielectric 2" (type "core") (thickness 1.065) (material "FR4"))',
         '      (layer "In2.Cu" (type "copper") (thickness 0.0356))',
         '      (layer "dielectric 3" (type "prepreg") (thickness 0.2104) (material "FR4"))',
         '      (layer "B.Cu" (type "copper") (thickness 0.07))',
         '      (copper_finish "ENIG")',
         "      (dielectric_constraints no)",
         "    )",
         "    (pad_to_mask_clearance 0)",
         "    (allow_soldermask_bridges_in_footprints no)",
         "  )"]
    for i, net in enumerate(order):
        o.append(f'  (net {i} "{net}")')

    x0, y0, x1, y1 = BOARD_X, BOARD_Y, BOARD_X + BOARD_W, BOARD_Y + BOARD_H
    for a, b in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                 ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
        o.append(f'  (gr_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) '
                 f'(stroke (width 0.1) (type solid)) (layer "Edge.Cuts") '
                 f'(uuid "{uid(f"edge-{a}-{b}")}"))')
    for i, (mx, my) in enumerate([(x0 + 5, y0 + 5), (x1 - 5, y0 + 5),
                                  (x1 - 5, y1 - 5), (x0 + 5, y1 - 5)]):
        o.append(f'  (gr_circle (center {n(mx)} {n(my)}) (end {n(mx + 1.6)} {n(my)}) '
                 f'(stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts") '
                 f'(uuid "{uid(f"mnt-{i}")}"))')

    for text, (tx, ty), layer in [
        ("Orio Carrier Board rev A", (x0 + 10, y0 + 12), "F.SilkS"),
        ("ISOLATION BARRIER - NO COPPER", (x1 - 72, y0 + 30), "F.SilkS"),
        (banner(), (x0 + 10, y1 - 8), "F.SilkS"),
    ]:
        o.append(f'  (gr_text "{esc(text)}" (at {n(tx)} {n(ty)}) (layer "{layer}") '
                 f'(uuid "{uid("txt-" + text[:12])}") '
                 f'(effects (font (size 1.5 1.5) (thickness 0.25)) (justify left)))')
    o.append(")")
    write(f"{PROJECT}.kicad_pcb", "\n".join(o))


def emit_project():
    classes = [{"name": "Default", "clearance": 0.2, "track_width": 0.25,
                "via_diameter": 0.8, "via_drill": 0.4, "nets": []}]
    for name, spec in design.NET_CLASSES.items():
        classes.append({"name": name, "clearance": spec["clearance"],
                        "track_width": spec["width"], "via_diameter": 1.2,
                        "via_drill": 0.6, "nets": spec["nets"]})
    import json
    pro = {
        "board": {"design_settings": {"rules": {"min_clearance": 0.2,
                                                "min_track_width": 0.2,
                                                "min_through_hole_diameter": 0.3}}},
        "net_settings": {"classes": classes},
        "sheets": [[uid(f"sheet-{k}"), t] for k, t in design.SHEETS],
        "meta": {"filename": f"{PROJECT}.kicad_pro", "version": 1},
    }
    write(f"{PROJECT}.kicad_pro", json.dumps(pro, indent=2))


def emit_lib_table():
    """Register the project symbol library.

    Without this KiCad cannot resolve any `orio-carrier:` lib_id and opens the
    schematic as a field of missing-symbol placeholders. ${KIPRJMOD} keeps it
    relative, so the project works from any checkout path. Footprints come from
    KiCad's own global libraries and need no table of their own.
    """
    write("sym-lib-table",
          "(sym_lib_table\n"
          "  (version 7)\n"
          f'  (lib (name "{PROJECT}")(type "KiCad")'
          f'(uri "${{KIPRJMOD}}/{PROJECT}.kicad_sym")(options "")'
          '(descr "Orio carrier board project symbols"))\n'
          ")")


def emit_bom(parts):
    rows = {}
    for p in parts:
        if p["sym"] in ("PWR_FLAG", "TestPoint"):
            continue
        k = (p["value"], p["fp"], p["dnp"])
        rows.setdefault(k, []).append(p["ref"])
    with open(os.path.join(OUT, "bom.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Value", "Footprint", "Populate", "References"])
        for (value, fp, dnp), refs in sorted(rows.items(), key=lambda kv: kv[0][0]):
            w.writerow([len(refs), value, fp, "DNP" if dnp else "yes",
                        " ".join(sorted(refs))])


# ---------------------------------------------------------------------------
def kicad_footprint_dir():
    """Find a local KiCad footprint library, or None if KiCad isn't installed."""
    import glob
    for var in os.environ:
        if var.startswith("KICAD") and var.endswith("FOOTPRINT_DIR"):
            if os.path.isdir(os.environ[var]):
                return os.environ[var]
    patterns = [
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\KiCad\*\share\kicad\footprints"),
        r"C:\Program Files\KiCad\*\share\kicad\footprints",
        "/usr/share/kicad/footprints",
        "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
    ]
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return None


def check_footprints():
    """Every footprint must resolve, or F8 in the PCB editor fails on it.

    Silently skipped when KiCad isn't installed -- the generator still has to
    run on a machine that only has Python.
    """
    root = kicad_footprint_dir()
    if not root:
        return None
    missing = []
    for fp in sorted({p["fp"] for p in design.PARTS if p["fp"]}):
        lib, _, name = fp.partition(":")
        if not os.path.exists(os.path.join(root, lib + ".pretty", name + ".kicad_mod")):
            missing.append(fp)
    return missing


def banner():
    bad = []
    if not morpho_map.VERIFIED:
        bad.append("Morpho pin map")
    if part_pinouts.unverified():
        bad.append("IC pinouts: " + ", ".join(part_pinouts.unverified()))
    return ("DO NOT FABRICATE - unverified: " + "; ".join(bad)) if bad else \
           "Pin tables verified"


def esc(s):
    return str(s).replace("\\", "\\\\").replace('"', '\\"')


def write(name, text):
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as f:
        f.write(text + "\n")


def main():
    by_sheet = {k: [] for k, _ in design.SHEETS}
    seen = {}
    for p in design.PARTS:
        if p["ref"] in seen:
            raise ValueError(f'duplicate reference {p["ref"]}')
        seen[p["ref"]] = p
        by_sheet[p["sheet"]].append(p)

    emit_symbol_lib()
    emit_root()

    nets = {}
    for i, (key, title) in enumerate(design.SHEETS):
        for net, pins in emit_sheet(key, title, by_sheet[key], i + 2).items():
            nets.setdefault(net, []).extend(pins)

    emit_pcb(nets)
    emit_project()
    emit_lib_table()
    emit_bom(design.PARTS)

    # A net with one pin on it is a wiring mistake, not a style question.
    floating = {k: v for k, v in nets.items() if len(v) < 2}
    print(f"{len(design.PARTS)} parts, {len(nets)} nets, {len(design.SHEETS)} sheets")
    if floating:
        print("\nFLOATING NETS (single connection):")
        for k, v in sorted(floating.items()):
            print(f"  {k:<18} {v[0]}")
    else:
        print("no floating nets")

    missing_fp = check_footprints()
    if missing_fp is None:
        print("footprints: not checked (no local KiCad install found)")
    elif missing_fp:
        print(f"\nUNRESOLVED FOOTPRINTS ({len(missing_fp)}) - 'Update PCB from "
              f"Schematic' will fail on these:")
        for fp in missing_fp:
            print(f"  {fp}")
    else:
        print("footprints: all resolve against the local KiCad libraries")

    if design.FP_PLACEHOLDER:
        print("\nPLACEHOLDER FOOTPRINTS - correct land pattern still to be drawn:")
        for fp, why in sorted(design.FP_PLACEHOLDER.items()):
            print(f"  {fp}\n    {why}")

    if morpho_map.VERIFIED and not part_pinouts.unverified():
        print("\npin tables verified - board is fabricable")
    else:
        print(f"\n{banner()}")
        write("UNVERIFIED.md",
              "# Do not fabricate\n\n"
              "Generated with unverified pin tables:\n\n"
              + ("- `tools/morpho_map.py` (Nucleo Morpho headers)\n"
                 if not morpho_map.VERIFIED else "")
              + "".join(f"- `tools/part_pinouts.py` -> `{k}`\n"
                        for k in part_pinouts.unverified())
              + "\nThe netlist is correct by signal name. Only the physical pin "
                "numbers are pending.\nFix the tables, re-run `python "
                "tools/generate.py`, and this file disappears.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
