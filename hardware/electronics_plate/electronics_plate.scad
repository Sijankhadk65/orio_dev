// =====================================================================
//  Electronics pegboard + board carriers
//
//  The pegboard is a plate with an M3 hole grid. Each board sits on its
//  own small printed carrier with standoffs matching the board's holes.
//  The carrier's fixing holes are spaced on the grid pitch, so it can be
//  bolted anywhere on the pegboard, in any of four orientations.
//  Edge notches and closed windows let wires pass between stacked levels.
//
//  Modelled lying flat as printed: X = plate width, Y = plate depth,
//  Z = thickness, origin at the plate's front-left bottom corner.
//  (A 360 x 10 x 280 panel standing upright is the same part rotated.)
//
//  Board-local coordinates: origin at the board's bottom-left corner seen
//  from the component side; for a Nucleo the ST-LINK/USB end is +Y.
//
//  OpenSCAD 2021.01 or newer, no libraries.
// =====================================================================

/* [Render] */
part = "assembly";        // [assembly, pegboard, nucleo_carrier, pcb_carrier]
show_boards = true;       // transparent board previews in the assembly (not part of any STL)

/* [Pegboard] */
plate_w = 360;            // plate width (X)
plate_d = 280;            // plate depth (Y)
plate_t = 10;             // plate thickness (Z)
corner_r = 8;             // plate corner radius
edge_step_w = 5;          // width of the stepped-down ledge around the top edge (0 = none)
edge_step_depth = 5;      // how far the ledge sits below the top face
pitch = 10;              // hole grid pitch
edge_min = 6;             // minimum hole-centre distance to the plate edge (grid is centred)
hole_style = "nut";       // [nut, through, tap] M3 hole + hex nut trap underneath, plain M3 clearance, or self-tap
hole_d = 3.4;             // M3 clearance
tap_d = 2.5;              // pilot hole for M3 self-tapping screws
m3_nut_af = 5.5;          // M3 nut across flats
m3_nut_t = 2.4;           // M3 nut thickness
nut_fit = 0.3;            // extra across-flats clearance for nut traps
hole_fn = 24;             // facets per grid hole (keeps the 900+ holes quick to render)

/* [Wire pass-throughs between stacked levels] */
// Edge notches are open to the plate edge, so a cable can be dropped in
// sideways without feeding its connector through.
notch_w = 30;             // notch width along the edge
notch_depth = 20;         // how far the notch reaches into the plate
notches_front_back = [60, 180, 300];  // notch centres (X) along both the front and back edges
notches_left_right = [];  // notch centres (Y) along both the left and right edges, e.g. [140]
// Optional closed windows inside the plate, sized for pushing a plug through.
wire_windows = [];        // window centres [X, Y], e.g. [[120, 50], [240, 230]]
wire_window_size = [40, 15];  // window size [X, Y]
opening_r = 5;            // corner radius of notches and windows (no sharp corners to cut insulation)
opening_keep = 1.5;       // grid holes closer than this to an opening are left out

/* [Carriers] */
carrier_t = 3;            // carrier plate thickness
carrier_margin = 6;       // carrier edge beyond its fixing holes
head_clear = 5;           // fixing holes stay at least this far outside the board outline
board_clearance = 12;     // board underside above the pegboard top (Nucleo morpho pins poke out underneath)
standoff_d = 8;           // standoff outer diameter
fixing = "insert";        // [insert, nut, tap] how the board screws hold in the standoffs
insert_d = 4.0;           // hole for an M3 heat-set insert
insert_len = 5.7;         // insert length
window_r = 3;             // carrier window corner radius
label_depth = 0.6;        // engraved carrier label depth (0 = none)
label_size = 4;           // carrier label text height

/* [Nucleo-64 (both boards)] */
nucleo_size = [70, 82.5]; // board outline
// M3 holes in the MCU section, board-local. Taken from KiCad's official
// STM32_Nucleo-64_Morpho template (MB1136). Check them against your boards.
nucleo_holes = [[10.87, 54.57], [59.13, 53.30], [43.89, 2.60]];
nucleo_window = [30, 18]; // wiring window through the carrier, centred under the board

/* [Custom PCB - placeholder until measured] */
pcb_size = [100, 80];     // PCB outline
pcb_hole_inset = 4;       // corner holes this far in from each edge...
pcb_holes_custom = [];    // ...or list the holes explicitly, board-local, e.g. [[4,4],[96,4],[4,76],[96,76]]
pcb_window = [50, 30];    // wiring window through the carrier

/* [Assembly preview] */
// Where each carrier goes: grid [column, row] of its first fixing hole
// (lower-left before rotation), and a rotation about that hole.
nucleo1_name = "DRIVETRAIN L152RE";
nucleo1_at = [2, 8];
nucleo1_rot = 0;          // 0 / 90 / 180 / 270
nucleo2_name = "MOTION C031C6";
nucleo2_at = [12, 8];
nucleo2_rot = 0;
pcb_name = "PCB";
pcb_at = [22, 9];
pcb_rot = 0;

$fn = 40;

// =====================================================================
//  Derived values
// =====================================================================
nx = floor((plate_w - 2 * edge_min) / pitch) + 1;   // grid columns
ny = floor((plate_d - 2 * edge_min) / pitch) + 1;   // grid rows
gx0 = (plate_w - (nx - 1) * pitch) / 2;
gy0 = (plate_d - (ny - 1) * pitch) / 2;
function hole_xy(ij) = [gx0 + ij[0] * pitch, gy0 + ij[1] * pitch];

// Every wire opening as a rectangle [xmin, ymin, xmax, ymax]. Notches run past the
// plate edge far enough that their rounded ends fall outside the plate.
notch_ext = opening_r + 1;
opening_rects = concat(
    [for (x = notches_front_back, y = [[-notch_ext, notch_depth], [plate_d - notch_depth, plate_d + notch_ext]])
        [x - notch_w / 2, y[0], x + notch_w / 2, y[1]]],
    [for (y = notches_left_right, x = [[-notch_ext, notch_depth], [plate_w - notch_depth, plate_w + notch_ext]])
        [x[0], y - notch_w / 2, x[1], y + notch_w / 2]],
    [for (c = wire_windows)
        [c[0] - wire_window_size[0] / 2, c[1] - wire_window_size[1] / 2,
         c[0] + wire_window_size[0] / 2, c[1] + wire_window_size[1] / 2]]);
function near_opening(p, m) =
    len([for (r = opening_rects) if (p[0] > r[0] - m && p[0] < r[2] + m && p[1] > r[1] - m && p[1] < r[3] + m) 1]) > 0;

// Grid holes, minus those that would break into an opening (checked against the
// largest feature per hole, the nut trap, so both layers drop the same holes).
hole_keep = (hole_style == "nut" ? (m3_nut_af + nut_fit) / cos(30) : hole_d) / 2 + opening_keep;
grid_pts = [for (i = [0 : nx - 1], j = [0 : ny - 1]) let(p = hole_xy([i, j])) if (!near_opening(p, hole_keep)) p];

pcb_holes = len(pcb_holes_custom) > 0 ? pcb_holes_custom :
    [for (x = [pcb_hole_inset, pcb_size[0] - pcb_hole_inset], y = [pcb_hole_inset, pcb_size[1] - pcb_hole_inset]) [x, y]];

// Fixing-hole span of a carrier: the board plus head clearance, rounded up to whole grid steps.
function span(size) = [for (k = [0, 1]) ceil((size[k] + 2 * head_clear) / pitch) * pitch];
function board_offset(size) = (span(size) - size) / 2;   // board origin in carrier coordinates

// [name, board size, board holes, window, grid position, rotation]
carriers = [
    [nucleo1_name, nucleo_size, nucleo_holes, nucleo_window, nucleo1_at, nucleo1_rot],
    [nucleo2_name, nucleo_size, nucleo_holes, nucleo_window, nucleo2_at, nucleo2_rot],
    [pcb_name,     pcb_size,    pcb_holes,    pcb_window,    pcb_at,     pcb_rot]];

function rot2(p, r) = [p[0] * cos(r) - p[1] * sin(r), p[0] * sin(r) + p[1] * cos(r)];
function outline_pts(c) = let(s = span(c[1]), m = carrier_margin)
    [for (p = [[-m, -m], [s[0] + m, -m], [-m, s[1] + m], [s[0] + m, s[1] + m]]) hole_xy(c[4]) + rot2(p, c[5])];
function fix_pts(c) = let(s = span(c[1]))
    [for (p = [[0, 0], [s[0], 0], [0, s[1]], s]) hole_xy(c[4]) + rot2(p, c[5])];
function cmin(c) = [min([for (p = outline_pts(c)) p[0]]), min([for (p = outline_pts(c)) p[1]])];
function cmax(c) = [max([for (p = outline_pts(c)) p[0]]), max([for (p = outline_pts(c)) p[1]])];
function overlap(p, q) = cmin(p)[0] < cmax(q)[0] && cmin(q)[0] < cmax(p)[0] &&
                         cmin(p)[1] < cmax(q)[1] && cmin(q)[1] < cmax(p)[1];

// =====================================================================
//  Checks and report
// =====================================================================
assert(hole_style == "nut" || hole_style == "through" || hole_style == "tap", "hole_style must be nut, through or tap");
assert(fixing == "insert" || fixing == "nut" || fixing == "tap", "fixing must be insert, nut or tap");
assert(pitch - (m3_nut_af + nut_fit) / cos(30) >= 2 || hole_style != "nut", "pitch too small for the nut traps");
assert(fixing != "insert" || insert_len <= board_clearance, "insert longer than the standoff");
assert(edge_step_w >= 0 && edge_step_depth < plate_t - (hole_style == "nut" ? m3_nut_t + 0.3 : 0),
       "edge_step_depth must stay above the nut-trap layer");
assert(min(gx0, gy0) - hole_d / 2 >= edge_step_w + 1, "edge step cuts into the outer grid holes - raise edge_min");

for (c = carriers) {
    assert(c[5] % 90 == 0, str(c[0], ": rotation must be a multiple of 90"));
    for (h = c[2]) assert(h[0] >= 0 && h[1] >= 0 && h[0] <= c[1][0] && h[1] <= c[1][1],
                          str(c[0], ": hole ", h, " is outside the board"));
    for (h = c[2]) assert(abs(h[0] - c[1][0] / 2) > c[3][0] / 2 + standoff_d / 2 ||
                          abs(h[1] - c[1][1] / 2) > c[3][1] / 2 + standoff_d / 2,
                          str(c[0], ": wiring window runs into the standoff at ", h));
    if (part == "assembly") {
        assert(cmin(c)[0] >= edge_step_w && cmin(c)[1] >= edge_step_w &&
               cmax(c)[0] <= plate_w - edge_step_w && cmax(c)[1] <= plate_d - edge_step_w,
               str(c[0], " carrier hangs over the pegboard's stepped edge"));
        for (p = fix_pts(c)) assert(!near_opening(p, hole_keep),
                                    str(c[0], " carrier fixing hole ", p, " lands on a wire opening"));
        for (p = fix_pts(c)) assert(p[0] >= gx0 - 0.01 && p[1] >= gy0 - 0.01 &&
                                    p[0] <= gx0 + (nx - 1) * pitch + 0.01 && p[1] <= gy0 + (ny - 1) * pitch + 0.01,
                                    str(c[0], " carrier fixing hole ", p, " is off the grid"));
    }
}
if (part == "assembly")
    for (i = [0 : len(carriers) - 2], j = [i + 1 : len(carriers) - 1])
        assert(!overlap(carriers[i], carriers[j]), str(carriers[i][0], " and ", carriers[j][0], " carriers overlap"));

for (w = wire_windows)
    assert(w[0] - wire_window_size[0] / 2 >= edge_step_w + 3 && w[1] - wire_window_size[1] / 2 >= edge_step_w + 3 &&
           w[0] + wire_window_size[0] / 2 <= plate_w - edge_step_w - 3 &&
           w[1] + wire_window_size[1] / 2 <= plate_d - edge_step_w - 3,
           str("Wire window at ", w, " is too close to the plate edge - use a notch instead"));
assert(notch_depth < min(plate_w, plate_d) / 2, "notch_depth reaches past the middle of the plate");
assert(2 * opening_r <= min(notch_w, wire_window_size[0], wire_window_size[1]), "opening_r too big for the openings");

echo(str("Pegboard ", plate_w, " x ", plate_d, " x ", plate_t, " mm, ", len(grid_pts), " holes at ", pitch,
         " mm pitch (", nx * ny - len(grid_pts), " left out around ", len(opening_rects), " wire openings)"));
for (c = carriers)
    echo(str(c[0], " carrier: fixing holes ", span(c[1]), " apart (", span(c[1]) / pitch, " grid steps), outline ",
             span(c[1]) + 2 * [carrier_margin, carrier_margin], " mm"));

// =====================================================================
//  Modules
// =====================================================================
module rounded_rect(size, r) {
    offset(r = r) offset(delta = -r) square(size);
}

module grid2d(d, fn) {
    for (p = grid_pts) translate(p) circle(d = d, $fn = fn);
}

module openings2d() {
    for (r = opening_rects) translate([r[0], r[1]]) rounded_rect([r[2] - r[0], r[3] - r[1]], opening_r);
}

module outline2d() {
    rounded_rect([plate_w, plate_d], corner_r);
}

// Built from 2D layers so hundreds of holes render quickly: an optional nut-trap
// layer, the holed body inside the stepped edge, and a hole-free ledge ring
// whose top sits edge_step_depth below the top face.
module pegboard() {
    hd = hole_style == "tap" ? tap_d : hole_d;
    nd = hole_style == "nut" ? m3_nut_t + 0.3 : 0;
    w = edge_step_w;
    if (nd > 0)
        linear_extrude(nd) difference() {
            outline2d();
            grid2d((m3_nut_af + nut_fit) / cos(30), 6);
            openings2d();
        }
    translate([0, 0, nd - 0.01]) linear_extrude(plate_t - nd + 0.01) difference() {
        if (w > 0) offset(r = -w) outline2d(); else outline2d();
        grid2d(hd, hole_fn);
        openings2d();
    }
    if (w > 0)
        translate([0, 0, nd - 0.01]) linear_extrude(plate_t - edge_step_depth - nd + 0.01) difference() {
            outline2d();
            offset(r = -w - 0.5) outline2d();   // overlap the holed body by 0.5 mm so the two fuse
            openings2d();
        }
}

// Hole in one board standoff (standoff runs from z = 0 to board_clearance).
module standoff_hole() {
    if (fixing == "insert") {
        translate([0, 0, board_clearance - insert_len - 0.5]) cylinder(d = insert_d, h = insert_len + 1);
        translate([0, 0, -1]) cylinder(d = hole_d, h = board_clearance);   // screw tip relief, open underneath
    } else if (fixing == "tap") {
        translate([0, 0, -1]) cylinder(d = tap_d, h = board_clearance + 2);
    } else {
        translate([0, 0, -1]) cylinder(d = hole_d, h = board_clearance + 2);
        translate([0, 0, -0.01]) cylinder(d = (m3_nut_af + nut_fit) / cos(30), h = m3_nut_t + 0.3, $fn = 6);
    }
}

// Carrier in its own coordinates: first fixing hole at the origin, standoffs up.
module carrier(size, holes, window, name) {
    s = span(size);
    off = board_offset(size);
    difference() {
        union() {
            translate([-carrier_margin, -carrier_margin, 0])
                linear_extrude(carrier_t) rounded_rect(s + 2 * [carrier_margin, carrier_margin], 4);
            for (h = holes) translate(off + h) cylinder(d = standoff_d, h = board_clearance);
        }
        for (p = [[0, 0], [s[0], 0], [0, s[1]], s]) translate([p[0], p[1], -1]) cylinder(d = hole_d, h = carrier_t + 2);
        for (h = holes) translate(off + h) standoff_hole();
        translate([off[0] + (size[0] - window[0]) / 2, off[1] + (size[1] - window[1]) / 2, -1])
            linear_extrude(carrier_t + 2) rounded_rect(window, window_r);
        if (label_depth > 0 && name != "")
            translate([s[0] / 2, (off[1] - carrier_margin) / 2, carrier_t - label_depth])
                linear_extrude(label_depth + 1)
                    text(name, size = label_size, halign = "center", valign = "center",
                         font = "Liberation Sans:style=Bold");
    }
}

// Transparent board preview in carrier coordinates, with a marker on the Nucleo ST-LINK/USB end.
module board_mock(size, holes) {
    translate([board_offset(size)[0], board_offset(size)[1], board_clearance]) {
        difference() {
            cube([size[0], size[1], 1.6]);
            for (h = holes) translate([h[0], h[1], -1]) cylinder(d = 3.2, h = 4);
        }
        if (size == nucleo_size) translate([size[0] / 2 - 4, size[1] - 6, 1.6]) cube([8, 6, 3]);
    }
}

module assembly() {
    color("Gold") pegboard();
    for (c = carriers) {
        p = hole_xy(c[4]);
        translate([p[0], p[1], plate_t]) rotate(c[5]) {
            color("SteelBlue") carrier(c[1], c[2], c[3], c[0]);
            if (show_boards) %board_mock(c[1], c[2]);
        }
    }
}

if (part == "assembly") assembly();
else if (part == "pegboard") pegboard();
else if (part == "nucleo_carrier") carrier(nucleo_size, nucleo_holes, nucleo_window, "NUCLEO-64");   // print 2
else if (part == "pcb_carrier") carrier(pcb_size, pcb_holes, pcb_window, pcb_name);
else assert(false, str("Unknown part \"", part, "\""));
