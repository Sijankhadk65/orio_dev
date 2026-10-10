// =====================================================================
//  Twin VL53L5CX time-of-flight bracket
//  Hangs two SparkFun VL53L5CX boards in a forward "^" under the front
//  bottom rail (square aluminium tube) of the robot chassis.
//
//  Coordinates (all mm): origin on the floor directly under the apex,
//  +X = robot's right, +Y = forward, +Z = up. The rail runs along X with
//  its FRONT face in the plane y = 0.
//
//  Pieces:
//    head   - the two board plates, the rear tongue and the yaw braces
//    stem   - lower clamp jaw (or zip-tie cradle) + stem + fork (this
//             is "clamp_b": the lower jaw carries the stem, so the
//             clamp-to-stem gussets are one solid print)
//    clamp_a - upper clamp cap (screw style only)
//  With pitch_trim_slots = false, head and stem are fused into one piece.
//
//  OpenSCAD 2021.01 or newer, no libraries.
// =====================================================================

/* [Render] */
part = "assembly";  // [assembly, head_stem, head, stem, clamp_a, clamp_b]
show_fov  = true;   // show the keep-out cones as transparent previews
show_rail = true;   // show the rail as a transparent preview
show_boards = true; // show the PCBs (assembly only)
rail_show_len = 300;// length of the previewed rail, also the length checked against the cones
$fn = 48;           // curve resolution

/* [Rail] */
rail_w = 20;              // rail cross-section width (front-back, Y)
rail_h = 20;              // rail cross-section height (Z)
rail_underside_z = 137;   // rail underside height above the floor

/* [Sensor placement] */
sensor_height = 75;       // optical centre height above the floor
face_yaw = 22.5;          // outward yaw of each board face, degrees
face_pitch = 0;           // tilt of the faces, degrees, + = looking down
apex_y = 1;               // apex position ahead of the rail front face (>= 0)
apex_gap = 2;             // gap between the two boards' inner edges at the apex (along X)
min_ground_clearance = 45;// nothing may go below this height (asserted)

/* [Board] */
board_w = 25.4;           // board width
board_h = 25.4;           // board height
board_t = 1.6;            // PCB thickness
hole_spacing_x = 20.32;   // mounting hole centre spacing, horizontal
hole_spacing_y = 20.32;   // mounting hole centre spacing, vertical
hole_d = 2.7;             // screw hole diameter (M2.5 clearance)
sensor_offset_x = 0;      // aperture offset from board centre, + = to YOUR right looking at the board face
sensor_offset_y = 0;      // aperture offset from board centre, + = up
aperture_above_pcb = 0;   // aperture height above the PCB front face (0 = conservative)

/* [Board plate] */
plate_t = 3;              // thickness of each board's backing plate
plate_margin = 2.5;       // plate size beyond the board on each side
back_clearance = 3;       // standoff height behind the PCB (parts on the back)
standoff_d = 5.5;         // standoff outer diameter
insert_style = "nut";     // [nut, insert] M2.5 hex nut trap on the back, or heat-set insert
insert_d = 3.2;           // hole for an M2.5 heat-set insert
insert_len = 4;           // heat-set insert length
m25_nut_af = 5.0;         // M2.5 nut across flats
m25_nut_t = 2.0;          // M2.5 nut thickness
nut_access = 8;           // clear path behind each nut trap so the nut can be pushed in
header_slot_w = 14;       // header opening width (along the board)
header_slot_h = 5;        // header opening height
header_slot_offset_x = 0; // header opening offset, + = to YOUR right looking at the board face
header_slot_drop = 0.5;   // how far the opening starts below the board's bottom edge

/* [Stem] */
stem_w = 14;              // stem cross-section, along X
stem_t = 6;               // stem cross-section, along Y
stem_flare_h = 10;        // height of the stem-to-fork flare (pitch gusset)
gusset_len = 12;          // leg length of the triangular gussets under the clamp
gusset_t = 3;             // thickness of single gusset fins
cable_slot = true;        // cable channel up the back of the stem
rib_t = 3;                // channel side-rib thickness (along X)
rib_d = 4;                // channel depth (along Y)
loop_t = 1.6;             // thickness of the cable loops bridging the channel

/* [Head joint] */
pitch_trim_slots = true;  // two M3 screws in arc slots for +/- trim; false = one solid piece
pitch_trim_range = 3;     // +/- pitch trim, degrees
tongue_t = 5;             // head tongue thickness (along X)
joint_fit = 0.3;          // clearance each side between tongue and fork cheeks
joint_len = 22;           // joint length along Y
joint_h = 16;             // joint height along Z
joint_gap = 2;            // minimum gap between moving head and fixed stem
joint_screw_edge = 5;     // joint screw centre to joint front/back edge
brace_h = 6;              // height of the rear yaw braces (tongue to plates)

/* [Clamp] */
clamp_style = "screw";    // [screw, ziptie] split clamp with 2 screws, or zip-tie cradle
clamp_wall = 4;           // clamp wall thickness around the rail
clamp_len = 30;           // clamp length along the rail
clamp_fit = 0.3;          // clearance between clamp and rail, per side
clamp_screw_d = 3.4;      // M3 clearance (clamp and joint screws)
clamp_split_gap = 1;      // gap between the two clamp halves so they can be tightened
clamp_flange_h = 6;       // thickness of each clamp flange
m3_nut_af = 5.5;          // M3 nut across flats
m3_nut_t = 2.4;           // M3 nut thickness
nut_fit = 0.3;            // extra across-flats clearance on nut traps
ziptie_w = 5;             // zip-tie slot width
ziptie_t = 2;             // zip-tie slot height

/* [Keep-out] */
fov_half_angle = 33;      // half-angle of the keep-out cone (63 deg diagonal FOV + margin)
fov_len = 60;             // length of the keep-out cone for the preview

/* [Print layout] */
layout_dx = 70;           // spacing between pieces in "head_stem"

// =====================================================================
//  Derived values - no need to edit below here
// =====================================================================
a = face_yaw;
front_of_plate = board_t + back_clearance;   // board face -> plate front
T = front_of_plate + plate_t;                // board face -> plate back
s0 = apex_gap / 2 / cos(a);                  // apex -> board inner edge, along the face
sc = s0 + board_w / 2;                       // apex -> board centre, along the face
zb = sensor_height - sensor_offset_y;        // board centre height
head_top0 = zb + board_h / 2 + plate_margin; // top of the plates (before pitch)
plate_bot0 = zb - board_h / 2 - plate_margin;
so_d = insert_style == "insert" ? max(standoff_d, insert_d + 2.4) : standoff_d;

// Board model: board facing +Y, origin at the centre of its front face.
// Seen from the front, the model's +X is on your LEFT.
sensor_model = [-sensor_offset_x, aperture_above_pcb, sensor_offset_y];
hole_pts = [for (i = [-1, 1], j = [-1, 1]) [i * hole_spacing_x / 2, j * hole_spacing_y / 2]];

function yaw_of(side) = side > 0 ? -a : a;   // side +1 = right board, -1 = left
function rotz(p, ang) = [p[0] * cos(ang) - p[1] * sin(ang), p[0] * sin(ang) + p[1] * cos(ang), p[2]];
function board_centre(side) = [side * sc * cos(a), apex_y - sc * sin(a), zb];
function b2h(side, p) = board_centre(side) + rotz(p, yaw_of(side));   // board model -> head frame

// Pitch axis: along X, through the sensor centres.
piv_y = (b2h(1, sensor_model)[1] + b2h(-1, sensor_model)[1]) / 2;
piv_z = sensor_height;
function rotx_about(p, ang) = let(qy = p[1] - piv_y, qz = p[2] - piv_z)
    [p[0], piv_y + qy * cos(ang) - qz * sin(ang), piv_z + qy * sin(ang) + qz * cos(ang)];
function h2w(p) = rotx_about(p, -face_pitch);                         // head frame -> world
function dir_h2w(d) = h2w(d + [0, piv_y, piv_z]) - [0, piv_y, piv_z];

function sensor_pos(side) = h2w(b2h(side, sensor_model));
function sensor_axis(side) = dir_h2w(rotz([0, 1, 0], yaw_of(side)));

// Clamp
cav_y0 = -rail_w - clamp_fit;
cav_y1 = clamp_fit;
cav_z0 = rail_underside_z - clamp_fit;
cav_z1 = rail_underside_z + rail_h + clamp_fit;
jaw_y0 = cav_y0 - clamp_wall;
jaw_y1 = cav_y1 + clamp_wall;
jaw_z0 = cav_z0 - clamp_wall;                // clamp underside; the stem hangs from here
jaw_z1 = cav_z1 + clamp_wall;
z_split = rail_underside_z + rail_h / 2;
flange_w = clamp_screw_d + 2 * clamp_wall;
flange_y = [jaw_y1 + flange_w / 2, jaw_y0 - flange_w / 2];
cradle_top = cav_z0 + 0.6 * rail_h;
lower_top = clamp_style == "screw" ? z_split - clamp_split_gap / 2 : cradle_top;

// Stem and head joint
stem_yc = -rail_w / 2;
stem_yf = stem_yc + stem_t / 2;
stem_yb = stem_yc - stem_t / 2;
stem_yback = stem_yb - (cable_slot ? rib_d : 0);   // back of the ribs
joint_yf = apex_y - T / cos(a) + 0.5;               // just inside the plate backs at x = 0
joint_yb = joint_yf - joint_len;
r_far = norm([joint_yb - piv_y, head_top0 - piv_z]);
joint_zb = head_top0 + joint_gap + r_far * sin(abs(face_pitch) + pitch_trim_range);
joint_zt = joint_zb + joint_h;
r_top = norm([joint_yb - piv_y, joint_zt - piv_z]);
tongue_top = joint_zt - joint_gap - r_top * sin(pitch_trim_range);
ear_z0 = pitch_trim_slots ? joint_zb : head_top0 - 0.5;   // solid: fuse onto the plate tops, clear of the nut traps
joint_screw_z = joint_zb + joint_h * 0.45;
joint_screws = [[0, joint_yf - joint_screw_edge, joint_screw_z],
                [0, joint_yb + joint_screw_edge, joint_screw_z]];
cheek_t = (stem_w - tongue_t - 2 * joint_fit) / 2;
flare_h = min(stem_flare_h, jaw_z0 - joint_zt);
g_len = min(gusset_len, jaw_z0 - joint_zt);

// =====================================================================
//  Checks and report
// =====================================================================
function cone_angle(p, side) = let(v = p - sensor_pos(side), l = norm(v))
    l < 1e-6 ? 180 : acos(max(-1, min(1, v * sensor_axis(side) / l)));

keepout_head = concat(
    [for (side = [-1, 1], sx = [-1, 1], sz = [-1, 1])
        b2h(side, [sx * board_w / 2, 0, sz * board_h / 2])],                  // both boards' face corners
    [for (z = [zb - board_h / 2, zb + board_h / 2]) [0, apex_y, z]],          // apex
    [for (sz = [-1, 1]) [0, apex_y - front_of_plate / cos(a), zb + sz * (board_h / 2 + plate_margin)]],
    [for (side = [-1, 1], sz = [-1, 1])
        b2h(side, [side * (board_w / 2 + plate_margin), -front_of_plate, sz * (board_h / 2 + plate_margin)])],
    [[0, joint_yf, tongue_top]]);
keepout_world = concat(
    [for (x = [-rail_show_len / 2 : 10 : rail_show_len / 2], z = [rail_underside_z, rail_underside_z + rail_h])
        [x, 0, z]],                                                           // rail front edges
    [for (sx = [-1, 1]) [sx * clamp_len / 2, jaw_y1, jaw_z0]],
    clamp_style == "screw"
        ? [for (sx = [-1, 1]) [sx * clamp_len / 2, jaw_y1 + flange_w, lower_top - clamp_flange_h]] : [],
    [[0, min(stem_yf + g_len, jaw_y1), jaw_z0]],                              // front gusset tip
    [for (sx = [-1, 1]) [sx * stem_w / 2, joint_yf, ear_z0]]);                // fork front corners
keepout_pts = concat(keepout_world, [for (p = keepout_head) h2w(p)]);
min_cone_angle = min([for (p = keepout_pts, side = [-1, 1]) cone_angle(p, side)]);

low_pts = [for (p = concat(
    [for (side = [-1, 1], sx = [-1, 1], y = [-front_of_plate, -T])
        b2h(side, [sx * (board_w / 2 + plate_margin), y, -(board_h / 2 + plate_margin)])],
    [for (y = [apex_y - front_of_plate / cos(a), apex_y - T / cos(a), joint_yb]) [0, y, plate_bot0]]))
    h2w(p)];
lowest_z = min([for (p = low_pts) p[2]]);

function yaw_deg(side) = atan2(sensor_axis(side)[0], sensor_axis(side)[1]);

echo(str("Drop from rail underside to sensor centre: ", rail_underside_z - sensor_height, " mm"));
echo(str("Right sensor aperture (x,y,z): ", sensor_pos(1), "  height ", sensor_pos(1)[2], " mm, looking ",
         yaw_deg(1), " deg right"));
echo(str("Left  sensor aperture (x,y,z): ", sensor_pos(-1), "  height ", sensor_pos(-1)[2], " mm, looking ",
         -yaw_deg(-1), " deg left"));
echo(str("Horizontal coverage about: ", yaw_deg(-1) - 22.5, " .. ", yaw_deg(1) + 22.5, " deg"));
echo(str("Lowest point of the part: ", lowest_z, " mm (limit ", min_ground_clearance, ")"));
echo(str("Closest checked point to a sensor axis: ", min_cone_angle, " deg (keep-out ", fov_half_angle, ")"));
echo(str("Stem length between fork and clamp: ", jaw_z0 - joint_zt, " mm"));

assert(apex_y >= 0, "apex_y must be >= 0 so the boards sit ahead of the rail");
assert(lowest_z >= min_ground_clearance,
       str("Part reaches down to ", lowest_z, " mm, below min_ground_clearance"));
assert(min_cone_angle >= fov_half_angle,
       str("Something is inside a keep-out cone (", min_cone_angle, " deg < ", fov_half_angle, ")"));
assert(jaw_z0 - joint_zt >= 3,
       "Not enough height between the head and the rail for the stem - lower sensor_height or check rail_underside_z");
assert(joint_yf < apex_y && stem_yf < apex_y, "Stem must stay behind the boards");
assert(cheek_t >= m3_nut_t + 1.2, "Fork cheeks too thin for the M3 nut trap - widen stem_w or thin tongue_t");
assert(tongue_top - joint_screw_z >= clamp_screw_d / 2 + 1.5, "Joint too short - raise joint_h");
assert(clamp_style == "screw" || clamp_style == "ziptie", "clamp_style must be \"screw\" or \"ziptie\"");
assert(insert_style == "nut" || insert_style == "insert", "insert_style must be \"nut\" or \"insert\"");

// =====================================================================
//  Helpers
// =====================================================================
module to_world() {   // head frame -> world (applies face_pitch about the sensor axis)
    translate([0, piv_y, piv_z]) rotate([-face_pitch, 0, 0]) translate([0, -piv_y, -piv_z]) children();
}

module place_board(side) {
    translate(board_centre(side)) rotate([0, 0, yaw_of(side)]) children();
}

module hex_prism(af, h) {
    cylinder(d = af / cos(30), h = h, $fn = 6);
}

// Right-angle triangular gussets hanging under z0: one leg along the plane's
// horizontal axis (from x0/y0 to x1/y1), the other dz straight down.
module gusset_xz(x0, x1, y0, th, z0, dz) {   // triangle in the XZ plane
    hull() {
        translate([min(x0, x1), y0, z0 - 0.01]) cube([abs(x1 - x0), th, 0.01]);
        translate([x0 - (x1 > x0 ? 0 : 0.01), y0, z0 - dz]) cube([0.01, th, dz]);
    }
}
module gusset_yz(y0, y1, x0, th, z0, dz) {   // triangle in the YZ plane
    hull() {
        translate([x0, min(y0, y1), z0 - 0.01]) cube([th, abs(y1 - y0), 0.01]);
        translate([x0, y0 - (y1 > y0 ? 0 : 0.01), z0 - dz]) cube([th, 0.01, dz]);
    }
}

// =====================================================================
//  Head (built in the head frame = world with face_pitch = 0)
// =====================================================================

// One board's backing plate with standoffs and header opening, trimmed at x = 0
// so the two plates meet on the centre line. Nothing rises in front of the PCB's
// back face, so there are no walls near the keep-out cone to chamfer.
module board_plate(side) {
    ext = board_w;   // extra plate on the apex side, trimmed away at x = 0
    x0 = side > 0 ? -(board_w / 2 + plate_margin + ext) : -(board_w / 2 + plate_margin);
    x1 = side > 0 ? board_w / 2 + plate_margin : board_w / 2 + plate_margin + ext;
    intersection() {
        translate([side > 0 ? 0 : -500, -500, -500]) cube([500, 1000, 1000]);
        place_board(side) difference() {
            union() {
                translate([x0, -T, -board_h / 2 - plate_margin])
                    cube([x1 - x0, plate_t, board_h + 2 * plate_margin]);
                for (p = hole_pts)
                    translate([p[0], -front_of_plate - 0.01, p[1]]) rotate([-90, 0, 0])
                        cylinder(d = so_d, h = back_clearance + 0.01);
            }
            // header connector passes backwards through here
            translate([-header_slot_offset_x - header_slot_w / 2, -T - 1, -board_h / 2 - header_slot_drop])
                cube([header_slot_w, plate_t + 2, header_slot_h]);
        }
    }
}

// Screw holes and nut traps (or insert holes). Cut at head level so the nut
// access path also clears the tongue behind the apex.
module board_holes(side) {
    place_board(side) for (p = hole_pts) translate([p[0], 0, p[1]]) {
        if (insert_style == "insert") {
            translate([0, -board_t + 0.01, 0]) rotate([90, 0, 0])
                cylinder(d = insert_d, h = insert_len + 0.5);
        } else {
            translate([0, -T - 1, 0]) rotate([-90, 0, 0]) cylinder(d = hole_d, h = T + 2);
            translate([0, -T - nut_access, 0]) rotate([-90, 0, 0])
                hex_prism(m25_nut_af + nut_fit, nut_access + m25_nut_t + 0.2);
        }
    }
}

// Vertical web behind the apex; carries the joint to the stem.
module tongue() {
    translate([-tongue_t / 2, joint_yb, plate_bot0])
        cube([tongue_t, joint_yf - joint_yb, tongue_top - plate_bot0]);
}

// Horizontal struts from the back of the tongue to the back of each plate:
// triangulate tongue and plates so the head cannot flex in yaw.
module brace(side) {
    pad = b2h(side, [0, -T + 1.5, 0]);
    hull() {
        translate([-tongue_t / 2, joint_yb, head_top0 - brace_h]) cube([tongue_t, 4, brace_h]);
        translate([pad[0], pad[1], head_top0 - brace_h]) cylinder(r = 1.5, h = brace_h);
    }
}

// Slot for a joint screw: the screw's head-frame position swept +/- pitch_trim_range about the pivot.
module arc_slot(p) {
    n = 6;
    for (k = [-n : n - 1]) hull() for (j = [k, k + 1])
        translate(rotx_about(p, j * pitch_trim_range / n)) rotate([0, 90, 0])
            cylinder(d = clamp_screw_d, h = tongue_t + 4, center = true);
}

module head() {
    difference() {
        union() {
            board_plate(1);
            board_plate(-1);
            tongue();
            brace(1);
            brace(-1);
        }
        board_holes(1);
        board_holes(-1);
        // screws sit at fixed world positions; in the head frame that is +face_pitch
        if (pitch_trim_slots) for (p = joint_screws) arc_slot(rotx_about(p, face_pitch));
    }
}

// =====================================================================
//  Stem (world frame): bar, flare into the fork, gussets, cable channel
// =====================================================================
module stem() {
    difference() {
        union() {
            translate([-stem_w / 2, stem_yback, joint_zt - 0.01])
                cube([stem_w, stem_yf - stem_yback, jaw_z0 - joint_zt + 1]);
            // fork (solid block when there is no trim joint)
            translate([-stem_w / 2, joint_yb, ear_z0]) cube([stem_w, joint_len, joint_zt - ear_z0]);
            // flare from fork to bar: the stem-to-head pitch gusset
            hull() {
                translate([-stem_w / 2, joint_yb, joint_zt - 0.01]) cube([stem_w, joint_len, 0.01]);
                translate([-stem_w / 2, stem_yback, joint_zt + flare_h - 0.01])
                    cube([stem_w, stem_yf - stem_yback, 0.01]);
            }
            // stem-to-clamp gussets: sideways (roll/yaw) ...
            for (sx = [-1, 1])
                gusset_xz(sx * stem_w / 2, sx * clamp_len / 2, stem_yb, stem_t, jaw_z0,
                          min(g_len, clamp_len / 2 - stem_w / 2));
            // ... and fore-aft (pitch)
            gusset_yz(stem_yf, min(stem_yf + g_len, jaw_y1), -gusset_t / 2, gusset_t, jaw_z0,
                      min(g_len, jaw_y1 - stem_yf));
            for (x = cable_slot ? [-(stem_w / 2 - rib_t / 2), stem_w / 2 - rib_t / 2] : [0])
                let(th = cable_slot ? rib_t : gusset_t)
                    gusset_yz(stem_yback, jaw_y0, x - th / 2, th, jaw_z0, min(g_len, stem_yback - jaw_y0));
        }
        if (pitch_trim_slots) {
            translate([-(tongue_t / 2 + joint_fit), joint_yb - 1, ear_z0 - 1])
                cube([tongue_t + 2 * joint_fit, joint_len + 2, joint_zt - ear_z0 + 1]);
            for (p = joint_screws) {
                translate(p) rotate([0, 90, 0]) cylinder(d = clamp_screw_d, h = stem_w + 2, center = true);
                translate([stem_w / 2 - m3_nut_t - 0.3, p[1], p[2]]) rotate([0, 90, 0])
                    hex_prism(m3_nut_af + nut_fit, m3_nut_t + 1);
            }
        }
        if (cable_slot)
            translate([-(stem_w / 2 - rib_t), stem_yb - 50, joint_zt + 2])
                cube([stem_w - 2 * rib_t, 50, jaw_z0 - joint_zt - 2 + 0.01]);
    }
    // loops across the open back of the channel to hold the cables
    if (cable_slot)
        for (f = [0.35, 0.8])
            translate([-(stem_w / 2 - rib_t) - 0.01, stem_yback, joint_zt + 2 + f * (jaw_z0 - joint_zt - 2) - 1.5])
                cube([stem_w - 2 * rib_t + 0.02, loop_t, 3]);
}

// =====================================================================
//  Clamp (world frame)
// =====================================================================
module rail_cavity(top) {
    translate([-clamp_len, cav_y0, cav_z0]) cube([2 * clamp_len, cav_y1 - cav_y0, top - cav_z0]);
}

// Lower jaw (screw) or U-cradle (ziptie); the stem hangs from its underside.
module clamp_lower() {
    if (clamp_style == "screw") {
        difference() {
            union() {
                translate([-clamp_len / 2, jaw_y0, jaw_z0]) cube([clamp_len, jaw_y1 - jaw_y0, lower_top - jaw_z0]);
                for (fy = flange_y)
                    translate([-clamp_len / 2, fy - flange_w / 2, lower_top - clamp_flange_h])
                        cube([clamp_len, flange_w, clamp_flange_h]);
            }
            rail_cavity(cav_z1);
            for (fy = flange_y) {
                translate([0, fy, jaw_z0 - 1]) cylinder(d = clamp_screw_d, h = 100);
                translate([0, fy, lower_top - clamp_flange_h - 0.01]) hex_prism(m3_nut_af + nut_fit, m3_nut_t + 0.3);
            }
        }
    } else {
        // Pushed up onto the rail from below; each zip tie runs through a tunnel in
        // the floor, up the outside of both walls and over the top of the rail.
        difference() {
            translate([-clamp_len / 2, jaw_y0, jaw_z0]) cube([clamp_len, jaw_y1 - jaw_y0, cradle_top - jaw_z0]);
            rail_cavity(cav_z1 + 10);
            for (sx = [-1, 1])
                translate([sx * (clamp_len / 2 - ziptie_w / 2 - 2.5) - ziptie_w / 2, jaw_y0 - 1,
                           (jaw_z0 + cav_z0) / 2 - ziptie_t / 2])
                    cube([ziptie_w, jaw_y1 - jaw_y0 + 2, ziptie_t]);
        }
    }
}

// Upper cap (screw style only).
module clamp_upper() {
    if (clamp_style == "screw") {
        z0 = z_split + clamp_split_gap / 2;
        difference() {
            union() {
                translate([-clamp_len / 2, jaw_y0, z0]) cube([clamp_len, jaw_y1 - jaw_y0, jaw_z1 - z0]);
                for (fy = flange_y)
                    translate([-clamp_len / 2, fy - flange_w / 2, z0]) cube([clamp_len, flange_w, clamp_flange_h]);
            }
            rail_cavity(cav_z1);
            for (fy = flange_y) translate([0, fy, z0 - 1]) cylinder(d = clamp_screw_d, h = 100);
        }
    }
}

module clamp() {
    clamp_lower();
    clamp_upper();
}

// =====================================================================
//  Previews
// =====================================================================
module fov_cone(side) {
    place_board(side) translate(sensor_model) rotate([-90, 0, 0])
        cylinder(h = fov_len, r1 = 0, r2 = fov_len * tan(fov_half_angle));
}

module board_mock(side) {
    place_board(side) {
        color("ForestGreen") difference() {
            translate([-board_w / 2, -board_t, -board_h / 2]) cube([board_w, board_t, board_h]);
            for (p = hole_pts) translate([p[0], 0, p[1]]) rotate([90, 0, 0])
                cylinder(d = hole_d, h = 10, center = true);
        }
        color("Black") translate([-sensor_offset_x - 3.2, -0.01, sensor_offset_y - 1.5]) cube([6.4, 1.5, 3]);
    }
}

// =====================================================================
//  Pieces and print layout
// =====================================================================
module stem_piece() {   // "stem" == "clamp_b"
    difference() {
        union() {
            stem();
            if (!pitch_trim_slots) to_world() head();
        }
        // keep the fused fork out of the board screw and nut paths
        if (!pitch_trim_slots) to_world() { board_holes(1); board_holes(-1); }
    }
    clamp_lower();
}

// Printed upside down: flat clamp split face (or cradle rim) on the bed, stem vertical.
module print_stem() {
    translate([0, 0, lower_top]) rotate([180, 0, 0]) stem_piece();
}

// Printed upright on the flat bottom edges of the plates, faces vertical.
module print_head() {
    translate([0, 0, -plate_bot0]) head();
}

// Printed with its flat top on the bed.
module print_clamp_a() {
    translate([0, 0, jaw_z1]) rotate([180, 0, 0]) clamp_upper();
}

module assembly() {
    color("SteelBlue") stem_piece();
    color("SteelBlue") clamp_upper();
    if (pitch_trim_slots) color("Orange") to_world() head();
    if (show_boards) to_world() { board_mock(1); board_mock(-1); }
    if (show_fov) %to_world() { fov_cone(1); fov_cone(-1); }
    if (show_rail) %translate([-rail_show_len / 2, -rail_w, rail_underside_z]) cube([rail_show_len, rail_w, rail_h]);
}

if (part == "assembly") assembly();
else if (part == "head_stem") {
    print_stem();
    if (pitch_trim_slots) translate([layout_dx, 0, 0]) print_head();
}
else if (part == "head") {
    if (pitch_trim_slots) print_head();
    else echo("pitch_trim_slots = false: the head is part of \"stem\" / \"head_stem\"");
}
else if (part == "stem" || part == "clamp_b") print_stem();
else if (part == "clamp_a") {
    if (clamp_style == "screw") print_clamp_a();
    else echo("clamp_style = \"ziptie\": there is no clamp_a");
}
else assert(false, str("Unknown part \"", part, "\""));
