"""Talos 5-axis arm + gripper: parametric CadQuery model and STEP assembly export.

Servo layout follows the SO-ARM100 / SO-101 pattern: six Feetech STS3215 bus
servos, each sitting in a printed shell, with the next link clamped to the
output horn on one side and the passive idler horn on the other. The printed
geometry is original: faceted octagonal beams, a fluted column base and
separate hub caps.

Printing: the upper arm and forearm stand on a flat underside (Z_BED) and print
without support; their idler-side clevis plates are a separate part screwed
into a pocket, so the gap between the clevis plates is open.

Wiring: the bus board (same Waveshare adapter as SO-101) sits on a deck behind
the column. Each servo's two bus connectors face the side opposite its shaft;
every pocket has a bay for the mated plugs, and the daisy-chain cable runs
from bay to bay through clips on the arm's -Y side.

Units are mm. Every link is built in its own joint frame (X along the link,
Y along the joint axis) and placed by forward kinematics, so changing the
joint angles in POSE re-poses the whole assembly.

    python talos_arm.py                 # writes talos_arm_assembly.step
    python talos_arm.py --pose zero     # straight-up pose
"""

import argparse
import math
from pathlib import Path

import cadquery as cq
from OCP.gp import gp_Trsf

# ---------------------------------------------------------------- dimensions

H_PAN = 62.0      # base pan horn face above the table
H_SH = 46.0       # shoulder axis above the pan horn face
L1 = 116.0        # shoulder -> elbow
L2 = 135.0        # elbow -> wrist flex
X_ROLL = 63.0     # wrist flex axis -> wrist roll horn face (roll servo plugs clear the forearm)
XJ, ZJ = 42.0, 6.0  # jaw pivot in the gripper frame

# Y bands around a horizontal servo (servo mid-plane at y = 0)
SHELL_OUT = 18.4                   # shell walls' outer faces beside the servo (clevis plates start at 18.9)
CLV_IN, CLV_OUT = 18.9, 22.9       # clevis plates on the next link's horns
IDLER_HUB_R = 10.5                 # idler-side plate hub: just over the passive horn, inside the plugs
HORN_FACE = 18.7                   # output horn's outer face, from the servo mid-plane
PASSIVE_FACE = 18.55               # passive horn's outer face, from the servo mid-plane
PAD_R = 9.5                        # clevis plate pads that bear on the horns (inside the shell's R10.8 opening)
PASSIVE_SCREW = 6                  # M3 x 6 into the passive horn, heads on the idler plate's outer face: the
                                   # tip stops 1.65 into the horn (2.2 thick at its holes), 1.9 short of the case
Z_BED = -15.0                      # arm links print standing on this flat underside
WRIST_TOP = 13.6                   # the wrist prints upside down on its flat top face
DOUBLER = 1.4                      # the bump's channel runs 0.6 past the shell wall; this leaves 0.8 over it
SHELL_IN = 16.9                    # shell wall inner face on the connector side (bottom face 16.5 + FIT_C)
WEB_R = 4.3                        # centre hole widened where the M3 counterbores would leave a 0.2 mm web
BASE_SEAT = 19.6                   # pan servo's case screw seats: level with the floor of the passive horn opening
FOREARM = dict(h0=28.0, h1=30.8, w_mid=32.0, h_mid=24.0)  # arm_link() sizes for the forearm

POSES = {
    # pan, shoulder, elbow, wrist flex, wrist roll, gripper (degrees)
    "ready": (0, 20, 95, 35, 0, 18),
    "zero": (0, 0, 0, 0, 0, 0),
}

C_SHELL = cq.Color(0.88, 0.88, 0.86)     # off-white printed structure
C_GRAPHITE = cq.Color(0.33, 0.34, 0.37)  # hub caps, moving jaw
C_SERVO = cq.Color(0.08, 0.08, 0.09)
C_HORN = cq.Color(0.78, 0.79, 0.80)
C_PLUG = cq.Color(0.80, 0.78, 0.72)
C_STEEL = cq.Color(0.45, 0.46, 0.48)
C_PCB = cq.Color(0.16, 0.22, 0.20)


# ---------------------------------------------------------------- helpers

def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, y, z))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


def prism_xz(pts, y0, y1):
    """Extrude a closed (x, z) polygon over y0..y1."""
    return cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0)).translate((0, y0, 0))


def octagon(w, h, c):
    """Chamfered rectangle as (y, z) points, width w, height h, chamfer c."""
    a, b = w / 2, h / 2
    return [(-a + c, -b), (a - c, -b), (a, -b + c), (a, b - c),
            (a - c, b), (-a + c, b), (-a, b - c), (-a, -b + c)]


def loc_r(rows, t=(0, 0, 0)):
    """Location from a 3x3 rotation (row-major) and a translation."""
    T = gp_Trsf()
    (a, b, c), (d, e, f), (g, h, i) = rows
    T.SetValues(a, b, c, t[0], d, e, f, t[1], g, h, i, t[2])
    return cq.Location(T)


def T(x=0, y=0, z=0):
    return cq.Location(cq.Vector(x, y, z))


def rot(axis, deg):
    return cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), float(deg))


def try_chamfer(wp, selector, d):
    try:
        return wp.edges(selector).chamfer(d)
    except Exception:
        return wp


def moved(wp, loc):
    return cq.Workplane("XY").add(wp.val().moved(loc))


def compound(wps):
    return cq.Workplane("XY").add(cq.Compound.makeCompound([w.val() for w in wps]))


# Servo orientations (servo frame: shaft = +Z, body centre at x = -12.5).
R_LINK = ((1, 0, 0), (0, 0, 1), (0, -1, 0))   # shaft +Y, body trails along -X
R_SHOULDER = ((0, 1, 0), (0, 0, 1), (1, 0, 0))  # shaft +Y, body hangs along -Z
R_ROLL = ((0, 0, 1), (0, -1, 0), (1, 0, 0))   # shaft +X, body hangs along -Z

# rotations taking +Z to each axis direction (for screws)
TO_DIR = {"+z": ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "-z": ((1, 0, 0), (0, -1, 0), (0, 0, -1)),
          "+y": ((1, 0, 0), (0, 0, 1), (0, -1, 0)), "-y": ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
          "+x": ((0, 0, 1), (0, 1, 0), (-1, 0, 0)), "-x": ((0, 0, -1), (0, 1, 0), (1, 0, 0))}


# ---------------------------------------------------------------- servo
# Feetech STS3215, measured from the Feetech CAD model that SO-101's assembly uses
# (cad/reference/so101/step/STS3215_from_SO101.step, re-measured by servo_reference.py).
# Every STS3215 variant (C001 7.4 V, C018/C047 12 V) has this same case.
#
# Servo frame: output shaft on +Z through the origin, output horn face at z = 18.7, body
# toward -X (centre x = -12.5). The case is not a plain box with flat caps:
#   * the top face (output side) is at 15.5 away from the shaft (x < -5.5), with a raised
#     rib 14 wide on it (16.45, with four pips to 16.6); round the shaft the case drops to
#     14.0 under the horn;
#   * the bottom face (connector side) is at -16.5 away from the shaft, with a bump 12 x 18.4
#     beside the connectors (rim at -18.4, modelled to -18.5); the passive horn sits in a
#     recess round the shaft, where the case is at -15.0;
#   * the idler boss the passive horn turns on reaches -19.1, past the passive horn face.
# Both horns come with the servo and each is held by one centre screw.

CASE_X = (-35.1, 10.1)          # case length 45.2
CASE_HW = 12.4                  # case half width (24.8)
CORE_Z = (-15.0, 14.0)          # full case outline, shaft end included
FACE_X = -5.5                   # the top face and bottom face run from the far end to here
TOP_FACE = 15.5                 # top face (output side), x < FACE_X
RIB = (-34.3, -9.0, 7.0, 16.6)  # raised rib on the top face: x from, x to, half width, top
BOTTOM_FACE = 16.5              # bottom face (connector side) at z = -16.5, x < FACE_X
BUMP = (-29.7, -17.7, 9.2, 18.5)  # bump beside the connectors: x from, x to, half width, down to z = -18.5
HORN_R, HUB_R = 9.6, 5.3        # output horn disc z 16.4 to 18.7, hub z 14.0 to 16.4
HORN_DISC = 16.4
PASSIVE_DISC = 16.6             # passive horn disc z -16.6 to -18.55 (hub R4.46 inside the recess)
SPLINE = (3.0, 17.2)            # bare output spline (horn off): radius, top
BOSS = (3.0, 19.1)              # idler boss: radius, tip at z = -19.1
HORN_PCD_R = 7.0                # four M3 holes in each horn on a 14 mm circle

HORN_HOLES = [(HORN_PCD_R * math.cos(math.radians(45 + 90 * k)),
               HORN_PCD_R * math.sin(math.radians(45 + 90 * k))) for k in range(4)]  # tapped M3
# M2 self-tapping holes (1.6 mm) in the case, measured on the Feetech CAD. Each face has a second
# pair beside the shaft, at x = -8.3; only the base uses it (CASE_BACK_SHAFT).
CASE_FRONT = ((-29.0, 10.25), (-29.0, -10.25))      # top face
CASE_BACK = ((-32.75, 10.25), (-32.75, -10.25))     # bottom face
CASE_BACK_SHAFT = ((-8.3, 10.25), (-8.3, -10.25))   # bottom face, beside the shaft
CONN_X = (-16.5, -11.5)                        # connector sockets, plugs go in from the bottom
CONN_Y = ((0.5, 10.0), (-10.0, -0.5))          # two 3-pin sockets, side by side
PLUG_ZONE = (-17.6, -10.6, -10.6, 10.6, -28.5)  # mated plugs + cable bend (x, x, y, y, z)

# Printing tolerance: FDM parts come out a little small inside, so every pocket is the
# servo plus FIT_C all round; anything that only has to pass (rib, bump, boss, spline,
# plugs) gets PASS_C. The horn openings are R10.8: the R9.6 horns go through with 1.2 to spare.
FIT_C = 0.4
PASS_C = 0.5
HORN_OPENING = 10.8
# Links and gripper: the rib's way in runs under one of the top-face case screws, so both
# top-face screws seat 1.0 over the rib channel (heads stand 0.7 proud, out of the next link's reach)
FRONT_SEAT = RIB[3] + PASS_C + 1.0


def sts3215():
    """Feetech STS3215: (body, horns, mated plugs), servo frame."""
    cx = (CASE_X[0] + CASE_X[1]) / 2
    body = (cq.Workplane("XY").rect(CASE_X[1] - CASE_X[0], 2 * CASE_HW).extrude(CORE_Z[1] - CORE_Z[0])
            .translate((cx, 0, CORE_Z[0]))
            .edges("|Z and >X").chamfer(3.0)
            .edges("|Z and <X").fillet(1.0))
    recess = cyl_z(11.0, -30, 30)  # the top and bottom faces stop short of the horns
    top = box(CASE_X[0], FACE_X, -CASE_HW + 0.1, CASE_HW - 0.1, CORE_Z[1], TOP_FACE).cut(recess)
    rib = box(RIB[0], RIB[1], -RIB[2], RIB[2], TOP_FACE - 0.01, RIB[3]).cut(cyl_z(11.0, -30, 30))
    bottom = box(CASE_X[0], FACE_X, -CASE_HW + 0.1, CASE_HW - 0.1, -BOTTOM_FACE, CORE_Z[0]).cut(recess)
    bump = box(BUMP[0], BUMP[1], -BUMP[2], BUMP[2], -BUMP[3], -BOTTOM_FACE + 0.01)
    body = body.union(top).union(rib).union(bottom).union(bump)
    body = body.union(cyl_z(SPLINE[0], CORE_Z[1] - 0.01, SPLINE[1]))           # output spline
    body = body.union(cyl_z(BOSS[0], -BOSS[1], CORE_Z[0] + 0.01))              # idler boss
    body = body.cut(cyl_z(1.25, -BOSS[1] - 0.1, CORE_Z[0]))                    # boss tapped hole (M3)
    for x0, x1 in ((CONN_X[0] - 1.0, CONN_X[1] + 0.6),):                       # plug opening
        body = body.cut(box(x0, x1, -10.5, 10.5, -BOTTOM_FACE - 0.1, CORE_Z[0] + 0.5))
    for x, y in CASE_FRONT:
        body = body.cut(cyl_z(0.8, TOP_FACE - 4.0, TOP_FACE + 0.1, x, y))
    for x, y in CASE_BACK + CASE_BACK_SHAFT:
        body = body.cut(cyl_z(0.8, -BOTTOM_FACE - 0.1, -BOTTOM_FACE + 4.0, x, y))
    horn = cyl_z(HORN_R, HORN_DISC, HORN_FACE).union(cyl_z(HUB_R, CORE_Z[1] + 0.05, HORN_DISC))
    for x, y in HORN_HOLES:
        horn = horn.cut(cyl_z(1.25, HORN_DISC - 1, HORN_FACE + 1, x, y))
    horn = horn.cut(cyl_z(SPLINE[0] + 0.05, 0, SPLINE[1] + 0.05)).cut(cyl_z(1.6, 0, 19))  # on the spline
    idler = (cyl_z(HORN_R, -PASSIVE_FACE, -PASSIVE_DISC)
             .union(cyl_z(4.46, -PASSIVE_DISC, CORE_Z[0] - 0.2))
             .cut(cyl_z(BOSS[0] + 0.05, -20, -14)))  # passive horn, turns on the boss
    for x, y in HORN_HOLES:
        idler = idler.cut(cyl_z(1.25, -20, -PASSIVE_DISC + 1, x, y))
    plugs = compound([box(CONN_X[0] - 0.4, CONN_X[1] + 0.4, y0 - 0.2, y1 + 0.2, -26.0, CORE_Z[0])
                      for y0, y1 in CONN_Y])
    return body, horn.union(idler), plugs


def _env(c, p):
    """Servo envelope grown by c (case) and p (rib, bump, spline, boss), servo frame, as boxes
    and cylinders: [("box", lo, hi) | ("cyl", r, z0, z1)]."""
    x0, x1 = CASE_X
    return [("box", (x0 - c, -CASE_HW - c, CORE_Z[0]), (x1 + c, CASE_HW + c, CORE_Z[1])),
            ("box", (x0 - c, -CASE_HW - c, CORE_Z[1] - 0.01), (x1 + c, CASE_HW + c, TOP_FACE + c)),
            # the rib channel runs on into the R10.8 horn opening and the spline channel, so no sliver is left
            ("box", (RIB[0] - p, -RIB[2] - p, TOP_FACE), (-SPLINE[0] - p, RIB[2] + p, RIB[3] + p)),
            ("box", (x0 - c, -CASE_HW - c, -BOTTOM_FACE - c), (x1 + c, CASE_HW + c, CORE_Z[0] + 0.01)),
            ("box", (BUMP[0] - p, -BUMP[2] - p, -BUMP[3] - p), (BUMP[1] + p, BUMP[2] + p, -BOTTOM_FACE)),
            ("cyl", SPLINE[0] + p, CORE_Z[1], SPLINE[1] + c),  # c on top: leaves 0.8 of the 18.4 wall
            ("cyl", BOSS[0] + p, -BOSS[1] - p, CORE_Z[0])]


def servo_pocket(c=FIT_C, p=PASS_C):
    """Pocket for a servo and its mated plugs, in the servo frame: the case plus c all round,
    the rib, bump, spline and boss plus p, the plug bay, and the R10.8 openings for both horns."""
    env = None
    for e in _env(c, p):
        if e[0] == "box":
            (ax0, ay0, az0), (ax1, ay1, az1) = e[1], e[2]
            s = box(ax0, ax1, ay0, ay1, az0, az1)
        else:
            s = cyl_z(e[1], e[2], e[3])
        env = s if env is None else env.union(s)
    x0, x1, y0, y1, z0 = PLUG_ZONE
    env = env.union(box(x0, x1, y0, y1, z0, CORE_Z[0]))                              # plug bay
    env = env.union(box(x1, x1 + 1.6, -4.5, 4.5, -BOSS[1] - p, CORE_Z[0]))           # no knife edge at the R10.8
    env = env.union(cyl_z(HORN_OPENING, -BOSS[1] - p, CORE_Z[0]))                    # passive horn
    env = env.union(cyl_z(HORN_OPENING, CORE_Z[1], HORN_FACE + 0.8))                 # output horn
    return env


def pocket_in(loc):
    return moved(servo_pocket(), loc)


# How each servo goes into its part: the way back out of the pocket (servo frame), and whether
# the horns may stay on. The parts cut this path; part_checks.py and servo_reference.py slide
# the servo along it.
SERVO_ENTRY = {"base": ("+z", True), "shoulder": ("+x", False), "upper arm": ("+y", False),
               "forearm": ("+y", False), "wrist": ("+z", True), "gripper": ("-y", False)}


def servo_entry(direction="+y", c=FIT_C, p=PASS_C, depth=60.0):
    """Path the servo sweeps when it is pushed into its pocket from `direction` (servo frame),
    with both horns off. "+y" means the servo arrives from +Y and moves toward -Y.

    Everything that stands off the case gets a channel out to the opening: the rib, the
    bump, the plug bay, the bare spline and the idler boss.
    """
    ax, sg = "xyz".index(direction[1]), 1 if direction[0] == "+" else -1

    def swept(lo, hi):
        lo, hi = list(lo), list(hi)
        if sg > 0:
            hi[ax] = depth
        else:
            lo[ax] = -depth
        return box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])

    cut = None
    for e in _env(c, p):
        if e[0] == "box":
            s = swept(e[1], e[2])
        else:  # a cylinder on the shaft axis: itself plus a slot of its width out along the way in
            _, r, z0, z1 = e
            s = cyl_z(r, z0, z1)
            if ax != 2:
                s = s.union(swept((0, -r, z0), (0, r, z1)) if ax == 0 else swept((-r, 0, z0), (r, 0, z1)))
            else:
                s = s.union(swept((-r, -r, z0), (r, r, z1)))
        cut = s if cut is None else cut.union(s)
    x0, x1, y0, y1, z0 = PLUG_ZONE
    cut = cut.union(swept((x0, y0, z0), (x1, y1, CORE_Z[0])))                             # plug bay
    # along Y the boss channel is merged with the plug bay so no loose strip of wall is left
    if ax == 1:
        r = BOSS[0] + p
        cut = cut.union(swept((x0, 0, z0), (r, 0, CORE_Z[0])))
    return cut


# ---------------------------------------------------------------- fasteners

SCREW = {3: dict(shank=1.2, head_r=2.85, head_h=1.65, clear=1.7, cbore=3.25),   # clear: 3.4 for FDM
         2.5: dict(shank=0.95, head_r=2.35, head_h=1.38, clear=1.4, cbore=2.7),
         2: dict(shank=0.7, head_r=1.75, head_h=1.04, clear=1.2, cbore=2.1)}


def screw(d, length, at, direction):
    """Button-head screw: head underside at `at`, shank along `direction` ('+x', '-y', ...).

    The shank is modelled at thread-root size so it sits inside a tapped hole.
    """
    s = SCREW[d]
    wp = (cq.Workplane("XY").circle(s["shank"]).extrude(length)
          .union(cq.Workplane("XY").circle(s["head_r"]).extrude(-s["head_h"])
                 .faces("<Z").edges().fillet(0.5 * s["head_h"])))
    return moved(wp, loc_r(TO_DIR[direction], at))


def case_screw_length(seat, face):
    """Stock M2 length that puts the tip 2.9 to 3.9 mm into the case from a head seat at `seat`.
    (SO-101's M2 x 6 go 3.8 mm in, through its 2.2 mm walls.)"""
    return math.ceil(seat - face + 2.9 - 1e-6)


def case_screws(front_seat=None, back_seat=None, back_holes=None):
    """M2 case screws into the servo's mounting holes: (cuts, screws), servo frame.

    A seat is the distance from the servo mid-plane to the screw-head underside;
    the counterbore runs from the seat outward through whatever is in the way.
    """
    s, cuts, screws = SCREW[2], [], []
    for seat, holes, sign, face in ((front_seat, CASE_FRONT, 1, TOP_FACE),
                                    (back_seat, back_holes or CASE_BACK, -1, BOTTOM_FACE)):
        if seat is None:
            continue
        length = case_screw_length(seat, face)
        for x, y in holes:
            z_face, z_seat = sign * face, sign * seat
            lo, hi = sorted((z_face, z_seat))
            cuts.append(cyl_z(s["clear"], lo - 0.1, hi + 0.1, x, y))
            lo, hi = sorted((z_seat, sign * 45))
            cuts.append(cyl_z(s["cbore"], lo, hi, x, y))
            screws.append(screw(2, length, (x, y, z_seat), "-z" if sign > 0 else "+z"))
    cut = cuts[0]
    for c in cuts[1:]:
        cut = cut.union(c)
    return cut, compound(screws)


def horn_holes_y(y0, y1, center_r=3.5):
    """Four M3 clearance holes on the horn pattern plus a centre access hole, along Y."""
    cut = cyl_y(center_r, y0, y1)
    for x, z in HORN_HOLES:
        cut = cut.union(cyl_y(SCREW[3]["clear"], y0, y1, x, z))
    return cut


def horn_screws_y(y_head, length=6):
    """Four M3 button heads on a plate face at y_head, shanks toward -Y into the horn."""
    return [screw(3, length, (x, y_head, z), "-y") for x, z in HORN_HOLES]


def cable_clip(at, rows):
    """Snap-in cable clip: a 5 x 3.5 mm tunnel with a slit, for one bus cable.

    Built in (u = along the cable, v = across, w = out of the face) and placed by `rows`.
    """
    clip = box(-3, 3, -5, 5, -1, 5)
    clip = clip.cut(box(-4, 4, -2.5, 2.5, 0.8, 4.3)).cut(box(-4, 4, -0.75, 0.75, 4.2, 6))
    clip = try_chamfer(clip, "|X and >Z", 1.0)
    return moved(clip, loc_r(rows, at))


# ---------------------------------------------------------------- parts

# Waveshare Bus Servo Adapter (A): 42 x 33 mm, 2.5 mm holes on 37 x 28 mm (same board as SO-101)
BOARD = dict(x=(-88.0, -55.0), y=(-21.0, 21.0), z=15.0, holes=((-57.5, 18.5), (-57.5, -18.5),
                                                              (-85.5, 18.5), (-85.5, -18.5)))


def base_parts():
    top = H_PAN - 2.8
    plinth = cq.Workplane("XY").polygon(12, 132).extrude(10)
    deck = cq.Workplane("XY").polyline([(-40, -32), (-88, -32), (-100, -20), (-100, 20),
                                        (-88, 32), (-40, 32)]).close().extrude(10)
    plinth = try_chamfer(plinth.union(deck), ">Z", 2.5)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = 54 * math.cos(a), 54 * math.sin(a)
        plinth = plinth.cut(cyl_z(2.2, -1, 11, x, y)).cut(cyl_z(4.2, 6.5, 11, x, y))
    collar = cyl_z(45, 10, 14).faces(">Z").edges().chamfer(2.0)
    tower = cyl_z(40, 10, top).faces(">Z").edges().chamfer(1.5)
    body = plinth.union(collar).union(tower)
    for x, y in BOARD["holes"]:  # standoffs for the bus servo board
        body = body.union(cyl_z(3.2, 9, BOARD["z"], x, y)).cut(cyl_z(1.05, 8, BOARD["z"] + 1, x, y))  # M2.5 pilot
    for k in range(18):  # column fluting
        a = math.radians(10 + 20 * k)
        x, y = 41.4 * math.cos(a), 41.4 * math.sin(a)
        z0 = 28 if x < 0 and abs(y) < 9 else 19  # the two over the cable tunnel start above it
        flute = cyl_z(2.6, z0, 52, x, y)
        flute = flute.union(cq.Workplane("XY").sphere(2.6).translate((x, y, z0)))
        flute = flute.union(cq.Workplane("XY").sphere(2.6).translate((x, y, 52)))
        body = body.cut(flute)
    servo_loc = T(0, 0, H_PAN - 18.7)
    body = body.cut(pocket_in(servo_loc)).cut(box(-36, 11, -CASE_HW - FIT_C, CASE_HW + FIT_C, top - 1, top + 5))
    body = body.cut(box(-60, -13, -6, 6, 14.8, 24.5))  # cable tunnel: plug bay -> board deck
    # Four case screws from below, through the pocket floor: both ends of the servo's bottom face
    # are pulled down onto the floor, so the arm's overturning moment can't rock the servo in its pocket.
    cut, screws = case_screws(back_seat=BASE_SEAT, back_holes=CASE_BACK + CASE_BACK_SHAFT)
    x0, x1, _, y1, z0 = PLUG_ZONE
    for sy in (1, -1):  # the pair beside the shaft runs 0.2 mm from the plug bay: open that wall up
        y = sorted((sy * (CASE_BACK_SHAFT[0][1] - SCREW[2]["cbore"]), sy * y1))
        cut = cut.union(box(x1, CASE_BACK_SHAFT[0][0], *y, z0, -BASE_SEAT))
    body = body.cut(moved(cut, servo_loc))
    pcb = box(*BOARD["x"], *BOARD["y"], BOARD["z"], BOARD["z"] + 1.6)
    for x, y in BOARD["holes"]:
        pcb = pcb.cut(cyl_z(1.25, BOARD["z"] - 1, BOARD["z"] + 3, x, y))
    parts = cq.Workplane("XY").add(cq.Compound.makeCompound([
        moved(screws, servo_loc).val(),
        *[screw(2.5, 6, (x, y, BOARD["z"] + 1.6), "-z").val() for x, y in BOARD["holes"]]]))
    return dict(base=body, pcb=pcb, screws=parts), servo_loc


def shoulder_parts():
    servo_loc = loc_r(R_SHOULDER, (0, 0, H_SH))
    turntable = (cyl_z(37, 0, 6).faces(">Z").edges().chamfer(2.0)
                 .faces("<Z").edges().chamfer(0.8))
    cradle = box(-17, 17, -SHELL_OUT, SHELL_OUT, 5, 34)
    cradle = try_chamfer(cradle, "|Y", 3.0)
    posts = box(-17, 17, -SHELL_OUT, SHELL_OUT, 30, H_SH + 4)
    posts = posts.cut(box(-12.9, 12.9, -30, 30, 0, 80))
    # cheeks start on the cradle wall (no slit between them) and carry the case screw heads
    cheeks = prism_xz([(-24, 6), (24, 6), (14, 21), (-14, 21)], SHELL_OUT, 25.0)
    cheeks = cheeks.union(prism_xz([(-24, 6), (24, 6), (14, 21), (-14, 21)], -25.0, -SHELL_OUT))
    # doubler over the back-pad channel above the -Y cheek (the upper arm's idler plate can't reach it)
    doubler = box(-10.5, 10.5, -SHELL_OUT - DOUBLER, -SHELL_OUT, H_SH - 31.0, 34)
    body = turntable.union(cradle).union(posts).union(cheeks).union(doubler)
    # clip on the cradle's back face anchors the pan service loop
    body = body.union(cable_clip((-17, -9, 21), ((0, 0, -1), (0, 1, 0), (1, 0, 0))))
    body = body.cut(pocket_in(servo_loc)).cut(box(-CASE_HW - FIT_C, CASE_HW + FIT_C, -SHELL_IN, TOP_FACE + FIT_C, H_SH + CASE_X[0] - FIT_C, 80))
    body = body.cut(moved(servo_entry(SERVO_ENTRY["shoulder"][0]), servo_loc))  # in from the top, far end first
    body = body.cut(cyl_y(4, -30, 30, 0, 14))
    # pan horn: centre access and four M3 screws from inside the cradle
    body = body.cut(cyl_z(3.5, -1, 11)).cut(cyl_z(WEB_R, 4, 11))
    screws = []
    for x, y in HORN_HOLES:
        body = body.cut(cyl_z(SCREW[3]["clear"], -1, 4, x, y)).cut(cyl_z(SCREW[3]["cbore"], 4, 11, x, y))
        screws.append(screw(3, 6, (x, y, 4), "-z"))
    for k in range(4):  # arc windows in the turntable, clear of the cradle and cheeks
        win = (cq.Workplane("XY").moveTo(27 * math.cos(math.radians(-18)), 27 * math.sin(math.radians(-18)))
               .threePointArc((27, 0), (27 * math.cos(math.radians(18)), 27 * math.sin(math.radians(18))))
               .lineTo(33 * math.cos(math.radians(18)), 33 * math.sin(math.radians(18)))
               .threePointArc((33, 0), (33 * math.cos(math.radians(-18)), 33 * math.sin(math.radians(-18))))
               .close().extrude(8).translate((0, 0, -1)).rotate((0, 0, 0), (0, 0, 1), 90 * k))
        body = body.cut(win)
    cut, cscrews = case_screws(front_seat=18.5, back_seat=18.5)  # through the cheeks
    body = body.cut(moved(cut, servo_loc))
    screws.append(moved(cscrews, servo_loc))
    return dict(shoulder=body, screws=compound(screws)), servo_loc


def clevis_plate(hub_r=16.0, reach=30.0, half_h=15.0):
    """Output-side clevis plate (+Y), screwed to the servo horn."""
    t = CLV_OUT - CLV_IN
    out = (cq.Workplane("XZ").circle(hub_r).extrude(-t)
           .union(prism_xz([(0, -half_h), (reach, -half_h), (reach, half_h), (0, half_h)], 0, t))
           .translate((0, CLV_IN, 0)))
    # the plate clears the parent shell by 0.5 mm, so a 0.2 mm pad reaches the horn face
    out = out.union(cyl_y(PAD_R, HORN_FACE, CLV_IN))
    return out.cut(horn_holes_y(CLV_IN - 1, CLV_OUT + 1))


def between_plates():
    """Space between a link's clevis plates, kept clear for the parent servo's shell.
    The horn pads stay: they sit inside the shell's R10.8 horn openings."""
    keep = cyl_y(PAD_R, HORN_FACE, CLV_IN).union(cyl_y(PAD_R, -CLV_IN, -PASSIVE_FACE))
    return cyl_y(21.0, -CLV_IN, CLV_IN).union(box(-30, 0, -CLV_IN, CLV_IN, -30, 30)).cut(keep)


def idler_neck(reach=30.0, half_h=15.0, grow=0.0):
    """(x, z) outline of the idler plate's waisted neck, grown by `grow` for a pocket."""
    g = grow
    return [(0, -IDLER_HUB_R - g), (6, -8.5 - g), (0.6 * reach, -8.5 - g), (reach + g, -half_h - g),
            (reach + g, half_h + g), (0.6 * reach, 8.5 + g), (6, 8.5 + g), (0, IDLER_HUB_R + g)]


# arm-link idler plates screw into a pocket on the beam's -Y side: (x, z) of the two M3 x 10
IDLER_TAB_SCREWS = ((26.0, 7.0), (26.0, -7.0))


def idler_plate(reach=30.0, half_h=15.0, tab=False):
    """Idler-side clevis plate (-Y) on the servo's passive horn, in the link frame.

    The plate stays inside IDLER_HUB_R near the axis and its neck is pinched to
    17 mm just past the hub, so the parent servo's bus plugs clear it at full
    bend. With `tab`, it is a separate part held by two M3 screws through its
    end into the arm-link beam (TAL-110). The four M3 x 6 into the passive horn
    sit on the plate's outer face (no counterbores).
    """
    t = CLV_OUT - CLV_IN
    idl = (cq.Workplane("XZ").circle(IDLER_HUB_R).extrude(-t)
           .union(prism_xz(idler_neck(reach, half_h), 0, t))
           .translate((0, -CLV_OUT, 0)))
    # Like the output side, the plate is screwed to the passive horn, which turns on the
    # servo's idler boss; nothing is screwed into the boss itself. A pad reaches the horn face.
    idl = idl.union(cyl_y(PAD_R, -CLV_IN, -PASSIVE_FACE))
    idl = idl.cut(cyl_y(3.5, -CLV_OUT - 1, -PASSIVE_FACE + 1))       # centre: clears the boss
    for x, z in HORN_HOLES:  # plain holes: the heads sit on the outer face, so the bed face is one solid layer
        idl = idl.cut(cyl_y(SCREW[3]["clear"], -CLV_OUT - 1, -PASSIVE_FACE + 1, x, z))
    if tab:
        for x, z in IDLER_TAB_SCREWS:
            idl = idl.cut(cyl_y(SCREW[3]["clear"] + 0.1, -CLV_OUT - 1, -CLV_IN + 1, x, z))
    return idl


def clevis(hub_r=16.0, reach=30.0, half_h=15.0):
    """Output plate (+Y) on the horn and slim idler plate (-Y) on the passive horn, as one solid."""
    return clevis_plate(hub_r, reach, half_h).union(idler_plate(reach, half_h))


def clevis_screws():
    """Four M3 x 6 into the output horn and four M3 x 6 into the passive horn."""
    passive = [screw(3, PASSIVE_SCREW, (x, -CLV_OUT, z), "+y") for x, z in HORN_HOLES]
    return horn_screws_y(CLV_OUT) + passive


def idler_tab_screws():
    """Two M3 x 10 through the separate idler plate into the beam."""
    return [screw(3, 10, (x, -CLV_OUT, z), "+y") for x, z in IDLER_TAB_SCREWS]


def hub_cap(large=True):
    """One snap-on cap in its own frame: inner face on y = 0, outer face at y = 3.4.

    The outer face is plain, so it prints face down on one solid first layer.
    """
    r, recess = (12.5, 10.4) if large else (10.2, 3.4)
    return (cyl_y(r, 0, 3.4).faces(">Y").edges().chamfer(0.8)
            .cut(cyl_y(recess, -0.1, 1.8)))                        # recess over the screw heads


GAUGE_CLEAR = ((FIT_C, PASS_C), (0.15, 0.25))  # fit gauge windows: the pockets' clearance, and a tight one


def gauge_window(cp):
    """Rectangles (x0, x1, y0, y1) of the fit-gauge window at clearance (c, p), in the servo's
    X (length) and Z (thickness) directions with the servo frame origin at (0, 0): the case,
    plus notches for the rib, the bare spline, the bump and the idler boss."""
    c, p = cp
    return [(CASE_X[0] - c, CASE_X[1] + c, -BOTTOM_FACE - c, TOP_FACE + c),
            (RIB[0] - p, RIB[1] + p, 0.0, RIB[3] + p),
            (-SPLINE[0] - p, SPLINE[0] + p, 0.0, SPLINE[1] + p),
            (BUMP[0] - p, BUMP[1] + p, -BUMP[3] - p, 0.0),
            (-BOSS[0] - p, BOSS[0] + p, -BOSS[1] - p, 0.0)]


def fit_gauge(h=8.0, wall=3.0):
    """Servo fit gauge (TAL-111), printed flat. Returns (solid, window origins).

    Each window is the profile the servo presents (horns off) when it is pushed into a
    pocket sideways (along its width): at the pockets' clearance (one notch) and tight
    (two notches). Print it before the arm, with the same settings. The bare servo should
    push through the one-notch window by hand and not through the two-notch one.
    """
    out, origins, y_at = None, [], 0.0
    for k, cp in enumerate(GAUGE_CLEAR):
        c, p = cp
        ylo, yhi = -BOSS[1] - p, SPLINE[1] + p
        ox, oy = -CASE_X[0] + c + wall, y_at + wall - ylo
        frame = box(0, CASE_X[1] - CASE_X[0] + 2 * c + 2 * wall, y_at, y_at + (yhi - ylo) + 2 * wall, 0, h)
        for x0, x1, y0, y1 in gauge_window(cp):
            frame = frame.cut(box(x0 + ox, x1 + ox, y0 + oy, y1 + oy, -1, h + 1))
        for n in range(k + 1):  # notches on the end wall's top edge: 1 = clearance, 2 = tight
            ny = y_at + wall + 6 + 4 * n
            frame = frame.cut(box(-1, 1.2, ny, ny + 1.5, h - 1.5, h + 1))
        out = frame if out is None else out.union(frame)
        origins.append((ox, oy))
        y_at += (yhi - ylo) + wall
    return out, origins


def hub_caps():
    """Cap over the output-horn screws. The idler side has none: a cap wide enough for the
    passive-horn screws would hit the parent servo's plugs."""
    return hub_cap(True).translate((0, CLV_OUT, 0))


def ruled_loft(stations):
    """Faceted loft through [(x, [(y, z), ...]), ...] sections normal to X."""
    x_prev = stations[0][0]
    wp = cq.Workplane("YZ", origin=(x_prev, 0, 0)).polyline(stations[0][1]).close()
    for x, pts in stations[1:]:
        wp = wp.workplane(offset=x - x_prev).polyline(pts).close()
        x_prev = x
    return wp.loft(ruled=True)


def ridge_profile(h):
    zb, zt = h / 2 - 3, h / 2 + 1.8
    return [(-4, zb), (4, zb), (4, zt - 1.6), (2.4, zt), (-2.4, zt), (-4, zt - 1.6)]


CLIP_NEG_Y = ((1, 0, 0), (0, 0, -1), (0, 1, 0))  # clip on a -Y face, cable along X


def octagon_flat(w, h, c, z_bed=Z_BED):
    """Beam profile: top at h / 2 with chamfered corners, flat square-cornered underside on z_bed.

    The underside runs the full width, so on the bed it joins the clevis plate's
    edge into one first layer.
    """
    a, t = w / 2, h / 2
    return [(-a, z_bed), (a, z_bed), (a, t - c), (a - c, t), (-a + c, t), (-a, t - c)]


def arm_link(L, h0=30.0, h1=30.8, w_mid=34.0, h_mid=25.0, spine=True, servo_entry_below=True):
    """Clevis output plate at the origin, waisted octagonal beam, servo shell at x = L.

    Prints standing on its flat underside (z = Z_BED): the waist dips on top only.
    The idler plate is a separate part (TAL-110) screwed into a pocket on the -Y
    side, so the gap between the clevis plates needs no support.
    Returns (body, servo location, screws, idler plate).
    """
    servo_loc = loc_r(R_LINK, (L, 0, 0))
    x0, x1 = 20.0, L - 36.0
    xm = (x0 + x1) / 2
    # (x, width, height, chamfer) at the clevis end, the waist and the shell end
    st = ((x0, 2 * CLV_OUT, h0, 6.0), (xm, w_mid, h_mid, 5.0), (x1, 2 * SHELL_OUT, h1, 6.0))
    beam = ruled_loft([(x, octagon_flat(w, h, c)) for x, w, h, c in st])
    xa, b = L - 38.2, h1 / 2  # shell: chamfered on top only, flat underside
    shell = prism_xz([(xa, Z_BED), (L, Z_BED), (L, b), (xa + 4, b), (xa, b - 4)], -SHELL_OUT, SHELL_OUT)
    shell = shell.union(cyl_y(18.0, -SHELL_OUT, SHELL_OUT, L, 0))
    body = clevis_plate().union(beam).union(shell)
    body = body.cut(box(-40, L + 40, -40, 40, -60, Z_BED))  # flat underside: hub and shell trimmed
    # nose floor just past the servo: ties both shell walls together on the bed (stays inside R21)
    body = body.union(box(L + 9.0, L + 13.5, -SHELL_OUT, SHELL_OUT, Z_BED, Z_BED + 4))  # servo entry cuts its back
    # doubler over the back-pad channel in the -Y wall (beyond the next link's reach, |elbow| <= 105 deg)
    body = body.union(box(L - 31.0, L - 16.4, -SHELL_OUT - DOUBLER, -SHELL_OUT, Z_BED, 10.5))
    if spine:
        ridge = ruled_loft([(x, ridge_profile(h)) for x, _, h, _ in st])
        body = body.union(ridge.intersect(box(x0 + 8, x1 - 4, -10, 10, -40, 40)))
    body = body.cut(prism_xz(idler_neck(grow=0.3), -CLV_OUT - 1, -CLV_IN))  # idler plate pocket, 0.3 all round
    for x, z in IDLER_TAB_SCREWS:
        body = body.cut(cyl_y(1.25, -CLV_IN - 0.1, -CLV_IN + 8.0, x, z))  # M3 pilot, 2.5 mm
    # twin windows, one per tapered segment, kept 1.5 mm inside the chamfer facets
    for (xa, _, ha, ca), (xb, _, hb, cb), ia, ib in ((*st[:2], 8, 3), (*st[1:], 3, 4)):
        def half(x, xa=xa, xb=xb, ha=ha, hb=hb, ca=ca, cb=cb):
            t = (x - xa) / (xb - xa)
            return (ha + t * (hb - ha)) / 2 - (ca + t * (cb - ca)) - 1.5
        a, b = xa + ia, xb - ib
        ka, kb = half(a), half(b)
        body = body.cut(prism_xz([(a, 0), (a + ka, -half(a + ka)), (b - kb, -half(b - kb)), (b, 0),
                                  (b - kb, half(b - kb)), (a + ka, half(a + ka))], -30, 30))
    body = body.union(cable_clip((xm, -w_mid / 2 - 0.2, 0), CLIP_NEG_Y))  # bus cable, -Y side
    body = body.cut(between_plates())
    body = body.cut(pocket_in(servo_loc))
    if servo_entry_below:  # servo pushes straight up from below: floor open under its whole length
        body = body.cut(moved(servo_entry(SERVO_ENTRY["upper arm"][0]), servo_loc))
    else:
        body = body.cut(box(L - 35.5, L - 4, -SHELL_IN, SHELL_IN, -40, -12))
    cut, cscrews = case_screws(front_seat=FRONT_SEAT, back_seat=SHELL_IN + 1.0)
    body = body.cut(moved(cut, servo_loc))
    screws = compound(clevis_screws() + idler_tab_screws() + [moved(cscrews, servo_loc)])
    return body, servo_loc, screws, idler_plate(tab=True)


def octagon_xz(xa, xb, h):
    b = h / 2
    return [(xa + 4, -b), (xb, -b), (xb, b), (xa + 4, b), (xa, b - 4), (xa, -b + 4)]


def wrist_parts():
    xs = X_ROLL - 18.7  # roll servo shaft origin
    servo_loc = loc_r(R_ROLL, (xs, 0, 0))
    # Prints upside down on a flat top face at z = WRIST_TOP: the cradle top, the lugs and the
    # output plate all reach it (hubs trimmed), so the output plate is built from the bed up.
    top = WRIST_TOP
    # side lugs only: the cradle's back wall already spans the middle
    bridge = box(20.5, 31, -CLV_OUT, CLV_OUT, -12, top)
    cradle = box(20.5, X_ROLL - 2.8, -15, 15, -38.5, top)
    cradle = try_chamfer(cradle, "|X", 4.0)
    body = clevis(reach=24, half_h=12).union(box(0, 24, CLV_IN, CLV_OUT, 0, top))
    body = body.cut(horn_holes_y(CLV_IN - 1, CLV_OUT + 1))  # re-cut the horn holes the extension covered
    body = body.union(bridge).union(cradle).cut(box(-40, 80, -40, 40, top, 40))
    body = body.union(cable_clip((46, -15.2, -30), CLIP_NEG_Y))
    body = body.cut(between_plates())  # keeps both horn pads
    body = body.cut(pocket_in(servo_loc)).cut(box(xs - 16.0, 80, -CASE_HW - FIT_C, CASE_HW + FIT_C, CASE_X[0] - FIT_C, CASE_X[1] + FIT_C))
    # roll servo plugs face the wrist axis; the cable leaves the bay under the -Y lug
    body = body.cut(box(xs - 28.5, xs - 15.5, -18.6, 0, -17.6, -10.6))
    body = body.cut(box(xs - 28.5, xs - 15.5, -30, 0, -17.6, -12.4))
    body = body.cut(prism_xz(hex_pts(xs - 8, 54 + 3, -12, 8), -30, 30))  # side windows onto the servo
    cut, cscrews = case_screws(back_seat=18.0)  # from the back wall, on the clevis side
    body = body.cut(moved(cut, servo_loc))
    screws = compound(clevis_screws() + [moved(cscrews, servo_loc)])
    return dict(wrist=body, screws=screws), servo_loc


def hex_pts(xa, xb, zc, hw):
    """Stretched hexagon in (x, z): the window motif used across the arm."""
    return [(xa, zc), (xa + hw, zc - hw), (xb - hw, zc - hw), (xb, zc),
            (xb - hw, zc + hw), (xa + hw, zc + hw)]


FINGER_HW = 10.0  # jaw fingers are 20 mm wide, centred on the roll axis
ROLL_CBORE_RUN = 14.0  # the lower roll-horn counterbores run on to here, a screw length past the seat


def gripper_parts():
    servo_loc = loc_r(R_LINK, (XJ, 0, ZJ))
    disc = cyl_x(15.5, 0, 4).faces(">X").edges().chamfer(0.6)
    housing = box(0, XJ, -SHELL_OUT, SHELL_OUT, ZJ - 15.4, ZJ + 15.4)
    housing = try_chamfer(housing, "|Y", 3.0)
    housing = housing.union(cyl_y(18.0, -SHELL_OUT, SHELL_OUT, XJ, ZJ))
    # fixed jaw: a keel under the housing, then a slim finger
    keel = prism_xz([(24, -9.4), (56, -9.4), (56, -17), (30, -17), (24, -13)],
                    -FINGER_HW, FINGER_HW)
    finger = prism_xz([(48, -9.4), (54, -9.4), (66, -0.5), (108, -0.5), (108, -6),
                       (64, -17), (48, -17)], -FINGER_HW, FINGER_HW)
    doubler = box(XJ - 31.0, XJ - 16.4, -SHELL_OUT - DOUBLER, -SHELL_OUT, ZJ - 10.5, ZJ + 15.4)  # bump channel
    body = disc.union(housing).union(keel).union(finger).union(doubler)
    body = body.cut(pocket_in(servo_loc))
    body = body.cut(moved(servo_entry(SERVO_ENTRY["gripper"][0]), servo_loc))  # jaw servo goes in from the top
    body = body.cut(prism_xz(hex_pts(9, 33, ZJ, 7), -30, 30))  # side windows onto the servo
    for k in range(6):  # grip serrations
        x = 72 + 6 * k
        body = body.cut(prism_xz([(x, 0), (x + 1.5, -1.6), (x + 3, 0)], -13, 24))
    # roll horn: four M3 from inside the housing (counterbored), centre access
    screws = []
    body = body.cut(cyl_x(3.5, -1, 7)).cut(cyl_x(WEB_R, 3.5, 7))
    for y, z in HORN_HOLES:
        # the lower two heads sit below the servo pocket's floor: their counterbores run on as
        # grooves in the floor, so the screw can be laid in and slid home
        body = body.cut(cyl_x(SCREW[3]["clear"], -1, 3.5, y, z))
        body = body.cut(cyl_x(SCREW[3]["cbore"], 3.5, ROLL_CBORE_RUN if z < 0 else 7, y, z))
        screws.append(screw(3, 6, (3.5, y, z), "-x"))
    cut, cscrews = case_screws(front_seat=FRONT_SEAT, back_seat=SHELL_IN + 1.0)
    body = body.cut(moved(cut, servo_loc))
    screws.append(moved(cscrews, servo_loc))
    return dict(gripper=body, screws=compound(screws)), servo_loc


def moving_jaw():
    """Moving jaw in its pivot frame (pivot at origin, rotates about Y)."""
    plate = (cq.Workplane("XZ").circle(13).extrude(-(CLV_OUT - CLV_IN))
             .union(prism_xz([(0, -13), (34, -5.5), (34, 9.1), (0, 13)], 0, CLV_OUT - CLV_IN))
             .translate((0, CLV_IN, 0))
             .union(cyl_y(PAD_R, HORN_FACE, CLV_IN)))  # pad on the horn face
    profile = [(22, -5.5), (64, -5.5), (64, -1.5), (52, 5), (30, 10), (22, 9)]
    finger = prism_xz(profile, -FINGER_HW, FINGER_HW)
    knuckle = prism_xz([(22, -5.5), (34, -5.5), (34, 9.1), (30, 10), (22, 9)],
                       FINGER_HW, CLV_IN)  # joins the finger to the horn plate
    body = plate.union(finger).union(knuckle)
    for k in range(6):
        x = 30 + 6 * k
        body = body.cut(prism_xz([(x, -5.5), (x + 1.5, -3.9), (x + 3, -5.5)], -13, CLV_IN))  # finger only
    return body.cut(horn_holes_y(CLV_IN - 1, CLV_OUT + 1))


# ---------------------------------------------------------------- printing

# How each printed part sits on the bed: the face that goes down (in the part's own
# frame) and what, if anything, needs support. Chosen by measuring the overhang in all
# six orientations. Slicer: support on, but not for slopes steeper than 45 degrees and
# never inside horizontal screw holes (drill those to size); bridges print unsupported.
PRINT_SETUP = {
    "TAL-101": ("-Z", "plinth down; no support"),
    "TAL-102": ("-Z", "turntable down; no support"),
    "TAL-103": ("-Z", "flat underside down; no support (servo-opening roof and window tops bridge)"),
    "TAL-104": ("-Z", "flat underside down; no support (servo-opening roof and window tops bridge)"),
    "TAL-105": ("+Z", "upside down, flat top face down; support only under the idler plate's neck (5 mm up); "
                      "the cradle floor and plug bay roof bridge"),
    "TAL-106": ("-X", "standing on the roll-horn disc; no support (one 32 mm bridge at the front of the "
                      "servo opening); use a brim, the part is 108 tall"),
    "TAL-107": ("+Y", "horn plate down; support under the finger"),
    "TAL-108": ("+Y", "outer face down; no support"),
    "TAL-110": ("-Y", "outer face down; no support"),
    "TAL-111": ("-Z", "flat; no support. Print it first, with the same settings as the arm"),
}


def printed_parts():
    """Every printed part in its own frame: {part no: (slug, quantity, solid)}."""
    base, _ = base_parts()
    shoulder, _ = shoulder_parts()
    wrist, _ = wrist_parts()
    grip, _ = gripper_parts()
    upper, _, _, idler = arm_link(L1)
    fore, _, _, _ = arm_link(L2, **FOREARM)
    return {"TAL-101": ("base", 1, base["base"]), "TAL-102": ("shoulder", 1, shoulder["shoulder"]),
            "TAL-103": ("upper-arm", 1, upper), "TAL-104": ("forearm", 1, fore),
            "TAL-105": ("wrist", 1, wrist["wrist"]), "TAL-106": ("gripper-body", 1, grip["gripper"]),
            "TAL-107": ("moving-jaw", 1, moving_jaw()), "TAL-108": ("hub-cap", 3, hub_cap(True)),
            "TAL-110": ("idler-plate", 2, idler), "TAL-111": ("servo-fit-gauge", 1, fit_gauge()[0])}


BED_ROT = {"-Z": None, "+Z": ((1, 0, 0), 180), "+Y": ((1, 0, 0), -90), "-Y": ((1, 0, 0), 90),
           "-X": ((0, 1, 0), -90), "+X": ((0, 1, 0), 90)}  # rotation that puts each face down


def on_bed(solid, down):
    """Turn a part so its `down` face ("-Z", "+Y", ...) sits on z = 0, centred on the origin."""
    s = solid.val() if isinstance(solid, cq.Workplane) else solid
    if BED_ROT[down]:
        axis, deg = BED_ROT[down]
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), deg)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


# ---------------------------------------------------------------- assembly

LINKS = ("talos_arm", "shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_link", "gripper_link")


def joint_transforms(pose):
    """Relative joint transforms (parent frame -> child frame), J1..J6."""
    q1, q2, q3, q4, q5, q6 = pose
    return (T(0, 0, H_PAN) * rot((0, 0, 1), q1),
            T(0, 0, H_SH) * rot((0, 1, 0), -90 + q2),
            T(L1, 0, 0) * rot((0, 1, 0), q3),
            T(L2, 0, 0) * rot((0, 1, 0), q4),
            T(X_ROLL, 0, 0) * rot((1, 0, 0), q5),
            T(XJ, 0, ZJ) * rot((0, 1, 0), -q6))


def link_frames(pose):
    """World frame of each link sub-assembly; joint k's axis passes through frame k."""
    frames, acc = {LINKS[0]: cq.Location()}, cq.Location()
    for name, J in zip(LINKS[1:], joint_transforms(pose)):
        acc = acc * J
        frames[name] = acc
    return frames


def build(pose):
    servo_body, servo_horn, servo_plugs = sts3215()
    J1, J2, J3, J4, J5, J6 = joint_transforms(pose)
    J0 = joint_transforms((0, 0, 0, 0, 0, 0))

    def add_servo(assy, name, loc, k, child, child_loc=cq.Location()):
        """Body and plugs stay with the link; both horns are bolted to the next link (joint k)."""
        assy.add(servo_body, name=f"{name}_body", loc=loc, color=C_SERVO)
        assy.add(servo_plugs, name=f"{name}_plugs", loc=loc, color=C_PLUG)
        child.add(servo_horn, name=f"{name}_horns", loc=child_loc * J0[k].inverse * loc, color=C_HORN)

    base, s1 = base_parts()
    shoulder, s2 = shoulder_parts()
    upper, s3, upper_screws, upper_idler = arm_link(L1)
    fore, s4, fore_screws, fore_idler = arm_link(L2, **FOREARM)
    wrist, s5 = wrist_parts()
    grip, s6 = gripper_parts()
    caps = hub_caps()

    a_base = cq.Assembly(name="talos_arm")
    a_sh = cq.Assembly(name="shoulder_link", loc=J1)
    a_ua = cq.Assembly(name="upper_arm_link", loc=J2)
    a_fa = cq.Assembly(name="forearm_link", loc=J3)
    a_wr = cq.Assembly(name="wrist_link", loc=J4)
    a_gr = cq.Assembly(name="gripper_link", loc=J5)

    a_base.add(base["base"], name="base", color=C_SHELL)
    a_base.add(base["pcb"], name="bus_servo_board", color=C_PCB)
    a_base.add(base["screws"], name="base_screws", color=C_STEEL)
    add_servo(a_base, "servo1_pan", s1, 0, a_sh)

    a_sh.add(shoulder["shoulder"], name="shoulder", color=C_SHELL)
    a_sh.add(shoulder["screws"], name="shoulder_screws", color=C_STEEL)
    add_servo(a_sh, "servo2_shoulder", s2, 1, a_ua)

    a_ua.add(upper, name="upper_arm", color=C_SHELL)
    a_ua.add(upper_idler, name="upper_arm_idler_plate", color=C_SHELL)
    a_ua.add(caps, name="shoulder_hub_caps", color=C_GRAPHITE)
    a_ua.add(upper_screws, name="upper_arm_screws", color=C_STEEL)
    add_servo(a_ua, "servo3_elbow", s3, 2, a_fa)

    a_fa.add(fore, name="forearm", color=C_SHELL)
    a_fa.add(fore_idler, name="forearm_idler_plate", color=C_SHELL)
    a_fa.add(caps, name="elbow_hub_caps", color=C_GRAPHITE)
    a_fa.add(fore_screws, name="forearm_screws", color=C_STEEL)
    add_servo(a_fa, "servo4_wrist_flex", s4, 3, a_wr)

    a_wr.add(wrist["wrist"], name="wrist", color=C_SHELL)
    a_wr.add(caps, name="wrist_hub_caps", color=C_GRAPHITE)
    a_wr.add(wrist["screws"], name="wrist_screws", color=C_STEEL)
    add_servo(a_wr, "servo5_wrist_roll", s5, 4, a_gr)

    a_gr.add(grip["gripper"], name="gripper_body", color=C_SHELL)
    a_gr.add(grip["screws"], name="gripper_screws", color=C_STEEL)
    add_servo(a_gr, "servo6_jaw", s6, 5, a_gr, child_loc=J6)  # jaw horn turns with the jaw
    a_gr.add(moving_jaw(), name="moving_jaw", loc=J6, color=C_GRAPHITE)
    a_gr.add(compound(horn_screws_y(CLV_OUT)), name="moving_jaw_screws", loc=J6, color=C_STEEL)

    a_wr.add(a_gr)
    a_fa.add(a_wr)
    a_ua.add(a_fa)
    a_sh.add(a_ua)
    a_base.add(a_sh)
    return a_base


def flatten(assy, parent=cq.Location()):
    """Yield (name, shape, color) with absolute placement."""
    here = parent * assy.loc
    if assy.obj is not None:
        shape = assy.obj.val() if isinstance(assy.obj, cq.Workplane) else assy.obj
        yield assy.name, shape.moved(here), assy.color
    for child in assy.children:
        yield from flatten(child, here)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="ready", choices=sorted(POSES))
    ap.add_argument("--out", default=None)
    ap.add_argument("--z-up", action="store_true", help="keep the model's Z-up frame in the STEP")
    args = ap.parse_args()
    out = Path(args.out or Path(__file__).with_name(
        "talos_arm_assembly.step" if args.pose == "ready" else f"talos_arm_{args.pose}.step"))
    assy = build(POSES[args.pose])
    if not args.z_up:  # SolidWorks and most CAD use Y up: stand the arm upright in their iso view
        assy.loc = rot((1, 0, 0), -90)
    assy.export(str(out))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
