"""Dimensional check of every printed part, run on the mesh that goes into its print file.

Each probe is a feature the design calls for (a hole, a pocket wall, a plate face, the
pivot spacing), written in the part's own frame from the design constants and the
STS3215 sizes below. It is measured on the welded print mesh, already turned onto its
print face, by casting rays, so it checks the file the slicer gets, not only the model.

    hole   radial rays from a hole's axis: every ray must land on the design radius
           (so the hole is the right size and in the right place)
    ray    distance from a point to the first surface along a direction
    span   distance between the first surfaces either side of a point (a gap or a wall)
    open   nothing in the way for 80 mm (a servo's way in)
    clear  a tool's way out from a screw head or a horn: the axis and a ring of lines round
           it, all clear for 80 mm (a screwdriver on its screw, a horn through its opening)

Servo pocket walls are checked against the servo itself (the Feetech STS3215 as measured
from SO-101's CAD): each wall must sit FIT_C (0.4 mm) off the servo's face, the printing
tolerance, and the channels for the rib, the bump and the idler boss PASS_C (0.5 mm) off them.
servo_reference.py checks the same pockets against the Feetech model itself.

The fit checks (servo in its pocket, servo out along its way in, idler plate in its
pocket) run on the solids.

    python part_checks.py        # every part, every probe
"""

import math

import cadquery as cq
import numpy as np

import talos_arm as ta

# STS3215 sizes (talos_arm.py, measured from the Feetech CAD), servo frame: shaft on +Z,
# body centre at x = -12.5.
SV = dict(cx=-12.5, half_len=(ta.CASE_X[1] - ta.CASE_X[0]) / 2, half_w=ta.CASE_HW,
          top=ta.TOP_FACE, bottom=ta.BOTTOM_FACE, rib=ta.RIB[3], bump=ta.BUMP[3],
          horn_face=ta.HORN_FACE, passive_face=ta.PASSIVE_FACE, horn_r=ta.HORN_R, pcd_r=ta.HORN_PCD_R)
FIT = (ta.FIT_C, ta.FIT_C)     # servo face to pocket wall
PASS = (ta.PASS_C, ta.PASS_C)  # rib, bump and boss to their channels
TOL = 0.05         # flat features
TOL_R = 0.03       # hole radii (mesh chord tolerance is 0.01)
M3_CLEAR, M3_CBORE, M2_CLEAR, M2_CBORE = ta.SCREW[3]["clear"], 3.25, 1.2, 2.1
HORN_OPENING = ta.HORN_OPENING  # shell opening round each horn: horn R9.6 + 1.2


# ---------------------------------------------------------------- probes

def hole(name, c, axis, r, angles=range(0, 360, 45), tol=TOL_R):
    return ("hole", name, np.array(c, float), np.array(axis, float), r, tuple(angles), tol)


def ray(name, p, d, lo, hi=None):
    return ("ray", name, np.array(p, float), np.array(d, float), lo, lo if hi is None else hi)


def span(name, p, d, lo, hi=None):
    return ("span", name, np.array(p, float), np.array(d, float), lo, lo if hi is None else hi)


def open_(name, p, d):
    return ("open", name, np.array(p, float), np.array(d, float), 80.0, math.inf)


def clear(name, p, d, r):
    """Straight way out from p along d for something of radius r (a driver, a screw head, a horn)."""
    return ("clear", name, np.array(p, float), np.array(d, float), r)


DRIVER_R = 1.9   # M2 case screws: the head (R1.75) and a 3.5 mm driver shaft
KEY_R = 2.9      # M3 button heads: the head itself (R2.85) on its way to the seat


def servo_frame(loc):
    T = loc.wrapped.Transformation()
    R = np.array([[T.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    t = np.array([T.Value(i, 4) for i in (1, 2, 3)])
    return lambda p: R @ np.array(p, float) + t, lambda d: R @ np.array(d, float)


def wall(name, P, D, p, d, half, fit=FIT):
    """Pocket wall facing the servo: from point p (servo frame) along d, the servo face is `half` away."""
    return ray(name, P(p), D(d), half + fit[0] - TOL, half + fit[1] + TOL)


def rib_and_bump(P, D, x_rib=-20, y=-5, x_bump=-24):
    """Channels over the top-face rib and under the bump, PASS_C off them."""
    return [wall("rib channel", P, D, (x_rib, y, 0), (0, 0, 1), SV["rib"], PASS),
            wall("bump channel", P, D, (x_bump, y, 0), (0, 0, -1), SV["bump"], PASS)]


def horn_pattern(at, axis, name, r=M3_CLEAR, keep_off=()):
    """Four holes on the horn's 14 mm PCD at 45 degrees, at point `at` on the joint axis.

    keep_off: directions (degrees from the hole's outward radial) where the hole is meant to
    run into a neighbour, e.g. 180 for a counterbore that opens into the centre hole."""
    out = []
    for k, (u, v) in enumerate(ta.HORN_HOLES):
        c = np.array(at, float)
        if axis == "y":
            c += (u, 0, v)
        elif axis == "x":
            c += (0, u, v)
        else:
            c += (u, v, 0)
        phi = math.degrees(math.atan2(v, u))
        angles = [a for a in range(0, 360, 45)
                  if all(abs((a - phi - off + 180) % 360 - 180) > 75 for off in keep_off)]
        out.append(hole(f"{name} {k + 1}", c, {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis], r, angles))
    return out


def case_holes(P, D, holes, z_land, z_cbore, seat=None, cbore_angles=None):
    """M2 case screws: clearance hole through the land, counterbore beyond, servo frame.
    With `seat` (signed z of the head seat), also the driver's way out from the head."""
    out = []
    for x, y in holes:
        out.append(hole(f"M2 case screw clearance at ({x:g}, {y:g})", P((x, y, z_land)), D((0, 0, 1)), M2_CLEAR))
        out.append(hole(f"M2 case screw c'bore at ({x:g}, {y:g})", P((x, y, z_cbore)), D((0, 0, 1)), M2_CBORE,
                        **({"angles": cbore_angles(x, y)} if cbore_angles else {})))
        if seat is not None:
            sg = 1 if seat > 0 else -1
            out.append(clear(f"driver reaches the M2 case screw at ({x:g}, {y:g})", P((x, y, seat + 0.05 * sg)),
                             D((0, 0, sg)), DRIVER_R))
    return out


def horn_ways(P, D):
    """Both horns go on after the servo: each must pass out through its opening in the shell."""
    return [clear("output horn passes through its opening", P((0, 0, SV["horn_face"] - 2.2)), D((0, 0, 1)), SV["horn_r"] + 0.3),
            clear("passive horn passes through its opening", P((0, 0, -SV["passive_face"] + 1.9)), D((0, 0, -1)),
                  SV["horn_r"] + 0.3)]


def clevis_side(y_mid, out_side=True):
    """Horn centre hole, the four horn screws and the plate thickness at y_mid (link frame)."""
    s = "output" if out_side else "idler"
    pr = [hole(f"{s} plate centre hole", (0, y_mid, 0), (0, 1, 0), 3.5)]
    pr += horn_pattern((0, y_mid, 0), "y", f"{s} plate horn screw")
    pr.append(span(f"{s} plate thickness", (12, y_mid, 0), (0, 1, 0), ta.CLV_OUT - ta.CLV_IN))
    return pr


def horn_pads():
    """Both plate pads bear on the horn faces (output 18.7, passive 18.55 from the servo mid-plane)."""
    return [ray("output pad on the horn face", (5, 0, 0), (0, 1, 0), SV["horn_face"]),
            ray("idler pad on the passive horn face", (5, 0, 0), (0, -1, 0), SV["passive_face"])]


# ---------------------------------------------------------------- parts

def base_probes():
    _, loc = ta.base_parts()
    P, D = servo_frame(loc)
    top = ta.H_PAN - 2.8
    pr = [ray("column top (H_PAN - 2.8)", (30, 0, 100), (0, 0, -1), 100 - top),
          ray("plinth top at 10", (0, -60, 30), (0, 0, -1), 20),
          ray("flat underside at 0", (0, -60, -20), (0, 0, 1), 20),
          ray("board standoff top", (-57.5 + 2.2, 18.5, 40), (0, 0, -1), 40 - ta.BOARD["z"]),
          hole("board standoff M2.5 pilot", (-57.5, 18.5, 12), (0, 0, 1), 1.05)]
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = 54 * math.cos(a), 54 * math.sin(a)
        pr += [hole(f"table bolt hole {k + 1}", (x, y, 3), (0, 0, 1), 2.2),
               hole(f"table bolt c'bore {k + 1}", (x, y, 8.5), (0, 0, 1), 4.2)]
    c = (SV["cx"], 0, 0)
    pr += [wall("pocket end, shaft side", P, D, c, (1, 0, 0), SV["half_len"]),
           wall("pocket end, far side", P, D, c, (-1, 0, 0), SV["half_len"]),
           wall("pocket side +", P, D, c, (0, 1, 0), SV["half_w"]),
           wall("pocket side -", P, D, c, (0, -1, 0), SV["half_w"]),
           wall("pocket floor", P, D, (-25, 11, 0), (0, 0, -1), SV["bottom"]),
           wall("bump recess in the floor", P, D, (-24, 8, 0), (0, 0, -1), SV["bump"], PASS),
           open_("servo way in (from the top)", P(c), D((0, 0, 1)))]
    z_land = -(SV["bottom"] + ta.FIT_C + ta.BASE_SEAT) / 2
    pr += case_holes(P, D, ta.CASE_BACK, z_land, -ta.BASE_SEAT - 1.0, seat=-ta.BASE_SEAT)
    # the pair beside the shaft opens into the plug bay on its -X side (no 0.2 mm wall left between them)
    pr += case_holes(P, D, ta.CASE_BACK_SHAFT, z_land, -ta.BASE_SEAT - 1.0, seat=-ta.BASE_SEAT,
                     cbore_angles=lambda x, y: (0, 45, 90, 135, 315) if y > 0 else (0, 45, 225, 270, 315))
    return pr


def shoulder_probes():
    _, loc = ta.shoulder_parts()
    P, D = servo_frame(loc)
    pr = [hole("pan horn centre hole", (0, 0, 2), (0, 0, 1), 3.5),
          ray("turntable R37", (60, 0, 3), (-1, 0, 0), 60 - 37)]
    pr += horn_pattern((0, 0, 2), "z", "pan horn screw")
    pr += horn_pattern((0, 0, 8), "z", "pan horn screw c'bore", M3_CBORE, keep_off=(180,))
    pr.append(span("bump channel wall with doubler (0.8)", (0, -ta.SHELL_OUT - ta.DOUBLER + 0.4, 25), (0, 1, 0),
                   ta.SHELL_OUT + ta.DOUBLER - SV["bump"] - ta.PASS_C))
    pr += [wall("pocket side +", P, D, (-21, 0, 0), (0, 1, 0), SV["half_w"]),
           wall("pocket side -", P, D, (-21, 0, 0), (0, -1, 0), SV["half_w"]),
           wall("pocket face, horn side", P, D, (-21, -10, 0), (0, 0, 1), SV["top"]),
           wall("pocket face, idler side", P, D, (-32, 6, 0), (0, 0, -1), SV["bottom"]),
           *rib_and_bump(P, D),
           wall("pocket floor", P, D, (SV["cx"], 8, 0), (-1, 0, 0), SV["half_len"]),
           open_("servo way in (from the top)", P((SV["cx"], 0, 0)), D((1, 0, 0))),
           # the case screws locate the servo: shoulder axis at H_SH
           span("cheek fused to the cradle wall (no slit)", (8, 21, 13), (0, 1, 0), 25.0 - (SV["top"] + ta.FIT_C))]
    pr += case_holes(P, D, ta.CASE_FRONT, 17.0, 20.0, seat=18.5)
    pr += case_holes(P, D, ta.CASE_BACK, -17.5, -20.0, seat=-18.5)
    pr += horn_ways(P, D)
    for k, (x, y) in enumerate(ta.HORN_HOLES):  # fitted from above, before the shoulder servo goes in
        pr.append(clear(f"pan horn screw {k + 1} goes in from above", (x, y, 4.05), (0, 0, 1), KEY_R))
    return pr


def link_probes(L, h1, w_mid, h_mid):
    P, D = servo_frame(ta.loc_r(ta.R_LINK, (L, 0, 0)))
    xm = (20.0 + L - 36.0) / 2
    pr = clevis_side(ta.CLV_IN + 2)
    pr += [ray("output pad on the horn face", (5, 0, 0), (0, 1, 0), SV["horn_face"]),
           # pivot spacing: the servo's horn opening is centred L from the clevis axis
           hole(f"horn opening at x = L ({L:g})", (L, 18.0, 0), (0, 1, 0), HORN_OPENING),
           hole(f"passive horn opening at x = L ({L:g})", (L, -17.3, 0), (0, 1, 0), HORN_OPENING,
                angles=(0, 45, 90, 135))]
    c = (SV["cx"], 0, 0)
    pr += [wall("pocket end, shaft side", P, D, c, (1, 0, 0), SV["half_len"]),
           wall("pocket end, far side", P, D, c, (-1, 0, 0), SV["half_len"]),
           wall("pocket face, horn side", P, D, (-20, -10, 0), (0, 0, 1), SV["top"]),
           wall("pocket face, idler side", P, D, (-34, 0, 0), (0, 0, -1), SV["bottom"]),
           *rib_and_bump(P, D),
           wall("pocket roof", P, D, c, (0, -1, 0), SV["half_w"]),
           open_("servo way in (from below)", P(c), D((0, 1, 0))),
           ray("shell width +Y (36.8)", (L - 34, 40, 5), (0, -1, 0), 40 - ta.SHELL_OUT),
           ray("shell width -Y (36.8)", (L - 34, -40, 5), (0, 1, 0), 40 - ta.SHELL_OUT),
           ray("doubler face over the bump channel", (L - 25, -40, 0), (0, 1, 0), 40 - ta.SHELL_OUT - ta.DOUBLER),
           span("bump channel wall with doubler (0.8)", (L - 25, -ta.SHELL_OUT - ta.DOUBLER + 0.4, 0), (0, 1, 0),
                ta.SHELL_OUT + ta.DOUBLER - SV["bump"] - ta.PASS_C),
           ray(f"shell top at {h1 / 2:g}", (L - 12.5, 8, 40), (0, 0, -1), 40 - h1 / 2),
           ray("flat underside at Z_BED", (xm, 8, -40), (0, 0, 1), 40 + ta.Z_BED),
           ray(f"waist top at {h_mid / 2:g}", (xm, 8, 40), (0, 0, -1), 40 - h_mid / 2),
           ray(f"waist width {w_mid:g}", (xm, 40, -10), (0, -1, 0), 40 - w_mid / 2),
           ray("idler plate pocket floor", (24, -21, 0), (0, 1, 0), -ta.CLV_IN + 21),
           ray("idler plate pocket end (plate 30 + 0.3)", (24, -19.5, 6), (1, 0, 0), 30.3 - 24)]
    for x, z in ta.IDLER_TAB_SCREWS:
        pr.append(hole(f"idler plate M3 pilot at ({x:g}, {z:g})", (x, -15, z), (0, 1, 0), 1.25))
    seat, bseat = ta.FRONT_SEAT, ta.SHELL_IN + 1.0
    pr += case_holes(P, D, ta.CASE_FRONT, seat - 0.5, seat + 0.15, seat=seat)
    pr += case_holes(P, D, ta.CASE_BACK, -(SV["bottom"] + bseat) / 2, -bseat - 0.25, seat=-bseat)
    pr += horn_ways(P, D)
    return pr


def idler_probes():
    y = -ta.CLV_IN - 2
    pr = clevis_side(y, out_side=False)
    pr += [ray("pad on the passive horn face", (5, -10, 0), (0, -1, 0), SV["passive_face"] - 10),
           ray(f"pad R{ta.PAD_R:g}", (0, -18.75, -20), (0, 0, 1), 20 - ta.PAD_R),
           ray("neck end at reach 30", (40, y, 0), (-1, 0, 0), 10),
           ray("hub R10.5", (0, y, 30), (0, 0, -1), 30 - ta.IDLER_HUB_R)]
    for k, (u, v) in enumerate(ta.HORN_HOLES):  # the heads sit on the plain outer face (the bed face)
        e = np.array((u, v)) / math.hypot(u, v) * 2.3
        pr.append(ray(f"outer face under screw head {k + 1} (no counterbore)", (u + e[0], -30, v + e[1]), (0, 1, 0),
                      30 - ta.CLV_OUT))
    for x, z in ta.IDLER_TAB_SCREWS:
        pr.append(hole(f"M3 tab screw clearance at ({x:g}, {z:g})", (x, y, z), (0, 1, 0), M3_CLEAR + 0.1))
    return pr


def wrist_probes():
    _, loc = ta.wrist_parts()
    P, D = servo_frame(loc)
    xs = ta.X_ROLL - 18.7
    pr = clevis_side(ta.CLV_IN + 2) + clevis_side(-ta.CLV_IN - 2, out_side=False) + horn_pads()
    pr += [ray(f"flat top at WRIST_TOP {ta.WRIST_TOP:g}", (40, 0, 40), (0, 0, -1), 40 - ta.WRIST_TOP),
           ray("cradle end (roll horn face X_ROLL - 2.8)", (80, 14, 5), (-1, 0, 0), 80 - (ta.X_ROLL - 2.8)),
           wall("pocket side +", P, D, (-30, 0, 0), (0, -1, 0), SV["half_w"]),
           wall("pocket side -", P, D, (-30, 0, 0), (0, 1, 0), SV["half_w"]),
           wall("pocket floor", P, D, (SV["cx"], 0, 0), (-1, 0, 0), SV["half_len"]),
           wall("pocket back wall", P, D, (-25, 11, 0), (0, 0, -1), SV["bottom"]),
           wall("bump recess in the back wall", P, D, (-24, 0, 0), (0, 0, -1), SV["bump"], PASS),
           open_("servo way in (along the roll axis)", P((SV["cx"], 0, 0)), D((0, 0, 1)))]
    pr += case_holes(P, D, ta.CASE_BACK, -17.25, -18.6, seat=-18.0)
    assert abs(P((0, 0, 0))[0] - xs) < 1e-9
    return pr


def gripper_probes():
    _, loc = ta.gripper_parts()
    P, D = servo_frame(loc)
    pr = [hole("roll horn centre hole", (1.75, 0, 0), (1, 0, 0), 3.5),
          ray("roll horn disc R15.5", (2, 0, -40), (0, 0, 1), 40 - 15.5),
          hole(f"jaw servo horn opening at ({ta.XJ:g}, {ta.ZJ:g})", (ta.XJ, 18.0, ta.ZJ), (0, 1, 0), HORN_OPENING,
               angles=(0, 45, 90, 135, 225, 270, 315)),
          ray("finger tip at 108", (130, 0, -3), (-1, 0, 0), 22),
          ray("finger width 20, +Y", (80, 30, -8), (0, -1, 0), 30 - ta.FINGER_HW),
          ray("finger width 20, -Y", (80, -30, -8), (0, 1, 0), 30 - ta.FINGER_HW)]
    pr += horn_pattern((1.75, 0, 0), "x", "roll horn screw")
    pr += horn_pattern((5.0, 0, 0), "x", "roll horn screw c'bore", M3_CBORE, keep_off=(180,))
    for k, (y, z) in enumerate(ta.HORN_HOLES):
        if z < 0:  # the lower two counterbores run on as grooves in the pocket floor
            pr.append(hole(f"roll horn screw {k + 1} groove in the pocket floor", (ta.ROLL_CBORE_RUN - 2, y, z),
                           (1, 0, 0), M3_CBORE, angles=(225, 270, 315)))
        # the head (R2.85) must travel a screw's length along the axis without catching on the pocket floor
        for a in (0, 90, 180, 270):
            dy, dz = KEY_R * math.cos(math.radians(a)), KEY_R * math.sin(math.radians(a))
            pr.append(ray(f"roll horn screw {k + 1}: head clears on its way in ({a} deg)", (3.55, y + dy, z + dz),
                          (1, 0, 0), ta.ROLL_CBORE_RUN - 4.0, 80.0))
        # 2 mm hex key: short leg along the screw axis, long leg up through the open top
        pr.append(ray(f"roll horn screw {k + 1}: room along its axis for the hex key's short leg",
                      (3.55, y, z), (1, 0, 0), 30.0, 80.0))
        pr.append(clear(f"roll horn screw {k + 1}: hex key's long leg up through the open top",
                        (3.5 + 18, y, z), (0, 0, 1), 1.2))
    pr.append(span("bump channel wall with doubler (0.8)", (ta.XJ - 25, -ta.SHELL_OUT - ta.DOUBLER + 0.4, ta.ZJ + 14),
                   (0, 1, 0), ta.SHELL_OUT + ta.DOUBLER - SV["bump"] - ta.PASS_C))
    c = (SV["cx"], 0, 0)
    pr += [wall("pocket end, shaft side", P, D, c, (1, 0, 0), SV["half_len"]),
           wall("pocket end, far side", P, D, c, (-1, 0, 0), SV["half_len"]),
           wall("pocket face, horn side", P, D, (-22, 11, 0), (0, 0, 1), SV["top"]),
           wall("pocket face, idler side", P, D, (-22, 11, 0), (0, 0, -1), SV["bottom"]),
           *rib_and_bump(P, D, x_rib=-34.5, y=5, x_bump=-29.9),
           wall("pocket floor", P, D, c, (0, 1, 0), SV["half_w"]),
           open_("servo way in (from the top)", P(c), D((0, -1, 0)))]
    seat, bseat = ta.FRONT_SEAT, ta.SHELL_IN + 1.0
    pr += case_holes(P, D, ta.CASE_FRONT, seat - 0.5, seat + 0.15, seat=seat)
    pr += case_holes(P, D, ta.CASE_BACK, -(SV["bottom"] + bseat) / 2, -bseat - 0.25, seat=-bseat)
    pr += horn_ways(P, D)
    return pr


def jaw_probes():
    y = ta.CLV_IN + 2
    pr = [hole("horn centre hole", (0, y, 0), (0, 1, 0), 3.5),
          span("horn plate thickness", (8, y, 6), (0, 1, 0), ta.CLV_OUT - ta.CLV_IN),
          ray("pad on the horn face", (5, 0, 0), (0, 1, 0), SV["horn_face"]),
          ray("finger tip at 64", (90, 0, -3), (-1, 0, 0), 26),
          ray("finger width 20", (45, 30, 0), (0, -1, 0), 30 - ta.FINGER_HW)]
    return pr + horn_pattern((0, y, 0), "y", "horn screw")


def cap_probes():
    return [ray("cap R12.5", (30, 2, 0), (-1, 0, 0), 30 - 12.5),
            span("cap thickness 3.4", (11.5, 1.7, 0), (0, 1, 0), 3.4),
            hole("recess R10.4 over the screw heads", (0, 0.9, 0), (0, 1, 0), 10.4),
            ray("recess depth 1.8", (0, -5, 0), (0, 1, 0), 6.8)]


def gauge_probes():
    _, origins = ta.fit_gauge()
    pr = []
    for (c, p), (ox, oy) in zip(ta.GAUGE_CLEAR, origins):
        length = ta.CASE_X[1] - ta.CASE_X[0]
        pr += [span(f"window length {length:g} + 2 x {c:g}", (ox + SV["cx"], oy + 5, 4), (1, 0, 0), length + 2 * c),
               span(f"window thickness 31.9 + 2 x {c:g}", (ox - 8, oy, 4), (0, 1, 0), SV["top"] + SV["bottom"] + 2 * c),
               span(f"rib notch height {SV['rib']:g} + {p:g}", (ox - 12, oy + 1, 4), (0, 1, 0),
                    SV["rib"] + p + SV["bottom"] + c),
               span(f"idler boss notch 6 + 2 x {p:g}", (ox, oy - 17.5, 4), (1, 0, 0), 6 + 2 * p),
               ray("gauge height 8", (1.5, oy, 20), (0, 0, -1), 12)]
    return pr


def probes():
    """{part no: [probe, ...]} in each part's own frame."""
    f = ta.FOREARM
    return {"TAL-101": base_probes(), "TAL-102": shoulder_probes(),
            "TAL-103": link_probes(ta.L1, 30.8, 34.0, 25.0),
            "TAL-104": link_probes(ta.L2, f["h1"], f["w_mid"], f["h_mid"]),
            "TAL-105": wrist_probes(), "TAL-106": gripper_probes(), "TAL-107": jaw_probes(),
            "TAL-108": cap_probes(), "TAL-110": idler_probes(), "TAL-111": gauge_probes()}


# ---------------------------------------------------------------- measuring

class Mesh:
    """Triangles for ray casting (Moller-Trumbore, all triangles at once)."""

    def __init__(self, V, T):
        self.a = V[T[:, 0]]
        self.e1 = V[T[:, 1]] - self.a
        self.e2 = V[T[:, 2]] - self.a

    def hit(self, p, d):
        d = d / np.linalg.norm(d)
        h = np.cross(d, self.e2)
        det = np.einsum("ij,ij->i", self.e1, h)
        ok = np.abs(det) > 1e-12
        f = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        s = p - self.a
        u = f * np.einsum("ij,ij->i", s, h)
        q = np.cross(s, self.e1)
        v = f * (q @ d)
        t = f * np.einsum("ij,ij->i", self.e2, q)
        good = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6)
        return float(t[good].min()) if good.any() else math.inf


def bed_transform(solid, down):
    """(R, t) taking the part's own frame to its print-bed frame, as ta.on_bed places it."""
    R = np.eye(3)
    if ta.BED_ROT[down]:
        (ax, ay, az), deg = ta.BED_ROT[down]
        k = np.array((ax, ay, az), float)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        th = math.radians(deg)
        R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    s = solid.val() if isinstance(solid, cq.Workplane) else solid
    bed = ta.on_bed(s, down)
    c0, c1 = s.Center(), bed.Center()
    return R, np.array((c1.x, c1.y, c1.z)) - R @ np.array((c0.x, c0.y, c0.z))


def measure(mesh, R, t, pr):
    """(passed, measured text, expected text)."""
    kind, name = pr[0], pr[1]
    if kind == "hole":
        _, _, c, axis, r, angles, tol = pr
        axis = axis / np.linalg.norm(axis)
        # angle 0 along u, 90 along v: (x, z) about Y, (y, z) about X, (x, y) about Z
        i = int(np.argmax(np.abs(axis)))
        if abs(axis[i]) > 0.999:
            u, v = np.eye(3)[{0: [1, 2], 1: [0, 2], 2: [0, 1]}[i]]
        else:
            u = np.cross(axis, (0, 0, 1))
            u /= np.linalg.norm(u)
            v = np.cross(axis, u)
        got = [mesh.hit(R @ c + t, R @ (math.cos(math.radians(a)) * u + math.sin(math.radians(a)) * v))
               for a in angles]
        ok = all(abs(g - r) <= tol for g in got)
        return ok, f"R {min(got):.3f}..{max(got):.3f}", f"R {r:g} ±{tol:g}"
    if kind == "clear":
        _, _, p, d, r = pr
        d = d / np.linalg.norm(d)
        a = np.cross(d, (1, 0, 0)) if abs(d[0]) < 0.9 else np.cross(d, (0, 1, 0))
        a /= np.linalg.norm(a)
        b = np.cross(d, a)
        starts = [p] + [p + r * (math.cos(math.radians(k)) * a + math.sin(math.radians(k)) * b)
                        for k in range(0, 360, 30)]
        got = min(mesh.hit(R @ q + t, R @ d) for q in starts)
        return got >= 80.0, ("clear" if got == math.inf else f"blocked at {got:.2f}"), f"R {r:g} clear for 80"
    _, _, p, d, lo, hi = pr
    P, Dd = R @ p + t, R @ d
    got = mesh.hit(P, Dd) + (mesh.hit(P, -Dd) if kind == "span" else 0.0)
    if kind == "open":
        return got >= lo, ("clear" if got == math.inf else f"blocked at {got:.2f}"), "clear for 80"
    tol = TOL if lo == hi else 0.0
    ok = lo - tol <= got <= hi + tol
    want = f"{lo:.2f} ±{TOL:g}" if lo == hi else f"{lo:.2f}..{hi:.2f}"
    return ok, f"{got:.3f}", want


def check_part(part_no, solid, V, T, verbose=False):
    """Run the part's probes on its bed-frame mesh. Returns (n passed, failures)."""
    down, _ = ta.PRINT_SETUP[part_no]
    R, t = bed_transform(solid, down)
    mesh = Mesh(V, T)
    fails, n = [], 0
    for pr in PROBES[part_no]:
        ok, got, want = measure(mesh, R, t, pr)
        n += ok
        line = f"    {'ok  ' if ok else 'FAIL'} {pr[1]}: {got} (design {want})"
        if not ok:
            fails.append(line)
        if verbose or not ok:
            print(line)
    return n, fails


# ---------------------------------------------------------------- fits (solids)

ENTRY = {name: way for name, (way, _) in ta.SERVO_ENTRY.items()}


def fit_checks():
    """Servo in its pocket without touching, out along its way in, idler plates in their pockets."""
    body, _, plugs = ta.sts3215()
    base, l1 = ta.base_parts()
    sh, l2 = ta.shoulder_parts()
    up, l3, _, idler = ta.arm_link(ta.L1)
    fo, l4, _, idler2 = ta.arm_link(ta.L2, **ta.FOREARM)
    wr, l5 = ta.wrist_parts()
    gr, l6 = ta.gripper_parts()
    rows = [("base", base["base"], l1), ("shoulder", sh["shoulder"], l2), ("upper arm", up, l3),
            ("forearm", fo, l4), ("wrist", wr["wrist"], l5), ("gripper", gr["gripper"], l6)]
    out = []
    for name, part, loc in rows:
        p = part.val()
        sv = body.val().moved(loc)
        overlap = p.intersect(sv).Volume()
        _, D = servo_frame(loc)
        d = D({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[ENTRY[name][1]]) * (1 if ENTRY[name][0] == "+" else -1)
        blocked = [s for s in (2, 5, 10, 20, 30, 45) if p.intersect(sv.translate(cq.Vector(*(s * d)))).Volume() > 0.01]
        out.append((f"{name}: servo in its pocket", overlap < 0.01, f"overlap {overlap:.2f} mm3"))
        out.append((f"{name}: servo slides out {ENTRY[name]} (servo frame)", not blocked,
                    "free for 45 mm" if not blocked else f"blocked at {blocked[0]} mm"))
        pl = plugs.val().moved(loc)  # the cable is plugged in before the servo goes in
        blocked = [s for s in (0, 1, 2, 3.5, 5, 7.5, 10, 15, 20, 30, 45)
                   if p.intersect(pl.translate(cq.Vector(*(s * d)))).Volume() > 0.01]
        out.append((f"{name}: both plugs fitted, in the bay and on the way in", not blocked,
                    "free for 45 mm" if not blocked else f"blocked at {blocked[0]} mm"))
    for name, part, plate in (("upper arm", up, idler), ("forearm", fo, idler2)):
        ov = part.val().intersect(plate.val()).Volume()
        out.append((f"{name}: idler plate TAL-110 in its pocket", ov < 0.01, f"overlap {ov:.2f} mm3"))
    return out


PROBES = probes()


def main():
    import sys
    import print_files as pf
    sys.stdout.reconfigure(encoding="utf-8")
    total = bad = 0
    for part_no, (slug, qty, solid) in ta.printed_parts().items():
        s = ta.on_bed(solid, ta.PRINT_SETUP[part_no][0])
        V, T = pf.welded_mesh(s)
        n, fails = check_part(part_no, solid, V, T, verbose=True)
        total += len(PROBES[part_no])
        bad += len(fails)
        print(f"{part_no} {slug}: {n}/{len(PROBES[part_no])} pass")
    for name, ok, txt in fit_checks():
        total += 1
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {txt}")
    print(f"{total - bad}/{total} checks pass")


if __name__ == "__main__":
    main()
