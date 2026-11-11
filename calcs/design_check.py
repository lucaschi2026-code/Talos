"""Talos design check: the CAD in ../cad/talos_arm.py against the servo and the plastic.

Mass properties come straight from the CAD solids, so editing a dimension in
talos_arm.py and re-running this script re-checks the design.

    python design_check.py          # writes design_check_results.md and plots
    python design_check.py --fast   # skip the slow CAD joint-range sweep
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cad"))
import talos_arm as ta  # noqa: E402

# ---------------------------------------------------------------- inputs

G = 9.81
TAU_STALL = 2.94          # N*m, STS3215 12 V (30 kg*cm), same as torque_check.m
LIMIT = TAU_STALL / 3     # continuous-load limit used in torque_check.m
COUNTS = 4096             # STS3215 magnetic encoder counts per revolution

M_SERVO = 55.0            # g, STS3215 with horn (datasheet)
M_HARDWARE = 6.0          # g per servo: screws, cable share
RHO_PLA = 1.24e-3         # g/mm^3
T_SHELL = 1.2             # mm, 3 perimeters x 0.4 mm (and top/bottom skins)
INFILL = 0.20             # sparse infill fraction

E_PLA = 2.3e3             # N/mm^2, printed PLA, conservative
SIGMA_ALLOW = 10.0        # N/mm^2, sustained bending in printed PLA (creep-limited)
BEARING_ALLOW = 20.0      # N/mm^2, screw bearing on a printed hole
HORN_SCREW_R = 7.0        # mm, horn screw circle radius (Feetech STS3215 horn, 14 mm circle)
HORN_SCREW_D = 3.0        # mm, M3 (the horn holes are 2.5 mm tapping size)
M_BOARD = 10.0            # g, bus servo adapter board (fixed, in the base)
MU_GRIP = 0.3             # friction, PLA jaw on a typical object

PAYLOAD = 0.200           # kg, R3 in the build plan
GRIP_POINT = (88.0, 0.0, 0.0)   # payload centre in the gripper frame (mid-jaw)
TIP_POINT = (106.0, 0.0, 0.0)   # jaw tip in the gripper frame
D_TOOL = ta.X_ROLL + GRIP_POINT[0]  # wrist-flex axis -> payload centre, mm

# joint name, link whose frame holds the axis, local axis
JOINTS = [("pan", "shoulder_link", 2), ("shoulder", "upper_arm_link", 1),
          ("elbow", "forearm_link", 1), ("wrist_flex", "wrist_link", 1),
          ("wrist_roll", "gripper_link", 0)]
HORIZONTAL = (0, 90, 0, 0, 0, 0)   # arm straight out: worst case for gravity


# ---------------------------------------------------------------- mass model

def gprops(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, p)
    c, M = p.CentreOfMass(), p.MatrixOfInertia()
    I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return p.Mass(), np.array([c.X(), c.Y(), c.Z()]), I


def printed_mass(volume, area):
    shell = min(volume, area * T_SHELL)
    return RHO_PLA * (shell + INFILL * (volume - shell))


def collect_parts():
    """Every part with mass (kg), CoM (mm) and inertia (kg*mm^2) in its link frame."""
    assy = ta.build(ta.POSES["zero"])
    parts = []

    def walk(node, link):
        for child in node.children:
            if child.obj is None:
                walk(child, child.name)
                continue
            shape = child.obj.val().moved(child.loc)
            V, c, I = gprops(shape)
            if child.name.endswith(("_horns", "_plugs", "_screws")):
                m = 0.0  # included in the servo mass and hardware allowance
            elif child.name.startswith("servo"):
                m = M_SERVO + M_HARDWARE
            elif child.name == "bus_servo_board":
                m = M_BOARD
            else:
                m = printed_mass(V, shape.Area())
            parts.append(dict(name=child.name, link=link, m=m / 1000.0, com=c,
                              I=I * (m / 1000.0) / V, shape=shape, V=V))

    walk(assy, assy.name)
    return parts


def frame_rt(loc):
    t = loc.wrapped.Transformation()
    R = np.array([[t.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    p = np.array([t.Value(i, 4) for i in (1, 2, 3)])
    return R, p


class Pose:
    """Parts placed in world coordinates for one set of joint angles."""

    def __init__(self, parts, q):
        self.q = q
        self.frames = {k: frame_rt(v) for k, v in ta.link_frames(q).items()}
        self.parts = []
        for p in parts:
            R, t = self.frames[p["link"]]
            self.parts.append(dict(p, r=R @ p["com"] + t, Iw=R @ p["I"] @ R.T))

    def point(self, link, local):
        R, t = self.frames[link]
        return R @ np.array(local) + t

    def axis(self, joint):
        _, link, k = next(j for j in JOINTS if j[0] == joint)
        R, t = self.frames[link]
        return R[:, k], t, ta.LINKS.index(link)

    def distal(self, idx):
        return [p for p in self.parts if ta.LINKS.index(p["link"]) >= idx]

    def gravity_torque(self, joint, payload=0.0, payload_at=GRIP_POINT):
        """Torque (N*m) the joint must hold against gravity."""
        a, o, idx = self.axis(joint)
        F = np.array([0, 0, -G])
        tau = sum(np.cross((p["r"] - o) / 1000, p["m"] * F) @ a for p in self.distal(idx))
        if payload:
            r = self.point("gripper_link", payload_at)
            tau += np.cross((r - o) / 1000, payload * F) @ a
        return abs(tau)

    def unit_payload_torque(self, joint, at=GRIP_POINT):
        a, o, _ = self.axis(joint)
        r = self.point("gripper_link", at)
        return abs(np.cross((r - o) / 1000, np.array([0, 0, -G])) @ a)

    def inertia(self, joint, payload=0.0):
        """Moment of inertia (kg*m^2) of everything distal about the joint axis."""
        a, o, idx = self.axis(joint)
        J = 0.0
        for p in self.distal(idx):
            d = p["r"] - o
            perp2 = d @ d - (d @ a) ** 2
            J += p["m"] * perp2 + a @ p["Iw"] @ a
        if payload:
            d = self.point("gripper_link", GRIP_POINT) - o
            J += payload * (d @ d - (d @ a) ** 2)
        return J * 1e-6


def link_mass(parts, link):
    return sum(p["m"] for p in parts if p["link"] == link)


# ---------------------------------------------------------------- kinematics

def ik(r, z, tool_elev):
    """Planar IK: payload centre at radius r, height z (mm), tool at tool_elev (deg).

    Returns joint angles in the talos_arm convention, or None if out of reach.
    """
    e4 = math.radians(tool_elev)
    wx = r - D_TOOL * math.cos(e4)
    wz = z - D_TOOL * math.sin(e4) - (ta.H_PAN + ta.H_SH)
    D2 = wx * wx + wz * wz
    c3 = (D2 - ta.L1 ** 2 - ta.L2 ** 2) / (2 * ta.L1 * ta.L2)
    if abs(c3) > 1:
        return None
    q3 = math.acos(c3)
    e2 = math.atan2(wz, wx) + math.atan2(ta.L2 * math.sin(q3), ta.L1 + ta.L2 * math.cos(q3))
    e3 = e2 - q3
    return (0, 90 - math.degrees(e2), math.degrees(q3), math.degrees(e3 - e4), 0, 0)


def within(q, ranges):
    for name, k in (("shoulder", 1), ("elbow", 2), ("wrist_flex", 3)):
        lo, hi = ranges.get(name, (-180, 180))
        if not lo <= q[k] <= hi:
            return False
    return True


# ---------------------------------------------------------------- CAD checks

def collides(static, moving, min_vol=1.0):
    for a in static:
        ba = a.BoundingBox()
        for b in moving:
            bb = b.BoundingBox()
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin
                    or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            if a.intersect(b).Volume() > min_vol:
                return True
    return False


def joint_ranges(parts, step=5, limit=175):
    """Collision-free range of each pitch joint and the jaw, from the CAD solids.

    The moving side is the whole distal chain held straight; the static side is
    the link that carries the joint's servo.
    """
    out = {}
    for k, (name, link) in enumerate((("shoulder", "upper_arm_link"),
                                      ("elbow", "forearm_link"),
                                      ("wrist_flex", "wrist_link")), start=1):
        idx = ta.LINKS.index(link)
        static = [p["shape"] for p in parts if p["link"] == ta.LINKS[idx - 1]]

        def moving_at(angle):
            q = [0, 0, 0, 0, 0, 0]
            q[k] = angle
            frames = ta.link_frames(q)
            base = frames[ta.LINKS[idx - 1]].inverse
            return [p["shape"].moved(base * frames[p["link"]])
                    for p in parts if ta.LINKS.index(p["link"]) >= idx
                    and not p["name"].startswith(("moving_jaw", "servo6_jaw_horns"))]

        out[name] = tuple(sweep(lambda a: collides(static, moving_at(a)), s, step, limit)
                          for s in (-1, 1))

    body = [p["shape"] for p in parts if p["name"] in ("gripper_body", "servo6_jaw_body")]
    jaw = ta.moving_jaw().val()

    def jaw_hits(a):
        return collides(body, [jaw.moved(ta.joint_transforms((0, 0, 0, 0, 0, a))[5])])

    out["gripper"] = (sweep(jaw_hits, -1, 1, 30), sweep(jaw_hits, 1, step, 150))
    return out


def sweep(hit, sign, step, limit):
    a = 0
    while a + step <= limit:
        if hit(sign * (a + step)):
            return sign * a
        a += step
    return sign * limit


def face_props(face, z0):
    """Area, first moment and second moment about z = z0 of a planar face (mm units)."""
    p = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face.wrapped, p)
    A, zc = p.Mass(), p.CentreOfMass().Z()
    Icm = p.MatrixOfInertia().Value(2, 2)  # about the centroid, axis Y
    # MatrixOfInertia(2,2) on a face normal to X is integral of (x^2 + z^2) dA, x = 0
    return A, A * zc, Icm + A * (zc - z0) ** 2


def section(shape, x, delta=0.2):
    """Bending properties of a link cross-section at station x (bending about Y)."""
    slab = shape.intersect(cq.Solid.makeBox(delta, 200, 200, cq.Vector(x - delta / 2, -100, -100)))
    faces = [f for f in slab.Faces() if abs(f.normalAt().x) > 0.99
             and abs(f.Center().x - (x + delta / 2)) < 1e-3]
    if not faces:
        return None
    A = sum(face_props(f, 0)[0] for f in faces)
    zc = sum(face_props(f, 0)[1] for f in faces) / A
    I_solid = sum(face_props(f, zc)[2] for f in faces)
    I_eff = 0.0
    for f in faces:  # the core inside the printed perimeters only carries infill
        I_f = face_props(f, zc)[2]
        try:
            outer = f.outerWire().offset2D(-T_SHELL)[0]
            inner = [w.offset2D(T_SHELL)[0] for w in f.innerWires()]
            I_core = face_props(cq.Face.makeFromWires(outer, inner), zc)[2]
            I_eff += (I_f - I_core) + INFILL * I_core
        except Exception:
            # No core could be offset inside this face. A wall up to two perimeters thick has
            # none (it prints solid); anything thicker gets a flat 0.6, on that face only.
            wall = 2 * f.Area() / sum(e.Length() for e in f.Edges())
            I_eff += I_f if wall <= 2 * T_SHELL + 0.6 else 0.6 * I_f
    zs = [v.Z for f in faces for v in f.Vertices()]
    c = max(max(zs) - zc, zc - min(zs))
    return dict(A=A, I=I_solid, I_eff=I_eff, c=c)


def post_section(shape, z, delta=0.2):
    """One shoulder cradle post (the +X one) cut at height z: (I about its own Y axis with the
    core at INFILL stiffness, half thickness), or None where the cut isn't a free-standing post."""
    slab = shape.intersect(cq.Solid.makeBox(100, 200, delta, cq.Vector(0, -100, z - delta / 2)))
    faces = [f for f in slab.Faces() if abs(f.normalAt().z) > 0.99 and abs(f.Center().z - (z + delta / 2)) < 1e-3]
    if not faces:
        return None
    xs = [v.X for f in faces for v in f.Vertices()]
    t = max(xs) - min(xs)
    if t > 6.0:  # still inside the cradle box, where the side walls tie the posts together
        return None
    A = sum(face_props(f, 0)[0] for f in faces)
    props = []
    for f in faces:
        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(f.wrapped, g)
        props.append((g.Mass(), g.CentreOfMass().X(), g.MatrixOfInertia().Value(2, 2)))  # (2,2) = int x^2 dA here
    xc = sum(a * x for a, x, _ in props) / A
    I = sum(i + a * (x - xc) ** 2 for a, x, i in props)
    core = max(t - 2 * T_SHELL, 0.0) / t
    return I * (1 - (1 - INFILL) * core ** 3), max(max(xs) - xc, xc - min(xs))


def beam_check(parts, pose, link, L):
    """Bending stress and tip-deflection contribution along one arm link (pose horizontal)."""
    shape = next(p for p in parts if p["link"] == link and p["name"] in ("upper_arm", "forearm"))
    shape = shape["shape"]
    idx = ta.LINKS.index(link)
    R, o = pose.frames[link]
    tip = pose.point("gripper_link", TIP_POINT)
    rows = []
    for x in np.arange(4.0, L - 2.0, 2.0):
        s = section(shape, x)
        if s is None:
            continue
        station = R @ np.array([x, 0, 0]) + o
        M = 0.0
        for p in pose.parts:
            li = ta.LINKS.index(p["link"])
            if li > idx or (li == idx and p["r"][0] > station[0]):
                M += p["m"] * G * (p["r"][0] - station[0]) / 1000
        M += PAYLOAD * G * (pose.point("gripper_link", GRIP_POINT)[0] - station[0]) / 1000
        sigma = M * 1000 * s["c"] / s["I_eff"]
        rows.append(dict(x=x, M=M, sigma=sigma, I_eff=s["I_eff"], c=s["c"],
                         arm=(tip[0] - station[0])))
    # tip deflection: integral of curvature times lever arm to the tip
    defl = 0.0
    for a, b in zip(rows, rows[1:]):
        dx = b["x"] - a["x"]
        k = 0.5 * (a["M"] * 1000 / (E_PLA * a["I_eff"]) + b["M"] * 1000 / (E_PLA * b["I_eff"]))
        defl += k * 0.5 * (a["arm"] + b["arm"]) * dx
    return rows, defl


# ---------------------------------------------------------------- report

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true", help="skip the CAD joint-range sweep")
    args = ap.parse_args()

    parts = collect_parts()
    out = []
    w = out.append
    w("# Talos design check results\n")
    w("Generated by `design_check.py` from the CAD in `../cad/talos_arm.py`.\n")

    # --- masses
    w("## Mass budget\n")
    w(f"Printed parts: PLA {RHO_PLA*1e3:.2f} g/cm3, {T_SHELL} mm perimeters/skins, "
      f"{INFILL:.0%} infill. Servos: {M_SERVO:.0f} g + {M_HARDWARE:.0f} g screws and cable.\n")
    w("| Link | Printed (g) | Servo (g) | Total (g) |")
    w("| --- | --- | --- | --- |")
    total = 0
    for link in ta.LINKS:
        pr = sum(p["m"] for p in parts if p["link"] == link and not p["name"].startswith("servo")
                 and p["name"] != "bus_servo_board")
        sv = sum(p["m"] for p in parts if p["link"] == link and p["name"].startswith("servo"))
        total += pr + sv
        label = "base (fixed)" if link == ta.LINKS[0] else link
        w(f"| {label} | {pr*1000:.0f} | {sv*1000:.0f} | {(pr+sv)*1000:.0f} |")
    moving = sum(link_mass(parts, l) for l in ta.LINKS[1:])
    w(f"| **total** | | | **{total*1000:.0f}** |\n")
    w(f"Moving mass above the base: {moving*1000:.0f} g.\n")

    ranges = {} if args.fast else joint_ranges(parts)

    # --- statics
    hz = Pose(parts, HORIZONTAL)
    w("## 1. Static holding torque, arm straight out (worst case)\n")
    w(f"Limit: {LIMIT:.2f} N*m continuous (1/3 of {TAU_STALL} N*m stall). "
      f"Payload {PAYLOAD*1000:.0f} g at mid-jaw ({D_TOOL:.0f} mm past the wrist-flex axis).\n")
    w("| Joint | No payload (N*m) | With payload (N*m) | % stall | vs limit |")
    w("| --- | --- | --- | --- | --- |")
    static = {}
    for name in ("shoulder", "elbow", "wrist_flex"):
        t0, t1 = hz.gravity_torque(name), hz.gravity_torque(name, PAYLOAD)
        static[name] = (t0, t1)
        w(f"| {name} | {t0:.2f} | {t1:.2f} | {100*t1/TAU_STALL:.0f} % | "
          f"{'OK' if t1 <= LIMIT else 'OVER'} ({t1/LIMIT:.2f}x) |")
    roll = max(Pose(parts, (0, 90, 0, 0, a, 0)).gravity_torque("wrist_roll", PAYLOAD)
               for a in range(0, 360, 15))
    w(f"| wrist_roll (worst roll angle) | | {roll:.3f} | {100*roll/TAU_STALL:.1f} % | OK |")
    tip_sh = hz.gravity_torque("shoulder", PAYLOAD, TIP_POINT)
    w(f"\nPayload held at the very tip instead: shoulder {tip_sh:.2f} N*m.\n")
    shoulder_share = PAYLOAD * hz.unit_payload_torque("shoulder")
    w(f"The payload is {shoulder_share:.2f} N*m of the shoulder load "
      f"({100*shoulder_share/static['shoulder'][1]:.0f} %).\n")

    # comparison with torque_check.m inputs
    w("### Against the earlier `torque_check.m` model\n")
    w("| | torque_check.m (SO-101 values) | This CAD |")
    w("| --- | --- | --- |")
    w(f"| Upper arm L1 | 120 mm | {ta.L1:.0f} mm |")
    w(f"| Forearm L2 | 120 mm | {ta.L2:.0f} mm |")
    w(f"| Wrist flex -> tip | 160 mm | {ta.X_ROLL + TIP_POINT[0]:.0f} mm |")
    w(f"| Moving links (4) | 385 g | {sum(link_mass(parts, l) for l in ta.LINKS[2:])*1000:.0f} g |")
    w(f"| Shoulder, 200 g payload | 1.62 N*m | {static['shoulder'][1]:.2f} N*m |")
    w(f"| Elbow, 200 g payload | 0.96 N*m | {static['elbow'][1]:.2f} N*m |\n")

    # lumped inputs for torque_check.m
    w("Lumped inputs to paste into `torque_check.m` (per-link mass incl. servo, "
      "CoM distance from the link's own joint, arm straight):\n")
    w("```")
    for link, nm, axis_joint in (("upper_arm_link", "upper arm + elbow servo", "shoulder"),
                                 ("forearm_link", "forearm + wrist-flex servo", "elbow"),
                                 ("wrist_link", "wrist + wrist-roll servo", "wrist_flex"),
                                 ("gripper_link", "gripper + jaw servo", "wrist_flex")):
        ps = [p for p in hz.parts if p["link"] == link]
        m = sum(p["m"] for p in ps)
        cx = sum(p["m"] * p["r"][0] for p in ps) / m
        ox = hz.axis(axis_joint)[1][0]
        w(f"{nm:28s} m = {m:.3f} kg, CoM {cx-ox:6.1f} mm from {axis_joint} axis")
    w("```\n")

    # torque vs shoulder elevation
    elev = np.arange(-90, 91, 5)
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for q3, style in ((0, "-"), (45, "--"), (90, ":")):
        tau = [Pose(parts, (0, 90 - e, q3, 0, 0, 0)).gravity_torque("shoulder", PAYLOAD) for e in elev]
        ax.plot(elev, tau, style, lw=1.8, label=f"elbow bent {q3} deg")
    ax.axhline(LIMIT, color="tab:red", ls="--", lw=1, label="1/3 stall")
    ax.axhline(TAU_STALL, color="k", lw=1, label="stall")
    if ranges:
        e_min = 90 - ranges["shoulder"][1]
        ax.axvspan(-90, e_min, color="0.85", zorder=0, label="beyond shoulder stop")
    ax.set_xlim(-90, 90)
    ax.set_xlabel("Upper-arm elevation from horizontal (deg)")
    ax.set_ylabel("Shoulder torque (N*m)")
    ax.set_title(f"Shoulder torque with {PAYLOAD*1000:.0f} g payload (CAD masses)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "shoulder_torque_cad.png", dpi=150)
    plt.close(fig)
    w("![Shoulder torque vs elevation](shoulder_torque_cad.png)\n")

    # --- joint ranges
    if ranges:
        w("## 2. Joint ranges from CAD collision sweep\n")
        w("Swept in 5 deg steps until any solid overlaps by more than 1 mm3. "
          "Angles in the model convention (shoulder 0 = upper arm vertical, "
          "+ = forward/down).\n")
        w("| Joint | Min (deg) | Max (deg) | Span (deg) |")
        w("| --- | --- | --- | --- |")
        for k, (lo, hi) in ranges.items():
            w(f"| {k} | {lo} | {hi} | {hi - lo} |")
        w("| pan, wrist_roll | no mechanical stop | | servo 0-360 deg, cable-limited |\n")

    # --- payload envelope
    w("## 3. Payload envelope\n")
    w("Largest payload that keeps shoulder, elbow and wrist under the continuous limit, "
      "from IK over the reachable workspace.\n")
    fig, ax = plt.subplots(figsize=(7, 4.2))
    env_rows = []
    for label, z, elev_t in (("top-down pick, object centre 25 mm above table", 25.0, -90),
                             ("side grasp, object centre 45 mm above table", 45.0, 0),
                             ("horizontal reach at shoulder height", ta.H_PAN + ta.H_SH, 0)):
        rs, ms = [], []
        for r in np.arange(60, 460, 5):
            q = ik(r, z, elev_t)
            if q is None or (ranges and not within(q, ranges)):
                continue
            pz = Pose(parts, q)
            cap = min((LIMIT - pz.gravity_torque(j)) / pz.unit_payload_torque(j)
                      if pz.unit_payload_torque(j) > 1e-9 else 9.9
                      for j in ("shoulder", "elbow", "wrist_flex"))
            rs.append(r)
            ms.append(max(cap, 0) * 1000)
        ax.plot(rs, np.minimum(ms, 1500), lw=1.8, label=label)
        if rs:
            r200 = max((r for r, m in zip(rs, ms) if m >= PAYLOAD * 1000), default=None)
            env_rows.append((label, min(rs), max(rs), r200))
    ax.axhline(PAYLOAD * 1000, color="tab:red", ls="--", lw=1, label=f"{PAYLOAD*1000:.0f} g target")
    ax.set_xlabel("Payload radius from base axis (mm)")
    ax.set_ylabel("Max continuous payload (g)")
    ax.set_ylim(0, 1500)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Payload envelope (1/3-stall limit on every pitch joint)")
    fig.tight_layout()
    fig.savefig(HERE / "payload_envelope.png", dpi=150)
    plt.close(fig)
    w("| Task | Reachable radius (mm) | 200 g OK out to (mm) |")
    w("| --- | --- | --- |")
    for label, r0, r1, r200 in env_rows:
        w(f"| {label} | {r0:.0f} - {r1:.0f} | {r200 if r200 is not None else 'never':} |")
    w("\n![Payload envelope](payload_envelope.png)\n")

    # --- dynamics
    w("## 4. Dynamic torque\n")
    w("Triangular velocity profile (accelerate half way, decelerate half way): "
      "alpha = 4*theta/t^2, omega_peak = 2*theta/t.\n")
    w("| Joint | Move | Inertia (kg*m^2) | alpha (rad/s^2) | omega peak (rpm) | "
      "Accel torque (N*m) | Static + accel (N*m) | % stall |")
    w("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for joint, deg, t in (("pan", 180, 1.5), ("shoulder", 90, 1.0), ("elbow", 90, 0.8)):
        J = hz.inertia(joint, PAYLOAD)
        th = math.radians(deg)
        alpha, om = 4 * th / t ** 2, 2 * th / t
        ta_ = J * alpha
        st = 0 if joint == "pan" else hz.gravity_torque(joint, PAYLOAD)
        w(f"| {joint} | {deg} deg in {t} s | {J:.4f} | {alpha:.1f} | {om*60/(2*math.pi):.0f} | "
          f"{ta_:.3f} | {st + ta_:.2f} | {100*(st+ta_)/TAU_STALL:.0f} % |")
    w("\nPeak speeds are well under the servo's no-load speed (about 0.22 s/60 deg, "
      "45 rpm at 12 V; check the datasheet for your variant). Short peaks above the "
      "1/3 line are fine; the limit is for sustained holding.\n")

    # --- pan bearing and tipping
    w("## 5. Base: pan servo overturning moment and tipping\n")
    a, o, idx = hz.axis("pan")
    Mot = np.zeros(3)
    for p in hz.distal(idx):
        Mot += np.cross((p["r"] - o) / 1000, p["m"] * np.array([0, 0, -G]))
    pr = hz.point("gripper_link", GRIP_POINT)
    Mot += np.cross((pr - o) / 1000, PAYLOAD * np.array([0, 0, -G]))
    Mot_h = math.hypot(Mot[0], Mot[1])
    W_above = (moving + PAYLOAD) * G
    w(f"With the arm straight out, the pan servo's output shaft carries an overturning "
      f"moment of **{Mot_h:.2f} N*m** plus {W_above:.1f} N of axial load. On the SO-101 "
      f"layout this goes through the servo's own output bearing, and the turntable "
      f"rides 2.8 mm clear of the column.\n")
    apothem = 66 * math.cos(math.radians(15)) / 1000
    m_all = total + PAYLOAD
    xg = (sum(p["m"] * p["r"][0] for p in hz.parts) + PAYLOAD * pr[0]) / m_all / 1000
    M_tip = m_all * G * (xg - apothem)
    w(f"Whole-arm centre of mass is {xg*1000:.0f} mm from the base axis; the base edge "
      f"is {apothem*1000:.0f} mm out. ")
    if M_tip > 0:
        bolt_x = 54 * math.cos(math.radians(45)) / 1000
        F_bolt = M_tip / (apothem + bolt_x) / 2
        w(f"**The arm tips over unless the base is fixed down** (net overturning "
          f"{M_tip:.2f} N*m). Clamping through the two rear M4 holes needs "
          f"{F_bolt:.1f} N per bolt before dynamic loads; an M4 bolt in a printed "
          f"counterbore is good for well over 100 N.\n")
    else:
        w(f"The arm is stable on its own (margin {-M_tip:.2f} N*m).\n")

    # --- beams
    w("## 6. Link bending stress and deflection\n")
    w(f"Arm straight out with {PAYLOAD*1000:.0f} g. Sections are cut from the CAD every 2 mm; "
      f"the core inside the {T_SHELL} mm perimeters is counted at {INFILL:.0%} stiffness. "
      f"E = {E_PLA/1000:.1f} GPa, allowable {SIGMA_ALLOW:.0f} MPa sustained.\n")
    w("| Link | Worst station (mm from joint) | Moment (N*m) | I_eff (mm^4) | Stress (MPa) | "
      "Safety factor | Tip deflection (mm) |")
    w("| --- | --- | --- | --- | --- | --- | --- |")
    total_defl = 0.0
    for link, L in (("upper_arm_link", ta.L1), ("forearm_link", ta.L2)):
        rows, defl = beam_check(parts, hz, link, L)
        total_defl += defl
        worst = max(rows, key=lambda r: r["sigma"])
        w(f"| {link} | {worst['x']:.0f} | {worst['M']:.2f} | {worst['I_eff']:.0f} | "
          f"{worst['sigma']:.1f} | {SIGMA_ALLOW/worst['sigma']:.1f} | {defl:.2f} |")
    w(f"\nTip sag from bending of the two printed links: about {total_defl:.1f} mm.\n")

    # --- horn screws
    w("## 7. Horn screw bearing\n")
    w("All joint torque goes through the output horn's 4 screws (the idler side only "
      "supports). Bearing stress on the printed clevis plate:\n")
    t_plate = ta.CLV_OUT - ta.CLV_IN
    w("| Joint | Peak torque (N*m) | Force per screw (N) | Bearing stress (MPa) | Safety factor |")
    w("| --- | --- | --- | --- | --- |")
    for joint, tau in (("shoulder (servo stall)", TAU_STALL),
                       ("shoulder (holding, 200 g)", static["shoulder"][1])):
        F = tau / (4 * HORN_SCREW_R / 1000)
        sb = F / (HORN_SCREW_D * t_plate)
        w(f"| {joint} | {tau:.2f} | {F:.0f} | {sb:.1f} | {BEARING_ALLOW/sb:.1f} |")
    w(f"\nPlate thickness {t_plate:.1f} mm, M{HORN_SCREW_D:.0f} screws on a {2*HORN_SCREW_R:.0f} mm "
      f"circle, allowable {BEARING_ALLOW:.0f} MPa.\n")
    # separate idler plates (TAL-110): two M3 into the beam; worst case they carry the whole joint reaction
    n_tab = len(ta.IDLER_TAB_SCREWS)
    w("The idler plates on the upper arm and forearm are a separate part (TAL-110), held by "
      f"{n_tab} M3 screws into the beam. They only support the passive horn, so worst case "
      "they carry the whole joint reaction (everything beyond the joint plus the payload), "
      "doubled for dynamics:\n")
    w("| Idler plate | Joint reaction x2 (N) | Force per screw (N) | Bearing stress (MPa) | Safety factor |")
    w("| --- | --- | --- | --- | --- |")
    for link, joint in (("upper_arm_link", "shoulder"), ("forearm_link", "elbow")):
        idx = ta.LINKS.index(link)
        F = 2 * (sum(link_mass(parts, l) for l in ta.LINKS[idx:]) + PAYLOAD) * G
        sb = F / n_tab / (HORN_SCREW_D * t_plate)
        w(f"| {link} ({joint}) | {F:.1f} | {F/n_tab:.1f} | {sb:.2f} | {BEARING_ALLOW/sb:.0f} |")
    w("")

    # --- gripper
    w("## 8. Gripper\n")
    r_mid, r_tip = 43.0, 64.0
    for lbl, tau in (("continuous (1/3 stall)", LIMIT), ("stall", TAU_STALL)):
        w(f"- Grip force, {lbl}: {tau/(r_mid/1000):.0f} N at mid-jaw, "
          f"{tau/(r_tip/1000):.0f} N at the tip.")
    F = LIMIT / (r_mid / 1000)
    w(f"- With friction {MU_GRIP} on both jaws, the continuous grip holds "
      f"{2*MU_GRIP*F/G:.1f} kg; the 200 g target needs only "
      f"{PAYLOAD*G/(2*MU_GRIP):.1f} N of squeeze, so run the jaw servo at a low torque "
      f"limit to avoid crushing parts.")
    # fixed finger root, just past the housing: grip force at mid-jaw bends it about Y
    grip = next(p["shape"] for p in parts if p["name"] == "gripper_body")
    x_root, x_contact = 60.0, ta.XJ + r_mid
    s = section(grip, x_root)
    for lbl, tau in (("continuous", LIMIT), ("stall", TAU_STALL)):
        M = tau / (r_mid / 1000) * (x_contact - x_root) / 1000
        sig = M * 1000 * s["c"] / s["I_eff"]
        w(f"- Fixed finger root ({2*ta.FINGER_HW:.0f} mm wide, {x_root:.0f} mm out), {lbl} grip: "
          f"{M:.2f} N*m, {sig:.1f} MPa, safety factor {SIGMA_ALLOW/sig:.1f}.")
    if ranges:
        o_max = ranges["gripper"][1]
        th = math.radians(o_max)
        tip = np.array([64 * math.cos(th) + 5.5 * math.sin(th), 64 * math.sin(th) - 5.5 * math.cos(th)])
        gap = tip[1] + ta.ZJ + 0.5
        w(f"- Jaw opens {o_max} deg before hitting the housing: about {gap:.0f} mm "
          f"between the jaw tips.")
    w("")

    # --- resolution
    w("## 9. Resolution and stiffness at the tip\n")
    step = 2 * math.pi / COUNTS
    arms = {j: np.linalg.norm((hz.point("gripper_link", TIP_POINT) - hz.axis(j)[1])[[0, 2]]) / 1000
            for j in ("shoulder", "elbow", "wrist_flex")}
    w("| Joint | Lever arm to tip (mm) | Tip motion per encoder count (mm) | per 0.5 deg backlash (mm) |")
    w("| --- | --- | --- | --- |")
    for j, r in arms.items():
        w(f"| {j} | {r*1000:.0f} | {r*step*1000:.2f} | {r*math.radians(0.5)*1000:.1f} |")
    w(f"\nOne count on every pitch joint moves the tip up to "
      f"{sum(arms.values())*step*1000:.1f} mm. Repeatability will be set by gear backlash "
      f"and the {total_defl:.1f} mm link sag, not by encoder resolution.\n")

    # --- forearm length sensitivity (rebuilds the CAD)
    w("## 10. Forearm length sensitivity\n")
    w("CAD rebuilt with a different elbow-to-wrist length; everything else unchanged. "
      "Arm straight out, 200 g payload.\n")
    w("| Forearm L2 (mm) | Reach, shoulder to tip (mm) | Shoulder (N*m) | Elbow (N*m) | Elbow vs limit |")
    w("| --- | --- | --- | --- | --- |")
    L2_design = ta.L2
    for L2 in (115.0, 120.0, 125.0, L2_design):
        ta.L2 = L2
        pz = Pose(collect_parts(), HORIZONTAL)
        sh, el = pz.gravity_torque("shoulder", PAYLOAD), pz.gravity_torque("elbow", PAYLOAD)
        reach = ta.L1 + L2 + ta.X_ROLL + TIP_POINT[0]
        mark = " (design)" if L2 == L2_design else ""
        w(f"| {L2:.0f}{mark} | {reach:.0f} | {sh:.2f} | {el:.2f} | {el/LIMIT:.2f}x |")
    ta.L2 = L2_design
    w("")

    # --- servo mounts
    w("## 11. Servo mounts\n")
    w("### Shoulder cradle posts\n")
    sh_shape = next(p["shape"] for p in parts if p["name"] == "shoulder")  # in its own link frame
    z_top = sh_shape.BoundingBox().zmax
    z_screws = ta.H_SH + (ta.CASE_FRONT[0][0] + ta.CASE_BACK[0][0]) / 2
    rows = [(z, post_section(sh_shape, z)) for z in np.arange(z_top - 1.0, 20.0, -1.0)]
    rows = [(z, r) for z, r in rows if r]
    w("The shoulder servo's reaction torque is a couple between its case screws, low in the cradle, and "
      "the top of one cradle post beside the shaft. That bends the post about its root, across the layer "
      "lines (the shoulder prints turntable down). Worst case: the case bears on the very top of the post.\n")
    w("| Load | Torque (N*m) | Force on the post (N) | Root moment (N*m) | Stress (MPa) | Safety factor |")
    w("| --- | --- | --- | --- | --- | --- |")
    post = {}
    for lbl, tau in (("holding, no payload", static["shoulder"][0]), ("holding, 200 g", static["shoulder"][1]),
                     ("servo stall", TAU_STALL)):
        F = tau / ((z_top - z_screws) / 1000)
        z, (I, c) = max(rows, key=lambda r: (z_top - r[0]) * r[1][1] / r[1][0])
        M = F * (z_top - z) / 1000
        sig = M * 1000 * c / I
        post[lbl] = sig
        w(f"| {lbl} | {tau:.2f} | {F:.0f} | {M:.2f} | {sig:.1f} | {SIGMA_ALLOW/sig:.1f} |")
    w(f"\nPost root at z {z:.0f} above the turntable underside, top at {z_top:.0f}; case screws at "
      f"{z_screws:.1f}. Allowable {SIGMA_ALLOW:.0f} MPa sustained, as for the links, but here the stress "
      "pulls the layers apart, where printed PLA is weakest.\n")
    w("### Pan servo case screws\n")
    w(f"The pan servo carries the {Mot_h:.2f} N*m overturning moment into the base through its bottom face "
      "and its case screws. The servo tips about an edge of its bottom face; the screws on the far side "
      "hold it down. The arm pans, so the moment can point any way.\n")
    w("| Tipping about | Far pair only, per screw (N) | Both pairs, per screw (N) |")
    w("| --- | --- | --- |")
    far, near = ta.CASE_BACK[0][0], ta.CASE_BACK_SHAFT[0][0]
    hy, M = ta.CASE_BACK[0][1], Mot_h * 1000
    worst4 = 0.0
    for lbl, lev2, lev4 in (
            ("the shaft-side edge of the bottom face", [ta.FACE_X - far] * 2,
             [ta.FACE_X - far] * 2 + [ta.FACE_X - near] * 2),
            ("the far end", [far - ta.CASE_X[0]] * 2, [far - ta.CASE_X[0]] * 2 + [near - ta.CASE_X[0]] * 2),
            ("a long side", [ta.CASE_HW - hy, ta.CASE_HW + hy], [ta.CASE_HW - hy, ta.CASE_HW + hy] * 2)):
        f2 = M * max(lev2) / sum(d * d for d in lev2)
        f4 = M * max(lev4) / sum(d * d for d in lev4)
        worst4 = max(worst4, f4)
        w(f"| {lbl} | {f2:.0f} | {f4:.0f} |")
    seat = math.pi * (ta.SCREW[2]["head_r"] ** 2 - ta.SCREW[2]["clear"] ** 2)
    w(f"\nWith four screws the worst is {worst4:.0f} N on an M2, {worst4/seat:.1f} MPa under its head "
      f"(allowable {BEARING_ALLOW:.0f} MPa). With only the far pair, a moment that lifts the shaft end has "
      "a few millimetres of lever, so the servo rocks in its 0.4 mm clearance instead of being held.\n")

    (HERE / "design_check_results.md").write_text("\n".join(out), encoding="utf-8")
    print("\n".join(out))


if __name__ == "__main__":
    main()
