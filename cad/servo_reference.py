"""Check Talos against the real servo: the Feetech STS3215 CAD from SO-101's assembly.

reference/so101/step/STS3215_from_SO101.step is the servo model inside SO-101's own
assembly STEP (TheRobotStudio/SO-ARM100). This script

  1. measures it (case faces, rib, bump, horns, boss, the M2 mounting holes and the horns'
     M3 holes) and compares each size with the constants talos_arm.py builds its pockets from,
  2. puts the real case into every Talos servo pocket and checks that
       * it sits in the pocket without touching (and how much room it has),
       * it slides out along its way in with both horns off (every servo), and with the
         horns on where the part allows it (base, wrist),
       * each horn clears the shell it turns in, and where the servo goes in with its horns
         off, each horn passes out through its opening (so it can be refitted), and
  3. checks the horn screws against the real horns: thread in the horn, and the tip short
     of the case behind it.

    python servo_reference.py
"""

import math
import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

import talos_arm as ta

REF = Path(__file__).with_name("reference") / "so101" / "step" / "STS3215_from_SO101.step"
# SO-101 servo frame -> Talos servo frame: shaft +Y (horn face y = 0) becomes +Z (horn face 18.7)
TO_TALOS = ta.loc_r(((1, 0, 0), (0, 0, -1), (0, 1, 0)), (0, 0, ta.HORN_FACE))
NAMES = ("mid case", "top case", "bottom case", "motor", "pcb", "gear", "output horn", "passive horn")


def real_parts():
    """{name: solid} of the Feetech model's bodies, in the Talos servo frame."""
    solids = cq.importers.importStep(str(REF)).solids().vals()
    return {name: s.moved(TO_TALOS) for name, s in zip(NAMES, solids)}


def real_servo(parts=None):
    """(case, output horn, passive horn) as solids in the Talos servo frame."""
    parts = parts or real_parts()
    case = parts["mid case"].fuse(parts["top case"]).fuse(parts["bottom case"]).fuse(parts["gear"])
    return case, parts["output horn"], parts["passive horn"]


def z_holes(sol, r, tol=0.06):
    """[(x, y, z from, z to)] of the cylindrical faces of radius r whose axis is along Z."""
    out = {}
    for f in sol.Faces():
        a = BRepAdaptor_Surface(f.wrapped)
        if a.GetType() != GeomAbs_Cylinder:
            continue
        c = a.Cylinder()
        if abs(c.Radius() - r) > tol or abs(c.Axis().Direction().Z()) < 0.999:
            continue
        o, b = c.Axis().Location(), f.BoundingBox()
        key = (round(o.X(), 2), round(o.Y(), 2))
        lo, hi = out.get(key, (b.zmin, b.zmax))
        out[key] = (min(lo, b.zmin), max(hi, b.zmax))
    return [(x, y, lo, hi) for (x, y), (lo, hi) in sorted(out.items())]


def hole_rows(parts):
    """[(what, measured, model, 0)]: the case's M2 mounting holes and the horns' M3 holes."""
    rows = []
    for face, body, model in (("top-face", "top case", ta.CASE_FRONT), ("bottom-face", "bottom case", ta.CASE_BACK),
                              ("bottom-face", "bottom case", ta.CASE_BACK_SHAFT)):
        real = z_holes(parts[body], 0.8)
        for mx, my in model:
            x, y, _, _ = min(real, key=lambda h: (h[0] - mx) ** 2 + (h[1] - my) ** 2)
            rows += [(f"{face} M2 hole ({mx:g}, {my:g}): x", x, mx, 0), (f"{face} M2 hole ({mx:g}, {my:g}): y", y, my, 0)]
    for name in ("output horn", "passive horn"):
        radii = [math.hypot(x, y) for x, y, _, _ in z_holes(parts[name], 1.25) if math.hypot(x, y) > 1]
        rows.append((f"{name} M3 hole circle radius ({len(radii)} holes)", sum(radii) / len(radii), ta.HORN_PCD_R, 0))
    return rows


def screw_rows(parts, case):
    """[(check, ok, text)]: the horn screws against the real horns and the case behind them."""
    out = []
    for name, sign, tip, face in (("output horn", 1, ta.CLV_OUT - 6, ta.HORN_FACE),
                                  ("passive horn", -1, ta.CLV_OUT - ta.PASSIVE_SCREW, ta.PASSIVE_FACE)):
        holes = [h for h in z_holes(parts[name], 1.25) if math.hypot(h[0], h[1]) > 1]
        depth = min(h[3] - h[2] for h in holes)                     # thread length the horn offers
        x, y = holes[0][0], holes[0][1]
        e = slab_extent(case, "z", 0, 0, (x - 1.3, x + 1.3, y - 1.3, y + 1.3, *sorted((0, sign * 30))))
        behind = abs(e[1] if sign > 0 else e[0])                    # the case behind the horn, at the screw circle
        thread, short = min(face - tip, depth), tip - behind
        length = 6 if sign > 0 else ta.PASSIVE_SCREW
        out.append((f"M3 x {length} into the {name}", thread >= 1.5 and short >= 0.8,
                    f"{thread:.2f} mm of thread (horn {depth:.2f} thick at its holes), tip {short:.2f} mm short of the case"))
    return out


def slab_extent(sol, axis, lo, hi, region=None):
    """Extent of `sol` along `axis` inside the box `region` (x0, x1, y0, y1, z0, z1)."""
    if region:
        x0, x1, y0, y1, z0, z1 = region
        sol = sol.intersect(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)))
    if sol.Volume() < 1e-6:
        return None
    b = sol.BoundingBox()
    return {"x": (b.xmin, b.xmax), "y": (b.ymin, b.ymax), "z": (b.zmin, b.zmax)}[axis]


def measurements(case, horn, passive):
    """[(what, measured, model, outward)] for every size the pockets are built from.

    outward says which way is "more room" (see main): +1 a larger model value is a bigger
    servo, -1 a smaller one is, 0 a mating size that must match."""
    big = 200
    rows = []
    e = slab_extent(case, "x", 0, 0, (-big, big, -big, big, -10, 10))
    rows += [("case length, from", e[0], ta.CASE_X[0], -1), ("case length, to", e[1], ta.CASE_X[1], 1)]
    e = slab_extent(case, "y", 0, 0, (-30, 0, -big, big, -10, 10))
    rows += [("case half width", e[1], ta.CASE_HW, 1)]
    rows += [("top face (x -25, y 10)", slab_extent(case, "z", 0, 0, (-26, -24, 9.5, 10.5, 0, 30))[1], ta.TOP_FACE, 1),
             # the rib is 16.45 with four small pips to 16.6: measured over the whole rib
             ("rib top (whole rib)", slab_extent(case, "z", 0, 0, (-35, -8, -8, 8, 0, 30))[1], ta.RIB[3], 1),
             ("rib from x", slab_extent(case, "x", 0, 0, (-40, 0, -1, 1, ta.TOP_FACE + 0.3, 30))[0], ta.RIB[0], -1),
             ("rib to x (at its corners)", slab_extent(case, "x", 0, 0, (-30, -5, 5.5, 7.5, ta.TOP_FACE + 0.3, 30))[1],
              ta.RIB[1], 1),
             ("rib half width", slab_extent(case, "y", 0, 0, (-25, -15, 0, 20, ta.TOP_FACE + 0.3, 30))[1], ta.RIB[2], 1),
             ("bottom face (x -25, y 11)", -slab_extent(case, "z", 0, 0, (-26, -24, 10.5, 11.5, -30, 0))[0],
              ta.BOTTOM_FACE, 1),
             # the bump is 18.25 in the middle with a rim at 18.4: measured over the whole bump
             ("bump bottom (whole bump)", -slab_extent(case, "z", 0, 0, (-30, -17, -10, 10, -30, 0))[0], ta.BUMP[3], 1),
             ("bump from x", slab_extent(case, "x", 0, 0, (-40, -12, -1, 1, -30, -ta.BOTTOM_FACE - 0.3))[0], ta.BUMP[0], -1),
             ("bump to x", slab_extent(case, "x", 0, 0, (-28, -12, -1, 1, -30, -ta.BOTTOM_FACE - 0.3))[1], ta.BUMP[1], 1),
             ("bump half width", slab_extent(case, "y", 0, 0, (-28, -20, 0, 20, -30, -ta.BOTTOM_FACE - 0.3))[1],
              ta.BUMP[2], 1),
             ("idler boss tip", -slab_extent(case, "z", 0, 0, (-4, 4, -4, 4, -30, 0))[0], ta.BOSS[1], 1),
             ("output spline top", slab_extent(case, "z", 0, 0, (-4, 4, -4, 4, 0, 30))[1], ta.SPLINE[1], 1),
             ("output horn face", horn.BoundingBox().zmax, ta.HORN_FACE, 0),
             ("output horn radius", horn.BoundingBox().xmax, ta.HORN_R, 1),
             ("passive horn face", -passive.BoundingBox().zmin, ta.PASSIVE_FACE, 0),
             ("passive horn radius", passive.BoundingBox().xmax, ta.HORN_R, 1)]
    return rows


SLIDE_STEPS = (1, 2, 3, 4.5, 6, 8, 10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30, 35, 40, 45, 50, 55, 60)  # mm
HORN_STEPS = (0.5, 1, 1.5, 2, 3, 4, 6, 8, 12, 20, 30)  # mm: a horn's way out through its opening

ENTRY = ta.SERVO_ENTRY


def holders():
    base, l1 = ta.base_parts()
    sh, l2 = ta.shoulder_parts()
    up, l3, _, _ = ta.arm_link(ta.L1)
    fo, l4, _, _ = ta.arm_link(ta.L2, **ta.FOREARM)
    wr, l5 = ta.wrist_parts()
    gr, l6 = ta.gripper_parts()
    return [("base", base["base"], l1), ("shoulder", sh["shoulder"], l2), ("upper arm", up, l3),
            ("forearm", fo, l4), ("wrist", wr["wrist"], l5), ("gripper", gr["gripper"], l6)]


def gap(part, sol, d, step=0.05, far=2.0):
    """How far `sol` can move along d before it touches `part` (up to `far`)."""
    s = 0.0
    while s < far:
        s2 = s + step
        if part.intersect(sol.translate(cq.Vector(*(s2 * d)))).Volume() > 1e-4:
            return s
        s = s2
    return far


def fit_report(case, horn, passive):
    """[(check, ok, text)] for the real servo in every Talos pocket."""
    out = []
    for name, part, loc in holders():
        p = part.val()
        c, h, ph = (s.moved(loc) for s in (case, horn, passive))
        T = loc.wrapped.Transformation()

        def D(v):
            return cq.Vector(*[sum(T.Value(i, j) * v[j - 1] for j in (1, 2, 3)) for i in (1, 2, 3)])

        ov = p.intersect(c).Volume()
        out.append((f"{name}: real servo case in its pocket", ov < 1e-3, f"overlap {ov:.3f} mm3"))
        room = {k: gap(p, c, D(v)) for k, v in (("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+y", (0, 1, 0)),
                                                 ("-y", (0, -1, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1)))}
        out.append((f"{name}: room round the case (servo frame)", min(room.values()) >= 0.2,
                    ", ".join(f"{k} {v:.2f}" + ("+" if v >= 2.0 else "") for k, v in room.items())))
        way, horns_on = ENTRY[name]
        for hn, hs, hd in (("output horn", h, D((0, 0, 1))), ("passive horn", ph, D((0, 0, -1)))):
            ovh = p.intersect(hs).Volume()
            out.append((f"{name}: {hn} clears the shell", ovh < 1e-3, f"overlap {ovh:.3f} mm3"))
            if not horns_on:  # the horn is refitted after the servo is in: straight in along the shaft
                blocked = [s for s in HORN_STEPS if p.intersect(hs.translate(hd * s)).Volume() > 1e-3]
                out.append((f"{name}: {hn} passes through its opening", not blocked,
                            "free for 30 mm" if not blocked else f"blocked at {blocked[0]} mm"))
        d = D({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[way[1]]) * (1 if way[0] == "+" else -1)
        for label, sol in ((("horns off", c),) + ((("horns on", c.fuse(h).fuse(ph)),) if horns_on else ())):
            blocked = [s for s in SLIDE_STEPS if p.intersect(sol.translate(d * s)).Volume() > 1e-3]
            out.append((f"{name}: slides out {way} with {label}", not blocked,
                        "free for 60 mm" if not blocked else f"blocked at {blocked[0]} mm"))
    return out


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    parts = real_parts()
    case, horn, passive = real_servo(parts)
    bad = 0
    print("Feetech STS3215 (SO-101 CAD) vs the sizes talos_arm.py uses, mm, servo frame:")
    for what, real, model, out in measurements(case, horn, passive) + hole_rows(parts):
        # A model size may be up to 0.3 mm bigger than the real one in the direction that only
        # adds room (out = +1: larger is bigger, -1: smaller is bigger), never smaller.
        # Mating sizes (0) must match: hole positions and horn faces to 0.05.
        d = (model - real) * out
        ok = abs(real - model) <= 0.05 or (out != 0 and 0 < d <= 0.3)
        bad += not ok
        note = "" if abs(real - model) <= 0.05 else "  (model has extra room)" if ok else ""
        print(f"  {'ok  ' if ok else 'DIFF'} {what:46s} real {real:7.2f}   model {model:7.2f}{note}")
    print("\nHorn screws against the real horns:")
    for what, ok, txt in screw_rows(parts, case):
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {what}: {txt}")
    print("\nReal servo in every Talos pocket:")
    for what, ok, txt in fit_report(case, horn, passive):
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {what}: {txt}")
    if bad:
        raise SystemExit(f"{bad} problems")
    print("\nall good")


if __name__ == "__main__":
    main()
