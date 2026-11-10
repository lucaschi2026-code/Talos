"""Check Talos against the real servo: the Feetech STS3215 CAD from SO-101's assembly.

reference/so101/step/STS3215_from_SO101.step is the servo model inside SO-101's own
assembly STEP (TheRobotStudio/SO-ARM100). This script

  1. measures it (case faces, rib, bump, horns, boss) and compares each size with the
     constants talos_arm.py builds its pockets from, and
  2. puts the real case into every Talos servo pocket and checks that
       * it sits in the pocket without touching (and how much room it has),
       * it slides out along its way in with both horns off (every servo), and with the
         horns on where the part allows it (base, wrist),
       * each horn clears the shell it turns in.

    python servo_reference.py
"""

import sys
from pathlib import Path

import cadquery as cq

import talos_arm as ta

REF = Path(__file__).with_name("reference") / "so101" / "step" / "STS3215_from_SO101.step"
# SO-101 servo frame -> Talos servo frame: shaft +Y (horn face y = 0) becomes +Z (horn face 18.7)
TO_TALOS = ta.loc_r(((1, 0, 0), (0, 0, -1), (0, 1, 0)), (0, 0, ta.HORN_FACE))
NAMES = ("mid case", "top case", "bottom case", "motor", "pcb", "gear", "output horn", "passive horn")


def real_servo():
    """(case, output horn, passive horn) as solids in the Talos servo frame."""
    solids = cq.importers.importStep(str(REF)).solids().vals()
    parts = dict(zip(NAMES, solids))
    case = parts["mid case"].fuse(parts["top case"]).fuse(parts["bottom case"]).fuse(parts["gear"])
    return tuple(s.moved(TO_TALOS) for s in (case, parts["output horn"], parts["passive horn"]))


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
    """[(what, measured, model)] for every size the pockets are built from."""
    big = 200
    rows = []
    e = slab_extent(case, "x", 0, 0, (-big, big, -big, big, -10, 10))
    rows += [("case length, from", e[0], ta.CASE_X[0]), ("case length, to", e[1], ta.CASE_X[1])]
    e = slab_extent(case, "y", 0, 0, (-30, 0, -big, big, -10, 10))
    rows += [("case half width", e[1], ta.CASE_HW)]
    rows += [("top face (x -25, y 10)", slab_extent(case, "z", 0, 0, (-26, -24, 9.5, 10.5, 0, 30))[1], ta.TOP_FACE),
             ("rib top (x -20, y 0)", slab_extent(case, "z", 0, 0, (-21, -19, -1, 1, 0, 30))[1], ta.RIB[3]),
             ("rib from x", slab_extent(case, "x", 0, 0, (-40, 0, -1, 1, ta.TOP_FACE + 0.3, 30))[0], ta.RIB[0]),
             ("rib to x (at its corners)", slab_extent(case, "x", 0, 0, (-30, -5, 5.5, 7.5, ta.TOP_FACE + 0.3, 30))[1], ta.RIB[1]),
             ("rib half width", slab_extent(case, "y", 0, 0, (-25, -15, 0, 20, ta.TOP_FACE + 0.3, 30))[1], ta.RIB[2]),
             ("bottom face (x -25, y 11)", -slab_extent(case, "z", 0, 0, (-26, -24, 10.5, 11.5, -30, 0))[0],
              ta.BOTTOM_FACE),
             ("bump bottom (x -24, y 0)", -slab_extent(case, "z", 0, 0, (-25, -23, -1, 1, -30, 0))[0], ta.BUMP[3]),
             ("bump from x", slab_extent(case, "x", 0, 0, (-40, -12, -1, 1, -30, -ta.BOTTOM_FACE - 0.3))[0], ta.BUMP[0]),
             ("bump to x", slab_extent(case, "x", 0, 0, (-28, -12, -1, 1, -30, -ta.BOTTOM_FACE - 0.3))[1], ta.BUMP[1]),
             ("bump half width", slab_extent(case, "y", 0, 0, (-28, -20, 0, 20, -30, -ta.BOTTOM_FACE - 0.3))[1],
              ta.BUMP[2]),
             ("idler boss tip", -slab_extent(case, "z", 0, 0, (-4, 4, -4, 4, -30, 0))[0], ta.BOSS[1]),
             ("output spline top", slab_extent(case, "z", 0, 0, (-4, 4, -4, 4, 0, 30))[1], ta.SPLINE[1]),
             ("output horn face", horn.BoundingBox().zmax, ta.HORN_FACE),
             ("output horn radius", horn.BoundingBox().xmax, ta.HORN_R),
             ("passive horn face", -passive.BoundingBox().zmin, ta.PASSIVE_FACE),
             ("passive horn radius", passive.BoundingBox().xmax, ta.HORN_R)]
    return rows


# per measurement: which way is "more room" (see main)
OUTWARD = (-1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 0, 1, 0, 1)

ENTRY = {"base": ("+z", True), "shoulder": ("+x", False), "upper arm": ("+y", False),
         "forearm": ("+y", False), "wrist": ("+z", True), "gripper": ("-y", False)}


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
        for hn, hs in (("output horn", h), ("passive horn", ph)):
            ovh = p.intersect(hs).Volume()
            out.append((f"{name}: {hn} clears the shell", ovh < 1e-3, f"overlap {ovh:.3f} mm3"))
        way, horns_on = ENTRY[name]
        d = D({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[way[1]]) * (1 if way[0] == "+" else -1)
        for label, sol in ((("horns off", c),) + ((("horns on", c.fuse(h).fuse(ph)),) if horns_on else ())):
            blocked = [s for s in (1, 2, 4, 7, 10, 15, 20, 25, 30, 40, 50, 60)
                       if p.intersect(sol.translate(d * s)).Volume() > 1e-3]
            out.append((f"{name}: slides out {way} with {label}", not blocked,
                        "free for 60 mm" if not blocked else f"blocked at {blocked[0]} mm"))
    return out


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    case, horn, passive = real_servo()
    bad = 0
    print("Feetech STS3215 (SO-101 CAD) vs the sizes talos_arm.py uses, mm, servo frame:")
    for (what, real, model), out in zip(measurements(case, horn, passive), OUTWARD):
        # a model size may be up to 1.8 mm bigger than the real one in the direction that only
        # adds room (out = +1: larger is bigger, -1: smaller is bigger); mating faces (0) must match
        d = (model - real) * out
        ok = abs(real - model) <= 0.15 or (out != 0 and 0 < d <= 1.8)
        bad += not ok
        note = "" if abs(real - model) <= 0.15 else "  (model has extra room)" if ok else ""
        print(f"  {'ok  ' if ok else 'DIFF'} {what:28s} real {real:7.2f}   model {model:7.2f}{note}")
    print("\nReal servo in every Talos pocket:")
    for what, ok, txt in fit_report(case, horn, passive):
        bad += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {what}: {txt}")
    if bad:
        raise SystemExit(f"{bad} problems")
    print("\nall good")


if __name__ == "__main__":
    main()
