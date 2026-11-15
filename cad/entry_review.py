"""How easy is each servo to put in? The real Feetech servo, pushed along its way in.

For each of the six servos, the Feetech model (horns off, or on where the servo goes in with
them) is moved back out along its entry path in steps. At each depth it reports the room
on the four sides across the path, so a pinch narrower than the seated pocket shows up, and
how far the servo slides with the pocket's tight clearance before it is free. It also says
whether the servo goes in through the face the part is printed on: that face's first layers
flare (elephant's foot), so it needs the lead-in chamfer.

    python entry_review.py
"""

import math
import sys

import cadquery as cq
import numpy as np

import talos_arm as ta
import servo_reference as sr

PART_NO = {"base": "TAL-101", "shoulder": "TAL-102", "upper arm": "TAL-103", "forearm": "TAL-104",
           "wrist": "TAL-105", "gripper": "TAL-106"}
DEPTHS = (0, 3, 6, 10, 15, 20, 25, 30, 40, 50)  # mm the servo has been pulled back out
FAR = 1.5  # room is measured up to this


def room(part, sol, d, far=FAR, tol=0.01):
    """How far `sol` can move along d before it touches `part` (bisection, up to `far`)."""
    def hits(s):
        return part.intersect(sol.translate(d * s)).Volume() > 1e-4
    if not hits(far):
        return far
    lo, hi = 0.0, far
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if hits(mid) else (mid, hi)
    return lo


def bed_dir(part_no, v):
    """A part-frame direction in the print-bed frame."""
    down = ta.PRINT_SETUP[part_no][0]
    if not ta.BED_ROT[down]:
        return np.array(v)
    (ax, ay, az), deg = ta.BED_ROT[down]
    k = np.array((ax, ay, az), float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    th = math.radians(deg)
    R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
    return R @ np.array(v)


def review():
    case, horn, passive = sr.real_servo()
    out = []
    for name, part, loc in sr.holders():
        p = part.val()
        way, horns_on = ta.SERVO_ENTRY[name]
        T = loc.wrapped.Transformation()

        def D(v):
            return cq.Vector(*[sum(T.Value(i, j) * v[j - 1] for j in (1, 2, 3)) for i in (1, 2, 3)])

        axes = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}
        sg = 1 if way[0] == "+" else -1
        d = D(tuple(sg * a for a in axes[way[1]]))
        across = [k for k in "xyz" if k != way[1]]
        sol = case.fuse(horn).fuse(passive) if horns_on else case
        sol = sol.moved(loc)
        rows, free_at = [], None
        for s in DEPTHS:
            at = sol.translate(d * s)
            r = {f"{sgn}{k}": room(p, at, D(tuple(m * a for a in axes[k])))
                 for k in across for sgn, m in (("+", 1), ("-", -1))}
            rows.append((s, r))
            if free_at is None and min(r.values()) >= FAR:
                free_at = s
        bd = bed_dir(PART_NO[name], (d.x, d.y, d.z))
        out.append((name, way, horns_on, rows, free_at, bd))
    return out


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    worst = []
    for name, way, horns_on, rows, free_at, bd in review():
        seated = min(rows[0][1].values())
        tight = [s for s, r in rows if min(r.values()) < FAR]
        pinch = min(min(r.values()) for _, r in rows)
        through_bed = bd[2] < -0.99
        print(f"\n{PART_NO[name]} {name}: way out {way} (servo frame), horns {'on' if horns_on else 'off'}; "
              f"in the print, the servo goes in {'from below, through the bed face' if through_bed else 'from ' + ('above' if bd[2] > 0.99 else 'the side')}")
        for s, r in rows:
            print(f"   {s:4.0f} mm out: " + ", ".join(f"{k} {v:.2f}{'+' if v >= FAR else ''}" for k, v in r.items()))
        print(f"   free of the part after {free_at if free_at is not None else '> 50'} mm; "
              f"tightest room on the way in {pinch:.2f} (seated {seated:.2f})")
        worst.append((name, pinch, seated, through_bed))
    print("\nsummary:")
    for name, pinch, seated, bed in worst:
        flag = "ok" if pinch >= seated - 0.02 else "PINCH"
        print(f"   {flag:5s} {name}: tightest {pinch:.2f}, seated {seated:.2f}"
              + ("; goes in through the bed face" if bed else ""))


if __name__ == "__main__":
    main()
