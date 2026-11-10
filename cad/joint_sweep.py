"""Sweep each pitch joint through its travel and report where the moving link first hits.

For the shoulder, elbow and wrist flex, the joint is turned in 5 degree steps from 0 each
way (other joints at zero) until a part of the moving side touches a part of the fixed side
(more than 0.5 mm3). Servo bodies, horns, plugs and screws are included.

    python joint_sweep.py
"""

import functools
import itertools
import sys

import talos_arm as ta

# the part builders are pure: build each once
for fn in ("base_parts", "shoulder_parts", "arm_link", "wrist_parts", "gripper_parts", "moving_jaw",
           "hub_caps", "sts3215"):
    setattr(ta, fn, functools.lru_cache(maxsize=None)(getattr(ta, fn)))
_arm_link = ta.arm_link
ta.arm_link = lambda L, **kw: _arm_link(L, **kw) if not kw else _arm_link(L, **kw)

JOINTS = {"shoulder": 1, "elbow": 2, "wrist flex": 3}   # index into the pose tuple
LINK_OF_JOINT = {1: "upper_arm_link", 2: "forearm_link", 3: "wrist_link"}


def parts_by_side(assy, moving_root, parent=None, inside=False, out=None):
    """[(name, shape, moving?)] with absolute placement."""
    out = [] if out is None else out
    here = (parent or ta.cq.Location()) * assy.loc
    inside = inside or assy.name == moving_root
    if assy.obj is not None:
        shape = assy.obj.val() if isinstance(assy.obj, ta.cq.Workplane) else assy.obj
        out.append((assy.name, shape.moved(here), inside))
    for child in assy.children:
        parts_by_side(child, moving_root, here, inside, out)
    return out


def hits(pose, j):
    parts = parts_by_side(ta.build(pose), LINK_OF_JOINT[j])
    fixed = [(n, s, s.BoundingBox()) for n, s, m in parts if not m]
    moving = [(n, s, s.BoundingBox()) for n, s, m in parts if m]
    found = []
    for (na, sa, ba), (nb, sb, bb) in itertools.product(moving, fixed):
        if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin
                or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
            continue
        v = sa.intersect(sb).Volume()
        if v > 0.5:
            found.append(f"{na} x {nb} ({v:.1f} mm3)")
    return found


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    for name, j in JOINTS.items():
        limits = []
        for sign in (1, -1):
            q, last_free, first_hit = 0, 0, None
            while abs(q) <= 150:
                pose = [0, 0, 0, 0, 0, 0]
                pose[j] = q
                h = hits(tuple(pose), j)
                if h:
                    first_hit = (q, h)
                    break
                last_free = q
                q += 5 * sign
            limits.append((last_free, first_hit))
        (hi, hit_hi), (lo, hit_lo) = limits
        print(f"{name}: free from {lo:+d} to {hi:+d} degrees")
        for q_h in (hit_hi, hit_lo):
            if q_h:
                print(f"    at {q_h[0]:+d}: " + "; ".join(q_h[1][:4]))


if __name__ == "__main__":
    main()
