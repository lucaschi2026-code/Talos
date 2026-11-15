"""Bus cable runs against the cable length, through every joint's travel.

Each hop of the daisy chain (board -> pan -> shoulder -> elbow -> wrist flex -> wrist roll -> jaw)
follows the route in README.md (Wiring) as a line through waypoints fixed to the links: the plug
at each servo, the clips, and the point where it crosses a pitch joint (on the joint's axis, just
outside the idler plate, so a pitch joint barely changes the length). Where the cable has to wrap
round a turning axis (the pan and the wrist roll), that stretch is a helix round the axis at the
wrap radius. The length is the worst over the joint's whole travel, plus ROUTE_FACTOR for the bends
and sag a real cable takes. It must fit the cable's free length: its overall length less the two
plug bodies.

    python cable_check.py               # the cables in CABLES (the BOM)
    python cable_check.py --cable 210   # every run on one length
"""

import argparse
import math

import numpy as np

import talos_arm as ta

# Cable for each run, plug tip to plug tip (mm). The servos come with about 210 mm cables; the two runs
# that wind round a turning axis need longer ones.
CABLES = {"board -> pan": 210, "pan -> shoulder": 350, "shoulder -> elbow": 210, "elbow -> wrist flex": 210,
          "wrist flex -> wrist roll": 210, "wrist roll -> jaw": 300}
PLUG_BODY = 10.0      # mm of the cable's measured length inside each plug
ROUTE_FACTOR = 1.15   # bends, sag, and not running exactly along the straight lines
PAN_RANGE = 110.0     # software limits (README: the pan's service loop sets the pan range)
ROLL_RANGE = 150.0
PITCH = {1: 100.0, 2: 100.0, 3: 95.0}   # shoulder, elbow, wrist flex (joint_sweep.py)
CROSS_Y = -(ta.CLV_OUT + 3.0)           # where the cable crosses a pitch joint: outside the idler plate
PLUG_EXIT = (-14.1, 0.0, -26.0)         # cable leaves the mated plugs, servo frame


def servo_locs():
    return {"pan": ("talos_arm", ta.base_parts()[1]), "shoulder": ("shoulder_link", ta.shoulder_parts()[1]),
            "elbow": ("upper_arm_link", ta.arm_link(ta.L1)[1]),
            "wrist flex": ("forearm_link", ta.arm_link(ta.L2, **ta.FOREARM)[1]),
            "wrist roll": ("wrist_link", ta.wrist_parts()[1]), "jaw": ("gripper_link", ta.gripper_parts()[1])}


def _apply(loc, p):
    T = loc.wrapped.Transformation()
    return [sum(T.Value(i, j) * p[j - 1] for j in (1, 2, 3)) + T.Value(i, 4) for i in (1, 2, 3)]


def polyline(pts):
    return sum(float(np.linalg.norm(b - a)) for a, b in zip(pts, pts[1:]))


def helix(a, b, axis_pt, axis, sweep_deg, r):
    """Length from a to b wrapping `sweep_deg` round an axis at radius r: the arc and the axial travel
    as one helix, plus the radial legs from each end's own radius to r."""
    axis = np.asarray(axis, float)
    def radial(p):
        d = p - axis_pt
        return np.linalg.norm(d - np.dot(d, axis) * axis)
    along = abs(np.dot(b - a, axis))
    arc = r * math.radians(abs(sweep_deg))
    return math.hypot(arc, along) + abs(radial(a) - r) + abs(radial(b) - r)


def angle_about(p, axis_pt, axis, ref):
    """Angle of p round the axis, measured from the direction `ref`."""
    axis = np.asarray(axis, float)
    d = p - axis_pt
    d = d - np.dot(d, axis) * axis
    ref = np.asarray(ref, float) - np.dot(ref, axis) * axis
    return math.degrees(math.atan2(np.dot(np.cross(ref, d), axis), np.dot(ref, d)))


def hops(cable=None):
    """[(run, shortest, longest, joint angle at the longest, free length)]; cable=None uses CABLES."""
    S = servo_locs()

    def plug(name, frames):
        link, loc = S[name]
        return np.array(_apply(frames[link] * loc, PLUG_EXIT))

    def pt(frames, link, p):
        return np.array(_apply(frames[link], p))

    board = (sum(ta.BOARD["x"]) / 2, 0.0, ta.BOARD["z"] + 1.6)
    tunnel_in, tunnel_out = (-14.1, 0.0, 19.6), (-60.0, 0.0, 19.6)
    out = []

    def sweep(name, joint, rng, fn, steps=21):
        lengths = []
        for a in np.linspace(-rng, rng, steps):
            q = [0, 0, 0, 0, 0, 0]
            q[joint] = a
            lengths.append((fn(ta.link_frames(q), a), a))
        lo, hi = min(lengths), max(lengths)
        free = (cable or CABLES[name]) - 2 * PLUG_BODY
        out.append((name, lo[0] * ROUTE_FACTOR, hi[0] * ROUTE_FACTOR, hi[1], free))

    # board -> pan: through the tunnel from the deck, static
    sweep("board -> pan", 0, 0.0, lambda f, a: polyline([pt(f, "talos_arm", board), pt(f, "talos_arm", tunnel_out),
                                                    pt(f, "talos_arm", tunnel_in), plug("pan", f)]))

    # pan -> shoulder: back out of the tunnel, up the back of the column, round the column to the
    # clip on the cradle's back face, then to the shoulder servo's plugs
    col_top = (-44.0, 0.0, ta.H_PAN)
    f0 = ta.link_frames((0, 0, 0, 0, 0, 0))
    z = np.array((0.0, 0.0, 1.0))
    pan_wrap0 = angle_about(pt(f0, "shoulder_link", (-17.0, -9.0, 21.0)), np.zeros(3), z, pt(f0, "talos_arm", col_top))

    def pan_hop(f, q):
        a = pt(f, "talos_arm", col_top)
        clip = pt(f, "shoulder_link", (-17.0, -9.0, 21.0))
        wrap = pan_wrap0 + q  # the loop winds on as the shoulder turns: it can't jump to the other side
        return (polyline([plug("pan", f), pt(f, "talos_arm", tunnel_in), pt(f, "talos_arm", tunnel_out), a])
                + helix(a, clip, np.zeros(3), z, wrap, 44.0) + polyline([clip, plug("shoulder", f)]))
    sweep("pan -> shoulder", 0, PAN_RANGE, pan_hop)

    # pitch joints: across the joint on its axis, outside the idler plate, along the link to its clip
    for name, joint, parent, child, clip in (
            ("shoulder -> elbow", 1, "shoulder", "elbow", ("upper_arm_link", (50.0, -17.2 - 2.5, 0.0))),
            ("elbow -> wrist flex", 2, "elbow", "wrist flex", ("forearm_link", (59.5, -16.2 - 2.5, 0.0)))):
        child_link = S[child][0]
        sweep(name, joint, PITCH[joint], lambda f, a, parent=parent, child=child, child_link=child_link, clip=clip:
              polyline([plug(parent, f), pt(f, child_link, (0.0, CROSS_Y, 0.0)), pt(f, *clip), plug(child, f)]))

    # wrist flex -> wrist roll: across the wrist joint, into the roll servo's plug bay from -Y
    sweep("wrist flex -> wrist roll", 3, PITCH[3], lambda f, a: polyline(
        [plug("wrist flex", f), pt(f, "wrist_link", (0.0, CROSS_Y, 0.0)), pt(f, "wrist_link", (22.0, -26.0, -14.0)),
         plug("wrist roll", f)]))

    # wrist roll -> jaw: out of the bay, through the wrist's clip, a loop round the roll axis, into the
    # plug window in the gripper housing
    def roll_geometry(f):
        clip = pt(f, "wrist_link", (46.0, -15.2 - 2.5, -30.0))
        axis_pt = pt(f, "wrist_link", (0.0, 0.0, 0.0))
        axis = pt(f, "wrist_link", (1.0, 0.0, 0.0)) - axis_pt
        return clip, axis_pt, axis
    c0, ap0, ax0 = roll_geometry(f0)
    roll_wrap0 = angle_about(plug("jaw", f0), ap0, ax0, c0 - ap0)

    def roll_hop(f, q):
        bay = pt(f, "wrist_link", (22.0, -26.0, -14.0))
        clip, axis_pt, axis = roll_geometry(f)
        jaw = plug("jaw", f)
        wrap = roll_wrap0 + q
        return polyline([plug("wrist roll", f), bay, clip]) + helix(clip, jaw, axis_pt, axis, wrap, 28.0)
    sweep("wrist roll -> jaw", 4, ROLL_RANGE, roll_hop)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cable", type=float, default=None, help="one cable length for every run (mm)")
    args = ap.parse_args()
    rows = hops(args.cable)
    bad = 0
    print(f"Cable for each run, less {2 * PLUG_BODY:.0f} mm inside the plugs. "
          f"Run lengths include x{ROUTE_FACTOR} for bends and sag.")
    for name, lo, hi, at, free in rows:
        ok = hi <= free
        bad += not ok
        span = f"{lo:.0f} to {hi:.0f} mm" if hi - lo > 1 else f"{hi:.0f} mm"
        worst = f" (longest at {at:+.0f} deg)" if hi - lo > 1 else ""
        cable = free + 2 * PLUG_BODY
        print(f"  {'ok  ' if ok else 'LONG'} {name:26s} {cable:.0f} mm cable: needs {span}{worst}; spare {free - hi:+.0f} mm")
    if bad:
        raise SystemExit(f"{bad} runs too long for their cables")


if __name__ == "__main__":
    main()
