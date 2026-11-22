"""Run the auto pick and the drop in the box with no window, across a spread of block and box spots.

    python sim/test_autopick.py           # every case (a few minutes)
    python sim/test_autopick.py --quick   # three cases

It drives the arm exactly as drive.py does when you press G or P: the autopilot picks keys every
physics step and step_arm() runs them under the same servo limits. Use it after changing the model,
the jaw or autopick.py, to check the auto pick still works where the README says it does.
Exits with 1 if any case that should work doesn't.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from autopick import AutoPick, PlanError  # noqa: E402
from drive import CONTINUOUS, JOG_SPEED, Block, Box, load_model, prepare, step_arm  # noqa: E402

TIMEOUT = 90.0   # s of simulated time per case


def make_cases():
    """(block distance from the base axis mm, its direction deg, its turn deg, box distance mm, box direction deg,
    expected). Directions are seen from above, counterclockwise from +x, which the arm faces at pan 0.

    "done" cases cover the README's ranges: block 165-220 mm out, up to 100 deg either way and turned up to
    40 deg; box 180-300 mm out within +-105 deg, near the block or across the base from it (the longest swing).
    "refused" cases sit just outside them, and the auto pick must say so before it starts."""
    cases = [(190, 90, 0, 200, -96, "done")]   # the default spots
    for r in (165, 190, 220):
        for a in (-100, -45, 45, 100):
            s = np.sign(a)
            near, across = a - s * 60, -s * 100
            # Box at 180 mm across the base: too tight to clear the walls with the block close in (165 mm), and
            # with it far out (220 mm) after a 200 deg swing the pull-in tilts it out of the jaw. Both are refused.
            tight = "refused" if r == 165 or (r == 220 and abs(a) == 100) else "done"
            cases += [(r, a, 0, 180, across, tight), (r, a, s * 25, 240, near, "done"),
                      (r, a, s * 40, 300, across, "done")]
    cases += [(155, 90, 0, 200, -96, "refused"),    # block too close to the base
              (230, 90, 0, 200, -96, "refused"),    # block too far out
              (190, 90, 0, 170, -96, "refused"),    # box too close to the base
              (190, -100, 0, 180, 110, "refused"),  # box at the edge of the pan range
              (190, 90, 0, 200, 75, "refused")]     # box right next to the block
    return cases


CASES = make_cases()
QUICK = [CASES[0], CASES[5], CASES[-2]]


def polar(r_mm, deg):
    a = np.radians(deg)
    return r_mm / 1000 * np.cos(a), r_mm / 1000 * np.sin(a)


def run(m, lead, block_at, box_at, drop=True):
    """One attempt from the ready pose. Returns (state, message, simulated seconds)."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("ready").id)
    mujoco.mj_forward(m, d)
    block, box = Block(m, d), Box(m)
    block.place(m, d, *polar(*block_at[:2]), np.radians(block_at[2]))
    box.place(m, d, *polar(*box_at), 0.0)
    for _ in range(int(0.3 / m.opt.timestep)):   # let the block settle on the table
        mujoco.mj_step(m, d)
    try:
        auto = AutoPick(m, d, JOG_SPEED, CONTINUOUS, drop=box.drop(d) if drop else None)
    except PlanError as e:
        return "refused", str(e), 0.0
    jaw_state, t = dict(closing=False), 0.0
    while auto.state == "running" and t < TIMEOUT:
        step_arm(m, d, auto.keys(d), m.opt.timestep, lead, jaw_state)
        t += m.opt.timestep
    if auto.state == "running":
        return "timeout", f"still running after {TIMEOUT:.0f} s", t
    return auto.state, auto.message, t


def main():
    cases = QUICK if "--quick" in sys.argv else CASES
    m = load_model(pick=True)
    lead = prepare(m)
    failed = 0
    print(f"{'block: r, dir, turn':<24} {'box: r, dir':<16} {'pick':<8} {'pick + drop':<12} {'':<5} result")
    for r, a, turn, box_r, box_a, expect in cases:
        # Pick only, then pick and drop. A refused case only needs the drop refused: the block alone may be fine.
        pick = run(m, lead, (r, a, turn), (box_r, box_a), drop=False) if expect == "done" else ("-", "", 0.0)
        both = run(m, lead, (r, a, turn), (box_r, box_a), drop=True)
        ok = both[0] == expect and pick[0] in ("done", "-")
        failed += not ok
        print(f"{f'{r} mm, {a:.0f} deg, {turn:.0f} deg':<24} {f'{box_r} mm, {box_a:.0f} deg':<16} {pick[0]:<8} "
              f"{both[0]:<12} {'ok' if ok else 'FAIL':<5} {pick[1] if pick[0] not in ('done', '-') else both[1]}")
    print(f"\n{len(cases) - failed} of {len(cases)} cases did what they should"
          f"{'.' if not failed else f'; {failed} did not.'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
