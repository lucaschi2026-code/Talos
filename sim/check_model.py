"""Test the MuJoCo model against the CAD and the design calcs, then report what the physics adds.

    python sim/check_model.py            # writes sim/check_results.md and sim/renders/ (several minutes)
    python sim/check_model.py --fast     # one block width in the grip table; prints only, keeps the saved results

Part A compares the simulation with numbers it did not get to choose: the "ready" STEP,
and the tables in calcs/design_check_results.md. A FAIL means the model is wrong.
Part B runs things the static calcs can't: holding under real servo limits, a fast move,
self-collision across the workspace, and a pick.
"""

import re
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_model import BODIES, LIMITS_DEG, ROOT, SERVO, body_of, read_constants, read_step, tessellate  # noqa: E402

XML = HERE / "model" / "talos.xml"
XML_PICK = HERE / "model" / "talos_pick.xml"
RENDERS = HERE / "renders"

cad = read_constants(ROOT / "cad" / "talos_arm.py")
calc = read_constants(ROOT / "calcs" / "design_check.py")
TAU_STALL, LIMIT, G = calc["TAU_STALL"], calc["TAU_STALL"] / 3, calc["G"]
JOINTS = ["pan", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"]
STRAIGHT_OUT = np.radians([0, 90, 0, 0, 0, 0])
READY = np.radians(cad["POSES"]["ready"])
FAST = "--fast" in sys.argv   # one block width instead of seven

# Published in calcs/design_check_results.md
EXPECT_MASS_G = {"moving": 529.0, "upper_arm_link": 115, "forearm_link": 119, "gripper+jaw": 103}
EXPECT_TORQUE = {"shoulder": (0.96, 1.75), "elbow": (0.51, 1.07), "wrist_flex": (0.14, 0.43)}

lines, failures = [], []


def w(text=""):
    lines.append(text)
    print(text)


def check(name, ok, detail):
    w(f"| {name} | {detail} | {'PASS' if ok else '**FAIL**'} |")
    if not ok:
        failures.append(name)


def set_pose(m, d, q):
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = q
    d.ctrl[:6] = q
    mujoco.mj_forward(m, d)


def payload_force(m, d, payload):
    """Apply the payload's weight at the tool site."""
    d.xfrc_applied[:] = 0
    if payload:
        b = m.body("gripper_link").id
        f = np.array([0, 0, -payload * G])
        d.xfrc_applied[b, :3] = f
        d.xfrc_applied[b, 3:] = np.cross(d.site("tool").xpos - d.xipos[b], f)


def holding_torque(m, d, payload):
    """Static joint torque from MuJoCo's gravity terms, plus the payload through the tool Jacobian."""
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, m.site("tool").id)
    return d.qfrc_bias[:6] + (jacp.T @ np.array([0, 0, payload * G]))[:6]


# ---------------------------------------------------------------- part A

def grow(boxes, name, v):
    lo, hi = v.min(0), v.max(0)
    if name in boxes:
        lo, hi = np.minimum(lo, boxes[name][0]), np.maximum(hi, boxes[name][1])
    boxes[name] = (lo, hi)


def bbox_from_model(m, d):
    """Per-body bounding box of the visual meshes where MuJoCo puts them, mm."""
    boxes = {}
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] == 0:
            mid = m.geom_dataid[g]
            a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            v = m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            grow(boxes, m.body(m.geom_bodyid[g]).name, v * 1000)
    return boxes


def bbox_from_step(path):
    boxes = {}
    for p in read_step(path)[0]:
        v, _ = tessellate(p["shape"], linear=0.1, angular=0.35)
        grow(boxes, body_of(p), (p["M"][:3, :3] @ v.T).T + p["M"][:3, 3])
    return boxes


def part_a(m, d):
    w("## A. Does the model match the CAD and the calcs?\n")
    w("| Check | Result | |")
    w("| --- | --- | --- |")

    # Geometry in a pose the build never saw: the joints move the zero-pose meshes to the "ready" pose,
    # then each body is compared with the separately exported ready-pose STEP.
    set_pose(m, d, READY)
    sim, step = bbox_from_model(m, d), bbox_from_step(ROOT / "cad" / "talos_arm_assembly.step")
    worst, where = 0.0, ""
    for body in BODIES:
        err = max(np.abs(sim[body][0] - step[body][0]).max(), np.abs(sim[body][1] - step[body][1]).max())
        if err > worst:
            worst, where = err, body
    check("Ready pose vs `talos_arm_assembly.step`", worst < 0.25, f"worst bounding-box error {worst:.3f} mm ({where})")

    mass = {m.body(b).name: m.body_mass[b] * 1000 for b in range(1, m.nbody)}
    moving = sum(v for k, v in mass.items() if k != "base")
    check("Moving mass", abs(moving - EXPECT_MASS_G["moving"]) < 1.5, f"{moving:.1f} g vs {EXPECT_MASS_G['moving']:.0f} g")
    for link, got in (("upper_arm_link", mass["upper_arm_link"]), ("forearm_link", mass["forearm_link"]),
                      ("gripper+jaw", mass["gripper_link"] + mass["moving_jaw"])):
        check(f"Mass, {link}", abs(got - EXPECT_MASS_G[link]) < 1.5, f"{got:.1f} g vs {EXPECT_MASS_G[link]} g")

    set_pose(m, d, STRAIGHT_OUT)
    for payload, col, label in ((0.0, 0, "no payload"), (calc["PAYLOAD"], 1, "200 g")):
        tau = holding_torque(m, d, payload)
        for joint, expect in EXPECT_TORQUE.items():
            got = abs(tau[JOINTS.index(joint)])
            check(f"{joint} torque, {label}", abs(got - expect[col]) < 0.015, f"{got:.3f} vs {expect[col]:.2f} N*m")

    tip = d.site("tip").xpos
    reach = np.linalg.norm(tip - d.body("upper_arm_link").xpos) * 1000
    check("Reach, shoulder axis to jaw tip", abs(reach - 420) < 3, f"{reach:.1f} mm vs about 420 mm")
    check("Build plan R2: reach from base axis", np.hypot(*tip[:2]) * 1000 >= 350,
          f"{np.hypot(*tip[:2]) * 1000:.0f} mm, needs at least 350 mm")

    ok = all(np.allclose(np.degrees(m.jnt_range[m.joint(j).id]), LIMITS_DEG[j]) for j in JOINTS)
    check("Joint limits", ok, "match the CAD sweep and cable limits")

    set_pose(m, d, np.zeros(6))
    z0 = d.site("tip").xpos[2]
    set_pose(m, d, np.radians([0, 30, 0, 0, 0, 0]))
    down = d.site("tip").xpos[2] < z0
    set_pose(m, d, np.radians([0, 90, 0, 0, 0, 60]))
    opens = d.geom_xpos[m.geom("pad_moving").id][2] > d.geom_xpos[m.geom("pad_fixed").id][2]
    check("Joint directions", down and opens, "shoulder + moves forward and down; gripper + opens")
    w()


# ---------------------------------------------------------------- part B

def holding(m, d):
    w("## B. What the physics adds\n")
    w("### Holding a pose with torque-limited servos\n")
    w("3 s from rest. Sag and tip drop come from the servo gains, so treat them as estimates until measured on the real arm.\n")
    w("| Pose | Payload | Shoulder (N*m) | % stall | Elbow (N*m) | Wrist (N*m) | Shoulder sag (deg) | Tip drop (mm) | vs 1/3 limit |")
    w("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for name, q in (("Ready", READY), ("Straight out", STRAIGHT_OUT)):
        for payload in (0.0, calc["PAYLOAD"]):
            set_pose(m, d, q)
            tip0 = d.site("tip").xpos.copy()
            log = []
            for _ in range(int(3.0 / m.opt.timestep)):
                payload_force(m, d, payload)
                mujoco.mj_step(m, d)
                log.append(d.actuator_force[:6].copy())
            f = np.abs(np.array(log[-200:])).mean(0)
            over = [JOINTS[i] for i in (1, 2, 3) if f[i] > LIMIT]
            w(f"| {name} | {payload * 1000:.0f} g | {f[1]:.2f} | {100 * f[1] / TAU_STALL:.0f} % | {f[2]:.2f} | {f[3]:.2f} | "
              f"{np.degrees(abs(q[1] - d.qpos[1])):.2f} | {(tip0[2] - d.site('tip').xpos[2]) * 1000:.1f} | "
              f"{'OVER: ' + ', '.join(over) if over else 'under'} |")
    d.xfrc_applied[:] = 0
    w()


def shoulder_move(m, d, seconds, payload, q_from, q_to):
    """Shoulder between two angles on a triangular velocity profile."""
    q = np.zeros(6)
    q[1] = q_from
    set_pose(m, d, q)
    for _ in range(int(1.0 / m.opt.timestep)):   # settle first, so the numbers are the move and not the start-up jolt
        payload_force(m, d, payload)
        mujoco.mj_step(m, d)
    peak, lag, stalled = 0.0, 0.0, 0.0
    for k in range(int((seconds + 1.0) / m.opt.timestep)):
        s = min(k * m.opt.timestep / seconds, 1.0)
        d.ctrl[1] = q_from + (q_to - q_from) * (2 * s * s if s < 0.5 else 1 - 2 * (1 - s) ** 2)
        lag = max(lag, abs(d.ctrl[1] - d.qpos[1]))   # before the step, when the servo sees this error
        payload_force(m, d, payload)
        mujoco.mj_step(m, d)
        peak = max(peak, abs(d.actuator_force[1]))
        stalled += (abs(d.actuator_force[1]) >= TAU_STALL * 0.999) * m.opt.timestep
    d.xfrc_applied[:] = 0
    return dict(peak=peak, lag=np.degrees(lag), stalled=stalled, end=np.degrees(abs(q_to - d.qpos[1])),
                rpm=2 * abs(q_to - q_from) / seconds * 60 / (2 * np.pi))


def fast_move(m, d):
    w("### Shoulder speed: the servo's torque falls as it speeds up\n")
    w(f"Static calcs treat stall torque as always available, but a motor has none left at its no-load speed. The SO-101 joint damping "
      f"({SERVO['damping']} N*m per rad/s) has about the same slope as a straight line from stall to 45 rpm "
      f"(2.94 N*m / 4.71 rad/s = 0.62), so at speed it eats into the torque the same way. That value comes from SO-101, not a test on this arm, so "
      "**treat these as a warning to measure, not a result**.\n")
    w("Shoulder moving 90 deg with 200 g on a triangular velocity profile. Section 4 of the design check uses the 1.0 s move:\n")
    w("| Move | Time (s) | Peak speed (rpm) | Peak servo torque (N*m) | Time at stall (s) | Worst lag (deg) | Error 1 s after (deg) |")
    w("| --- | --- | --- | --- | --- | --- | --- |")
    for label, a, b in (("Lift: out to up", np.pi / 2, 0.0), ("Lower: up to out", 0.0, np.pi / 2)):
        for seconds in (1.0, 1.5, 2.0, 3.0, 4.0):
            r = shoulder_move(m, d, seconds, calc["PAYLOAD"], a, b)
            w(f"| {label} | {seconds:.1f} | {r['rpm']:.0f} | {r['peak']:.2f} | {r['stalled']:.2f} | {r['lag']:.1f} | {r['end']:.2f} |")
    w()


def collision_scan(m, d, samples=4000):
    w("### Self-collision across the workspace\n")
    rng = np.random.default_rng(1)
    lo, hi = m.jnt_range[:6, 0], m.jnt_range[:6, 1]
    pairs, table_hits, n_self = {}, 0, 0
    for _ in range(samples):
        set_pose(m, d, rng.uniform(lo, hi))
        touching, hit_table = set(), False
        for c in d.contact[:d.ncon]:
            if c.dist > -0.0005:
                continue
            a, b = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
            if "world" in (a, b):
                hit_table = True
            else:
                touching.add(tuple(sorted((a, b))))
        for key in touching:
            pairs[key] = pairs.get(key, 0) + 1
        n_self += bool(touching)
        table_hits += hit_table
    w(f"{samples} random poses inside the joint limits, using simplified convex shapes that err on the side of touching: "
      f"**{100 * n_self / samples:.0f} %** have the arm touching itself and **{100 * table_hits / samples:.0f} %** hit the table. "
      "The single-joint limits don't protect every combination, so control software should check a pose before moving to it.\n")
    top = sorted(pairs.items(), key=lambda kv: -kv[1])[:6]
    if top:
        w("| Bodies in contact | Share of poses |")
        w("| --- | --- |")
        for (a, b), n in top:
            w(f"| {a} / {b} | {100 * n / samples:.1f} % |")
    w()


# ---------------------------------------------------------------- pick

def ik_top_down(r_mm, z_mm):
    """Straight-down gripper with the tool point at radius r and height z. Same maths as ik() in design_check.py."""
    L1, L2 = cad["L1"], cad["L2"]
    wx, wz = r_mm, z_mm + cad["X_ROLL"] + calc["GRIP_POINT"][0] - (cad["H_PAN"] + cad["H_SH"])
    c3 = (wx * wx + wz * wz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c3) > 1:
        raise ValueError(f"({r_mm}, {z_mm}) is out of reach")
    q3 = np.arccos(c3)
    e2 = np.arctan2(wz, wx) + np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
    q2 = np.pi / 2 - e2
    return np.array([0, q2, q3, np.pi - q2 - q3, 0])


def pick_model(width_mm):
    half = width_mm / 2000
    text = XML_PICK.read_text(encoding="utf-8")
    text = re.sub(r'(<geom name="block" type="box" size=")[^"]*"', rf'\g<1>{half} {half} {half}"', text)
    text = re.sub(r'(<body name="block" pos=")[^"]*"', rf'\g<1>0.19 0 {half}"', text)
    text = re.sub(r'(<key name="\w+" qpos="(?:\S+ ){6})\S+ \S+ \S+', rf'\g<1>0.19 0 {half}', text)
    tmp = XML_PICK.with_name("_tmp_pick.xml")   # next to the original so the mesh paths still resolve
    tmp.write_text(text, encoding="utf-8")
    try:
        return mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()


def run_pick(width_mm, shoot=None, mu=None):
    """Straight-down pick of a 200 g cube. In this pose the gripper's +Z points away from the base,
    so the fixed finger is on the inner side of the block and the moving jaw swings in from outside."""
    m = pick_model(width_mm)
    if mu is not None:
        m.geom_friction[:, 0] = mu
    m.actuator_forcerange[5] = (-LIMIT, LIMIT)  # like the servo's torque-limit setting: squeeze, don't crush
    d = mujoco.MjData(m)
    block = m.body("block").id
    r_block = 190.0
    gap = 3.0  # mm between the fixed finger and the block on the way down
    r_tool = r_block - width_mm / 2 + 0.5 - gap
    z_low, z_high = 25.0, 80.0  # tool height: 25 mm keeps the 18 mm of finger past the tool off the table
    opening = np.radians(min(100.0, np.degrees(np.arcsin(min(0.95, (width_mm + 10) / 46))) + 10))

    q = np.r_[ik_top_down(r_tool, z_high), opening]
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = d.ctrl[:6] = q
    mujoco.mj_forward(m, d)

    def wait(seconds):
        for _ in range(int(seconds / m.opt.timestep)):
            mujoco.mj_step(m, d)

    def cartesian(z_from, z_to, seconds):
        steps = int(seconds / m.opt.timestep)
        peak = np.zeros(6)
        for k in range(steps):
            s = (k + 1) / steps
            d.ctrl[:5] = ik_top_down(r_tool, z_from + (z_to - z_from) * s * s * (3 - 2 * s))
            mujoco.mj_step(m, d)
            peak = np.maximum(peak, np.abs(d.actuator_force[:6]))
        return peak

    wait(0.3)
    if shoot:
        shoot(m, d, "1 above the block")
    cartesian(z_high, z_low, 1.5)
    wait(0.3)
    if shoot:
        shoot(m, d, "2 lowered")
    z0 = d.xpos[block][2]

    # Command the jaw fully closed. The torque limit sets the squeeze, and it keeps squeezing through the lift.
    target = d.ctrl[5]
    for _ in range(int(2.5 / m.opt.timestep)):
        target = max(0.0, target - np.radians(40) * m.opt.timestep)
        d.ctrl[5] = target
        mujoco.mj_step(m, d)
    angle = np.degrees(d.qpos[5])
    grip = abs(d.actuator_force[5])
    if shoot:
        shoot(m, d, "3 gripped")

    peak = cartesian(z_low, z_high, 1.5)
    if shoot:
        shoot(m, d, "4 lifted")
    log = []
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        log.append(np.abs(d.actuator_force[:6]).copy())
    if shoot:
        shoot(m, d, "5 held 2 s")
    return dict(raised=(d.xpos[block][2] - z0) * 1000, angle=angle, grip=grip, peak=peak, hold=np.mean(log[-200:], axis=0))


def pick_section():
    w("### Picking up a block\n")
    w(f"A 200 g cube on the table, {190} mm from the base axis: straight-down approach, close with the jaw torque limited to 1/3 stall, "
      f"lift 55 mm, hold 2 s. Friction {calc['MU_GRIP']}, as in the design check.\n")

    frames, holder = [], {}

    def shoot(m, d, label):
        if "r" not in holder:
            holder["r"] = mujoco.Renderer(m, 480, 640)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = (0.17, 0, 0.07)
            cam.distance, cam.azimuth, cam.elevation = 0.42, 90, -8
            holder["c"] = cam
        holder["r"].update_scene(d, holder["c"])
        frames.append((label, holder["r"].render().copy()))

    r = run_pick(25, shoot)
    holder["r"].close()
    save_strip(frames, RENDERS / "pick.png")
    ok = r["raised"] > 40
    w(f"25 mm cube: **{'held' if ok else 'dropped'}**, raised {r['raised']:.1f} mm. Jaw at {r['angle']:.1f} deg, {r['grip']:.2f} N*m. "
      f"Holding: shoulder {r['hold'][1]:.2f}, elbow {r['hold'][2]:.2f}, wrist {r['hold'][3]:.2f} N*m "
      f"({'all under' if r['hold'][1:4].max() < LIMIT else 'above'} the {LIMIT:.2f} N*m limit). "
      f"Peak in the lift: shoulder {r['peak'][1]:.2f}, elbow {r['peak'][2]:.2f} N*m.\n")
    w("![Pick sequence](renders/pick.png)\n")

    w("The moving jaw pivots instead of sliding, so a wider block sits in a steeper wedge. Same test at other widths and friction values. "
      "The pads are flat, so the real serrations could grip better than this:\n")
    frictions = (0.3, 0.5, 0.8)
    w("| Block width (mm) | Jaw angle at grip (deg) | " + " | ".join(f"Friction {f}" for f in frictions) + " |")
    w("| --- | --- | " + " | ".join("---" for _ in frictions) + " |")
    for width in (25,) if FAST else (15, 20, 25, 30, 35, 40, 50):
        runs = [run_pick(width, mu=f) for f in frictions]
        cells = ["held" if r["raised"] > 40 else "dropped" for r in runs]
        w(f"| {width} | {runs[0]['angle']:.1f} | " + " | ".join(cells) + " |")
    w()


def save_strip(frames, path):
    path.parent.mkdir(exist_ok=True)
    h, wd = frames[0][1].shape[:2]
    strip = Image.new("RGB", (wd * len(frames), h), "white")
    for i, (label, img) in enumerate(frames):
        tile = Image.fromarray(img)
        ImageDraw.Draw(tile).text((14, 12), label, fill=(20, 20, 20), font_size=26)
        strip.paste(tile, (i * wd, 0))
    strip.save(path)


def render_views(m, d):
    RENDERS.mkdir(exist_ok=True)
    renderer = mujoco.Renderer(m, 720, 960)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0.08, 0, 0.2)
    camera.distance, camera.azimuth, camera.elevation = 0.95, 135, -18
    for name, q in (("zero", np.zeros(6)), ("ready", READY)):
        set_pose(m, d, q)
        renderer.update_scene(d, camera)
        Image.fromarray(renderer.render()).save(RENDERS / f"{name}.png")
    renderer.close()


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    w("# Talos simulation check\n")
    w("Generated by `sim/check_model.py`. MuJoCo model built from `cad/talos_arm_zero.step`.\n")
    part_a(m, d)
    holding(m, d)
    fast_move(m, d)
    collision_scan(m, d)
    pick_section()
    w("## Summary\n")
    w(f"**{'All CAD and calc checks pass.' if not failures else 'FAILED: ' + ', '.join(failures)}**\n")
    render_views(m, d)
    if not FAST:   # a quick run must not replace the full table
        (HERE / "check_results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
