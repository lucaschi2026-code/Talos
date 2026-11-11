"""Drive the simulated Talos arm from the keyboard.

    python sim/drive.py            # the arm on a table
    python sim/drive.py --pick     # with a 200 g block to pick up and a box to drop it in

Hold a key to move a joint. Left drag orbits the view, right drag pans, scroll zooms.
With --pick, G (or its button) picks the block up by itself and shows the keys it presses, P also
drops it in the box, and B lets you move the block or the box.
"""

import argparse
from pathlib import Path

import glfw
import mujoco
import numpy as np

from autopick import AutoPick, PlanError

HERE = Path(__file__).resolve().parent
JOINTS = ["pan", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"]
JAW = JOINTS.index("gripper")

# Change these to whatever feels right to drive: joint -> (key for +, key for -).
KEYMAP = {
    "pan": (glfw.KEY_A, glfw.KEY_D),
    "shoulder": (glfw.KEY_W, glfw.KEY_S),
    "elbow": (glfw.KEY_I, glfw.KEY_K),
    "wrist_flex": (glfw.KEY_U, glfw.KEY_J),
    "wrist_roll": (glfw.KEY_Q, glfw.KEY_E),
    "gripper": (glfw.KEY_O, glfw.KEY_L),
}
# With --pick: pick up the block, pick it up and drop it in the box, move the block or box.
AUTO_KEY, DROP_KEY, MOVE_KEY = glfw.KEY_G, glfw.KEY_P, glfw.KEY_B
MOVE_DIRS = {glfw.KEY_UP: (1, 0), glfw.KEY_DOWN: (-1, 0), glfw.KEY_RIGHT: (0, 1), glfw.KEY_LEFT: (0, -1)}
MOVE_TURN = (glfw.KEY_Z, glfw.KEY_X)           # turn it left, right
MOVE_SPEED, MOVE_SPIN = 0.10, np.radians(90)   # m/s and rad/s while those keys are held
# The box: an open-top tray, prop only (not part of the arm or the generated model files).
BOX_INNER, BOX_WALL, BOX_TOP = 0.035, 0.003, 0.035  # m: inside half-width, wall thickness, wall height
BOX_HOME = (-0.02, -0.20, 0.0)                      # x, y (m) and turn (rad) at start
JOG_SPEED = np.radians(45)     # deg/s while a key is held; the servo's no-load limit is 270 deg/s
STALL = 2.94                   # N*m
CONTINUOUS = STALL / 3         # N*m: rule of thumb from calcs/ for sustained load, not a hard limit


def prepare(m):
    """Set the jaw's squeeze and return how far a target may lead its joint."""
    m.actuator_forcerange[JAW] = (-CONTINUOUS, CONTINUOUS)  # like the servo's torque-limit setting
    # Enough lead to reach stall torque while moving at jog speed, and no more.
    return (STALL + m.actuator_biasprm[:6, 2] * -JOG_SPEED) / m.actuator_gainprm[:6, 0]


def step_arm(m, d, pressed, seconds, lead, jaw_state):
    """Advance the simulation, moving each joint's target by the keys held.

    Targets move a little every physics step: stepping them once per drawn frame would make the
    stiff servos slam into stall every frame. An arm target stays within `lead` of its joint, so a
    blocked arm doesn't wind its target far away and then ignore the keys until the target comes back.

    The jaw works differently, like a servo with a torque limit and a goal past the object: after you
    close it onto something it keeps closing, so the squeeze survives the block shifting. jaw_state
    remembers that you asked it to close, since a jaw pushed open by the table feels the same as a
    squeeze. Opening starts from where the jaw actually is, so it responds at once.
    """
    step = JOG_SPEED * m.opt.timestep
    rate = np.array([pressed.get(j, 0) for j in JOINTS]) * step
    low, high = m.actuator_ctrlrange[:6, 0], m.actuator_ctrlrange[:6, 1]
    arm = np.arange(6) != JAW
    jaw_key = pressed.get("gripper", 0)
    if jaw_key:
        jaw_state["closing"] = jaw_key < 0
    for _ in range(int(round(seconds / m.opt.timestep))):
        target = d.ctrl[:6] + rate
        target[arm] = np.clip(target[arm], d.qpos[:6][arm] - lead[arm], d.qpos[:6][arm] + lead[arm])
        if jaw_key > 0:
            target[JAW] = max(d.ctrl[JAW], d.qpos[JAW]) + step
        elif jaw_key == 0 and jaw_state["closing"] and d.actuator_force[JAW] <= -0.9 * CONTINUOUS:
            target[JAW] = d.ctrl[JAW] - step
        d.ctrl[:6] = np.clip(target, low, high)
        mujoco.mj_step(m, d)


def joint_load(m, d):
    """Torque the motor is producing. The joint damping stands in for the motor's own speed
    limit, so it's taken off: it isn't load on the servo."""
    return np.abs(d.actuator_force[:6] - m.dof_damping[:6] * d.qvel[:6])


def load_flag(i, load):
    if i == JAW:
        return ""                         # the jaw is deliberately held at its torque limit
    if load >= 0.95 * STALL:
        return "   AT STALL: blocked or overloaded"
    return "   above 1/3 stall" if load > CONTINUOUS else ""


def key_label(key):
    return (glfw.get_key_name(key, 0) or "?").upper()


def jog_label(joint, sign):
    """The key that moves a joint one way, e.g. jog_label("shoulder", -1) -> "S"."""
    return key_label(KEYMAP[joint][0 if sign > 0 else 1])


def status_text(d, load, props=()):
    """Two overlay columns. load is smoothed: the 1/3 rule is about sustained load, not brief peaks."""
    values = [f"{np.degrees(d.qpos[i]):.1f} deg   {load[i]:.2f} N*m{load_flag(i, load[i])}" for i in range(len(JOINTS))]
    tip = d.site("tip").xpos * 1000
    names, values = JOINTS + ["tip (mm)"], values + [f"x {tip[0]:.0f}   y {tip[1]:.0f}   z {tip[2]:.0f}"]
    for prop in props:
        x, y, yaw = prop.pose(d)
        names.append(f"{prop.name} (mm)")
        values.append(f"x {x * 1000:.0f}   y {y * 1000:.0f}   turned {np.degrees(yaw):.0f} deg")
    return "\n".join(names), "\n".join(values)


def help_text(pick, moving):
    names = JOINTS + ["reset"]
    keys = [f"{key_label(a)} / {key_label(b)}" for a, b in KEYMAP.values()] + ["R"]
    if pick:
        names += ["auto pick", "pick + drop in box", "move block / box"]
        keys += [key_label(AUTO_KEY), key_label(DROP_KEY), f"{key_label(MOVE_KEY)}  ({moving or 'off'})"]
        if moving:
            names += ["   slide", "   turn"]
            keys += ["arrows, or drag it", f"{key_label(MOVE_TURN[0])} / {key_label(MOVE_TURN[1])}"]
    return "\n".join(names), "\n".join(keys)


def auto_text(auto, note):
    """Top-right panel: each step of the auto pick and the keys it held, in order."""
    if auto is None:
        return "AUTO PICK", note or f"press {key_label(AUTO_KEY)}, {key_label(DROP_KEY)} or a button"
    state = {"running": "running...", "done": "done", "failed": "FAILED", "stopped": "stopped"}[auto.state]
    rows = [("AUTO PICK", state)] + auto.report(jog_label)
    current = min(auto.i, len(auto.phases) - 1)
    rows.append(("   aiming for", auto.goals_text(current)))
    if auto.message or note:
        rows.append(("", note or auto.message))
    return "\n".join(r[0] for r in rows), "\n".join(r[1] for r in rows)


def load_model(pick):
    """The arm, plus the block and the box with --pick. The box is added here rather than in
    build_model.py: it's a prop for driving, and check_model.py's tests shouldn't see it."""
    if not pick:
        return mujoco.MjModel.from_xml_path(str(HERE / "model" / "talos.xml"))
    spec = mujoco.MjSpec.from_file(str(HERE / "model" / "talos_pick.xml"))
    box = spec.worldbody.add_body(name="box", mocap=True, pos=[*BOX_HOME[:2], 0])
    half_out = BOX_INNER + BOX_WALL
    parts = [((0, 0, BOX_WALL / 2 + 0.0005), (half_out, half_out, BOX_WALL / 2))]   # floor, just off the table
    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        size = (BOX_WALL / 2, half_out, BOX_TOP / 2) if sx else (half_out, BOX_WALL / 2, BOX_TOP / 2)
        parts.append(((sx * (BOX_INNER + BOX_WALL / 2), sy * (BOX_INNER + BOX_WALL / 2), BOX_TOP / 2), size))
    for pos, size in parts:
        g = box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(pos), size=list(size), rgba=[0.25, 0.45, 0.75, 1])
        g.contype, g.conaffinity = 0, 3          # touches the block (contype 2) and the arm's hulls (contype 1)
        g.friction = [0.3, 0.005, 0.0001]
    return spec.compile()


class Block:
    """The block on the table, and where R puts it back to."""
    name = "block"

    def __init__(self, m, d):
        self.id = m.body("block").id
        joint = m.body_jntadr[self.id]
        self.qadr, self.vadr = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
        self.half = m.geom("block").size[0]
        self.home = self.pose(d)

    def pose(self, d):
        """x, y and which way it's turned (taken from whichever side faces are vertical)."""
        R = d.xmat[self.id].reshape(3, 3)
        side = R[:, (int(np.argmax(np.abs(R[2]))) + 1) % 3]
        return d.xpos[self.id][0], d.xpos[self.id][1], wrap_quarter(np.arctan2(side[1], side[0]))

    def place(self, m, d, x, y, yaw):
        """Put it flat on the table, outside the base and inside the table edge."""
        r = np.hypot(x, y)
        if r < 0.07:
            x, y = np.array([x, y]) * 0.07 / r if r > 1e-6 else (0.0, 0.07)
        x, y = np.clip([x, y], -0.45, 0.45)
        d.qpos[self.qadr:self.qadr + 3] = (x, y, self.half)
        d.qpos[self.qadr + 3:self.qadr + 7] = (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2))
        d.qvel[self.vadr:self.vadr + 6] = 0
        mujoco.mj_forward(m, d)
        self.home = self.pose(d)

    def under(self, d, x, y):
        bx, by, _ = self.pose(d)
        return np.hypot(x - bx, y - by) < 2.5 * self.half


class Box:
    """The box to drop the block in. It's a mocap body: solid, but moved by setting where it is."""
    name = "box"

    def __init__(self, m):
        self.mocap = m.body_mocapid[m.body("box").id]
        self.home = BOX_HOME

    def pose(self, d):
        q = d.mocap_quat[self.mocap]
        return d.mocap_pos[self.mocap][0], d.mocap_pos[self.mocap][1], wrap_quarter(2 * np.arctan2(q[3], q[0]))

    def place(self, m, d, x, y, yaw):
        r, keep_out = np.hypot(x, y), 0.07 + BOX_INNER + BOX_WALL   # keep it off the base
        if r < keep_out:
            x, y = np.array([x, y]) * keep_out / r if r > 1e-6 else (0.0, -keep_out)
        x, y = np.clip([x, y], -0.45, 0.45)
        d.mocap_pos[self.mocap] = (x, y, 0)
        d.mocap_quat[self.mocap] = (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2))
        mujoco.mj_forward(m, d)
        self.home = self.pose(d)

    def under(self, d, x, y):
        bx, by, _ = self.pose(d)
        return np.hypot(x - bx, y - by) < 1.2 * (BOX_INNER + BOX_WALL)

    def drop(self, d):
        """What AutoPick needs to drop into it."""
        x, y, yaw = self.pose(d)
        return dict(x=x, y=y, yaw=yaw, inner=BOX_INNER, top=BOX_TOP)


def wrap_quarter(a):
    """A cube looks the same every 90 deg, so show its turn as -45..45 deg."""
    return (a + np.pi / 4) % (np.pi / 2) - np.pi / 4


def table_point(m, cam, relx, rely, aspect, z):
    """Where the ray under the cursor meets the horizontal plane at height z, or None.
    relx, rely run 0..1 from the bottom left of the view."""
    az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
    forward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    up = np.array([-np.sin(el) * np.cos(az), -np.sin(el) * np.sin(az), np.cos(el)])
    right = np.cross(forward, up)
    eye = np.array(cam.lookat) - cam.distance * forward
    t = np.tan(np.radians(m.vis.global_.fovy) / 2)
    ray = forward + (2 * relx - 1) * t * aspect * right + (2 * rely - 1) * t * up
    if ray[2] > -1e-6:
        return None
    return eye + ray * (z - eye[2]) / ray[2]


def layout(width, height):
    """Button and key-cap rectangles, bottom right, in framebuffer pixels (origin bottom left)."""
    scale = max(1.0, height / 860)
    w, h, gap, pad = int(400 * scale), int(46 * scale), int(8 * scale), int(14 * scale)
    x = width - w - pad
    rects: dict = dict(move=mujoco.MjrRect(x, pad, w, h), drop=mujoco.MjrRect(x, pad + h + gap, w, h),
                       auto=mujoco.MjrRect(x, pad + 2 * (h + gap), w, h))
    # Key caps: one column per joint, its name above its two keys, in a row above the buttons.
    cap, row = int(56 * scale), int(34 * scale)
    col = 2 * cap + int(4 * scale)
    y_caps = pad + 3 * (h + gap) + gap
    for i, joint in enumerate(JOINTS):
        left = width - pad - (len(JOINTS) - i) * col - (len(JOINTS) - 1 - i) * gap
        rects[joint] = mujoco.MjrRect(left, y_caps + row + int(4 * scale), col, row)
        rects[joint, 1] = mujoco.MjrRect(left, y_caps, cap, row)
        rects[joint, -1] = mujoco.MjrRect(left + col - cap, y_caps, cap, row)
    return rects


def inside(rect, x, y):
    return rect.left <= x < rect.left + rect.width and rect.bottom <= y < rect.bottom + rect.height


def draw_panel(rects, ctx, running, moving, held):
    """running: None, "auto" or "drop", for which button is the one to stop it."""
    font = mujoco.mjtFont.mjFONT_NORMAL
    for joint in JOINTS:
        mujoco.mjr_label(rects[joint], font, joint, 0.1, 0.1, 0.12, 0.55, 1, 1, 1, ctx)
        for sign in (1, -1):
            if (joint, sign) in held:
                mujoco.mjr_label(rects[joint, sign], font, jog_label(joint, sign), 1.0, 0.62, 0.1, 1, 0, 0, 0, ctx)
            else:
                mujoco.mjr_label(rects[joint, sign], font, jog_label(joint, sign), 0.16, 0.16, 0.18, 0.8, 0.8, 0.8, 0.8, ctx)
    for name, key, text in (("auto", AUTO_KEY, "Grab block and pick up"), ("drop", DROP_KEY, "Pick up and drop in box")):
        if running == name:
            mujoco.mjr_label(rects[name], font, f"Stop  ({key_label(key)})", 0.75, 0.2, 0.2, 0.95, 1, 1, 1, ctx)
        else:
            mujoco.mjr_label(rects[name], font, f"{text}  ({key_label(key)})", 0.15, 0.55, 0.3, 0.95, 1, 1, 1, ctx)
    label = f"Move: {moving or 'off'}  ({key_label(MOVE_KEY)} to switch)"
    if moving:
        mujoco.mjr_label(rects["move"], font, label, 0.2, 0.45, 0.8, 0.95, 1, 1, 1, ctx)
    else:
        mujoco.mjr_label(rects["move"], font, label, 0.25, 0.25, 0.28, 0.9, 1, 1, 1, ctx)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pick", action="store_true", help="add a 200 g block on the table")
    args = ap.parse_args()

    m = load_model(args.pick)
    d = mujoco.MjData(m)
    home = m.key("ready").id
    lead = prepare(m)

    mujoco.mj_resetDataKeyframe(m, d, home)
    mujoco.mj_forward(m, d)
    block = Block(m, d) if args.pick else None
    box = Box(m) if args.pick else None
    props = {"block": block, "box": box} if args.pick else {}
    jaw_state = dict(closing=False)
    # moving: None, "block" or "box". drag: (prop, offset from the cursor) while dragging one.
    ui: dict = dict(auto=None, running=None, note="", moving=None, drag=None, eat_drag=False)

    def reset():
        mujoco.mj_resetDataKeyframe(m, d, home)
        for prop in props.values():
            prop.place(m, d, *prop.home)     # back where you last put it
        mujoco.mj_forward(m, d)
        jaw_state["closing"] = False
        stop_auto("Reset.")

    def stop_auto(why):
        if ui["auto"] and ui["auto"].state == "running":
            ui["auto"].stop(why)
            print(ui["auto"].summary(jog_label))

    def toggle_auto(drop=False):
        """Start a pick (and drop in the box, with drop), or stop the one running."""
        if ui["auto"] and ui["auto"].state == "running":
            stop_auto("Stopped.")
            return
        ui["moving"], ui["note"] = None, ""
        try:
            ui["auto"] = AutoPick(m, d, JOG_SPEED, CONTINUOUS, drop=box.drop(d) if drop else None)
            ui["running"] = "drop" if drop else "auto"
        except PlanError as e:
            ui["auto"], ui["note"] = None, str(e)

    def next_move_mode():
        ui["moving"] = {None: "block", "block": "box", "box": None}[ui["moving"]]
        if ui["moving"]:
            stop_auto(f"Stopped to move the {ui['moving']}.")

    if not glfw.init():
        raise RuntimeError("could not start GLFW")
    window = glfw.create_window(1280, 860, "Talos simulation", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    cam, opt, scene = mujoco.MjvCamera(), mujoco.MjvOption(), mujoco.MjvScene(m, maxgeom=4000)
    ctx = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
    cam.lookat[:] = (0.1, 0, 0.17)
    cam.distance, cam.azimuth, cam.elevation = 0.95, 135, -18

    mouse = dict(x=0.0, y=0.0)

    def cursor():
        """Cursor in framebuffer pixels from the bottom left, and the framebuffer size."""
        (w, h), (fw, fh) = glfw.get_window_size(window), glfw.get_framebuffer_size(window)
        return mouse["x"] * fw / max(w, 1), fh - mouse["y"] * fh / max(h, 1), fw, fh

    def cursor_on_table():
        x, y, fw, fh = cursor()
        return table_point(m, cam, x / fw, y / fh, fw / fh, block.half)

    def on_move(_, x, y):
        dx, dy = x - mouse["x"], y - mouse["y"]
        mouse.update(x=x, y=y)
        if ui["drag"] is not None:
            prop, offset = ui["drag"]
            p = cursor_on_table()
            if p is not None:
                prop.place(m, d, *(p[:2] + offset), prop.pose(d)[2])
            return
        left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        if not (left or right) or (left and ui["eat_drag"]):
            return
        action = mujoco.mjtMouse.mjMOUSE_ROTATE_V if left else mujoco.mjtMouse.mjMOUSE_MOVE_V
        height = glfw.get_window_size(window)[1]
        mujoco.mjv_moveCamera(m, action, dx / height, dy / height, scene, cam)

    def on_click(_, button, action, __):
        if button != glfw.MOUSE_BUTTON_LEFT:
            return
        if action == glfw.RELEASE:
            ui["drag"], ui["eat_drag"] = None, False
            return
        if not block:
            return
        x, y, fw, fh = cursor()
        rects = layout(fw, fh)
        buttons = (("auto", toggle_auto), ("drop", lambda: toggle_auto(drop=True)), ("move", next_move_mode))
        for name, act in buttons:
            if inside(rects[name], x, y):
                act()
                ui["eat_drag"] = True     # a click on a button shouldn't also turn the view
                return
        if ui["moving"]:
            # Drag whichever is under the cursor, trying the one being moved first: after a drop the
            # block sits inside the box.
            p = cursor_on_table()
            for prop in ((box, block) if ui["moving"] == "box" else (block, box)):
                if p is not None and prop.under(d, *p[:2]):
                    if prop is block:
                        block.place(m, d, *block.pose(d))   # set it down flat first
                    ui["drag"] = (prop, np.array(prop.pose(d)[:2]) - p[:2])
                    break

    def on_scroll(_, __, dy):
        mujoco.mjv_moveCamera(m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * dy, scene, cam)

    def on_key(_, key, __, action, ___):
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_R:
            reset()
        elif block and key == AUTO_KEY:
            toggle_auto()
        elif block and key == DROP_KEY:
            toggle_auto(drop=True)
        elif block and key == MOVE_KEY:
            next_move_mode()

    glfw.set_cursor_pos_callback(window, on_move)
    glfw.set_mouse_button_callback(window, on_click)
    glfw.set_scroll_callback(window, on_scroll)
    glfw.set_key_callback(window, on_key)

    load = np.zeros(6)
    last = glfw.get_time()
    while not glfw.window_should_close(window):
        now = glfw.get_time()
        frame = min(now - last, 0.05)
        last = now

        if ui["moving"]:
            down = [k for k in (*MOVE_DIRS, *MOVE_TURN) if glfw.get_key(window, k) == glfw.PRESS]
            fwd, side = sum((np.array(MOVE_DIRS[k]) for k in down if k in MOVE_DIRS), np.zeros(2))
            spin = (MOVE_TURN[0] in down) - (MOVE_TURN[1] in down)
            if fwd or side or spin:
                # Arrows move relative to the view: up is away from you, right is to your right.
                az = np.radians(cam.azimuth)
                ahead, right = np.array([np.cos(az), np.sin(az)]), np.array([np.sin(az), -np.cos(az)])
                prop = props[ui["moving"]]
                x, y, yaw = prop.pose(d)
                step = (fwd * ahead + side * right) * MOVE_SPEED * frame
                prop.place(m, d, x + step[0], y + step[1], yaw + spin * MOVE_SPIN * frame)

        pressed = {j: (glfw.get_key(window, a) == glfw.PRESS) - (glfw.get_key(window, b) == glfw.PRESS)
                   for j, (a, b) in KEYMAP.items()}
        auto = ui["auto"]
        if auto and auto.state == "running" and any(pressed.values()):
            stop_auto("Stopped: you pressed a driving key.")
        if auto and auto.state == "running":
            # The autopilot picks its keys every physics step, then they go through step_arm like yours.
            held = set()
            for _ in range(int(round(frame / m.opt.timestep))):
                keys = auto.keys(d)
                held |= {(j, s) for j, s in keys.items() if s}
                step_arm(m, d, keys, m.opt.timestep, lead, jaw_state)
            if auto.state != "running":
                print(auto.summary(jog_label))
        else:
            step_arm(m, d, pressed, frame, lead, jaw_state)
            held = {(j, s) for j, s in pressed.items() if s}
        load += (joint_load(m, d) - load) * min(1.0, frame / 1.0)  # about a 1 s average

        width, height = glfw.get_framebuffer_size(window)
        view = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(view, scene, ctx)
        font = mujoco.mjtFontScale.mjFONTSCALE_150
        mujoco.mjr_overlay(font, mujoco.mjtGridPos.mjGRID_TOPLEFT, view, *help_text(bool(block), ui["moving"]), ctx)
        mujoco.mjr_overlay(font, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, view, *status_text(d, load, props.values()), ctx)
        if block:
            mujoco.mjr_overlay(font, mujoco.mjtGridPos.mjGRID_TOPRIGHT, view, *auto_text(ui["auto"], ui["note"]), ctx)
            running = ui["running"] if ui["auto"] and ui["auto"].state == "running" else None
            draw_panel(layout(width, height), ctx, running, ui["moving"], held)
        glfw.swap_buffers(window)
        glfw.poll_events()
    glfw.terminate()


if __name__ == "__main__":
    main()
