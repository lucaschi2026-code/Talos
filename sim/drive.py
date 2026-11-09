"""Drive the simulated Talos arm from the keyboard.

    python sim/drive.py            # the arm on a table
    python sim/drive.py --pick     # with a 200 g block to pick up

Hold a key to move a joint. Left drag orbits the view, right drag pans, scroll zooms.
"""

import argparse
from pathlib import Path

import glfw
import mujoco
import numpy as np

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


def status_text(d, load):
    """Two overlay columns. load is smoothed: the 1/3 rule is about sustained load, not brief peaks."""
    values = [f"{np.degrees(d.qpos[i]):.1f} deg   {load[i]:.2f} N*m{load_flag(i, load[i])}" for i in range(len(JOINTS))]
    tip = d.site("tip").xpos * 1000
    return "\n".join(JOINTS + ["tip (mm)"]), "\n".join(values + [f"x {tip[0]:.0f}   y {tip[1]:.0f}   z {tip[2]:.0f}"])


def help_text():
    keys = [f"{glfw.get_key_name(a, 0).upper()} / {glfw.get_key_name(b, 0).upper()}" for a, b in KEYMAP.values()]
    return "\n".join(JOINTS + ["reset"]), "\n".join(keys + ["R"])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pick", action="store_true", help="add a 200 g block on the table")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(str(HERE / "model" / ("talos_pick.xml" if args.pick else "talos.xml")))
    d = mujoco.MjData(m)
    home = m.key("ready").id

    jaw_state = dict(closing=False)

    def reset():
        mujoco.mj_resetDataKeyframe(m, d, home)
        mujoco.mj_forward(m, d)
        jaw_state["closing"] = False

    reset()

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

    def on_move(_, x, y):
        dx, dy = x - mouse["x"], y - mouse["y"]
        mouse.update(x=x, y=y)
        left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        if not (left or right):
            return
        action = mujoco.mjtMouse.mjMOUSE_ROTATE_V if left else mujoco.mjtMouse.mjMOUSE_MOVE_V
        height = glfw.get_window_size(window)[1]
        mujoco.mjv_moveCamera(m, action, dx / height, dy / height, scene, cam)

    def on_scroll(_, __, dy):
        mujoco.mjv_moveCamera(m, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * dy, scene, cam)

    def on_key(_, key, __, action, ___):
        if action == glfw.PRESS and key == glfw.KEY_R:
            reset()

    glfw.set_cursor_pos_callback(window, on_move)
    glfw.set_scroll_callback(window, on_scroll)
    glfw.set_key_callback(window, on_key)

    lead = prepare(m)
    load = np.zeros(6)
    last = glfw.get_time()
    while not glfw.window_should_close(window):
        now = glfw.get_time()
        frame = min(now - last, 0.05)
        last = now

        pressed = {j: (glfw.get_key(window, a) == glfw.PRESS) - (glfw.get_key(window, b) == glfw.PRESS)
                   for j, (a, b) in KEYMAP.items()}
        step_arm(m, d, pressed, frame, lead, jaw_state)
        load += (joint_load(m, d) - load) * min(1.0, frame / 1.0)  # about a 1 s average

        width, height = glfw.get_framebuffer_size(window)
        view = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(view, scene, ctx)
        mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, view,
                           *help_text(), ctx)
        mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, view,
                           *status_text(d, load), ctx)
        glfw.swap_buffers(window)
        glfw.poll_events()
    glfw.terminate()


if __name__ == "__main__":
    main()
