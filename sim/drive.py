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
LOAD_WARNING = 2.94 / 3        # N*m: the continuous limit used in calcs/


def jog(ctrl, pressed, dt, low, high):
    """Move each joint's target by the keys held this frame, keeping it inside the joint limits.

    pressed maps a joint name to +1, -1 or 0. Targets move, not torques: like the real
    servo, the arm then chases the target as far as its torque allows.
    """
    out = ctrl.copy()
    for i, joint in enumerate(JOINTS):
        out[i] = np.clip(out[i] + pressed.get(joint, 0) * JOG_SPEED * dt, low[i], high[i])
    return out


def status_text(d, load):
    """Two overlay columns. load is the smoothed torque: the limit is about sustained load, not brief peaks."""
    values = [f"{np.degrees(d.qpos[i]):.1f} deg   {load[i]:.2f} N*m{'   OVER LIMIT' if load[i] > LOAD_WARNING else ''}"
              for i in range(len(JOINTS))]
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

    def reset():
        mujoco.mj_resetDataKeyframe(m, d, home)
        mujoco.mj_forward(m, d)

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

    low, high = m.actuator_ctrlrange[:6, 0], m.actuator_ctrlrange[:6, 1]
    jaw = JOINTS.index("gripper")
    m.actuator_forcerange[jaw] = (-LOAD_WARNING, LOAD_WARNING)  # squeeze at the continuous limit, don't crush
    jaw_windup = LOAD_WARNING / m.actuator_gainprm[jaw, 0]
    load = np.zeros(6)
    last = glfw.get_time()
    while not glfw.window_should_close(window):
        now = glfw.get_time()
        frame = min(now - last, 0.05)
        last = now

        pressed = {j: (glfw.get_key(window, a) == glfw.PRESS) - (glfw.get_key(window, b) == glfw.PRESS)
                   for j, (a, b) in KEYMAP.items()}
        d.ctrl[:6] = jog(d.ctrl[:6], pressed, frame, low, high)
        # Holding "close" on a block must not wind the target far past it, or opening would lag.
        d.ctrl[jaw] = max(d.ctrl[jaw], d.qpos[jaw] - jaw_windup)
        for _ in range(int(frame / m.opt.timestep)):
            mujoco.mj_step(m, d)
        load += (np.abs(d.actuator_force[:6]) - load) * min(1.0, frame / 1.0)  # about a 1 s average

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
