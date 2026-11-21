"""SO-101 straight-arm statics from its URDF, for comparison with Talos.

Same method as design_check.py: arm held straight out (worst case), gravity
moments about each joint, 200 g payload at the tool point.

Get the URDF from TheRobotStudio/SO-ARM100 (Simulation/SO101/so101_new_calib.urdf):

    python so101_compare.py path/to/so101_new_calib.urdf
"""

import itertools
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81
PAYLOAD = 0.200  # kg, at the URDF tool point (gripper_frame_link)
PITCH = ["shoulder_lift", "elbow_flex", "wrist_flex"]
CHAIN = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link",
         "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]


def vec(s):
    return np.array([float(v) for v in s.split()])


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rot(a, q):
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def load(path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        i = link.find("inertial")
        if i is not None and link.get("name") in CHAIN:
            links[link.get("name")] = (float(i.find("mass").get("value")),
                                       vec(i.find("origin").get("xyz")))
    joints = []
    for j in root.findall("joint"):
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        joints.append(dict(
            name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=vec(o.get("xyz")), rpy=vec(o.get("rpy")),
            axis=vec(a.get("xyz")) if a is not None else np.zeros(3),
            lim=(float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None))
    return links, joints


def fk(joints, q):
    frames, axes, todo = {"base_link": (np.eye(3), np.zeros(3))}, {}, list(joints)
    while todo:
        for j in list(todo):
            if j["parent"] not in frames:
                continue
            R, p = frames[j["parent"]]
            Rj, pj = R @ rpy(*j["rpy"]), p + R @ j["xyz"]
            axes[j["name"]] = (Rj @ j["axis"], pj)
            if j["type"] == "revolute":
                Rj = Rj @ axis_rot(j["axis"], q.get(j["name"], 0.0))
            frames[j["child"]] = (Rj, pj)
            todo.remove(j)
    return frames, axes


def straight_out(joints, sign, n=49):
    """Pitch angles that push the tool point furthest along sign * X."""
    lims = {j["name"]: j["lim"] for j in joints if j["lim"]}
    best = None
    for qs in itertools.product(*[np.linspace(*lims[k], n) for k in PITCH]):
        q = dict(zip(PITCH, qs))
        frames, axes = fk(joints, q)
        d = sign * (frames["gripper_frame_link"][1][0] - axes["shoulder_pan"][1][0])
        if best is None or d > best[0]:
            best = (d, q)
    return best[1]


def report(links, joints, sign, label):
    q = straight_out(joints, sign)
    frames, axes = fk(joints, q)
    pan = axes["shoulder_pan"][1]
    tcp = frames["gripper_frame_link"][1]
    pts = {n: frames[n][0] @ c + frames[n][1] for n, (m, c) in links.items()}

    def torque(joint, first, payload):
        a, o = axes[joint]
        M = sum(np.cross(pts[n] - o, [0, 0, -G * links[n][0]])
                for n in links if CHAIN.index(n) >= first)
        M = M + np.cross(tcp - o, [0, 0, -G * payload])
        return math.hypot(M[0], M[1]) if joint == "shoulder_pan" else abs(M @ a)

    print(f"\n{label}")
    print(f"  pose (deg): " + ", ".join(f"{k} {math.degrees(v):.0f}" for k, v in q.items()))
    print(f"  tool point {1000*sign*(tcp - pan)[0]:.0f} mm from the pan axis, "
          f"{1000*np.linalg.norm((tcp - axes['shoulder_lift'][1])[[0, 2]]):.0f} mm from the shoulder axis")
    print("  | Joint | No payload (N*m) | 200 g (N*m) |")
    for joint, first in (("shoulder_pan", 1), ("shoulder_lift", 2), ("elbow_flex", 3), ("wrist_flex", 4)):
        print(f"  | {joint} | {torque(joint, first, 0):.2f} | {torque(joint, first, PAYLOAD):.2f} |")
    xs = ", ".join(f"{sign*(pts[n] - pan)[0]:.4f}" for n in CHAIN[1:])
    print(f"  lever arms from the pan axis (m), shoulder link .. jaw: [{xs}], tool {sign*(tcp - pan)[0]:.4f}")


def main():
    links, joints = load(sys.argv[1])
    frames, axes = fk(joints, {})
    off = 1000 * (axes["shoulder_lift"][1] - axes["shoulder_pan"][1])
    print(f"Moving mass: {1000*sum(m for n, (m, _) in links.items() if n != 'base_link'):.0f} g "
          f"(base {1000*links['base_link'][0]:.0f} g)")
    print(f"Shoulder axis: {off[2] + 1000*axes['shoulder_pan'][1][2]:.0f} mm above the base frame, "
          f"{off[0]:.1f} mm forward of the pan axis ({off[1]:.1f} mm along its own axis)")
    lims = {j["name"]: j["lim"] for j in joints if j["lim"]}
    print("Joint limits (deg): " + ", ".join(
        f"{k} {math.degrees(a):.0f}/{math.degrees(b):.0f}" for k, (a, b) in lims.items()))
    report(links, joints, 1, "Reaching forward (over the shoulder offset)")
    report(links, joints, -1, "Reaching backward (shoulder offset behind the reach)")


if __name__ == "__main__":
    main()
