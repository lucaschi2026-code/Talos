"""Build the MuJoCo model of the Talos arm from the CAD in ../cad.

Geometry comes from cad/talos_arm_zero.step, joint dimensions from cad/talos_arm.py
and the mass model from calcs/design_check.py, so re-running this after a CAD change
updates the simulation.

    python sim/build_model.py              # writes sim/model/
    python sim/build_model.py --no-coacd   # single convex hull per link (fast, crude)

Only OCP is needed, not cadquery: on a PC with Smart App Control on, cadquery's
drawing dependency (VTK) can be blocked, but its CAD kernel still loads.
"""

import argparse
import ast
import json
from pathlib import Path

import numpy as np
import trimesh

from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = HERE / "model"
STEP_ZERO = ROOT / "cad" / "talos_arm_zero.step"

# Joint limits in degrees, in the model convention (see cad/talos_arm.py).
# Shoulder, elbow, wrist flex, gripper: CAD collision sweep, calcs/design_check_results.md section 2.
# Pan and roll have no hard stop: software cable limits from cad/README.md (Wiring).
LIMITS_DEG = {"pan": (-110, 110), "shoulder": (-100, 100), "elbow": (-105, 105),
              "wrist_flex": (-105, 95), "wrist_roll": (-150, 150), "gripper": (0, 115)}

# STS3215 values from SO-ARM100 Simulation/SO101/so101_new_calib.xml, which uses the same servo.
# Not measured on Talos. Stall torque is the project's own 2.94 N*m (30 kg*cm at 12 V).
SERVO = dict(damping=0.60, frictionloss=0.052, armature=0.028, kp=998.22, kv=2.731)

# Bodies in kinematic order. Each STEP sub-assembly is a rigid group; the jaw is split out of
# the gripper because its parts sit in gripper_link but turn about joint 6.
BODIES = ("base", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_link", "gripper_link", "moving_jaw")
JAW_PARTS = {"moving_jaw", "moving_jaw_screws", "servo6_jaw_horns"}
ROOT_LINK = "talos_arm"

VISUAL_COLOURS = {"shell": "C_SHELL", "graphite": "C_GRAPHITE", "servo": "C_SERVO",
                  "horn": "C_HORN", "plug": "C_PLUG", "steel": "C_STEEL", "pcb": "C_PCB"}


# ---------------------------------------------------------------- inputs from the CAD and calcs

def read_constants(path):
    """Module-level literals and cq.Color(r, g, b) calls, read without importing the file."""
    out = {}
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            names = [target] if isinstance(target, ast.Name) else getattr(target, "elts", [])
            values = [node.value] if isinstance(target, ast.Name) else getattr(node.value, "elts", [])
            for t, v in zip(names, values):
                if not isinstance(t, ast.Name):
                    continue
                if isinstance(v, ast.Call) and getattr(v.func, "attr", "") == "Color":
                    out[t.id] = tuple(a.value for a in v.args)
                    continue
                try:
                    out[t.id] = ast.literal_eval(v)
                except (ValueError, TypeError, SyntaxError):
                    pass
    return out


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    R = {"x": [[1, 0, 0], [0, c, -s], [0, s, c]],
         "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
         "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    M = np.eye(4)
    M[:3, :3] = R
    return M


def trans(x=0.0, y=0.0, z=0.0):
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def joint_table(cad):
    """One row per joint: child body, parent body, frame in the parent at q = 0 (mm), axis, name.

    Mirrors joint_transforms() in cad/talos_arm.py. The signs are checked against the STEP.
    """
    return [
        ("pan", "shoulder_link", "base", trans(z=cad["H_PAN"]), (0, 0, 1)),
        ("shoulder", "upper_arm_link", "shoulder_link", trans(z=cad["H_SH"]) @ rot("y", -90), (0, 1, 0)),
        ("elbow", "forearm_link", "upper_arm_link", trans(x=cad["L1"]), (0, 1, 0)),
        ("wrist_flex", "wrist_link", "forearm_link", trans(x=cad["L2"]), (0, 1, 0)),
        ("wrist_roll", "gripper_link", "wrist_link", trans(x=cad["X_ROLL"]), (1, 0, 0)),
        ("gripper", "moving_jaw", "gripper_link", trans(x=cad["XJ"], z=cad["ZJ"]), (0, -1, 0)),
    ]


# ---------------------------------------------------------------- STEP reading

def trsf_matrix(loc):
    t = loc.Transformation()
    return np.array([[t.Value(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)] + [[0, 0, 0, 1]])


def read_step(path):
    """Every leaf part as dict(name, link, shape, M), M being its placement in the Z-up model frame.

    The exporter stands the model Y-up by rotating the top instance, so the walk starts
    under that instance. Also returns the frame of each link sub-assembly.
    """
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise RuntimeError(f"could not read {path}")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def label_name(lab):
        n = TDataStd_Name()
        return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else ""

    def components(lab):
        seq = TDF_LabelSequence()
        tool.GetComponents_s(lab, seq)
        return [seq.Value(i) for i in range(1, seq.Length() + 1)]

    parts, frames = [], {}

    def walk(lab, M, link):
        for comp in components(lab):
            ref = TDF_Label()
            tool.GetReferredShape_s(comp, ref)
            placed = M @ trsf_matrix(tool.GetLocation_s(comp))
            if tool.IsAssembly_s(ref):
                sub = label_name(ref)
                frames[sub] = placed
                walk(ref, placed, sub)
            else:
                parts.append(dict(name=label_name(comp), proto=label_name(ref), link=link,
                                  shape=tool.GetShape_s(ref), M=placed))

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    top = components(roots.Value(1))[0]
    ref = TDF_Label()
    tool.GetReferredShape_s(top, ref)
    walk(ref, np.eye(4), ROOT_LINK)
    return parts, frames


def tessellate(shape, linear, angular):
    """(vertices, faces) in the shape's own coordinates, mm."""
    BRepMesh_IncrementalMesh(shape, linear, False, angular, True)
    verts, faces, offset = [], [], 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            T = trsf_matrix(loc)
            v = np.array([[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z(), 1.0]
                          for i in range(1, tri.NbNodes() + 1)])
            verts.append((T @ v.T).T[:, :3])
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                faces.append((a - 1 + offset, c - 1 + offset, b - 1 + offset) if flip
                             else (a - 1 + offset, b - 1 + offset, c - 1 + offset))
            offset += tri.NbNodes()
        explorer.Next()
    return np.vstack(verts), np.array(faces)


def volume_props(shape):
    """Volume (mm^3), area (mm^2), CoM (mm) and inertia at the CoM for unit density (mm^5)."""
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    s = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, s)
    c, I = p.CentreOfMass(), p.MatrixOfInertia()
    return (p.Mass(), s.Mass(), np.array([c.X(), c.Y(), c.Z()]),
            np.array([[I.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]))


# ---------------------------------------------------------------- mass model

def part_mass_g(part, volume, area, calc):
    """Same rules as collect_parts() in calcs/design_check.py."""
    name = part["name"] if part["name"] and not part["name"].isdigit() else part["proto"]
    if name.endswith(("_horns", "_plugs", "_screws")):
        return 0.0                                         # inside the servo + hardware allowance
    if name.startswith("servo"):
        return calc["M_SERVO"] + calc["M_HARDWARE"]
    if name == "bus_servo_board":
        return calc["M_BOARD"]
    shell = min(volume, area * calc["T_SHELL"])
    return calc["RHO_PLA"] * (shell + calc["INFILL"] * (volume - shell))


def visual_class(part):
    name = part["proto"]
    if name.endswith("_screws"):
        return "steel"
    if name.endswith("_horns"):
        return "horn"
    if name.endswith("_plugs"):
        return "plug"
    if name.startswith("servo"):
        return "servo"
    if name == "bus_servo_board":
        return "pcb"
    if name.endswith("hub_caps") or name == "moving_jaw":
        return "graphite"
    return "shell"


def body_of(part):
    if part["name"] in JAW_PARTS:
        return "moving_jaw"
    return "base" if part["link"] == ROOT_LINK else part["link"]


def combine_inertials(items):
    """items: (mass kg, com m, inertia kg*m^2 at own com). Returns the lumped mass, com, inertia."""
    mass = sum(m for m, _, _ in items)
    com = sum(m * c for m, c, _ in items) / mass
    total = np.zeros((3, 3))
    for m, c, I in items:
        d = c - com
        total += I + m * (d @ d * np.eye(3) - np.outer(d, d))
    return mass, com, total


# ---------------------------------------------------------------- collision shapes

def convex_pieces(mesh, use_coacd, threshold, max_hulls):
    if use_coacd:
        import coacd
        coacd.set_log_level("error")
        result = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=threshold,
                                 max_convex_hull=max_hulls, preprocess_resolution=40,
                                 resolution=1000, mcts_iterations=60, seed=1)
        return [trimesh.Trimesh(v, f) for v, f in result]
    return [mesh.convex_hull]


# ---------------------------------------------------------------- MJCF

def fmt(values, digits=6):
    return " ".join(f"{float(v):.{digits}f}".rstrip("0").rstrip(".") if v else "0" for v in values)


def quat_of(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-8:
        return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)])
    raise ValueError("rotation too close to 180 degrees for this helper")


def body_xml(name, table, info, depth):
    pad = "  " * depth
    row = next((r for r in table if r[1] == name), None)
    i = info[name]
    lines = []
    if row:
        joint, _, _, M, axis = row
        attrs = f'name="{name}" pos="{fmt(M[:3, 3] / 1000)}"'
        if not np.allclose(M[:3, :3], np.eye(3)):
            attrs += f' quat="{fmt(quat_of(M[:3, :3]))}"'
        lines.append(f'{pad}<body {attrs}>')
        lo, hi = np.radians(LIMITS_DEG[joint])
        lines.append(f'{pad}  <joint name="{joint}" class="sts3215" axis="{fmt(axis)}" range="{lo:.6f} {hi:.6f}"/>')
    else:
        lines.append(f'{pad}<body name="{name}">')
    I = i["inertia"]
    lines.append(f'{pad}  <inertial pos="{fmt(i["com"])}" mass="{i["mass"]:.6f}" '
                 f'fullinertia="{fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]], 9)}"/>')
    for vis in i["visuals"]:
        lines.append(f'{pad}  <geom class="visual" mesh="{name}__{vis}" material="{vis}"/>')
    hull_class = "finger_hull" if name in ("gripper_link", "moving_jaw") else "collision"
    for k in range(i["n_hulls"]):
        lines.append(f'{pad}  <geom class="{hull_class}" mesh="{name}__hull{k}"/>')
    lines.extend(f"{pad}  {extra}" for extra in i.get("extras", []))
    for child in (r[1] for r in table if r[2] == name):
        lines.extend(body_xml(child, table, info, depth + 1))
    lines.append(f"{pad}</body>")
    return lines


def write_mjcf(path, table, info, cad, calc, block=None):
    q = cad["POSES"]
    joints = [r[0] for r in table]
    ready = np.radians(q["ready"])
    zero = np.zeros(6)
    # the free block is the last body in the file, so its seven coordinates follow the arm's six
    qextra = "" if block is None else f" {block['pos'][0]} {block['pos'][1]} {block['pos'][2]} 1 0 0 0"
    meshes = []
    for name in BODIES:
        meshes += [f'    <mesh name="{name}__{v}" file="visual/{name}__{v}.stl"/>' for v in info[name]["visuals"]]
        meshes += [f'    <mesh name="{name}__hull{k}" file="collision/{name}__hull{k}.stl"/>'
                   for k in range(info[name]["n_hulls"])]
    materials = [f'    <material name="{v}" rgba="{fmt(cad[c])} 1"/>' for v, c in VISUAL_COLOURS.items()]
    actuators = [f'    <position name="{j}" joint="{j}" class="sts3215" ctrlrange="{np.radians(LIMITS_DEG[j][0]):.6f} '
                 f'{np.radians(LIMITS_DEG[j][1]):.6f}"/>' for j in joints]
    block_xml = []
    if block:
        s = block["half"]
        block_xml = [f'    <body name="block" pos="{fmt(block["pos"])}">', "      <freejoint/>",
                     f'      <geom name="block" type="box" size="{s} {s} {s}" mass="{block["mass"]}" '
                     f'rgba="0.85 0.35 0.2 1" friction="{calc["MU_GRIP"]} 0.005 0.0001" condim="4" contype="2" conaffinity="5"/>',
                     "    </body>"]
    text = [
        '<mujoco model="talos">',
        "  <!-- Generated by sim/build_model.py. Edit that script, not this file. -->",
        '  <compiler angle="radian" meshdir="." autolimits="true"/>',
        f'  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10" gravity="0 0 -{calc["G"]}"/>',
        "  <default>",
        '    <default class="sts3215">',
        f'      <joint damping="{SERVO["damping"]}" frictionloss="{SERVO["frictionloss"]}" armature="{SERVO["armature"]}"/>',
        f'      <position kp="{SERVO["kp"]}" kv="{SERVO["kv"]}" forcerange="-{calc["TAU_STALL"]} {calc["TAU_STALL"]}"/>',
        "    </default>",
        '    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="0" mass="0"/></default>',
        f'    <default class="collision"><geom type="mesh" contype="1" conaffinity="1" group="3" mass="0" '
        f'friction="{calc["MU_GRIP"]} 0.005 0.0001" condim="4" rgba="0.3 0.6 0.9 0.4"/></default>',
        # Contact bits: 1 arm hull, 2 table and block, 4 grip pad, 8 finger hull. Finger hulls hit the arm and
        # table but not the block, which grips only on the flat pads: a rough hull pushes it at odd angles.
        f'    <default class="finger_hull"><geom type="mesh" contype="8" conaffinity="1" group="3" mass="0" '
        f'friction="{calc["MU_GRIP"]} 0.005 0.0001" condim="4" rgba="0.3 0.6 0.9 0.4"/></default>',
        '    <default class="pad"><geom type="box" contype="4" conaffinity="0" group="3" mass="0" '
        f'friction="{calc["MU_GRIP"]} 0.005 0.0001" condim="4" rgba="0.9 0.3 0.3 0.5"/></default>',
        "  </default>",
        "  <asset>",
        '    <texture type="skybox" builtin="gradient" rgb1="0.93 0.95 0.97" rgb2="0.62 0.67 0.73" width="512" height="3072"/>',
        '    <texture name="grid" type="2d" builtin="checker" rgb1="0.78 0.8 0.82" rgb2="0.7 0.72 0.75" width="256" height="256"/>',
        '    <material name="table" texture="grid" texrepeat="10 10" reflectance="0.05"/>',
        *materials,
        *meshes,
        "  </asset>",
        "  <visual>",
        '    <global offwidth="1600" offheight="1200" azimuth="135" elevation="-20"/>',
        '    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>',
        '    <rgba haze="0.9 0.92 0.95 1"/>',
        "  </visual>",
        "  <worldbody>",
        '    <light pos="0.4 -0.4 1.2" dir="-0.3 0.3 -1" diffuse="0.5 0.5 0.5"/>',
        f'    <geom name="table" type="plane" size="1 1 0.05" material="table" contype="2" conaffinity="11" '
        f'friction="{calc["MU_GRIP"]} 0.005 0.0001" condim="4"/>',
        *body_xml("base", table, info, 2),
        *block_xml,
        "  </worldbody>",
        "  <contact>",
        *[f'    <exclude body1="{a}" body2="{b}"/>' for a, b in info["_exclude"]],
        "  </contact>",
        "  <actuator>",
        *actuators,
        "  </actuator>",
        "  <keyframe>",
        f'    <key name="zero" qpos="{fmt(zero)}{qextra}" ctrl="{fmt(zero)}"/>',
        f'    <key name="ready" qpos="{fmt(ready)}{qextra}" ctrl="{fmt(ready)}"/>',
        "  </keyframe>",
        "</mujoco>",
    ]
    path.write_text("\n".join(text) + "\n", encoding="utf-8")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-coacd", action="store_true", help="one convex hull per link instead of a decomposition")
    args = ap.parse_args()

    cad = read_constants(ROOT / "cad" / "talos_arm.py")
    calc = read_constants(ROOT / "calcs" / "design_check.py")
    table = joint_table(cad)

    print(f"reading {STEP_ZERO.name}")
    parts, step_frames = read_step(STEP_ZERO)

    # Body frames at q = 0 from the joint dimensions, checked against the STEP's sub-assembly placements.
    frames = {"base": np.eye(4)}
    for _, child, parent, M, _ in table:
        frames[child] = frames[parent] @ M
    for link, M in step_frames.items():
        if not np.allclose(M, frames[link], atol=1e-6):
            raise RuntimeError(f"{link}: talos_arm.py dimensions do not match the STEP frame")
    print(f"  {len(parts)} parts; {len(step_frames)} link frames match talos_arm.py")

    for folder in (OUT / "visual", OUT / "collision"):
        folder.mkdir(parents=True, exist_ok=True)
        for old in folder.glob("*.stl"):  # files only: OneDrive can hold a lock on the folders
            old.unlink()

    info, report = {}, {}
    for body in BODIES:
        inv = np.linalg.inv(frames[body])
        items, by_visual, solid_meshes, part_rows = [], {}, [], []
        for part in (p for p in parts if body_of(p) == body):
            M = inv @ part["M"]                      # shape coordinates -> body frame (mm)
            R, t = M[:3, :3], M[:3, 3]
            V, A, c, I_unit = volume_props(part["shape"])
            m_g = part_mass_g(part, V, A, calc)
            part_rows.append(dict(name=part["name"], proto=part["proto"], mass_g=m_g))
            if m_g > 0:
                items.append((m_g / 1000.0, (R @ c + t) / 1000.0, R @ I_unit @ R.T * (m_g / 1000.0) / V * 1e-6))
            v, f = tessellate(part["shape"], linear=0.1, angular=0.35)
            v = (R @ v.T).T + t
            by_visual.setdefault(visual_class(part), []).append(trimesh.Trimesh(v / 1000.0, f, process=False))
            if not part["proto"].endswith("_screws"):  # screws add nothing to contact shape
                solid_meshes.append(trimesh.Trimesh(v / 1000.0, f))
        mass, com, inertia = combine_inertials(items)
        for vis, meshes in by_visual.items():
            trimesh.util.concatenate(meshes).export(OUT / "visual" / f"{body}__{vis}.stl")

        merged = trimesh.util.concatenate(solid_meshes)
        merged.merge_vertices()
        hulls = convex_pieces(merged, not args.no_coacd, threshold=0.06, max_hulls=14)
        for k, h in enumerate(hulls):
            h.export(OUT / "collision" / f"{body}__hull{k}.stl")
        info[body] = dict(mass=mass, com=com, inertia=inertia, visuals=sorted(by_visual), n_hulls=len(hulls))
        report[body] = dict(mass_g=mass * 1000, printed_g=sum(r["mass_g"] for r in part_rows
                                                              if not r["proto"].startswith("servo")
                                                              and r["proto"] != "bus_servo_board"),
                            com_mm=(com * 1000).tolist(), hulls=len(hulls))
        print(f"  {body:<16} {mass * 1000:6.1f} g   {len(hulls):2d} convex pieces")

    # Flat grip pads, from the finger profiles in gripper_parts() and moving_jaw(), for clean contact.
    hw = cad["FINGER_HW"] / 1000.0
    info["gripper_link"]["extras"] = [
        f'<geom name="pad_fixed" class="pad" pos="0.087 0 -0.003" size="0.021 {hw} 0.0025"/>',
        '<site name="tool" pos="{} {} {}" size="0.004" rgba="1 0.8 0 1"/>'.format(*(np.array(calc["GRIP_POINT"]) / 1000)),
        '<site name="tip" pos="{} {} {}" size="0.003" rgba="1 0.4 0 1"/>'.format(*(np.array(calc["TIP_POINT"]) / 1000)),
    ]
    info["moving_jaw"]["extras"] = [f'<geom name="pad_moving" class="pad" pos="0.047 0 -0.003" size="0.017 {hw} 0.0025"/>']

    # MuJoCo ignores contact between a moving parent and its child, so a joint's two links can't jam.
    # The base doesn't count as a parent because it has no joint, so joint 1 needs this by hand: their
    # convex shapes overlap, while the real turntable rides 2.8 mm clear of the column.
    info["_exclude"] = [("base", "shoulder_link")]

    write_mjcf(OUT / "talos.xml", table, info, cad, calc)
    write_mjcf(OUT / "talos_pick.xml", table, info, cad, calc,
               block=dict(half=0.0125, mass=calc["PAYLOAD"], pos=[0.0, 0.19, 0.0125]))  # clear of the ready pose, and square to the gripper at 90 deg of pan
    (OUT / "build_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"wrote {OUT / 'talos.xml'}")


if __name__ == "__main__":
    main()
