"""Export what the MATLAB design check needs from the CAD into cad_inputs.json.

design_check.m can't build the CadQuery model, so this script does the CAD part once: every part's
mass, centre of mass and inertia, the beam cross-sections, the joint-range collision sweep, and the
dimensions the screw and mount checks use. Run it on the cad branch after any change to
cad/talos_arm.py, then run design_check.m.

    python calcs/cad_export/export_cad_inputs.py    # writes cad_inputs.json; 5-15 minutes, mostly the collision sweep
"""

import json
import re
import subprocess
from pathlib import Path

import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))
import design_check as dc  # noqa: E402
from design_check import ta  # noqa: E402

HERE = Path(__file__).resolve().parent
L2_VARIANTS = (115.0, 120.0, 125.0)   # forearm lengths for the sensitivity check (design L2 is added)


def rnd(a, digits=6):
    """Round for a readable, diff-friendly file. 6 decimals is far below what any result prints."""
    return (np.round(np.asarray(a, dtype=float), digits) + 0.0).tolist()   # + 0.0 turns -0.0 into 0.0


def dump(data):
    """Indented JSON with each list of numbers on one line."""
    text = json.dumps(data, indent=1)
    return re.sub(r"\[\s+([-+0-9.eE,\s]+?)\s+\]", lambda m: "[" + " ".join(m.group(1).split()) + "]", text)


def part_rows(parts):
    return [dict(name=p["name"], link=p["link"], m=rnd(p["m"], 7), com=rnd(p["com"], 4), I=rnd(p["I"], 4))
            for p in parts]


def beam(parts, name, L):
    shape = next(p["shape"] for p in parts if p["name"] == name)
    rows = [(x, dc.section(shape, x)) for x in np.arange(4.0, L - 2.0, 2.0)]
    rows = [(x, s) for x, s in rows if s is not None]
    return dict(L=L, x=rnd([x for x, _ in rows], 3), I_eff=rnd([s["I_eff"] for _, s in rows], 3),
                c=rnd([s["c"] for _, s in rows], 4))


def main():
    parts = dc.collect_parts()
    grip = next(p["shape"] for p in parts if p["name"] == "gripper_body")
    finger = dc.section(grip, 60.0)
    shoulder = next(p["shape"] for p in parts if p["name"] == "shoulder")
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True,
                            cwd=HERE).stdout.strip()

    data = dict(
        source=dict(script="calcs/cad_export/export_cad_inputs.py", cad="cad/talos_arm.py", commit=commit),
        units="lengths mm, masses kg, inertias kg*mm^2 about each part's centre of mass, in its link frame",
        links=list(ta.LINKS),
        mass_model=dict(RHO_PLA=dc.RHO_PLA, T_SHELL=dc.T_SHELL, INFILL=dc.INFILL, M_SERVO=dc.M_SERVO,
                        M_HARDWARE=dc.M_HARDWARE, M_BOARD=dc.M_BOARD),
        geometry=dict(
            # kinematics (joint_transforms in talos_arm.py)
            H_PAN=ta.H_PAN, H_SH=ta.H_SH, L1=ta.L1, L2=ta.L2, X_ROLL=ta.X_ROLL, XJ=ta.XJ, ZJ=ta.ZJ,
            # clevis plates and horn screws
            CLV_IN=ta.CLV_IN, CLV_OUT=ta.CLV_OUT, HORN_FACE=ta.HORN_FACE,
            n_idler_tab_screws=len(ta.IDLER_TAB_SCREWS),
            # gripper
            FINGER_HW=ta.FINGER_HW,
            # servo mounts
            FIT_C=ta.FIT_C, CRADLE_TOP=ta.CRADLE_TOP, POST_IN=ta.POST_IN, CASE_HW=ta.CASE_HW,
            SHELL_OUT=ta.SHELL_OUT, SHELL_IN=ta.SHELL_IN, TOP_FACE=ta.TOP_FACE, FACE_X=ta.FACE_X,
            CASE_X0=ta.CASE_X[0], CASE_FRONT_X=ta.CASE_FRONT[0][0], CASE_BACK_X=ta.CASE_BACK[0][0],
            CASE_BACK_Y=ta.CASE_BACK[0][1], CASE_BACK_SHAFT_X=ta.CASE_BACK_SHAFT[0][0],
            M2_HEAD_R=ta.SCREW[2]["head_r"], M2_CLEAR=ta.SCREW[2]["clear"],
            shoulder_top_z=rnd(shoulder.BoundingBox().zmax, 4),
        ),
        parts=part_rows(parts),
        beams=dict(upper_arm_link=beam(parts, "upper_arm", ta.L1), forearm_link=beam(parts, "forearm", ta.L2)),
        finger_root=dict(x=60.0, I_eff=rnd(finger["I_eff"], 3), c=rnd(finger["c"], 4)),
        joint_ranges={k: list(v) for k, v in dc.joint_ranges(parts).items()},
    )

    # forearm length sensitivity: the CAD rebuilt with each L2, everything else unchanged
    variants, L2_design = [], ta.L2
    for L2 in L2_VARIANTS:
        ta.L2 = L2
        variants.append(dict(L2=L2, parts=part_rows(dc.collect_parts())))
    ta.L2 = L2_design
    data["forearm_variants"] = variants

    (HERE / "cad_inputs.json").write_text(dump(data) + "\n", encoding="utf-8", newline="\n")
    print(f"cad_inputs.json: {len(parts)} parts, {len(variants)} forearm variants, commit {commit}")


if __name__ == "__main__":
    main()
