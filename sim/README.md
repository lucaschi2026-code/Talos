# Talos simulation

A MuJoCo model of the Talos arm, built from the CAD in [`../cad`](../cad). Use it to drive the arm from the keyboard, check what it can reach and lift, and find problems before printing.

![Pick sequence](renders/pick.png)

## Run it

On Windows, double-click `sim\run.bat`, or run it from a terminal:

```bat
sim\run.bat                       :: drive the arm
sim\run.bat drive.py --pick       :: with a 200 g block to pick up
sim\run.bat check_model.py --fast
sim\run.bat build_model.py
```

The first run sets up Python 3.12 and the packages in `%USERPROFILE%\.venvs\talos-sim`, which needs [uv](https://docs.astral.sh/uv/) (`winget install astral-sh.uv`). Set `TALOS_SIM_VENV` to use another folder. It runs the base Python with the environment's packages, because Smart App Control can block the environment's own `python.exe`.

On macOS or Linux, set up once, with Python 3.12:

```bash
uv venv --python 3.12 ~/.venvs/talos-sim
uv pip install --python ~/.venvs/talos-sim -r sim/requirements.txt
```

Keep the environment outside OneDrive, because it holds thousands of files. Then, from the repo root, with that environment's Python:

```bash
python sim/drive.py              # drive the arm
python sim/drive.py --pick       # with a 200 g block to pick up
python sim/check_model.py        # re-run every check and save the results (several minutes)
python sim/check_model.py --fast # quicker version, prints only
python sim/build_model.py        # rebuild the model after a CAD change
```

## Driving

Hold a key to move a joint. Left drag orbits the view, right drag pans and scroll zooms. Change the keys in `KEYMAP` at the top of `drive.py`.

| Joint | + | − |
| --- | --- | --- |
| Pan | A | D |
| Shoulder (+ is forward and down) | W | S |
| Elbow (+ is down) | I | K |
| Wrist flex | U | J |
| Wrist roll | Q | E |
| Gripper (+ opens) | O | L |
| Back to the ready pose | R | |

With `--pick`, the block sits to the arm's left, square to the gripper at 90° of pan. Hold A for about 2 s to face it.

To pick it up, open the jaw (O), line the fixed finger up just inside the block, and lower until the tip is about 5–10 mm above the table (`z` in the readout). Hold L until the jaw is squeezing, and it keeps squeezing after you let go. Press O to release. Don't press down into the table first: the jaw then grips at a steeper angle, and the block can squeeze out when you lift.

The readout shows each joint's angle and its motor torque averaged over about 1 s:
- **`above 1/3 stall`:** more than the sustained-load rule of thumb used in [`../calcs`](../calcs). It's a warning about heat if held for a long time, and nothing stops the arm.
- **`AT STALL`:** the servo has no torque left, usually because the arm is pressing on the table or the block.

## Where the numbers come from

| Input | Source |
| --- | --- |
| Shapes | `cad/talos_arm_zero.step` |
| Joint positions | `cad/talos_arm.py`, checked against the STEP on every build |
| Masses | The PLA model in `calcs/design_check.py`: 1.24 g/cm³, 1.2 mm perimeters, 20 % infill, 61 g per servo |
| Shoulder, elbow, wrist flex and gripper limits | CAD collision sweep in `calcs/design_check_results.md` |
| Pan and wrist roll limits | Cable limits in `cad/README.md` (±110°, ±150°) |
| Stall torque | 2.94 N·m, as in `calcs/` |
| Servo gains, damping, friction, rotor inertia | The SO-101 MuJoCo model, which uses the same servo. **Not measured on Talos** |
| Grip friction | 0.3, as in the design check |

Generated files live in `sim/model/`. Don't edit them by hand, because the next build replaces them. Edit `build_model.py` instead.

## Results

[`check_results.md`](check_results.md) has the full output. The model matches the CAD and calcs: the ready pose is within 0.001 mm of `talos_arm_assembly.step`, and the moving mass, holding torques and reach all match `design_check_results.md`.

What the simulation adds:

- **Speed costs torque.** With 200 g, lifting the shoulder 90° in 1.0 s stalls the servo for about 0.65 s and it falls about 9° behind. At 1.5 s it keeps up, at 94 % of stall. The design check's 72 % matches the same move going downhill, where gravity helps. This depends on the SO-101 damping value, so measure it on the real arm.
- **Block size is limited by the pivoting jaw.** Wider blocks sit in a steeper wedge and squeeze out. At friction 0.3 the jaw holds a 200 g cube up to 25 mm and drops it from 30 mm. At friction 0.5 it holds up to 40 mm. Grip pads or a rubber coating would help most. The model uses flat pads, so real serrations may do better.
- **Single-joint limits don't cover every pose.** About 1 % of poses inside the limits touch the arm itself and 19 % hit the table, so control software should check a pose before moving to it.

## Simplifications

- **The base is fixed to the table,** as if clamped. `calcs/` shows it tips over otherwise.
- **Backlash, printed-part flex and servo heating aren't modelled.** Those need measuring on the real arm.
- **Sag is an estimate.** It comes from the servo gains, and the 2.1 mm of link bending in the design check isn't included.
- **Collision shapes are simplified.** Each link is a set of convex pieces, which err on the side of touching. The block only touches the flat grip pads on the gripper, so it can pass through the rest of the gripper housing.

## Smart App Control

`mujoco` is pinned to 3.8.0 because Windows Smart App Control, which is on for this PC, blocks a plugin DLL in the newest release. It also blocks part of VTK, which stops `cadquery` importing here, so `cad/talos_arm.py` and `calcs/design_check.py` won't run on this machine as they are. `build_model.py` avoids `cadquery` by reading the STEP with OCP, the CAD kernel underneath it. It needs the STEP files to be up to date, so re-export them on a machine that can run `talos_arm.py` after a CAD change.
