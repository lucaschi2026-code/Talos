# Talos arm CAD

Parametric CadQuery model of the Talos 5-axis arm plus gripper, exported as a full STEP assembly.

![ready pose](renders/ready/iso.png)

## Files

| File | What it is |
| --- | --- |
| `talos_arm_assembly.step` | Full assembly in the "ready" display pose |
| `talos_arm_zero.step` | Same assembly with every joint at 0 (arm straight up) |
| `talos_arm.py` | Generator. All dimensions and poses live at the top |
| `render.py` | Renders PNG views into `renders/` and checks every part pair for interference |
| `drawings/Talos_drawings.pdf` | Technical drawing set, 11 A3 sheets: everything needed to rebuild each part by hand |
| `drawings.py` | Generates the drawing set from `talos_arm.py`, so the drawings always match the model |

## Drawings

[`drawings/Talos_drawings.pdf`](drawings/Talos_drawings.pdf) has one sheet per printed part, plus three shared sheets. There's also a PNG of each sheet in the same folder.

| Sheet | Contents |
| --- | --- |
| 1 TAL-A00 | General assembly (zero pose), joint layout heights, parts list and hardware |
| 2 REF-01 | STS3215 servo reference and the **servo pocket P** that every part cuts |
| 3 C-01 | Common features: clevis plates, horn and case screw holes, cable clip, hex window |
| 4–11 | TAL-01 base, TAL-02 shoulder, TAL-03 upper arm, TAL-04 forearm, TAL-05 wrist, TAL-06 gripper body, TAL-07 moving jaw, TAL-08/09 hub caps |

Each part sheet has:
- front, top and right views in third-angle projection, with hidden lines dashed
- an isometric view
- the key dimensions
- a numbered **feature list**: the modelling steps in order, with every coordinate needed to rebuild the part in SolidWorks

Coordinates are in the part's own frame, the frame `talos_arm.py` builds it in, and each sheet's title block says where that frame sits. For the arm links, the origin is on the proximal joint axis, X runs to the distal axis, and Y is the joint axis. Rebuild each part in that frame and the parts mate on the same axes as the STEP. The STEP itself is a good check: import it alongside your model and compare.

Regenerate after any change to the model:

```bash
python drawings.py
```

The STEP keeps the kinematic tree as nested sub-assemblies (`shoulder_link` → `upper_arm_link` → `forearm_link` → `wrist_link` → `gripper_link`), so each link can be suppressed or re-mated in SolidWorks. Parts carry colours:
- off-white for printed structure
- graphite grey for hub caps and the moving jaw
- black for the servo bodies and aluminium for their horns
- cream for the bus plugs
- steel for the screws
- dark green-grey for the bus board

Each servo's horns sit in the next link's sub-assembly, because that's the part they're bolted to and turn with.

The STEP files are exported **Y-up**, the SolidWorks convention, so the arm stands upright in the isometric view, with the base on the Front-Top ground plane. The arm reaches along +X. Internally the model, calcs and renders use Z-up; `python talos_arm.py --z-up` exports in that frame instead.

## Layout

Servo placement follows the SO-ARM100 / SO-101 pattern: six Feetech STS3215 servos, each held in a printed shell. The next link clamps the servo's output horn on one side and its idler on the other.

| Joint | Servo | Mounting |
| --- | --- | --- |
| 1 Base pan | vertical, in the base column | turntable bolts to the horn |
| 2 Shoulder | horizontal, hanging in the shoulder cradle | upper-arm clevis: output horn (+Y) and passive horn (−Y) |
| 3 Elbow | in the upper arm's end shell | forearm clevis, same pattern |
| 4 Wrist flex | in the forearm's end shell | wrist clevis, same pattern |
| 5 Wrist roll | in the wrist cradle, shaft along the arm | gripper bolts to the horn |
| 6 Jaw | in the gripper housing | moving jaw on the horn |

Link lengths are roughly SO-101 size: 116 mm shoulder to elbow, 135 mm elbow to wrist, 63 mm wrist to roll flange and 108 mm roll flange to jaw tip. That gives about 420 mm of reach from the shoulder axis. The pan horn is 62 mm above the table and the shoulder axis is 108 mm above it.

## Wiring

The six servos are daisy-chained on one bus from the board in the base. Each STS3215 has two 3-pin connectors on its back face, opposite the shaft, just past mid-length. Every servo pocket includes a plug bay: the full connector footprint plus 12 mm for the mated plugs and the cable bend. The plugs are modelled (white), and the interference and joint-range checks include them.

| Hop | Route |
| --- | --- |
| Board → pan servo | The board sits on a deck behind the column, on four M3 standoffs (37 × 28 mm pattern, same Waveshare bus servo adapter as SO-101). A 12 × 9.7 mm tunnel runs from the deck into the pan servo's plug bay under the servo. |
| Pan → shoulder | Back out through the same tunnel, then a service loop up the back of the column to a clip on the shoulder cradle's back face, and on to the shoulder servo's plugs on the cradle's −Y side. The loop sets the pan range: keep it to about ±110° in software, like SO-101. |
| Shoulder → elbow | Across the shoulder joint on the −Y side, outside the slim idler plate, through the clip on the upper arm's waist, and into the plug window in the elbow shell. |
| Elbow → wrist flex | Same pattern along the forearm. |
| Wrist flex → wrist roll | Across the wrist joint on the −Y side, into the roll servo's plug bay behind the servo. It opens under the −Y lug so the cable can enter from the side. |
| Wrist roll → jaw | Through the clip under the wrist cradle's side window, then a loop around the roll axis to the plug window in the gripper housing's −Y side. Keep roll to about ±150° in software. |

**Why the idler plates are slim:** the plugs sit only 11 mm from each pitch joint's axis, on the passive-horn side. So the −Y plate of each clevis is an R10.5 hub (just over the Ø20 passive horn) with a neck pinched to 17 mm, which keeps it clear of the plugs through the whole joint range. The +Y plate on the output horn keeps the full R16 hub. The wrist roll servo also sits 3 mm further forward than in the first version, so its plugs clear the end of the forearm shell.

## Screws

All holes and counterbores are in the parts, and the screws are modelled in the assembly. Horn and passive-horn screws are M3 into the horns' 2.5 mm tapped holes. Servo case screws are M2 self-tapping into the case's 1.5 mm holes. Counts are for one arm:

| Where | Screw | Qty | Notes |
| --- | --- | --- | --- |
| Output horns: turntable, upper arm, forearm, wrist, gripper, moving jaw | M3 × 6 button head | 24 | 4 per horn on the 9.8 mm square pattern, plus a Ø7 centre hole for the horn's own spline screw. The turntable's go in from inside the cradle and the gripper's from inside the housing, both counterbored. |
| Passive horns: upper arm, forearm, wrist | M3 × 8 button head | 3 | One centre screw into the idler boss |
| Servo cases: elbow, wrist flex, jaw | M2 × 5 self-tapping | 12 | 2 per face, counterbored 1.1 mm into the shell walls |
| Servo case: shoulder | M2 × 6 self-tapping | 4 | Through counterbores in the cradle cheeks |
| Servo case: pan | M2 × 6 self-tapping | 2 | From under the base, through Ø4.2 tunnels |
| Servo case: wrist roll | M2 × 6 self-tapping | 2 | Through the cradle back wall; fit before the gripper |
| Bus board | M2.5 × 6 self-tapping | 4 | Through the board's Ø2.5 holes into Ø2.1 pilots in the standoffs |
| Base to table | M4, user's choice | 4 | Counterbored holes in the plinth; the arm tips over unless these (or a clamp) are used |

The hub caps press onto the clevis plates over the screw heads. Each has a 1.8 mm recess, which clears the M3 button heads (1.65 mm).

The design language is original rather than a copy of the SO-101 parts. It uses:
- a 12-sided plinth with a fluted column
- faceted octagonal beams that narrow at the waist, with a raised spine and twin tapering hex windows
- matching hex windows on the wrist and gripper that show the servo inside
- arc windows in the turntable
- separate graphite hub caps
- slim 20 mm serrated jaws, centred on the wrist roll axis

The geometry is lightweighted where mass costs the most torque, in the gripper, wrist and forearm. The moving arm is 529 g, 69 % of it servos. See [`../calcs/README.md`](../calcs/README.md) for the numbers.

## Regenerate

```bash
python talos_arm.py               # ready pose -> talos_arm_assembly.step
python talos_arm.py --pose zero   # -> talos_arm_zero.step
python render.py                  # renders + interference report
```

Requires `cadquery` (2.8) and, for `render.py`, `vtk` and `numpy`, which install with CadQuery. Add poses to `POSES` in `talos_arm.py` to export other configurations.

## Known simplifications

- The servo is a simplified STS3215, measured from the SO-ARM100 reference model: body, back pad, connector headers, case holes and horn holes. Check the connector positions and case-hole depths against a real servo before printing.
- The board is modelled as a bare 42 × 33 mm PCB with Ø2.5 holes on a 37 × 28 mm pattern. That matches Waveshare's spec for the Bus Servo Adapter (A) that SO-101 uses. Components aren't modelled, so check where the power and USB ports sit before relying on rear access.
- Cables aren't modelled as solids, only their bays, tunnel and clips. Service loops at the pan and roll joints need slack: about 120 mm at the pan and 80 mm at the roll.
- Parts are single solids. Before printing, check wall thickness (the servo shells are 2.2 mm beside the caps) and split anything that won't print without support.
