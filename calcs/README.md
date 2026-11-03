# Calculations

Four scripts check the arm:
- `torque_check.m` is the original quick hand calc.
- `pan_base_check.m` covers the pan servo and the base.
- `design_check.py` reads the CAD in [`../cad/talos_arm.py`](../cad/talos_arm.py) directly and checks everything else. Full output: [`design_check_results.md`](design_check_results.md).
- `so101_compare.py` runs the same statics on SO-101's URDF for comparison.

## Design review (CAD v1, lightweighted)

**Verdict: the design holds up, with two loads to manage and one clamp to fit.** Everything structural has margin. The servos at the shoulder (and just barely the elbow) are the limit, as in the first calc.

| Check | Result | Status |
| --- | --- | --- |
| Moving mass | 529 g above the base, 730 g total | |
| Shoulder holding, arm straight, 200 g | 1.75 N·m = 59 % of stall, 1.8× the 1/3 limit | ❌ over (first calc: 1.62) |
| Elbow holding, arm straight, 200 g | 1.07 N·m, 1.09× the limit | ⚠️ just over (first calc: 0.96) |
| Wrist flex / roll | 0.43 / 0.004 N·m | ✅ |
| 200 g payload, top-down picks | OK out to 190 mm from the base axis | ✅ typical work zone |
| Dynamics, shoulder 90° in 1 s | +0.37 N·m peak, 72 % of stall total | ✅ transient |
| Pan servo shaft | 1.75 N·m overturning moment (SO-101: 1.87 reaching forward), carried by the servo's own bearing | ✅ within SO-101's own load; measure play (see below) |
| Base tipping | Tips over at full reach (1.14 N·m net) unless fixed down | ❌ clamp or bolt it |
| Beam bending | Worst 3.2 MPa (upper arm at the shoulder clevis, through the horn screw holes), safety factor 3.1 | ✅ |
| Tip sag from link bending | 2.1 mm | ✅ |
| Horn screw bearing | M3: 5.3 MPa holding, 8.8 MPa at stall (SF 2.3) | ✅ |
| Grip force | 23 N continuous at mid-jaw, so 200 g needs only 3.3 N | ✅ |
| Fixed finger root (20 mm wide) | 1.8 MPa continuous, 5.5 MPa at jaw stall (SF 1.8) | ✅ |
| Jaw opening | 115°, about 67 mm between the tips | ✅ |
| Joint ranges (CAD collision sweep) | Shoulder ±100°, elbow ±105°, wrist −105/+95° | ✅ |

### Weight reduction

The printed parts were reworked to cut mass where it costs the most torque. A gram in the gripper loads the shoulder about 4× more than a gram in the upper arm.

| Link | Printed before (g) | Printed after (g) | Change |
| --- | --- | --- | --- |
| Gripper | 53 | 42 | Jaws slimmed from 35 to 20 mm wide and centred on the roll axis; keel narrowed; hex windows onto the servo |
| Wrist | 32 | 30 | Bridge cut back to side lugs, slimmer clevis neck, hex windows onto the servo |
| Forearm | 63 | 59 | Waisted beam (32 × 24 mm at the waist), twin tapering hex windows |
| Upper arm | 56 | 54 | Waisted beam (34 × 25 mm at the waist), twin tapering hex windows |
| Shoulder | 39 | 38 | Four arc windows in the turntable |
| **Moving total** | **548** (with servos) | **529** | Shoulder 1.79 → 1.73 N·m, elbow 1.09 → 1.05 N·m |

The wiring and screw revision afterwards kept the moving mass at 529 g. It added plug bays, screw holes and counterbores, the slim idler plates, the board deck, and moved the wrist roll servo 3 mm forward. The shoulder load rose slightly to 1.75 N·m because of the longer wrist.

The base was left alone on purpose. It's fixed, so it adds no joint torque, and its weight works against tipping.

The gains are modest because 69 % of the moving mass is servos (6 × 61 g). The printed parts are already mostly 20 % infill, so their weight is in the outer skin. Cutting more plastic would cost stiffness for little torque.

**The bigger lever is print settings.** The same mass model puts 15 % infill with 2 perimeters (0.8 mm) at 163 g of moving printed parts instead of 224 g, a 27 % cut. That's three times what the geometry changes saved. It costs strength, since the outer skin carries most of the bending. To check it, set `INFILL = 0.15` and `T_SHELL = 0.8` in `design_check.py` and re-run.

### Talos vs SO-101

SO-101 numbers come from its own URDF via [`so101_compare.py`](so101_compare.py), using the same straight-arm method. 200 g sits at each arm's tool point: Talos mid-jaw, SO-101's `gripper_frame_link`.

| | SO-101 (URDF) | Talos (CAD) |
| --- | --- | --- |
| Upper arm / forearm | 116 / 135 mm | 116 / 135 mm |
| Shoulder axis height | 117 mm | 108 mm |
| Shoulder axis offset from pan axis | 30.4 mm forward | 0 (centred) |
| Tool point from shoulder axis | 410 mm | 402 mm (420 to the jaw tip) |
| Tool point from pan axis, reaching forward | 441 mm | 402 mm |
| Moving mass | 485 g | 529 g (499 g without the 30 g screws and cable allowance the URDF leaves out) |
| Shoulder, straight out, 200 g | 1.67 N·m | 1.75 N·m |
| Elbow, straight out, 200 g | 1.03 N·m | 1.07 N·m |
| Wrist flex, straight out, 200 g | 0.43 N·m | 0.43 N·m |
| Pan shaft, straight out, 200 g | 1.87 N·m | 1.75 N·m |
| Joint limits (shoulder / elbow / wrist flex) | ±100° / ±97° / ±95° | ±100° / ±105° / −105°…+95° (CAD collision sweep) |
| Gripper range | −10°…100° | 0°…115° |

The two arms load their servos almost identically: within 5 % at every pitch joint, and 6 % lower at the pan shaft on Talos.

On a like-for-like basis (no hardware allowance) Talos is about 3 % heavier. Its tool point is about 8 mm closer to the shoulder, so the torques come out about level.

The real trade is the shoulder offset. SO-101's forward offset buys it about 40 mm more reach from the base axis, at the cost of a higher pan load. Talos centres the shoulder, which lowers the pan load but gives up that reach.

Both arms put the shoulder over the 1/3-stall line with 200 g straight out. So the "200 g with the elbow bent" rule applies to SO-101 as much as to Talos.

### What changed from the first calc

`torque_check.m` used SO-101 numbers: L1 = 120, L2 = 120, wrist to tip 160 mm, 385 g of moving links. The CAD is 116 / 135 / 169 mm, and the links weigh 429 g once modelled. The longer forearm and the extra mass raise the shoulder load by about 8 % and push the elbow just past the limit.

Shortening the forearm barely helps. At 115 mm the elbow just drops under the limit (0.99×; section 10 of the results), but that costs 20 mm of reach. The 135 mm forearm stays, since it matches SO-101 reach.

### Decisions this supports

1. **Keep the v1 rule from the first calc:** 200 g with the elbow bent (top-down picks within 190 mm), and light objects at full reach. The payload envelope shows where the line falls:

   ![Payload envelope](payload_envelope.png)

2. **Fix the base down.** With the arm straight out, the whole arm's centre of mass is 186 mm out and the base edge is only 64 mm out. The two rear M4 holes need about 6 N each, which is trivial, but they must be used (or a table clamp, like SO-101).
3. **No pan bearing ring for v1. Measure the turntable and keep the retrofit ready.**
   - **The load:** the pan servo carries the arm's whole overturning moment through its own shaft (1.78 N·m with 200 g at the tip), and Feetech publishes no rating for that.
   - **The precedent:** SO-101 runs the same servo the same way. Its shoulder axis sits 30.4 mm forward of the pan axis, so reaching forward its pan shaft sees 1.87 N·m with 200 g. Talos is about 5 % below that, because its shoulder axis is centred.
   - **The risk:** the real cost of shaft play is accuracy: any rim lift at the turntable shows up 11.4× larger at the tip.
   - **The test:** after assembly, load the arm straight out and put a dial indicator on the turntable rim. Up to 0.1 mm of lift (1.1 mm at the tip) is fine. Above that, fit a slide ring or thin bearing in the existing 2.8 mm gap under the turntable. See `pan_base_check.m`.
   - **Correction:** an earlier version of this README said Talos was 24–28 % beyond SO-101. That used the original calc's approximate SO-101 inputs (120/120 mm links) and assumed the offset pointed backwards. `so101_compare.py` now works from SO-101's actual URDF.
4. **The shoulder counterbalance from the build plan is still the right v2 upgrade.** It needs about 0.77 N·m of help at horizontal to bring a straight-arm 200 g hold under the limit.

![Shoulder torque vs elevation](shoulder_torque_cad.png)

### Assumptions to replace with measurements

- **Printed mass:** PLA, 1.2 mm perimeters, 20 % infill. Servo 55 g + 6 g of screws and cable. Weigh the printed parts and update `M_SERVO`, `INFILL` and `T_SHELL`.
- **STS3215 12 V figures:** stall 2.94 N·m, 4096 counts/rev, about 45 rpm no-load. Check these against the datasheet for your variant.
- **Material:** printed PLA E = 2.3 GPa, 10 MPa sustained bending allowable (creep-limited), 20 MPa screw bearing allowable.
- **Horn screws:** M3 on a 13.9 mm circle, into the horns' Ø2.5 tapped holes (per the reference servo model).
- **Backlash:** isn't known. Each 0.5° at the shoulder moves the tip 3.6 mm, so measure it before promising ±2 mm repeatability.

## Shoulder and elbow torque (`torque_check.m`)

**Question:** can STS3215 servos at the shoulder and elbow hold the arm and a 200 g payload?

**Method:** static moment balance with the arm held straight out (the worst case):

```
tau = g * sum(m_i * x_i)
```

where `x_i` is each mass's horizontal distance from the joint axis. Masses and centre-of-mass offsets are the lumped CAD values printed by `design_check.py` (each link includes its servo).

**Limit:** keep continuous load under 1/3 of stall (2.94 N·m / 3 ≈ 0.98 N·m) so the servo doesn't overheat or droop.

| Input | Value |
| --- | --- |
| Upper arm (shoulder → elbow) | 116 mm |
| Forearm (elbow → wrist) | 135 mm |
| Wrist → gripper tip | 169 mm |
| Reach (shoulder → tip) | 420 mm |
| Payload | 200 g at the tip |
| Servo stall torque | 2.94 N·m (30 kg·cm, 12 V) |

With the CAD inputs, the formula gives **1.78 N·m at the shoulder (61 % of stall) and 1.10 N·m at the elbow (38 %)**, with the payload at the tip. That matches `design_check.py` for the same case. [`results.txt`](results.txt) holds those numbers, computed with a line-for-line Python port because MATLAB wasn't available. `shoulder_torque.png` is still the old SO-101 plot until the script is re-run.

**Run it:** open `torque_check.m` in MATLAB and press F5. It prints the results, shows each mass's share of the shoulder torque, and saves `shoulder_torque.png`.

## Pan servo and base (`pan_base_check.m`)

**Question:** does the pan servo need a bearing ring, and does the base need fixing down?

**Method:** the same straight-arm statics, taken about the pan axis and about the front edge of the base. Masses and CoM distances come from the CAD. The script also compares against SO-101 and converts turntable rim lift into tip error.

| Result | Value |
| --- | --- |
| Pan shaft moment, Talos | 0.96 N·m unloaded, 1.78 N·m with 200 g at the tip |
| Pan shaft moment, SO-101 (URDF, reaching forward) | 1.01 N·m unloaded, 1.87 N·m with 200 g at its tool point |
| Talos / SO-101 | 0.95× unloaded, 0.95× loaded |
| Axial load on the pan servo | 7.2 N |
| Whole-arm CoM | 188 mm from the base axis; the front edge is 64 mm out |
| Tipping | Tips at 1.16 N·m net, so it needs 5.7 N per rear M4 bolt |
| Rim lift → tip error | ×11.4: 0.05 mm → 0.57 mm, 0.1 → 1.14, 0.2 → 2.27, 0.3 → 3.41 |

These figures come from a Python transliteration of the script, with the payload at the tip. `design_check.py` puts it at mid-jaw, so its numbers run slightly lower. Open `pan_base_check.m` in MATLAB and press F5 to confirm them. It also saves `pan_moment.png`.

## Full design check (`design_check.py`)

Builds the CAD, then runs ten checks:
- mass budget
- joint holding torque
- CAD collision sweep for joint ranges
- payload envelope from inverse kinematics
- dynamic torque
- pan shaft moment and base tipping
- beam stress and sag from CAD cross-sections
- horn screw bearing
- gripper force, finger stress and opening
- encoder resolution and forearm-length sensitivity

```bash
python design_check.py          # about 30 s; writes design_check_results.md and the plots
python design_check.py --fast   # skips the CAD collision sweep
```

It uses the same Python environment as the CAD (`cadquery`, `numpy`, `matplotlib`). Because it reads the CAD directly, re-run it after any dimension change in `talos_arm.py`.
