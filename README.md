<div align="center">

# T A L O S

### A 3D-printed, 5-axis desktop robot arm

![5 DOF + gripper](https://img.shields.io/badge/DOF-5%20%2B%20gripper-2f6fdb?style=for-the-badge)
![420 mm reach](https://img.shields.io/badge/reach-420%20mm-2f6fdb?style=for-the-badge)
![522 g moving mass](https://img.shields.io/badge/moving%20mass-522%20g-2f6fdb?style=for-the-badge)
<br>
![Feetech STS3215](https://img.shields.io/badge/servos-Feetech%20STS3215-555?style=flat-square)
![SolidWorks](https://img.shields.io/badge/CAD-SolidWorks-555?style=flat-square)
![PLA](https://img.shields.io/badge/printed-PLA-555?style=flat-square)
![MuJoCo](https://img.shields.io/badge/sim-MuJoCo-555?style=flat-square)

<img src="cad/assembly_iso.png" alt="Talos in its ready pose" width="560">


</div>

---

## At a glance

| | |
| --- | --- |
| **Joints** | Pan · shoulder · elbow · wrist flex · wrist roll · gripper |
| **Servos** | 6 × Feetech STS3215, 12 V, 2.94 N·m stall, daisy-chained on one bus |
| **Links** | 116 mm upper arm · 135 mm forearm · 63 mm wrist · 108 mm gripper |
| **Joint ranges** | Shoulder and elbow ±100° · wrist flex −100° to +95° · pan ±110° · roll ±150° |
| **Grip** | 115° jaw opening (67 mm between the tips) · 23 N continuous |
| **Build** | 9 printed parts in PLA, no supports on the arm links · 738 g total |

## Design decisions

| Decision | Why |
| --- | --- |
| **116 / 135 mm links** | Proven proportions for this servo (SO-101's). They give 402 mm of reach against a 350 mm target. A shorter forearm saves only 9 % of elbow torque for 20 mm less reach |
| **Shoulder on the pan axis** | Same reach in every direction, and 6 % less load on the pan servo than an offset shoulder |
| **Waisted octagonal links with windows** | The shoulder servo is the torque limit, so mass comes out of mid-span. 70 % of the moving mass is servos |
| **Flat-bottomed links, separate idler plates** | Every link prints flat with no supports, on one solid first layer |
| **Pockets 0.3 mm over the servo** | Set from a printed clearance gauge, not guessed. Pockets are built from the manufacturer's servo CAD |
| **Clevis joints on both horns** | Each joint is supported on both sides of the servo, not hung off one shaft |
| **1.0 mm clevis gap** | The first 0.5 mm gap rubbed on a test print |
| **Slim idler plates** | The servo's cable plugs sit 11 mm from the joint axis; the plates clear them through the full range |
| **Four screws on the pan and roll servos** | Stops the servo rocking under the arm's overturning load |
| **Cables cross on the joint axes** | Bending a joint barely changes the cable's length, so the stock 21 cm cables reach |
| **Clamped base (4 × M4)** | At full reach the arm's centre of mass is 188 mm out, but the base edge is only 64 mm out |

## Five problems We solved

| Problem | Fix |
| --- | --- |
| 🔩 **The first prints didn't fit the servos** | Rebuilt every pocket from the manufacturer's CAD. A script now checks the real servo in every part |
| 🚪 **Servos had no way into their parts** | Added a slide-in path with channels for everything sticking out. Four servos go in with their horns off |
| 📏 **Unknown printer clearance** | Printed a six-window gauge and two test pieces before the arm. Settled on 0.3 mm, plus lead-ins and room for sagging bridges |
| ⚙️ **Test joint rubbed when turning** | Doubled the clevis gap to 1.0 mm without changing the screws |
| 🔌 **Two cable runs too short** | Measured every run through its joint's travel. Specified 35 cm and 30 cm cables and a clip for the slack |

## Printing

- **Setup:** Ender 3 V3 KE, 0.4 mm nozzle, Creality PLA, 0.2 mm layers, 3 walls, 20 % infill.
- **Files:** print-ready 3MFs are in [`cad/print/`](cad/print), already turned onto their print face. STEPs are in [`cad/step/`](cad/step).
- **Order:** print the fit gauge (TAL-111) first, then the test pieces, then the arm.
- **Calcs:** torque, stress and payload checks are in [`calcs/`](calcs).

## Simulation

- **What:** a MuJoCo model of the arm, built from the CAD, in [`sim/`](sim). Drive it from the keyboard, or let it pick up a 200 g block and drop it in a box by itself.
- **Run:** on Windows, double-click `sim\run.bat`. It sets up Python the first time. [`sim/README.md`](sim/README.md) has the keys and the other platforms.
- **Checks:** the model matches the CAD and the calcs: [`sim/check_results.md`](sim/check_results.md). The physics adds what static calcs can't, such as how fast the shoulder can lift 200 g and how wide a block the jaw can hold.
