# Talos

A 3D-printed desktop robot arm: five axes plus a gripper, about 420 mm of reach, driven by six Feetech STS3215 bus servos. It's a two-person project. Lucas does the mechanical design (CAD, calcs, printing), and Ryan does the simulation and software. The arm takes its size and servo from the open-source SO-101 arm, but every part is its own design.

![Talos in its ready pose](cad/assembly_iso.png)

## The arm in numbers

| | |
| --- | --- |
| Servos | 6 × Feetech STS3215, 12 V, 30 kg·cm (2.94 N·m stall), on one serial bus |
| Joints | Pan, shoulder, elbow, wrist flex, wrist roll, gripper |
| Links | Shoulder to elbow 116 mm, elbow to wrist 135 mm, wrist to roll flange 63 mm, roll flange to jaw tip 108 mm |
| Reach | About 420 mm from the shoulder axis |
| Joint ranges | Shoulder ±100°, elbow ±100°, wrist flex −100° to +95°, pan ±110° and wrist roll ±150° (set by the cables) |
| Moving mass | 522 g (70 % of it servos), 738 g with the base |
| Material | PLA, 0.2 mm layers, 3 walls, 20 % infill |

## Design decisions

### Overall layout

**Link lengths: 116 mm shoulder to elbow, 135 mm elbow to wrist.** These are SO-101's lengths. SO-101 is a proven arm built on the same servo, so its proportions are a known-good starting point. The project needed at least 350 mm of reach from the base axis to the gripper tip; these lengths give 402 mm to the tool point, and about 420 mm from the shoulder axis to the tip.

Longer links cost torque, though. The servo's continuous limit is about a third of its 2.94 N·m stall. With the arm straight out and 200 g in the gripper, the elbow is just over that limit (1.08×).
- **Shorter forearm:** a 115 mm forearm would bring the elbow just under the limit, but it gives up 20 mm of reach for about 9 % less elbow torque.
- **Choice:** the forearm stays at 135 mm, and the arm follows a simple rule instead: full payload only with the elbow bent.

**The shoulder sits on the pan axis.** SO-101 puts its shoulder 30 mm in front of the pan axis, which gains it about 40 mm of reach. Talos centres the shoulder instead. That keeps the pan servo's bearing load lower (about 6 % below SO-101's, with 200 g) and makes the arm's reach the same in every direction. The cost is reach, which the 402 mm already covers.

**Heights: pan horn 62 mm above the table, shoulder axis 46 mm above the pan horn.**
- **Pan horn height:** the pan servo stands upright inside the base column, with its cable plugs and a cable bend below it, so its horn ends up 62 mm above the table.
- **Shoulder height:** the shoulder servo hangs in the cradle with 35 mm of its body below its shaft. 46 mm is just enough for that to clear the cradle floor, which carries the turntable's screws into the pan horn. Those go in before the servo.

**Wrist to roll flange: 63 mm.** That's the shortest distance that keeps the roll servo's cable plugs clear of the forearm's end as the wrist bends. It was first 3 mm shorter, and the plugs hit.

**Gripper: 108 mm from the roll flange to the jaw tip, 20 mm fingers.** The jaw opens 115°, about 67 mm between the tips, so it can close on objects up to about 60 mm across. The fingers are centred on the roll axis, so rolling the wrist turns the grip about its own centre. Grip force is 23 N continuous at mid-jaw, so a 200 g object needs only a sixth of it.

**Base: a 132 mm, 12-sided plinth with four M4 holes on a 108 mm circle.** With the arm straight out, the whole arm's centre of mass is 188 mm from the pan axis but the base's edge is only 64 mm out, so the arm tips over unless the base is fixed down. Two of the bolts need only about 6 N each, so small M4 bolts or a table clamp hold it. The 80 mm column holds the pan servo upright, with a deck behind it for the servo bus board.

### Holding the servos

**Pockets 0.3 mm bigger than the servo all round.** FDM holes come out a little small. 0.3 mm is the clearance that slid smoothly on a printed test gauge (TAL-111, six windows from 0.2 to 0.7 mm); 0.2 was snug.
- **Why not 0.2:** a real pocket is four times deeper than the gauge and the parts are bigger, so 0.2 leaves no margin.
- **What locates the servo:** the screws, not a press fit.
- **Features that only pass through:** the rib, the bump by the connectors, the shaft and the boss get 0.4 mm.
- **Printer:** these numbers belong to the printer setup used (Ender 3 V3 KE, 0.4 mm nozzle, Creality PLA, Creality Print). On another printer, print the gauge first.

**Shell walls 18.4 mm from the servo's centre plane.** That's the servo's half-thickness (16.5 mm on the connector side), plus the 0.3 mm clearance, plus a 1.6 mm wall. Thin walls keep the joints narrow: with the clevis plates outside them, every joint is under 46 mm wide. Where a screw head needs more under it, it sits on a small pad instead of a thicker wall.

**Case screws: 2.4 mm of print under every head.** The servos' screws may be M2 × 5 or M2 × 6. A 2.4 mm wall suits both: a 5 mm screw gets 2.6 mm of thread in the servo, and a 6 mm screw goes 3.6 mm in, short of the 3.8 mm SO-101 uses.

**Four screws on the pan and wrist roll servos.** Each has two screws at each end. The pan servo carries the whole arm's overturning moment, and with screws at one end only it could rock in its 0.3 mm of clearance. With four, the worst screw carries 38 N.

**Shoulder cradle: the posts beside the shaft stand 0.9 mm off the servo.** As the shoulder servo works, its body tries to twist about its screws.
- **Before:** the posts were the first thing it reached. They're slender (4.1 mm then), and at stall they would have seen 13.9 MPa across their layer lines, over PLA's 10 MPa long-term limit.
- **Now:** with the posts set back, the servo bears on the cradle's two long walls instead: 5.4 MPa at stall.

### The joints

**Clevis joints that clamp both sides of each servo.** At the shoulder, elbow and wrist flex, the next link screws onto both of the servo's horns, like SO-101.
- **Output horn (shaft):** carries the torque.
- **Passive horn:** turns freely on the back of the servo, so the joint is supported on both sides instead of hanging off one shaft.

**Clevis plates 1.0 mm clear of the shell, 3.5 mm thick.**
- **Gap:** the gap was first 0.5 mm, and the printed test joint rubbed. 1.0 mm leaves room for elephant's foot on the shell's edge and end play in the passive horn.
- **Pads:** a Ø19 mm pad on each plate's inside reaches the horn. The horn screws go through pad and plate, so they stay M3 × 6.
- **Strength:** at 3.5 mm, the plates bend at a safety factor of 3 or more when holding a load.

**Output plates have a 16 mm radius hub; idler plates a 10.5 mm hub, pinched to a 17 mm neck.**
- **Output side:** the hub takes the horn's four screws on a 14 mm circle, with a wide root into the link.
- **Passive side:** the servo's cable plugs sit only 11 mm from the joint axis, so the idler plate is just big enough to cover the passive horn (Ø19.2 mm) and narrow enough to pass the plugs through the whole joint range.

**Shell openings: 10.8 mm radius around each horn.** The horns are 9.6 mm in radius, so they can come off and go back on through the shell, with 1.2 mm to spare. Four servos have to go in with their horns off, so the horns are refitted through these openings.

**Joint ranges ±100° at the shoulder and elbow, −100° to +95° at the wrist.**
- **What sets them:** 5° past each limit, either an idler plate touches the parent servo's plugs, or one link meets the next.
- **SO-101:** its elbow is ±97°, so this matches it.
- **Pan and wrist roll:** no mechanical stop; their cables set the limits (±110° and ±150°).

### The links

**Waisted, faceted octagonal beams with windows.** The shoulder is the limiting servo, and every gram further out costs shoulder torque, so mass comes out where it's cheapest:
- **The beam:** narrows at the waist (34 mm wide and 25 mm tall on the upper arm), between the full-size clevis at one end and the servo shell at the other.
- **Windows:** twin hex windows take out more.
- **Spine:** a raised spine along the top puts some stiffness back.

The result: the moving arm is 522 g, 70 % of it servos. The worst link stress is 5.8 MPa (safety factor 1.7), at the upper arm's elbow end, where the floor is open for the servo to go in. Tip sag from bending is about 2.3 mm.

**Flat underside, taper on top only.** The links print standing on a flat underside, so they need no support and sit on one solid first layer. The waist dips on the top side only for the same reason.

**Separate idler plates on the arm links.** If the idler plate were part of the link, the gap between the two clevis plates would need support under it. As a separate part (TAL-110) screwed into a pocket in the beam, everything prints flat.

**Openings through the print bed face get a 0.5 mm × 45° lead-in.** The arm links take their servo from below, through the face they print on. That's where elephant's foot makes the first layers flare inward, exactly where the servo has to start, so the chamfer opens that edge.

**Pocket walls that print as bridges get 0.2 mm more room.** On the wrist and gripper, one pocket wall prints as a 25–33 mm bridge along the servo's whole slide. Bridges sag, so those walls stand back a little further.

### Cables

**Each cable crosses its joint on the joint's axis,** just outside the idler plate, so bending that joint barely changes the cable's length. A 21 cm cable then covers the shoulder, elbow and wrist runs with room to spare.

**The two runs that wind round a turning axis need longer cables.**
- **Pan → shoulder:** wraps part way round the column as the arm pans, and needs a 35 cm cable.
- **Wrist roll → jaw:** winds round the roll axis, and needs a 30 cm cable.
- **Slack:** each changes length by about 10 cm over its joint's travel, so one end of the travel leaves a loose loop. A clip on the back of the column holds the pan cable below the turntable's edge, so its loop can't catch there.

## The five biggest problems, and how they were solved

| # | Problem | Cause | Fix |
| --- | --- | --- | --- |
| 1 | **The first printed parts didn't take the servos** | The servo model came from SO-101's simplified simulation mesh, which is up to 1 mm small in places and missing features. The pockets also had too little clearance for FDM | Every pocket was rebuilt from Feetech's own CAD of the STS3215, measured feature by feature. A script now puts that real servo in every part and checks it fits |
| 2 | **Most servos had no way into their part** | Only the seated servo was checked. On the way in, its output shaft and the rib on its top face caught on the walls | Every pocket got a path the servo can slide along, with channels for everything that sticks out. Four servos go in with their horns off, and the horns are refitted through openings in the shell. A script slides the real servo in and checks there's no tight spot anywhere |
| 3 | **Nobody knew how much clearance this printer needed** | FDM holes print small, by an amount that depends on the printer, filament and slicer | Three test prints before the arm: a six-window gauge (0.2–0.7 mm), a real pocket cut from the upper arm, and a real elbow joint. The pockets went from 0.4 to 0.3 mm, and the lead-ins and bridge room were added for the print effects the gauge can't show |
| 4 | **The elbow test joint rubbed as it turned** | The clevis plates were only 0.5 mm off the shell. Elephant's foot and the passive horn's end play took that up | The plates moved out to 1.0 mm clear, with taller pads so the horn screws didn't change. The idler plate now goes onto the clevis first, which lines it up flush with the free-spinning passive horn |
| 5 | **Two cable runs were longer than the servos' 21 cm cables** | The pan and wrist roll runs wind round a turning axis, so they need about 10 cm more at one end of the travel | A script measures every run through its joint's travel. Those two runs got 35 cm and 30 cm cables, and a column clip manages the pan cable's slack loop |

## Printing

- **Settings:** PLA, 0.4 mm nozzle, 0.2 mm layers, 3 walls, 20 % infill, elephant's-foot compensation on. Use the same profile as for the gauge, so the 0.3 mm fit holds.
- **Keep the orientation in each 3MF file.** They're already turned the right way.
- **Print order:**
  1. TAL-111 gauge, then the test pieces. Only print the arm once they pass.
  2. The arm in build order, base first, so assembly can start while the rest prints.
- **Bought parts:**
  - 6 × STS3215 servos and a Waveshare Bus Servo Adapter (A)
  - 36 × M3 × 6 and 4 × M3 × 10 button head screws
  - 24 × M2 × 6 (or × 5) and 4 × M2.5 × 6 self-tapping screws
  - one 35 cm and one 30 cm 3-pin bus servo cable (the servos' own 21 cm cables do the other four runs)
  - 4 × M4 bolts or a clamp to fix the base down

