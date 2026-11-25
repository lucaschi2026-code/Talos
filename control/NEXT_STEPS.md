# Next steps: driving the real arm from a computer

The arm is built, and the simulation in [`sim/`](../sim) drives a model of it from the keyboard. This plan covers getting from there to driving the real arm the same way. It's ordered: each phase depends on the one before, and each has a "done when" so you know when to move on.

The short version:

1. Power up safely and talk to every servo.
2. Give every servo its own ID.
3. Calibrate, so a servo reading means the same angle as the sim.
4. Move one joint at a time, slowly, with low torque.
5. Drive the real arm from the keyboard, reusing `drive.py`.
6. Measure the real arm and correct the sim with the results.

## What we're working with

| Part | What matters for the software |
| --- | --- |
| 6 × Feetech STS3215, 12 V | Serial bus servos. Each one has an ID, reads 0–4095 over one turn (0.088° per count) and reports position, speed, load, voltage, temperature and current |
| Waveshare bus servo adapter (in the base) | USB to the servo bus. It also passes the 12 V supply to the servos. USB alone can't move a servo |
| 12 V supply, ≥ 5 A | Powers all six servos through the board |
| Laptop | Runs Python. No microcontroller is needed: the board turns USB into the servo bus |

Joint ranges from the sim (`sim/model/talos.xml`), with 0 = arm straight up:

| Joint | Range | Notes |
| --- | --- | --- |
| Pan | ±110° | Cable loop limit |
| Shoulder | ±100° | + is forward and down |
| Elbow | ±100° | + is down |
| Wrist flex | −100° to +95° | |
| Wrist roll | ±150° | Cable loop limit |
| Gripper | 0° to 115° | 0 = closed, + opens |

The ready pose is pan 0°, shoulder 20°, elbow 95°, wrist flex 35°, roll 0°, gripper 18°.

## Which library

Use Feetech's own Python SDK, **`feetech-servo-sdk`** on PyPI (`pip install feetech-servo-sdk`, imported as `scservo_sdk`). It depends only on `pyserial`, which is pure Python, so Smart App Control shouldn't block it. It handles the packets, and we write the arm-level code ourselves, which fits the project's "our own design" goal.

[LeRobot](https://huggingface.co/docs/lerobot/so101) drives SO-101, which uses the same servos and the same board. Don't install it to run Talos: it's a big machine-learning framework built around SO-101's joints. Use its Feetech code as a reference when ours misbehaves, especially for calibration.

## Phase 1: Power up and find the servos

**Before power:**

- Check the 12 V supply with a multimeter before connecting it, and confirm the servos are the 12 V version (C018 or C047, not the 7.4 V C001). The build plan lists the wrong voltage as the one way to destroy the servos.
- On the bus adapter, set **both jumpers to B (USB)**. A is for driving the bus from a microcontroller.
- Clamp the base to the table. `calcs/` shows the arm tips over otherwise, and the sim assumes it's clamped.
- Pan through ±110° and roll through ±150° by hand, and check that the cable loops don't catch (see Wiring in `cad/README.md`).

**Then:**

1. Plug in the 12 V supply, then USB. In Device Manager, look under Ports (COM & LPT) for a new COM port. Windows' built-in driver usually works. If no port appears, install the CH343 driver from Waveshare.
2. Set up Python with `feetech-servo-sdk`. It can go in the existing `talos-sim` environment, so one environment runs the sim and the arm.
3. Write `control/scan.py`: open the port at 1,000,000 baud (the factory setting), ping every ID from 0 to 253, and print each one that answers, with its position, voltage and temperature.

**Done when:** the scan sees the servos, and each reports about 12 V (register 62 reads about 120) and room temperature.

## Phase 2: Give every servo its own ID

New STS3215s all ship as **ID 1**. Two servos with the same ID both answer, their replies collide, and neither can be read. The scan in phase 1 tells you where you are:

- **Six different IDs answer:** they were set before assembly. Write down which ID is which joint by moving one and watching its reading change, then go to phase 3.
- **Only ID 1 answers, or nothing does:** they still share ID 1. Set them as below.

Plan the IDs to match SO-101: pan 1, shoulder 2, elbow 3, wrist flex 4, wrist roll 5, gripper 6.

**Setting IDs on the built arm.** Every servo has to be set while it's the only one at ID 1, and the arm is already wired as one chain: board → pan → shoulder → elbow → wrist flex → wrist roll → jaw. So unplug the chain after the pan servo and add one servo at a time:

1. With only the pan servo connected, change its ID from 1 to a temporary **10**. Its final ID is 1, which every new servo also has, so it has to stay out of the way until the end.
2. Plug in the shoulder servo. It answers as ID 1. Change it to 2.
3. Plug in the elbow, change 1 → 3. Repeat for wrist flex (4), wrist roll (5) and gripper (6).
4. Change the pan servo from 10 to 1.
5. Scan again. Exactly IDs 1–6 should answer.

The ID is in EEPROM (register 5), and the servo protects EEPROM with a lock (register 55). Write 0 to the lock, write the ID, then write 1 to the lock, or the change is lost at power-off. Power-cycle and scan again to be sure the IDs stuck.

**Done when:** IDs 1–6 answer after a power cycle, and each one moves the joint you expect when you turn it by hand.

## Phase 3: Calibrate, so the servos and the sim agree

A servo reads 0–4095 over one turn, but where 0 falls depends on how its horn went on during assembly. Calibration finds, for each joint, the reading at the sim's 0° and which way is +. After that, code can work in the same radians as the sim:

```
reading = 2048 + direction × angle_rad × 4096 / (2π)
angle_rad = direction × (reading − 2048) × 2π / 4096
```

**The servo does most of the work.** Writing **128** to the torque register (40) tells the servo to call its current position 2048, by setting its position offset (register 31) for you. So:

1. With the arm powered and torque off, move it by hand into the **zero pose**: every joint at 0, arm straight up (`sim/renders/zero.png`). Hold the gripper closed.
2. For each servo, write 128 to register 40. Each now reads 2048 at the sim's zero.
3. For each joint, move it by hand in the sim's + direction (the table above) and see whether its reading goes up or down. That gives `direction` = +1 or −1.
4. Save the IDs, directions and ranges to `control/calibration.json` in the repo.

Centring at 2048 also keeps every joint clear of the 4095 → 0 wrap. The largest range, roll's ±150°, is ±1707 counts around 2048, so readings stay between 341 and 3755. A joint that crosses the wrap would make position control swing the long way round.

**Hardware limits as a backstop.** Write each joint's range into the servo's min and max angle limits (registers 9 and 11, EEPROM, unlocked as in phase 2), about 2° inside the sim's range. Then even a bug in our code can't drive a joint into its hard stop or the cable loop.

**Done when:** with torque off, moving the arm by hand into the ready pose prints angles within about 2° of the ready pose above, and the sim viewer shows the same pose.

## Phase 4: First moves

Start each joint weak and slow, then work up. These settings are RAM (registers 40 and up), so they reset at power-off and the code sets them every time it connects:

| Setting | Register | Start at | Why |
| --- | --- | --- | --- |
| Torque limit | 48 | 300 (30 %) | A wrong command pushes gently. Raise it as you go. The shoulder needs most of the servo's torque to lift the arm, so it will need more |
| Acceleration | 41 | about 20 (175°/s²) | Smooth starts and stops. 0 means no limit |
| Speed | 46 | about 500 counts/s (44°/s) | The same 45°/s the sim jogs at |

1. **Before turning torque on, set each servo's goal position to its present position.** Otherwise it jumps to whatever goal it last had.
2. Hold the arm, turn torque on for one joint, and move it about 10° and back. Start at the wrist and gripper, where a mistake costs least, and finish with the shoulder.
3. Log present position, load, current and temperature while it moves, and check the direction matches the sim.

**Stopping.** Turning torque off drops the arm, since the shoulder and elbow hold it up. Make the main stop key **hold position** (goal = present position on every joint), and keep torque off as a second key for when someone is holding the arm.

**Done when:** every joint moves both ways in the right direction, stops where told, and holds the ready pose with each servo's load close to what the sim's readout shows for that pose.

## Phase 5: Drive the real arm from the keyboard

The sim already has the driving code worth keeping: the key map, jog speed, the "lead" that stops a blocked joint winding its target away, the gripper's squeeze at a set torque, and `autopick.py`, which drives the arm by holding keys. So don't write a second control program. Split `sim/drive.py` so the keyboard and jog logic talk to an arm, and give it two arms:

| | Sim arm | Real arm |
| --- | --- | --- |
| Read joint angles | `d.qpos` | Read present position from all six servos at once (sync read) |
| Send targets | `d.ctrl` | Write all six goal positions at once (sync write) |
| Gripper squeeze | Force range set to 1/3 stall | Torque limit (register 48) on the gripper set to about 333 |

Send targets about 50 times a second: one sync write and one sync read per cycle is a few milliseconds at 1 Mbaud. Every target is clamped to the joint ranges before it's sent.

Two things the sim gives the real arm for free:

- **A live twin.** Copy the real joint angles into the sim's viewer each cycle, so you see the real arm in the sim with its readout of load and torque.
- **A pose check.** The sim README found that 1 % of poses inside the joint limits hit the arm itself and 19 % hit the table. Before sending a target, put it in the sim, run `mj_forward` and check for contacts. If it would collide, don't send it.

`autopick.py` presses keys rather than setting angles, so once the real arm takes the same key input, the auto pick can run on it too. Try it only after manual picks work, with the block and box in the same spots as the sim.

**Done when:** you can pick up the 200 g block and drop it in the box by keyboard, with the sim viewer following the real arm.

## Phase 6: Measure the real arm and correct the sim

The sim is honest about what it guessed: its servo gains, damping, friction and rotor inertia come from SO-101's model, not from Talos. The real arm can now measure them:

- **Holding load.** Hold the ready pose and a few others, read each servo's load, and compare with the sim's holding torques and `calcs/`. The shoulder is the one to watch.
- **Speed costs torque.** The sim predicts a 90° shoulder lift with 200 g stalls in 1.0 s but keeps up in 1.5 s. Time both on the real arm and log load and lag.
- **Sag and backlash.** Command a pose, measure where the tip really is, and compare with the sim. Neither printed-part flex nor backlash is in the model.
- **Grip.** The sim drops blocks wider than 25 mm at friction 0.3. Try 20, 25, 30 and 40 mm blocks with and without grip pads.
- **Heat.** Hold 200 g at full reach for 30 s and log temperature (the build plan's R3 test). The servo cuts its own torque after an overload, which drops the arm, so know where that point is before demonstrating.

Feed what you measure back into `sim/build_model.py` and rerun `sim/test_autopick.py`. Log every test to CSV in `control/logs/` for the build log.

**Done when:** the sim's predictions for holding load and the timed shoulder lift are within about 15 % of the measurements, or the differences are explained.

## Where the code goes

```
control/
  NEXT_STEPS.md        this file
  scan.py              phase 1: find servos
  set_ids.py           phase 2: one servo at a time
  calibrate.py         phase 3: zero pose, directions, hardware limits
  calibration.json     saved by calibrate.py, committed
  bus.py               open the port, sync read and write, unit conversions
  real_arm.py          the real arm, same methods as the sim arm
  logs/                CSV from tests
```

`sim/drive.py` gains an option such as `--real COM5` to drive the real arm with the sim as its twin.

## Servo registers we'll use

From Waveshare's [ST servo memory table](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation). Two-byte values are low byte first. EEPROM values persist and need the lock opened; RAM values reset at power-off.

| Address | Name | Memory | Units and notes |
| --- | --- | --- | --- |
| 5 | ID | EEPROM | 0–253, factory 1 |
| 6 | Baud rate | EEPROM | 0 = 1,000,000 (factory) |
| 9, 11 | Min and max angle limit | EEPROM | Counts, 0–4095 |
| 13 | Max temperature | EEPROM | °C, factory 70 |
| 31 | Position offset | EEPROM | Set by writing 128 to register 40 |
| 33 | Mode | EEPROM | 0 = position. Leave it at 0 |
| 40 | Torque | RAM | 0 off, 1 on, 128 = call this position 2048 |
| 41 | Acceleration | RAM | 0–254, 8.7°/s² each, 0 = no limit |
| 42 | Goal position | RAM | Counts |
| 46 | Goal speed | RAM | Counts per second (50 = 0.73 rpm) |
| 48 | Torque limit | RAM | 0–1000, 0.1 % of max |
| 55 | EEPROM lock | RAM | 0 = writes stick, 1 = locked |
| 56 | Present position | RAM | Counts |
| 60 | Present load | RAM | 0.1 % of max, bit 10 is the direction |
| 62 | Present voltage | RAM | 0.1 V |
| 63 | Present temperature | RAM | °C |
| 65 | Status | RAM | Error bits: voltage, temperature, encoder, current, overload |
| 69 | Present current | RAM | 6.5 mA each |

## What's optional

Inverse kinematics (type a point, the arm goes there) and the accuracy grid test (R5) aren't needed for manual control. Add them after phase 5 if there's time: the sim already has the kinematics, so it can check an IK solution before the real arm moves.

## Sources

- [LeRobot SO-101 setup](https://huggingface.co/docs/lerobot/so101): ID setup one motor at a time, calibration, jumpers on B
- [Waveshare ST servo memory table](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation): register addresses and units
- [Waveshare bus servo adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) and its [wiring example](https://docs.waveshare.com/Bus_Servo_Adapter_A/Product-Wiring-Example): jumper B for USB
- [feetech-servo-sdk on PyPI](https://pypi.org/project/feetech-servo-sdk): Feetech's Python SDK
- [FTServo_Python](https://github.com/StarCycle/FTServo_Python): SDK source with STS examples (`sms_sts`)
- [Seeed Feetech servo hub](https://wiki.seeedstudio.com/feetech_servo/) and the [STS3215 datasheet](https://core-electronics.com.au/attachments/uploads/sts3215-smart-servo-datasheet-translated.pdf)
