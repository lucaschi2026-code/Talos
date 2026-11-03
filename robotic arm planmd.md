# Robotic Arm Build Plan

Design and build an original 5-axis desktop arm with a gripper, demo-ready by Dec 6, 2026 and fully documented by Dec 31 — about 8 hours a week each and roughly CA$350 total.

## At a glance

Build your own arm, not a kit: an original SolidWorks design driven by your own torque calculations, with smart bus servos, your own inverse kinematics, and measured accuracy numbers at the end.

- **The arm:** 5 axes (base yaw, shoulder, elbow, wrist pitch, wrist roll) plus a gripper, about 350 mm reach, 3D-printed links, Feetech STS3215 serial bus servos, controlled from a laptop in Python.
- **Why bus servos:** each one reports its own position, load and temperature, so you get real test data instead of "it moved." The open-source SO-101 arm uses the same servos, which proves the parts work; your design is the original part.
- **What it proves to employers:** requirements → hand calcs → CAD → prototype → test → redesign. That loop, with numbers attached, is what separates a project from a tutorial.
- **The twist:** a designed and measured shoulder counterbalance (or belt reduction), plus a repeatability test that ends in a single number you can put on your resume (for example "±1.5 mm repeatability").
- **Do not copy the SO-101 STLs.** Study it as a benchmark, then design your own. A cloned kit is weak on a resume; an original design that you can defend in an interview is strong.

## Requirements

Six specs, each one testable with a number. Write them in week 1 and do not change them without writing down why.

| # | Requirement | Target | How you test it |
| --- | --- | --- | --- |
| R1 | Degrees of freedom | 5 axes + gripper | Each joint moves through its full range |
| R2 | Reach | ≥ 350 mm from base axis to gripper tip | Tape measure at full extension |
| R3 | Payload at full reach | ≥ 200 g, held 30 s, no servo overload flag | Weighted object, log servo load and temperature |
| R4 | Repeatability | ≤ ±2 mm | Return to the same point 30 times, measure with a dial indicator |
| R5 | Positioning accuracy | ≤ ±5 mm over a 200 × 200 mm grid | Command 9 printed grid targets, measure error at each |
| R6 | Budget | ≤ CA$400 total | Running BOM spreadsheet |

Targets are deliberately modest. Beating a modest spec with measured data looks better than missing an ambitious one.

## Scope tiers

Finish the MVP by Nov 8 no matter what; the target tier is the real goal; touch stretch only once target is done.

| Tier | Done when | Due |
| --- | --- | --- |
| MVP | 3 axes (base, shoulder, elbow) move under keyboard joint control; forward kinematics shows the tip position | Nov 8 |
| Target | All 5 axes + gripper; inverse kinematics moves the tip to typed x, y, z; scripted pick-and-place of 3 blocks; R1–R6 tested and logged | Nov 29 |
| Stretch | USB webcam finds coloured blocks (OpenCV) and the arm sorts them by colour on its own | Dec 6 |

The stretch goal makes a great demo video, but the target tier alone is already a strong resume project.

## Schedule

The real deadline is Dec 6, not late December: Fall 2026 finals run Dec 10–22 and campus closes Dec 24–31, so all printing and building must happen before exams. Over the break you only edit the video and write things up.

Assumes about 8 hours a week each. Midterm weeks are marked lighter; shift them to match your own midterm dates.

| Week | Milestone (visible output) | Mechanical lead | Electronics and software lead |
| --- | --- | --- | --- |
| Oct 5–11 | Requirements signed off; **parts ordered by Oct 9** | Benchmark SO-101 and 2 other arms; first concept sketches | Order servos, driver, PSU; set up GitHub repo and build log |
| Oct 12–18 | Link lengths and torque budget fixed | Torque spreadsheet; choose counterbalance or belt reduction; print servo-fit test gauge | Python env + Feetech SDK; write the servo bench-test script |
| Oct 19–25 (lighter) | CAD assembly v1 with mates and a motion study | Base, shoulder and upper-arm parts in SolidWorks | Bench-test servos on arrival: read position, load, temperature |
| Oct 26–Nov 1 (lighter) | v1 base + shoulder + elbow printed | Print at RPC; assemble lower arm | Joint control, homing and calibration routine |
| Nov 2–8 | **MVP:** 3 axes under keyboard control | Forearm, wrist and gripper CAD | Forward kinematics; plot tip position vs CAD |
| Nov 9–15 | Full 5-axis arm + gripper assembled | Print and assemble wrist and gripper; cable routing | Inverse kinematics (analytic); move to typed x, y, z |
| Nov 16–22 | v2 redesign of weak parts | Fix what flexed or cracked; FEA on the worst link; tune counterbalance | Smooth trajectories; scripted pick-and-place |
| Nov 23–29 | **Target tier:** R1–R6 tested | Repeatability and payload tests | Accuracy grid test; log all data to CSV |
| Nov 30–Dec 6 | **Feature freeze Dec 6**; raw demo footage shot | Engineering drawings of 2 key parts; photos | Stretch: webcam colour sorting, or polish |
| Dec 7–9 | Buffer only | Fix anything broken | Clean up repo README |
| Dec 10–22 | Exams. No project work | — | — |
| Dec 23–31 | Video, portfolio page, resume bullets done | Write the design and test sections | Edit video; finish README |

Ordering in week 1 matters most: servos shipped from overseas can take 2–3 weeks, and every later week depends on them.

## Who does what

Split by lead, not by exclusive ownership: each of you must be able to explain the whole arm in an interview.

- **Mechanical lead:** requirements, torque calcs, SolidWorks CAD, FEA, printing, assembly, drawings.
- **Electronics and software lead:** wiring and power, servo bus, calibration, kinematics, test scripts, data logging.
- **Shared on purpose:** each person designs at least 2 printed parts in SolidWorks (so both can claim CAD), and each writes at least one test script.
- **One 1-hour sync a week** (same day every week): demo what changed, update the build log, decide the next week's tasks.
- **Review each other's work:** the mech lead checks the kinematics against CAD; the software lead checks the torque numbers. Catching each other's mistakes is a good interview story too.

## Budget and parts list

About CA$350 total, roughly CA$175 each. Prices are approximate conversions; check current prices and shipping times before you order.

| Part | Qty | Approx. cost (CA$) | Notes |
| --- | --- | --- | --- |
| [Feetech STS3215 bus servo, 12 V, 30 kg·cm](https://shop.wowrobo.com/products/feetech-sts3215-servo-12v-30kg-high-torque-servo-for-so-arm100) | 7 (1 spare) | 145 | About US$14–16 each, cheaper in packs |
| Waveshare serial bus servo driver board | 1 | 20 | USB to the servo bus; also feeds servo power |
| 12 V power supply, ≥ 5 A, barrel plug | 1 | 25 | Match the servo voltage exactly |
| Bearings: one large thrust bearing or lazy-Susan ring for the base, 608 bearings for joints | 1 + 10 | 25 | Base bearing carries the arm's tipping moment |
| GT2 belt and pulleys, or extension springs | 1 set | 20 | Shoulder reduction or counterbalance |
| M2/M3 screw, nut and heat-set insert kit | 1 | 25 | Heat-set inserts make printed parts last |
| PETG filament (if not printing free at RPC) | 1 kg | 30 | PETG over PLA for stiffer, tougher links |
| Dial indicator + magnetic base | 1 | 35 | For the repeatability test; borrow if you can |
| Table clamps, wire, cable sleeve | — | 15 |  |
| USB webcam (stretch only) | 1 | 30 | Skip unless you reach the stretch tier |
| **Total** |  | **≈ 340–370** |  |

You use your own laptop as the controller, so no Raspberry Pi is needed. The [Rapid Prototyping Centre](https://uwaterloo.ca/rapid-prototyping-centre/) in E5-2002 has dozens of self-serve 3D printers (256 mm cube build volume) open to any Waterloo student, so plan parts to fit that volume.

## Engineering calculations

A first-pass statics check shows the shoulder needs help: with the arm straight out, it needs about 1.6 N·m, over half the servo's 2.94 N·m stall torque. Fixing that is your best design story.

Worst case is the arm horizontal at full reach. Treat it as a cantilever (ME 100 / PHYS 115 statics) and sum each mass times its horizontal distance from the shoulder axis:

```latex
\tau_{shoulder} = g \sum_i m_i \, x_i
```

Example numbers (assumed: 150 mm upper arm, 150 mm forearm, wrist and gripper centre at 330 mm, tip at 380 mm). Replace them with masses from your CAD and a kitchen scale.

| Mass on the shoulder | m (kg) | x (m) | Torque (N·m) |
| --- | --- | --- | --- |
| Payload at gripper tip | 0.200 | 0.38 | 0.75 |
| Wrist assembly (2 servos + gripper servo + gripper) | 0.200 | 0.33 | 0.65 |
| Forearm link | 0.040 | 0.225 | 0.09 |
| Elbow servo | 0.055 | 0.15 | 0.08 |
| Upper-arm link | 0.040 | 0.075 | 0.03 |
| **Total** |  |  | **≈ 1.59** |

The servo's 30 kg·cm rating is about 2.94 N·m at stall. A common rule of thumb is to keep continuous load under about a third of stall (≈ 1.0 N·m) so the servo does not overheat or droop. Two fixes to compare:

1. **2:1 GT2 belt reduction at the shoulder:** doubles available torque, so the load falls to about 27 % of stall. Costs some speed and adds backlash.
2. **Spring counterbalance:** cancels the arm's own weight (about 0.85 N·m of the total), leaving mostly the payload, about 25 % of stall. No speed loss, but the spring must be sized and tested.

Pick one, size it, then **measure it**: the servo reports its load, so log shoulder load with and without the fix. "Cut shoulder servo load by X % with a spring counterbalance" is a ready-made resume line. Repeat the same check at the elbow.

## Testing plan

Four tests turn the arm into numbers; run each on v1 and again on v2 so you can show the improvement.

1. **Repeatability (R4):** clamp the arm, put the dial indicator against the gripper tip, send the arm away and back to the same point 30 times. Report the mean and standard deviation in mm.
2. **Accuracy (R5):** print a 3 × 3 grid of targets 100 mm apart, command each with inverse kinematics, measure the miss with calipers. Report the worst and average error.
3. **Payload (R3):** hold 100, 150, 200 and 250 g at full reach for 30 s each. Log servo load, temperature and joint sag (commanded vs read-back position).
4. **Shoulder fix:** log shoulder load through the same motion with and without the counterbalance or belt.

Save every run as a CSV in the repo, and make one plot per test in Python. Those plots go straight into your portfolio.

## Risks and fallbacks

The biggest risk is the schedule, not the engineering: lose two weeks and you hit exams with a half-built arm.

| Risk | Early sign | Fallback |
| --- | --- | --- |
| Servos arrive late | No tracking number by Oct 16 | Order from a faster Canadian seller even at a higher price; keep doing CAD and code against one servo |
| Shoulder overloads or droops | Load readout near limit, servo hot after 1 min | Use the fix from Engineering calculations; shorten reach to 300 mm |
| Printed parts don't fit or crack | Servo-fit gauge fails; cracks at screw holes | Calibrate tolerances with the gauge first; heat-set inserts; PETG; thicker walls |
| Midterms eat two weeks | Milestones slipping in late Oct | Drop the stretch tier; MVP still lands by Nov 8 |
| Inverse kinematics takes too long | No x, y, z moves by Nov 15 | Use taught waypoints (move by hand, record angles, replay) for the demo |
| One partner does most of the work | Uneven commits and build-log entries | Raise it at the weekly sync; rebalance tasks early |
| Wrong supply voltage fries servos | — | Label the PSU; check voltage with a multimeter before the first power-up |

## Turning it into resume value

The resume line is only as strong as the evidence behind it, so document every week, not just at the end.

- **Build log:** one short entry per week in the repo: what you did, a photo or CAD screenshot, what broke and how you fixed it.
- **Deliverables by Dec 31:** a 60–90 s demo video, a GitHub repo with README, test plots and CAD exports, and a one-page portfolio entry (problem, requirements, key decisions, test results, what you'd change).
- **Resume bullet shapes** (fill in only your real numbers):
  - Designed a 5-DOF, 3D-printed robotic arm in SolidWorks from hand torque calculations; achieved ±\_\_ mm repeatability over 30 trials.
  - Designed a spring counterbalance for the shoulder joint, cutting measured servo load by \_\_ % at full reach.
  - Implemented inverse kinematics and serial-bus servo control in Python, reaching ±\_\_ mm accuracy over a 200 × 200 mm workspace.
- **Interview story (STAR):** pick the shoulder problem — the calculation that showed it, the fix you chose and why, and the before/after data.

Apply with what you have: if WaterlooWorks postings open before the arm is finished, list it as in progress with the MVP results.

## This weekend

Five steps, about 3 hours total, and the project is moving.

- [ ] Meet for an hour: agree on the requirements table, who leads what, and your weekly sync day
- [ ] Agree a budget split and order the servos, driver board and power supply
- [ ] Make the GitHub repo with a README, a build-log file and a BOM sheet
- [ ] Each sketch one arm concept on paper (link lengths, where the servos sit) and compare them
- [ ] Look through the SO-101 repo as a benchmark: note 3 things to copy and 3 to do differently

## Sources

- [Waterloo Fall 2026 final examination period](https://uwaterloo.ca/important-dates/graduate/2026-2027/final-examination-period) — exams Dec 10–23
- [Waterloo important dates, Fall 2026](https://acadcalendar.com/uwaterloo-important-dates/) — campus holiday closure Dec 24–31
- [Rapid Prototyping Centre, E5-2002](https://uwaterloo.ca/rapid-prototyping-centre/) — self-serve printers and access
- [Engineering Student Shops](https://uwaterloo.ca/engineering-student-shops/) — machine shop and laser cutting
- [SO-ARM100 / SO-101 repository](https://github.com/adityachandrath/SO-ARM100) — benchmark design and servo choices
- [SO-101 specs overview](https://www.ay-robots.com/robots/so-101) — STS3215 servos, parts cost
- [Feetech STS3215 12 V servo listing](https://shop.wowrobo.com/products/feetech-sts3215-servo-12v-30kg-high-torque-servo-for-so-arm100) — torque rating and price
