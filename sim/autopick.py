"""Pick up the block by pressing the driving keys, the way a person would.

The autopilot never sets joint angles itself. Every physics step it chooses which driving keys to
hold, and drive.py runs them through the same step_arm() as the keyboard. So the arm moves at jog
speed under the same servo limits, and the log is a key sequence you could repeat by hand.

The plan is the straight-down pick from check_model.py: fixed finger 3 mm inside the block's near
face, lower until the tool point is 25 mm above the table, close with the jaw at its torque limit,
then lift. It works out the pan and wrist roll from wherever the block is, so you can move it.

Given a box to drop into, it then carries the block over the box and lets go. The carry never
touches the wrist, since tilting the wrist while holding is what drops the block (see Lift below).
"""

from collections import Counter

import mujoco
import numpy as np

GAP = 0.003                    # m between the fixed finger and the block on the way down, as in check_model.py
Z_LOW = 0.025                  # m, tool-point height to grip at. The 18 mm of finger past it stays off the table
Z_HIGH = np.arange(0.080, 0.0549, -0.005)  # m, approach heights to try, highest first. 80 mm is check_model.py's;
                               # straight down from there can need a degree more wrist flex than its 95 deg limit
WAYPOINT = 0.005               # m between waypoints going down and up, so the gripper moves close to straight
PHASE_TIMEOUT = 10.0           # s
SETTLE_SPEED = np.radians(3)   # rad/s: a move ends once every joint is slower than this,
SETTLE_MAX = 1.0               # s    or after this long, since a loaded joint can keep creeping
LIFT = np.radians(20)          # elbow up this far to lift: about 50 mm
PICKED_MM = 20                 # raised at least this far after the hold counts as picked up
GRIPPING_FOR = 0.15           # s at the jaw's torque limit before letting go of the close key
HOLD_FOR = 1.0                 # s to hold the block up before calling it picked
CLEAR = 0.012                  # m the held block's lowest corner and the fingertip stay above the box walls
BACK_AWAY = np.radians(10)     # elbow up this far after letting go, so the fingers clear the box


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class PlanError(Exception):
    """The block is somewhere the arm can't pick it from. The message says why, for the screen."""


class Phase:
    """One step. goals: list of {joint index: angle}, reached in turn. make(d) fills goals when the step
    starts, for steps that depend on how the earlier ones went. check(d) ends a "hold" step: it returns
    None to go on, or (state, message) to finish there."""

    def __init__(self, name, kind, goals=(), make=None, check=None, seconds=HOLD_FOR):
        self.name, self.kind, self.goals, self.make, self.check = name, kind, list(goals), make, check
        self.seconds = seconds
        self.waypoint, self.t, self.settled, self.count = 0, 0.0, 0.0, 0.0


class AutoPick:
    """One pick attempt. Call keys(d) before every physics step; read state, message and report().

    drop: None to pick up and hold, or the box to drop into, as dict(x, y, yaw, inner, top): its
    centre, which way it's turned, the inside half-width and the height of the wall tops, in m and rad.
    """

    def __init__(self, m, d, jog_speed, squeeze, drop=None):
        self.m = m
        self.drop = drop
        self.scratch = mujoco.MjData(m)
        self.names = [m.actuator(i).name for i in range(m.nu)]
        self.jaw = self.names.index("gripper")
        self.step = jog_speed * m.opt.timestep
        self.squeeze = squeeze
        self.block = m.body("block").id
        self.half = m.geom("block").size[0]
        self.low, self.high = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
        self._measure_arm()
        self.phases = self._plan(d)
        self.log = [Counter() for _ in self.phases]   # per phase: (joint, +1/-1) -> seconds held
        self.i, self.state, self.message = 0, "running", ""
        self.z0 = None

    # ------------------------------------------------------------ planning

    def _measure_arm(self):
        """Link lengths from the model itself, so a rebuild after a CAD change carries through."""
        m, scratch = self.m, mujoco.MjData(self.m)
        mujoco.mj_kinematics(m, scratch)
        self.H = scratch.body("upper_arm_link").xpos[2]          # shoulder axis above the table
        self.L1 = m.body("forearm_link").pos[0]                  # shoulder -> elbow
        self.L2 = m.body("wrist_link").pos[0]                    # elbow -> wrist flex
        self.D = m.body("gripper_link").pos[0] + m.site("tool").pos[0]  # wrist flex -> tool point
        # Which way wrist roll turns the jaw, seen from above. At roll 0 the jaw opens away from the base.
        scratch.qpos[:5] = (0, *self._ik(0.19, Z_LOW), 0.3)
        mujoco.mj_kinematics(m, scratch)
        opens = scratch.body("gripper_link").xmat.reshape(3, 3)[:, 2]
        self.roll_sign = np.sign(np.arctan2(opens[1], opens[0]))

    def _ik(self, r, z):
        """Shoulder, elbow and wrist flex for a straight-down gripper with the tool point at radius r,
        height z. Same maths as ik_top_down() in check_model.py."""
        wx, wz = r, z + self.D - self.H
        c3 = (wx * wx + wz * wz - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if abs(c3) > 1:
            raise PlanError(f"The block is out of reach ({r * 1000:.0f} mm from the base axis).")
        q3 = np.arccos(c3)
        q2 = np.pi / 2 - (np.arctan2(wz, wx) + np.arctan2(self.L2 * np.sin(q3), self.L1 + self.L2 * np.cos(q3)))
        return q2, q3, np.pi - q2 - q3

    def _grip_face(self, yaw, toward):
        """Which side face of the block the fixed finger goes against, as the direction (rad, seen
        from above) the jaw opens in. A cube has four choices, yaw + k*90 deg."""
        return min((yaw + k * np.pi / 2 for k in range(4)), key=lambda a: abs(wrap(a - toward)))

    def _plan(self, d):
        c, R = d.xpos[self.block].copy(), d.xmat[self.block].reshape(3, 3)
        up = int(np.argmax(np.abs(R[2])))   # which of the block's axes is vertical
        if abs(R[2, up]) < np.cos(np.radians(5)) or c[2] > self.half + 0.005:
            raise PlanError("The block isn't sitting flat on the table. Press R, or B to place it.")
        side = R[:, (up + 1) % 3]
        face = self._grip_face(np.arctan2(side[1], side[0]), np.arctan2(c[1], c[0]))
        opens = np.array([np.cos(face), np.sin(face)])
        tool = c[:2] - (self.half - 0.0005 + GAP) * opens  # the pad face sits 0.5 mm in from the tool point
        pan, r = np.arctan2(tool[1], tool[0]), np.hypot(*tool)
        roll = self.roll_sign * wrap(face - pan)
        self.target = dict(x=c[0], y=c[1], face=face, pan=pan, roll=roll, r=r)

        limit = None
        for z_high in Z_HIGH:   # the highest approach that keeps every joint inside its limits
            heights = np.arange(z_high, Z_LOW - 1e-9, -WAYPOINT)
            arm = {z: dict(zip((1, 2, 3), self._ik(r, z))) for z in heights}
            limit = next((self.names[j] for q in arm.values() for j, v in {0: pan, 4: roll, **q}.items()
                          if not self.low[j] + 0.01 < v < self.high[j] - 0.01), None)
            if limit is None:
                break
        else:
            raise PlanError(f"Can't reach the block {r * 1000:.0f} mm out at {np.degrees(pan):.0f} deg: "
                            f"past the {limit} limit.")
        self.target["z_high"] = z_high

        width = 2 * self.half * 1000   # mm; opening from check_model.py: clears the block with 10 deg spare
        opening = np.radians(min(100.0, np.degrees(np.arcsin(min(0.95, (width + 10) / 46))) + 10))
        grip = arm[heights[-1]]
        if self.drop:
            self._check_box_plan(c, R, opens, np.array([pan, grip[1], grip[2] - LIFT, grip[3], roll]))
        return [
            # Up to the approach height first: the ready pose reaches out low enough to sweep a far block,
            # or open the jaw into one sitting under it. Elbow first, since the ready fingertip is only
            # 14 mm up and moving the shoulder or wrist before the elbow has lifted dips it into the table.
            Phase("Raise the arm", "move", [{2: arm[heights[0]][2]}, arm[heights[0]]]),
            Phase("Open the jaw", "move", [{self.jaw: max(opening, d.ctrl[self.jaw])}]),
            Phase("Turn to face the block", "move", [{0: pan, 4: roll}]),
            Phase("Lower around the block", "move", [arm[z] for z in heights[1:]]),
            Phase("Close the jaw", "close"),
            # Elbow up only (K). The pivoting jaw lets go if the gripper tilts the wrong way or too fast while
            # lifting: wrist flex up (U) alone, S + K together, and S alone past about 200 mm all drop it.
            # K alone held it at every spot tested. A straight-up lift needs U, so this one swings out a little.
            Phase("Lift the block", "move", [{2: grip[2] - LIFT}]),
        ] + (self._drop_phases(opening) if self.drop else [Phase("Hold it up", "hold", check=self._check_held)])

    # ------------------------------------------------------------ dropping in the box

    def _drop_phases(self, opening):
        return [
            Phase("Check the grip", "hold", check=lambda d: r if (r := self._check_held(d))[0] == "failed" else None,
                  seconds=0.4),
            # One joint at a time, wrist untouched: elbow up a bit more if the block would pass low over a
            # wall, pan, then elbow and shoulder in whichever order keeps the block above the walls.
            Phase("Turn to the box", "move", make=lambda d: self._carry_goals(d, turn=True)),
            # Planned again after the turn, since a long swing can shift the block in the jaw.
            Phase("Reach over the box", "move", make=lambda d: self._carry_goals(d, turn=False)),
            Phase("Let go", "move", [{self.jaw: opening}]),
            Phase("Back away", "move", make=lambda d: [{2: d.ctrl[2] - BACK_AWAY}]),
            Phase("Check the box", "hold", check=self._check_in_box, seconds=0.8),
        ]

    def _block_in_hand(self, q, rel):
        """Where the held block's centre is, and the fingertip's height, with the arm at q (5 joints).
        rel is the block centre in the gripper's frame."""
        s = self.scratch
        s.qpos[:5] = q
        mujoco.mj_kinematics(self.m, s)
        g = s.body("gripper_link")
        return g.xpos + g.xmat.reshape(3, 3) @ rel, s.site("tip").xpos[2]

    def _path_clear(self, poses, rel, tilt, clear, samples=10):
        """True if, along straight joint-space moves through poses, the held block's lowest corner and
        the fingertip stay above `clear` wherever the block is over or beside the box. Elsewhere the
        walls can't be hit, so the block may hang lower. tilt: the block's orientation in the gripper frame."""
        box = self.drop
        near = (box["inner"] + 0.005) * np.sqrt(2) + 1.5 * self.half + 0.01
        for a, b in zip(poses, poses[1:]):
            for k in np.linspace(0, 1, samples):
                p, tip_z = self._block_in_hand(a + (b - a) * k, rel)
                if np.hypot(p[0] - box["x"], p[1] - box["y"]) > near:
                    continue
                R = self.scratch.body("gripper_link").xmat.reshape(3, 3) @ tilt
                if min(p[2] - self.half * np.abs(R[2]).sum(), tip_z) < clear:
                    return False
        return True

    def _solve_carry(self, q, rel, tilt, margin=0.0):
        """Pan, shoulder and elbow (wrist as it is) that hang the block over the box's centre, and the
        order to move them in, so that the block and fingertip stay above the walls the whole way.
        First the elbow comes up by `up` (0, 10 or 20 deg, the least that works) if the block would
        otherwise swing in low over a wall, then pan, then elbow before shoulder or the other way round.
        margin adds clearance, for checking ahead with only an estimate of where the block sits in the jaw.
        Returns (pose, joint order, up); raises PlanError if there's no such move."""
        box = self.drop
        clear = box["top"] + CLEAR + margin   # a full CLEAR: under load the arm sags a few mm below its targets
        for up, extra in ((u, e) for u in np.radians([0, 10, 20]) for e in (0.0, 0.01, 0.02, 0.03, 0.045)):
            raised = q.copy()
            raised[2] -= up
            if raised[2] < self.low[2] + 0.01:
                continue
            target = np.array([box["x"], box["y"], clear + 1.5 * self.half + extra])
            for seed in (0.0, 0.3, -0.3):   # a few starts, in case Newton wanders off from one
                x, err = raised.copy(), np.inf
                x[0] = np.arctan2(box["y"], box["x"])   # start facing the box: a long swing can lose Newton
                x[1] += seed
                for _ in range(60):   # Newton's method on (pan, shoulder, elbow), numerical Jacobian
                    p, _ = self._block_in_hand(x, rel)
                    err = target - p
                    if np.linalg.norm(err) < 2e-4:
                        break
                    J = np.empty((3, 3))
                    for k in range(3):
                        dx = x.copy()
                        dx[k] += 1e-5
                        J[:, k] = (self._block_in_hand(dx, rel)[0] - p) / 1e-5
                    x[:3] += np.clip(np.linalg.lstsq(J, err, rcond=None)[0], -0.2, 0.2)
                if np.linalg.norm(err) > 1e-3 or not all(self.low[j] + 0.01 < x[j] < self.high[j] - 0.01
                                                         for j in range(3)):
                    continue
                turned = np.r_[x[0], raised[1:]]
                for order in ((2, 1), (1, 2)):
                    mid = turned.copy()
                    mid[order[0]] = x[order[0]]
                    if self._path_clear([q, raised, turned, mid, x], rel, tilt, clear):
                        return x, order, up
        r = np.hypot(box["x"], box["y"]) * 1000
        raise PlanError(f"Can't carry the block over the box {r:.0f} mm out without hitting it. "
                        f"Move the box (B), to about 180-300 mm from the base.")

    def _check_box_plan(self, c, R_block, opens, q_lift):
        """Before starting: is the box clear of the block, and can the arm reach over it? Uses where the
        block should end up in the jaw (pushed about 4 mm onto the fixed finger), measured again later."""
        box = self.drop
        reach = box["inner"] + 0.005 + 1.5 * self.half + 0.02   # box centre to block centre when they'd touch
        if np.hypot(c[0] - box["x"], c[1] - box["y"]) < reach:
            raise PlanError("The block is in or right next to the box. Move one of them apart (B).")
        grip = q_lift.copy()
        grip[2] += LIFT
        s = self.scratch
        s.qpos[:5] = grip
        mujoco.mj_kinematics(self.m, s)
        g = s.body("gripper_link")
        Rg = g.xmat.reshape(3, 3)
        rel, tilt = Rg.T @ (np.r_[c[:2] - 0.004 * opens, c[2]] - g.xpos), Rg.T @ R_block
        # The lift swings the block outward a little: it mustn't swing into the box on the way up.
        for k in np.linspace(0, 1, 10):
            p, _ = self._block_in_hand(grip + (q_lift - grip) * k, rel)
            if p[2] - 1.8 * self.half < box["top"] + 0.005 and np.hypot(p[0] - box["x"], p[1] - box["y"]) < reach:
                raise PlanError("The box is in the way of the lift. Move it further from the block (B).")
        self._solve_carry(q_lift, rel, tilt, margin=0.006)

    def _carry_goals(self, d, turn):
        """Plan the carry from where the block actually is in the jaw, and return the goals for the
        turn (elbow up if needed, then pan) or for the reach (elbow and shoulder, in order). If planning
        again for the reach fails, the plan made for the turn still stands."""
        Rg = d.body("gripper_link").xmat.reshape(3, 3)
        rel = Rg.T @ (d.xpos[self.block] - d.body("gripper_link").xpos)
        tilt = Rg.T @ d.xmat[self.block].reshape(3, 3)
        try:
            self.carry, self.carry_order, up = self._solve_carry(d.qpos[:5].copy(), rel, tilt)
        except PlanError as e:
            if turn:
                self._finish("failed", str(e), None)
                return [{}]
            up = 0.0
        raise_first = [{2: d.ctrl[2] - up}] if up else []
        if turn:
            return raise_first + [{0: self.carry[0]}]
        return raise_first + [{j: self.carry[j]} for j in self.carry_order]

    def _check_held(self, d):
        raised = (d.xpos[self.block][2] - self.z0) * 1000
        if raised > PICKED_MM:
            return "done", f"Picked up: block {raised:.0f} mm off the table."
        return "failed", f"The block slipped out (raised {raised:.0f} mm)."

    def _check_in_box(self, d):
        box = self.drop
        off = d.xpos[self.block][:2] - (box["x"], box["y"])
        c, s = np.cos(box["yaw"]), np.sin(box["yaw"])
        local = np.array([c * off[0] + s * off[1], -s * off[0] + c * off[1]])  # into the box's frame
        if np.all(np.abs(local) < box["inner"]) and d.xpos[self.block][2] < box["top"]:
            return "done", f"Dropped in the box, {np.hypot(*off) * 1000:.0f} mm from its centre."
        return "failed", f"Missed the box: the block landed {np.hypot(*off) * 1000:.0f} mm from its centre."

    # ------------------------------------------------------------ driving

    def keys(self, d):
        """The keys to hold for the next physics step, {joint name: +1, -1 or 0}. Advances the plan."""
        pressed = dict.fromkeys(self.names, 0)
        if self.state != "running":
            return pressed
        dt, ph = self.m.opt.timestep, self.phases[self.i]
        ph.t += dt
        if ph.t > PHASE_TIMEOUT:
            return self._finish("failed", f"Stuck at '{ph.name}' for {PHASE_TIMEOUT:.0f} s.", pressed)

        if ph.make and not ph.goals:
            ph.goals = ph.make(d)
            if self.state != "running":
                return pressed

        if ph.kind == "move":
            if ph.waypoint < len(ph.goals):
                for j, goal in ph.goals[ph.waypoint].items():
                    err = goal - d.ctrl[j]
                    if abs(err) > self.step / 2:
                        pressed[self.names[j]] = int(np.sign(err))
                if not any(pressed.values()):
                    ph.waypoint += 1
            else:
                ph.settled += dt
                still = np.max(np.abs(d.qvel[:self.jaw])) < SETTLE_SPEED
                if (still and ph.settled > 0.1) or ph.settled > SETTLE_MAX:
                    self._next()

        elif ph.kind == "close":
            if self.z0 is None:
                self.z0 = d.xpos[self.block][2]
            if ph.count < GRIPPING_FOR:
                pressed["gripper"] = -1
                squeezing = d.actuator_force[self.jaw] <= -0.9 * self.squeeze
                ph.count = ph.count + dt if squeezing else 0.0
                if d.ctrl[self.jaw] <= self.low[self.jaw] + 1e-6 and not squeezing:
                    return self._finish("failed", "The jaw closed on nothing: the block wasn't between the fingers.",
                                        dict.fromkeys(self.names, 0))
            else:
                ph.settled += dt   # let go of the key; drive.py's jaw keeps squeezing on its own
                if ph.settled > 0.4:
                    self._next()

        elif ph.kind == "hold":
            if ph.t > ph.seconds:
                result = ph.check(d)
                if result:
                    return self._finish(*result, pressed)
                self._next()

        for joint, sign in pressed.items():
            if sign:
                self.log[self.i][joint, sign] += dt
        return pressed

    def _next(self):
        self.i += 1

    def _finish(self, state, message, pressed):
        self.state, self.message = state, message
        return pressed

    def stop(self, why):
        if self.state == "running":
            self._finish("stopped", why, None)

    # ------------------------------------------------------------ reporting

    def report(self, key_name):
        """Rows for the screen: (phase label, keys held). key_name(joint, sign) gives a key's label."""
        rows = []
        for i, (ph, held) in enumerate(zip(self.phases, self.log)):
            mark = ">" if (i == self.i and self.state == "running") else ("-" if i > self.i else " ")
            keys = "   ".join(f"{key_name(j, s)} {f'{t:.2f} s' if t >= 0.05 else 'tap'}" for (j, s), t in held.items())
            if i < self.i and not keys:
                keys = "(already there)"
            rows.append((f"{mark} {i + 1} {ph.name}", keys))
        return rows

    def goals_text(self, i):
        """The angles a phase drives toward, in degrees."""
        ph = self.phases[i]
        if not ph.goals:
            return {"close": "close until the jaw is squeezing", "hold": f"hold still {ph.seconds:.1f} s",
                    "move": "worked out when this step starts"}[ph.kind]
        return ", ".join(f"{self.names[j]} {np.degrees(v):.1f}" for j, v in ph.goals[-1].items())

    def summary(self, key_name):
        t = self.target
        lines = [f"Auto pick: block at x {t['x'] * 1000:.0f}, y {t['y'] * 1000:.0f} mm. "
                 f"Pan {np.degrees(t['pan']):.1f} deg, wrist roll {np.degrees(t['roll']):.1f} deg."]
        for i, (label, keys) in enumerate(self.report(key_name)):
            lines.append(f"  {label[2:]:<26} {keys or '-':<40} -> {self.goals_text(i)}")
        lines.append(f"  {self.state.upper()}: {self.message}")
        return "\n".join(lines)
