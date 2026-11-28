"""Each drive's gearbox: its rotor free across the play, the box wound up past it, the link turned.

The rotor runs free across the play, winds the box and the structure up past it against the
mesh's friction, and the link is turned by that wind-up alone - every driven joint's box while
`physics.BOXED`, else the joints' with a spring in series (`drives.series`: the robot's ankle's leaf), the
rest rigid (`sprung`). A sprung joint's board closes its torque loop on what it has - its rotor's
encoder, its phase current and the link's rate as the law's chain has it, nothing across the leaf -:
an observer of the torque its box passes on.

    boxes = Boxes(np, boxed)                            # the numbers a joint (`machine.body.drives`)
    passed = boxes.step(np, ctrl, q, qd, brake, dt, feed)   # the links' torques; the rotors stepped on
    boxes.moved(q)                                      # the links where the world's step left them
    boxes.delta, boxes.w, boxes.hat                     # each rotor's angle past its link, its speed, its board's estimate
"""
from machine.body import drives, peripherals
from machine.body.figure import JOINTS

_mj = peripherals.mujoco()

#: A board's torque counts as taking up the play from this, N m (`peripherals.mujoco()`).
PLAY_NM = float(_mj['play_nm'])

#: A series spring's torque loop (`drives.series`, `Boxes.step`): its rotor driven TRACK times the
#: shortfall of the torque its box passes on, as the board observes it, against the torque its pass
#: feeds forward (the stack's: what the floor is to give), and the box's damping at the spring's
#: winding rate that torque's rate asks less the rotor's rate past the link's (a PD, its time the
#: box's own c/k); the pass's own PD on the motor's angle goes straight on the rotor - a free joint's
#: spring winds nothing: looped on the whole command a released sole's leaf snapped its foot 75 deg
#: toe-down and the march's stop fell; the drive clipped to the board's peak. 0 the drive's torque alone on the rotor,
#: the built leaf's (1040 N m/rad; under 1.24 of the robot's weight in a walking turn it unwound as the load
#: left and threw the robot into the air). 12 the leaf at the text's 0.55 M g L (143 N m/rad): a P loop on
#: the leaf alone at 3 had the push-off's 60 N m 60-75 ms late (the rotor winding 17.6 deg at 6
#: rad/s, the robot's hip past its clamp 197 times in 6 s) and fell at 5-15 unclipped; at 12 the robot walks (29
#: passes, v 1.25, the drive at its peak 53 % of the time: the leaf asks more of the rotor than 197 N m
#: on 0.293 kg m^2 gives) and marches (93, v 1.40), the robot's leaf storing 10-12 J a step; 3 and 6 fall
#: (237, 179), 18 and 24 fall (299, 95) and the moves play falls at every gain: the law's ankle asks
#: and heel rise are a stiff joint's. Subtracting the box's own damping and mesh at the rotor's rate
#: from the observed torque, to loop on the spring's alone, flipped 2 x 3.9 N m x TRACK at every sign
#: change of that rate against the observer's lag: the peak 84 % of the time, 98 passes.
TRACK = float(_mj['track'])
#: The board's observer of the torque its box passes on runs this many times faster than the loop's
#: own mode (sqrt(k (1 + TRACK) / J)), 1/s: its rotor's generalized momentum J w against the integral
#: of its own drive less that estimate, no rate of w differenced. At 143 N m/rad and TRACK 12 that is
#: 180 1/s: at 1500 the ask's 3 ms steps went through to the robot's hip (203 passes, it fell) and the
#: lead that cancels the observer's pole (0.6 of it) 82; slower, 100 fell in the walk's stop and 60 was 54. On
#: the built leaf 180 fell at 4.7 s (its mode 174 1/s lagged into a swing), 400 walked, 1500 53.
OBSERVE_X = float(_mj['observe_x'])
#: The command's rate is read through this first-order lag, s: a pass's step spread over the world's steps.
RATE_S = float(_mj['rate_s'])
#: The mesh's friction rounded over this speed, rad/s; the rotors stepped this many times within
#: the world's step (`peripherals.mujoco`).
MESH_RAD_S = float(_mj['mesh_rad_s'])
SUBSTEPS = int(_mj['substeps'])


class Boxes:

    """The gearboxes' numbers a joint and their state: half each play, rad (`drives.play`); the
    wind-up past it, N m/rad (`drives.wind`) - in series with the joint's spring where it has one
    (`drives.series`) -, and its damping, N m s/rad (`drives.BOX_ZETA`); the mesh's friction, N m
    (`drives.mesh`); the rotor's inertia, kg m^2 (`drives.armature`, off the joint's armature while
    stepped); the rotor's angle past the link's and its speed; the board's estimate of the torque
    its box passes on (`hat`), its observer's rate (`observe`, OBSERVE_X) and integral, its last
    drive, the command and its rate (RATE_S), the board's peak (`drives.peak`); `sprung` which
    joints are stepped: every driven one `boxed`, else those with a spring."""

    def __init__(self, np, boxed=False):
        self.driven = np.array(_mj['driven'], bool)
        self.series = np.array(_mj['series'])
        self.sprung = self.driven & (bool(boxed) | (self.series > 0.0))
        self.gap = np.array(_mj['play'])
        wound = np.array(_mj['wind'])
        self.k = np.where(self.series > 0.0, 1.0 / (1.0 / np.maximum(wound, 1e-9) + 1.0 / np.maximum(self.series, 1e-9)), wound)
        self.j = np.array(_mj['armature'])
        self.c = 2.0 * np.array(_mj['zeta']) * np.sqrt(self.k * self.j)
        self.track = np.array(_mj.get('tracks', TRACK)) * (self.series > 0.0)
        self.observe = np.where(self.sprung, OBSERVE_X * np.sqrt(self.k * (1.0 + self.track) / np.maximum(self.j, 1e-9)), 0.0)
        self.mesh = np.array(_mj['mesh'])
        self.shock = np.array(_mj['shock'])
        self.peak = np.array(_mj['box_peak'])
        self.delta, self.w, self.zero = np.zeros(len(JOINTS)), np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        self.held, self.q_was = float(_mj['rigid_play_rad']), np.zeros(len(JOINTS))
        self.q_step, self.went = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        self.hat, self.p_int, self.drive = np.zeros(len(JOINTS)), np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        self.applied, self.tracked = np.zeros(len(JOINTS)), np.zeros(len(JOINTS), bool)
        self.asked, self.rate = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        self.v_mesh, self.inv_m = np.full(len(JOINTS), MESH_RAD_S), np.zeros(len(JOINTS))

    def reset(self, q):
        """Every rotor on its joint at `q`, still, its board's observer at rest: a board's encoder
        reads its hold there."""
        self.delta[:] = self.w[:] = self.zero[:] = 0.0
        self.hat[:] = self.p_int[:] = self.drive[:] = self.applied[:] = self.asked[:] = self.rate[:] = 0.0
        self.tracked[:] = False
        self.q_was[:] = q

    def rigid(self, np, q):
        """The boards' encoders on rigid boxes: each held within `held` of its joint at `q`, rad,
        every board's - as the rigid world was modelled before the boxes (`drives.RIGID_PLAY_DEG`)."""
        self.delta[:] = np.where(self.sprung, self.delta, np.clip(self.delta + self.q_was - q, -self.held, self.held))
        self.q_was[:] = q
        return self.delta

    def ahead(self, np, torque):
        """How far past its setpoint a board on the motor's encoder asks its rotor, rad, so the
        link lands on it: the box wound up by `torque` and the play that torque has taken up.
        On the play alone the robot's toes dragged 20 mm at lift where 1 rigid."""
        k = np.where(self.sprung, self.k, 1.0)
        return np.where(self.sprung, drives.AHEAD_WIND * torque / k
                        + drives.AHEAD_PLAY * self.gap * np.tanh(torque / PLAY_NM), 0.0)

    def step(self, np, ctrl, q, qd, brake, dt, feed=None, limit=None, kp=None):
        """The links' torques from the motors' `ctrl` with the links at `q` rad and `qd` rad/s
        through the step - the step's middle's, the start's and half a step of the last
        acceleration: against the start's the mesh's slope and the damping on a stale speed
        added (c + mesh / MESH_RAD_S) dt qdd to a drive's torque, 0.13 kg m^2 at the waist -:
        what each box passes on - the wind-up past the play, its damping, the mesh's friction -,
        the rotors stepped `dt` on by what is left of their motors' torques against it, `brake`
        each rotor's braking, N m s/rad (a short's), implicit. A sprung joint's board drives its rotor
        with the command, TRACK times the shortfall of the torque its box passes on - its observer's
        (`hat`: the rotor's momentum J w against the integral of the board's own drive less the
        estimate, `observe`) - against `feed`, the torque its pass feeds forward (the stack's; the
        command itself unsaid), and the box's damping at the spring's winding rate that torque's rate
        (RATE_S) asks less the rotor's rate past the link's (`qd` the law's), within its peak; the
        command's rest, the pass's PD on the motor's angle, goes on the rotor as it is. `moved` sets
        each rotor's angle past its link right after. The boxes' state stepped in place: the robot's world's steps in
        C step the same arrays (wbc_world.c, `tools.cores.world.boxed`)."""
        top = self.peak if limit is None else np.minimum(self.shock, limit)
        feed = ctrl if feed is None else np.clip(feed, -top, top)
        on = self.sprung
        j = np.where(on, self.j, 1.0)
        k = np.maximum(self.k, 1.0)
        h = dt / SUBSTEPS
        mean, mdrv = np.zeros(len(ctrl)), np.zeros(len(ctrl))
        self.q_step[:], self.went[:] = q, dt * qd
        lim = self.gap + self.shock / k
        self.rate += (dt / RATE_S) * ((feed - self.asked) / dt - self.rate)
        self.asked[:] = feed
        pd_on = np.zeros(len(ctrl), bool) if kp is None else (kp > 0.0)
        trk = np.where((top > 0.0) & (~pd_on) & ((feed != 0.0) | (self.rate != 0.0)), self.track, 0.0)
        cd = np.where(trk > 0.0, np.maximum(trk * self.c,
                      2.0 * np.sqrt((self.k * (1.0 + trk)) * j) - (1.0 + trk) * self.c), 0.0)
        for _ in range(SUBSTEPS):
            rel = self.w - qd
            past = np.sign(self.delta) * np.maximum(np.abs(self.delta) - self.gap, 0.0)
            passed = np.where(past != 0.0, self.k * past + self.c * rel
                              + self.mesh * np.tanh(rel / self.v_mesh), 0.0)
            passed = np.where(on, np.clip(passed, -self.shock, self.shock), passed)
            self.p_int[:] = np.where(on, self.p_int + h * (self.drive - brake * self.w - self.hat), 0.0)
            self.hat[:] = self.observe * (self.p_int - j * self.w)
            cap = np.minimum(top, np.maximum(np.abs(self.hat), np.abs(feed)) + PLAY_NM)
            drive = ctrl + trk * np.clip(feed - self.hat, -cap, cap) + cd * (self.rate / k - rel)
            self.drive[:] = np.where(on, np.clip(drive, -top, top), ctrl)
            self.w[:] = np.where(on, (self.w + h * (self.drive - passed) / j) / (1.0 + h * brake / j), qd)
            self.delta[:] = np.where(on, np.clip(self.delta + h * (self.w - qd), -lim, lim), self.delta)
            mean += passed / SUBSTEPS
            mdrv += self.drive / SUBSTEPS
        self.applied[:], self.tracked[:] = mdrv, on & (trk != 0.0)
        return np.where(on, mean, ctrl)

    def moved(self, q):
        """The links landed at `q` after the step: each rotor's angle past its link is the rotor's
        turn less the link's. Taken as `qd` dt, the link's turn left dt qd on every wind-up - a
        knee at 2 rad/s wound 2 mrad of its 4.4 mrad play by its speed alone - and its rotor's
        second difference carried the link's jerk."""
        self.delta += (self.went - (q - self.q_step)) * self.sprung
