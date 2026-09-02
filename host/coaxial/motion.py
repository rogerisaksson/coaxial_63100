"""Motion on top of the drive: stepper, servo, velocity - and the price.

Host-side loops over the link. A write lands in ~7 ms, so nothing here
runs faster than a few tens of hertz - an arm's outer loop, a gimbal, a
thrust command. A quad's rate loop belongs where 50 kHz lives (`Drive/`);
what runs here is what a flight controller or a motion planner would do
with this board on the other end of a wire.

    stage = device.gates.arm(...)            # arming stays ONE call,
    with device.motion.stepper(amps=2) as m: # nothing here ever arms
        m.to(90)                             # mech degrees, slewed
    with device.motion.servo(amps=2) as m:
        m.to(45)                             # the shaft sensor closes it
    with device.motion.velocity(amps=4) as m:
        m.rpm(1500)                          # sensorless, coaxial.loop

Every block enters its drive mode on `with` and leaves the drive OFF
however it ends. Angles are MECHANICAL degrees throughout; the electrical
turn is `pole_pairs` times faster and stays this module's business.
"""
import math
import time

from .errors import RigError
from .sensorless import TWO_PI


class _Mode:

    """Enter a drive mode on `with`; the drive is OFF on the way out."""

    def __init__(self, device):
        self.device = device
        self.drive = device.drive
        p = self.drive.params()
        self.poles = int(p['motor_pole_pairs'] or 1)
        self._params = p

    def __enter__(self):
        self._start()
        return self

    def __exit__(self, *exc):
        self.drive.off()

    def _check(self):
        """Raise with the board's own word if the stage tripped.

        Every verb polls this each pass. Without it a trip mid-move -
        overcurrent, the supply, the stage taken away - was commanded
        against for the rest of the block, since nothing here read the
        fault the state reply carries. The drive holds its own hardware
        against a trip; this is so the LOOP stops asking, and the caller
        hears why rather than watching a dead shaft not move."""
        fault = self.drive.state()['fault']
        if fault:
            raise RigError('the drive tripped mid-move - %s. The stage is '
                           'down; the block is over.' % fault)

    def _slew_to(self, theta_e, deg_s, pitch=0.25):
        """Walk the command to `theta_e`, `pitch` mech degrees a write:
        the spring is never asked to span more than a few degrees at
        once. THE slew - the stepper's move and the servo's correction
        are this one loop, not two copies of it."""
        step = math.radians(pitch) * self.poles
        pause = pitch / deg_s
        while abs(theta_e - self._theta_e) > step:
            self._check()
            self._theta_e += math.copysign(step, theta_e - self._theta_e)
            self.drive.setpoint(theta=self._theta_e)
            time.sleep(pause)
        self._theta_e = theta_e
        self.drive.setpoint(theta=self._theta_e)

    def _energize(self, amps, steps=6, settle=0.05):
        """HOLD, with the current RAMPED - a stepper driver's soft
        energize. Snapping full current onto an unknown rotor is a yank
        of up to half a pole that an underdamped rotor rides straight
        through, pole after pole; grown over a few link writes it
        detents into the nearest pole and stays. Where the rotor ends
        up is where angles count from."""
        self._theta_e = self.drive.state()['theta_hat']
        self.drive.setpoint(id_ref=amps / steps, iq_ref=0.0,
                            theta=self._theta_e, omega_target=0.0)
        self.drive.mode('hold')
        for k in range(2, steps + 1):
            time.sleep(settle)
            self._check()              # a trip mid-ramp must not be
            self.drive.setpoint(id_ref=amps * k / steps)   # stepped past
        time.sleep(2.0 * settle)


class Stepper(_Mode):

    """The PMSM as its own microstepper.

    HOLD holds a current vector at a commanded angle and the rotor is
    dragged along by the load-angle spring, `amps kt sin(delta)`. Open
    loop: overload it and it slips poles silently, exactly as a stepper
    does - the servo below is the same move with the slip measured out.

    `to()` slews rather than steps: the command walks at `deg_s` in
    increments the link can carry, so the spring is never asked to span
    more than a few degrees at once.
    """

    def __init__(self, device, amps, deg_s=90.0):
        super().__init__(device)
        self.amps, self.deg_s = float(amps), float(deg_s)
        self._theta_e = self._zero = 0.0

    def _start(self):
        # Soft energize, then count from wherever the rotor detented:
        # an incremental frame - absolute needs an encoder offset
        # nothing has commissioned yet.
        self._energize(self.amps)
        self._zero = self._theta_e

    @property
    def position(self):
        """Where the COMMAND is, mech degrees from where the block
        began - the rotor is trusted to follow, the stepper bargain."""
        return math.degrees((self._theta_e - self._zero) / self.poles)

    def to(self, degrees, pitch=0.25):
        """Slew the command to `degrees`, `pitch` mech degrees a write."""
        self._slew_to(self._zero + math.radians(degrees) * self.poles,
                      self.deg_s, pitch)
        return self.position

    def step(self, n=1, degrees=1.8):
        """`n` full steps of `degrees` each - the classic vocabulary."""
        return self.to(self.position + n * degrees)


class Servo(_Mode):

    """Position over the shaft sensor, closed once per MOVE.

    Not a rate servo: this link corrects at tens of hertz and the
    load-angle spring rings at tens of hertz, so a per-pass loop samples
    its own resonance aliased and pumps it - measured, six clean passes
    wound the rotor through a pole slip into a freewheel. The honest
    shape is a stepper with the slip measured out: slew smoothly, let
    the ring die, read the sensor, correct what the load stole. The
    A1335 must see a magnet; on the stand-in it reads the same virtual
    rotor the drive torques.
    """

    def __init__(self, device, amps, deg_s=90.0, settle=0.35):
        super().__init__(device)
        self.amps, self.deg_s = float(amps), float(deg_s)
        self.settle = float(settle)
        self._theta_e = 0.0
        self._shaft0 = None
        self._last = None
        self._turns = 0.0
        self._error = float('nan')

    def _start(self):
        # Soft energize like the stepper, then the shaft AFTER the
        # detent is zero: every angle is relative to it. The zero is a
        # MEAN - the detent rings and one read froze up to the ring's
        # amplitude into the frame for the life of the block.
        self._energize(self.amps)
        self._shaft0 = 0.0
        self._shaft0 = self._measure()

    def _shaft(self):
        """The shaft, unwrapped, mech degrees from where the block began."""
        now = self.device.angle.state()['degrees']
        if self._last is not None:
            delta = now - self._last
            if delta > 180.0:
                self._turns -= 360.0
            elif delta < -180.0:
                self._turns += 360.0
        self._last = now
        return now + self._turns - (self._shaft0 or 0.0)

    @property
    def error(self):
        """Target minus shaft, mech degrees, from the last correction."""
        return self._error

    def _slew(self, by_degrees):
        self._slew_to(self._theta_e + math.radians(by_degrees) * self.poles,
                      self.deg_s)

    def _measure(self, reads=9, span=0.2):
        """The shaft as a short MEAN: the ring is symmetric about the
        equilibrium the load set, so its average is the position while a
        single read is wherever the oscillation happened to be."""
        got = 0.0
        for _ in range(reads):
            got += self._shaft()
            time.sleep(span / reads)
        return got / reads

    def to(self, degrees, tol=0.5, tries=4):
        """Drive the shaft to `degrees`: move, settle, measure, correct.

        Each correction is what the LOAD stole - a spring wound by
        holding torque, or poles slipped outright. Raises after `tries`
        corrections still outside `tol`: a stalled arm is a fact, not a
        return code.
        """
        for _ in range(int(tries)):
            self._check()
            got = self._measure()
            self._error = degrees - got
            if abs(self._error) <= tol:
                return got
            self._slew(self._error)
            time.sleep(self.settle)
        got = self._measure()
        self._error = degrees - got
        if abs(self._error) <= tol:
            return got
        raise RigError('the shaft stayed %.1f deg short of %.1f after %d '
                       'corrections - load past %.1f A of holding torque, '
                       'or no magnet in front of the sensor'
                       % (self._error, degrees, tries, self.amps))


class Velocity(_Mode):

    """Sensorless speed under `coaxial.loop`'s own law - the ESC's job.

    The drive commutates itself at 50 kHz from the record's tune
    (`auto_tune.ipynb` is what writes it); this loop reads `omega_hat`
    and writes `iq_ref` at link rate. `j`/`b` default to the SMALLEST
    plausible machine - the stand-in's own - because an overstated j
    scales kp by the same factor and the discrete loop flips sign and
    doubles: measured, j five times the plant took +900 rpm asked to
    -1552 delivered. Understating only makes a big machine sluggish;
    identify the real pair (`speed_loop.ipynb`) at the bench.
    """

    #: `load_k` is the LOOP's knowledge - the propeller law its
    #: feedforward leans on. It moves no air: on the stand-in the plant's
    #: drag is fed separately (`model_param(load=...)` from a `watch`,
    #: as the notebooks do), and at the bench the air is the air.
    def __init__(self, device, amps, hz=3.0, j=2e-5, b=1e-5, load_k=0.0,
                 rate_hz=25.0):
        super().__init__(device)
        from .loop import Signals, SpeedLoop
        from .motor import Parameters, Propeller
        p = self._params
        motor = Parameters(
            name='the record', r=p['motor_r_uohm'], ld=p['motor_ld_nh'],
            lq=p['motor_lq_nh'], lam=p['motor_lambda_uvs'],
            poles=self.poles, j=j, b=b, measured=False)
        self.loop = SpeedLoop(hz, float(amps),
                              motor, load=Propeller(load_k) if load_k else None)
        self.bus = Signals()
        self.pause = 1.0 / float(rate_hz)

    def _start(self):
        self.drive.setpoint(id_ref=0.0, iq_ref=0.0)
        self.drive.mode('sensorless')
        time.sleep(0.2)                        # the injection lock

    @property
    def rpm_now(self):
        """The observer's speed, mechanical rpm."""
        return (self.drive.state()['omega_hat'] / self.poles
                * 60.0 / TWO_PI)

    def rpm(self, target, seconds=1.5, accel_rpm_s=None, watch=None):
        """Ramp to `target` rpm and serve the loop for `seconds` after.

        The reference slews at `accel_rpm_s` (default: reach the target
        in a third of `seconds`), so the current stays a control action
        rather than a step. `watch(self)` runs once a pass - a notebook's
        logger, a mission's guard. Returns the rpm it settled at.
        """
        w_ref = self.bus.w_ref
        w_target = float(target) * TWO_PI / 60.0
        if accel_rpm_s is None:
            # Reach the target in a third of the block, whole rpm terms.
            accel_rpm_s = (abs(target - w_ref * 60.0 / TWO_PI) * 3.0
                           / max(seconds, 0.1))
        slew = accel_rpm_s * TWO_PI / 60.0
        end = time.monotonic() + seconds
        last = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            dt, last = now - last, now
            move = max(-slew * dt, min(slew * dt, w_target - w_ref))
            w_ref += move
            self.bus.w_ref = w_ref
            self.bus.a_ref = move / dt if dt else 0.0
            # One state read a pass, and the fault rides it: a trip here
            # is a runaway or an overcurrent, the one place stopping the
            # loop matters most. No extra round trip - omega_hat and the
            # fault come off the same reply.
            st = self.drive.state()
            if st['fault']:
                raise RigError('the drive tripped mid-spin - %s. The stage '
                               'is down; the loop is over.' % st['fault'])
            self.bus.w = st['omega_hat'] / self.poles
            self.loop(self.bus, dt)
            self.drive.setpoint(iq_ref=self.bus.iq_ref)
            if watch is not None:
                watch(self)
            time.sleep(self.pause)
        return self.rpm_now

    def stop(self, seconds=1.0):
        """Back to rest, the same ramp down."""
        return self.rpm(0.0, seconds)


class Motion:

    """The factory `device.motion` answers with. Three verbs, one rule:
    the stage is armed FIRST, by you, through `device.gates.arm()` -
    these helpers refuse to be the second place arming lives."""

    def __init__(self, device):
        self._device = device

    def _armed(self):
        if not self._device.gates.armed():
            raise RigError('the stage is not armed - device.gates.arm() '
                           'is the one place that does it, and nothing '
                           'in coaxial.motion will be the second')

    def stepper(self, amps, deg_s=90.0):
        self._armed()
        return Stepper(self._device, amps, deg_s)

    def servo(self, amps, deg_s=90.0, settle=0.35):
        self._armed()
        return Servo(self._device, amps, deg_s, settle)

    def velocity(self, amps, hz=3.0, **kw):
        self._armed()
        return Velocity(self._device, amps, hz, **kw)
