"""Motion on top of the drive: stepper, servo, velocity - and the price."""
import math
import time

from coaxial.control.controller import Direct, Loop, Polled, Slew, SpeedPI
from coaxial.errors import RigError
from coaxial.model.motor import Parameters, Propeller
from coaxial.model.sensorless import RAD_S_PER_RPM

#: Mechanical degrees: a full turn, and the jump past which a reading
#: has wrapped rather than the shaft having moved.
TURN = 360.0
HALF_TURN = TURN / 2


def _turned(delta):
    """The whole turn a reading stepped across, if it did: a jump past half a
    turn is the sensor wrapping, not the shaft.
    """
    if delta > HALF_TURN:
        return -TURN
    if delta < -HALF_TURN:
        return TURN
    return 0.0


class _Mode:

    """Enter a drive mode on `with`; the drive is OFF on the way out."""

    def __init__(self, device):
        self.device = device
        self.drive = device.drive
        p = self.drive.params()
        self.poles = int(p['motor_pole_pairs'] or 1)
        self._params = p

    def _start(self):
        """What the mode does on the way in; each verb's own."""
        raise NotImplementedError

    def __enter__(self):
        self._start()
        return self
    def __exit__(self, *exc):
        self.drive.off()

    def _check(self):
        """Raise with the board's own word if the stage tripped."""
        fault = self.drive.state()['fault']
        if fault:
            raise RigError('the drive tripped mid-move - %s. The stage is '
                           'down; the block is over.' % fault)

    def _slew_to(self, theta_e, deg_s, pitch=0.25):
        """Walk the command to `theta_e`, `pitch` mech degrees a write: the
        spring is never asked to span more than a few degrees at once.
        """
        step = math.radians(pitch) * self.poles
        pause = pitch / deg_s
        while abs(theta_e - self._theta_e) > step:
            self._check()
            self._theta_e += math.copysign(step, theta_e - self._theta_e)
            self.drive.write(theta=self._theta_e)
            time.sleep(pause)
        self._theta_e = theta_e
        self.drive.write(theta=self._theta_e)

    def _energize(self, amps, steps=6, settle=0.05):
        """HOLD, with the current RAMPED - a stepper driver's soft energize."""
        self._theta_e = self.drive.state()['theta_hat']
        self.drive.write(id_ref=amps / steps, iq_ref=0.0,
                         theta=self._theta_e, omega_target=0.0)
        self.drive.hold()
        for k in range(2, steps + 1):
            time.sleep(settle)
            self._check()              # a trip mid-ramp must not be
            self.drive.write(id_ref=amps * k / steps)   # stepped past
        time.sleep(2.0 * settle)


class Stepper(_Mode):

    """The PMSM as its own microstepper."""

    def __init__(self, device, amps, deg_s=90.0):
        super().__init__(device)
        self.amps, self.deg_s = float(amps), float(deg_s)
        self._theta_e = self._zero = 0.0

    def _start(self):
        # Soft energize, then count from wherever the rotor detented: an
        # incremental frame - absolute needs an encoder offset nothing has
        # commissioned yet.
        self._energize(self.amps)
        self._zero = self._theta_e

    @property
    def position(self):
        """Where the COMMAND is, mech degrees from where the block began - the
        rotor is trusted to follow, the stepper bargain.
        """
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

    """Position over the shaft sensor, closed once per MOVE."""

    #: A measurement reads the shaft for SPAN seconds, READ_GAP apart -
    #: on the wire a read is its own round trip and sets the pace; the
    #: stand-in answers at once, and the gap keeps a mean from being a
    #: burst. A shaft seen moving more than RING across it is RINGING
    #: and is read for RING_SPAN instead: the slew's own pitch, the least
    #: a held shaft is ever commanded to move.
    SPAN = 0.2
    RING_SPAN = 1.0
    RING = 0.25
    READ_GAP = 0.01

    def __init__(self, device, amps, deg_s=90.0, settle=0.35):
        super().__init__(device)
        self.amps, self.deg_s = float(amps), float(deg_s)
        self.settle = float(settle)
        self._theta_e = 0.0
        self._shaft0 = None
        self._last = None
        self._turns = 0.0
        self._error = float('nan')
        self._swing = float('nan')

    def _start(self):
        # Soft energize like the stepper, then the shaft AFTER the detent is
        # zero: every angle is relative to it.
        self._energize(self.amps)
        self._shaft0 = 0.0
        self._shaft0 = self._measure()

    def _shaft(self):
        """The shaft, unwrapped, mech degrees from where the block began."""
        now = self.device.angle.state()['degrees']
        if self._last is not None:
            self._turns += _turned(now - self._last)
        self._last = now
        return now + self._turns - (self._shaft0 or 0.0)

    @property
    def error(self):
        """Target minus shaft, mech degrees, from the last correction."""
        return self._error

    @property
    def swing(self):
        """How far the shaft moved, peak to peak in mech degrees, while the
        last measurement watched it: the ring a held rotor carries.
        """
        return self._swing

    def _slew(self, by_degrees):
        self._slew_to(self._theta_e + math.radians(by_degrees) * self.poles,
                      self.deg_s)

    def _measure(self):
        """The shaft as a MEAN over its ring, never a read of it."""
        began = time.monotonic()
        deadline = began + self.SPAN
        total, count = 0.0, 0
        low = high = None
        while True:
            got = self._shaft()
            total += got
            count += 1
            low = got if low is None else min(low, got)
            high = got if high is None else max(high, got)
            if high - low > self.RING:
                deadline = began + self.RING_SPAN
            if time.monotonic() >= deadline:
                break
            time.sleep(self.READ_GAP)
        self._swing = high - low
        return total / count

    def _arrived(self, degrees, tol):
        """Within `tol` of `degrees` AND STILL THERE a settle later."""
        got = self._measure()
        self._error = degrees - got
        if abs(self._error) > tol:
            return False
        time.sleep(self.settle)
        again = self._measure()
        self._error = degrees - again
        return abs(self._error) <= tol

    def to(self, degrees, tol=0.5, tries=4):
        """Drive the shaft to `degrees`: move, settle, measure, correct."""
        for _ in range(int(tries)):
            self._check()
            if self._arrived(degrees, tol):
                return degrees - self._error
            self._slew(self._error)
            time.sleep(self.settle)
        self._check()
        if self._arrived(degrees, tol):
            return degrees - self._error
        raise RigError('the shaft stayed %.1f deg short of %.1f after %d '
                       'corrections, ringing %.1f deg peak to peak - load '
                       'past %.1f A of holding torque, or no magnet in '
                       'front of the sensor'
                       % (self._error, degrees, tries, self._swing,
                          self.amps))


class Velocity(_Mode):

    """Sensorless speed, a `Loop`: the drive's state in, `SpeedPI` by default, `iq_ref` out.

    `load_k` is the regulator's feedforward, not a load: the stand-in's drag is
    `model.configure(load=...)` from a `watch`. `regulator=`, `estimator=` swap the parts.
    """

    def __init__(self, device, amps, hz=3.0, j=2e-5, b=1e-5, load_k=0.0,
                 rate_hz=25.0, regulator=None, estimator=None):
        super().__init__(device)
        p = self._params
        motor = Parameters(
            name='the record', r=p['motor_r_uohm'], ld=p['motor_ld_nh'],
            lq=p['motor_lq_nh'], lam=p['motor_lambda_uvs'],
            poles=self.poles, j=j, b=b, measured=False)
        self.slew = Slew(w=0.0)
        self.loop = Loop(
            Polled(self.drive.state), self.drive,
            regulator or SpeedPI(hz, float(amps), motor,
                                 load=Propeller(load_k) if load_k else None),
            estimator or Direct(w=('omega_hat', 1.0 / self.poles)),
            prefilters=(self.slew,), rate_hz=rate_hz)

    def _start(self):
        self.drive.write(id_ref=0.0, iq_ref=0.0)
        self.drive.on('sensorless')
        time.sleep(0.2)                        # the injection lock

    @property
    def rpm_now(self):
        """The observer's speed, mechanical rpm."""
        return (self.drive.state()['omega_hat'] / self.poles
                / RAD_S_PER_RPM)

    def rpm(self, target, seconds=1.5, accel_rpm_s=None, watch=None):
        """Ramp to `target` rpm - in a third of the block unless `accel_rpm_s` - and serve
        the loop for `seconds`; `watch(velocity)` after every pass."""
        now_rpm = self.slew.at.get('w', 0.0) / RAD_S_PER_RPM
        if accel_rpm_s is None:
            accel_rpm_s = abs(target - now_rpm) * 3.0 / max(seconds, 0.1)
        self.slew.rates['w'] = accel_rpm_s * RAD_S_PER_RPM
        self.loop.move(seconds, (lambda _: watch(self)) if watch else None,
                       w=float(target) * RAD_S_PER_RPM)
        return self.rpm_now

    def stop(self, seconds=1.0):
        """Back to rest, the same ramp down."""
        return self.rpm(0.0, seconds)


class Motion:

    """The factory `device.motion` answers with."""

    def __init__(self, device):
        self._device = device

    def _armed(self):
        if not self._device.gates.is_on():
            raise RigError('the stage is not armed - device.gates.on() '
                           'is the one place that does it, and nothing '
                           'in coaxial.control.motion will be the second')

    def stepper(self, amps, deg_s=90.0):
        self._armed()
        return Stepper(self._device, amps, deg_s)

    def servo(self, amps, deg_s=90.0, settle=0.35):
        self._armed()
        return Servo(self._device, amps, deg_s, settle)

    def velocity(self, amps, hz=3.0, **kw):
        self._armed()
        return Velocity(self._device, amps, hz, **kw)
