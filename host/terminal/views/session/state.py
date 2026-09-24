"""The session dash's state: the session, its two activities, what each undoes."""
import time
from typing import Any

from coaxial.devices import angle
from terminal.ui.screen import QUIET, steady


#: FIELD, from the A1335's register map.
ANGLE_REG_FIELD = 0x2A

#: Either side of a rail change, before anything is believed. The board's
#: own settle for the same reason, and the same number.
SETTLE_S = 0.5

#: Where the gate drivers start. Half is the duty that puts equal volt-seconds
#: on both halves of every leg, so with all three at it there is no voltage
#: between phases and no phase current - the state the whole thermal
#: calibration was taken in.
DEFAULT_DUTY = 0.50

#: Which legs. Named because a leg at a time is how a per-leg thermal
#: question gets asked, and three at once is how a bulk one does.
DEFAULT_PHASES = ('U', 'V', 'W')


def _start_imu(rig):
    """Enable the rotation vector for the dashboard. True if it took."""
    if steady(rig.board.imu.settled, seconds=6.0) is not True:
        return False
    try:
        with rig.board.imu.configuring():
            rig.board.imu.feature(0x05, 20000)
        return True
    except QUIET:
        return False


class Session:

    """The rig, what is running on it, and what to put back."""

    def __init__(self, rig):
        self.rig = rig
        self.running = {}          # activity name -> what to undo, in order
        self.note = ''
        self.duty = DEFAULT_DUTY
        self.phases = DEFAULT_PHASES
        self.legs = DEFAULT_PHASES     # which of them are switching now
        self.plan: Any = None
        # Both fetched once.
        self.scaling = steady(rig.board.analog.scaling)
        self.field = self._field()
        #: The rail as the session found it - main() sets it after probing,
        #: teardown puts it back. None: never learned, leave it alone.
        self.afe_found = None
        #: Whether the session enabled the IMU's rotation vector, and so
        #: owes its disable on the way out.
        self.imu_started = False

    def _field(self):
        """The magnetic field at the A1335, in gauss. None if it refused."""
        if steady(self.rig.board.angle.hold) is None:
            return None
        try:
            got = steady(self.rig.board.angle.read, ANGLE_REG_FIELD)
        finally:
            steady(self.rig.board.angle.resume)
        return None if got is None else angle.gauss(got['value'])

    def set_duty(self, duty):
        """Move the duty, and push it if the stage is already switching."""
        self.duty = min(1.0, max(0.0, duty))
        if 'switching' in self.running and self.push() is None:
            return
        self.note = 'duty %.0f %%' % (100.0 * self.duty)

    def push(self):
        """Write the duty to the legs that are live, and zero to the rest."""
        load = dict(('Phase %s' % leg,
                     self.duty if leg in self.legs else 0.0)
                    for leg in DEFAULT_PHASES)
        if steady(self.rig.write, analog=load) is None:
            self.note = 'the duty write did not take'
            return None
        return load

    def sample(self):
        """Stand down, let the thermal observer measure, and go back to
        switching.
        """
        rig = self.rig
        if steady(rig.gates.off) is None:
            self.note = 'could not stand down for a sample'
            return False

        taken = None
        if steady(rig.board.afe.on) is not None:
            time.sleep(SETTLE_S)
            taken = steady(rig.board.thermal.state)
            steady(rig.board.afe.off)
            time.sleep(SETTLE_S)

        if steady(rig.gates.on, bypass_sto=True,
                  ignore_interlock=True) is None:
            self.note = 'sampled, but the stage would not re-arm'
            return False
        self.push()

        if taken and taken.get('ntc') is not None:
            self.note = 'sampled: NTC %.2f C' % taken['ntc']
        return True

    def set_legs(self, legs):
        """Which legs switch. The rest are held at zero duty."""
        self.legs = tuple(legs)
        if 'switching' in self.running:
            self.push()

    def toggle(self, activity):
        if activity.name in self.running:
            self.stop(activity.name)
            return
        undo = activity.start(self)
        if undo is not None:
            self.running[activity.name] = undo
            self.note = '%s started' % activity.name

    def stop(self, name):
        # One try per step: a failed first undo must not skip the rest, which
        # is how a disarm gets missed and a stage is left switching.
        for undo in self.running.pop(name, ()):
            try:
                undo()
            except QUIET as exc:
                self.note = '%s: %s' % (name, exc)
        self.note = '%s stopped' % name

    def stop_all(self):
        """Stop everything, and say what each one was."""
        said = []
        for name in list(self.running):
            self.stop(name)
            said.append((name, UNDONE.get(name, 'stopped')))
        return said


class Switching:

    """Three legs at a duty, and the AFE off because the gate is inverted."""

    name = 'switching'
    key = 's'
    what = 'gate drivers, three legs'
    undone = 'three legs to zero duty, stage disarmed'

    def start(self, session):
        rig = session.rig
        if steady(rig.board.afe.off) is None:
            session.note = 'could not put the AFE down - not arming'
            return None
        if steady(rig.gates.on, bypass_sto=True, ignore_interlock=True) is None:
            session.note = 'the stage refused to arm'
            return None

        load = session.push()
        if load is None:
            steady(rig.gates.off)
            session.note = 'armed but the duty did not take - disarmed again'
            return None

        return (lambda: rig.write(analog=dict.fromkeys(load, 0.0)),
                rig.gates.off)


class Acquiring:

    """A DAQ task, so the link has load and the counters move."""

    name = 'acquiring'
    key = 'd'
    what = 'DAQ running, every channel'
    undone = 'acquisition task stopped'

    def start(self, session):
        rig = session.rig
        if steady(rig.configure, accumulate=8, digital=True) is None:
            session.note = 'the board refused the configuration'
            return None
        if steady(rig.start) is None:
            return None
        return (rig.stop,)


ACTIVITIES = (Switching(), Acquiring())

#: What each one puts back, for the teardown lines. Off the classes, so
#: a new activity carries its own sentence rather than needing one here.
UNDONE = dict((a.name, a.undone) for a in ACTIVITIES)
