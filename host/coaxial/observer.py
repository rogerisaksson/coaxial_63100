"""The rotor observers, and finding out what machine they are watching.

TWO THINGS LIVE HERE. `chain()` reads the back-EMF observer the firmware
runs beside the control loop - `drive/src/drive_observer.c`, behind
`0x6E` device 10 op 14 - and says what it is worth in the terms a drive
cares about. `autodetect()` finds out what the machine on the shaft
actually is, which is the question every one of those numbers rests on:
an observer given the wrong flux linkage is confidently wrong.

NO NEW WIRE. Everything here is composed out of ops that already exist -
the drive's, the shaft sensor's, the commissioning steps' - so this
subsystem is the same code against a board and against the stand-in, and
there is nothing for the parity suite to have to hold together.

WHAT AUTODETECT CAN AND CANNOT FIND. R, Ld, Lq and lambda are measured by
the commissioning steps and land in the calibration record, which is
where every conversion in this tree lives (invariant 7). The POLE PAIRS
are measured here, against the shaft sensor, because nothing else was
measuring them and every observer's speed and every rpm on every page is
divided by them. The SLOT COUNT is not measurable from the terminals at
all - it changes the winding factor and the cogging, neither of which
this board can see - so it is asked for, not guessed.

The board is not asked whether the answer is good. `measured` says which
numbers came off an instrument and which came out of the record already;
a test executive beside a calibrated meter decides the rest (invariant 10).
"""
import math

from .errors import RigError
from .motor import Parameters
from .subsystem import Subsystem

#: How far the rotor is walked to count pole pairs, in electrical turns.
#: Enough that one shaft reading's error is small against the travel: the
#: A1335 resolves to about a tenth of a degree, and fourteen pole pairs
#: put fourteen electrical turns in one of the shaft's.
TURNS = 28.0
#: How fast it is walked, electrical radians a second. Slow enough that
#: the rotor follows the commanded angle rather than lagging it - HOLD is
#: a stepper and a stepper that is asked for more than it has slips.
WALK_RAD_S = 40.0


class Identified(Parameters):

    """A machine that was measured, with the steps that measured it.

    `Parameters` carries `__slots__` on purpose - it is the shape every
    profile and every notebook passes around, and a stray attribute on
    one of those is a number nobody can trace. This adds exactly two,
    and only autodetect makes one: what the steps returned, so a bench
    can see WHY a number is what it is, and the slot count it was told,
    which is the one thing on the shaft that cannot be measured from the
    terminals.
    """

    __slots__ = ('steps', 'slots')


class Observer(Subsystem):

    """The observer chain, and what it is watching."""

    def chain(self):
        """The firmware's back-EMF chain, in the terms a drive cares about.

        The board answers raw fields; the arithmetic on top of them is the
        same three questions every time. `error` is the chain's angle
        minus the loop's own estimate, out of ONE reply - the two are 15
        ms apart otherwise, which at 4000 rad/s electrical is sixty
        radians and none of it the observer's.
        """
        got = dict(self.board.drive.observers())
        span = got['blend_hi'] - got['blend_lo']
        got['error_deg'] = math.degrees(got['error'])
        #: What a wrong angle costs: torque follows the cosine of it, so
        #: five degrees is a third of a percent and thirty is thirteen.
        got['torque_fraction'] = math.cos(got['error'])
        got['carried_by'] = 'flux' if got['blend'] >= 0.5 else 'dual'
        got['in_hand_over'] = 0.0 < got['blend'] < 1.0 and span > 0.0
        return got

    def machine(self, slots=None):
        """What the record says is on the shaft, and what it cannot say.

        `slots` is passed through untouched, `None` and all: a drawing or
        a report that wants a slot count has to be told one, and this is
        the place that refuses to invent it.
        """
        params = self.board.drive.params()
        pairs = int(params.get('motor_pole_pairs') or 0)
        return {'pole_pairs': pairs, 'poles': 2 * pairs, 'slots': slots,
                'r': params.get('motor_r_uohm'),
                'ld': params.get('motor_ld_nh'), 'lq': params.get('motor_lq_nh'),
                'lam': params.get('motor_lambda_uvs'),
                'name': '%dN%dP' % (slots, 2 * pairs) if slots and pairs
                        else '%d poles' % (2 * pairs) if pairs else 'unknown'}

    def pole_pairs(self, turns=TURNS, omega=WALK_RAD_S, amps=None):
        """Pole pairs, counted against the shaft sensor.

        HOLD commutates on the COMMANDED angle - it is a microstepper -
        so walking the command through `turns` electrical revolutions
        walks the rotor through `turns / p` mechanical ones, and the ratio
        is the pole count. It is the one machine constant this board can
        measure directly, and the only one that is an integer: the answer
        is rounded, and how far it had to be rounded is reported beside
        it, because a fit that lands on 7.4 is a rotor that slipped.

        The shaft sensor is what makes it possible and what limits it. Off
        its magnet the A1335 reads noise that looks exactly like an angle,
        so the field is checked first and a weak one refuses rather than
        answering a plausible number - the failure mode this tree has
        already been bitten by twice with the AFE.
        """
        angle, drive = self.board.angle, self.board.drive
        before = angle.state()
        if before.get('degrees') is None:
            raise RigError(
                'the shaft sensor is not reporting an angle, so there is '
                'nothing to count pole pairs against - the A1335 needs its '
                'magnet in front of it and AFE_ON up before it reads')
        travel = turns * 2.0 * math.pi
        drive.setpoint(id_ref=amps if amps is not None
                       else drive.params()['drv_i_max_ma'] * 0.5,
                       iq_ref=0.0, theta=0.0, omega_target=omega,
                       accel=omega * 4.0)
        try:
            drive.mode('hold')
            walked = self._walk(angle, travel / omega)
        finally:
            drive.setpoint(omega_target=0.0)
            drive.off()
        if walked <= 0.0:
            raise RigError(
                'the shaft did not move while the command walked %.0f '
                'electrical turns - either the rotor is held, the stage is '
                'not switching, or the current is below what it takes to '
                'turn this machine' % turns)
        exact = travel / walked
        pairs = int(round(exact))
        return {'pole_pairs': max(1, pairs), 'exact': exact,
                'rounded_by': abs(exact - pairs),
                'shaft_turns': walked / (2.0 * math.pi),
                'measured': abs(exact - pairs) < 0.25}

    @staticmethod
    def _walk(angle, seconds):
        """Shaft radians travelled while the command walks, unwrapped.

        Unwrapped by the short way round each sample: the sensor reports
        0 to 360 and a rotor crossing zero would otherwise read as a turn
        backwards. Sampling has to be quick enough that no half turn
        happens between two reads, which is what `WALK_RAD_S` is for.
        """
        import time

        was = math.radians(angle.state()['degrees'])
        total = 0.0
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            now = math.radians(angle.state()['degrees'])
            step = (now - was + math.pi) % (2.0 * math.pi) - math.pi
            total += step
            was = now
        return abs(total)

    def autodetect(self, arm=None, slots=None, name='autodetected',
                   log=None, electrical=True):
        """Find out what machine is on the shaft, and write it down.

        The electrical steps are `coaxial.commission`'s - dead time and R,
        then the inductance map for Ld and Lq, then an I/f spin for the
        flux linkage - and each of them writes its result into the
        calibration record as it goes, so the board is left describing the
        machine it just measured rather than the one it was shipped with.
        The pole count is counted here against the shaft.

        `arm` is what the stage is armed with, the dict `gates.arm()`
        takes; without it the steps that need to switch refuse and say so,
        which is `Commissioning`'s own policy and not a second one.

        Returns `coaxial.motor.Parameters`. `measured` on it is true only
        when every one of the four electrical numbers and the pole count
        came off an instrument in this call - a partly-identified machine
        is still returned, still usable, and still says what it is.
        """
        from .commission import Commissioning

        say = log or (lambda line: None)
        steps = Commissioning(self._rig(), arm=arm, log=say)
        got = {}
        if electrical:
            for step in ('deadtime', 'l_map', 'flux'):
                got[step] = getattr(steps, step)()
                say('%s %s' % (step, 'measured' if got[step].get('measured')
                               else 'NOT measured'))
        got['poles'] = self.pole_pairs()
        say('pole pairs %(pole_pairs)d, fit %(exact).2f' % got['poles'])
        pairs = got['poles']['pole_pairs']
        self.board.drive.set_params(motor_pole_pairs=pairs)

        record = self.board.drive.params()
        measured = (got['poles']['measured']
                    and all(got.get(k, {}).get('measured')
                            for k in ('deadtime', 'l_map', 'flux'))
                    if electrical else False)
        found = Identified(
            name=('%dN%dP' % (slots, 2 * pairs)) if slots else name,
            r=record['motor_r_uohm'], ld=record['motor_ld_nh'],
            lq=record['motor_lq_nh'], lam=record['motor_lambda_uvs'],
            poles=pairs, measured=measured, source='observer.autodetect')
        found.steps = got
        found.slots = slots
        return found

    def _rig(self):
        """The rig `Commissioning` wants, which is one level up from here.

        A subsystem holds the board, and the steps hold the rig: they need
        `gates` and `analog` as well as the drive. The board carries the
        way back, and a board that does not is a board this cannot
        commission - said here rather than as an AttributeError six frames
        down inside a step that has already armed the stage.
        """
        rig = getattr(self._board, 'rig', None)
        if rig is None:
            raise RigError(
                'autodetect needs the whole rig, not just the board - it '
                'arms the stage and reads the shunt scaling, and this board '
                'handle was made without one. Reach it as device.observer '
                'off Coaxial63100')
        return rig
