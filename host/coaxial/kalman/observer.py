"""The rotor observers, and finding out what machine they are watching."""
import math
import time

from ..errors import RigError
from ..motor import Parameters
from ..subsystem import Subsystem
from ..commission import Commissioning

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

    """A machine that was measured, with the steps that measured it."""

    __slots__ = ('steps', 'slots')


class Observer(Subsystem):

    """The observer chain, and what it is watching."""

    def chain(self):
        """The firmware's back-EMF chain, in the terms a drive cares about."""
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
        """What the record says is on the shaft, and what it cannot say."""
        params = self.board.drive.params()
        pairs = int(params.get('motor_pole_pairs') or 0)
        return {'pole_pairs': pairs, 'poles': 2 * pairs, 'slots': slots,
                'r': params.get('motor_r_uohm'),
                'ld': params.get('motor_ld_nh'), 'lq': params.get('motor_lq_nh'),
                'lam': params.get('motor_lambda_uvs'),
                'name': '%dN%dP' % (slots, 2 * pairs) if slots and pairs
                        else '%d poles' % (2 * pairs) if pairs else 'unknown'}

    def pole_pairs(self, turns=TURNS, omega=WALK_RAD_S, amps=None):
        """Pole pairs, counted against the shaft sensor."""
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
        """Shaft radians travelled while the command walks, unwrapped."""

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
        """Find out what machine is on the shaft, and write it down."""

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
        """The rig `Commissioning` wants, which is one level up from here."""
        rig = self._board.rig
        if rig is None:
            raise RigError(
                'autodetect needs the whole rig, not just the board - it '
                'arms the stage and reads the shunt scaling, and this board '
                'handle was made without one. Reach it as device.observer '
                'off Coaxial63100')
        return rig
