"""The gate drive: the board's raw ops, and the policy for arming them."""
from abc import ABC, abstractmethod

from coaxial.errors import RigError


class GateControl(ABC):

    """The board's gate-drive ops, behind `0x6E` device 4."""

    @abstractmethod
    def state(self):
        """Everything the gate drivers know, from one conversion's worth."""

    @abstractmethod
    def enable(self):
        """Set MOE, at zero duty."""

    @abstractmethod
    def disable(self):
        """Clear MOE."""

    @abstractmethod
    def duty(self, ticks, periods=0):
        """All three compares in timer ticks, or none of them."""

    @abstractmethod
    def duty_fine(self, ticks):
        """The same, in Q16.16, so the mean duty is what was asked for."""

    @abstractmethod
    def alternate(self, ticks_a, ticks_b):
        """Two triples, A one period and B the next, swapped by the board every
        PWM period until the next duty write.
        """

    @abstractmethod
    def dead_time(self, nanoseconds=None, skew=None):
        """Read or set DTG and its skew. The board floors it at 20 ns."""

    @abstractmethod
    def arm(self):
        """Arm the sync, so the injected triple latches per period."""

    @abstractmethod
    def disarm(self):
        """Disarm the sync."""

    @abstractmethod
    def trigger(self, ticks=None):
        """Where in the period the phases are sampled - CCR4."""

    @abstractmethod
    def bypass_break(self, on):
        """Disconnect TIM1's break input. Bench work only; a reset restores."""

    @abstractmethod
    def clear_fault(self):
        """Clear the break latch. Does not re-arm - see the STO interlock."""

    @abstractmethod
    def reset_worst_gap(self):
        """Forget the longest keepalive gap, so a run measures on its own."""


class GateStage:

    """Arming a power stage, and the checks that come first."""

    #: What the schematic wants true before the gate drive is armed, as volts
    #: at the pin. The charge pump has to have pumped and the level detector
    #: has to have tripped; arming under either of them is arming into a
    #: supply that is still coming up.
    #:
    #: Volts and not codes: a threshold in codes stops meaning anything the
    #: moment a divider changes, and the divider is the board's, not this
    #: file's (invariant 7).
    INTERLOCK = (('Cinj', 3.0), ('Clevel', 3.0))

    def __init__(self, board):
        self._board = board
        #: Whether THIS session was what set MOE. `armed()` reads the board
        #: and answers for everybody; a shared board needs both.
        self._armed_here = False

    def __repr__(self):
        return ('<GateStage - check(), arm(), disarm(). A duty write is '
                'refused until arm() has been called>')

    @property
    def control(self):
        """The raw ops, for anything this policy does not cover."""
        return self._board.gate_drivers

    def state(self):
        """What the gate drivers report now."""
        return self.control.state()

    def armed(self):
        """Whether MOE is set, read off the board rather than remembered."""
        return bool(self.control.state()['pwm_enabled'])

    def interlock(self):
        """What the arming conditions read now, and which of them hold."""
        if not self._board.afe.is_on():
            # AFE_ON powers the reference, so with it off every one of these
            # reads exact mid-scale and would pass or fail by accident.
            return [('AFE_ON', None, False, None)]

        rows = []
        readings = {r['signal']: r for r in
                    self._board.analog.read_all(nr_of_samples=32)['channels']}
        for name, want in self.INTERLOCK:
            got = readings.get(name)
            volts = got['volts_at_pin'] if got else None
            rows.append((name, volts, volts is not None and volts >= want,
                         want))
        return [('AFE_ON', None, True, None)] + rows

    def check(self):
        """Refuse a stage with no dead time, or with a leg's gates shorted."""
        state = self.control.state()
        if not state['deadtime']:
            raise RigError(
                'TIM1 BDTR DTG reads 0, so there is no dead time and the '
                '2EDL8034 has no interlock of its own - both FETs of a leg '
                'would conduct together. Check TIM1.DeadTime in the .ioc '
                'and that the generated MX_TIM1_Init still applies it')
        if state.get('gate_shorts'):
            raise RigError(
                'the gate pins of leg %s are on one node, so that leg cannot '
                'be driven complementary: both FETs get the same command and '
                'the leg never switches. Measured by the board, which drives '
                'one pin and watches the other sink through its own pull-down.'
                % ', '.join(state['gate_shorts']))
        return state

    def dead_time(self, nanoseconds=None, skew=None):
        """Read the dead time, or set it and its skew."""
        if skew is None:
            return self._board.gate_drivers.dead_time(nanoseconds)
        return self._board.gate_drivers.dead_time(nanoseconds, skew=skew)


    def arm(self, bypass_sto=False, ignore_interlock=False):
        """Set MOE. Nothing switches before this and everything can after."""
        self.check()
        if not ignore_interlock:
            self._require_interlock()
        if bypass_sto:
            self.control.bypass_break(True)
        self.control.enable()
        self._armed_here = True
        return self.control.state()

    def _require_interlock(self):
        """Raise unless every arming condition reads true now."""
        failed = [row for row in self.interlock() if not row[2]]
        if failed:
            raise RigError(
                'the arming interlock is not satisfied: %s. The '
                'schematic wants the charge pump up and the level '
                'detector tripped before the gate drive is armed. Pass '
                'ignore_interlock=True to arm anyway, which is what an '
                'unmodified bench board needs'
                % ', '.join(
                    '%s %s' % (name, 'is off' if volts is None
                               else '%.2f V, wants %.1f' % (volts, want))
                    for name, volts, _, want in failed))

    @property
    def armed_here(self):
        """Whether this session armed the stage - not whether it is armed."""
        return self._armed_here

    def disarm(self, keep_bypass=False):
        """Clear MOE, and put the break input back unless told otherwise."""
        # The flag drops only once the board confirmed.
        self.control.disable()
        self._armed_here = False
        if not keep_bypass:
            self.control.bypass_break(False)
        return self.control.state()
