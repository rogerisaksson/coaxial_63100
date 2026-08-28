"""The gate drive: the board's raw ops, and the policy for arming them.

Two things here, and the split is the point.

`GateControl` is what the board answers - the twelve ops behind `0x6E` device
4. `GateDrivers` and the stand-in both implement it, which is what stops the
stand-in drifting from the part it stands in for.

`GateStage` is the policy on top: what has to hold before MOE is set, and what
to put back afterwards. It lived on `Coaxial63100` as six methods that each
named the subsystem they acted on - `arm_gate_drivers`, `gate_drivers_armed`,
`gate_drivers_check`, `configure_pwm`, `stop_pwm` - which said it once in the
receiver and again in the name. Said once here.

The policy is deliberately not on `GateControl`. The board's ops are a dumb
slave's (invariant 10) and a stand-in has to answer them identically; refusing
to arm on an interlock is a host's judgement, and there is exactly one of it.
"""
from abc import ABC, abstractmethod

from .errors import RigError


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
    def duty(self, ticks):
        """All three compares in timer ticks, or none of them.

        A half update runs one cycle with two phases from this call and one
        from the last.
        """

    @abstractmethod
    def duty_fine(self, ticks):
        """The same, in Q16.16, so the mean duty is what was asked for."""

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

    """Arming a power stage, and the checks that come first.

    One object because the checks are not optional decoration: `arm()` runs
    `check()` and the interlock before it sets MOE, and both are reachable on
    their own so a view can show the conditions coming up rather than only
    learning of them when an arm is refused.
    """

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
        """What the arming conditions read now, and which of them hold.

        Measured every time. Returns a list of (name, volts, ok, want) - it
        does not raise, so a view can show the conditions coming up.
        """
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
        """Refuse a stage with no dead time, or with a leg's gates shorted.

        Dead time is the one thing between the two FETs of a leg. Read every
        time rather than trusted once: a `.ioc` regeneration, a CubeMX mode
        name bound to the wrong channel - which has happened twice here - or a
        stray BDTR write all land in the same place, and none of them announce
        themselves.
        """
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

    def arm(self, bypass_sto=False, ignore_interlock=False):
        """Set MOE. Nothing switches before this and everything can after.

        **This arms a power stage**, at zero duty - all three low sides on, a
        braked stage rather than a floating one.

        TIM1's dead time is the only protection: the 2EDL8034's inputs are
        independent and it has no interlock. Measured in the silicon, not the
        `.ioc` - BDTR DTG 19, CR1 CKD 00, 237.5 MHz, so **80.0 ns** against
        about 65 ns needed. `check()` re-reads it and refuses at zero.

        `ignore_interlock` skips `INTERLOCK`, which this bench board needs:
        Cinj reads 0.77 V and Clevel 0.06 V against 3 V each. `bypass_sto`
        disconnects the break input, without which a latched break outranks
        this. Both are decisions, which is why neither is silent.
        """
        self.check()

        if not ignore_interlock:
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

        if bypass_sto:
            self.control.bypass_break(True)
        self.control.enable()
        return self.control.state()

    def disarm(self, keep_bypass=False):
        """Clear MOE, and put the break input back unless told otherwise."""
        self.control.disable()
        if not keep_bypass:
            self.control.bypass_break(False)
        return self.control.state()
