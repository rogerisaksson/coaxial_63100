"""The gate drive: the board's raw ops, and the policy for switching them on."""
from abc import ABC, abstractmethod

from coaxial.devices.roles import Output
from coaxial.errors import RigError


class GateControl(Output, ABC):

    """The board's gate-drive ops, behind `0x6E` device 4.

    write(ticks, periods=0)      all three compares, for `periods` (0: until the next write)
    write(ticks, then=b)         two triples, swapped every PWM period
    write(fractions=f)           fractions of full scale, dithered in Q16.16
    configure(dead_time_ns, skew, trigger, bypass_break, sync)
        DTG (floored at 20 ns) and its skew; CCR4, where the triple is sampled;
        TIM1's break input off (bench only, a reset restores); the injected
        triple latched every period
    """

    def write(self, ticks=None, periods=0, then=None, fractions=None):
        if fractions is not None:
            return self._duty_fine(tuple(fractions))
        if then is not None:
            return self._alternate(ticks, then)
        return self._duty(ticks, periods)

    def configure(self, dead_time_ns=None, skew=0, trigger=None, bypass_break=None, sync=None):
        """What each setting given took."""
        took = {}
        if dead_time_ns is not None:
            took['dead_time'] = self._dead_time(dead_time_ns, skew)
        if trigger is not None:
            took['trigger'] = self._trigger(trigger)
        if bypass_break is not None:
            took['bypass_break'] = self._bypass(bypass_break)
        if sync is not None:
            took['sync'] = self._sync(sync)
        return took

    def dead_time(self):
        """The dead time: nanoseconds, skew, and the floor under both."""
        state = self.state()
        return {'nanoseconds': state['deadtime_ns'], 'skew': state['deadtime_skew'],
                'floor': state['deadtime_floor']}

    def is_on(self):
        return bool(self.state()['pwm_enabled'])

    @abstractmethod
    def state(self):
        """Everything the gate drivers know, from one conversion's worth."""

    @abstractmethod
    def on(self):
        """Set MOE, at zero duty."""

    @abstractmethod
    def off(self):
        """Clear MOE; every output drops to its idle level in hardware."""

    @abstractmethod
    def clear(self):
        """Clear the break latch. Does not switch back on - see the STO interlock."""

    @abstractmethod
    def reset_worst_gap(self):
        """Forget the longest keepalive gap, so a run measures on its own."""

    @abstractmethod
    def _duty(self, ticks, periods):
        """All three compares in timer ticks, or none of them."""

    @abstractmethod
    def _duty_fine(self, fractions):
        """The same as fractions of full scale, in Q16.16."""

    @abstractmethod
    def _alternate(self, ticks_a, ticks_b):
        """Two triples, A one period and B the next."""

    @abstractmethod
    def _dead_time(self, nanoseconds, skew):
        """Set DTG and its skew; what the board took."""

    @abstractmethod
    def _trigger(self, ticks):
        """Set CCR4; the ticks the board took."""

    @abstractmethod
    def _bypass(self, on):
        """Disconnect TIM1's break input, or connect it again."""

    @abstractmethod
    def _sync(self, on):
        """Latch the injected triple every period, or give the converters back."""


class GateStage(Output):

    """The power stage switched on under its checks: dead time, shorts, interlock, STO."""

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
        #: Whether THIS session was what set MOE. `is_on()` reads the board
        #: and answers for everybody; a shared board needs both.
        self._armed_here = False

    def __repr__(self):
        return '<GateStage - check(), on(), write(), off(); a write refuses until on()>'

    @property
    def control(self):
        """The raw ops, for anything this policy does not cover."""
        return self._board.gate_drivers

    def state(self):
        """What the gate drivers report now."""
        return self.control.state()

    def is_on(self):
        """Whether MOE is set, read off the board rather than remembered."""
        return self.control.is_on()

    def write(self, **values):
        return self.control.write(**values)

    def configure(self, **settings):
        return self.control.configure(**settings)

    def clear(self):
        return self.control.clear()

    def interlock(self):
        """What the arming conditions read now, and which of them hold."""
        if not self._board.afe.is_on():
            # AFE_ON powers the reference, so with it off every one of these
            # reads exact mid-scale and would pass or fail by accident.
            return [('AFE_ON', None, False, None)]

        rows = []
        readings = {r['signal']: r for r in
                    self._board.analog.read(samples=32)['channels']}
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

    def dead_time(self):
        """The dead time: nanoseconds, skew, and the floor under both."""
        return self.control.dead_time()

    def on(self, bypass_sto=False, ignore_interlock=False):
        """Set MOE. Nothing switches before this and everything can after."""
        self.check()
        if not ignore_interlock:
            self._require_interlock()
        if bypass_sto:
            self.control.configure(bypass_break=True)
        self.control.on()
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

    def off(self, keep_bypass=False):
        """Clear MOE, and put the break input back unless told otherwise."""
        # The flag drops only once the board confirmed.
        self.control.off()
        self._armed_here = False
        if not keep_bypass:
            self.control.configure(bypass_break=False)
        return self.control.state()
