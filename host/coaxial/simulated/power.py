"""The power stage stood down: the rails, and the gate drivers under the real arming policy."""
import time
from typing import Any

from coaxial.devices.gates import GateControl
from coaxial.devices.power import named
from coaxial.errors import RigError
from coaxial.simulated.values import NOMINAL, _sweep
from machine.roles import Output


class SimulatedPower(Output):
    """Rail reference counts without a board."""

    def __init__(self):
        self._mask = 0

    def state(self):
        return {'afe': {'on': self._mask != 0, 'users': named(self._mask),
                        'mask': self._mask, 'count': bin(self._mask).count('1'),
                        'blocked': False, 'leased': []}}

    def off(self):
        self._mask = 0
        return True


class SimulatedGateDrivers(GateControl):
    """TIM1, the injected triple and the STO chain, without any of them."""

    PERIOD = 2376
    DEADTIME = 19
    TRIGGER = 2360
    #: The update rate the counted hold and the update counter run at.
    PWM_HZ = 50000

    def __init__(self):
        self._deadtime = self.DEADTIME
        self._at = 0                    # where in the period the counter is
        self._deadtime_ns = self.DEADTIME * 4210 // 1000
        self._skew = 0
        #: The drive whose sample point this register moves; the board
        #: wires it.
        self._drive: Any = None
        self._armed = False
        self._enabled = False
        self._compares = (0, 0, 0)
        self._hold_until = None
        self._at_trigger = self.TRIGGER
        self._updates = 0
        self._keepalive = 0
        self._bypassed = False

    def state(self):
        self._keepalive += 214000        # the measured idle toggle rate
        if self._armed:
            self._updates += self.PWM_HZ
        left = self._periods_left()
        at = self._cnt()
        return {
            'pwm_ready': True, 'pwm_enabled': self._enabled,
            'fault': not self._bypassed,
            'sync_ready': True, 'sync_armed': self._armed, 'afe_on': True,
            'pilot_ok': True, 'level_ok': True,
            'period': self.PERIOD, 'deadtime': self.DEADTIME,
            'duty': self._compares, 'trigger': self._at_trigger,
            'phase': (1433, -8136, 390), 'at': 1385,
            'updates': self._updates, 'overruns': 0,
            'keepalive': self._keepalive,
            'worst_gap_cycles': 24700,
            'pilot_raw': 15149, 'pilot_microvolts': 763000,
            'level_raw': 1305, 'level_microvolts': 65000,
            'break_bypassed': self._bypassed,
            # TICKS, like the board: it sends Q16.16 of a CCR count and the
            # host divides that back.
            'requested_ticks': tuple(float(d) for d in self._compares),
            'pins': self._gates(at),
            'pins_at': at,
            'deadtime_ns': self._deadtime_ns,
            'deadtime_skew': self._skew,
            'periods_left': left,
            'deadtime_floor': self.DEADTIME_FLOOR,
            'gate_shorts': (),
            # The injected sequence's DC link and NTC (MINOR 2): the link the
            # drive runs on (DCBUS_V), the NTC where the analog reads it.
            'dcbus_raw': int(NOMINAL[5]),
            'ntc_raw': int(NOMINAL[4] + _sweep(4)),
        }

    #: DTG counts for 20 ns at 237.5 MHz, rounded up - the same floor the
    #: board computes, because the 2EDL8034 has no interlock either way.
    DEADTIME_FLOOR = 5
    DTG_MAX = 127
    DTS_PS = 4210

    def _dead_time(self, nanoseconds, skew):
        counts = max(self.DEADTIME_FLOOR,
                     int(nanoseconds) * 1000 // self.DTS_PS)
        if counts + abs(int(skew)) > self.DTG_MAX:
            raise RigError("that dead time plus its skew is past DTG's "
                           "linear range - ask for 535 ns or less")
        if counts - abs(int(skew)) < self.DEADTIME_FLOOR:
            raise RigError('that skew would take one of the two dead times '
                           'under the 20 ns floor - raise the dead time '
                           'first, or skew it less')
        self._deadtime, self._skew = counts, int(skew)
        self._deadtime_ns = counts * self.DTS_PS // 1000
        return self.dead_time()

    #: Counts per read, coprime with PERIOD so reads walk the whole period (a
    #: wall-clock counter moved 7 ticks in 60 reads; every sample one side).
    CNT_STEP = 617

    def _cnt(self):
        """Somewhere in the period, and somewhere else next time."""
        self._at = (self._at + self.CNT_STEP) % self.PERIOD
        return self._at

    def _gates(self, at):
        """The six signals a real one would show at this count."""
        # Enabled, L is not H by construction: a shoot-through check cannot fail here.
        out = {}
        for leg, duty in zip(('U', 'V', 'W'), self._compares):
            high = self._enabled and at < duty
            out[leg + 'L'] = bool(self._enabled and not high)
            out[leg + 'H'] = bool(high)
        return {k: out[k] for k in ('UL', 'UH', 'VL', 'VH', 'WL', 'WH')}

    def reset_worst_gap(self):
        return True

    def _bypass(self, on):
        self._bypassed = bool(on)
        return True

    def _periods_left(self):
        """A counted hold's periods still to run - and the compares zeroed when
        it has run out, which is the update interrupt's job on the board.
        """
        if self._hold_until is None:
            return 0
        remaining = self._hold_until - time.monotonic()
        if remaining > 0.0:
            return max(1, int(remaining * self.PWM_HZ))
        self._compares = (0, 0, 0)
        self._hold_until = None
        return 0

    def on(self):
        # Refuses for the reason the real board refuses: the break is latched
        # because nFAULT is low, and clearing the latch does not help while it
        # stays low.
        if not self._bypassed:
            raise RigError('the board refused to enable the gate drivers - check '
                           'fault, and whether the STO chain has released '
                           '(simulated)')
        self._enabled = True
        return True

    def off(self):
        self._enabled = False
        self._compares = (0, 0, 0)
        return True

    def _duty(self, ticks, periods):
        ticks = tuple(int(t) for t in ticks)
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        if len(ticks) != 3 or any(t > self.PERIOD - 1 for t in ticks):
            raise RigError('the board refused %r - past ARR (simulated)'
                           % (ticks,))
        self._compares = ticks
        # The counted hold, wall-paced like the rest of the stand-in: the
        # virtual interrupt zeroes the compares when the count runs out, and
        # state() is where the expiry is noticed - the board's own shape, seen
        # from the link.
        self._hold_until = (time.monotonic() + periods / self.PWM_HZ
                            if periods else None)
        return True

    def _duty_fine(self, fractions):
        if len(fractions) != 3:
            raise ValueError('%d duties, not 3' % len(fractions))
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        period = self.PERIOD - 1
        self._compares = tuple(max(0.0, min(1.0, f)) * period for f in fractions)
        return True

    def _alternate(self, ticks_a, ticks_b):
        ticks_a, ticks_b = tuple(int(t) for t in ticks_a), tuple(int(t) for t in ticks_b)
        if len(ticks_a) != 3 or len(ticks_b) != 3:
            raise ValueError('two triples of 3 compare values')
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        if any(t > self.PERIOD - 1 for t in ticks_a + ticks_b):
            raise RigError('the board refused %r / %r - past ARR (simulated)'
                           % (ticks_a, ticks_b))
        # The stand-in holds A: the real board's state shows whichever triple
        # the last update wrote.
        self._compares = ticks_a
        return True

    def _sync(self, on):
        self._armed = bool(on)
        return True

    def _trigger(self, ticks):
        self._at_trigger = min(int(ticks), self.PERIOD - 1)
        if self._drive is not None:
            self._drive._move_trigger(self._at_trigger)
        return self._at_trigger

    def clear(self):
        return True
