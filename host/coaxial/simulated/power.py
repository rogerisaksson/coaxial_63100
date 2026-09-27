"""The power stage stood down: the rails, and the gate drivers under the real arming policy."""
import time
from typing import Any

from coaxial.devices.gates import GateControl
from coaxial.devices.power import named
from coaxial.errors import RigError
from coaxial.model import inverter
from coaxial.model.inverter import GATE_UVLO_V
from coaxial.devices.scaling import ADC_CODES, ADC_HALF_CODES
from coaxial.simulated import sto
from coaxial.simulated.values import (DCBUS_V, NOMINAL, VGATE_PIN_RATIO, phase_codes,
                                      quiet_code)
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
    """TIM1, the injected triple and the STO chain (coaxial.simulated.sto): BKIN its
    FAULTOUT, BIF latched while it is low and the break enabled, MOE cleared by it."""

    PERIOD = 2376
    #: DTG's step at 237.5 MHz, ps.
    DTS_PS = 4210
    #: The record's dead time as Board_PwmInit writes it: DTG 8, 33.7 ns (inverter.T_DEAD);
    #: CubeMX's 19 lasts until then.
    DEADTIME = round(inverter.T_DEAD * 1e12 / DTS_PS)
    TRIGGER = 2360
    #: The update rate the counted hold and the update counter run at.
    PWM_HZ = 50000
    #: The keepalive's edges a second, measured idle.
    KEEPALIVE_HZ = 214000

    #: The keepalive's worst gap, cycles: with AFE_ON low and MOE clear main() sleeps in WFI
    #: and the toggle waits up to a SysTick, 1 ms at 475 MHz; awake, the measured 52 us.
    GAP_ASLEEP, GAP_AWAKE = 475000, 24700

    def __init__(self):
        self._deadtime = self.DEADTIME
        self._at = 0                    # where in the period the counter is
        self._deadtime_ns = self.DEADTIME * self.DTS_PS // 1000
        self._skew = 0
        #: The drive whose sample point this register moves; the board
        #: wires it.
        self._drive: Any = None
        self._armed = False
        self._enabled = False
        self._compares = (0, 0, 0)
        self._hold_until = None
        #: CCR5, written at the first arming (Board_SyncTrigger reads 0 until then).
        self._at_trigger = 0
        #: Whether the AFE's reference is up, and the thermal stand-in the NTC follows; the
        #: board wires both.
        self._afe_on = lambda: True
        self._thermal: Any = None
        self._updates = 0
        self._keepalive = 0
        self._counted_at = time.monotonic()
        self._bypassed = False
        #: The chain; the board wires its +5 and pump.
        self._sto = sto.SimulatedSto(DCBUS_V)
        #: BIF: the chain is down from power-on, the break enabled.
        self._fault = True

    def _chain(self):
        """The chain on to now, and the break as its FAULTOUT stands: BIF latched while it is
        low and the break enabled, MOE cleared."""
        self._sto.advance()
        if not self._sto.faultout and not self._bypassed:
            self._fault = True
            self._enabled = False
        return self._sto

    def _driving(self):
        """MOE set and the drivers supplied: the 2EDL8034's outputs follow TIM1."""
        chain = self._chain()
        return self._enabled and chain.vgate >= GATE_UVLO_V

    @staticmethod
    def _pin(volts):
        """A single-ended pin's code and microvolts, the converter's range clipping it."""
        code = int(round(max(0.0, min(ADC_CODES, volts / 3.3 * ADC_CODES))))
        return code, int(round(code / ADC_CODES * 3.3e6))

    def state(self):
        # The keepalive's edges at the measured idle rate and the triple's updates at the PWM's,
        # over the wall's time since the last look.
        now = time.monotonic()
        span, self._counted_at = now - self._counted_at, now
        self._keepalive += int(self.KEEPALIVE_HZ * span)
        if self._armed:
            self._updates += int(self.PWM_HZ * span)
        chain = self._chain()
        left = self._periods_left()
        at = self._cnt()
        phase, dcbus, ntc = self._latched()
        afe = self._afe_on()
        pilot, level = ((self._pin(chain.cinj), self._pin(chain.clevel)) if afe
                        else ((int(ADC_HALF_CODES), 1650000),) * 2)
        return {
            'pwm_ready': True, 'pwm_enabled': self._enabled,
            'fault': self._fault,
            'sync_ready': True, 'sync_armed': self._armed, 'afe_on': afe,
            'pilot_ok': True, 'level_ok': True,
            'period': self.PERIOD, 'deadtime': self._deadtime,
            'duty': self._duty_ticks(), 'trigger': self._at_trigger,
            'phase': phase, 'at': at if (self._armed or self._enabled) else 0,
            'updates': self._updates, 'overruns': 0,
            'keepalive': self._keepalive,
            'worst_gap_cycles': (self.GAP_AWAKE if (afe or self._enabled)
                                 else self.GAP_ASLEEP),
            'pilot_raw': pilot[0], 'pilot_microvolts': pilot[1],
            'level_raw': level[0], 'level_microvolts': level[1],
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
            # The injected sequence's DC link and NTC (MINOR 2).
            'dcbus_raw': dcbus,
            'ntc_raw': ntc,
            # PE15 and +15V7 (MINOR 23): the supply through Vgate's pin, mid-scale's with the
            # reference down.
            'nfault': chain.faultout,
            'vgate_mv': None if self._armed else int(round(
                (min(chain.vgate * VGATE_PIN_RATIO, 3.3) if afe else 1.65)
                / VGATE_PIN_RATIO * 1000.0)),
        }

    def _latched(self):
        """The injected triple's last codes, as Board_SyncLatest holds them: the phases on the
        drive's current, the DC link and the NTC at rank 2 - none until the sync is armed."""
        if not self._armed:
            return (0, 0, 0), 0, 0
        amps, theta = (self._drive._carrying() if self._drive is not None
                       else (0.0, 0.0))
        phase = tuple(int(NOMINAL[leg] + phase_codes(signal, amps, theta))
                      for leg, signal in enumerate(('Phase U', 'Phase V', 'Phase W')))
        return phase, int(NOMINAL[5]), int(quiet_code(4, self._thermal))

    def _duty_ticks(self):
        """What the compares hold, ticks: the drive's modulator while it owns them, as TIM1's
        CCRs on the board."""
        drive = self._drive
        if drive is not None and drive._mode != 'off':
            return tuple(int(round(d * self.PERIOD)) for d in drive._duty())
        return self._compares

    #: DTG counts for 20 ns at 237.5 MHz, rounded up - the same floor the
    #: board computes, because the 2EDL8034 has no interlock either way.
    DEADTIME_FLOOR = 5
    DTG_MAX = 127

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
        self._chain()
        self._bypassed = bool(on)
        if self._bypassed:
            self._fault = False          # BKE and BIF cleared together
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
        # because nFAULT went low, and clearing the latch does not help while it
        # stays low.
        self._chain()
        if self._fault and not self._bypassed:
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
        if self._armed and not self._at_trigger:
            self._at_trigger = self.TRIGGER
        return True

    def _trigger(self, ticks):
        self._at_trigger = min(int(ticks), self.PERIOD - 1)
        if self._drive is not None:
            self._drive._move_trigger(self._at_trigger)
        return self._at_trigger

    def clear(self):
        """BIF cleared, if BKIN has let go."""
        chain = self._chain()
        if chain.faultout or self._bypassed:
            self._fault = False
        return True
