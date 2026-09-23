"""The three-phase gate drivers: TIM1, the synced phase triple, and Safe Torque
Off.
"""
from . import protocol
from .gates import GateControl
from .protocol import GateOp
from .subsystem import Device
from .wire import Reader, pack, q16
from typing import Any

#: Bit positions in the state reply's first byte, in order.
FLAGS = ('pwm_ready', 'pwm_enabled', 'fault', 'sync_ready', 'sync_armed',
         'afe_on', 'pilot_ok', 'level_ok')

PHASES = 3

#: The legs, in the order the gate-short mask names them.
LEGS = ('U', 'V', 'W')

#: PE8..PE13 in pin order, which is low side then high side per leg.
GATES = ('UL', 'UH', 'VL', 'VH', 'WL', 'WH')

#: The one-byte switch every on/off op takes.
ON = pack(('u8', 1))
OFF = pack(('u8', 0))


def _triple(ticks):
    """Three compare values on the wire, or a raise: a half update would run
    one cycle with two phases from this call and one from the last.
    """
    ticks = tuple(ticks)
    if len(ticks) != PHASES:
        raise ValueError('%d compare values, not %d' % (PHASES, len(ticks)))
    return pack(*(('u16', int(t)) for t in ticks))


def _unit(fraction):
    """A duty clamped to [0, 1]."""
    return max(0.0, min(1.0, fraction))


class GateDrivers(Device, GateControl, device=protocol.DEVICE_GATE_DRIVERS):

    """TIM1's compare registers, the injected triple and the STO chain."""

    def dead_time(self, nanoseconds=None, skew=0):
        """Read the dead time, or set it and its skew."""
        if nanoseconds is None:
            state = self.state()
            return {'nanoseconds': state['deadtime_ns'],
                    'skew': state['deadtime_skew'],
                    'floor': state['deadtime_floor']}

        reply = self._op(GateOp.DEADTIME,
                         pack(('u32', int(nanoseconds)), ('i8', int(skew))))
        # took() raises on a refusal and returns True otherwise, so the reader
        # is built here and the took byte read off it.
        r = Reader(reply)
        self.took(reply)
        r.u8()
        return {'nanoseconds': r.u32(), 'skew': r.i8(), 'floor': r.u8()}

    def state(self):
        """Everything the gate drivers know, from one conversion's worth of
        time.
        """
        r = Reader(self._op(GateOp.STATE))
        out: dict[str, Any] = r.flags(FLAGS)
        out['period'] = r.u16()
        out['deadtime'] = r.u8()
        out['duty'] = tuple(r.u16() for _ in range(PHASES))
        out['trigger'] = r.u16()
        out['phase'] = tuple(r.i16() for _ in range(PHASES))
        out['at'] = r.u16()
        out['updates'] = r.u32()
        out['overruns'] = r.u32()
        out['keepalive'] = r.u32()
        out['worst_gap_cycles'] = r.u32()
        out['pilot_raw'] = r.i32()
        out['pilot_microvolts'] = r.i32()
        out['level_raw'] = r.i32()
        out['level_microvolts'] = r.i32()
        out.update(r.flags(('break_bypassed',)))
        # Asked for, in ticks Q16.16, beside what the register holds this
        # period.
        out['requested_ticks'] = tuple(r.q16() for _ in range(PHASES))
        # Six gate signals in one IDR load with TIM1->CNT beside it: six
        # separate asks at 50 kHz can straddle an edge and show a leg with
        # both FETs on, the one state dead time prevents.
        out['pins'] = r.flags(GATES)
        out['pins_at'] = r.u16()
        out['deadtime_ns'] = r.u32()
        out['deadtime_skew'] = r.i8()
        out['deadtime_floor'] = r.u8()
        # Which legs have their two gate pins on one node.
        out['gate_shorts'] = tuple(leg for leg, joined in r.flags(LEGS).items()
                                   if joined)
        # The DC link the injected sequence read beside the triple, raw
        # single-ended - rank 2 on ADC3, MINOR 2.
        out['dcbus_raw'] = r.maybe('u32')
        # And the NTC, rank 2 on ADC1 - the thermal observer's thermometer
        # while the drive holds the converters.
        out['ntc_raw'] = r.maybe('u32')
        # Periods left of a counted hold, MINOR 8. Zero when free-running.
        out['periods_left'] = r.maybe('u32')
        return out

    def enable(self):
        """Set the master output enable, always at zero duty."""
        return self._ack(GateOp.PWM, ON)

    def disable(self):
        """Clear MOE. Every output drops to its idle level in hardware."""
        self._op(GateOp.PWM, OFF)
        return True

    def duty(self, ticks, periods=0):
        """All three compare registers, or none of them."""
        counted = pack(('u32', int(periods))) if periods else b''
        return self._ack(GateOp.DUTY, _triple(ticks) + counted)

    def alternate(self, ticks_a, ticks_b):
        """Two compare triples, A one PWM period and B the next, swapped by
        TIM1's update interrupt for as long as they stand.
        """
        return self._ack(GateOp.ALTERNATE, _triple(ticks_a) + _triple(ticks_b))

    def duty_fine(self, fractions):
        """Duty as a fraction of full scale, dithered to hit it exactly."""
        fractions = tuple(fractions)
        if len(fractions) != PHASES:
            raise ValueError('%d duties, not %d' % (PHASES, len(fractions)))

        period = self.state()['period'] - 1
        return self._ack(GateOp.DUTY_FINE,
                         pack(*(('u32', q16(_unit(f) * period))
                                for f in fractions)))

    def arm(self):
        """Start latching the injected triple."""
        return self._ack(GateOp.SYNC, ON)

    def disarm(self):
        """Stop latching, and give the converters back to the meter."""
        self._op(GateOp.SYNC, OFF)
        return True

    def trigger(self, ticks=None):
        """Where in the PWM period the triple is taken, as CCR4 in ticks."""
        if ticks is None:
            return self.state()['trigger']
        return Reader(self._op(GateOp.TRIGGER,
                               pack(('u16', int(ticks))))).u16()

    def bypass_break(self, on=True):
        """Disconnect TIM1's break input so the gate drivers can run on the
        bench.
        """
        return self._ack(GateOp.BYPASS, ON if on else OFF)

    def reset_worst_gap(self):
        """Forget the longest keepalive gap, so a run is measured on its own.
        """
        return bool(Reader(self._op(GateOp.GAP_RESET)).u8())

    def clear_fault(self):
        """Clear the break latch. Does NOT re-arm; the caller asks again."""
        return bool(Reader(self._op(GateOp.CLEAR)).u8())
