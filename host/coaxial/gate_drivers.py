"""The three-phase gate drivers: TIM1, the synced phase triple, and Safe Torque Off.

One device because they are one question. Tuning the sample point means
reading where the trigger sits, what came back and whether the STO chain
still holds - and three round trips would sample three different moments.

Nothing here judges a reading. `state()` returns registers and raw codes;
the writers return what the board accepted, which is not always what was
asked for.
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
    """Three compare values on the wire, or a raise: a half update would
    run one cycle with two phases from this call and one from the last."""
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
        """Read the dead time, or set it and its skew.

        `nanoseconds` None reads. Setting floors at 20 ns on the board -
        the 2EDL8034 has no interlock, so this is the only thing between
        the two FETs of a leg - and refuses in the board's own words.

        `skew` is in DTG counts and trims a stage whose two transitions are
        not symmetric: positive lengthens the dead time on the transition
        the counter reaches counting up and shortens the other by the same,
        so the pair still averages what was asked for. **Not measured** -
        what it does at the gates needs two probes and a scope.

        Returns what the board reads back, which is not always what was
        asked: nanoseconds land on a DTG count, and DTG counts are 4.21 ns
        apart at 237.5 MHz.
        """
        if nanoseconds is None:
            state = self.state()
            return {'nanoseconds': state['deadtime_ns'],
                    'skew': state['deadtime_skew'],
                    'floor': state['deadtime_floor']}

        reply = self._op(GateOp.DEADTIME,
                         pack(('u32', int(nanoseconds)), ('i8', int(skew))))
        # took() raises on a refusal and returns True otherwise, so the
        # reader is built here and the took byte read off it.
        r = Reader(reply)
        self.took(reply)
        r.u8()
        return {'nanoseconds': r.u32(), 'skew': r.i8(), 'floor': r.u8()}

    def state(self):
        """Everything the gate drivers know, from one conversion's worth of time.

        `at` is TIM1->CNT as the interrupt read it, not the instant the
        sample was taken: measured, the handler runs about 965 ticks
        (4.06 us) after the trigger. The sample point itself is `trigger`.
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
        # period. With the dither running they differ by a tick most of the
        # time and that is the point, not a rounding.
        # TICKS, not a fraction: the board sends Q16.16 of a CCR count,
        # so this is `requested_ticks` against `period`. Reading it as a
        # duty drew 118700 % on a stage running at half.
        out['requested_ticks'] = tuple(r.q16() for _ in range(PHASES))
        # Six gate signals in one IDR load with TIM1->CNT beside it: six
        # separate asks at 50 kHz can straddle an edge and show a leg with
        # both FETs on, the one state dead time prevents.
        #
        # ONE INSTANT, NOT A DUTY. Averaging is only honest while `pins_at`
        # spreads across the period; with the sync armed CNT lands in the
        # same band every time - measured, 89.5 % high at 50 % duty.
        out['pins'] = r.flags(GATES)
        out['pins_at'] = r.u16()
        out['deadtime_ns'] = r.u32()
        out['deadtime_skew'] = r.i8()
        out['deadtime_floor'] = r.u8()
        # Which legs have their two gate pins on one node. A joined pair
        # cannot go complementary, so that leg never switches: its driver
        # sees a level and the phase node floats. The board measures it by
        # borrowing the pins, so it reads no legs while armed.
        out['gate_shorts'] = tuple(leg for leg, joined in r.flags(LEGS).items()
                                   if joined)
        # The DC link the injected sequence read beside the triple, raw
        # single-ended - rank 2 on ADC3, MINOR 2. Older firmware stops
        # before it.
        out['dcbus_raw'] = r.maybe('u32')
        # And the NTC, rank 2 on ADC1 - the thermal observer's thermometer
        # while the drive holds the converters.
        out['ntc_raw'] = r.maybe('u32')
        # Periods left of a counted hold, MINOR 8. Zero when free-running.
        out['periods_left'] = r.maybe('u32')
        return out

    def enable(self):
        """Set the master output enable, always at zero duty.

        Raises if the board refused. A latched break outranks the request and
        re-latches the moment it is cleared while nFAULT is still low, so a
        refusal here usually means the STO chain has not released.
        """
        return self._ack(GateOp.PWM, ON)

    def disable(self):
        """Clear MOE. Every output drops to its idle level in hardware."""
        self._op(GateOp.PWM, OFF)
        return True

    def duty(self, ticks, periods=0):
        """All three compare registers, or none of them.

        `ticks` is three compare values against `period - 1`. A half update
        would run one cycle with two phases from this call and one from the
        last, which is a step nobody asked for.

        `periods` > 0 rides as an optional u32 (MINOR 8): the board's
        update interrupt zeroes the compares after exactly that many PWM
        periods - 500 is 10.000 ms at 50 kHz, where a link-timed hold was
        93-108. Older firmware refuses the longer payload in its own words.
        """
        counted = pack(('u32', int(periods))) if periods else b''
        return self._ack(GateOp.DUTY, _triple(ticks) + counted)

    def alternate(self, ticks_a, ticks_b):
        """Two compare triples, A one PWM period and B the next, swapped by
        TIM1's update interrupt for as long as they stand.

        What one host write per 15 ms cannot do: a phase pair driven back
        and forth every 20 us - A = (d, 0, 0), B = (0, d, 0) is U high
        against V low, then V high against U low. Whole ticks against
        `period - 1`; the next duty() or duty_fine() ends it.
        """
        return self._ack(GateOp.ALTERNATE, _triple(ticks_a) + _triple(ticks_b))

    def duty_fine(self, fractions):
        """Duty as a fraction of full scale, dithered to hit it exactly.

        One tick of the period is 0.0421 % at ARR 2375, so an asked-for
        0.3454 is 820.32 ticks and neither 820 nor 821 is it. The board
        keeps the fraction and a first-order sigma-delta in TIM1's update
        interrupt pays it back, so the **mean** duty is what was asked for.

        That costs idle tones: the dither pattern is periodic and its lines
        sit below the switching frequency. First order buys three adds in a
        50 kHz interrupt, and this is where the price is written down.
        """
        fractions = tuple(fractions)
        if len(fractions) != PHASES:
            raise ValueError('%d duties, not %d' % (PHASES, len(fractions)))

        period = self.state()['period'] - 1
        return self._ack(GateOp.DUTY_FINE,
                         pack(*(('u32', q16(_unit(f) * period))
                                for f in fractions)))

    def arm(self):
        """Start latching the injected triple.

        This takes the three converters away from the meter for as long as it
        is armed: the injected sequence needs all three phases preselected at
        once, and the meter clears PCSEL per read.
        """
        return self._ack(GateOp.SYNC, ON)

    def disarm(self):
        """Stop latching, and give the converters back to the meter."""
        self._op(GateOp.SYNC, OFF)
        return True

    def trigger(self, ticks=None):
        """Where in the PWM period the triple is taken, as CCR4 in ticks.

        Returns CCR4 as it reads back, which is the only answer worth
        having: a value past ARR changes nothing and the reply says so.
        Zero disables the trigger outright - OC4REF in PWM1 mode never goes
        active - so the triples stop rather than moving.
        """
        if ticks is None:
            return self.state()['trigger']
        return Reader(self._op(GateOp.TRIGGER,
                               pack(('u16', int(ticks))))).u16()

    def bypass_break(self, on=True):
        """Disconnect TIM1's break input so the gate drivers can run on the bench.

        Clearing the latch alone cannot work: with PE15 low the break is a
        level, so the hardware holds MOE clear and software cannot set it.
        This drops BDTR.BKE instead.

        What makes it safe is the board, not this call. The STO chain gates
        the gate drivers' own DC/DC, which no MCU pin reaches - with no pilot
        tone the drivers have no supply and the six outputs toggle into
        unpowered inputs. A reset puts the break back.
        """
        return self._ack(GateOp.BYPASS, ON if on else OFF)

    def reset_worst_gap(self):
        """Forget the longest keepalive gap, so a run is measured on its own.

        The gap is raw CYCCNT ticks, not microseconds: dividing cycles down
        moves the wrap off a power of two and the unsigned arithmetic breaks
        across it. Divide by the core clock here, where nothing wraps.
        """
        return bool(Reader(self._op(GateOp.GAP_RESET)).u8())

    def clear_fault(self):
        """Clear the break latch. Does NOT re-arm; the caller asks again."""
        return bool(Reader(self._op(GateOp.CLEAR)).u8())
