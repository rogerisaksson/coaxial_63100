"""The three-phase gate drivers: TIM1, the synced phase triple, and Safe Torque Off.

One device because they are one question. Tuning the sample point means
reading where the trigger sits, what came back and whether the STO chain
still holds - and three round trips would sample three different moments.

Nothing here judges a reading. `state()` returns registers and raw codes;
the writers return what the board accepted, which is not always what was
asked for.
"""
import struct

from . import protocol
from .gates import GateControl
from .subsystem import Subsystem
from .wire import Reader

#: Bit positions in the state reply's first byte, in order.
FLAGS = ('pwm_ready', 'pwm_enabled', 'fault', 'sync_ready', 'sync_armed',
         'afe_on', 'pilot_ok', 'level_ok')

PHASES = 3


def _signed8(value):
    """A skew off the wire, which carries it unsigned."""
    return value - 256 if value & 0x80 else value

#: PE8..PE13 in pin order, which is low side then high side per leg.
GATES = ('UL', 'UH', 'VL', 'VH', 'WL', 'WH')

OP_STATE = 0
OP_DEADTIME = 9
OP_PWM = 1
OP_DUTY = 2
OP_SYNC = 3
OP_TRIGGER = 4
OP_CLEAR = 5
OP_BYPASS = 6
OP_DUTY_FINE = 8
OP_GAP_RESET = 7


class GateDrivers(Subsystem, GateControl):

    """TIM1's compare registers, the injected triple and the STO chain."""

    def _op(self, op, payload=b''):
        """One 0x6E request for the gate drivers device."""
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_GATE_DRIVERS, op]) + bytes(payload))

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

        reply = self._op(OP_DEADTIME,
                         struct.pack('>Ib', int(nanoseconds), int(skew)))
        # took() raises on a refusal and returns True otherwise, so the
        # reader is built here and the took byte read off it.
        r = Reader(reply)
        self.took(reply)
        r.u8()
        return {'nanoseconds': r.u32(), 'skew': _signed8(r.u8()),
                'floor': r.u8()}

    def state(self):
        """Everything the gate drivers know, from one conversion's worth of time.

        `at` is TIM1->CNT as the interrupt read it, not the instant the
        sample was taken: measured, the handler runs about 965 ticks
        (4.06 us) after the trigger. The sample point itself is `trigger`.
        """
        r = Reader(self._op(OP_STATE))
        flags = r.u8()
        out = {name: bool(flags >> i & 1) for i, name in enumerate(FLAGS)}
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
        out['break_bypassed'] = bool(r.u8() & 0x01)
        # Asked for, in ticks Q16.16, beside what the register holds this
        # period. With the dither running they differ by a tick most of the
        # time and that is the point, not a rounding.
        # TICKS, not a fraction: the board sends Q16.16 of a CCR count,
        # so this is `requested_ticks` against `period`. Reading it as a
        # duty drew 118700 % on a stage running at half.
        out['requested_ticks'] = tuple(r.u32() / 65536.0
                                       for _ in range(PHASES))
        # Six gate signals in one IDR load with TIM1->CNT beside it: six
        # separate asks at 50 kHz can straddle an edge and show a leg with
        # both FETs on, the one state dead time prevents.
        #
        # ONE INSTANT, NOT A DUTY. Averaging is only honest while `pins_at`
        # spreads across the period; with the sync armed CNT lands in the
        # same band every time - measured, 89.5 % high at 50 % duty.
        pins = r.u8()
        out['pins'] = {name: bool(pins >> i & 1) for i, name in enumerate(GATES)}
        out['pins_at'] = r.u16()
        out['deadtime_ns'] = r.u32()
        out['deadtime_skew'] = _signed8(r.u8())
        out['deadtime_floor'] = r.u8()
        # Which legs have their two gate pins on one node. A joined pair
        # cannot go complementary, so that leg never switches: its driver
        # sees a level and the phase node floats. The board measures it by
        # borrowing the pins, so it reads no legs while armed.
        shorts = r.u8()
        out['gate_shorts'] = tuple(
            name for i, name in enumerate(('U', 'V', 'W')) if shorts >> i & 1)
        return out

    def enable(self):
        """Set the master output enable, always at zero duty.

        Raises if the board refused. A latched break outranks the request and
        re-latches the moment it is cleared while nFAULT is still low, so a
        refusal here usually means the STO chain has not released.
        """
        self.took(self._op(OP_PWM, b'\x01'))
        return True

    def disable(self):
        """Clear MOE. Every output drops to its idle level in hardware."""
        self._op(OP_PWM, b'\x00')
        return True

    def duty(self, ticks):
        """All three compare registers, or none of them.

        `ticks` is three compare values against `period - 1`. A half update
        would run one cycle with two phases from this call and one from the
        last, which is a step nobody asked for.
        """
        ticks = tuple(ticks)
        if len(ticks) != PHASES:
            raise ValueError('%d compare values, not %d' % (PHASES, len(ticks)))

        payload = b''.join(int(t).to_bytes(2, 'big') for t in ticks)
        self.took(self._op(OP_DUTY, payload))
        return True

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
        payload = b''.join(
            int(round(max(0.0, min(1.0, f)) * period * 65536)).to_bytes(4, 'big')
            for f in fractions)
        self.took(self._op(OP_DUTY_FINE, payload))
        return True

    def arm(self):
        """Start latching the injected triple.

        This takes the three converters away from the meter for as long as it
        is armed: the injected sequence needs all three phases preselected at
        once, and the meter clears PCSEL per read.
        """
        self.took(self._op(OP_SYNC, b'\x01'))
        return True

    def disarm(self):
        """Stop latching, and give the converters back to the meter."""
        self._op(OP_SYNC, b'\x00')
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
        return int.from_bytes(self._op(OP_TRIGGER,
                                       int(ticks).to_bytes(2, 'big')), 'big')

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
        self.took(self._op(OP_BYPASS, bytes([1 if on else 0])))
        return True

    def reset_worst_gap(self):
        """Forget the longest keepalive gap, so a run is measured on its own.

        The gap is raw CYCCNT ticks, not microseconds: dividing cycles down
        moves the wrap off a power of two and the unsigned arithmetic breaks
        across it. Divide by the core clock here, where nothing wraps.
        """
        return self._op(OP_GAP_RESET)[0] == 1

    def clear_fault(self):
        """Clear the break latch. Does NOT re-arm; the caller asks again."""
        return self._op(OP_CLEAR)[0] == 1
