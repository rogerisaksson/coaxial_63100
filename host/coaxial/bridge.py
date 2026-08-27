"""The three-phase bridge: TIM1, the synced phase triple, and Safe Torque Off.

One device because they are one question. Tuning the sample point means
reading where the trigger sits, what came back and whether the STO chain
still holds - and three round trips would sample three different moments.

Nothing here judges a reading. `state()` returns registers and raw codes;
the writers return what the board accepted, which is not always what was
asked for.
"""
from . import protocol
from .errors import RigError
from .subsystem import Subsystem
from .wire import Reader

#: Bit positions in the state reply's first byte, in order.
FLAGS = ('pwm_ready', 'pwm_enabled', 'fault', 'sync_ready', 'sync_armed',
         'afe_on', 'pilot_ok', 'level_ok')

PHASES = 3

OP_STATE = 0
OP_PWM = 1
OP_DUTY = 2
OP_SYNC = 3
OP_TRIGGER = 4
OP_CLEAR = 5
OP_BYPASS = 6
OP_GAP_RESET = 7


class Bridge(Subsystem):

    """TIM1's compare registers, the injected triple and the STO chain."""

    def _op(self, op, payload=b''):
        """One 0x6E request for the bridge device."""
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_BRIDGE, op]) + bytes(payload))

    def state(self):
        """Everything the bridge knows, from one conversion's worth of time.

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
        return out

    def enable(self):
        """Set the master output enable, always at zero duty.

        Raises if the board refused. A latched break outranks the request and
        re-latches the moment it is cleared while nFAULT is still low, so a
        refusal here usually means the STO chain has not released.
        """
        if self._op(OP_PWM, b'\x01')[0] != 1:
            raise RigError('the board refused to enable the bridge - check '
                           'fault, and whether the STO chain has released')
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
        if self._op(OP_DUTY, payload)[0] != 1:
            raise RigError('the board refused %r - past ARR, or the bridge is '
                           'not enabled' % (ticks,))
        return True

    def arm(self):
        """Start latching the injected triple.

        This takes the three converters away from the meter for as long as it
        is armed: the injected sequence needs all three phases preselected at
        once, and the meter clears PCSEL per read.
        """
        if self._op(OP_SYNC, b'\x01')[0] != 1:
            raise RigError('the board refused to arm the synced triple')
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
        """Disconnect TIM1's break input so the bridge can run on the bench.

        Clearing the latch alone cannot work: with PE15 low the break is a
        level, so the hardware holds MOE clear and software cannot set it.
        This drops BDTR.BKE instead.

        What makes it safe is the board, not this call. The STO chain gates
        the gate drivers' own DC/DC, which no MCU pin reaches - with no pilot
        tone the drivers have no supply and the six outputs toggle into
        unpowered inputs. A reset puts the break back.
        """
        if self._op(OP_BYPASS, bytes([1 if on else 0]))[0] != 1:
            raise RigError('the board refused to change the break bypass')
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
