"""The board's loop behind `0x6E` device 12: machine.parts in C, rows streamed and held."""
import math

from coaxial.comm import protocol
from coaxial.comm.protocol import CtrlOp
from coaxial.comm.wire import Reader, label, pack
from coaxial.devices.subsystem import Device
from machine.roles import Controller

#: Slots in pass order, as `machine.controller.Feedback` names them.
SLOTS = ('prefilter', 'measure', 'estimator', 'regulator')

#: Kinds as ctrl.h numbers them: the machine.parts class names, 0 none.
KINDS = ('', 'Gain', 'Slew', 'Wrap', 'LowPass', 'SpeedKalman', 'PI', 'AngleHold', 'Direct',
         'SpeedPI')

#: What the loop measures: the A1335's angle, deg; omega_hat, rad/s; iq, A.
MEASURED = ('angle', 'omega_hat', 'iq')

#: Where its command goes: the drive's theta, rad; iq_ref, A.
COMMANDS = ('theta', 'iq_ref')

#: Rows one op 3 carries (`CTRL_ROWS_PER_OP`, cmd_ctrl.c).
ROWS_PER_OP = 40

#: The longest row, ms: a u16.
ROW_MS_MAX = 0xFFFF

MILLI = 1e3

#: Significant digits a `dec` keeps: past float32's, inside an i32.
DEC_DIGITS = 9


def dec(value):
    """`value` as the wire's `dec`: `i32 m, i8 e`, m x 10^e, trailing zeros dropped."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('%r is not a number the board can hold' % value)
    if value == 0.0:
        return pack(('i32', 0), ('i8', 0))
    e = math.floor(math.log10(abs(value))) - (DEC_DIGITS - 1)
    m = int(round(value / 10.0 ** e))
    while m and m % 10 == 0:
        m, e = m // 10, e + 1
    return pack(('i32', m), ('i8', e))


def _index(names, name, what):
    if name not in names:
        raise ValueError('%r is not a %s; they are %s' % (name, what, ', '.join(n for n in names if n)))
    return names.index(name)


def _rows(rows):
    """(seconds, setpoint) as (ms, milli) wire rows; a row past a u16 of ms is several."""
    out = []
    for seconds, setpoint in rows:
        ms, milli = int(round(float(seconds) * MILLI)), int(round(float(setpoint) * MILLI))
        while ms > ROW_MS_MAX:
            out.append((ROW_MS_MAX, milli))
            ms -= ROW_MS_MAX
        out.append((max(ms, 0), milli))
    return out


class Ctrl(Device, Controller, device=protocol.DEVICE_CTRL):

    """A feedback of machine.parts on the board, ticked in the drive's sample; rows in."""

    def state(self):
        """The loop: running, what it measures and drives, the queue, its channels."""
        r = Reader(self._op(CtrlOp.STATE))
        flags = r.u8()
        return {
            'running': bool(flags & 0x01), 'playing': bool(flags & 0x02),
            'measured': label(MEASURED, r.u8(), 'measured'),
            'command': label(COMMANDS, r.u8(), 'command'),
            'hz': r.u16(), 'rows': r.u16(), 'free': r.u16(),
            'queued_s': r.u32() / MILLI,
            'played': r.u32(), 'idle': r.u32(), 'blind': r.u32(),
            'setpoint': r.milli(), 'ref': r.milli(), 'value': r.milli(),
            'estimate': r.milli(), 'out': r.milli(),
        }

    def read(self, count=None, timeout=None):
        return self.state()

    def slot(self, slot, part=None):
        """`part`, a machine.parts part or None, into `slot` (SLOTS); stopped only."""
        index = _index(SLOTS, slot, 'slot')
        kind = _index(KINDS, type(part).__name__, 'kind') if part is not None else 0
        values = [getattr(part, name) for name in part.PARAMS] if part is not None else []
        return self._ack(CtrlOp.SLOT, pack(('u8', index), ('u8', kind), ('u8', len(values)))
                         + b''.join(dec(v) for v in values))

    def load(self, feedback):
        """A `machine.controller.Feedback`'s four slots; stopped only."""
        for name in SLOTS:
            self.slot(name, getattr(feedback, name))
        return True

    def wire(self, measured, command, hz):
        """What it measures (MEASURED), where its command goes (COMMANDS), ticks a second."""
        return self._ack(CtrlOp.WIRE, pack(('u8', _index(MEASURED, measured, 'measured')),
                                           ('u8', _index(COMMANDS, command, 'command')),
                                           ('u16', int(hz))))

    def rows(self, rows):
        """(seconds, setpoint) rows onto the ring, 40 an op, each op all or none; how many."""
        wire = _rows(rows)
        for i in range(0, len(wire), ROWS_PER_OP):
            chunk = wire[i:i + ROWS_PER_OP]
            self._ack(CtrlOp.ROWS, pack(('u8', len(chunk)))
                      + b''.join(pack(('u16', ms), ('i32', milli)) for ms, milli in chunk))
        return len(wire)

    def drop(self):
        """The queued rows dropped; the setpoint held."""
        return self._ack(CtrlOp.CLEAR)

    def write(self, setpoint):
        """One setpoint now, held: a row of no length."""
        return self.rows([(0.0, setpoint)])

    def on(self):
        """The feedback reset, ticking; needs a wire and a regulator."""
        return self._ack(CtrlOp.RUN, pack(('u8', 1)))

    def off(self):
        """Stopped; the drive keeps the last command."""
        return self._ack(CtrlOp.RUN, pack(('u8', 0)))

    def is_on(self):
        return self.state()['running']
