"""One acquisition task: configure, start, read.

DAQmx's shape, cut down to what this board has. There is one task, not many
- one MCU, three converters, one timer - so `configure` replaces whatever
was there rather than adding to it.

Nothing here knows the record's shape. `layout()` asks the board what each
field is and `read()` decodes from that, so a channel added to
`Board/Src/board_adc.c` shows up in a capture without this file being told.
A decoder written against a fixed field order is the copy that goes stale.
"""
import struct
import time

from . import protocol
from .errors import RigError
from .subsystem import Subsystem
from .wire import Reader

#: Clock sources. SOFTWARE is the main loop; TIM1 is the injected group, one
#: record per PWM period, and it carries the three phases and nothing else.
SOFTWARE = 0
TIM1 = 1

CLOCKS = {'software': SOFTWARE, 'tim1': TIM1}

DAQ_OP_STATE = 0
DAQ_OP_CONFIGURE = 1
DAQ_OP_START = 2
DAQ_OP_STOP = 3
DAQ_OP_READ = 4
DAQ_OP_LAYOUT = 5

#: BOARD_UNIT_* as Comms/Inc/board.h numbers them. Getting this wrong
#: labelled the NTC 'mV' and the DC bus 'mA' without changing a value.
UNITS = {0: None, 1: 'mV', 2: 'centi-degC', 3: 'mA'}


class Daq(Subsystem):

    """Configure, trigger, read - against the board's own channel table."""

    def _op(self, op, payload=b''):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_DAQ, op]) + bytes(payload))

    def state(self):
        """What the task is, and what it has produced."""
        r = Reader(self._op(DAQ_OP_STATE))
        flags = r.u8()
        return {
            'running': bool(flags & 0x01),
            'done': bool(flags & 0x02),
            'stride': r.u16(),
            'fields': r.u8(),
            'available': r.u32(),
            'produced': r.u32(),
            'dropped': r.u32(),
            'channels': r.u8(),
            'clock': 'tim1' if r.u8() == TIM1 else 'software',
            'sample_time': r.u8(),
            'decimate': r.u16(),
            'accumulate': r.u16(),
            'records': r.u32(),
        }

    def layout(self):
        """What each field of a record carries, named by the board.

        This is the whole reason `read()` needs no field order of its own.
        """
        r = Reader(self._op(DAQ_OP_LAYOUT))
        fields = r.u8()
        stride = r.u16()
        out = []
        for _ in range(fields):
            index = r.u8()
            unit = r.u8()
            differential = bool(r.u8())
            out.append({'channel': index, 'unit': UNITS.get(unit, unit),
                        'differential': differential, 'signal': r.string()})
        return {'stride': stride, 'fields': out}

    def _resolve(self, channels):
        """Channel names or indices to a bitmask, asking the board for names."""
        if isinstance(channels, int):
            return channels

        mask = 0
        for c in channels:
            mask |= 1 << (c if isinstance(c, int)
                          else self.board.analog.index_of(c))
        return mask

    def configure(self, channels, clock='software', sample_time=0,
                  decimate=1, accumulate=1, records=0):
        """Replace the task. Refused while one is running.

        `accumulate` sums, it does not average: summing keeps the bits an
        average would throw away and the count is right here to divide by.
        `decimate` keeps one trigger in N. `records` of 0 runs until stopped.
        """
        if clock not in CLOCKS and clock not in (SOFTWARE, TIM1):
            raise ValueError('clock is %s, not one of %s'
                             % (clock, ', '.join(CLOCKS)))
        if decimate < 1 or accumulate < 1:
            raise ValueError('decimate and accumulate count samples, so both '
                             'are at least 1')

        payload = struct.pack('>BBBHHI', self._resolve(channels),
                              CLOCKS.get(clock, clock), sample_time,
                              decimate, accumulate, records)
        if self._op(DAQ_OP_CONFIGURE, payload)[0] != 1:
            raise RigError('the board refused that task - a TIM1 clock '
                           'carries only the phases, a task cannot be '
                           'reconfigured while running, and sample_time is '
                           '0..7')
        return self.layout()

    def start(self):
        if self._op(DAQ_OP_START)[0] != 1:
            raise RigError('the board refused to start - configure it first')
        return True

    def stop(self):
        self._op(DAQ_OP_STOP)
        return True

    def read(self, want=0, layout=None):
        """Whole records, oldest first, decoded from the board's layout.

        Pass `layout` to save a round trip when draining in a loop.
        """
        fields = (layout or self.layout())['fields']
        raw = self._op(DAQ_OP_READ, bytes([min(int(want), 255)]))
        got = raw[0]
        stride = 4 + 4 * len(fields)
        out = []
        for i in range(got):
            at = 1 + i * stride
            values = struct.unpack('>I%di' % len(fields), raw[at:at + stride])
            out.append({'at': values[0],
                        **{f['signal']: v
                           for f, v in zip(fields, values[1:])}})
        return out

    def acquire(self, channels, records, clock='software', sample_time=0,
                decimate=1, accumulate=1, timeout=10.0):
        """Configure, start, drain until the task says done. The one-shot.

        Raises rather than returning a short capture: a run that stopped
        early is a different measurement, not a smaller one.
        """
        layout = self.configure(channels, clock=clock, sample_time=sample_time,
                                decimate=decimate, accumulate=accumulate,
                                records=records)
        self.start()

        out = []
        deadline = time.time() + timeout
        while len(out) < records:
            batch = self.read(layout=layout)
            if batch:
                out.extend(batch)
                continue
            if time.time() > deadline:
                self.stop()
                raise RigError('%d of %d records in %.1f s - %s'
                               % (len(out), records, timeout, self.state()))
            time.sleep(0.001)

        self.stop()
        return out[:records], layout
