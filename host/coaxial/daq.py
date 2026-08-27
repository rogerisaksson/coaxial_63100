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
DAQ_OP_LIVE = 6

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
            # Stopped because AFE_ON went off, and the buffers emptied with
            # it: that pin powers the ADC reference, so anything held would
            # have divided out to a plausible mid-scale (invariant 9).
            'lost_power': bool(flags & 0x04),
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
            'digital': bool(r.u8()),
            'interval_us': r.u32(),
            'max_rate_hz': r.u32(),
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

        # The digital word's bits, named by the board. Counting rows of a
        # table this file does not hold is the copy the layout exists to
        # avoid, so the names come off the wire with everything else.
        pins = []
        if r.remaining and r.u8():
            for _ in range(r.u8()):
                direction = ('in', 'out', 'inout')[r.u8()]
                pins.append({'signal': r.string(), 'direction': direction})
        return {'stride': stride, 'fields': out, 'pins': pins}

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
                  decimate=1, accumulate=1, records=0, digital=False,
                  rate_hz=None, interval_us=None):
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

        # A software clock has to be a clock. Left unlimited it samples
        # whatever the main loop has spare, which took the link down: seven
        # channels is about 190 us of converter work a turn and RTU discards
        # a frame whose characters arrive more than t1.5 - 143 us at 115200 -
        # apart. Unlimited is still reachable, and is only safe for a short
        # finite run.
        if interval_us is None:
            interval_us = 0 if rate_hz is None else int(1e6 / float(rate_hz))

        payload = struct.pack('>BBBHHIBI', self._resolve(channels),
                              CLOCKS.get(clock, clock), sample_time,
                              decimate, accumulate, records,
                              1 if digital else 0, int(interval_us))
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
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        raw = self._op(DAQ_OP_READ, bytes([min(int(want), 255)]))
        got = raw[0]
        stride = 4 + 4 * len(fields) + (4 if pins else 0)
        fmt = '>I%di%s' % (len(fields), 'I' if pins else '')
        out = []
        for i in range(got):
            at = 1 + i * stride
            values = struct.unpack(fmt, raw[at:at + stride])
            rec = {'at': values[0]}
            rec.update({f['signal']: v for f, v in zip(fields, values[1:])})
            if pins:
                bits = values[1 + len(fields)]
                rec['digital'] = {p['signal']: bool(bits >> n & 1)
                                  for n, p in enumerate(pins)}
            out.append(rec)
        return out

    def latest(self, layout=None, block=True, timeout=2.0, poll=0.002):
        """The live accumulator, taken and reset. Cannot overflow.

        Every trigger adds to it and this takes it away, so a late reader
        gets a wider averaging window rather than a backlog - the opposite
        of `read()`, which drains a ring that drops when it is full. Message
        in a bottle or fibre, the same call.

        Returns each channel's sum and the number of additions that went
        into it - `arr[channel][additions]` - because the channels do not
        sample at the same rate. `mean` is done for you. `block` waits for a sample that has not been taken yet, on
        this side: a slave that sat on a reply waiting for one would break
        RTU framing for everyone else on the segment.
        """
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        deadline = time.time() + timeout

        while True:
            r = Reader(self._op(DAQ_OP_LIVE))
            if r.u8():
                break
            if not block:
                return None
            if time.time() > deadline:
                raise RigError('no sample in %.1f s - is the task running? %s'
                               % (timeout, self.state()))
            time.sleep(poll)

        out = {'first': r.u32(), 'last': r.u32(), 'sum': {}, 'count': {}}
        # One count per channel, not one for the lot: the board reads one
        # channel per turn of its loop, so over any window they have had
        # different numbers of samples and a single count would divide most
        # of them by the wrong number.
        out['lowest'], out['highest'] = {}, {}
        for f in fields:
            name = f['signal']
            out['sum'][name] = r.i32()
            out['count'][name] = r.u32()
            # What the channel did in the window, measured. A mean and a
            # count cannot tell you a spike happened.
            out['lowest'][name] = r.i32()
            out['highest'][name] = r.i32()
        out['mean'] = {k: (v / out['count'][k] if out['count'][k] else None)
                       for k, v in out['sum'].items()}
        if pins:
            bits = r.u32()
            out['digital'] = {p['signal']: bool(bits >> n & 1)
                              for n, p in enumerate(pins)}
        return out

    def drain(self, limit=None, layout=None):
        """Block read: keep taking full frames until the ring reports empty.

        One `read()` is already a full Modbus PDU, so blocking bigger buys
        no bytes - the link is the ceiling. At 115200 a 253-byte reply is
        22 ms, which is 11.5 kB/s however it is cut up, and a 36-byte record
        makes that about 320 records/s. Past that the only thing that helps
        is producing fewer records, which is what `accumulate` and
        `decimate` do on the target before a byte is sent.
        """
        layout = layout or self.layout()
        out = []
        while limit is None or len(out) < limit:
            batch = self.read(layout=layout)
            if not batch:
                break
            out.extend(batch)
        return out[:limit] if limit is not None else out

    def acquire(self, channels, records, clock='software', sample_time=0,
                decimate=1, accumulate=1, timeout=10.0, digital=False,
                rate_hz=None):
        """Configure, start, drain until the task says done. The one-shot.

        Raises rather than returning a short capture: a run that stopped
        early is a different measurement, not a smaller one.
        """
        layout = self.configure(channels, clock=clock, sample_time=sample_time,
                                decimate=decimate, accumulate=accumulate,
                                records=records, digital=digital,
                                rate_hz=rate_hz)
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
