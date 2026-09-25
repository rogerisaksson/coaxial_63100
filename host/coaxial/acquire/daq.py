"""One acquisition task: configure, start, read."""
import itertools
import struct
import time
from typing import Any

from coaxial.acquire.acquisition import Acquisition
from coaxial.comm import protocol
from coaxial.comm.hostclock import clock_of
from coaxial.comm.protocol import DaqOp
from coaxial.comm.wire import BYTE_FRACTION, Reader, pack
from coaxial.devices.subsystem import Device
from coaxial.errors import RigError

#: Clock sources. SOFTWARE is the main loop; TIM1 is the injected group, one
#: record per PWM period, and it carries the three phases and nothing else.
SOFTWARE = 0
TIM1 = 1

CLOCKS = {'software': SOFTWARE, 'tim1': TIM1}
CLOCK_NAMES = {code: name for name, code in CLOCKS.items()}

#: The state reply's first byte, bit by bit. `lost_power` is stopped
#: because AFE_ON went off, and the buffers emptied with it: that pin
#: powers the ADC reference, so anything held would have divided out to a
#: plausible mid-scale (invariant 9).
FLAGS = ('running', 'done', 'lost_power')

#: Coefficients cross as Q28 - the wire carries no floating point, and a
#: biquad's a1 reaches -2, so 2^28 leaves a range of +/-8 and a
#: resolution three orders inside what a float holds anyway.
COEFF_SCALE = 1 << 28

#: A biquad section is five coefficients.
SECTION = 5

#: Bytes of records one read answers (`DAQ_REPLY_ROOM`, cmd_daq.c).
REPLY_ROOM = 240


def _sections(sections):
    """Biquad sections as five Q28 words each, in the order given."""
    return b''.join(pack(*(('i32', int(round(c * COEFF_SCALE)))
                           for c in section))
                    for section in sections)


class Daq(Device, Acquisition, device=protocol.DEVICE_DAQ):

    """Configure, trigger, read - against the board's own channel table."""

    #: Records still in the board's ring after the last `acquire()`,
    #: straight off that same reply. None before the first read, and on
    #: a board older than protocol MINOR 5.
    backlog = None

    def state(self):
        """What the task is, what it has produced, and how full it is."""
        r = Reader(self._op(DaqOp.STATE))
        state: dict[str, Any] = r.flags(FLAGS)
        state.update({
            'stride': r.u16(),
            'fields': r.u8(),
            'available': r.u32(),
            'produced': r.u32(),
            'dropped': r.u32(),
            'channels': r.u16(),
            'clock': CLOCK_NAMES.get(r.u8(), 'software'),
            'sample_time': r.u8(),
            'decimate': r.u16(),
            'accumulate': r.u16(),
            'records': r.u32(),
            'digital': bool(r.u8()),
            'interval_us': r.u32(),
            'max_rate_hz': r.u32(),
        })
        # Appended by MINOR 4, and read only if it is there: a board older than
        # that answers a shorter reply, and a decoder that assumed the field
        # would raise on it.
        state['capacity'] = r.maybe('u32')
        state['worst'] = r.maybe('u32')
        state['rung'] = r.maybe('u8') or 0
        state['rungs'] = r.maybe('u8') or 0
        state['rung_changes'] = r.maybe('u32') or 0
        # Sweeps, not records: what the loop manages underneath the decimation.
        state['triggers'] = r.maybe('u32')
        # Appended, MINOR 7: which sensor fields this build can put in a
        # record, and which the task carries now.
        state['sensors'] = r.maybe('u16')
        state['sensors_available'] = r.maybe('u16')
        state['sensors_supported'] = state['sensors_available'] is not None
        return state

    def layout(self):
        """What each field of a record carries, named by the board."""
        r = Reader(self._op(DaqOp.LAYOUT))
        fields = r.u8()
        stride = r.u16()
        out = [self._field(r) for _ in range(fields)]

        # The digital word's bits, named by the board.
        pins = []
        if r.remaining and r.u8():
            pins = [self._pin(r) for _ in range(r.u8())]
        return {'stride': stride, 'fields': out, 'pins': pins}

    @staticmethod
    def _field(r):
        index = r.u8()
        unit = r.u8()
        differential = bool(r.u8())
        return {'channel': index, 'unit': protocol.CHANNEL_UNITS.get(unit, unit),
                'differential': differential, 'signal': r.string()}

    @staticmethod
    def _pin(r):
        direction = protocol.DIRECTIONS.get(r.u8())
        return {'signal': r.string(), 'direction': direction}

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
                  decimate=1, accumulate=None, records=0, digital=False,
                  sample_rate=None, interval_us=None, adapt=False,
                  sensors=0):
        """Replace the task. Refused while one is running."""
        if accumulate is None:
            accumulate = 0 if sample_rate is not None else 1
        # Only what stops the request being formed is checked here: a name
        # that is not a clock cannot be packed into a byte.
        if clock not in CLOCKS and clock not in CLOCK_NAMES:
            raise ValueError('clock is %s, not one of %s'
                             % (clock, ', '.join(CLOCKS)))

        # The software clock's period, from `sample_rate`.
        if interval_us is None:
            interval_us = (0 if sample_rate is None
                           else int(1e6 / float(sample_rate)))

        # The mask is 16 bits: the ninth channel did not fit in eight, and a
        # mask that silently dropped one would configure a task the caller did
        # not ask for.
        payload = pack(('u16', self._resolve(channels)),
                       ('u8', CLOCKS.get(clock, clock)),
                       ('u8', sample_time), ('u16', decimate),
                       ('u16', accumulate), ('u32', records),
                       ('u8', int(bool(digital))), ('u32', int(interval_us)),
                       ('u8', int(bool(adapt))))
        if sensors:
            # Appended, MINOR 7: snapshot fields, software clock only.
            payload += pack(('u16', int(sensors)))
        self._ack(DaqOp.CONFIGURE, payload)
        return self.layout()

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain `coaxial.acquire.bessel` designed."""
        self._ack(DaqOp.FILTER,
                  pack(('u8', len(sections)), ('u16', int(decimate)))
                  + _sections(sections))
        return True

    #: What the generator makes. SINE has a frequency, so the chain's
    #: answer to it is a gain and a phase. RAMP is `offset + (n * hz) mod
    #: amplitude`, an integer sequence a host computes in closed form -
    #: which is what lets every record be checked exactly rather than
    #: statistically.
    SINE, RAMP = 0, 1

    def ladder(self, chains):
        """Load the whole ladder, bottom rung first."""
        for n, chain in enumerate(chains):
            self._ack(DaqOp.RUNG,
                      pack(('u8', n), ('u16', int(chain['boxcar'])),
                           ('u8', len(chain['sections'])),
                           ('u16', int(chain['decimate'])))
                      + _sections(chain['sections']))
        return True

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """A known sine in the converter's place, or `hz=0` for the converter
        again.
        """
        self._ack(DaqOp.TONE,
                  pack(('u32', int(hz)), ('u32', int(rate_hz)),
                       ('i32', int(amplitude)), ('i32', int(offset)),
                       ('u8', int(kind))))
        return True

    def start(self):
        return self._ack(DaqOp.START)

    def stop(self):
        self._op(DaqOp.STOP)
        return True

    def decode(self, blob, layout=None):
        """Whole records out of raw record bytes."""
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        sensors = layout.get('sensors') or []
        words = sum(x['words'] for x in sensors)
        stride = layout['stride']
        fmt = '>I%di%dB%dhH' % (len(fields), len(pins), words)
        return [self._record(struct.unpack(fmt, blob[at:at + stride]),
                             fields, pins, sensors)
                for at in range(0, len(blob) // stride * stride, stride)]

    @staticmethod
    def _record(values, fields, pins, sensors):
        """One record from its unpacked words, laid out as the board said."""
        rec = {'at': values[0], 'samples': values[-1]}
        rec.update({f['signal']: v for f, v in zip(fields, values[1:])})
        first = 1 + len(fields)
        if pins:
            # A duty, not a level: the pin went through the same window as
            # everything else, and 255 is all of it.
            rec['digital'] = {p['signal']: values[first + n] / BYTE_FRACTION
                              for n, p in enumerate(pins)}
        first += len(pins)
        if sensors:
            # Snapshots, not sums: raw and source-defined, as device 5
            # carries them; the scale stays this host's.
            ends = list(itertools.accumulate((x['words'] for x in sensors),
                                             initial=first))
            rec['sensors'] = {x['signal']: tuple(values[a:b])
                              for x, a, b in zip(sensors, ends, ends[1:])}
        return rec

    def acquire(self, want=0, layout=None):
        """Whole records, oldest first, decoded from the board's layout."""
        layout = layout or self.layout()
        # The board's stride, not one worked out here.
        stride = layout['stride']
        # The reply's length is known: the first payload byte is the record
        # count and the stride is in hand, so the read skips the 8 ms of
        # silence that ends every other transaction.
        raw = self._op(DaqOp.READ, pack(('u8', min(int(want), 255))),
                       reply_shape={'at': 0, 'head': 1, 'stride': stride,
                                    'tail': 4})
        got = raw[0]
        end = 1 + (got * stride)
        out = self.decode(raw[1:end], layout)

        # The backlog the read itself answered: records still in the board's
        # ring the instant this read took its own.
        self.backlog = Reader(raw[end:]).maybe('u32')
        return out

    def latest(self, layout=None, block=True, timeout=2.0, poll=0.002):
        """The live accumulator, taken and reset. Cannot overflow."""
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        clock = clock_of(self)
        deadline = clock.now() + timeout

        while True:
            r = Reader(self._op(DaqOp.LIVE))
            if r.u8():
                break
            if not block:
                return None
            if clock.now() > deadline:
                raise RigError('no sample in %.1f s - is the task running? %s'
                               % (timeout, self.state()))
            time.sleep(poll)

        out = {'first': r.u32(), 'last': r.u32(), 'sum': {}, 'count': {}}
        # One count per channel, not one for the lot: the board reads one
        # channel per turn of its loop, so over any window they have had
        # different numbers of samples and a single count would divide most of
        # them by the wrong number.
        out['lowest'], out['highest'] = {}, {}
        for f in fields:
            name = f['signal']
            out['sum'][name] = r.i32()
            out['count'][name] = r.u32()
            # What the channel did in the window, measured.
            out['lowest'][name] = r.i32()
            out['highest'][name] = r.i32()
        out['mean'] = {k: (v / out['count'][k] if out['count'][k] else None)
                       for k, v in out['sum'].items()}
        if pins:
            bits = r.u32()
            out['digital'] = {p['signal']: bool(bits >> n & 1)
                              for n, p in enumerate(pins)}
        return out
