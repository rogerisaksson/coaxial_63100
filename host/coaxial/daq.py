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
from .acquisition import Acquisition
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
DAQ_OP_FILTER = 7
DAQ_OP_TONE = 8

#: Coefficients cross as Q28 - the wire carries no floating point, and a
#: biquad's a1 reaches -2, so 2^28 leaves a range of +/-8 and a
#: resolution three orders inside what a float holds anyway.
COEFF_SCALE = 1 << 28

#: BOARD_UNIT_* as Comms/Inc/board.h numbers them. Getting this wrong
#: labelled the NTC 'mV' and the DC bus 'mA' without changing a value.
UNITS = {0: None, 1: 'mV', 2: 'centi-degC', 3: 'mA'}


class Daq(Subsystem, Acquisition):

    """Configure, trigger, read - against the board's own channel table."""

    def _op(self, op, payload=b''):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_DAQ, op]) + bytes(payload))

    def state(self):
        """What the task is, what it has produced, and how full it is.

        `available` against `capacity` is the buffer level; `worst` is the
        fullest it has been, which is the number that says whether the
        next record drops. Both are None on a board older than MINOR 4.
        """
        r = Reader(self._op(DAQ_OP_STATE))
        flags = r.u8()
        state = {
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
            'channels': r.u16(),
            'clock': 'tim1' if r.u8() == TIM1 else 'software',
            'sample_time': r.u8(),
            'decimate': r.u16(),
            'accumulate': r.u16(),
            'records': r.u32(),
            'digital': bool(r.u8()),
            'interval_us': r.u32(),
            'max_rate_hz': r.u32(),
        }
        # Appended by MINOR 4, and read only if it is there: a board
        # older than that answers a shorter reply, and a decoder that
        # assumed the field would raise on a board that is simply older.
        if r.remaining >= 8:
            state['capacity'] = r.u32()
            state['worst'] = r.u32()
        else:
            state['capacity'] = state['worst'] = None
        return state

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
                  decimate=1, accumulate=None, records=0, digital=False,
                  sample_rate=None, interval_us=None):
        """Replace the task. Refused while one is running.

        `sample_rate` is what the HOST gets, in records a second. The
        converter is not slowed to it: it runs at whatever the loop
        manages - megasamples a second is the reason to sum on the target
        at all - and the board closes a record on the clock, carrying the
        sum of everything that arrived and the count that made it. Ask for
        a rate the link can drain and the averaging is free.

        `accumulate` is the other way to close a record: N samples make
        one, and `sample_rate` then gates the triggers instead. Left unset
        it follows `sample_rate` - a rate means the clock closes it, no
        rate means one sample per record.

        Summing rather than averaging keeps the bits an average would
        throw away; `record['samples']` is the divisor and comes off the
        wire with the sums. `decimate` keeps one trigger in N. `records`
        of 0 runs until stopped.
        """
        if accumulate is None:
            accumulate = 0 if sample_rate is not None else 1
        # Only what stops the request being FORMED is checked here - a name
        # that is not a clock cannot be packed into a byte. Everything the
        # board can judge, the board judges, and says why.
        if clock not in CLOCKS and clock not in (SOFTWARE, TIM1):
            raise ValueError('clock is %s, not one of %s'
                             % (clock, ', '.join(CLOCKS)))

        # A software clock has to be a clock. Left unlimited it samples
        # whatever the main loop has spare, which took the link down: seven
        # channels is about 190 us of converter work a turn and RTU discards
        # a frame whose characters arrive more than t1.5 - 143 us at 115200 -
        # apart. Unlimited is still reachable, and is only safe for a short
        # finite run.
        if interval_us is None:
            interval_us = (0 if sample_rate is None
                           else int(1e6 / float(sample_rate)))

        # The mask is 16 bits: the ninth channel did not fit in eight, and
        # a mask that silently dropped one would configure a task the
        # caller did not ask for.
        payload = struct.pack('>HBBHHIBI', self._resolve(channels),
                              CLOCKS.get(clock, clock), sample_time,
                              decimate, accumulate, records,
                              1 if digital else 0, int(interval_us))
        self.took(self._op(DAQ_OP_CONFIGURE, payload))
        return self.layout()

    def shape(self, sections=(), decimate=1):
        """Load the anti-alias chain `coaxial.bessel` designed.

        THE CHAIN'S BOXCAR IS THE TASK'S `accumulate`, not a second
        stage: configure with `accumulate=chain['boxcar']` and pass the
        sections and `chain['decimate']` here. Two boxcars would be two
        answers to what the first stage is.

        No arguments clears it and the task sums as it did.
        """
        payload = struct.pack('>BH', len(sections), int(decimate))
        for section in sections:
            payload += struct.pack('>5i', *[int(round(c * COEFF_SCALE))
                                            for c in section])
        self.took(self._op(DAQ_OP_FILTER, payload))
        return True

    #: What the generator makes. SINE has a frequency, so the chain's
    #: answer to it is a gain and a phase. RAMP is `offset + (n * hz) mod
    #: amplitude`, an integer sequence a host computes in closed form -
    #: which is what lets every record be checked exactly rather than
    #: statistically.
    SINE, RAMP = 0, 1

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """A known sine in the converter's place, or `hz=0` for the
        converter again.

        For proving the path rather than measuring anything: a host that
        knows the frequency, the rate and the decimation knows what every
        output sample should be, so a record that fell out of the ring
        shows up as a phase that jumped rather than as nothing at all.
        """
        self.took(self._op(DAQ_OP_TONE,
                           struct.pack('>IIiiB', int(hz), int(rate_hz),
                                       int(amplitude), int(offset),
                                       int(kind))))
        return True

    def start(self):
        self.took(self._op(DAQ_OP_START))
        return True

    def stop(self):
        self._op(DAQ_OP_STOP)
        return True

    def acquire(self, want=0, layout=None):
        """Whole records, oldest first, decoded from the board's layout.

        Pass `layout` to save a round trip when draining in a loop.
        """
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        raw = self._op(DAQ_OP_READ, bytes([min(int(want), 255)]))
        got = raw[0]
        # THE BOARD'S STRIDE, not one worked out here. It says so in the
        # layout for exactly this reason, and a decoder that recomputes it
        # mis-frames every record after the first the day the record grows
        # a field - which is how the sample count arrived.
        stride = layout['stride']
        fmt = '>I%di%dBH' % (len(fields), len(pins))
        out = []
        for i in range(got):
            at = 1 + i * stride
            values = struct.unpack(fmt, raw[at:at + stride])
            rec = {'at': values[0], 'samples': values[-1]}
            rec.update({f['signal']: v for f, v in zip(fields, values[1:])})
            if pins:
                # A DUTY, not a level: the pin went through the same
                # window as everything else, and 255 is all of it. A
                # level sampled once and decimated by two thousand is
                # aliased by construction - KEEPALIVE toggles at
                # ~100 kHz and read as a coin toss.
                first = 1 + len(fields)
                rec['digital'] = {
                    p['signal']: values[first + n] / 255.0
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

    def once(self, channels, records, clock='software', sample_time=0,
                decimate=1, accumulate=1, timeout=10.0, digital=False,
                sample_rate=None):
        """Configure, start, drain until the task says done. The one-shot.

        Called `once` and not `acquire` because `acquire` is the loop: start
        once, then acquire what has arrived, over and over. This is the whole
        capture in one call, a different shape of thing - and not `take`
        either, which `Capture` already uses for draining N records.

        Raises rather than returning a short capture: a run that stopped
        early is a different measurement, not a smaller one.
        """
        layout = self.configure(channels, clock=clock, sample_time=sample_time,
                                decimate=decimate, accumulate=accumulate,
                                records=records, digital=digital,
                                sample_rate=sample_rate)
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
