"""One acquisition task: configure, start, read.

DAQmx's shape, cut down to what this board has. There is one task, not many
- one MCU, three converters, one timer - so `configure` replaces whatever
was there rather than adding to it.

Nothing here knows the record's shape. `layout()` asks the board what each
field is and `read()` decodes from that, so a channel added to
`board/src/board_adc.c` shows up in a capture without this file being told.
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
DAQ_OP_RUNG = 9

#: Coefficients cross as Q28 - the wire carries no floating point, and a
#: biquad's a1 reaches -2, so 2^28 leaves a range of +/-8 and a
#: resolution three orders inside what a float holds anyway.
COEFF_SCALE = 1 << 28

#: BOARD_UNIT_* as comms/inc/board.h numbers them. Getting this wrong
#: labelled the NTC 'mV' and the DC bus 'mA' without changing a value.
UNITS = {0: None, 1: 'mV', 2: 'centi-degC', 3: 'mA'}


class Daq(Subsystem, Acquisition):

    """Configure, trigger, read - against the board's own channel table."""

    #: Records still in the board's ring after the last `acquire()`,
    #: straight off that same reply. None before the first read, and on
    #: a board older than protocol MINOR 5.
    backlog = None

    def _op(self, op, payload=b'', **kwargs):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_DAQ, op]) + bytes(payload),
                            **kwargs)

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
        if r.remaining >= 6:
            state['rung'] = r.u8()
            state['rungs'] = r.u8()
            state['rung_changes'] = r.u32()
        else:
            state['rung'] = state['rungs'] = 0
            state['rung_changes'] = 0
        # SWEEPS, not records: what the loop manages underneath the
        # decimation. Differentiate it and you have the rate the
        # chain was designed against, live.
        state['triggers'] = r.u32() if r.remaining >= 4 else None
        # Appended, MINOR 7: which sensor fields this build can put in a
        # record, and which the task carries now. None on older boards -
        # `catalogue()` marks the rows unselectable off exactly this.
        if r.remaining >= 4:
            state['sensors'] = r.u16()
            state['sensors_available'] = r.u16()
        else:
            state['sensors'] = state['sensors_available'] = None
        state['sensors_supported'] = state['sensors_available'] is not None
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
                  sample_rate=None, interval_us=None, adapt=False,
                  sensors=0):
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
        payload = struct.pack('>HBBHHIBIB', self._resolve(channels),
                              CLOCKS.get(clock, clock), sample_time,
                              decimate, accumulate, records,
                              1 if digital else 0, int(interval_us),
                              1 if adapt else 0)
        if sensors:
            # Appended, MINOR 7 - SNAPSHOT fields, software clock only.
            # Sent only when asked for: an older board ignores unread
            # tail bytes, and a silently dropped request is exactly what
            # the front door's `selectable` gate exists to refuse first.
            payload += struct.pack('>H', int(sensors))
        self._ack(DAQ_OP_CONFIGURE, payload)
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
        self._ack(DAQ_OP_FILTER, payload)
        return True

    #: What the generator makes. SINE has a frequency, so the chain's
    #: answer to it is a gain and a phase. RAMP is `offset + (n * hz) mod
    #: amplitude`, an integer sequence a host computes in closed form -
    #: which is what lets every record be checked exactly rather than
    #: statistically.
    SINE, RAMP = 0, 1

    def ladder(self, chains):
        """Load the whole ladder, bottom rung first.

        The board climbs it when its ring fills and comes back down when
        the link has caught up, so what a slow link costs is bandwidth
        rather than records. Rung 0 forgets every rung above it, which is
        what stops a rebuilt ladder leaving a stale one behind - so this
        sends 0 first, always.

        Each rung is a whole design: `configure(adapt=True)` and the
        board does the rest. `state()['rung']` says which one is running
        and `record['samples']` says it again, per record.
        """
        for n, chain in enumerate(chains):
            payload = struct.pack('>BHBH', n, int(chain['boxcar']),
                                  len(chain['sections']),
                                  int(chain['decimate']))
            for section in chain['sections']:
                payload += struct.pack('>5i',
                                       *[int(round(c * COEFF_SCALE))
                                         for c in section])
            self._ack(DAQ_OP_RUNG, payload)
        return True

    def tone(self, hz=0, rate_hz=0, amplitude=10000, offset=32768, kind=0):
        """A known sine in the converter's place, or `hz=0` for the
        converter again.

        For proving the path rather than measuring anything: a host that
        knows the frequency, the rate and the decimation knows what every
        output sample should be, so a record that fell out of the ring
        shows up as a phase that jumped rather than as nothing at all.
        """
        self._ack(DAQ_OP_TONE,
                           struct.pack('>IIiiB', int(hz), int(rate_hz),
                                       int(amplitude), int(offset),
                                       int(kind)))
        return True

    def start(self):
        self._ack(DAQ_OP_START)
        return True

    def stop(self):
        self._op(DAQ_OP_STOP)
        return True

    def decode(self, blob, layout=None):
        """Whole records out of raw record bytes.

        ONE DECODER FOR THE WIRE. The bytes arrive two ways now - straight
        off a read, or out of the broker's shared ring, which holds them
        exactly as they came - and two decoders for one format is how a
        stride drifts. `blob` is whole records and nothing else: no count
        byte in front, no backlog behind.
        """
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        sensors = layout.get('sensors') or []
        words = sum(x['words'] for x in sensors)
        stride = layout['stride']
        fmt = '>I%di%dB%dhH' % (len(fields), len(pins), words)
        out = []
        for i in range(len(blob) // stride):
            at = i * stride
            values = struct.unpack(fmt, blob[at:at + stride])
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
            if sensors:
                # SNAPSHOTS, not sums: raw and source-defined, the way
                # device 5 carries them - the scale stays this host's.
                first = 1 + len(fields) + len(pins)
                rec['sensors'] = {}
                for x in sensors:
                    rec['sensors'][x['signal']] = tuple(
                        values[first:first + x['words']])
                    first += x['words']
            out.append(rec)

        return out

    def acquire(self, want=0, layout=None):
        """Whole records, oldest first, decoded from the board's layout.

        Pass `layout` to save a round trip when draining in a loop.
        """
        layout = layout or self.layout()
        fields, pins = layout['fields'], layout.get('pins') or []
        # THE BOARD'S STRIDE, not one worked out here. It says so in the
        # layout for exactly this reason, and a decoder that recomputes it
        # mis-frames every record after the first the day the record grows
        # a field - which is how the sample count arrived.
        stride = layout['stride']
        # AND ITS REPLY'S LENGTH IS KNOWABLE, so say so: the first
        # payload byte is the record count and the stride is already in
        # hand, which turns the 8 ms of silence that ends every other
        # transaction into nothing. `tail` is the backlog MINOR 5
        # appends.
        raw = self._op(DAQ_OP_READ, bytes([min(int(want), 255)]),
                       reply_shape={'at': 0, 'head': 1, 'stride': stride,
                                    'tail': 4})
        got = raw[0]
        out = self.decode(raw[1:1 + (got * stride)], layout)

        # THE BACKLOG THE READ ITSELF ANSWERED, the way a DAQ card does
        # it: records still in the board's ring the instant this read
        # took its own. A separate state() costs a round trip and
        # answers about a different moment, which is the wrong number
        # to pace a reader with.
        #
        # Read only if it is there - appended by MINOR 5, and a board
        # older than that answers a reply that stops after the records.
        end = 1 + (got * stride)
        self.backlog = (struct.unpack('>I', raw[end:end + 4])[0]
                        if len(raw) >= end + 4 else None)
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
