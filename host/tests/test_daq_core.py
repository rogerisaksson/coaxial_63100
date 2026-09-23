"""The acquisition engine, run as the C that will run on the board."""
import ctypes
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
DAQ = os.path.join(REPO, 'daq')
FILTER = os.path.join(REPO, 'filter')
SOURCES = [os.path.join(DAQ, 'test', 'harness.c'),
           os.path.join(DAQ, 'src', 'daq.c'),
           os.path.join(FILTER, 'src', 'filter.c')]
INCLUDES = [os.path.join(DAQ, 'inc'), os.path.join(FILTER, 'inc')]

SINE, RAMP = 0, 1
CYCLES_PER_SECOND = 1_000_000       # a host clock the tests can do sums in
IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0]  # a biquad that passes its input through


class Engine:

    """The harness, typed once, with a record decoder beside it."""

    def __init__(self, lib):
        u8, u16, u32, i32 = (ctypes.c_uint8, ctypes.c_uint16,
                             ctypes.c_uint32, ctypes.c_int32)
        i32p, u32p, u8p, f32p, i16p = (ctypes.POINTER(i32), ctypes.POINTER(u32),
                                       ctypes.POINTER(u8), ctypes.POINTER(ctypes.c_float),
                                       ctypes.POINTER(ctypes.c_int16))
        lib.daq_h_init.argtypes = [u32]
        lib.daq_h_ladder.argtypes = [u32, u32, u32, u32, u32]
        lib.daq_h_begin.argtypes = [u8, u16, u16, u32, u8, u16, u32, ctypes.c_int]
        lib.daq_h_start.argtypes = [u32]
        lib.daq_h_words.argtypes = [i16p]
        lib.daq_h_feed.argtypes = [i32p, u32, u32]
        lib.daq_h_trigger_due.argtypes = [u32]
        lib.daq_h_sweep_begin.argtypes = [u32, u32]
        lib.daq_h_sweep_put.argtypes = [i32]
        lib.daq_h_sweep_close.argtypes = [u32]
        lib.daq_h_live_insert.argtypes = [u8, i32, u32, u32]
        lib.daq_h_take_live.argtypes = [i32p, u32p, i32p, i32p, u32p]
        for name in ('daq_h_set_filter',):
            getattr(lib, name).argtypes = [f32p, u8, u16]
            getattr(lib, name).restype = ctypes.c_char_p
        lib.daq_h_set_rung.argtypes = [u8, u16, f32p, u8, u16]
        lib.daq_h_set_rung.restype = ctypes.c_char_p
        lib.daq_h_set_tone.argtypes = [u32, u32, i32, i32, u8, u32, u32]
        lib.daq_h_set_tone.restype = ctypes.c_char_p
        lib.daq_h_tone_poll.argtypes = [u32, u32]
        lib.daq_h_take.argtypes = [u8p, u16]
        lib.daq_h_take.restype = u16
        for name in ('available', 'capacity', 'dropped', 'produced', 'worst',
                     'triggers', 'rung', 'rung_changes', 'accumulate', 'stride',
                     'triggers_per_record', 'max_channels', 'max_sensors',
                     'max_additions', 'record_max'):
            getattr(lib, 'daq_h_' + name).restype = u32
        self.lib = lib
        self.channels = lib.daq_h_max_channels()
        self.sensors = lib.daq_h_max_sensors()
        self.max_additions = lib.daq_h_max_additions()
        self.record_max = lib.daq_h_record_max()
        self.fields = 0
        self.pins = 0
        self.sensor_mask = 0

    # -- set-up --

    def fresh(self, bytes_=4096, fields=2, accumulate=1, decimate=1,
              interval=0, pins=0, sensors=0, records=0, adapt=0, start=True,
              now=0):
        self.lib.daq_h_init(bytes_)
        self.lib.daq_h_ladder(6, 1, 8, 512, 64)
        self.begin(fields, accumulate, decimate, interval, pins, sensors,
                   records, adapt)
        if start:
            assert self.lib.daq_h_start(now) == 1
        return self

    def begin(self, fields, accumulate=1, decimate=1, interval=0, pins=0,
              sensors=0, records=0, adapt=0):
        self.fields, self.pins, self.sensor_mask = fields, pins, sensors
        self.lib.daq_h_begin(fields, accumulate, decimate, interval, pins,
                             sensors, records, adapt)

    def words(self, table):
        flat = [w for row in table for w in row]
        flat += [0] * (self.sensors * 4 - len(flat))
        self.lib.daq_h_words((ctypes.c_int16 * len(flat))(*flat))

    # -- samples in --

    def feed(self, values, at=0, digital=0):
        values = list(values) + [0] * (self.channels - len(values))
        self.lib.daq_h_feed((ctypes.c_int32 * len(values))(*values), at, digital)

    def feed_records(self, n, value=100, at=0):
        for k in range(n):
            self.feed([value] * self.fields, at + k)

    # -- records out --

    def take(self, max_records=1000):
        stride = self.lib.daq_h_stride()
        buf = (ctypes.c_uint8 * (stride * max_records))()
        got = self.lib.daq_h_take(buf, max_records)
        raw = bytes(buf)[:got * stride]
        return [self.decode(raw[i * stride:(i + 1) * stride]) for i in range(got)]

    def decode(self, rec):
        """The record as a host reads it: start, sums, duties, words, count."""
        at = 4
        start = struct.unpack('>I', rec[:4])[0]
        sums = list(struct.unpack('>%di' % self.fields, rec[at:at + 4 * self.fields]))
        at += 4 * self.fields
        duties = list(rec[at:at + self.pins])
        at += self.pins
        words = {}
        for bit in range(self.sensors):
            if self.sensor_mask & (1 << bit):
                words[bit] = list(struct.unpack('>4h', rec[at:at + 8]))
                at += 8
        count = struct.unpack('>H', rec[at:at + 2])[0]
        assert at + 2 == len(rec), 'record of %d bytes decoded to %d' % (len(rec), at + 2)
        return {'start': start, 'sums': sums, 'duties': duties, 'words': words,
                'count': count}

    def live(self):
        n = self.channels
        sum_, add, lo, hi = ((ctypes.c_int32 * n)(), (ctypes.c_uint32 * n)(),
                             (ctypes.c_int32 * n)(), (ctypes.c_int32 * n)())
        span = (ctypes.c_uint32 * 3)()
        fresh = self.lib.daq_h_take_live(sum_, add, lo, hi, span)
        return {'fresh': bool(fresh), 'first': span[0], 'last': span[1],
                'digital': span[2],
                'slots': [(sum_[f], add[f], lo[f], hi[f]) for f in range(n)]}

    def coeffs(self, sections):
        flat = [c for s in sections for c in s]
        return (ctypes.c_float * max(1, len(flat)))(*flat)


def test_the_ring(report, e):
    """Records go in whole, come out whole, and the ring says how full."""
    e.fresh(bytes_=4096, fields=2)
    stride = e.lib.daq_h_stride()
    report.check('the stride is the record: start, a sum a field, the count',
                 stride == 4 + 4 * 2 + 2, str(stride))
    e.feed([100, -7], at=0x01020304)
    e.feed([200, 8], at=0x01020305)
    report.check('two feeds at accumulate 1 are two records',
                 e.lib.daq_h_available() == 2, str(e.lib.daq_h_available()))
    got = e.take()
    report.check('a record is its start time, big endian, then the sums',
                 got[0]['start'] == 0x01020304 and got[0]['sums'] == [100, -7]
                 and got[1]['sums'] == [200, 8], str(got))
    report.check('and the count, last, says one went in',
                 got[0]['count'] == 1, str(got[0]['count']))
    report.check('taken records are gone', e.lib.daq_h_available() == 0)

    capacity = e.lib.daq_h_capacity()
    report.check('the capacity is the ring at this stride',
                 capacity == 4096 // stride, str(capacity))
    # A record goes in while more than a stride is free, so the ring holds
    # every whole record it has room for and never lets the head catch the
    # tail: that many, and not one more.
    held = -(-(4096 - stride) // stride)
    e.feed_records(capacity + 5)
    report.check('past the ring the rest is dropped and counted, not wrapped',
                 e.lib.daq_h_dropped() == capacity + 5 - held
                 and e.lib.daq_h_available() == held,
                 'dropped %d, held %d' % (e.lib.daq_h_dropped(), e.lib.daq_h_available()))
    report.check('the high-water mark is the fullest it was',
                 e.lib.daq_h_worst() == held, str(e.lib.daq_h_worst()))
    report.check('the record path took the interrupt hold each time',
                 e.lib.daq_h_holds() >= capacity + 5, str(e.lib.daq_h_holds()))


def test_the_window(report, e):
    """Accumulate sums, decimate skips, and the count travels with the sum."""
    e.fresh(fields=1, accumulate=4)
    for k in range(3):
        e.feed([10], at=k)
    report.check('three of four samples is no record yet',
                 e.lib.daq_h_available() == 0)
    e.feed([12], at=3)
    got = e.take()
    report.check('the fourth closes it: the SUM, not a mean, and the count',
                 got[0]['sums'] == [42] and got[0]['count'] == 4, str(got))
    report.check('the record starts when its window did',
                 got[0]['start'] == 0, str(got[0]['start']))

    e.fresh(fields=1, accumulate=1, decimate=3)
    for k in range(9):
        e.feed([k], at=k)
    got = e.take()
    report.check('decimate 3 keeps one trigger in three, the first of each',
                 [r['sums'][0] for r in got] == [0, 3, 6], str([r['sums'] for r in got]))
    report.check('triggers count what was kept, not what was fed',
                 e.lib.daq_h_triggers() == 3, str(e.lib.daq_h_triggers()))


def test_closed_by_the_clock(report, e):
    """Accumulate 0: the window is whatever the loop managed, the interval
    closes it, and the count says how many went in."""
    e.fresh(fields=1, accumulate=0, interval=1000)
    for k in range(7):
        e.feed([5], at=k * 100)
    report.check('700 cycles into a 1000-cycle window is no record',
                 e.lib.daq_h_available() == 0)
    for k in range(7, 11):
        e.feed([5], at=k * 100)
    got = e.take()
    report.check('the sample at the interval closes it, and is in it',
                 len(got) == 1 and got[0]['count'] == 11 and got[0]['sums'] == [55],
                 str(got))

    e.fresh(fields=1, accumulate=0, interval=0xFFFFFFFF)
    for k in range(e.max_additions + 10):
        e.feed([65535], at=k)
    e.feed([65535], at=0xFFFFFFFF)
    got = e.take()
    report.check('a window past the bound saturates instead of wrapping: the '
                 'mean over what went in stays true',
                 got[0]['count'] == e.max_additions
                 and got[0]['sums'] == [e.max_additions * 65535],
                 'count %d, sum %d' % (got[0]['count'], got[0]['sums'][0]))


def test_pins_and_sensors(report, e):
    """A pin is a duty of the window; a sensor field is four words snapshot
    at the close, after the pins and before the count."""
    e.fresh(fields=1, accumulate=4, pins=2)
    for k in range(4):
        e.feed([1], at=k, digital=0b01 if k < 2 else 0b11)
    got = e.take()
    report.check('pin 0 high all window is 255, pin 1 half of it is 128',
                 got[0]['duties'] == [255, 128], str(got[0]['duties']))

    e.fresh(fields=1, accumulate=1, sensors=0b10011)
    e.words([[1, 2, 3, 4], [-5, 6, -7, 8], [0, 0, 0, 0], [0, 0, 0, 0],
             [4095, 9, 0x20, 1]])
    e.feed([7], at=1)
    got = e.take()
    report.check('the stride grew by eight bytes a sensor field',
                 e.lib.daq_h_stride() == 4 + 4 + 8 * 3 + 2, str(e.lib.daq_h_stride()))
    report.check('the words come back in field order, big endian, signed',
                 got[0]['words'] == {0: [1, 2, 3, 4], 1: [-5, 6, -7, 8],
                                     4: [4095, 9, 0x20, 1]},
                 str(got[0]['words']))
    report.check('and the count is still last', got[0]['count'] == 1)


def test_a_finite_task(report, e):
    """`records` stops the task, and says so."""
    e.fresh(fields=1, records=3)
    e.feed_records(5)
    report.check('three records and it stops running, done',
                 e.lib.daq_h_produced() == 3 and e.lib.daq_h_running() == 0
                 and e.lib.daq_h_done() == 1,
                 'produced %d' % e.lib.daq_h_produced())
    report.check('start refuses while running, and with nothing configured',
                 e.lib.daq_h_start(0) == 1 and e.lib.daq_h_start(0) == 0)
    e.lib.daq_h_init(4096)
    report.check('nothing configured cannot start', e.lib.daq_h_start(0) == 0)


def test_losing_power(report, e):
    """The reference gone: the task stops and every buffer empties."""
    e.fresh(fields=1)
    e.feed_records(3)
    e.lib.daq_h_live_insert(0, 5, 1, 0)
    e.lib.daq_h_lose_power()
    report.check('stopped, and the ring is empty',
                 e.lib.daq_h_running() == 0 and e.lib.daq_h_available() == 0)
    report.check('and the live accumulator with it',
                 not e.live()['fresh'])


def test_the_live_accumulator(report, e):
    """Every sample adds; a take empties; it saturates rather than wraps."""
    e.fresh(fields=2)
    for k, v in enumerate((10, -3, 25)):
        e.lib.daq_h_live_insert(0, v, 100 + k, 0b101)
    e.lib.daq_h_live_insert(1, 7, 103, 0b101)
    live = e.live()
    report.check('the slot is sum, count, lowest, highest',
                 live['slots'][0] == (32, 3, -3, 25) and live['slots'][1] == (7, 1, 7, 7),
                 str(live['slots'][:2]))
    report.check('with the span it came from and the pins at the last',
                 live['fresh'] and live['first'] == 100 and live['last'] == 103
                 and live['digital'] == 0b101, str(live))
    again = e.live()
    report.check('a take empties it: not fresh, nothing in the slots',
                 not again['fresh'] and again['slots'][0] == (0, 0, 0, 0))
    for k in range(e.max_additions + 5):
        e.lib.daq_h_live_insert(0, 65535, k, 0)
    live = e.live()
    report.check('the sum stops widening at the bound, the extremes still move',
                 live['slots'][0] == (e.max_additions * 65535, e.max_additions,
                                      65535, 65535), str(live['slots'][0]))
    report.check('the live path takes the interrupt hold',
                 e.lib.daq_h_holds() >= e.max_additions + 5)


def test_the_sweep_and_the_gate(report, e):
    """One field a turn, closed through the count's gate."""
    e.fresh(fields=3, accumulate=1, interval=100)
    # The gate counts from the counter's zero, so the first trigger is due once
    # an interval has passed since it - immediately, on a board whose counter
    # has been running.
    report.check('the interval gates: due, not 50 cycles on, due again at 100',
                 e.lib.daq_h_trigger_due(1000) == 1 and e.lib.daq_h_trigger_due(1050) == 0
                 and e.lib.daq_h_trigger_due(1100) == 1)
    e.lib.daq_h_sweep_begin(1000, 0b1)
    puts = [e.lib.daq_h_sweep_put(v) for v in (1, 2, 3)]
    report.check('the sweep is whole on its last field',
                 puts == [0, 0, 1], str(puts))
    e.lib.daq_h_sweep_close(1150)
    report.check('closed 50 cycles after the last trigger, the gate holds it',
                 e.lib.daq_h_available() == 0)
    e.lib.daq_h_sweep_begin(1000, 0b1)
    for v in (1, 2, 3):
        e.lib.daq_h_sweep_put(v)
    e.lib.daq_h_sweep_close(1200)
    got = e.take()
    report.check('closed at the interval, the sweep is the record',
                 len(got) == 1 and got[0]['sums'] == [1, 2, 3] and got[0]['start'] == 1000,
                 str(got))
    e.fresh(fields=1, accumulate=1, interval=0)
    report.check('no interval is every sweep',
                 all(e.lib.daq_h_trigger_due(t) for t in (0, 1, 2)))


def test_the_chain(report, e):
    """The chain shapes the mean and puts the sum back; its decimation
    swallows windows; the refusals say why."""
    e.fresh(fields=1, accumulate=2, start=False)
    said = e.lib.daq_h_set_filter(e.coeffs([IDENTITY]), 1, 2)
    report.check('an identity biquad at decimate 2 is taken', said is None, str(said))
    assert e.lib.daq_h_start(0) == 1
    for k in range(8):
        e.feed([100], at=k)
    got = e.take()
    report.check('four windows through decimate 2 are two records',
                 len(got) == 2, str(len(got)))
    report.check('each carrying the SUM the divisor expects, mean times count',
                 all(r['sums'] == [200] and r['count'] == 2 for r in got), str(got))
    report.check('triggers a record: accumulate 2, decimate 1, the chain\'s 2',
                 e.lib.daq_h_triggers_per_record() == 4,
                 str(e.lib.daq_h_triggers_per_record()))

    said = e.lib.daq_h_set_filter(e.coeffs([IDENTITY]), 1, 1)
    report.check('refused while running, in words',
                 said is not None and b'running' in said, str(said))
    e.lib.daq_h_stop()
    said = e.lib.daq_h_set_filter(e.coeffs([IDENTITY] * 5), 5, 1)
    report.check('five sections refused - four biquads',
                 said is not None and b'four biquads' in said, str(said))
    said = e.lib.daq_h_set_filter(e.coeffs([]), 0, 0)
    report.check('decimate 0 refused', said is not None and b'smallest is 1' in said)
    said = e.lib.daq_h_set_filter(e.coeffs([]), 0, 1)
    report.check('no sections at decimate 1 clears the chain',
                 said is None and e.lib.daq_h_filtering() == 0)


def test_the_ladder(report, e):
    """Rungs are built from the bottom; the task climbs when the ring
    backs up and comes down only after a while at the low mark."""
    e.fresh(bytes_=16 * 10 + 8, fields=1, accumulate=1, adapt=1, start=False)
    stride = e.lib.daq_h_stride()
    capacity = e.lib.daq_h_capacity()
    e.lib.daq_h_ladder(6, 1, 8, 512, 3)
    said = [e.lib.daq_h_set_rung(2, 1, e.coeffs([]), 0, 1),
            e.lib.daq_h_set_rung(4, 1, e.coeffs([]), 0, 1),
            e.lib.daq_h_set_rung(0, 1, e.coeffs([]), 0, 1),
            e.lib.daq_h_set_rung(1, 2, e.coeffs([]), 0, 1)]
    report.check('rung 2 before 1 is refused, rung 4 does not exist, 0 then 1 take',
                 said[0] is not None and b'bottom' in said[0]
                 and said[1] is not None and b'four rungs' in said[1]
                 and said[2] is None and said[3] is None, str(said))
    assert e.lib.daq_h_start(0) == 1
    climb = capacity * 6 // 8
    e.feed_records(climb)
    report.check('at six eighths of the ring the task takes rung 1: '
                 'its boxcar is the accumulate now',
                 e.lib.daq_h_rung() == 1 and e.lib.daq_h_accumulate() == 2
                 and e.lib.daq_h_rung_changes() == 1,
                 'rung %d, accumulate %d, capacity %d' % (
                     e.lib.daq_h_rung(), e.lib.daq_h_accumulate(), capacity))
    e.take()
    e.feed_records(2 * 2)          # two records at boxcar 2, ring near empty
    report.check('one look at an empty ring is not enough to come down',
                 e.lib.daq_h_rung() == 1)
    e.take()
    e.feed_records(2 * 2)
    report.check('the third record at the low mark brings it down to rung 0',
                 e.lib.daq_h_rung() == 0 and e.lib.daq_h_accumulate() == 1
                 and e.lib.daq_h_rung_changes() == 2,
                 'rung %d after %d changes' % (e.lib.daq_h_rung(), e.lib.daq_h_rung_changes()))
    report.check('the stride never changed: a rung is a design, not a shape',
                 e.lib.daq_h_stride() == stride)


def test_the_tone(report, e):
    """The generator owes exactly the samples the elapsed cycles bought."""
    e.fresh(fields=2, accumulate=1)
    said = e.lib.daq_h_set_tone(3, 1000, 10, 5, RAMP, CYCLES_PER_SECOND, 0)
    report.check('a ramp: step 3, modulo 10, offset 5, at 1 kHz', said is None, str(said))
    e.lib.daq_h_tone_poll(5000, 64)
    got = e.take()
    report.check('5 ms at 1 kHz is five samples, every field the same, integer '
                 'all the way', [r['sums'] for r in got] ==
                 [[5, 5], [8, 8], [11, 11], [14, 14], [7, 7]],
                 str([r['sums'] for r in got]))
    e.lib.daq_h_tone_poll(6500, 64)
    e.lib.daq_h_tone_poll(7000, 64)
    got = e.take()
    report.check('the half sample owed at 6.5 ms is carried, not dropped',
                 len(got) == 2, str(len(got)))
    e.lib.daq_h_tone_poll(1_007_000, 64)
    got = e.take()
    report.check('a long gap is bounded to the burst, and the rest is dropped '
                 'rather than owed', len(got) == 64, str(len(got)))
    e.lib.daq_h_tone_poll(1_008_000, 64)
    report.check('the debt did not carry: one more millisecond is one sample',
                 e.lib.daq_h_available() == 1, str(e.lib.daq_h_available()))

    e.fresh(fields=1)
    said = e.lib.daq_h_set_tone(100, 1000, 1000, 0, SINE, CYCLES_PER_SECOND, 0)
    e.lib.daq_h_tone_poll(10_000, 64)
    got = e.take()
    want = [int(round(1000 * math.sin(2 * math.pi * 100 * (n + 1) / 1000)))
            for n in range(10)]
    close = all(abs(r['sums'][0] - w) <= 1 for r, w in zip(got, want))
    report.check('a 100 Hz sine at 1 kHz: ten samples a period, to the LSB',
                 said is None and len(got) == 10 and close,
                 str([r['sums'][0] for r in got]))
    e.lib.daq_h_tone_poll(1_100_000, 4096)
    got = e.take()
    report.check('a thousand rotations on, the unit vector has not crept',
                 max(abs(r['sums'][0]) for r in got) <= 1001, str(max(abs(r['sums'][0]) for r in got)))

    refusals = [e.lib.daq_h_set_tone(600, 1000, 1, 0, SINE, CYCLES_PER_SECOND, 0),
                e.lib.daq_h_set_tone(1, 0, 1, 0, SINE, CYCLES_PER_SECOND, 0),
                e.lib.daq_h_set_tone(1, 1000, 1, 0, 2, CYCLES_PER_SECOND, 0),
                e.lib.daq_h_set_tone(1, 1000, 1, 0, RAMP, CYCLES_PER_SECOND, 0)]
    report.check('an aliasing tone, no rate, an unknown kind and a ramp of one '
                 'are refused in words',
                 all(r is not None for r in refusals)
                 and b'alias' in refusals[0] and b'sample rate' in refusals[1]
                 and b'kind' in refusals[2] and b'at least 2' in refusals[3],
                 str(refusals))
    said = e.lib.daq_h_set_tone(0, 0, 0, 0, SINE, CYCLES_PER_SECOND, 0)
    report.check('hz 0 turns it off', said is None and e.lib.daq_h_tone_on() == 0)


ROSTER = (test_the_ring, test_the_window, test_closed_by_the_clock,
          test_pins_and_sensors, test_a_finite_task, test_losing_power,
          test_the_live_accumulator, test_the_sweep_and_the_gate,
          test_the_chain, test_the_ladder, test_the_tone)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, INCLUDES, name='daqcore')
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('daq/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    engine = Engine(lib)
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, engine)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
