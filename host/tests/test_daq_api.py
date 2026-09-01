#!/usr/bin/env python3
"""The acquisition front door: picking channels, reading them, shaping them.

No board and no port - every test here runs against the stand-in, which is
what makes them cheap enough to be run on every change. That is also their
limit, and it is worth saying out loud: they hold the HOST's arithmetic and
its lifecycles to account, not the wire.

WHY THIS FILE EXISTS. The surface it covers was written without it, and a
review found `read()` spinning forever when asked for more records than a
bounded run makes - three loops where one belongs, in the most-used call in
the library, sitting through several commits because nothing exercised it.
Every check below is either that defect or a neighbour of it.

Run from the host directory:  python tests/test_daq_api.py
"""
import contextlib
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                            # noqa: E402
from coaxial.errors import RigError                         # noqa: E402
from coaxial.fanout import Fanout                           # noqa: E402


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


@contextlib.contextmanager
def opened(**kw):
    """A stand-in session with the front end up, closed on the way out.

    A context manager and not a function returning an open device: `with
    device` calls `open()` again, which takes a second run at the preflight
    and drops the rail reference `enable()` had taken - the first version of
    this helper did exactly that and every tare below was refused.
    """
    device = Coaxial63100(simulated_device=True, **kw).open()
    try:
        device.set_time_from_pc()
        device.daq.enable()
        yield device
    finally:
        device.close()


# -- naming ------------------------------------------------------------------

def test_catalogue(report):
    with opened() as device:
        rows = device.daq.catalogue()
        kinds = {r['kind'] for r in rows}
        names = [r['name'] for r in rows]

    report.check('every row says what kind it is',
                 kinds == {'analog', 'digital', 'sensor'}, sorted(kinds))
    report.check('every row says whether configure() takes it',
                 all('selectable' in r for r in rows))
    report.check('the analog channels are the board\'s own',
                 'Phase U' in names and 'NTC' in names)
    # The sensor fields are listed AND refused: a name that is simply
    # missing tells a caller nothing about why.
    sensors = [r for r in rows if r['kind'] == 'sensor']
    report.check('the sensor fields are listed', len(sensors) == 5,
                 [r['name'] for r in sensors])
    report.check('and marked not selectable, since no record carries them',
                 not any(r['selectable'] for r in sensors))


def test_pick(report):
    with opened() as device:
        daq = device.daq
        loose = daq.pick('phaseU', 'ntc', 'DC_bus')
        report.check('case and punctuation do not count',
                     loose == ['Phase U', 'NTC', 'DC bus'], loose)

        order = daq.pick('ntc', 'phaseU')
        report.check('and the answer is in the BOARD\'s order, not the ask\'s',
                     order == ['Phase U', 'NTC'], order)

        try:
            daq.pick('currentU')
            report.check('an unknown name raises', False)
        except RigError as exc:
            report.check('an unknown name raises', True)
            report.check('naming what the board does have',
                         'Phase U' in str(exc), str(exc)[:60])

        try:
            daq.pick('orientation')
            report.check('a listed-but-unselectable name is refused', False)
        except RigError as exc:
            report.check('a listed-but-unselectable name is refused',
                         'record' in str(exc), str(exc)[:60])


def test_configure_takes_names_or_a_list(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', 'ntc')
        report.check('names as arguments',
                     daq.channel_names() == ['Phase U', 'NTC'],
                     daq.channel_names())

        daq.configure(daq.channels()[:3])
        report.check('a list, sliced, works the same',
                     len(daq.channel_names()) == 3, daq.channel_names())

        # A pin is a GROUP: naming one turns them all on and none of them
        # goes in the channel mask.
        daq.configure('phaseU', 'AFE_ON')
        report.check('a pin does not become an analog field',
                     daq.channel_names() == ['Phase U'], daq.channel_names())
        report.check('but the pins ride the record',
                     bool((device.layout or {}).get('pins')))

        try:
            daq.configure('AFE_ON')
            report.check('pins alone do not make a record', False)
        except RigError as exc:
            report.check('pins alone do not make a record',
                         'analog' in str(exc), str(exc)[:60])


# -- reading -----------------------------------------------------------------

def _read_within(daq, count, budget=10.0):
    """`daq.read(count)` on a thread, so a hang is a failure and not one."""
    got = []
    worker = threading.Thread(target=lambda: got.append(daq.read(count)),
                              daemon=True)
    began = time.time()
    worker.start()
    worker.join(budget)
    return (got[0] if got else None), time.time() - began


def test_read_of_a_finite_run(report):
    """THE REGRESSION. `read(50)` of a 5-record run spun forever, and the
    first two fixes for it threw all five away instead."""
    for ask, expect in ((50, 5), (5, 5), (3, 3), (-1, 5)):
        with opened() as device:
            daq = device.daq
            daq.configure('phaseU', records=5, sample_rate=500)
            daq.start()
            got, spent = _read_within(daq, ask)
        report.check('read(%d) of a 5-record run returns %d' % (ask, expect),
                     got is not None and len(got) == expect,
                     'hung' if got is None else '%d in %.2f s'
                     % (len(got), spent))


def test_read_of_a_running_task(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', 'ntc', sample_rate=1000)
        with daq:
            some, _ = _read_within(daq, 7)
            lot, _ = _read_within(daq, -1)
        report.check('read(n) gives exactly n', len(some or []) == 7,
                     len(some or []))
        report.check('read(-1) gives what there is, and waits for one',
                     bool(lot), len(lot or []))
        report.check('a record carries both channels',
                     lot and set(lot[0].channel_name) == {'Phase U', 'NTC'})


def test_capture_is_a_single_shot(report):
    """A burst at the loop's rate, ended by the record count.

    WHAT THE STAND-IN CANNOT SHOW, and it is the point of the feature: it
    invents a record when one is asked for, so its buffer never fills
    faster than the link empties it. The property that a capture samples
    ahead of the link for as long as the ring lasts needs the board. What
    is checked here is everything else - that the count is exact, that
    nothing is dropped, that no chain is left shaping it, and that the
    records come back whole.
    """
    with opened() as device:
        daq = device.daq
        for ask in (400, 1500):
            burst = daq.capture('phaseU', 'phaseV', records=ask)
            state = daq.state()
            report.check('capture(records=%d) returns exactly that' % ask,
                         len(burst) == ask, len(burst))
            report.check('and drops nothing, since the run ends at the ring',
                         not state.get('dropped'), state.get('dropped'))

        report.check('the records carry every channel asked for',
                     set(burst[0].channel_name) == {'Phase U', 'Phase V'},
                     burst[0].channel_name)
        report.check('nothing gates it: one sample a record',
                     burst[0].count == 1, burst[0].count)
        report.check('and the task is stopped afterwards',
                     not daq.state()['running'])

        # A capture with no count fills the ring, which is what the board
        # says it holds at this stride rather than a number chosen here.
        daq.configure('phaseU', accumulate=1, interval_us=0)
        capacity = daq.state()['capacity']
        report.check('the ring size comes from the board',
                     capacity > 1000, capacity)


def test_the_task_brackets_itself(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', sample_rate=500)
        with daq:
            pass
        report.check('`with daq` stops the task on the way out',
                     not daq.state()['running'])

        try:
            with daq:
                raise ValueError('the block failed')
        except ValueError:
            pass
        report.check('and stops it when the block raises',
                     not daq.state()['running'])


# -- the record --------------------------------------------------------------

def test_record_shape(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', 'ntc', sample_rate=1000)
        with daq:
            values = daq.read(20)
        first = values[0]

    report.check('start_time is a wall clock', first.start_time > 1.7e9,
                 first.start_time)
    report.check('dt is measured and positive', first.dt and first.dt > 0,
                 first.dt)
    report.check('channel_name is parallel to samples',
                 list(first.channel_name) == [s.name for s in first.samples])
    report.check('value() is the sum over the count',
                 abs(first.value('NTC')
                     - first['NTC'] / first.count) < 1e-9)
    report.check('sample() reaches one channel',
                 first.sample('NTC').unit == 'centi-degC')
    # The mapping underneath is untouched, which is what stops every
    # caller written against it from moving.
    report.check("record['samples'] is still the COUNT",
                 first['samples'] == first.count, first['samples'])
    try:
        first.value('Phase Q')
        report.check('an unknown channel raises with the record\'s list',
                     False)
    except KeyError as exc:
        report.check('an unknown channel raises with the record\'s list',
                     'Phase U' in str(exc), str(exc)[:50])


def test_series_and_columns(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', 'ntc', 'AFE_ON', sample_rate=1000)
        with daq:
            values = daq.read(20)
        ntc = daq.series(values, 'ntc')
        stamps = daq.series(values, 'time')
        gaps = daq.series(values, 'dt')
        pin = daq.series(values, 'AFE_ON')
        cols = daq.columns(values)

    report.check('series() takes the name as loosely as configure()',
                 len(ntc) == len(values), len(ntc))
    report.check('time and dt are spellings too',
                 len(stamps) == len(values) and len(gaps) == len(values))
    report.check('a pin is a series as well as a column',
                 len(pin) == len(values) and all(0.0 <= v <= 1.0 for v in pin))
    report.check('columns() carries the channels, the pins, time and dt',
                 {'Phase U', 'NTC', 'AFE_ON', 'time', 'dt'} <= set(cols),
                 sorted(cols)[:6])
    report.check('and every column is the same length',
                 len({len(v) for v in cols.values()}) == 1)


# -- buffers -----------------------------------------------------------------

def test_configure_buffer(report):
    with opened() as device:
        daq = device.daq
        report.check('configure_buffer answers what it took',
                     daq.configure_buffer(10000) == 10000)
        daq.configure('phaseU', sample_rate=500)
        with daq:
            daq.read(5)
            held = daq.buffered
    report.check('buffered() reports both ends and the losses',
                 {'host', 'peak', 'dropped', 'backlog', 'lost'} <= set(held),
                 sorted(held))


def test_fanout_ring(report):
    """The broker's shared ring, without a broker: cursors and losses."""
    ring = Fanout(stride=4, capacity=8)
    ring.put(bytes(range(4 * 6)))                    # six records

    first = ring.take(0)
    second = ring.take(0)
    report.check('two readers see the same records',
                 first[0] == second[0] and len(first[0]) // 4 == 6)
    report.check('and neither takes from the other',
                 first[3] == second[3] == 6)

    ring.put(bytes(range(4 * 6)))                    # twelve written, eight fit
    blob, start, lost, nxt = ring.take(0)
    report.check('a lapped cursor is told what it lost', lost == 4, lost)
    report.check('and where what it did get begins', start == 4, start)
    report.check('a caught-up cursor gets nothing and loses nothing',
                 ring.take(nxt)[0] == b'' and ring.take(nxt)[2] == 0)
    state = ring.state()
    report.check('the ring says how full it is',
                 state['held'] == 8 and state['overwritten'] == 4, state)


# -- the front end and the record it is scaled by ----------------------------

def test_enable_is_session_scoped(report):
    device = Coaxial63100(simulated_device=True).open()
    board = device.board
    report.check('the rail is down before anyone asks', not board.afe.is_on())
    device.daq.enable()
    device.daq.enable()
    report.check('enable() powers it, twice is not two references',
                 board.afe.is_on())
    device.close()
    report.check('and close() gives back exactly what was taken',
                 not board.afe.is_on())


def test_compensate_and_tare(report):
    with opened() as device:
        cal = device.board.calibration
        got = cal.compensate('phaseU', gain=1.002, offset=-7155, save=False)
        report.check('gain is a multiplier here and ppm on the wire',
                     got == {'offset_raw': -7155, 'gain_ppm': 2000}, got)

        kept = cal.compensate('phaseU', gain=1.004, save=False)
        report.check('writing one leaves the other alone',
                     kept['offset_raw'] == -7155, kept)

        cal.tare('phaseU', auto=True, save=False)
        after = {c['index']: c for c in cal.read()['channels']}[0]
        report.check('tare() writes a measured offset',
                     after['offset_raw'] != -7155, after)
        report.check('and leaves the gain it did not measure',
                     after['gain_ppm'] == 4000, after)

        device.board.afe.disable()
        try:
            cal.tare(save=False)
            report.check('a tare with the rail down is refused', False)
        except RigError as exc:
            report.check('a tare with the rail down is refused',
                         'AFE_ON' in str(exc), str(exc)[:50])


def test_scaled_columns_use_the_record(report):
    with opened() as device:
        daq = device.daq
        cal = device.board.calibration
        daq.configure('phaseU', sample_rate=1000)
        with daq:
            values = daq.read(20)
        try:
            plain = daq.frame(values, scaled=True)
        except RigError as exc:
            report.check('frame() says what it needs when pandas is absent',
                         'pandas' in str(exc), str(exc)[:50])
            return

        cal.compensate('phaseU', offset=1000, save=False)
        shifted = daq.frame(values, scaled=True)

    report.check('a scaled column moves when the offset does',
                 plain['Phase U (A)'].iloc[0] != shifted['Phase U (A)'].iloc[0],
                 '%.3f -> %.3f' % (plain['Phase U (A)'].iloc[0],
                                   shifted['Phase U (A)'].iloc[0]))
    report.check('the codes stay under their own name',
                 plain['Phase U'].iloc[0] == shifted['Phase U'].iloc[0])
    report.check('and the index is time',
                 plain.index.name == 'time', plain.index.name)


def test_frames_rolls_a_window(report):
    with opened() as device:
        daq = device.daq
        daq.configure('phaseU', sample_rate=1000)
        widths = []
        with daq:
            try:
                for frame in daq.frames(window=0.2, buffer=1.0,
                                        seconds=1.2, scaled=False):
                    widths.append(frame.index[-1] - frame.index[0])
            except RigError as exc:
                report.check('frames() says what it needs without pandas',
                             'pandas' in str(exc), str(exc)[:50])
                return
            deep = daq.history()

    report.check('frames() yields more than once', len(widths) > 2, len(widths))
    report.check('the window stops growing at what was asked',
                 max(widths) <= 0.21, '%.3f s' % max(widths))
    report.check('the buffer behind it is deeper',
                 (deep.index[-1] - deep.index[0]) > max(widths),
                 '%.3f s' % (deep.index[-1] - deep.index[0]))
    report.check('and the newest sample sits at zero',
                 abs(deep.index[-1]) < 1e-6, deep.index[-1])


def main():
    report = Report()
    for test in (test_catalogue, test_pick,
                 test_configure_takes_names_or_a_list,
                 test_read_of_a_finite_run, test_read_of_a_running_task,
                 test_capture_is_a_single_shot,
                 test_the_task_brackets_itself,
                 test_record_shape, test_series_and_columns,
                 test_configure_buffer, test_fanout_ring,
                 test_enable_is_session_scoped, test_compensate_and_tare,
                 test_scaled_columns_use_the_record,
                 test_frames_rolls_a_window):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
