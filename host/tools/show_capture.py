#!/usr/bin/env python3
"""Buffered capture: the AFE, the digital pins and both SPI parts at once.

    python tools/show_capture.py
    python tools/show_capture.py --port COM4 --hz 10

Two buffers on the board feed this, and neither is polled per value. The
acquisition task (`0x6E` device 6) samples every analog channel and the
digital word into a ring; the event ring (device 5) takes what the angle
and IMU loops produce. This drains both and draws the newest of each,
with the rates the board actually managed.

Every name here comes off the wire. The analog fields and the digital bits
are named by the task's own layout, and the SPI rows by the ring's source
tags - a channel added to `Board/Src/board_adc.c` appears with nothing
here told.

Nothing judges a reading. Raw codes and the board's own units, and
invariant 10 applies to a capture exactly as it applies to a meter.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import scaling                                # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from coaxial import Coaxial63100                           # noqa: E402
from screen import TO_MENU, Keys, banner, paint, closing, say  # noqa: E402

ROTATION_VECTOR = 0x05


class Rate:
    """Records per second, over a sliding window of wall time."""

    def __init__(self, window=2.0):
        self.window = window
        self.marks = []

    def add(self, n):
        now = time.time()
        self.marks.append((now, n))
        self.marks = [m for m in self.marks if now - m[0] <= self.window]

    def per_second(self):
        if len(self.marks) < 2:
            return 0.0
        span = self.marks[-1][0] - self.marks[0][0]
        return sum(n for _, n in self.marks[1:]) / span if span > 0 else 0.0


def start(rig, args):
    """Arm both buffers, and say what the board accepted.

    Through `Coaxial63100`, which is the DAQ API's front door: it takes
    every channel the board reports rather than a list written here, and
    `read()` off the same object is what puts `samples` on a record.
    """
    board = rig.board
    layout = rig.configure(clock=args.clock, digital=True,
                           sample_time=args.sample_time,
                           decimate=args.decimate,
                               accumulate=args.accumulate,
                               rate_hz=args.rate)
    rig.start()
    say('ok', 'task', '%d channels + %d pins, stride %d, %s clock'
        % (len(layout['fields']), len(layout['pins']), layout['stride'],
           args.clock))

    # The IMU only produces when a report is enabled, so a capture that did
    # not ask for one would show a source that looks broken rather than idle.
    try:
        # No reset first. The poll loop brings the part up on its own, and
        # a reset immediately before a Set Feature is what stops the feature
        # taking: the write's wake handshake runs its own reset when the
        # acknowledge does not arrive, and right after a host reset it does
        # not, so the write lands on a part that has just restarted.
        # Measured 2026-08-27: reset then feature, 0 rotation vectors;
        # feature alone, 49.0 a second. See FINDINGS.
        with board.imu.configuring():
            board.imu.feature(ROTATION_VECTOR, args.interval_us)
        say('ok', 'rotation vector', 'every %d us' % args.interval_us)
    except RigError as exc:
        say('warn', 'rotation vector', '%s - the ring will show angle only'
            % exc)

    board.capture.arm(['angle', 'imu'])
    say('ok', 'ring', 'angle and imu')
    return layout


def analog_rows(layout, record, width, params=None):
    """One line per analog field, named and united by the board.

    Divided by `samples`, because a record's value is the SUM of that many
    readings and not one. Shown raw it doubled the instant `adapt` raised
    accumulation, and a number that moves when the buffering changes is not
    a measurement.

    The code and what it converts to, because the task buffers codes and
    does not scale them - the unit in a layout says what the channel means,
    not what the number is in. Printing the two together said 405
    centi-degC for an NTC that `ntc_temperature()` read as 38.1 C off the
    same code, and 20811 mV for a 24.81 V bus. `scaling.converter` is the
    same one the meter bridge uses.
    """
    out = []
    samples = max(1, (record or {}).get('samples', 1))
    for field in layout['fields']:
        value = record.get(field['signal']) if record else None
        if value is None:
            out.append(('  %-9s %12s' % (field['signal'], '-'))[:width])
            continue
        code = value // samples
        convert = scaling.converter(field['unit'], field['differential'],
                                    params=params)
        out.append('  %-9s %+7d  %+9.2f %s'
                   % (field['signal'], code, convert(code),
                      scaling.symbol(field['unit'],
                                     field['signal']))[:width])
    return out


def digital_rows(layout, record, width):
    """One line per digital bit, named by the board's own pin table."""
    bits = (record or {}).get('digital') or {}
    out = []
    for pin in layout['pins']:
        level = bits.get(pin['signal'])
        out.append('  %-12s %-4s %-5s'
                   % (pin['signal'], pin['direction'],
                      '-' if level is None else ('high' if level else 'low'))[:width])
    return out


def spi_rows(latest, rates, width):
    """The newest record from each SPI source, however the board tagged it."""
    shape = {
        'angle': lambda v: 'value %6d  crc %2d  reg %3d' % (v[0], v[1], v[2]),
        'imu': lambda v: 'i %6d  j %6d  k %6d  real %6d' % v,
    }
    out = []
    for source in ('angle', 'imu'):
        record = latest.get(source)
        body = shape[source](record['v']) if record else 'nothing yet'
        out.append('  %-7s %-46s %8.0f /s'
                   % (source, body, rates[source].per_second())[:width])
    return out


def compose(origin, console, layout, view, width):
    """The whole frame, as a list of lines."""
    daq, ring = view['daq'], view['ring']
    # A blank FIRST: paint addresses absolute rows from 1, so the
    # banner would otherwise land under the shell's own decoration,
    # taking the LIVE/SIMULATED tag with it.
    lines = ['', banner(origin, 'buffered capture', console,
                    'Q closes, ESC for the menu'), '']
    lines.append(' TASK  %s clock  dec %d%s  acc %d%s  st %d   %8.0f rec/s   '
                 'buffered %d   dropped %d'
                 % (daq['clock'], daq['decimate'], '',
                    daq['accumulate'],
                    '*' if daq['accumulate'] > view['asked'] else '',
                    daq['sample_time'],
                    view['daq_rate'].per_second(), daq['available'],
                    daq['dropped']))
    lines.append(' ' + '-' * max(10, width - 2))
    analog = analog_rows(layout, view['record'], width, view['scaling'])
    digital = digital_rows(layout, view['record'], width)
    for i in range(max(len(analog), len(digital))):
        left = analog[i] if i < len(analog) else ' ' * 36
        right = digital[i] if i < len(digital) else ''
        lines.append(('%-38s%s' % (left, right.strip() and right or ''))[:width])
    lines += ['', ' RING  %s   buffered %d of %d   dropped %d   thinned %d%s'
              % (', '.join(ring['sources']) or 'nothing armed',
                 ring['count'], ring['depth'], ring['dropped'],
                 ring.get('thinned', 0),
                 '   missed %d' % view['missed'] if view['missed'] else ''),
              ' ' + '-' * max(10, width - 2)]
    lines += spi_rows(view['latest'], view['rates'], width)
    return lines


def drain(rig, layout, view):
    """Take what both buffers hold, and keep the newest of each kind.

    A missed reply is counted, not raised. Measured on this VCP, about one
    transaction in fifty goes unanswered when the board is busy, and a live
    view that dies on one is a view that cannot be left running - which is
    exactly how this one used to end, with a traceback over the frame.
    """
    board = rig.board
    try:
        return _drain(rig, board, layout, view)
    except RigError:
        view['missed'] += 1
        return None


def _drain(rig, board, layout, view):
    batch = rig.acquire()
    view['daq_rate'].add(len(batch))
    if batch:
        view['record'] = batch[-1]

    for record in board.capture.drain(limit=90):
        view['latest'][record['source']] = record
        view['rates'][record['source']].add(1)
    for source in ('angle', 'imu'):
        view['rates'][source].add(0)

    view['daq'] = board.daq.state()
    view['ring'] = board.capture.state()


def adapt(rig, layout, args, view):
    """Accumulate harder when the link cannot carry what the board produces.

    `dropped` is the board's own counter, so this closes the loop on the
    thing that actually overflows rather than on a guess about the wire.
    Accumulation and not decimation: summing keeps every sample's
    contribution where subsampling throws it away, and it is what measured
    clean - seven channels and the digital word drop 3851 records at
    accumulate 1 and none at all at 16.

    It happens on the target, before a byte is sent, which is the only place
    it saves anything: the payload ceiling here is about 3.8 kB/s whatever
    the record size, so the way through is fewer records, not bigger reads.

    Reconfiguring empties the ring, so this backs off in doublings rather
    than nudging: re-arming every frame would show a buffer that never
    filled and a rate that never settled.
    """
    dropped = view['daq']['dropped']
    gained = dropped > view['dropped_was']
    view['dropped_was'] = dropped
    view['quiet'] = 0 if gained else view['quiet'] + 1

    want = view['accumulate']
    if gained and want < 1024:
        want = want * 2
    elif view['quiet'] >= 20 and want > args.accumulate:
        want = want // 2

    if want == view['accumulate']:
        return layout

    view['accumulate'] = want
    try:
        rig.stop()
        fresh = rig.configure(clock=args.clock, digital=True,
                              sample_time=args.sample_time,
                              decimate=args.decimate, accumulate=want,
                                  rate_hz=args.rate)
        rig.start()
    except RigError:
        view['missed'] += 1
        return layout
    view['quiet'] = 0
    return fresh


def put_back(board):
    """Both buffers disarmed, and the supply as it was found."""
    done = []
    for name, what, undo in (
            ('acquisition', 'task stopped', board.daq.stop),
            ('capture ring', 'disarmed', board.capture.stop)):
        try:
            undo()
            done.append((name, what))
        except RigError as exc:
            done.append((name, 'FAILED: %s' % exc))
    try:
        with board.imu.configuring():
            board.imu.feature(ROTATION_VECTOR, 0)
        done.append(('rotation vector', 'disabled'))
    except RigError as exc:
        done.append(('rotation vector', 'FAILED: %s' % exc))
    return done


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--clock', default='software', choices=('software',))
    parser.add_argument('--sample-time', type=int, default=0)
    parser.add_argument('--decimate', type=int, default=1)
    parser.add_argument('--accumulate', type=int, default=1,
                        help='samples summed per record. Raised on its own '
                             'when the board reports drops')
    parser.add_argument('--interval-us', type=int, default=20000,
                        help='what to ask the IMU to report at')
    parser.add_argument('--rate', type=float, default=2000.0,
                        help='software clock, in hertz. 0 is unlimited')
    parser.add_argument('--simulated', action='store_true')
    parser.add_argument('--frames', type=int, default=0)
    args = parser.parse_args(argv)

    rig = Coaxial63100(port=args.port,
                       simulated_device=bool(args.simulated)).open()
    origin, board = rig.origin, rig.board
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    try:
        layout = start(rig, args)
    except RigError as exc:
        say('fail', 'task', str(exc))
        rig.close()
        return 1

    view = {'record': None, 'latest': {}, 'daq': board.daq.state(),
            'ring': board.capture.state(), 'daq_rate': Rate(),
            'rates': {'angle': Rate(), 'imu': Rate()},
            'accumulate': args.accumulate, 'asked': args.accumulate,
            'dropped_was': 0, 'quiet': 0, 'missed': 0,
            'scaling': board.analog.scaling()}

    console = sys.stdout.isatty()
    if console and os.name == 'nt':
        os.system('')
    shown, leaving, frame = [], None, 0

    try:
        with Keys(console) as keys:
            while True:
                width = shutil_width()
                drain(rig, layout, view)
                layout = adapt(rig, layout, args, view)
                lines = compose(origin, console, layout, view, width)
                sys.stdout.write(paint(shown, lines, console))
                sys.stdout.flush()
                shown = lines
                frame += 1
                if args.frames and frame >= args.frames:
                    break
                leaving, _ = keys.poll()
                if leaving:
                    break
                time.sleep(1.0 / max(args.hz, 0.5))
    except KeyboardInterrupt:
        pass
    finally:
        done = put_back(board)
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        closing(done, console, len(shown))

    return TO_MENU if leaving == 'menu' else 0


def shutil_width():
    """The window, or a usable default when this is not a terminal."""
    try:
        return max(60, os.get_terminal_size().columns - 2)
    except OSError:
        return 100


if __name__ == '__main__':
    sys.exit(main())
