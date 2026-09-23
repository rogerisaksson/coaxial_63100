#!/usr/bin/env python3
"""Every analog channel as a meter bridge, redrawn until you close it.

    python tools/show_desk.py
    python tools/show_desk.py --port COM4 --hz 10 --samples 32

The drawing itself is `coaxial.desk`, which is pure and tested; this file is
the loop, the screen and the cable. Nothing here judges a reading - the face
is the converter's own scale and invariant 10 applies to a meter exactly as
it applies to a table.

The AFE has to be on for any of it to mean anything (invariant 9), and it is
put back the way it was found on the way out.
"""
import argparse
import collections
import math
import os
import re
import sys
import time
from contextlib import suppress

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import bessel, desk, scaling                  # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from screen import Feed, TO_MENU, closing, say            # noqa: E402

import screen as _screen                                   # noqa: E402
from screen import hud                                     # noqa: E402
from screen import gauge, hud                              # noqa: E402
from screen import open_rig                                # noqa: E402
from screen import frame_of, run_view, stage               # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll


#: The codes a scale is measured between. Not 0 and 65535: the thermistor
#: conversion diverges at both rails - infinite resistance one end, zero the
#: other - so its scale is quoted between codes the converter can actually
#: resolve rather than at two asymptotes.
SPAN_CODES = (1024, 64512)


def drain(rig, cap=32):
    """Every record the ring is holding, up to `cap`."""
    got = []
    while len(got) < cap:
        block = rig.acquire()
        if not block:
            break
        got.extend(block)
    return got


def rows_from(records, layout):
    """A drained block as the rows this renderer already understands."""
    out = []
    for field in layout['fields']:
        name = field['signal']
        total = sum(r[name] for r in records)
        count = sum(max(1, r['samples']) for r in records)
        means = [r[name] / max(1, r['samples']) for r in records]
        out.append({
            'index': field['channel'],
            'signal': name,
            'unit': field['unit'],
            'differential': field['differential'],
            'mean_raw': total / max(1, count),
            'min_raw': min(means),
            'max_raw': max(means),
            'samples': count,
        })
    return out


def duties_from(records):
    """Each pin's duty, meaned over the block."""
    seen = [r['digital'] for r in records if r.get('digital')]
    if not seen:
        return {}
    return {name: sum(d[name] for d in seen) / len(seen)
            for name in seen[0]}


#: Records the host holds between the reader and the frame that draws
#: them. Its own ring, and it can overflow the same way the board's
#: does - which is the point of showing both.
HOST_RING = 2048


#: How much of what the board says it can carry a task actually asks
#: for. A ring produced at exactly the drain rate overflows on the first
#: slow read; `bessel.for_link` uses the same 0.8 for the same reason.
#: What a task asks for against what the board says it carries. OVER
#: ONE on purpose: the board should stay a little ahead of the link
#: so every read finds a full reply and the ring absorbs the rest.
#: Measured at ten channels and stride 55, ring flat and nothing
#: dropped: 0.8 held 39% of the line, 1.2 held 56%, 1.45 held 65% and
#: 2.5 holds 73%. Higher asks buy throughput by taking DECIMATION out
#: - ratio 7, then 4, then 2 - and two is the floor: at one there is
#: nothing left for the chain to shape. The passband follows the rate
#: that actually comes out (see `bessel.design`), so this cannot buy
#: speed by putting the cutoff above Nyquist, which is what it did
#: before that was fixed.
LINK_SHARE = 2.5

#: Sweeps a record below which the anti-alias chain would cost more
#: link than it is worth - the loop spends N sweeps for one record
#: and those sweeps come off the link, because sampling and the
#: Modbus handler share main(). Not reached since the board started
#: sampling while the UART drains, which took the loop from 380 to
#: 1880 sweeps/s and left plenty to decimate.
MIN_OVERSAMPLE = 4.0

#: How far ahead of the link the board is asked to run when nothing
#: is gating it. Measured: 2x gave 53% of the line, 4x 63%, 6x 68%,
#: 10x nothing more - the transaction floor is what is left.
RUN_AHEAD = 6.0


def sweep_rate(rig, records=300, timeout=6.0):
    """What the acquisition loop manages, in sweeps a second."""
    rig.shape()
    rig.configure(accumulate=1, digital=True, records=records,
                  interval_us=0)
    began = time.time()
    rig.start()
    while time.time() - began < timeout and not rig.state()['done']:
        time.sleep(0.005)
    span = time.time() - began
    state = rig.state()
    rig.stop()
    return (state['produced'] + state['dropped']) / max(span, 1e-6)


def under_load(rig, settle=0.5, window=1.5):
    """(sweeps/s, records/s, ring drift) while the link is streaming."""
    rig.start()
    time.sleep(settle)                        # the reader reaches its pace
    was = rig.state() or {}
    first = rig.buffered
    began = time.time()
    time.sleep(window)
    now = rig.state() or {}
    last = rig.buffered
    span = time.time() - began
    rig.stop()
    if span <= 0 or was.get('triggers') is None:
        return 0.0, 0.0, 0, 0.0
    sweeps = max(0.0, (now['triggers'] - was['triggers']) / span)
    records = max(0.0, (last['records'] - first['records']) / span)
    drift = (now.get('available') or 0) - (was.get('available') or 0)
    reads = max(1e-9, (last['reads'] - first['reads']) / span)
    return sweeps, records, drift, records / reads


def load(rig, args, sweeps, rate):
    """Design for `sweeps` and put it on the board. (layout, chain)."""
    rig.shape()
    try:
        chain = bessel.design(fs=sweeps, out_rate=rate, order=args.order)
    except ValueError:
        # A CHAIN NEEDS A RATE TO BE DESIGNED AGAINST.
        return rig.configure(sample_rate=rate, digital=True), None
    layout = rig.configure(accumulate=chain['boxcar'], digital=True)
    rig.shape(chain['sections'], chain['decimate'])
    chain['sweeps'] = sweeps
    return layout, chain


def plan(rig, args):
    """Measure what the loop gives, design the low-pass for it, load it."""
    # TAKING THE BOARD OVER STARTS BY TAKING IT OVER.
    rig.stop()
    sweeps = sweep_rate(rig)
    # WHAT THE LINK CARRIES, NOT WHAT THE SCREEN DRAWS.
    carries = (rig.state() or {}).get('max_rate_hz') or 0
    rate = args.rate if args.rate > 0 else carries * LINK_SHARE
    if rate <= 0:
        rate = max(1.0, args.hz)

    # A CHAIN ONLY EARNS ITS KEEP ON OVERSAMPLING.
    layout, chain = load(rig, args, sweeps, rate)
    if chain is None:
        return layout, chain

    # ONE REDESIGN, AGAINST THE LOOP IT WILL RUN IN.
    live, made, drift, per_read = under_load(rig)
    off = live and abs(live - sweeps) > 0.2 * sweeps
    fresh = load(rig, args, live, rate) if off else (None, None)
    if fresh[1] is not None:
        layout, chain = fresh
        chain['idle_sweeps'] = sweeps
    return layout, chain


def _line_share(link):
    """How much of the line the stream is actually claiming, when the link
    knows its baud and its bits.
    """
    if not (link.get('baud') and link.get('bits')):
        return ''
    return '   %2.0f%% of line' % (100.0 * link['bits'] / float(link['baud']))


def chain_box(chain, sweeps, channels=0):
    """The low-pass the board is running, and the loop underneath it."""

    title = ('LOW PASS (%d CHANNELS)' % channels if channels
             else 'LOW PASS')
    if chain is None:
        return hud(title, [
            '  none - the window averages instead',
            '  the loop could not be measured to design one',
            '  loop    %8s sweeps/s' % ('%.0f' % sweeps if sweeps else '-'),
        ])

    return hud(title, [
        '  cutoff  %8.2f Hz   order %d' % (chain['cutoff'],
                                            chain['order']),
        '  rejects %8.1f dB   %d biquads' % (chain['worst_alias_db'],
                                              len(chain['sections'])),
        '  sum %5d x keep 1/%-4d -> %6.1f rec/s' % (chain['boxcar'],
                                                     chain['decimate'],
                                                     chain['out_rate']),
        '  loop    %8s sweeps/s%s'
        % ('%.0f' % sweeps if sweeps else '-',
           '' if not chain.get('sweeps') else
           '  (designed %.0f)' % chain['sweeps']),
    ])


#: The indicator, and the two colours it comes in. One glyph, so the
#: only difference is the colour - U+25A3, a thin-framed box with a fill.
#: Written as ANSI because the art reaches the frame as a string that
#: `Marquee` parses; a Rich style object would arrive as literal text.
BOX = '\u25a3'
LIT = '\x1b[38;5;40m'          # green: the pin was high for most of the window
DARK = '\x1b[38;5;242m'        # grey: it was not
OFF = '\x1b[0m'
LABEL = '\x1b[38;5;66m'        # the theme's `label`, the street
VALUE = '[38;5;214m'       # the theme's `value`, the light source


#: The half of a pin's name that is a PERIPHERAL FUNCTION rather than a
#: signal. The board names a pin both ways - `TIM1_CH1N/PWMUL` is the
#: timer's channel and the schematic's gate - and truncating the string
#: keeps the wrong one: six gates all read `TIM1_CH1N/` and nothing said
#: which leg. Dropped, `PWMUL` and `nFAULT` are what is left.
PERIPHERAL = re.compile(r'^(TIM|SPI|USART|UART|JT|NJ)')


def short(name):
    """The signal half of a pin's name, at most nine cells."""
    parts = [p for p in name.split('/') if not PERIPHERAL.match(p)]
    return (parts[0] if parts else name)[:9]


def digital_box(pins, width=34):
    """Every sampled pin as a lit or dark box, in its own frame."""

    if not pins:
        return hud('DIGITAL', ['  this task carries no pins'])

    # THE COLOUR IS THE VALUE.
    cells = ['%s%s%s %s%-10s%s'
             % (LIT if duty >= 0.5 else DARK, BOX, OFF, LABEL,
                short(name), OFF)
             for name, duty in pins.items()]

    # Wrapped to the region rather than the terminal: the frame crops, and a
    # row that ran past it would slide instead of showing.
    lines, line = [], ''
    for cell in cells:
        plain = re.sub(r'\x1b\[[0-9;]*m', '', line + cell)
        if line and len(plain) > width:
            lines.append(line.rstrip())
            line = ''
        line += cell + '  '
    if line:
        lines.append(line.rstrip())
    return hud('DIGITAL', lines)


#: Bytes a read costs beyond its records: unit, function code and CRC
#: on the request and the reply, plus the count byte and the backlog the
#: reply appends.
PER_READ_BYTES = 4 + 4 + 1 + 4


def wire_rate(bits):
    """Bits a second as the largest unit that still leaves digits."""
    if bits >= 1e6:
        return '%.1f Mbit/s' % (bits / 1e6)
    if bits >= 1e3:
        return '%.1f kbit/s' % (bits / 1e3)
    return '%.0f bit/s' % bits


def take_link(link, seen, now):
    """Fold one sample of the library reader's queue into `link`."""
    if not link['at']:
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    elif now - link['at'] > 0.2:
        since = now - link['at']
        link['rate'] = (seen['reads'] - link['reads']) / since
        # WHAT ACTUALLY GOES DOWN THE WIRE, not the records' own size: a
        # record's bytes plus the transaction around it - unit, function, the
        # count byte, the backlog and the CRC - and ten bits a byte, because
        # 8N1 sends a start and a stop with every one.
        payload = (seen['records'] - link['seen']) * link['stride']
        frames = (seen['reads'] - link['reads']) * PER_READ_BYTES
        link['bits'] = (payload + frames) * 10.0 / since
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    link.update({k: seen[k] for k in ('host', 'peak', 'dropped', 'backlog')})


def buffer_box(state, host, link=None):
    """Every buffer between the converter and the frame, and the rate."""

    def row(name, held, peak, dropped, capacity=None):
        # the scale is the buffer's OWN PEAK, never its capacity: the ring
        # holds thousands and a reader keeping up leaves one in it, so a bar
        # against capacity is pinned at zero however hard the link works.
        top = float(max(peak, 1))
        room = '' if capacity is None else ' of %d' % capacity
        bar = gauge(min(1.0, held / top), 12)
        return ['  %-7s ' % name + bar + ' %4d' % held,
                '          peak %4d%s   dropped %d' % (peak, room, dropped)]

    lines = []
    if link is not None:
        lines += row('LINK', link['host'], link['peak'], link['dropped'])
    lines += row('HOST', host['held'], host['peak'], host['dropped'])

    capacity = (state or {}).get('capacity') or 0
    if not capacity:
        lines.append('  TARGET  this board reports no level')
    else:
        lines += row('TARGET', state.get('available') or 0,
                     state.get('worst') or 0, state.get('dropped') or 0,
                     capacity)
    if link is not None and link.get('rate'):
        lines.append('  link    %6.1f reads/s%s'
                     % (link['rate'], _line_share(link)))

    # THE RATE IN THE TITLE, the way LOW PASS carries its channel count - and
    # in a unit that has digits to show.
    title = 'BUFFER'
    if link is not None and link.get('bits'):
        title = 'BUFFER (%s)' % wire_rate(link['bits'])
    return hud(title, lines)


def scale(rows, params=None):
    """Add the scaled value and the channel's own scale, in its own unit."""
    low_code, high_code = SPAN_CODES

    for row in rows:
        unit, raw = row.get('unit'), row['mean_raw']

        convert = scaling.converter(unit, row['differential'],
                                    signal=row.get('signal'),
                                    params=params)

        # The inset codes are the thermistor's alone.
        if row['differential']:
            first, last = -32768, 32767
        elif unit == 'centi-degC':
            first, last = low_code, high_code
        else:
            first, last = 0, 65535

        with suppress(ValueError):
            row['reading'] = convert(raw)
            ends = (convert(first), convert(last))
            row['span'] = (min(ends), max(ends))

    return rows


def legend(rows, held):
    """The channel legend: full name, live value, held min and max."""

    lines = []
    for row in rows:
        name = row['signal']
        now = row.get('reading')
        unit = scaling.symbol(row.get('unit'), name)
        if now is None:
            continue
        convert = scaling.converter(row.get('unit'), row['differential'],
                                    signal=name, params=row.get('params'))
        try:
            lo = convert(row['min_raw'])
            hi = convert(row['max_raw'])
        except (KeyError, ValueError):
            lo = hi = now
        keep = held.setdefault(name, [now, now])
        keep[0] = min(lo, keep[0] + desk.RELEASE * (now - keep[0]))
        keep[1] = max(hi, keep[1] + desk.RELEASE * (now - keep[1]))
        lines.append((name, '%+9.3f %-2s  %+8.2f/%+8.2f'
                      % (now, unit, keep[0], keep[1])))
    return hud('LEGEND  now / held lo / hi', lines)


def _clocked(clock, rig, now):
    """The board's state, and the loop's own rate differentiated off the
    board's trigger count - live, and not the figure the chain was
    designed against: the two part company the moment the link is busy.
    """
    state = rig.state()
    seen, since = state.get('triggers'), now - clock['at']
    if seen is not None and clock['triggers'] is not None and since:
        clock['sweeps'] = (seen - clock['triggers']) / since
    clock['triggers'] = seen
    clock['state'], clock['at'] = state, now


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=8.0,
                        help='screen refreshes per second')
    parser.add_argument('--rate', type=float, default=0.0,
                        help='records a second the board produces. Default 0 '
                             'follows --hz, which is what a meter wants: one '
                             'reading a frame, averaged over the whole frame. '
                             'The converter is not slowed to it - it runs flat '
                             'out underneath and the board sums into each '
                             'record')
    parser.add_argument('--order', type=int, default=4,
                        help='Bessel order of the low-pass the board runs')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed')
    args = parser.parse_args(argv)

    # power_afe SAID: invariant 9 - with the rail down the board refuses to
    # start the task at all, and that refusal used to escape as a traceback
    # rather than a said line.
    rig = open_rig('LINKING CONVERTERS', port=args.port, power_afe=True,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return 1
    origin = rig.origin
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    # Every channel the board reports, summed AND SHAPED on the board rather
    # than read one at a time: a meter face wants a low-pass, and one record a
    # frame is what it draws.
    try:
        layout, chain = plan(rig, args)
        params = rig.board.analog.scaling()
        rig.start()
    except RigError as exc:
        # The board's own sentence, said - not a traceback.
        say('fail', 'task', str(exc))
        rig.close()
        return 1
    if chain is None:
        say('warn', 'low pass', 'none - the loop could not be measured, '
                                'so the window averages instead')
    else:
        say('ok', 'low pass', '%.2f Hz, order %d, %.1f dB of what would '
            'fold' % (chain['cutoff'], chain['order'],
                      chain['worst_alias_db']))

    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')
    return watch(rig, args, layout, chain, params)


#: The stand-in's machine, for the meters to show. Simulated only: a
#: bench with the stage down reads three offsets and their noise, and so
#: does the stand-in now that its phases carry the machine's current and
#: nothing invented (values.py) - so the drive holds a current vector
#: turning at DEMO_HZ electrical, "one electrical revolution every seven
#: seconds or so, slow enough to watch", and the current runs up and down
#: DEMO_AMPS over DEMO_S: a machine at crawl. On a board nothing here
#: touches the stage; the view opens onto whatever the drive is doing.
DEMO_HZ = 0.14
DEMO_AMPS = 30.0
DEMO_S = 45.0


def demo_machine(rig, origin):
    """Turn the stand-in's machine; the per-frame step that runs it up
    and down, or None on a board."""

    if origin.real:
        return None
    rig.board.gate_drivers.bypass_break(True)
    rig.board.gate_drivers.enable()
    drive = rig.drive
    drive.source('model')
    # The stand-in's record clamps the current at 5 A; the meters are 100 A
    # wide.
    drive.set_params(drv_i_max_ma=DEMO_AMPS)
    drive.setpoint(id_ref=0.0, iq_ref=0.0, theta=0.0,
                   omega_target=2.0 * math.pi * DEMO_HZ)
    drive.mode('hold')
    began = time.time()

    def step(now):
        phase = (now - began) / DEMO_S
        drive.setpoint(id_ref=DEMO_AMPS * 0.5
                       * (1.0 - math.cos(2.0 * math.pi * phase)))
    return step


def watch(rig, args, layout, chain, params):
    """Draw it until Q, ESC or the frame count runs out."""
    origin = rig.origin
    demo = demo_machine(rig, origin)
    # THE BAR FILLS THE WINDOW: at 38 columns the face floated in a sea of
    # frame.
    try:
        columns = os.get_terminal_size().columns
    except OSError:
        columns = 100
    bridge = desk.Desk(bar=max(20, min(80, columns - 72)))
    period = 1.0 / max(args.hz, 0.5)


    held = {}
    last = {'rows': [], 'pins': {}}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # THE LINK IS THIS THREAD'S AND NOTHING ELSE'S.
    clock = {'at': 0.0, 'state': None, 'triggers': None, 'sweeps': 0.0}
    # The library's own reader, sampled BEFORE the drain below empties it.
    link = {'host': 0, 'peak': 0, 'dropped': 0, 'backlog': None,
            'rate': 0.0, 'reads': 0, 'records': 0, 'seen': 0, 'bits': 0.0,
            'at': 0.0, 'stride': layout.get('stride') or 0,
            'baud': rig.baud}
    # The host's own ring.
    inbox = collections.deque()
    host = {'held': 0, 'peak': 0, 'dropped': 0}

    def read():
        now = time.time()
        if demo is not None:
            demo(now)
        if clock['state'] is None or now - clock['at'] > 0.5:
            # The buffer gauge moves slowly by construction, and this is a
            # whole round trip spent on it.
            _clocked(clock, rig, now)
        # The level BEFORE the drain: after it every queue is empty by
        # construction and the gauge would read nothing on a task that is only
        # just keeping up.
        take_link(link, rig.buffered, now)
        for record in drain(rig):
            if len(inbox) >= HOST_RING:
                # THE HOST'S OWN DROP, counted rather than hidden.
                inbox.popleft()
                host['dropped'] += 1
            inbox.append(record)
        host['held'] = len(inbox)
        host['peak'] = max(host['peak'], host['held'])
        return clock['state']

    # NOT TIED TO THE FRAME RATE.
    feed = Feed(read, period=0.01).start()

    def draw():
        # Re-fitted every frame: the bars shrink with the tty instead of
        # overflowing the frame and sliding.
        bridge.bar = max(20, min(80, (board_view.size.width or 100) - 72))
        # Everything queued since the last frame, taken in one go.
        records = [inbox.popleft() for _ in range(len(inbox))]
        host['held'] = len(inbox)
        if records:
            last['rows'] = scale(rows_from(records, layout), params)
            last['pins'] = duties_from(records)
        rows = last['rows']
        if not rows:
            said = ('no reading: %s' % feed.error if feed.error
                    else 'waiting for the first block')
            return frame_of(board_view, origin, 'METER BRIDGE', said,
                            [buffer_box(feed.latest, host, link)],
                            (('Q', 'EXIT'), ('ESC', 'MENU')),
                            art_title='ANALOGUE')
        face = bridge.update(rows, colour=console)
        for row in rows:
            row['params'] = params
        return frame_of(board_view, origin, 'METER BRIDGE', face,
                        [legend(rows, held),
                         chain_box(chain, clock['sweeps'], len(rows)),
                         buffer_box(feed.latest, host, link)],
                        (('Q', 'EXIT'), ('ESC', 'MENU')),
                        art_title='ANALOGUE',
                        under=digital_box(last['pins'], bridge.bar + 30))

    try:
        leaving = run_view(board_view, console, period, args.frames, draw)
    finally:
        # Stopped BEFORE anything puts the board back: two threads on one
        # serial transport is the one thing this arrangement must not do.
        feed.stop()
        done = [('acquisition', 'task stopped')]
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
