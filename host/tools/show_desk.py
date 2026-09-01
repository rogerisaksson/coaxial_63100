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
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import bessel, desk, scaling                  # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from screen import Feed, TO_MENU, closing, say            # noqa: E402

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll


#: The codes a scale is measured between. Not 0 and 65535: the thermistor
#: conversion diverges at both rails - infinite resistance one end, zero the
#: other - so its scale is quoted between codes the converter can actually
#: resolve rather than at two asymptotes.
SPAN_CODES = (1024, 64512)


def drain(rig, cap=32):
    """Every record the ring is holding, up to `cap`.

    REAL DAQ, not a point a frame: the board buffers at its own rate and
    the link carries whole blocks, so a frame reads what accumulated
    between frames instead of one reading and a shrug about the rest.

    THE CAP IS SMALL ON PURPOSE. One `acquire` is one PDU - five records
    at a ten-channel stride - so a big cap is a hundred round trips in a
    frame: measured at 512, a frame took 5.8 s and the view read as hung
    while the board made another five hundred to replace them. A meter
    face wants one averaged reading a frame, and asks the board for a
    rate that gives it exactly that.
    """
    got = []
    while len(got) < cap:
        block = rig.acquire()
        if not block:
            break
        got.extend(block)
    return got


def rows_from(records, layout):
    """A drained block as the rows this renderer already understands.

    The mean is every sum in the block over every count in it, so a frame
    that drained forty records reports the mean of all of them rather than
    of the last one.

    THE ENDS ARE THE RECORDS', not the raw samples'. The board already
    averaged inside each record - that is what stops the aliasing - so
    the extremes here are of what it produced at the record rate, which
    is what this meter is showing. The live accumulator's lowest and
    highest were per sample and are a different measurement; nothing here
    claims them.
    """
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
    """Each pin's duty, meaned over the block. {} when none were asked for.

    Kept apart from `rows_from` because a row is one analog channel and a
    duty belongs to the record - averaging it per row would report the
    same number as many times as there are channels.
    """
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
    """What the acquisition loop manages, in sweeps a second.

    A FINITE RUN. Asked for no rate the board substitutes what the LINK
    carries and gates the triggers to it, so a free-running measurement
    reads the link rather than the loop. A run that ends is left alone,
    because it ends.
    """
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
    """(sweeps/s, records/s, ring drift) while the link is streaming.

    NOT THE SAME LOOP, AND NOT THE SAME LINK. Sampling and the Modbus
    handler share main() on this board, so the loop measured with the link
    idle is not the one a stream runs in - measured 1870 sweeps/s designing
    against 635 running. And the rate the link sustains is not the rate it
    was asked for. Both come from the board's own counters, so this counts
    what happened rather than what was intended.

    `drift` is what the ring gained over the window. Positive means the
    board is making more than the link takes, which is the aliasing-free
    way to overrun and also the way to fill 8340 records and start
    dropping - so it is the signal to decimate harder.
    """
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
        # A CHAIN NEEDS A RATE TO BE DESIGNED AGAINST. Where the loop
        # cannot be measured - the stand-in produces only when read, so it
        # measures zero - the clock-closed window is the honest fallback.
        return rig.configure(sample_rate=rate, digital=True), None
    layout = rig.configure(accumulate=chain['boxcar'], digital=True)
    rig.shape(chain['sections'], chain['decimate'])
    chain['sweeps'] = sweeps
    return layout, chain


def plan(rig, args):
    """Measure what the loop gives, design the low-pass for it, load it.

    A METER IS A LOW-PASS, and this is where it becomes one rather than
    a decimation with a hopeful name: the converter runs flat out, the
    chain shapes what it saw, and one record a frame comes out. The
    cutoff is a fifth of that frame rate, which is the design's default
    and the reason a needle settles instead of dancing.
    """
    # TAKING THE BOARD OVER STARTS BY TAKING IT OVER. A session that
    # died between start() and stop() leaves a task running, and the
    # board then refuses every shape() and configure() - correctly, since
    # coefficients must not change under a half-drained buffer. This view
    # is replacing the task wholesale, so that buffer is not its concern.
    # MEASURED 2026-09-01: a crashed script left one running and the view
    # printed the board's refusal instead of drawing, every run, until
    # someone stopped it by hand. stop() is idempotent - checked idle.
    rig.stop()
    sweeps = sweep_rate(rig)
    # WHAT THE LINK CARRIES, NOT WHAT THE SCREEN DRAWS. Pinned to the frame
    # rate the meter asked for 8 rec/s and a 1.60 Hz passband - a needle so
    # damped it read as broken. The board says what it can carry for the
    # stride it actually has, and a fifth of that is the passband. `--rate`
    # still takes you at your word. The records land in the host queue
    # either way; the frame draws whatever is there.
    carries = (rig.state() or {}).get('max_rate_hz') or 0
    rate = args.rate if args.rate > 0 else carries * LINK_SHARE
    if rate <= 0:
        rate = max(1.0, args.hz)

    # A CHAIN ONLY EARNS ITS KEEP ON OVERSAMPLING. Decimating by N means
    # the loop spends N sweeps to produce one record, and on this board
    # the loop and the Modbus handler share main() - so those sweeps come
    # straight off the link. MEASURED, ten channels at stride 55: the
    # chain at ratio 3 moved 44.5 kbit/s, at ratio 1 moved 48.8, and the
    # clock-closed window with no chain at all moved 72.1 - 63% of the
    # line against 39%. Below three sweeps a record there is not enough
    # oversampling left to be worth that, so the window takes it.
    # THE CHAIN IS A FIXED POINT, NOT A CALCULATION. Its decimation sets
    # how many sweeps the loop spends per record; those sweeps come off
    # the link, which changes the loop, which changes the right
    # decimation. Designing once against a loop measured with the link
    # idle over-decimated by about 1.7x and left the view at 39% of the
    # line. So: design, run it, measure what the board and the link
    # actually did, design again against those. It settles in two or
    # three rounds and each costs two seconds of startup.
    #
    # WITHOUT ALIASING, which is the whole constraint. Every round is a
    # COMPLETE design for the loop rate measured in it - boxcar,
    # coefficients and decimation together - so the passband is always a
    # fifth of the output the link is actually carrying and what folds is
    # always stopped by a filter that ran at the rate it was designed for.
    # Raising the output alone would be the aliasing.
    layout, chain = load(rig, args, sweeps, rate)
    if chain is None:
        return layout, chain

    # ONE REDESIGN, AGAINST THE LOOP IT WILL RUN IN. `sweep_rate` measures
    # a burst with the link idle and that is not the loop a stream runs in.
    #
    # A CLOSED LOOP WAS TRIED AND MEASURED WORSE. Three rounds of "run it,
    # measure what the board and the link did, design again" settled at 43%
    # of the line where this fixed ask holds 65%: `under_load` has to read
    # the loop rate while the reader is still reaching its pace, so it
    # overstates - 2780 sweeps/s against the 635 the stream settles at - and
    # every round then over-decimates. Making the window long enough to be
    # honest costs ten seconds of startup, which a meter does not have. The
    # controller is the right shape and the measurement under it is not.
    live, made, drift, per_read = under_load(rig)
    if live and abs(live - sweeps) > 0.2 * sweeps:
        fresh = load(rig, args, live, rate)
        if fresh[1] is not None:
            layout, chain = fresh
            chain['idle_sweeps'] = sweeps
    return layout, chain


def chain_box(chain, sweeps, channels=0):
    """The low-pass the board is running, and the loop underneath it.

    `sweeps` is measured live off the board's own trigger count, not the
    figure the chain was designed against: the two differ the moment the
    link gets busy, and which one is which is worth seeing.
    """
    from screen import hud

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
    """Every sampled pin as a lit or dark box, in its own frame.

    A DUTY, NOT A LEVEL. The record carries what fraction of the window
    the pin was high, because a level sampled once and decimated is
    aliased by construction - KEEPALIVE toggles at ~100 kHz and read as a
    coin toss. So the box is lit when the pin was high for most of the
    window, and the percentage beside it is what it actually did.
    """
    from screen import hud

    if not pins:
        return hud('DIGITAL', ['  this task carries no pins'])

    # THE COLOUR IS THE VALUE. The record carries a duty because the
    # window is long and a pin can toggle inside it; under a half the box
    # is dark and over it is lit, and printing the same bit as a digit
    # beside it only asks the eye to read twice. A switching gate then
    # flickers, which is the reading rather than noise in it.
    cells = ['%s%s%s %s%-10s%s'
             % (LIT if duty >= 0.5 else DARK, BOX, OFF, LABEL,
                short(name), OFF)
             for name, duty in pins.items()]

    # Wrapped to the region rather than the terminal: the frame crops, and
    # a row that ran past it would slide instead of showing.
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
    """Fold one sample of the library reader's queue into `link`.

    `reads` is a counter, so the rate is differentiated here the way the
    trigger rate is - and the first sample only sets the origin. It used
    to read `link['at'] or now`, which takes a starting 0.0 as falsy and
    made every interval zero, so the rate never left 0.
    """
    if not link['at']:
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    elif now - link['at'] > 0.2:
        since = now - link['at']
        link['rate'] = (seen['reads'] - link['reads']) / since
        # WHAT ACTUALLY GOES DOWN THE WIRE, not the records' own size: a
        # record's bytes plus the transaction around it - unit, function,
        # the count byte, the backlog and the CRC - and ten bits a byte,
        # because 8N1 sends a start and a stop with every one.
        payload = (seen['records'] - link['seen']) * link['stride']
        frames = (seen['reads'] - link['reads']) * PER_READ_BYTES
        link['bits'] = (payload + frames) * 10.0 / since
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    link.update({k: seen[k] for k in ('host', 'peak', 'dropped', 'backlog')})


def buffer_box(state, host, link=None):
    """Every buffer between the converter and the frame, and the rate.

    THREE PLACES A RECORD CAN WAIT, and each fills for its own reason:
    TARGET when the link cannot keep up, LINK when the reader thread is
    starved of turns, HOST when the terminal is slow. Naming them apart
    is the whole point - a slowness charged to the wrong one sent 208
    dropped records to the board's account once.

    Levels are sampled BEFORE the frame drains them, and the bar runs to
    each buffer's own high-water mark rather than its capacity: the ring
    holds 5349 records and a reader that is keeping up leaves 1 in it, so
    a bar against capacity is pinned at zero however hard the link works.
    The peak is what says how close the next record came to being
    dropped, and it is the number to read.
    """
    from screen import gauge, hud

    def row(name, held, peak, dropped, capacity=None):
        # the scale is the buffer's OWN PEAK, never its capacity: the ring
        # holds thousands and a reader keeping up leaves one in it, so a
        # bar against capacity is pinned at zero however hard the link
        # works. Against the peak it answers a question worth asking -
        # how full is this now, against the worst it has been.
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
        # How much of the line the stream is actually claiming. The
        # question this answers was asked out loud once - a 115200 link
        # carries 11.5 kB/s and a task at 57 records a second is using a
        # third of it, with the rest going to the fixed cost of a
        # transaction rather than to records.
        share = ''
        if link.get('baud') and link.get('bits'):
            share = '   %2.0f%% of line' % (100.0 * link['bits']
                                            / float(link['baud']))
        lines.append('  link    %6.1f reads/s%s' % (link['rate'], share))

    # THE RATE IN THE TITLE, the way LOW PASS carries its channel count -
    # and in a unit that has digits to show. Megabits was stone dead on
    # this link: 0.035 rounds to 0.0 and stays there however hard the
    # stream works, so the scale follows the number instead of the number
    # being flattened to fit one scale. Fibre will read in Mbit/s on its
    # own.
    title = 'BUFFER'
    if link is not None and link.get('bits'):
        title = 'BUFFER (%s)' % wire_rate(link['bits'])
    return hud(title, lines)


def scale(rows, params=None):
    """Add the scaled value and the channel's own scale, in its own unit.

    Here rather than in desk.py because the renderer takes a reading and its
    scale and returns text: which channel means amperes, and what its ends
    come to, are the analog layer's facts. The conversions are the ones
    scaling.py already carries and test_simulated already checks.

    The scale is measured, not extrapolated. Dividing a reading by its
    fraction of full scale gives the right answer for the phases and the DC
    link, because both are linear in the code - and the wrong one for the
    thermistor, which is not. Converting the end codes works for all three.
    """
    low_code, high_code = SPAN_CODES

    for row in rows:
        unit, raw = row.get('unit'), row['mean_raw']

        convert = scaling.converter(unit, row['differential'],
                                    signal=row.get('signal'),
                                    params=params)

        # The inset codes are the thermistor's alone. Every other conversion
        # here is linear in the code and has a finite value at both rails, so
        # quoting it short of them would understate the converter's range.
        if row['differential']:
            first, last = -32768, 32767
        elif unit == 'centi-degC':
            first, last = low_code, high_code
        else:
            first, last = 0, 65535

        try:
            row['reading'] = convert(raw)
            ends = (convert(first), convert(last))
            row['span'] = (min(ends), max(ends))
        except ValueError:
            pass          # at a divider rail; the raw code still stands

    return rows


#: How fast the held peaks fall back, per frame, as a fraction of the
#: hold's distance to the current reading. 0.02 at 8 Hz is a few seconds
#: of decay - the mixing-desk feel: instant attack, slow release.
PEAK_DECAY = 0.02


def legend(rows, held):
    """The channel legend: full name, live value, held min and max.

    `held` is the view's memory across frames - attack is instant, decay
    creeps, the way a desk's peak lamps behave. Values arrive converted,
    so the legend speaks each channel's own unit.
    """
    from screen import hud

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
        keep[0] = min(lo, keep[0] + PEAK_DECAY * (now - keep[0]))
        keep[1] = max(hi, keep[1] + PEAK_DECAY * (now - keep[1]))
        lines.append((name, '%+9.3f %-2s  %+8.2f/%+8.2f'
                      % (now, unit, keep[0], keep[1])))
    return hud('LEGEND  now / held lo / hi', lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
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

    # power_afe SAID: invariant 9 - with the rail down the board refuses
    # to start the task at all, and that refusal used to escape as a
    # traceback rather than a said line.
    from screen import open_rig
    rig = open_rig('LINKING CONVERTERS', port=args.port, power_afe=True,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return 1
    origin = rig.origin
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    # Every channel the board reports, summed AND SHAPED on the board
    # rather than read one at a time: a meter face wants a low-pass, and
    # one record a frame is what it draws.
    try:
        layout, chain = plan(rig, args)
        params = rig.board.analog.scaling()
        rig.start()
    except RigError as exc:
        # The board's own sentence, said - not a traceback. The menu keeps
        # running either way.
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


def watch(rig, args, layout, chain, params):
    """Draw it until Q, ESC or the frame count runs out.

    Its own function because `main` is setup and this is the run -
    and because the two together were 159 lines, which is past what
    the structure suite lets a reader hold in one piece."""
    origin = rig.origin
    # THE BAR FILLS THE WINDOW: at 38 columns the face floated in a sea
    # of frame. Legend and labels take ~44; the bar gets the rest.
    try:
        columns = os.get_terminal_size().columns
    except OSError:
        columns = 100
    bridge = desk.Desk(bar=max(20, min(80, columns - 72)))
    period = 1.0 / max(args.hz, 0.5)

    from screen import frame_of, run_view, stage

    held = {}
    last = {'rows': [], 'pins': {}}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # THE LINK IS THIS THREAD'S AND NOTHING ELSE'S. Everything the frame
    # needs is read here and handed over as one object; `draw` below
    # touches the board not at all, so a frame costs what it costs to
    # render and the round trips happen while it is on screen.
    clock = {'at': 0.0, 'state': None, 'triggers': None, 'sweeps': 0.0}
    # The library's own reader, sampled BEFORE the drain below empties it.
    # Its `reads` is a counter, so the rate is differentiated here the way
    # the trigger rate is.
    link = {'host': 0, 'peak': 0, 'dropped': 0, 'backlog': None,
            'rate': 0.0, 'reads': 0, 'records': 0, 'seen': 0, 'bits': 0.0,
            'at': 0.0, 'stride': layout.get('stride') or 0,
            'baud': getattr(rig, 'baud', 0) or 0}
    # The host's own ring. `deque` because append and popleft are atomic
    # under the GIL - the reader fills one end and the frame empties the
    # other, and neither waits for the other to finish.
    inbox = collections.deque()
    host = {'held': 0, 'peak': 0, 'dropped': 0}

    def read():
        now = time.time()
        if clock['state'] is None or now - clock['at'] > 0.5:
            # The buffer gauge moves slowly by construction, and this is
            # a whole round trip spent on it.
            state = rig.state()
            # The loop's own rate, differentiated off the board's trigger
            # count - live, and not the figure the chain was designed
            # against: the two part company the moment the link is busy.
            seen, since = state.get('triggers'), now - clock['at']
            if seen is not None and clock['triggers'] is not None and since:
                clock['sweeps'] = (seen - clock['triggers']) / since
            clock['triggers'] = seen
            clock['state'], clock['at'] = state, now
        # The level BEFORE the drain: after it every queue is empty by
        # construction and the gauge would read nothing on a task that
        # is only just keeping up.
        take_link(link, rig.buffered, now)
        for record in drain(rig):
            if len(inbox) >= HOST_RING:
                # THE HOST'S OWN DROP, counted rather than hidden. It was
                # hidden before: the reader replaced its last result and
                # whatever the frame had not drawn went with it.
                inbox.popleft()
                host['dropped'] += 1
            inbox.append(record)
        host['held'] = len(inbox)
        host['peak'] = max(host['peak'], host['held'])
        return clock['state']

    # NOT TIED TO THE FRAME RATE. The reader's job is to keep the
    # board's ring empty, which is the link's business and not the
    # screen's: at a period of 1/(2*hz) a one-frame-a-second terminal
    # read twice a second and the TARGET ring overflowed - 356 of 356
    # and 208 dropped - for a slowness that was entirely the
    # terminal's. Ten milliseconds, and a slow terminal now fills the
    # HOST ring instead, which is where the blame belongs.
    feed = Feed(read, period=0.01).start()

    def draw():
        # Re-fitted every frame: the bars shrink with the tty instead of
        # overflowing the frame and sliding. 72 is labels (~44) plus the
        # instrument column's share.
        bridge.bar = max(20, min(80, (board_view.size.width or 100) - 72))
        # Everything queued since the last frame, taken in one go. The
        # reader may add while this runs; those are the next frame's.
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
