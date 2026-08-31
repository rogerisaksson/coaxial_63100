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


#: Records the host holds between the reader and the frame that draws
#: them. Its own ring, and it can overflow the same way the board's
#: does - which is the point of showing both.
HOST_RING = 256


def sweep_rate(rig, records=300, timeout=6.0):
    """What the acquisition loop manages, in sweeps a second.

    A FINITE RUN. Asked for no rate the board substitutes what the LINK
    carries and gates the triggers to it, so a free-running measurement
    reads the link rather than the loop. A run that ends is left alone,
    because it ends.
    """
    rig.shape()
    rig.configure(accumulate=1, digital=False, records=records,
                  interval_us=0)
    began = time.time()
    rig.start()
    while time.time() - began < timeout and not rig.state()['done']:
        time.sleep(0.005)
    span = time.time() - began
    state = rig.state()
    rig.stop()
    return (state['produced'] + state['dropped']) / max(span, 1e-6)


def plan(rig, args):
    """Measure what the loop gives, design the low-pass for it, load it.

    A METER IS A LOW-PASS, and this is where it becomes one rather than
    a decimation with a hopeful name: the converter runs flat out, the
    chain shapes what it saw, and one record a frame comes out. The
    cutoff is a fifth of that frame rate, which is the design's default
    and the reason a needle settles instead of dancing.
    """
    rate = args.rate if args.rate > 0 else max(1.0, args.hz)
    sweeps = sweep_rate(rig)
    rig.shape()

    # A CHAIN NEEDS A RATE TO BE DESIGNED AGAINST. Where the loop cannot
    # be measured - the stand-in produces only when read, so it measures
    # zero - there is nothing to design, and the clock-closed window is
    # the honest fallback: it needs no design, averages whatever the
    # window held, and says so in the box rather than drawing a filter
    # that is not running.
    try:
        chain = bessel.design(fs=sweeps, out_rate=rate, order=args.order)
    except ValueError:
        return rig.configure(sample_rate=rate, digital=False), None

    layout = rig.configure(accumulate=chain['boxcar'], digital=False)
    rig.shape(chain['sections'], chain['decimate'])
    chain['sweeps'] = sweeps
    return layout, chain


def chain_box(chain, sweeps):
    """The low-pass the board is running, and the loop underneath it.

    `sweeps` is measured live off the board's own trigger count, not the
    figure the chain was designed against: the two differ the moment the
    link gets busy, and which one is which is worth seeing.
    """
    from screen import hud

    if chain is None:
        return hud('LOW PASS', [
            '  none - the window averages instead',
            '  the loop could not be measured to design one',
            '  loop    %8s sweeps/s' % ('%.0f' % sweeps if sweeps else '-'),
        ])

    return hud('LOW PASS', [
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


def buffer_box(state, host):
    """Both ends of the pipe, because either can be the one that fills.

    HOST is what the reader thread has taken off the link and the frame
    has not drawn yet; TARGET is the board's own ring. A slow link fills
    the second; a slow terminal fills the first, and neither says
    anything about the other.

    The high-water marks are what to read. A level sampled between
    frames is a level between the peaks, and the peak is the one that
    says how close the next record came to being dropped.
    """
    from screen import gauge, hud

    lines = ['  HOST    ' + gauge(host['held'] / float(HOST_RING), 14) +
             ' %4d/%-4d' % (host['held'], HOST_RING),
             '          peak %4d   dropped %d'
             % (host['peak'], host['dropped'])]

    capacity = (state or {}).get('capacity') or 0
    if not capacity:
        lines.append('  TARGET  this board reports no level')
        return hud('BUFFER', lines)

    held = (state.get('available') or 0)
    lines.append('  TARGET  ' + gauge(held / float(capacity), 14) +
                 ' %4d/%-4d' % (held, capacity))
    lines.append('          peak %4d   dropped %d'
                 % (state.get('worst') or 0, state.get('dropped') or 0))
    return hud('BUFFER', lines)

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
    last = {'rows': []}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # THE LINK IS THIS THREAD'S AND NOTHING ELSE'S. Everything the frame
    # needs is read here and handed over as one object; `draw` below
    # touches the board not at all, so a frame costs what it costs to
    # render and the round trips happen while it is on screen.
    clock = {'at': 0.0, 'state': None, 'triggers': None, 'sweeps': 0.0}
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
        # The level BEFORE the drain: after it the ring is empty by
        # construction and the gauge would read nothing on a task that
        # is only just keeping up.
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
        rows = last['rows']
        if not rows:
            said = ('no reading: %s' % feed.error if feed.error
                    else 'waiting for the first block')
            return frame_of(board_view, origin, 'METER BRIDGE', said,
                            [buffer_box(feed.latest, host)],
                            (('Q', 'EXIT'), ('ESC', 'MENU')))
        face = bridge.update(rows, colour=console)
        for row in rows:
            row['params'] = params
        return frame_of(board_view, origin, 'METER BRIDGE', face,
                        [legend(rows, held),
                         chain_box(chain, clock['sweeps']),
                         buffer_box(feed.latest, host)],
                        (('Q', 'EXIT'), ('ESC', 'MENU')))

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
