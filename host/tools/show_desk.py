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
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import desk, scaling                          # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from screen import TO_MENU, closing, say                  # noqa: E402

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


def buffer_box(state, took):
    """The board's own buffer level: what is waiting, what it holds, and
    the fullest it has ever been.

    The high-water mark is the number worth reading. A level sampled
    after a drain is a level between the peaks and reads as an empty
    ring right up until a record is dropped; `worst` is what says how
    close the last one came.
    """
    from screen import gauge, hud

    capacity = state.get('capacity') or 0
    if not capacity:
        return hud('BUFFER', ['  this board does not report a level'])

    held, worst = state.get('available') or 0, state.get('worst') or 0
    dropped = state.get('dropped') or 0
    return hud('BUFFER', [
        '  ' + gauge(worst / float(capacity), 22),
        '  peak    %5d of %5d records' % (worst, capacity),
        '  waiting %5d      took %4d' % (held, took),
        '  dropped %5d' % dropped,
    ])


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

    # Every channel the board reports, summed on the board rather than read
    # one at a time: the accumulator carries the mean AND the two ends of
    # the window, which is exactly what a meter face wants.
    try:
        # A METER, NOT A LOGGER: one record a frame, averaged over the
        # frame. Asking for more only fills a ring this view then has to
        # spend round trips emptying.
        rate = args.rate if args.rate > 0 else max(1.0, args.hz)
        layout = rig.configure(sample_rate=rate, digital=False)
        params = rig.board.analog.scaling()
        rig.start()
    except RigError as exc:
        # The board's own sentence, said - not a traceback. The menu keeps
        # running either way.
        say('fail', 'task', str(exc))
        rig.close()
        return 1

    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')

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
    last = {'rows': [], 'state': rig.state(), 'at': 0.0}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    def draw():
        # Re-fitted every frame: the bars shrink with the tty instead of
        # overflowing the frame and sliding. 72 is labels (~44) plus the
        # instrument column's share.
        bridge.bar = max(20, min(80, (board_view.size.width or 100) - 72))
        try:
            # The level BEFORE the drain: after it the ring is empty by
            # construction and the gauge would read nothing on a task
            # that is only just keeping up.
            #
            # Twice a second, not every frame: it is a round trip spent
            # on the buffer box alone, and at eight frames a second that
            # is a third of the frame budget for a gauge that moves
            # slowly by construction.
            now = time.time()
            if now - last['at'] > 0.5:
                last['state'], last['at'] = rig.state(), now
            state = last['state']
            records = drain(rig)
        except RigError as exc:
            return frame_of(board_view, origin, 'METER BRIDGE',
                            'no reading: %s' % exc, [],
                            (('Q', 'EXIT'), ('ESC', 'MENU')))
        if records:
            last['rows'] = scale(rows_from(records, layout), params)
        rows = last['rows']
        if not rows:
            return frame_of(board_view, origin, 'METER BRIDGE',
                            'waiting for the first block', 
                            [buffer_box(state, 0)],
                            (('Q', 'EXIT'), ('ESC', 'MENU')))
        face = bridge.update(rows, colour=console)
        for row in rows:
            row['params'] = params
        return frame_of(board_view, origin, 'METER BRIDGE', face,
                        [legend(rows, held), buffer_box(state, len(records))],
                        (('Q', 'EXIT'), ('ESC', 'MENU')))

    try:
        leaving = run_view(board_view, console, period, args.frames, draw)
    finally:
        done = [('acquisition', 'task stopped')]
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
