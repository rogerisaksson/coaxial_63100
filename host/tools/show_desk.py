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
from coaxial import Coaxial63100                           # noqa: E402
from screen import paced, TO_MENU, Keys, closing, say  # noqa: E402

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll


#: The codes a scale is measured between. Not 0 and 65535: the thermistor
#: conversion diverges at both rails - infinite resistance one end, zero the
#: other - so its scale is quoted between codes the converter can actually
#: resolve rather than at two asymptotes.
SPAN_CODES = (1024, 64512)


def rows_from(live, layout):
    """The accumulator as the rows this renderer already understands.

    The gate drivers wants a mean and the two ends of the window. The task
    reports a sum, a count and the lowest and highest it saw, per channel,
    so the mean is a division and the ends are already measured - which is
    what a meter's ticks should be rather than something inferred from a
    mean.
    """
    out = []
    for field in layout['fields']:
        name = field['signal']
        count = max(1, live['count'][name])
        out.append({
            'index': field['channel'],
            'signal': name,
            'unit': field['unit'],
            'differential': field['differential'],
            'mean_raw': live['sum'][name] / count,
            'min_raw': live['lowest'][name],
            'max_raw': live['highest'][name],
            'samples': count,
        })
    return out


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
    parser.add_argument('--samples', type=int, default=32,
                        help='burst length per refresh. The min and max ticks '
                             'are taken over this window')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed')
    args = parser.parse_args(argv)

    # power_afe SAID: invariant 9 - with the rail down the board refuses
    # to start the task at all, and that refusal used to escape as a
    # traceback rather than a said line.
    try:
        from screen import boot
        with boot('LINKING CONVERTERS'):
            rig = Coaxial63100(port=args.port, power_afe=True,
                               simulated_device=bool(args.simulated)).open()
    except RigError as exc:
        # The board's own sentence - a rail refused while a stage is armed
        # lands here - said instead of thrown, so the menu can come back.
        say('fail', 'open', str(exc))
        return 1
    origin = rig.origin
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    # Every channel the board reports, summed on the board rather than read
    # one at a time: the accumulator carries the mean AND the two ends of
    # the window, which is exactly what a meter face wants.
    try:
        layout = rig.configure(rate_hz=None, accumulate=args.samples,
                               digital=False)
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
    gate_drivers = desk.Desk(bar=max(20, min(80, columns - 72)))
    period = 1.0 / max(args.hz, 0.5)
    frame = 0

    from screen import curtain, frame_of, stage

    held = {}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    try:
        with curtain(board_view) as show, Keys(console) as keys:
            while True:
                boxes = []
                # Re-fitted every frame: the bars shrink with the tty
                # instead of overflowing the frame and sliding. 72 is
                # labels (~44) plus the instrument column's share.
                width = board_view.size.width or 100
                gate_drivers.bar = max(20, min(80, width - 72))
                try:
                    live = rig.latest()
                    rows = scale(rows_from(live, layout), params)
                    face = gate_drivers.update(rows, colour=console)
                    for row in rows:
                        row['params'] = params
                    boxes = [legend(rows, held)]
                except RigError as exc:
                    face = 'no reading: %s' % exc

                frame += 1
                show.update(frame_of(board_view, origin, 'METER BRIDGE',
                                     face, boxes,
                                     (('Q', 'EXIT'), ('ESC', 'MENU'))),
                            refresh=True)

                if args.frames and frame >= args.frames:
                    break
                # These two have nothing to zoom, so the wheel is ignored.
                leaving, _moved, _typed = paced(keys, period)
                if leaving:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        done = [('acquisition', 'task stopped')]
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
