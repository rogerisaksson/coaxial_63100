#!/usr/bin/env python3
"""The main terminal page: the chooser, drawn like the reference screens.

A list to the left under `// MAIN TERMINAL ACCESS`, the board itself to the
right, turning slowly - the toon mesh off the CAD export, so the first thing
the terminal shows is the hardware it is for. No session is opened here:
the page has to be instant, so the only live datum is whether a broker is
serving - and even that is fetched by a background thread, because probing
it inline cost 2 029 ms A FRAME (measured, tools/uibench.py) and the page
drew at half a frame per second.

The choice leaves through the EXIT CODE - stdout is the drawing's:
    0            quit
    101 + index  the picked entry, in ENTRIES order
"""
import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.console import Group                             # noqa: E402
from rich.layout import Layout                             # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.table import Table                               # noqa: E402
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from coaxial import orientation                            # noqa: E402
from screen import Keys, curtain, footer, paced, stage     # noqa: E402

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

#: (hotkey, name, what) in demo.ps1's order - the exit code indexes this.
#: The hotkeys are first letters, all six unique; digits work too.
ENTRIES = (
    ('S', 'SESSION', 'one dash over the whole board'),
    ('B', 'BOARD ATTITUDE', 'the CAD export, turned by the IMU'),
    ('A', 'SHAFT ANGLE', 'a protractor face over the A1335'),
    ('M', 'METER BRIDGE', 'every analog channel, metered'),
    ('G', 'GATE DRIVERS', 'six signals, currents, a burst'),
    ('T', 'THERMAL OBSERVER', 'where the heat sits'),
)

#: Degrees of yaw per second for the idle tumble, with slower sways
#: about the other two axes riding on top - all three turn, none of them
#: fast enough to read as spinning.
TURN_DPS = 30.0

_BROKER = {'text': None}


def _watch_broker():
    """Keep the masthead's broker status fresh, off the frame loop."""
    from coaxial import broker

    while True:
        try:
            serving = broker.serving()
            count = broker.clients() or 0
        except Exception:                                     # noqa: BLE001
            serving, count = None, 0
        _BROKER['text'] = (('BROKER: %d SESSION%s'
                            % (count, '' if count == 1 else 'S'))
                           if serving else '')
        time.sleep(3.0)


def masthead(port):
    """The top strip: the terminal's name left, the live facts right."""
    right = Text()
    right.append('PORT: %s' % port, style='label')
    status = _BROKER['text']
    if status is None:
        right.append('   LINK: PROBING', style='label')
    elif status:
        right.append('   ' + status, style='value')
    bar = Table.grid(expand=True, padding=(0, 1))
    bar.add_column(justify='left')
    bar.add_column(justify='right')
    bar.add_row(Text('  COAXIAL 63100', style='bar'), right)
    bar.style = 'bar.dim'
    return bar


def roster(picked):
    """The access list. ONLY the picked entry carries light - the rest sit
    in the same quiet label grey, so the eye finds the choice and nothing
    else competes with it."""
    # EVERY row is the same one line whether picked or not - the framed
    # highlight changed the list's height and the whole page jumped with
    # each keypress. Only the COLOUR moves now.
    lines = [Text('// MAIN TERMINAL ACCESS', style='name'), Text('')]
    for i, (key, name, what) in enumerate(ENTRIES):
        if i == picked:
            lines.append(Text.assemble(('  > ', 'value'), (key, 'value'),
                                       ('  ', ''), (name, 'value'),
                                       ('   ' + what, 'label')))
        else:
            lines.append(Text.assemble('    ', (key, 'label'), ('  ', ''),
                                       (name, 'label'),
                                       ('   ' + what, 'label')))
        lines.append(Text(''))
    return Group(*lines)


def turntable(seconds, zoom=1.0):
    """The board tumbling on the stand: the same vector drawing the
    attitude view flies, idling through a slow multi-axis roll."""
    from coaxial import wireframe

    yaw = math.radians(TURN_DPS * seconds) / 2.0
    pitch = math.radians(18.0 * math.sin(seconds * 0.5)) / 2.0
    roll = math.radians(12.0 * math.sin(seconds * 0.83 + 1.3)) / 2.0
    q = orientation._qmul(
        (math.sin(pitch), 0.0, 0.0, math.cos(pitch)),
        orientation._qmul(
            (0.0, math.sin(roll), 0.0, math.cos(roll)),
            (0.0, 0.0, math.sin(yaw), math.cos(yaw))))
    return wireframe.render(q, 52, 18, zoom=zoom, axes=False)


def compose(port, picked, seconds, zoom=1.0):
    from rich.align import Align

    body = Layout()
    body.split_row(
        Layout(Panel(roster(picked), box=box.ROUNDED,
                     border_style='frame.hud', padding=(1, 2), expand=True),
               name='list'),
        Layout(Panel(Align(Text.from_ansi(turntable(seconds, zoom)),
                           align='center', vertical='middle'),
                     title=Text(' COAXIAL 63100 ', style='name'),
                     title_align='left', box=box.HEAVY, border_style='frame',
                     padding=(0, 1), expand=True), name='board', size=58))

    whole = Layout()
    whole.split_column(
        Layout(masthead(port), size=1),
        Layout(body, name='body'),
        Layout(footer((('UP DOWN', 'NAVIGATE'), ('ENTER', 'SELECT'),
                       ('S B A M G T', 'DIRECT'), ('WHEEL', 'ZOOM'),
                       ('Q', 'EXIT'))),
               size=1))
    return whole


def _act(typed, picked, hotkeys):
    """(new pick, chosen exit code or None) for one frame of keys."""
    for key in typed:
        if key == 'up':
            picked = (picked - 1) % len(ENTRIES)
        elif key == 'down':
            picked = (picked + 1) % len(ENTRIES)
        elif key in ('\r', '\n'):
            return picked, 101 + picked
        elif key.isdigit() and 1 <= int(key) <= len(ENTRIES):
            return picked, 101 + int(key) - 1
        elif key.lower() in hotkeys:
            return picked, 101 + hotkeys[key.lower()]
    return picked, None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--frames', type=int, default=0,
                        help='draw this many and exit 0 - the smoke test')
    parser.add_argument('--simulated', action='store_true',
                        help='accepted for the view suite; the page opens '
                             'no session either way')
    args = parser.parse_args(argv)

    page = stage()
    console = page.is_terminal
    picked, frame, began = 0, 0, time.monotonic()
    zoom = 1.0

    if not console and not args.frames:
        # No terminal to page on: read the choice as a line, the way the
        # old chooser fell back. `echo 3 | demo.ps1` still picks a view,
        # and a closed stdin is a quit rather than a spin.
        line = sys.stdin.readline().strip().lower()
        if line.isdigit() and 1 <= int(line) <= len(ENTRIES):
            return 101 + int(line) - 1
        for i, (key, _name, _what) in enumerate(ENTRIES):
            if line == key.lower():
                return 101 + i
        return 0

    threading.Thread(target=_watch_broker, daemon=True).start()
    hotkeys = {key.lower(): i for i, (key, _n, _w) in enumerate(ENTRIES)}

    with curtain(page) as live, Keys(console, mouse=True) as keys:
        while True:
            frame += 1
            live.update(compose(args.port, picked,
                                time.monotonic() - began, zoom),
                        refresh=True)
            if args.frames and frame >= args.frames:
                return 0

            leave, moved, typed = paced(keys, 0.08)
            if leave:
                return 0
            if moved:
                zoom = max(0.4, min(4.0, zoom * (1.0 + moved)))
            picked, chosen = _act(typed, picked, hotkeys)
            if chosen is not None:
                return chosen


if __name__ == '__main__':
    sys.exit(main())
