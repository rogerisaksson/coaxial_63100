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
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from coaxial import orientation                            # noqa: E402
from screen import band_of, Keys, curtain, footer, live, paced, stage  # noqa: E402

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

#: (hotkey, name, what) in coaxial_tty.ps1's order - the exit code indexes this.
#: The hotkeys are first letters, all six unique; digits work too.
ENTRIES = (
    ('S', 'SESSION', 'board dashpanel'),
    ('B', 'BOARD ATTITUDE', 'board orientation visualizer'),
    ('A', 'SHAFT ANGLE', 'motor axle rotation position'),
    ('M', 'METER BRIDGE', 'metered channels'),
    ('G', 'GATE DRIVERS', 'half bridge control'),
    ('T', 'THERMAL OBSERVER', 'thermals estimation'),
)

#: Degrees of yaw per second for the idle tumble, with slower sways
#: about the other two axes riding on top - all three turn, none of them
#: fast enough to read as spinning.
TURN_DPS = 30.0

_BROKER = {'held': None}


def _watch_broker():
    """Keep the masthead's broker status fresh, off the frame loop."""
    from coaxial import broker

    while True:
        try:
            # Nobody serving means no connect: the file outlives a killed
            # broker, and a connect nothing answers takes its full 2 s.
            count = broker.clients() if broker.serving() else None
        except Exception:                                     # noqa: BLE001
            count = None
        # The broker holding the port IS a session; its clients ride on
        # it. Counting clients alone read 0 SESSIONS on a page opened by
        # the very demo that had the port.
        _BROKER['held'] = count + 1 if count is not None else 0
        time.sleep(3.0)


def masthead(port):
    """The top strip: the views' band, with the LIVE chip when the broker
    holds the port."""
    held = _BROKER['held']
    tag = (Text('LINK: PROBING', style='bar.dim') if held is None
           else live(held) if held else None)
    return band_of('COAXIAL 63100', 'PORT: %s' % port, tag)


def roster(picked):
    """The access list. ONLY the picked entry carries light - the rest sit
    in the same quiet label grey, so the eye finds the choice and nothing
    else competes with it."""
    # EVERY row is the same one line whether picked or not - the framed
    # highlight changed the list's height and the whole page jumped with
    # each keypress. Only the COLOUR moves now.
    lines = [Text('')]
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


#: The turntable's zoom: it opens at SWELL_FROM, fills to SWELL_LO over
#: SWELL_IN seconds, then breathes between SWELL_LO and SWELL_HI while
#: it turns. Grabbed - a drag turns it, the wheel zooms it - it holds
#: still; released, the tumble and the breath resume FROM where it was
#: left: the pose is state, and the breath re-seats its phase on the
#: zoom it finds.
SWELL_FROM, SWELL_IN = 0.25, 2.5
SWELL_LO, SWELL_HI = 0.75, 1.20
SWELL_PERIOD = 11.0
#: render()'s zoom 1.0 fits the bounding sphere at ANY attitude, which in
#: the box is 56% of its width - measured, and 2.0 is the first zoom
#: that reaches every edge. The envelope is in shares of the box, so
#: it rides on this base: 1.2 lands at 2.16, past the box, so the
#: board clips into its frame; 0.75 at 1.35, four fifths of it.
SWELL_BASE = 1.8
#: Seconds after the last touch before the idle motion takes over again.
HOLD = 0.6


def seat(zoom):
    """The breath's phase whose zoom is nearest `zoom`, on the rising
    side, so it climbs first from wherever the hand let go."""
    mid = (SWELL_LO + SWELL_HI) / 2.0
    half = (SWELL_HI - SWELL_LO) / 2.0
    return math.asin(max(-1.0, min(1.0, (zoom - mid) / half)))


def idle(view, now, dt):
    """One frame of the turntable on its own: tumble, and breathe."""
    from show_render import turn

    if now - view['touched'] < HOLD:
        return
    if view['phase'] is None and now - view['opened'] < SWELL_IN:
        # The fill, eased out: fast at first, settling at SWELL_LO.
        t = (now - view['opened']) / SWELL_IN
        view['zoom'] = SWELL_FROM + (SWELL_LO - SWELL_FROM) * (
            1.0 - (1.0 - t) ** 3)
    else:
        if view['phase'] is None:
            view['phase'] = seat(view['zoom'])
        view['phase'] += dt * 2.0 * math.pi / SWELL_PERIOD
        mid = (SWELL_LO + SWELL_HI) / 2.0
        half = (SWELL_HI - SWELL_LO) / 2.0
        want = mid + half * math.sin(view['phase'])
        # Toward the envelope rather than onto it: a zoom the wheel left
        # outside the band glides back instead of snapping.
        view['zoom'] += (want - view['zoom']) * min(1.0, 4.0 * dt)
    wobble = view['spun'] = view['spun'] + dt
    turn(view, (0.35 * math.sin(wobble * 0.5),
                0.25 * math.sin(wobble * 0.83 + 1.3), 1.0),
         TURN_DPS * dt)


def grab(view, keys, moved, now):
    """The hand on the turntable: wheel zoom, and a left-drag turn that
    pauses the idle motion while it lasts - the same degrees per cell
    and carry easing show_render.py uses against batched drag reports."""
    from show_render import DRAG_DEG, turn

    if moved:
        view['zoom'] = max(0.25, min(4.0, view['zoom'] * (1.0 + moved)))
        view['touched'], view['phase'] = now, None
    dx, dy = keys.dragged()
    cx, cy = view['carry']
    cx, cy = cx + dx, cy + dy
    if dx or dy:
        view['touched'], view['phase'] = now, None
    if abs(cx) > 0.01 or abs(cy) > 0.01:
        ax, ay = cx * 0.5, cy * 0.5
        turn(view, (0, 1, 0), ax * DRAG_DEG)
        turn(view, (1, 0, 0), ay * DRAG_DEG * 2.0)
        cx, cy = cx - ax, cy - ay
    view['carry'] = (cx, cy)


def turntable(view, width=52, height=18):
    """The board on the stand, at the pose and zoom the view holds - lit
    as the render demo lights it: camera straight down the axis, no
    horizon. The attitude view's 34-degree tip folds into the lean and
    dimmed the same board half a class. Blank until `_warm` has built
    the solids."""
    if not _STAGE['ready']:
        return ''
    return _draw(view, width, height)


def _draw(view, width, height):
    from coaxial import wireframe

    return wireframe.render(view['pose'], width, height, zoom=view['zoom'],
                            horizon=False, tip=0.0, lift=0.5,
                            least=wireframe.CREW_LEAST)


#: The turntable's solids build off the frame loop: the page is up in the
#: import's 0.3 s and the board arrives when the parse and two decimations
#: are done. The first frame used to wait 2.0 s for them, the parse twice.
_STAGE = {'ready': False}


def _warm():
    try:
        _draw({'pose': (0.0, 0.0, 0.0, 1.0), 'zoom': SWELL_FROM}, 8, 4)
    finally:
        _STAGE['ready'] = True


#: The turntable's box: this many columns of the page, and the drawing
#: fills the box's inside, so a zoom past 1.0 clips INTO the frame.
BOX = 58


def compose(port, picked, view, size=None):
    from rich.align import Align

    tall = max(8, (size.height if size else 24) - 4)
    body = Layout()
    body.split_row(
        Layout(Panel(roster(picked), box=box.ROUNDED,
                     title=Text(' MAIN TERMINAL ACCESS ', style='name'),
                     title_align='left', border_style='frame.hud',
                     padding=(1, 2), expand=True),
               name='list'),
        Layout(Panel(Align(Text.from_ansi(turntable(view, BOX - 4, tall)),
                           align='center', vertical='middle'),
                     title=Text(' COAXIAL 63100 ', style='name'),
                     title_align='left', box=box.HEAVY, border_style='frame',
                     padding=(0, 1), expand=True), name='board', size=BOX))

    whole = Layout()
    whole.split_column(
        Layout(masthead(port), size=1),
        Layout(body, name='body'),
        Layout(footer((('UP DOWN', 'NAVIGATE'), ('ENTER', 'SELECT'),
                       ('S B A M G T', 'DIRECT'), ('DRAG', 'TURN'),
                       ('WHEEL', 'ZOOM'), ('Q', 'EXIT'))),
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
    # The turntable's state: pose and zoom persist across a grab, so the
    # idle motion carries on from wherever the hand left it.
    view = {'pose': (0.0, 0.0, 0.0, 1.0), 'zoom': SWELL_FROM,
            'phase': None, 'opened': began, 'touched': -1e9, 'spun': 0.0,
            'carry': (0.0, 0.0)}
    last = began

    if not console and not args.frames:
        # No terminal to page on: read the choice as a line, the way the
        # old chooser fell back. `echo 3 | coaxial_tty.ps1` still picks a view,
        # and a closed stdin is a quit rather than a spin.
        line = sys.stdin.readline().strip().lower()
        if line.isdigit() and 1 <= int(line) <= len(ENTRIES):
            return 101 + int(line) - 1
        for i, (key, _name, _what) in enumerate(ENTRIES):
            if line == key.lower():
                return 101 + i
        return 0

    threading.Thread(target=_watch_broker, daemon=True).start()
    warm = threading.Thread(target=_warm, daemon=True)
    warm.start()
    if args.frames:
        warm.join()      # the smoke test draws the board, not the wait
    hotkeys = {key.lower(): i for i, (key, _n, _w) in enumerate(ENTRIES)}

    with curtain(page) as live, Keys(console, mouse=True) as keys:
        while True:
            frame += 1
            now = time.monotonic()
            idle(view, now, now - last)
            last = now
            live.update(compose(args.port, picked, view, page.size),
                        refresh=True)
            if args.frames and frame >= args.frames:
                return 0

            leave, moved, typed = paced(keys, 0.08)
            if leave:
                return 0
            grab(view, keys, moved, time.monotonic())
            picked, chosen = _act(typed, picked, hotkeys)
            if chosen is not None:
                return chosen


if __name__ == '__main__':
    sys.exit(main())
