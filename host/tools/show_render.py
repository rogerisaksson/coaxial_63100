#!/usr/bin/env python3
"""The engine watched live: the cube - or the board - under your hands.

The same staged engine the attitude view flies, on the exporter's own
cube testbed, drawn on the house stage. Grab it with the LEFT mouse
button to turn it in the room; x/y/z add five degrees about the room's
axes and X/Y/Z take them back, SPACE spins, R resets, M
swaps cube and board, the wheel zooms, Q leaves.
The HUD shows the pose, the mesh budget and what the frame cost.

    python tools/show_render.py
    python tools/show_render.py --model board
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.layout import Layout                             # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.align import Align                               # noqa: E402
from rich.table import Table                               # noqa: E402
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from coaxial import wireframe                              # noqa: E402
from coaxial.orientation import _qmul, matrix, normalise   # noqa: E402
import facecheck                                           # noqa: E402
from screen import (Keys, curtain, footer, paced, stage,   # noqa: E402
                    TO_MENU)

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False

#: Degrees per keypress, and per second under SPACE.
STEP = 5.0
SPIN_DPS = 25.0

#: Degrees per dragged cell. Terminals BATCH drag reports - a burst
#: landing as one turn read as a 40-degree jump - so the deltas go into
#: a carry that eases out at half per frame: the same total rotation,
#: spread smooth.
DRAG_DEG = 2.0


def solid_of(model):
    return facecheck.solid_for('cube-' if model == 'cube' else '')


def turn(view, axis, deg):
    """One rotation IN THE ROOM: about the screen's own axis, whatever
    the pose - pre-multiplied, which is what makes a drag feel like
    grabbing the object instead of twisting its body frame."""
    h = math.radians(deg) / 2.0
    s = math.sin(h)
    spin_q = (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h))
    view['pose'] = normalise(_qmul(spin_q, view['pose']))


def shown_angles(pose):
    """The pose as display degrees, decomposed Rz*Ry*Rx - HUD only."""
    m = matrix(pose)
    return (math.degrees(math.atan2(m[7], m[8])),
            math.degrees(math.asin(max(-1.0, min(1.0, -m[6])))),
            math.degrees(math.atan2(m[3], m[0])))


def hud(view, tris, cost_ms):
    shown = shown_angles(view['pose'])
    grid = Table.grid(padding=(0, 1))
    grid.add_column(style='label', justify='right')
    grid.add_column(style='value')
    for name, value in (('MODEL', view['model'].upper()),
                        ('X', '%+7.1f' % shown[0]),
                        ('Y', '%+7.1f' % shown[1]),
                        ('Z', '%+7.1f' % shown[2]),
                        ('SPIN', 'ON' if view['spin'] else 'off'),
                        ('ZOOM', '%.2f' % view['zoom']),
                        ('TRIS', '%d' % tris),
                        ('FRAME', '%.1f ms' % cost_ms),
                        ('LIGHT', '%.2f' % -wireframe.DUSK),
                        ('SPOT', '%.2f' % wireframe.SPOT)):
        grid.add_row(name, value)
    return Panel(grid, title=' ENGINE ', title_align='left',
                 box=box.ROUNDED, border_style='frame.hud')


def compose(view, size):
    wide = max(24, size.width - 26)
    tall = max(10, size.height - 4)
    q = view['pose']
    if view['model'] == 'cube':
        override = solid_of('cube')
        solid = override
    else:
        override = None                  # render's own board, art and all
        solid = wireframe._model()[1]
    began = time.perf_counter()
    art = wireframe.render(q, wide, tall, zoom=view['zoom'], colour=True,
                           horizon=False, tip=0.0,
                           solid=override, crew=view.get('crew'))
    cost = 1000.0 * (time.perf_counter() - began)

    body = Layout()
    body.split_row(
        Layout(Panel(Align(Text.from_ansi(art), align='center',
                           vertical='middle'),
                     title=' RENDER ', title_align='left', box=box.HEAVY,
                     border_style='frame', padding=0), name='view'),
        Layout(hud(view, len(solid[1]) // 3, cost), name='hud', size=24))

    whole = Layout()
    whole.split_column(
        Layout(body, name='body'),
        Layout(footer((('DRAG', 'TURN'), ('UP DN', 'LIGHT'),
                       ('LT RT', 'SPOT'), ('x/X y/Y z/Z', 'DEG'),
                       ('SPACE', 'SPIN'), ('R', 'RESET'), ('M', 'MODEL'),
                       ('WHEEL', 'ZOOM'), ('Q', 'EXIT'))), size=1))
    return whole


AXES3 = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def act_on(typed, view):
    for key in typed:
        if key in ('up', 'down'):
            step = -0.03 if key == 'up' else 0.03
            wireframe.DUSK = min(0.60, max(-0.30, wireframe.DUSK + step))
            continue
        if key in ('left', 'right'):
            step = 0.03 if key == 'right' else -0.03
            wireframe.SPOT = min(0.80, max(0.0, wireframe.SPOT + step))
            continue
        if key in 'xyz':
            turn(view, AXES3['xyz'.index(key)], STEP)
            continue
        if key in 'XYZ':
            turn(view, AXES3['XYZ'.index(key)], -STEP)
            continue
        if key == ' ':
            view['spin'] = not view['spin']
        elif key in 'rR':
            view['pose'] = (0.0, 0.0, 0.0, 1.0)
        elif key in 'mM':
            view['model'] = 'board' if view['model'] == 'cube' else 'cube'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--model', choices=('cube', 'board'),
                        default='cube')
    parser.add_argument('--frames', type=int, default=0,
                        help='draw this many and exit 0 - the smoke test')
    parser.add_argument('--simulated', action='store_true',
                        help='accepted for the view suite; nothing here '
                             'opens a session either way')
    parser.add_argument('--workers', type=int, default=0,
                        help='raster the board as row bands in this many '
                             'processes (0: this one). Measured at 220x60: '
                             '14.8 ms to 5.8 with eight')
    args = parser.parse_args(argv)

    page = stage()
    # Spin is OFF at start: the view opens under the hand, not on a
    # carousel - SPACE starts the idle tumble.
    view = {'model': args.model, 'pose': (0.0, 0.0, 0.0, 1.0),
            'spin': False, 'zoom': 1.0,
            'carry': (0.0, 0.0)}
    if args.workers > 0:
        from coaxial.crew import Crew
        view['crew'] = Crew(wireframe._lods(), art=wireframe._face(),
                            workers=args.workers)
    frame, last = 0, time.monotonic()

    try:
        return loop(args, page, view, frame, last)
    finally:
        if view.get('crew'):
            view['crew'].close()


def loop(args, page, view, frame, last):
    with curtain(page) as live, Keys(page.is_terminal, mouse=True) as keys:
        while True:
            now = time.monotonic()
            if view['spin']:
                step = SPIN_DPS * (now - last)
                turn(view, (0.4, 0.3, 1.0), step)
            last = now

            frame += 1
            live.update(compose(view, page.size), refresh=True)
            if args.frames and frame >= args.frames:
                return 0

            leave, moved, typed = paced(keys, 0.06)
            if leave:
                return TO_MENU if leave == 'menu' else 0
            if moved:
                view['zoom'] = max(0.3, min(4.0,
                                            view['zoom'] * (1.0 + moved)))
            view['reports'] = getattr(keys, 'reports', 0)
            dx, dy = keys.dragged()
            cx, cy = view['carry']
            cx, cy = cx + dx, cy + dy
            if dx or dy:
                view['spin'] = False
            if abs(cx) > 0.01 or abs(cy) > 0.01:
                # Half the carry each frame; rows are half-height
                # cells, so the vertical gesture doubles.
                ax, ay = cx * 0.5, cy * 0.5
                turn(view, (0, 1, 0), ax * DRAG_DEG)
                turn(view, (1, 0, 0), ay * DRAG_DEG * 2.0)
                cx, cy = cx - ax, cy - ay
            view['carry'] = (cx, cy)
            act_on(typed, view)


if __name__ == '__main__':
    sys.exit(main())
