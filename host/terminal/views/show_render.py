#!/usr/bin/env python3
"""The engine watched live: the cube - or the board - under your hands.

The same staged engine the attitude view flies, on the exporter's own
cube testbed, drawn on the house stage. Grab it with the left mouse
button to turn it in the room; x/y/z add five degrees about the room's
axes and X/Y/Z take them back, SPACE spins, R resets, M
swaps cube and board, the wheel zooms, Q leaves.
The HUD shows the pose, the mesh budget and what the frame cost.

    python terminal/views/show_render.py
    python terminal/views/show_render.py --model board
"""
import argparse
import math
import sys
import time

from rich.layout import Layout

from coaxial.draw.orientation import _qmul, matrix, normalise
from coaxial.errors import RigError
from coaxial.graphics import card, gpu, shading, wireframe
from coaxial.graphics.crew import Crew
from terminal.loader import answer
from terminal.ui import bindings, screen as _screen
from terminal.ui.console import WHEEL_STEP, Keys
from terminal.ui.rate import rate_of
from terminal.ui.screen import paced
from terminal.ui.stage import band_of, curtain, hud, stage, viewport
from terminal.ui.taskbar import bar_of, column, footer, rail, room
from tools.render import facecheck

_screen.CHATTER = False

#: Degrees per keypress, and per second under SPACE.
STEP = 5.0
SPIN_DPS = 25.0

#: Degrees per dragged cell. Terminals batch drag reports (a burst
#: landing as one turn read as a 40-degree jump), so the deltas go into
#: a carry that eases out at half per frame: the same total rotation,
#: spread smooth.
DRAG_DEG = 2.0


def solid_of(model):
    return facecheck.solid_for('cube-' if model == 'cube' else '')


def turn(view, axis, deg):
    """One rotation in the room: about the screen's own axis, whatever
    the pose. Pre-multiplied, so a drag grabs the object instead of
    twisting its body frame."""
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


def engine(view, tris, cost_ms):
    """The column's instrument: the pose, the mesh budget, what the frame cost (`stage.hud`)."""
    shown = shown_angles(view['pose'])
    return hud('ENGINE', (('MODEL', view['model'].upper()),
                          ('X', '%+7.1f' % shown[0]),
                          ('Y', '%+7.1f' % shown[1]),
                          ('Z', '%+7.1f' % shown[2]),
                          ('SPIN', 'ON' if view['spin'] else 'off'),
                          ('ZOOM', '%.2f' % view['zoom']),
                          ('TRIS', '%d' % tris),
                          ('FRAME', '%.1f ms' % cost_ms),
                          ('LIGHT', '%.2f' % -shading.DUSK),
                          ('SPOT', '%.2f' % shading.SPOT)))


def compose(view, size, rate=''):
    wide = max(24, room(size.width))
    tall = max(10, size.height - 4)
    q = view['pose']
    # The cube is drawn from its own solid; the board is render's own, art and
    # all.
    cube = view['model'] == 'cube'
    override = solid_of('cube') if cube else None
    if cube and override is None:
        raise RigError('no cube to draw')
    solid = override if override is not None else wireframe._model()[1]
    began = time.perf_counter()
    art = wireframe.render(q, wide, tall, zoom=view['zoom'], colour=True,
                           horizon=False, tip=0.0,
                           solid=override, crew=view.get('crew'))
    cost = 1000.0 * (time.perf_counter() - began)

    body = Layout()
    body.split_row(Layout(viewport('RENDER', art, rate, 'RENDER'), name='view'),
                   Layout(engine(view, len(solid[1]) // 3, cost) if bindings.UX['column'] else rail(), name='hud',
                          size=column()))

    whole = Layout()
    bar = footer(bar_of(PAD, time.monotonic())[0])
    whole.split_column(
        Layout(band_of('RENDER', 'ENGINE TESTBED'), size=1),
        Layout(body, name='body'),
        Layout(bar, size=bar.row_count))
    return whole


AXES3 = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def _zoomed(view, key, _now):
    """+ -: nearer, farther, a wheel's step."""
    view['zoom'] = max(0.3, min(4.0, view['zoom'] * (1.0 + (WHEEL_STEP if key in '+=' else -WHEEL_STEP))))


def _lit(_view, key, _now):
    """UP DOWN: the light's dusk lower, higher."""
    shading.DUSK = min(0.60, max(-0.30, shading.DUSK + (-0.03 if key == 'up' else 0.03)))


def _spot(_view, key, _now):
    """LEFT RIGHT: the spot less, more."""
    shading.SPOT = min(0.80, max(0.0, shading.SPOT + (0.03 if key == 'right' else -0.03)))


def _turned(view, key, _now):
    """x y z: STEP deg about the axis, X Y Z back."""
    turn(view, AXES3['xyz'.index(key.lower())], STEP if key.islower() else -STEP)


#: The page's keys, rows of the terminal's one table (`terminal.ui.bindings`); F lends the mouse
#: to the view, the console's own (`console.SELECT_KEYS`).
PAD = bindings.page((
    ('VIEW', 'F', "the mouse the view's, or the terminal's", (('F', 'MOUSE'),), {}),
    ('VIEW', 'WHEEL / + -', 'nearer, farther', (('+ -', 'ZOOM'),), dict.fromkeys('+=-_', _zoomed)),
    ('LIGHT', 'UP DOWN', "the light's dusk lower, higher", (('UP DN', 'LIGHT'),), dict.fromkeys(('up', 'down'), _lit)),
    ('LIGHT', 'LEFT RIGHT', 'the spot less, more', (('LT RT', 'SPOT'),), dict.fromkeys(('left', 'right'), _spot)),
    ('POSE', 'x/X y/Y z/Z', '%.0f deg about an axis, each way' % STEP, (('x/X y/Y z/Z', 'DEG'),), dict.fromkeys('xyzXYZ', _turned)),
    ('POSE', 'SPACE', 'the idle tumble, or still', (('SPACE', 'SPIN'),), {' ': bindings.did(lambda v: v.update(spin=not v['spin']))}),
    ('POSE', 'R', 'the pose square again', (('R', 'RESET'),),
     bindings.cased('r', bindings.did(lambda v: v.update(pose=(0.0, 0.0, 0.0, 1.0))))),
    ('POSE', 'M', 'the cube, or the board', (('M', 'MODEL'),),
     bindings.cased('m', bindings.did(lambda v: v.update(model='board' if v['model'] == 'cube' else 'cube')))),
))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--model', choices=('cube', 'board'),
                        default='cube')
    parser.add_argument('--frames', type=int, default=0,
                        help='draw this many and exit 0 - the smoke test')
    parser.add_argument('--simulated', action='store_true',
                        help='accepted for the view suite; nothing here '
                             'opens a session either way')
    parser.add_argument('--workers', type=int, default=0,
                        help='raster the board as row bands in this many '
                             'processes (0: the GPU where a card answers, else '
                             'this one). Measured at 220x60: 14.8 ms to 5.8 '
                             'with eight')
    args = parser.parse_args(argv)

    page = stage()
    # Spin is off at start; SPACE starts the idle tumble.
    view = {'model': args.model, 'pose': (0.0, 0.0, 0.0, 1.0),
            'spin': False, 'zoom': 1.0,
            'carry': (0.0, 0.0)}
    if args.workers > 0:
        view['crew'] = Crew(wireframe._lods(), art=shading._face(),
                            workers=args.workers)
    elif (found := card.adapter()) is not None:
        view['crew'] = gpu.GpuCrew(wireframe._lods(), art=shading._face(), found=found)
    frame, last = 0, time.monotonic()

    try:
        return loop(args, page, view, frame, last)
    finally:
        if view.get('crew'):
            view['crew'].close()


def loop(args, page, view, frame, last):
    with curtain(page) as live, Keys(page.is_terminal, mouse=True, quits=frozenset(), menus=frozenset()) as keys:
        while True:
            now = time.monotonic()
            if view['spin']:
                step = SPIN_DPS * (now - last)
                turn(view, (0.4, 0.3, 1.0), step)
            last = now

            frame += 1
            live.update(compose(view, page.size, rate_of(page).label()),
                        refresh=True)
            rate_of(page).tick(now)
            if args.frames and frame >= args.frames:
                return 0

            leave, moved, typed = paced(keys, 0.06)
            leave = leave or bindings.act_on(PAD, typed, view)[0]
            if leave:
                return answer(leave)
            if moved:
                view['zoom'] = max(0.3, min(4.0,
                                            view['zoom'] * (1.0 + moved)))
            view['reports'] = keys.reports
            dx, dy = keys.dragged()
            if dx or dy:
                view['spin'] = False
            carry(view, dx, dy)


def carry(view, dx, dy):
    """A drag into the pose, eased: half the carry each frame, and rows are
    half-height cells, so the vertical gesture doubles."""
    cx, cy = view['carry']
    cx, cy = cx + dx, cy + dy
    if abs(cx) > 0.01 or abs(cy) > 0.01:
        ax, ay = cx * 0.5, cy * 0.5
        turn(view, (0, 1, 0), ax * DRAG_DEG)
        turn(view, (1, 0, 0), ay * DRAG_DEG * 2.0)
        cx, cy = cx - ax, cy - ay
    view['carry'] = (cx, cy)


if __name__ == '__main__':
    sys.exit(main())
