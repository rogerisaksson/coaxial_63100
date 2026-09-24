#!/usr/bin/env python3
"""Side by side on the stage: the engine, the oracle, the reference.

Three panels per pose in the house theme - the engine's coloured
render, the analytic oracle's expectation, and the CAD exporter's own
ASCII where a fixture exists. What the numbers in render/render_demo.ps1
summarise, this shows.

    python tools/render/rendershow.py                          # cube, 45/45/45
    python tools/render/rendershow.py --model board --pose x30y0z0
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'tests'))

from rich import box                                       # noqa: E402
from rich.columns import Columns                           # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.text import Text                                 # noqa: E402

import test_render                                         # noqa: E402
from coaxial.graphics import engine, shading, wireframe             # noqa: E402
from terminal.ui.stage import stage  # noqa: E402
from tools.render import facecheck                                           # noqa: E402

WIDTH, HEIGHT = 64, 32
#: Columns past which the exporter's render is shown at half scale, so
#: the three panels stay side by side.
THUMBNAIL_AT = 70


def _export_panel(path):
    """The exporter's own render of this pose, when it left one."""
    with open(path, encoding='ascii') as f:
        rows = f.read().splitlines()
    wide = max((len(r) for r in rows), default=0)
    thumb = wide > THUMBNAIL_AT
    rows = [r[::2] for r in rows[::2]] if thumb else rows
    return Panel(Text('\n'.join(rows), style='label'),
                 title=' EXPORTER 1:2 ' if thumb else ' EXPORTER ',
                 title_align='left', box=box.ROUNDED,
                 border_style='frame.hud')


def parse_pose(text):
    got = re.match(r'x(-?\d+)y(-?\d+)z(-?\d+)$', text)
    if not got:
        raise SystemExit('a pose is written x45y45z45')
    x, y, z = (int(v) for v in got.groups())
    return x, y, z


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--model', choices=('cube', 'board'),
                        default='cube')
    parser.add_argument('--pose', default='x45y45z45')
    args = parser.parse_args(argv)
    rot = parse_pose(args.pose)

    fixture = '%s-%s.txt' % ('cube' if args.model == 'cube' else 'ascii',
                             args.pose)
    solid = facecheck.solid_for('cube-' if args.model == 'cube' else '')
    q = facecheck.euler(*rot, order='xyz')

    mine = wireframe.render(q, WIDTH, HEIGHT, colour=True, 
                            horizon=False, tip=0.0, solid=solid)

    if args.model == 'cube':
        got, half = test_render.cube()
        reach = sum(h * h for h in half) ** 0.5
        cam = engine.camera(WIDTH, HEIGHT, reach,
                            distance=test_render.DISTANCE)
        rows = test_render.oracle(engine.pose(*rot), cam, half,
                                  shading.PIVOT, shading.SLOPE,
                                  shading.FLOOR)
        want = '\n'.join(''.join(' .:'[c] for c in line).rstrip()
                         for line in rows)
    else:
        want = '(the oracle only knows the cube)'

    panels = [
        Panel(Text.from_ansi(mine), title=' ENGINE ', title_align='left',
              box=box.HEAVY, border_style='frame'),
        Panel(Text(want, style='label'), title=' ORACLE ',
              title_align='left', box=box.ROUNDED,
              border_style='frame.hud'),
    ]
    path = os.path.join(facecheck.RENDERS, fixture)
    if os.path.exists(path):
        panels.append(_export_panel(path))

    stage().print(Columns(panels, equal=False, expand=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
