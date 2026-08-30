#!/usr/bin/env python3
"""Side by side on the stage: the engine, the oracle, the reference.

Three panels per pose in the house theme - the engine's coloured
render, the analytic oracle's expectation, and the CAD exporter's own
ASCII where a fixture exists. What the numbers in render/render_demo.ps1
summarise, this shows.

    python tools/rendershow.py                          # cube, 45/45/45
    python tools/rendershow.py --model board --pose x30y0z0
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'tests'))

from rich.columns import Columns                           # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from coaxial import engine, wireframe                      # noqa: E402
import facecheck                                           # noqa: E402
import test_render                                         # noqa: E402

WIDTH, HEIGHT = 64, 32


def parse_pose(text):
    got = re.match(r'x(-?\d+)y(-?\d+)z(-?\d+)$', text)
    if not got:
        raise SystemExit('pose skrivs som x45y45z45')
    return tuple(int(v) for v in got.groups())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
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
                                  wireframe.PIVOT, wireframe.SLOPE,
                                  wireframe.FLOOR)
        want = '\n'.join(''.join(' .:'[c] for c in line).rstrip()
                         for line in rows)
    else:
        want = '(oraklet kan bara kuben)'

    panels = [
        Panel(Text.from_ansi(mine), title=' MOTORN ', title_align='left',
              box=box.HEAVY, border_style='frame'),
        Panel(Text(want, style='label'), title=' ORAKLET ',
              title_align='left', box=box.ROUNDED,
              border_style='frame.hud'),
    ]
    path = os.path.join(facecheck.RENDERS, fixture)
    if os.path.exists(path):
        with open(path, encoding='ascii') as f:
            rows = f.read().splitlines()
        wide = max((len(r) for r in rows), default=0)
        title = ' EXPORTERN '
        if wide > 70:
            # The raw exports run past a terminal; a half-scale
            # thumbnail keeps the three panels side by side.
            rows = [r[::2] for r in rows[::2]]
            title = ' EXPORTERN 1:2 '
        panels.append(Panel(Text('\n'.join(rows), style='label'),
                            title=title, title_align='left',
                            box=box.ROUNDED, border_style='frame.hud'))

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from screen import stage                               # noqa: E402
    stage().print(Columns(panels, equal=False, expand=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
