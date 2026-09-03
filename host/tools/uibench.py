#!/usr/bin/env python3
"""Frame-cost benchmark for the stage: what each surface costs to draw.

Two numbers per surface, measured separately because they bound different
things: COMPOSE is building the renderable (data formatting, the 3D
raster), PRINT is rich rendering it to a styled 110x40 terminal buffer.
Their sum is the frame floor; the achievable rate is 1/sum.

    python tools/uibench.py            # everything
    python tools/uibench.py menu       # one surface
"""
import io
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.console import Console                           # noqa: E402

from screen import THEME                                   # noqa: E402


def timed(fn, seconds=1.0):
    """Mean milliseconds per call over `seconds` of calling it."""
    fn()                                     # warm caches out of the timing
    began = time.perf_counter()
    calls = 0
    while time.perf_counter() - began < seconds:
        fn()
        calls += 1
    return 1000.0 * (time.perf_counter() - began) / calls


def surface_menu():
    import menu
    return lambda: menu.compose('COM4', 1, time.perf_counter())


def surface_attitude():
    from coaxial import orientation
    q = (0.1, 0.2, 0.05, 0.97)
    return lambda: orientation.render(q, 58, 18, toon=True, colour=True)


def surface_protractor():
    from coaxial import dial
    return lambda: dial.render(215.0, 64, 23, field=380, colour=True)


def surface_thermalmap():
    from coaxial import thermalmap
    hot = dict((n, 40.0) for n in
               ('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v',
                'phase_w', 'mcu', 'regulators', 'afe'))
    return lambda: thermalmap.render(hot, board_c=30.0, colour=True,
                                     reserve=12, margin=38)


def surface_wireframe():
    from coaxial import wireframe
    q = (0.1, 0.2, 0.05, 0.97)
    return lambda: wireframe.render(q, 62, 22)


SURFACES = {
    'menu': surface_menu,
    'wireframe': surface_wireframe,
    'attitude': surface_attitude,
    'protractor': surface_protractor,
    'thermalmap': surface_thermalmap,
}


def main(argv=None):
    wanted = [a for a in (argv or sys.argv[1:]) if not a.startswith('-')]
    court = Console(theme=THEME, width=110, height=40, file=io.StringIO(),
                    force_terminal=True, highlight=False)

    print('%-12s %9s %9s %9s %7s' % ('surface', 'compose', 'print',
                                     'frame', 'fps'))
    for name, build in SURFACES.items():
        if wanted and name not in wanted:
            continue
        fn = build()
        compose = timed(fn)
        made = fn()
        spend = timed(lambda: court.print(made))
        frame = compose + spend
        print('%-12s %7.1f ms %7.1f ms %7.1f ms %7.0f'
              % (name, compose, spend, frame, 1000.0 / frame))
    return 0


if __name__ == '__main__':
    sys.exit(main())
