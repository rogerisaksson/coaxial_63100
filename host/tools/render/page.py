#!/usr/bin/env python3
"""A terminal page's last frame, simulated, to text and a PNG - no window.

    python tools/render/page.py desk --png desk.png
    python tools/render/page.py rotor_observer --frames 40 --size 150 44
    python tools/render/page.py menu

The page runs its own `main` with `--simulated --frames N` on a console of the given
size, as a terminal would size it; the last frame it drew is printed to text and,
with `--png`, drawn by `machine.ansi.png`. PAGES names each page's module.
"""
import argparse
import importlib
import inspect
import io
import os
import shutil
import sys

from rich.console import Console
from rich.live import Live

from machine import ansi
from terminal.ui import console as keys_module
from terminal.ui import stage as stage_module

PAGES = {
    'menu': 'terminal.menu',
    'angle': 'terminal.views.show_angle',
    'capture': 'terminal.views.show_capture',
    'chat': 'terminal.views.show_chat',
    'desk': 'terminal.views.show_desk',
    'gate_drivers': 'terminal.views.show_gate_drivers',
    'orientation': 'terminal.views.show_orientation',
    'render': 'terminal.views.show_render',
    'rotor_observer': 'terminal.views.show_rotor_observer',
    'session': 'terminal.views.show_session',
    'thermal_observer': 'terminal.views.show_thermal_observer',
}


class _Still(keys_module.Keys):
    """Keys on no console: nothing read, the page draws its frames and ends."""

    def __init__(self, _console, *args, **kwargs):
        super().__init__(False, *args, **kwargs)


def frame(name, width=150, height=44, frames=12):
    """The page's last frame as ANSI text."""
    drawn = []

    def sized():
        return Console(file=io.StringIO(), force_terminal=True, width=width, height=height,
                       color_system='truecolor', theme=stage_module.THEME, highlight=False,
                       legacy_windows=False)

    update = Live.update

    def kept(self, renderable, *, refresh=False):
        drawn.append(renderable)
        return update(self, renderable, refresh=refresh)

    # A page that sizes itself off the terminal (the thermal map) is told this size too.
    told = os.terminal_size((width, height))
    was = (stage_module.stage, keys_module.Keys, Live.update, shutil.get_terminal_size,
           os.get_terminal_size)
    stage_module.stage, keys_module.Keys, Live.update = sized, _Still, kept
    shutil.get_terminal_size = lambda *_args, **_kwargs: told
    os.get_terminal_size = lambda *_args, **_kwargs: told
    try:
        module = importlib.import_module(PAGES[name])
        setattr(module, 'stage', sized)
        if hasattr(module, 'Keys'):
            setattr(module, 'Keys', _Still)
        argv = ['--simulated', '--frames', str(frames)]
        if inspect.signature(module.main).parameters:
            module.main(argv)
        else:
            saved, sys.argv = sys.argv, [name] + argv
            try:
                module.main()
            finally:
                sys.argv = saved
    finally:
        (stage_module.stage, keys_module.Keys, Live.update, shutil.get_terminal_size,
         os.get_terminal_size) = was
    if not drawn:
        raise RuntimeError('%s drew no frame' % name)
    out = Console(file=io.StringIO(), record=True, force_terminal=True, width=width,
                  height=height, color_system='truecolor', theme=stage_module.THEME,
                  highlight=False, legacy_windows=False)
    out.print(drawn[-1])
    return out.export_text(styles=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('page', choices=sorted(PAGES))
    parser.add_argument('--size', type=int, nargs=2, default=(150, 44),
                        metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--frames', type=int, default=12)
    parser.add_argument('--png', help='also the picture as a PNG here')
    args = parser.parse_args(argv)
    art = frame(args.page, *args.size, frames=args.frames)
    print(art)
    if args.png:
        size = ansi.png(art, args.png)
        print('%s: %dx%d' % (args.png, size[0], size[1]), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
