#!/usr/bin/env python3
"""A terminal page's last frame, simulated, to text and a PNG - no window.

    python tools/render/page.py desk --png desk.png
    python tools/render/page.py rotor_observer --frames 40 --size 150 44
    python tools/render/page.py menu
    python tools/render/page.py desk --port emulator://      # the image on Renode
    python tools/render/page.py atlas --feed 'CHUNK|CHUNK'   # what a terminal sends, a chunk a poll
    python tools/render/page.py humanoid --live --frames 300   # its drawing never waited for, as on a terminal

The page runs its own `main` with `--simulated --frames N` - or `--port PORT` - on a
console of the given size, as a terminal would size it; the last frame it drew is printed to text and,
with `--png`, drawn by `machine.rig.ansi.png`. PAGES names each page's module.
"""
import argparse
import importlib
import inspect
import io
import os
import shutil
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from rich.console import Console
from rich.live import Live

from machine.rig import ansi
from terminal.ui import console as keys_module
from terminal.ui import screen as screen_module
from terminal.ui import stage as stage_module

PAGES = {
    'menu': 'terminal.menu',
    'aerobat': 'terminal.views.show_aerobat',
    'angle': 'terminal.views.show_angle',
    'atlas': 'terminal.views.show_atlas',
    'capture': 'terminal.views.show_capture',
    'chat': 'terminal.views.show_chat',
    'desk': 'terminal.views.show_desk',
    'gate_drivers': 'terminal.views.show_gate_drivers',
    'helicopter': 'terminal.views.show_helicopter',
    'humanoid': 'terminal.views.show_humanoid',
    'orientation': 'terminal.views.show_orientation',
    'ornithopter': 'terminal.views.show_ornithopter',
    'quad': 'terminal.views.show_quad',
    'render': 'terminal.views.show_render',
    'rotor_observer': 'terminal.views.show_rotor_observer',
    'session': 'terminal.views.show_session',
    'thermal_observer': 'terminal.views.show_thermal_observer',
}


class _Still(keys_module.Keys):
    """Keys on no console: nothing read, the page draws its frames and ends."""

    def __init__(self, _console, *args, **kwargs):
        super().__init__(False, *args, **kwargs)


class _Fed(keys_module.Keys):
    """Keys fed what a terminal would send (`FED`, a chunk a poll): its mouse reports and keys through the
    console's own decoder, the mouse held where the page asks it; no console read or set."""
    FED = []

    def __init__(self, _console, *args, **kwargs):
        super().__init__(True, *args, **kwargs)

    def __enter__(self):
        if self.mouse:
            keys_module.Keys.holder, self._grabbed = self, not self._select
        return self

    def __exit__(self, *exc_info):
        if keys_module.Keys.holder is self:
            keys_module.Keys.holder = None
        return False

    def grab(self, on):
        self._grabbed = bool(self.mouse and on)
        return self._grabbed

    def _hold(self):
        self._held_at = time.monotonic()

    def _drain(self):
        return [_Fed.FED.pop(0)] if _Fed.FED else []


class _Nowhere(io.StringIO):
    """A console's file that keeps nothing: the page's frames drawn and dropped."""

    def write(self, text):
        return len(text)


def frame(name, width=150, height=44, frames=12, port=None, keys='', fed=None, live=False, timed=None, extra=()):
    """The page's last frame as ANSI text: simulated, or on `port`; `keys` typed a frame each
    where the page takes `--keys`; `fed` what a terminal would send, a chunk a poll, through the
    console's own decoder (mouse reports among it); `live` its drawing never waited for, as on a terminal
    (HUMANOID's `--live`); `timed` a list each frame's (its update's start, rich's update, the page's own draw before
    it) is appended to, s - the draw timed by the `draw` the page hands its `run_view`.
    Only the last is kept: every frame and the console's whole
    output grew 0.5 MB a frame, and two six-minute pages ran this laptop's commit out."""
    drawn = []

    def sized():
        return Console(file=_Nowhere(), force_terminal=True, width=width, height=height,
                       color_system='truecolor', theme=stage_module.THEME, highlight=False,
                       legacy_windows=False)

    update, tubed, view, drew = Live.update, stage_module._Tube.update, screen_module.run_view, [0.0]

    def kept(self, renderable, *, refresh=False):
        drawn[:] = [renderable]
        return update(self, renderable, refresh=refresh)

    def shown(self, renderable, *, refresh=False):
        began = time.perf_counter()
        try:
            return tubed(self, renderable, refresh=refresh)
        finally:
            if timed is not None:
                timed.append((began, time.perf_counter() - began, drew[0]))

    def viewed(board_view, console, period, frames_, draw, *args, **kwargs):
        def drawing():
            began = time.perf_counter()
            try:
                return draw()
            finally:
                drew[0] = time.perf_counter() - began
        return view(board_view, console, period, frames_, drawing, *args, **kwargs)

    # A page that sizes itself off the terminal (the thermal map) is told this size too.
    told = os.terminal_size((width, height))
    was = (stage_module.stage, keys_module.Keys, screen_module.Keys, Live.update, shutil.get_terminal_size,
           os.get_terminal_size)
    stand_in, _Fed.FED = (_Fed, list(fed)) if fed else (_Still, [])
    stage_module.stage, keys_module.Keys, screen_module.Keys, Live.update = sized, stand_in, stand_in, kept
    if timed is not None:
        stage_module._Tube.update = shown
    shutil.get_terminal_size = lambda *_args, **_kwargs: told
    os.get_terminal_size = lambda *_args, **_kwargs: told
    module = None
    try:
        module = importlib.import_module(PAGES[name])
        setattr(module, 'stage', sized)
        if hasattr(module, 'Keys'):
            setattr(module, 'Keys', stand_in)
        if timed is not None and getattr(module, 'run_view', None) is view:
            setattr(module, 'run_view', viewed)
        argv = ((['--port', port] if port else ['--simulated']) + ['--frames', str(frames)]
                + (['--keys', keys] if keys else []) + (['--live'] if live else []) + list(extra))
        if inspect.signature(module.main).parameters:
            module.main(argv)
        else:
            saved, sys.argv = sys.argv, [name] + argv
            try:
                module.main()
            finally:
                sys.argv = saved
    finally:
        (stage_module.stage, keys_module.Keys, screen_module.Keys, Live.update, shutil.get_terminal_size,
         os.get_terminal_size) = was
        stage_module._Tube.update = tubed
        if getattr(module, 'run_view', None) is viewed:
            setattr(module, 'run_view', view)
    if not drawn:
        raise RuntimeError('%s drew no frame' % name)
    out = Console(file=io.StringIO(), record=True, force_terminal=True, width=width,
                  height=height, color_system='truecolor', theme=stage_module.THEME,
                  highlight=False, legacy_windows=False)
    out.print(drawn[-1])
    return out.export_text(styles=True)


def paced(timed):
    """A line of `timed`'s (`frame`'s) hot frames, its first third left out: the frame's period, the page's own
    draw and rich's update of it at their medians and 90th percentiles, ms - its wait for its beat the rest."""
    hot = timed[len(timed) // 3:]
    period = sorted(b[0] - a[0] for a, b in zip(hot, hot[1:]))
    draw, update = sorted(d for _t, _u, d in hot), sorted(u for _t, u, _d in hot)
    at = lambda xs, q: 1e3 * xs[min(len(xs) - 1, int(q * len(xs)))] if xs else 0.0
    return ('timed %d hot frames: period %.1f ms p50 %.1f p90 (%.1f fps), draw %.1f p50 %.1f p90, rich update '
            '%.1f p50 %.1f p90' % (len(hot), at(period, 0.5), at(period, 0.9), 1e3 / max(1e-9, at(period, 0.5)),
                                   at(draw, 0.5), at(draw, 0.9), at(update, 0.5), at(update, 0.9)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('page', choices=sorted(PAGES))
    parser.add_argument('--size', type=int, nargs=2, default=(150, 44),
                        metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--frames', type=int, default=12)
    parser.add_argument('--png', help='also the picture as a PNG here')
    parser.add_argument('--port', help='a port, not the stand-in (emulator:// for the image)')
    parser.add_argument('--keys', default='', help="typed a frame each, '.' none (HUMANOID)")
    parser.add_argument('--live', action='store_true', help='its drawing never waited for, as on a terminal (HUMANOID)')
    parser.add_argument('--timed', action='store_true',
                        help="its frames' pace, its own draw's and rich's share of each, its first third left out as "
                             "cold, said on stderr")
    parser.add_argument('--feed', default='', help="what a terminal would send, a chunk a poll, '|' between: "
                                                  'escapes as in Python, ESC [<0;60;20M a left press reported')
    parser.add_argument('--model', default='', help='model override for render page (cube, board, head, robot)')
    for flag in ('--yaw', '--pitch', '--zoom'):
        parser.add_argument(flag, type=float, default=None)
    parser.add_argument('--pan', type=float, nargs=2, default=None)
    parser.add_argument('--column', action='store_true', help='unfold the right-hand HUD column')
    args = parser.parse_args(argv)
    fed = [chunk.encode('latin-1', 'backslashreplace').decode('unicode_escape')
           for chunk in args.feed.split('|')] if args.feed else None
    timed = [] if args.timed else None
    extra = (['--model', args.model] if args.model else [])
    for k in ('yaw', 'pitch', 'zoom'):
        if getattr(args, k) is not None:
            extra += [f'--{k}', str(getattr(args, k))]
    if args.pan is not None:
        extra += ['--pan', str(args.pan[0]), str(args.pan[1])]
    from terminal.ui import bindings as _bindings
    was_col = _bindings.UX['column']
    if args.column:
        _bindings.UX['column'] = True
    try:
        art = frame(args.page, *args.size, frames=args.frames, port=args.port, keys=args.keys, fed=fed, live=args.live,
                    timed=timed, extra=extra)
    finally:
        _bindings.UX['column'] = was_col
    print(art)
    if timed:
        print(paced(timed), file=sys.stderr)
    if args.png:
        size = ansi.png(art, args.png)
        print('%s: %dx%d' % (args.png, size[0], size[1]), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
