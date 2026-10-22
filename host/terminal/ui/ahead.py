"""A page's 3D drawn a frame ahead in a process of its own, its raster its own.

The render - a pure function of what it is given - is asked for the next frame as this one begins
(at the next frame's instant on the beat, `screen.drawn_next`) and taken as that one begins: the
page's own work, its instruments and rich, on one core while the drawing takes another.

    ahead = Ahead('coaxial.graphics.gynoid', 'render')     # started; the page starts beside it
    ahead.name                                             # its raster's name, once it is up
    lines = ahead.drawn(*args, lit=True, **kwargs)         # the frame before's asked; this one's asked
    ahead.close()

A module with a `warm()` has it run in the worker before the worker says it is up: what its first
frame waits on, built while the page opens its board.
"""
import importlib
import os
import queue
import sys
import traceback
import types

from machine.apart import CONTEXT

#: The longest a page waits for a drawing, s: the worker's first loads its meshes and its raster.
WAIT_S = 60.0

#: A module asked as an argument goes by its name, (MODULE, name): a module does not pickle.
MODULE = 'module'


def _lit(held, found):
    """The worker's lit raster on `found`, made on the first ask for it; None without a card."""
    from coaxial.graphics import gpu
    if 'lit' not in held:
        held['lit'] = gpu.LitRaster(found=found) if found is not None else None
    return held['lit']


def serve(asks, drawn, module, function):
    """The worker: the render `module.function` on its own raster (`lit=True` asks for it), the
    module's `warm()` first where it has one, each ask drawn and said; its prints kept off the
    page's terminal, a fault said as its trace."""
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    os.dup2(null, 2)
    sys.stdout = sys.stderr = open(os.devnull, 'w', encoding='utf-8')
    try:
        from coaxial.graphics import card
        found, held = card.adapter(), {}
        drawer = importlib.import_module(module)
        render = getattr(drawer, function)
        getattr(drawer, 'warm', lambda: None)()
        drawn.put(('ready', found.info.get('device', 'GPU') if found is not None else None))
        while True:
            ask = asks.get()
            if ask is None:
                return
            args, kwargs = ask
            kwargs = {k: importlib.import_module(v[1]) if isinstance(v, tuple) and v[:1] == (MODULE,)
                      else v for k, v in kwargs.items()}
            if kwargs.get('lit') is True:
                kwargs['lit'] = _lit(held, found)
            drawn.put(('drawn', render(*args, **kwargs)))
    except Exception:
        drawn.put(('error', traceback.format_exc()))


class Ahead:

    """`module.function` drawing in a process of its own, a frame ahead of the page that asks."""

    def __init__(self, module, function):
        self._asks, self._drawn = CONTEXT.Queue(), CONTEXT.Queue()
        for each in (self._asks, self._drawn):
            each.cancel_join_thread()
        self._process = CONTEXT.Process(target=serve, daemon=True,
                                        args=(self._asks, self._drawn, module, function))
        self._process.start()
        self._up = None
        self._asked = False

    def _ready(self):
        """The card the worker draws on, or None: waited for once, its start and `warm()` done."""
        if self._up is None:
            self._up = (self._take(),)
        return self._up[0]

    @property
    def name(self):
        return self._ready()

    def _take(self):
        try:
            kind, value = self._drawn.get(timeout=WAIT_S)
        except queue.Empty:
            raise RuntimeError('the drawing said nothing in %.0f s' % WAIT_S) from None
        if kind == 'error':
            raise RuntimeError('the drawing: ' + value.strip().splitlines()[-1])
        return value

    def drawn(self, *args, **kwargs):
        """What the frame before asked to be drawn - this frame's, drawn while it was shown - and
        `args`, `kwargs` asked for the next; the first frame waits for its own."""
        kwargs = {k: (MODULE, v.__name__) if isinstance(v, types.ModuleType) else v
                  for k, v in kwargs.items()}
        self._ready()
        if not self._asked:
            self._asks.put((args, kwargs))
        before = self._take()
        self._asks.put((args, kwargs))
        self._asked = True
        return before

    def close(self, wait=1.0):
        try:
            self._asks.put_nowait(None)
        except queue.Full:
            pass
        self._process.join(timeout=wait)
        if self._process.is_alive():
            self._process.terminate()
