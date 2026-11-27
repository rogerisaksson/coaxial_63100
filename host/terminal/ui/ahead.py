"""A page's 3D drawn a frame ahead in a process of its own, its raster its own.

The render - a pure function of what it is given - is asked for the next frame as this one begins
(at the next frame's instant on the beat, `screen.drawn_next`) and taken as that one begins: the
page's own work, its instruments and rich, on one core while the drawing takes another.

    ahead = Ahead('coaxial.graphics.gynoid', 'render')     # started; the page starts beside it
    ahead.name                                             # its raster's name, once it is up
    lines = ahead.drawn(*args, lit=True, **kwargs)         # the frame before's asked; this one's asked
    ahead.waits = False                                    # a live page's: the newest drawn, never waited for
    Ahead(module, function, laid=True)                     # its ANSI lines read there (`marquee.laid`), not here
    ahead.behind                                           # the calls since the drawing returned was asked
    ahead.close()

A page that waits for each drawing (`waits`, a smoke's, its frames each of its own state) stands still while the
drawing takes longer than a frame: no key read, its first frame's cold render 3 s and more. A live page does not wait:
it shows the newest drawing done - none before the first - and asks the next once the one asked is drawn, its keys and
its instruments at the frame's pace whatever the drawing costs.

A drawing can be started before its page opens and kept for it (`keep`, `take`, `let_go`): the
terminal starts the pages' that say so (their AHEAD) as it opens and after a page closes.

A module with a `warm()` has it run in the worker before the worker says it is up: what its first
frame waits on, built while the page opens its board. The worker is a daemon and has no children:
nothing it draws or warms may start a pool or a process crew (`coaxial.draw.easel` decimates
itself).
"""
import importlib
import os
import queue
import sys
import threading
import traceback
import types

from machine.process import placed
from machine.process.apart import CONTEXT

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


def serve(asks, drawn, module, function, lay=False):
    """The worker: the render `module.function` on its own raster (`lit=True` asks for it), the
    module's `warm()` first where it has one, each ask drawn and said - its ANSI lines read into runs
    here where `lay` (`marquee.laid`: the page's own reading of them was a quarter of its work) -; its
    prints kept off the page's terminal, a fault said as its trace."""
    from terminal.ui import marquee
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    os.dup2(null, 2)
    sys.stdout = sys.stderr = open(os.devnull, 'w', encoding='utf-8')
    placed.placed()
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
            got = render(*args, **kwargs)
            drawn.put(('drawn', marquee.laid(got) if lay else got))
    except Exception:
        drawn.put(('error', traceback.format_exc()))


def _word(said):
    """What the worker said, (kind, value): its value, its fault raised as its trace's last line."""
    kind, value = said
    if kind == 'error':
        raise RuntimeError('the drawing: ' + value.strip().splitlines()[-1])
    return value


class Ahead:

    """`module.function` drawing in a process of its own, a frame ahead of the page that asks."""

    def __init__(self, module, function, waits=True, laid=False):
        self._asks, self._drawn = CONTEXT.Queue(), CONTEXT.Queue()
        for each in (self._asks, self._drawn):
            each.cancel_join_thread()
        self._process = CONTEXT.Process(target=serve, daemon=True,
                                        args=(self._asks, self._drawn, module, function, laid))
        self._process.start()
        self._up = None
        self._asked = False
        # Each drawing waited for, or the newest drawn (`drawn`); the calls since the one returned was asked; the
        # calls so far, the call whose ask is being drawn and the one whose drawing was returned; that drawing.
        self.waits, self.behind = waits, 0
        self._calls, self._at, self._of, self._shown = 0, 0, 0, None

    def alive(self):
        """Whether its worker runs."""
        return self._process.is_alive()

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
            said = self._drawn.get(timeout=WAIT_S)
        except queue.Empty:
            raise RuntimeError('the drawing said nothing in %.0f s' % WAIT_S) from None
        return _word(said)

    def up(self):
        """Whether the worker has said it is up, asked without waiting; its fault raised."""
        if self._up is None:
            try:
                said = self._drawn.get_nowait()
            except queue.Empty:
                return False
            self._up = (_word(said),)
        return True

    def drawn(self, *args, **kwargs):
        """What the frame before asked to be drawn - this frame's, drawn while it was shown - and
        `args`, `kwargs` asked for the next; the first frame waits for its own. Not `waits`: the newest
        drawing done, never waited for - None before the first -, `args` asked only once the one asked
        is drawn; `behind` the calls since the one returned was asked."""
        kwargs = {k: (MODULE, v.__name__) if isinstance(v, types.ModuleType) else v
                  for k, v in kwargs.items()}
        self._ready()
        self._calls += 1
        if self.waits:
            if not self._asked:
                self._asks.put((args, kwargs))
                self._at = self._calls
            self._shown, self._of = self._take(), self._at
        elif self._asked:
            try:
                said = self._drawn.get_nowait()
            except queue.Empty:
                self.behind = self._calls - self._of
                return self._shown
            self._shown, self._of = _word(said), self._at
        self._asks.put((args, kwargs))
        self._asked, self._at = True, self._calls
        self.behind = self._calls - self._of
        return self._shown

    def close(self, wait=1.0):
        try:
            self._asks.put_nowait(None)
        except queue.Full:
            pass
        self._process.join(timeout=wait)
        if self._process.is_alive():
            self._process.terminate()


#: Drawings started before the page that draws with them, by (module, function).
_KEPT: dict = {}
_KEPT_LOCK = threading.Lock()


def keep(module, function):
    """A drawing of `module.function` started now and kept for the page that takes it; one a
    drawing, the one kept left running."""
    with _KEPT_LOCK:
        kept = _KEPT.get((module, function))
        if kept is None or not kept.alive():
            _KEPT[(module, function)] = Ahead(module, function)


def take(module, function):
    """The kept drawing of `module.function` where it runs, else one started now."""
    with _KEPT_LOCK:
        kept = _KEPT.pop((module, function), None)
    if kept is not None and kept.alive():
        return kept
    if kept is not None:
        kept.close()
    return Ahead(module, function)


def let_go():
    """Every kept drawing closed: the terminal on its way out."""
    let_go_except(None)


def let_go_except(keep_pair=None):
    """Close every kept drawing except `keep_pair` `(module, function)` so idle workers release GPUs."""
    with _KEPT_LOCK:
        drop = [v for k, v in list(_KEPT.items()) if k != keep_pair and _KEPT.pop(k, None) is not None]
    for each in drop:
        each.close()
