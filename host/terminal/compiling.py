"""The machines' control systems compiled behind the front page: its entries greyed till each is done.

The pages that lay a machine's control system say which (their COMPILED: the robot, a craft). The terminal starts one
compile for those behind their sources as its front page opens (`start`, `tools.sim.warmed` in a process of its own),
the robot first; each step it says is kept here (STATE) for the front page to show on the machine's entry, greyed and
not chosen till its machine is done (`busy`, `said`); the terminal on its way out ends it (`stop`).

    compiling.start(['robot', 'quad'])      # those behind compiled apart, told here as they go
    compiling.busy('robot')                 # whether its compile is under way
    compiling.said('robot')                 # 'ROBOT: STANDING: 12 OF 34 BUILT ON 6 CORES  0:41', None when done
    compiling.stop()                        # the compile ended, its machines left as they stand
"""
import threading
import time

#: {machine: [its compile's share, its last step's text, when the compile began (time.monotonic)]} while its compile
#: is under way; a machine not here is done or was never behind.
STATE: dict = {}
_LOCK = threading.Lock()
#: The compile's process while it runs.
_RUN: list = [None]
#: Shared C11 / 3D warmup targets per page name (`board`: `native://` C11 firmware/world,
#: `raster`: `raster.c` 3D engine & Braille C11, `mesh`: STL board decimates & shadowmap).
PAGE_WARMED = {
    'session': ('board',),
    'imu': ('board', 'raster', 'mesh'),
    'angle': ('board',),
    'adc': ('board',),
    'motor': ('board',),
    'thermal_observer': ('board', 'raster', 'mesh'),
    'humanoid': ('raster',),
    'quad': ('raster',),
    'helicopter': ('raster',),
    'ornithopter': ('raster',),
    'aerobat': ('raster',),
    'atlas': ('raster',),
}


def needs_of(page):
    """Tuple of compile/warmup target keys that `page` waits on while active in `STATE`."""
    own = getattr(page, 'COMPILED', None)
    extra = PAGE_WARMED.get(getattr(page, 'NAME', ''), ())
    return ((own,) if own is not None else ()) + tuple(t for t in extra if t != own)


def note(target, text, share=0.0):
    """Record or update an in-process C11 or mesh compile step for `target` in `STATE`."""
    now = time.monotonic()
    with _LOCK:
        if target in STATE:
            STATE[target][:2] = [share, text]
        else:
            STATE[target] = [share, text, now]


def done(target):
    """Clear `target` from `STATE` once its compile or preload finishes."""
    with _LOCK:
        STATE.pop(target, None)


def start(machines):
    """`machines` behind their sources compiled in a process of its own, the robot first, read from a thread into
    STATE; none behind, none started. The machines it compiles."""
    import atexit
    from tools.sim import warmed
    if _RUN[0] is not None:
        return []
    behind = warmed.behind(machines)
    if not behind:
        return []
    began = time.monotonic()
    with _LOCK:
        STATE.update({m: [0.0, 'WAITING', began] for m in behind})
    run = _RUN[0] = warmed.started(behind)
    # Windows ends no child with its parent: a terminal closed mid-compile left it building on.
    atexit.register(stop)
    threading.Thread(target=_read, args=(run, behind), name='compiling', daemon=True).start()
    return behind


def _read(run, behind):
    """`run`'s lines into STATE as they come, each machine let go at its end - its last line's, or the run's."""
    from tools.sim import warmed

    def step(machine, share, text):
        with _LOCK:
            for m in behind[:behind.index(machine)] if machine in behind else ():
                STATE.pop(m, None)
            if machine in STATE:
                STATE[machine][:2] = [share, text]
    def done(machine):
        with _LOCK:
            STATE.pop(machine, None)
    try:
        warmed.told(run, step, done)
    finally:
        with _LOCK:
            for m in behind:
                STATE.pop(m, None)
        _RUN[0] = None


def busy(machine):
    """Whether `machine`'s compile is under way."""
    with _LOCK:
        return machine in STATE


def said(machine, now=None):
    """What `machine`'s entry says while its compile is under way - its last step and the time the compile has
    stood -, None when it is done."""
    with _LOCK:
        got = STATE.get(machine)
    if got is None:
        return None
    _share, text, began = got
    seconds = int((time.monotonic() if now is None else now) - began)
    return '%s: %s  %d:%02d' % (machine.upper(), text, seconds // 60, seconds % 60)


def stop():
    """The compile ended where it runs: its machines left as they stand, compiled again when next behind."""
    run = _RUN[0]
    if run is not None and run.poll() is None:
        run.kill()
