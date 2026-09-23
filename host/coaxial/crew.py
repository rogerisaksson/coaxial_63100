"""The staged engine's frame, cut into row bands drawn by processes.

Pure Python holds the GIL, so threads buy nothing here - processes it
is, the same shape farm.py gives the three.js-style renderer. Each
worker holds the solids from its start - every level of detail the
view can ask for, since the zoom picks among them and a pool holding
one was skipped the moment the zoom moved, measured - and the face
art. Per frame only the rotation matrix, the camera constants, the
shadow map, a row range and which solid cross the pipe; a strip of
finished cells comes back: classes, tone levels, bare geometry, seeds
and coverage. A band owns its rows outright, so the strips
concatenate with nothing to merge. What every band repeats is the
vertex pass AND the setup of every triangle before its row reject -
measured on the threadripper 2026-09-23, 2.0 and 12.8 ms of an empty
band's 15.3, so the setup is the floor and more workers past eight buy
nothing (sixteen measured no faster).

    with Crew([solid16, solid24, solid32]) as crew:
        art = wireframe.render(q, width, height, crew=crew)
"""
import multiprocessing
import os

from . import engine
from .errors import RigError
from . import mesh

class _Worker:
    """The solids and the art, set once per worker by `_load`: 50,000
    floats down a pipe every frame would cost more than the drawing."""
    bodies = ()
    art = None

#: Past this the bands get thinner than the model is tall: most workers
#: draw nothing and only the repeated vertex pass is left.
MAX_WORKERS = 8


def _load(solids, art):
    _Worker.bodies, _Worker.art = solids, art


def _band(job) -> tuple:
    """One strip: the dot-resolution raster, fold to cells, shade - all
    of a cell's work that needs no neighbour."""
    which, m, cam, beam, sun_min, band, shading = job
    first, last = band
    rows = last - first
    width = cam['width']
    fine = engine.fine(cam)
    depth, top, sun = engine.raster(_Worker.bodies[which], m, fine, beam=beam,
                                    sun_min=sun_min,
                                    band=(engine.DOTS_Y * first,
                                          engine.DOTS_Y * last))
    depth, top, sun, coverage, reached = engine.fold(depth, top, sun, width,
                                                     rows)
    if shading is None:
        return depth, top, sun, coverage, reached
    pivot, slope, floor, shadow, shadow_step, bias, art, planes = shading
    # shade() back-projects each cell from its row: hand it a camera
    # whose cy is shifted by the band's first row so row 0 of the strip
    # is row `first` of the frame.
    strip = dict(cam, height=rows, cy=cam['cy'] - first)
    n = width * rows
    levels, bare, seed = [0.0] * n, [0.0] * n, [0.0] * n
    classes = engine.shade(depth, top, sun, strip, m, pivot, slope, floor,
                           art=_Worker.art if art else None, shadow=shadow,
                           shadow_step=shadow_step, bias=bias,
                           levels=levels, bare=bare, seed=seed,
                           planes=planes)
    return depth, coverage, reached, classes, levels, bare, seed


def _decimate(job):
    path, divisions = job
    with open(path, 'rb') as f:
        raw = f.read()
    return divisions, mesh._clustered(mesh._centred(list(mesh._faces(raw))),
                                      divisions)


def decimate(path, grids, progress=None):
    """{divisions: solid} for every grid in `grids`, each decimated in
    its own process. Six levels took 5.0 s one after another, measured;
    in parallel they take the slowest one plus the spawn. `progress
    (done, total, divisions)` is called as each lands."""
    out = {}
    with multiprocessing.Pool(min(len(grids), os.cpu_count() or 1)) as pool:
        for i, (divisions, solid) in enumerate(pool.imap_unordered(
                _decimate, [(path, d) for d in grids])):
            out[divisions] = solid
            if progress:
                progress(i + 1, len(grids), divisions)
    return out


def split(height, workers):
    """(first, last) per band, the rows shared as evenly as they go."""
    workers = max(1, min(workers, height))
    step, extra = divmod(height, workers)
    bands, row = [], 0
    for i in range(workers):
        rows = step + (1 if i < extra else 0)
        bands.append((row, row + rows))
        row += rows
    return bands


def _serve(conn, solids, art):
    """One worker: the solids and the art held from the start, then a
    band a message until None."""
    _load(solids, art)
    while True:
        job = conn.recv()
        if job is None:
            break
        conn.send(_band(job))
    conn.close()


#: How long a worker gets to leave on its own before it is terminated.
CLOSE_S = 1.0


class Crew:
    """Workers holding some solids, each on its own pipe, ready to draw
    bands.

    Built once and reused: on Windows a process starts by importing
    everything again, about a second each; per frame after that a crew
    costs the pipe and nothing else. ITS OWN PIPES, NOT A POOL
    (2026-09-23): multiprocessing.Pool sends every job from a handler
    thread that needs the GIL, and a parent busy painting hands it over
    every 5 ms - measured on the threadripper, the eight jobs left the
    parent over ~40 ms and nothing could overlap the wait. Sent from the
    calling thread they are in the workers in 1.5 ms. `submit` and
    `collect` are the two halves of a frame, so a caller can hand the
    crew one pose and paint another while it works
    (wireframe._face_ahead); `frame` and `raster` are the two together.
    """

    def __init__(self, solids, art=None, workers=None):
        if workers is None:
            workers = min(MAX_WORKERS, os.cpu_count() or 1)
        self.workers = max(1, workers)
        self.solids = tuple(solids)
        context = multiprocessing.get_context('spawn')
        self.conns, self.procs = [], []
        for _ in range(self.workers):
            ours, theirs = context.Pipe()
            proc = context.Process(target=_serve, args=(theirs, self.solids, art),
                                   daemon=True)
            proc.start()
            theirs.close()
            self.conns.append(ours)
            self.procs.append(proc)
        #: Bands sent and not yet received - one frame at most.
        self.pending = 0

    def holds(self, solid):
        """Whether `solid` - by identity - is one the workers were given."""
        return any(held is solid for held in self.solids)

    def close(self):
        if self.conns is None:
            return
        for conn in self.conns:
            try:
                conn.send(None)
            except (OSError, ValueError, EOFError):   # a worker already gone
                pass
        for proc in self.procs:
            proc.join(timeout=CLOSE_S)
            if proc.is_alive():
                proc.terminate()
        for conn in self.conns:
            conn.close()
        self.conns = self.procs = None

    def _live(self):
        """The pipes, while there are any: a closed crew renders nothing."""
        if self.conns is None:
            raise RigError('the crew is closed')
        return self.conns

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
        return False

    def _which(self, solid):
        return next(i for i, held in enumerate(self.solids) if held is solid)

    def submit(self, solid, m, cam, beam, sun_min, shading):
        """This frame's bands to the workers, one each. One frame in
        flight at a time: a second submit before `collect` is refused,
        because the pipes would then hold two frames' results in order
        and a caller could not tell which it painted."""
        if self.pending:
            raise RigError('a frame is in flight; collect it first')
        conns = self._live()
        bands = split(cam['height'], self.workers)
        which = self._which(solid)
        for conn, band in zip(conns, bands):
            conn.send((which, m, cam, beam, sun_min, band, shading))
        self.pending = len(bands)

    def collect(self):
        """The frame `submit` sent, waited for and concatenated in band
        order: (depth, top, sun, coverage, reached) for a raster, (depth,
        coverage, reached, classes, levels, bare, seed) for a shaded
        frame."""
        if not self.pending:
            raise RigError('nothing is in flight')
        conns = self._live()
        strips = [conn.recv() for conn in conns[:self.pending]]
        self.pending = 0
        whole = [[] if isinstance(field, list) else bytearray()
                 for field in strips[0]]
        for strip in strips:
            for field, part in zip(whole, strip):
                field += part
        return tuple(whole)

    def raster(self, solid, m, cam, beam=None, sun_min=0.0):
        """(depth, top, sun, coverage, reached) at cell resolution for the
        whole frame: the dot raster and fold, as bands, concatenated in
        order."""
        self.submit(solid, m, cam, beam, sun_min, None)
        return self.collect()

    def frame(self, solid, m, cam, beam, sun_min, shading):
        """(depth, coverage, reached, classes, levels, bare, seed) for the
        whole frame, each band rastered, folded AND shaded by its worker.
        `shading` = (pivot, slope, floor, shadow, shadow_step, bias,
        art, planes) - `art` a flag: the worker holds the face itself;
        `planes` the slab's (top, bottom) z the art is read on."""
        self.submit(solid, m, cam, beam, sun_min, shading)
        return self.collect()
