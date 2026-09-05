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
vertex pass.

    with Crew([solid16, solid24, solid32]) as crew:
        art = wireframe.render(q, width, height, crew=crew)
"""
import multiprocessing
import os

from . import engine

#: The solids and the art, set once per worker: 50,000 floats down a
#: pipe every frame would cost more than the drawing.
_BODIES = ()
_ART = None

#: Past this the bands get thinner than the model is tall: most workers
#: draw nothing and only the repeated vertex pass is left.
MAX_WORKERS = 8


def _load(solids, art):
    global _BODIES, _ART
    _BODIES = solids
    _ART = art


def _band(job):
    """One strip: 2x2 raster, fold to cells, shade - all of a cell's
    work that needs no neighbour."""
    which, m, cam, beam, sun_min, band, shading = job
    first, last = band
    rows = last - first
    width = cam['width']
    fine = dict(cam, width=2 * width, height=2 * cam['height'],
                scale=2.0 * cam['scale'], cx=2.0 * cam['cx'],
                cy=2.0 * cam['cy'])
    depth, top, sun = engine.raster(_BODIES[which], m, fine, beam=beam,
                                    sun_min=sun_min,
                                    band=(2 * first, 2 * last))
    depth, top, sun, coverage, quads = engine.fold(depth, top, sun, width,
                                                   rows)
    if shading is None:
        return depth, top, sun, coverage, quads
    pivot, slope, floor, shadow, shadow_step, bias, art = shading
    # shade() back-projects each cell from its row: hand it a camera
    # whose cy is shifted by the band's first row so row 0 of the strip
    # is row `first` of the frame.
    strip = dict(cam, height=rows, cy=cam['cy'] - first)
    n = width * rows
    levels, bare, seed = [0.0] * n, [0.0] * n, [0.0] * n
    classes = engine.shade(depth, top, sun, strip, m, pivot, slope, floor,
                           art=_ART if art else None, shadow=shadow,
                           shadow_step=shadow_step, bias=bias,
                           levels=levels, bare=bare, seed=seed)
    return depth, coverage, quads, classes, levels, bare, seed


def _decimate(job):
    path, divisions = job
    from . import mesh
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


class Crew:
    """A pool of workers holding some solids, ready to draw bands.

    Built once and reused: on Windows a process starts by importing
    everything again, about a second each; per frame after that the
    pool costs the pipe and nothing else."""

    def __init__(self, solids, art=None, workers=None):
        if workers is None:
            workers = min(MAX_WORKERS, os.cpu_count() or 1)
        self.workers = max(1, workers)
        self.solids = tuple(solids)
        self.pool = multiprocessing.Pool(self.workers, _load,
                                         (self.solids, art))

    def holds(self, solid):
        """Whether `solid` - by identity - is one the workers were given."""
        return any(held is solid for held in self.solids)

    def close(self):
        if self.pool is not None:
            self.pool.terminate()
            self.pool.join()
            self.pool = None

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
        return False

    def _which(self, solid):
        return next(i for i, held in enumerate(self.solids) if held is solid)

    def raster(self, solid, m, cam, beam=None, sun_min=0.0):
        """(depth, top, sun, coverage, quads) at cell resolution for the
        whole frame: 2x2 raster and fold, as bands, concatenated in
        order."""
        jobs = [(self._which(solid), m, cam, beam, sun_min, band, None)
                for band in split(cam['height'], self.workers)]
        depth, top, sun, coverage = [], bytearray(), bytearray(), []
        quads = bytearray()
        for d, t, s, c, q in self.pool.map(_band, jobs):
            depth += d
            top += t
            sun += s
            coverage += c
            quads += q
        return depth, top, sun, coverage, quads

    def frame(self, solid, m, cam, beam, sun_min, shading):
        """(depth, coverage, quads, classes, levels, bare, seed) for the
        whole frame, each band rastered, folded AND shaded by its worker.
        `shading` = (pivot, slope, floor, shadow, shadow_step, bias,
        art) - `art` a flag: the worker holds the face itself."""
        jobs = [(self._which(solid), m, cam, beam, sun_min, band, shading)
                for band in split(cam['height'], self.workers)]
        depth, coverage, quads, classes = [], [], bytearray(), bytearray()
        levels, bare, seed = [], [], []
        for d, c, q, k, lv, b, s in self.pool.map(_band, jobs):
            depth += d
            coverage += c
            quads += q
            classes += k
            levels += lv
            bare += b
            seed += s
        return depth, coverage, quads, classes, levels, bare, seed
