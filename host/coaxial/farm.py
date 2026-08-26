"""The renderer, cut into horizontal bands and drawn by several processes.

Pure Python holds the GIL, so threads buy nothing here - measured on this
machine, `sys._is_gil_enabled()` is True and the build is not free-threaded.
Processes it is.

Bands rather than triangle ranges, because a band owns its slice of the
z-buffer outright: nothing has to be merged afterwards, and each worker
returns the finished characters for its own rows. Splitting by triangle
would mean shipping four framebuffers back and comparing them depth by
depth in the parent, which is most of what was saved.

What is duplicated is the vertex pass, since every band needs every vertex.
Measured at 150x44: 8.0 ms of a 139 ms frame, so 94% of the work divides.
"""
import multiprocessing
import os

from . import ascii3d

#: The model, set once per worker. Sending 200,000 floats down a pipe every
#: frame would cost more than the drawing.
_MODEL = None

#: More than this many workers stops helping: the bands get thinner than the
#: model is tall, so most of them draw nothing and the vertex pass - which
#: every worker repeats - is all that is left.
MAX_WORKERS = 16


def _load(model):
    global _MODEL
    _MODEL = model


def _band(job):
    """One strip, from its own z-buffer to its own characters."""
    (matrix, distance, scale, cx, cy, cols, top, bottom, lamp, cull,
     width, cell_rows, supersample, ramp, invert) = job

    depth, value = ascii3d.rasterise(_MODEL, matrix, distance, scale, cx, cy,
                                     cols, top, bottom, lamp, cull)
    return ascii3d.resolve(depth, value, width, (bottom - top) // cell_rows,
                           cols, cell_rows, supersample, ramp, invert)


class Farm:

    """A pool of workers holding the model, ready to draw bands of it.

    Built once and reused: on Windows a process starts by spawning a fresh
    interpreter and importing everything again, which costs about a second.
    Per frame after that, all that crosses the pipe is a rotation matrix and
    a few lines of text back.
    """

    def __init__(self, model, workers=None):
        if workers is None:
            workers = min(MAX_WORKERS, os.cpu_count() or 1)
        self.workers = max(1, workers)
        self.pool = multiprocessing.Pool(self.workers, _load, (model,))
        self.model = model

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

    def render(self, matrix, width, height, distance=None,
               ramp=ascii3d.CHARACTERS, invert=True,
               supersample=ascii3d.SUPERSAMPLE, zoom=1.0, centre=None,
               light=None, aspect=ascii3d.CELL_ASPECT, cull=ascii3d.CULLING):
        """The same picture `ascii3d.render` draws, in parallel."""
        cols, rows, cell_rows, distance, scale, cx, cy = ascii3d._setup(
            self.model, matrix, width, height, distance, zoom, supersample,
            aspect, centre)
        lamp = light if light else ascii3d.light_position()

        jobs = []
        for top, bottom in _split(height, cell_rows, self.workers):
            jobs.append((matrix, distance, scale, cx, cy, cols, top, bottom,
                         lamp, cull, width, cell_rows, supersample, ramp,
                         invert))

        return '\n'.join(self.pool.map(_band, jobs))


def _split(height, cell_rows, workers):
    """(top, bottom) framebuffer rows per worker, on character boundaries.

    A band has to end where a character row ends, or the cell it cuts in half
    is averaged from two workers that never see each other's pixels.
    """
    bands = []
    at = 0

    for index in range(workers):
        # Spread the remainder rather than giving it all to the last band:
        # one band a dozen rows taller than the rest is a dozen rows every
        # other worker waits for.
        take = height // workers + (1 if index < height % workers else 0)
        if not take:
            break
        bands.append((at * cell_rows, (at + take) * cell_rows))
        at += take

    return bands
