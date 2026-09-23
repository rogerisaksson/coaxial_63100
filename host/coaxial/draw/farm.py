"""The renderer, cut into horizontal bands and drawn by several processes."""
import multiprocessing
import os

from coaxial.draw import ascii3d
from coaxial.errors import RigError

class _Worker:
    """The model, set once per worker by `_load`: 200,000 floats down a pipe
    every frame would cost more than the drawing.
    """
    model = None

#: More than this many workers stops helping: the bands get thinner than the
#: model is tall, so most of them draw nothing and the vertex pass - which
#: every worker repeats - is all that is left.
MAX_WORKERS = 16


def _load(model):
    _Worker.model = model


def _band(job):
    """One strip, from its own z-buffer to its own characters."""
    (matrix, distance, scale, cx, cy, cols, top, bottom, lamp, cull,
     width, cell_rows, supersample, ramp, invert) = job

    depth, value = ascii3d.rasterise(_Worker.model, matrix, distance, scale, cx, cy,
                                     cols, top, bottom, lamp, cull)
    return ascii3d.resolve(depth, value, width, (bottom - top) // cell_rows,
                           cols, cell_rows, supersample, ramp, invert)


class Farm:

    """A pool of workers holding the model, ready to draw bands of it."""

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

    def _live(self):
        """The pool, while there is one: a closed farm renders nothing."""
        if self.pool is None:
            raise RigError('the farm is closed')
        return self.pool
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

        return '\n'.join(self._live().map(_band, jobs))


def _split(height, cell_rows, workers):
    """(top, bottom) framebuffer rows per worker, on character boundaries."""
    bands = []
    at = 0

    for index in range(workers):
        # Spread the remainder rather than giving it all to the last band: one
        # band a dozen rows taller than the rest is a dozen rows every other
        # worker waits for.
        take = height // workers + (1 if index < height % workers else 0)
        if not take:
            break
        bands.append((at * cell_rows, (at + take) * cell_rows))
        at += take

    return bands
