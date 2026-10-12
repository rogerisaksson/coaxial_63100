"""The quad's camera's picture: its world from the gimbal's lens, dots over its sky and ground.

    body = onboard.picture(eye, aim, gate, line_)        # for `inset.docked`

What the page's own view draws (`scenery`: the ground's grid, what stands, the course's gates and
its line) from the lens at `eye`, looking where the gimbal's `aim` (pan, tilt, roll) has it: the
horizon a row, level or leant as the sight is rolled. An edge fades into its cell's air as it
goes off, not into black; a grid's lines end where they would close to a band (APART).
"""
import functools
import math

from coaxial.graphics import inset, scenery, shapes
from coaxial.graphics.engine import fine
from coaxial.graphics.lit import FLOOR_INK, project
from coaxial.graphics.raster import DOTS_X, DOTS_Y

#: The picture, cells; the lens' field across it, degrees; how far off an edge is drawn in its
#: full ink, m - `scenery.FAR_SIZE` of it four times as far, and no further.
WIDE, HIGH, FIELD_DEG, FULL_M = 36, 12, 84.0, 12.0

#: A row's air: the sky from the horizon to overhead, the ground from the horizon to underfoot.
SKY, GROUND = ((16, 40, 54), (4, 10, 20)), ((10, 26, 30), (2, 6, 8))

#: A grid's lines are drawn as far as they stand this many dots apart, seen from the lens'
#: height: further they close to a band under the horizon.
APART = 3.0


@functools.lru_cache(maxsize=None)
def lens():
    """The lens as the engine's camera, at the dots' resolution: FIELD_DEG across the picture,
    its framed point FULL_M along the sight."""
    return fine({'scale': WIDE / (2.0 * math.tan(math.radians(FIELD_DEG) / 2.0)),
                 'cx': WIDE / 2.0, 'cy': HIGH / 2.0, 'distance': FULL_M, 'width': WIDE,
                 'height': HIGH, 'reach': FULL_M})


def air(tilt, roll=0.0):
    """(HIGH, WIDE, 3): each cell's ground with the sight `tilt` rad over the horizon and
    rolled `roll` - the sky, darker overhead, over the ground, darker underfoot; a cell the
    horizon crosses both, by their shares of it. The horizon is where the lens has the far
    ground either side of its sight."""
    from coaxial.model.blocks import numpy as np
    m, centre = sight((0.0, 0.0, 0.0), (0.0, tilt, roll))
    sx, sy, _w = project([(1e4 * math.sin(a), 0.0, 1e4 * math.cos(a)) for a in (-0.3, 0.3)],
                         m, lens(), centre)
    cols = (np.arange(WIDE) + 0.5) * DOTS_X
    horizon = (sy[0] + (sy[1] - sy[0]) / (sx[1] - sx[0]) * (cols - sx[0]))[None, :]
    top = (np.arange(HIGH) * float(DOTS_Y))[:, None]
    over = np.clip((horizon - top) / DOTS_Y, 0.0, 1.0)[..., None]
    off = (top + DOTS_Y / 2.0 - horizon) / (HIGH * DOTS_Y)
    tones = [np.asarray(near, float)
             + (np.asarray(far, float) - near) * np.clip(way, 0.0, 1.0)[..., None]
             for (near, far), way in ((SKY, -off), (GROUND, off))]
    return tones[0] * over + tones[1] * (1.0 - over)


def sight(eye, aim):
    """(m, centre): the engine's view of the lens at `eye`, m, looking as `aim` has it (pan, rad
    about up from z toward x; tilt, rad over the horizon; roll, rad, leant to the side a
    growing pan looks to - none where `aim` is two), and the point `lens` frames."""
    from coaxial.model.blocks import numpy as np
    level = np.asarray(shapes.view(math.degrees(aim[0]) + 180.0,
                                   -math.degrees(aim[1]))).reshape(3, 3)
    c, s = (math.cos(aim[2]), math.sin(aim[2])) if len(aim) > 2 else (1.0, 0.0)
    m = np.vstack([c * level[0] - s * level[1], s * level[0] + c * level[1], level[2]])
    return tuple(m.ravel().tolist()), np.asarray(eye, float) - lens()['distance'] * level[2]


def picture(eye, aim, gate=None, line_=False):
    """The picture's body (`inset.dotted`) from the lens at `eye` looking as `aim` has it
    (`sight`): the ground's grid and what stands, with the course's gates where `gate` is the
    one flown to next and its line with `line_`."""
    from coaxial.model.blocks import numpy as np
    (m, centre), cam = sight(eye, aim), lens()
    ground = air(aim[1], aim[2] if len(aim) > 2 else 0.0)
    (floor, pitch), (ends, inks) = scenery.grid(), scenery.edges(gate, line_)
    palette = np.asarray(list(scenery.INKS.values()) + [FLOOR_INK], float)
    reach = np.concatenate([np.sqrt(cam['scale'] * max(0.0, float(eye[1])) * pitch / APART),
                            np.full(len(inks), np.inf)])
    inks = np.concatenate([np.full(len(pitch), len(palette) - 1), inks])
    ys, xs, which, share = scenery.drawn(np.vstack([floor, ends]), m, cam, centre, reach)
    dots = np.zeros((HIGH * DOTS_Y, WIDE * DOTS_X), bool)
    dots[ys, xs] = True
    rows, cols = ys // DOTS_Y, xs // DOTS_X
    ink = ground.copy()
    ink[rows, cols] = ground[rows, cols] + (
        palette[inks[which]] - ground[rows, cols]) * share[:, None]
    return inset.dotted(dots, ink, ground)
