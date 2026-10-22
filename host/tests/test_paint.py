"""The face painted at once (coaxial.graphics.paint, strokes): cell for cell, glyph and tone, what
the reference passes (`wireframe._paint`) paint one cell at a time."""
import math
import sys

from coaxial.draw import orientation
from coaxial.graphics import engine, paint, wireframe
from structure_kit import Report
from tools.dev.focus import chosen


def _reference(cells, cam, m, colour, persist, foreign):
    """`wireframe._paint` on a blank grid, as the layer `paint.painted` hands over."""
    width, height = cam['width'], cam['height']
    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]
    wireframe._paint(grid, tone, cells, cam, m, colour, persist, foreign)
    return [(r, c, grid[r][c], tone[r][c]) for r in range(height) for c in range(width)
            if grid[r][c] != ' ']


def _listed(layer, width):
    """`paint.painted`'s arrays as the reference's layer: (row, col, glyph, (r, g, b) or None)."""
    at, codes, inks = layer
    return [(a // width, a % width, chr(c), None if v < 0 else (v >> 16, (v >> 8) & 0xFF, v & 0xFF))
            for a, c, v in zip(at.tolist(), codes.tolist(), inks.tolist())]


def _turned(axis, degrees):
    """The quaternion (i, j, k, real) of `degrees` about `axis`."""
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(math.radians(degrees) / 2.0)
    return tuple(a / n * s for a in axis) + (math.cos(math.radians(degrees) / 2.0),)


#: A tumble about a tilted axis, a frame's exposure following the last, then face down, edge
#: on and level.
POSES = tuple(_turned((0.3, 0.8, 0.52), d) for d in range(0, 360, 45)) + (
    _turned((1.0, 0.0, 0.0), 180.0), _turned((0.0, 1.0, 0.0), 89.0), (0.0, 0.0, 0.0, 1.0))


def _camera(width, height, zoom):
    """The solid a frame this size earns and the render's camera on it."""
    _edges, solid = wireframe._model(zoom, wireframe._earned(width, height, zoom))
    pts = solid[0]
    reach = max(math.sqrt(pts[3 * i] ** 2 + pts[3 * i + 1] ** 2 + pts[3 * i + 2] ** 2)
                for i in range(len(pts) // 3))
    return solid, engine.camera(width, height, reach, distance=3.2, zoom=zoom,
                                tip=wireframe.CAMERA_TIP, lift=orientation.LIFT)


def test_the_face_is_painted_at_once(report):
    """Over a turning board at two sizes, colour and mono, and the board as a foreign solid's
    face alone: the layer and the exposure it leaves equal the reference's."""
    for width, height, zoom in ((60, 20, 1.0), (72, 26, 1.3)):
        solid, cam = _camera(width, height, zoom)
        frames = []
        for q in POSES:
            m = engine.multiply(cam['view'], orientation.matrix(q))
            frames.append((m, wireframe._cells(solid, m, cam, None, True, False)))
        for colour, foreign in ((True, False), (False, False), (True, True)):
            held_ref, held = {}, {}
            differ = []
            for k, (m, cells) in enumerate(frames):
                a = _reference(cells, cam, m, colour, held_ref, foreign)
                b = _listed(paint.painted(cells, cam, m, colour, held, foreign), width)
                if a != b or held_ref.get('exposure') != held.get('exposure'):
                    differ.append('pose %d: %d cells against %d' % (k, len(b), len(a)))
            report.check('the face painted at once is the reference\'s, %dx%d%s%s'
                         % (width, height, '' if colour else ' mono',
                            ' foreign' if foreign else ''), not differ, '; '.join(differ[:3]))


ROSTER = (test_the_face_is_painted_at_once,)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
