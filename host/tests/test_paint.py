"""The face painted at once (coaxial.graphics.paint, strokes): cell for cell, glyph and tone, what
the reference passes (`wireframe._paint`) paint one cell at a time; the shadow it casts, the meshes under it."""
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


def _cast(solid, m, light, size, extent):
    """The shadow map triangle by triangle, as `shading._shadowmap` rastered it before
    `shadow.cast`: the reference."""
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m
    lx, ly, lz = light
    rn = math.sqrt(lz * lz + lx * lx)
    right = (lz / rn, 0.0, -lx / rn)
    up = (ly * right[2] - lz * right[1],
          lz * right[0] - lx * right[2],
          lx * right[1] - ly * right[0])
    pos, idx, _nrm = solid
    n = len(pos) // 3
    la, lb, lc = [0.0] * n, [0.0] * n, [0.0] * n
    half = (size - 1) / 2.0
    for i in range(n):
        x, y, z = pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]
        tx = m0 * x + m1 * y + m2 * z
        ty = m3 * x + m4 * y + m5 * z
        tz = m6 * x + m7 * y + m8 * z
        la[i] = ((tx * right[0] + ty * right[1] + tz * right[2])
                 / extent + 1.0) * half
        lb[i] = ((tx * up[0] + ty * up[1] + tz * up[2])
                 / extent + 1.0) * half
        lc[i] = tx * lx + ty * ly + tz * lz

    sbuf = [-9.0] * (size * size)
    for t in range(len(idx) // 3):
        a, b, c = idx[3 * t], idx[3 * t + 1], idx[3 * t + 2]
        x0, y0, x1, y1, x2, y2 = la[a], lb[a], la[b], lb[b], la[c], lb[c]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if area == 0.0:
            continue
        da, db, dc = lc[a], lc[b], lc[c]
        if area < 0.0:
            x1, y1, x2, y2 = x2, y2, x1, y1
            db, dc = dc, db
            area = -area
        lo_x = max(0, int(min(x0, x1, x2)))
        hi_x = min(size - 1, int(max(x0, x1, x2)) + 1)
        lo_y = max(0, int(min(y0, y1, y2)))
        hi_y = min(size - 1, int(max(y0, y1, y2)) + 1)
        inv = 1.0 / area
        # The row terms hoisted as in engine.raster: the same floats, less work
        # per pixel.
        e0x, e0y = x2 - x1, y2 - y1
        e1x, e1y = x0 - x2, y0 - y2
        for py in range(lo_y, hi_y + 1):
            row = py * size
            r0 = e0x * (py - y1)
            r1 = e1x * (py - y2)
            for px in range(lo_x, hi_x + 1):
                w0 = (r0 - e0y * (px - x1)) * inv
                if w0 < 0.0:
                    continue
                w1 = (r1 - e1y * (px - x2)) * inv
                if w1 < 0.0:
                    continue
                w2 = 1.0 - w0 - w1
                if w2 < 0.0:
                    continue
                here = w0 * da + w1 * db + w2 * dc
                at = row + px
                if here > sbuf[at]:
                    sbuf[at] = here
    return sbuf


def test_the_shadow_is_cast_at_once(report):
    """`shadow.cast`, texel for texel, is the triangle-by-triangle raster: the casters at a
    tumble's poses and face down."""
    from coaxial.graphics import shading, shadow, solids
    casters = solids._casters()
    differ = []
    for k, q in enumerate(POSES):
        m = orientation.matrix(q)
        once = shadow.cast(casters, m, shading.LIGHT, 56, 1.3)[0].tolist()
        if once != _cast(casters, m, shading.LIGHT, 56, 1.3):
            differ.append(k)
    report.check('the shadow map cast at once is the reference\'s at %d poses' % len(POSES),
                 not differ, 'poses %s differ' % differ)


def test_the_crew_decimates_as_this_process_does(report):
    """A level of detail the crew's processes decimate - the preload missing or stale - is the
    one `solids._decimated` makes here, the bore kept: without it the bench saw two holes."""
    from coaxial.graphics import crew, solids
    grid = solids.LODS[0][1]
    _grid, crewed = crew._decimate((orientation.MODEL, grid, solids._bore_keep))
    report.check("the crew's decimate at grid %d is this process's" % grid,
                 crewed == solids._decimated(orientation.MODEL, grid), 'differs')


ROSTER = (test_the_face_is_painted_at_once, test_the_shadow_is_cast_at_once,
          test_the_crew_decimates_as_this_process_does)


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
