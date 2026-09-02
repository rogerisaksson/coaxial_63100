"""The 3D engine, stage by stage, against exact expectations.

Five stages - pose, camera, raster, shade, compose - each checked in
isolation, then the whole chain against an analytic ray-cast oracle on
the exporter's cube: no triangles, no raster, just slab intersection
and the same depth ramp. Interior cells (not bordering a class change,
where a raster's half-cell aliasing lives) must match 100 %.

The pose stage cross-validates against the QUATERNION path in
coaxial.orientation - two independent implementations that must agree.
render/render_demo.ps1 is the bench that runs this file.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from coaxial import engine, mesh, orientation                # noqa: E402
from coaxial.orientation import _qmul                        # noqa: E402
from coaxial import wireframe                                # noqa: E402

CUBE_STL = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                        'render', 'models', 'cube.stl')
WIDTH, HEIGHT = 100, 50
DISTANCE = 3.2


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


def quat_pose(rx, ry, rz, order='xyz'):
    """The same rotation through quaternions - the independent path."""
    def turn(axis, deg):
        h = math.radians(deg) / 2.0
        s = math.sin(h)
        return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(h))

    parts = {'x': turn((1, 0, 0), rx), 'y': turn((0, 1, 0), ry),
             'z': turn((0, 0, 1), rz)}
    q = (0.0, 0.0, 0.0, 1.0)
    for name in order:
        q = _qmul(q, parts[name])
    return orientation.matrix(q)


def spin(m, v):
    return tuple(m[3 * r] * v[0] + m[3 * r + 1] * v[1]
                 + m[3 * r + 2] * v[2] for r in range(3))


def near(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def test_pose(report):
    m = engine.pose(90, 0, 0)
    report.check('pose: Rx90 takes Y to Z', near(spin(m, (0, 1, 0)),
                                                 (0, 0, 1)))
    m = engine.pose(0, 0, 90)
    report.check('pose: Rz90 takes X to Y', near(spin(m, (1, 0, 0)),
                                                 (0, 1, 0)))
    a = engine.pose(90, 0, 90, 'xyz')
    b = engine.pose(90, 0, 90, 'zyx')
    report.check('pose: order matters', not near(a, b, 1e-6))
    for rot in ((45, 45, 45), (30, 0, 120), (75, 20, 130)):
        want = quat_pose(*rot)
        got = engine.pose(*rot)
        report.check('pose: x%dy%dz%d matches the quaternion path' % rot,
                     near(got, want, 1e-9))


def test_camera(report):
    cam = engine.camera(WIDTH, HEIGHT, 1.0, distance=DISTANCE)
    eye = (1, 0, 0, 0, 1, 0, 0, 0, 1)
    x, y, _w = engine.project(cam, eye, (0, 0, 0))
    report.check('camera: origin projects to centre',
                 abs(x - cam['cx']) < 1e-9 and abs(y - cam['cy']) < 1e-9)
    x1, _y, _w = engine.project(cam, eye, (0.5, 0, 0))
    report.check('camera: +X projects right', x1 > cam['cx'])
    _x, y1, _w = engine.project(cam, eye, (0, 0.5, 0))
    report.check('camera: +Y projects up', y1 < cam['cy'])
    double = engine.camera(WIDTH, HEIGHT, 1.0, distance=DISTANCE,
                           zoom=2.0)
    x2, _y, _w = engine.project(double, eye, (0.5, 0, 0))
    report.check('camera: zoom 2 doubles the offset',
                 abs((x2 - cam['cx']) - 2 * (x1 - cam['cx'])) < 1e-9)
    # Far camera approaches orthographic: symmetric points project
    # symmetrically.
    far = engine.camera(WIDTH, HEIGHT, 1.0, distance=60.0)
    ax, _y, _w = engine.project(far, eye, (0.5, 0, 0.5))
    bx, _y, _w = engine.project(far, eye, (-0.5, 0, -0.5))
    lop = abs((ax - far['cx']) + (bx - far['cx']))
    report.check('camera: D=60 is near-orthographic',
                 lop < 0.02 * abs(ax - far['cx']),
                 'asymmetri %.4f' % lop)


def oracle(q_m, cam, half, pivot, slope, floor):
    """The expected picture: rays against the cube's slabs, no raster."""
    m = q_m
    rows = []
    for py in range(cam['height']):
        v = (cam['cy'] - (py + 0.5)) / (cam['scale'] * 0.5)
        line = []
        for px in range(cam['width']):
            u = (px + 0.5 - cam['cx']) / cam['scale']
            ex, ey, ez = (m[6] * cam['distance'], m[7] * cam['distance'],
                          m[8] * cam['distance'])
            dx = m[0] * u + m[3] * v - m[6]
            dy = m[1] * u + m[4] * v - m[7]
            dz = m[2] * u + m[5] * v - m[8]
            t0, t1, ok = 0.0, 1e9, True
            for e, d, h in ((ex, dx, half[0]), (ey, dy, half[1]),
                            (ez, dz, half[2])):
                if abs(d) < 1e-12:
                    if abs(e) > h:
                        ok = False
                        break
                    continue
                ta, tb = (-h - e) / d, (h - e) / d
                if ta > tb:
                    ta, tb = tb, ta
                t0, t1 = max(t0, ta), min(t1, tb)
                if t0 > t1:
                    ok = False
                    break
            if not ok:
                line.append(0)
                continue
            level = pivot + slope * (cam['distance'] - t0) / cam['reach']
            if level < floor:
                level = floor
            level = 0.0 if level < 0.0 else (2.0 if level > 2.0
                                             else level)
            line.append(int(level + 0.5))
        rows.append(line)
    return rows


def interior(rows, x, y):
    me = rows[y][x]
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx, ny = x + dx, y + dy
        if 0 <= ny < len(rows) and 0 <= nx < len(rows[0]) \
                and rows[ny][nx] != me:
            return False
    return True


def cube():
    got = wireframe._decimated(CUBE_STL, 400)
    pos = got[0]
    half = tuple(max(abs(pos[3 * i + k]) for i in range(len(pos) // 3))
                 for k in range(3))
    return got, half


def test_shade_units(report):
    cam = {'width': 1, 'height': 1, 'distance': DISTANCE, 'scale': 10.0,
           'cx': 0.5, 'cy': 0.5}
    eye = (1, 0, 0, 0, 1, 0, 0, 0, 1)

    def one(tz, top=0, sun=0, art=None, shadow=None, step=0.0,
            pivot=2.0, slope=1.0, floor=0.55):
        depth = [1.0 / (DISTANCE - tz)]
        return engine.shade(depth, bytearray([top]), bytearray([sun]),
                            cam, eye, pivot, slope, floor, art=art,
                            shadow=shadow, shadow_step=step)[0]

    report.check('shade: tz 0 is the near class', one(0.0) == 2)
    report.check('shade: tz -1 is the far class', one(-1.0) == 1)
    report.check('shade: the floor holds bare geometry at "."',
                 one(-3.0) == 1)
    report.check('shade: floor 0 lets geometry go blank',
                 one(-3.0, floor=0.0) == 0)
    art = (['.'], 1, 1, [[2]])
    report.check('shade: art ink 2 rides the base',
                 one(0.0, top=1, art=art) == 2)
    art = (['.'], 1, 1, [[1]])
    report.check('shade: art ink 1 steps one down',
                 one(0.0, top=1, art=art) == 1)
    art = (['.'], 1, 1, [[0]])
    report.check('shade: art blank opens the cell',
                 one(0.0, top=1, art=art) == 0)

    # One cell of a 2x2 raster: three subsamples hit, the nearest (0.9)
    # sits bottom-right with top set and sun clear.
    depth2 = [0.5, 0.0,
              0.7, 0.9]
    top2, sun2 = bytearray([0, 0, 0, 1]), bytearray([1, 1, 1, 0])
    fdepth, ftop, fsun, cover = engine.fold(depth2, top2, sun2, 1, 1)
    report.check('fold: nearest subsample, its flags, and 3/4 coverage',
                 (fdepth, bytes(ftop), bytes(fsun), cover)
                 == ([0.9], b'\x01', b'\x00', [0.75]))
    always = ([9.0], 1, 1.0, (1, 0, 0), (0, 1, 0), (0, 0, 1))
    report.check('shade: a shadowed art cell steps down',
                 one(0.0, top=1, sun=1, art=(['.'], 1, 1, [[2]]),
                     shadow=always, step=1.0) == 1)
    report.check('shade: no sun flag, no shadow test',
                 one(0.0, top=1, sun=0, art=(['.'], 1, 1, [[2]]),
                     shadow=always, step=1.0) == 2)


def test_chain(report):
    solid, half = cube()
    reach = math.sqrt(sum(h * h for h in half))
    pivot, slope, floor = (wireframe.PIVOT, wireframe.SLOPE,
                           wireframe.FLOOR)
    for rot in ((0, 0, 0), (45, 45, 45), (30, 0, 0), (0, 60, 25),
                (75, 20, 130), (10, 80, 200)):
        cam = engine.camera(WIDTH, HEIGHT, reach, distance=DISTANCE)
        m = engine.pose(*rot)
        depth, top, sun = engine.raster(solid, m, cam)
        got = engine.shade(depth, top, sun, cam, m, pivot, slope, floor)
        want = oracle(m, cam, half, pivot, slope, floor)
        hit = miss = 0
        for y in range(HEIGHT):
            for x in range(WIDTH):
                if not interior(want, x, y):
                    continue
                same = got[y * WIDTH + x] == want[y][x]
                hit += same
                miss += not same
        share = hit / float(hit + miss or 1)
        report.check('chain: cube x%dy%dz%d matches the oracle' % rot,
                     share >= 0.995, '%.1f%% inre' % (100 * share))


def test_outline(report):
    """The wireframe overlay's edge choice on a synthetic solid, exactly.

    A box 0.1 tall on a slab at z 0, both as indexed triangles with the
    real face normals. The creases: the box's four top edges and four
    vertical corners - eight, one loop, extent the box's width. Not
    creases: the slab's own diagonals (coplanar), the box top's diagonal
    (coplanar), and the box's four base edges - at z 0 the height gate
    drops them, as it drops every pad and hole on the real board.
    """
    pos, idx, nrm = [], [], []

    def vertex(p):
        pos.extend(p)
        return len(pos) // 3 - 1

    def quad(a, b, c, d):
        for tri in ((a, b, c), (a, c, d)):
            idx.extend(tri)
            n = mesh.face_normal(pos[3 * tri[0]:3 * tri[0] + 3],
                                 pos[3 * tri[1]:3 * tri[1] + 3],
                                 pos[3 * tri[2]:3 * tri[2] + 3],
                                 (0.0, 0.0, 1.0))
            nrm.extend(n)

    s = [vertex(p) for p in ((-1, -1, 0), (1, -1, 0), (1, 1, 0), (-1, 1, 0))]
    quad(*s)                                             # the slab
    lo = [vertex(p) for p in ((-.2, -.2, 0), (.2, -.2, 0),
                              (.2, .2, 0), (-.2, .2, 0))]
    hi = [vertex(p) for p in ((-.2, -.2, .1), (.2, -.2, .1),
                              (.2, .2, .1), (-.2, .2, .1))]
    quad(*hi)                                            # the lid
    for i in range(4):                                   # the walls
        j = (i + 1) % 4
        quad(lo[i], lo[j], hi[j], hi[i])
    solid = (pos, idx, nrm)

    report.check('outline: the slab top is measured, not assumed',
                 abs(wireframe._slab_top(pos)) < 1e-9,
                 '%.4f' % wireframe._slab_top(pos))
    edges = {tuple(sorted(e))
             for e in wireframe._features(solid, min_rise=0.02)}
    want = {tuple(sorted((hi[i], hi[(i + 1) % 4]))) for i in range(4)}
    want |= {tuple(sorted((lo[i], hi[i]))) for i in range(4)}
    report.check('outline: a box on a slab is its lid and its corners',
                 edges == want,
                 'extra %s, missing %s' % (sorted(edges - want),
                                           sorted(want - edges)))
    loops = wireframe._outline_loops(solid)
    shape = sorted((round(e, 3), len(m)) for e, m in loops)
    report.check("outline: the box's lid and corners, its footprint on the "
                 "slab, and the slab's own rim",
                 shape == [(0.4, 4), (0.4, 8), (2.0, 4)], str(shape))
    loops = [(e, m) for e, m in loops if len(m) == 8]   # the box alone below
    # The size filter: at a camera where 0.4 units is under OUTLINE_CELLS
    # the loop is skipped; where it spans the frame it draws. Same box,
    # two zooms, drawn onto a buffer the box's lid occupies at depth 1.
    def drawn_at(zoom):
        cam = engine.camera(40, 12, 1.5, distance=3.2, zoom=zoom)
        m = (1, 0, 0, 0, 1, 0, 0, 0, 1)
        grid = [[' '] * 40 for _ in range(12)]
        tone = [[None] * 40 for _ in range(12)]
        # A depth buffer the lines always pass: nothing in front.
        buf = [0.0] * (40 * 12)
        real = wireframe._outline_source
        wireframe._outline_source = lambda: (solid, loops)
        try:
            n = wireframe._outline(grid, tone, buf, cam, m, False)
        finally:
            wireframe._outline_source = real
        return n, grid

    # At zoom 0.6 this camera puts 3.9 cells on a unit, so the 0.4 box is
    # 1.6 cells - under OUTLINE_CELLS, filtered; at zoom 3 it is 7.8.
    report.check('outline: a loop under OUTLINE_CELLS is not drawn',
                 drawn_at(0.6)[0] == 0, str(drawn_at(0.6)[0]))
    n, grid = drawn_at(3.0)
    braille = [g for row in grid for g in row if 0x2800 <= ord(g) < 0x2900]
    report.check('outline: past the cells it draws, in braille dots',
                 n > 0 and len(braille) == n,
                 '%d cells, %d braille' % (n, len(braille)))


def test_key_light(report):
    """The key light on a synthetic plane: leaning into the beam is
    brighter than flat, leaning away is darker - the sign, held exactly.

    Nine cells of class 2 at one level, no grain, full coverage. `bare`
    is view z per cell: flat, rising to the LEFT (the surface's normal
    leans right, into LIGHT's +x), and rising to the right (away). The
    centre cell's tone luma orders the three.
    """
    w = h = 3
    cam = {'width': w, 'height': h, 'distance': 3.2, 'scale': 60.0,
           'reach': 1.0, 'cx': 1.5, 'cy': 1.5}
    classes = bytearray([2] * 9)
    levels = [2.0] * 9
    seed = [0.5] * 9
    coverage = [1.0] * 9

    def luma_at_centre(slope):
        bare = [2.0 + slope * (c - 1) for _r in range(h) for c in range(w)]
        grid = [[' '] * w for _ in range(h)]
        tone = [[None] * w for _ in range(h)]
        wireframe._glow(grid, tone, classes, levels, bare, seed, coverage,
                        w, h, True, cam=cam)
        r, g, b = tone[1][1]
        return 0.2126 * r + 0.7152 * g + 0.0722 * b

    # A gentle slope: this camera scales a bare step of 0.04 a cell to a
    # gradient of 0.3 - a plane a third tilted. Steeper than the beam's
    # own angle a plane tilts PAST the light and darkens on both sides,
    # which the first draft of this test measured and misread as a sign
    # error: a 77-degree plane facing a beam that is 77 % frontal gets
    # less of it than face-on.
    flat, toward, away = (luma_at_centre(0.0), luma_at_centre(-0.04),
                          luma_at_centre(0.04))
    report.check('key light: a plane leaning into the beam is brighter',
                 toward > flat + 1.0, '%.1f vs %.1f' % (toward, flat))
    report.check('key light: a plane leaning away is darker',
                 away < flat - 1.0, '%.1f vs %.1f' % (away, flat))


def main():
    report = Report()
    print('\n-- the 3D engine, stage by stage --')
    test_pose(report)
    test_camera(report)
    test_shade_units(report)
    test_chain(report)
    test_outline(report)
    test_key_light(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
