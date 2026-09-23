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

from coaxial import engine, mesh, orientation, raster        # noqa: E402
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

    # One cell of the dot raster, 2 wide by 4 tall: five subsamples hit,
    # the nearest (0.9) sits at lane 1 row 3 with top set and sun clear,
    # and the reached mask is those five dots' braille bits.
    depth2 = [0.5, 0.0,
              0.7, 0.0,
              0.0, 0.6,
              0.8, 0.9]
    top2 = bytearray([0, 0, 0, 0, 0, 0, 0, 1])
    sun2 = bytearray([1, 1, 1, 1, 1, 1, 1, 0])
    fdepth, ftop, fsun, cover, reached = engine.fold(depth2, top2, sun2,
                                                     1, 1)
    hit = (raster.BRAILLE_BITS[0][0] | raster.BRAILLE_BITS[0][1]
           | raster.BRAILLE_BITS[1][2] | raster.BRAILLE_BITS[0][3]
           | raster.BRAILLE_BITS[1][3])
    report.check('fold: nearest subsample, its flags, 5/8 coverage and '
                 'the reached dots as braille bits',
                 (fdepth, bytes(ftop), bytes(fsun), cover, bytes(reached))
                 == ([0.9], b'\x01', b'\x00', [0.625], bytes([hit])))
    fine = engine.fine(engine.camera(10, 5, 1.0))
    report.check('fine: the dot camera is 2x4 the cells, square samples',
                 (fine['width'], fine['height'], fine['aspect'],
                  fine['scale'] / engine.camera(10, 5, 1.0)['scale'])
                 == (20, 20, 1.0, 2.0))
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
            if n is not None:
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
    report.check("outline: the box's lid and corners alone - its footprint "
                 "and the slab's rim are the raster's silhouette, not loops",
                 shape == [(0.4, 8)], str(shape))
    report.check('outline: a slab with one face has no bottom',
                 wireframe._slab_bottom(pos, wireframe._slab_top(pos)) is None)
    # The same slab given its bottom face, a millimetre and a half down,
    # and a box hanging under it: the bottom is found and the hanging
    # box drawn, its lid and corners like the one on top.
    both, both_idx, both_nrm = list(pos), list(idx), list(nrm)
    under = [vertex(p) for p in ((-1, -1, -.05), (1, -1, -.05),
                                 (1, 1, -.05), (-1, 1, -.05))]
    quad(under[0], under[3], under[2], under[1])          # facing down
    lo2 = [vertex(p) for p in ((.5, .5, -.05), (.7, .5, -.05),
                               (.7, .7, -.05), (.5, .7, -.05))]
    # A tenth tall, not more: the density gate (OUTLINE_DENSITY) takes a
    # 0.2 box's lid and corners up to 1.2 units of edge, exactly.
    hi2 = [vertex(p) for p in ((.5, .5, -.1), (.7, .5, -.1),
                               (.7, .7, -.1), (.5, .7, -.1))]
    quad(hi2[0], hi2[3], hi2[2], hi2[1])
    for i in range(4):
        j = (i + 1) % 4
        quad(lo2[i], hi2[i], hi2[j], lo2[j])
    two = (pos, idx, nrm)
    shape = sorted((round(e, 3), len(m)) for e, m in wireframe._outline_loops(two))
    report.check('outline: with a bottom face the top is still the top, the '
                 'bottom is found, and a box under the slab is a loop too',
                 shape == [(0.2, 8), (0.4, 8)]
                 and abs(wireframe._slab_top(pos)) < 1e-9
                 and abs(wireframe._slab_bottom(pos, 0.0) + 0.05) < 1e-9,
                 str(shape))
    del pos[len(both):]
    del idx[len(both_idx):]
    del nrm[len(both_nrm):]
    loops = wireframe._outline_loops(solid)
    loops = [l for l in loops if len(l[1]) == 8]         # the box alone below
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


def test_the_edge_is_the_rasters_silhouette(report):
    """The slab's edge and its holes are drawn from the coverage the fold
    reported, dot by dot, so they cannot sit beside what the face
    drew - the bench saw the mesh's ring beside the raster's hole. On a
    synthetic coverage: a full rectangle with a 2x2 hole and a one-cell
    pinhole. The rectangle's perimeter and the hole's four-neighbours
    get edge dots; the interior, the pinhole's neighbours and the frame's
    own edge do not.
    """
    width, height = 20, 10
    reached = bytearray(width * height)
    for r in range(2, 8):
        for c in range(3, 17):
            reached[r * width + c] = 0xFF
    for r, c in ((4, 8), (4, 9), (5, 8), (5, 9)):     # the hole
        reached[r * width + c] = 0
    reached[6 * width + 13] = 0                        # the pinhole
    cam = {'width': width, 'height': height}

    def drawn():
        grid = [[' '] * width for _ in range(height)]
        tone = [[None] * width for _ in range(height)]
        n = wireframe._edge(grid, tone, (None, None, reached), cam, False)
        return n, {(r, c) for r in range(height) for c in range(width)
                   if grid[r][c] != ' '}
    n, cells = drawn()
    perimeter = {(r, c) for r in range(2, 8) for c in range(3, 17)
                 if r in (2, 7) or c in (3, 16)}
    ring = {(3, 8), (3, 9), (6, 8), (6, 9), (4, 7), (5, 7), (4, 10), (5, 10)}
    report.check("edge: the rectangle's perimeter and the hole's "
                 "four-neighbours, %d cells" % n,
                 cells == perimeter | ring and n == len(cells),
                 'extra %s, missing %s' % (sorted(cells - perimeter - ring),
                                           sorted((perimeter | ring) - cells)))
    report.check("edge: the pinhole's neighbours are not edges",
                 not ({(6, 12), (6, 14), (5, 13), (7, 13)} & cells
                      - perimeter))
    # The same rectangle against the frame's left edge: no line there.
    reached = bytearray(width * height)
    for r in range(2, 8):
        for c in range(0, 14):
            reached[r * width + c] = 0xFF
    n, cells = drawn()
    report.check("edge: the frame's own edge is not an edge - a board cut "
                 "by the frame has no line there",
                 (3, 0) not in cells and (4, 0) not in cells
                 and (2, 0) in cells and (7, 0) in cells, sorted(cells)[:6])
    left = drawn()
    reached[2 * width + 0] = 0x40 | 0x80                 # only the bottom row of dots
    n, cells = drawn()
    report.check('edge: the dots are the coverage\'s own - a cell with '
                 'its lower row alone draws that row',
                 (2, 0) in cells and n == left[0], str(n))
    # THE LIT COVERAGE: with the light given, a covered cell the light
    # left at zero - a wall seen edge-on, drawn blank - counts as empty,
    # so the line goes round what is drawn. A dark band three cells
    # wide on the right moves the edge to the last lit column and draws
    # nothing in the band; a dark pinhole inside is no edge either.
    reached = bytearray(width * height)
    for r in range(2, 8):
        for c in range(3, 17):
            reached[r * width + c] = 0xFF
    heat = [1.0] * (width * height)
    for r in range(2, 8):
        for c in range(14, 17):
            heat[r * width + c] = 0.0
    heat[5 * width + 6] = 0.0
    grid = [[' '] * width for _ in range(height)]
    tone = [[None] * width for _ in range(height)]
    wireframe._edge(grid, tone, (None, None, reached), cam, False, heat=heat)
    cells = {(r, c) for r in range(height) for c in range(width)
             if grid[r][c] != ' '}
    lit = {(r, c) for r in range(2, 8) for c in range(3, 14)
           if r in (2, 7) or c in (3, 13)}
    report.check('edge: with the light given the line goes round the lit '
                 'coverage - a dark wall band lies outside it',
                 cells == lit,
                 'extra %s, missing %s' % (sorted(cells - lit),
                                           sorted(lit - cells)))


def test_ink_never_leans_below_the_floor(report):
    """At a steep tilt the lean dimming took every cell of the art to
    class 0 - 599 of 653 blank cells at 73 degrees, the face gone and
    the parts' walls left standing as a thick block - so an inked art
    cell now floors at the bare geometry's floor. Rendered at the
    bench's screenshot pose (73 degrees off face-on) at 108x44: fewer
    than a tenth of the covered cells draw blank, where half did.
    """
    import re
    q = (-0.600, 0.264, -0.257, 0.710)
    keep = {}
    real = wireframe._cells

    def hook(*a, **k):
        got = real(*a, **k)
        keep['reached'] = got[2]
        return got
    wireframe._cells = hook
    try:
        art = wireframe.render(q, 108, 44, zoom=1.2672, colour=True,
                               horizon=False, triad=False,
                               lift=orientation.LIFT,
                               least=wireframe.CREW_LEAST, persist={})
    finally:
        wireframe._cells = real
    rows = [re.sub(r'\x1b\[[0-9;]*m', '', r) for r in art.split('\n')]
    reached = keep['reached']
    covered = blank = 0
    for r in range(44):
        for c in range(108):
            if not reached[r * 108 + c]:
                continue
            covered += 1
            g = rows[r][c] if c < len(rows[r]) else ' '
            if g == ' ' or g == chr(raster.BRAILLE):
                blank += 1
    share = blank / covered if covered else 1.0
    report.check('ink floor: at 73 degrees under a tenth of the covered '
                 'cells draw blank', 0 < covered and share < 0.10,
                 '%d of %d blank, %.0f%%' % (blank, covered, 100 * share))


def test_the_art_stops_at_its_disc(report):
    """A surface point past ART_DISC of the span takes no ink: the art's
    own ink reaches 0.94 to 1.00 by direction, and a rim cell landing
    on its blank outside drew nothing, pulling the edge a cell in. Face
    on (the identity pose, the plane at z = 0) a point at 0.90 hits the
    art and one at 0.98 does not; the hit is the top view's own cell.
    """
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    distance, w, h = 3.2, 106, 54
    inside = engine._art_hit(identity, 0.90 / distance, 0.0, distance, 0.0,
                             False, w, h)
    outside = engine._art_hit(identity, 0.98 / distance, 0.0, distance, 0.0,
                              False, w, h)
    report.check('art: a point at 0.90 of the span hits the art, at 0.98 '
                 'it does not', inside is not None and outside is None,
                 '%s / %s' % (inside, outside))
    centre = engine._art_hit(identity, 0.0, 0.0, distance, 0.0, False, w, h)
    report.check('art: the origin lands on the middle cell, %d of %d and '
                 '%d of %d' % (w // 2, w, h // 2, h),
                 centre is not None and centre[:2] == (w // 2, h // 2),
                 str(centre))


def test_the_outline_holds_together(report):
    """The outline's hidden-line grace follows the cell's own depth
    span, so a lid's edge no longer loses to the lid's near corner in
    the same cell: at the component side 65 degrees off face-on, the
    parts' loops drawn alone at 108x44 fall into pieces whose largest
    is over a hundred cells (163 measured; 45 with the fixed grace).
    """
    import re
    q = _diagonal_pose(65.0)
    keep = {}
    real = wireframe._outline

    def hook(grid, tone, buf, cam, m, colour, heat=None):
        keep['args'] = (buf, cam, m, heat)
        return 0
    wireframe._outline = hook
    try:
        wireframe.render(q, 108, 44, zoom=1.2672, colour=True, horizon=False,
                         triad=False, lift=orientation.LIFT,
                         least=wireframe.CREW_LEAST, persist={})
    finally:
        wireframe._outline = real
    buf, cam, m, heat = keep['args']
    grid = [[' '] * 108 for _ in range(44)]
    tone = [[None] * 108 for _ in range(44)]
    wireframe._outline(grid, tone, buf, cam, m, True, heat=heat)
    cells = {(r, c) for r in range(44) for c in range(108)
             if grid[r][c] != ' '}
    seen, largest = set(), 0
    for start in cells:
        if start in seen:
            continue
        stack, size = [start], 0
        seen.add(start)
        while stack:
            r, c = stack.pop()
            size += 1
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        largest = max(largest, size)
    report.check('outline: at 65 degrees the largest connected piece is '
                 'over a hundred cells', largest >= 100,
                 '%d cells in the largest piece of %d' % (largest, len(cells)))


def _diagonal_pose(off_deg):
    """The component side `off_deg` off face-on with the normal leaning
    equally in x and y, searched over two tilts in 5-degree steps."""
    want = math.cos(math.radians(off_deg))
    cam = engine.camera(108, 44, 1.5, distance=3.2, zoom=1.2672,
                        tip=wireframe.CAMERA_TIP, lift=orientation.LIFT)
    best = None
    for a in range(0, 360, 5):
        for b in range(0, 360, 5):
            q = _qmul((math.sin(math.radians(a) / 2), 0.0, 0.0,
                       math.cos(math.radians(a) / 2)),
                      (0.0, math.sin(math.radians(b) / 2), 0.0,
                       math.cos(math.radians(b) / 2)))
            m = engine.multiply(cam['view'], orientation.matrix(q))
            nx, ny, nz = m[2], m[5], m[8]
            score = (abs(nz - want) + abs(abs(nx) - abs(ny))
                     + 0.2 * (nx < 0) + 0.2 * (ny > 0))
            if best is None or score < best[0]:
                best = (score, q)
    return best[1]


def test_stereotypes(report):
    """The pre-scan fits each part's loops to the simple geometry it
    is, once: a box's lid and corners are a block of eight segments; a
    lone lid of twelve corners on one radius is a drum; a chamfered
    square's eight corners share a radius too but sit on a box's sides,
    so it is a block (the CPU came out round once); two end profiles of
    one width and height facing across are one block; and two loops on
    one footprint - a rounded part's base ring and lid - are one part.
    """
    pos = []

    def vertex(p):
        pos.extend(p)
        return len(pos) // 3 - 1

    def ring(points, z):
        ids = [vertex((x, y, z)) for x, y in points]
        return [(ids[i], ids[(i + 1) % len(ids)]) for i in range(len(ids))]

    def loop_of(members):
        return (0.4, members)
    top, bottom = 0.0, None
    square = [(-.2, -.2), (.2, -.2), (.2, .2), (-.2, .2)]
    lo = [vertex((x, y, 0.0)) for x, y in square]
    hi = [vertex((x, y, 0.1)) for x, y in square]
    box = [(hi[i], hi[(i + 1) % 4]) for i in range(4)] + [(lo[i], hi[i]) for i in range(4)]
    prims = wireframe._stereotype_loops(pos, [loop_of(box)], top, bottom)
    report.check("stereotype: a box's lid and corners are one block of "
                 "twelve segments", [(k, len(d)) for k, _e, d in prims] == [('block', 12)]
                 and abs(prims[0][1] - 0.4) < 1e-6, str([(k, round(e, 3)) for k, e, d in prims]))
    circle = [(0.2 * math.cos(2 * math.pi * k / 12), 0.2 * math.sin(2 * math.pi * k / 12))
              for k in range(12)]
    prims = wireframe._stereotype_loops(pos, [loop_of(ring(circle, 0.1))], top, bottom)
    report.check('stereotype: a lone lid of twelve corners on one radius is '
                 'a drum of that radius',
                 [k for k, _e, _d in prims] == ['drum']
                 and abs(prims[0][2][2] - 0.2) < 1e-6
                 and abs(prims[0][2][3] - 0.1) < 1e-9, str(prims))
    c = 0.02
    chamfered = [(-.2 + c, -.2), (.2 - c, -.2), (.2, -.2 + c), (.2, .2 - c),
                 (.2 - c, .2), (-.2 + c, .2), (-.2, .2 - c), (-.2, -.2 + c)]
    prims = wireframe._stereotype_loops(pos, [loop_of(ring(chamfered, 0.1))], top, bottom)
    report.check('stereotype: a chamfered square is a block, not a drum',
                 [k for k, _e, _d in prims] == ['block'], str(prims))
    # two end profiles of a rounded extrusion: up, across, down, in the
    # planes y = -0.1 and y = +0.1
    arches = []
    for y in (-0.1, 0.1):
        ids = [vertex(p) for p in ((-.1, y, 0.0), (-.1, y, .08), (-.05, y, .1),
                                   (.05, y, .1), (.1, y, .08), (.1, y, 0.0))]
        arches.append((0.2, [(ids[i], ids[i + 1]) for i in range(5)]))
    prims = wireframe._stereotype_loops(pos, arches, top, bottom)
    report.check('stereotype: two end profiles of one width and height, '
                 'facing across, are one block',
                 [(k, len(d)) for k, _e, d in prims] == [('block', 12)], str(prims))
    base = ring([(-.22, -.22), (.22, -.22), (.22, .22), (-.22, .22)], 0.03)
    lid = ring(square, 0.1)
    prims = wireframe._stereotype_loops(pos, [loop_of(base), loop_of(lid)], top, bottom)
    report.check('stereotype: a base ring and a lid on one footprint are '
                 'one part, one block',
                 [(k, len(d)) for k, _e, d in prims] == [('block', 12)]
                 and abs(prims[0][1] - 0.44) < 1e-6, str([(k, round(e, 3)) for k, e, d in prims]))
    # a lid with a small circle on it - a PE terminal's screw hole -
    # is a block AND a ring; a lid with a stray diagonal ridge keeps
    # its own orientation, the box round every point turned the USB
    # shell 60 degrees; a crest narrower than the base leans the legs
    # in, so a rounded shoulder stands inside the lid's edge
    screw = ring([(0.05 * math.cos(2 * math.pi * k / 8), 0.05 * math.sin(2 * math.pi * k / 8))
                  for k in range(8)], 0.1)
    prims = wireframe._stereotype_loops(pos, [loop_of(lid), loop_of(screw)], top, bottom)
    report.check('stereotype: a lid with a small circle on it is a block '
                 'and a ring', [(k, len(d)) for k, _e, d in prims] == [('block', 12), ('ring', 24)],
                 str([(k, round(e, 3)) for k, e, d in prims]))
    stray = ring([(.12, .12), (.22, .12), (.22, .22), (.12, .22)], 0.05)
    prims = wireframe._stereotype_loops(pos, [loop_of(lid), loop_of(stray)], top, bottom)
    edge = prims[0][2][0]
    report.check("stereotype: the block takes its widest loop's "
                 "orientation - a nested loop poking out a corner does "
                 "not turn it",
                 [k for k, _e, _d in prims] == ['block']
                 and abs(edge[1] - edge[4]) < 1e-9, str([(k, round(e, 3)) for k, e, d in prims]) + str(prims[0][2][:1]))
    lo3 = [vertex((x, y, 0.0)) for x, y in square]
    hi3 = [vertex((x * 0.5, y * 0.5, 0.1)) for x, y in square]
    tapered = [(hi3[i], hi3[(i + 1) % 4]) for i in range(4)] + [(lo3[i], hi3[i]) for i in range(4)]
    prims = wireframe._stereotype_loops(pos, [loop_of(tapered)], top, bottom)
    legs = [s for s in prims[0][2] if abs(s[2] - s[5]) > 1e-9]
    report.check('stereotype: a crest narrower than the base is still one '
                 'block over the base - lid 0.4 over a base 0.4, legs '
                 'straight',
                 [k for k, _e, _d in prims] == ['block'] and len(legs) == 4
                 and all(abs(abs(s[0]) - 0.2) < 1e-6 and abs(abs(s[3]) - 0.2) < 1e-6 for s in legs),
                 str(legs))
    lone = [vertex((.5, .5, 0.0)), vertex((.56, .54, .05))]
    prims = wireframe._stereotype_loops(pos, [(0.07, [(lone[0], lone[1])])], top, bottom)
    report.check('stereotype: a loop of one edge is its own stroke, not a '
                 'box round a diagonal',
                 [(k, len(d)) for k, _e, d in prims] == [('stroke', 1)], str(prims))
    # a screw terminal's two profiles per wall: the outer pair 0.5 wide
    # and 0.15 tall at y -0.2 and +0.2, the inner a box 0.4 wide and 0.1
    # tall - one block, the box round both, to the taller's height
    outer = []
    for y in (-0.2, 0.2):
        ids = [vertex(p) for p in ((-.25, y, 0.0), (-.25, y, .15), (.25, y, .15), (.25, y, 0.0))]
        outer.append((0.5, [(ids[i], ids[i + 1]) for i in range(3)]))
    prims = wireframe._stereotype_loops(pos, [loop_of(box)] + outer, top, bottom)
    tops = sorted(set(round(s[2], 3) for k, e, d in prims for s in d) | set(round(s[5], 3) for k, e, d in prims for s in d))
    report.check('stereotype: a block inside another block is one block, '
                 'the box round both to the taller height',
                 [(k, len(d)) for k, _e, d in prims] == [('block', 12)]
                 and abs(prims[0][1] - 0.5) < 1e-6 and tops == [0.0, 0.15],
                 str([(k, round(e, 3)) for k, e, d in prims]) + str(tops))
    # neighbours whose footprints overlap by a tenth stay two parts
    other = ring([(.15, -.2), (.55, -.2), (.55, .2), (.15, .2)], 0.1)
    prims = wireframe._stereotype_loops(pos, [loop_of(lid), loop_of(other)], top, bottom)
    report.check('stereotype: two lids overlapping by a tenth are two blocks',
                 [k for k, _e, _d in prims] == ['block', 'block'], str([(k, round(e, 3)) for k, e, d in prims]))
    # a hole in a wall: a closed loop in the plane y = -0.2 floating
    # 0.03 over the base, 0.06 wide and 0.03 tall - an oval, not an arch
    ids = [vertex((x, -0.2, z)) for x, z in ((-.03, .045), (0.0, .03), (.03, .045),
                                             (.03, .06), (0.0, .06), (-.03, .06))]
    hole = (0.06, [(ids[i], ids[(i + 1) % 6]) for i in range(6)])
    prims = wireframe._stereotype_loops(pos, [hole], top, bottom)
    report.check('stereotype: a closed loop floating in a wall is a hole, '
                 'an oval of twelve segments in its plane',
                 [(k, len(d[0])) for k, _e, d in prims] == [('hole', 12)]
                 and all(abs(s[1] + 0.2) < 1e-9 and abs(s[4] + 0.2) < 1e-9 for s in prims[0][2][0]),
                 str([(k, round(e, 3)) for k, e, d in prims]))
    # ...and drawn only where its wall faces the camera: face-on the
    # wall is edge-on and the oval a dash, so nothing; tilted 60
    # degrees about x the wall faces the camera and the oval draws.
    real = wireframe._outline_source
    wireframe._outline_source = lambda: ((pos, [], []), [hole])
    wireframe._STEREO.clear()
    try:
        cam = engine.camera(40, 12, 1.5, distance=3.2, zoom=6.0)
        buf = [0.0] * (40 * 12)

        def drawn(m):
            grid = [[' '] * 40 for _ in range(12)]
            tone = [[None] * 40 for _ in range(12)]
            return wireframe._outline(grid, tone, buf, cam, m, False)
        flat = drawn((1, 0, 0, 0, 1, 0, 0, 0, 1))
        c, s = math.cos(math.radians(60)), math.sin(math.radians(60))
        tilted = drawn((1, 0, 0, 0, c, -s, 0, s, c))
    finally:
        wireframe._outline_source = real
        wireframe._STEREO.clear()
    report.check('stereotype: a hole in a wall draws nothing edge-on and '
                 'its oval when the wall faces the camera',
                 flat == 0 and tilted > 0, '%d cells flat, %d tilted' % (flat, tilted))


def test_the_preload_is_adopted(report):
    """A preload bundle adopted into the caches is what every later ask
    finds: the LODs are its solids, the outline source its exact index
    and loops, the stereotypes its primitives - by identity, nothing
    rebuilt.
    """
    solids = wireframe._lods()
    exact, loops = wireframe._outline_source()
    prims = wireframe._stereotypes()
    bundle = {'lods': {d: s for (_z, d), s in zip(wireframe.LODS, solids)},
              'exact': (exact, loops), 'prims': prims}
    wireframe._forget()
    wireframe._OUTLINES.clear()
    took = wireframe._adopt(orientation.MODEL, bundle)
    again = wireframe._lods()
    report.check('preload: adopted, the LODs, the outline source and the '
                 'stereotypes are the bundle\'s own objects',
                 took == (len(wireframe.LODS), len(loops), len(prims))
                 and all(a is b for a, b in zip(again, solids))
                 and wireframe._outline_source()[1] is loops
                 and wireframe._stereotypes() is prims, str(took))


def test_the_decimate_keeps_the_bore(report):
    """The bore's wall is thinner than a grid-48 cell, so clustering
    merged its rings and the see-through came out smaller and shifted
    from the mesh's circle the art is drawn to - two holes on the
    bench. Corners within BORE_KEEP of the axis stay exact: the 48
    decimate holds the ring of 86 corners at radius 0.100, and nothing
    of the plate's face inside it.
    """
    _edges, solid = wireframe._model(1.2672, wireframe.CREW_LEAST)
    pts = solid[0]
    top = wireframe._slab_top(pts)
    bottom = wireframe._slab_bottom(pts, top)
    ring = inside = 0
    for i in range(len(pts) // 3):
        z = pts[3 * i + 2]
        if not (bottom - 0.005 <= z <= top + 0.005):
            continue
        r = math.hypot(pts[3 * i], pts[3 * i + 1])
        if abs(r - 0.100) < 0.002:
            ring += 1
        elif r < 0.098:
            inside += 1
    report.check('decimate: the bore\'s ring of corners at radius 0.100 '
                 'is kept exact', ring >= 80, '%d corners' % ring)
    report.check('decimate: no corner of the plate inside the bore',
                 inside == 0, '%d inside' % inside)


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
        tone: list = [[None] * w for _ in range(h)]
        wireframe._glow(grid, tone, classes, levels, bare, seed, coverage,
                        w, h, True, cam=cam)
        cell = tone[1][1]
        if cell is None:
            raise AssertionError('no tone in the cell')
        r, g, b = cell
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


def test_the_face_is_a_halftone(report):
    """The glyph is a dither at dot resolution: a 64 x 64 blue-noise
    mask laid over the dots, against the light sampled at each dot,
    between a density floor and a ceiling.

    ONE RUNG PER CELL WAS THE BLOCK; sampling the light per dot on the
    ladder's own order was the speckle after it; and an 8 x 8 Bayer
    matrix was the blocks after that - its two-by-two clusters read as
    small squares across a real window. Each was seen in a raster of
    the frame, which the glyph counts never showed. Blue noise has no
    structure at any density; the floor keeps the dark side a surface
    and the ceiling keeps the bright parts a texture. Held here on
    synthetic fields, and on the shipped board at the page's own size.
    """
    import collections
    import math

    from coaxial import orientation, raster
    w = wireframe

    n = w.NOISE_N
    flat = sorted(v for row in w.NOISE for v in row)
    report.check('the mask holds every rank once, 0 to %d' % (n * n - 1),
                 n == 64 and flat == list(range(n * n)))

    def field(width, height, heat_of):
        classes = bytearray([1] * (width * height))
        coverage = [1.0] * (width * height)
        heat = [heat_of(x, y) for y in range(height) for x in range(width)]
        grid = [[' '] * width for _ in range(height)]
        w._dots(grid, heat, classes, coverage, width, height, (0.0, 1.0))
        return grid

    def dots_in(grid, x0, y0, wide, tall):
        return sum(bin(ord(grid[y][x]) - raster.BRAILLE).count('1')
                   for y in range(y0, y0 + tall)
                   for x in range(x0, x0 + wide))

    floor, ceil = w.DENSITY_FLOOR, w.DENSITY_CEIL
    ok, said = True, []
    for s in (0.0, 0.25, 0.5, 0.75, 1.0):
        want = int((floor + (ceil - floor) * s) * n * n + 0.5)
        got = dots_in(field(n // 2, n // 4, lambda x, y, s=s: s),
                      0, 0, n // 2, n // 4)
        said.append('%.2f: %d of %d' % (s, got, want))
        # A cell the mask leaves blank keeps one dot - a drawn cell is
        # never blank - so at the floor, one dot a cell on average, the
        # count runs over by up to a dot for every cell left blank; and
        # on two rows of four the mask's ranks are a sample of the whole,
        # so the count sits within a couple of per cent of its share.
        ok = ok and want - 0.02 * n * n <= got <= want + (n // 2) * (n // 4)
    report.check('a flat field lights the mask\'s share of a tile: the '
                 'floor, then the light, up to the ceiling',
                 ok, ', '.join(said))

    # BLUE: at the window's middle every 8 x 8 window of dots holds near
    # the middle's share - no clusters, no voids - which a Bayer tile
    # holds exactly and a random field does not hold at all.
    grid = field(n // 2, n // 4, lambda x, y: 0.5)
    middle = 0.5 * (floor + ceil) * 64.0
    windows = [dots_in(grid, x, y, 4, 2)
               for y in range(0, n // 4, 2) for x in range(0, n // 2, 4)]
    report.check('at the middle density every 8 x 8 window of dots holds '
                 'within six of %.0f - even, and no structure to it'
                 % middle,
                 all(abs(got - middle) <= 6 for got in windows),
                 '%d..%d' % (min(windows), max(windows)))
    report.check('and a cell whose density clears no dot keeps one',
                 all(ord(g) > raster.BRAILLE
                     for row in field(8, 4, lambda x, y: -5.0) for g in row))

    # On two rows of four the mask is a sample of itself, so a band can
    # come out a dot or two under the one before it; the ramp still
    # climbs, and end to end it climbs by more than any dip.
    ramp = field(n, 4, lambda x, y: x / float(n - 1))
    columns = [dots_in(ramp, x, 0, 8, 4) for x in range(0, n, 8)]
    report.check('a ramp gains dots from one band to the next, within the '
                 'mask\'s own grain',
                 all(b >= a - 3 for a, b in zip(columns, columns[1:]))
                 and columns[-1] > columns[0] + 8,
                 ' '.join(map(str, columns)))

    report.check('the floor rolls off: continuous at the knee, ordered '
                 'below it, never DIMMEST',
                 w._floor(w.DIMMEST + w.KNEE) == w.DIMMEST + w.KNEE
                 and w.DIMMEST < w._floor(-10.0) < w._floor(0.0)
                 < w._floor(0.3) < w._floor(w.DIMMEST + w.KNEE))

    classes = bytearray([1] * 100)
    heat_a = [1.0 + 2.0 * i / 99.0 for i in range(100)]
    heat_b = [h + 3.0 for h in heat_a]
    persist = {}
    lo_a, hi_a = w._expose(heat_a, classes, persist)
    lo_b, hi_b = w._expose(heat_b, classes, persist)
    raw_lo, raw_hi = w._expose(heat_b, classes)
    report.check('the exposure follows a jump a third of the way a frame',
                 abs((lo_b - lo_a) - w.EXPOSE_FOLLOW * (raw_lo - lo_a)) < 1e-9
                 and hi_a < hi_b < raw_hi,
                 '%.2f -> %.2f toward %.2f' % (lo_a, lo_b, raw_lo))

    # THE SHIPPED BOARD at the attitude page's size, at the attitude the
    # stand-in reports - `(i, j, k, real)`, rpy -5.6, +2.8, -0.6: a board
    # lying on a bench. The colour path's own stages, so the count is of
    # the FACE's cells and not the ground grid's. Measured before, at
    # this pose: 299 of 561 lit cells at one dot, eight glyphs, the face
    # a fifth lit.
    from coaxial import engine
    q = (-0.0489, 0.0245, -0.0036, 0.9984)
    width, height, zoom = 78, 30, 0.88
    _edges, solid = w._model(zoom, 0)
    pts = solid[0]
    reach = max(math.sqrt(pts[3 * i] ** 2 + pts[3 * i + 1] ** 2
                          + pts[3 * i + 2] ** 2)
                for i in range(len(pts) // 3))
    cam = engine.camera(width, height, reach, distance=3.2, zoom=zoom,
                        tip=w.CAMERA_TIP, lift=orientation.LIFT)
    m = engine.multiply(cam['view'], orientation.matrix(q))
    buf, coverage, reached, classes, levels, bare, seed = w._cells(
        solid, m, cam, None, True, False)
    if classes is None:
        raise AssertionError('no classes came back')
    grid = [[' '] * width for _ in range(height)]
    tone: list = [[None] * width for _ in range(height)]
    heat = [0.0] * (width * height)
    w._glow(grid, tone, classes, levels, bare, seed, coverage, width,
            height, True, cam=cam, buf=buf, heat_out=heat)
    w._dots(grid, heat, classes, coverage, width, height,
            w._expose(heat, classes), reached)
    w._rim(grid, tone, classes, reached, heat, width, height, True)
    face = [grid[i // width][i % width] for i in range(width * height)
            if classes[i]]
    # NO SURFACE UNDER THE DOTS. A background per cell - the face's tone
    # dimmed, then blurred and coverage-weighted - was built and taken
    # out on the bench's word: blocks in colour, then a haze. The tone
    # stays a foreground alone, and a cell's ink is one RGB tuple.
    report.check('no lit cell carries a background: the ink is the '
                 'foreground alone',
                 all(isinstance(tone[i // width][i % width], tuple)
                     and len(tone[i // width][i % width]) == 3
                     for i in range(width * height) if classes[i]))
    counts = [bin(ord(g) - raster.BRAILLE).count('1') for g in face]
    hist = collections.Counter(counts)
    mean = sum(counts) / float(8 * len(face))
    # THE DENSITY WINDOW IS THE POINT, on the bench's screenshot: in the
    # terminal the glyph box is narrower than the cell, and past about
    # three dots in ten every cell reads as a brick. The face sits
    # between the floor and the ceiling, the rim cells solid above it.
    report.check('the shipped board at the page\'s size is scanlines, not '
                 'a wall: the face sits inside the density window, no '
                 'cell is blank, and whole and broken lines both occur',
                 floor - 0.03 <= mean <= ceil + 0.05 and hist[0] == 0
                 and hist[4] and (hist[2] or hist[3]),
                 '%.2f lit, %s' % (mean, ' '.join(
                     '%d:%d' % (r, hist[r]) for r in range(0, 9))))
    # Near-whole scanlines on two rows of four: the face's own alphabet
    # is the handful of patterns those rows make, plus the rim's lines.
    report.check('and it wears more glyphs than a carpet of one',
                 len(set(face)) >= 8, '%d distinct' % len(set(face)))

    # THE RIM, CLIPPED AND LIT. A part-covered cell's dots are all dots
    # the fine raster reached - nothing spills past the board - and the
    # rim cells wear the edge tone, brighter than the face.
    def luma(rgb):
        return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]

    rim, spilled, edge, inside = 0, 0, [], []
    for i in range(width * height):
        if not classes[i]:
            continue
        cell = tone[i // width][i % width]
        if reached[i] in (0, 0xFF):
            inside.append(luma(cell))
            continue
        rim += 1
        edge.append(luma(cell))
        mask = ord(grid[i // width][i % width]) - raster.BRAILLE
        spilled += bin(mask & ~reached[i] & 0xFF).count('1')
    edge.sort()
    inside.sort()
    report.check('no dot past the rim: every dot of a part-covered cell is '
                 'one the model reaches',
                 rim > 0 and spilled == 0,
                 '%d spilled over %d rim cells' % (spilled, rim))
    report.check('and the rim is a line of light, brighter than the face',
                 edge and inside
                 and edge[len(edge) // 2] > inside[len(inside) // 2],
                 'rim %.0f against face %.0f luma'
                 % (edge[len(edge) // 2], inside[len(inside) // 2]))


def test_triad(report):
    """The board's axes in the corner: each lettered once, X right and
    Y up at rest, and a quarter turn about Z puts X where Y was."""
    def letters(q):
        text = wireframe.render(q, 60, 20, zoom=1.0, colour=False,
                                horizon=False, triad=True)
        found = {}
        for r, line in enumerate(text.split('\n')):
            for c, ch in enumerate(line):
                if ch in 'XYZ':
                    found.setdefault(ch, []).append((c, r))
        return found

    rest = letters((0.0, 0.0, 0.0, 1.0))
    report.check('triad: X, Y and Z each lettered once at rest',
                 sorted(rest) == ['X', 'Y', 'Z']
                 and all(len(v) == 1 for v in rest.values()), str(rest))
    # Reach 4 at 20 rows (the floor), so in a 60-column frame the origin
    # sits at column 54, row 3 - flush into the upper right, see _triad.
    ox, oy = 54, 3
    x_at = rest.get('X', [(0, 0)])[0]
    y_at = rest.get('Y', [(0, 99)])[0]
    report.check('triad: at rest X points right and Y up',
                 x_at[0] > ox and y_at[1] < oy, '%s %s' % (x_at, y_at))
    half = math.sqrt(0.5)
    turned = letters((0.0, 0.0, half, half))        # a quarter turn about Z
    report.check('triad: a quarter turn about Z puts X where Y was',
                 turned.get('X') == rest.get('Y'),
                 '%s vs %s' % (turned.get('X'), rest.get('Y')))


def test_steady(report):
    """Three frames vote: a frame that differs from both its neighbours
    is outvoted, a change that stays shows one frame late and stays."""
    def draw(q, persist=None):
        return wireframe.render(q, 60, 20, zoom=1.0, colour=False,
                                horizon=False, persist=persist)

    a = (0.0, 0.0, 0.0, 1.0)
    b = (0.0, 0.0, math.sin(math.radians(2.0)), math.cos(math.radians(2.0)))
    fresh_a, fresh_b = draw(a), draw(b)
    state = {}
    draw(a, state)
    draw(b, state)
    report.check('steady: a one-frame blink is outvoted by its neighbours',
                 draw(a, state) == fresh_a, 'the blink showed')
    state = {}
    draw(a, state)
    draw(a, state)
    report.check('steady: a change shows one frame late',
                 draw(b, state) == fresh_a, 'shown at once')
    report.check('steady: and then stays',
                 draw(b, state) == fresh_b, 'not shown')


def test_the_face_is_held_while_the_pose_holds(report):
    """At rest the face is replayed from `persist` and only the ground
    is drawn: no raster, no shading, no outline. A new pose is drawn in
    full FACE_SETTLE times first so the exposure has glided, and a
    changed pose is a new drawing. THE FANS: a frame that costs more
    than its period never sleeps, and a board on a bench is at rest
    nearly always - measured, 50 ms a frame to 3 with the face held."""
    calls, rasters = [], []
    real, real_cells = wireframe._paint, wireframe._cells

    def counted(*args, **kwargs):
        calls.append(1)
        return real(*args, **kwargs)

    def counted_cells(*args, **kwargs):
        rasters.append(1)
        return real_cells(*args, **kwargs)

    q = (0.05, 0.02, 0.0, 0.998)
    state = {}
    wireframe._paint, wireframe._cells = counted, counted_cells
    try:
        shown = [wireframe.render(q, 60, 20, zoom=1.0, colour=True,
                                  persist=state, scroll=0.0)
                 for _ in range(wireframe.FACE_SETTLE + 1)]
        drawn, rastered = len(calls), len(rasters)
        held = wireframe.render(q, 60, 20, zoom=1.0, colour=True,
                                persist=state, scroll=0.0)
        moved = wireframe.render(q, 60, 20, zoom=1.0, colour=True,
                                 persist=state, scroll=0.7)
        after_hold = len(calls)
        turned = wireframe.render((0.3, 0.02, 0.0, 0.95), 60, 20, zoom=1.0,
                                  colour=True, persist=state, scroll=0.7)
        after_turn = len(calls)
    finally:
        wireframe._paint, wireframe._cells = real, real_cells

    report.check('a new pose is drawn in full FACE_SETTLE + 1 times',
                 drawn == wireframe.FACE_SETTLE + 1, '%d drawings' % drawn)
    report.check('from ONE raster: the cells are kept while the pose '
                 'settles and only the exposure moves', rastered == 1,
                 '%d rasters' % rastered)
    report.check('then held: two more frames at the same pose cost no '
                 'drawing', after_hold == drawn,
                 '%d drawings' % (after_hold - drawn))
    report.check('and the held frame is the drawn one, cell for cell',
                 held == shown[-1], 'differs')
    report.check('the ground still moves under the held face',
                 moved != held, 'the same picture')
    report.check('a turned board is drawn again',
                 after_turn == drawn + 1 and turned != moved,
                 '%d drawings' % (after_turn - drawn))
    report.check('and the cache is one entry, keyed by everything the '
                 'face depends on',
                 state['face']['settles'] == 0
                 and state['face']['key'][:2] == (60, 20),
                 str(state['face']['key']))


def test_the_crew_paints_one_pose_behind(report):
    """With `ahead`, a crew and `persist`, a moving board is painted one
    pose behind the one asked for - the crew rastering the newest while
    the previous is painted - and every picture is the one the
    synchronous path draws, a frame later. Measured on the threadripper
    (2026-09-23): the crew's 33-39 ms wait hidden under the parent's
    paint, the view's loop 76 -> 46 ms a frame.

    A rest first, so both paths' steady votes hold the same two
    frames; then four poses; then a rest on the last. Sync paints
    request i at frame i; ahead paints request i-1 from the first turn
    on, and the held picture stands for the frame the lag begins.
    """
    from coaxial import crew as crewmod

    rest = (0.05, 0.02, 0.0, 0.998)
    turn = [(0.05 + 0.04 * k, 0.02, 0.01 * k, 0.998) for k in range(1, 5)]
    asked = [rest] * 3 + turn + [turn[-1]] * 3
    pool = crewmod.Crew(wireframe._lods(), art=wireframe._face(), workers=4)
    try:
        sync, ahead = {}, {}
        s = [wireframe.render(q, 60, 20, zoom=1.0, colour=True, crew=pool,
                              persist=sync, scroll=0.0) for q in asked]
        a = [wireframe.render(q, 60, 20, zoom=1.0, colour=True, crew=pool,
                              persist=ahead, scroll=0.0, ahead=True)
             for q in asked]
        drained = ahead.get('flight') is None and pool.pending == 0
    finally:
        pool.close()
    report.check('at rest the two paths draw the same frames',
                 a[:3] == s[:3], 'differ')
    report.check('the frame the lag begins on is the held picture',
                 a[3] == s[2], 'differs')
    report.check('from then on every picture is the sync path\'s, one '
                 'frame later - through the turn and the rest after it',
                 all(a[i] == s[i - 1] for i in range(4, len(asked))),
                 str([i for i in range(4, len(asked)) if a[i] != s[i - 1]]))
    report.check('and a rest drains the crew: nothing in flight',
                 drained, str(ahead.get('flight')))
    # The steady vote shows the previous frame's glyphs, so the turn's
    # first picture still looks like the rest: count from the second.
    report.check('the pictures moved: the turn was drawn, not held',
                 len(set(s[3:8])) == 5, '%d distinct' % len(set(s[3:8])))


def test_scroll(report):
    """The ground moves: a quarter spacing on, every rung sits nearer
    the camera - lower on the screen - and the backdrop differs."""
    cam = engine.camera(60, 20, 1.5, distance=3.2, zoom=1.0,
                        tip=wireframe.CAMERA_TIP)
    static = wireframe._ground_static(60, 20, 3.2, cam['view'])
    before = wireframe._rungs(static, 0.0)
    after = wireframe._rungs(static, 0.25)
    report.check('scroll: a quarter spacing on, every rung is nearer',
                 len(before) == len(after)
                 and all(b[0] < a[0] for b, a in zip(before, after)),
                 '%d rungs' % len(before))
    report.check('scroll: the backdrop differs between the two phases',
                 wireframe._backdrop(60, 20, 3.2, cam['view'], 0.0)
                 != wireframe._backdrop(60, 20, 3.2, cam['view'], 0.25),
                 'the same')
    # A step is under half a dot row for every rung on screen: measured
    # 0.105 rows at 108x44 and 150x44, 0.048 at 60x20 (2026-09-23) -
    # a rung slides rather than jumps.
    a = wireframe._rungs(static, 0.0)
    b = wireframe._rungs(static, 1.0 / wireframe.RUNG_STEPS)
    moved = max(y1 - y0 for (y0, *_), (y1, *_) in zip(a, b)
                if static['hrow'] <= y0 < 20)
    report.check('scroll: one step moves a rung on screen under half a '
                 'dot row', 0.0 < moved < 0.125, '%.3f rows' % moved)


def test_fan_lines(report):
    """A ground line is its supercover - every dot it passes through, a
    chain of touching dots. Sampled one a dot along the steeper axis it
    skipped a row in every truncated piece and read as dashes."""
    lit = set()

    def dot(fx, fy, _depth, _k):
        lit.add((int(fx * 2.0), int(fy * 4.0)))
    # from dot (0, 0) to dot (6, 7), crossing six columns and seven
    # rows, none at a corner: 1 + 6 + 7 dots
    wireframe._segment(dot, (0.3, 0.1, 1.0), 3.2, 1.9, 2.0, 0)
    report.check('fan: a segment lights every dot it crosses',
                 len(lit) == 14, '%d dots' % len(lit))
    report.check('fan: no dot off the line',
                 all(0 <= a <= 6 and 0 <= b <= 7 for a, b in lit),
                 str(sorted(lit)))
    chained = all(any((a + da, b + db) in lit
                      for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)))
                  for a, b in lit)
    report.check('fan: every dot touches another along or across',
                 chained, str(sorted(lit)))
    lit.clear()
    wireframe._segment(dot, (0.3, 0.1, 1.0), 0.3, 0.1, 1.0, 0)
    report.check('fan: a segment of no length is one dot',
                 lit == {(0, 0)}, str(sorted(lit)))


def test_backdrop_cache(report):
    """The backdrop cache holds BACKDROPS_KEPT steps and then starts
    over - two window sizes' worth at 96 steps a spacing - so a session
    that resizes does not keep every size it ever had."""
    cam = engine.camera(60, 20, 1.5, distance=3.2, zoom=1.0,
                        tip=wireframe.CAMERA_TIP)
    kept = wireframe.BACKDROPS_KEPT
    wireframe.BACKDROPS_KEPT = 4
    wireframe._BACKDROP.clear()
    try:
        for s in range(4):
            wireframe._backdrop(60, 20, 3.2, cam['view'],
                                s / wireframe.RUNG_STEPS)
        full = len(wireframe._BACKDROP)
        wireframe._backdrop(60, 20, 3.2, cam['view'],
                            4.0 / wireframe.RUNG_STEPS)
        after = len(wireframe._BACKDROP)
    finally:
        wireframe.BACKDROPS_KEPT = kept
    report.check('backdrop: the cache fills to the cap', full == 4,
                 str(full))
    report.check('backdrop: the step past the cap starts it over',
                 after == 1, str(after))


def test_ladder(report):
    """The tone ladder: every pattern in U+2800, bucketed by dot count.

    A GLYPH RAMP OF THREE CHARACTERS HAS TWO STEPS ABOVE BLANK, and an
    ASCII render carries its 3D in the characters - so a leaning face had
    one step to fall through and a board came out as a flat carpet with a
    rim. Eight dots in a cell is a nine-step ladder in the same space,
    and the phases spend the rest of the block.
    """
    from coaxial import raster, wireframe

    rows = raster.SHADE
    report.check('nine rungs, blank to all eight dots', len(rows) == 9,
                 str([len(r) for r in rows]))
    report.check('every pattern in the block, once',
                 len({c for r in rows for c in r}) == 256
                 and sum(len(r) for r in rows) == 256,
                 '%d distinct of %d'
                 % (len({c for r in rows for c in r}),
                    sum(len(r) for r in rows)))
    report.check('a rung holds the patterns with that many dots',
                 all(all(bin(ord(c) - raster.BRAILLE).count('1') == rung
                         for c in row) for rung, row in enumerate(rows)))
    # THE EVEN ONE FIRST. Phase 0 is what a flat surface wears, so it has
    # to be the arrangement that reads as a tone rather than a clump: the
    # ordering is by adjacent lit pairs, and rung 4's first pattern has
    # none where its last has four.
    report.check('each rung is ordered smoothest first',
                 raster._spread(ord(rows[4][0]) - raster.BRAILLE)
                 < raster._spread(ord(rows[4][-1]) - raster.BRAILLE),
                 '%s then %s' % (rows[4][0], rows[4][-1]))
    report.check('a drawn cell is never blank, however dark',
                 wireframe._pattern(0, 0.0) != rows[0][0]
                 and wireframe._pattern(-3, 0.9) != rows[0][0])
    report.check('and never past the top',
                 wireframe._pattern(99, 0.0) == rows[8][0])

    # NO GRAIN. A per-cell phase picked among the 28 patterns that carry
    # six dots, uniformly and then cubed toward the even end, and either
    # way a flat face wore a different pattern in every cell - 107
    # distinct glyphs on the board's top at one pose against 79 with it
    # off, and the 79 are real edges. "Blocky", on the bench. A flat
    # surface is a flat pattern; the block is spent where the level
    # changes.
    phases = {wireframe._pattern(6, i / 32.0) for i in range(32)}
    report.check('a rung is one pattern whatever the phase',
                 phases == {rows[6][0]}, ''.join(sorted(phases)))

    # THE MONO LADDER IS THE CLASS SCALE, spread and in the exporter's
    # own order: his ' ', '.' and ':' rank the same way, only further
    # apart, because one rung between the two glyphs a picture is made of
    # is the carpet this replaces.
    dots = [bin(ord(wireframe._mono(float(c))) - raster.BRAILLE).count('1')
            for c in (0, 1, 2)]
    # Two and four, near the exporter's own luma; six read as a slab.
    report.check('mono keeps the exporter\'s ordering at his weight',
                 dots == [1, 2, 4], str(dots))


def test_the_alphabet(report):
    """`coaxial.braille`: the whole block, and the words to ask for one.

    HAND-PICKED GLYPHS STAY A HANDFUL AND THE CORNERS COME OUT WRONG. A
    run of dots ending against a column under it is two marks that happen
    to touch, and nobody notices until the drawing is read closely -
    `chr(0x28A4)` at a call site is a guess that has to be decoded before
    it can be reviewed. The alphabet is the fix: a cell is eight dots in
    two lanes, and the line-drawing names sit on top of that.
    """
    from coaxial import braille as b

    report.check('all 256 patterns, in order',
                 len(b.ALL) == 256 and len(set(b.ALL)) == 256
                 and b.ALL[0] == chr(0x2800) and b.ALL[255] == chr(0x28FF))
    report.check('a dot number and a coordinate name the same dot',
                 all(b.numbered(n) == b.glyph([at])
                     for n, at in b.AT.items()))
    report.check('what is read back is what was drawn',
                 all(b.glyph(b.lit(c)) == c for c in b.ALL))
    # THE BENCH ASKS IN DOT NUMBERS: `⠲` is 2, 5 and 6, and that is how
    # the corner arrived in the first place.
    report.check('the chart\'s own numbering answers the chart\'s glyph',
                 b.numbered(2, 5, 6) == chr(0x2832), b.numbered(2, 5, 6))

    report.check('a run is a horizontal on its dot row',
                 b.RUN == ('\u2809', '\u2812', '\u2824', '\u28c0'),
                 ''.join(b.RUN))
    report.check('a fall is a column in its own lane',
                 b.FALL == ('\u2847', '\u28b8'), ''.join(b.FALL))

    # A CORNER THE LINE ENDS AT IS A HOOK; one it falls THROUGH has to
    # reach the cell's floor or it breaks against the row below.
    report.check('a hook stops two dots along',
                 (b.corner(1, 0), b.corner(1, 1)) == ('\u2816', '\u2832'),
                 b.corner(1, 0) + b.corner(1, 1))
    report.check('and carrying on reaches the floor',
                 (b.corner(1, 0, through=True),
                  b.corner(1, 1, through=True)) == ('\u2856', '\u28b2'),
                 b.corner(1, 0, through=True)
                 + b.corner(1, 1, through=True))
    report.check('a tee is met, not turned',
                 (b.tee(0, 0), b.tee(0, 1)) == ('\u284f', '\u28b9'),
                 b.tee(0, 0) + b.tee(0, 1))
    # THE SAME TURN THE OTHER WAY: a run on dot row 2 climbing to
    # the cell's top in the far lane, for a leader that rises to
    # what it names instead of falling to it.
    report.check('a corner turning up mirrors one turning down',
                 b.corner(2, 1, up=True, through=True) == '\u283c',
                 b.corner(2, 1, up=True, through=True))
    report.check('a dot off the cell is not drawn',
                 b.glyph([(9, 9), (0, 0)]) == b.glyph([(0, 0)]))


def main():
    report = Report()
    print('\n-- the 3D engine, stage by stage --')
    test_pose(report)
    test_camera(report)
    test_shade_units(report)
    test_chain(report)
    test_outline(report)
    test_the_edge_is_the_rasters_silhouette(report)
    test_ink_never_leans_below_the_floor(report)
    test_the_decimate_keeps_the_bore(report)
    test_the_preload_is_adopted(report)
    test_the_art_stops_at_its_disc(report)
    test_the_outline_holds_together(report)
    test_stereotypes(report)
    test_key_light(report)
    test_the_face_is_a_halftone(report)
    test_triad(report)
    test_steady(report)
    test_the_face_is_held_while_the_pose_holds(report)
    test_the_crew_paints_one_pose_behind(report)
    test_scroll(report)
    test_fan_lines(report)
    test_backdrop_cache(report)
    test_ladder(report)
    test_the_alphabet(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
