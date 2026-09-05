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
    test_key_light(report)
    test_triad(report)
    test_steady(report)
    test_scroll(report)
    test_ladder(report)
    test_the_alphabet(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
