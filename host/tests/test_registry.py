#!/usr/bin/env python3
"""The robot's map's look in C (`tools.cores.registry`, wbc_registry.c) beside its numpy twin (`registry.laid(twin=True)`):
looks drawn to take every path - each thing its own slot at once, the loop where two meet one, a full registry's
least salient taken, things moving, unseen, pruned, the robot's eyes' frustum or the whole room - and the robot's
lodge's own looks walking out of its door, as the robot's process passes them and as its vision apart does
(its changed rows restacked alone), every slot's every field and class bit for bit after each; a look's time beside
numpy's. A look's rows that moved said alone and patched into the robot's things as the look laid whole."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The draws' looks and their seed; the kinds they are of; the robot's door errand's seconds.
LOOKS, SEED, DOOR_S = 600, 17, 16.0
KINDS = ('chair', 'table', 'lamp', 'bed', 'glass', 'box', 'wall', 'door')


def _same(c, twin):
    """The fields of the C's slots and the twin's apart, bit for bit, and whether their classes match: (names, ok)."""
    a, b = c['rows'], twin['rows']
    apart = [name for name in a if a[name].tobytes() != b[name].tobytes()]
    return apart, c['kinds'] == twin['kinds']


def _turn(rng):
    q, _r = np.linalg.qr(rng.standard_normal((3, 3)))
    return q * np.sign(np.linalg.det(q))


def _head(rng):
    yaw, pitch = rng.uniform(-math.pi, math.pi), rng.uniform(-0.3, 0.3)
    w, x, y, z = (math.cos(pitch / 2) * math.cos(yaw / 2), math.sin(pitch / 2) * math.cos(yaw / 2),
                  math.cos(pitch / 2) * math.sin(yaw / 2), -math.sin(pitch / 2) * math.sin(yaw / 2))
    return {'pos': np.array([rng.uniform(-2, 2), 1.5, rng.uniform(-2, 2)]), 'quat': np.array([w, x, y, z])}


def _thing(rng, k):
    """A thing drawn: its kind, centre, half sizes, rotation, its id (none for a third) and whether over it is clear."""
    return [KINDS[k % len(KINDS)], rng.uniform(-4, 4, 3) * (1.0, 0.3, 1.0) + (0.0, 0.6, 0.0), rng.uniform(0.03, 0.8, 3),
            _turn(rng), int(k + 1) if k % 3 else 0, float(rng.integers(2))]


def _drawn(rng, things, t):
    """The things as a look sees them `t` on: some moved a little, a few a lot, some unseen, some new; a twin of one
    beside it at times (the loop's path)."""
    for th in things:
        th[1] = th[1] + rng.normal(0.0, 0.004, 3) + (rng.normal(0.0, 0.3, 3) if rng.random() < 0.02 else 0.0)
        th[2] = np.abs(th[2] + rng.normal(0.0, 0.002, 3)) + 0.01
    if rng.random() < 0.3:
        things.append(_thing(rng, len(things) + int(1e3 * t)))
    look = [tuple(th) for th in things if rng.random() > 0.15]
    if look and rng.random() < 0.2:
        kind, at, half, R, _i, clear = look[int(rng.integers(len(look)))]
        look.append((kind, at + rng.normal(0.0, 0.01, 3), half, R, 0, clear))
    return look


def test_the_robots_map_looks_in_c_as_in_numpy(report, c):
    """Drawn looks through both, passed and seen in turn: every field and class bit for bit after each - the loop's,
    a full registry's eviction and the frustum's paths all taken."""
    from machine.eyes import registry
    rng = np.random.default_rng(SEED)
    got, twin = registry.laid(), registry.laid(twin=True)
    things, t, worst, paths = [_thing(rng, k) for k in range(40)], 0.0, [], {'full': 0, 'moving': 0, 'pruned': 0}
    for k in range(LOOKS):
        dt = float(rng.choice((0.009, 0.05, 0.4)))
        t += dt
        head, everywhere = _head(rng), bool(k % 3 == 0)
        if k == LOOKS // 2:
            things = [_thing(rng, 5000 + j) for j in range(registry.SLOTS)]
        look = _drawn(rng, things, t)[:registry.SLOTS]
        for slots in (got, twin):
            registry.passed(slots, head, dt)
            registry.seen(slots, look, head, t, everywhere=everywhere)
        apart, alike = _same(got, twin)
        if apart or not alike:
            worst.append((k, apart, alike))
        r = twin['rows']
        paths['full'] += int(r['on'].sum()) == registry.SLOTS
        paths['moving'] += int((r['moving'] > 0).sum() > 0)
        paths['pruned'] += int(((r['on'] == 0) & (r['salience'] != 0)).sum() > 0)
    report.check('its map looks in C as in numpy, every field and class bit for bit after each of %d looks' % LOOKS,
                 not worst and all(paths.values()), 'apart %s; looks full %d, a thing moving %d, pruned %d'
                 % (worst[:3], paths['full'], paths['moving'], paths['pruned']))


def test_the_robots_lodge_looks_in_c_as_in_numpy(report, c):
    """The robot's lodge's looks as it walks out of its door (`told`), the robot's registry's C and a numpy
    twin fed each, and a third fed as its vision apart is (`vision.main`: the rows not the same objects as the
    last look's, kept and restacked alone): every field and class bit for bit after each look; a look's time beside
    numpy's."""
    from machine.eyes import registry, vision
    from machine.world import room
    from tools.sim import told
    twins, spent, worst = {}, ([], []), []
    start = vision.Here.start
    changer = vision.Apart.__new__(vision.Apart)
    changer._looks = []

    def beside(self, g, looks, head, dt, everywhere, at=None):
        twin, apart_ = twins.get(id(g['registry']), (None, None))
        if twin is None:
            twin, apart_ = twins[id(g['registry'])] = registry.laid(twin=True), (registry.laid(), {'looks': []})
        t = g['t'] if at is None else at
        t0 = time.perf_counter()
        start(self, g, looks, head, dt, everywhere, at)
        t1 = time.perf_counter()
        registry.passed(twin, head, dt)
        registry.seen(twin, looks, head, t, everywhere=everywhere)
        t2 = time.perf_counter()
        changed, (slots, kept) = changer._changed(looks), apart_
        kept['looks'] = kept['looks'] if len(kept['looks']) == len(looks) else [None] * len(looks)
        for k, look in changed.items():
            kept['looks'][k] = look
        registry.passed(slots, head, dt)
        registry.seen(slots, kept['looks'], head, t, everywhere=everywhere, kept=kept, changed=changed)
        spent[0].append(t1 - t0)
        spent[1].append(t2 - t1)
        apart, alike = _same(g['registry'], twin)
        restacked, kinds = _same(slots, twin)
        if apart or not alike or restacked or not kinds:
            worst.append((round(g['t'], 3), apart, alike, restacked, kinds))
    vision.Here.start = beside
    try:
        r = told.Run(room=True)
        r.tell(say='exit_door')
        r.go(DOOR_S, lambda q: float(q.s['g']['world'].data.qpos[2]) > room.TO)
    finally:
        vision.Here.start = start
    report.check("the robot's lodge's looks in C as in numpy walking out of its door, passed whole and restacked as the robot's vision "
                 "apart's, every field and class bit for bit",
                 len(spent[0]) > 400 and not worst, '%d looks, apart %s' % (len(spent[0]), worst[:3]))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a look of the robot's lodge in C on this host", us[0] < us[1], '%.0f us a look, numpy %.0f' % tuple(us))


def _bits(things):
    """Every number of `things` (`registry.things`' laid) as bytes, row by row and in its arrays, with its kinds and
    slots: two the same to the bit, a zero's sign too."""
    rows = [(t[0], t[4], np.array([*t[1], *t[2], *np.ravel(t[3]), t[5], t[6]], float).tobytes()) for t in things]
    return rows, tuple(a.tobytes() for a in things.arrays), [type(x) for t in things for x in t[1]]


def test_a_looks_rows_patched_as_laid_whole(report, c):
    """The robot's vision apart says a look's rows that moved alone (`vision.ROWS`): `registry.moved` finds every row
    whose numbers are not the last look's to the bit - a zero's sign flipped among them - and `registry.patched` lays
    the robot's things from them as `registry.things` lays the look whole, every number and type to the bit, the other
    rows the same tuples; a look of other things or slots told whole (None)."""
    from machine.eyes import registry
    rng, worst, told = np.random.default_rng(SEED), [], 0
    for _ in range(200):
        n = int(rng.integers(1, 90))
        was = (['k%d' % int(k) for k in rng.integers(0, 6, n)], rng.normal(size=(n, 3)), rng.uniform(0.01, 1, (n, 3)),
               rng.normal(size=(n, 3, 3)), list(range(n)), rng.integers(0, 2, n).astype(float).tolist(),
               rng.uniform(0, 0.1, n).tolist())
        now = [list(was[0]), was[1].copy(), was[2].copy(), was[3].copy(), list(was[4]), list(was[5]), list(was[6])]
        rows = sorted(set(rng.integers(0, n, int(rng.integers(0, 4))).tolist()))
        for k in rows:
            field = int(rng.integers(0, 5))
            if field == 0:
                now[1][k, int(rng.integers(3))] += 1e-12
            elif field == 1:
                now[2][k] *= 1.0 + 1e-15
            elif field == 2:
                now[3][k, 1, 2] = -now[3][k, 1, 2] if now[3][k, 1, 2] else -0.0
            else:
                now[5 + field - 3][k] = -0.0 if not now[5 + field - 3][k] else now[5 + field - 3][k] * 2.0
        now[3][rows[:1], 0, 0] = 0.0
        was[3][rows[:1], 0, 0] = -0.0
        found = registry.moved(was, now)
        assert found is not None, 'the same things and slots told whole'
        laid = registry.patched(registry.things(*was), found, now[1][found], now[2][found], now[3][found],
                                [now[5][k] for k in found], [now[6][k] for k in found])
        whole = registry.things(*now)
        told += len(found)
        if found.tolist() != rows or _bits(laid) != _bits(whole):
            worst.append((n, rows, found.tolist()))
    others = (registry.moved((['other'] * len(now[0]), *was[1:]), now),
              registry.moved((*was[:4], [k + 1 for k in was[4]], *was[5:]), now))
    report.check("a look's rows that moved found to the bit and patched as the look laid whole (%d rows over 200 looks), "
                 "a look of other things or slots told whole" % told, not worst and others == (None, None),
                 '%d apart: %s; %s' % (len(worst), worst[:2], others))


def test_c11_vision_and_way_bit_exact_parity(report, _c):
    """C11 `vision.c` epipolar warp, stereo SGM disparity, 3D voxel raycast/scatter, and `way.c` 8-way A* match twins."""
    from machine.eyes import epipolar, gimbal, grids, stereo, voxels
    from tools.cores import vision as vision_core, way as way_core
    rng = np.random.default_rng(77)
    fine_l = rng.integers(0, 256, (epipolar.FINE * stereo.H, epipolar.FINE * stereo.W, 3), dtype=np.uint8)
    fine_r = np.roll(fine_l, -28, axis=1)
    laid = epipolar.Laid(gimbal.eye('left', (0.12, -0.08)), gimbal.eye('right', (0.10, -0.08)), xp=np)
    a_c, b_c = laid._warped(fine_l, 0, in_c=True), laid._warped(fine_r, 1, in_c=True)
    a_py, b_py = laid._warped(fine_l, 0, in_c=False), laid._warped(fine_r, 1, in_c=False)
    warp_same = np.array_equal(a_c, a_py) and np.array_equal(b_c, b_py)
    got_d, got_k = stereo.cpu_disparity(a_c, b_c)
    twin_d, twin_k = stereo.cpu_disparity_twin(a_py, b_py)
    disp_same = np.array_equal(got_d.view(np.uint64), twin_d.view(np.uint64)) and np.array_equal(got_k, twin_k)

    pal = {'wood': (180, 140, 90), 'wall': (210, 210, 210)}
    m_c, m_py = voxels.Map(pal), voxels.Map(pal)
    vox_same = True
    for _ in range(4):
        xyz = rng.uniform([-2.5, 0.05, -1.5], [2.5, 2.2, 4.0], (6000, 3))
        rgb = rng.integers(0, 256, (6000, 3), dtype=np.uint8)
        eye = rng.uniform([-0.5, 1.3, -0.5], [0.5, 1.6, 0.5], 3)
        f_c, f_py = m_c.add(xyz, rgb, eye), m_py.add_twin(xyz, rgb, eye)
        vox_same = (vox_same and np.array_equal(f_c, f_py)
                    and np.array_equal(m_c.hits, m_py.hits)
                    and np.array_equal(m_c.told, m_py.told)
                    and np.array_equal(m_c.free, m_py.free)
                    and np.array_equal(m_c.top.view(np.uint64), m_py.top.view(np.uint64))
                    and np.array_equal(m_c.moments.view(np.uint64), m_py.moments.view(np.uint64))
                    and np.array_equal(m_c.colour.view(np.uint32), m_py.colour.view(np.uint32)))
    bad_xyz = np.array([[np.nan, 1.0, 1.0], [1.0, np.inf, 1.0], [1.0, 1.0, -np.inf]])
    bad_rgb = np.zeros((3, 3), dtype=np.uint8)
    hits_before = m_c.hits.copy()
    bad_flat = m_c.add(bad_xyz, bad_rgb, np.array([0.0, 1.4, 0.0]))
    vox_same = vox_same and np.array_equal(bad_flat, np.full(3, -1)) and np.array_equal(m_c.hits, hits_before)
    astar_off = 0
    for _ in range(40):
        grid = rng.uniform(1.0, 5.0, (32, 32))
        grid[rng.random((32, 32)) < 0.25] = np.inf
        s_pt, g_pt = (int(rng.integers(0, 32)), int(rng.integers(0, 32))), (int(rng.integers(0, 32)), int(rng.integers(0, 32)))
        grid[s_pt], grid[g_pt] = 1.0, 1.0
        astar_off += grids.astar(grid, s_pt, g_pt) != grids.astar_twin(grid, s_pt, g_pt)
    report.check('C11 vision.c warp, SGM, voxel raycast/scatter, and way.c A* match twins bit-for-bit',
                 vision_core.ensure() is not None and way_core.ensure() is not None
                 and warp_same and disp_same and vox_same and astar_off == 0,
                 'warp=%s disp=%s vox=%s astar_off=%d' % (warp_same, disp_same, vox_same, astar_off))
    from machine.way import obstacles, route
    pts_s = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]])
    up_r = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    th_s = [('chair', (10.0, 0.5, 10.0), (0.3, 0.5, 0.3), up_r, 0, 1.0)]
    c_scalar = obstacles.costed(pts_s, th_s, 0.0, seen=1.0)
    c_vec = obstacles.costed(pts_s, th_s, 0.0, seen=np.ones(len(pts_s)))
    c_nan = way_core.costed(
        np.array([[0.0, 0.0]]), np.array([[2.0, 0.0], [0.0, 10.0]]),
        np.array([[[1.0, 0.0], [0.0, 1.0]], [[1.0, 0.0], [0.0, 1.0]]]),
        np.array([[0.5, 0.5], [np.nan, 0.5]]), np.array([0.5, 0.5]),
        None, (True, 1.0, 0.5), (False, 0.0), np.array([1.0]), True,
    )
    route._grid()
    report.check('way.costed broadcasts scalar seen, preserves NaN half-extents, and route._GRID is a tuple',
                 np.array_equal(c_scalar, c_vec) and c_nan is not None and bool(np.isnan(c_nan[0]))
                 and isinstance(route._GRID, tuple),
                 'scalar_eq=%s nan=%s grid=%s' % (np.array_equal(c_scalar, c_vec), c_nan, type(route._GRID).__name__))
    bad_d = np.full((4, 4, 3), -1.0, dtype=np.float64)
    bad_d[0, 0, 0], bad_d[0, 1, 1], bad_d[0, 2, 0] = np.nan, np.inf, -np.inf
    epi_got = vision_core.epi_from_d(bad_d, 4, 4, 100.0, 5, 2)
    assert epi_got is not None
    (ev, eu), eon, _ = epi_got
    kept_c = laid.kept(got_d)
    inner = laid._inner
    assert inner is not None
    rows_e, cols_e = stereo.indices(np)
    kept_py = inner[0] & inner[1][rows_e, np.clip(np.rint(cols_e - got_d).astype(np.int64), 0, stereo.W - 1)]
    ro_ok = (not laid.E.flags.writeable and not laid.on[0].flags.writeable
             and not laid.at[0][0].flags.writeable and not inner[0].flags.writeable)
    report.check('epi_from_d rejects non-finite rays cleanly, epi_kept matches numpy, and Laid cache is read-only',
                 not bool(eon[0, 0]) and not bool(eon[0, 1]) and not bool(eon[0, 2])
                 and int(ev[0, 0]) == 0 and int(eu[0, 0]) == 0
                 and np.array_equal(kept_c, kept_py) and ro_ok,
                 'on=%s kept_eq=%s ro=%s' % (eon[0, :3].tolist(), np.array_equal(kept_c, kept_py), ro_ok))
    mixed_pts = [(0, 0), (2.5, 0.0), (2.5, 2.5), (0.0, 2.5), (1.0, 1.0), (1.2, 1.3), (0.5, 0.5), (1.5, 1.5)]
    h_mixed = grids.convex(mixed_pts)
    h_set = grids.convex({(i, i * i) for i in range(10)})
    h_gen = grids.convex((i, i * i) for i in range(10))
    big_x = (1 << 55) + 1
    h_big = grids.convex([(0.0, 0.0), (big_x, 0), (big_x, 1), (0, 10), (1, 1), (2, 2), (3, 3), (4, 4)])
    bad_shape = vision_core.convex_f64(np.zeros((5, 3)))
    report.check('grids.convex handles iterables, mixed int/float, large ints, and invalid shapes safely',
                 h_mixed == [(0, 0), (2.5, 0.0), (2.5, 2.5), (0.0, 2.5)] and h_set == h_gen and len(h_set) == 10
                 and (big_x, 0) in h_big and bad_shape is None,
                 'mixed=%s set_gen=%s big=%s bad=%s' % (h_mixed, h_set == h_gen, (big_x, 0) in h_big, bad_shape))


ROSTER = (test_c11_vision_and_way_bit_exact_parity, test_the_robots_map_looks_in_c_as_in_numpy,
          test_the_robots_lodge_looks_in_c_as_in_numpy, test_a_looks_rows_patched_as_laid_whole)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
