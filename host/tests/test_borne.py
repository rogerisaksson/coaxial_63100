#!/usr/bin/env python3
"""What bears the robot off its soles in C (`tools.cores.borne`, wbc_borne.c) beside its python (`borne.read`, the robot's seat's
cone `emancipation.seated`): at every pass of its law lying into a bed from its end and walking, on the
robot's arrays, its support, palms and every contact bit for bit - each push turned to the world, its room,
its Jacobian and drift MuJoCo's own, the robot's seat's corners -, a read's time beside its python's."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The robot's lie into a bed from its end and the robot's rise off it, s; the robot's walk's passes.
LIE_S, RISE_S, WALK_PASSES = 12.0, 8.0, 9000


def test_what_bears_the_robot_in_c_as_in_python(report, c):
    """The robot's C read (`g['borne']`) and `borne.read` on the same arrays at every pass of its law: lying
    into the bed, several segments bearing it at once, and walking - its support's share and box, the
    robot's palms, every contact's point, push, Jacobian, drift, patch, side, room, body and give bit for bit; a
    read's time beside its python's."""
    from machine.moves import emancipation
    from machine.process import worker
    from machine.senses import borne
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    from tools.sim import told
    tally, spent = {'reads': 0, 'contacts': 0, 'apart': 0, 'most': 0, 'by': 0.0, 'unlike': 0, 'corners': 0}, ([], [])

    def held(g):
        real = g['borne']

        def beside(b, s, y, weight, hands=0.0, let_go=(0.0, 0.0), g=None):
            t0 = time.perf_counter()
            (wp, wc, wh), twin = borne.read(b, s, y, weight, hands, let_go), dict(g or {})
            if g is not None:
                seat = dict(s, contacts=wc)
                emancipation.seated(twin, seat)
                wc = seat['contacts']
            t1 = time.perf_counter()
            got = real(b, s, y, weight, hands, let_go, g)
            spent[0].append(time.perf_counter() - t1)
            spent[1].append(t1 - t0)
            gp, gc, gh = got
            seats = [(np.atleast_1d(twin[k]), np.atleast_1d(g[k])) for k in ('seat_on', 'seat_near')] if g is not None else []
            pairs = ([(np.atleast_1d(wp[0]), np.atleast_1d(gp[0])), (wp[1], gp[1]), (wp[2], gp[2]), (wh, gh)] + seats
                     + [(np.atleast_1d(a[k]), np.atleast_1d(x[k])) for a, x in zip(wc, gc) for k in a])
            alike = len(wc) == len(gc) and all(set(a) == set(x) for a, x in zip(wc, gc))
            ok = alike and all(same(p, q) for p, q in pairs)
            tally['unlike'] += not alike
            tally['by'] = max(tally['by'], max(float(np.abs(np.asarray(p, float) - np.asarray(q, float)).max())
                                               for p, q in pairs))
            tally['reads'] += 1
            tally['contacts'] += len(wc)
            tally['apart'] += not ok
            tally['most'] = max(tally['most'], len(wc))
            tally['corners'] += sum('corner' in c for c in wc)
            return got
        g['borne'] = beside
    r = told.Run('bed_foot_behind')
    held(r.s['g'])
    r.tell(say='lie_bed')
    r.go(LIE_S)
    r.tell(say='rise_bed')
    r.go(RISE_S)
    s = worker.session(walks=True)
    held(s['g'])
    for _ in range(WALK_PASSES):
        worker.stepped(s)
    line = "%d reads, %d contacts (%d at once at most, %d the robot's seat's corners), %d apart, %.3g at most" % (
        tally['reads'], tally['contacts'], tally['most'], tally['corners'], tally['apart'], tally['by'])
    if c_motion.exact_host():
        report.check("what bears the robot in C is its python bit for bit lying into a bed and walking, its seat's cone too",
                     tally['reads'] > 1000 and tally['most'] >= 3 and tally['corners'] > 0 and not tally['apart'], line)
    else:
        report.check("what bears the robot in C is its python within its last bits where numpy dispatches elsewhere",
                     tally['reads'] > 1000 and tally['most'] >= 3 and tally['corners'] > 0 and not tally['unlike']
                     and tally['by'] <= 1e-9, line)
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check('a read of what bears the robot in C on this host', us[0] < us[1], '%.1f us, its python %.1f' % tuple(us))


def test_the_robots_supports_in_c_as_in_python(report, c):
    """The robot's supports besides its soles in C on borne's contacts as read
    (`tools.cores.borne.supported`) and `emancipation.supports` on their dicts at every pass of its law
    lying into a bed, rising off it and walking: the robot's points, their rooms and each leg's own the same shapes
    and bits."""
    from machine.moves import emancipation
    from machine.process import worker
    from tools.sim import told
    tally = {'calls': 0, 'apart': 0, 'most': 0, 'felt': 0}

    def held(g):
        real = g['em_supports']

        def beside(s, obs, told_, floor, legs, loads, on=None):
            got = real(s, obs, told_, floor, legs, loads, on)
            twin = emancipation.supports(s, obs, told_, floor, legs, loads, on)
            tally['apart'] += any(a.shape != b.shape or a.dtype != b.dtype or a.tobytes() != b.tobytes()
                                  for a, b in zip(got, twin))
            tally['calls'] += 1
            tally['most'] = max(tally['most'], len(got[0]))
            tally['felt'] += int(len(got[0]) > sum(len(f['pts']) for f in s['feet']))
            return got
        g['em_supports'] = beside
    r = told.Run('bed_foot_behind')
    held(r.s['g'])
    r.tell(say='lie_bed')
    r.go(LIE_S)
    r.tell(say='rise_bed')
    r.go(RISE_S)
    s = worker.session(walks=True)
    held(s['g'])
    for _ in range(WALK_PASSES // 3):
        worker.stepped(s)
    report.check('its supports in C as in python lying, rising and walking: %(calls)d law passes (%(felt)d with more than '
                 'its soles, %(most)d points at most)' % tally,
                 tally['calls'] > 1000 and tally['felt'] > 100 and not tally['apart'], '%(apart)d apart' % tally)


def test_the_robots_lean_in_c_as_in_python(report, c):
    """What bears the robot but its hands must take of its weight and its hands' share sat in C on borne's contacts
    as read (`tools.cores.borne.leaned`) and `somatic.leaned` on their dicts at every pass of its law lying into a
    bed, rising off it and walking: both the same bits and types; its C's time beside its python's."""
    from machine.law import somatic
    from machine.process import worker
    from tools.sim import told
    tally, spent = {'calls': 0, 'apart': 0, 'borne': 0, 'hands': 0}, ([], [])

    def held(g):
        real = g['leaned']

        def beside(s, way, weight):
            t0 = time.perf_counter()
            got = real(s, way, weight)
            t1 = time.perf_counter()
            twin = somatic.leaned(s, way, weight)
            spent[0].append(t1 - t0)
            spent[1].append(time.perf_counter() - t1)
            tally['apart'] += any(type(a) is not type(b) or np.float64(a).tobytes() != np.float64(b).tobytes()
                                  for a, b in zip(got, twin))
            tally['calls'] += 1
            tally['borne'] += float(twin[0]) > 0.0
            tally['hands'] += float(twin[1]) not in (0.0, 1.0)
            return got
        g['leaned'] = beside
    r = told.Run('bed_foot_behind')
    held(r.s['g'])
    r.tell(say='lie_bed')
    r.go(LIE_S)
    r.tell(say='rise_bed')
    r.go(RISE_S)
    s = worker.session(walks=True)
    held(s['g'])
    for _ in range(WALK_PASSES // 3):
        worker.stepped(s)
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check('its lean in C as in python lying, rising and walking: %(calls)d law passes (%(borne)d with something '
                 'bearing it, %(hands)d with its hands sharing)' % tally,
                 tally['calls'] > 1000 and tally['borne'] > 100 and not tally['apart'],
                 '%d apart; %.1f us a call, its python %.1f' % ((tally['apart'],) + tuple(us)))


def test_the_robots_hull_in_c_as_in_python(report, c):
    """The hull of the robot's support's floor points in C (`tools.cores.borne.hulled`) and `barrier.hull` on draws
    of 0 to 40 points - a sole's corners, repeated points, points on a line, equal x - the same shapes and bits."""
    from machine.law import barrier
    from tools.cores import borne as c_borne
    hull, rng, apart, draws = c_borne.hulled(), np.random.default_rng(7), 0, 0
    for n in range(41):
        for kind in range(4):
            pts = rng.normal(size=(n, 2))
            if kind == 1 and n:
                pts = pts[rng.integers(0, n, n)]
            elif kind == 2:
                pts[:, 1] = 2.0 * pts[:, 0] + 0.5
            elif kind == 3:
                pts = np.round(pts, 1)
            got, want = hull(pts), barrier.hull(pts)
            apart += got.shape != want.shape or got.dtype != want.dtype or got.tobytes() != want.tobytes()
            draws += 1
    report.check('its support\'s hull in C as in python: %d draws' % draws, not apart, '%d apart' % apart)


ROSTER = (test_what_bears_the_robot_in_c_as_in_python, test_the_robots_supports_in_c_as_in_python,
          test_the_robots_lean_in_c_as_in_python, test_the_robots_hull_in_c_as_in_python)


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
