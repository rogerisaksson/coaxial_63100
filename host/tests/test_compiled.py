#!/usr/bin/env python3
"""Her numpy compiled to C beside numpy's own (`tools.cores.trace`, `emit`, `held`, `bodied`).

Her sensing on her body over random configurations - every array it returns, her data and her
scratch after, bit for bit, its time and numpy's - and at every pass of her law as she walks.
The regressor's: test_sysid. Where numpy's arithmetic is not this C's (`law.exact_host`), what
is not held stays numpy's own."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Configurations her sensing is held at; the passes of her walk it is held over, her law every third.
POSES, PASSES = 40, 3000
#: Her data's arrays and her body's scratch compared after each.
DATA = ('qpos', 'qvel', 'xpos', 'xmat', 'xquat', 'cdof', 'qfrc_bias', 'qfrc_passive', 'subtree_com')
SCRATCH = ('_M', '_jp', '_jr', '_am')


def _configurations(b, rng):
    """(qpos, qvel): a free joint anywhere and turned anyhow, a hinge within its range, her speeds drawn."""
    m, free = b.m, int(b.mj.mjtJoint.mjJNT_FREE)
    for _ in range(POSES):
        q, v = np.zeros(m.nq), np.zeros(m.nv)
        for j in range(m.njnt):
            at = m.jnt_qposadr[j]
            if m.jnt_type[j] == free:
                quat = rng.normal(size=4)
                q[at:at + 3], q[at + 3:at + 7] = rng.normal(size=3), quat / np.linalg.norm(quat)
            else:
                lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-1.5, 1.5)
                q[at] = rng.uniform(lo, hi)
        v[:b.n] = rng.normal(size=b.n)
        yield q, v


def _apart(got, want):
    from tools.cores.orders import same
    return any(not same(got[k], want[k]) for k in want)


def test_her_sensing_in_c_is_her_numpys(report, c):
    """`wbc.sense` compiled on her body beside its numpy on a body of its own, over random
    configurations: every array it returns, her data and her scratch after, bit for bit."""
    from machine import wbc
    from tools.cores import bodied, law, trace, trace_mj
    from tools.cores.orders import same
    b = c['body']
    twin, configurations = trace_mj.twin(b), list(_configurations(b, np.random.default_rng(11)))
    sense = bodied.on_body(wbc.sense, 'sense')
    try:
        shape = sense.wait(b, *configurations[0]).prog.shape
    except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
        report.check("her sensing in C held, or numpy's own here", not law.exact_host(), str(e)[:200])
        return
    bad = 0
    for q, v in configurations:
        bad += _apart(trace.leaves_of(shape, sense(b, q, v)), trace.leaves_of(shape, wbc.sense(twin, q, v)))
        bad += any(not same(getattr(b.d, x), getattr(twin.d, x)) for x in DATA)
        bad += any(not same(getattr(b, x), getattr(twin, x)) for x in SCRATCH)
    spent = {}
    for f, key in ((sense, 'C'), (wbc.sense, 'numpy')):
        t = time.perf_counter()
        for q, v in configurations * 25:
            f(b, q, v)
        spent[key] = 1e6 * (time.perf_counter() - t) / (25 * len(configurations))
    report.check("her sensing in C is her numpy's bit for bit over %d configurations" % len(configurations),
                 not bad, "%d apart; %.0f us a call, numpy's %.0f" % (bad, spent['C'], spent['numpy']))


def test_her_sensing_in_c_as_she_walks(report, c):
    """Her law's own sensing (`law.laid`'s) at every pass of her law over PASSES of her walk, built
    at its first and held there, beside `wbc.sense` on a twin of her law's body."""
    from machine import wbc, worker
    from tools.cores import law, trace, trace_mj
    s = worker.session(walks=True)
    g = s['g']
    real, twin, tally = g['sensed'], trace_mj.twin(g['body']), [0, 0, None]

    def sensed(b, qpos, qvel):
        if real.c is None and real.why is None:
            try:
                real.wait(b, qpos, qvel)
            except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
                tally[2] = e
        got = real(b, qpos, qvel)
        if real.c is not None:
            shape = real.c.prog.shape
            tally[0] += 1
            tally[1] += _apart(trace.leaves_of(shape, got), trace.leaves_of(shape, wbc.sense(twin, qpos, qvel)))
        return got
    g['sensed'] = sensed
    for _ in range(PASSES):
        worker.stepped(s)
    if tally[2] is not None:
        report.check("her sensing in C held, or numpy's own here", not law.exact_host(), str(tally[2])[:200])
        return
    report.check("her sensing in C is her numpy's bit for bit at %d of her law's passes walking" % tally[0],
                 tally[0] >= PASSES // 3 - 1 and not tally[1], '%d apart' % tally[1])


ROSTER = (test_her_sensing_in_c_is_her_numpys, test_her_sensing_in_c_as_she_walks)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    from machine import wbc
    report = Report()
    c = {'body': wbc.Body()}
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
