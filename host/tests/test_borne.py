#!/usr/bin/env python3
"""What bears her off her soles in C (`tools.cores.borne`, wbc_borne.c) beside its python (`borne.read`, her seat's
cone `emancipation.seated`): at every pass of her law lying into a bed from its end and walking, on her arrays, her
support, palms and every contact bit for bit - each push turned to the world, its room, its Jacobian and drift
MuJoCo's own, her seat's corners -, a read's time beside its python's."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: Her lie into a bed from its end and her rise off it, s; her walk's passes.
LIE_S, RISE_S, WALK_PASSES = 12.0, 8.0, 9000


def test_what_bears_her_in_c_as_in_python(report, c):
    """Her C read (`g['borne']`) and `borne.read` on the same arrays at every pass of her law: lying into the bed,
    several segments bearing her at once, and walking - her support's share and box, her palms, every contact's
    point, push, Jacobian, drift, patch, side, room, body and give bit for bit; a read's time beside its python's."""
    from machine.moves import emancipation
    from machine.process import worker
    from machine.senses import borne
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    from tools.sim import told
    tally, spent = {'reads': 0, 'contacts': 0, 'apart': 0, 'most': 0, 'by': 0.0, 'unlike': 0, 'corners': 0}, ([], [])

    def held(g):
        real = g['borne']

        def beside(b, s, y, weight, hands=0.0, let_go=(0.0, 0.0), dt=0.0, g=None):
            t0 = time.perf_counter()
            (wp, wc, wh), twin = borne.read(b, s, y, weight, hands, let_go, dt), dict(g or {})
            if g is not None:
                seat = dict(s, contacts=wc)
                emancipation.seated(twin, seat)
                wc = seat['contacts']
            t1 = time.perf_counter()
            got = real(b, s, y, weight, hands, let_go, dt, g)
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
    r = told.Run('bed_behind')
    held(r.s['g'])
    r.tell(say='lie_bed')
    r.go(LIE_S)
    r.tell(say='rise_bed')
    r.go(RISE_S)
    s = worker.session(walks=True)
    held(s['g'])
    for _ in range(WALK_PASSES):
        worker.stepped(s)
    line = "%d reads, %d contacts (%d at once at most, %d her seat's corners), %d apart, %.3g at most" % (
        tally['reads'], tally['contacts'], tally['most'], tally['corners'], tally['apart'], tally['by'])
    if c_motion.exact_host():
        report.check("what bears her in C is her python bit for bit lying into a bed and walking, her seat's cone too",
                     tally['reads'] > 1000 and tally['most'] >= 3 and tally['corners'] > 0 and not tally['apart'], line)
    else:
        report.check("what bears her in C is her python within its last bits where numpy dispatches elsewhere",
                     tally['reads'] > 1000 and tally['most'] >= 3 and tally['corners'] > 0 and not tally['unlike']
                     and tally['by'] <= 1e-9, line)
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check('a read of what bears her in C on this host', us[0] < us[1], '%.1f us, its python %.1f' % tuple(us))


ROSTER = (test_what_bears_her_in_c_as_in_python,)


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
