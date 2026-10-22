#!/usr/bin/env python3
"""Her law's parts ported to C by hand beside their python (`tools.cores.surety`, `touch`, `twist`,
`wait`, `brace`): each at every pass of her law over PASSES of her walk, on copies of her arrays,
bit for bit - her surety field, her touch's filter, her twist's step, her table's waits, her
surety's brace on the solve - and each step's time beside its python's. Where numpy's arithmetic
is not this C's (`motion.exact_host`), within the last bits. The tracer's parts: test_compiled."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The passes of her walk each part is held over, her law every third, four times: into her walk.
PASSES = 3000


def test_her_surety_in_c_as_she_walks(report, c):
    """Her surety field in C (`c_motion.surety_laid`, wbc_surety.c) beside its python
    (`motion.surety_field`) on her arrays at every call of her law over PASSES of her walk - her
    surety, its vector and its probe bit for bit -, and a step's time beside its python's."""
    from machine import motion, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, tally, spent = g['surety_field'], [0, 0, 0.0], [[], []]

    def beside(prev, dt, ct, t):
        t0 = time.perf_counter()
        want, want_vec = motion.surety_field(g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'], prev, dt, ct, t)
        t1 = time.perf_counter()
        cur, vec = real(prev, dt, ct, t)
        spent[0].append(time.perf_counter() - t1)
        spent[1].append(t1 - t0)
        tally[0] += 1
        tally[1] += not (same([cur], [want]) and same(vec, want_vec))
        tally[2] = max(tally[2], abs(cur - want), float(np.abs(vec - want_vec).max()))
        return cur, vec
    g['surety_field'] = beside
    for _ in range(4 * PASSES):
        worker.stepped(s)
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her surety field in C is her python's bit for bit at %d calls of her law walking" % n,
                     n >= 4 * PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her surety field in C is her python's within its last bits where numpy dispatches elsewhere",
                     n >= 4 * PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check('a step of her surety field in C on this host', us[0] < 20.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_touch_in_c_as_she_walks(report, c):
    """Her touch's filter in C (`c_touch.laid`, wbc_touch.c) beside its python (`touch.stepped`) on
    copies of her arrays at every pass of her law over PASSES of her walk - the latent out, the
    ask, the filter's state and her twist bit for bit -, and a step's time beside its python's."""
    from machine import edge, modal, touch, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, tally, spent = g['touched_step'], [0, 0, 0.0], [[], []]

    def beside(asked, twist, tw_on, heading, dt, vel):
        a, out, state = g['a'].copy(), g['out'].copy(), g['touched'].copy()
        soled = touch.soles(g['body'], g['sense'][edge.TOUCH].reshape(-1, 3), g['_skin_ids'], g['_tips'], g['y'],
                            g['fr'], g['a'][modal.STANCE])
        t0 = time.perf_counter()
        want = touch.stepped(g['sense'], vel, g['y'], g['fr'], a, g['st'], out, g['rest'], state, soled, asked, twist,
                             tw_on, heading, dt)
        t1 = time.perf_counter()
        got = real(asked, twist, tw_on, heading, dt, vel)
        spent[0].append(time.perf_counter() - t1)
        spent[1].append(t1 - t0)
        tally[0] += 1
        pairs = ((g['a'], a), (g['out'], out), (g['touched'], state), (np.array(got), np.array(want)))
        tally[1] += not all(same(x, y) for x, y in pairs)
        tally[2] = max(tally[2], max(float(np.abs(x - y).max()) for x, y in pairs))
        return got
    g['touched_step'] = beside
    for _ in range(4 * PASSES):
        worker.stepped(s)
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her touch's filter in C is her python's bit for bit at %d passes of her law walking" % n,
                     n >= PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her touch's filter in C is her python's within its last bits where numpy dispatches elsewhere",
                     n >= PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a step of her touch's filter in C on this host", us[0] < 30.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_twist_in_c_as_she_walks(report, c):
    """Her twist's step in C (`c_twist.laid`, wbc_twist.c) beside its python (`se2.twisted`) on
    copies of her arrays at every pass of her law over PASSES of her walk - fr, st, the latent
    out, her posture, the state and the words bit for bit -, and a step's time beside its python's."""
    from machine import se2, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, tally, spent = g['twist_step'], [0, 0, 0.0], [[], []]

    def beside(dt, vel, held, prior):
        copies = [g[name].copy() for name in ('fr', 'st', 'out', 'posture', 'twisted', 'words')]
        t0 = time.perf_counter()
        fr, st, out, posture, state, words = copies
        se2.twisted(g['sense'], vel, g['y'], fr, g['a'], st, out, posture, held, prior, state, words, dt, g['waist_idx'],
                    g['head_idx'])
        t1 = time.perf_counter()
        real(dt, vel, held, prior)
        spent[0].append(time.perf_counter() - t1)
        spent[1].append(t1 - t0)
        tally[0] += 1
        pairs = [(g[name], want) for name, want in zip(('fr', 'st', 'out', 'posture', 'twisted', 'words'), copies)]
        tally[1] += not all(same(x, y) for x, y in pairs)
        tally[2] = max(tally[2], max(float(np.nan_to_num(np.abs(x - y)).max()) for x, y in pairs))
    g['twist_step'] = beside
    for _ in range(4 * PASSES):
        worker.stepped(s)
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her twist's step in C is her python's bit for bit at %d passes of her law walking" % n,
                     n >= PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her twist's step in C is her python's within its last bits where numpy dispatches elsewhere",
                     n >= PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a step of her twist in C on this host", us[0] < 30.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_waits_in_c_as_she_walks(report, c):
    """Her table's waits in C (`c_wait.laid`, wbc_wait.c) beside their python (`waits.held_rows`,
    `risen_rows`, `landed_rows`, `seated_rows`, `woken_rows`) on copies of her table and her state at
    every pass of her law over PASSES of her walk and down onto a chair behind her - the table's
    times, the shift, the first row, the state and her rhythm woken bit for bit -, and a step's
    time."""
    from machine import latent, waits as py_waits, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    waits, tally, spent = g['waits'], [0, 0, 0.0], [[], []]

    class Beside:

        def held(self, tab, t, dt, held_s):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = py_waits.held_rows(mine, g['a'], g['out'], t, dt, held_s)
            t1 = time.perf_counter()
            got = waits.held(tab, t, dt, held_s)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

        def risen(self, tab, t, dt, prop_until):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = py_waits.risen_rows(mine, g['y'], g['fr'], g['a'], g['st'], t, dt, prop_until)
            t1 = time.perf_counter()
            got = waits.risen(tab, t, dt, prop_until)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

        def landed(self, tab, t, dt):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = py_waits.landed_rows(mine, g['y'], g['st'], t, dt)
            t1 = time.perf_counter()
            got = waits.landed(tab, t, dt)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

        def seated(self, tab, t, dt):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = py_waits.seated_rows(mine, g['y'], t, dt)
            t1 = time.perf_counter()
            got = waits.seated(tab, t, dt)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

        def woken(self, tab, t, dt):
            mine = g['st'].copy()
            t0 = time.perf_counter()
            py_waits.woken_rows(tab, mine, t, dt)
            t1 = time.perf_counter()
            waits.woken(tab, t, dt)
            self.tallied(t1 - t0, time.perf_counter() - t1, g['st'], mine, g['st'][latent.THETA], mine[latent.THETA])

        @staticmethod
        def tallied(py_s, c_s, tab, mine, got, want):
            spent[0].append(c_s)
            spent[1].append(py_s)
            tally[0] += 1
            tally[1] += not (same(tab, mine) and same(got, want))
            tally[2] = max(tally[2], float(np.nan_to_num(np.abs(np.asarray(got) - np.asarray(want))).max()),
                           float(np.abs(tab - mine).max()) if len(tab) else 0.0)
    g['waits'] = Beside()
    for _ in range(4 * PASSES):
        worker.stepped(s)
    worker.apply(s, {'scenario': 'chair_behind'})
    worker.apply(s, {'say': 'sit_chair'})
    for _ in range(2 * PASSES):
        worker.stepped(s)
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her table's waits in C are her python's bit for bit at %d calls of her law walking" % n,
                     n >= 3 * PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her table's waits in C are her python's within their last bits where numpy dispatches elsewhere",
                     n >= 3 * PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a wait in C on this host", us[0] < 20.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_brace_in_c_as_she_walks(report, c):
    """Her surety's brace on the solve in C (`c_brace.laid`, wbc_brace.c) beside its python
    (`motion.scaled`, `braced`) on copies at every pass of her law over PASSES of her walk - the
    solve's M, its answer blended, her drives' inertia and her legs' stiffness bit for bit -, and
    a blend's time beside its python's."""
    from machine import motion, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, body, tally, spent = g['braced'], g['body'], [0, 0, 0.0], [[], []]

    class Beside:

        def scaled(self, M, w_up):
            want = motion.scaled(M, g['_acv_sh'], w_up)
            got = real.scaled(M, w_up)
            tally[1] += not same(got, want)
            tally[2] = max(tally[2], float(np.abs(got - want).max()))
            return got

        def blend(self, M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance):
            mine = (qacc.copy(), tau.copy(), over.copy())
            t0 = time.perf_counter()
            want = motion.braced(M, Ms, q, v, h, posture, *mine, sw, bw, d_ax, d_lat, w_hop, w_up, stance, g['brace'],
                                 body.act, g['_acv_sh'], g['_cv_sh'], body.clamp, g['free_kp'], g['legs'])
            t1 = time.perf_counter()
            got = real.blend(M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance)
            spent[0].append(time.perf_counter() - t1)
            spent[1].append(t1 - t0)
            tally[0] += 1
            pairs = list(zip((qacc, tau, over.astype(float)) + tuple(got), mine[:2] + (mine[2].astype(float),) + tuple(want)))
            tally[1] += not all(same(x, y) for x, y in pairs)
            tally[2] = max(tally[2], max(float(np.abs(np.asarray(x) - np.asarray(y)).max()) for x, y in pairs))
            return got
    g['braced'] = Beside()
    for _ in range(4 * PASSES):
        worker.stepped(s)
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her brace in C is her python's bit for bit at %d passes of her law walking" % n,
                     n >= PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her brace in C is her python's within its last bits where numpy dispatches elsewhere",
                     n >= PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a blend of her brace in C on this host", us[0] < 40.0, '%.1f us, its python %.0f' % tuple(us))


ROSTER = (test_her_surety_in_c_as_she_walks,
          test_her_touch_in_c_as_she_walks,
          test_her_twist_in_c_as_she_walks,
          test_her_waits_in_c_as_she_walks,
          test_her_brace_in_c_as_she_walks)


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
