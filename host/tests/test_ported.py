#!/usr/bin/env python3
"""Her law's parts ported to C by hand beside their python (`tools.cores.surety`, `touch`, `twist`,
`wait`, `brace`): each at every pass of her law over PASSES of her walk, on copies of her arrays,
bit for bit - her surety field, her touch's filter, her twist's step, her table's waits, her
surety's brace on the solve - and each step's time beside its python's; her table's wait while a move plays
on her bed's rise as said; her way out of a bed on senses drawn at random; her parts' leaves laid in
C as Python lays them (`tools.cores.quick`). Where numpy's arithmetic is not this C's
(`motion.exact_host`), within the last bits. The tracer's parts: test_compiled."""
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
    (`surety.field`) on her arrays at every call of her law over PASSES of her walk - her
    surety, its vector and its probe bit for bit -, and a step's time beside its python's."""
    from machine.law import surety
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, tally, spent = g['surety_field'], [0, 0, 0.0], [[], []]

    def beside(prev, dt, ct, t):
        t0 = time.perf_counter()
        want, want_vec = surety.field(g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'], prev, dt, ct, t)
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
                     n >= PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her surety field in C is her python's within its last bits where numpy dispatches elsewhere",
                     n >= PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check('a step of her surety field in C on this host', us[0] < 20.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_touch_in_c_as_she_walks(report, c):
    """Her touch's filter in C (`c_touch.laid`, wbc_touch.c) beside its python (`touch.stepped`) on
    copies of her arrays at every pass of her law over PASSES of her walk - the latent out, the
    ask, the filter's state and her twist bit for bit -, and a step's time beside its python's."""
    from machine.senses import edge
    from machine.law import modal
    from machine.senses import touch
    from machine.process import worker
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
    copies of her arrays at every pass of her law over PASSES of her walk told slower and turning, each
    bit for bit, and a step's time beside its python's."""
    from machine.law import se2
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    worker.apply(s, {'twist': (0.35, 0.0, 0.3)})
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
    """Her table's waits in C (`c_wait.laid`, wbc_wait.c: a law pass's in one call, `backs`) beside their python
    (`waits.held_rows`, `risen_rows`, `landed_rows`, `sunk_rows`, `borne_rows`, `latched_rows` in that order,
    `woken_rows`) on copies of her table and her state at every pass over PASSES of her walk - the table's times,
    each one's shift and first row, the state and her rhythm woken bit for bit -, and a wait's time."""
    from machine.law import latent, waits as py_waits
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    waits, tally, spent = g['waits'], [0, 0, 0.0], [[], []]

    class Beside:

        def backs(self, tab, t, dt, held_s, prop_until):
            mine, st = tab.copy(), g['st'].copy()
            t0 = time.perf_counter()
            want = [py_waits.held_rows(mine, g['a'], g['out'], t, dt, held_s),
                    py_waits.risen_rows(mine, g['y'], g['fr'], g['a'], g['st'], t, dt, prop_until),
                    py_waits.landed_rows(mine, g['y'], g['st'], t, dt), py_waits.sunk_rows(mine, g['y'], g['fr'], t, dt),
                    py_waits.borne_rows(mine, g['y'], g['fr'], t, dt)]
            py_waits.latched_rows(mine, g['y'], g['fr'], g['a'], st, t, dt)
            t1 = time.perf_counter()
            got = waits.backs(tab, t, dt, held_s, prop_until)
            py_s, c_s = (t1 - t0) / 6.0, (time.perf_counter() - t1) / 6.0
            for pair, each in zip(got[0], want):
                self.tallied(py_s, c_s, tab, mine, pair, each[:2])
            self.tallied(py_s, c_s, g['st'], st, got[1:], (want[0][2], want[1][2]))
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
    n, bad, apart = tally
    if c_motion.exact_host():
        report.check("her table's waits in C are her python's bit for bit at %d calls of her law walking" % n,
                     n >= 3 * PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her table's waits in C are her python's within their last bits where numpy dispatches elsewhere",
                     n >= 3 * PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a wait in C on this host", us[0] < 20.0, '%.1f us, its python %.0f' % tuple(us))


def test_her_played_wait_in_c_as_said(report, c):
    """Her table's wait while a move of hers plays in C (`c_wait`'s `played`, wbc_wait.c) beside its python
    (`waits.played_rows`) on copies of her bed's rise as said (`maneuver.entry`), a pass's wait every two passes
    of her clock over its own length and a second, its rows coming due one by one: the table's times, the shift
    and the time after which rows wait bit for bit."""
    from machine.law import latent, se2, waits as py_waits
    from machine.process import worker
    from machine.texts import maneuver
    from tools.cores.orders import same
    s = worker.session()
    g = s['g']
    wire = maneuver.entry('rise_bed')[1]
    tab, dt, bad, n = latent.table(wire, 0.0, g['rest']), 1e-3, 0, 0
    for k in range(int((se2.rise_lead(wire) + 1.0) / dt)):
        mine = tab.copy()
        want = py_waits.played_rows(mine, 2.0 * k * dt, dt)
        got = g['waits'].played(tab, 2.0 * k * dt, dt)
        n, bad = n + 1, bad + (not (same(tab, mine) and same(got, want)))
    report.check("her table's wait while a move plays in C is her python's bit for bit at %d passes" % n,
                 n > 1000 and not bad, '%d apart' % bad)


def test_her_bed_exit_in_c_as_in_python(report, c):
    """Her way out of a bed in C (`tools.cores.bedexit`, wbc_bedexit.c) beside its python (`machine.moves.bedexit`)
    from poses and senses drawn at random - her pose, her soles' and shanks' shares, her trunk's pitch - through every
    phase to its seat: the move's state, her joints' command and its end bit for bit at every pass."""
    from machine.moves import bedexit as py
    from tools.cores import bedexit as c_exit
    from tools.cores.orders import same
    rng = np.random.default_rng(17)
    passes, bad, phases = 0, 0, set()
    for _ in range(12):
        q, t = rng.uniform(-60.0, 60.0, len(py.JOINTS)), 0.0
        mine, theirs, cmd_py, cmd_c = py.laid(q, t), c_exit.laid(q, t), q.copy(), q.copy()
        bad += not same(mine, theirs)
        for n in range(40000):
            t += 1e-3 * (1 + (n % 3 == 0))
            sensed, soles, shins, pitch = (rng.uniform(40.0, 160.0, len(py.JOINTS)), rng.uniform(0.0, 1.0),
                                           rng.uniform(0.0, 0.05), rng.uniform(-40.0, 10.0))
            want = py.step(mine, cmd_py, sensed, t, 1e-3, soles, shins, pitch)
            got = c_exit.step(theirs, cmd_c, sensed, t, 1e-3, soles, shins, pitch)
            passes, bad = passes + 1, bad + (not (same(mine, theirs) and same(cmd_py, cmd_c) and got == want))
            phases.add(int(mine[py.PHASE]))
            if want:
                break
    report.check("her bed exit in C is her python's bit for bit at %d passes through its %d phases" % (passes, len(phases)),
                 not bad and len(phases) == len(py.PHASES), '%d apart' % bad)


def test_her_scoot_in_c_as_in_python(report, c):
    """Her scoot on the floor in C (`tools.cores.scoot`, wbc_scoot.c) beside its python (`machine.moves.scoot`) from
    poses and senses drawn at random - her trunk's pitch, her fists' and feet's shares, her heading across its wrap,
    her twist held, turned and let go every 2 s -: the move's state and her joints' command bit for bit at every pass,
    through every phase."""
    from machine.moves import scoot as py
    from tools.cores import scoot as c_scoot
    from tools.cores.orders import same
    rng = np.random.default_rng(23)
    passes, bad, phases, keys = 0, 0, set(), (-1.0, 0.0, 1.0)
    for _ in range(8):
        q = rng.uniform(-60.0, 60.0, len(py.JOINTS))
        mine, theirs, cmd_py, cmd_c = py.laid(q, 0.0), c_scoot.laid(q, 0.0), q.copy(), q.copy()
        bad, back, turn = bad + (not same(mine, theirs)), 0.0, 0.0
        for n in range(20000):
            back, turn = (rng.choice(keys), rng.choice(keys)) if n % 2000 == 0 else (back, turn)
            hands, left, right = rng.uniform(0.0, 0.3, 3)
            sensed = (rng.uniform(-40.0, -16.0), hands, left, right, back, turn, rng.uniform(-180.0, 180.0))
            dt = 1e-3 * (1 + (n % 3 == 0))
            want = py.step(mine, cmd_py, q, 0.0, dt, *sensed)
            got = c_scoot.step(theirs, cmd_c, q, 0.0, dt, *sensed)
            passes, bad = passes + 1, bad + (not (same(mine, theirs) and same(cmd_py, cmd_c) and got == want))
            phases.add(int(mine[py.PHASE]))
    report.check("her scoot in C is her python's bit for bit at %d passes through its %d phases" % (passes, len(phases)),
                 not bad and len(phases) == len(py.PHASES), '%d apart' % bad)


def test_her_bed_entry_in_c(report, c):
    """Her sensed moves in C (wbc_sensed.c) as in python, pass by pass (`bed-entry`, `chair-mount`, `chair-dismount`)."""
    from machine.moves import sensed as py
    from tools.cores import sensed as ce
    from tools.cores.orders import same
    rng = np.random.default_rng(29)
    for name in ('bed-entry', 'chair-mount', 'chair-dismount'):
        pm, cm = (m.move(name) for m in (py, ce))
        bad, phases = 0, set()
        for _ in range(12):
            q, (half, side, pace) = rng.uniform(-60, 60, len(py.JOINTS)), rng.uniform((0.8, -1, 0.05), (1.2, 1, 0.12))
            ps, cs, a, b = pm.laid(q, 0.0), cm.laid(q, 0.0), q.copy(), q.copy()
            for n in range(1, 40000):
                along = np.sign(side) * (half - 0.1 - pace * n * 1.3e-3)
                soles = rng.random() if name == 'chair-dismount' and n > 900 else rng.random() / 10
                sensed = (0.0, 1.3e-3, soles, along, half, rng.uniform(-110, 10), rng.random() / 50)
                want, got = pm.step(ps, a, q, *sensed), cm.step(cs, b, q, *sensed)
                bad += not (same(ps, cs) and same(a, b) and got == want)
                phases.add(int(ps[py.PHASE]))
                if want:
                    break
        report.check("her %s in C is her python's, %d phases" % (name, len(phases)),
                     not bad and len(phases) == len(pm.tables.names), '%d apart' % bad)


def test_her_brace_in_c_as_she_walks(report, c):
    """Her surety's brace on the solve in C (`c_brace.laid`, wbc_brace.c) beside its python
    (`brace.scaled`, `braced`) on copies at every pass of her law over PASSES of her walk - the
    solve's M, its answer blended, her drives' inertia and her legs' stiffness bit for bit -, and
    a blend's time beside its python's."""
    from machine.law import brace
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    real, body, tally, spent = g['braced'], g['body'], [0, 0, 0.0], [[], []]

    class Beside:

        def scaled(self, M, w_up):
            want = brace.scaled(M, g['_acv_sh'], w_up)
            got = real.scaled(M, w_up)
            tally[1] += not same(got, want)
            tally[2] = max(tally[2], float(np.abs(got - want).max()))
            return got

        def blend(self, M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance):
            mine = (qacc.copy(), tau.copy(), over.copy())
            t0 = time.perf_counter()
            want = brace.braced(M, Ms, q, v, h, posture, *mine, sw, bw, d_ax, d_lat, w_hop, w_up, stance, g['brace'],
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


def _leaves(x, d, s):
    """An array, a dict's two and a number: a walk's every kind of leaf."""
    return x * s + d['a'] - d['b'].sum()


def test_her_leaves_laid_in_c_as_in_python(report, c):
    """`_leaves` on her memory (`bodied.on_args`), its walk ported to C (`tools.cores.quick`) beside the lines
    it gives up to (`glue.walk`'s): fresh arrays, a strided and a bool array and an int handed to Python, a
    shape not traced, a key not there - the same answer, table, scalars and own blocks; called, numpy's bits."""
    from tools.cores import bodied, glue, quick
    from tools.cores.orders import same
    if quick.module() is None:
        report.check('her glue in C built here', False, 'quick stays python')
        return
    rng = np.random.default_rng(43)
    x, d = rng.normal(size=4), {'a': rng.normal(size=4), 'b': rng.normal(size=3)}
    part = bodied.on_args(_leaves, 'leaves', (0, 1, 2))
    b = part.wait(x, d, 0.5)
    walker, quick.walker = quick.walker, lambda bound, todo, slow: None
    try:
        slow = glue.walk(b, b.all_args)
    finally:
        quick.walker = walker
    fast, words, owns = glue.walk(b, b.all_args), b.words.copy(), [o for o in b.own if o is not None]
    fits = {'fresh': (rng.normal(size=4), {'a': rng.normal(size=4), 'b': rng.normal(size=3)}, 1.5),
            'strided': (rng.normal(size=8)[::2], d, 0.25), 'bool': (rng.normal(size=4) > 0.0, d, 2.0),
            'an int': (x, d, 2)}
    cases = dict(fits, **{'a shape not traced': (rng.normal(size=5), d, 0.5), 'a key not there': (x, {'a': d['a']}, 0.5)})
    apart = []
    for name, args in cases.items():
        laid = []
        for walk in (fast, slow):
            b.seen[:], b.words[:] = [None] * len(b.seen), words
            for o in owns:
                o[...] = 0.0
            laid.append((walk(args), b.words.copy(), [o.copy() for o in owns]))
        (ok, w, o), (ok2, w2, o2) = laid
        if ok != ok2 or (ok and not (np.array_equal(w, w2) and all(map(np.array_equal, o, o2)))):
            apart.append(name)
    b.words[:] = words
    called = [same(part(*args), _leaves(*args)) for args in fits.values()]
    report.check("her leaves laid in C as Python lays them, %d cases; called, numpy's bits" % len(cases),
                 not apart and all(called) and part.c is not None and part.why is None,
                 'apart %s; called %s; why %s' % (apart, called, part.why))


ROSTER = (test_her_surety_in_c_as_she_walks,
          test_her_touch_in_c_as_she_walks,
          test_her_twist_in_c_as_she_walks,
          test_her_waits_in_c_as_she_walks,
          test_her_played_wait_in_c_as_said,
          test_her_bed_exit_in_c_as_in_python,
          test_her_scoot_in_c_as_in_python,
          test_her_bed_entry_in_c,
          test_her_brace_in_c_as_she_walks,
          test_her_leaves_laid_in_c_as_in_python)


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
