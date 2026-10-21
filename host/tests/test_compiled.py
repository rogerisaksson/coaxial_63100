#!/usr/bin/env python3
"""Her numpy compiled to C beside numpy's own (`tools.cores.trace`, `emit`, `held`, `bodied`).

numpy's reductions, selections, comparisons and its ties of zeros; her state carried on in place;
her sensing on her body over random configurations - every array it returns, her data and her
scratch after, bit for bit, its time and numpy's -; her law's compiled parts at every pass of it as
she walks.
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


def _numpys(x, y, m):
    """numpy's sums along each axis, contiguous and strided, full, its mean, min, max and initial,
    its products; where, clip, nan_to_num, isnan, its comparisons and logic, its norms; an array of
    her scalars; a tie of zeros at its maximum and minimum, the sign arctan2 reads."""
    with np.errstate(invalid='ignore'):
        roots = np.sqrt(x)
    zero = np.minimum(x, 0.0) * 0.0
    return {'sums': (x.sum(axis=1), x.sum(axis=0), x.sum(), x[:, ::2].sum(axis=1), np.add.reduce(m, axis=1), m.sum()),
            'mean': (np.mean(y), y.mean(axis=0)), 'min': (x.min(axis=1), np.min(y, axis=0, initial=0.25)),
            'max': (np.max(x, axis=0), m.max(axis=2)), 'where': np.where(x > 0.3, x, y), 'clip': np.clip(y, -0.5, 0.5),
            'nans': (np.nan_to_num(roots), np.isnan(roots)), 'flags': (x > 0.0) & ~(y < 0.0) | (x == y),
            'norms': (np.linalg.norm(x, axis=1), np.linalg.norm(y[0])),
            'ties': np.arctan2(np.maximum(zero, 0.0), -1.0) + 2.0 * np.arctan2(np.minimum(0.0, zero), -1.0),
            'arrays': np.array([[x[0, 0] * y[1, 1], 0.5], [np.min(y), 1.0 - x[2, 3]]]) @ np.asarray([x.sum(), 2.0]),
            'prods': (np.prod(x[0, :4]), m.prod(axis=1))}


def _carried(st, seen, w, dt):
    """Her state carried on in place: an array written into, a dict's entry added to and one rebound,
    a number traced, a constant `dt`, a dict's list pinned."""
    seen[:, :-1], seen[:, -1] = seen[:, 1:].copy(), st['v'][st['ids']] * w
    st['x'] += st['v'] * dt
    st['v'] = 0.5 * st['v'] + seen.mean(axis=1).sum()
    return st['x'].sum() * w


def test_her_state_in_place_in_c(report, c):
    """`_carried` compiled on her memory (`bodied.on_args`) beside numpy's over 20 calls on copies of
    one state: its result and every array of its state, bit for bit, the rebound entry the same array."""
    from tools.cores import bodied, trace
    from tools.cores.orders import same
    rng = np.random.default_rng(17)

    def state():
        return {'x': rng.normal(size=3), 'v': rng.normal(size=3), 'ids': [0, 2]}, rng.normal(size=(2, 4))
    (st, seen), w = state(), 0.75
    carried = bodied.on_args(_carried, 'carried', (0, 1, 2))
    try:
        carried.wait({'x': st['x'].copy(), 'v': st['v'].copy(), 'ids': st['ids']}, seen.copy(), w, 0.003)
    except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
        report.check('her state carried on in place in C', False, str(e)[:300])
        return
    ours, theirs = ({'x': st['x'].copy(), 'v': st['v'].copy(), 'ids': st['ids']}, seen.copy()), \
        ({'x': st['x'].copy(), 'v': st['v'].copy(), 'ids': st['ids']}, seen.copy())
    v_was, bad = theirs[0]['v'], 0
    for _ in range(20):
        want, got = _carried(*ours, w, 0.003), carried(*theirs, w, 0.003)
        bad += not same(got, want) or not all(same(theirs[0][k], ours[0][k]) for k in 'xv') or not same(theirs[1], ours[1])
    report.check('her state carried on in place in C, bit for bit over 20 calls', not bad and theirs[0]['v'] is v_was
                 and carried.c is not None, '%d apart' % bad)


def test_numpys_reductions_and_selections_in_c(report, c):
    """`_numpys` compiled (`held.compiled`): held bit for bit on its traced arrays and fresh draws."""
    from tools.cores import held, trace
    rng = np.random.default_rng(13)
    args = (rng.normal(size=(4, 9)), rng.normal(size=(4, 9)), rng.normal(size=(2, 3, 5)))
    try:
        run = held.compiled(_numpys, args, (0, 1, 2), 'numpys', fresh=4)
        report.check("numpy's reductions, selections and ties of zeros in C bit for bit", True,
                     '%d statements' % len(run.prog.statements))
    except (trace.Untraced, AssertionError, RuntimeError) as e:
        report.check("numpy's reductions, selections and ties of zeros in C bit for bit", False, str(e)[:300])


def test_her_sensing_in_c_is_her_numpys(report, c):
    """`wbc.sense` compiled on her body beside its numpy on a body of its own, over random
    configurations: every array it returns, her data and her scratch after, bit for bit."""
    from machine import wbc
    from tools.cores import bodied, motion, trace, trace_mj
    from tools.cores.orders import same
    b = c['body']
    twin, configurations = trace_mj.twin(b), list(_configurations(b, np.random.default_rng(11)))
    sense = bodied.on_body(wbc.sense, 'sense')
    try:
        shape = sense.wait(b, *configurations[0]).prog.shape
    except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
        report.check("her sensing in C held, or numpy's own here", not motion.exact_host(), str(e)[:200])
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


def _copies(args):
    """Each array copied, a dict's float arrays too - its others pinned (`bodied._tree_in`), the same."""
    return [a.copy() if isinstance(a, np.ndarray) else {
        k: v.copy() if isinstance(v, np.ndarray) and v.dtype.kind in 'fb' else v for k, v in a.items()}
        if isinstance(a, dict) else a for a in args]


def _arrays(args):
    return [x for a in args for x in (a.values() if isinstance(a, dict) else (a,)) if isinstance(x, np.ndarray)]


def _beside(real, fn, twin, tally, at=0):
    """`real` (Compiled) as her motion pass calls it, built and held at its first call, beside `fn` on
    copies - her body's `twin` at `at`, its state hers -: its result and its arguments after, bit for
    bit (`tally`)."""
    from tools.cores import trace, trace_mj

    def called(*args):
        if real.c is None and real.why is None:
            try:
                real.wait(*_copies(args))
            except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
                tally[2] = e
        ours = _copies(args)
        if twin is not None:
            trace_mj.copy_state(twin, args[at])
            ours[at] = twin
        want, got = fn(*ours), real(*args)
        if real.c is not None:
            shape = real.c.prog.shape
            tally[0] += 1
            tally[1] += _apart(trace.leaves_of(shape, got), trace.leaves_of(shape, want))
            tally[1] += _apart(dict(enumerate(_arrays(args))), dict(enumerate(_arrays(ours))))
        return got
    return called


def test_her_law_in_c_as_she_walks(report, c):
    """Her motion's compiled parts - her estimate, her sensing, her grip's feel and floor, her
    situation, her drive's cap, her ask's poses, her sensing flat (`motion.laid`) -
    at every pass of her motion over PASSES of her walk, built at their first and held there, each
    beside its numpy on copies."""
    from machine import estimate, grip, modal, motion, situation, touch, wbc, worker
    from tools.cores import motion as c_motion, trace_mj
    s = worker.session(walks=True)
    g = s['g']
    parts = {'estimated': estimate.step, 'sensed': wbc.sense, 'felt': grip.felt, 'outside': grip.outside,
             'readings': situation.readings, 'read': situation.read, 'gripped': motion.gripped, 'poses': modal.poses,
             'modal_sensed': modal.sensed}
    tallies, reals = {name: [0, 0, None] for name in parts}, {name: g[name] for name in parts}
    for name, fn in parts.items():
        twin = trace_mj.twin(g['body']) if name in ('sensed', 'estimated') else None
        g[name] = _beside(g[name], fn, twin, tallies[name], 1 if name == 'estimated' else 0)
    for _ in range(PASSES):
        worker.stepped(s)
    for name, tally in tallies.items():
        n, bad, why = tally[0], tally[1], tally[2] or reals[name].why
        if why is not None:
            report.check("her law's %s in C held, or numpy's own here" % name, not c_motion.exact_host(), str(why)[:200])
        else:
            report.check("her law's %s in C is her numpy's bit for bit at %d of her law's passes walking" % (name, n),
                         n >= PASSES // 3 - 1 and not bad, '%d apart' % bad)


def test_her_surety_in_c_as_she_walks(report, c):
    """Her surety field in C (`c_motion.surety_laid`, wbc_surety.c) beside its python
    (`motion.surety_field`) on her arrays at every call of her law over 4 PASSES of her walk - her
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
    copies of her arrays at every pass of her law over 4 PASSES of her walk - the latent out, the
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
    copies of her arrays at every pass of her law over 4 PASSES of her walk - fr, st, the latent
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
    """Her table's waits in C (`c_wait.laid`, wbc_wait.c) beside their python (`motion.held_rows`,
    `risen_rows`) on copies of her table at every pass of her law over 4 PASSES of her walk - the
    table's times, the shift, the first row and the state bit for bit -, and a step's time."""
    from machine import motion, worker
    from tools.cores import motion as c_motion
    from tools.cores.orders import same
    s = worker.session(walks=True)
    g = s['g']
    waits, tally, spent = g['waits'], [0, 0, 0.0], [[], []]

    class Beside:

        def held(self, tab, t, dt, held_s):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = motion.held_rows(mine, g['a'], g['out'], t, dt, held_s)
            t1 = time.perf_counter()
            got = waits.held(tab, t, dt, held_s)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

        def risen(self, tab, t, dt, prop_until):
            mine = tab.copy()
            t0 = time.perf_counter()
            want = motion.risen_rows(mine, g['y'], g['fr'], g['a'], g['st'], t, dt, prop_until)
            t1 = time.perf_counter()
            got = waits.risen(tab, t, dt, prop_until)
            self.tallied(t1 - t0, time.perf_counter() - t1, tab, mine, got, want)
            return got

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
                     n >= 2 * PASSES and not bad, '%d apart, %.3g at most' % (bad, apart))
    else:
        report.check("her table's waits in C are her python's within their last bits where numpy dispatches elsewhere",
                     n >= 2 * PASSES and apart <= 1e-9, '%d of %d not bit for bit, %.3g apart at most' % (bad, n, apart))
    us = [1e6 * float(np.median(each)) for each in spent]
    report.check("a wait in C on this host", us[0] < 20.0, '%.1f us, its python %.0f' % tuple(us))


def test_aircraft_substeps_in_c_are_their_numpys(report, c):
    """`quad.substep` and `wingborne.wing_substep` compiled on each aircraft's `Sky` beside numpy's
    on a twin `Sky`, over 40 substeps: `qpos`, `qvel`, `qacc` and `xfrc_applied` bit for bit."""
    from machine import quad, wingborne
    from machine.aerobat import sky as asky
    from machine.helicopter import sky as hsky
    from machine.ornithopter import sky as osky
    from tools.cores import motion as c_motion, trace, trace_mj
    from tools.cores.orders import same
    rng = np.random.default_rng(19)
    for label, cls, fn, attr, n_in in (
            ('quad', quad.Sky, quad.substep, '_step', 15),
            ('helicopter', hsky.Sky, quad.substep, '_step', 15),
            ('ornithopter', osky.Sky, quad.substep, '_step', 15),
            ('aerobat', asky.Sky, wingborne.wing_substep, '_wing_step', 8)):
        sky = cls()
        twin = trace_mj.twin(sky)
        comp = getattr(sky, attr)
        args = tuple(float(x) for x in rng.normal(size=n_in))
        try:
            comp.wait(sky, *args)
        except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
            report.check("%s's substep in C held, or numpy's own here" % label, not c_motion.exact_host(), str(e)[:200])
            continue
        trace_mj.copy_state(twin, sky)
        bad = 0
        for _ in range(40):
            draw = tuple(float(x) for x in rng.normal(size=n_in))
            comp(sky, *draw)
            fn(twin, *draw)
            bad += any(not same(getattr(sky.data, k), getattr(twin.data, k))
                       for k in ('qpos', 'qvel', 'qacc', 'xfrc_applied'))
        report.check("%s's MuJoCo substep in C is its numpy's bit for bit over 40 substeps" % label,
                     not bad and comp.c is not None, '%d apart' % bad)


ROSTER = (test_numpys_reductions_and_selections_in_c, test_her_state_in_place_in_c, test_her_sensing_in_c_is_her_numpys,
          test_her_law_in_c_as_she_walks, test_her_surety_in_c_as_she_walks, test_her_touch_in_c_as_she_walks,
          test_her_twist_in_c_as_she_walks, test_her_waits_in_c_as_she_walks,
          test_aircraft_substeps_in_c_are_their_numpys)


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
