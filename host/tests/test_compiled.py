#!/usr/bin/env python3
"""Her numpy compiled to C beside numpy's own (`tools.cores.trace`, `emit`, `held`, `bodied`).

numpy's reductions, selections, comparisons and its ties of zeros; her state carried on in place;
her sensing on her body over random configurations - every array it returns, her data and her
scratch after, bit for bit, its time and numpy's -; her law's compiled parts at every pass of it as
she walks; each of the tracer's numpy (RULES), power's fast paths and a singular solve as numpy
runs them, a part a shape, a method not held left numpy's.
The regressor's: test_sysid; her law's hand ports: test_ported. Where numpy's arithmetic is not
this C's (`law.exact_host`), what is not held stays numpy's own."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from machine.aircraft import aerobat, helicopter, ornithopter, quad, wingborne
from gynoid_kit import Report
from cores_kit import apart

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
    from machine.stack import wbc
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
        bad += apart(trace.leaves_of(shape, sense(b, q, v)), trace.leaves_of(shape, wbc.sense(twin, q, v)))
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
            tally[1] += apart(trace.leaves_of(shape, got), trace.leaves_of(shape, want))
            tally[1] += apart(dict(enumerate(_arrays(args))), dict(enumerate(_arrays(ours))))
        return got
    return called


def test_her_law_in_c_as_she_walks(report, c):
    """Her motion's compiled parts - her estimate, her sensing, her grip's feel and floor, her
    situation, her drive's cap, her ask's poses, her sensing flat (`motion.laid`) -
    at every pass of her motion over PASSES of her walk, built at their first and held there, each
    beside its numpy on copies."""
    from machine.senses import estimate, grip, situation, touch
    from machine.law import modal, motion
    from machine.stack import wbc
    from machine.process import worker
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


def test_aircraft_substeps_in_c_are_their_numpys(report, c):
    """`quad.substep` and `wingborne.wing_substep` compiled on each aircraft's `Sky` beside numpy's
    on a twin `Sky`, over 40 substeps: `qpos`, `qvel`, `qacc` and `xfrc_applied` bit for bit."""
    from tools.cores import motion as c_motion, trace, trace_mj
    from tools.cores.orders import same
    rng = np.random.default_rng(19)
    for label, cls, fn, attr, n_in in (
            ('quad', quad.Sky, quad.substep, '_step', 15),
            ('helicopter', helicopter.Sky, quad.substep, '_step', 15),
            ('ornithopter', ornithopter.Sky, quad.substep, '_step', 15),
            ('aerobat', aerobat.Sky, wingborne.wing_substep, '_wing_step', 8)):
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


#: A constant at matmul, C-ordered: its transpose numpy's TRANS operand, a strided view of it copied
#: for gemm at matrix @ matrix and numpy's own loop at matrix @ vector; and one solved against.
PHI = np.random.default_rng(23).normal(size=(5, 5))
GRAM = PHI @ PHI.T + np.eye(5)


def _equals(x, y):
    """== and != of her arrays and of a float: where they tie, the zeros' signs and nan apart."""
    return np.where(x == y, x, y), (x != 0.0) * 1.0, (y == y) * 1.0


def _truths(x, y, m):
    """any and all, as methods and numpy's, of floats and bools: along an axis, whole, kept."""
    return ((x > 0.5).any(axis=1), np.all(y < 1.0, axis=0), x.any(), np.any(m > 0.0, axis=(0, 2)),
            (x > y).all(keepdims=True), np.where((x > 1.0).any(axis=1, keepdims=True), x, y))


def _cast(x, y):
    """astype to float: of bools, of a transposed view laid as numpy lays it, of floats no copy."""
    return (x > y).astype(float) * 2.0, x.T.astype(np.float64), x.astype(float, copy=False) + 1.0


def _solved(a, b, c):
    """linalg's solve of a vector, of a matrix, through a transposed view, of and against a constant; inv."""
    return (np.linalg.solve(a, b), np.linalg.solve(a, c), np.linalg.solve(a.T, b), np.linalg.inv(a) @ b,
            np.linalg.solve(GRAM, b), np.linalg.solve(a, GRAM), np.linalg.inv(a.T))


def _gathers(x, v):
    """np.diag of a vector, of a matrix, off their diagonals; np.repeat whole and along an axis;
    np.roll along an axis and of every axis, of a transposed view."""
    return (np.diag(v), np.diag(v, -1), np.diag(x), np.diag(x, 2), np.repeat(v, 2), np.repeat(x, 3, axis=1),
            np.roll(x, -1, axis=0), np.roll(x, 2), np.roll(x.T, 1, axis=1))


def _pow_is_cs():
    """numpy's power C's pow here, or SVML's on AVX-512, a bit off."""
    rng = np.random.default_rng(7)
    x, y = np.abs(rng.normal(size=4096)) + 0.5, rng.normal(size=4096) * 3.0
    return all(math.pow(a, b) == v for a, b, v in zip(x, y, np.power(x, y)))


def _within_ulp(a, b):
    """The same doubles to an ulp; alike where not finite."""
    a, b = np.ascontiguousarray(a, dtype=float), np.ascontiguousarray(b, dtype=float)
    i, j, on = a.view(np.int64), b.view(np.int64), np.isfinite(a) & np.isfinite(b)
    return a.shape == b.shape and np.array_equal(i[~on], j[~on]) and bool((np.abs(i[on] - j[on]) <= 1).all())


def _powers(x, y, e):
    """numpy's ** at its fast paths and C's pow at 3 and 1.5; to a number of her state, to an exponent
    each element's, of a number; a reciprocal."""
    return (x ** 2, x ** -1, x ** 0.5, x ** 2.0, x ** -1.0, x ** 0, x ** 1.0, x ** 3, np.abs(x) ** 1.5,
            np.power(x, 0.5), x ** e, np.abs(x) ** y, 2.0 ** y, np.reciprocal(y))


def _fills(x, s):
    """np.full and np.full_like of a fill of her state, a number and a row broadcast; of a constant."""
    return (np.full(3, s), np.full((2, 9), x[0]), np.full_like(x, s), np.full_like(x, x[1]) * x,
            np.full((2, 2), 1.5) + s)


def _laid(x):
    """matmul's operands as numpy 2.5 lays them: a constant's transpose (gemm's TRANS, gemv's), a strided
    constant and a strided view of hers copied for gemm, a strided constant numpy's own loop at a vector."""
    return PHI @ x @ PHI.T, x @ PHI[:, ::2], PHI.T @ x[:, 0], PHI[:, ::2].T @ x[:, 1], x[:, ::2].T @ PHI


def _nothing(x, e, m):
    """Reductions over none: a sum from zero, a min and a max from their initial, any and all, along an
    axis of none and across it; beside her array."""
    return (e.sum(), e.min(initial=1.5), m.max(axis=0, initial=-2.0), m.any(axis=1), e.any(), e.all(),
            x.sum() + e.sum())


#: The tracer's numpy, a function each: (what it is, it, its arguments of a generator - every one traced).
RULES = (('any and all', _truths, lambda r: (r.normal(size=(4, 9)), r.normal(size=(4, 9)), r.normal(size=(2, 3, 5)))),
         ('reductions over none', _nothing, lambda r: (r.normal(size=5), np.zeros(0), np.zeros((0, 3)))),
         ('== and !=', _equals, lambda r: (np.array([0.0, -0.0, 1.0, np.nan, 2.0]), np.array([-0.0, 0.0, 1.0, np.nan, 3.0]))),
         ('astype', _cast, lambda r: (r.normal(size=(4, 9)), r.normal(size=(4, 9)))),
         ("linalg's solve and inv", _solved, lambda r: (r.normal(size=(5, 5)), r.normal(size=5), r.normal(size=(5, 3)))),
         ('diag, repeat and roll', _gathers, lambda r: (r.normal(size=(4, 9)), r.normal(size=5))),
         ('power and **', _powers, lambda r: (r.normal(size=(4, 9)), r.normal(size=(4, 9)), 0.75)),
         ('full and full_like', _fills, lambda r: (r.normal(size=(4, 9)), 0.25)),
         ("matmul's operands as numpy laid them", _laid, lambda r: (r.normal(size=(5, 5)),)))


def test_the_tracers_numpy_in_c(report, c):
    """Each of RULES compiled: held bit for bit on its arrays and four fresh draws; the powers' part
    refused where numpy's pow is not C's."""
    from tools.cores import held, trace
    rng = np.random.default_rng(29)
    cs = _pow_is_cs()
    for label, fn, drawn in RULES:
        args = drawn(rng)
        try:
            run = held.compiled(fn, args, range(len(args)), fn.__name__[1:], fresh=4)
            report.check('%s in C bit for bit' % label, True, '%d statements' % len(run.prog.statements))
        except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
            refused = fn is _powers and not cs and isinstance(e, AssertionError)
            report.check('%s in C bit for bit' % label, refused, str(e)[:300])


def _raised(x, e, a, b):
    """x to a power of her state - the fast paths by its value as it runs -, of her base; a solve."""
    return x ** e, np.power(e, x), np.linalg.solve(a, b)


def test_powers_and_a_singular_solve_as_numpy_runs_them(report, c):
    """`_raised` compiled, held, then beside numpy at each fast path's exponent and others, at zeros,
    infinities and nan: bit for bit (an ulp, or refused, where numpy's pow is not C's); at a singular
    matrix its C raises numpy's LinAlgError, as numpy does, and goes on."""
    from tools.cores import bodied, trace
    from tools.cores.orders import same
    rng = np.random.default_rng(31)
    x, a, b = np.concatenate([rng.normal(size=12), [0.0, -0.0, np.inf, -np.inf, np.nan]]), rng.normal(size=(4, 4)), rng.normal(size=4)
    part, cs = bodied.on_args(_raised, 'raised', (0, 1, 2, 3)), _pow_is_cs()
    try:
        part.wait(x, 0.75, a, b)
    except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
        refused = not cs and isinstance(e, AssertionError)
        report.check('powers and a singular solve in C as numpy', refused, str(e)[:300])
        return
    bad, near = 0, same if cs else _within_ulp
    with np.errstate(all='ignore'):
        for e in (-1.0, 0.0, -0.0, 0.5, 1.0, 2.0, 3.0, 1.5, -2.0, np.nan, np.inf):
            got = part(x, e, a, b)
            bad += got is None or not all(near(g, w) for g, w in zip(got, _raised(x, e, a, b)))
    raised = []
    for singular in (np.zeros((4, 4)), np.ones((4, 4))):
        for f in (part, _raised):
            try:
                f(x, 0.5, singular, b)
                raised.append(False)
            except np.linalg.LinAlgError:
                raised.append(True)
    report.check('powers in C at their fast paths and C\'s pow, bit for bit', not bad and part.c is not None,
                 '%d of 11 exponents apart' % bad)
    report.check('a singular solve in C raises LinAlgError as numpy, and goes on compiled',
                 raised[0] and raised[::2] == raised[1::2] and part.c is not None and part.why is None,
                 'C, numpy raised: %s' % raised)


def _ringed(acc, ring, rest):
    """A sweep over a ring of any length: its edges' normals (np.roll), their reach, a product of them."""
    edge = np.roll(ring, -1, axis=0) - ring
    n = np.stack([edge[:, 1], -edge[:, 0]], axis=1)
    h = (n * ring).sum(axis=1)
    return acc + rest * (n.T @ np.maximum(h, 0.0)), h.max()


def test_a_part_a_shape(report, c):
    """`_ringed` compiled a ring's length a part (`bodied.on_shapes`): each length traced, built and held
    on its own, the first still its C once the second came; beside numpy bit for bit, both in turn."""
    from tools.cores import bodied, trace
    from tools.cores.orders import same
    rng = np.random.default_rng(41)
    part = bodied.on_shapes(_ringed, 'ringed', (0, 1, 2))
    try:
        for k in (4, 6):
            part.wait(rng.normal(size=2), rng.normal(size=(k, 2)), 0.5)
    except (trace.Untraced, AssertionError, RuntimeError, OSError) as e:
        report.check('a part a shape', False, str(e)[:300])
        return
    bad = 0
    for _ in range(10):
        for k in (6, 4):
            args = (rng.normal(size=2), rng.normal(size=(k, 2)), float(rng.normal()))
            bad += not all(same(g, w) for g, w in zip(part(*args), _ringed(*args)))
    each = [p.c is not None and p.why is None for p in part.values()]
    report.check('a part a ring\'s length, each held and its C, bit for bit', len(each) == 2 and all(each) and not bad,
                 '%d of 20 apart; %s' % (bad, sorted(p.name for p in part.values())))


def _unheld(x, s):
    """An array's method a Traced does not hold (cumsum)."""
    return x.cumsum() * s


def _from_state(x, s):
    """A reduction from an initial of her state."""
    return x.min(initial=s)


def test_a_method_not_held_leaves_numpy(report, c):
    """`_unheld` and `_from_state` on her memory (`bodied.on_args`): Untraced at their trace, no
    AttributeError or ValueError - numpy's own result each call, said once on stderr."""
    import contextlib
    import io
    from tools.cores import bodied, trace
    from tools.cores.orders import same
    x = np.random.default_rng(37).normal(size=6)
    for label, fn in (('a method not held', _unheld), ('a reduction from her state', _from_state)):
        said = io.StringIO()
        part = bodied.on_args(fn, fn.__name__[1:], (0, 1))
        with contextlib.redirect_stderr(said):
            got = [part(x, 0.5, paced=False) for _ in range(3)]
        report.check("%s leaves her part numpy's own, said once" % label,
                     all(same(g, fn(x, 0.5)) for g in got) and part.c is None and isinstance(part.why, trace.Untraced)
                     and not isinstance(part.why, AttributeError) and said.getvalue().count('stays numpy') == 1,
                     '%s; %r' % (type(part.why).__name__, said.getvalue().strip()[:120]))


ROSTER = (test_numpys_reductions_and_selections_in_c, test_her_state_in_place_in_c, test_her_sensing_in_c_is_her_numpys,
          test_her_law_in_c_as_she_walks, test_aircraft_substeps_in_c_are_their_numpys, test_the_tracers_numpy_in_c,
          test_powers_and_a_singular_solve_as_numpy_runs_them, test_a_part_a_shape, test_a_method_not_held_leaves_numpy)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    from machine.stack import wbc
    report = Report()
    c = {'body': wbc.Body()}
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
