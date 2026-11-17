#!/usr/bin/env python3
"""The robot's motion control beside a twin whose senses are nudged a last bit: where its answer jumps.

    python tools/sim/nudged.py              # test_tick's walk and bed lie, a pass at a time: the worst passes
    python tools/sim/nudged.py --ulps 4     # the senses nudged four units in their last place

test_tick holds the robot's law in C to its python twins bit for bit where numpy computes as the C does, and within
1e-9 (law) and 1e-5 (solve) where it does not: a host whose numpy parts the last bits. A law continuous in its
senses keeps a last bit's difference near a last bit; one that branches on a tie turns it into a jump. Here both
sessions run the same law; each pass the twin takes the robot's state (test_tick's `_sync_control`) and its senses
(`g['sense']`) nudged by `--ulps` units in their last place (zeros kept), and the two answers are compared as
test_tick compares them: the passes past its tolerances, the worst by law and by solve, with the robot's doing.
"""
import argparse
import os
import sys

import numpy as np

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [HOST, os.path.join(HOST, 'tests')]

#: test_tick's tolerances: the law's arrays and numbers, the solve's.
LAW_TOL, SOLVE_TOL = 1e-9, 1e-5


def nudge(x, ulps, rng):
    """`x` (in place) moved `ulps` units in the last place each, up or down at random; zeros kept."""
    sign = rng.choice((-1.0, 1.0), size=x.shape)
    x[...] = np.where(x != 0.0, x + sign * ulps * np.spacing(x), x)


def told_matrix(H, A, b, act):
    """The normal matrix `H` of a solve: its spectrum's ends, and its least directions' largest entries; the rows
    held (`act`) and the null space they leave."""
    w, v = np.linalg.eigh(H)
    print('  H %d x %d: eigenvalues %s ... %s' % (len(w), len(w), np.array2string(w[:6], precision=3),
                                                   np.array2string(w[-2:], precision=3)))
    for j in range(4):
        big = np.argsort(-np.abs(v[:, j]))[:8]
        print('    least %d (%.3g): %s' % (j, w[j], ', '.join('%d:%+.2f' % (i, v[i, j]) for i in big)))
    if len(act):
        Aa = A[list(act)]
        _u, sv, vt = np.linalg.svd(Aa)
        null = vt[len(sv[sv > 1e-9 * sv[0]]):]
        Hn = null @ H @ null.T if len(null) else np.zeros((0, 0))
        print('  held %d rows; on their null space (%d) H spans %s' % (
            len(act), len(null), np.array2string(np.linalg.eigvalsh(Hn)[:6], precision=3) if len(null) else '-'))


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--ulps', type=float, default=1.0, help='units in the last place the twin\'s senses move')
    p.add_argument('--worst', type=int, default=8, help='the worst passes told')
    p.add_argument('--at', type=float, help="with --paths: the solve's normal matrix told at this second, the robot's")
    p.add_argument('--paths', action='store_true',
                   help="both on the python twins, each pass's dual active set calls told: raised, none, rows held")
    args = p.parse_args(argv)
    from machine.law import motion
    from machine.process import sprung, worker
    from tools.sim import told
    import test_tick as tt
    rng = np.random.default_rng(1)
    skip = tt.CONTROL_SKIP | set(__import__('tools.cores.motion', fromlist=['HOOKS']).HOOKS)
    rows, pair, real = [], [], motion.control
    from machine.stack import qp, within
    calls, solve0, mats = [], qp.solve, []

    def told_solve(H, g, A, b, *rest):
        try:
            u, act, mult = solve0(H, g, A, b, *rest)
        except np.linalg.LinAlgError:
            calls.append('raised')
            raise
        calls.append('none' if u is None else str(len(act)))
        mats.append((H, A, b, act))
        return u, act, mult
    if args.paths:
        within.qp.solve = told_solve

    def beside(g, dt):
        if not pair or g is not pair[0]:
            return real(g, dt)
        g2 = pair[1]
        try:
            tt._sync_control(g, g2, skip)
        except IndexError:   # the robot on its python twins too: its chain step's warm rows on its body
            b1, b2 = g['body'], g2['body']
            b2.chain_warm, b2.chain_let, b2.load = b1.chain_warm, set(b1.chain_let), b1.load.copy()
        nudge(g2['sense'], args.ulps, rng)
        calls.clear()
        mats.clear()
        real(g, dt)
        one = '/'.join(calls)
        if args.at is not None and abs(float(g['t']) - args.at) < 5e-4 and mats:
            told_matrix(*mats[0])
        calls.clear()
        real(g2, dt)
        two = '/'.join(calls)
        law = max([float(np.nanmax(np.abs(g[k] - g2[k]), initial=0.0)) for k in tt.CONTROL_ARRAYS]
                  + [abs(float(g.get(k, 0.0)) - float(g2.get(k, 0.0))) for k in tt.CONTROL_NUMBERS])
        solve = max(float(np.nanmax(np.abs(a - b), initial=0.0)) for a, b in (
            (g['seat_f'], g2['seat_f']), (g['drives'][1]['tau'], g2['drives'][1]['tau']),
            (g['drives'][1]['qacc'], g2['drives'][1]['qacc']), (g['cmd']['feed'], g2['cmd']['feed'])))
        tau = np.abs(g['drives'][1]['tau'] - g2['drives'][1]['tau'])
        rows.append((float(g['t']), law, solve, int(np.argmax(tau)), sprung.doing(g), int(g.get('chained', 0)),
                     one, two))
    motion.control = beside
    try:
        from tools.cores import motion as c_motion
        twin = c_motion.twins if args.paths else (lambda g: None)
        s1, s2 = worker.session(walks=True), worker.session(walks=True)
        twin(s1['g'])
        twin(s2['g'])
        pair[:] = [s1['g'], s2['g']]
        for _ in range(900):
            worker.stepped(s1)
        r1, r2 = told.Run('bed_behind'), told.Run('bed_behind')
        twin(r1.s['g'])
        twin(r2.s['g'])
        pair[:] = [r1.s['g'], r2.s['g']]
        r1.tell(say='lie_bed')
        r1.go(8.2)
    finally:
        motion.control = real
        within.qp.solve = solve0
    law_bad, solve_bad = sum(r[1] > LAW_TOL for r in rows), sum(r[2] > SOLVE_TOL for r in rows)
    print('nudged %g ulp: %d passes, law past %g on %d (worst %.3g), solve past %g on %d (worst %.3g)' % (
        args.ulps, len(rows), LAW_TOL, law_bad, max(r[1] for r in rows), SOLVE_TOL, solve_bad, max(r[2] for r in rows)))
    for t, law, solve, joint, doing, chained, one, two in sorted(rows, key=lambda r: -r[2])[:args.worst]:
        print('  t %7.3f  law %.3g  solve %.3g (tau %d)  %s chained %d  %s | %s' % (t, law, solve, joint, doing,
                                                                                chained, one, two))
    if args.paths:
        parted = [r for r in rows if r[6] != r[7]]
        print('paths parted on %d passes; of the %d past %g in solve, %d parted' % (
            len(parted), solve_bad, SOLVE_TOL, sum(r[6] != r[7] for r in rows if r[2] > SOLVE_TOL)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
