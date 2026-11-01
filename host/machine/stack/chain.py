"""Her step in closed chains: what bears her by complementarity, her posture in its null space.

No sole standing and none asked to land - her legs let go of the floor onto a bed, a seat's edge, the
floor under her knees and hands -, her inverted pendulum is gone: the physics is a parallel chain with
contacts that stick or slide (closed_chain_complementarity.lisp). The upright step
(`machine.stack.wrench`) holds every bearing point still at W_STILL and plans its push; on a yielding
support both are asks it cannot meet (lying back on a bed she spasmed at the foam's 1.9 Hz and rolled
19 deg). This step asks no balance of what bears her: one damped least squares (Levenberg-Marquardt,
Cholesky) over every acceleration takes the chain's tasks - each patch's points held on its normal and
its centroid on its tangents, torsion free as a pivot on the buttocks or a fist is; her pelvis's and
trunk's turns; a sole asked to swing; her posture in what is left -; the contacts' forces come from a
box-projected Gauss-Seidel over her Delassus matrix on the dofs no drive holds - pressing alone, within
their friction's box, their slip bled -, warm from what is felt; her torques follow with them, shrunk
to her clamps as the upright step's are. wbc_chain.c is it in C, bit for bit; `motion.control` picks it
by her release alone (`chained`), never by a text's name, and hands it her standing soles as patches
(`bearing`): held by their friction's box, never planned as a push.
"""
import collections
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.stack.shrunk import box, shrunk
from machine.stack.wbc import HELD_K, POSTURE_KD, SWING_KD, TURN_KD, turn_error

#: The regime as its text says it (`closed-chain-regime`): the least squares' damping m, the Gauss-Seidel's sweeps, a
#: contact's softness m/s^2 a N, a sticking contact's slip bled 1/s, its friction; the tasks' weights - a patch held,
#: a turn, a swing, the posture and the base at rest.
Regime = collections.namedtuple('Regime', 'lam sweeps soft bleed mu stick turn swing posture')


def chained(ask):
    """1 where this step is hers, by her own release alone: a leg let go or no sole standing, and no sole asked to
    land; else 0. Picked only once no sole stood, the upright step kept the second leg's lift: its balance rows planned
    278-316 N on the standing sole, the seat unloaded, her joints went 255 to 566 deg/s in 0.15 s, her pelvis rose
    0.15 m and she was airborne (the lodge's lie at 16.6-16.8 s); picked on a bouncing sole alone it flipped 5 times
    in 0.9 s. The chair's sit and rise, both soles standing and none let go, stay the upright step's."""
    return int(not ask.get('swing') and (any(ask.get('let_go', ())) or not any(ask['stance'])))


def bearing(s, loads, stance, chained):
    """`s`'s contacts with each standing sole as a patch of its own where `chained`: its middle, its Jacobian's
    translation (`J6`'s last three rows), its drift, its load `loads` (N) felt along up; a body of its own below 0."""
    soles = [{'p': f['sole'], 'f': np.array([0.0, float(loads[k]), 0.0]), 'J': np.ascontiguousarray(f['J6'][3:6]),
              'dJv': np.asarray(f['dJv'][3:6], float), 'patch': 0, 'side': k, 'body': -1 - k}
             for k, f in enumerate(s['feet']) if chained and stance[k]]
    return list(s.get('contacts', ())) + soles


def regime(said):
    """The `Regime` of the kernel's `closed-chain-regime` constants `said` ({param: value}, `kernel.read`)."""
    return Regime(float(said['dls-damping-m']), int(round(said['pgs-sweeps'])), float(said['contact-softness-m-s2-per-n']),
                  float(said['stick-bleed-per-s']), float(said['friction-mu']), float(said['stick-weight']),
                  float(said['turn-weight']), float(said['swing-weight']), float(said['posture-weight']))


def cholesky(G):
    """L, lower, with L L^T = `G` (m, m), row by row (Cholesky-Banachiewicz); raises where G is not positive."""
    m, L = len(G), np.zeros_like(G)
    for i in range(m):
        for j in range(i + 1):
            s = float(G[i, j])
            for k in range(j):
                s = s - float(L[i, k]) * float(L[j, k])
            if i == j:
                if s <= 0.0:
                    raise np.linalg.LinAlgError('Matrix is not positive definite')
                L[i, i] = math.sqrt(s)
            else:
                L[i, j] = s / float(L[j, j])
    return L


def solved(L, r):
    """x with L L^T x = `r`: forward on L, back on L^T."""
    m, y, x = len(L), np.zeros(len(r)), np.zeros(len(r))
    for i in range(m):
        s = float(r[i])
        for k in range(i):
            s = s - float(L[i, k]) * float(y[k])
        y[i] = s / float(L[i, i])
    for i in range(m - 1, -1, -1):
        s = float(y[i])
        for k in range(i + 1, m):
            s = s - float(L[k, i]) * float(x[k])
        x[i] = s / float(L[i, i])
    return x


def frame(f0, f1, f2):
    """A frame's rows (3, 3) of a felt force: its normal - the force's direction, up where it feels none - and two
    tangents."""
    norm = math.sqrt(f0 * f0 + f1 * f1 + f2 * f2)
    n0, n1, n2 = (f0 / norm, f1 / norm, f2 / norm) if norm > 0.0 else (0.0, 1.0, 0.0)
    e0, e1, e2 = (0.0, 0.0, 1.0) if abs(n0) > 0.9 else (1.0, 0.0, 0.0)
    t0, t1, t2 = n1 * e2 - n2 * e1, n2 * e0 - n0 * e2, n0 * e1 - n1 * e0
    tn = math.sqrt(t0 * t0 + t1 * t1 + t2 * t2)
    t0, t1, t2 = t0 / tn, t1 / tn, t2 / tn
    return np.array(((n0, n1, n2), (t0, t1, t2), (n1 * t2 - n2 * t1, n2 * t0 - n0 * t2, n0 * t1 - n1 * t0)))


def frames(f):
    """Each contact's frame (k, 3, 3) of its felt force `f` (3 k,)."""
    return np.array([frame(float(f[3 * i]), float(f[3 * i + 1]), float(f[3 * i + 2])) for i in range(len(f) // 3)]).reshape(-1, 3, 3)


def tasks(b, s, ask, regime):
    """Every acceleration (n,) the closed chain's tasks ask, one damped least squares (Levenberg-Marquardt, Cholesky):
    each patch's - a body's contacts' - points held on the patch's normal and its centroid on its tangents
    (`regime.stick`, their slip bled; torsion about the normal free), her pelvis's and trunk's turns as asked
    (`regime.turn`), a sole asked to swing (`regime.swing`), her driven joints on their posture and her base at rest
    (`regime.posture`), her held joints damped (`regime.stick`); damped by `regime.lam` squared."""
    n, v, act = b.n, s['v'], np.asarray(b.act, int)
    rows, rhs, patches = [], [], {}
    for c in s.get('contacts', ()):
        patches.setdefault(int(c.get('body', 0)), []).append(c)
    for cs in patches.values():
        f = [0.0, 0.0, 0.0]
        for c in cs:
            f = [f[0] + float(c['f'][0]), f[1] + float(c['f'][1]), f[2] + float(c['f'][2])]
        Q, Jm, cm = frame(f[0], f[1], f[2]), np.zeros((3, n)), np.zeros(3)
        for c in cs:
            J = np.asarray(c['J'], float)
            ci = -np.asarray(c['dJv'], float) - regime.bleed * (J @ v)
            rows.append(regime.stick * (Q[0, 0] * J[0] + Q[0, 1] * J[1] + Q[0, 2] * J[2]))
            rhs.append(regime.stick * (Q[0, 0] * ci[0] + Q[0, 1] * ci[1] + Q[0, 2] * ci[2]))
            Jm, cm = Jm + J, cm + ci
        Jm, cm = Jm / float(len(cs)), cm / float(len(cs))
        for a in (1, 2):
            rows.append(regime.stick * (Q[a, 0] * Jm[0] + Q[a, 1] * Jm[1] + Q[a, 2] * Jm[2]))
            rhs.append(regime.stick * (Q[a, 0] * cm[0] + Q[a, 1] * cm[1] + Q[a, 2] * cm[2]))
    for t, turn in zip(s['turns'], ask.get('turns', ())):
        err = turn_error(np.asarray(turn, float), t['quat'])
        for a in range(3):
            rows.append(regime.turn * np.asarray(t['Jr'][a], float))
            rhs.append(regime.turn * (TURN_KD[0] * float(err[a]) - TURN_KD[1] * float(t['w'][a]) - float(t['dJr'][a])))
    for k, (at, speed, acc, quat) in ask.get('swing', {}).items():
        f = s['feet'][k]
        twist = f['J6'] @ v
        err = turn_error(np.asarray(quat, float), f['quat'])
        a6 = np.concatenate([TURN_KD[0] * err - TURN_KD[1] * twist[:3],
                             np.asarray(acc, float) + SWING_KD[0] * (np.asarray(at, float) - f['sole'])
                             + SWING_KD[1] * (np.asarray(speed, float) - twist[3:])])
        for a in range(6):
            rows.append(regime.swing * np.asarray(f['J6'][a], float))
            rhs.append(regime.swing * (float(a6[a]) - float(f['dJv'][a])))
    want = POSTURE_KD[0] * (np.asarray(ask['posture'], float) - s['q']) - POSTURE_KD[1] * v[act]
    for i, j in enumerate(act):
        row = np.zeros(n)
        row[j] = regime.posture
        rows.append(row)
        rhs.append(regime.posture * float(want[i]))
    for j in range(6):
        row = np.zeros(n)
        row[j] = regime.posture
        rows.append(row)
        rhs.append(0.0)
    for j in np.asarray(b.held, int):
        row = np.zeros(n)
        row[j] = regime.stick
        rows.append(row)
        rhs.append(regime.stick * ((-HELD_K) * float(v[j])))
    R, r = np.ascontiguousarray(np.vstack(rows)), np.array(rhs, float)
    RT = np.ascontiguousarray(R.T)
    H = RT @ R
    for i in range(n):
        H[i, i] = H[i, i] + regime.lam * regime.lam
    return solved(cholesky(H), RT @ r)


def framed(Dl, bb, f, Q):
    """(the Delassus matrix, its offset, the felt forces) turned into the contacts' frames `Q`, 3 rows a contact."""
    k, m = len(Q), len(bb)
    Dloc, bloc, zloc = np.zeros((m, m)), np.zeros(m), np.zeros(m)
    for i in range(k):
        for a in range(3):
            s, t = 0.0, 0.0
            for c in range(3):
                s = s + float(Q[i, a, c]) * float(bb[3 * i + c])
                t = t + float(Q[i, a, c]) * float(f[3 * i + c])
            bloc[3 * i + a], zloc[3 * i + a] = s, t
            for j in range(k):
                for e in range(3):
                    s = 0.0
                    for c in range(3):
                        for d in range(3):
                            s = s + float(Q[i, a, c]) * float(Dl[3 * i + c, 3 * j + d]) * float(Q[j, e, d])
                    Dloc[3 * i + a, 3 * j + e] = s
    return Dloc, bloc, zloc


def world(Q, zloc):
    """The frames' `zloc` back in the world, 3 a contact."""
    z = np.zeros(len(zloc))
    for i in range(len(Q)):
        for c in range(3):
            s = 0.0
            for a in range(3):
                s = s + float(Q[i, a, c]) * float(zloc[3 * i + a])
            z[3 * i + c] = s
    return z


def pgs(Dl, b, z, mu, soft, sweeps):
    """Box-projected Gauss-Seidel on w = `Dl` z + `b` in the contacts' frames, 3 a contact (normal, two tangents):
    each normal pressing alone (0 <= w_n, 0 <= z_n, one of them 0), each tangent within the box |z_t| <= `mu` z_n
    (the infinity norm's), `sweeps` over, `z` in place - warm from what is felt -, a contact's `soft` give on the
    diagonal; the complementarity's residual, the largest |min(w_n, z_n)| after."""
    m = len(b)
    for _ in range(sweeps):
        for r in range(m):
            w = float(b[r])
            for c in range(m):
                w = w + float(Dl[r, c]) * float(z[c])
            d = float(Dl[r, r]) + soft
            x = float(z[r]) - w / d
            if r % 3 == 0:
                z[r] = x if x > 0.0 else 0.0
            else:
                lim = mu * float(z[r - r % 3])
                z[r] = -lim if x < -lim else (lim if x > lim else x)
    residual = 0.0
    for i in range(0, m, 3):
        w = float(b[i])
        for c in range(m):
            w = w + float(Dl[i, c]) * float(z[c])
        gap = abs(min(w, float(z[i])))
        residual = gap if gap > residual else residual
    return residual


def step(b, s, ask, regime):
    """{tau, qacc, bears, cop, over, slack, held, borne, scale, residual}: `ask` as `wrench.step` takes it, no sole
    standing; `borne` each contact's force (k, 3), world; `scale` what her drives carry of the ask; `residual` the
    complementarity's. Her driven joints go as the chain's `tasks` ask, her held joints damped (HELD_K), her base and
    passive joints as the forces and her dynamics move them."""
    n, M, h, v = b.n, s['M'], s['h'], s['v']
    contacts = s.get('contacts', ())
    A, H = np.asarray(b.act, int), np.asarray(b.held, int)
    P = np.flatnonzero((np.arange(n) < 6) | b.passive)
    given, k, nd = np.concatenate([A, H]), len(contacts), len(b.act)
    held = -HELD_K * v[H]
    J = np.vstack([c['J'] for c in contacts]) if k else np.zeros((0, n))
    dJv = np.concatenate([np.asarray(c['dJv'], float) for c in contacts]) if k else np.zeros(0)
    f = np.concatenate([np.asarray(c['f'], float) for c in contacts]) if k else np.zeros(0)
    # 1. her driven joints as the closed chain's tasks ask them
    a_des = tasks(b, s, ask, regime)[A]
    qg, qg0 = np.concatenate([a_des, held]), np.concatenate([np.zeros(nd), held])
    Mpp, Mpg = np.ascontiguousarray(M[np.ix_(P, P)]), np.ascontiguousarray(M[np.ix_(P, given)])
    Minv = np.linalg.inv(Mpp)
    rhs, rhs0 = -h[P] - Mpg @ qg, -h[P] - Mpg @ qg0
    # 2. the contacts' forces by complementarity over her Delassus matrix on the dofs no drive holds
    z, residual = np.zeros(3 * k), 0.0
    JpT, JT = np.zeros((len(P), 0)), np.zeros((n, 0))
    if k:
        Jp, Jg = np.ascontiguousarray(J[:, P]), np.ascontiguousarray(J[:, given])
        JpT, JT = np.ascontiguousarray(Jp.T), np.ascontiguousarray(J.T)
        W = Jp @ Minv
        Dl = W @ JpT
        bb = W @ rhs + Jg @ qg + dJv + regime.bleed * (J @ v)
        Q = frames(f)
        Dloc, bloc, zloc = framed(Dl, bb, f, Q)
        residual = pgs(Dloc, bloc, zloc, regime.mu, regime.soft, regime.sweeps)
        z = world(Q, zloc)
    Jtz = JT @ z

    def answered(qg_, rhs_):
        """(the drives' torques, every acceleration) with the forces, her driven and held joints going `qg_`."""
        qdd = np.zeros(n)
        qdd[given] = qg_
        qdd[P] = Minv @ (JpT @ z + rhs_)
        return (M @ qdd + h - Jtz)[A], qdd
    # 3. her torques, shrunk to her clamps as the upright step's: toward nothing asked of her joints, then toward 0
    tau1, qdd1 = answered(qg, rhs)
    tau0, qdd0 = answered(qg0, rhs0)
    limit, lo, hi = box(b, ask)
    if (np.abs(tau1) > limit).any():
        scale = shrunk(tau1, tau0, -limit, limit)
        tau, qdd = tau0 + scale * (tau1 - tau0), qdd0 + scale * (qdd1 - qdd0)
        s0 = shrunk(tau, np.zeros(nd), -limit, limit) if (np.abs(tau0) > limit).any() else 1.0
        if s0 < 1.0:
            tau, qdd = s0 * tau, s0 * qdd
        scale = s0 * scale
    else:
        scale, tau, qdd = 1.0, tau1, qdd1
    past = np.abs(tau1) > b.clamp * (1.0 + 1e-6)
    over = past.copy()
    for p, r in b.pairs:
        over[p] = over[r] = past[p] or past[r]
    b.load = np.abs(tau1) / b.clamp
    return {'tau': np.clip(tau, lo, hi), 'qacc': qdd, 'bears': [0.0, 0.0], 'cop': [], 'over': over, 'slack': [],
            'held': (), 'borne': z.reshape(-1, 3), 'scale': scale, 'residual': residual}
