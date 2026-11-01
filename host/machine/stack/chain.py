"""Her step in closed chains: what bears her by complementarity, her posture in its null space.

No sole standing and what is not her soles bearing her - a bed, a seat's edge, the floor under her knees and
hands -, her inverted pendulum is gone: the physics is a parallel chain with contacts that stick or slide
(closed_chain_complementarity.lisp). The upright step (`machine.stack.wrench`) holds every bearing point
still at W_STILL and plans its push; on a yielding support both are asks it cannot meet (lying back on a bed
she spasmed at the foam's 1.9 Hz and rolled 19 deg). This step asks no balance of what bears her: her
driven joints follow her posture in the damped null space of the contacts' Jacobian (Levenberg-Marquardt,
Cholesky); the contacts' forces come from a box-projected Gauss-Seidel over her Delassus matrix on the dofs no
drive holds - pressing alone, within their friction's box, their slip bled -, warm from what is felt; her
torques follow with them, shrunk to her clamps as the upright step's are. wbc_chain.c is it in C, bit for
bit; `motion.control` picks it by her stance alone, never by a text's name.
"""
import collections
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.stack.shrunk import box, shrunk
from machine.stack.wbc import HELD_K, POSTURE_KD

#: The regime as its text says it (`closed-chain-regime`): the null space's damping m, the Gauss-Seidel's sweeps, a
#: contact's softness m/s^2 a N, a sticking contact's slip bled 1/s, its friction.
Regime = collections.namedtuple('Regime', 'lam sweeps soft bleed mu')


def chained(ask, s):
    """1 where this step is hers, by her state alone - no sole standing and what is not her soles bearing her -, else
    0. Measured on the bed (rolled, jerk pelvis / head / right foot): this 0.1 deg, 392 / 599 / 1277; asking two bodies
    or more 1.1, 570 / 896 / 2034 (the regime flickered as her pelvis bore alone); latched on the prop's impact-blanked
    comparator 0.6, 526 / 816 / 2074 (the leg swap's hop ran on the upright step and slammed her foot)."""
    return int(not any(ask['stance']) and bool(s.get('contacts')))


def regime(said):
    """The `Regime` of the kernel's `closed-chain-regime` constants `said` ({param: value}, `kernel.read`)."""
    return Regime(float(said['dls-damping-m']), int(round(said['pgs-sweeps'])), float(said['contact-softness-m-s2-per-n']),
                  float(said['stick-bleed-per-s']), float(said['friction-mu']))


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


def frames(f):
    """Each contact's frame (k, 3, 3) of its felt force `f` (3 k,), its rows: its normal - the force's direction, up
    where it feels none - and two tangents."""
    Q = np.zeros((len(f) // 3, 3, 3))
    for i in range(len(Q)):
        f0, f1, f2 = float(f[3 * i]), float(f[3 * i + 1]), float(f[3 * i + 2])
        norm = math.sqrt(f0 * f0 + f1 * f1 + f2 * f2)
        n0, n1, n2 = (f0 / norm, f1 / norm, f2 / norm) if norm > 0.0 else (0.0, 1.0, 0.0)
        e0, e1, e2 = (0.0, 0.0, 1.0) if abs(n0) > 0.9 else (1.0, 0.0, 0.0)
        t0, t1, t2 = n1 * e2 - n2 * e1, n2 * e0 - n0 * e2, n0 * e1 - n1 * e0
        tn = math.sqrt(t0 * t0 + t1 * t1 + t2 * t2)
        t0, t1, t2 = t0 / tn, t1 / tn, t2 / tn
        Q[i] = ((n0, n1, n2), (t0, t1, t2), (n1 * t2 - n2 * t1, n2 * t0 - n0 * t2, n0 * t1 - n1 * t0))
    return Q


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
    complementarity's. Her driven joints on her posture (POSTURE_KD) in the contacts' null space, her held joints
    damped (HELD_K), her base and passive joints as the forces and her dynamics move them."""
    n, M, h, v, q = b.n, s['M'], s['h'], s['v'], s['q']
    contacts = s.get('contacts', ())
    A, H = np.asarray(b.act, int), np.asarray(b.held, int)
    P = np.flatnonzero((np.arange(n) < 6) | b.passive)
    given, k, nd = np.concatenate([A, H]), len(contacts), len(b.act)
    want = POSTURE_KD[0] * (np.asarray(ask['posture'], float) - q) - POSTURE_KD[1] * v[A]
    held = -HELD_K * v[H]
    J = np.vstack([c['J'] for c in contacts]) if k else np.zeros((0, n))
    dJv = np.concatenate([np.asarray(c['dJv'], float) for c in contacts]) if k else np.zeros(0)
    f = np.concatenate([np.asarray(c['f'], float) for c in contacts]) if k else np.zeros(0)
    # 1. her posture in the contacts' damped null space on her driven joints (Levenberg-Marquardt)
    a_des = want.copy()
    if k:
        Ja = np.ascontiguousarray(J[:, A])
        JaT = np.ascontiguousarray(Ja.T)
        G = Ja @ JaT
        for i in range(3 * k):
            G[i, i] = G[i, i] + regime.lam * regime.lam
        a_des = want - JaT @ solved(cholesky(G), Ja @ want)
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
