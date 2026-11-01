"""Her step in closed chains: contacts by complementarity, her tasks answered through her dynamics.

No sole standing and none asked to land - her legs let go of the floor onto a bed, a seat's edge, the
floor under her knees and hands -, her inverted pendulum is gone: the physics is a parallel chain with
contacts that stick or slide (closed_chain_complementarity.lisp, closed_chain_plan.lisp). The upright
step (`machine.stack.wrench`) holds every bearing point still at W_STILL and plans its push; on a
yielding support both are asks it cannot meet (lying back on a bed she spasmed at the foam's 1.9 Hz and
rolled 19 deg). This step asks no balance of what bears her. Its plan (`planned`) is dynamics-consistent:
her base's and passive joints' accelerations are what the contacts held and her dynamics make of her
driven joints', so her pelvis's and trunk's turns and a sole's swing are answered by her drives, not by
free base dofs the forces then overrule; her posture follows in their damped null space (Levenberg-
Marquardt); a contact the plan would pull on is let go of, one it would drag past its friction's box
slides with its friction known, round by round. The contacts' forces come from a box-projected
Gauss-Seidel over her Delassus matrix on the dofs no drive holds - pressing alone, within their
friction's box, their slip bled -, warm from the plan's own forces (warm from what is felt, the
mattress's lag kept them there and the push the plan counted on never reached her drives); her torques follow with them, shrunk to her
clamps as the upright step's are. wbc_chain.c is it in C, bit for bit; `motion.control` picks it by her
release alone (`chained`), never by a text's name, and hands it her standing soles as patches
(`bearing`): held by their friction's box, never planned as a push.
"""
import collections
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.stack.shrunk import box, shrunk
from machine.stack.wbc import HELD_K, SWING_KD, turn_error

#: The regime as its text says it (`closed-chain-regime`): the least squares' damping, the tasks' above posture their
#: own, the Gauss-Seidel's sweeps, the plan's rounds, a contact's softness m/s^2 a N, a sticking contact's slip bled
#: 1/s, its friction; the tasks' weights - a turn, a swing, the posture in their null space, the contacts' forces near
#: what is felt (rad/s^2 a N) -; its own gains, a turn's
#: and the posture's (kp, kd): lying down is slow, the gait's asked tens of rad/s^2 and the plan pulled her off the seat.
Regime = collections.namedtuple('Regime', 'lam task sweeps rounds soft bleed mu hold turn swing posture force tkp tkd pkp pkd')


def chained(ask):
    """1 where the step is hers: a leg let go, or no sole standing, and no sole asked to land (`motion.control`)."""
    return int(not ask.get('swing') and (any(ask.get('let_go', ())) or not any(ask['stance'])))


def bearing(s, loads, stance, chained):
    """Her contacts with each standing sole one more patch where the step is hers (`chained`): its load up (felt), its
    Jacobian's translation, a body below 0 - held in its friction's box, never planned as a push."""
    soles = [{'p': f['sole'], 'f': np.array([0.0, float(loads[k]), 0.0]), 'J': np.ascontiguousarray(f['J6'][3:6]),
              'dJv': np.asarray(f['dJv'][3:6], float), 'patch': 0, 'side': k, 'body': -1 - k}
             for k, f in enumerate(s['feet']) if chained and stance[k]]
    return list(s.get('contacts', ())) + soles


def regime(said):
    """The `Regime` of the kernel's `closed-chain-regime` as evaluated."""
    return Regime(float(said['dls-damping-m']), float(said['task-damping']), int(round(said['pgs-sweeps'])), int(round(said['plan-rounds'])),
                  float(said['contact-softness-m-s2-per-n']), float(said['stick-bleed-per-s']), float(said['friction-mu']),
                  float(said['hold-n']),
                  float(said['turn-weight']), float(said['swing-weight']), float(said['posture-weight']), float(said['force-weight']),
                  float(said['turn-kp']), float(said['turn-kd']), float(said['posture-kp']), float(said['posture-kd']))


def frame(f0, f1, f2):
    """A contact's frame (3, 3), rows: its normal - the felt force's direction, up where it feels none -, two tangents."""
    norm = math.sqrt(f0 * f0 + f1 * f1 + f2 * f2)
    n0, n1, n2 = (f0 / norm, f1 / norm, f2 / norm) if norm > 0.0 else (0.0, 1.0, 0.0)
    e0, e1, e2 = (0.0, 0.0, 1.0) if abs(n0) > 0.9 else (1.0, 0.0, 0.0)
    t0, t1, t2 = n1 * e2 - n2 * e1, n2 * e0 - n0 * e2, n0 * e1 - n1 * e0
    tn = math.sqrt(t0 * t0 + t1 * t1 + t2 * t2)
    t0, t1, t2 = t0 / tn, t1 / tn, t2 / tn
    return np.array(((n0, n1, n2), (t0, t1, t2), (n1 * t2 - n2 * t1, n2 * t0 - n0 * t2, n0 * t1 - n1 * t0)))


def frames(f):
    """Each contact's `frame` of the felt forces `f` (3k): (k, 3, 3)."""
    return np.array([frame(float(f[3 * i]), float(f[3 * i + 1]), float(f[3 * i + 2])) for i in range(len(f) // 3)]).reshape(-1, 3, 3)


def tasks(b, s, ask, regime):
    """(R (m, n), r (m), want (nd)): the tasks above her posture, weighed - her pelvis's and trunk's turns as asked
    (`regime.turn`), a sole asked to swing (`regime.swing`) -, and her driven joints' posture acceleration, at the
    regime's own gains."""
    n, v, act = b.n, s['v'], np.asarray(b.act, int)
    rows, rhs = [], []
    for t, turn in zip(s['turns'], ask.get('turns', ())):
        err = turn_error(np.asarray(turn, float), t['quat'])
        for a in range(3):
            rows.append(regime.turn * np.asarray(t['Jr'][a], float))
            rhs.append(regime.turn * (regime.tkp * float(err[a]) - regime.tkd * float(t['w'][a]) - float(t['dJr'][a])))
    for k, (at, speed, acc, quat) in ask.get('swing', {}).items():
        f = s['feet'][k]
        twist = f['J6'] @ v
        err = turn_error(np.asarray(quat, float), f['quat'])
        a6 = np.concatenate([regime.tkp * err - regime.tkd * twist[:3],
                             np.asarray(acc, float) + SWING_KD[0] * (np.asarray(at, float) - f['sole'])
                             + SWING_KD[1] * (np.asarray(speed, float) - twist[3:])])
        for a in range(6):
            rows.append(regime.swing * np.asarray(f['J6'][a], float))
            rhs.append(regime.swing * (float(a6[a]) - float(f['dJv'][a])))
    want = regime.pkp * (np.asarray(ask['posture'], float) - s['q']) - regime.pkd * v[act]
    return (np.vstack(rows) if rows else np.zeros((0, n))), np.array(rhs, float), want


def planned(regime, nd, held, Minv, Mpg, rhs0, Jp, Jg, Jv, dJv, f, Q, RP, RG, r, want):
    """(x (nd), qdd_P, z (3k)): her driven joints' accelerations the plan asks, the base's and passive joints' it
    foresees, the contacts' forces it counts on (world; a freed contact's 0).
    Each round: the contacts held (rows3: a point's normal, its two tangents) and her dynamics make the dofs no drive
    holds affine in her driven ones, qdd_P = p0 + Pg qg, with the forces known (`Fk`, a sliding contact's friction),
    the map's Delassus matrix damped by `regime.lam` squared as the least squares are; the tasks' rows in the driven
    accelerations alone, the held joints fixed, and beside them the contacts' internal forces - those with no effect
    on the dofs no drive holds, null(Ja_p^T) - kept at the felt split (`regime.force`: two contacts on one prescribed
    chain hold her in any split of her weight, and the damped map picked a pull of 280 N on her pelvis against 630 on
    her sole, a fight her drives alone would pay for; a force that moves her base is free); those rows by damped
    least squares (`regime.task`), her posture in their null space; then the plan's own
    forces: a contact pulled on goes free, one dragged past its box |z_t| <= mu z_n slides, its friction the box's.
    A contact felt under `regime.hold` N starts free: her fingers' arrival at 2-4 N, held, fought her arms' pose and
    threw her elbow. The C twin spells every product in this order."""
    k3, m1, lam2, task2 = 3 * len(Q), len(r), regime.lam * regime.lam, regime.task * regime.task
    rows3, Fk, eye, zfull = np.ones(k3, bool), np.zeros(k3), np.eye(nd), np.zeros(k3)
    for i in range(len(Q)):
        rows3[3 * i:3 * i + 3] = math.sqrt(f[3 * i] * f[3 * i] + f[3 * i + 1] * f[3 * i + 1] + f[3 * i + 2] * f[3 * i + 2]) >= regime.hold
    x, p0, Pg, qg = np.zeros(nd), np.zeros(len(rhs0)), np.zeros((len(rhs0), nd + len(held))), np.concatenate([np.zeros(nd), held])
    for _ in range(regime.rounds):
        Ja_p, Ja_g, na = Jp[rows3], Jg[rows3], int(rows3.sum())
        rhsk = rhs0 + np.ascontiguousarray(Jp.T) @ Fk if k3 else rhs0
        MM = Minv @ Mpg
        Dinv, Bg, c0 = np.zeros((0, 0)), np.zeros((0, len(MM[0]))), np.zeros(0)
        if na:
            Ja_pT = np.ascontiguousarray(Ja_p.T)
            W = Ja_p @ Minv
            Dl = W @ Ja_pT
            for i in range(na):
                Dl[i, i] = Dl[i, i] + lam2
            bb0 = W @ rhsk + dJv[rows3]
            Bg = Ja_g - W @ Mpg
            c0 = (-regime.bleed) * Jv[rows3] - bb0
            Dinv = np.linalg.inv(Dl)
            K = (Minv @ Ja_pT) @ Dinv
            p0 = K @ c0 + Minv @ rhsk
            Pg = -(K @ Bg + MM)
            # the internal forces, with no effect on the dofs no drive holds: null(Ja_p^T), the drives alone fight them
            G0 = Ja_pT @ Ja_p
            for i in range(len(G0)):
                G0[i, i] = G0[i, i] + lam2
            Nint = np.eye(na) - Ja_p @ np.linalg.solve(G0, Ja_pT)
            Zx = Nint @ (Dinv @ np.ascontiguousarray(Bg[:, :nd]))
            zc = Nint @ (Dinv @ (c0 - np.ascontiguousarray(Bg[:, nd:]) @ held) - f[rows3])
        else:
            p0, Pg = Minv @ rhsk, -MM
        PgD, PgH = np.ascontiguousarray(Pg[:, :nd]), np.ascontiguousarray(Pg[:, nd:])
        R1x = RP @ PgD + np.ascontiguousarray(RG[:, :nd])
        r1x = (r - RP @ p0) - (RP @ PgH + np.ascontiguousarray(RG[:, nd:])) @ held
        if na:
            R1x = np.vstack([R1x, regime.force * Zx])
            r1x = np.concatenate([r1x, regime.force * zc])
        if m1 + na:
            R1xT = np.ascontiguousarray(R1x.T)
            G1 = R1xT @ R1x
            H1 = G1 + task2 * eye
            x1 = np.linalg.solve(H1, R1xT @ r1x)
            N = eye - np.linalg.solve(H1, G1)
        else:
            x1, N = np.zeros(nd), eye
        R2n = regime.posture * N
        R2nT = np.ascontiguousarray(R2n.T)
        H2 = R2nT @ R2n + lam2 * eye
        x2 = np.linalg.solve(H2, R2nT @ (regime.posture * (want - x1)))
        x = x1 + N @ x2
        qg = np.concatenate([x, held])
        if not na:
            zfull = Fk.copy()
            break
        za = Dinv @ (c0 - Bg @ qg)
        zfull = Fk.copy()
        zfull[rows3] = zfull[rows3] + za
        changed = False
        for i in range(len(Q)):
            if not rows3[3 * i]:
                continue
            loc = Q[i] @ zfull[3 * i:3 * i + 3]
            zn, zt1, zt2 = float(loc[0]), float(loc[1]), float(loc[2])
            if zn < 0.0:
                rows3[3 * i:3 * i + 3], Fk[3 * i:3 * i + 3], changed = False, 0.0, True
            elif rows3[3 * i + 1] and max(abs(zt1), abs(zt2)) > regime.mu * zn:
                lim = regime.mu * zn
                tb1, tb2 = (-lim if zt1 < -lim else (lim if zt1 > lim else zt1)), (-lim if zt2 < -lim else (lim if zt2 > lim else zt2))
                for c in range(3):
                    Fk[3 * i + c] = float(Q[i, 1, c]) * tb1 + float(Q[i, 2, c]) * tb2
                rows3[3 * i + 1:3 * i + 3], changed = False, True
        if not changed:
            break
    return x, p0 + Pg @ qg, zfull


def framed(Dl, bb, f, Q):
    """(the Delassus matrix, its offset, the forces warm) turned into the contacts' frames `Q`, 3 rows a contact."""
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
    """{tau, qacc, bears, cop, over, slack, held, borne, scale, residual, foreseen}: `ask` as `wrench.step` takes it, no
    sole standing; `borne` each contact's force (k, 3), world; `scale` what her drives carry of the ask; `residual` the
    complementarity's; `foreseen` the base's and passive joints' accelerations the plan counted on. Her driven joints go as the chain's plan asks (`planned`), her held joints damped (HELD_K), her
    base and passive joints as the forces and her dynamics move them."""
    n, M, h, v = b.n, s['M'], s['h'], s['v']
    contacts = s.get('contacts', ())
    A, H = np.asarray(b.act, int), np.asarray(b.held, int)
    P = np.flatnonzero((np.arange(n) < 6) | b.passive)
    given, k, nd = np.concatenate([A, H]), len(contacts), len(b.act)
    held = -HELD_K * v[H]
    J = np.vstack([c['J'] for c in contacts]) if k else np.zeros((0, n))
    dJv = np.concatenate([np.asarray(c['dJv'], float) for c in contacts]) if k else np.zeros(0)
    f = np.concatenate([np.asarray(c['f'], float) for c in contacts]) if k else np.zeros(0)
    Mpp, Mpg = np.ascontiguousarray(M[np.ix_(P, P)]), np.ascontiguousarray(M[np.ix_(P, given)])
    Minv = np.linalg.inv(Mpp)
    rhs0 = -h[P]
    Jp, Jg = np.ascontiguousarray(J[:, P]), np.ascontiguousarray(J[:, given])
    Jv, Q = (J @ v if k else np.zeros(0)), frames(f)
    # 1. her driven joints as the chain's plan asks them, the tasks answered through the contacts and her dynamics
    R, r, want = tasks(b, s, ask, regime)
    x, foreseen, zwarm = planned(regime, nd, held, Minv, Mpg, rhs0, Jp, Jg, Jv, dJv, f, Q, np.ascontiguousarray(R[:, P]),
                                 np.ascontiguousarray(R[:, given]), r, want)
    qg, qg0 = np.concatenate([x, held]), np.concatenate([np.zeros(nd), held])
    rhs, rhs0 = rhs0 - Mpg @ qg, rhs0 - Mpg @ qg0
    # 2. the contacts' forces by complementarity over her Delassus matrix on the dofs no drive holds
    z, residual = np.zeros(3 * k), 0.0
    JpT, JT = np.zeros((len(P), 0)), np.zeros((n, 0))
    if k:
        JpT, JT = np.ascontiguousarray(Jp.T), np.ascontiguousarray(J.T)
        W = Jp @ Minv
        Dl = W @ JpT
        bb = W @ rhs + Jg @ qg + dJv + regime.bleed * Jv
        Dloc, bloc, zloc = framed(Dl, bb, zwarm, Q)
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
    for p, q in b.pairs:
        over[p] = over[q] = past[p] or past[q]
    b.load = np.abs(tau1) / b.clamp
    return {'tau': np.clip(tau, lo, hi), 'qacc': qdd, 'bears': [0.0, 0.0], 'cop': [], 'over': over, 'slack': [],
            'held': (), 'borne': z.reshape(-1, 3), 'scale': scale, 'residual': residual, 'foreseen': foreseen}
