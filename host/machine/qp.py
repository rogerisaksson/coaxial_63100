"""Dense convex QPs and their stacks: a dual active set, an interior point where it gives up.

Goldfarb and Idnani's dual active set; Mehrotra's interior point where it gives up.

    x, held, u = solve(H, g, A, b)           # min 1/2 x'Hx + g'x  s.t.  A x <= b; None: infeasible
    x, held, u = solve(H, g, A, b, held)     # warm: the rows the last solve ended on, tried first
    x, held, slack = stack(E, e, G, h, levels, held)   # tasks by priority, each in the null
                                             # space of those above, the equalities eliminated

H positive definite; `held` the active rows, `u` their multipliers. A row is scaled to unit norm.
Each change of the active set refactors J0'N by one QR rather than Givens updates: simpler, and
the sets change by a row or two a step. In a stack, a row a level holds with a positive
multiplier stays held below it - leaving it would cost that level its optimum - so it joins the
equalities whose null space the lower levels search: kept an inequality, the feasible set below
had no interior and both methods failed on it (a shove, 2026-10-10).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

#: A constraint violated by more than TOL (its row unit-norm) is added; a direction or a
#: multiplier under TINY is none.
TOL, TINY = 1e-9, 1e-12

#: A row in the span of the held ones met if over by less than LOOSE (unit-norm): at 1e-6 a
#: shove's third level, over by 2.4e-6, was found infeasible; from 3e-6 it solves.
LOOSE = 1e-5

#: A level's tie-break, of its largest curvature: the levels below it at LOWER, and EPS of
#: every direction. At 1e-8 alone a level of 9 rows over 30 directions (condition 1.6e8) was
#: found infeasible with z = 0 meeting every row, from 1e-6 it solves, its optimum unmoved; on
#: the least norm alone the balance's level bore her on corners at their friction's edges, the
#: rows frozen, and no sole could rise (shoves, 2026-10-10).
EPS, LOWER = 1e-6, 1e-3

#: A held row's multiplier under the task alone, of the task's gradient, from which the levels
#: below keep it held.
STRONG = 1e-6


def _factor(J0, N):
    """(J, R): J0'N = Q [R; 0], J = J0 Q."""
    q = N.shape[1]
    if not q:
        return J0, np.zeros((0, 0))
    Q, R = np.linalg.qr(J0.T @ N, mode='complete')
    return J0 @ Q, R[:q, :q]


def _pair(J0, x0, N, d, act):
    """(act, J, R, x, u): the minimiser on `act` as equalities, rows dependent on those before
    them and rows whose multipliers come out negative dropped - Goldfarb and Idnani's S-pair."""
    del act[J0.shape[0]:]
    while True:
        J, R = _factor(J0, N[:, act])
        diag = np.abs(np.diag(R))
        if not act or diag.min() > 1e-9 * max(1.0, diag.max()):
            break
        act.pop(int(np.argmin(diag)))
    while act:
        w = np.linalg.solve(R.T, d[act] - N[:, act].T @ x0)
        u = np.linalg.solve(R, w)
        if u.min() >= 0.0:
            return act, J, R, x0 + J[:, :len(act)] @ w, u
        act.pop(int(np.argmin(u)))
        J, R = _factor(J0, N[:, act])
    return act, J, R, x0.copy(), np.zeros(0)


def solve(H, g, A, b, warm=(), most=None):
    """(x, held rows, their multipliers) minimising 1/2 x'Hx + g'x with A x <= b; x None if none
    meets it: the dual active set, else the interior point, on the rows scaled to unit norm -
    unscaled, rows from 5e-17 to 327 long, the interior point failed a stack's third level."""
    if not len(b):
        return np.linalg.solve(H, -g), (), np.zeros(0)
    norm = np.sqrt((A * A).sum(axis=1))
    rows = np.flatnonzero(norm >= TINY)
    if (b[norm < TINY] < -TOL).any():
        return None, (), np.zeros(0)
    A, b = A[rows] / norm[rows, None], b[rows] / norm[rows]
    back = {int(r): i for i, r in enumerate(rows)}
    x, act, u = _dual(H, g, A, b, [back[i] for i in warm if i in back], most)
    if x is None or (len(b) and float((A @ x - b).max()) > LOOSE):
        x, act, u = interior(H, g, A, b)
    return x, tuple(int(rows[i]) for i in act), u / norm[rows[list(act)]] if len(act) else u


def _dual(H, g, A, b, warm, most):
    L = np.linalg.cholesky(H)
    J0 = np.linalg.inv(L).T
    x0 = -J0 @ (J0.T @ g)
    m = len(b)
    if not m:
        return x0, (), np.zeros(0)
    N, d = -A.T, -b
    act, J, R, x, u = _pair(J0, x0, N, d, [i for i in dict.fromkeys(warm) if 0 <= i < m])
    most = most or 10 * (len(g) + m)
    steps, met = 0, set()
    while True:
        s = N.T @ x - d
        s[act] = np.inf
        s[[i for i in met if s[i] > -LOOSE]] = np.inf
        p = int(np.argmin(s))
        if s[p] >= -TOL:
            return x, tuple(act), u
        n, up = N[:, p], 0.0
        while True:
            steps += 1
            if steps > most:
                return None, tuple(act), u
            q = len(act)
            dv = J.T @ n
            z = J[:, q:] @ dv[q:]
            r = np.linalg.solve(R, dv[:q]) if q else np.zeros(0)
            zn = float(z @ n)
            t2 = -float(n @ x - d[p]) / zn if zn > TINY * max(1.0, float(dv @ dv)) else np.inf
            if t2 == np.inf and -float(n @ x - d[p]) < LOOSE:
                # A row in the span of the held ones, over by noise: met. Stepped on in the
                # dual, a 2.4e-6 miss drove the multipliers to 1e15 (a shove, 2026-10-10).
                met.add(p)
                act, J, R, x, u = _pair(J0, x0, N, d, act)
                break
            t1, k = np.inf, -1
            if q and (r > TINY).any():
                ratio = np.where(r > TINY, u / np.where(r > TINY, r, 1.0), np.inf)
                k = int(np.argmin(ratio))
                t1 = float(ratio[k])
            t = min(t1, t2)
            if t == np.inf:
                return None, tuple(act), u
            if t2 < np.inf:
                x = x + t * z
            u, up = u - t * r, up + t
            if t2 <= t1:
                act.append(p)
                u = np.append(u, up)
                J, R = _factor(J0, N[:, act])
                break
            act.pop(k)
            u = np.delete(u, k)
            J, R = _factor(J0, N[:, act])


def interior(H, g, A, b, most=60):
    """(x, active rows, their multipliers) by Mehrotra's predictor-corrector; x None if it does
    not converge."""
    n, m = len(g), len(b)
    x = np.zeros(n)
    if not m:
        return np.linalg.solve(H, -g), (), np.zeros(0)
    scale = 1.0 + max(float(np.abs(g).max()), float(np.abs(b).max()))
    s = np.maximum(b - A @ x, 1.0)
    lam = np.ones(m)
    for _ in range(most):
        rd, rp = H @ x + g + A.T @ lam, A @ x + s - b
        mu = float(s @ lam) / m
        if (np.abs(rd).max() < 1e-8 * scale and np.abs(rp).max() < 1e-8 * scale
                and mu < 1e-10 * scale):
            act = np.flatnonzero(s < 1e-6 * scale)
            return x, tuple(int(i) for i in act), lam[act]
        D = lam / s
        K = H + A.T @ (A * D[:, None])
        # The held rows' lam/s runs to 1e12 near the end: K tied on its diagonal where Cholesky
        # refuses it (7 random problems of 300 refused at their 14th-17th step).
        C = None
        for tie in (0.0, 1e-12, 1e-9, 1e-6):
            try:
                C = np.linalg.cholesky(K + tie * float(K.diagonal().max()) * np.eye(n))
                break
            except np.linalg.LinAlgError:
                continue
        if C is None:
            return None, (), np.zeros(0)

        def newton(rc, C):
            dx = np.linalg.solve(C.T, np.linalg.solve(C, -rd - A.T @ (D * rp - rc / s)))
            dl = D * (A @ dx + rp) - rc / s
            return dx, dl, -(rc + s * dl) / lam

        def reach(v, dv):
            neg = dv < 0.0
            return min(1.0, float((-v[neg] / dv[neg]).min())) if neg.any() else 1.0

        dx, dl, ds = newton(s * lam, C)
        a = min(reach(s, ds), reach(lam, dl))
        sigma = (float((s + a * ds) @ (lam + a * dl)) / m / mu) ** 3
        dx, dl, ds = newton(s * lam + ds * dl - sigma * mu, C)
        a = 0.99 * min(reach(s, ds), reach(lam, dl))
        x, lam, s = x + a * dx, lam + a * dl, s + a * ds
    return None, (), np.zeros(0)


def _strong(A, act, grad):
    """The rows of `act` the level's own gradient `grad` presses on: multipliers of the task
    alone (G_act'u = -grad), not of the tie-break - its own held rows' multipliers, frozen, pinned
    each sole's centre of pressure to an edge, the next step to the other (a shove, 2026-10-10)."""
    if not act:
        return []
    Ga = A[list(act)]
    Ga = Ga / np.maximum(np.sqrt((Ga * Ga).sum(axis=1)), TINY)[:, None]
    u = np.linalg.lstsq(Ga.T, -grad, rcond=None)[0]
    return [i for i, ui in zip(act, u) if ui > STRONG * max(TINY, float(np.linalg.norm(grad)))]


def null(B, rel=1e-9):
    """An orthonormal basis of B's null space, columns."""
    if not B.shape[0]:
        return np.eye(B.shape[1])
    _u, sv, vt = np.linalg.svd(B, full_matrices=True)
    rank = int((sv > rel * max(1.0, sv[0] if len(sv) else 0.0)).sum())
    return vt[rank:].T


def stack(E, e, G, h, levels, warm=(), eps=EPS):
    """(x, held, slacks) of tasks by priority under E x = e and G x <= h: `levels` [(A, b, S, s,
    w)] each least squares |A x - b|^2 with its soft rows S x <= s + slack, w|slack|^2, solved in
    the null space of E and of every level above - their achieved A x, their slacks and the rows
    they held with a positive multiplier kept; `held` each level's active rows, warm for the
    next call (`solve`). (None, ..) where G cannot be met."""
    if len(e):
        _u, sv, vt = np.linalg.svd(E, full_matrices=True)
        rank = int((sv > 1e-9 * max(1.0, sv[0])).sum())
        x = vt[:rank].T @ ((_u[:, :rank].T @ e) / sv[:rank])
        Z = vt[rank:].T
    else:
        x, Z = np.zeros(E.shape[1]), np.eye(E.shape[1])
    rows, bounds, live, held, slacks = [G], [h], [np.ones(len(h), bool)], [], []
    # The levels below each level, its tie-break: what they want, LOWER of its weight.
    below = [(np.vstack([lv[0] for lv in levels[k + 1:]] or [np.zeros((0, len(x)))]),
              np.concatenate([lv[1] for lv in levels[k + 1:]] or [np.zeros(0)]))
             for k in range(len(levels))]
    for k, (A, b, S, s, w) in enumerate(levels):
        nz, ns = Z.shape[1], len(s)
        if not nz:
            break
        Gl, hl, on = np.vstack(rows), np.concatenate(bounds), np.concatenate(live)
        at = np.flatnonzero(on)
        AZ, r = A @ Z, b - A @ x
        LZ, lr = below[k][0] @ Z, below[k][1] - below[k][0] @ x
        Hz = np.zeros((nz + ns, nz + ns))
        Hz[:nz, :nz] = AZ.T @ AZ
        scale = max(1.0, float(Hz.diagonal().max()))
        low = LOWER * scale / max(1.0, float((LZ * LZ).sum(axis=0).max())) if len(lr) else 0.0
        Hz[:nz, :nz] += low * (LZ.T @ LZ) + eps * scale * np.eye(nz)
        Hz[nz:, nz:] = (w + eps * scale) * np.eye(ns)
        gz = np.concatenate([-(AZ.T @ r) - low * (LZ.T @ lr), np.zeros(ns)])
        Aq = [np.hstack([Gl[at] @ Z, np.zeros((len(at), ns))])]
        # Met by the level above to its tolerance: a row over by 1e-9 that the null space left
        # cannot move made the third level infeasible (a shove, 2026-10-10).
        room = hl[at] - Gl[at] @ x
        bq = [np.maximum(room, 0.0) if held else room]
        if ns:
            Aq += [np.hstack([S @ Z, -np.eye(ns)]), np.hstack([np.zeros((ns, nz)), -np.eye(ns)])]
            bq += [s - S @ x, np.zeros(ns)]
        y, act, u = solve(Hz, gz, np.vstack(Aq), np.concatenate(bq),
                          warm[k] if k < len(warm) else ())
        if y is None:
            return None, tuple(held), slacks
        x = x + Z @ y[:nz]
        held.append(act)
        slacks.append(y[nz:])
        strong = _strong(np.vstack(Aq), act, np.concatenate([AZ.T @ (AZ @ y[:nz] - r),
                                                              w * y[nz:]]))
        kept = [at[i] for i in strong if i < len(at)]
        soft = [i - len(at) for i in strong if len(at) <= i < len(at) + ns]
        if ns:
            rows.append(S)
            bounds.append(s + y[nz:])
            live.append(np.ones(ns, bool))
        on = np.concatenate(live)
        on[kept] = False
        on[len(hl) + np.array(soft, int)] = False
        live = [on]
        rows, bounds = [np.vstack(rows)], [np.concatenate(bounds)]
        fixed = np.vstack([Gl[kept], S[soft]]) if kept or soft else np.zeros((0, len(x)))
        Z = Z @ null(np.vstack([AZ, fixed @ Z]))
    return x, tuple(held), slacks
