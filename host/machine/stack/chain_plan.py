"""The closed chains' plan: her tasks answered through her dynamics, the contacts held round by round.

`machine.stack.chain.step` plans by it (closed_chain_plan.lisp and closed_chain_release.lisp say it);
wbc_chain_plan.c is it in C, bit for bit.
"""
import math

from machine.senses.borne import CARPAL, LET
from machine.stack.rounds import GROW_N


def said(f, patch, body, unweight):
    """The forces her plan aims at told a twist (3k, world): the stick-slip ratchet's split of what she feels
    (`modulate-stick-slip-sled-ratchet`) - her sled, what she sits on (no hand or foot), 1 - `unweight` of it, her
    anchors, hands and feet, `unweight` (the seated butt-scoot's buttock-unweight-ratio), each point its kind's share as
    it feels it. On full friction her sled, 55% of her, sent 108 N through her anchors: her fingers landing at 114 N
    were pushed 190 N aside on 57, slipped, and her elbow whipped 241 deg/s."""
    import numpy as np
    k = len(patch)
    felt = [math.sqrt(f[3 * i] * f[3 * i] + f[3 * i + 1] * f[3 * i + 1] + f[3 * i + 2] * f[3 * i + 2]) for i in range(k)]
    sled = [int(body[i]) >= 0 and int(patch[i]) < CARPAL for i in range(k)]
    whole, on = 0.0, 0.0
    for i in range(k):
        whole = whole + felt[i]
        on = on + felt[i] * float(sled[i])
    out = np.array(f, float)
    for i in range(k):
        part, share = (on, 1.0 - unweight) if sled[i] else (whole - on, unweight)
        if part > 0.0:
            scale = share * whole / part
            for c in range(3):
                out[3 * i + c] = float(f[3 * i + c]) * scale
    return out


def freed(patch, body, free):
    """Whether a contact is on a foot told free (`free`, left and right: her heels stepping in turn): a let-go foot's
    patch (`borne.LET`'s) or a standing sole (`body` below 0)."""
    foot = patch // LET - 1 if patch >= LET else (-1 - body if body < 0 else -1)
    return foot >= 0 and bool(free[foot])


def frame_rows(Q, i, outs, srcs):
    """Contact `i`'s three rows of each of `srcs` (3k rows, or 3k) into `outs`, turned into its frame (`Q`): its normal
    and two tangents, so a sliding contact holds its normal and lets its tangents go. Laid by her world's rows (x, y, z,
    `borne`'s Jacobians), a slide let go of world y - a seat's or a sole's normal - and held x: her seat told a twist
    bore 0 N in her plan, her hands 150 N each."""
    for out, src in zip(outs, srcs):
        for e in range(3):
            out[3 * i + e] = (float(Q[i, e, 0]) * src[3 * i] + float(Q[i, e, 1]) * src[3 * i + 1]) + float(Q[i, e, 2]) * src[3 * i + 2]


def started(regime, felt, fs, patch, body, Q, slide, free, srcs):
    """(rows3, Fk, framed, outs): the rows each contact starts held by, the forces known on it, whether its rows lie in
    its frame and `srcs` with theirs so (`frame_rows`) - a patch felt under `regime.hold` N starts free, a foot told free
    too (`freed`); told a twist, her sled - no hand or foot - slides along it at mu what it feels (`fs`), on its normal."""
    import numpy as np
    k = len(Q)
    rows3, Fk, framed, outs = np.ones(3 * k, bool), np.zeros(3 * k), np.zeros(k, bool), tuple(np.array(a, float) for a in srcs)
    for i in range(k):
        rows3[3 * i:3 * i + 3] = sum(float(patch[j] == patch[i]) * felt[j] for j in range(k)) >= regime.hold
        if freed(int(patch[i]), int(body[i]), free):
            rows3[3 * i:3 * i + 3] = False
        if rows3[3 * i] and (slide[0] != 0.0 or slide[2] != 0.0) and int(body[i]) >= 0 and int(patch[i]) < CARPAL:
            sn = (float(Q[i, 0, 0]) * slide[0] + float(Q[i, 0, 1]) * slide[1]) + float(Q[i, 0, 2]) * slide[2]
            t = [slide[c] - sn * float(Q[i, 0, c]) for c in range(3)]
            nt = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
            fn = math.sqrt(fs[3 * i] * fs[3 * i] + fs[3 * i + 1] * fs[3 * i + 1] + fs[3 * i + 2] * fs[3 * i + 2])
            if nt > 0.0:
                for c in range(3):
                    Fk[3 * i + c] = -(regime.mu * fn / nt) * t[c]
                rows3[3 * i + 1:3 * i + 3], framed[i] = False, True
                frame_rows(Q, i, outs, srcs)
    return rows3, Fk, framed, outs


def unframed(zfull, z1, za, za1, rows3, Fk, Q, framed):
    """A framed contact's forces - its held rows' (`za`, `za1`) in its frame - back in the world, in place."""
    at = 0
    for i in range(len(Q)):
        if framed[i]:
            rf, r1, h = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], at
            for e in range(3):
                if rows3[3 * i + e]:
                    rf[e], r1[e], h = float(za[h]), float(za1[h]), h + 1
            for c in range(3):
                q0, q1, q2 = float(Q[i, 0, c]), float(Q[i, 1, c]), float(Q[i, 2, c])
                zfull[3 * i + c] = float(Fk[3 * i + c]) + ((q0 * rf[0] + q1 * rf[1]) + q2 * rf[2])
                z1[3 * i + c] = float(Fk[3 * i + c]) + ((q0 * r1[0] + q1 * r1[1]) + q2 * r1[2])
        at += int(rows3[3 * i]) + int(rows3[3 * i + 1]) + int(rows3[3 * i + 2])


def within(H, g, A, b, x, S, s, w):
    """x minimising 1/2 x'Hx + g'x + w/2 |slack|^2 under A x <= b and S x <= s + slack, slack >= 0 (the stack's dual
    active set, `qp.solve`) where the unconstrained `x` is over a row; `x` as it came where none is (bit for bit) or no
    x meets A. A primal active set's KKT by solve took dependent rows and answered 1e7-1e9 past them; S held hard, no
    x met it on 40% of her passes over the bed's edge (her leg must slide) and her plan flipped back and forth."""
    import numpy as np
    from machine.stack import qp
    n, ns = len(x), len(s)
    if float(np.max(np.concatenate([A @ x - b, S @ x - s, [-1.0]]))) <= qp.TOL:
        return x
    Hs, gs = np.zeros((n + ns, n + ns)), np.concatenate([g, np.zeros(ns)])
    Hs[:n, :n], Hs[n:, n:] = H, w * np.eye(ns)
    As = np.vstack([np.hstack([A, np.zeros((len(b), ns))]), np.hstack([S, -np.eye(ns)]), np.hstack([np.zeros((ns, n)), -np.eye(ns)])])
    got, _held, _u = qp.solve(Hs, gs, As, np.concatenate([b, s, np.zeros(ns)]))
    return x if got is None else got[:n]


def torqued(torque, rows3, framed, Q, Fk, held, p0, Pg, Dinv, Bg, c0, nd, na, mu, stops=None):
    """(A, b, S, s): her drives' clamps on the plan, |tau(x)| <= limit as rows A x <= b, tau(x) = T x + t her driven
    joints' torques as the plan has her move (`torque`: M[act, given], M[act, P], h[act], J[:, act], limit), its
    held contacts' forces with her - the step shrank every torque toward none past a clamp (to 0.00-0.21 of the ask
    with her leg over the bed's edge, and she fell); and each contact all held within its friction's box,
    |t z| <= mu n z: the step's torques come of the Gauss-Seidel's forces, and planned outside the box it moved them,
    her torques with them past the clamps the plan had met (0.05-0.10 of the ask). `stops` (rows, bounds) her joints'
    stops on her driven accelerations (`chain.stop_rows`), rows of A beside the clamps' or alone; None with neither:
    her spine lay on its -30 deg stop as her sit-up asked it +400 deg/s^2 with 45-75 N m into the stop."""
    import numpy as np
    if torque is None:
        return None if stops is None else (stops[0], stops[1], np.zeros((0, nd)), np.zeros(0))
    MAG, MAP, hA, JA, limit = torque
    T = MAG[:, :nd] + MAP @ Pg[:, :nd]
    t = MAG[:, nd:] @ held + MAP @ (p0 + Pg[:, nd:] @ held) + hA - JA.T @ Fk
    rows, bounds = [], []
    if na:
        S, col = np.zeros((len(Fk), na)), 0
        for i in range(len(Q)):
            for e in range(3):
                if rows3[3 * i + e]:
                    if framed[i]:
                        S[3 * i:3 * i + 3, col] = Q[i, e]
                    else:
                        S[3 * i + e, col] = 1.0
                    col += 1
        Sx, s0 = S @ (Dinv @ Bg[:, :nd]), S @ (Dinv @ (c0 - Bg[:, nd:] @ held))
        T = T + JA.T @ Sx
        t = t - JA.T @ s0
        # z_i(x) = Fk_i + s0_i - Sx_i x; its box: (+-t_e - mu n) z_i(x) <= 0
        for i in np.flatnonzero(rows3[0::3] & rows3[1::3] & rows3[2::3] & ~np.asarray(framed, bool)):
            for e in (1, 2):
                for side in (1.0, -1.0):
                    edge = side * Q[i, e] - mu * Q[i, 0]
                    rows.append(-(edge @ Sx[3 * i:3 * i + 3]))
                    bounds.append(-float(edge @ (Fk[3 * i:3 * i + 3] + s0[3 * i:3 * i + 3])))
    A, b = np.vstack([T, -T]), np.concatenate([limit - t, limit + t])
    if stops is not None:
        A, b = np.vstack([A, stops[0]]), np.concatenate([b, stops[1]])
    return A, b, np.array(rows).reshape(-1, nd), np.array(bounds, float)


def held_map(Ja_p, Ja_g, Minv, Mpg, rhsk, dJva, Jva, fsa, lam2, bleed, gives=None):
    """(Ja_p^T, the map's Delassus matrix, its driven part Bg, its offset c0) of the rows held (`planned`'s): each
    row's acceleration bled of its slip at `bleed`, the matrix damped by `lam2` - or a row whose support gives more than
    her floor giving as it gives (`gives`, m/s^2 a N, its row's) over what it feels (`fsa`), a firm row at lam2 over it
    as far as what gives bears her (its rows' share of what the rows held feel): held rigid on her bed's foam, her
    points sinking 0.16-0.47 m/s into it, the plan asked it 1.2-8.3 kN at her pelvis, freed her torso and thighs while
    they bore 200-350 N; the floor giving too her scoot's sit fell, held at what it felt her rise off a chair never
    stood, and held at none beside the foam her rise off the bed fell to the floor."""
    import numpy as np
    Ja_pT = np.ascontiguousarray(Ja_p.T)
    W = Ja_p @ Minv
    Dl = W @ Ja_pT
    for i in range(len(Dl)):
        Dl[i, i] = Dl[i, i] + (float(gives[i]) if gives is not None and gives[i] > 0.0 else lam2)
    bb0, Bg = W @ rhsk + dJva, Ja_g - W @ Mpg
    c0 = (-bleed) * Jva - bb0
    if gives is None:
        return Ja_pT, Dl, Bg, c0
    on, whole = 0.0, 0.0
    for i in range(len(fsa)):
        whole = whole + abs(float(fsa[i]))
        on = on + abs(float(fsa[i])) * float(gives[i] > 0.0)
    soft = on / whole if whole > 0.0 else 0.0
    for i in range(len(c0)):
        c0[i] = c0[i] + (float(gives[i]) if gives[i] > 0.0 else soft * lam2) * float(fsa[i])
    return Ja_pT, Dl, Bg, c0


def capped(felt, patch, hands):
    """Each contact's push cap, N: what it feels (`felt`, k) by GROW_N, a hand's patch (`borne.CARPAL`'s) no more than
    its share as felt of `hands`, what her text tells her hands to bear (`ask['hand_cap']`, `pressed.patches`' rule;
    none where not finite). Held as felt, lifting a leg onto the bed her hands bore 240-299 N of her told 89-102, her
    pelvis 0 N: her release's gate shut and the leg fell to the floor."""
    palm, nh, caps = 0.0, 0, []
    for i in range(len(felt)):
        palm, nh = palm + felt[i] * float(CARPAL <= int(patch[i]) < LET), nh + int(CARPAL <= int(patch[i]) < LET)
    for i in range(len(felt)):
        cap = felt[i] + GROW_N
        if CARPAL <= int(patch[i]) < LET and math.isfinite(hands):
            told = hands * (felt[i] / palm if palm > 0.0 else 1.0 / float(nh if nh > 1 else 1))
            cap = told if told < cap else cap
        caps.append(cap)
    return caps


def planned(regime, nd, held, Minv, Mpg, rhs0, Jp, Jg, Jv, dJv, f, patch, body, Q, RP, RG, r, want, slide=(0.0, 0.0, 0.0),
            free=(False, False), torque=None, released=True, stops=None, gives=None, hands=math.inf):
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
    forces: a contact pulled on goes free, one dragged past its box |z_t| <= mu z_n slides, its friction the box's;
    a hand or let-go foot only her posture pulls on bears what it feels, a row beside the tasks (her rest's pulled her
    fists and feet free, -1 to -15 N; what she sits or lies on goes free: held, her sit-up off the bed stalled).
    A patch felt under `regime.hold` N - the segments one surface bears, `patch` (k): a fist's hand and fingers, the
    pelvis's two ischia - starts free, its points with it: her fingers' arrival at 2-4 N, held, fought her arms' pose
    and threw her elbow; her fists' 93 N split over hand and fingers, each under 30, left them out of the chain's
    support at the bed. A hand's patch or a sole patch (`borne.CARPAL`'s, `body` below 0) asked past what it feels
    by GROW_N (`rounds`, the upright step's own cap), a hand's past what her text tells them (`capped`), is held at
    that cap, its friction boxed by it - what she sits or
    lies on is not: uncapped, on the bed's long side the plan asked her arms 250-554 N on 15-70 felt and her standing
    sole 273 N on 181, unloaded her pelvis, and she slid off the bed onto that sole; her seat capped too, it took 12 N
    of her 329 as she sat on the floor and she lay back. Told a twist (`slide`), her sled - no hand or foot - slides
    along it at mu what it feels (held, it locked her pelvis: 1.4 m/s^2 asked, 0.000 m), its rows at the ratchet's
    split (`said`), on its normal (`frame_rows`); a foot told free starts free (`freed`). The C twin spells every
    product in this order."""
    import numpy as np
    k3, m1, lam2, task2 = 3 * len(Q), len(r), regime.lam * regime.lam, regime.task * regime.task
    eye, zfull, yields = np.eye(nd), np.zeros(k3), np.zeros(len(Q), bool)
    felt = [math.sqrt(f[3 * i] * f[3 * i] + f[3 * i + 1] * f[3 * i + 1] + f[3 * i + 2] * f[3 * i + 2]) for i in range(len(Q))]
    caps = capped(felt, patch, hands)
    fs = said(f, patch, body, regime.unweight) if slide[0] != 0.0 or slide[2] != 0.0 else f
    srcs = (Jp, Jg, Jv, dJv, fs)
    rows3, Fk, framed, (JpF, JgF, JvF, dJvF, fsF) = started(regime, felt, fs, patch, body, Q, slide, free, srcs)
    x, p0, Pg, qg = np.zeros(nd), np.zeros(len(rhs0)), np.zeros((len(rhs0), nd + len(held))), np.concatenate([np.zeros(nd), held])
    for _ in range(regime.rounds):
        Ja_p, Ja_g, na = JpF[rows3], JgF[rows3], int(rows3.sum())
        rhsk = rhs0 + np.ascontiguousarray(Jp.T) @ Fk if k3 else rhs0
        MM = Minv @ Mpg
        Dinv, Bg, c0 = np.zeros((0, 0)), np.zeros((0, len(MM[0]))), np.zeros(0)
        if na:
            Ja_pT, Dl, Bg, c0 = held_map(Ja_p, Ja_g, Minv, Mpg, rhsk, dJvF[rows3], JvF[rows3], fsF[rows3], lam2, regime.bleed,
                                         None if gives is None else np.repeat(np.asarray(gives, float), 3)[rows3])
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
            zc = Nint @ (Dinv @ (c0 - np.ascontiguousarray(Bg[:, nd:]) @ held) - fsF[rows3])
        else:
            p0, Pg = Minv @ rhsk, -MM
        PgD, PgH = np.ascontiguousarray(Pg[:, :nd]), np.ascontiguousarray(Pg[:, nd:])
        R1x = RP @ PgD + np.ascontiguousarray(RG[:, :nd])
        r1x = (r - RP @ p0) - (RP @ PgH + np.ascontiguousarray(RG[:, nd:])) @ held
        if na:
            R1x = np.vstack([R1x, regime.force * Zx])
            r1x = np.concatenate([r1x, regime.force * zc])
            for i in np.flatnonzero(yields & rows3[0::3] & rows3[1::3] & rows3[2::3]):
                at, nrm = int(rows3[:3 * i].sum()), Q[i, 0]
                R1x = np.vstack([R1x, nrm @ (Dinv[at:at + 3] @ np.ascontiguousarray(Bg[:, :nd]))])
                r1x = np.append(r1x, nrm @ (Fk[3 * i:3 * i + 3] + Dinv[at:at + 3] @ (c0 - np.ascontiguousarray(Bg[:, nd:]) @ held)
                                            - fs[3 * i:3 * i + 3]))
        if m1 + na:
            R1xT = np.ascontiguousarray(R1x.T)
            G1 = R1xT @ R1x
            H1 = G1 + task2 * eye
            x1 = np.linalg.solve(H1, R1xT @ r1x)
            N = eye - np.linalg.solve(H1, G1)
        else:
            x1, N = np.zeros(nd), eye
        # her drives' clamps and her joints' stops on both levels (`torqued`, `within`): none past them, the plan as it was
        bounds = torqued(torque, rows3, framed, Q, Fk, held, p0, Pg, Dinv, Bg, c0, nd, na, regime.mu, stops)
        if bounds is not None:
            x1 = within(H1 if m1 + na else task2 * eye, -(R1xT @ r1x) if m1 + na else np.zeros(nd), bounds[0], bounds[1], x1,
                        bounds[2], bounds[3], regime.force * regime.force)
        R2n = regime.posture * N
        R2nT = np.ascontiguousarray(R2n.T)
        H2 = R2nT @ R2n + lam2 * eye
        x2 = np.linalg.solve(H2, R2nT @ (regime.posture * (want - x1)))
        if bounds is not None:
            x2 = within(H2, -(R2nT @ (regime.posture * (want - x1))), bounds[0] @ N, bounds[1] - bounds[0] @ x1, x2,
                        bounds[2] @ N, bounds[3] - bounds[2] @ x1, regime.force * regime.force)
        x = x1 + N @ x2
        qg = np.concatenate([x, held])
        if not na:
            zfull = Fk.copy()
            break
        za = Dinv @ (c0 - Bg @ qg)
        zfull, z1, za1 = Fk.copy(), Fk.copy(), Dinv @ (c0 - Bg @ np.concatenate([x1, held]))
        zfull[rows3] = zfull[rows3] + za
        z1[rows3] = z1[rows3] + za1
        unframed(zfull, z1, za, za1, rows3, Fk, Q, framed)
        changed = False
        for i in range(len(Q)):
            if not rows3[3 * i]:
                continue
            loc = Q[i] @ zfull[3 * i:3 * i + 3]
            zn, zt1, zt2 = float(loc[0]), float(loc[1]), float(loc[2])
            if (zn < 0.0 and not yields[i] and bool(rows3[3 * i + 1]) and bool(rows3[3 * i + 2]) and int(body[i]) >= 0
                    and int(patch[i]) >= CARPAL and float(Q[i, 0] @ z1[3 * i:3 * i + 3]) >= 0.0):
                yields[i], changed = True, True
            elif zn < 0.0:
                rows3[3 * i:3 * i + 3], Fk[3 * i:3 * i + 3], changed = False, 0.0, True
            elif (int(body[i]) < 0 or CARPAL <= int(patch[i]) < LET) and zn > caps[i]:
                cap = caps[i]
                lim = regime.mu * cap
                tb1, tb2 = (-lim if zt1 < -lim else (lim if zt1 > lim else zt1)), (-lim if zt2 < -lim else (lim if zt2 > lim else zt2))
                for c in range(3):
                    Fk[3 * i + c] = float(Q[i, 0, c]) * cap + float(Q[i, 1, c]) * tb1 + float(Q[i, 2, c]) * tb2
                rows3[3 * i:3 * i + 3], changed = False, True
            elif released and rows3[3 * i + 1] and max(abs(zt1), abs(zt2)) > regime.mu * zn:
                # not `released`, one past its box stays held, its slip the step's Gauss-Seidel's
                lim = regime.mu * zn
                tb1, tb2 = (-lim if zt1 < -lim else (lim if zt1 > lim else zt1)), (-lim if zt2 < -lim else (lim if zt2 > lim else zt2))
                for c in range(3):
                    Fk[3 * i + c] = float(Q[i, 1, c]) * tb1 + float(Q[i, 2, c]) * tb2
                rows3[3 * i + 1:3 * i + 3], changed = False, True
        if not changed:
            break
    return x, p0 + Pg @ qg, zfull
