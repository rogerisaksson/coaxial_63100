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


def planned(regime, nd, held, Minv, Mpg, rhs0, Jp, Jg, Jv, dJv, f, patch, body, Q, RP, RG, r, want, slide=(0.0, 0.0, 0.0),
            free=(False, False)):
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
    by GROW_N (`rounds`, the upright step's own cap) is held at that cap, its friction boxed by it - what she sits or
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
            Ja_pT = np.ascontiguousarray(Ja_p.T)
            W = Ja_p @ Minv
            Dl = W @ Ja_pT
            for i in range(na):
                Dl[i, i] = Dl[i, i] + lam2
            bb0 = W @ rhsk + dJvF[rows3]
            Bg = Ja_g - W @ Mpg
            c0 = (-regime.bleed) * JvF[rows3] - bb0
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
            elif (int(body[i]) < 0 or CARPAL <= int(patch[i]) < LET) and zn > felt[i] + GROW_N:
                cap = felt[i] + GROW_N
                lim = regime.mu * cap
                tb1, tb2 = (-lim if zt1 < -lim else (lim if zt1 > lim else zt1)), (-lim if zt2 < -lim else (lim if zt2 > lim else zt2))
                for c in range(3):
                    Fk[3 * i + c] = float(Q[i, 0, c]) * cap + float(Q[i, 1, c]) * tb1 + float(Q[i, 2, c]) * tb2
                rows3[3 * i:3 * i + 3], changed = False, True
            elif rows3[3 * i + 1] and max(abs(zt1), abs(zt2)) > regime.mu * zn:
                lim = regime.mu * zn
                tb1, tb2 = (-lim if zt1 < -lim else (lim if zt1 > lim else zt1)), (-lim if zt2 < -lim else (lim if zt2 > lim else zt2))
                for c in range(3):
                    Fk[3 * i + c] = float(Q[i, 1, c]) * tb1 + float(Q[i, 2, c]) * tb2
                rows3[3 * i + 1:3 * i + 3], changed = False, True
        if not changed:
            break
    return x, p0 + Pg @ qg, zfull
