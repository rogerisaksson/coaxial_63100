"""The closed chains' step as one QP: her tasks and the contacts' forces as one within friction.

    width, S, mu, bears, sled, mode, st = chain_qp.unknowns(patch, body, felt, Q, slide, free, regime, Jv)   # zeta's blocks
    holds = [not loose[i] and keys[i] not in let for i in range(k)]                        # keys = chain_qp.keyed(..)
    maps = chain_qp.affine(Minv, Mpg, rhs0, held_q, Jp, Jg, dJv, Q, S, RP, RG, r, M_A, h_A, J_A, nd)
    H1, g1, G1, A, bnd = chain_qp.level1(maps, S, width, mu, holds, stick, mode, st, gives, Jv, f, Q, regime, bounds, stops, mass)
    u, warm, met = chain_qp.solved(H1, g1, G1, A, bnd, maps, S, Jg, hands, want, regime, warm)
    x, qdd_P, z, tau, residual, al = chain_qp.answer(u, maps, S, Q, nd)
    let = chain_qp.letting(keys, holds, let_now, warm[0], chain_qp.zrows(holds, width, mode), al)   # for the next pass

`machine.stack.chain.step` runs it (closed_chain_qp.lisp says it); wbc_chain_qp.c is it in C, bit for bit.
"""
import collections
import math

from machine.senses.borne import CARPAL, LET

#: Level 1's affine maps in u = (x, zeta): the dofs no drive holds qdd_P = p0 + Px x + Pz zeta, the contacts'
#: accelerations in their frames a0 + Ax x + Az zeta, her tasks Rx x + Rz zeta = rr, her torques t0 + Tx x + Tz zeta.
Maps = collections.namedtuple('Maps', 'p0 Px Pz a0 Ax Az Rx Rz rr t0 Tx Tz')
#: The friction pyramid's eight directions on the tangents (k pi / 4) and its faces' cos(pi / 8): inscribed in the cone.
ROOT_HALF = math.sqrt(0.5)
OCTAGON = ((1.0, 0.0), (ROOT_HALF, ROOT_HALF), (0.0, 1.0), (-ROOT_HALF, ROOT_HALF), (-1.0, 0.0), (-ROOT_HALF, -ROOT_HALF),
           (0.0, -1.0), (ROOT_HALF, -ROOT_HALF))
COS8 = math.sqrt(0.5 + 0.5 * ROOT_HALF)


def freed(patch, body, free):
    """Whether a contact is on a foot told free (`free`, left and right: her heels stepping in turn): a let-go foot's
    patch (`borne.LET`'s) or a standing sole (`body` below 0)."""
    foot = patch // LET - 1 if patch >= LET else (-1 - body if body < 0 else -1)
    return foot >= 0 and bool(free[foot])


def unknowns(patch, body, felt, Q, slide, free, regime, Jv):
    """(width (k), S (3k, nz), mu (k), bears (k), sled (k), mode (k), st (k, 2)): each contact's unknowns in zeta -
    three in its frame, or one for her sled (a body of hers, a patch under CARPAL: what she sits or lies on) told a
    twist (`slide`, unit) on a level slide - its normal up within 45 deg and along the way under `regime.tilt`; a back
    against a tilted rest braces, no slide rows - and sliding past `regime.slide` along it (`Jv`): kinetic (`mode` 2),
    it brakes at its friction's edge against its slip, z = z_n (n - mu sign(v . s_t) s_t), s_t the way's unit tangent
    in its frame (`st`); at rest such a sled is static (`mode` 1: its shear within the pyramid, dissipative, drawn to
    the edge by `level1`'s soft pin); `mode` 0 any other -; the map zeta -> z in frames; its friction (her sled's
    `regime.mu`, an anchor's `regime.anchor`); whether it bears - its patch `regime.hold`, its points together, and its
    foot not told free (`freed`): an anchor's tangents stick then - and whether it is her sled."""
    import numpy as np
    k = len(Q)
    width, mu, held, sleds, mode, cols = [], [], [], [], [], []
    st = np.zeros((k, 2))
    for i in range(k):
        sled = int(body[i]) >= 0 and int(patch[i]) < CARPAL
        by = sum(float(patch[j] == patch[i]) * felt[j] for j in range(k))
        held.append(by >= regime.hold and not freed(int(patch[i]), int(body[i]), free))
        mu.append(regime.mu if sled else regime.anchor)
        sleds.append(sled)
        sn = (float(Q[i, 0, 0]) * slide[0] + float(Q[i, 0, 1]) * slide[1]) + float(Q[i, 0, 2]) * slide[2]
        col, m = None, 0
        if sled and float(Q[i, 0, 1]) >= ROOT_HALF and abs(sn) < regime.tilt and (slide[0] != 0.0 or slide[2] != 0.0):
            t = [slide[c] - sn * float(Q[i, 0, c]) for c in range(3)]
            nt = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
            if nt > 0.0:
                jv = [float(Jv[3 * i + c]) for c in range(3)]
                vt1 = (float(Q[i, 1, 0]) * jv[0] + float(Q[i, 1, 1]) * jv[1]) + float(Q[i, 1, 2]) * jv[2]
                vt2 = (float(Q[i, 2, 0]) * jv[0] + float(Q[i, 2, 1]) * jv[1]) + float(Q[i, 2, 2]) * jv[2]
                st[i, 0] = ((float(Q[i, 1, 0]) * t[0] + float(Q[i, 1, 1]) * t[1]) + float(Q[i, 1, 2]) * t[2]) / nt
                st[i, 1] = ((float(Q[i, 2, 0]) * t[0] + float(Q[i, 2, 1]) * t[1]) + float(Q[i, 2, 2]) * t[2]) / nt
                vs = vt1 * float(st[i, 0]) + vt2 * float(st[i, 1])
                m = 2 if abs(vs) > regime.slide else 1
                if m == 2:
                    sgn = 1.0 if vs > 0.0 else -1.0
                    col = [1.0, -(mu[i] * (sgn * float(st[i, 0]))), -(mu[i] * (sgn * float(st[i, 1])))]
        mode.append(m)
        width.append(1 if col is not None else 3)
        cols.append(col)
    S, at = np.zeros((3 * k, sum(width))), 0
    for i in range(k):
        if width[i] == 1:
            S[3 * i:3 * i + 3, at] = cols[i]
        else:
            S[3 * i, at], S[3 * i + 1, at + 1], S[3 * i + 2, at + 2] = 1.0, 1.0, 1.0
        at += width[i]
    return width, S, mu, held, sleds, mode, st

def keyed(patch, body):
    """Each contact's key (body, patch, its ordinal among those of both): what the plan let go is remembered by."""
    keys, seen = [], {}
    for i in range(len(patch)):
        kb = (int(body[i]), int(patch[i]))
        keys.append(kb + (seen.get(kb, 0),))
        seen[kb] = seen.get(kb, 0) + 1
    return keys


def zrows(holds, width, mode):
    """Each contact's row z_n >= 0 among level 1's laws (`level1`'s order: 2 rows held, 3 free, its pyramid's 8 where
    its tangents are unknown, a static sled's 2)."""
    out, at = [], 0
    for h, w, m in zip(holds, width, mode):
        out.append(at)
        at += (2 if h else 3) + (8 if w == 3 else 0) + (2 if m == 1 else 0)
    return out

def letting(keys, holds, let, held1, zrow, al):
    """The contacts the plan lets go of for the next pass: one held whose row z_n >= 0 level 1 held (`held1`: the plan
    would pull on it), one let already (`let`) the plan still lifts (al_n > 0). Complementarity resolved pass by pass
    from the plan's own multiplier - a held contact presses with no cap, a let one carries at most what it feels and
    may leave, its skin frees it as its force goes -, no release told by a text."""
    return {keys[i] for i in range(len(keys)) if (holds[i] and zrow[i] in held1) or (let[i] and al[3 * i] > 0.0)}


def framed(Q, M):
    """`M` (3k rows) with each contact's three rows turned into its frame `Q` (rows: normal, two tangents): T^T M."""
    import numpy as np
    out = np.zeros(M.shape)
    for i in range(len(Q)):
        for a in range(3):
            out[3 * i + a] = (float(Q[i, a, 0]) * M[3 * i] + float(Q[i, a, 1]) * M[3 * i + 1]) + float(Q[i, a, 2]) * M[3 * i + 2]
    return out


def affine(Minv, Mpg, rhs0, held_q, Jp, Jg, dJv, Q, S, RP, RG, r, M_A, h_A, J_A, nd):
    """The `Maps` of u = (x, zeta): her dynamics on the dofs no drive holds (M_PP qdd_P + M_PG qg + h_P = J_P^T z, qg =
    (x, `held_q`) with the held joints' accelerations known) make qdd_P affine in u, and with it the contacts'
    accelerations in their frames (`framed`), her tasks' rows (RP on P, RG on the given) and her drives' torques
    (`M_A` her mass matrix's driven rows over (P, given), `h_A`, `J_A` the contacts' Jacobians' driven columns). The C
    twin spells every product in this order."""
    import numpy as np
    k3 = len(dJv)
    TS = np.zeros(S.shape)
    for i in range(len(Q)):
        TS[3 * i:3 * i + 3] = np.ascontiguousarray(Q[i].T) @ S[3 * i:3 * i + 3]
    MpgD, MpgH = np.ascontiguousarray(Mpg[:, :nd]), np.ascontiguousarray(Mpg[:, nd:])
    JgD, JgH = np.ascontiguousarray(Jg[:, :nd]), np.ascontiguousarray(Jg[:, nd:])
    p0 = Minv @ (rhs0 - MpgH @ held_q)
    Px = -(Minv @ MpgD)
    Pz = Minv @ (np.ascontiguousarray(Jp.T) @ TS) if k3 else np.zeros((len(rhs0), 0))
    c0 = (Jp @ p0 + JgH @ held_q) + dJv
    a0, Ax, Az = framed(Q, c0.reshape(-1, 1)).ravel(), framed(Q, Jp @ Px + JgD), framed(Q, Jp @ Pz)
    Rx, Rz, rr = RP @ Px + np.ascontiguousarray(RG[:, :nd]), RP @ Pz, (r - RP @ p0) - np.ascontiguousarray(RG[:, nd:]) @ held_q
    npp = len(rhs0)
    MAP, MAD, MAH = (np.ascontiguousarray(M_A[:, :npp]), np.ascontiguousarray(M_A[:, npp:npp + nd]),
                     np.ascontiguousarray(M_A[:, npp + nd:]))
    t0 = (MAP @ p0 + MAH @ held_q) + h_A
    Tx = MAP @ Px + MAD
    Tz = MAP @ Pz - np.ascontiguousarray(J_A.T) @ TS
    return Maps(p0, Px, Pz, a0, Ax, Az, Rx, Rz, rr, t0, Tx, Tz)


def level1(maps, S, width, mu, held, stick, mode, st, gives, Jv, f, Q, regime, bounds, stops, mass):
    """(H1, g1, G1, A, b): level 1's least squares - her tasks' rows; each held contact's normal stick row at
    `regime.stick`, al_n + give (z_n - f_n) = -bleed v_n (its support giving as the force grows past what is felt);
    an anchor's tangents' al_t = -bleed v_t where it bears (`stick`), its slip bled - her sled's tangents are its
    friction's within the pyramid, no lock (held still on a mattress her thighs locked her hip and knee against a leg
    swing and she fell) -; a static sled's (`mode` 1) soft pin at `regime.pin` over her `mass`, z_t . s_t = -mu z_n:
    drawn to the cone's edge against the way (`st`), the hands push and the seat unloads until it goes; the forces
    regularised: each contact's tangents toward none at `regime.reg`, its normal toward what is felt at `regime.nreg`
    (a force is a state, changing it is what costs: toward none a touching hand was unloaded and its give lifted it
    off the seat); x damped at `regime.task` - and the laws A u <= b: a contact never sinks into its support faster than its give says (it may
    leave faster); held (`held`: touching, not let go nor on a foot told free) it presses alone, z_n >= 0, with no
    cap on its growth (felt + hold was the upright step's GROW_N: it froze a scoot's hands at what they felt); free it
    carries at most what it feels, 0 <= z_n <= f_n, free to unload and leave - no lever the world never gave (a sole
    felt at 10 N was planned at 262 N while leaving the floor at 8.7 m/s^2); its friction within the eight-sided
    pyramid inscribed in its cone (`OCTAGON`, `COS8`) where its tangents are unknown; a static sled's shear along the
    way dissipative alone, z_t . s_t <= 0 (the seat never helps), and never planned to slide against the way, al_t .
    s_t >= 0 - with the pin's soft row that is static friction: z_t . s_t = -F while the hands' F is under the cone,
    a = 0, no backward motor and no reaction wheel (a soft pin alone gives z = (m a* - F - w mu N) / (1 + w), a
    motor again); a kinetic sled (`mode` 2, one unknown) brakes at the edge against its slip and may slow -; her
    torques within `bounds` (lo, hi); her joints' stops (`stops`: rows on x, bounds) where asked; the static sleds'
    brake together no more than her anchors' friction can drive, -sum z_t . s_t <= sum mu_a z_n over the anchors whose
    tangents stick (a static friction is a reaction: pinned at the cone with her hands bearing 7 N the plan had a
    -165 N motor, asked -5.6 m/s^2 backward and pitched her 28 deg onto the backrest). Rows a contact: 2 held, 3 free,
    its pyramid's 8 where its tangents are unknown, a static sled's 2; then her torques', her stops', the brake's."""
    import numpy as np
    nd, nz, k = maps.Px.shape[1], S.shape[1], len(Q)
    nu, bleed = nd + nz, regime.bleed
    rows, rhs, A, b = [np.hstack([maps.Rx, maps.Rz])], [maps.rr], [], []
    vl, fl = framed(Q, np.asarray(Jv, float).reshape(-1, 1)).ravel(), framed(Q, np.asarray(f, float).reshape(-1, 1)).ravel()
    w_pin = regime.pin / mass
    for i in range(k):
        give = float(gives[i])
        zn = np.concatenate([np.zeros(nd), S[3 * i]])
        normal = np.concatenate([maps.Ax[3 * i], maps.Az[3 * i] + give * S[3 * i]])
        right = (-bleed * vl[3 * i]) + give * fl[3 * i] - maps.a0[3 * i]
        along = np.concatenate([np.zeros(nd), float(st[i, 0]) * S[3 * i + 1] + float(st[i, 1]) * S[3 * i + 2]])
        if held[i]:
            rows.append(regime.stick * normal.reshape(1, -1))
            rhs.append(np.array([regime.stick * right]))
            for a in (1, 2):
                if width[i] == 3 and stick[i]:
                    rows.append(regime.stick * np.concatenate([maps.Ax[3 * i + a], maps.Az[3 * i + a]]).reshape(1, -1))
                    rhs.append(np.array([regime.stick * ((-bleed * vl[3 * i + a]) - maps.a0[3 * i + a])]))
        if mode[i] == 1:
            rows.append((w_pin * (along + mu[i] * zn)).reshape(1, -1))
            rhs.append(np.array([0.0]))
        A.append(-zn)
        b.append(0.0)
        A.append(-normal)
        b.append(-right)
        if not held[i]:
            A.append(zn)
            b.append(fl[3 * i])
        if width[i] == 3:
            for d1, d2 in OCTAGON:
                A.append(np.concatenate([np.zeros(nd), (d1 * S[3 * i + 1] + d2 * S[3 * i + 2]) - (mu[i] * COS8) * S[3 * i]]))
                b.append(0.0)
        if mode[i] == 1:
            A.append(along)
            b.append(0.0)
            A.append(-(float(st[i, 0]) * np.concatenate([maps.Ax[3 * i + 1], maps.Az[3 * i + 1]])
                       + float(st[i, 1]) * np.concatenate([maps.Ax[3 * i + 2], maps.Az[3 * i + 2]])))
            b.append(float(st[i, 0]) * maps.a0[3 * i + 1] + float(st[i, 1]) * maps.a0[3 * i + 2])
    L, e = np.vstack(rows), np.concatenate(rhs)
    LT = np.ascontiguousarray(L.T)
    G1 = LT @ L
    # each contact's tangents regularised toward none at `regime.reg`, its normal toward what is felt at `regime.nreg`
    damp = [regime.task * regime.task] * nd
    for i in range(k):
        damp += [regime.nreg * regime.nreg] + [regime.reg * regime.reg] * (width[i] - 1)
    H1 = G1 + np.diag(np.array(damp))
    g1 = -(LT @ e)
    at = nd
    for i in range(k):
        g1[at] = g1[at] - (regime.nreg * regime.nreg) * fl[3 * i]
        at += width[i]
    T = np.hstack([maps.Tx, maps.Tz])
    A += [T, -T]
    b += [bounds[1] - maps.t0, maps.t0 - bounds[0]]
    if stops is not None:
        A.append(np.hstack([stops[0], np.zeros((len(stops[1]), nz))]))
        b.append(stops[1])
    if any(m == 1 for m in mode):
        # the static sleds' brake together no more than her anchors' friction can drive: a static friction is a reaction
        # (pinned at the cone with her hands bearing 7 N the plan had a -165 N motor and pitched her onto the backrest)
        cap = np.zeros(nu)
        for i in range(k):
            if mode[i] == 1:
                cap -= np.concatenate([np.zeros(nd), float(st[i, 0]) * S[3 * i + 1] + float(st[i, 1]) * S[3 * i + 2]])
            elif stick[i]:
                cap -= mu[i] * np.concatenate([np.zeros(nd), S[3 * i]])
        A.append(cap)
        b.append(0.0)
    return H1, g1, G1, np.vstack([np.atleast_2d(a) for a in A]), np.concatenate([np.atleast_1d(v) for v in b])

def projector(Jg, hands, nd, lam2):
    """P (nd, nd): her posture's rows off her held hands' motion, I - J^T (J J^T + lam2 I)^-1 J, J the held hand
    contacts' (`hands`, their rows of `Jg`) linear Jacobians on her driven joints; I with none."""
    import numpy as np
    if not len(hands):
        return np.eye(nd)
    J = np.ascontiguousarray(np.vstack([Jg[3 * i:3 * i + 3, :nd] for i in hands]))
    JT = np.ascontiguousarray(J.T)
    K = J @ JT
    for i in range(len(K)):
        K[i, i] = K[i, i] + lam2
    return np.eye(nd) - JT @ np.linalg.solve(K, J)


def solved(H1, g1, G1, A, b, maps, S, Jg, hands, want, regime, warm):
    """(u, (held1, held2), met): level 1 by the dual active set warm from `warm`'s rows (`qp.solve`; none meeting the
    laws, the least squares alone, `met` False), level 2 - her posture, `regime.posture` weighed, its rows through
    `projector` - in level 1's damped null space N = I - H1^-1 G1 under the laws shifted."""
    import numpy as np
    from machine.stack import qp
    nd, nu, lam2 = maps.Px.shape[1], len(g1), regime.lam * regime.lam
    u1, held1, _u = qp.solve(H1, g1, A, b, warm[0])
    met = u1 is not None
    if not met:
        u1, held1 = np.linalg.solve(H1, -g1), ()
    N = np.eye(nu) - np.linalg.solve(H1, G1)
    P = projector(Jg, hands, nd, lam2)
    L2 = np.hstack([regime.posture * P, np.zeros((nd, nu - nd))])
    R2n = L2 @ N
    R2nT = np.ascontiguousarray(R2n.T)
    e2 = regime.posture * (P @ (want - u1[:nd]))
    H2 = R2nT @ R2n
    for i in range(nu):
        H2[i, i] = H2[i, i] + lam2
    g2 = -(R2nT @ e2)
    y, held2, _u = qp.solve(H2, g2, A @ N, b - A @ u1, warm[1]) if met else (None, (), None)
    if y is None:
        y, held2 = np.linalg.solve(H2, -g2), ()
    return u1 + N @ y, (held1, held2), met


def answer(u, maps, S, Q, nd):
    """(x, qdd_P, z (3k, world), tau, residual, al): the answer's parts, the complementarity's residual the largest
    |min(al_n, z_n)| over the contacts, `al` the contacts' accelerations in their frames (3k)."""
    import numpy as np
    x, zeta = u[:nd], u[nd:]
    zl = S @ zeta
    z, residual = np.zeros(len(zl)), 0.0
    al = (maps.a0 + maps.Ax @ x) + maps.Az @ zeta
    for i in range(len(Q)):
        z[3 * i:3 * i + 3] = np.ascontiguousarray(Q[i].T) @ zl[3 * i:3 * i + 3]
        gap = abs(min(float(al[3 * i]), float(zl[3 * i])))
        residual = gap if gap > residual else residual
    return x, (maps.p0 + maps.Px @ x) + maps.Pz @ zeta, z, (maps.t0 + maps.Tx @ x) + maps.Tz @ zeta, residual, al
