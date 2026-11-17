"""The upright step's bounded solve (wbc_rounds.c): its rows' least squares within its bounds.

    u, first = within.within(H0, g0, held, bears, lo, hi, geo, stride)   # `wrench.step`'s solve
    rows, rhs = within.bounds(nu, frames, stance, edges, mu, caps, pressed, bears, slips, give)
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.stack import qp
from machine.stack.rounds import CONE, COS16, GROW_N, NU, ROOM, YAW_M, taken
from machine.stack.wbc import MU, G

#: Striding, a standing sole's wrench kept in the projection's metric, of its share of the robot's weight:
#: its soles pushed against each other change its motion by nothing; unkept the faces were met
#: through it - 926 N along one sole, -906 the other - till a sole pulled and both were let go, 11 times in 12
#: s of the robot's walk. At 10 none, the robot's walk 418 J/m, 2.53 off a woman's (412, 2.61); at 30 2.84; at
#: 100 let go 187 times. And in the bounded solve (`within`), of the acceleration it gives the robot: two standing
#: soles' inner wrench - what their net says nothing of, past its least in the part metric - kept least (unkept, a
#: throw asked past their room parted them 340 and 131 N for a 1 mrad tilt's 0.007 N m of pitch and a 5 ms hop never
#: left; their wrenches kept at the rows' unbounded answer instead, the robot was laid off its bed's side and leant
#: in a turn's single support); a sliding contact's push kept near what it bears, its friction given (unkept, the
#: step let it go to be rid of its friction).
W_KEEP = 3.0
#: A solve with no answer is solved again with this share of its largest diagonal on its free unknowns:
#: falling on a ramp, two bearing points 16 mm apart left the rounds' matrix rank 21 of 27 (cond 2.9e19),
#: their split said by no row, and the solve raised.
RIDGE = 1e-12


def solved(K, rhs, free):
    """K^-1 rhs, K ridged by RIDGE on its `free` unknowns where it has no inverse."""
    try:
        return np.linalg.solve(K, rhs)
    except np.linalg.LinAlgError:
        return np.linalg.solve(K + np.diag(free * RIDGE * float(np.diag(K).max())), rhs)


def bounds(nu, frames, stance, edges, mu, caps, pressed, bears, slips, give):
    """([rows], [right-hand sides]): each standing sole's bounds and each contact's as rows A u <= b, in wbc_rounds.c's
    order - a sole pressing, its friction's pyramid along the world's x and z, its centre of pressure in its room
    along it (`edges`: 1 its ball, -1 its heel, a share between) and across it, its yaw moment's; a contact
    pressing, no more than GROW_N over what it `bears` nor its cap, of that `give` the robot's own to plan (none
    where it is not `pressed`), its friction within the sixteen-sided pyramid inscribed in its cone (`rounds.CONE`)
    - or, sliding (`slips`: sliding, its slip along the world's x and z), its kinetic friction against its
    slip."""
    rows, rhs = [], []

    def row(entries, c=0.0):
        a = np.zeros(nu)
        for i, v in entries:
            a[i] = v
        rows.append(a)
        rhs.append(c)
    for k, R in enumerate(frames):
        if stance[k] <= 0.0:
            continue
        fy, e, muk = 6 * k + 4, float(edges[k]), float(mu[k])
        row([(fy, -1.0)])
        for f in (6 * k + 3, 6 * k + 5):
            row([(f, 1.0), (fy, -muk)])
            row([(f, -1.0), (fy, -muk)])
        hi0, lo0 = ROOM[1] * (1.0 - 2.0 * max(e, 0.0)), -ROOM[1] * (1.0 - 2.0 * max(-e, 0.0))
        axis = [[(6 * k + i, float(R[i, j])) for i in range(3)] for j in range(3)]
        row(axis[0] + [(fy, -hi0)])
        row([(i, -v) for i, v in axis[0]] + [(fy, lo0)])
        for j, cj in ((1, muk * YAW_M), (2, float(ROOM[0]))):
            row(axis[j] + [(fy, -cj)])
            row([(i, -v) for i, v in axis[j]] + [(fy, -cj)])
    for i in range(len(bears)):
        fy, sliding, sx, sz = NU + 3 * i + 1, float(slips[i][0]), float(slips[i][1]), float(slips[i][2])
        den = max(float(np.hypot(sx, sz)), 1e-12)
        row([(fy, -1.0)])
        row([(fy, 1.0)], max(min(float(bears[i]) + GROW_N, float(caps[i])) * give, 0.0) if pressed[i] else 0.0)
        if sliding > 0.0:
            for f, sf in ((NU + 3 * i, sx), (NU + 3 * i + 2, sz)):
                c = MU * sf / den
                row([(f, 1.0), (fy, c)])
                row([(f, -1.0), (fy, -c)])
        else:
            for d1, d2 in CONE:
                row([(NU + 3 * i, d1), (NU + 3 * i + 2, d2), (fy, -(MU * COS16))])
    return rows, rhs


def within(H0, g0, held, bears, lo, hi, geo, stride):
    """(The unknowns, the first answer - the floor unbounded) for the rows' normal matrix `H0` and
    right-hand side `g0` (R^T c), the contacts' `held` rows (`bears`, N up), the robot's height's band `lo` to `hi`:
    the rows' least squares within each standing sole's and each contact's bounds (`bounds`) and the band where
    grounded - let go where nothing meets it -, by the stack's dual active set (`qp.solve`, wbc_qp.c), each call its
    own; then the stride. A contact told to bear nothing is let go. Clipped round by round and the clipped
    pinned for the next, a sole near 0 N flipped its pins pass to pass and bought an inner force pair: landing on a
    chair the robot rolled onto one buttock, 646 N to 0, and sat 10 deg askew and 7 cm aside, its rise's scoot
    lost. Two standing soles' inner wrench is kept least, a sliding contact's push near what it bears (W_KEEP).
    `geo`: (nu, k3,
    moment, part, up, lift, mass, grounded, stance, frames, edges, mu, caps, slips, give - the share of a contact's
    push the robot's own to plan -, net - the two standing soles' wrenches' net about the centre of mass (6, 12), None
    but where both stand); `stride`: (strides, its kinetic metric)."""
    nu, _k3, _moment, part, up, lift, mass, grounded, stance, frames, edges, mu, cap, slips, give, net = geo
    strides, kinetic = stride
    caps = np.broadcast_to(cap, (len(bears),))
    pressed = (caps > 0.0).astype(float)
    if held:
        Hr = H0 + sum(p * S.T @ S for p, (S, _r) in zip(pressed, held))
        gr = g0 + sum(p * S.T @ r for p, (S, r) in zip(pressed, held))
    else:
        Hr, gr = H0, g0
    ones = np.ones(nu)
    first = solved(Hr, gr, ones)
    slid = np.array([float(sl[0] > 0.0) for sl in slips])
    keep = (W_KEEP * G) * np.concatenate([np.zeros(NU), np.outer(slid, [0.0, 1.0, 0.0]).ravel()]) * part
    Hk = Hr + np.diag(keep * keep)
    if net is not None:
        w2 = part[:12] * part[:12]
        least = (net.T / w2[:, None]) @ np.linalg.solve((net / w2[None, :]) @ net.T, net)
        inner = (W_KEEP * G) * (part[:12][:, None] * (np.eye(12) - least))
        Hk[:12, :12] = Hk[:12, :12] + inner.T @ inner
    gk = gr + keep * keep * np.concatenate([np.zeros(NU), np.outer(np.asarray(bears, float), [0.0, 1.0, 0.0]).ravel()])
    rows, rhs = bounds(nu, frames, stance, edges, mu, caps, pressed, bears, slips, give)
    banded = grounded > 0.0
    if banded:
        rows, rhs = rows + [lift / mass, -lift / mass], rhs + [hi + G, -(lo + G)]
    A, b = np.array(rows).reshape(-1, nu), np.array(rhs, float)

    def answered(H):
        u, act, _mult = qp.solve(H, -gk, A, b)
        if u is None and banded:
            u, act, _mult = qp.solve(H, -gk, A[:-2], b[:-2])
        return u, act
    try:
        u, act = answered(Hk)
    except np.linalg.LinAlgError:
        u, act = None, ()
    if u is None:
        # no inverse, or none found where nothing pushing meets every bound - two bearing points 16 mm apart, their
        # split said by no row -: ridged as `solved` ridges
        u, act = answered(Hk + np.diag(ones * RIDGE * float(np.diag(Hk).max())))
    if u is None:
        raise np.linalg.LinAlgError("no answer within the robot's soles' and contacts' bounds")
    if strides > 0.0 and len(act):
        u = u + strides * (taken(solved(Hr, gr, ones), kinetic, up, frames, stance, edges, (lo, hi), grounded, mu) - u)
    return u, first
