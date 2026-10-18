"""Her body identified of what she measures: each link's inertia and each drive's friction, fitted.

    lay = sysid.laid(body)                          # her links and the columns, on her law's model
    ys, taus, touched = campaign.excite(world, ref) # her edge arrays and her drives' torques a step
    Y, tau = sysid.rows(lay, body, ys, taus, dt)    # the regressor of a campaign's edge arrays
    free = sysid.free(body, ys, every)              # off the stand, the rows the model holds on
    Yb, fb = sysid.borne(lay, body, ys, dt, every)  # her pelvis's rows her insoles measure, standing
    base = sysid.base(lay, Y)                       # the combinations its torques tell apart
    got = sysid.identify(base, Y, tau)              # each one fitted, with what it is worth
    identified.correction(lay, body, Y, tau)        # the least move of her build that fits it

Her drives' torques are linear in what moves her: each link's ten inertial parameters - its
mass, its first moment and its inertia about its frame's origin - and each drive's armature,
viscous and Coulomb friction: tau = Y(q, v, a) theta. Y is Newton-Euler down her tree from her
pelvis, whose IMU gives its turn, spin and specific force (gravity rides in it), her encoders
each joint's angle and rate, their differences the accelerations (`machine.edge`); only her
drives' rows are measured, so her pelvis's own parameters never show and what a link shares
with its neighbours, or what a campaign never moves apart, comes out as a sum: `base` finds
those sums numerically (Gautier) in the campaign's own regressor, and `identify` fits them by
least squares; `machine.identified` moves her build the least that fits the campaign and lays
it over her law's model. On the stand (`mjcf.STAND`) her pelvis is welded to the world and nothing
touches her soles; a robot of her build hangs the same way, its boards' currents its torques.
"""
import math

from machine import edge
from machine.figure import JOINTS

#: A link's inertial parameters, in its frame about its origin.
INERTIAL = ('m', 'hx', 'hy', 'hz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz')
#: A drive's: the rotor's inertia at the joint, kg m^2; viscous, N m s/rad; Coulomb, N m.
FRICTION = ('J', 'b', 'f')
#: A column is one of the base when, scaled, this much of it stands clear of the base before it;
#: one under NONE of the largest is nothing but rounding.
INDEPENDENT, NONE = 1e-3, 1e-9
#: A base parameter is trusted within this of itself.
TRUST = 0.10
#: A drive's row is read only where it slides faster than this, rad/s: at rest its Coulomb
#: friction is whatever holds it, anything within its limit.
STICK = 0.25


def laid(body):
    """Her links and the columns, on her law's model: {bodies (hers, parents first), parent,
    dofs (a body's hinge dofs), hinge (every hinge dof, MuJoCo's order), driven (their indices
    among the hinges, JOINTS order), names, col (a body's ten columns' start, the drives' after
    them)}."""
    m = body.m
    pelvis = body.pelvis
    hers = [b for b in range(m.nbody) if edge._rooted(m, b, pelvis)]
    dofs = {}
    for b in hers:
        first = int(m.body_jntadr[b])
        dofs[b] = [int(m.jnt_dofadr[j]) for j in range(first, first + int(m.body_jntnum[b]))
                   if b != pelvis]
    hinge = [d for b in hers for d in dofs[b]]
    driven = [hinge.index(int(d)) for d in body.act]
    names = ['%s.%s' % (m.body(b).name, p) for b in hers for p in INERTIAL]
    names += ['%s.%s' % (m.joint(int(m.dof_jntid[hinge[k]])).name, p) for k in driven for p in FRICTION]
    return {'bodies': hers, 'parent': {b: int(m.body_parentid[b]) for b in hers}, 'dofs': dofs,
            'hinge': hinge, 'driven': driven, 'names': names,
            'col': {b: 10 * k for k, b in enumerate(hers)}, 'drive': 10 * len(hers)}


def truth(lay, m):
    """theta as a model `m` has her: each body's mass, first moment and inertia about its origin
    in its own frame; each drive's armature, damping and friction loss."""
    import numpy as np
    theta = np.zeros(len(lay['names']))
    for b in lay['bodies']:
        mass, c = float(m.body_mass[b]), np.array(m.body_ipos[b], float)
        r = _rotation(m.body_iquat[b])
        about = r @ np.diag(np.array(m.body_inertia[b], float)) @ r.T
        about += mass * (float(c @ c) * np.eye(3) - np.outer(c, c))
        k = lay['col'][b]
        theta[k], theta[k + 1:k + 4] = mass, mass * c
        theta[k + 4:k + 10] = about[np.triu_indices(3)]
    k = lay['drive']
    for i, j in enumerate(lay['driven']):
        d = lay['hinge'][j]
        theta[k + 3 * i:k + 3 * i + 3] = (m.dof_armature[d], m.dof_damping[d], m.dof_frictionloss[d])
    return theta


def _rotation(q):
    import numpy as np
    w, x, y, z = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def regressor(lay, body, qpos, qvel, qacc, a0, wacc=None):
    """Y (N, hinges + 6, P) at N states of her law's model: `qpos` (N, nq) and `qvel`, `qacc`
    (N, nv) in MuJoCo's convention, `a0` (N, 3) her pelvis's origin's acceleration less gravity,
    world; every hinge's row, as its torque would be in a world without gravity, then her
    pelvis's six: the force on her, world, and the moment about her pelvis's origin in its
    frame - what the floor and whatever else touches her must supply. `wacc` (N, driven) each
    motor's acceleration through its gearbox, the armature's column: the joint's own without."""
    import numpy as np
    m, d, mj = body.m, body.d, body.mj
    n = len(qpos)
    xpos, xmat = np.zeros((n, m.nbody, 3)), np.zeros((n, m.nbody, 3, 3))
    axis, anchor = np.zeros((n, m.njnt, 3)), np.zeros((n, m.njnt, 3))
    for k in range(n):
        d.qpos[:] = qpos[k]
        mj.mj_kinematics(m, d)
        xpos[k], xmat[k] = d.xpos, d.xmat.reshape(-1, 3, 3)
        axis[k], anchor[k] = d.xaxis, d.xanchor
    pelvis = body.pelvis
    R, p = xmat[:, pelvis], xpos[:, pelvis]
    w = np.einsum('nij,nj->ni', R, qvel[:, 3:6])
    al = np.einsum('nij,nj->ni', R, qacc[:, 3:6])
    vp = qvel[:, 0:3]
    # Plucker at the world's origin: (w, u) the body's point there, (al, a) its rate.
    V = {pelvis: (w, vp - np.cross(w, p))}
    A = {pelvis: (al, a0 - np.cross(w, vp) - np.cross(al, p))}
    S = {}
    for b in lay['bodies'][1:]:
        wv, uv = V[lay['parent'][b]]
        wa, ua = A[lay['parent'][b]]
        for dof in lay['dofs'][b]:
            j = int(m.dof_jntid[dof])
            sw, su = axis[:, j], np.cross(anchor[:, j], axis[:, j])
            S[dof] = (sw, su)
            qd, qdd = qvel[:, dof:dof + 1], qacc[:, dof:dof + 1]
            dsw, dsu = np.cross(wv, sw), np.cross(wv, su) + np.cross(uv, sw)
            wa, ua = wa + sw * qdd + dsw * qd, ua + su * qdd + dsu * qd
            wv, uv = wv + sw * qd, uv + su * qd
        V[b], A[b] = (wv, uv), (wa, ua)
    hinge = lay['hinge']
    at = {dof: k for k, dof in enumerate(hinge)}
    Y = np.zeros((n, len(hinge) + 6, len(lay['names'])))
    force, moment = slice(len(hinge), len(hinge) + 3), slice(len(hinge) + 3, len(hinge) + 6)
    for b in lay['bodies']:
        Rb, pb = xmat[:, b], xpos[:, b]
        wv, uv = V[b]
        wa, ua = A[b]
        # In the body's frame at its origin: its spin, its angular and its classical acceleration.
        om = np.einsum('nji,nj->ni', Rb, wv)
        alb = np.einsum('nji,nj->ni', Rb, wa)
        ac = np.einsum('nji,nj->ni', Rb, ua + np.cross(wa, pb) + np.cross(wv, uv + np.cross(wv, pb)))
        f, nn = np.zeros((n, 3, 10)), np.zeros((n, 3, 10))
        f[:, :, 0] = ac
        for k in range(3):
            e = np.zeros(3)
            e[k] = 1.0
            f[:, :, 1 + k] = np.cross(alb, e) + np.cross(om, np.cross(om, e))
            nn[:, :, 1 + k] = np.cross(e, ac)
        nn[:, :, 4:10] = _tensor(alb) + np.cross(om[:, :, None], _tensor(om), axis=1)
        # To the world, about its origin; up her tree, each joint passed takes its row.
        fw = np.einsum('nij,njc->nic', Rb, f)
        nw = np.einsum('nij,njc->nic', Rb, nn) + np.cross(pb[:, :, None], fw, axis=1)
        cols = slice(lay['col'][b], lay['col'][b] + 10)
        Y[:, force, cols] = fw
        Y[:, moment, cols] = np.einsum('nji,njc->nic', R, nw - np.cross(p[:, :, None], fw, axis=1))
        up = b
        while up != pelvis:
            for dof in lay['dofs'][up]:
                sw, su = S[dof]
                Y[:, at[dof], cols] = (np.einsum('ni,nic->nc', sw, nw)
                                       + np.einsum('ni,nic->nc', su, fw))
            up = lay['parent'][up]
    k = lay['drive']
    for i, j in enumerate(lay['driven']):
        dof = hinge[j]
        Y[:, j, k + 3 * i] = qacc[:, dof] if wacc is None else wacc[:, i]
        Y[:, j, k + 3 * i + 1] = qvel[:, dof]
        Y[:, j, k + 3 * i + 2] = np.sign(qvel[:, dof])
    return Y


def _tensor(w):
    """L(w) (N, 3, 6): L(w) [Ixx Ixy Ixz Iyy Iyz Izz] = I w."""
    import numpy as np
    L = np.zeros((len(w), 3, 6))
    L[:, 0, 0], L[:, 0, 1], L[:, 0, 2] = w[:, 0], w[:, 1], w[:, 2]
    L[:, 1, 1], L[:, 1, 3], L[:, 1, 4] = w[:, 0], w[:, 1], w[:, 2]
    L[:, 2, 2], L[:, 2, 4], L[:, 2, 5] = w[:, 0], w[:, 1], w[:, 2]
    return L


def _state(body, ys, dt, every):
    """(qpos, qvel, qacc, a0, wacc, at) of her edge arrays `ys` a step `dt` apart, every
    `every`th: her turn, spin and specific force her IMU's, her joints her encoders'; a rate's
    difference over the step its acceleration; what she does not measure - her hair, her hems
    - at rest; each motor's acceleration through its gearbox (QM) by differences over the steps
    about each sample, the armature's column - rigid, the joint's own."""
    import numpy as np
    m = body.m
    at = np.arange(0, len(ys) - 1, every)
    n = len(at)
    qpos, qvel, qacc = np.zeros((n, m.nq)), np.zeros((n, m.nv)), np.zeros((n, m.nv))
    qpos[:, 3] = 1.0
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
    vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
    ys = np.asarray(ys)
    now, then, before = ys[at], ys[at + 1], ys[np.maximum(at - 1, 0)]
    qpos[:, 3:7], qpos[:, qadr] = now[:, edge.QUAT], now[:, edge.Q]
    qvel[:, 3:6], qvel[:, vadr] = now[:, edge.GYRO], now[:, edge.QD]
    qacc[:, 3:6] = (then[:, edge.GYRO] - now[:, edge.GYRO]) / dt
    qacc[:, vadr] = (then[:, edge.QD] - now[:, edge.QD]) / dt
    a0 = np.stack([_rotation(q) @ a for q, a in zip(then[:, edge.QUAT], then[:, edge.ACCEL])])
    driven = np.flatnonzero(body.driven)
    wacc = (then[:, edge.QM] - 2.0 * now[:, edge.QM] + before[:, edge.QM])[:, driven] / dt ** 2
    wacc[at == 0] = qacc[at == 0][:, vadr[driven]]
    return qpos, qvel, qacc, a0, wacc, at


def rows(lay, body, ys, taus, dt, every=1):
    """(Y (N, driven, P), tau (N, driven)) of a campaign: `ys` (N + 1, SENSED) her edge arrays a
    step `dt` apart, `taus` (N, JOINTS) her drives' torques over each step, every `every`th
    step kept (`_state`); a drive's row zero where it does not slide (STICK)."""
    import numpy as np
    qpos, qvel, qacc, a0, wacc, at = _state(body, ys, dt, every)
    Y = regressor(lay, body, qpos, qvel, qacc, a0, wacc)[:, lay['driven']]
    tau = np.asarray(taus, float)[at][:, np.flatnonzero(body.driven)]
    slides = np.abs(qvel[:, [lay['hinge'][j] for j in lay['driven']]]) >= STICK
    return Y * slides[:, :, None], tau * slides


#: Her pelvis's force row is read in N m at this lever, m, beside the moment rows.
LEVER_M = 0.1


def borne(lay, body, ys, dt, every=1):
    """(Y (N, 3, P), f (N, 3)) her pelvis's rows her insoles measure, standing: the floor's push
    up on her, LEVER_M as a moment, and its moments about the point under her pelvis's origin
    at her insoles' height, along the world's x and z, where a push along the floor turns
    nothing - `f` what her insoles read, each one's pressure along its sole's normal at its
    centre of pressure. Her weight and its moments: her pelvis's own mass and first moments,
    which her drives' rows never show."""
    import numpy as np
    m, d, mj = body.m, body.d, body.mj
    qpos, qvel, qacc, a0, wacc, at = _state(body, ys, dt, every)
    now = np.asarray(ys)[at]
    Y = regressor(lay, body, qpos, qvel, qacc, a0, wacc)
    n_h, pelvis = len(lay['hinge']), body.pelvis
    insoles = [m.body(name).id for name in edge.INSOLES]
    out, f = np.zeros((len(at), 3, Y.shape[2])), np.zeros((len(at), 3))
    for k in range(len(at)):
        d.qpos[:] = qpos[k]
        mj.mj_kinematics(m, d)
        R, p = d.xmat[pelvis].reshape(3, 3), d.xpos[pelvis]
        push, turn, low, weight = np.zeros(3), np.zeros(3), 0.0, 0.0
        for i, b in enumerate(insoles):
            Ri, pi = d.xmat[b].reshape(3, 3), d.xpos[b]
            pressed = float(now[k, edge.PRESS][i])
            where = pi + Ri @ now[k, edge.COP][3 * i:3 * i + 3]
            push, turn = push + Ri[:, 1] * pressed, turn + np.cross(where - p, Ri[:, 1] * pressed)
            low, weight = low + pressed * where[1], weight + pressed
        h = p[1] - low / max(weight, 1e-9)
        F, M = Y[k, n_h:n_h + 3], R @ Y[k, n_h + 3:n_h + 6]
        out[k] = (LEVER_M * F[1], M[0] + h * F[2], M[2] - h * F[0])
        f[k] = (LEVER_M * push[1], turn[0] + h * push[2], turn[2] - h * push[0])
    return out, f


def hung(lay, body, ys, dt, pulls, every=1):
    """(Y (N, 6, P), f (N, 6)) her pelvis's rows a load cell in her harness measures, hung: the
    harness's force on her, world, LEVER_M as a moment, and its moment about her pelvis's
    origin, world - `pulls` (N + 1, 6) what the cell reads a step (`World.pull`). Her pelvis's
    own ten join the base parameters, her mass among them."""
    import numpy as np
    m, d, mj = body.m, body.d, body.mj
    qpos, qvel, qacc, a0, wacc, at = _state(body, ys, dt, every)
    Y = regressor(lay, body, qpos, qvel, qacc, a0, wacc)
    n_h, pelvis = len(lay['hinge']), body.pelvis
    out = np.zeros((len(at), 6, Y.shape[2]))
    f = np.asarray(pulls, float)[at] * np.r_[[LEVER_M] * 3, [1.0] * 3]
    for k in range(len(at)):
        d.qpos[:] = qpos[k]
        mj.mj_kinematics(m, d)
        out[k, :3] = LEVER_M * Y[k, n_h:n_h + 3]
        out[k, 3:] = d.xmat[pelvis].reshape(3, 3) @ Y[k, n_h + 3:n_h + 6]
    return out, f


#: A drive's row off the stand is read where the model holds: no joint of its limb within
#: STOP_DEG of a stop, nothing touching the limb's skin from the sample before to the one after.
#: A limb's joints and its skin's segments, a leg's and an arm's.
STOP_DEG = 2.0
LIMBS = ((('hip', 'knee', 'ankle', 'foot'), ('thigh', 'shank', 'foot', 'toes')),
         (('shoulder', 'elbow', 'wrist', 'gripper'), ('upper_arm', 'forearm', 'hand', 'fingers')))


def free(body, ys, every=1, stop_deg=STOP_DEG):
    """(N, driven) which drives' rows of `ys` (`rows`' arrays, every `every`th) the model holds
    on, walking: none of the drive's limb's joints within `stop_deg` of a stop, nothing touching
    the limb's skin (LIMBS) from the sample before to the one after - against the world's own
    model a swinging hip's row read 0 within its swing and 2-6 N m the sample its sole landed."""
    import numpy as np
    m = body.m
    seen = np.asarray(ys)[np.arange(0, len(ys) - 1, every)]
    touch = np.linalg.norm(seen[:, edge.TOUCH].reshape(len(seen), -1, 3), axis=2) > 0.0
    near = np.zeros((len(seen), len(JOINTS)), bool)
    for i, j in enumerate(JOINTS):
        jid = m.joint(j).id
        if m.jnt_limited[jid]:
            lo, hi = m.jnt_range[jid]
            near[:, i] = (seen[:, edge.Q][:, i] < lo + np.radians(stop_deg)) | (seen[:, edge.Q][:, i] > hi - np.radians(stop_deg))
    out = np.ones((len(seen), int(np.sum(body.driven))), bool)
    for i, j in enumerate(j for j, d in zip(JOINTS, body.driven) if d):
        side = j.split('_')[0] + '_' if j.startswith(('left_', 'right_')) else None
        for kinds, skins in LIMBS:
            if side and any(w in j for w in kinds):
                limb = [k for k, n in enumerate(JOINTS) if n.startswith(side) and any(w in n for w in kinds)]
                skin = [k for k, n in enumerate(edge.SKIN) if n.startswith(side) and any(w in n for w in skins)]
                clear = ~touch[:, skin].any(axis=1)
                out[:, i] = ~near[:, limb].any(axis=1) & clear & np.roll(clear, 1) & np.roll(clear, -1)
    return out


def base(lay, Y):
    """The base parameters of a campaign: {ind, dep, K, names}: theta_b = theta[ind] +
    K theta[dep], the columns of `ind` what its torques tell apart (Gautier, numerically),
    each of `dep` a combination of them - `Y` (N, driven, P) its regressor (`rows`). Pivoted
    Cholesky on the scaled Gram matrix: the column standing clearest of the base so far taken
    next, down to INDEPENDENT of itself."""
    import numpy as np
    Y = Y.reshape(-1, Y.shape[-1])
    scale = np.linalg.norm(Y, axis=0)
    none = scale <= NONE * scale.max()
    scale[none] = 1.0
    G = (Y / scale).T @ (Y / scale)
    G[none], G[:, none] = 0.0, 0.0
    left, ind, P = G.diagonal().copy(), [], len(scale)
    L = np.zeros((P, 0))
    while True:
        k = int(np.argmax(left))
        if left[k] <= INDEPENDENT ** 2:
            break
        col = (G[:, k] - L @ L[k]) / math.sqrt(left[k])
        L = np.column_stack([L, col])
        left = np.maximum(left - col * col, 0.0)
        left[k] = 0.0
        ind.append(k)
    ind.sort()
    dep = [k for k in range(P) if k not in ind]
    K = np.linalg.solve(G[np.ix_(ind, ind)], G[np.ix_(ind, dep)])
    K = K * scale[dep][None, :] / scale[ind][:, None]
    K[np.abs(K) < 1e-9] = 0.0
    return {'ind': np.array(ind, int), 'dep': np.array(dep, int), 'K': K,
            'names': [lay['names'][k] for k in ind]}


def projected(b, theta):
    """A full theta as the base parameters have it."""
    return theta[b['ind']] + b['K'] @ theta[b['dep']]


def identify(b, Y, tau, min_condition=1e-9):
    """The base parameters fitted to `Y` (N, driven, P) and `tau` (N, driven) by least
    squares: {theta, names, uncertainty (relative, each), trusted, condition, residual (N m
    rms a drive over its rows read), read (rows a drive), samples}. Refused where the campaign
    cannot tell them apart."""
    import numpy as np
    rows_ = Y[:, :, b['ind']].reshape(-1, len(b['ind']))
    target = tau.reshape(-1)
    scale = np.linalg.norm(rows_, axis=0)
    scale[scale == 0.0] = 1.0
    U, singular, Vt = np.linalg.svd(rows_ / scale, full_matrices=False)
    condition = float(singular[-1] / singular[0]) if singular[0] else 0.0
    if condition < min_condition:
        raise ValueError('this campaign cannot separate the base parameters - condition %.2e '
                         'against a floor of %.0e: every drive has to move, each at tones of '
                         'its own' % (condition, min_condition))
    theta = Vt.T @ ((U.T @ target) / singular) / scale
    left = (target - rows_ @ theta).reshape(tau.shape)
    freedom = max(1, rows_.shape[0] - rows_.shape[1])
    variance = float((left * left).sum()) / freedom
    err = np.sqrt(variance * ((Vt.T / singular) ** 2).sum(axis=1)) / scale
    unc = err / np.maximum(np.abs(theta), 1e-12)
    read = (np.abs(Y).sum(axis=2) > 0.0).sum(axis=0)
    return {'theta': theta, 'names': b['names'], 'uncertainty': unc, 'trusted': unc < TRUST,
            'condition': condition, 'residual': np.sqrt((left * left).sum(axis=0) / np.maximum(read, 1)),
            'read': read, 'samples': int(len(tau))}


def motor(t, q, tau, gear=1.0, hz=None):
    """{J, b, f, gravity, offset, residual, rms} of one drive off its own rows: `t` s, `q` its
    shaft, rad, `tau` its torque, N m - J qdd + b qd + f sgn qd + gravity sin(q / gear) + offset
    = tau by least squares, q and tau low-passed at `hz` (`campaign.SMOOTH_HZ`) both ways, the
    first and last second left out, and the rows either side of a gap in `t` (a record lost on
    the wire: its differences are no derivative); residual N m rms of what is left, rms the
    torque's, gaps how many rows a gap cost."""
    import numpy as np
    from machine import campaign
    dt = float(np.median(np.diff(t)))
    qs, taus = campaign.lowpass(q, dt, hz or campaign.SMOOTH_HZ), campaign.lowpass(tau, dt, hz or campaign.SMOOTH_HZ)
    w = np.gradient(qs, t)
    A = np.column_stack([np.gradient(w, t), w, np.sign(w), np.sin(qs / gear), np.ones(len(t))])
    keep = (t > t[0] + 1.0) & (t < t[-1] - 1.0)
    gap = np.flatnonzero(np.diff(t) > 1.5 * dt)
    near = np.unique(np.clip(gap[:, None] + np.arange(-2, 4), 0, len(t) - 1))
    gaps = int(keep[near].sum())
    keep[near] = False
    x, *_ = np.linalg.lstsq(A[keep], taus[keep], rcond=None)
    left = taus[keep] - A[keep] @ x
    return {'J': float(x[0]), 'b': float(x[1]), 'f': float(x[2]), 'gravity': float(x[3]), 'offset': float(x[4]),
            'residual': float(np.sqrt(np.mean(left ** 2))), 'rms': float(np.sqrt(np.mean(taus[keep] ** 2))), 'gaps': gaps}
