"""Her body identified of what she measures: each link's inertia and each drive's friction, fitted.

    lay = sysid.laid(body)                          # her links and the columns, on her law's model
    world.reset(sysid.pose(), where=(0.0, mjcf.STAND_M, 0.0))   # on the stand (mjcf.STAND)
    ref = sysid.setpoints(world, seconds, rng)      # a stage a limb, its drives at tones
    ys, taus, touched = sysid.excite(world, ref)    # her on the stand, played a step at a time
    Y, tau = sysid.rows(lay, body, ys, taus, dt)    # the regressor of a campaign's edge arrays
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

from machine import drives, edge, figure, terms
from machine.errors import MachineError
from machine.figure import JOINTS, LEG

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
#: The excitation's sines a drive, Hz; the share of a joint's range it swings through and the
#: share kept from either stop; an unlimited joint's swing, rad; the parts whose unlimited
#: joint swings abducted; a stage's ease in and out, s.
TONES, SWING, MARGIN, FREE_RAD = ((0.23, 0.61, 1.37), (0.31, 0.79, 1.13)), 0.35, 0.05, 0.5
ABDUCTED, RAMP_S = ('hip',), 0.5
#: Her pose on the stand: the stand's (`figure.stand`) with each (part, movement, deg) of
#: POSE laid on it (`machine.terms`) - her arms held out before her, clear of her thighs.
POSE = (('shoulder', 'flexion', 90.0),)
#: The campaign's stages, a limb each: its driven joints swung, the rest held where they stand.
STAGES = (('spine_roll', 'spine', 'waist', 'neck', 'head'),) + tuple(
    tuple(side + k for k in kinds) for kinds in (('shoulder', 'elbow'), tuple(k[1:] for k in LEG))
    for side in ('left_', 'right_'))


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


def regressor(lay, body, qpos, qvel, qacc, a0):
    """Y (N, hinges, P) at N states of her law's model: `qpos` (N, nq) and `qvel`, `qacc`
    (N, nv) in MuJoCo's convention, `a0` (N, 3) her pelvis's origin's acceleration less gravity,
    world; every hinge's row, as its torque would be in a world without gravity."""
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
    Y = np.zeros((n, len(hinge), len(lay['names'])))
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
        Y[:, j, k + 3 * i] = qacc[:, dof]
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


def rows(lay, body, ys, taus, dt, every=1):
    """(Y (N, driven, P), tau (N, driven)) of a campaign: `ys` (N + 1, SENSED) her edge arrays a
    step `dt` apart, `taus` (N, JOINTS) her drives' torques over each step, every `every`th
    step kept. Her turn, spin and specific force are her IMU's, her joints her encoders'; a
    rate's difference over the step its acceleration; what she does not measure - her hair,
    her hems - at rest; a drive's row zero where it does not slide (STICK)."""
    import numpy as np
    m = body.m
    at = np.arange(0, len(ys) - 1, every)
    n = len(at)
    qpos, qvel, qacc = np.zeros((n, m.nq)), np.zeros((n, m.nv)), np.zeros((n, m.nv))
    qpos[:, 3] = 1.0
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
    vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
    ys = np.asarray(ys)
    now, then, taus = ys[at], ys[at + 1], np.asarray(taus, float)[at]
    qpos[:, 3:7], qpos[:, qadr] = now[:, edge.QUAT], now[:, edge.Q]
    qvel[:, 3:6], qvel[:, vadr] = now[:, edge.GYRO], now[:, edge.QD]
    qacc[:, 3:6] = (then[:, edge.GYRO] - now[:, edge.GYRO]) / dt
    qacc[:, vadr] = (then[:, edge.QD] - now[:, edge.QD]) / dt
    a0 = np.stack([_rotation(q) @ a for q, a in zip(then[:, edge.QUAT], then[:, edge.ACCEL])])
    Y = regressor(lay, body, qpos, qvel, qacc, a0)[:, lay['driven']]
    tau = taus[:, np.flatnonzero(body.driven)]
    slides = np.abs(qvel[:, [lay['hinge'][j] for j in lay['driven']]]) >= STICK
    return Y * slides[:, :, None], tau * slides


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


def pose():
    """{joint: deg} she takes on the stand: the stand's, POSE's movements on it, every limb's."""
    degrees = figure.stand()
    for part, movement, deg in POSE:
        found = terms.find(part, movement)
        if found is None:
            raise MachineError('she has no %s of the %s to pose' % (movement, part))
        kind_, sign = found[0], found[1]
        for name in JOINTS:
            if drives.kind(name) == kind_:
                degrees[name] = degrees[name] + sign * deg
    return degrees


def swing(world, index):
    """(middle rad, half swing rad) of drive `index`'s excitation: SWING of its range, MARGIN
    from either stop, from where it stands as far as that leaves it; FREE_RAD with no stop,
    about where it stands - a joint of ABDUCTED swung abducted from there (`machine.terms`)."""
    m = world.model
    name = JOINTS[index]
    j, q0 = m.joint(name).id, float(world.target[index])
    if m.jnt_limited[j]:
        lo, hi = m.jnt_range[j]
        amp, keep = SWING * (hi - lo), MARGIN * (hi - lo)
        return min(max(q0, lo + keep + amp), hi - keep - amp), amp
    for part in ABDUCTED:
        found = terms.find(part, 'abduction')
        if found and drives.kind(name) == found[0]:
            return q0 + found[1] * FREE_RAD, FREE_RAD
    return q0, FREE_RAD


def setpoints(world, seconds, rng):
    """(N + 1, JOINTS) rad, the campaign's setpoints a step of `seconds`: a stage a limb
    (STAGES) in turn, its drives each at TONES about its swing's middle, the phases drawn, eased
    in and out over RAMP_S; the rest where they stand."""
    import numpy as np
    from machine import physics
    h = physics.STEP_S
    per = int(round(seconds / len(STAGES) / h))
    ref = np.tile(world.target, (per * len(STAGES) + 1, 1))
    t = np.arange(per) * h
    ease = np.minimum(1.0, np.minimum(t, t[-1] - t) / RAMP_S)
    for s, stage in enumerate(STAGES):
        for name in stage:
            i = JOINTS.index(name)
            mid, amp = swing(world, i)
            tone = np.array(TONES[i % len(TONES)])
            phase = rng.uniform(0.0, 2.0 * math.pi, size=len(tone))
            swung = mid + amp * np.sin(2.0 * math.pi * np.outer(t, tone) + phase).sum(axis=1) / len(tone)
            ref[s * per:(s + 1) * per, i] = world.target[i] + ease * (swung - world.target[i])
    return ref


def excite(world, ref):
    """(ys (N + 1, SENSED), taus (N, JOINTS), touched) of `ref` (N + 1, JOINTS) played on the
    stand a step each: her edge read every step and her drives' torques over it; `touched` the
    steps something touched her."""
    import numpy as np
    from machine import physics
    y = edge.laid(world)
    driven = np.flatnonzero(world.driven)
    now = [world.at]
    world.clock = lambda: now[0]
    ys, taus, touched = [], [], 0
    for k in range(len(ref)):
        ys.append(edge.felt(world, y).copy())
        touched += int(world.hers[world.model.geom_bodyid[world.data.contact.geom[:world.data.ncon]]].any())
        if k == len(ref) - 1:
            break
        world.target[driven] = ref[k + 1, driven]
        now[0] = world.at + physics.STEP_S
        world.advance()
        taus.append(world.motor.copy())
    world.clock = lambda: world.data.time
    return np.array(ys), np.array(taus), touched
