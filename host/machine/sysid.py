"""Her body identified of what she measures: each link's inertia and each drive's friction, fitted.

    lay = sysid.laid(body)                          # her links and the columns, on her law's model
    ys, taus, touched = campaign.excite(world, ref) # her edge arrays and her drives' torques a step
    Y, tau = sysid.rows(lay, body, ys, taus, dt)    # the regressor of a campaign's edge arrays
    free = sysid.free(lay, ys, every)               # off the stand, the rows the model holds on
    Yb, fb = sysid.borne(lay, body, ys, dt, every)  # her pelvis's rows her insoles measure, standing
    base = sysid.base(lay, Y)                       # the combinations its torques tell apart
    got = sysid.identify(base, Y, tau)              # each one fitted, with what it is worth
    identified.correction(lay, body, Y, tau)        # the least move of her build that fits it

Her drives' torques are linear in what moves her: each link's ten inertial parameters - mass,
first moment and inertia about its frame's origin - and each drive's armature, viscous and
Coulomb friction: tau = Y(q, v, a) theta. Y is Newton-Euler down her tree from her pelvis, whose
IMU gives turn, spin and specific force (gravity rides in it), her encoders each joint's angle
and rate, their differences the accelerations (`machine.edge`); only her drives' rows are
measured, so her pelvis's own parameters never show and what a link shares with its neighbours,
or what a campaign never moves apart, comes out as a sum: `base` finds those sums numerically
(Gautier) in the campaign's regressor, and `identify` fits them by least squares;
`machine.identified` moves her build the least that fits the campaign and lays it over her law's
model. On the stand (`mjcf.STAND`) her pelvis is welded to the world and nothing touches her soles.
"""
import math

from machine import edge, regressor
from machine.figure import JOINTS

#: The states a part of `rowing` reads a regressor of: a part of her wear's window a pass.
ROWS_AT = 100
#: A link's inertial parameters, in its frame about its origin.
INERTIAL = ('m', 'hx', 'hy', 'hz', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz')
#: A drive's: the rotor's inertia at the joint, kg m^2; viscous, N m s/rad; Coulomb, N m.
FRICTION = ('J', 'b', 'f')
#: A column is one of the base when, scaled, this much of it stands clear of the base before it;
#: one under NONE of the largest is nothing but rounding; two within TIE of each other stand
#: alike.
INDEPENDENT, NONE, TIE = 1e-3, 1e-9, 1e-9
#: A base parameter is trusted within this of itself.
TRUST = 0.10
#: A drive's row is read only where it slides faster than this, rad/s: at rest its Coulomb
#: friction is whatever holds it, anything within its limit.
STICK = 0.25
#: Her pelvis's force rows are read in N m at this lever, m, beside the moment rows.
LEVER_M = 0.1
#: A drive's row off the stand is read where the model holds: no joint of its limb within
#: STOP_DEG of a stop, nothing touching the limb's skin from the sample before to the one after.
#: A limb's joints and its skin's segments, a leg's and an arm's.
STOP_DEG = 2.0
LIMBS = ((('hip', 'knee', 'ankle', 'foot'), ('thigh', 'shank', 'foot', 'toes')),
         (('shoulder', 'elbow', 'wrist', 'gripper'), ('upper_arm', 'forearm', 'hand', 'fingers')))


def laid(body):
    """Her links and the columns, on her law's model, laid once: {bodies (hers, parents first),
    parent (each one's place among them, the pelvis -1), span (each one's hinges' range in
    hinge), hinge (every hinge dof, MuJoCo's order), jnt (each one's joint), above (body, hinge:
    the hinge lies between the body and her pelvis), col (a MuJoCo body's ten columns' start, -1
    not hers), drive (the drives' columns' start), driven (the drives' places among the hinges,
    JOINTS' order), dof, joint (their dofs, their places in JOINTS), drives (their names), names
    (every column's), qadr, vadr (JOINTS' in qpos and qvel), lo, hi (JOINTS' stops, rad), limb
    (drive, JOINTS) and skin (drive, SKIN): its limb's}."""
    import numpy as np
    m = body.m
    bodies = np.array([b for b in range(m.nbody) if edge._rooted(m, b, body.pelvis)])
    place = np.full(m.nbody, -1)
    place[bodies] = np.arange(len(bodies))
    parent = place[m.body_parentid[bodies]]
    count = np.where(bodies == body.pelvis, 0, m.body_jntnum[bodies])
    span = np.column_stack([np.cumsum(count) - count, np.cumsum(count)])
    jnt = np.concatenate([np.arange(f, f + c) for f, c in zip(m.body_jntadr[bodies], count)]).astype(int)
    hinge = np.array(m.jnt_dofadr[jnt], int)
    above = np.zeros((len(bodies), len(hinge)), bool)
    for k in range(len(bodies)):
        up = k
        while up > 0:
            above[k, span[up, 0]:span[up, 1]] = True
            up = parent[up]
    ids = [m.joint(j).id for j in JOINTS]
    qadr, vadr = np.array(m.jnt_qposadr[ids], int), np.array(m.jnt_dofadr[ids], int)
    joint = np.flatnonzero(body.driven)
    dof = vadr[joint]
    driven = np.array([int(np.flatnonzero(hinge == d)[0]) for d in dof])
    drives = tuple(JOINTS[i] for i in joint)
    col = np.full(m.nbody, -1)
    col[bodies] = 10 * np.arange(len(bodies))
    limited, rng = np.array(m.jnt_limited[ids], bool), np.array(m.jnt_range[ids], float)
    limb, skin = np.zeros((len(joint), len(JOINTS)), bool), np.zeros((len(joint), len(edge.SKIN)), bool)
    for i, name in enumerate(drives):
        side = name.split('_')[0] + '_' if name.startswith(('left_', 'right_')) else None
        for kinds, skins in LIMBS:
            if side and any(w in name for w in kinds):
                limb[i] = [n.startswith(side) and any(w in n for w in kinds) for n in JOINTS]
                skin[i] = [n.startswith(side) and any(w in n for w in skins) for n in edge.SKIN]
        if not side:
            kinds = ('spine', 'waist') if any(w in name for w in ('spine', 'waist')) else ('neck', 'head')
            limb[i] = [any(w in n for w in kinds) for n in JOINTS]
            skin[i] = [not s.endswith(('_foot', '_toes')) for s in edge.SKIN]
    return {'bodies': bodies, 'parent': parent, 'span': span, 'hinge': hinge, 'jnt': jnt, 'above': above,
            'col': col, 'drive': 10 * len(bodies), 'driven': driven, 'dof': dof, 'joint': joint,
            'drives': drives, 'names': ['%s.%s' % (m.body(b).name, p) for b in bodies for p in INERTIAL]
            + ['%s.%s' % (name, p) for name in drives for p in FRICTION], 'qadr': qadr, 'vadr': vadr,
            'lo': np.where(limited, rng[:, 0], -np.inf), 'hi': np.where(limited, rng[:, 1], np.inf),
            'limb': limb, 'skin': skin}


def shifted(mass, h):
    """The inertia a link's `mass` and first moment `h` add about its origin over its centre's,
    (h.h I - h h^T) / mass, the upper triangle (6,); none massless."""
    import numpy as np
    if mass <= 0.0:
        return np.zeros(6)
    return ((float(h @ h) * np.eye(3) - np.outer(h, h)) / mass)[np.triu_indices(3)]


def following(mass, h):
    """(4, 6) how `shifted` moves with (mass, hx, hy, hz): a link's inertia about its origin
    following its mass and first moment, its centre's own as built."""
    import numpy as np
    if mass <= 0.0:
        return np.zeros((4, 6))
    eye, up = np.eye(3), np.triu_indices(3)
    return np.array([-shifted(mass, h) / mass] + [
        ((2.0 * h[a] * eye - np.outer(eye[a], h) - np.outer(h, eye[a])) / mass)[up] for a in range(3)])


def truth(lay, m):
    """theta as a model `m` has her: each body's mass, first moment and inertia about its origin
    in its own frame; each drive's armature, damping and friction loss."""
    import numpy as np
    theta = np.zeros(len(lay['names']))
    links = theta[:lay['drive']].reshape(-1, 10)
    for k, b in enumerate(lay['bodies']):
        mass = float(m.body_mass[b])
        h = mass * np.array(m.body_ipos[b], float)
        r = _rotations(m.body_iquat[b])
        links[k, 0], links[k, 1:4] = mass, h
        links[k, 4:] = (r @ np.diag(np.array(m.body_inertia[b], float)) @ r.T)[np.triu_indices(3)] + shifted(mass, h)
    dof = lay['dof']
    theta[lay['drive']:] = np.column_stack([m.dof_armature[dof], m.dof_damping[dof], m.dof_frictionloss[dof]]).ravel()
    return theta


def _rotations(q):
    """(..., 3, 3) of quaternions `q` (..., 4), w first."""
    import numpy as np
    w, x, y, z = (np.asarray(q, float)[..., k] for k in range(4))
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def _state(lay, body, ys, dt, every, extra=None):
    """(qpos, qvel, qacc, a0, wacc, at) of her edge arrays `ys` a step `dt` apart, every
    `every`th: her turn, spin and specific force her IMU's, her joints her encoders'; a rate's
    difference over the step its acceleration; what she does not measure - her hair, her hems
    - at rest unless `extra` is given; each motor's acceleration through its gearbox (QM) by
    differences over the steps about each sample, the armature's column - rigid, the joint's own."""
    import numpy as np
    m, at = body.m, np.arange(0, len(ys) - 1, every)
    n, qadr, vadr = len(at), lay['qadr'], lay['vadr']
    qpos, qvel, qacc = np.zeros((n, m.nq)), np.zeros((n, m.nv)), np.zeros((n, m.nv))
    ys_arr = np.asarray(ys)
    now, then, before = (ys_arr[np.maximum(at + d, 0)] for d in (0, 1, -1))
    qpos[:, 3:7], qpos[:, qadr] = now[:, edge.QUAT], now[:, edge.Q]
    qvel[:, 3:6], qvel[:, vadr] = now[:, edge.GYRO], now[:, edge.QD]
    qacc[:, 3:6] = (then[:, edge.GYRO] - now[:, edge.GYRO]) / dt
    qacc[:, vadr] = (then[:, edge.QD] - now[:, edge.QD]) / dt
    if extra is not None:
        qpos[:, extra[0]], qvel[:, extra[1]], qacc[:, extra[1]] = extra[2:]
    a0 = np.einsum('nij,nj->ni', _rotations(then[:, edge.QUAT]), then[:, edge.ACCEL])
    wacc = (then[:, edge.QM] - 2.0 * now[:, edge.QM] + before[:, edge.QM])[:, lay['joint']] / dt ** 2
    wacc[at == 0] = qacc[at == 0][:, lay['dof']]
    return qpos, qvel, qacc, a0, wacc, at


def rows(lay, body, ys, taus, dt, every=1, extra=None):
    """(Y (N, driven, P), tau (N, driven)) of a campaign: `ys` (N + 1, SENSED) her edge arrays a
    step `dt` apart, `taus` (N, JOINTS) her drives' torques over each step, every `every`th
    step kept (`_state`); a drive's row zero where it does not slide (STICK)."""
    return done(rowing(lay, body, ys, taus, dt, every, extra))


def done(parts):
    """A generator run to its end (`rowing`, `wear.windowing`): its return."""
    while True:
        try:
            next(parts)
        except StopIteration as end:
            return end.value


def rowing(lay, body, ys, taus, dt, every=1, extra=None):
    """`rows` a part at a time: a generator yielding after each ROWS_AT states' regressor, `rows`'
    its return - one state's program the same over any number of them (`regressor.in_c`)."""
    import numpy as np
    qpos, qvel, qacc, a0, wacc, at = _state(lay, body, ys, dt, every, extra)
    yield
    Y = np.empty((len(at), len(lay['driven']), len(lay['names'])))
    for s in range(0, len(at), ROWS_AT):
        e = min(s + ROWS_AT, len(at))
        Y[s:e] = regressor.in_c(lay, body, qpos[s:e], qvel[s:e], qacc[s:e], a0[s:e], wacc[s:e])[:, lay['driven']]
        yield
    tau = np.asarray(taus, float)[at][:, lay['joint']]
    slides = np.abs(qvel[:, lay['dof']]) >= STICK
    Y *= slides[:, :, None]
    return Y, tau * slides


def read(Y):
    """(driven,) the rows of `Y` (N, driven, P) each drive is read on: those not zeroed."""
    import numpy as np
    return (np.abs(Y).sum(axis=2) > 0.0).sum(axis=0)


def misfit(Y, tau, theta):
    """(driven,) N m rms a drive over its rows read: `tau` less Y theta."""
    import numpy as np
    left = tau - Y @ theta
    return np.sqrt((left * left).sum(axis=0) / np.maximum(read(Y), 1))


def free(lay, ys, every=1, stop_deg=STOP_DEG):
    """(N, driven) which drives' rows of `ys` (`rows`' arrays, every `every`th) the model holds
    on, walking: none of the drive's limb's joints within `stop_deg` of a stop, nothing touching
    the limb's skin (LIMBS) from the sample before to the one after, a sample beyond the arrays
    unknown - against the world's own model a swinging hip's row read 0 within its swing and
    2-6 N m the sample its sole landed."""
    import numpy as np
    seen = np.asarray(ys)[np.arange(0, len(ys) - 1, every)]
    q, margin = seen[:, edge.Q], math.radians(stop_deg)
    near = (q < lay['lo'] + margin) | (q > lay['hi'] - margin)
    touch = np.linalg.norm(seen[:, edge.TOUCH].reshape(len(seen), -1, 3), axis=2) > 0.0
    stopped = (near.astype(int) @ lay['limb'].T) > 0
    clear = (touch.astype(int) @ lay['skin'].T) == 0
    edges = ~lay['skin'].any(axis=1)[None, :]
    return ~stopped & clear & np.r_[edges, clear[:-1]] & np.r_[clear[1:], edges]


def borne(lay, body, ys, dt, every=1):
    """(Y (N, 3, P), f (N, 3)) her pelvis's rows her insoles measure, standing: the floor's push
    up on her, LEVER_M as a moment, and its moments about the point under her pelvis's origin
    at her insoles' height, along the world's x and z, where a push along the floor turns
    nothing - `f` what her insoles read, each one's pressure along its sole's normal at its
    centre of pressure. Her weight and its moments: her pelvis's own mass and first moments,
    which her drives' rows never show."""
    return done(borning(lay, body, ys, dt, every))


def borning(lay, body, ys, dt, every=1):
    """`borne` a part at a time: its frames and its regressor ROWS_AT states a part, laid whole as
    one call lays them - numpy's own orders the same on them."""
    import numpy as np
    m = body.m
    qpos, qvel, qacc, a0, wacc, at = _state(lay, body, ys, dt, every)
    now = np.asarray([ys[i] for i in at])
    yield
    parts, Y = [], None
    for s in range(0, len(at), ROWS_AT):
        e = min(s + ROWS_AT, len(at))
        part = regressor.posed(lay, body, qpos[s:e])
        got = regressor.in_c(lay, body, qpos[s:e], qvel[s:e], qacc[s:e], a0[s:e], wacc[s:e], part)
        if Y is None:
            Y = np.empty((len(at),) + got.shape[1:])
        Y[s:e] = got
        parts.append(part)
        yield
    if Y is None:
        frames = regressor.posed(lay, body, qpos)
        Y = regressor.in_c(lay, body, qpos, qvel, qacc, a0, wacc, frames)
    else:
        frames = tuple(np.concatenate([p[k] for p in parts]) for k in range(4))
    xpos, xmat, _axis, _anchor = frames
    soles = lay['col'][[m.body(name).id for name in edge.INSOLES]] // 10
    Ri, pi = xmat[:, soles], xpos[:, soles]
    pressed, cop = now[:, edge.PRESS], now[:, edge.COP].reshape(len(at), -1, 3)
    normal = Ri[:, :, :, 1]
    where = pi + np.einsum('nsij,nsj->nsi', Ri, cop)
    R, p = xmat[:, 0], xpos[:, 0]
    push = np.einsum('nsi,ns->ni', normal, pressed)
    turn = np.einsum('nsi,ns->ni', np.cross(where - p[:, None], normal), pressed)
    h = p[:, 1] - (pressed * where[:, :, 1]).sum(axis=1) / np.maximum(pressed.sum(axis=1), 1e-9)
    n_h = len(lay['hinge'])
    F, M = Y[:, n_h:n_h + 3], np.einsum('nij,njc->nic', R, Y[:, n_h + 3:n_h + 6])
    out = np.stack([LEVER_M * F[:, 1], M[:, 0] + h[:, None] * F[:, 2], M[:, 2] - h[:, None] * F[:, 0]], axis=1)
    f = np.stack([LEVER_M * push[:, 1], turn[:, 0] + h * push[:, 2], turn[:, 2] - h * push[:, 0]], axis=1)
    return out, f


def hung(lay, body, ys, dt, pulls, every=1):
    """(Y (N, 6, P), f (N, 6)) her pelvis's rows a load cell in her harness measures, hung: the
    harness's force on her, world, LEVER_M as a moment, and its moment about her pelvis's
    origin, world - `pulls` (N + 1, 6) what the cell reads a step (`World.pull`). Her pelvis's
    own ten join the base parameters, her mass among them."""
    import numpy as np
    qpos, qvel, qacc, a0, wacc, at = _state(lay, body, ys, dt, every)
    Y = regressor.in_c(lay, body, qpos, qvel, qacc, a0, wacc)
    n_h = len(lay['hinge'])
    out = np.concatenate([LEVER_M * Y[:, n_h:n_h + 3],
                          np.einsum('nij,njc->nic', _rotations(qpos[:, 3:7]), Y[:, n_h + 3:n_h + 6])], axis=1)
    return out, np.asarray([pulls[i] for i in at], float) * np.r_[[LEVER_M] * 3, [1.0] * 3]


def base(lay, Y):
    """The base parameters of a campaign: {ind, dep, K, names}: theta_b = theta[ind] +
    K theta[dep], the columns of `ind` what its torques tell apart (Gautier, numerically),
    each of `dep` a combination of them - `Y` (N, driven, P) its regressor (`rows`). Pivoted
    Cholesky on the scaled Gram matrix: the column standing clearest of the base so far taken
    next, down to INDEPENDENT of itself; of columns standing alike (TIE), the first - a sum
    lands on the parent, her pelvis's own first."""
    import numpy as np
    Y = Y.reshape(-1, Y.shape[-1])
    G = Y.T @ Y
    scale = np.sqrt(np.maximum(G.diagonal(), 0.0))
    none = scale <= NONE * scale.max()
    scale[none] = 1.0
    G = G / np.outer(scale, scale)
    G[none], G[:, none] = 0.0, 0.0
    left, ind, P = G.diagonal().copy(), [], len(scale)
    L = np.zeros((P, 0))
    while True:
        k = int(np.argmax(left >= (1.0 - TIE) * left.max()))
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
    ind, rows_, target = b['ind'], Y.reshape(-1, Y.shape[-1]), tau.reshape(-1)
    G0 = (rows_.T @ rows_)[np.ix_(ind, ind)]
    scale = np.sqrt(np.maximum(G0.diagonal(), 0.0))
    scale[scale == 0.0] = 1.0
    eig, V = np.linalg.eigh(G0 / np.outer(scale, scale))
    singular, V = np.sqrt(np.maximum(eig[::-1], 0.0)), V[:, ::-1]
    condition = float(singular[-1] / singular[0]) if singular[0] else 0.0
    if condition < min_condition:
        raise ValueError('this campaign cannot separate the base parameters - condition %.2e '
                         'against a floor of %.0e: every drive has to move, each at tones of '
                         'its own' % (condition, min_condition))
    theta = (V @ ((V.T @ ((rows_.T @ target)[ind] / scale)) / singular ** 2)) / scale
    left, seen = tau - Y[:, :, ind] @ theta, read(Y)
    err = np.sqrt(float((left * left).sum()) / max(1, rows_.shape[0] - len(ind)) * ((V / singular) ** 2).sum(axis=1)) / scale
    unc = err / np.maximum(np.abs(theta), 1e-12)
    return {'theta': theta, 'names': b['names'], 'uncertainty': unc, 'trusted': unc < TRUST,
            'condition': condition, 'residual': np.sqrt((left * left).sum(axis=0) / np.maximum(seen, 1)),
            'read': seen, 'samples': int(len(tau))}


def motor(t, q, tau, gear=1.0, hz=None):
    """{J, b, f, gravity, offset, residual, rms} of one drive off its own rows: `t` s, `q` its
    shaft, rad, `tau` its torque, N m - J qdd + b qd + f sgn qd + gravity sin(q / gear) + offset
    = tau by least squares, q and tau low-passed at `hz` (`campaign.SMOOTH_HZ`) both ways, the
    first and last second left out, and the rows either side of a gap in `t` (a record lost on
    the wire: its differences are no derivative); residual N m rms of what is left, rms the
    torque's, gaps how many rows a gap cost."""
    import numpy as np
    from machine import campaign
    dt, hz = float(np.median(np.diff(t))), hz or campaign.SMOOTH_HZ
    qs, taus = campaign.lowpass(q, dt, hz), campaign.lowpass(tau, dt, hz)
    w = np.gradient(qs, t)
    A = np.column_stack([np.gradient(w, t), w, np.sign(w), np.sin(qs / gear), np.ones(len(t))])
    keep = (t > t[0] + 1.0) & (t < t[-1] - 1.0)
    near = np.unique(np.clip(np.flatnonzero(np.diff(t) > 1.5 * dt)[:, None] + np.arange(-2, 4), 0, len(t) - 1))
    gaps = int(keep[near].sum())
    keep[near] = False
    x, *_ = np.linalg.lstsq(A[keep], taus[keep], rcond=None)
    left = taus[keep] - A[keep] @ x
    return dict(zip(('J', 'b', 'f', 'gravity', 'offset'), map(float, x)),
                residual=float(np.sqrt(np.mean(left ** 2))), rms=float(np.sqrt(np.mean(taus[keep] ** 2))), gaps=gaps)
