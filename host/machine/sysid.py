"""Her body identified from what she measures alone: least squares on her motion, hung and standing.

    run = sysid.laid(body, passes)       # her chain from her model (`wbc.Body`), the figure's values as prior, a log
    sysid.record(run, y, tau, hung)      # a pass: her array (`machine.edge`), her motors' torques, a harness on her
    got = sysid.solve(run)               # each link's (m, m c, I), each drive's armature and friction, its box
    q = sysid.excite(run, t)             # setpoints that excite her hung: a multisine a joint within its stops

A joint's torque is linear in its subtree's links' ten inertial parameters and its own armature,
viscous and Coulomb friction (Newton-Euler in each link's frame: the pelvis's turn, spin and
specific force from her IMU, her encoders' angles and rates, accelerations by central
differences); her weight and its moments about her pelvis, against her soles' loads and centres
of pressure, are the pelvis's rows. A joint's row is dropped while something touches its
subtree, the pelvis's while a harness holds her. Solved toward the figure's values: where a run
says nothing the figure stays, and `said` tells how much of each number the run decided.
"""
import math

from machine import drives, edge
from machine.figure import JOINTS, SEGMENTS
from machine.flight import TOUCH_N

#: The multisine's lines, Hz; each joint's amplitude, this share of its room to its nearer stop,
#: DEG at most.
LINES, SHARE, DEG = (0.17, 0.29, 0.43, 0.71, 1.07), 0.6, 30.0
#: Parameters a link - m, m c (3), I about its frame (xx xy xz yy yz zz) - and a joint: a drive's
#: armature, viscous and Coulomb friction; a sprung joint's viscous friction, stiffness and preload.
LINK, DRIVE = 10, 3
#: The prior's doubt, a share of each value, over these floors: kg, kg m, kg m^2; kg m^2,
#: N m s/rad, N m, N m/rad, N m.
DOUBT, FLOORS = 1.0, {'m': 0.05, 'h': 0.01, 'I': 1e-3, 'J': 0.01, 'b': 0.1, 'f': 0.1, 'k': 1.0,
                      'k0': 0.1}
#: A row's weight: the pelvis's force row a N against a joint's N m.
FORCE_W = 0.1
#: A box's torque counts as past its play from this, N m; its deflection as moving from this, rad.
PLAY_NM, MOVED_RAD = 1.0, 1e-5
#: Where the pelvis's rows read: its moments about x and z, its force along y, of its wrench.
BASE = (0, 2, 4)
#: Sides' soles, by the joint kinds their subtrees hold a sole of.
SIDES = ('left', 'right')


def _rotation(quat):
    w, x, y, z = (float(c) for c in quat)
    import numpy as np
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _skew(v):
    import numpy as np
    s = np.zeros(v.shape[:-1] + (3, 3))
    s[..., 0, 1], s[..., 0, 2], s[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    s[..., 1, 0], s[..., 2, 0], s[..., 2, 1] = v[..., 2], -v[..., 1], v[..., 0]
    return s


def _about(axis, q):
    """Rodrigues: (N, 3, 3) turns of `q` (N,) about the unit `axis` (3,)."""
    import numpy as np
    c, s = np.cos(q), np.sin(q)
    k = _skew(axis)
    return c[:, None, None] * np.eye(3) + s[:, None, None] * k + (1.0 - c)[:, None, None] * np.outer(axis, axis)


def _chain(body):
    """Her links in her model's order, the pelvis first: a joint of `JOINTS` a link; a body with
    no joint, or a hinge she does not measure, rides its parent's link fixed at the model's rest."""
    import numpy as np
    m, d, mj = body.m, body.d, body.mj
    measured = {m.joint(j).id: k for k, j in enumerate(JOINTS)}
    d.qpos[:], d.qvel[:] = m.qpos0, 0.0
    mj.mj_kinematics(m, d)
    links, of = [], {}
    for b in range(m.nbody):
        up = b
        while up and up != body.pelvis:
            up = int(m.body_parentid[up])
        if up != body.pelvis:
            continue
        parent = int(m.body_parentid[b])
        first, count = int(m.body_jntadr[b]), int(m.body_jntnum[b])
        if b == body.pelvis:
            links.append({'name': 'pelvis', 'joint': -1, 'parent': -1, 'body': b, 'R0': np.eye(3),
                          'p0': np.zeros(3), 'axis': np.zeros(3), 'anchor': np.zeros(3),
                          'lumped': []})
        elif not count or first not in measured:
            at = links[of[parent]]['body']
            R = d.xmat[at].reshape(3, 3).T @ d.xmat[b].reshape(3, 3)
            p = d.xmat[at].reshape(3, 3).T @ (d.xpos[b] - d.xpos[at])
            links[of[parent]]['lumped'].append((b, R, p))
        else:
            for k in range(count):
                j = first + k
                links.append({'name': m.joint(j).name, 'joint': measured[j],
                              'parent': of[parent] if k == 0 else len(links) - 1,
                              'body': b if k == count - 1 else -1,
                              'R0': _rotation(m.body_quat[b]) if k == 0 else np.eye(3),
                              'p0': np.array(m.body_pos[b], float) if k == 0 else np.zeros(3),
                              'axis': np.array(m.jnt_axis[j], float),
                              'anchor': np.array(m.jnt_pos[j], float), 'lumped': []})
        of[b] = len(links) - 1
    for i, link in enumerate(links):
        link['subtree'] = [k for k in range(len(links)) if _under(links, k, i)]
    return links, of


def _under(links, k, i):
    while k >= 0:
        if k == i:
            return True
        k = links[k]['parent']
    return False


def inertial(run, m):
    """(L, LINK) each link's (m, m c, I about its frame) of model `m`: its body's and what rides it."""
    import numpy as np
    out = np.zeros((len(run['links']), LINK))
    for i, link in enumerate(run['links']):
        parts = ([(link['body'], np.eye(3), np.zeros(3))] if link['body'] >= 0 else []) + link['lumped']
        for b, R, p in parts:
            mass = float(m.body_mass[b])
            c = p + R @ np.array(m.body_ipos[b], float)
            Rq = R @ _rotation(m.body_iquat[b])
            I = Rq @ np.diag(np.array(m.body_inertia[b], float)) @ Rq.T + mass * (c @ c * np.eye(3) - np.outer(c, c))
            out[i, 0] += mass
            out[i, 1:4] += mass * c
            out[i, 4:] += I[np.triu_indices(3)]
    return out


def laid(body, passes, seed=0):
    """The run's memory: her chain, each joint's kind and stops, the prior (`body`'s model, the
    figure's), the multisine, and a log of `passes` passes."""
    import numpy as np
    m = body.m
    links, of = _chain(body)
    jadr = [m.joint(j).id for j in JOINTS]
    vadr = np.array([m.jnt_dofadr[j] for j in jadr])
    spring = [drives.passive(j) for j in JOINTS]
    kind = np.array([0 if s is None else 2 if s[0] is None else 1 for s in spring])
    stiff = np.array([float(m.jnt_stiffness[j]) for j in jadr])
    rest = np.array([float(m.qpos_spring[m.jnt_qposadr[j]]) for j in jadr])
    prior_j = np.where(kind[:, None] == 0,
                       np.c_[m.dof_armature[vadr], m.dof_damping[vadr], m.dof_frictionloss[vadr]],
                       np.c_[m.dof_damping[vadr], stiff, -stiff * rest])
    scale_j = np.where(kind[:, None] == 0, (FLOORS['J'], FLOORS['b'], FLOORS['f']),
                       (FLOORS['b'], FLOORS['k'], FLOORS['k0']))
    low = np.array([m.jnt_range[j][0] if m.jnt_limited[j] else -math.pi for j in jadr])
    high = np.array([m.jnt_range[j][1] if m.jnt_limited[j] else math.pi for j in jadr])
    at = np.where(np.array([m.jnt_limited[j] for j in jadr]), 0.5 * (low + high), 0.0)
    room = np.minimum(np.minimum(at - low, high - at) * SHARE, math.radians(DEG)) * (kind == 0)
    rng = np.random.default_rng(seed)
    link_of_joint = {link['joint']: i for i, link in enumerate(links)}
    soles = np.zeros((len(JOINTS), 2), bool)
    for k, j in enumerate(JOINTS):
        for s, side in enumerate(SIDES):
            toes = link_of_joint[JOINTS.index(side + '_foot')]
            soles[k, s] = toes in links[link_of_joint[k]]['subtree']
    skin = np.array([of[m.body(seg[0]).id] for seg in SEGMENTS])
    return {'links': links, 'kind': kind, 'low': low, 'high': high, 'at': at, 'room': room,
            'phase': rng.uniform(0.0, 2.0 * math.pi, (len(JOINTS), len(LINES))),
            'prior': inertial({'links': links}, m), 'prior_j': prior_j, 'scale_j': scale_j,
            'soles': soles, 'skin': skin, 'insole': [of[m.body(n).id] for n in edge.INSOLES],
            'y': np.zeros((passes, edge.SENSED)), 'tau': np.zeros((passes, len(JOINTS))),
            'hung': np.zeros(passes, bool), 't': np.zeros(passes), 'n': 0}


def excite(run, t):
    """Every joint's setpoint at `t` s, rad: its rest and its multisine; none on what no drive turns."""
    import numpy as np
    w = 2.0 * math.pi * np.array(LINES) * t
    return run['at'] + run['room'] * np.sin(w + run['phase']).sum(axis=1) / len(LINES)


def record(run, y, tau, hung, t):
    """A pass into the log: her array, her motors' torques (N m at the joints), whether a harness
    holds her, when."""
    n = run['n']
    run['y'][n], run['tau'][n], run['hung'][n], run['t'][n] = y, tau, hung, t
    run['n'] = n + 1


def kinematics(run, q, quat):
    """Every link's frame for joints `q` (N, J) and the pelvis turned `quat` (N, 4) at the
    origin: R (L, N, 3, 3), p (L, N, 3), each joint's axis and anchor there, and the turn and
    place of each link in its parent's frame (`_motion` reads them)."""
    import numpy as np
    links, N = run['links'], len(q)
    L = len(links)
    R, p = np.zeros((L, N, 3, 3)), np.zeros((L, N, 3))
    Rrel, prel = np.zeros((L, N, 3, 3)), np.zeros((L, N, 3))
    R[0] = np.array([_rotation(c) for c in quat])
    for i, link in enumerate(links[1:], 1):
        Rq = _about(link['axis'], q[:, link['joint']])
        Rrel[i] = link['R0'] @ Rq
        prel[i] = link['p0'] + (link['R0'] @ (link['anchor'] - Rq @ link['anchor']).T).T
        par = link['parent']
        R[i] = R[par] @ Rrel[i]
        p[i] = p[par] + np.einsum('nij,nj->ni', R[par], prel[i])
    return R, p, Rrel, prel


def _wrenches(run, R, p, Rrel, prel, q, qd, qdd, gyro, alpha, accel):
    """Each link's wrench about the origin, world, as a linear map of its parameters:
    (L, N, 6, LINK); its spatial velocity and acceleration carried down the chain."""
    import numpy as np
    links, N = run['links'], len(q)
    L = len(links)
    V, A = np.zeros((L, N, 6)), np.zeros((L, N, 6))
    V[0, :, :3], A[0, :, :3], A[0, :, 3:] = gyro, alpha, accel
    Y = np.zeros((L, N, 6, LINK))
    for i, link in enumerate(links):
        if i:
            par, j = link['parent'], link['joint']
            Rt = Rrel[i].transpose(0, 2, 1)
            S = np.r_[link['axis'], np.cross(link['anchor'], link['axis'])]
            for src, dst in ((V, V), (A, A)):
                dst[i, :, :3] = np.einsum('nij,nj->ni', Rt, src[par, :, :3])
                dst[i, :, 3:] = np.einsum('nij,nj->ni', Rt, src[par, :, 3:] + np.cross(src[par, :, :3], prel[i]))
            V[i] += S * qd[:, j, None]
            w, v = V[i, :, :3], V[i, :, 3:]
            A[i] += S * qdd[:, j, None]
            A[i, :, :3] += np.cross(w, S[:3]) * qd[:, j, None]
            A[i, :, 3:] += (np.cross(w, S[3:]) + np.cross(v, S[:3])) * qd[:, j, None]
        w, v, al, a = V[i, :, :3], V[i, :, 3:], A[i, :, :3], A[i, :, 3:]
        sw, sv, sal, sa = _skew(w), _skew(v), _skew(al), _skew(a)
        phi = np.zeros((N, 6, LINK))
        phi[:, 3:, 0] = a + np.cross(w, v)
        phi[:, :3, 1:4] = -sa - sw @ sv + sv @ sw
        phi[:, 3:, 1:4] = sal + sw @ sw
        phi[:, :3, 4:] = _lin(al) + sw @ _lin(w)
        T = np.zeros((N, 6, 6))
        T[:, :3, :3] = T[:, 3:, 3:] = R[i]
        T[:, :3, 3:] = _skew(p[i]) @ R[i]
        Y[i] = T @ phi
    return Y


def _lin(x):
    """L (N, 3, 6) with L iota = I x for iota = (xx, xy, xz, yy, yz, zz)."""
    import numpy as np
    L = np.zeros((len(x), 3, 6))
    L[:, 0, 0], L[:, 0, 1], L[:, 0, 2] = x[:, 0], x[:, 1], x[:, 2]
    L[:, 1, 1], L[:, 1, 3], L[:, 1, 4] = x[:, 0], x[:, 1], x[:, 2]
    L[:, 2, 2], L[:, 2, 4], L[:, 2, 5] = x[:, 0], x[:, 1], x[:, 2]
    return L


def regressor(run, q, qd, qdd, qm_dd, quat, gyro, alpha, accel):
    """Y (N, J + 3, columns) and its frames: each joint's torque, then the pelvis's moments
    about x and z and its force along y, linear in the links' parameters (LINK a link) and the
    joints' (DRIVE a joint)."""
    import numpy as np
    links, J, N = run['links'], len(JOINTS), len(q)
    R, p, Rrel, prel = kinematics(run, q, quat)
    W = _wrenches(run, R, p, Rrel, prel, q, qd, qdd, gyro, alpha, accel)
    Y = np.zeros((N, J + 3, LINK * len(links) + DRIVE * J))
    for i, link in enumerate(links[1:], 1):
        j = link['joint']
        a = np.einsum('nij,j->ni', R[i], link['axis'])
        o = p[i] + np.einsum('nij,j->ni', R[i], link['anchor'])
        S = np.c_[a, np.cross(o, a)]
        for k in link['subtree']:
            Y[:, j, LINK * k:LINK * (k + 1)] = np.einsum('nr,nrc->nc', S, W[k])
    for k in range(len(links)):
        Y[:, J:, LINK * k:LINK * (k + 1)] = W[k][:, BASE]
    at = LINK * len(links)
    for j in range(J):
        c = at + DRIVE * j
        if run['kind'][j] == 0:
            Y[:, j, c], Y[:, j, c + 1], Y[:, j, c + 2] = qm_dd[:, j], qd[:, j], np.sign(qd[:, j])
        elif run['kind'][j] == 1:
            Y[:, j, c], Y[:, j, c + 1], Y[:, j, c + 2] = qd[:, j], q[:, j], 1.0
    return Y, R, p


def _back(x, t):
    """dx/dt over the pass before each sample: the step's own difference, as the IMU's
    acceleration is the pass's (`edge.felt`)."""
    import numpy as np
    return np.diff(x, axis=0, prepend=x[:1]) / np.maximum(np.diff(t, prepend=t[0] - 1.0), 1e-9)[:, None]


def series(run, every):
    """The log every `every`th pass, its rates differenced on the whole: q, qd, qdd, qm_dd,
    quat, gyro, alpha, accel, tau, press, cop, touch, hung."""
    import numpy as np
    n = run['n']
    y, t = run['y'][:n], run['t'][:n]
    qdd = _back(y[:, edge.QD], t)
    qm_dd = _back(_back(y[:, edge.QM], t), t)
    alpha = _back(y[:, edge.GYRO], t)
    s = slice(2 * every, n - every, every)
    return {'q': y[s, edge.Q], 'qd': y[s, edge.QD], 'qdd': qdd[s], 'qm_dd': qm_dd[s],
            'qm': y[s, edge.QM], 'quat': y[s, edge.QUAT], 'gyro': y[s, edge.GYRO],
            'alpha': alpha[s], 'accel': y[s, edge.ACCEL], 'tau': run['tau'][:n][s],
            'press': y[s, edge.PRESS], 'cop': y[s, edge.COP].reshape(-1, len(edge.INSOLES), 3),
            'touch': np.linalg.norm(y[s, edge.TOUCH].reshape(len(y[s]), -1, 3), axis=2),
            'hung': run['hung'][:n][s], 't': t[s]}


def rows(run, Y, R, p, d):
    """(targets (N, J + 3), kept (N, J + 3) bool) of the series `d`: each driven joint's
    motor torque, a sprung joint's none, the pelvis's rows from her soles; a joint's row kept
    while nothing touches its subtree and it is not held, the pelvis's while no harness holds
    her and nothing but her soles touches."""
    import numpy as np
    J, N = len(JOINTS), len(d['q'])
    links = run['links']
    target = np.zeros((N, J + 3))
    target[:, :J] = np.where(run['kind'] == 0, d['tau'], 0.0)
    bears = d['press'] >= TOUCH_N / 2.0
    for k, i in enumerate(run['insole']):
        at = p[i] + np.einsum('nij,nj->ni', R[i], d['cop'][:, k])
        f = d['press'][:, k] * bears[:, k]
        target[:, J] += -f * at[:, 2]
        target[:, J + 1] += f * at[:, 0]
        target[:, J + 2] += f
    touched = d['touch'] > edge.TOUCHED_N
    touched_link = np.zeros((N, len(links)), bool)
    for s, i in enumerate(run['skin']):
        touched_link[:, i] |= touched[:, s]
    kept = np.zeros((N, J + 3), bool)
    for link in links[1:]:
        j = link['joint']
        kept[:, j] = (run['kind'][j] != 2) & ~touched_link[:, link['subtree']].any(axis=1)
    insoles = set(run['insole'])
    others = [i for i in range(len(links)) if i not in insoles]
    kept[:, J:] = (~d['hung'] & ~touched_link[:, others].any(axis=1))[:, None]
    return target, kept


def solve(run, every=5, chunk=400):
    """Her parameters of the log: {'links' (L, LINK), 'joints' (J, DRIVE), 'boxes' (J, 3: wind-up
    N m/rad, play rad, damping), 'said' for each - how much of it the run decided, 0 the prior -,
    'rms' (J,) each joint's torque misfit, N m, the prior's beside it in 'rms0', 'samples'}."""
    import numpy as np
    d = series(run, every)
    J, L = len(JOINTS), len(run['links'])
    prior = np.r_[run['prior'].ravel(), run['prior_j'].ravel()]
    floors = np.r_[np.tile([FLOORS['m']] + [FLOORS['h']] * 3 + [FLOORS['I']] * 6, L), run['scale_j'].ravel()]
    scale = DOUBT * np.maximum(np.abs(prior), floors)
    weight = np.r_[np.ones(J), 1.0, 1.0, FORCE_W]
    C = len(prior)
    H, g = np.eye(C), np.zeros(C)
    miss0, miss, count = np.zeros(J + 3), np.zeros(J + 3), np.zeros(J + 3)
    chunks = []
    for start in range(0, len(d['q']), chunk):
        s = slice(start, start + chunk)
        part = {k: v[s] for k, v in d.items()}
        Y, R, p = regressor(run, part['q'], part['qd'], part['qdd'], part['qm_dd'], part['quat'],
                            part['gyro'], part['alpha'], part['accel'])
        target, kept = rows(run, Y, R, p, part)
        Ys = Y * scale * (weight * kept)[..., None]
        r = (target - Y @ prior) * weight * kept
        Yf, rf = Ys.reshape(-1, C), r.ravel()
        H += Yf.T @ Yf
        g += Yf.T @ rf
        miss0 += (r * r).sum(axis=0)
        count += kept.sum(axis=0)
        chunks.append((Ys, r, kept))
    cov = np.linalg.inv(H)
    z = cov @ g
    for Ys, r, kept in chunks:
        e = r - Ys @ z
        miss += (e * e).sum(axis=0)
    pi = prior + scale * z
    said = 1.0 - np.diag(cov)
    got = {'links': pi[:LINK * L].reshape(L, LINK), 'joints': pi[LINK * L:].reshape(J, DRIVE),
           'said': {'links': said[:LINK * L].reshape(L, LINK), 'joints': said[LINK * L:].reshape(J, DRIVE)},
           'rms0': np.sqrt(miss0 / np.maximum(count, 1)), 'rms': np.sqrt(miss / np.maximum(count, 1)),
           'rows': count, 'samples': len(d['q'])}
    got['boxes'], got['said']['boxes'] = boxes(run, d, got['joints'][:, 0])
    return got


def boxes(run, d, armature):
    """Each drive's gearbox of the series: (J, 3) wind-up N m/rad, play rad and damping N m
    s/rad by least squares on what its rotor passes - its motor's torque less its armature's -
    against its deflection past the link, where that moves; the figure's where it does not."""
    import numpy as np
    J = len(JOINTS)
    out = np.array([[drives.wind(j), drives.play(j), 0.0] if run['kind'][k] == 0 else [0.0] * 3
                    for k, j in enumerate(JOINTS)])
    out[:, 2] = np.where(run['kind'] == 0, 2.0 * drives.BOX_ZETA * np.sqrt(out[:, 0] * armature), 0.0)
    said = np.zeros((J, 3))
    delta = d['qm'] - d['q']
    ddelta = _back(delta, d['t'])
    passed = d['tau'] - armature * d['qm_dd']
    for j in range(J):
        on = (np.abs(passed[:, j]) > PLAY_NM) & (np.abs(delta[:, j]) > MOVED_RAD)
        if run['kind'][j] or on.sum() < 50 or delta[on, j].std() < MOVED_RAD:
            continue
        X = np.c_[delta[on, j], -np.sign(delta[on, j]), ddelta[on, j]]
        fit, _res, rank, _sv = np.linalg.lstsq(X, passed[on, j], rcond=None)
        if rank < 3 or fit[0] <= 0.0:
            continue
        out[j] = fit[0], fit[1] / fit[0], fit[2]
        said[j] = 1.0
    return out, said
