"""Her generalized momentum observer: the external force on her, blind.

    obs = observer.laid(body)              # her patches' points laid once, the observer at rest
    observer.felt(obs, y, dt)              # each drive pass: her drives' torques taken (`edge.TAU`)
    observer.step(obs, body, s, qpos, dt)  # each law pass on her law's sense and estimate: r, her patches
    obs['r'], obs['normal']                # the external force on her dofs; each manifold's, N (MANIFOLDS)

The residual of her momentum over a law pass, M (v - v') / dt + h - S'tau - her pelvis's linear
acceleration her IMU's, tau her drives' mean over it and their friction -, low-passed at the kernel's bandwidth (`configure-momentum-observer`), is J'f, the external
force on her. Each manifold's frames (`anatomical-manifold-registry`) are points of her skin, each
pressed along its outward normal alone, a sole a whole wrench: f the least squares of J'f = r, each
normal pushing, the least of it where pushes cancel - both hips squeezed along one line - by the
QP's tie-break (`qp.solve`, `qp.EPS`). Blind: no skin, no load cell.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, kernel, qp
from machine.figure import BODY, CONTACTS, FIST_AT, FIST_R, PADS, pad
from machine.figure import JOINTS

#: Her manifolds, the registry's order.
MANIFOLDS = tuple(kernel.read()['anatomical-manifold-registry'])
#: Her drives' friction its loss (`drives.backdrive`, as her model has it) times tanh(v / SLIP_RAD_S)
#: (the text's Stribeck). Measured on MuJoCo's: 0.42 of the loss at 3-10 mrad/s, 0.65 at 10-30, 0.85 at
#: 30-100, 0.97 past; unmodelled, her soles' 1.8 m s^-2 walking read 19.5 N light and her seat 44 N
#: borne, modelled 3.3 and 27.
SLIP_RAD_S = 0.03
_UP, _BACK = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def _capsule(part, k):
    """`part`'s k-th capsule of `figure.BODY`: (radius, its axis's middle)."""
    _p, r, a, b = [c for c in BODY if c[0] == part][k]
    return r, np.add(a, b) / 2.0


def _under(part, k):
    r, mid = _capsule(part, k)
    return tuple(mid - (0.0, r, 0.0))


def _behind(part, k):
    r, mid = _capsule(part, k)
    return tuple(mid - (0.0, 0.0, r))


def _thigh():
    """A thigh's underside where the seat's patch ends ahead of the buttocks (`cop-bounds-mm`), on her
    thigh's capsule, sat: pressed up from behind it."""
    r, mid = _capsule('thigh', 0)
    ahead = kernel.read()['anatomical-manifold-registry']['ischial-bilateral']['cop-bounds-mm']['x'][1] / 1e3
    return (mid[0], -ahead, mid[2] - r)


def _hip(x):
    """A hip's outermost point on her pelvis, its trochanter's pad (`figure.PADS`), `x` 1 the left."""
    _p, axis, radius, toward, size, _w = [p for p in PADS if p[0] == 'pelvis'][-1]
    c = pad(axis, radius, toward, size, x)
    return (c[0] + x * size, c[1], c[2])


_TOES = next(c for c in CONTACTS if c[0] == 'toes')
_TOE = tuple(np.subtract(_TOES[3], (0.0, _TOES[2][1], 0.0)))
_PALM = tuple(np.subtract(FIST_AT, (0.0, FIST_R, 0.0)))
#: Each point: (its manifolds, each (manifold, the segment whose frame holds its normal, that normal)),
#: its segment, the point, its outward normal - None a whole wrench -, in the segment's frame. A
#: sole's wrench at `wbc.Body.sole`; the toes' underside; each buttock's; each thigh's back where the
#: seat's patch ends ahead of the buttocks and the back's begins below the sacrum - sat, pressed up
#: her pelvis (the seat's); lain, pressed along her back (the back's) -; each hip's side, pressed
#: in; the sacrum, the lumbar and thoracic back and the skull's, pressed forward; each palm's heel,
#: pressed up its arm.
_SEAT, _LAIN = ('ischial-bilateral', 'pelvis', _UP), ('dorsal-supine-continuum', 'pelvis', _BACK)
POINTS = tuple((((m, seg if frame is None else frame, n if axis is None else axis),) if isinstance(m, str) else m,
                seg, p, n) for m, frame, axis, seg, p, n in (
    ('plantar-bilateral', None, _UP, 'left_foot', None, None), ('plantar-bilateral', None, _UP, 'right_foot', None, None),
    ('plantar-bilateral', None, None, 'left_toes', _TOE, _UP), ('plantar-bilateral', None, None, 'right_toes', _TOE, _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 0), _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 1), _UP),
    ((_SEAT, _LAIN), None, None, 'left_thigh', _thigh(), _BACK), ((_SEAT, _LAIN), None, None, 'right_thigh', _thigh(), _BACK),
    ('trochanter-gluteal-lateral', None, None, 'pelvis', _hip(1.0), (-1.0, 0.0, 0.0)),
    ('trochanter-gluteal-lateral', None, None, 'pelvis', _hip(-1.0), (1.0, 0.0, 0.0)),
    ('dorsal-supine-continuum', None, None, 'pelvis', _behind('pelvis', 2), _BACK),
    ('dorsal-supine-continuum', None, None, 'torso', _behind('torso', 0), _BACK),
    ('dorsal-supine-continuum', None, None, 'torso', _behind('torso', 2), _BACK),
    ('dorsal-supine-continuum', None, None, 'head', _behind('head', 0), _BACK),
    ('palmar-carpal-bilateral', None, None, 'left_hand', _PALM, _UP),
    ('palmar-carpal-bilateral', None, None, 'right_hand', _PALM, _UP)))
#: The unknowns of each point: a sole's six, a normal's one.
WIDTH = tuple(6 if n is None else 1 for _m, _s, _p, n in POINTS)


def laid(body):
    """The observer at rest on her law's `body` (`wbc.Body`): its points' bodies, its sums empty."""
    m = body.m
    ids = [m.body(seg).id for _m, seg, _p, _n in POINTS]
    frames = [[(MANIFOLDS.index(man), m.body(frame).id, np.array(axis)) for man, frame, axis in shares]
              for shares, _s, _p, _n in POINTS]
    pts = [np.array(body.sole if p is None else p) for _m, _s, p, _n in POINTS]
    driven = np.array([j for j in range(len(JOINTS)) if body.driven[j]])
    nu = sum(WIDTH)
    return {'ids': ids, 'frames': frames, 'pts': pts, 'driven': driven, 'tau': np.zeros(len(driven)), 'acc': np.zeros(3), 'span': 0.0,
            'v': None, 'r': np.zeros(body.n), 'f': np.zeros(nu), 'held': (), 'A': np.zeros((body.n, nu)),
            'alpha': kernel.said('configure-momentum-observer', 'observer-bandwidth-hz') * 2.0 * math.pi,
            'normal': np.zeros(len(MANIFOLDS)), 'jp': np.zeros((3, m.nv)), 'jr': np.zeros((3, m.nv)),
            'loss': m.dof_frictionloss[:body.n].copy()}


def felt(obs, y, dt):
    """Her drives' torques and her pelvis's acceleration (her IMU's specific force turned to the
    world, gravity added) over a drive pass `dt`, into the law pass's sums."""
    w, x, yy, z = y[edge.QUAT]
    turn = np.array([[1 - 2 * (yy * yy + z * z), 2 * (x * yy - w * z), 2 * (x * z + w * yy)],
                     [2 * (x * yy + w * z), 1 - 2 * (x * x + z * z), 2 * (yy * z - w * x)],
                     [2 * (x * z - w * yy), 2 * (yy * z + w * x), 1 - 2 * (x * x + yy * yy)]])
    obs['tau'] += dt * y[edge.TAU][obs['driven']]
    obs['acc'] += dt * (turn @ y[edge.ACCEL] + edge.G)
    obs['span'] += dt


def _columns(obs, body, qpos):
    """Each unknown's column of J' at `qpos`: a sole's twist rows, a point's normal along its translation."""
    m, d, n, A = body.m, body.d, body.n, obs['A']
    d.qpos[:len(qpos)] = qpos
    body.mj.mj_kinematics(m, d)
    body.mj.mj_comPos(m, d)
    jp, jr, col = obs['jp'], obs['jr'], 0
    for (_m, _s, _p, normal), bid, p in zip(POINTS, obs['ids'], obs['pts']):
        R = d.xmat[bid].reshape(3, 3)
        body.mj.mj_jac(m, d, jp, jr, d.xpos[bid] + R @ p, bid)
        if normal is None:
            A[:, col:col + 3], A[:, col + 3:col + 6] = jr[:, :n].T, jp[:, :n].T
            col += 6
        else:
            A[:, col] = jp[:, :n].T @ (R @ np.array(normal))
            col += 1
    return A


def step(obs, body, s, qpos, dt):
    """A law pass on her law's sense `s` (`wbc.sense`) at her estimate `qpos`: r, f and each
    manifold's normal force; her drives' sum emptied."""
    v, tau, span = s['v'], -obs['loss'] * np.tanh(s['v'] / SLIP_RAD_S), max(obs['span'], dt)
    tau[body.act] += obs['tau'] / span
    if obs['v'] is not None:
        a = (v - obs['v']) / dt
        a[:3] = obs['acc'] / span
        e = s['M'] @ a + s['h'] - tau
        obs['r'] += (1.0 - math.exp(-obs['alpha'] * dt)) * (e - obs['r'])
    obs['tau'][:], obs['acc'][:], obs['span'] = 0.0, 0.0, 0.0
    obs['v'] = v.copy()
    A = _columns(obs, body, qpos)
    pushing = np.flatnonzero(np.array([w == 1 for w in WIDTH for _k in range(w)]))
    G = -np.eye(A.shape[1])[pushing]
    H = A.T @ A
    H[np.diag_indices_from(H)] += qp.EPS * H.diagonal().max()
    f, obs['held'], _u = qp.solve(H, -A.T @ obs['r'], G, np.zeros(len(pushing)), obs['held'])
    assert f is not None  # nothing pushing meets every row
    obs['f'] = f
    obs['normal'] = shared(body.d.xmat, obs, [f[c:c + w] for c, w in zip(np.cumsum((0,) + WIDTH), WIDTH)])
    return obs


def shared(xmat, obs, forces):
    """(MANIFOLDS,) each manifold's normal force, N, of each point's force (a sole's wrench, a
    normal's push) posed by `xmat`: a sole's along its up, a point's along each of its manifolds'
    normals, pushing."""
    normal = np.zeros(len(MANIFOLDS))
    for (_m, _s, _p, nrm), bid, frames, f in zip(POINTS, obs['ids'], obs['frames'], forces):
        world = f[3:6] if nrm is None else f[0] * (xmat[bid].reshape(3, 3) @ np.array(nrm))
        for k, frame, axis in frames:
            along = float(world @ (xmat[frame].reshape(3, 3) @ axis))
            normal[k] += along if nrm is None else max(0.0, along)
    return normal
