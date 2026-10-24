"""Her generalized momentum observer: the external force on her, blind.

    obs = observer.laid(body)              # her patches' points laid once, the observer at rest
    observer.felt(obs, y, dt)              # each drive pass: her drives' torques taken (`edge.TAU`)
    observer.step(obs, body, s, qpos, dt)  # each law pass on her law's sense and estimate: r, her patches
    obs['r'], obs['normal']                # the external force on her dofs; each manifold's, N (MANIFOLDS)

The residual of her momentum over a law pass, M (v - v') / dt + h - S'tau - her pelvis's linear
acceleration her IMU's, tau her drives' mean over it and their friction -, low-passed at the kernel's bandwidth (`configure-momentum-observer`), is J'f, the external
force on her. Each manifold's frames (`anatomical-manifold-registry`) are points of her skin, each
pressed along its outward normal alone, a sole a whole wrench: f the least squares of J'f = r, each
normal pushing - ROUNDS solves, each pulling normal pinned for the next -, the least of it where
pushes cancel (both hips squeezed along one line) by the QP's tie-break (`qp.EPS`). Blind: no skin,
no load cell.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, kernel, ortho, qp
from machine.figure import BALL, BODY, CONTACTS, FIST_AT, FIST_R, HEEL, JOINTS, PADS, pad

#: Her manifolds: the registry's, then its somatic extensions'; her forefeet's, of her soles' points.
MANIFOLDS = tuple(kernel.read()['anatomical-manifold-registry']) + tuple(kernel.read()['somatic-contact-patch-extensions'])
EQUINUS = MANIFOLDS.index('metatarsal-equinus-bilateral')
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


def _forearm():
    """A forearm's front by its wrist (`figure.BODY`'s forearm), pressed back: a chair shoved."""
    _p, r, _a, b = next(c for c in BODY if c[0] == 'forearm')
    return (b[0], b[1] + r, b[2] + r)
#: Each point: (its manifolds, each (manifold, the segment whose frame holds its normal, that normal)),
#: its segment, the point, its outward normal - None a whole wrench -, in the segment's frame. A
#: sole's wrench at `wbc.Body.sole`; the toes' underside; each buttock's; each thigh's
#: back where the seat's patch ends ahead of the buttocks and the back's begins below the sacrum -
#: its own, and sat pressed up her pelvis (the seat's), lain pressed along her back (the back's) -;
#: each hip's side, pressed in; the sacrum, the lumbar and thoracic back and the skull's, pressed
#: forward; each palm's heel, pressed up its arm, a brace's or a shove's; each forearm's front by its
#: wrist, pressed back, a shove's.
_SEAT, _LAIN = ('ischial-bilateral', 'pelvis', _UP), ('dorsal-supine-continuum', 'pelvis', _BACK)
POINTS = tuple((((m, seg if frame is None else frame, n if axis is None else axis),) if isinstance(m, str) else m,
                seg, p, n) for m, frame, axis, seg, p, n in (
    ('plantar-bilateral', None, _UP, 'left_foot', None, None), ('plantar-bilateral', None, _UP, 'right_foot', None, None),
    ('plantar-bilateral', None, None, 'left_toes', _TOE, _UP), ('plantar-bilateral', None, None, 'right_toes', _TOE, _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 0), _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 1), _UP),
    ((('posterior-femoral-bilateral', 'left_thigh', _BACK), _SEAT, _LAIN), None, None, 'left_thigh', _thigh(), _BACK),
    ((('posterior-femoral-bilateral', 'right_thigh', _BACK), _SEAT, _LAIN), None, None, 'right_thigh', _thigh(), _BACK),
    ('trochanter-gluteal-lateral', None, None, 'pelvis', _hip(1.0), (-1.0, 0.0, 0.0)),
    ('trochanter-gluteal-lateral', None, None, 'pelvis', _hip(-1.0), (1.0, 0.0, 0.0)),
    ('dorsal-supine-continuum', None, None, 'pelvis', _behind('pelvis', 2), _BACK),
    ('dorsal-supine-continuum', None, None, 'torso', _behind('torso', 0), _BACK),
    ('dorsal-supine-continuum', None, None, 'torso', _behind('torso', 2), _BACK),
    ('dorsal-supine-continuum', None, None, 'head', _behind('head', 0), _BACK),
    ((('palmar-carpal-bilateral', 'left_hand', _UP), ('ulnar-carpal-shove-bilateral', 'left_hand', _UP)), None, None,
     'left_hand', _PALM, _UP),
    ((('palmar-carpal-bilateral', 'right_hand', _UP), ('ulnar-carpal-shove-bilateral', 'right_hand', _UP)), None, None,
     'right_hand', _PALM, _UP),
    ('ulnar-carpal-shove-bilateral', None, None, 'left_forearm', _forearm(), (0.0, 0.0, -1.0)),
    ('ulnar-carpal-shove-bilateral', None, None, 'right_forearm', _forearm(), (0.0, 0.0, -1.0))))
#: The unknowns of each point: a sole's six, a normal's one.
WIDTH = tuple(6 if n is None else 1 for _m, _s, _p, n in POINTS)


#: Her rows' manifolds (`ortho.MANIFOLDS`) among the observer's; the text's comparators on each,
#: of her weight (`latch-hysteretic-contact`): engaged at, released under, held below that long,
#: none released before so long engaged; its weights' slews up and down, a second
#: (`bind-wrench-driven-homotopy`).
ROWS = np.array([[m.split('-')[0] for m in MANIFOLDS].index(m) for m in ortho.MANIFOLDS])
_LATCH = kernel.node('latch-hysteretic-contact', 'contact-comparators')
ENGAGE, RELEASE = _LATCH['schmitt-high-engage-fz'], _LATCH['schmitt-low-release-fz']
DWELL_S, BLANK_S = 1e-3 * _LATCH['min-state-dwell-ms'], 1e-3 * _LATCH['impact-blanking-window-ms']
UP, DOWN = (kernel.said('bind-wrench-driven-homotopy', k) for k in ('slew-rate-up-per-s', 'slew-rate-down-per-s'))


def weighed(obs, told, weight, dt):
    """(her rows' manifolds,) what bears her as her law takes it, `dt` s on: each manifold her
    rows tell weighed as told where her comparator (ENGAGE, RELEASE, DWELL_S, BLANK_S) feels it -
    its normal force of her `weight` -, gated up at UP and down at DOWN a second; what a manifold
    not felt was told, her soles'. Her soles alone bear her as told; a seat pulled away weighs
    nothing at once."""
    n = obs['normal'][ROWS] / weight
    latched = obs['latched']
    on = (1.0 - latched) * (n >= ENGAGE)
    under = latched * (n < RELEASE)
    obs['below'] = under * (obs['below'] + dt)
    obs['since'] = (1.0 - on) * (obs['since'] + dt)
    obs['latched'] = latched + on - under * (obs['since'] >= BLANK_S) * (obs['below'] >= DWELL_S)
    gate = obs['gate'] = obs['gate'] + np.clip(obs['latched'] - obs['gate'], -DOWN * dt, UP * dt)
    w = told * gate
    w[0] = told[0] + float(told[1:] @ (1.0 - gate[1:]))
    return w


def bearing(obs):
    """(k, 3) the points that press on a manifold her comparator feels, world: what bears her besides
    her soles."""
    rows = np.full(len(MANIFOLDS), -1)
    rows[ROWS] = np.arange(len(ROWS))
    felt = np.concatenate([obs['latched'], [0.0]])[rows[obs['manifold']]]
    return obs['points'][(~obs['sole']) & (felt > 0.5) & (obs['f'][obs['cols']] > 0.0)]


#: The rounds of the split: a least squares, each normal pulling pinned for the next - the
#: whole-body step's (`wrench.ROUNDS`) - fixed, so a pass's cost is.
ROUNDS = 4


def _link(seg):
    """A segment's link in her chain (wbc_model.h): its last joint's, the pelvis the root."""
    from machine.figure import SEGMENTS
    joints = next(j for name, _p, j, *_r in SEGMENTS if name == seg)
    return joints[-1][0] if joints else 'root'


def laid(body):
    """The observer at rest on her law's `body` (`wbc.Body`), in C (wbc_gmo.c): its points, shares and
    her drives' friction laid, its sums empty; its outputs viewed (`r`, `f`, `normal`)."""
    import ctypes
    from tools.cores import gmo, wbc as cores
    lib, names = cores.library(), cores.Core().names
    o = gmo.Gmo()
    o.rounds, o.slip, o.eps = ROUNDS, SLIP_RAD_S, qp.EPS
    o.alpha = kernel.said('configure-momentum-observer', 'observer-bandwidth-hz') * 2.0 * math.pi
    o.loss[:] = body.m.dof_frictionloss[:body.n].tolist()
    driven = [k for k, j in enumerate(JOINTS) if body.driven[k]]
    o.driven_joint[:] = driven
    o.driven_at[:] = [5 + names.index(JOINTS[k]) for k in driven]
    col = 0
    for p, (shares, seg, point, normal) in enumerate(POINTS):
        o.link[p], o.sole[p], o.col[p] = names.index(_link(seg)), int(normal is None), col
        o.local[p][:] = list(body.sole if point is None else point)
        o.normal[p][:] = [0.0, 0.0, 0.0] if normal is None else list(normal)
        col += WIDTH[p]
    rows = [(p, MANIFOLDS.index(m), names.index(_link(frame)), axis)
            for p, (shares, *_r) in enumerate(POINTS) for m, frame, axis in shares]
    for k, (p, m, link, axis) in enumerate(rows):
        o.share_point[k], o.share_manifold[k], o.share_link[k] = p, m, link
        o.share_axis[k][:] = np.asarray(axis, float).tolist()
    view = lambda field: np.ctypeslib.as_array(getattr(o, field))
    return {'c': o, 'at': ctypes.byref(o), 'felt': lib.wbh_gmo_felt, 'step': lib.wbh_gmo_step,
            'r': view('r'), 'f': view('f'), 'normal': view('force'), 'points': view('at'),
            'cols': np.array([o.col[p] for p in range(len(POINTS))]), 'sole': np.array([n is None for *_r, n in POINTS]),
            'manifold': np.array([MANIFOLDS.index(sh[0][0]) for sh, *_r in POINTS]),
            'feet': [([k for k, (_sh, seg, _p, n) in enumerate(POINTS) if seg == side + '_foot'][0],
                      [k for k, (_sh, seg, _p, n) in enumerate(POINTS) if seg == side + '_toes'][0]) for side in ('left', 'right')], 'base': np.zeros(12), 'q': np.zeros(body.n - 6),
            'u': np.zeros(body.n), 'latched': np.zeros(len(ROWS)), 'below': np.zeros(len(ROWS)),
            'since': np.zeros(len(ROWS)), 'gate': np.zeros(len(ROWS))}


def felt(obs, y, dt):
    """A drive pass `dt` of her edge's array `y` into the law pass's sums: her drives' torques, her
    pelvis's acceleration (her IMU's specific force turned to the world, gravity added)."""
    at = y.ctypes.data
    obs['felt'](obs['at'], at + 8 * edge.QUAT.start, at + 8 * edge.ACCEL.start, at + 8 * edge.TAU.start, dt)


def step(obs, body, s, qpos, dt):
    """A law pass on her law's sense `s` (`wbc.sense`) at her estimate `qpos`: r, f and each
    manifold's normal force; her drives' sums emptied."""
    from tools.cores.model import rotation
    v, n, base, u = s['v'], body.n, obs['base'], obs['u']
    R = rotation(qpos[3:7])
    base[:9], base[9:] = R.ravel(), qpos[:3]
    u[:3], u[3:6], u[6:] = v[3:6], R.T @ v[:3], v[6:n]
    obs['q'][:] = qpos[7:7 + n - 6]
    obs['step'](obs['at'], base.ctypes.data, obs['q'].ctypes.data, u.ctypes.data, dt)
    body.d.qpos[:len(qpos)] = qpos
    body.mj.mj_kinematics(body.m, body.d)
    obs['normal'][EQUINUS] = forefoot(obs['f'], obs['cols'], obs['feet'], body.d.xmat[list(body.feet)])
    return obs


def forefoot(f, cols, feet, xmat):
    """Her forefeet's push (`metatarsal-equinus-bilateral`), N, of the points' forces `f` and her feet's
    rotations `xmat` (2, 9): each toes' and of each sole's push the share its centre of pressure has
    gone from the sole's middle to its ball (`figure.BALL`)."""
    out = 0.0
    for (sole, toes), rot in zip(feet, xmat):
        R = rot.reshape(3, 3)
        moment, force = f[cols[sole]:cols[sole] + 3], f[cols[sole] + 3:cols[sole] + 6]
        push = float(force @ R[:, 1])
        ahead = -float(moment @ R[:, 0]) / max(push, 1e-9)
        out += push * min(1.0, max(0.0, ahead / (0.5 * (BALL + HEEL)))) + f[cols[toes]]
    return out
