"""Her whole body as one stack of QPs a step: torques that keep her contacts, by priority.

    body = Body()                           # her model laid once, a MjData of its own
    s = sense(body, qpos, qvel)             # M, bias, Jacobians, the soles, the centre of mass
    out = step(body, s, ask)                # ask: stance, the centre of mass's acceleration,
                                            # the pelvis's and the trunk's turns, a swing, posture
    out['tau'], out['qacc'], out['force']   # driven joints' N m (JOINTS order), rad/s^2, N

The plant is MuJoCo's: M(q) with the rotors' reflected inertia, the bias C q' + g, the passive
forces; the floating base on SE(3), a turn's error SO(3)'s log. The unknowns: her 39
accelerations and a force at each corner of a standing sole; the torques follow. Held at every
level (`qp.stack`): the undriven rows of the dynamics, a standing sole still, each corner's
force pressing inside the friction pyramid, the drives' clamps as built (the ankle's pair as
`physics.paired`, past the clamp to WEP's ceiling where granted), the stops a horizon ahead.
By priority then: the centre of mass along the floor and her weight borne (soft); the pelvis's
and the trunk's turns, a swinging sole and each sole's centre of pressure inside its edge
(soft); her form weighed - her height, the angular momentum, the posture, the torques, the
forces - and each drive within its clamp (soft). Each level's tie-break is the levels below.
"""
import importlib
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402

from machine import drives, linkage, qp  # noqa: E402
from machine.drives import kind  # noqa: E402
from machine.errors import MachineError  # noqa: E402
from machine.figure import JOINTS, SOLE_HALF  # noqa: E402
from machine.gait import ANKLE_H, BALL, HEEL  # noqa: E402

G = 9.81

#: The friction pyramid's slope, inside the soles' elliptic cone of `mjcf.FRICTION` 1.0; a
#: standing corner presses at least F_MIN_N.
MU, F_MIN_N = 0.6, 0.0

#: A sole's centre of pressure MARGIN_M inside its edge, softly (weight SLACK_W against the
#: centre of mass's m/s^2).
MARGIN_M, SLACK_W = 0.015, 1e3

#: Her vertical acceleration within HEIGHT_BAND m/s^2 of her height's ask on the balance's
#: level, her height itself under the turns and a swing.
HEIGHT_BAND = 3.0

#: A stop met no nearer than STOP_RAD, LIMIT_S ahead at the acceleration asked.
LIMIT_S, STOP_RAD = 0.05, 0.01

#: A standing sole's slip bled at CONTACT_K 1/s; a held joint's (`drives.passive` None) at HELD_K.
CONTACT_K, HELD_K = 20.0, 50.0

#: A drive asked no more than CLAMP_SHARE of its clamp as its board derates it - past it, to the
#: stack's own peak at its board's amps (`drives.peak`), only granted war emergency power (the
#: board's derate held off, `thermal.wep`) and only where the centre of mass asks it; the soft
#: rows laid for the drives that bore more than NEAR of their clamps the step before.
CLAMP_SHARE, NEAR = 0.95, 0.6

#: Her neck's and head's ranges, deg, the stack's own - the model has none: granted WEP, the
#: head and the neck ran at their stacks' 51 N m, 6.4 and 3.4 times their clamps, spun as
#: weights (120 N, 2026-10-10).
RANGES = {'neck': (-45.0, 60.0), 'head': (-80.0, 80.0)}

#: (kp 1/s^2, kd 1/s): the turns', the posture's and a swinging sole's (at 400 and 40 a 0.2 s
#: step lagged its path 11 cm, 100 N, 2026-10-10); the angular momentum bled at MOMENTUM_K 1/s - at 10 on the level under the centre of mass it pinned both soles'
#: centres of pressure to their edges, flipping each 50 ms (a 60 N shove, 2026-10-10).
TURN_KD, POSTURE_KD, SWING_KD, MOMENTUM_K = (100.0, 20.0), (100.0, 20.0), (1600.0, 80.0), 3.0

#: The turns' weight against a swinging sole's m/s^2; her form's against the posture's
#: rad/s^2: the angular momentum's N m, the torques' (a clamp's share), the corners' forces
#: (her weight's share), her height's m/s^2.
TURN_W, MOMENTUM_W, POSTURE_W, TORQUE_W, FORCE_W, HEIGHT_W = 3.0, 0.3, 1.0, 3.0, 1.0, 1.0

#: A swinging knee's fold's weight against its sole's m/s^2.
FOLD_W = 0.3


def _currents(b, T, t0):
    """(rows, offsets): each drive's load as its boards carry it - a joint's own torque, a
    parallel pair's two currents times the first's kt and the second's (`physics.paired`)."""
    rows, offs = T.copy(), t0.copy()
    for p, r in b.pairs:
        a, c = T[p] / b.kt[p], T[r] / b.kt[r]
        rows[p], rows[r] = (a + c) * b.kt[p], (a - c) * b.kt[r]
        a0, c0 = t0[p] / b.kt[p], t0[r] / b.kt[r]
        offs[p], offs[r] = (a0 + c0) * b.kt[p], (a0 - c0) * b.kt[r]
    return rows, offs


def _skew(p):
    return np.array([[0.0, -p[2], p[1]], [p[2], 0.0, -p[0]], [-p[1], p[0], 0.0]])


def turn_error(want, have):
    """The world-frame rotation vector taking quaternion `have` to `want`, rad (SO(3)'s log)."""
    w0, x0, y0, z0 = have
    w1, x1, y1, z1 = want
    # want * conj(have)
    w = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0
    v = np.array([-w1 * x0 + x1 * w0 - y1 * z0 + z1 * y0,
                  -w1 * y0 + y1 * w0 - z1 * x0 + x1 * z0,
                  -w1 * z0 + z1 * w0 - x1 * y0 + y1 * x0])
    if w < 0.0:
        w, v = -w, -v
    s = float(np.linalg.norm(v))
    return v * (2.0 * math.atan2(s, w) / s) if s > 1e-12 else 2.0 * v


class Body:

    """Her model laid once for the law: a MjData of its own, her degrees of freedom sorted
    driven, held and free, the clamps, the soles' corners."""

    def __init__(self, xml=None):
        from machine import mjcf, physics
        mj = self.mj = importlib.import_module('mujoco')
        m = self.m = mj.MjModel.from_xml_string(xml or mjcf.mjcf())
        m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONSTRAINT)
        self.d = mj.MjData(m)
        self.pelvis, self.torso = m.body('pelvis').id, m.body('torso').id

        def hers(body):
            while body:
                if body == self.pelvis:
                    return True
                body = m.body_parentid[body]
            return False

        self.n = n = sum(hers(m.dof_bodyid[i]) for i in range(m.nv))
        vadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
        qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
        spring = [drives.passive(j) for j in JOINTS]
        self.driven = np.array([p is None for p in spring])
        self.act, self.qact = vadr[self.driven], qadr[self.driven]
        self.held = vadr[[p is not None and p[0] is None for p in spring]]
        self.free = np.array(sorted(set(range(n)) - set(self.act) - set(self.held)))
        names = [j for j, p in zip(JOINTS, spring) if p is None]
        self.clamp = np.array([min(physics.SERVO[kind(j)][0], drives.peak(j)) if physics.CLAMPED
                               else physics.SERVO[kind(j)][0] for j in names])
        self.wep = np.maximum(self.clamp, [drives.peak(j) for j in names])
        self.top = CLAMP_SHARE * self.clamp
        #: Each drive's load the last step, of its clamp as derated (`_currents`); those of
        #: them over NEAR granted WEP this step.
        self.load, self.near = np.zeros(len(names)), np.zeros(0, int)
        #: Each knee's index among the driven, left and right.
        self.knees = (names.index('left_knee'), names.index('right_knee'))
        self.kt = np.array([drives.kt(j) for j in names])
        self.pairs = [(names.index(j), names.index(j[:-len(kind(j))] + linkage.PAIRS[kind(j)]))
                      for j in names if kind(j) in linkage.PAIRS]
        self.single = np.ones(len(names), bool)
        for p, r in self.pairs:
            self.single[p] = self.single[r] = False
        stops = [(k, m.jnt_range[m.joint(j).id] if m.jnt_limited[m.joint(j).id]
                  else np.radians(RANGES[kind(j)])) for k, j in enumerate(names)
                 if m.jnt_limited[m.joint(j).id] or kind(j) in RANGES]
        self.stops = np.array([k for k, _r in stops], int)
        self.stop_lo = np.array([r[0] for _k, r in stops])
        self.stop_hi = np.array([r[1] for _k, r in stops])
        self.feet = (m.body('left_foot').id, m.body('right_foot').id)
        self.corners = np.array([(x, -ANKLE_H, z) for z in (-HEEL, BALL)
                                 for x in (-SOLE_HALF, SOLE_HALF)])
        self.sole = np.array((0.0, -ANKLE_H, (BALL - HEEL) / 2.0))
        self.mass = float(m.body_subtreemass[self.pelvis])
        self._M = np.zeros((m.nv, m.nv))
        self._jp, self._jr, self._am = (np.zeros((3, m.nv)) for _ in range(3))
        #: Each stance's active rows a level, the last step's (`qp.stack`'s warm start).
        self.warm = {}


def sense(b, qpos, qvel):
    """What the law reads of her at (qpos, qvel): {M, h (bias less passive), v, q, com, vcom,
    k (angular momentum about the centre of mass), feet [{R, quat, pts, Jc, J6, dJv, sole}],
    turns [{quat, Jr, dJr, w}] (pelvis, trunk)}, world frame."""
    m, d, mj, n = b.m, b.d, b.mj, b.n
    d.qpos[:len(qpos)], d.qvel[:len(qvel)] = qpos, qvel
    mj.mj_fwdPosition(m, d)
    mj.mj_fwdVelocity(m, d)
    mj.mj_fullM(m, d, b._M)
    v = d.qvel[:n].copy()
    jp, jr = b._jp, b._jr
    mj.mj_jacSubtreeCom(m, d, jp, b.pelvis)
    vcom = jp[:, :n] @ v
    mj.mj_angmomMat(m, d, b._am, b.pelvis)
    feet = []
    for foot in b.feet:
        R, o = d.xmat[foot].reshape(3, 3).copy(), d.xpos[foot].copy()
        pts = o + b.corners @ R.T
        Jc = np.empty((4, 3, n))
        for i in range(4):
            mj.mj_jac(m, d, jp, None, pts[i], foot)
            Jc[i] = jp[:, :n]
        sole = o + R @ b.sole
        mj.mj_jac(m, d, jp, jr, sole, foot)
        J6 = np.vstack([jr[:, :n], jp[:, :n]])
        mj.mj_jacDot(m, d, jp, jr, sole, foot)
        feet.append({'R': R, 'quat': d.xquat[foot].copy(), 'pts': pts, 'Jc': Jc, 'J6': J6,
                     'dJv': np.vstack([jr[:, :n], jp[:, :n]]) @ v, 'sole': sole})
    turns = []
    for body in (b.pelvis, b.torso):
        at = d.xpos[body].copy()
        mj.mj_jac(m, d, None, jr, at, body)
        Jr = jr[:, :n].copy()
        mj.mj_jacDot(m, d, None, jr, at, body)
        turns.append({'quat': d.xquat[body].copy(), 'Jr': Jr, 'dJr': jr[:, :n] @ v, 'w': Jr @ v})
    return {'M': b._M[:n, :n].copy(), 'h': d.qfrc_bias[:n] - d.qfrc_passive[:n], 'v': v,
            'q': d.qpos[b.qact].copy(), 'com': d.subtree_com[b.pelvis].copy(), 'vcom': vcom,
            'k': b._am[:, :n] @ v, 'feet': feet, 'turns': turns}


def step(b, s, ask):
    """{tau, qacc, force, cop, bears, slack, held, over}: the stack solved for `ask` - stance (left,
    right) bools; com_acc (3,) m/s^2; turns (pelvis, trunk) quaternions; swing {side: (sole's
    place, speed, acceleration, quaternion)}; fold {side: (knee rad, rad/s, rad/s^2)}, a
    swinging knee's; posture, the driven joints' rad; optionally kdot (3,) N m, else the
    angular momentum bled; derate, each drive's board's (1); wep, the drives granted war
    emergency power (none). `bears`: each sole's planned load, N; `over`: the drives asked past
    their clamps, whose boards must hold their derates off this step."""
    n = b.n
    stance = [k for k in (0, 1) if ask['stance'][k]]
    nx = n + 12 * len(stance)
    held = _held(b, s, ask, stance)
    E, e, Gh, hh, T, t0, Th, th, nominal = held
    levels = _levels(b, s, ask, stance, held)
    key = tuple(stance)
    x, active, slack = qp.stack(E, e, Gh, hh, levels, b.warm.get(key, ()))
    if x is None:
        # The held rows cannot all be met: their least violation first, the levels under it.
        x, active, slack = qp.stack(E, e, np.zeros((0, nx)), np.zeros(0),
                                    [(np.zeros((0, nx)), np.zeros(0), Gh, hh, 1.0)] + levels)
        active = active[1:]
    if x is None:
        raise MachineError('the whole-body stack found no solution')
    b.warm[key] = active
    f = b.mass * G * x[n:].reshape(-1, 3)
    cop, bears = [], [0.0, 0.0]
    for j, k in enumerate(stance):
        load = f[4 * j:4 * j + 4, 1]
        bears[k] = float(load.sum())
        cop.append(b.corners[:, (0, 2)].T @ load / load.sum() if load.sum() > 1e-6 else None)
    b.load = np.abs(Th @ x + th) / nominal
    past = b.load > 1.0 + 1e-6
    over = past.copy()
    for p, r in b.pairs:
        over[p] = over[r] = past[p] or past[r]
    return {'tau': T @ x + t0, 'qacc': x[:n], 'force': f, 'cop': cop, 'bears': bears,
            'slack': slack, 'held': active, 'over': over}


def _held(b, s, ask, stance):
    """(E, e, G, h, T, t0, Th, th, nominal): the rows held at every level - E x = e the undriven
    dynamics, the held joints, the standing soles; G x <= h the corners' pyramids, the drives'
    clamps (WEP's ceiling where granted and laid), the stops; tau = T x +
    t0, the drives' loads Th x + th (`_currents`), their clamps as derated."""
    n, fs = b.n, b.mass * G
    nc = 4 * len(stance)
    nx, M, h, v = n + 3 * nc, s['M'], s['h'], s['v']
    Jc = (np.concatenate([s['feet'][k]['Jc'].reshape(12, n) for k in stance]) if stance
          else np.zeros((0, n)))
    JcT = Jc.T * fs
    T, t0 = np.hstack([M[b.act], -JcT[b.act]]), h[b.act]
    E, e = [np.hstack([M[b.free], -JcT[b.free]])], [-h[b.free]]
    if len(b.held):
        held = np.zeros((len(b.held), nx))
        held[np.arange(len(b.held)), b.held] = 1.0
        E.append(held)
        e.append(-HELD_K * v[b.held])
    for k in stance:
        f = s['feet'][k]
        E.append(np.hstack([f['J6'], np.zeros((6, 3 * nc))]))
        e.append(-f['dJv'] - CONTACT_K * (f['J6'] @ v))
    Gs, hs = [], []
    if nc:
        cone = np.array([[1.0, -MU, 0.0], [-1.0, -MU, 0.0], [0.0, -MU, 1.0], [0.0, -MU, -1.0],
                         [0.0, -1.0, 0.0]])
        Gf = np.zeros((5 * nc, nx))
        for i in range(nc):
            Gf[5 * i:5 * i + 5, n + 3 * i:n + 3 * i + 3] = cone
        Gs.append(Gf)
        hs.append(np.tile([0.0, 0.0, 0.0, 0.0, -F_MIN_N / fs], nc))
    nominal = b.clamp * np.broadcast_to(np.asarray(ask.get('derate', 1.0), float), b.clamp.shape)
    granted = np.broadcast_to(np.asarray(ask.get('wep', False), bool), b.clamp.shape)
    # WEP's ceiling only for those whose soft rows are laid (`_levels`): unlaid, the head was
    # asked 3.6 times its clamp from under NEAR in one step (80 N from behind, 2026-10-10).
    b.near = np.flatnonzero(granted & (b.load > NEAR))
    ceiling = nominal.copy()
    ceiling[b.near] = b.wep[b.near]
    Th, th = _currents(b, T, t0)
    Gs += [Th, -Th]
    hs += [ceiling - th, ceiling + th]
    if len(b.stops):
        dof, q = b.act[b.stops], s['q'][b.stops]
        ahead = q + v[dof] * LIMIT_S
        stop = np.zeros((2 * len(dof), nx))
        stop[np.arange(len(dof)), dof] = 1.0
        stop[len(dof) + np.arange(len(dof)), dof] = -1.0
        Gs.append(stop)
        hs.append(np.concatenate([2.0 * (b.stop_hi - STOP_RAD - ahead) / LIMIT_S ** 2,
                                  -2.0 * (b.stop_lo + STOP_RAD - ahead) / LIMIT_S ** 2]))
    return (np.vstack(E), np.concatenate(e), np.vstack(Gs), np.concatenate(hs), T, t0, Th, th,
            nominal)


def _levels(b, s, ask, stance, held):
    """The stack's levels (`qp.stack`) for `ask`, on the rows `_held` laid."""
    T, t0, Th, th, nominal = held[4:]
    n, mass, v = b.n, b.mass, s['v']
    nc, fs = 4 * len(stance), mass * G
    nx = n + 3 * nc
    # The centre of mass along the floor, by Newton on the corners' forces; her weight borne,
    # her vertical acceleration within HEIGHT_BAND of her height's ask.
    A1, up = np.zeros((2 if nc else 0, nx)), np.zeros((1 if nc else 0, nx))
    for i in range(nc):
        A1[:, n + 3 * i:n + 3 * i + 3] = np.eye(3)[[0, 2]] * fs / mass
        up[0, n + 3 * i + 1] = fs / mass
    ay = float(ask['com_acc'][1]) + G
    levels = [(A1, np.asarray(ask['com_acc'])[[0, 2]][:len(A1)], np.vstack([up, -up]),
               np.array([ay + HEIGHT_BAND, HEIGHT_BAND - ay])[:2 * len(up)], SLACK_W)]
    # The pelvis's and the trunk's turns, a swinging sole and its knee's fold: swung on the
    # level above the turns, the pelvis pitched to throw the foot, 4 to 25 deg in 0.1 s; the
    # knee straight, its sole could not rise - asked up 17 m/s^2, given 1.5 (80 N from behind,
    # 2026-10-10).
    A2, b2 = [np.zeros((0, nx))], [np.zeros(0)]
    for t, want in zip(s['turns'], ask['turns']):
        wdot = TURN_KD[0] * turn_error(want, t['quat']) - TURN_KD[1] * t['w']
        A2.append(TURN_W * np.hstack([t['Jr'], np.zeros((3, 3 * nc))]))
        b2.append(TURN_W * (wdot - t['dJr']))
    for k, (at, speed, acc, want) in ask.get('swing', {}).items():
        f = s['feet'][k]
        twist = f['J6'] @ v
        a6 = np.concatenate([TURN_KD[0] * turn_error(want, f['quat']) - TURN_KD[1] * twist[:3],
                             np.asarray(acc) + SWING_KD[0] * (np.asarray(at) - f['sole'])
                             + SWING_KD[1] * (np.asarray(speed) - twist[3:])])
        A2.append(np.hstack([f['J6'], np.zeros((6, 3 * nc))]))
        b2.append(a6 - f['dJv'])
    for k, (knee, rate, acc) in ask.get('fold', {}).items():
        i = b.knees[k]
        row = np.zeros((1, nx))
        row[0, b.act[i]] = FOLD_W
        A2.append(row)
        b2.append(FOLD_W * np.array([acc + SWING_KD[0] * (knee - s['q'][i])
                                     + SWING_KD[1] * (rate - v[b.act[i]])]))
    # Each sole's centre of pressure inside its edge, softly, here: on the balance's level its
    # slack held corners of the standing sole at their friction's edge, frozen below it, and the
    # swing could not lift its sole (80 N from behind, 2026-10-10).
    S2 = np.zeros((4 * len(stance), nx))
    edges = ((0, -SOLE_HALF + MARGIN_M, 1.0), (0, SOLE_HALF - MARGIN_M, -1.0),
             (2, -HEEL + MARGIN_M, 1.0), (2, BALL - MARGIN_M, -1.0))
    for j in range(len(stance)):
        for r, (axis, edge, sign) in enumerate(edges):
            for i in range(4):
                S2[4 * j + r, n + 3 * (4 * j + i) + 1] = -sign * (b.corners[i][axis] - edge)
    levels.append((np.vstack(A2), np.concatenate(b2), S2, np.zeros(len(S2)), SLACK_W))
    # Her form, weighed: her height, the angular momentum, the posture, the torques, the
    # forces. Her height held over the centre of mass's way straightened her standing knees from
    # 6 to 1 deg, and left no room for a sole to rise (2026-10-10).
    A3, b3 = [], []
    if nc:
        A3.append(HEIGHT_W * up)
        b3.append(HEIGHT_W * np.array([ay]))
        pts = np.concatenate([s['feet'][k]['pts'] for k in stance])
        Ak = np.zeros((3, nx))
        for i in range(nc):
            Ak[:, n + 3 * i:n + 3 * i + 3] = _skew(pts[i] - s['com']) * fs
        kdot = ask.get('kdot')
        A3.append(MOMENTUM_W * Ak)
        b3.append(MOMENTUM_W * (-MOMENTUM_K * s['k'] if kdot is None else np.asarray(kdot)))
    pose = np.zeros((len(b.act), nx))
    pose[np.arange(len(b.act)), b.act] = POSTURE_W
    A3 += [pose, TORQUE_W * T / b.top[:, None]]
    b3 += [POSTURE_W * (POSTURE_KD[0] * (np.asarray(ask['posture']) - s['q'])
                        - POSTURE_KD[1] * v[b.act]), -TORQUE_W * t0 / b.top]
    if nc:
        A3.append(np.hstack([np.zeros((3 * nc, n)), FORCE_W * np.eye(3 * nc)]))
        b3.append(np.zeros(3 * nc))
    # The drives granted WEP within CLAMP_SHARE of their clamps, softly, here: on the balance's
    # level the neck's slack bent it - bearing less weight eased the neck.
    near = b.near
    cap = nominal[near]
    levels.append((np.vstack(A3), np.concatenate(b3),
                   np.vstack([Th[near] / cap[:, None], -Th[near] / cap[:, None]]),
                   np.concatenate([CLAMP_SHARE - th[near] / cap, CLAMP_SHARE + th[near] / cap]),
                   SLACK_W))
    return levels
