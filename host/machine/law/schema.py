"""The robot's shield (proprioceptive_body_schema.lisp): capsule ECBF on each other; the room's boxes.

    posture, held = schema.shielded(body, near, s, ask, volts)    # the robot's posture holding each near pair's ECBF
    boxes = schema.room(props)                                    # the room's boxes, laid once a refresh

The robot's posture's ask (its law's lowest row, `wbc.POSTURE_KD`: the null space of the robot's
balance and its steps) is held to h'' + k_d h' + k_p h >= 0 on each pair of its capsules
(`machine.body.capsules`) within the text's influence (`specialize-with-nullspace-limb-retraction`), h its
clearance less its floor and its braking distance v^2 / (2 a) within the cocoon's horizon - v its closing
speed, a the most the robot's drives' braking bounds (`machine.body.bemf`) can open it by through the robot's
mass matrix -, on the pair's evading joints, least in its mass's metric, each within its drive's peak;
what they cannot is the shield's spill.

The room: each prop a box (`world.props()` through the robot's map, `machine.eyes.registry`), its wayfinder's
gauge's (`machine.way.wayfinder`). The robot's arms' ECBF against them through their elbows alone (the text's
limb synergy it has) spared no strike at a doorway's jamb and halved its shoulder's yield after
it: not held; the text's holds the robot's trunk by its balance bus and its legs by the
robot's footholds.
"""
import functools
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import bemf
from machine.texts import kernel
from machine.stack import qp
from machine.body.capsules import EVADES, FLOOR, RETRACTION
from machine.stack.wbc import POSTURE_KD

_SHIELD = kernel.node('xform-universal-spatial-awareness', 'ecbf-shield')
#: The ECBF's gains: k_p 1/s^2, k_d 1/s.
KP, KD = _SHIELD['position-gain-kp'], _SHIELD['velocity-gain-kd']
#: A pair's clearance past its floor under which the shield holds it, m.
INFLUENCE = RETRACTION['activation-distance-m']
#: A shortfall's weight against the robot's posture's change in its mass's metric, kg: what its evading joints
#: cannot meet within their peaks is left as the shield's spill.
SPILL_KG = 1e6
#: The braking distance's horizon, s (`extrude-velocity-braking-cocoon`).
HORIZON = kernel.node('xform-universal-spatial-awareness', 'braking-cocoon')['lookahead-s']
#: The bands' refresh, s: the robot's things read so often.
REFRESH_S = 1.0 / kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['refresh-rate-hz']


def room(props):
    """{centre (C, 3), half (C, 3), R (C, 3, 3)} of the room's boxes `props` (`world.props()`'s: kind,
    centre, half sizes, rotation), laid once per list: the robot's things change at REFRESH_S, its law passes
    oftener; its registry's things (`registry.Things`) laid off the arrays they keep. Laid to a doubling's
    capacity C - the least power of two that holds them, the last of them told again for the rest: a gap among
    them is a least and an any (`wayfinder.gap`), the same with a box told twice, and the robot's gauge's C is one a
    capacity where it was one a count - the robot's eyes' things told 25 counts in a door errand, each traced and
    held anew, 15 ms."""
    if _LAID.get('of') is not props:
        if getattr(props, 'arrays', None) is not None:
            centre, half, R = props.arrays[:3]
        else:
            boxes = [p for p in props if len(p) >= 4 and len(p[1]) == 3 and np.size(p[2]) == 3 and np.size(p[3]) == 9]
            centre, half, R = (np.array([p[k] for p in boxes], float).reshape(shape)
                               for k, shape in ((1, (-1, 3)), (2, (-1, 3)), (3, (-1, 3, 3))))
        more = (1 << max(len(centre) - 1, 0).bit_length()) * bool(len(centre)) - len(centre)
        centre, half, R = (np.concatenate([a, np.repeat(a[-1:], more, axis=0)]) for a in (centre, half, R))
        _LAID.update(of=props, centre=centre, half=half, R=R)
    return {k: _LAID[k] for k in ('centre', 'half', 'R')}


#: The last props laid as boxes.
_LAID: dict = {}


def braking(J, M, act, dq, volts):
    """The most each row of `J` can be opened by, m/s^2: the robot's drives' bounds at their speeds `dq` on
    `volts` (`bemf.bounds`) through its mass matrix `M`, each drive at the bound that opens it."""
    lo, hi = bemf.bounds(dq, volts, bemf.PEAK)
    C = np.linalg.solve(M, J.T)[act].T
    return np.maximum(C * lo, C * hi).sum(1)


def _evaded(M, J, act, free, short):
    """(x, held): the change on the robot's evading joints `free` of `act` and each row's spill meeting
    `short` through `J`, least in its mass's metric `M`, each joint within its drive's peak over its
    own inertia; the rows `qp.solve` held, x zeros where none meets it."""
    k, f = len(short), len(free)
    Jf, Mf = J[:, act[free]], M[np.ix_(act[free], act[free])]
    top = np.asarray(bemf.PEAK)[free] / np.diag(Mf)
    # the QP's blocks laid as np.block lays them - each block's own zeros, -np.eye's negative zeros too -: np.block
    # was 50 us of each shield short
    H, A, spill = np.zeros((f + k, f + k)), np.zeros((2 * (k + f), f + k)), -np.eye(k)
    H[:f, :f], H[f:, f:] = Mf, SPILL_KG * np.eye(k)
    A[:k, :f], A[:k, f:], A[k:2 * k, f:] = -Jf, spill, spill
    A[2 * k:2 * k + f, :f], A[2 * k + f:, :f] = np.eye(f), -np.eye(f)
    x, held, _u = _solve()(H, np.zeros(f + k), A, np.concatenate([-short, np.zeros(k), top, top]))
    return (np.zeros(f + k) if x is None else x), held


@functools.cache
def _solve():
    """The shield's QP: the stack's dual active set in C (`tools.cores.qp`, wbc_qp.c: `qp.solve` bit for bit on an
    exact host - numpy's 207 us a solve with the robot sat, its shield short on every pass), `qp.solve` itself where
    no host compiler builds it."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        return qp.solve
    from tools.cores import qp as c_qp
    return c_qp.solve


def shielded(body, near, s, ask, volts, shield_=None):
    """(the robot's posture's ask, {d, h, which, held, spill}): `ask['posture']` changed on the text's evading
    joints of the pairs within INFLUENCE of their floors (EVADES), least in its mass's metric, each
    joint within its drive's peak over its own inertia, so that each pair keeps h'' + KD h' + KP h >= 0
    - h its clearance less its floor and its braking distance -, what it cannot its spill (m/s^2);
    `near` every pair's clearance and its Jacobian at `s` (`capsules.felt`); each pair's clearance,
    the near ones' h, the pairs it held. Every pair's h and shortfall at once (`shield`, `shield_` its C,
    `machine.law.motion.laid`); a near one short, the QP's (`_shielded`), else the robot's posture as
    asked."""
    act, posture = body.act, np.asarray(ask['posture'], float)
    which = np.flatnonzero(near['d'] - FLOOR < INFLUENCE)
    h, short = (shield if shield_ is None else shield_)(near['d'], near['J'], s['M'], s['v'], s['q'], posture, volts,
                                                         act)
    if (short[which] > 0.0).any():
        return _shielded(body, near, s, ask, volts)
    f = _evading(which.tobytes())
    return posture + np.zeros(len(act)) / POSTURE_KD[0], {'d': near['d'], 'h': h[which], 'which': which,
                                                          'spill': np.zeros(f + len(which))[f:], 'held': []}


@functools.lru_cache(maxsize=256)
def _evading(which):
    """How many joints the text's evasions move (EVADES) for the near pairs `which` (`np.flatnonzero`'s bytes): told
    once a set of pairs - each law pass 13 us, the pairs near the robot's floors a few sets in an errand."""
    return int(EVADES[np.frombuffer(which, dtype=np.intp)].any(0).sum())


def shield(d, J, M, v, q, posture, volts, act):
    """Each pair's h and its ECBF's shortfall (`_shielded`'s), of arrays alone: the robot's drives' braking
    bounds (`bemf.bounds`) through its mass matrix, the pair's closing speed, its posture's want
    on its joints `act`."""
    back, peak = bemf.E * v[act], _PEAK
    lo = np.maximum(-peak, np.minimum(0.0, bemf.K * (-(bemf.REACH * volts) - back)))
    hi = np.minimum(peak, np.maximum(0.0, bemf.K * (bemf.REACH * volts - back)))
    C = np.linalg.solve(M, J.T)[act].T
    reach = np.maximum(C * lo, C * hi).sum(1)
    rate = J @ v
    closing = np.maximum(0.0, -rate)
    h = d - FLOOR - np.minimum((closing * closing) / (2.0 * np.maximum(reach, 1e-9)), closing * HORIZON)
    want = POSTURE_KD[0] * (posture - q) - POSTURE_KD[1] * v[act]
    return h, -KD * rate - KP * h - J[:, act] @ want


#: The robot's drives' peaks, each (`bemf.PEAK`), as `bemf.bounds` takes them.
_PEAK = np.asarray(bemf.PEAK, float)


def _shielded(body, near, s, ask, volts):
    """`shielded` where a near pair falls short: the QP on its evading joints (`_evaded`), every pair at once."""
    act, posture = body.act, np.asarray(ask['posture'], float)
    which = np.flatnonzero(near['d'] - FLOOR < INFLUENCE)
    J = near['J'][which]
    rate = J @ s['v']
    reach = braking(J, s['M'], act, s['v'][act], volts)
    closing = np.maximum(0.0, -rate)
    h = near['d'][which] - FLOOR[which] - np.minimum((closing * closing) / (2.0 * np.maximum(reach, 1e-9)), closing * HORIZON)
    want = POSTURE_KD[0] * (posture - s['q']) - POSTURE_KD[1] * s['v'][act]
    free = np.flatnonzero(EVADES[which].any(0))
    k, f = len(which), len(free)
    short = -KD * rate - KP * h - J[:, act] @ want
    # nothing short, nothing changes: the QP's least is its origin
    x, held = _evaded(s['M'], J, act, free, short) if (short > 0.0).any() else (np.zeros(f + k), ())
    change = np.zeros(len(act))
    change[free] = x[:f]
    return posture + change / POSTURE_KD[0], {'d': near['d'], 'h': h, 'which': which, 'spill': x[f:],
                                              'held': [int(which[r]) for r in held if r < k]}
