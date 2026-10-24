"""Her shield (proprioceptive_body_schema.lisp): capsules' ECBF on each other; the room's boxes.

    posture, held = schema.shielded(body, d, s, ask, volts, at)   # her posture holding each near pair's ECBF
    boxes = schema.room(props)                                    # the room's boxes, laid once a refresh

Her posture's ask (her law's lowest row, `wbc.POSTURE_KD`: the null space of her balance and her steps)
is held to h'' + k_d h' + k_p h >= 0 on each pair of her capsules (`machine.capsules`) within the text's
influence (`specialize-with-nullspace-limb-retraction`), h its clearance less its floor and its braking
distance v^2 / (2 a) within the cocoon's horizon - v its closing speed, a the most her drives' braking
bounds (`machine.bemf`) can open it by through her mass matrix -, on the pair's evading joints, least in
her mass's metric, each within its drive's peak; what they cannot is the shield's spill.

The room: each prop a box (`world.props()` through her map, `machine.registry`), her wayfinder's gauge's
(`machine.wayfinder`). Her arms' ECBF against them through their elbows alone (the text's limb synergy
she has) spared no strike at a doorway's jamb and halved her shoulder's yield after it: not held; the
text's holds her trunk by her balance bus and her legs by her footholds.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import bemf, kernel, qp
from machine.capsules import EVADES, FLOOR, PAIRS, RETRACTION, laid, nearest, posed, rows
from machine.wbc import POSTURE_KD

_SHIELD = kernel.node('xform-universal-spatial-awareness', 'ecbf-shield')
#: The ECBF's gains: k_p 1/s^2, k_d 1/s.
KP, KD = _SHIELD['position-gain-kp'], _SHIELD['velocity-gain-kd']
#: A pair's clearance past its floor under which the shield holds it, m.
INFLUENCE = RETRACTION['activation-distance-m']
#: A shortfall's weight against her posture's change in her mass's metric, kg: what her evading joints
#: cannot meet within their peaks is left as the shield's spill.
SPILL_KG = 1e6
#: The braking distance's horizon, s (`extrude-velocity-braking-cocoon`).
HORIZON = kernel.node('xform-universal-spatial-awareness', 'braking-cocoon')['lookahead-s']
#: The bands' refresh, s: her things read so often.
REFRESH_S = 1.0 / kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['refresh-rate-hz']


def room(props):
    """{centre (P, 3), half (P, 3), R (P, 3, 3)} of the room's boxes `props` (`world.props()`'s: kind,
    centre, half sizes, rotation), laid once per list: her things change at REFRESH_S, her law passes
    oftener."""
    if _LAID.get('of') is not props:
        boxes = [p for p in props if len(p) >= 4 and len(p[1]) == 3 and np.size(p[2]) == 3 and np.size(p[3]) == 9]
        _LAID.update(of=props, centre=np.array([p[1] for p in boxes], float).reshape(-1, 3),
                     half=np.array([p[2] for p in boxes], float).reshape(-1, 3),
                     R=np.array([p[3] for p in boxes], float).reshape(-1, 3, 3))
    return {k: _LAID[k] for k in ('centre', 'half', 'R')}


#: The last props laid as boxes.
_LAID: dict = {}


def braking(J, M, act, dq, volts):
    """The most each row of `J` can be opened by, m/s^2: her drives' bounds at their speeds `dq` on
    `volts` (`bemf.bounds`) through her mass matrix `M`, each drive at the bound that opens it."""
    lo, hi = bemf.bounds(dq, volts, bemf.PEAK)
    C = np.linalg.solve(M, J.T)[act].T
    return np.maximum(C * lo, C * hi).sum(1)


def shielded(body, d, s, ask, volts, at):
    """(her posture's ask, {d, h, which, held, spill}): `ask['posture']` changed on the text's evading
    joints of the pairs within INFLUENCE of their floors (EVADES), least in her mass's metric, each
    joint within its drive's peak over its own inertia, so that each pair keeps h'' + KD h' + KP h >= 0
    - h its clearance less its floor and its braking distance -, what it cannot its spill (m/s^2); `d`
    the shield's data of her law's model posed at `s`, its capsules' segments `at` (`capsules.ids`);
    each pair's clearance, the near ones' h, the pairs it held."""
    act, posture = body.act, np.asarray(ask['posture'], float)
    near = nearest(laid(posed(body, d, s), at), PAIRS)
    which = np.flatnonzero(near['d'] - FLOOR < INFLUENCE)
    J, rate = rows(body, d, near, PAIRS, which, at, s['v'])
    reach = braking(J, s['M'], act, s['v'][act], volts)
    closing = np.maximum(0.0, -rate)
    h = near['d'][which] - FLOOR[which] - np.minimum(closing ** 2 / (2.0 * np.maximum(reach, 1e-9)), closing * HORIZON)
    want = POSTURE_KD[0] * (posture - s['q']) - POSTURE_KD[1] * s['v'][act]
    free = np.flatnonzero(EVADES[which].any(0))
    Jf, k, f = J[:, act[free]], len(which), len(free)
    short = -KD * rate - KP * h - J[:, act] @ want
    Mf = s['M'][np.ix_(act[free], act[free])]
    top = np.asarray(bemf.PEAK)[free] / np.diag(Mf)
    # nothing short, nothing changes: the QP's least is its origin
    H = np.block([[Mf, np.zeros((f, k))], [np.zeros((k, f)), SPILL_KG * np.eye(k)]])
    A = np.block([[-Jf, -np.eye(k)], [np.zeros((k, f)), -np.eye(k)], [np.eye(f), np.zeros((f, k))],
                  [-np.eye(f), np.zeros((f, k))]])
    x, held, _u = (qp.solve(H, np.zeros(f + k), A, np.concatenate([-short, np.zeros(k), top, top]))
                   if (short > 0.0).any() else (np.zeros(f + k), (), None))
    x = np.zeros(f + k) if x is None else x
    change = np.zeros(len(act))
    change[free] = x[:f]
    return posture + change / POSTURE_KD[0], {'d': near['d'], 'h': h, 'which': which, 'spill': x[f:],
                                              'held': [int(which[r]) for r in held if r < k]}
