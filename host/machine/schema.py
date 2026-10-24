"""Her shield (proprioceptive_body_schema.lisp): her capsules' ECBF against each other and the room.

    posture, held = schema.shielded(body, d, s, ask, volts, at, boxes)  # her posture holding each row's ECBF
    boxes = schema.room(props, weights, floor, touched)   # the room's boxes, each one's patches it takes, gated

Her posture's ask (her law's lowest row, `wbc.POSTURE_KD`: the null space of her balance and her steps)
is held to h'' + k_d h' + k_p h >= 0 on each pair of her capsules (`machine.capsules`) within the text's
influence (`specialize-with-nullspace-limb-retraction`), h its clearance less its floor and its braking
distance v^2 / (2 a) - v its closing speed, a the most her drives' braking bounds (`machine.bemf`) can
open it by through her mass matrix -, on the pair's evading joints, least in her mass's metric, each
within its drive's peak; what they cannot is the shield's spill.

The room: each prop a box (`world.props()`, her eyes' stand-in), each of her arms' capsules against
each box near it by golden section along its axis (the distance to a convex set is convex along a
line), its floor the text's skin (`extrude-velocity-braking-cocoon`) less the approach window as far
as the box takes a patch of that capsule's (`gate-affordance-contact-permeability`): a box takes the
patches the text's affordance kernels give it by its top over her lower sole, its upward face's tilt,
area and length, a top within the sub band her soles' (`configure-foliated-sdf-bands`) - and her rows
schedule or her observer feels the patch, or her skin feels the capsule's segment. Its braking distance
within the cocoon's horizon. Her trunk's capsules evade by her balance bus (the text's spill-over),
her legs by her footholds: neither held here.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import bemf, kernel, qp
from machine.capsules import (CAPSULE_EVADES, EVADES, FLOOR, NAMES, P0, P1, PAIRS, RETRACTION, SEGMENT, laid, nearest,
                              posed, rows, _sided)
from machine.observer import MANIFOLDS
from machine.wbc import POSTURE_KD

_SHIELD = kernel.node('xform-universal-spatial-awareness', 'ecbf-shield')
#: The ECBF's gains: k_p 1/s^2, k_d 1/s.
KP, KD = _SHIELD['position-gain-kp'], _SHIELD['velocity-gain-kd']
#: A pair's clearance past its floor under which the shield holds it, m.
INFLUENCE = RETRACTION['activation-distance-m']
#: A shortfall's weight against her posture's change in her mass's metric, kg: what her evading joints
#: cannot meet within their peaks is left as the shield's spill.
SPILL_KG = 1e6
_COCOON = kernel.node('xform-universal-spatial-awareness', 'braking-cocoon')
#: The room's floor under a capsule, m; the braking distance's horizon, s.
MARGIN, HORIZON = _COCOON['base-margin-m'], _COCOON['lookahead-s']
#: The approach window a scheduled patch's floor is opened over, m.
WINDOW = kernel.node('xform-universal-spatial-awareness', 'ecbf-permeability')['normal-attenuation-ramp-m']
#: The text's capsules (less their sides) each of the registry's patches (`observer.MANIFOLDS`) is of her.
PATCHES = {'plantar-bilateral': ('foot-shell',), 'metatarsal-equinus-bilateral': ('foot-shell',),
           'posterior-femoral-bilateral': ('thigh-femoral',), 'ischial-bilateral': ('pelvic-girdle',),
           'trochanter-gluteal-lateral': ('pelvic-girdle', 'thigh-femoral'),
           'dorsal-supine-continuum': ('pelvic-girdle', 'lumbar-column', 'thoracic-ribcage', 'cranial-vault'),
           'ulnar-carpal-shove-bilateral': ('forearm-ulnar', 'hand-carpal'), 'palmar-carpal-bilateral': ('hand-carpal',)}
#: Each capsule's patches, a row a capsule over MANIFOLDS.
ON = np.array([[_sided(n)[0] in PATCHES[m] for m in MANIFOLDS] for n in NAMES], float)
_KERNELS = {str(k['affordance']).lstrip(':'): k
            for k in kernel.node('evaluate-situational-affordances', 'affordances')['morphological-kernels']}
#: The bands' refresh, s: her things read so often.
REFRESH_S = 1.0 / kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['refresh-rate-hz']
#: The sub band's top, m: a box's top within it her soles' step.
SUB_M = 1e-3 * kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['slabs'][0].args['z-window-mm'][1]
#: The patches each affordance takes (the text's kernels'; a side's lying, the bed's roll, the supine's).
TAKES = {'sit-on-fixture': ('ischial-bilateral', 'posterior-femoral-bilateral'),
         'lie-supine-on-fixture': ('dorsal-supine-continuum', 'trochanter-gluteal-lateral'),
         'recruit-brace-support': ('ulnar-carpal-shove-bilateral', 'palmar-carpal-bilateral'),
         'relocate-mobile-object': ('ulnar-carpal-shove-bilateral', 'palmar-carpal-bilateral'),
         'step-on': ('plantar-bilateral', 'metatarsal-equinus-bilateral')}
#: The capsules the room holds: her arms', each by joints of its own (CAPSULE_EVADES) - her trunk's
#: evade by her balance bus (the text's spill-over), which her shield does not drive; her legs' by her
#: footholds (`snap-foothold-to-plantar-gradient`).
LIMBS = CAPSULE_EVADES.any(1)
#: The golden-section steps of a capsule against a box: its bracket 0.618 of its last each, from the
#: longest capsule to a thousandth of the skin.
_GOLD = (5.0 ** 0.5 - 1.0) / 2.0
SECTIONS = int(np.ceil(np.log(1e-3 * MARGIN / np.sqrt(((P1 - P0) ** 2).sum(1)).max()) / np.log(_GOLD)))
#: The patches each affordance takes, a row an affordance over MANIFOLDS.
_AFFORDS = tuple(TAKES)
_TAKEN = np.array([[m in TAKES[a] for m in MANIFOLDS] for a in _AFFORDS], float)


def _window(k, key):
    lo, hi = _KERNELS[k][key]
    return 1e-3 * lo, 1e-3 * hi


def affords(centre, half, R, floor):
    """(P, MANIFOLDS) of boxes (centre (P, 3), half sizes, rotations (P, 3, 3)): each patch a box takes
    by the text's kernels - its top over `floor` (m), its upward face's tilt, area and length; a hand's
    where its height spans the kernel's window, a sole's where its top is within the sub band."""
    up = np.abs(R[:, 1, :])
    top, bottom = centre[:, 1] + (up * half).sum(1) - floor, centre[:, 1] - (up * half).sum(1) - floor
    face = np.argmax(up, 1)
    cos = up[np.arange(len(up)), face]
    rest = np.where(np.arange(3)[None, :] == face[:, None], np.nan, half)
    area, major, thin = 4.0 * np.nanprod(rest, 1), 2.0 * np.nanmax(rest, 1), 2.0 * half.min(1)

    def within(x, w):
        return (x >= w[0]) & (x <= w[1])

    def spans(w):
        return (bottom <= w[1]) & (top >= w[0])
    sit, lie = _KERNELS['sit-on-fixture'], _KERNELS['lie-supine-on-fixture']
    took = {'sit-on-fixture': (cos >= sit['normal-alignment-cos-min']) & within(top, _window('sit-on-fixture', 'elevation-window-z-mm'))
            & (area >= 1e-6 * sit['min-polygon-area-mm2']),
            'lie-supine-on-fixture': (cos >= lie['normal-alignment-cos-min']) & within(top, _window('lie-supine-on-fixture', 'elevation-window-z-mm'))
            & (major >= 1e-3 * lie['min-major-axis-length-mm']),
            'recruit-brace-support': spans(_window('recruit-brace-support', 'elevation-window-z-mm')),
            'relocate-mobile-object': spans(_window('relocate-mobile-object', 'grasp-height-window-z-mm'))
            & within(thin, _window('relocate-mobile-object', 'grasp-aperture-window-mm')),
            'step-on': top <= SUB_M}
    return np.minimum(1.0, np.stack([took[a] for a in _AFFORDS], 1).astype(float).reshape(-1, len(_AFFORDS)) @ _TAKEN)


def room(props, weights, floor, touched):
    """{centre, half, R, takes (P, MANIFOLDS), touched} of the room's boxes `props` (`world.props()`:
    kind, centre, half sizes, rotation), each patch it takes as far as `weights` (MANIFOLDS: her rows'
    schedule or her observer's feel, 0 to 1) are on, over her lower sole's height `floor`; `touched`
    each capsule's segment her skin feels (`felt`): a live patch, open to every box."""
    if _LAID.get('of') is not props:
        boxes = [p for p in props if len(p) >= 4 and len(p[1]) == 3 and np.size(p[2]) == 3 and np.size(p[3]) == 9]
        _LAID.update(of=props, centre=np.array([p[1] for p in boxes], float).reshape(-1, 3),
                     half=np.array([p[2] for p in boxes], float).reshape(-1, 3),
                     R=np.array([p[3] for p in boxes], float).reshape(-1, 3, 3))
    centre, half, R = _LAID['centre'], _LAID['half'], _LAID['R']
    return {'centre': centre, 'half': half, 'R': R, 'takes': affords(centre, half, R, floor) * np.asarray(weights)[None, :],
            'touched': np.asarray(touched, float)}


#: The last props laid as boxes: her things change at REFRESH_S, her law passes oftener.
_LAID: dict = {}


def felt(touch):
    """(capsules,) whether her skin feels each capsule's segment: its touch (`edge.TOUCH`, (SKIN, 3)
    N) past `edge.TOUCHED_N`."""
    from machine import edge
    force = np.sqrt((np.asarray(touch).reshape(-1, 3) ** 2).sum(1))
    return np.array([force[edge.SKIN.index(seg)] >= edge.TOUCHED_N for seg in SEGMENT])


def _to_box(p, centre, half, R):
    """Each point `p` (k, 3)'s distance to its box and the box's nearest point, world."""
    local = np.einsum('kji,kj->ki', R, p - centre)
    near = np.clip(local, -half, half)
    out = local - near
    return np.sqrt((out * out).sum(1)), centre + np.einsum('kij,kj->ki', R, near)


def boxed(caps, laid, pairs):
    """{d, n, a, s} of each (capsule, box) of `pairs` (k, 2): its clearance, its normal from the box to
    its axis's nearest point, that point and its share along the axis."""
    i, j = pairs[:, 0], pairs[:, 1]
    p0, d1 = caps['p0'][i], caps['p1'][i] - caps['p0'][i]
    centre, half, R = laid['centre'][j], laid['half'][j], laid['R'][j]
    lo, hi = np.zeros(len(i)), np.ones(len(i))
    for _ in range(SECTIONS * bool(len(i))):
        c, e = hi - _GOLD * (hi - lo), lo + _GOLD * (hi - lo)
        left = _to_box(p0 + c[:, None] * d1, centre, half, R)[0] <= _to_box(p0 + e[:, None] * d1, centre, half, R)[0]
        hi, lo = np.where(left, e, hi), np.where(left, lo, c)
    s = 0.5 * (lo + hi)
    a = p0 + s[:, None] * d1
    dist, on = _to_box(a, centre, half, R)
    away = np.where(dist[:, None] > 0.0, a - on, a - centre)
    n = away / np.maximum(np.sqrt((away * away).sum(1)), np.finfo(float).eps)[:, None]
    r = caps['r0'][i] + s * (caps['r1'][i] - caps['r0'][i])
    return {'d': dist - r, 'n': n, 'a': a, 's': s}


def weighed(told, felt, weight):
    """(MANIFOLDS,) each patch's schedule or feel, 0 to 1: her rows' weights `told` (`ortho.MANIFOLDS`'s,
    by their names' heads) or her observer's normal forces `felt` (N, MANIFOLDS) over its comparator's
    engaging share of her `weight` (`observer.ENGAGE`), the more."""
    from machine import observer, ortho
    heads = [m.split('-')[0] for m in MANIFOLDS]
    rows = np.array([told[list(ortho.MANIFOLDS).index(h)] if h in ortho.MANIFOLDS else 0.0 for h in heads])
    return np.clip(np.maximum(rows, np.asarray(felt) / (observer.ENGAGE * weight)), 0.0, 1.0)


def near_room(caps, laid):
    """(k, 2) of each of LIMBS' capsules and each box within INFLUENCE and the skin of it: the box's
    distance to the capsule's middle less its reach from there."""
    arms = np.flatnonzero(LIMBS)
    mid = 0.5 * (caps['p0'][arms] + caps['p1'][arms])
    reach = 0.5 * np.sqrt(((caps['p1'][arms] - caps['p0'][arms]) ** 2).sum(1)) + np.maximum(caps['r0'][arms], caps['r1'][arms])
    n, m = len(arms), len(laid['centre'])
    gap = _to_box(np.repeat(mid, m, 0), np.tile(laid['centre'], (n, 1)), np.tile(laid['half'], (n, 1)),
                  np.tile(laid['R'], (n, 1, 1)))[0].reshape(n, m) - reach[:, None]
    near = np.argwhere(gap < INFLUENCE + MARGIN)
    return np.stack([arms[near[:, 0]], near[:, 1]], 1).reshape(-1, 2)


def braking(J, M, act, dq, volts):
    """The most each row of `J` can be opened by, m/s^2: her drives' bounds at their speeds `dq` on
    `volts` (`bemf.bounds`) through her mass matrix `M`, each drive at the bound that opens it."""
    lo, hi = bemf.bounds(dq, volts, bemf.PEAK)
    C = np.linalg.solve(M, J.T)[act].T
    return np.maximum(C * lo, C * hi).sum(1)


def room_rows(body, d, got, pairs, at, v):
    """(J, d') of capsule-box pairs `pairs` (`boxed`'s `got`): each one's clearance's Jacobian (k, n)
    along its normal at its capsule's axis point, the box still; d' its rate at her velocity `v`."""
    m, mj, n = body.m, body.mj, body.n
    jp = np.zeros((3, m.nv))
    J = np.zeros((len(pairs), n))
    for k, (c, _box) in enumerate(pairs):
        mj.mj_jac(m, d, jp, None, got['a'][k], int(at[c]))
        J[k] = got['n'][k] @ jp[:, :n]
    return J, J @ v


def shielded(body, d, s, ask, volts, at, boxes):
    """(her posture's ask, {d, h, which, held, spill, room}): `ask['posture']` changed on the text's
    evading joints of the rows near their floors - her pairs within INFLUENCE (EVADES), her capsules
    against the room's `boxes` (`room`) within it (CAPSULE_EVADES) -, least in her mass's metric, each
    joint within its drive's peak over its own inertia, so that each row keeps h'' + KD h' + KP h >= 0
    - h its clearance less its floor and its braking distance -, what it cannot its spill (m/s^2); `d`
    the shield's data of her law's model posed at `s`, its capsules' segments `at` (`ids`); each pair's
    clearance, the near ones' h, the pairs it held; the room's rows {pairs (capsule, box), d, n, lam}."""
    act, posture = body.act, np.asarray(ask['posture'], float)
    caps = laid(posed(body, d, s), at)
    near = nearest(caps, PAIRS)
    which = np.flatnonzero(near['d'] - FLOOR < INFLUENCE)
    J1, rate1 = rows(body, d, near, PAIRS, which, at, s['v'])
    close = near_room(caps, boxes)
    got = boxed(caps, boxes, close)
    open_ = np.maximum((ON[close[:, 0]] * boxes['takes'][close[:, 1]]).max(1, initial=0.0), boxes['touched'][close[:, 0]])
    floor2 = MARGIN - WINDOW * open_
    keep = np.flatnonzero(got['d'] - floor2 < INFLUENCE)
    close, floor2 = close[keep], floor2[keep]
    got = {k: v[keep] for k, v in got.items()}
    J2, rate2 = room_rows(body, d, got, close, at, s['v'])
    J, rate = np.vstack([J1, J2]), np.concatenate([rate1, rate2])
    clear = np.concatenate([near['d'][which] - FLOOR[which], got['d'] - floor2])
    reach = braking(J, s['M'], act, s['v'][act], volts)
    closing = np.maximum(0.0, -rate)
    h = clear - np.minimum(closing ** 2 / (2.0 * np.maximum(reach, 1e-9)), closing * HORIZON)
    want = POSTURE_KD[0] * (posture - s['q']) - POSTURE_KD[1] * s['v'][act]
    evades = np.vstack([EVADES[which], CAPSULE_EVADES[close[:, 0]]]).reshape(-1, len(act))
    free = np.flatnonzero(evades.any(0))
    Jf, k, f = J[:, act[free]], len(rate), len(free)
    short = -KD * rate - KP * h - J[:, act] @ want
    Mf = s['M'][np.ix_(act[free], act[free])]
    top = np.asarray(bemf.PEAK)[free] / np.diag(Mf)
    # nothing short, nothing changes: the QP's least is its origin
    H = np.block([[Mf, np.zeros((f, k))], [np.zeros((k, f)), SPILL_KG * np.eye(k)]])
    A = np.block([[-Jf, -np.eye(k)], [np.zeros((k, f)), -np.eye(k)], [np.eye(f), np.zeros((f, k))],
                  [-np.eye(f), np.zeros((f, k))]])
    x, held, u = (qp.solve(H, np.zeros(f + k), A, np.concatenate([-short, np.zeros(k), top, top]))
                  if (short > 0.0).any() else (np.zeros(f + k), (), np.zeros(0)))
    x = np.zeros(f + k) if x is None else x
    lam = np.zeros(k)
    lam[[r for r in held if r < k]] = [m for r, m in zip(held, u) if r < k]
    change = np.zeros(len(act))
    change[free] = x[:f]
    n1 = len(which)
    return posture + change / POSTURE_KD[0], {
        'd': near['d'], 'h': h[:n1], 'which': which, 'spill': x[f:], 'held': [int(which[r]) for r in held if r < n1],
        'room': {'pairs': close, 'd': got['d'], 'n': got['n'], 'lam': lam[n1:], 'h': h[n1:]}}
