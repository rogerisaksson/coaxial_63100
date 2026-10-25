"""The kernel's capsules on her skin (proprioceptive_body_schema.lisp): its 18, their clearances.

    caps = capsules.laid(body.d, at)         # each capsule's ends and radii in the world as `body.d` is posed
    near = capsules.nearest(caps, PAIRS)     # each pair's clearance, its normal and points
    d = capsules.smooth(near['d'])           # the text's log-sum-exp at ALPHA, under the least
    J, rate = capsules.rows(body, d, near, PAIRS, which, at, v)   # each pair's clearance Jacobian, its rate
    own = capsules.Own(body)                 # her law's body on the shield's own data, laid once
    near = capsules.felt(own, s, at)         # every pair's clearance and its Jacobian, `own` posed at `s`

The text's 18 capsules by its names, frames and 13 self-collision pairs, each on her segment and her
skin there (`figure.BODY`: the first of its capsules there to the last, tapering; her sole and knee of
her figure's numbers): its own are another body's - its shank 0.41 m long and 68 mm round where hers
0.38 and 50, its thighs 95 mm round 145 mm off her middle, through each other at her hips. A capsule
tapers from r0 at p0 to r1 at p1; two capsules' clearance is their axes' nearest points' distance
less each's radius there. Each pair's floor is its barrier's (`configure-self-collision-barriers`, 0 a
pair it names in no group), its evading joints its barrier's DOF on its sides; each capsule's against
Her shield on them: `machine.schema`.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import bemf, kernel
from machine.figure import ANKLE_H, BALL, BODY, HEEL, SOLE_HALF

_LAID = kernel.read()['anatomical-capsule-schema']
#: The text's capsules, its order.
NAMES = tuple(_LAID['capsules'])
#: A side's suffix in the text's names, its prefix in her segments'.
SIDES = {'l': 'left_', 'r': 'right_'}
#: Her frame's axes of the text's: her lateral, up and forward its y, z and x.
AXES = (1, 2, 0)


def _skin(part, ks):
    """(p0, p1, r0, r1) of her skin's capsules `ks` on `part` (`figure.BODY`'s order there) as one."""
    caps = [c for c in BODY if c[0] == part]
    a, b = caps[ks[0]], caps[ks[-1]]
    return a[2], b[3], a[1], b[1]


def _sole():
    """Her sole as a capsule: as wide as it, from her heel to her ball, its underside her sole."""
    y = SOLE_HALF - ANKLE_H
    return (0.0, y, SOLE_HALF - HEEL), (0.0, y, BALL - SOLE_HALF), SOLE_HALF, SOLE_HALF


def _knee():
    """Her knee where the text lays it on its frame, her shank's top's round."""
    c, r = _LAID['capsules']['knee-patellar-l'], _skin('shank', (0,))[2]
    return tuple(c['p0'][k] for k in AXES), tuple(c['p1'][k] for k in AXES), r, r


#: Each of the text's capsules (its name less its side) on her: her segment, her skin there.
SKIN = {'foot-shell': ('foot', _sole()), 'shank-crural': ('shank', _skin('shank', (0, 1))),
        'thigh-femoral': ('thigh', _skin('thigh', (0, 1))), 'pelvic-girdle': ('pelvis', _skin('pelvis', (2,))),
        'lumbar-column': ('torso', _skin('torso', (0,))), 'thoracic-ribcage': ('torso', _skin('torso', (2,))),
        'cranial-vault': ('head', _skin('head', (0,))), 'upper-arm-humeral': ('upper_arm', _skin('upper_arm', (0,))),
        'forearm-ulnar': ('forearm', _skin('forearm', (0,))), 'hand-carpal': ('hand', _skin('hand', (0,))),
        'knee-patellar': ('shank', _knee())}


def _sided(name):
    """(the text's capsule less its side, her segment's side prefix)."""
    base, side = name.rsplit('-', 1)
    return (base, SIDES[side]) if side in SIDES else (name, '')


_BOUND = [(prefix + SKIN[base][0], SKIN[base][1]) for base, prefix in map(_sided, NAMES)]
#: Each capsule's segment; its ends in that segment's frame (n, 3), m; its radii there, m.
SEGMENT = tuple(seg for seg, _c in _BOUND)
P0, P1 = (np.array([c[k] for _s, c in _BOUND], float) for k in (0, 1))
R0, R1 = (np.array([c[k] for _s, c in _BOUND], float) for k in (2, 3))
#: The text's self-collision pairs, as capsule indices.
PAIRS = np.array([[NAMES.index(str(n).lstrip(':')) for n in pair] for pair in _LAID['self-collision-pairs']])
#: Every pair, by its index (`felt`).
EVERY = np.arange(len(PAIRS))
_GROUPS = kernel.node('specialize-with-self-collision-manifold', 'self-collision').values()
#: Each pair's floor, m: the largest of the barriers' groups holding both its capsules.
FLOOR = np.array([max([g['min-clearance-m'] for g in _GROUPS
                       if {':' + NAMES[a], ':' + NAMES[b]} <= set(g['capsules'])], default=0.0) for a, b in PAIRS])
#: The log-sum-exp's sharpness, 1/m.
ALPHA = kernel.node('xform-universal-spatial-awareness', 'clearance-field')['aggregation-kernel'].args['alpha']
#: The text's limb retraction: its influence (`machine.schema`).
RETRACTION = kernel.node('specialize-with-nullspace-limb-retraction', 'limb-retraction')
#: The text's evasion DOFs (its barriers' `evasion-dof`, its synergies' `repulsion-mapping`) as her
#: joints' kinds: her shoulder only flexes, no glenohumeral abduction or adduction - a row only those
#: evade by is spilled whole.
DOFS = {'swing-circumduction-bow': ('hip_roll',), 'coxofemoral-external-rotation': ('hip_yaw',),
        'glenohumeral-abduction': (), 'glenohumeral-adduction': (), 'cubital-flexion': ('elbow',),
        'thoracic-axial-yaw': ('waist',), 'l4-l5-lumbar-flexion': ('spine',), 'iliac-coronal-shift': ('spine_roll',),
        'c7-t12-thoracic-flexion': ('spine',), 'c1-c2-atlas-pitch-comp': ('neck',), 'sacroiliac-z-duck': ('knee',)}


def _joints(dofs, sides):
    """Her driven joints (`bemf.DRIVEN` indices) of the text's DOFs `dofs` on `sides` (prefixes)."""
    kinds = {k for d in dofs for k in DOFS[str(d).lstrip(':')]}
    return {bemf.DRIVEN.index(k if k in bemf.DRIVEN else side + k) for k in kinds for side in sides}


def _pair_evades(a, b):
    """Her joints a pair of capsules evades by: its barriers' DOFs (`configure-self-collision-barriers`,
    each group holding either), on the sides of its sided capsules, on both where neither is."""
    sides = {_sided(n)[1] for n in (a, b)} - {''} or set(SIDES.values())
    return _joints([g['evasion-dof'] for g in _GROUPS if {':' + a, ':' + b} & set(g['capsules'])], sides)


#: Each pair's evading joints, a row a pair over her driven joints.
EVADES = np.array([[i in _pair_evades(NAMES[a], NAMES[b]) for i in range(len(bemf.DRIVEN))] for a, b in PAIRS])


def ids(m):
    """Each capsule's segment's body id in model `m`."""
    return np.array([m.body(seg).id for seg in SEGMENT])


def laid(d, at):
    """{p0, p1 (n, 3), r0, r1 (n,), ids}: each capsule in the world on her model's data `d` as
    posed, its segments' ids `at` (`ids`)."""
    R, o = d.xmat[at].reshape(-1, 3, 3), d.xpos[at]
    return {'p0': o + np.einsum('nij,nj->ni', R, P0), 'p1': o + np.einsum('nij,nj->ni', R, P1),
            'r0': R0, 'r1': R1, 'ids': at}


def nearest(caps, pairs):
    """{d, n, a, b, s, t} of each pair (k, 2) of capsules `caps`: its clearance (m, negative through),
    its normal from its second to its first, its nearest points on their axes and their shares
    along them (Ericson's clamped closest points of two segments)."""
    i, j = pairs[:, 0], pairs[:, 1]
    p0, q0 = caps['p0'][i], caps['p0'][j]
    d1, d2, r = caps['p1'][i] - p0, caps['p1'][j] - q0, p0 - q0
    a, e, f = (d1 * d1).sum(1), (d2 * d2).sum(1), (d2 * r).sum(1)
    c, bb = (d1 * r).sum(1), (d1 * d2).sum(1)
    tiny = np.finfo(float).eps
    den = a * e - bb * bb
    sa = np.maximum(a, tiny)
    s = np.clip(np.where(den > tiny * a * e, (bb * f - c * e) / np.maximum(den, tiny), -c / sa), 0.0, 1.0)
    t = (bb * s + f) / np.maximum(e, tiny)
    s = np.where(t < 0.0, np.clip(-c / sa, 0.0, 1.0), np.where(t > 1.0, np.clip((bb - c) / sa, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    pa, pb = p0 + s[:, None] * d1, q0 + t[:, None] * d2
    gap = pa - pb
    dist = np.sqrt((gap * gap).sum(1))
    ra = caps['r0'][i] + s * (caps['r1'][i] - caps['r0'][i])
    rb = caps['r0'][j] + t * (caps['r1'][j] - caps['r0'][j])
    return {'d': dist - ra - rb, 'n': gap / np.maximum(dist, tiny)[:, None], 'a': pa, 'b': pb, 's': s, 't': t}


def smooth(d, alpha=ALPHA):
    """The text's log-sum-exp of clearances `d` at `alpha`: -log(sum exp(-alpha d)) / alpha, under
    their least by at most log(len d) / alpha."""
    least = float(np.min(d))
    return least - float(np.log(np.exp(-alpha * (np.asarray(d) - least)).sum())) / alpha


def rows(body, d, near, pairs, which, at, v):
    """(J, d') of pairs `which`: each one's clearance's Jacobian (k, n) along its normal, its points'
    segments' (`at`), on her law's `body`'s model as `d` is posed; d' its rate at her velocity `v`."""
    m, mj, n = body.m, body.mj, body.n
    jp, jq = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    J = np.zeros((len(which), n))
    for k, w in enumerate(which):
        mj.mj_jac(m, d, jp, None, near['a'][w], int(at[pairs[w, 0]]))
        mj.mj_jac(m, d, jq, None, near['b'][w], int(at[pairs[w, 1]]))
        J[k] = near['n'][w] @ (jp - jq)[:, :n]
    return J, J @ v


def posed(body, d, s):
    """`d`, data of her law's `body`'s model of the shield's own, posed at the pass's sensed state
    `s` (its `qpos` and `v`): its frames and its Jacobians' terms - her law's own data the pass's
    steps pose as they need."""
    m, mj = body.m, body.mj
    d.qpos[:len(s['qpos'])], d.qvel[:body.n] = s['qpos'], s['v']
    mj.mj_kinematics(m, d)
    mj.mj_comPos(m, d)
    return d


class Own:

    """Her law's `body` on data of its own, `d` (the shield's): every other attribute read off hers as
    it is - traced as her body (`tools.cores.bodied.on_body`), what it reads of hers held the same."""

    def __init__(self, body):
        self.body, self.d = body, body.mj.MjData(body.m)

    def __getattr__(self, name):
        """Her body's `name`; a protocol's dunder probe (copy's, before its state) AttributeError."""
        if name.startswith('__'):
            raise AttributeError(name)
        return getattr(self.body, name)


def felt(body, s, at):
    """{d, J}: every pair's clearance and its clearance's Jacobian (`nearest`, `rows`), `body` (`Own`)
    posed at `s` on its own data (`posed`), its capsules' segments `at` (`ids`) - shapes fixed, her
    loop's compiled (`machine.motion.laid`)."""
    d = posed(body, body.d, s)
    near = nearest(laid(d, at), PAIRS)
    return {'d': near['d'], 'J': rows(body, d, near, PAIRS, EVERY, at, s['v'])[0]}
