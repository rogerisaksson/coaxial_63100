"""Her generalized momentum observer: the external force on her, blind.

    obs = observer.laid(body)              # her patches' points laid once, the observer at rest
    observer.felt(obs, y, dt)              # each drive pass: her drives' torques taken (`edge.TAU`)
    observer.step(obs, s, qpos, dt)        # each law pass on her law's sense and estimate: r, her patches
    obs['r'], obs['normal']                # the external force on her dofs; each manifold's, N (MANIFOLDS)

The residual of her momentum over a law pass, M (v - v') / dt + h - S'tau - her pelvis's linear
acceleration her IMU's, tau her drives' mean over it and their friction -, low-passed at the kernel's bandwidth
(`configure-momentum-observer`), is J'f: the external force on her and what her body does to itself. Split by least
squares over pushes alone (wbc_gmo.c, Lawson-Hanson), so it always sums to her whole body's balance and nothing pulls:
- her registry's points (`anatomical-manifold-registry`): a sole's four corners in her law's pyramid (`wbc.MU`), her
  seat, thighs and palms pressed up her IMU's vertical, the rest along their own normals;
- a joint on its stop pushed off it; two of her capsules touching pushed apart;
- a joint's row within its friction's band read as friction (`collapse-static-equilibrium-innovation`);
- a point off her soles her comparator has not felt joining only where her soles cannot bear her;
- her hands on a floor bearing her in the same pyramid;
- what none of it can tell, her soles their last pass's and the rest gravity's prior over her points
  (`regularize-manifold-simplex-with-priors`): upright, low and still bears, her weight's share each; a point
  moving past her grip's slip neither shares nor is free (`gate-supports-by-stillness`), its speed the least of
  its own and against her text's slide (`slide`, `motion`): the floor scoot's stroke slid her seat at 0.62 m/s,
  her seat weighed 0 under its told 0.41 and her legs stood her up off it.
The floor scoot's squat, her knees on their stops (17-25 N m) and her thighs on her belly (330-760 N): a sole a free
wrench read 72-143 N on her butt with none there and her soles pulling to -1 365 N. Blind: no skin, no load cell.
"""
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, kernel, ortho, touch
from machine.figure import BALL, BODY, CONTACTS, FIST_AT, FIST_R, HEEL, JOINTS, PADS, pad
from machine.wbc import CORNERS, MU, G
from tools.cores.model import rotation
from tools.cores.wbc import address

#: Her manifolds, the registry's; her forefeet's, of her soles' points.
MANIFOLDS = tuple(kernel.read()['anatomical-manifold-registry'])
EQUINUS = MANIFOLDS.index('metatarsal-equinus-bilateral')
#: Her drives' friction its loss (`drives.backdrive`, as her model has it) times tanh(v / SLIP_RAD_S)
#: (the text's Stribeck). Measured on MuJoCo's: 0.42 of the loss at 3-10 mrad/s, 0.65 at 10-30, 0.85 at
#: 30-100, 0.97 past; unmodelled, her soles' 1.8 m s^-2 walking read 19.5 N light and her seat 44 N
#: borne, modelled 3.3 and 27.
SLIP_RAD_S = 0.03
_UP, _BACK = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
#: Her soles alone bearing her, as her rows' manifolds weigh (`ortho.MANIFOLDS`).
SOLES = np.eye(len(ortho.MANIFOLDS))[0]
#: The least positive double: a divisor where a share may be none.
TINY = np.finfo(float).tiny


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
#: its segment, the point, its outward normal, in the segment's frame. Each sole's corners
#: (`wbc.CORNERS`); the toes' underside; each buttock's; each thigh's
#: back where the seat's patch ends ahead of the buttocks and the back's begins below the sacrum -
#: its own, and sat pressed up her pelvis (the seat's), lain pressed along her back (the back's) -;
#: each hip's side, pressed in; the sacrum, the lumbar and thoracic back and the skull's, pressed
#: forward; each palm's heel, pressed up its arm, a brace's or a shove's; each forearm's front by its
#: wrist, pressed back, a shove's.
_SEAT, _LAIN = ('ischial-bilateral', 'pelvis', _UP), ('dorsal-supine-continuum', 'pelvis', _BACK)
POINTS = tuple((((m, seg if frame is None else frame, n if axis is None else axis),) if isinstance(m, str) else m,
                seg, p, n) for m, frame, axis, seg, p, n in (
    *(('plantar-bilateral', None, None, side + '_foot', tuple(float(v) for v in c), _UP)
      for side in ('left', 'right') for c in CORNERS),
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
#: Her soles' segments, their corners pressing in her law's pyramid. Her other points press along one direction: in
#: a pyramid her butt's and thighs' friction read her chair's sit 0 N on her soles bearing 110-130, her toes' her
#: forefeet 142 N off walking.
SOLED = ('left_foot', 'right_foot')


def _pyramid(n):
    """Her law's pyramid's four edges about the normal `n`: n and MU along its two tangents either way."""
    n = np.asarray(n, float)
    t1 = np.cross(n, (1.0, 0.0, 0.0) if abs(n[0]) < 0.5 else (0.0, 1.0, 0.0))
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    return tuple(tuple(float(v) for v in n + MU * (a * t1 + b * t2)) for a in (-1.0, 1.0) for b in (-1.0, 1.0))


PYRAMID = _pyramid(_UP)
#: Gravity's prior where her rows cannot tell (the kernel's text): its tie-break, of her split's largest curvature;
#: a point's pushes' cosine with the vertical in it; its height over her lowest point in it, m. Tied to none, her
#: straight legs read 17 N on her butt standing and her hips' sides squeezed 231-899 N.
_PRIOR = 'regularize-manifold-simplex-with-priors'
FLOOR, UPRIGHT_COS, REACH_M = (kernel.said(_PRIOR, k) for k in ('min-prior-floor-mg-ratio', 'gravity-alignment-beta',
                                                                'height-reach-m'))
#: A support stands still: a point moving past this, m/s, takes no prior share and is not free (the kernel's
#: `gate-supports-by-stillness`): a hand she lifts or a sole in swing does not bear her.
STILL_M_S = kernel.said('gate-supports-by-stillness', 'support-slip-mps')
#: The manifolds whose points bear on a support under her, pressed up the world's vertical (her IMU's), not their
#: segment's: her palms' along her arm read 0 N of their 120-158 sat on the floor leaning back.
UPRIGHT = ('ischial-bilateral', 'posterior-femoral-bilateral', 'palmar-carpal-bilateral')
#: Her hands on a floor bear her as her soles do, in her law's pyramid up the vertical: pressing up alone, the floor
#: scoot's strokes read their 171-248 N on her seat, its 0 N.
HANDED = ('palmar-carpal-bilateral',)
#: Each point's pushes: its pyramid's edges, else its normal.
EDGES = tuple(PYRAMID if seg in SOLED or sh[0][0] in HANDED else (_UP,) if sh[0][0] in UPRIGHT else (n,)
              for sh, seg, _p, n in POINTS)


#: Her rows' manifolds (`ortho.MANIFOLDS`) among the observer's; the text's comparators on each,
#: of her weight (`latch-hysteretic-contact`): engaged at, released under, held below that long,
#: none released before so long engaged; its weights' slews up and down, a second
#: (`bind-wrench-driven-homotopy`).
ROWS = np.array([[m.split('-')[0] for m in MANIFOLDS].index(m) for m in ortho.MANIFOLDS])
_LATCH = kernel.node('latch-hysteretic-contact', 'contact-comparators')
ENGAGE, RELEASE = _LATCH['schmitt-high-engage-fz'], _LATCH['schmitt-low-release-fz']
DWELL_S, BLANK_S = 1e-3 * _LATCH['min-state-dwell-ms'], 1e-3 * _LATCH['impact-blanking-window-ms']
UP, DOWN = (kernel.said('bind-wrench-driven-homotopy', k) for k in ('slew-rate-up-per-s', 'slew-rate-down-per-s'))


def said(told):
    """`told` (`latent.MANIFOLDED`) as her law takes it: each manifold's share of what is told; nothing told
    (her rest), her soles alone."""
    total = told.sum()
    return np.where(total > 0.0, told / np.maximum(total, TINY), SOLES)


@functools.cache
def parts(m):
    """Each manifold's bodies of `m` (ROWS' order): its points' segments (POINTS), a whole arm where one of its
    segments is among them (`touch.ARMS`), and their children: lain back onto her forearm, her hands told, she was
    called fallen."""
    found = [{m.body(seg).id for shares, seg, _p, _n in POINTS if MANIFOLDS[r] in {s[0] for s in shares}} for r in ROWS]
    arms = [{m.body(edge.SKIN[i]).id for i in arm} for arm in touch.ARMS]
    found = [ids.union(*(arm for arm in arms if arm & ids)) for ids in found]
    return [ids | {b for b in range(m.nbody) if m.body_parentid[b] in ids} for ids in found]


def unborne(m, on, down):
    """[`down`'s bodies] less those of the manifolds `on` (ROWS', a bool each) bear (`parts`)."""
    return list(down - set().union(*(p for o, p in zip(on, parts(m)) if o)))


def weighed(obs, told, weight, dt):
    """(her rows' manifolds,) what bears her as her law takes it, `dt` s on: each manifold her
    rows tell weighed as told where her comparator (ENGAGE, RELEASE, DWELL_S, BLANK_S) feels it -
    its normal force of her `weight` -, gated up at UP and down at DOWN a second; what a manifold
    not felt was told, her soles'. Nothing told (her rest), her soles alone; told, they bear her as
    told; a seat pulled away weighs nothing at once."""
    told = said(told)
    n = obs['normal'][ROWS] / weight
    latched = obs['latched']
    on = (1.0 - latched) * (n >= ENGAGE)
    under = latched * (n < RELEASE)
    obs['below'] = under * (obs['below'] + dt)
    obs['since'] = (1.0 - on) * (obs['since'] + dt)
    obs['latched'] = latched + on - under * (obs['since'] >= BLANK_S) * (obs['below'] >= DWELL_S)
    gate = obs['gate'] = obs['gate'] + np.clip(obs['latched'] - obs['gate'], -DOWN * dt, UP * dt)
    aux = told[2:] @ (1.0 - gate[2:])
    w = told * gate
    w[1] += aux * gate[1]
    w[0] = told[0] + told[1] * (1.0 - gate[1]) + aux * (1.0 - gate[1])
    return w


def skin(world, y):
    """(MANIFOLDS,) her skin's push on each manifold, N, the truth her observer is blind to: each
    segment's touch (`edge.TOUCH`) along each of its points' manifolds' normals (POINTS), once a
    segment and normal."""
    pushed, m, xmat = y[edge.TOUCH].reshape(-1, 3), world.model, world.data.xmat
    world_of = lambda seg: xmat[m.body(seg).id].reshape(3, 3) @ pushed[edge.SKIN.index(seg)]
    out, seen = np.zeros(len(MANIFOLDS)), set()
    for shares, seg, _p, normal in POINTS:
        force = world_of(seg)
        for manifold, frame, axis in shares:
            key = (seg, manifold, frame, repr(axis))
            along = float(force @ (xmat[m.body(frame).id].reshape(3, 3) @ np.array(axis)))
            out[MANIFOLDS.index(manifold)] += 0.0 if key in seen else max(0.0, along)
            seen.add(key)
    # her forefeet: her toes' push and of each sole's the share its touch's centre has gone to its ball
    at = y[edge.AT].reshape(-1, 3)
    for side in ('left', 'right'):
        foot, toes = edge.SKIN.index(side + '_foot'), edge.SKIN.index(side + '_toes')
        push = max(0.0, float(pushed[foot][1]))
        ahead = (float(at[foot][2]) - 0.5 * (BALL - HEEL)) / (0.5 * (BALL + HEEL))
        out[EQUINUS] += push * min(1.0, max(0.0, ahead)) + max(0.0, float(pushed[toes][1]))
    return out


def named(g):
    """Her observer this pass, named: by manifold (MANIFOLDS) its normal force and her skin's (`skin`), N;
    by her rows' (`ortho.MANIFOLDS`) what is told, what she weighs, latched and its gate (`weighed`); each
    point pushing or with gravity's prior on it - its manifold, segment, place, push and prior, N, speed,
    m/s, gated -; the corners her barrier holds her over (`barrier.barred`)."""
    obs, rows = g['observer'], len(ortho.MANIFOLDS)
    normal, truth = obs['normal'], skin(g['world'], g['sense'])
    pushes = np.add.reduceat(obs['f'][:obs['pushes']], obs['cols'])
    prior, speed = (np.ctypeslib.as_array(getattr(obs['c'], k))[:len(POINTS)] for k in ('prior', 'speed'))
    told, weighed = (np.asarray(g.get(k, np.zeros(rows))) for k in ('told', 'weighed_w'))
    return {'manifolds': {m: {'felt': float(normal[k]), 'skin': float(truth[k])} for k, m in enumerate(MANIFOLDS)},
            'rows': {m: {'told': float(told[k]), 'weighed': float(weighed[k]), 'latched': float(obs['latched'][k]),
                         'gate': float(obs['gate'][k])} for k, m in enumerate(ortho.MANIFOLDS)},
            'points': [{'manifold': sh[0][0], 'segment': seg, 'at': obs['points'][p].tolist(), 'push': float(pushes[p]),
                        'prior': float(prior[p]), 'speed': float(speed[p]), 'gated': bool(obs['gated'][p])}
                       for p, (sh, seg, *_r) in enumerate(POINTS) if pushes[p] > 0.0 or prior[p] > 1.0],
            'support': np.asarray(g.get('support', np.zeros((0, 3)))).tolist()}


def bearing(obs):
    """(k, 3) the points that press on a manifold her comparator feels, world: what bears her besides
    her soles."""
    pushed = np.add.reduceat(obs['f'][:obs['pushes']], obs['cols']) > 0.0
    return obs['points'][obs['skin'] & (obs['latched'] > 0.5)[obs['row']] & pushed]


def _link(seg):
    """A segment's link in her chain (wbc_model.h): its last joint's, the pelvis the root."""
    from machine.figure import SEGMENTS
    joints = next(j for name, _p, j, *_r in SEGMENTS if name == seg)
    return joints[-1][0] if joints else 'root'


def _capsules(m):
    """(her capsules, each (segment, radius, its axis's ends in its frame), the pairs of them MuJoCo collides) of
    her model `m`'s colliding geoms: a sphere a capsule of no length, a box one along its longest side as thick as
    its thinnest; a pair on two bodies not welded, not parent and child, not excluded, their bits meeting."""
    from machine.figure import SEGMENTS
    segs, kinds = {s[0] for s in SEGMENTS}, m.geom_type
    geoms = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name in segs and kinds[g] in _AXES]
    out = []
    for g in geoms:
        R, size = rotation(m.geom_quat[g]), m.geom_size[g]
        r, k, half = _AXES[kinds[g]](size)
        out.append((m.body(m.geom_bodyid[g]).name, float(r), *(tuple(m.geom_pos[g] + s * half * R[:, k]) for s in (-1, 1))))
    weld = m.body_weldid
    up = lambda b: weld[m.body_parentid[weld[b]]]
    excluded = {int(s) for s in m.exclude_signature}
    meet = lambda a, b: (m.geom_contype[a] & m.geom_conaffinity[b]) | (m.geom_contype[b] & m.geom_conaffinity[a])
    pairs = [(i, j) for i, a in enumerate(geoms) for j, b in enumerate(geoms) if i < j
             and weld[m.geom_bodyid[a]] != weld[m.geom_bodyid[b]]
             and weld[m.geom_bodyid[a]] != up(m.geom_bodyid[b]) and weld[m.geom_bodyid[b]] != up(m.geom_bodyid[a])
             and (min(m.geom_bodyid[a], m.geom_bodyid[b]) << 16) + max(m.geom_bodyid[a], m.geom_bodyid[b]) not in excluded
             and meet(a, b)]
    return out, pairs


#: Each colliding geom's kind (mjtGeom): its capsule's (radius, axis, half its axis's length) of its sizes.
_AXES = {2: lambda s: (s[0], 2, 0.0), 3: lambda s: (s[0], 2, s[1]), 5: lambda s: (s[0], 2, s[1]),
         6: lambda s: (min(s), int(np.argmax(s)), max(s) - min(s))}


def laid(body):
    """The observer at rest on her law's `body` (`wbc.Body`), in C (wbc_gmo.c): its points, shares, her
    joints' stops, her capsules and her drives' friction laid, its sums empty; its outputs viewed (`r`,
    `f`, `normal`); her forefeet compiled from their numpy (`tools.cores.bodied`)."""
    import ctypes
    from tools.cores import bodied, gmo, wbc as cores
    lib, names = cores.library(), cores.Core().names
    o = gmo.Gmo()
    o.slip, o.eps = SLIP_RAD_S, FLOOR
    o.weight, o.upright, o.reach, o.still = body.mass * G, UPRIGHT_COS, REACH_M, STILL_M_S
    o.alpha = kernel.said('configure-momentum-observer', 'observer-bandwidth-hz') * 2.0 * math.pi
    o.loss[:] = body.m.dof_frictionloss[:body.n].tolist()
    driven = [k for k, j in enumerate(JOINTS) if body.driven[k]]
    o.driven_joint[:] = driven
    o.driven_at[:] = [5 + names.index(JOINTS[k]) for k in driven]
    cols = [sum(len(e) for e in EDGES[:p]) for p in range(len(POINTS))]
    o.pushes = cols[-1] + len(EDGES[-1])
    if o.pushes > gmo.GMO_PUSH:
        raise ValueError('her points push %d ways, wbc_gmo.h at most %d' % (o.pushes, gmo.GMO_PUSH))
    for p, (shares, seg, point, normal) in enumerate(POINTS):
        o.link[p], o.edges[p], o.col[p] = names.index(_link(seg)), len(EDGES[p]), cols[p]
        o.world[p], o.soled[p] = int(shares[0][0] in UPRIGHT), int(shares[0][0] == 'plantar-bilateral')
        o.local[p][:] = list(point)
        for e, along in enumerate(EDGES[p]):
            o.edge[p][e][:] = list(along)
    for h, name in enumerate(names[1:]):
        j = body.m.joint(name).id
        o.stopped[h] = int(body.m.jnt_limited[j])
        o.stop_lo[h], o.stop_hi[h] = (float(v) for v in body.m.jnt_range[j])
    capsules, pairs = _capsules(body.m)
    o.capsules, o.pairs = len(capsules), len(pairs)
    for c, (seg, r, a, b) in enumerate(capsules):
        o.capsule_link[c], o.capsule_r[c] = names.index(_link(seg)), r
        o.capsule_a[c][:], o.capsule_b[c][:] = list(a), list(b)
    for k, pair in enumerate(pairs):
        o.pair[k][:] = list(pair)
    rows = [(p, MANIFOLDS.index(m), names.index(_link(frame)), axis)
            for p, (shares, *_r) in enumerate(POINTS) for m, frame, axis in shares]
    for k, (p, m, link, axis) in enumerate(rows):
        o.share_point[k], o.share_manifold[k], o.share_link[k] = p, m, link
        o.share_axis[k][:] = np.asarray(axis, float).tolist()
    view = lambda field: np.ctypeslib.as_array(getattr(o, field))
    row = np.full(len(MANIFOLDS), -1)
    row[ROWS] = np.arange(len(ROWS))
    row = row[[MANIFOLDS.index(sh[0][0]) for sh, *_r in POINTS]]
    col = lambda seg: cols[next(p for p, (_sh, at, _p, _n) in enumerate(POINTS) if at == seg)]
    base, q, u = np.zeros(12), np.zeros(body.n - 6), np.zeros(body.n)
    return {'c': o, 'at': ctypes.byref(o), 'felt': lib.wbh_gmo_felt, 'step': lib.wbh_gmo_step,
            'r': view('r'), 'f': view('f'), 'normal': view('force'), 'points': view('at'), 'cols': np.array(cols),
            'pushes': o.pushes, 'gated': view('gated'), 'slide': view('slide'),
            'latch_of': np.array([[any(m == MANIFOLDS[k] for m, *_r in sh) for k in ROWS] for sh, *_r in POINTS], float),
            'soled': np.array([sh[0][0] == 'plantar-bilateral' for sh, *_r in POINTS]),
            'skin': np.array([seg not in SOLED for _sh, seg, *_r in POINTS]) & (row >= 0), 'row': row,
            'feet': [(col(side + '_foot'), col(side + '_toes')) for side in ('left', 'right')],
            'forefoot': bodied.on_args(forefoot, 'forefoot', (0,)), 'base': base, 'q': q, 'u': u,
            'ins': tuple(cores.pointer(a) for a in (base, q, u)), 'soles': np.zeros(2),
            'latched': np.zeros(len(ROWS)), 'below': np.zeros(len(ROWS)), 'since': np.zeros(len(ROWS)),
            'gate': np.zeros(len(ROWS))}


#: Her IMU's quaternion, its specific force and her drives' torques: bytes into her edge's array.
_QUAT, _ACCEL, _TAU = (8 * part.start for part in (edge.QUAT, edge.ACCEL, edge.TAU))


def felt(obs, y, dt):
    """A drive pass `dt` of her edge's array `y` into the law pass's sums: her drives' torques, her
    pelvis's acceleration (her IMU's specific force turned to the world, gravity added)."""
    at = address(y)
    obs['felt'](obs['at'], at + _QUAT, at + _ACCEL, at + _TAU, dt)


def step(obs, s, qpos, dt):
    """A law pass on her law's sense `s` (`wbc.sense`) at her estimate `qpos`: r, f and each
    manifold's normal force, a point off her soles not felt - none of its manifolds latched by her
    comparator (`weighed`) - joining the split only where her soles and what she feels cannot bear her;
    her drives' sums emptied."""
    obs['gated'][:] = ~obs['soled'] & (obs['latch_of'] @ (obs['latched'] > 0.5) == 0.0)
    v, base, u = s['v'], obs['base'], obs['u']
    R = rotation(qpos[3:7])
    base[:9], base[9:] = R.ravel(), qpos[:3]
    u[:3], u[3:6], u[6:] = v[3:6], R.T @ v[:3], v[6:]
    obs['q'][:] = qpos[7:7 + len(obs['q'])]
    obs['step'](obs['at'], *obs['ins'], dt)
    obs['normal'][EQUINUS], obs['soles'] = obs['forefoot'](obs['f'], obs['feet'])
    return obs


#: A sole's pushes: its heel's corners' edges, then its ball's.
_HEEL = 2 * len(EDGES[0])


def forefoot(f, feet):
    """(her forefeet's push (`metatarsal-equinus-bilateral`), each sole's and its toes' push (2,)), N, of
    the points' forces `f` - each foot's first corner's and toes' columns `feet`: the forefeet each toes'
    and of each sole's push the share its centre of pressure has gone from the sole's middle to its ball,
    its ball's corners' push less its heel's."""
    out, soles = 0.0, np.zeros(2)
    for k, (sole, toes) in enumerate(feet):
        heel, ball = f[sole:sole + _HEEL].sum(), f[sole + _HEEL:sole + 2 * _HEEL].sum()
        out += np.clip(ball - heel, 0.0, ball + heel) + f[toes]
        soles[k] = heel + ball + f[toes]
    return out, soles
