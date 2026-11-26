"""The robot's generalized momentum observer: the external force on it, blind.

    obs = momentum.laid(body)              # the robot's patches' points laid once, the observer at rest
    observer.felt(obs, y, dt)              # each drive pass: its drives' torques taken (`edge.TAU`)
    observer.step(obs, s, qpos, dt)        # each law pass on its law's sense and estimate: r, its patches
    obs['r'], obs['normal']                # the external force on its dofs; each manifold's, N (MANIFOLDS)

The residual of the robot's momentum over a law pass, M (v - v') / dt + h - S'tau - its pelvis's linear
acceleration its IMU's, tau its drives' mean over it and their friction -, low-passed at the kernel's
bandwidth (`configure-momentum-observer`), is J'f: the external force on the robot and what its body does to
itself. Split by least squares over pushes alone (wbc_gmo.c, Lawson-Hanson), so it always sums to the robot's whole
body's balance and nothing pulls:
- its registry's points (`anatomical-manifold-registry`): a sole's four corners in its law's pyramid (`wbc.MU`), its
  seat, thighs and palms pressed up its IMU's vertical, the rest along their own normals;
- a joint on its stop pushed off it; two of the robot's capsules touching pushed apart;
- a joint's row within its friction's band read as friction (`collapse-static-equilibrium-innovation`);
- a point off the robot's soles its comparator has not felt joining only where its soles cannot bear it;
- its hands on a floor bearing it in the same pyramid;
- what none of it can tell, the robot's soles their last pass's and the rest gravity's prior over its points
  (`regularize-manifold-simplex-with-priors`): upright, low and still bears, its weight's share each; a point
  moving past its grip's slip neither shares nor is free (`gate-supports-by-stillness`), its speed the least of
  its own and against the robot's text's slide (`slide`, `motion`): the floor scoot's stroke slid its seat at 0.62 m/s,
  its seat weighed 0 under its told 0.41 and the robot's legs stood it up off it.
The floor scoot's squat, the robot's knees on their stops (17-25 N m) and its thighs on its belly (330-760 N): a
sole a free wrench read 72-143 N on its butt with none there and its soles pulling to -1 365 N. Blind: no skin, no
load cell.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.senses import edge
from machine.texts import kernel, ortho
from machine.senses import touch
from machine.body.figure import BALL, BODY, CONTACTS, FIST_AT, FIST_R, HEEL, PADS, pad
from machine.stack.wbc import CORNERS, MU
from tools.cores.model import rotation
from tools.cores.wbc import address

#: The robot's manifolds, the registry's; its forefeet's, of its soles' points.
MANIFOLDS = tuple(kernel.read()['anatomical-manifold-registry'])
EQUINUS = MANIFOLDS.index('metatarsal-equinus-bilateral')
#: The robot's drives' friction its loss (`drives.backdrive`, as the robot's model has it) times tanh(v /
#: SLIP_RAD_S) (the text's Stribeck). Measured on MuJoCo's: 0.42 of the loss at 3-10 mrad/s, 0.65 at 10-30,
#: 0.85 at 30-100, 0.97 past; unmodelled, the robot's soles' 1.8 m s^-2 walking read 19.5 N light and its seat
#: 44 N borne, modelled 3.3 and 27.
SLIP_RAD_S = 0.03
_UP, _BACK = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
#: The robot's soles alone bearing it, as its rows' manifolds weigh (`ortho.MANIFOLDS`).
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


def _thigh(x=1.0):
    """A thigh's underside where the seat's patch ends ahead of the buttocks (`cop-bounds-mm`), on the robot's
    thigh's capsule, sat: pressed up from behind it; `x` -1 the right's, its capsule mirrored (`figure.skin_of`)."""
    r, mid = _capsule('thigh', 0)
    ahead = kernel.read()['anatomical-manifold-registry']['ischial-bilateral']['cop-bounds-mm']['x'][1] / 1e3
    return (x * mid[0], -ahead, mid[2] - r)


def _hip(x):
    """A hip's outermost point on the robot's pelvis, its trochanter's pad (`figure.PADS`), `x` 1 the left."""
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
#: its own, and sat pressed up the robot's pelvis (the seat's), lain pressed along its back (the
#: back's) -; each hip's side, pressed in; the sacrum, the lumbar and thoracic back and the skull's, pressed
#: forward; each palm's heel, pressed up its arm, a brace's or a shove's; each forearm's front by its wrist,
#: pressed back, a shove's.
_SEAT, _LAIN = ('ischial-bilateral', 'pelvis', _UP), ('dorsal-supine-continuum', 'pelvis', _BACK)
POINTS = tuple((((m, seg if frame is None else frame, n if axis is None else axis),) if isinstance(m, str) else m,
                seg, p, n) for m, frame, axis, seg, p, n in (
    *(('plantar-bilateral', None, None, side + '_foot', tuple(float(v) for v in c), _UP)
      for side in ('left', 'right') for c in CORNERS),
    ('plantar-bilateral', None, None, 'left_toes', _TOE, _UP), ('plantar-bilateral', None, None, 'right_toes', _TOE, _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 0), _UP),
    ('ischial-bilateral', None, None, 'pelvis', _under('pelvis', 1), _UP),
    ((('posterior-femoral-bilateral', 'left_thigh', _BACK), _SEAT, _LAIN), None, None, 'left_thigh', _thigh(), _BACK),
    ((('posterior-femoral-bilateral', 'right_thigh', _BACK), _SEAT, _LAIN), None, None, 'right_thigh', _thigh(-1.0), _BACK),
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
#: The robot's soles' segments, their corners pressing in its law's pyramid. The robot's other points press along
#: one direction: in a pyramid its butt's and thighs' friction read its chair's sit 0 N on its soles bearing
#: 110-130, its toes' its forefeet 142 N off walking.
SOLED = ('left_foot', 'right_foot')


def _pyramid(n):
    """The robot's law's pyramid's four edges about the normal `n`: n and MU along its two tangents either
    way."""
    n = np.asarray(n, float)
    t1 = np.cross(n, (1.0, 0.0, 0.0) if abs(n[0]) < 0.5 else (0.0, 1.0, 0.0))
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(n, t1)
    return tuple(tuple(float(v) for v in n + MU * (a * t1 + b * t2)) for a in (-1.0, 1.0) for b in (-1.0, 1.0))


PYRAMID = _pyramid(_UP)
#: Gravity's prior where the robot's rows cannot tell (the kernel's text): its tie-break, of the robot's split's
#: largest curvature; a point's pushes' cosine with the vertical in it; its height over the robot's lowest point in
#: it, m. Tied to none, the robot's straight legs read 17 N on its butt standing and its hips' sides squeezed
#: 231-899 N.
_PRIOR = 'regularize-manifold-simplex-with-priors'
FLOOR, UPRIGHT_COS, REACH_M = (kernel.said(_PRIOR, k) for k in ('min-prior-floor-mg-ratio', 'gravity-alignment-beta',
                                                                'height-reach-m'))
#: A support stands still: a point moving past this, m/s, takes no prior share and is not free (the kernel's
#: `gate-supports-by-stillness`): a hand the robot lifts or a sole in swing does not bear it.
STILL_M_S = kernel.said('gate-supports-by-stillness', 'support-slip-mps')
#: The manifolds whose points bear on a support under the robot, pressed up the world's vertical (its IMU's),
#: not their segment's: its palms' along its arm read 0 N of their 120-158 sat on the floor leaning back.
UPRIGHT = ('ischial-bilateral', 'posterior-femoral-bilateral', 'palmar-carpal-bilateral')
#: The robot's hands on a floor bear it as its soles do, in its law's pyramid up the
#: vertical: pressing up alone, the floor scoot's strokes read their 171-248 N on its seat, its 0 N.
HANDED = ('palmar-carpal-bilateral',)
#: Each point's pushes: its pyramid's edges, else its normal.
EDGES = tuple(PYRAMID if seg in SOLED or sh[0][0] in HANDED else (_UP,) if sh[0][0] in UPRIGHT else (n,)
              for sh, seg, _p, n in POINTS)


#: The robot's rows' manifolds (`ortho.MANIFOLDS`) among the observer's; the text's comparators on each,
#: of its weight (`latch-hysteretic-contact`): engaged at, released under, held below that long,
#: none released before so long engaged; its weights' slews up and down, a second
#: (`bind-wrench-driven-homotopy`).
ROWS = np.array([[m.split('-')[0] for m in MANIFOLDS].index(m) for m in ortho.MANIFOLDS])
_LATCH = kernel.node('latch-hysteretic-contact', 'contact-comparators')
ENGAGE, RELEASE = _LATCH['schmitt-high-engage-fz'], _LATCH['schmitt-low-release-fz']
DWELL_S, BLANK_S = 1e-3 * _LATCH['min-state-dwell-ms'], 1e-3 * _LATCH['impact-blanking-window-ms']
UP, DOWN = (kernel.said('bind-wrench-driven-homotopy', k) for k in ('slew-rate-up-per-s', 'slew-rate-down-per-s'))


def said(told):
    """`told` (`latent.MANIFOLDED`) as the robot's law takes it: each manifold's share of what is told;
    nothing told (the robot's rest), its soles alone. Told anew only as `told`'s bits change
    (`_SAID`), a copy each read: the robot's law's pass, its legs' gate and settle and the robot's
    referee read it of the same `told`, four times a law pass; traced (`tools.cores.trace`, a compiled
    part's), numpy's expression as it reads."""
    if type(told) is not np.ndarray:
        total = told.sum()
        return np.where(total > 0.0, told / np.maximum(total, TINY), SOLES)
    key = told.tobytes()
    if _SAID[0] != key:
        total = told.sum()
        _SAID[:] = key, np.where(total > 0.0, told / np.maximum(total, TINY), SOLES)
    return _SAID[1].copy()


#: `said`'s last: (the bits of what was told, its shares).
_SAID: list = [None, None]


def parts(m):
    """Each manifold's bodies of `m` (ROWS' order): its points' segments (POINTS), a whole arm where one of its
    segments is among them (`touch.ARMS`), and their children: lain back onto the robot's forearm, the robot's
    hands told, it was called fallen. Laid once a world by its referee, not cached by `m`: so cached, every world
    laid anew kept its model."""
    found = [{m.body(seg).id for shares, seg, _p, _n in POINTS if MANIFOLDS[r] in {s[0] for s in shares}} for r in ROWS]
    arms = [{m.body(edge.SKIN[i]).id for i in arm} for arm in touch.ARMS]
    found = [ids.union(*(arm for arm in arms if arm & ids)) for ids in found]
    return [ids | {b for b in range(m.nbody) if m.body_parentid[b] in ids} for ids in found]


def unborne(manifolds, on, down):
    """[`down`'s bodies] less those of the manifolds `on` (ROWS', a bool each) bear, `manifolds` each one's bodies
    (`parts`)."""
    return list(down - set().union(*(p for o, p in zip(on, manifolds) if o)))


def weighed(obs, told, weight, dt):
    """(the robot's rows' manifolds,) what bears it as its law takes it, `dt` s on: each
    manifold the robot's rows tell weighed as told where its comparator (ENGAGE, RELEASE, DWELL_S,
    BLANK_S) feels it - its normal force of the robot's `weight` -, gated up at UP and down at DOWN a second;
    what a manifold not felt was told, its soles'. Nothing told (the robot's rest), its soles alone;
    told, they bear it as told; a seat pulled away weighs nothing at once."""
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
    """(MANIFOLDS,) the robot's skin's push on each manifold, N, the truth its observer is blind to:
    each segment's touch (`edge.TOUCH`) along each of its points' manifolds' normals (POINTS), once a segment
    and normal."""
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
    # the robot's forefeet: its toes' push and of each sole's the share its touch's centre has gone to
    # its ball
    at = y[edge.AT].reshape(-1, 3)
    for side in ('left', 'right'):
        foot, toes = edge.SKIN.index(side + '_foot'), edge.SKIN.index(side + '_toes')
        push = max(0.0, float(pushed[foot][1]))
        ahead = (float(at[foot][2]) - 0.5 * (BALL - HEEL)) / (0.5 * (BALL + HEEL))
        out[EQUINUS] += push * min(1.0, max(0.0, ahead)) + max(0.0, float(pushed[toes][1]))
    return out


def named(g):
    """The robot's observer this pass, named: by manifold (MANIFOLDS) its normal force and the robot's skin's
    (`skin`), N; by its rows' (`ortho.MANIFOLDS`) what is told, what it weighs, latched and its
    gate (`weighed`); each point pushing or with gravity's prior on it - its manifold, segment, place, push
    and prior, N, speed, m/s, gated -; the corners the robot's barrier holds it over
    (`barrier.barred`)."""
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
    """(k, 3) the points that press on a manifold the robot's comparator feels, world: what bears it besides
    its soles."""
    pushed = np.add.reduceat(obs['f'][:obs['pushes']], obs['cols']) > 0.0
    return obs['points'][obs['skin'] & (obs['latched'] > 0.5)[obs['row']] & pushed]


#: The robot's IMU's quaternion, its specific force, the robot's drives' torques, its motors' angles through their
#: boxes and its joints': bytes into its edge's array.
_QUAT, _ACCEL, _TAU, _QM, _Q = (8 * part.start for part in (edge.QUAT, edge.ACCEL, edge.TAU, edge.QM, edge.Q))


def felt(obs, y, dt):
    """A drive pass `dt` of the robot's edge's array `y` into the law pass's sums: the torque each link is turned
    by - its drive's, or what a box with a spring in series passes on, its motor's angle past its joint's -, its
    pelvis's acceleration (its IMU's specific force turned to the world, gravity added)."""
    at = address(y)
    obs['felt'](obs['at'], at + _QUAT, at + _ACCEL, at + _TAU, at + _QM, at + _Q, dt)


def _prepared(obs, qpos, v):
    latched = obs['latched'].tobytes()
    if obs['gated_of'][0] != latched:     # the robot's gate of its comparator's latches alone: told anew as they change
        obs['gated'][:] = ~obs['soled'] & (obs['latch_of'] @ (obs['latched'] > 0.5) == 0.0)
        obs['gated_of'][0] = latched
    base, u = obs['base'], obs['u']
    R = rotation(qpos[3:7])
    base[:9], base[9:] = R.ravel(), qpos[:3]
    u[:3], u[3:6], u[6:] = v[3:6], R.T @ v[:3], v[6:len(u)]
    obs['q'][:] = qpos[7:7 + len(obs['q'])]


def ahead(obs, qpos, qvel, dt):
    """Begin the C momentum observer pass on a sibling thread beside `sensed` (`wbh_gmo_ahead`)."""
    _prepared(obs, qpos, qvel)
    if obs.get('ahead') is not None and obs['ahead'](obs['at'], *obs['ins'], dt):
        obs['_ahead'] = True
    else:
        obs['step'](obs['at'], *obs['ins'], dt)


def behind(obs):
    """Join an asynchronous momentum observer pass (`ahead`) and update forefoot shares."""
    if obs.get('_ahead'):
        obs['_ahead'] = False
        obs['behind']()
    obs['normal'][EQUINUS], obs['soles'] = obs['forefoot'](obs['f'], obs['feet'])
    return obs


def beside(obs, g, body, limbs, qpos, qvel, dt):
    """Pose and sense `body` on the main thread while `obs` steps in C on a sibling thread."""
    async_obs = 'ahead' in obs
    if async_obs:
        ahead(obs, qpos, qvel, dt)
    try:
        s = g['sensed'](body, qpos, qvel)
        s['hips'], s['arms'], s['elbows'], s['qpos'] = (body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']],
                                                        s['q'][limbs['elbows']], qpos.copy())
        if async_obs and 'clear' in g and 'schema_body' in g:
            g['_near'] = g['clear'](g['schema_body'], s, g['schema_at'])
    finally:
        if async_obs:
            behind(obs)
    if not async_obs:
        step(obs, s, qpos, dt)
    return s


def cleared(g, s):
    """Return the pre-computed capsule clearances from `beside` or compute them now."""
    near = g.pop('_near', None)
    return g['clear'](g['schema_body'], s, g['schema_at']) if near is None else near


def step(obs, s, qpos, dt):
    """A law pass on the robot's law's sense `s` (`wbc.sense`) at its estimate `qpos`: r, f and each
    manifold's normal force, a point off its soles not felt - none of its manifolds latched by the
    robot's comparator (`weighed`) - joining the split only where its soles and what it feels
    cannot bear it; its drives' sums emptied."""
    _prepared(obs, qpos, s['v'])
    obs['step'](obs['at'], *obs['ins'], dt)
    obs['normal'][EQUINUS], obs['soles'] = obs['forefoot'](obs['f'], obs['feet'])
    return obs


#: A sole's pushes: its heel's corners' edges, then its ball's.
_HEEL = 2 * len(EDGES[0])


def forefoot(f, feet):
    """(the robot's forefeet's push (`metatarsal-equinus-bilateral`), each sole's and its toes' push (2,)), N,
    of the points' forces `f` - each foot's first corner's and toes' columns `feet`: the forefeet each toes'
    and of each sole's push the share its centre of pressure has gone from the sole's middle to its ball, its
    ball's corners' push less its heel's."""
    out, soles = 0.0, np.zeros(2)
    for k, (sole, toes) in enumerate(feet):
        heel, ball = f[sole:sole + _HEEL].sum(), f[sole + _HEEL:sole + 2 * _HEEL].sum()
        out += np.clip(ball - heel, 0.0, ball + heel) + f[toes]
        soles[k] = heel + ball + f[toes]
    return out, soles
