"""The robot's eyes' gaze: three servos turn its eyes by pushrods, across, down and against each other.

    gz = gaze.moved(g)               # the robot's gaze now (across, down, vergence): a saccade's or its fixation held
    gaze.settled(g)                  # whether its eyes stand on what they look at: a look only so (no saccade's blur)
    gaze.aimed(world, gz)            # its eyes' cameras turned and placed so on its head, as built (`gimbal`)
    gaze.swung(world, gz); gaze.placed(world)     # the same in two: swung each pass, placed where a look is due
    held = gaze.held(g, gz)          # its gimbals as it holds them, each command through its play as told it
    head = gaze.gazed(head, gz)      # the head its eyes look from: its own turned by its gaze about its eye
    gaze.asked(g, gz)                # a look taken at it: the next fixation asked where it is due
    gaze.scan(g, points)             # each of `points` (x, y, z) looked at in turn, HELD looks each
    gaze.scanned(g)                  # whether they all were, what they showed in hand
    gaze.fix(g, point, way, lead)    # held on a world point wherever the robot stands; walking `way`, led by its ground
    gaze.rest(g)                     # let go: the robot's eyes at rest, held there

Each eye turns across (version, left +), down past rest (`DOWN_DEG` +) and in vergence (converging +) on
its gimbal (`gimbal`): stereo rectifies by the robot's calibrated turns (`sight.Eyes.stereo`, `squint`).
Gaze moves in saccades - a minimum-jerk quintic over `MAIN` or `RATE` - then holds world fixations via
120 Hz VOR while vergence lags by `TAU`. Looks fire only when `settled`. Standing idle, the robot
explores unseen `SECTORS` and glimpses by information rate (`_next`) until `rest` (`_arrived`); walking
free, it anchors `HELD` looks ahead (`_anchored`); on a way, it scans waypoints (`scan`) then holds its
target (`fix`) and lead ground (`_ahead`) before `rest`.
"""
import math

from machine import libm
from machine.eyes import gimbal, registry
from machine.senses import inekf
from machine.eyes.epipolar import F
from machine.eyes.gimbal import DOWN_DEG, EYES, turned
from machine.eyes.stereo import BASELINE_M, H, W, _np
from machine.eyes.voxels import FAR_M, NEAR_M

#: The robot's eyes' reach, rad: across either way, and down past their rest from up - a person's eyes'; its
#: vergence's, from parallel to NEAR_M's (`voxels.NEAR_M`, 0.7 m: what it matches no nearer, 9.8 deg).
YAW, PITCH, VERGE = math.radians(50.0), (math.radians(-25.0), math.radians(35.0)), 2.0 * libm.arctan(BASELINE_M / 2.0 / 0.7)
#: A servo's pace, rad/s: an ordinary one's, 60 deg in 0.1 s.
RATE = math.radians(60.0) / 0.1
#: A saccade's time: the main sequence's, s and s a degree (a person's); the robot's vergence's lag, s.
MAIN, TAU = (0.025, 0.0022), 0.13
#: The robot's gaze at rest, rad.
REST = (0.0, 0.0, 0.0)
#: Looks a place of a way the robot scans is held for: what it shows the robot's own by then, its registry's
#: HIT a look to its TAKEN.
HELD = math.ceil(registry.TAKEN / registry.HIT)
#: The robot's view's half across and half up, rad (`stereo.W`, `stereo.H`, `epipolar.F`).
HALF = (libm.arctan(W / 2.0 / F), libm.arctan(H / 2.0 / F))
#: What the robot explores: the world round it in bearings a view's half apart, two rows - at rest and a
#: view's half down past it -, each a look's fixation FAR_M off (`voxels.FAR_M`, its map's reach) till a look
#: has shown it.
SECTORS = tuple((math.tau * k / math.ceil(math.tau / HALF[0]), row) for row in (0.0, HALF[1])
                for k in range(math.ceil(math.tau / HALF[0])))


def _frame(q):
    """The rotation of quaternion `q` (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    return _np().array([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]])


def _quat(R):
    """The quaternion (w, x, y, z) of rotation `R`, its w not negative."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    q = _np().array([w, x, y, z])
    return q / _np().linalg.norm(q)


def _bearings(head, point):
    """(across, down, vergence) rad the robot's eyes look at world `point` with from its `head` {pos, quat},
    past their reach too: each eye's bearing across from its own middle - the robot's version their mean, its
    vergence its right's past its left's -, down from between them past its eyes' rest."""
    np = _np()
    R = _frame(head['quat'])
    d = R.T @ (np.asarray(point, float) - np.asarray(head['pos'], float)) - np.asarray(EYES, float)
    across = [libm.arctan2(d[0] + dx, d[2]) for dx in (0.0, BASELINE_M)]
    diff = math.remainder(across[1] - across[0], math.tau)
    down = libm.arctan2(-d[1], math.hypot(d[0] + BASELINE_M / 2.0, d[2])) - math.radians(DOWN_DEG)
    return math.remainder(across[0] + 0.5 * diff, math.tau), down, diff


def toward(head, point, told=None):
    """(across, down, vergence) rad of `_bearings` (or `told` of them) within the robot's eyes' reach."""
    a, d, v = told or _bearings(head, point)
    return max(-YAW, min(YAW, a)), max(PITCH[0], min(PITCH[1], d)), max(0.0, min(VERGE, v))


def _reached(head, point, told=None):
    """Whether the robot's eyes reach world `point` from its `head` (`_bearings`, or `told` of them, in YAW and
    PITCH)."""
    a, d, _v = told or _bearings(head, point)
    return abs(a) <= YAW and PITCH[0] <= d <= PITCH[1]


def _heading(head):
    """rad: the robot's `head`'s bearing across the floor, the world's (its z's)."""
    R = _frame(head['quat'])
    return libm.arctan2(R[0, 2], R[2, 2])


def _shown(head, sector):
    """(3,) a sector's (`SECTORS`) fixation: FAR_M from the robot's eye along its bearing, its row down past
    its eyes' rest."""
    np = _np()
    down = math.radians(DOWN_DEG) + sector[1]
    eye = np.asarray(head['pos'], float) + _frame(head['quat']) @ np.asarray(EYES, float)
    return eye + FAR_M * np.array([libm.sin(sector[0]) * libm.cos(down), -libm.sin(down), libm.cos(sector[0]) * libm.cos(down)])


def _head(g):
    """The robot's head {pos, quat} in its things' frame: its filter's (`inekf.head`) under its eyes, its sense's
    without."""
    return inekf.head(g['inekf'], g['body']) if g.get('eyes') and 'inekf' in g else g['s']['head']


def _floor(g):
    """The robot's lower sole's height (m) in its things' frame."""
    off = float(g['inekf']['p'][1]) - float(g['est']['pos'][1]) if g.get('eyes') and 'inekf' in g else 0.0
    return min(float(f['sole'][1]) for f in g['s']['feet']) + off


def _laid(g):
    """The robot's gaze's memory, laid at its first read: at rest, exploring where it stands, nothing scanned
    or fixed. Laid there alone: a default handed `setdefault` was laid at every read, the robot's heading with
    it, 61 us a law pass."""
    st = g.get('_gaze')
    if st is None:
        head = _head(g)
        st = g['_gaze'] = {'at': REST, 'to': REST, 'from': REST, 't0': float(g['t']), 'T': 0.0, 'jump': True,
                           'on': False, 't': float(g['t']), 'points': [], 'held': 0, 'after': None, 'fix': None,
                           'way': None, 'lead': 0.0, 'ground': None, 'walking': False, 'anchor': None, 'exploring': True,
                           'explore': None, 'seen': set(), 'visited': set(),
                           'here': (float(head['pos'][0]), float(head['pos'][2])), 'heading': _heading(head)}
    return st


def _glimpses(g):
    """[(slot, centre)] the robot's registry's live slots not yet its own (`registry.TAKEN`): its things at
    any salience (`g['vision'].boxes`) but its things' (`g['things']`); none without its vision."""
    if g.get('vision') is None:
        return []
    taken = {t[4] for t in g.get('things') or () if len(t) > 4}
    return [(t[4], tuple(map(float, t[1]))) for t in g['vision'].boxes(g, -math.inf) if len(t) > 4 and t[4] not in taken]


def _next(g, st):
    """What the robot explores next: of the sectors it has not seen from here and its glimpses not yet looked
    at, those its eyes reach, the one whose look tells most a second - each one's word on what it has not
    seen, a sector's a look and a glimpse's HELD (`registry.TAKEN` over its HIT), over its saccade's time and
    its looks' -; none, None."""
    from machine.eyes.sight import LOOK_S
    head, at = _head(g), st['at']
    open_ = [(_shown(head, s), 1, ('sector', s)) for s in SECTORS if s not in st['seen']]
    open_ += [(c, HELD, ('slot', k)) for k, c in _glimpses(g) if k not in st['visited']]
    best, rate = None, 0.0
    for point, need, key in open_:
        if not _reached(head, point):
            continue
        aim = toward(head, point)
        r = 1.0 / (saccade(math.hypot(aim[0] - at[0], aim[1] - at[1])) + need * LOOK_S)
        if r > rate:
            best, rate = {'point': tuple(map(float, point)), 'need': need, 'looks': 0, 'key': key}, r
    return best


def _told_of(g, st, gz):
    """The sectors a look at gaze `gz` showed the robot whole from its head (`_head`): each one's bearings and
    rows - its half the sectors' spacing either way - within the robot's view's half across and up of where
    its eyes looked, seen from here; a sector's neighbour in its view's edge shown in part, not seen."""
    view, wide = _heading(_head(g)) + gz[0], math.tau / len({s[0] for s in SECTORS}) / 2.0
    for s in SECTORS:
        if (abs(math.remainder(s[0] - view, math.tau)) + wide <= HALF[0] + 1e-9
                and abs(s[1] - gz[1]) + HALF[1] / 2.0 <= HALF[1] + 1e-9):
            st['seen'].add(s)


def _aim(g, st, told=(None, None)):
    """(across, down, vergence) rad of what the robot looks at now: a way's place it scans, the ground its way
    takes it to or the place it fixes, the ground it walks onto, what it explores - from its head in its
    things' frame (`_head`) -, else rest; `told` (a point, its bearings) those `moved` told this pass
    already."""
    point = next((p for p in (st['points'][0] if st['points'] else None, st['ground'], st['fix'], st['anchor'],
                              (st['explore'] or {}).get('point')) if p is not None), None)
    return REST if point is None else toward(_head(g), point, told[1] if point is told[0] else None)


def _anchored(g):
    """(x, y, z) the ground the robot walks onto: where its twist asked (`g['twist']`: ahead, to its left,
    m/s, turning rad/s) takes it from its head's place along its body's heading HELD looks on - what a look
    shows there its own by the time it stands on it -, on its floor; past its eyes' direct reach, clamped onto
    its near floor along its way."""
    from machine.eyes.sight import LOOK_S
    head, T = _head(g), HELD * LOOK_S
    vx, vy, wz = (float(v) for v in g.get('twist', (0.0, 0.0, 0.0)))
    a = (libm.arctan2(float(g['inekf']['R'][0, 2]), float(g['inekf']['R'][2, 2])) if g.get('eyes') and 'inekf' in g
         else (float(g['heading']) if 'heading' in g else _heading(head)))
    b = a + wz * T
    # the robot's heading's sine and cosine walked over T: a turn's arc, or a line
    sn, cs = ((libm.cos(a) - libm.cos(b)) / wz, (libm.sin(b) - libm.sin(a)) / wz) if abs(wz) * T > 1e-9 else (libm.sin(a) * T, libm.cos(a) * T)
    p = (float(head['pos'][0]) + vx * sn + vy * cs, _floor(g),
         float(head['pos'][2]) + vx * cs - vy * sn)
    if _reached(head, p):
        return p
    if math.hypot(vx, vy) <= 1e-6:
        return None
    np = _np()
    a_c, _, _ = _bearings(head, p)
    a_c, down = max(-0.95 * YAW, min(0.95 * YAW, a_c)), math.radians(DOWN_DEG) + 0.95 * PITCH[1]
    R = _frame(head['quat'])
    mid = np.asarray(head['pos'], float) + R @ (np.asarray(EYES, float) - np.array([0.5 * BASELINE_M, 0.0, 0.0]))
    ray = R @ np.array([libm.sin(a_c) * libm.cos(down), -libm.sin(down), libm.cos(a_c) * libm.cos(down)])
    dist = (float(mid[1]) - _floor(g)) / max(-float(ray[1]), 0.25)
    q = tuple(map(float, mid + min(FAR_M, max(NEAR_M + 0.35, dist)) * ray))
    return q if _reached(head, q) else None


def _shows(head, gz, point):
    """Whether a look at gaze `gz` from the robot's `head` shows world `point`: ahead of its left eye, within
    its view."""
    np = _np()
    eye = gazed(head, gz)
    d = gimbal.mounted().T @ (_frame(eye['quat']).T @ (np.asarray(point, float) - eye['pos']) - np.asarray(EYES, float))
    return bool(d[2] < 0.0 and 2.0 * F * abs(d[0]) <= -d[2] * W and 2.0 * F * abs(d[1]) <= -d[2] * H)


def _ahead(g, st):
    """(x, y, z) the ground the robot's way (`fix`'s) takes it to `lead` s on from its place nearest it at its
    pace, on its floor, where its eyes reach it and a look at what it fixes shows it not; else None."""
    np = _np()
    head, way = _head(g), st['way']
    here = np.array([float(head['pos'][0]), float(head['pos'][2])])
    a, ab, s, v = way[:-1, :2], np.diff(way[:, :2], axis=0), way[:, 2], way[:, 3]
    u = np.clip(np.einsum('ij,ij->i', here - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-300), 0.0, 1.0)
    k = int(np.argmin(np.linalg.norm(a + u[:, None] * ab - here, axis=1)))
    # the robot's way's time walked to each of its places, each leg at its ends' pace's mean
    t = np.r_[0.0, np.cumsum(np.diff(s) / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-300))]
    to = float(np.interp(float(np.interp(s[k] + u[k] * (s[k + 1] - s[k]), s, t)) + st['lead'], t, s))
    p = (float(np.interp(to, s, way[:, 0])), _floor(g), float(np.interp(to, s, way[:, 1])))
    return None if not _reached(head, p) or _shows(head, toward(head, st['fix']), p) else p


def _arrived(g, st):
    """The robot's gaze where nothing tells it - no way scanned, no place fixed, its errand none -: walking
    (its twist asked ahead or aside), what it explored dropped and its eyes on the ground it walks onto
    (`_anchored`), let go as it stops; its twist asked none and its head past NEAR_M from where it last
    explored (what it sees nearer its stereo matches not), exploring anew - nothing seen from there, nothing
    looked at -; turned a sector on, what it had not reached from here explored."""
    if st['points'] or st['fix'] is not None or (g.get('_errand') or {}).get('phase') not in (None, 'look'):
        return
    twist = g.get('twist', (0.0, 0.0, 0.0))
    walking = any(abs(float(v)) > 1e-9 for v in twist[:2])
    if walking != st['walking']:
        st.update(walking=walking, anchor=None, jump=True, **({'exploring': False, 'explore': None} if walking else {}))
    if walking and st['anchor'] is None:
        st['anchor'] = _anchored(g)
        st['jump'] = st['jump'] or st['anchor'] is not None
    # exploring or walking neither row below reads the robot's head: its heading told each law pass was 20 us
    # of it
    if st['exploring'] or walking:
        return
    head = _head(g)
    here, heading = (float(head['pos'][0]), float(head['pos'][2])), _heading(head)
    if not st['exploring'] and not any(abs(float(v)) > 1e-9 for v in twist) and math.dist(here, st['here']) > NEAR_M:
        st.update(exploring=True, explore=None, seen=set(), visited=set(), here=here, heading=heading)
    elif not walking and not st['exploring'] and abs(math.remainder(heading - st.get('heading', heading), math.tau)) > 0.5 * HALF[0]:
        st.update(exploring=True, explore=None, heading=heading)


def _quintic(u):
    """A minimum jerk's share of its way at `u` of its time: 10 u^3 - 15 u^4 + 6 u^5."""
    u = max(0.0, min(1.0, u))
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def saccade(a):
    """s: a saccade's time over `a` rad - the main sequence's (MAIN), no shorter than a minimum jerk's peak (1.875 of
    its mean pace) the robot's servos reach (RATE)."""
    return max(MAIN[0] + MAIN[1] * math.degrees(a), 1.875 * a / RATE)


def moved(g):
    """The robot's gaze now (across, down, vergence) rad: a saccade asked (`jump`) launched toward what it
    looks at from where its eyes stand and flown its quintic; else its fixation held on it, its servos' RATE
    the time since at most; its vergence lagged TAU toward its depth either way; `on` while its eyes stand on
    it, no saccade in flight."""
    st = _laid(g)
    _arrived(g, st)
    # what the robot explores told its bearings once a pass, its reach of it and its aim at it both theirs: 23
    # us each
    point, head = (None if st['explore'] is None else st['explore']['point']), _head(g)
    told = (point, None if point is None else _bearings(head, point))
    if st['explore'] is not None and not _reached(head, point, told[1]):
        st['explore'] = None
    if st['exploring'] and st['explore'] is None and not st['points'] and st['fix'] is None:
        st['explore'] = _next(g, st)
        st['exploring'], st['jump'] = st['explore'] is not None, True
    t, aim = float(g['t']), _aim(g, st, told)
    dt, at = max(t - st['t'], 0.0), st['at']
    if st['jump']:
        st.update(jump=False, t0=t, T=saccade(math.hypot(aim[0] - at[0], aim[1] - at[1])),
                  **{'from': at[:2], 'to': aim[:2]})
    if t < st['t0'] + st['T']:
        s = _quintic((t - st['t0']) / st['T'])
        turn = tuple(a + s * (b - a) for a, b in zip(st['from'], st['to']))
    else:
        reach = RATE * dt
        turn = tuple(a + max(-reach, min(reach, b - a)) for a, b in zip(at[:2], aim[:2]))
    verge = aim[2] + (at[2] - aim[2]) * libm.exp(-dt / TAU)
    st['on'] = t >= st['t0'] + st['T'] and all(abs(a - b) <= RATE * dt for a, b in zip(turn, aim[:2]))
    st['at'], st['t'] = turn + (verge,), t
    return st['at']


def settled(g):
    """Whether the robot's eyes stand on what they look at: no saccade in flight, its fixation held (`moved`'s
    `on`)."""
    return bool((g.get('_gaze') or {}).get('on', False))


def asked(g, gz):
    """A look taken at gaze `gz` (the robot's eyes settled: `settled`): a way's place counts HELD looks; led
    by a way, its ground's and its dock's in turn; walking, its anchor held or moved on; exploring,
    what the look showed seen (`_told_of`), what it explores counted its looks and, done, the next asked
    (`_next`) - none left, it holds still."""
    st = _laid(g)
    if st['points']:
        st['held'] += 1
        if st['held'] >= HELD:
            st['points'], st['held'], st['jump'] = st['points'][1:], 0, True
            st['after'] = 0 if not st['points'] else None
    elif st['after'] is not None:
        st['after'] += 1
    elif st['way'] is not None:
        st['held'] += 1
        if st['held'] >= HELD:
            was, st['held'] = st['ground'], 0
            st['ground'] = None if was is not None else _ahead(g, st)
            st['jump'] = was is not None or st['ground'] is not None
    elif st['walking']:
        # the robot's anchor held while a look at it shows the ground it walks onto, else a saccade on to it:
        # a sawtooth
        head, a, ahead = _head(g), st['anchor'], _anchored(g)
        if (a is None or ahead is None or not _reached(head, a) or not _shows(head, toward(head, a), ahead)
                or math.dist(a, ahead) > 0.35 and abs(_bearings(head, ahead)[0]) >= 0.9 * YAW):
            st['anchor'], st['jump'] = ahead, True
    elif st['fix'] is None and st['exploring']:
        _told_of(g, st, gz)
        e = st['explore']
        if e is not None:
            e['looks'] += 1
            if e['looks'] >= e['need']:
                st['visited' if e['key'][0] == 'slot' else 'seen'].add(e['key'][1])
                st['explore'] = _next(g, st)
                st['exploring'], st['jump'] = st['explore'] is not None, True


def _over(g, **told):
    """The robot's gaze told to look as an errand asks (`told`), its exploring and its walk's anchor over, no way led
    but one told."""
    head = _head(g)
    _laid(g).update(dict(held=0, after=None, exploring=False, explore=None, jump=True, way=None, lead=0.0, ground=None,
                         walking=False, anchor=None, heading=_heading(head)), **told)


def scan(g, points):
    """The robot's eyes on each of `points` [(x, y, z)] in turn, HELD looks each (`asked`), its exploring
    over."""
    _over(g, points=[tuple(map(float, p)) for p in points], fix=None)


def scanned(g):
    """Whether the robot's eyes looked at each place told (`scan`), and a look on: their things its own."""
    st = g.get('_gaze') or {}
    return not st.get('points') and (st.get('after') or 0) >= 1


def fix(g, point, way=None, lead=0.0):
    """The robot's gaze held on world `point` (x, y, z) wherever it stands (`moved`), its exploring over;
    walking its `way` ((n, 4+) rows x, z, its length walked to each and its pace there: its planner's curve),
    led by it: HELD looks at a time on `point`, then on its way's ground `lead` s on (`_ahead`)."""
    np = _np()
    _over(g, points=[], fix=tuple(map(float, point)), lead=float(lead),
          way=None if way is None or len(way) < 2 else np.array(np.asarray(way, float)[:, :4]))


def rest(g):
    """The robot's gaze let go: its eyes at rest and held there till it stands somewhere new (`_arrived`)."""
    _over(g, points=[], fix=None)


def aimed(world, gz):
    """The robot's eyes' cameras on its head at its servos' gaze `gz`: its gimbals swung to it (`swung`), its
    cameras placed on them (`placed`)."""
    swung(world, gz)
    placed(world)


def swung(world, gz):
    """The robot's gimbals swung to its servos' gaze `gz`, each through its pushrods and behind their play
    (`gimbal.played`, from where it was: `world.gimbals`) - each pass of its gaze, its play's side its
    path."""
    was = getattr(world, 'gimbals', None) or {s: (0.0, 0.0) for s in ('left', 'right')}
    world.gimbals = {s: gimbal.played(was[s], gimbal.angles(s, gz), s) for s in ('left', 'right')}


def placed(world):
    """The robot's eyes' cameras on its head as its gimbals stand (`swung`): each turned and its middle placed
    on its gimbal as built (`gimbal.eye`, `gimbal.place`), their poses laid anew (`mj_camlight`): what its
    eyes draw next looks so. Read at a look alone (`camera.POSES`, `sight.Eyes.see`), the robot's world's step
    laying them again after: placed each law pass they were 100 us of it."""
    for side, a in world.gimbals.items():
        i, R = world.model.camera('eye_' + side).id, gimbal.eye(side, a)
        world.model.cam_quat[i], world.model.cam_pos[i] = _quat(R), gimbal.place(side, a, R=R)
    world._mj.mj_camlight(world.model, world.data)


def held(g, gz):
    """{side: (across, down) rad} the robot's gimbals as it holds them at its servos' gaze `gz`: each command
    it gives, as its gimbals take each (`aimed`), through its pushrods' play as its calibration told it
    (`gimbal.played`'s told) - sent with a look, its pictures laid by it (`sight.Eyes.stereo`). Told from a
    look's gaze alone, the play's side a saccade's path and the robot's vergence turn left its eyes 0.2-0.5
    deg apart off as built and its depth 5-21 % off."""
    st = _laid(g)
    was = st.get('gimbals') or {s: (0.0, 0.0) for s in ('left', 'right')}
    st['gimbals'] = {s: gimbal.played(was[s], gimbal.angles(s, gz, True), s, True) for s in ('left', 'right')}
    return st['gimbals']


def gazed(head, gz):
    """The head the robot's eyes look from at gaze `gz`: `head` {pos, quat} turned by its version about its
    eye - its rotation its own turned so, its place where its eye stays where it is."""
    np = _np()
    R = _frame(head['quat'])
    turn = R @ turned(gz[:2])
    eye = np.asarray(head['pos'], float) + R @ np.asarray(EYES, float)
    return {'pos': eye - turn @ np.asarray(EYES, float), 'quat': _quat(turn)}
