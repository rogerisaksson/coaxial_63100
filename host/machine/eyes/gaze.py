"""Her eyes' gaze: three servos turn her eyes by pushrods, across, down and against each other.

    gz = gaze.moved(g)               # her gaze now (across, down, vergence): a saccade's or her fixation held
    gaze.settled(g)                  # whether her eyes stand on what they look at: a look only so (no saccade's blur)
    gaze.aimed(world, gz)            # her eyes' cameras turned and placed so on her head, as built (`gimbal`)
    gaze.swung(world, gz); gaze.placed(world)     # the same in two: swung each pass, placed where a look is due
    held = gaze.held(g, gz)          # her gimbals as she holds them, each command through her play as told her
    head = gaze.gazed(head, gz)      # the head her eyes look from: hers turned by her gaze about her eye
    gaze.asked(g, gz)                # a look taken at it: the next fixation asked where it is due
    gaze.scan(g, points)             # each of `points` (x, y, z) looked at in turn, HELD looks each
    gaze.scanned(g)                  # whether they all were, what they showed in hand
    gaze.fix(g, point, way, lead)    # held on a world point wherever she stands; walking `way`, led by its ground
    gaze.rest(g)                     # let go: her eyes at rest, held there

Three ordinary servos turn her eyes - one across (version, to her left +), one down past their rest (pitch, DOWN_DEG's
way +), one across against each other (vergence, converging +), each eye on its own gimbal by small pushrods
(`gimbal`: squinted, played, levered as built) -: a turned or converged pair's rows are no longer its planes through
both eyes, so her stereo lays both pictures on those planes by her eyes' turns as her calibration told her them and her
pictures since (`sight.Eyes.stereo`, `squint`). Her eyes go from what they look at to the next as a person's do, two
pace apart: a saccade, ballistic - a minimum jerk's quintic over the main sequence's time (MAIN), no shorter than her
servos turn (RATE) -, then a fixation held on its world point as her head moves under it (her VOR, from her head as she
senses it, 120 times a second); her vergence slower, the lag of TAU to the fixation's depth. A look is taken only with
her eyes standing on what they look at (`settled`): a look in a saccade's blur none - her map and her squint's word
from still pictures alone. What she looks at: where she stands what she has not seen - the world round her in
sectors (SECTORS), her registry's glimpses not yet hers -, the one whose look tells most a second first (`_next`): the
sectors' wide saccades and single looks before her glimpses' near ones held HELD looks, a person's ambient look then
her focal; nothing left in her eyes' reach, her gaze held still at rest till she stands somewhere new (`_arrived`).
Walking with no errand, her eyes on the ground she walks onto HELD looks on, held as she walks up to it, a saccade on
once a look there shows it no more (`_anchored`): a person's sawtooth.
Told a way, each of its places held till what it shows is hers (`scan`), then the place she goes to held as she walks
(`fix`) - between its looks the ground her way takes her to where a look at that place shows it not, a person's foot
zone between her affordance's looks -, let go as she gets there (`rest`). What a look tells is read from her eye as it looked (`gazed`).
"""
import math

from machine.eyes import gimbal, registry
from machine.eyes.epipolar import F
from machine.eyes.gimbal import DOWN_DEG, EYES, turned
from machine.eyes.stereo import BASELINE_M, H, W, _np
from machine.eyes.voxels import FAR_M, NEAR_M

#: Her eyes' reach, rad: across either way, and down past their rest from up - a person's eyes'; her vergence's, from
#: parallel to NEAR_M's (`voxels.NEAR_M`, 0.7 m: what she matches no nearer, 9.8 deg).
YAW, PITCH, VERGE = math.radians(50.0), (math.radians(-25.0), math.radians(35.0)), 2.0 * math.atan(BASELINE_M / 2.0 / 0.7)
#: A servo's pace, rad/s: an ordinary one's, 60 deg in 0.1 s.
RATE = math.radians(60.0) / 0.1
#: A saccade's time: the main sequence's, s and s a degree - a person's 25 ms and 2.2 ms a degree -; her vergence's lag, s.
MAIN, TAU = (0.025, 0.0022), 0.13
#: Her gaze at rest, rad.
REST = (0.0, 0.0, 0.0)
#: Looks a place of a way she scans is held for: what it shows hers by then, her registry's HIT a look to its TAKEN.
HELD = math.ceil(registry.TAKEN / registry.HIT)
#: Her view's half across and half up, rad (`stereo.W`, `stereo.H`, `epipolar.F`).
HALF = (math.atan(W / 2.0 / F), math.atan(H / 2.0 / F))
#: What she explores: the world round her in bearings a view's half apart, two rows - at rest and a view's half
#: down past it -, each a look's fixation FAR_M off (`voxels.FAR_M`, her map's reach) till a look has shown it.
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
    """(across, down, vergence) rad her eyes look at world `point` with from her `head` {pos, quat}, past their reach
    too: each eye's bearing across from its own middle - her version their mean, her vergence her right's past her
    left's -, down from between them past her eyes' rest."""
    np = _np()
    R = _frame(head['quat'])
    d = R.T @ (np.asarray(point, float) - np.asarray(head['pos'], float)) - np.asarray(EYES, float)
    across = [math.atan2(d[0] + dx, d[2]) for dx in (0.0, BASELINE_M)]
    down = math.atan2(-d[1], math.hypot(d[0] + BASELINE_M / 2.0, d[2])) - math.radians(DOWN_DEG)
    return 0.5 * (across[0] + across[1]), down, across[1] - across[0]


def toward(head, point):
    """(across, down, vergence) rad her eyes look at world `point` (x, y, z) with from her `head` {pos, quat}, within
    their reach (`_bearings`)."""
    a, d, v = _bearings(head, point)
    return max(-YAW, min(YAW, a)), max(PITCH[0], min(PITCH[1], d)), max(0.0, min(VERGE, v))


def _reached(head, point):
    """Whether her eyes reach world `point` from her `head` (`_bearings` within YAW and PITCH)."""
    a, d, _v = _bearings(head, point)
    return abs(a) <= YAW and PITCH[0] <= d <= PITCH[1]


def _heading(head):
    """rad: her `head`'s bearing across the floor, the world's (its z's)."""
    R = _frame(head['quat'])
    return math.atan2(R[0, 2], R[2, 2])


def _shown(head, sector):
    """(3,) a sector's (`SECTORS`) fixation: FAR_M from her eye along its bearing, its row down past her eyes' rest."""
    np = _np()
    down = math.radians(DOWN_DEG) + sector[1]
    eye = np.asarray(head['pos'], float) + _frame(head['quat']) @ np.asarray(EYES, float)
    return eye + FAR_M * np.array([math.sin(sector[0]) * math.cos(down), -math.sin(down), math.cos(sector[0]) * math.cos(down)])


def _laid(g):
    """Her gaze's memory, laid at its first read: at rest, exploring where she stands, nothing scanned or fixed. Laid
    there alone: a default handed `setdefault` was laid at every read, her heading with it, 61 us a law pass."""
    st = g.get('_gaze')
    if st is None:
        head = g['s']['head']
        st = g['_gaze'] = {'at': REST, 'to': REST, 'from': REST, 't0': float(g['t']), 'T': 0.0, 'jump': True,
                           'on': False, 't': float(g['t']), 'points': [], 'held': 0, 'after': None, 'fix': None,
                           'way': None, 'lead': 0.0, 'ground': None, 'walking': False, 'anchor': None, 'exploring': True,
                           'explore': None, 'seen': set(), 'visited': set(),
                           'here': (float(head['pos'][0]), float(head['pos'][2])), 'heading': _heading(head)}
    return st


def _glimpses(g):
    """[(slot, centre)] her registry's live slots not yet hers (`registry.TAKEN`): its things at any salience
    (`g['vision'].boxes`) but her things' (`g['things']`); none without her vision."""
    if g.get('vision') is None:
        return []
    taken = {t[4] for t in g.get('things') or () if len(t) > 4}
    return [(t[4], tuple(map(float, t[1]))) for t in g['vision'].boxes(g, -math.inf) if len(t) > 4 and t[4] not in taken]


def _next(g, st):
    """What she explores next: of the sectors she has not seen from here and her glimpses not yet looked at, those her
    eyes reach, the one whose look tells most a second - each one's word on what she has not seen, a sector's a look
    and a glimpse's HELD (`registry.TAKEN` over its HIT), over its saccade's time and its looks' -; none, None."""
    from machine.eyes.sight import LOOK_S
    head, at = g['s']['head'], st['at']
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
    """The sectors a look at gaze `gz` showed her whole from her head (`g['s']['head']`): each one's bearings and rows -
    its half the sectors' spacing either way - within her view's half across and up of where her eyes looked, seen
    from here; a sector's neighbour in her view's edge shown in part, not seen."""
    view, wide = _heading(g['s']['head']) + gz[0], math.tau / len({s[0] for s in SECTORS}) / 2.0
    for s in SECTORS:
        if (abs(math.remainder(s[0] - view, math.tau)) + wide <= HALF[0] + 1e-9
                and abs(s[1] - gz[1]) + HALF[1] / 2.0 <= HALF[1] + 1e-9):
            st['seen'].add(s)


def _aim(g, st):
    """(across, down, vergence) rad of what she looks at now: a way's place she scans, the ground her way takes her to
    or the place she fixes, the ground she walks onto, what she explores - from her head as she senses it
    (`g['s']['head']`) -, else rest."""
    point = next((p for p in (st['points'][0] if st['points'] else None, st['ground'], st['fix'], st['anchor'],
                              (st['explore'] or {}).get('point')) if p is not None), None)
    return REST if point is None else toward(g['s']['head'], point)


def _anchored(g):
    """(x, y, z) the ground she walks onto: where her twist asked (`g['twist']`: ahead, to her left, m/s, turning
    rad/s) takes her from her head's place and heading HELD looks on - what a look shows there hers by the time she
    stands on it -, on her floor; out of her eyes' reach, None."""
    from machine.eyes.sight import LOOK_S
    head, T = g['s']['head'], HELD * LOOK_S
    vx, vy, wz = (float(v) for v in g.get('twist', (0.0, 0.0, 0.0)))
    a = _heading(head)
    b = a + wz * T
    # her heading's sine and cosine walked over T: a turn's arc, or a line
    sn, cs = ((math.cos(a) - math.cos(b)) / wz, (math.sin(b) - math.sin(a)) / wz) if abs(wz) * T > 1e-9 else (math.sin(a) * T, math.cos(a) * T)
    p = (float(head['pos'][0]) + vx * sn + vy * cs, min(float(f['sole'][1]) for f in g['s']['feet']),
         float(head['pos'][2]) + vx * cs - vy * sn)
    return p if _reached(head, p) else None


def _shows(head, gz, point):
    """Whether a look at gaze `gz` from her `head` shows world `point`: ahead of her left eye, within its view."""
    np = _np()
    eye = gazed(head, gz)
    d = gimbal.mounted().T @ (_frame(eye['quat']).T @ (np.asarray(point, float) - eye['pos']) - np.asarray(EYES, float))
    return bool(d[2] < 0.0 and 2.0 * F * abs(d[0]) <= -d[2] * W and 2.0 * F * abs(d[1]) <= -d[2] * H)


def _ahead(g, st):
    """(x, y, z) the ground her way (`fix`'s) takes her to `lead` s on from its place nearest her at its pace, on her
    floor, where her eyes reach it and a look at what she fixes shows it not; else None."""
    np = _np()
    head, way = g['s']['head'], st['way']
    here = np.array([float(head['pos'][0]), float(head['pos'][2])])
    a, ab, s, v = way[:-1, :2], np.diff(way[:, :2], axis=0), way[:, 2], way[:, 3]
    u = np.clip(np.einsum('ij,ij->i', here - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-300), 0.0, 1.0)
    k = int(np.argmin(np.linalg.norm(a + u[:, None] * ab - here, axis=1)))
    # her way's time walked to each of its places, each leg at its ends' pace's mean
    t = np.r_[0.0, np.cumsum(np.diff(s) / np.maximum(0.5 * (v[:-1] + v[1:]), 1e-300))]
    to = float(np.interp(float(np.interp(s[k] + u[k] * (s[k + 1] - s[k]), s, t)) + st['lead'], t, s))
    p = (float(np.interp(to, s, way[:, 0])), min(float(f['sole'][1]) for f in g['s']['feet']), float(np.interp(to, s, way[:, 1])))
    return None if not _reached(head, p) or _shows(head, toward(head, st['fix']), p) else p


def _arrived(g, st):
    """Her gaze where nothing tells it - no way scanned, no place fixed, her errand none -: walking (her twist asked
    ahead or aside), what she explored dropped and her eyes on the ground she walks onto (`_anchored`), let go as she
    stops; her twist asked none and her head past NEAR_M from where she last explored (what she sees nearer her
    stereo matches not), exploring anew - nothing seen from there, nothing looked at -; turned a sector on, what she
    had not reached from here explored."""
    if st['points'] or st['fix'] is not None or (g.get('_errand') or {}).get('phase') not in (None, 'look'):
        return
    twist = g.get('twist', (0.0, 0.0, 0.0))
    walking = any(abs(float(v)) > 1e-9 for v in twist[:2])
    if walking != st['walking']:
        st.update(walking=walking, anchor=None, jump=True, **({'exploring': False, 'explore': None} if walking else {}))
    if walking and st['anchor'] is None:
        st['anchor'] = _anchored(g)
        st['jump'] = st['jump'] or st['anchor'] is not None
    # exploring or walking neither row below reads her head: her heading told each law pass was 20 us of it
    if st['exploring'] or walking:
        return
    head = g['s']['head']
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
    its mean pace) her servos reach (RATE)."""
    return max(MAIN[0] + MAIN[1] * math.degrees(a), 1.875 * a / RATE)


def moved(g):
    """Her gaze now (across, down, vergence) rad: a saccade asked (`jump`) launched toward what she looks at from where
    her eyes stand and flown its quintic; else her fixation held on it, her servos' RATE the time since at most;
    her vergence lagged TAU toward its depth either way; `on` while her eyes stand on it, no saccade in flight."""
    st = _laid(g)
    _arrived(g, st)
    if st['explore'] is not None and not _reached(g['s']['head'], st['explore']['point']):
        st['explore'] = None
    if st['exploring'] and st['explore'] is None and not st['points'] and st['fix'] is None:
        st['explore'] = _next(g, st)
        st['exploring'], st['jump'] = st['explore'] is not None, True
    t, aim = float(g['t']), _aim(g, st)
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
    verge = aim[2] + (at[2] - aim[2]) * math.exp(-dt / TAU)
    st['on'] = t >= st['t0'] + st['T'] and all(abs(a - b) <= RATE * dt for a, b in zip(turn, aim[:2]))
    st['at'], st['t'] = turn + (verge,), t
    return st['at']


def settled(g):
    """Whether her eyes stand on what they look at: no saccade in flight, her fixation held (`moved`'s `on`)."""
    return bool((g.get('_gaze') or {}).get('on', False))


def asked(g, gz):
    """A look taken at gaze `gz` (her eyes settled: `settled`): a way's place counts HELD looks; led by a way, its
    ground's and her dock's in turn; walking, her anchor held or moved on; exploring, what the look showed seen
    (`_told_of`), what she explores counted its looks and, done, the next asked (`_next`) - none left, she holds still."""
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
        # her anchor held while a look at it shows the ground she walks onto, else a saccade on to it: a sawtooth
        head, a, ahead = g['s']['head'], st['anchor'], _anchored(g)
        if a is None or ahead is None or not _reached(head, a) or not _shows(head, toward(head, a), ahead):
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
    """Her gaze told to look as an errand asks (`told`), her exploring and her walk's anchor over, no way led but one
    told."""
    head = g['s']['head']
    _laid(g).update(dict(held=0, after=None, exploring=False, explore=None, jump=True, way=None, lead=0.0, ground=None,
                         walking=False, anchor=None, heading=_heading(head)), **told)


def scan(g, points):
    """Her eyes on each of `points` [(x, y, z)] in turn, HELD looks each (`asked`), her exploring over."""
    _over(g, points=[tuple(map(float, p)) for p in points], fix=None)


def scanned(g):
    """Whether her eyes have looked at each place they were told to scan (`scan`), and a look on: their things hers."""
    st = g.get('_gaze') or {}
    return not st.get('points') and (st.get('after') or 0) >= 1


def fix(g, point, way=None, lead=0.0):
    """Her gaze held on world `point` (x, y, z) wherever she stands (`moved`), her exploring over; walking her `way`
    ((n, 4+) rows x, z, its length walked to each and her pace there: her planner's curve), led by it: HELD looks at a
    time on `point`, then on the ground it takes her to `lead` s on where a look at `point` shows it not (`_ahead`)."""
    np = _np()
    _over(g, points=[], fix=tuple(map(float, point)), lead=float(lead),
          way=None if way is None or len(way) < 2 else np.array(np.asarray(way, float)[:, :4]))


def rest(g):
    """Her gaze let go: her eyes at rest and held there till she stands somewhere new (`_arrived`)."""
    _over(g, points=[], fix=None)


def aimed(world, gz):
    """Her eyes' cameras on her head at her servos' gaze `gz`: her gimbals swung to it (`swung`), her cameras placed
    on them (`placed`)."""
    swung(world, gz)
    placed(world)


def swung(world, gz):
    """Her gimbals swung to her servos' gaze `gz`, each through its pushrods and behind their play (`gimbal.played`,
    from where it was: `world.gimbals`) - each pass of her gaze, her play's side its path."""
    was = getattr(world, 'gimbals', None) or {s: (0.0, 0.0) for s in ('left', 'right')}
    world.gimbals = {s: gimbal.played(was[s], gimbal.angles(s, gz), s) for s in ('left', 'right')}


def placed(world):
    """Her eyes' cameras on her head as her gimbals stand (`swung`): each turned and its middle placed on its gimbal
    as built (`gimbal.eye`, `gimbal.place`), their poses laid anew (`mj_camlight`): what her eyes draw next looks so.
    Read at a look alone (`camera.POSES`, `sight.Eyes.see`), her world's step laying them again after: placed each
    law pass they were 100 us of it."""
    for side, a in world.gimbals.items():
        i, R = world.model.camera('eye_' + side).id, gimbal.eye(side, a)
        world.model.cam_quat[i], world.model.cam_pos[i] = _quat(R), gimbal.place(side, a, R=R)
    world._mj.mj_camlight(world.model, world.data)


def held(g, gz):
    """{side: (across, down) rad} her gimbals as she holds them at her servos' gaze `gz`: each command she gives, as her
    gimbals take each (`aimed`), through her pushrods' play as her calibration told her it (`gimbal.played`'s told) -
    sent with a look, her pictures laid by it (`sight.Eyes.stereo`). Told from a look's gaze alone, the play's side
    a saccade's path and her vergence turn left her eyes 0.2-0.5 deg apart off as built and her depth 5-21 % off."""
    st = _laid(g)
    was = st.get('gimbals') or {s: (0.0, 0.0) for s in ('left', 'right')}
    st['gimbals'] = {s: gimbal.played(was[s], gimbal.angles(s, gz, True), s, True) for s in ('left', 'right')}
    return st['gimbals']


def gazed(head, gz):
    """The head her eyes look from at gaze `gz`: `head` {pos, quat} turned by its version about her eye - its rotation
    hers turned so, its place where her eye stays where it is."""
    np = _np()
    R = _frame(head['quat'])
    turn = R @ turned(gz[:2])
    eye = np.asarray(head['pos'], float) + R @ np.asarray(EYES, float)
    return {'pos': eye - turn @ np.asarray(EYES, float), 'quat': _quat(turn)}
