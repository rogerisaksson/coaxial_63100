"""Her errands: a word of her kinematic language (`machine.kin`) run on the one law.

    errands.ask(director, 'out')                      # a word of `room.WORDS`, from the page
    out = errands.step(director, bus, dt, out)        # every pass on the law

A word's rows are met one after another: `go` lines one way (`machine.pursuit`) from where
she is, round what her eyes have shown her (`machine.sight`, laid again where they bar it);
`face` her heading turned standing; `stand`; `pace` the row asked and the word over; `hand` a
hand raised before her while she is near its place; `reach` an arm raised till what it meets
answers, held, let down; `play` the arrival's keyframes from her pose as read (the director's
stage 'errand'), the last held. With no errand, on the law, the stand's row is asked where her
eyes bar the GUARD_M before her. What a row asks of her balance is the law's: no leg is named
here.
"""
import math

from machine import figure, kin, pace, physics, pursuit, room, sight
from machine.figure import add, apply, ry
from machine.pursuit import ROW_STAND

#: A way's first point is left out where she stands on its line to the next, within AXIS_M of
#: it and JOIN_M or more before that; her way is laid again every PLAN_S at most; `stand` holds
#: SETTLE_S.
AXIS_M, JOIN_M, PLAN_S, SETTLE_S = 0.15, 1.0, 1.0, 1.0
#: Walking on with no errand she stands where her map bars her way GUARD_M[0] to [1] m on.
GUARD_M = (0.4, 1.1)
#: A hand raised to its place, deg (shoulder, elbow), from HAND_M[0] before it to [1] past it
#: along her heading, up and down in HAND_S: with none raised her toes met the door's leaf first
#: and she stood in the doorway on its post; both raised, the hinge's side felled her.
HAND_DEG, HAND_M, HAND_S = (55.0, 25.0), (1.3, 0.3), 0.5
#: An arm reaching: raised REACH_DEG_S to REACH_DEG at most, its elbow to REACH_ELBOW, held
#: HOLD_S at the touch.
REACH_DEG_S, REACH_DEG, REACH_ELBOW, HOLD_S = 40.0, 80.0, 10.0, 0.5

#: A played word, s a keyframe: down onto the seat, laid back onto the bed, up; the pelvis back
#: over the seat, m, and down to it.
SIT_S, LIE_S, UP_S = 1.2, 1.5, 1.2
BACK_M, RISE_M = 0.45, 0.08
#: Her pelvis standing, m.
STAND_Y = 0.90
#: Played, she is down this far under its keyframe's pelvis, or tipped this far past its tilt:
#: sat her pelvis is 0.55 m up, lain she is tipped 88 deg.
FALLEN_M, FALLEN_DEG = 0.2, 35.0
#: The arms at rest, deg.
ARMS = {'left_shoulder': 10.0, 'right_shoulder': 10.0, 'left_elbow': 20.0, 'right_elbow': 20.0}

PROGRAMS = kin.compiled(room.WORDS)


def _moved(now, back, up, tilt, joints=None):
    """`now` (the director's pose keyframe) with the pelvis `back` m along her heading and `up`
    m from its height, tipped `tilt` deg, the feet where they stand, the arms `joints`."""
    pel = add(now['pelvis'], apply(ry(math.radians(now['yaw'])), (0.0, up, -back)))
    return dict(now, pelvis=pel, tilt=tilt, joints={**now['joints'], **ARMS, **(joints or {})})


def frames_of(word, now):
    """[(stage, seconds, keyframe)] a word plays from her pose `now`, the last held."""
    rest = now['pelvis'][1]
    if word == 'sit':
        seat = room.SEAT_H + room.SEAT[1] + RISE_M - rest
        return [('errand', 0.0, now), ('errand', SIT_S, _moved(now, BACK_M, seat + 0.05, 10.0)),
                ('errand', 0.6, _moved(now, BACK_M + 0.03, seat, 0.0))]
    if word == 'lie':
        edge = 2.0 * room.BED[1] + RISE_M - rest
        flat = {'left_shoulder': 0.0, 'right_shoulder': 0.0, 'left_elbow': 5.0, 'right_elbow': 5.0}
        return [('errand', 0.0, now), ('errand', SIT_S, _moved(now, BACK_M, edge + 0.05, 10.0)),
                ('errand', 0.6, _moved(now, BACK_M + 0.03, edge, 0.0)),
                ('errand', LIE_S, _moved(now, BACK_M + 0.15, edge, -70.0, flat)),
                ('errand', 1.0, _moved(now, BACK_M + 0.25, edge, -88.0, flat))]
    if word == 'up':
        return [('errand', 0.0, now),
                ('errand', UP_S, _moved(now, -float(now.get('back', 0.0)), STAND_Y - rest, 0.0))]
    return [('errand', 0.0, now)]


class Errand:

    """One word under way: its rows (`kin.bound`), the one it is at, and what that one keeps -
    her way, a clock, a hand raised."""

    def __init__(self, word):
        if word not in PROGRAMS:
            raise ValueError('no word %r in the room; they are %s' % (word, ', '.join(PROGRAMS)))
        self.word, self.rows, self.i = word, kin.bound(PROGRAMS[word], room.FRAMES), 0
        self.way, self.upto, self.joins, self.planned_at = None, 0, 0.0, 0.0
        self.since, self.ready, self.hand, self.raised = 0.0, False, None, 0.0
        self.arm, self.touched, self.was = 0.0, None, None

    def __repr__(self):
        return '%s: %s' % (self.word, self.phase)

    @property
    def phase(self):
        """The verb she is at, for the page."""
        return next((name for name, verb in kin.VERBS.items()
                     if self.i < len(self.rows) and verb == self.rows[self.i][0]), 'done')


def ask(director, word):
    """The page's `word`: an errand begun on the law - 'up' from a seat or a bed, at once."""
    if word == 'up':
        if director.stage == 'errand' and director.doing in ('sit', 'lie'):
            bus = director.machine.loop.bus
            now = director._now(bus, director.walker.last or {})
            now['back'] = director.back
            _play(director, 'up', now, director.walker.last or {})
        return
    if word == 'out' and director.machine.loop.bus['pelvis.pose.z'] > room.TO:
        word = 'in'             # asked through the door from outside it
    director.errand = Errand(word)
    if director.pace is None:
        director.pace = ROW_STAND   # off the walker: the law takes her as she stands


def kept(director):
    """Her eyes and her map (`sight.Keep`), made on the director's world at first use; None
    without `physics.SIGHT`, or where no GL context can be made (a headless runner)."""
    if not physics.SIGHT:
        return None
    if getattr(director, 'keep', None) is None:
        try:
            director.keep = sight.Keep(director.world)
        except Exception:       # noqa: BLE001 - mujoco.FatalError, no GL: blind, the page whole
            director.keep = False
    return director.keep or None


def looked(director, world, now):
    """Her eyes' look into her map, LOOK_S apart (`sight.Keep.look`); nothing without them."""
    keep = kept(director)
    if keep is not None:
        keep.look(world, now)


def seen(director):
    """Her last look for the page (`sight.Keep.view`); None without her eyes or a look."""
    keep = kept(director)
    return keep.view() if keep is not None else None


def _way(e, keep, at, walking):
    """Her way through the `go` rows from the one she is at: round what she knows to the
    first, then the rest; `e.upto` the row after them, `e.joins` how far along the first is."""
    e.upto = next((i for i in range(e.i, len(e.rows)) if e.rows[i][0] != kin.GO), len(e.rows))
    points = [(row[1], row[2]) for row in e.rows[e.i:e.upto]]
    if len(points) > 1:
        (ax, az), (bx, bz) = points[0], points[1]
        span = math.dist(points[0], points[1])
        on = ((at[0] - ax) * (bx - ax) + (at[1] - az) * (bz - az)) / span
        off = abs((at[0] - ax) * (bz - az) - (at[1] - az) * (bx - ax)) / span
        if off <= AXIS_M and 0.0 <= on <= span - JOIN_M:
            points = points[1:]
    corners = keep.map.plan(at, points[0])[:-1] if keep is not None else []
    after = e.rows[e.upto] if e.upto < len(e.rows) else None
    ends = not (after is not None and after[0] == kin.PACE and after[4] > ROW_STAND)
    way = pursuit.Way([at] + corners + points, ends=ends, walking=walking)
    e.joins = way.arc[-1] - sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return way


def _handed(e, law, at, out, dt):
    """`out` with her errand's hand raised before her while its place is HAND_M before her to
    past her, walking: it meets a door's leaf first, and where the leaf is lightest."""
    if e.hand is None:
        return
    _verb, x, z, _yaw, _amount, side, _name = e.hand
    before = (x - at[0]) * math.sin(law.heading) + (z - at[1]) * math.cos(law.heading)
    walks = e.way is not None and e.way.state == 'walk'
    want = 1.0 if walks and -HAND_M[1] < before < HAND_M[0] else 0.0
    e.raised += max(-dt / HAND_S, min(dt / HAND_S, want - e.raised))
    limb = 'left' if side > 0.0 else 'right'
    for joint, deg in zip(('_shoulder', '_elbow'), HAND_DEG):
        out[limb + joint] = out.get(limb + joint, 0.0) + e.raised * (deg - out.get(limb + joint, 0.0))


def _reached(e, director, out, dt):
    """`out` with the arm of the row she is at raised till the lamp answers or REACH_DEG, held,
    let down; whether it is down again."""
    limb = 'left' if e.rows[e.i][5] > 0.0 else 'right'
    lamp = bool(getattr(director.world, 'lamp', False))
    if e.was is None:
        e.was, e.arm, e.touched = lamp, 0.0, None
    if e.touched is None and (lamp != e.was or e.arm >= REACH_DEG):
        e.touched = 0.0
    if e.touched is None:
        e.arm += REACH_DEG_S * dt
    else:
        e.touched += dt
        if e.touched >= HOLD_S:
            e.arm -= REACH_DEG_S * dt
    rest = out.get(limb + '_shoulder', 0.0)
    k = max(0.0, min(1.0, e.arm / REACH_DEG))
    out[limb + '_shoulder'] = max(rest, e.arm)
    out[limb + '_elbow'] = out.get(limb + '_elbow', 0.0) + k * (REACH_ELBOW - out.get(limb + '_elbow', 0.0))
    return e.touched is not None and e.arm <= rest


def _play(director, word, now, out):
    """The word's keyframes played from `now`, the director's stage 'errand' and its word."""
    frames = frames_of(word, now)
    director.arrival.play(frames)
    director.stage, director.blend, director.age = 'errand', dict(out), 0.0
    director.doing, director.errand = word, None
    director.back = {'sit': BACK_M + 0.03, 'lie': BACK_M + 0.25}.get(word, 0.0)


def fallen(director, bus):
    """Whether she is down at a played word, against the keyframe held."""
    frame = director.arrival._at()[1]
    return (bus['pelvis.pose.y'] < frame['pelvis'][1] - FALLEN_M
            or director._tilt(bus) > abs(frame['tilt']) + FALLEN_DEG)


def held(director, bus, dt):
    """Her setpoints at a played word (the director's stage 'errand'): the keyframes played and
    the last held; 'up' done, the law takes her again."""
    out = director.walker.last = pace.blended(director, director.arrival.step(dt), dt)
    if director.doing == 'up' and director.arrival.t >= sum(s for _n, s, _f in director.arrival.frames):
        director.doing = None
        director.pace = ROW_STAND
        pace.take(director, bus, out)
    return out


def step(director, bus, dt, out):
    """Her setpoints this pass with her errand's row met or held: `out` the law's own, kept
    where a row asks nothing of them."""
    e, law = director.errand, director.going
    if law is None:
        return out
    at = (bus['pelvis.pose.x'], bus['pelvis.pose.z'])
    keep = kept(director)
    if e is None:
        # walking on with no errand: the stand's row asked where her map bars the way before her
        h = law.heading
        if (keep is not None and director.pace is not None and director.pace > ROW_STAND
                and keep.map.barred([(at[0] + m * math.sin(h), at[1] + m * math.cos(h))
                                     for m in GUARD_M])):
            director.pace = ROW_STAND
        return out
    while e.i < len(e.rows) and e.rows[e.i][0] in (kin.HAND, kin.PACE):
        row = e.rows[e.i]
        if row[0] == kin.HAND:
            e.hand = row
        else:
            director.pace = row[4]
        e.i += 1
    if e.i >= len(e.rows):
        _handed(e, law, at, out, dt)
        if e.raised <= 0.0:
            director.errand = None
        return out
    verb, _x, _z, yaw, _amount, _side, name = e.rows[e.i]
    if verb == kin.GO:
        if e.way is None:
            e.way, e.planned_at = _way(e, keep, at, director.k[0] > -0.99), bus['t']
        elif keep is not None and bus['t'] - e.planned_at >= PLAN_S and e.way.s < e.joins:
            # laid again where what she has seen since bars her way to its first point
            e.planned_at = bus['t']
            if keep.map.barred(e.way.rest(e.joins, GUARD_M[0])):
                e.way = _way(e, keep, at, e.way.state == 'walk')
        if pursuit.step(e.way, director, at, dt):
            e.i, e.way, e.ready, e.since = e.upto, None, False, 0.0
    elif verb == kin.FACE:
        e.ready, faced = pursuit.turned(director, pursuit.hold.wrapped(yaw - law.heading), e.ready, dt)
        if faced:
            e.i, e.ready, e.since = e.i + 1, False, 0.0
    elif verb == kin.STAND:
        director.pace = ROW_STAND
        e.since = e.since + dt if pursuit.settled(director) else 0.0
        if e.since >= SETTLE_S:
            e.i, e.since = e.i + 1, 0.0
    elif verb == kin.REACH:
        director.pace = ROW_STAND
        if _reached(e, director, out, dt):
            e.i, e.was = e.i + 1, None
    elif verb == kin.PLAY:
        director.pace = ROW_STAND
        e.since = e.since + dt if pursuit.settled(director) else 0.0
        if e.since >= SETTLE_S:
            _play(director, name, director._now(bus, out), out)
            return out
    _handed(e, law, at, out, dt)
    return out


def facing_of(bus):
    """Her heading as read, rad about up from z."""
    turn = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                       bus['pelvis.pose.qz'])
    return math.atan2(turn[0][2], turn[2][2])
