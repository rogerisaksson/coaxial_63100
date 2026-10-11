"""Her errands in the room: a word's target walked to on the one law, faced, and done there.

    errands.ask(director, 'sit')                      # a word of `room.WORDS`, from the page
    out = errands.step(director, bus, dt, out)        # every pass on the law: steered, then done

An errand goes 'go' - the walk's row asked, the law's heading turned toward the next waypoint
at TURN_RAD_S at most, each waypoint passed within NEAR_M, the last the word's target, come at
facing as the word says -; 'settle' - the stand's row, SETTLE_S; then the word's keyframes are
played from her pose as read (`arrival.play`, the director's stage 'errand') and the last held:
sat, lain, the lamp's switch touched. 'up' plays her back onto her feet, and the law takes her
again. The law walks no curve tighter than its heading's turn lets it and stands where it
stands: a word faced the other way round comes to its target round a loop of waypoints
(`room.WAYS`), never a turn on the spot.
"""
import math

from machine import figure, pace, room
from machine.figure import add, apply, ry

#: The law's heading turned this fast at most, rad/s - on the open floor at 0.05 she walked 16 s
#: turning, at 0.1 she fell at 11 s and at 0.15 at 5 s (2026-10-11) -, and only TURNS_S into
#: the walk's row; a waypoint passed within NEAR_M, the target within HOME_M; the stand's row
#: held SETTLE_S before the word is done; the walk's row asked on the way, the slow one within
#: LAST_M of the end.
TURN_RAD_S, TURNS_S, NEAR_M, HOME_M, SETTLE_S, LAST_M = 0.05, 2.0, 0.35, 0.2, 1.0, 0.8
ROW_GO, ROW_SLOW, ROW_STAND = 0.0, -0.6, -1.0

#: A word's doing, s a keyframe: down onto the seat, laid back onto the bed, the arm to the
#: switch; the pelvis back over the seat, m, and down to it.
SIT_S, LIE_S, REACH_S, UP_S = 1.2, 1.5, 1.0, 1.2
BACK_M, RISE_M = 0.30, 0.08
#: Her pelvis standing, m.
STAND_Y = 0.90

#: The arms at rest, deg, and raised to the switch.
ARMS = {'left_shoulder': 10.0, 'right_shoulder': 10.0, 'left_elbow': 20.0, 'right_elbow': 20.0}
REACHING = {'right_shoulder': 75.0, 'right_elbow': 10.0}


def _moved(now, back, up, tilt, joints=None):
    """`now` (the director's pose keyframe) with the pelvis `back` m along her heading and `up`
    m from its height, tipped `tilt` deg, the feet where they stand, the arms `joints`."""
    pel = add(now['pelvis'], apply(ry(math.radians(now['yaw'])), (0.0, up, -back)))
    return dict(now, pelvis=pel, tilt=tilt, joints=dict(now['joints'], **ARMS, **(joints or {})))


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
    if word == 'lamp':
        return [('errand', 0.0, now), ('errand', REACH_S, _moved(now, 0.0, 0.0, 0.0, REACHING)),
                ('errand', 0.8, _moved(now, 0.0, 0.0, 8.0, REACHING)),
                ('errand', REACH_S, _moved(now, 0.0, 0.0, 0.0))]
    if word == 'up':
        return [('errand', 0.0, now),
                ('errand', UP_S, _moved(now, -float(now.get('back', 0.0)), STAND_Y - rest, 0.0))]
    return [('errand', 0.0, now)]


class Errand:

    """One word under way: its waypoints [(x, z)] world, the last the target, the facing it
    ends in, rad about up from z; where it stands in its going."""

    def __init__(self, word):
        if word not in room.WORDS:
            raise ValueError('no word %r in the room; they are %s' % (word, ', '.join(room.WORDS)))
        self.word = word
        target, facing, self.does = room.WORDS[word]
        self.waypoints = list(room.WAYS.get(word, ())) + ([target] if target else [])
        self.facing, self.phase, self.since = facing, 'go' if target else 'settle', 0.0

    def __repr__(self):
        return '%s: %s' % (self.word, self.phase)


def ask(director, word):
    """The page's `word`: an errand begun on the law - 'up' from a seat or a bed, at once."""
    if word == 'up':
        if director.stage == 'errand' and director.doing in ('sit', 'lie'):
            bus = director.machine.loop.bus
            now = director._now(bus, director.walker.last or {})
            now['back'] = director.back
            _play(director, 'up', now, director.walker.last or {})
        return
    director.errand = Errand(word)
    if director.pace is None or director.pace <= ROW_STAND:
        director.pace = ROW_GO      # the law takes her from her stand at a row above it


def _bearing(at, to):
    """The law's heading from `at` (x, z) to `to`, rad about up from z, positive toward +x."""
    return math.atan2(to[0] - at[0], to[1] - at[1])


def _wrapped(a):
    return (a + math.pi) % math.tau - math.pi


def _play(director, word, now, out):
    """The word's keyframes played from `now`, the director's stage 'errand' and its word."""
    frames = frames_of(word, now)
    director.arrival.play(frames)
    director.stage, director.blend, director.age = 'errand', dict(out), 0.0
    director.doing, director.errand = word, None
    director.back = {'sit': BACK_M + 0.03, 'lie': BACK_M + 0.25}.get(word, 0.0)


def held(director, bus, dt):
    """Her setpoints at a word's doing (the director's stage 'errand'): the keyframes played and
    the last held; 'up' done, the law takes her again."""
    out = director.walker.last = pace.blended(director, director.arrival.step(dt), dt)
    if director.doing == 'up' and director.arrival.t >= sum(s for _n, s, _f in director.arrival.frames):
        director.doing = None
        director.pace = ROW_STAND
        pace.take(director, bus, out)
    return out


def step(director, bus, dt, out):
    """Her setpoints this pass with her errand steered on the law and, at its target, begun:
    `out` the law's own, kept while she goes."""
    e, law = director.errand, director.going
    if e is None or law is None:
        return out
    at = (bus['pelvis.pose.x'], bus['pelvis.pose.z'])
    if e.phase == 'go':
        to = e.waypoints[0]
        far = math.hypot(to[0] - at[0], to[1] - at[1])
        last = len(e.waypoints) == 1
        if far <= (HOME_M if last else NEAR_M):
            if last:
                e.phase, e.since = 'settle', 0.0
                director.pace = ROW_STAND
                return out
            e.waypoints.pop(0)
            to = e.waypoints[0]
            far = math.hypot(to[0] - at[0], to[1] - at[1])
            last = len(e.waypoints) == 1
        e.since += dt if director.k[0] >= ROW_GO - 0.05 else 0.0
        want = e.facing if last and far < LAST_M else _bearing(at, to)
        turn = _wrapped(want - law.heading) if e.since >= TURNS_S else 0.0
        law.heading += max(-TURN_RAD_S * dt, min(TURN_RAD_S * dt, turn))
        director.pace = ROW_SLOW if last and far < LAST_M else ROW_GO
        return out
    e.since += dt
    if e.since >= SETTLE_S and director.k[0] <= -0.99:
        if e.does == 'out':
            director.errand = None
        else:
            _play(director, e.does, director._now(bus, out), out)
    return out


def facing_of(bus):
    """Her heading as read, rad about up from z."""
    turn = figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                       bus['pelvis.pose.qz'])
    return math.atan2(turn[0][2], turn[2][2])
