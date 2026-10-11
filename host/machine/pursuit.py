"""Her way walked on the one law. A path of points followed by the law's heading: aimed a
stretch ahead on it, its sharp corners and its end met standing.

    way = Way([(x, z), ..], ends=True)              # from where she is
    there = pursuit.step(way, director, at, dt)     # every pass: the row asked, the heading turned

She walks the walk's row alone - a slower one shortens her steps on a bent knee (room.md) -
aimed LOOK_M along the path from where she is on it. The heading turns TURN_RAD_S at most
(turns.md), a 2.8 m curve at her walk: a corner past FACES_RAD is none she walks, so it is met
standing - the stand's row asked STOP_M before it, the heading turned on the spot once her
stand wants no step (`settled`), and on. Asked within FACES_RAD of her way she walks at once
and turns walking: a walk begun after a turn on the spot is her weakest start (turns.md).
Walking, she stops to turn only OFF_RAD off her way.
"""
import bisect
import math

from machine import hold

#: The heading turned this fast at most, rad/s, (walking, standing), walking only TURNS_S into
#: the walk's row: at 0.3 she turned 292 deg in 17 s and at 0.5 fell at 8 s; standing at 0.5 she
#: stepped round 258 deg in 9 s, at 1.0 fell at 4.5 s. Asked FACES_RAD off her way she turns
#: standing, till within FACED_RAD of it; walking, OFF_RAD off it - at FACES_RAD a walk begun
#: 23.5 deg off crossed it in its sway and stopped.
TURN_RAD_S, TURNS_S, FACES_RAD, FACED_RAD, OFF_RAD = (0.3, 0.5), 0.5, 0.5, 0.06, 1.0
#: Aimed LOOK_M along her way, no nearer than NEAR_M; where she is on it sought AHEAD_M on from
#: where she was; the stand's row asked STOP_M before a stop - the halt from the walk's row
#: carries her 0.45 m.
LOOK_M, NEAR_M, AHEAD_M, STOP_M = 0.8, 0.3, 1.5, 0.45
#: Settled, both feet have stood this long and she moves no faster, m/s.
SETTLED_S, STILL_M_S = 0.3, 0.1
ROW_GO, ROW_STAND = 0.0, -1.0


def bearing(at, to):
    """The law's heading from `at` (x, z) to `to`, rad about up from z, positive toward +x."""
    return math.atan2(to[0] - at[0], to[1] - at[1])


def settled(director):
    """Whether she stands still on the stand's row and her stand wants no step: both feet down
    SETTLED_S, neither to leave. Walked on from less - a foot just landed under her, a turn's
    last step - she stepped up to her front foot twice and sat back over her heels, 3 restarts
    of 6."""
    law = director.going
    return (director.k[0] <= -0.99 and law.stays is None
            and all(leg['stands'] and leg['t'] >= SETTLED_S for leg in law.legs.values())
            and math.hypot(law.v[0], law.v[2]) < STILL_M_S)


def turned(director, turn, ready, dt):
    """(ready, done): standing, the law's heading turned toward `turn` rad off it,
    TURN_RAD_S[1] at most, from when her stand wants no step; done as she faces it within
    FACED_RAD, settled."""
    director.pace = ROW_STAND
    ready = ready or settled(director)
    if ready:
        director.going.heading += max(-TURN_RAD_S[1] * dt, min(TURN_RAD_S[1] * dt, turn))
    return ready, ready and abs(turn) < FACED_RAD and settled(director)


class Way:

    """A path [(x, z), ..] from where she is, and where she is on it: `s` m along; `state` -
    'turn' standing, 'walk', 'halt' before a stop; `stops`, the arcs she stands at: its corners
    past FACES_RAD and, where it `ends`, its end."""

    def __init__(self, path, ends=True, walking=False):
        self.path = [tuple(path[0])]
        for p in path[1:]:
            if math.dist(self.path[-1], p) > 0.05:
                self.path.append(tuple(p))
        self.arc = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.arc.append(self.arc[-1] + math.dist(a, b))
        ways = [bearing(a, b) for a, b in zip(self.path, self.path[1:])]
        self.stops = [self.arc[i + 1] for i, (p, q) in enumerate(zip(ways, ways[1:]))
                      if abs(hold.wrapped(q - p)) > FACES_RAD]
        self.ends = ends
        if ends:
            self.stops.append(self.arc[-1])
        self.s, self.passed, self.since, self.ready = 0.0, -1.0, 0.0, False
        self.state = 'walk' if walking else 'turn'

    def at(self, s):
        """The point `s` m along, past its end on along its last stretch."""
        if len(self.path) < 2:
            return self.path[0]
        i = max(1, min(len(self.arc) - 1, bisect.bisect_right(self.arc, s)))
        a, b = self.path[i - 1], self.path[i]
        k = (s - self.arc[i - 1]) / (self.arc[i] - self.arc[i - 1])
        return (a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k)

    def on(self, at):
        """Where `at` is along the path, m: the nearest point of it from `s` to AHEAD_M on."""
        best, near = self.s, math.inf
        for i in range(1, len(self.path)):
            if self.arc[i] < self.s or self.arc[i - 1] > self.s + AHEAD_M:
                continue
            a, b = self.path[i - 1], self.path[i]
            span = self.arc[i] - self.arc[i - 1]
            k = ((at[0] - a[0]) * (b[0] - a[0]) + (at[1] - a[1]) * (b[1] - a[1])) / span ** 2
            k = max(0.0, min(1.0, k))
            far = math.hypot(a[0] + (b[0] - a[0]) * k - at[0], a[1] + (b[1] - a[1]) * k - at[1])
            if far < near:
                best, near = max(self.s, self.arc[i - 1] + k * span), far
        return best

    def rest(self, until, skip=0.0):
        """The path's points from `skip` m on from where she is to `until` m along."""
        s0 = self.s + skip
        if s0 >= until:
            return []
        return ([self.at(s0)] + [p for p, s in zip(self.path, self.arc) if s0 < s < until]
                + [self.at(until)])


def step(way, director, at, dt):
    """The row asked of `director` and its law's heading turned this pass, she at `at` (x, z);
    whether she has come to the path's end - standing there, or past it walking on."""
    law = director.going
    way.s = way.on(at)
    stop = next((s for s in way.stops if s > way.passed), None)
    if way.state == 'halt':
        director.pace = ROW_STAND
        if settled(director):
            if stop is None or (way.ends and stop >= way.arc[-1]):
                return True
            way.passed, way.s = stop, max(way.s, stop)
            way.state, way.ready = 'turn', True
        return False
    aim = way.at(way.s + LOOK_M if stop is None else min(way.s + LOOK_M, stop))
    near = math.dist(at, aim) < NEAR_M
    turn = 0.0 if near else hold.wrapped(bearing(at, aim) - law.heading)
    if way.state == 'turn':
        if not way.ready and abs(turn) <= FACES_RAD and not (near and stop is not None):
            way.state, way.since = 'walk', 0.0      # within the turn she walks: no step round
            return False
        way.ready, faced = turned(director, turn, way.ready, dt)
        if near and stop is not None:               # on a stop as she is asked
            if way.ends and stop >= way.arc[-1]:
                return way.ready
            way.passed = stop
        elif faced:
            way.state, way.since = 'walk', 0.0
        return False
    director.pace = ROW_GO
    way.since = way.since + dt if director.k[0] >= -0.01 else 0.0
    if stop is None and way.s >= way.arc[-1] - 1e-3:
        return True
    if stop is not None and stop - way.s <= STOP_M:
        way.state = 'halt'
    elif abs(turn) > OFF_RAD:
        way.state, way.ready = 'turn', False
    elif way.since >= TURNS_S:
        law.heading += max(-TURN_RAD_S[0] * dt, min(TURN_RAD_S[0] * dt, turn))
    return False
