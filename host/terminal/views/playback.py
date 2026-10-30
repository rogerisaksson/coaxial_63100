"""A page's playback: the states a world run apart said, drawn on a clock of their own.

At an even pace, blended between two: hers (`_blended`), a craft's (`terminal.views.quad.flying`).

    playback = Playback()
    playback.push(said)                       # the states its process said since the last frame
    now = playback.at(screen.drawn_at())      # the state to draw, blended, or None before the first
"""
import math

#: The page plays her back LAG_S of her time behind the newest state said, its clock's pace her
#: process's ratio to real time and CATCH_UP of the lag's error a second, eased over PACE_S; a
#: buffer of KEEP_S; a frame plays MOST_S of the wall at the most, STEADY_S held (`steady`).
LAG_S, CATCH_UP, PACE_S, KEEP_S, MOST_S, STEADY_S = 0.06, 1.5, 0.35, 1.0, 0.2, 0.1


class Playback:

    """States said, played back on a clock of their own at an even pace, the frame's state
    `blend`ed between the two it falls between: `push(states)`, `at(wall)`. `catch_up` of the
    lag's error a second; a frame `most_s` of the wall at the most; its pace its world's ratio to
    real time where it `follows` it."""

    def __init__(self, blend=None, catch_up=CATCH_UP, most_s=MOST_S, follows=True):
        self.states, self.shown, self.wall, self.pace = [], None, None, 1.0
        self.blend, self.catch_up, self.most_s, self.follows = blend or _blended, catch_up, most_s, follows

    @classmethod
    def steady(cls, blend=None):
        """For a world held to what its page has shown (`machine.process.apart`): in real time or slower,
        never ahead - no catch-up, a frame STEADY_S at the most, the wall's pace, not its world's
        ratio (held back, that ratio would slow the page that holds it)."""
        return cls(blend, catch_up=0.0, most_s=STEADY_S, follows=False)

    def push(self, states):
        for s in states:
            if self.states and s['t'] < self.states[-1]['t']:
                self.states, self.shown, self.wall = [], None, None
            self.states.append(s)
        if self.states:
            newest = self.states[-1]['t']
            self.states = [s for s in self.states if s['t'] >= newest - KEEP_S]

    def at(self, wall):
        """The state to draw at `wall` seconds, or None before the first."""
        if not self.states:
            return None
        if LAG_S <= 0.0 or len(self.states) < 2:
            self.wall = wall
            self.shown = self.states[-1]['t']
            return self.states[-1]
        newest = self.states[-1]['t']
        if self.shown is None or self.shown > newest or self.shown < self.states[0]['t']:
            self.shown, self.wall = max(self.states[0]['t'], newest - LAG_S), wall
        dt = max(0.0, min(self.most_s, wall - self.wall))
        self.wall = wall
        ratio = min(1.0, max(0.2, float(self.states[-1].get('ratio', 1.0)))) if self.follows else 1.0
        want = ratio + self.catch_up * ((newest - LAG_S) - self.shown)
        self.pace += (want - self.pace) * min(1.0, dt / PACE_S)
        self.shown = min(newest, self.shown + max(0.0, self.pace) * dt)
        after = next((i for i, s in enumerate(self.states) if s['t'] >= self.shown),
                     len(self.states) - 1)
        b = self.states[after]
        a = self.states[max(0, after - 1)]
        span = b['t'] - a['t']
        k = 0.0 if span <= 1e-9 else max(0.0, min(1.0, (self.shown - a['t']) / span))
        return dict(b, t=self.shown, **self.blend(a, b, k))


def _blended(a, b, k):
    """The joints, the pelvis's place, turn, velocity and speed k of the way from state a to b."""
    if math.dist(a['where'], b['where']) > 0.5:
        return {'angles': b['angles'], 'where': b['where'], 'turn': b['turn'],
                'velocity': b.get('velocity', (0.0, 0.0)), 'speed': b['speed']}
    turn = [x + (y - x) * k for x, y in zip(a['turn'], b['turn'])]
    norm = math.sqrt(sum(c * c for c in turn)) or 1.0
    vel_a, vel_b = a.get('velocity', (0.0, 0.0)), b.get('velocity', (0.0, 0.0))
    return {'angles': {j: v + (b['angles'].get(j, v) - v) * k for j, v in a['angles'].items()},
            'where': tuple(x + (y - x) * k for x, y in zip(a['where'], b['where'])),
            'turn': tuple(c / norm for c in turn),
            'velocity': tuple(x + (y - x) * k for x, y in zip(vel_a, vel_b)),
            'speed': a['speed'] + (b['speed'] - a['speed']) * k}

