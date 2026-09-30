"""The humanoid page's playback: her states said, drawn on a clock of their own at an even pace.

    playback = Playback()
    playback.push(said)                       # the states her process said since the last frame
    now = playback.at(time.perf_counter())    # the state to draw, blended, or None before the first
"""
import math

#: The page plays her back LAG_S of her time behind the newest state said, its clock's pace her
#: process's ratio to real time and CATCH_UP of the lag's error a second, eased over PACE_S; a
#: buffer of KEEP_S. Drawn as each state
#: came, a frame at 15 a second showed the same state again 65 times in 235 and the rest 0.036 s
#: of her time apart with 0.022 of spread: her process runs 0.7 of real time in slices of 0.05 s
#: (2026-09-28).
LAG_S, CATCH_UP, PACE_S, KEEP_S = 0.2, 1.0, 0.5, 1.0


class Playback:

    """Her states said, played back on a clock of their own at an even pace, the frame's state
    blended between the two it falls between: `push(states)`, `at(wall)`."""

    def __init__(self):
        self.states, self.shown, self.wall, self.pace = [], None, None, 1.0

    def push(self, states):
        self.states += states
        if self.states:
            newest = self.states[-1]['t']
            self.states = [s for s in self.states if s['t'] >= newest - KEEP_S]

    def at(self, wall):
        """The state to draw at `wall` seconds, or None before the first."""
        if not self.states:
            return None
        newest = self.states[-1]['t']
        if self.shown is None or self.shown > newest or self.shown < self.states[0]['t']:
            self.shown, self.wall = newest - LAG_S, wall
        dt = max(0.0, wall - self.wall)
        self.wall = wall
        ratio = min(1.0, self.states[-1].get('ratio', 1.0))
        want = ratio + CATCH_UP * ((newest - LAG_S) - self.shown)
        self.pace += (want - self.pace) * min(1.0, dt / PACE_S)
        self.shown = min(newest, self.shown + max(0.0, self.pace) * dt)
        after = next((i for i, s in enumerate(self.states) if s['t'] >= self.shown),
                     len(self.states) - 1)
        b = self.states[after]
        a = self.states[max(0, after - 1)]
        span = b['t'] - a['t']
        k = 0.0 if span <= 1e-9 else max(0.0, min(1.0, (self.shown - a['t']) / span))
        return dict(b, t=self.shown, **_blended(a, b, k))


def _blended(a, b, k):
    """The joints, the pelvis's place, turn and speed k of the way from state a to b."""
    turn = [x + (y - x) * k for x, y in zip(a['turn'], b['turn'])]
    norm = math.sqrt(sum(c * c for c in turn)) or 1.0
    return {'angles': {j: v + (b['angles'].get(j, v) - v) * k for j, v in a['angles'].items()},
            'where': tuple(x + (y - x) * k for x, y in zip(a['where'], b['where'])),
            'turn': tuple(c / norm for c in turn), 'speed': a['speed'] + (b['speed'] - a['speed']) * k}
