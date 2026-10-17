"""The tour: her scenes one after another, commanded as an agent commands her.

Each step lays a scene and sends what a player's keys or an agent send - a twist, a verb, a
push - at its seconds; the view looks where it is told. Nothing in it presses a key.
"""
import math

#: (twists, m/s and rad/s) a walk, a stop, an arc each way.
WALK, STOP, LEFT, RIGHT = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0), (0.69, 0.0, 0.7), (0.69, 0.0, -0.7)

#: (label, seconds, [(at s, kind, value)]): kind 'scene', 'twist', 'say', 'push' (N, from
#: the view's side), 'look' ((yaw, pitch) deg).
STEPS = [
    ('Look round the house', 4.0, [
        (0.0, 'scene', 'glass_room'), (0.5, 'look', (205.0, 8.0)), (2.0, 'look', (155.0, 8.0)),
        (3.5, 'look', (180.0, 8.0))]),
    ('Sit on the chair and rise', 12.0, [
        (0.0, 'scene', 'chair'), (0.5, 'say', 'sit_chair'), (7.0, 'say', 'rise_chair')]),
    ('Lie on the bed and rise', 14.0, [
        (0.0, 'scene', 'bed'), (0.5, 'say', 'lie_bed'), (8.0, 'say', 'rise_bed')]),
    ('Lie on the mattress and rise', 14.0, [
        (0.0, 'scene', 'mattress'), (0.5, 'say', 'lie_floor'), (8.0, 'say', 'rise_floor')]),
    ('Over the rug', 9.0, [(0.0, 'scene', 'rug'), (0.5, 'twist', WALK), (5.5, 'twist', STOP)]),
    ('Over the sill', 9.0, [(0.0, 'scene', 'sill'), (0.5, 'twist', WALK), (5.5, 'twist', STOP)]),
    ('Into the wall', 9.0, [(0.0, 'scene', 'wall'), (0.5, 'twist', WALK), (4.5, 'twist', STOP)]),
    ('Into the tree', 9.0, [(0.0, 'scene', 'tree'), (0.5, 'twist', WALK), (4.5, 'twist', STOP)]),
    ('On ice', 10.0, [(0.0, 'scene', 'ice'), (0.5, 'twist', WALK), (6.5, 'twist', STOP)]),
    ('On snow', 10.0, [(0.0, 'scene', 'snow'), (0.5, 'twist', WALK), (6.5, 'twist', STOP)]),
    ('Through the hole', 9.0, [(0.0, 'scene', 'hole'), (0.5, 'twist', WALK), (5.5, 'twist', STOP)]),
    ('Up the stairs', 14.0, [(0.0, 'scene', 'stairs'), (0.5, 'twist', WALK), (10.5, 'twist', STOP)]),
    ('Down the stairs', 14.0, [
        (0.0, 'scene', 'stairs_down'), (0.5, 'twist', WALK), (10.5, 'twist', STOP)]),
    ('Weave', 10.0, [
        (0.0, 'scene', 'flat'), (0.5, 'twist', WALK), (1.5, 'twist', LEFT), (2.5, 'twist', WALK),
        (3.5, 'twist', RIGHT), (4.5, 'twist', WALK), (5.5, 'twist', STOP)]),
    ('Shoved, 80 N', 6.0, [(0.0, 'scene', 'flat'), (1.0, 'push', 80.0)]),
]


class HouseTour:

    """The tour's clock and what it has sent."""

    def __init__(self, steps=None, loop=False):
        self.steps = list(steps or STEPS)
        self.loop = loop
        self.step_idx = 0
        self.step_t = 0.0
        self.done = False
        self._dispatched = set()

    def current_label(self):
        """The step under way."""
        if self.done:
            return 'TOUR COMPLETE'
        return self.steps[self.step_idx][0] if 0 <= self.step_idx < len(self.steps) else 'IDLE'

    def reset(self):
        """From the start."""
        self.step_idx, self.step_t, self.done = 0, 0.0, False
        self._dispatched.clear()

    def step(self, state, dt):
        """`dt` s on: what falls due sent to `state['body']`, the view's `state` turned."""
        if self.done or self.step_idx >= len(self.steps):
            return
        _label, duration, events = self.steps[self.step_idx]
        for idx, (at, kind, value) in enumerate(events):
            if self.step_t >= at and (self.step_idx, idx) not in self._dispatched:
                self._dispatched.add((self.step_idx, idx))
                sent(kind, value, state)
        self.step_t += dt
        if self.step_t >= duration - 1e-5:
            self.step_idx, self.step_t = self.step_idx + 1, 0.0
            if self.step_idx >= len(self.steps):
                if self.loop:
                    self.reset()
                else:
                    self.done = True


def sent(kind, value, state):
    """One of a step's commands: a scene, a twist, a verb or a push to her body; a look to the
    view."""
    body = state['body']
    if kind == 'scene':
        state['scenario'] = value
        body.send(scenario=value)
    elif kind == 'twist':
        body.send(twist=value)
    elif kind == 'say':
        body.send(say=value)
    elif kind == 'push':
        yaw = math.radians(state.get('yaw', 180.0))
        body.send(push=(-value * math.sin(yaw), 0.0, -value * math.cos(yaw)), seconds=0.12)
    elif kind == 'look':
        state['yaw'], state['pitch'] = value
