"""Automated house tour and synthetic maneuver generator for the gynoid inside the house.

Generates synthetic WASD keyboard chords, mouselook (yaw/pitch) controls, and stunt interactions
to navigate the glass room, visit props, and perform all operations from the stunt repertoire:
  1. Surveying house with stereo cameras / vision scanner
  2. WASD maneuvering to chair, sitting down, rising, hopping up and down
  3. WASD maneuvering to bed, lying down on compliant mattress, rising
  4. WASD maneuvering to floor mattress, lying down, rising
  5. Traversing rug and doorway sill with stumble reflex
  6. Pushing through swinging door into outside area
  7. Compliantly contacting tree and wall without falling
  8. Navigating slick ice and snow terrain
  9. Crossing 10cm hole without falling
 10. Ascending and descending stairs
 11. Absorbing moderate shove with parry and severe shove with controlled breakfall
"""
import math
import time

#: Step definitions for the automated house tour: (label, duration_s, commands)
#: commands is a list of tuples: (rel_time_s, 'key'|'mouse'|'push'|'scenario', value)
STEPS = [
    (
        'Survey glass room with stereo cameras',
        2.0,
        [
            (0.0, 'scenario', 'glass_room'),
            (0.2, 'mouse', (25.0, -5.0)), # Look towards chair
            (1.0, 'mouse', (-30.0, -5.0)),# Look towards bed
            (1.8, 'mouse', (0.0, 0.0)),   # Center view
        ]
    ),
    (
        'Maneuver to chair and sit down',
        4.5,
        [
            (0.0, 'scenario', 'chair'),
            (0.1, 'key', 'sit_chair'),    # Automatically approaches chair and sits down
        ]
    ),
    (
        'Rise from chair to standing',
        2.5,
        [
            (0.1, 'key', 'rise_chair'),    # Rise from chair to standing
        ]
    ),
    (
        'Hop up onto chair',
        3.0,
        [
            (0.1, 'key', 'hop_up_chair'),  # Hop up onto chair (jämfota)
        ]
    ),
    (
        'Hop down from chair',
        2.5,
        [
            (0.1, 'key', 'hop_down_chair'),# Hop down from chair (kontrollerat)
        ]
    ),
    (
        'Maneuver to bed and lie down',
        3.5,
        [
            (0.0, 'scenario', 'bed'),
            (0.1, 'key', 'lie_bed'),      # Approaches bed and reclines
            (2.4, 'key', 'rise_bed'),     # Rise from bed to standing
        ]
    ),
    (
        'Maneuver to floor mattress and lie down',
        3.5,
        [
            (0.0, 'scenario', 'flat'),
            (0.1, 'key', 'a'),            # WASD strafe left
            (0.8, 'key', 'lie_floor'),    # Lie on floor mattress
            (2.4, 'key', 'rise_floor'),   # Rise from floor
        ]
    ),
    (
        'Traverse rug and doorway sill threshold',
        3.0,
        [
            (0.0, 'scenario', 'rug'),
            (0.1, 'key', 'w'),            # Walk across rug
            (1.5, 'scenario', 'sill'),    # Doorway sill threshold
            (1.6, 'key', 'w'),            # Stumble reflex over sill
        ]
    ),
    (
        'Push open swinging door to exit house',
        3.0,
        [
            (0.0, 'scenario', 'glass_room'),
            (0.1, 'key', 'w'),            # Walk through doorway
            (1.5, 'key', 'w'),            # Push through swinging door
        ]
    ),
    (
        'Compliant collision into tree and wall',
        3.5,
        [
            (0.0, 'scenario', 'obstacle'),
            (0.1, 'key', 'w'),            # Contact tree compliantly
            (1.5, 'key', 'a'),            # Strafe left away from tree
            (2.5, 'key', 'w'),            # Lean into wall compliantly
        ]
    ),
    (
        'Navigate slick ice and snow terrain',
        3.5,
        [
            (0.0, 'scenario', 'ice'),
            (0.1, 'key', 'ice_walk'),     # Ice walk mode (slick surface)
            (1.8, 'scenario', 'snow'),
            (1.9, 'key', 'slope_walk'),   # Traverse snow patch
        ]
    ),
    (
        'Cross 10cm deep hole without falling',
        2.5,
        [
            (0.0, 'scenario', 'hole'),
            (0.1, 'key', 'w'),            # Step across 10cm hole
        ]
    ),
    (
        'Ascend and descend stairs',
        3.5,
        [
            (0.0, 'scenario', 'stairs'),
            (0.1, 'key', 'stairs_walk'),  # Stairs walk up
            (1.8, 'key', 'stairs_down'),  # Stairs descend
        ]
    ),
    (
        'Absorb shove perturbation and breakfall',
        3.5,
        [
            (0.0, 'scenario', 'flat'),
            (0.1, 'push', (80.0, 0.0, 0.0)),  # Moderate shove
            (0.2, 'key', 'parry_shove'),       # Parry shove reflex
            (1.8, 'push', (140.0, 0.0, 0.0)), # Severe shove
            (2.0, 'key', 'controlled_fall'),  # Controlled breakfall
        ]
    ),
]


class HouseTour:

    """Orchestrates an automated walkthrough inside the house and environment."""

    def __init__(self, steps=None, loop=False):
        self.steps = list(steps or STEPS)
        self.loop = loop
        self.step_idx = 0
        self.step_t = 0.0
        self.done = False
        self._dispatched = set()

    def current_label(self):
        """Current high-level tour phase label."""
        if self.done:
            return 'TOUR COMPLETE'
        if 0 <= self.step_idx < len(self.steps):
            return self.steps[self.step_idx][0]
        return 'IDLE'

    def reset(self):
        """Reset tour to beginning."""
        self.step_idx = 0
        self.step_t = 0.0
        self.done = False
        self._dispatched.clear()

    def step(self, state, dt):
        """Advance tour clock by `dt` and dispatch scheduled synthetic keys/mouselook.

        `state` is the view/simulation dictionary containing keys table, body, yaw, pitch, etc.
        """
        if self.done or self.step_idx >= len(self.steps):
            return
        label, duration, events = self.steps[self.step_idx]
        for idx, (ev_t, kind, val) in enumerate(events):
            token = (self.step_idx, idx)
            if self.step_t >= ev_t and token not in self._dispatched:
                self._dispatched.add(token)
                self._dispatch(kind, val, state)

        self.step_t += dt
        if self.step_t >= duration - 1e-5:
            self.step_idx += 1
            self.step_t = 0.0
            if self.step_idx >= len(self.steps):
                if self.loop:
                    self.reset()
                else:
                    self.done = True

    def _dispatch(self, kind, val, state):
        from terminal.views import humanoid_keys
        body = state.get('body')
        if kind == 'key':
            humanoid_keys.log_key(val, f'TOUR: {val}')
            keys_table = state.get('keys') or {}
            # If chord or lowercase key is present
            if val in keys_table:
                keys_table[val](state)
            elif body is not None:
                # Direct fallback to body.send
                if val in ('w', 'W'):
                    body.send(say=3)
                elif val in ('s', 'S'):
                    body.send(say=4)
                elif val in ('a', 'A'):
                    body.send(say='strafe_left')
                elif val in ('d', 'D'):
                    body.send(say='strafe_right')
                elif val in ('wa', 'WA'):
                    body.send(say='turn_left')
                elif val in ('wd', 'WD'):
                    body.send(say='turn_right')
                elif val == '~':
                    was_on = state.get('wasd_mode', False)
                    state['wasd_mode'] = not was_on
                else:
                    body.send(say=val)
        elif kind == 'mouse':
            yaw, pitch = val
            state['yaw'] = yaw
            state['pitch'] = pitch
        elif kind == 'scenario':
            state['scenario'] = val
            if body is not None:
                body.send(scenario=val)
        elif kind == 'push':
            if body is not None:
                body.send(push=val, seconds=0.15)
