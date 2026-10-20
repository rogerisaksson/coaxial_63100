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
    ('Sit on the chair and rise', 14.0, [
        (0.0, 'scene', 'flat'), (0.5, 'say', 'sit_chair'), (8.5, 'say', 'rise_chair')]),
    ('Lie on the bed and rise', 15.0, [
        (0.0, 'scene', 'flat'), (0.5, 'say', 'lie_bed'), (9.5, 'say', 'rise_bed')]),
    ('Lie on the mattress and rise', 14.0, [
        (0.0, 'scene', 'mattress'), (0.5, 'say', 'lie_floor'), (8.0, 'say', 'rise_floor')]),
    ('Out through the door', 10.0, [(0.0, 'scene', 'flat'), (0.5, 'say', 'exit_door')]),
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


_SAY_THOUGHTS = {
    'sit_chair': 'Walk over to the chair and sit down.',
    'rise_chair': 'Stand back up from the chair.',
    'lie_bed': 'Walk over to the bed and lie down.',
    'rise_bed': 'Sit up and rise from the bed.',
    'lie_floor': 'Lower onto the floor mattress and rest.',
    'rise_floor': 'Push up and rise from the mattress.',
    'exit_door': 'Walk to the doorway, push the door open and go out.',
    'hop': 'Charge and execute a two-footed jump.',
    'hop_up_chair': 'Hop up onto the chair seat.',
    'hop_down_chair': 'Hop down from the chair seat.',
    'crouch': 'Lower into a balanced crouch.',
    'stand': 'Settle into a quiet upright stand.',
}


def format_cmd(kind, value):
    """Format a dispatched agent/LLM command for the viewport and HUD feed."""
    if isinstance(kind, dict):
        return ' '.join('%s=%s' % (k, format_cmd(k, v).split('=', 1)[-1]) for k, v in kind.items())
    if kind == 'twist':
        return 'twist=(%+.2f,%+.2f,%+.2f)' % (float(value[0]), float(value[1]), float(value[2]))
    if kind == 'push':
        return 'push=%.0fN' % float(value)
    if kind == 'look':
        return 'look=(%.0f,%.0f)' % (float(value[0]), float(value[1]))
    return '%s=%s' % (kind, value)


def thought_for(kind, value, label=''):
    """Natural-language LLM directive describing `(kind, value)` within `label`."""
    if kind == 'say' and value in _SAY_THOUGHTS:
        return _SAY_THOUGHTS[value]
    if kind == 'twist':
        if value == STOP:
            return 'Brake and settle to a stand.'
        if value == LEFT:
            return 'Arc left around the room.'
        if value == RIGHT:
            return 'Arc right around the room.'
        if value[0] > 0.9:
            return 'Sprint forward across the floor.'
        if value[0] > 0.0:
            return 'Walk forward (%s).' % (label.lower() if label else 'steady gait')
        if value[0] < 0.0:
            return 'Walk backward carefully.'
        return 'Sidestep laterally.' if value[1] else 'Turn in place.'
    if kind == 'scene':
        return 'Prepare room scene: %s (%s).' % (value, label.lower() or value)
    if kind == 'push':
        return 'Apply %.0f N external disturbance.' % float(value)
    if kind == 'look':
        return 'Pan stereo gaze to yaw %.0f deg.' % float(value[0])
    return label or ('%s %s' % (kind, value))


def compile_prompt(text, now=None):
    """Compile a natural-language operator prompt into LLM thought and structured gynoid commands."""
    q = (text or '').strip().lower()
    if any(w in q for w in ('tour', 'bustle', 'explore', 'around the room', 'show me')):
        return {'thought': 'Starting autonomous LLM tour of the room.', 'cmds': [], 'tour': True}
    if any(w in q for w in ('door', 'exit', 'outside', 'leave', 'go out')):
        return {'thought': _SAY_THOUGHTS['exit_door'], 'cmds': [{'say': 'exit_door'}], 'tour': False}
    if any(w in q for w in ('chair', 'sit')):
        verb = 'rise_chair' if any(w in q for w in ('rise', 'up', 'stand')) else 'sit_chair'
        return {'thought': _SAY_THOUGHTS[verb], 'cmds': [{'say': verb}], 'tour': False}
    if any(w in q for w in ('mattress', 'floor')):
        verb = 'rise_floor' if any(w in q for w in ('rise', 'up', 'stand')) else 'lie_floor'
        return {'thought': _SAY_THOUGHTS[verb], 'cmds': [{'say': verb}], 'tour': False}
    if any(w in q for w in ('bed', 'lie', 'sleep', 'rest')):
        verb = 'rise_bed' if any(w in q for w in ('rise', 'up', 'stand', 'wake')) else 'lie_bed'
        return {'thought': _SAY_THOUGHTS[verb], 'cmds': [{'say': verb}], 'tour': False}
    if any(w in q for w in ('jump', 'hop', 'leap')):
        ch = 0.0 if ('short' in q or 'small' in q) else 1.0
        return {'thought': _SAY_THOUGHTS['hop'], 'cmds': [{'charge': ch, 'say': 'hop'}], 'tour': False}
    if any(w in q for w in ('crouch', 'squat', 'duck')):
        return {'thought': _SAY_THOUGHTS['crouch'], 'cmds': [{'say': 'crouch'}], 'tour': False}
    if any(w in q for w in ('run', 'sprint', 'jog')):
        return {'thought': 'Sprinting forward at 1.40 m/s.', 'cmds': [{'twist': (1.40, 0.0, 0.0)}], 'tour': False}
    if any(w in q for w in ('back', 'reverse')):
        return {'thought': 'Walking backward at 0.35 m/s.', 'cmds': [{'twist': (-0.35, 0.0, 0.0)}], 'tour': False}
    if any(w in q for w in ('sidestep', 'strafe')):
        vy = -0.50 if 'right' in q else 0.50
        return {'thought': 'Sidestepping %s at 0.50 m/s.' % ('right' if vy < 0 else 'left'),
                'cmds': [{'twist': (0.0, vy, 0.0)}], 'tour': False}
    if any(w in q for w in ('turn', 'spin')):
        wz = -0.70 if 'right' in q else 0.70
        return {'thought': 'Turning %s in place.' % ('right' if wz < 0 else 'left'),
                'cmds': [{'twist': (0.0, 0.0, wz)}], 'tour': False}
    if any(w in q for w in ('stop', 'halt', 'stand', 'rise', 'get up')):
        doing = str((now or {}).get('doing', ''))
        verb = ('rise_chair' if doing in ('seated', 'sit_chair', 'approach', 'lower')
                else 'rise_bed' if doing in ('bed_rest', 'lie_bed', 'sit_edge', 'recline')
                else 'rise_floor' if doing in ('floor_rest', 'lie_floor', 'kneel') else 'stand')
        cmd = {'say': verb} if verb != 'stand' else {'twist': (0.0, 0.0, 0.0), 'say': 'stand'}
        return {'thought': _SAY_THOUGHTS.get(verb, _SAY_THOUGHTS['stand']), 'cmds': [cmd], 'tour': False}
    return {'thought': 'Walking forward into the room.', 'cmds': [{'twist': (0.69, 0.0, 0.0)}], 'tour': False}


class HouseTour:

    """The tour's clock, LLM thought stream, and what it has sent."""

    def __init__(self, steps=None, loop=False):
        self.steps = list(steps or STEPS)
        self.loop = loop
        self.step_idx = 0
        self.step_t = 0.0
        self.done = False
        self._dispatched = set()
        self.history = []
        self.last_thought = 'Starting autonomous LLM tour of the room.'
        self.last_cmd = 'tour=start'

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
                th, cm = thought_for(kind, value, _label), format_cmd(kind, value)
                self.last_thought, self.last_cmd = th, cm
                self.history = (self.history + [(th, cm)])[-8:]
                state['llm_last'] = (th, cm)
                state['llm_feed'] = (state.get('llm_feed', []) + [(th, cm)])[-8:]
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


def _prop_near(g, kind):
    gx, gz = float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])
    d_min = math.inf
    for p in g['world'].props():
        if len(p) >= 3 and len(p[1]) >= 3:
            px, py, pz = float(p[1][0]), float(p[1][1]), float(p[1][2])
            sz = p[2] if hasattr(p[2], '__len__') and len(p[2]) >= 3 else (0.0, 0.0, 0.0)
            sx, sz_d = min(float(sz[0]), float(sz[2])), max(float(sz[0]), float(sz[2]))
            is_chair = p[0] == 'wood' and 0.35 <= py <= 0.60 and 0.12 <= sx <= sz_d <= 0.35
            is_bed = p[0] == 'bed' and py >= 0.15
            if (kind == 'sit_chair' and is_chair) or (kind == 'lie_bed' and is_bed):
                d_min = min(d_min, math.hypot(px - gx, pz - gz))
    return d_min


def start_errand(g, gym, sc, heard_fn, heard_say_fn):
    """Begin an autonomous room errand (`sit_chair`, `lie_bed`, `exit_door`) when not yet beside the prop."""
    from machine import latent
    if sc == 'exit_door' or (sc in ('sit_chair', 'lie_bed') and 0.85 <= _prop_near(g, sc) < 9.0):
        if g['st'][latent.NOW][latent.GAIN] < 1e-6:
            g['st'][latent.THETA] = 0.0
        g['_errand'] = {'kind': sc, 'phase': 'walk'}
        step_errand(g, gym, heard_fn, heard_say_fn)
        return True
    return False


def step_errand(g, gym, heard_fn, heard_say_fn):
    """Advance an active autonomous room errand (`sit_chair`, `lie_bed`, `exit_door`)."""
    from machine import latent, sprung
    er = g.get('_errand')
    if not er:
        return
    gx, gz = float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])
    spd = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2]))
    kind, ph = er['kind'], er['phase']
    if kind == 'exit_door':
        if gz > 4.1:
            g['_errand'] = None
            heard_fn(g, gym, {'twist': (0.0, 0.0, 0.0), '_errand_cmd': True})
        else:
            g['has_heading'], g['target_heading'] = 1.0, math.atan2(0.72 - gx, max(4.5, gz + 2.2) - gz)
            if not g.get('_twist_active'):
                heard_fn(g, gym, {'twist': (0.69, 0.0, 0.0), '_errand_cmd': True})
        return
    xd, zd, hdd, r_stop = (1.01, 0.44, math.pi, 0.38) if kind == 'sit_chair' else (-1.25, 1.75, math.pi / 2.0, 0.46)
    dist = math.hypot(xd - gx, zd - gz)
    if ph == 'walk':
        if dist > r_stop:
            g['has_heading'], g['target_heading'] = 1.0, math.atan2(xd - gx, zd - gz)
            if not g.get('_twist_active'):
                heard_fn(g, gym, {'twist': (0.69, 0.0, 0.0), '_errand_cmd': True})
        else:
            er['phase'] = 'align'
            heard_fn(g, gym, {'heading': hdd, 'twist': (0.0, 0.0, 0.0), '_errand_cmd': True})
    elif ph == 'align':
        err_hd = abs(math.atan2(math.sin(hdd - g['heading']), math.cos(hdd - g['heading'])))
        gain_max, spd_max = (0.015, 0.12) if kind == 'sit_chair' else (0.08, 0.25)
        if sprung.doing(g) in ('stand', 'rest') and float(g['st'][latent.NOW][latent.GAIN]) < gain_max and spd < spd_max and err_hd < 0.15:
            g['_errand'] = None
            by = {e[0]: e for e in gym}
            g['_twist_active'], g['_twist_stop_until'], g['_halt_at'] = False, 0.0, None
            g['twist'] = g['asked'] = (0.0, 0.0, 0.0)
            if kind in by:
                heard_say_fn(g, gym, by[kind])

