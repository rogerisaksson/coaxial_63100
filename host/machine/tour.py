"""The tour: her scenes one after another, commanded as an agent commands her.

Each step lays a scene and sends what a player's keys or an agent send - a twist, a verb, a
push - at its seconds; the view looks where it is told. Nothing in it presses a key.
"""
import bisect
import itertools
import math

#: (twists, m/s and rad/s) a walk, a stop, an arc each way.
WALK, STOP, LEFT, RIGHT = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0), (0.69, 0.0, 0.7), (0.69, 0.0, -0.7)

#: (label, seconds, [(at s, kind, value)]): kind 'scene', 'twist', 'say', 'push' (N, from
#: the view's side), 'look' ((yaw, pitch) deg).
STEPS = [
    ('Look round the lodge', 4.0, [
        (0.0, 'scene', 'lodge'), (0.5, 'look', (205.0, 8.0)), (2.0, 'look', (155.0, 8.0)),
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

_CMD_FMT = {
    'twist': lambda v: 'twist=(%+.2f,%+.2f,%+.2f)' % (float(v[0]), float(v[1]), float(v[2])),
    'push': lambda v: 'push=%.0fN' % float(v),
    'look': lambda v: 'look=(%.0f,%.0f)' % (float(v[0]), float(v[1])),
    'entries': lambda v: 'play=%s' % ','.join(str(e[0]) for e in v),
}

_TWIST_THOUGHTS = {
    STOP: 'Brake and settle to a stand.',
    LEFT: 'Arc left around the room.',
    RIGHT: 'Arc right around the room.',
}


def format_cmd(kind, value=None):
    """Format a dispatched agent/LLM command for the viewport and HUD feed."""
    if isinstance(kind, dict):
        return ' '.join('%s=%s' % (k, format_cmd(k, v).split('=', 1)[-1]) for k, v in kind.items())
    return _CMD_FMT.get(kind, lambda v: '%s=%s' % (kind, v))(value)


def _twist_thought(value, label):
    preset = _TWIST_THOUGHTS.get(tuple(value))
    if preset:
        return preset
    vx, vy = float(value[0]), float(value[1])
    return next(text for cond, text in (
        (vx > 0.9, 'Sprint forward across the floor.'),
        (vx > 0.0, 'Walk forward (%s).' % (label.lower() if label else 'steady gait')),
        (vx < 0.0, 'Walk backward carefully.'),
        (bool(vy), 'Sidestep laterally.'),
        (True, 'Turn in place.'),
    ) if cond)


_KIND_THOUGHTS = {
    'say': lambda v, lbl: _SAY_THOUGHTS.get(v, lbl or ('say %s' % v)),
    'twist': _twist_thought,
    'scene': lambda v, lbl: 'Prepare room scene: %s (%s).' % (v, lbl.lower() or v),
    'push': lambda v, _lbl: 'Apply %.0f N external disturbance.' % float(v),
    'look': lambda v, _lbl: 'Pan stereo gaze to yaw %.0f deg.' % float(v[0]),
}


def thought_for(kind, value, label=''):
    """Natural-language LLM directive describing `(kind, value)` within `label`."""
    return _KIND_THOUGHTS.get(kind, lambda v, lbl: lbl or ('%s %s' % (kind, v)))(value, label)


_RISE_WORDS = ('rise', 'up', 'stand', 'wake')
_DOING_RISE = {
    'seated': 'rise_chair', 'sit_chair': 'rise_chair', 'approach': 'rise_chair', 'lower': 'rise_chair',
    'bed_rest': 'rise_bed', 'lie_bed': 'rise_bed', 'sit_edge': 'rise_bed', 'recline': 'rise_bed',
    'floor_rest': 'rise_floor', 'lie_floor': 'rise_floor', 'kneel': 'rise_floor',
}


def _say_plan(verb):
    return {'thought': _SAY_THOUGHTS[verb], 'cmds': [{'say': verb}], 'tour': False}


def _stop_plan(_q, now):
    verb = _DOING_RISE.get(str((now or {}).get('doing', '')), 'stand')
    cmd = {'say': verb} if verb != 'stand' else {'twist': STOP, 'say': 'stand'}
    return {'thought': _SAY_THOUGHTS.get(verb, _SAY_THOUGHTS['stand']), 'cmds': [cmd], 'tour': False}


PROMPT_RULES = (
    (('tour', 'bustle', 'explore', 'around the room', 'show me'),
     lambda _q, _n: {'thought': 'Starting autonomous LLM tour of the room.', 'cmds': [], 'tour': True}),
    (('door', 'exit', 'outside', 'leave', 'go out'),
     lambda _q, _n: _say_plan('exit_door')),
    (('armchair', 'chair', 'sit'),
     lambda q, _n: _say_plan('rise_chair' if any(w in q for w in _RISE_WORDS) else 'sit_chair')),
    (('mattress', 'floor'),
     lambda q, _n: _say_plan('rise_floor' if any(w in q for w in _RISE_WORDS) else 'lie_floor')),
    (('bed', 'lie', 'sleep', 'rest'),
     lambda q, _n: _say_plan('rise_bed' if any(w in q for w in _RISE_WORDS) else 'lie_bed')),
    (('jump', 'hop', 'leap'),
     lambda q, _n: {'thought': _SAY_THOUGHTS['hop'],
                    'cmds': [{'charge': 0.0 if ('short' in q or 'small' in q) else 1.0, 'say': 'hop'}], 'tour': False}),
    (('crouch', 'squat', 'duck'),
     lambda _q, _n: _say_plan('crouch')),
    (('run', 'sprint', 'jog'),
     lambda _q, _n: {'thought': 'Sprinting forward at 1.40 m/s.', 'cmds': [{'twist': (1.40, 0.0, 0.0)}], 'tour': False}),
    (('back', 'reverse'),
     lambda _q, _n: {'thought': 'Walking backward at 0.35 m/s.', 'cmds': [{'twist': (-0.35, 0.0, 0.0)}], 'tour': False}),
    (('sidestep', 'strafe'),
     lambda q, _n: {'thought': 'Sidestepping %s at 0.50 m/s.' % ('right' if 'right' in q else 'left'),
                    'cmds': [{'twist': (0.0, -0.50 if 'right' in q else 0.50, 0.0)}], 'tour': False}),
    (('turn', 'spin'),
     lambda q, _n: {'thought': 'Turning %s in place.' % ('right' if 'right' in q else 'left'),
                    'cmds': [{'twist': (0.0, 0.0, -0.70 if 'right' in q else 0.70)}], 'tour': False}),
    (('stop', 'halt', 'stand', 'rise', 'get up'), _stop_plan),
)


def _knows(q, words):
    """Whether a rule's `words` name the prompt `q`: a phrase within it, a word among its words -
    'run' within 'runt' sprinted her at a prompt that said nothing she knows."""
    toks = q.split()
    return any((w in q) if ' ' in w else (w in toks) for w in words)


def compile_prompt(text, now=None):
    """Compile an operator prompt into LLM thought and structured gynoid commands - English words;
    the operator's own language is englished before this (`coaxial_ollama.gynoid_words`). Words no
    rule knows move nothing: the unmatched default walked her 0.69 m/s into the wall."""
    q = (text or '').strip().lower()
    fn = next((fn for words, fn in PROMPT_RULES if _knows(q, words)),
              lambda _q, _n: {'thought': 'No words I know in that - the chat hint lists what I take.',
                              'cmds': [], 'tour': False})
    return fn(q, now)


class HouseTour:

    """The tour's precompiled 1D timeline, LLM thought stream, and dispatched event log."""

    def __init__(self, steps=None, loop=False):
        self.steps = list(steps or STEPS)
        self.loop = loop
        self._ends = tuple(itertools.accumulate(float(s[1]) for s in self.steps))
        self._starts = (0.0,) + self._ends[:-1]
        self.total_s = self._ends[-1] if self._ends else 0.0
        self._events = tuple(
            (t0 + float(at), (s_idx, e_idx), label, kind, val)
            for s_idx, (t0, (label, _dur, evs)) in enumerate(zip(self._starts, self.steps))
            for e_idx, (at, kind, val) in enumerate(evs)
        )
        self.t = 0.0
        self.done = False
        self._dispatched = set()
        self.history = []
        self.last_thought = 'Starting autonomous LLM tour of the room.'
        self.last_cmd = 'tour=start'

    @property
    def step_idx(self):
        return len(self.steps) if self.done else min(
            bisect.bisect_right(self._ends, self.t - 1e-5), max(0, len(self.steps) - 1))

    @property
    def step_t(self):
        idx = self.step_idx
        return self.t - self._starts[idx] if idx < len(self._starts) else 0.0

    def current_label(self):
        """The step under way."""
        idx = self.step_idx
        return 'TOUR COMPLETE' if self.done else (self.steps[idx][0] if 0 <= idx < len(self.steps) else 'IDLE')

    def reset(self):
        """From the start."""
        self.t, self.done = 0.0, False
        self._dispatched.clear()

    def _emit(self, key, label, kind, value, state):
        self._dispatched.add(key)
        th, cm = thought_for(kind, value, label), format_cmd(kind, value)
        self.last_thought, self.last_cmd = th, cm
        self.history = (self.history + [(th, cm)])[-8:]
        state['llm_last'] = (th, cm)
        state['llm_feed'] = (state.get('llm_feed', []) + [(th, cm)])[-8:]
        rows: list = list(state.get('chat_rows') or ())
        rows.extend([('label', '  tour [%s]: %s' % (label, cm)), (None, th)])
        state['chat_rows'] = rows[-24:]
        sent(kind, value, state)

    def step(self, state, dt):
        """`dt` s on: query the flat timeline table for due events and advance `self.t`."""
        if self.done or not self.steps:
            return
        idx = self.step_idx
        t_end = self._ends[idx]
        due = [ev for ev in self._events if ev[1][0] == idx and ev[0] <= self.t + 1e-9 and ev[1] not in self._dispatched]
        for _at, key, label, kind, value in due:
            self._emit(key, label, kind, value, state)
        t_next = self.t + dt
        self.t = t_end if (t_next >= t_end - 1e-5 and self.t < t_end - 1e-5) else t_next
        over = self.t >= self.total_s - 1e-5
        self.done = over and not self.loop
        if over and self.loop:
            self.reset()


def _send_scene(value, state):
    state['scenario'] = value
    state['body'].send(scenario=value)


def _send_push(value, state):
    yaw = math.radians(state.get('yaw', 180.0))
    state['body'].send(push=(-value * math.sin(yaw), 0.0, -value * math.cos(yaw)), seconds=0.12)


def _send_look(value, state):
    state['yaw'], state['pitch'] = value


_SENT_DISPATCH = {
    'scene': _send_scene,
    'twist': lambda v, st: st['body'].send(twist=v),
    'say': lambda v, st: st['body'].send(say=v),
    'push': _send_push,
    'look': _send_look,
}


def sent(kind, value, state):
    """One of a step's commands: a scene, a twist, a verb or a push to her body; a look to the
    view."""
    _SENT_DISPATCH[kind](value, state)
