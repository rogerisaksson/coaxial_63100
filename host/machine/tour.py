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


def compile_prompt(text, now=None):
    """Compile a natural-language operator prompt into LLM thought and structured gynoid commands."""
    q = (text or '').strip().lower()
    fn = next((fn for words, fn in PROMPT_RULES if any(w in q for w in words)),
              lambda _q, _n: {'thought': 'Walking forward into the room.', 'cmds': [{'twist': WALK}], 'tour': False})
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


def errands():
    """The verbs she walks to a thing for (`start_errand`)."""
    from machine import route
    return tuple(route.docks())


def _fire_errand_stunt(g, gym, kind, heard_say_fn, at=None):
    from machine import maneuver
    g['_errand'] = None
    by = {e[0]: e for e in gym}
    g['_twist_active'], g['_twist_stop_until'], g['_halt_at'], g['has_heading'] = False, 0.0, None, 0.0
    g['twist'] = g['asked'] = STOP
    if kind in by:
        heard_say_fn(g, gym, maneuver.said(g, by[kind], at))


def start_errand(g, gym, sc, heard_fn, heard_say_fn, at=None):
    """Begin an autonomous SE(2) room errand (`sit_chair`, `lie_bed`, `exit_door`) from prop geometry,
    to the prop nearest `at` (x, z) - her without one."""
    from machine import furniture, latent, maneuver, room, route, stride
    specs = route.docks()
    if sc not in specs:
        return False
    d_near, px, pz, _top = furniture.nearest(g, sc, at, specs[sc][5])
    if sc != 'exit_door' and not (stride.STRIDE_M < d_near < 3.0 * room.WIDE):
        return False
    g['st'][latent.THETA] *= float(g['st'][latent.NOW][latent.GAIN] >= 1e-6)
    dx_rel, dz_rel, hdd, lat_tol, d_stage, _fb = specs[sc]
    x_dock, z_dock = px + dx_rel, pz + dz_rel
    nx, nz = math.sin(hdd), math.cos(hdd)
    cells = route.blocked(g)
    d_stage = d_stage if sc == 'exit_door' else route.staged((x_dock, z_dock), (nx, nz), d_stage, cells)
    stage = (x_dock + d_stage * nx, z_dock + d_stage * nz)
    way, curve, held = ([], (), ()) if sc == 'exit_door' else route.planned(g, stage, cells)
    g['_errand'] = {
        'kind': sc, 'phase': 'door' if sc == 'exit_door' else 'walk',
        'x_dock': x_dock, 'z_dock': z_dock, 'nx': nx, 'nz': nz, 'tx': nz, 'tz': -nx,
        'xd': stage[0], 'zd': stage[1],
        'hdd': hdd, 'lat_tol': lat_tol, 'd_stage': d_stage, 'turn_dir': 0.0,
        'standoff': maneuver.standoff(sc, g['suite']) if sc in maneuver.PLAYED else 0.0,
        'way': way, 'curve': curve, 'k': 0, 'held': held,
    }
    step_errand(g, gym, heard_fn, heard_say_fn)
    return True


def _tell(g, gym, heard_fn, cmd):
    """`cmd` as the errand's word to her, once: the same twist again would restart her gait."""
    if g.get('asked') != cmd.get('twist'):
        heard_fn(g, gym, dict(cmd, _errand_cmd=True))


def _told(twist):
    """A row's word: `twist`, her heading the dock's."""
    return lambda g, gym, er, c, heard_fn, _say_fn: _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': twist})


def _stood(g, gym, er, c, heard_fn, _say_fn):
    """Walked to the box: stood, her heading released, her turn's way forgotten."""
    er['turn_dir'], g['has_heading'] = 0.0, 0.0
    _tell(g, gym, heard_fn, {'twist': STOP})


def _cornered(g, gym, er, c, heard_fn, _say_fn):
    """Stood still on her way round the furniture: the rest of it planned from where she stands, its
    corners within a stride of her passed - her stage the last."""
    from machine import route, stride
    er['way'], er['curve'], er['held'] = route.planned(g, (er['xd'], er['zd']), route.blocked(g), er['held'], stride.STRIDE_M)
    er['k'], er['turn_dir'] = 0, 0.0


def _faced(g, gym, er, c, heard_fn, _say_fn):
    """Turned to face away from the prop: stood with the dock's heading, the turn's way forgotten."""
    er['turn_dir'] = 0.0
    _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': STOP})


def _ended(g, gym, er, c, heard_fn, _say_fn):
    """Out of the door: the errand over, stood."""
    g['_errand'], g['has_heading'] = None, 0.0
    _tell(g, gym, heard_fn, {'twist': STOP})


def _stunt(g, gym, er, c, heard_fn, heard_say_fn):
    """Docked and still: the stunt begins on the furniture as she sees it, the errand over."""
    _fire_errand_stunt(g, gym, er['kind'], heard_say_fn, (er['x_dock'], er['z_dock']))


def _nothing(g, gym, er, c, heard_fn, heard_say_fn):
    return None


def _to_door(g, gym, er, c, heard_fn, _say_fn):
    """Walking out: her heading at the doorway's middle on the door's line, her stage ahead at
    the least; the errand over that stage past the line. Aimed at that end a room's height
    ahead she met the post at x 0.5 for the middle's 0.75 and pushed it 30 s on Linux; her
    stopping distance ahead she swung to x 1.18 at the other post."""
    g['has_heading'] = 1.0
    g['target_heading'] = math.atan2(er['xd'] - c['gx'], max(er['z_dock'], c['gz'] + er['d_stage']) - c['gz'])
    _tell(g, gym, heard_fn, {'twist': WALK})


def _steer(g, gym, heard_fn, err, wz_dir, speed, off, lag):
    """Turn in place toward `err` rad while she is `off` her way past her list, standing, or past
    `route.CONE` as far as her rhythm runs (`latent.GAIN`) toward `speed`; within it, walk `speed`,
    on her arc while she `lag`s `route.ARCED` or more, else by `err`."""
    from machine import edge, latent, modal, route, se2
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    turn = abs(off) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and abs(lag) >= route.ARCED
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(edge.yaw(g['sense']) + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, math.copysign(se2.YAW_RATE, lag))), (True, (speed, 0.0, 0.0)))
    _tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})


def _to_aim(g, gym, er, c, heard_fn, _say_fn):
    """Walking to her aim (`step_errand`'s); her turn's way held while it is all but behind her."""
    from machine import modal, se2
    err_w, behind = c['err_w'], abs(c['err_w']) > math.pi - modal.LIST_RAD
    er['turn_dir'] = (er['turn_dir'] or math.copysign(1.0, err_w)) if behind else math.copysign(1.0, err_w)
    _steer(g, gym, heard_fn, err_w, er['turn_dir'], se2.WALK_SPEED, c['off_w'], c['lag'])


def _to_face(g, gym, er, c, heard_fn, _say_fn):
    """Turning in place to face away from the prop, the shorter way, held once begun."""
    er['turn_dir'] = er['turn_dir'] or math.copysign(1.0, c['err_yaw'])
    _steer(g, gym, heard_fn, c['err_yaw'], er['turn_dir'], 0.0, c['err_yaw'], 0.0)


def _to_line(g, gym, er, c, heard_fn, _say_fn):
    """Sidestepping toward the dock's line, facing away from the prop."""
    from machine import se2
    g['has_heading'], g['target_heading'] = 1.0, er['hdd']
    _tell(g, gym, heard_fn, {'heading': er['hdd'], 'twist': (0.0, math.copysign(se2.STRAFE_SPEED, c['way_left']), 0.0)})


def _to_edge(g, gym, er, c, heard_fn, _say_fn):
    """Backing onto the edge, her heading bent toward the dock's line as it bears from her stage:
    bent as the dock bears from her she came to it 0.5 rad askew and sat on its edge."""
    g['has_heading'] = 1.0
    g['target_heading'] = er['hdd'] + math.atan2(c['s_perp'], er['d_stage'])


def _on_rows():
    """The errand's phases, a row each: (on what -> the phase next, told what) in order, the first
    true row taken; and what she is told while none is. Read: walking, she stands where her
    stopping point enters the box; stood still (`idle`), she walks again past her stopping
    distance from it, turns to face away from the prop, sidesteps while her stop would end past
    the dock's tolerance off its line till that stop is on it (read where she stood, it flipped
    each pass; left 0.13 m off, her back-walk drifted her out), backs till her heels are the stunt's standoff off the edge
    (`maneuver.standoff`), and begins the stunt within both, else settles again: begun off
    them she sat on 54-60 of 100, within them 79-90."""
    from machine import figure, modal, se2, stride
    back = (-se2.BACK_SPEED, 0.0, 0.0)
    return {
        'door': (((lambda c, er: c['s_par'] > er['d_stage'] - 1.5 * stride.STRIDE_M, 'door', _ended),), _to_door),
        'walk': (((lambda c, er: c['corner'], 'corner', _stood), (lambda c, er: c['stops_in'], 'settle', _stood)), _to_aim),
        'corner': (((lambda c, er: c['idle'] and c['still'], 'walk', _cornered),), _nothing),
        'turn': (((lambda c, er: abs(c['err_yaw']) <= modal.LIST_RAD, 'settle', _faced),), _to_face),
        'settle': (((lambda c, er: c['idle'] and c['s_par'] > er['d_stage'] + se2.STOP_M, 'walk', _nothing),
                   (lambda c, er: c['idle'] and abs(c['err_yaw']) > modal.LIST_RAD, 'turn', _nothing),
                   (lambda c, er: c['idle'] and abs(c['s_perp_side']) > er['lat_tol'], 'side', _nothing),
                   (lambda c, er: c['idle'] and c['s_par'] > figure.HEEL + er['standoff'] + se2.BACK_STOP_M, 'back', _told(back)),
                   (lambda c, er: c['idle'] and c['still'], 'done', _stunt)), _nothing),
        'side': (((lambda c, er: c['s_perp_side'] * c['s_perp'] <= 0.0, 'settle', _told(STOP)),), _to_line),
        'back': (((lambda c, er: c['s_par'] <= figure.HEEL + er['standoff'] + se2.BACK_STOP_M, 'done', _told(STOP)),), _to_edge),
        'done': (((lambda c, er: c['idle'] and (abs(c['err_yaw']) > modal.LIST_RAD or abs(c['s_perp_side']) > er['lat_tol']), 'settle', _nothing), (lambda c, er: c['idle'] and c['still'], 'done', _stunt)), _nothing),
    }


def _settled(g):
    """m, positive where her latent loop stands her still enough for a stunt to begin: both soles
    stand, her capture point `xi = c + v / omega` inside their hull by more than her rhythm sways
    her, her drift `v / omega` within a sole's half-length."""
    from machine import figure, latent, modal, poise
    y, fr, a, sway = g['y'], g['fr'], g['a'], g['out'][latent.SWAY]
    rot = fr[modal.ROT].reshape(3, 3)
    c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
    drift = v[:2] / float(fr[modal.OMEGA])
    xi = c[:2] + drift
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    inside = min(float((xi - lo.min(axis=0)).min()), float((hi.max(axis=0) - xi).min()))
    swayed = math.hypot(float(sway[modal.DRIVE]), float(sway[modal.LATERAL]))
    return min(inside - swayed, figure.SOLE_HALF - math.hypot(*drift)) * float(a[modal.STANCE].prod())


def step_errand(g, gym, heard_fn, heard_say_fn):
    """Advance an active room errand (`sit_chair`, `lie_bed`, `exit_door`): her place in the dock's
    frame, her stop, her aim - her curve, her next stop, else its normal line -, the row
    (`_on_rows`)."""
    from machine import edge, latent, route, se2, sight, stride
    er = g.get('_errand')
    if not er:
        return
    rows, meanwhile = _on_rows()[er['phase']]
    d = g['world'].data
    gx, gz = float(d.qpos[0]), float(d.qpos[2])
    vx_w, vz_w = float(g['est']['vel'][0]), float(g['est']['vel'][2])
    pyaw = se2.wrap_angle(edge.yaw(g['sense']))
    e = er
    dx_d, dz_d = gx - e['x_dock'], gz - e['z_dock']
    s_par, s_perp = dx_d * e['nx'] + dz_d * e['nz'], dx_d * e['tx'] + dz_d * e['tz']
    t_stop = se2.STOP_M / se2.WALK_SPEED
    s_par_stop = s_par + (vx_w * e['nx'] + vz_w * e['nz']) * t_stop
    s_perp_stop = s_perp + (vx_w * e['tx'] + vz_w * e['tz']) * t_stop
    s_perp_side = s_perp + (vx_w * e['tx'] + vz_w * e['tz']) * se2.SIDE_STOP_M / se2.STRAFE_SPEED
    a_par = max(s_par - se2.STOP_M, e['d_stage'])
    stop = e['way'][0] if e['way'] else (e['x_dock'] + a_par * e['nx'], e['z_dock'] + a_par * e['nz'])
    e['k'], (ax, az), along, there = (route.ahead(e['curve'], e['k'], (gx, gz)) if len(e['curve'])
                                      else (e['k'], stop, math.atan2(stop[0] - gx, stop[1] - gz), pyaw))
    gap, closing = se2.closing(vx_w, vz_w, stop[0] - gx, stop[1] - gz)
    ctx = {
        'gx': gx, 'gz': gz, 'pyaw': pyaw, 's_par': s_par, 's_perp': s_perp, 's_perp_stop': s_perp_stop, 's_perp_side': s_perp_side,
        'way_left': -s_perp * (e['tx'] * math.cos(pyaw) - e['tz'] * math.sin(pyaw)),
        'stops_in': not e['way'] and (s_par_stop <= e['d_stage'] and abs(s_perp_stop) <= e['lat_tol']
                                      or math.hypot(s_par_stop - e['d_stage'], s_perp_stop) <= stride.STRIDE_M),
        'corner': bool(e['way']) and gap <= closing * t_stop + sight.CELL_M,
        'err_w': se2.wrap_angle(math.atan2(ax - gx, az - gz) - pyaw), 'off_w': se2.wrap_angle(along - pyaw),
        'lag': se2.wrap_angle(there - pyaw),
        'err_yaw': se2.wrap_angle(e['hdd'] - pyaw),
        'idle': _settled(g) > 0.0,
        'still': float(g['st'][latent.NOW][latent.GAIN]) < 1e-6,
    }
    nxt, say = next(((phase, say) for on, phase, say in rows if on(ctx, e)), (e['phase'], meanwhile))
    e['phase'] = nxt
    say(g, gym, e, ctx, heard_fn, heard_say_fn)
