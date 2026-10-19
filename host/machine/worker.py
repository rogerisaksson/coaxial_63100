"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import math
import queue
import time
from machine import apart, edge, floor, ortho, physics, sprung, wear
from machine.errors import MachineError
from machine.se2 import WALK_ON


def _start_mode(g, by, entry, at):
    from machine import latent, leap, se2
    name = entry[0]
    g['active_entry'] = name
    if name == 'hop':
        wire, names, dur, dt_load_ms = leap.standing_leap(entry[1], float(g.get('_hop_charge', 0.0)))
        g['_sync_cpg_at'], g['st'][latent.THETA] = None, ((0.10 - dt_load_ms * 0.001) % 1.0) * math.tau
        g['_jump_t0'], g['_jump_throw'], g['_jump_until'], g['_leap'] = at, at + dt_load_ms * 0.001, at + dur, False
        sprung.said(g, 'hop', wire, names, at=at, reset=True)
        if '#201' in by:
            sprung.said(g, *by['#201'], at=at + dur + 0.01, reset=True)
        return
    iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
    if name in se2.RESET_MODES:
        return sprung.said(g, *entry, at=at, reset=True)
    if (name in ('turn_left', 'turn_right') and g.get('_twist_active')) or name == '#202':
        return sprung.said(g, *entry, at=at)
    if name == '#203' and '#202' in by:
        sprung.said(g, *by['#202'], at=at)
        return sprung.said(g, *by['#203'], at=at + 0.80)
    if name in se2.INIT_GAITS and iw is not None:
        sprung.said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
        dt_iw = se2.INIT_DELAYS.get(name, se2.INIT_DELAYS['twist' if g.get('_twist_active') else 'say'])
        return sprung.said(g, *by[name], at=at + dt_iw)
    sprung.said(g, *entry, at=at)


def heard_say(g, gym, entry):
    from machine import latent, leap, modal, se2, stride
    by, name = {e[0]: e for e in gym}, entry[0]
    if g.get('_jump_until', 0.0) > g['t']:
        if name == 'hop':
            return leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, float(g.get('_hop_charge', 0.0)))
        if name == g.get('active_entry') and any(w > g['_jump_until'] for w, _ in g['labels']):
            return
        t_res = g['_jump_until'] + 0.02
        g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
        g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
        g['active_entry'] = name
        if name in ('#204', 'jog'):
            sprung.said(g, name, se2._WALK_ON_WIRE, (name,), at=t_res)
        elif name == '#201' and g.get('_leap') and '_brake_wire' in g:
            sprung.said(g, 'brake', g['_brake_wire'], ('brake',), at=t_res - 0.01, reset=True)
            sprung.said(g, *by['#201'], at=t_res + 0.54, reset=True)
        elif name in by:
            sprung.said(g, *by[name], at=t_res, reset=(name == '#201'))
        return
    if name == g.get('active_entry') and (name == '#201' or sprung.doing(g) not in ('rest', '#201')
                                          or bool(len(g['tab']) and (g['tab'][:, latent.AT] >= g['t']).any())):
        return
    striding = (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08 or g['st'][latent.NOW][latent.GAIN] > 0.02)
    lph, lamp = float(g['st'][latent.NOW][latent.PHASE][modal.LATERAL]), float(g['st'][latent.NOW][latent.AMP][modal.LATERAL])
    back_now = (math.sin(lph) * (1.0 if lamp >= 0.0 else -1.0)) < -0.2
    spd, ref = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2])), g['out'][latent.REF]
    t0, cur_d = g['t'] + 0.001, sprung.doing(g)
    off_c = not striding and cur_d not in ('#201', 'rest') and (
        max(abs(float(ref[modal.LIFT])), abs(float(ref[modal.SAGITTAL])), abs(float(ref[modal.LATERAL]))) > 0.15
        or abs(float(ref[modal.AXIAL])) > 0.25 or abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0)
    future = g['tab'][g['tab'][:, latent.AT] > g['t']]
    if cur_d in ('brake', '#201', 'init_walk') and g.get('active_entry') in sprung._FWD_GAITS and name in sprung._FWD_GAITS and len(future):
        t_end = float(future[0, latent.AT])
        t_last = min(t_end, g['t'] + 0.18) if (cur_d == 'init_walk' and name in ('jog', 'turn_left', 'turn_right')) else t_end
        g['tab'], g['labels'] = g['tab'][g['tab'][:, latent.AT] < t_end - 1e-4], [(w, n) for w, n in g['labels'] if w < t_end - 1e-4]
        g['active_entry'] = name
        return sprung.said(g, '#204', se2._WALK_ON_WIRE, ('#204',), at=t_last) if name == '#204' else sprung.said(g, *entry, at=t_last)
    if cur_d in se2.HOLD_PHASES and len(g['tab']):
        g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['t'] + 1.5]
        t0, off_c = (float(g['tab'][-1, latent.AT]) if len(g['tab']) else g['t']) + 0.05, True
    else:
        sprung._dropped(g)
    ae = g.get('active_entry')
    strafe_opp = ({name, ae} == {'strafe_left', 'strafe_right'}
                  or (((name in ('strafe_left', 'strafe_right')) != (ae in ('strafe_left', 'strafe_right'))) and spd > 0.28))
    same = cur_d not in ('brake', '#201') and not strafe_opp and ((name in sprung._FWD_GAITS and not back_now) or (name in sprung._BACK_GAITS and back_now))
    if name in ('parry_shove', 'controlled_fall'):
        g['active_entry'] = name
        return sprung.said(g, *entry, at=t0, reset=True)
    if (striding or g.get('_twist_active')) and name == 'hop' and '#201' in by:
        return se2.stride_hop(g, by, t0, spd, back_now, sprung._FWD_GAITS, sprung._BACK_GAITS, sprung.said)
    if striding and '#201' in by and not same:
        import numpy as np
        sprung.said(g, 'brake', sprung._BRAKE_BACK_WIRE if back_now else sprung._BRAKE_WIRE, ('brake',), at=t0, reset=True)
        g['active_entry'] = name
        if name != '#201' and name in (sprung._FWD_GAITS + sprung._BACK_GAITS):
            if name in sprung._BACK_GAITS:
                dt_b = se2.BRAKE_DELAYS['back_slow' if spd <= 0.55 else 'back_fast']
                rb = np.frombuffer(entry[1], latent.WIRE)
                if len(rb) > 1:
                    rb = rb[1:].copy()
                    rb['at'] -= rb['at'][0]
                    return sprung.said(g, name, rb.tobytes(), entry[2][1:], at=t0 + dt_b, reset=True)
                return sprung.said(g, *entry, at=t0 + dt_b, reset=True)
            dt_b = se2.BRAKE_DELAYS['fwd_slow' if spd <= 0.55 else 'fwd_fast']
            return (sprung.said(g, '#204', se2._WALK_ON_WIRE, ('#204',), at=t0 + dt_b, reset=True) if name == '#204'
                    else sprung.said(g, *entry, at=t0 + dt_b, reset=True))
        ts, tg = se2.BRAKE_DELAYS['stand_fast' if spd > 0.45 else 'stand_slow']
        sprung.said(g, *by['#201'], at=t0 + ts, reset=True)
        if name != '#201':
            g['_sync_cpg_at'] = t0 + tg
            _start_mode(g, by, entry, at=t0 + tg)
        return
    if striding and same:
        if g.get('active_entry') != name:
            g['active_entry'] = name
            rst = name in ('jog', '#204') and ae in ('strafe_left', 'strafe_right', 'turn_left', 'turn_right')
            sprung.said(g, '#204', se2._WALK_ON_WIRE, ('#204',), at=t0, reset=rst) if name == '#204' else sprung.said(g, *entry, at=t0, reset=rst)
        return
    if cur_d == '#202' and name in (('#203',) + sprung._FWD_GAITS):
        sprung.said(g, *by['#203'], at=t0, reset=True)
        return sprung.said(g, *by[name], at=t0 + 0.90) if (name != '#203' and name in by) else None
    if cur_d in ('over', 'touch') and name != 'one_sole':
        sprung.said(g, 'touch', sprung._TOUCH_WIRE, ('touch',), at=t0)
        sprung.said(g, *by['#201'], at=t0 + 0.80, reset=True)
        if name != '#201':
            g['_sync_cpg_at'] = t0 + 2.00
            _start_mode(g, by, entry, at=t0 + 2.00)
        return
    if cur_d in se2.RISES and name in (sprung._FWD_GAITS + sprung._BACK_GAITS) and se2.RISES[cur_d][0] in by:
        rm, dt_r = se2.RISES[cur_d]
        sprung.said(g, *by[rm], at=t0, reset=True)
        g['_sync_cpg_at'] = t0 + dt_r
        return _start_mode(g, by, entry, at=t0 + dt_r)
    if off_c and '#201' in by and name not in (('#201', 'hop') + tuple(sprung.STUNT_MODES)):
        dt_s = 0.55 if cur_d == 'down' else 1.35
        sprung.said(g, *by['#201'], at=t0, reset=True)
        g['_sync_cpg_at'] = t0 + dt_s
        return _start_mode(g, by, entry, at=t0 + dt_s)
    if not striding:
        if name != '#201':
            g['_sync_cpg_at'] = t0
        return _start_mode(g, by, entry, at=t0)
    g['active_entry'] = name
    sprung.said(g, *entry, at=t0)


def heard(g, gym, command):
    from machine import latent, leap, modal, se2, stride, wbc
    unknown = sorted(set(command) - set(sprung.HEARD))
    if unknown:
        raise MachineError('the loop hears %s: not %s' % (', '.join(sprung.HEARD), ', '.join(unknown)))
    if 'scenario' in command:
        sprung.reset_for_scenario(g, command['scenario'])
    if ('say' in command or command.get('walk') or command.get('entries')) and g['st'][latent.NOW][latent.GAIN] < 1e-6 and g.get('_jump_until', -9.0) + 0.03 <= g['t']:
        g['st'][latent.THETA] = 0.0
    if 'twist' in command:
        se2.heard_twist(g, gym, command['twist'], 'say' in command, sprung.said, sprung._dropped, heard_say, WALK_ON)
    by = {e[0]: e for e in gym}
    if 'charge' in command:
        g['_hop_charge'] = max(0.0, min(1.0, float(command['charge'])))
        if 'say' not in command and g.get('_jump_until', 0.0) > g['t']:
            leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, g['_hop_charge'])
    if 'say' in command:
        sc = command['say']
        entry = (gym[int(sc)] if (isinstance(sc, int) or (isinstance(sc, str) and sc.isdigit()))
                 and 0 <= int(sc) < len(gym) else by.get(sc))
        if entry is not None:
            if entry[0] == 'hop' and 'charge' not in command and g.get('_jump_until', 0.0) <= g['t']:
                g['_hop_charge'] = 0.0
            if 'twist' not in command and entry[0] != 'hop' and not (g.get('_twist_active') and entry[0] in se2.WASD_GAITS):
                g['_twist_active'], g['_twist_stop_until'], g['_resume_at'] = False, 0.0, None
                g['twist'] = g['asked'] = (0.0, 0.0, 0.0)
            if 'heading' not in command and entry[0] in ('turn_left', 'turn_right', 'turn_back_left', 'turn_back_right'):
                g['target_heading'] = None
            heard_say(g, gym, entry)
    if command.get('walk') and not (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                                    or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08
                                    or g['st'][latent.NOW][latent.GAIN] > 0.05):
        _start_mode(g, by, by['#204'], at=g['t'] + 0.001) if '#204' in by else sprung.walked(g)
    for entry in command.get('entries', ()):
        sprung.said(g, *entry)
    if 'heading' in command:
        g['target_heading'] = None if command['heading'] is None else float(command['heading'])
    if 'push' in command:
        g['world'].push(command['push'], command.get('seconds', sprung.SHOVE_S))
    if 'grip' in command:
        g['grip'][:] = g['grip_prior'][:] = min(float(command['grip']), wbc.MU)


def _is_fallen(g, world):
    """Fallen, the referee's: a part of her above her shins - thighs, pelvis, trunk, head, arms,
    hands - touches her ground (`floor.grounds`). By what she was doing, a tilt past 45 deg on a bed
    was a fall, and thrown back onto it she was down at 3.7 s."""
    m, d = world.model, world.data
    if g.get('_falls_by', (None,))[0] is not m:
        np = world._np
        down = {m.body(n).id for n in edge.SKIN if not n.endswith(('_shank', '_foot', '_toes'))}
        g['_falls_by'] = (m, np.isin(m.geom_bodyid, list(down)), np.isin(m.geom_bodyid, list(floor.grounds(m))))
    _m, down, ground = g['_falls_by']
    ncon = d.ncon
    if not ncon:
        return False
    geom = d.contact.geom[:ncon]
    a, b = geom[:, 0], geom[:, 1]
    return bool(((down[a] & ground[b]) | (down[b] & ground[a])).any())


#: A pass of her world, s; her drives' torque, power and watts averaged over AVERAGE_S; a wear
#: window's passes.
DT = 1.0 / sprung.RATE_HZ
K = DT / sprung.AVERAGE_S
WINDOW_STEPS = int(round(wear.WINDOW_S / DT))


def session(every=sprung.STACK_EVERY, walks=False, heard=None):
    """Her run on the loop, paced by nothing: her world `g`, the gym and what it says, the last
    refusal, when she fell, her drives' torque, power and watts, the epoch - a world laid anew
    each - and the passes in it. Each command heard goes to `heard` (a list's append, a queue's
    put) as (epoch, pass, t, command), each epoch's lay first."""
    s = {'every': every, 'walks': walks, 'gym_name': sprung.GYMS[0], 'gym': sprung.entries(),
         'refused': None, 'epoch': -1, 'ratio': 1.0, 'heard': heard or (lambda entry: None)}
    s['says'] = [s['gym_name']] + [entry[0] for entry in s['gym']]
    laid(s)
    return s


def laid(s):
    """A new epoch: her world laid anew, standing (walking where the session `walks`)."""
    import numpy as np
    g = s['g'] = sprung.laid(s['every'])
    if s['walks']:
        sprung.walked(g)
    n = len(g['world'].peak)
    s.update(fallen_at=None, torque=np.zeros(n), power=np.zeros(n), watts=0.0, epoch=s['epoch'] + 1,
             passes=0, wear=wear.laid(g['body']), ys=[], taus=[], wearing=None)
    s['wear_told'] = wear.told(s['wear'])
    s['heard']((s['epoch'], 0, g['t'], {'laid': {'every': s['every'], 'walks': s['walks'], 'gym': s['gym_name'],
                                                 'room': physics.ROOM, 'sight': physics.SIGHT}}))


def apply(s, command):
    """`command` heard at this pass, logged before it is done; a restart lays a new epoch."""
    s['heard']((s['epoch'], s['passes'], s['g']['t'], command))
    if command.get('restart'):
        laid(s)
        s['refused'] = None
    try:
        if command.get('gym') in sprung.GYMS:
            s['gym_name'], s['gym'] = command['gym'], sprung.entries(name=command['gym'])
            s['says'], s['refused'] = [command['gym']] + [entry[0] for entry in s['gym']], None
        if 'text' in command:
            command, s['refused'] = dict(command, entries=sprung.entries(command['text'])), None
        heard(s['g'], s['gym'], command)
        if 'scenario' in command:
            s['fallen_at'], s['ys'], s['taus'] = None, [], []
    except (SyntaxError, MachineError) as error:
        s['refused'] = str(error)


def stepped(s):
    """A pass of her world, judged: fallen, she lies RECOVER_S, then a new epoch is laid."""
    g, world = s['g'], s['g']['world']
    if s['fallen_at'] is not None and g['t'] - s['fallen_at'] >= sprung.RECOVER_S:
        return laid(s)
    try:
        if s['fallen_at'] is None:
            sprung.passed(g, DT)
            ys, taus = s['ys'], s['taus']
            ys.append(g['sense'].copy())
            taus.append(world.motor.copy())
            if len(ys) > WINDOW_STEPS:
                s['wearing'] = wear.wearing(s['wear'], ys, taus[:-1], DT, g)
                s['ys'], s['taus'] = [ys[-1]], [taus[-1]]
        else:
            from machine import motion
            s['ys'], s['taus'], g['surety'] = [], [], 0.0
            edge.felt(world, g['sense'])
            motion.drives(g, DT)
            edge.drive(world, g['joints'], **g['cmd'])
            g['t'] += DT
            world.advance()
        if s['wearing'] is not None:
            # a part of her wear's window a pass: in one the window held her world 0.12 s
            t_wear = time.perf_counter()
            try:
                next(s['wearing'])
            except StopIteration:
                s['wearing'] = None
                s['wear_told'] = wear.told(s['wear'])
            s['_wear_s'] = time.perf_counter() - t_wear
    except MachineError:
        s['fallen_at'] = g['t']
    if s['fallen_at'] is None and _is_fallen(g, world):
        s['fallen_at'] = g['t']
    s['passes'] += 1
    if g.get('_resume_at') is not None and g['t'] >= g['_resume_at']:
        g['_resume_at'] = None
        if any(abs(v) > 0.01 for v in g.get('asked', (0.0, 0.0, 0.0))):
            heard(g, s['gym'], {'twist': g['asked']})
    tau = world.data.ctrl
    s['torque'] += K * (tau - s['torque'])
    s['power'] += K * (tau * world.data.qvel[world.vadr] - s['power'])
    s['watts'] += K * (world.drawn() - s['watts'])


def told(s):
    """Her state as the page hears it (`sprung.state`), her epoch and her wear beside it, as its
    last window left it."""
    return dict(sprung.state(s['g'], s['torque'], s['power'], s['watts'], s['ratio'], s['fallen_at'],
                             s['says'], s['refused']), epoch=s['epoch'], wear=s['wear_told'])


class Her:

    """Her session (`session`) as `apart.paced` steps it: her time, her epoch, a command applied,
    a pass - its wear's seconds left out of the pace -, her state told at her ratio."""

    def __init__(self, s):
        self.s = s

    @property
    def t(self):
        return self.s['g']['t']

    @property
    def epoch(self):
        return self.s['epoch']

    def apply(self, command):
        apply(self.s, command)

    def step(self):
        stepped(self.s)
        return self.s.pop('_wear_s', 0.0)

    def said(self, ratio):
        if ratio is not None:
            self.s['ratio'] = ratio
        return told(self.s)


def run(commands, states, shown, heard=None, every=sprung.STACK_EVERY, walks=False, realtime=True):
    """The worker: she stands on the loop, paced to the clock (`realtime`, or flat out without it);
    commands heard - each put on `heard` at its pass -, her state said. Laid, she stands at rest
    unless `walks`. Run apart (`apart.paced`), no further than its lead past what her page has
    `shown`."""
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = session(every, walks, heard.put if heard is not None else None)
    apart.paced(Her(s), commands, states, shown=shown, realtime=realtime, say_s=sprung.SAY_S,
                rest_s=sprung.SAY_S / 4.0, slice_s=sprung.SLICE_S)


class Running(apart.Apart):

    """Her worker process on the loop (`run`, apart): `send` it commands, read its `latest` state
    and the commands it has `heard`, tell it what her page has `shown` - from `shown`, none held
    back till told; laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False, realtime=True, shown=math.inf):
        self._heard = apart.CONTEXT.Queue()
        super().__init__(run, self._heard, every, walks, realtime, states=8, kept=(self._heard,),
                         shown=shown)

    def send(self, **command):
        """{'say': the gym's entry by its place, in place of what was to come} | {'walk': True}:
        her walk from rest | {'entries': [(label, wire, names)]} | {'gym': one of GYMS} |
        {'text': orthokinematics, begun now} | {'push': (x, y, z) N, 'seconds': s} |
        {'restart': True}: laid anew. Any other key is refused, said with her state."""
        super().send(**command)

    def heard(self):
        """The commands she has heard since this was last asked, (epoch, pass, t, command) each:
        none is dropped, as a state can be."""
        out = []
        try:
            while True:
                out.append(self._heard.get_nowait())
        except queue.Empty:
            pass
        return out
