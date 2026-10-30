"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import ctypes
import math
import os
import queue
import time
from machine.process import apart, placed, sprung
from machine.world import physics, referee
from machine.senses import edge
from machine.texts import ortho
from machine.way import planner
from machine.law import se2
from machine.eyes import vision
from machine.identify import wear
from machine.errors import BusFault, MachineError


_TURN_GAITS = ('turn_left', 'turn_right', 'spin_left', 'spin_right')
_TURN_ALL = _TURN_GAITS + ('turn_back_left', 'turn_back_right')


def _sync_nominal_twist(g, name):
    from machine.law import se2
    from machine.senses import touch
    if not g.get('_twist_active') and name != 'hop':
        g['asked'] = se2.NOMINAL_TWIST.get(name, (0.0, 0.0, 0.0))
        g['twist'] = touch.against(g, g['asked'])


def _start_hop(g, by, entry, at):
    from machine.law import latent
    from machine.moves import leap
    wire, names, dur, dt_load_ms = leap.standing_leap(entry[1], g['fr'], float(g.get('_hop_charge', 0.0)))
    g['st'][latent.THETA] = ((0.10 - dt_load_ms * 0.001) % 1.0) * math.tau
    g['_jump_t0'], g['_jump_throw'], g['_jump_until'], g['_leap'] = at, at + dt_load_ms * 0.001, at + dur, False
    sprung.said(g, 'hop', wire, names, at=at, reset=True)
    if 'stand' in by:
        sprung.said(g, *by['stand'], at=at + dur + 0.01, reset=True)


def _start_mode(g, by, entry, at):
    from machine.law import se2
    name = g['active_entry'] = entry[0]
    _sync_nominal_twist(g, name)
    if name == 'hop':
        return _start_hop(g, by, entry, at)
    iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
    skip_iw = (name in _TURN_GAITS and g.get('_twist_active')) or name == 'sway' or name in se2.RESET_MODES
    use_iw = name in se2.INIT_GAITS and iw is not None and not skip_iw
    use_sway = name == 'step' and 'sway' in by
    if use_sway:
        sprung.said(g, *by['sway'], at=at)
    if use_iw:
        sprung.said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
    dt_lead = 0.80 if use_sway else (se2.init_lead(g) if use_iw else 0.0)
    sprung.said(g, *entry, at=at + dt_lead, reset=(name in se2.RESET_MODES))


def _queue_post_jump(g, by, entry):
    from machine.law import latent, se2
    from machine.moves import leap
    name = entry[0]
    if name == 'hop':
        return leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, float(g.get('_hop_charge', 0.0)))
    if name == g.get('active_entry') and any(w > g['_jump_until'] for w, _ in g['labels']):
        return
    t_res = g['_jump_until'] + 0.02
    g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
    g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
    g['active_entry'] = name
    _sync_nominal_twist(g, name)
    leap_brake = name == 'stand' and g.get('_leap') and '_brake_wire' in g and 'stand' in by
    if leap_brake:
        sprung.said(g, 'brake', g['_brake_wire'], ('brake',), at=t_res - 0.01, reset=True)
        return sprung.said(g, *by['stand'], at=t_res + 0.54, reset=True)
    post = (name, se2._WALK_ON_WIRE, (name,)) if name in ('walk', 'jog') else by.get(name)
    if post is not None:
        sprung.said(g, *post, at=t_res, reset=(name == 'stand'))


def _brake_and_switch(g, by, entry, t0, spd, back_now):
    import numpy as np
    from machine.law import latent, se2
    name = g['active_entry'] = entry[0]
    brake = np.frombuffer(sprung._BRAKE_BACK_WIRE if back_now else sprung._BRAKE_WIRE, latent.WIRE).copy()
    brake['blend'] = round(1e3 * se2.braked(spd))
    sprung.said(g, 'brake', brake.tobytes(), ('brake',), at=t0, reset=True)
    if name == 'stand' or name not in (sprung._FWD_GAITS + sprung._BACK_GAITS):
        ts, tg = se2.stand_lead(spd)
        sprung.said(g, *by['stand'], at=t0 + ts, reset=True)
        return _start_mode(g, by, entry, at=t0 + tg) if name != 'stand' else None
    g['st'][latent.NOW][latent.GAIN] = max(float(g['st'][latent.NOW][latent.GAIN]), 0.04)
    dt_b = se2.brake_lead(spd, float(name in sprung._BACK_GAITS))
    nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
    return sprung.said(g, *nxt, at=t0 + dt_b, reset=True)


def _splice_fwd(g, entry, future, cur_d):
    from machine.law import latent, se2
    name = g['active_entry'] = entry[0]
    t_end = float(future[0, latent.AT])
    t_last = min(t_end, g['t'] + 0.18) if (cur_d == 'init_walk' and name in ('jog', 'turn_left', 'turn_right')) else t_end
    g['tab'] = g['tab'][g['tab'][:, latent.AT] < t_end - 1e-4]
    g['labels'] = [(w, n) for w, n in g['labels'] if w < t_end - 1e-4]
    _sync_nominal_twist(g, name)
    nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
    sprung.said(g, *nxt, at=t_last)


def heard_say(g, gym, entry):
    from machine.law import latent, modal, se2, stride
    by, name = {e[0]: e for e in gym}, entry[0]
    if g.get('_jump_until', 0.0) > g['t']:
        return _queue_post_jump(g, by, entry)
    if name == g.get('active_entry') and (name == 'stand' or sprung.doing(g) not in ('rest', 'stand')
                                          or bool(len(g['tab']) and (g['tab'][:, latent.AT] >= g['t']).any())):
        return
    striding = (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08 or g['st'][latent.NOW][latent.GAIN] > 0.02)
    # her gait's direction is the one she was told, not her sway's phase: a spin's read as a backward walk
    back_now = g.get('active_entry') in sprung._BACK_GAITS
    spd, ref = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2])), g['out'][latent.REF]
    t0, cur_d = g['t'] + 0.001, sprung.doing(g)
    off_c = not striding and cur_d not in ('stand', 'rest') and (
        max(abs(float(ref[modal.LIFT])), abs(float(ref[modal.SAGITTAL])), abs(float(ref[modal.LATERAL]))) > 0.15
        or abs(float(ref[modal.AXIAL])) > 0.25 or abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0)
    future = g['tab'][g['tab'][:, latent.AT] > g['t']]
    same_dir = ((g.get('active_entry') in sprung._FWD_GAITS and name in sprung._FWD_GAITS)
                or (g.get('active_entry') in sprung._BACK_GAITS and name in sprung._BACK_GAITS))
    if (cur_d in ('brake', 'stand', 'init_walk') and same_dir and len(future)
            and (cur_d != 'init_walk' or abs(float(g.get('twist', (0.0,))[0])) >= 0.05)):
        return _splice_fwd(g, entry, future, cur_d)
    hold = cur_d in se2.HOLD_PHASES and len(g['tab']) > 0
    g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['t'] + 1.5] if hold else g['tab']
    if not hold:
        sprung._dropped(g)
    t0 = ((float(g['tab'][-1, latent.AT]) if len(g['tab']) else g['t']) + 0.05) if hold else t0
    off_c = off_c or hold
    ae = g.get('active_entry')
    strafe_opp = ({name, ae} == {'strafe_left', 'strafe_right'}
                  or (((name in ('strafe_left', 'strafe_right')) != (ae in ('strafe_left', 'strafe_right'))) and spd > 0.28))
    same = cur_d not in ('brake', 'stand') and not strafe_opp and ((name in sprung._FWD_GAITS and not back_now) or (name in sprung._BACK_GAITS and back_now))
    _sync_nominal_twist(g, name)
    if name in ('parry_shove', 'controlled_fall'):
        g['active_entry'] = name
        return sprung.said(g, *entry, at=t0, reset=True)
    if (striding or g.get('_twist_active')) and name == 'hop' and 'stand' in by:
        return se2.stride_hop(g, by, t0, spd, back_now, sprung._FWD_GAITS, sprung._BACK_GAITS, sprung.said)
    if striding and 'stand' in by and not same:
        return _brake_and_switch(g, by, entry, t0, spd, back_now)
    if striding and same:
        rst = name in ('jog', 'walk') and ae in (('strafe_left', 'strafe_right') + _TURN_GAITS)
        nxt = ('walk', se2._WALK_ON_WIRE, ('walk',)) if name == 'walk' else entry
        if ae != name:
            g['active_entry'] = name
            sprung.said(g, *nxt, at=t0, reset=rst)
        return
    if cur_d == 'sway' and name in (('step',) + sprung._FWD_GAITS):
        sprung.said(g, *by['step'], at=t0, reset=True)
        return sprung.said(g, *by[name], at=t0 + 0.90) if (name != 'step' and name in by) else None
    if cur_d in ('over', 'touch') and name != 'one_sole':
        sprung.said(g, 'touch', sprung._TOUCH_WIRE, ('touch',), at=t0)
        sprung.said(g, *by['stand'], at=t0 + 0.80, reset=True)
        return _start_mode(g, by, entry, at=t0 + 2.00) if name != 'stand' else None
    if cur_d in se2.RISE_OF and name in (sprung._FWD_GAITS + sprung._BACK_GAITS) and se2.RISE_OF[cur_d] in by:
        rm = se2.RISE_OF[cur_d]
        sprung.said(g, *by[rm], at=t0, reset=True)
        return _start_mode(g, by, entry, at=t0 + se2.rise_lead(by[rm][1]))
    if off_c and 'stand' in by and name not in (('stand', 'hop') + tuple(sprung.STUNT_MODES)):
        sprung.said(g, *by['stand'], at=t0, reset=True)
        return _start_mode(g, by, entry, at=t0 + (0.55 if cur_d == 'down' else 1.35))
    return _start_mode(g, by, entry, at=t0) if not striding else sprung.said(g, *entry, at=t0)


def _apply_say(g, gym, by, command, errand_step):
    from machine.way import errand
    from machine.texts import maneuver
    from machine.law import se2
    sc = command['say']
    sc_str = sprung.ALIASES.get(str(sc), str(sc))
    if not errand_step and errand.start_errand(g, gym, sc_str, heard, heard_say, command.get('at')):
        return
    is_idx = (isinstance(sc, int) or (isinstance(sc, str) and sc.isdigit())) and 0 <= int(sc) < len(gym)
    entry = gym[int(sc)] if is_idx else by.get(sc_str)
    if entry is None:
        return
    if entry[0] == 'hop' and 'charge' not in command and g.get('_jump_until', 0.0) <= g['t']:
        g['_hop_charge'] = 0.0
    if 'twist' not in command and entry[0] != 'hop' and not (g.get('_twist_active') and entry[0] in se2.WASD_GAITS):
        g['_twist_active'], g['_twist_stop_until'], g['_halt_at'] = False, 0.0, None
        g['twist'] = g['asked'] = (0.0, 0.0, 0.0)
    if 'heading' not in command and entry[0] in _TURN_ALL:
        g['target_heading'], g['has_heading'] = 0.0, 0.0
    heard_say(g, gym, maneuver.said(g, entry))


def heard(g, gym, command):
    from machine.law import latent, modal, se2, stride
    from machine.moves import leap
    from machine.stack import wbc
    errand_step = bool(command.get('_errand_cmd'))
    unknown = sorted(set(command) - set(sprung.HEARD) - {'_errand_cmd'})
    if unknown:
        raise MachineError('the loop hears %s: not %s' % (', '.join(sprung.HEARD), ', '.join(unknown)))
    if not errand_step and any(k in command for k in ('scenario', 'twist', 'say', 'walk', 'entries')):
        g['_errand'] = None
        if 'heading' not in command:
            g['target_heading'], g['has_heading'] = 0.0, 0.0
    if 'scenario' in command:
        sprung.reset_for_scenario(g, command['scenario'])
    if ('say' in command or command.get('walk') or command.get('entries')) and g['st'][latent.NOW][latent.GAIN] < 1e-6 and g.get('_jump_until', -9.0) + 0.03 <= g['t']:
        g['st'][latent.THETA] = 0.0
    if 'twist' in command:
        se2.heard_twist(g, gym, command['twist'], 'say' in command, heard_say)
    by = {e[0]: e for e in gym}
    if 'charge' in command:
        g['_hop_charge'] = max(0.0, min(1.0, float(command['charge'])))
    if 'charge' in command and 'say' not in command and g.get('_jump_until', 0.0) > g['t']:
        leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, g['_hop_charge'])
    if 'say' in command:
        _apply_say(g, gym, by, command, errand_step)
    if command.get('walk') and not (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                                    or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08
                                    or g['st'][latent.NOW][latent.GAIN] > 0.05):
        _start_mode(g, by, by['walk'], at=g['t'] + 0.001) if 'walk' in by else sprung.walked(g)
    for entry in command.get('entries', ()):
        sprung.said(g, *entry)
    if 'heading' in command:
        g['has_heading'] = 0.0 if command['heading'] is None else 1.0
        g['target_heading'] = 0.0 if command['heading'] is None else float(command['heading'])
    if 'push' in command:
        g['world'].push(command['push'], command.get('seconds', sprung.SHOVE_S))
    if 'grip' in command:
        g['grip'][:] = g['grip_prior'][:] = min(float(command['grip']), wbc.MU)
    if 'floor' in command:
        told_floor(g, command['floor'])


def told_floor(g, profile):
    """A floor told her into her frame's map (`modal.MAP_*`): (along, rise) m from where she
    stands along her heading, its heights over the floor under her; None forgets it."""
    import numpy as np
    from machine.law import modal
    fr = g['fr']
    fr[modal.MAP_N] = 0.0
    if profile is None:
        return
    pts, hd = np.array(profile, float)[:modal.MAP_K], float(g['heading'])
    fr[modal.MAP_AT], fr[modal.MAP_WAY] = (g['est']['pos'][0], g['est']['pos'][2]), (math.sin(hd), math.cos(hd))
    fr[modal.MAP_ALONG][:len(pts)], fr[modal.MAP_RISE][:len(pts)] = pts[:, 0], pts[:, 1]
    fr[modal.MAP_N] = float(len(pts))


#: A pass of her world, s; her drives' torque, power and watts averaged over AVERAGE_S; a wear
#: window's passes.
DT = 1.0 / sprung.RATE_HZ
#: The environment's port her loop's layers are served on (`introspect.attach`), unset none.
TELEMETRY = 'COAXIAL_TELEMETRY'
K = DT / sprung.AVERAGE_S
WINDOW_STEPS = int(round(wear.WINDOW_S / DT))


def session(every=sprung.STACK_EVERY, walks=False, heard=None, texts=None, ber=0.0):
    """Her run on the loop, paced by nothing, of her texts `texts` ({key: text}: her gym's and the
    suite's) where given, else their files': her world `g`, the gym and what it says, the last
    refusal, when she fell, her drives' torque, power and watts, the epoch - a world laid anew
    each - and the passes in it. Each command heard goes to `heard` (a list's append, a queue's
    put) as (epoch, pass, t, command), each epoch's lay first."""
    from machine.texts import dsp
    suite = None if texts is None else dsp.read(dsp.SUITE, texts)
    s = {'every': every, 'walks': walks, 'gym_name': sprung.GYMS[0], 'gym': sprung.entries(texts=texts, suite=suite),
         'texts': texts, 'suite': suite, 'each': [], 'links': [],
         'refused': None, 'epoch': -1, 'ratio': 1.0, 'headroom': 1.0, 'heard': heard or (lambda entry: None),
         'ber': ber}
    s['says'] = [s['gym_name']] + [entry[0] for entry in s['gym']]
    laid(s)
    return s


def laid(s):
    """A new epoch: her world laid anew, standing (walking where the session `walks`); the last one's camera
    ended, its planner and its vision kept."""
    import numpy as np
    if s.get('g') is not None and s['g']['keep'] is not None:
        s['g']['keep'].close()
    was = s.get('g')
    g = s['g'] = sprung.laid(s['every'], texts=s['texts'], suite=s['suite'])
    if was is not None:
        g['planner'], g['vision'] = was['planner'], was['vision']
    if s['walks']:
        sprung.walked(g)
    edge.noise(g['world'], s['ber'], seed=s['epoch'] + 2)
    n = len(g['world'].peak)
    s.update(fallen_at=None, shutdown=None, torque=np.zeros(n), power=np.zeros(n), watts=0.0, epoch=s['epoch'] + 1,
             passes=0, wear=wear.laid(g['body']), ys=[], taus=[], wearing=None)
    s['wear_told'] = wear.told(s['wear'])
    s['heard']((s['epoch'], 0, g['t'], {'laid': {'every': s['every'], 'walks': s['walks'], 'gym': s['gym_name'],
                                                 'room': physics.ROOM, 'sight': physics.SIGHT}}))


def apply(s, command):
    """`command` heard at this pass, logged before it is done; a restart lays a new epoch, a link a
    tuner's hook on each pass (`tap.attached`)."""
    s['heard']((s['epoch'], s['passes'], s['g']['t'], command))
    if command.get('restart'):
        laid(s)
        s['refused'] = None
    if 'link' in command:
        from machine.process import tap
        hook, link = tap.attached(command['link'])
        s['each'].append(hook)
        s['links'].append(link)
    try:
        if command.get('gym') in sprung.GYMS:
            s['gym_name'], s['gym'] = command['gym'], sprung.entries(name=command['gym'], texts=s['texts'], suite=s['suite'])
            s['says'], s['refused'] = [command['gym']] + [entry[0] for entry in s['gym']], None
        if 'text' in command:
            command, s['refused'] = dict(command, entries=sprung.entries(command['text'])), None
        heard(s['g'], s['gym'], command)
        if 'scenario' in command:
            s['fallen_at'], s['ys'], s['taus'] = None, [], []
    except (SyntaxError, MachineError) as error:
        s['refused'] = str(error)


def stepped(s):
    """A pass of her world, judged: fallen, she lies RECOVER_S, then a new epoch is laid; shut down
    by a bus (`edge.held`), her drives stay off till a restart."""
    g, world = s['g'], s['g']['world']
    if s['shutdown'] is not None:
        g['t'] += DT
        world.advance()
        s['passes'] += 1
        return None
    if s['fallen_at'] is not None and g['t'] - s['fallen_at'] >= sprung.RECOVER_S:
        return laid(s)
    try:
        if s['fallen_at'] is None:
            from machine.way import errand
            errand.step_errand(g, s['gym'], heard, heard_say)
            sprung.passed(g, DT)
            ys, taus = s['ys'], s['taus']
            ys.append(g['sense'].copy())
            taus.append(world.motor.copy())
            if len(ys) > WINDOW_STEPS:
                s['wearing'] = wear.wearing(s['wear'], ys, taus[:-1], DT, g)
                s['ys'], s['taus'] = [ys[-1]], [taus[-1]]
        else:
            # fallen she lies with her drives let go, as the bench's STO drops them: driven on
            # through the fall the rest law fought the ground - 1 658 deg/s, 3 793 N into it
            # supine, 40 deg/s and 45 N let go - the spasm of every fall and tipped lie.
            s['ys'], s['taus'], g['surety'] = [], [], 0.0
            edge.felt(world, g['sense'])
            edge.off(world)
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
    except BusFault as fault:
        s['shutdown'], s['fallen_at'] = str(fault), g['t']
        edge.off(world)
    except MachineError:
        s['fallen_at'] = g['t']
    if s['fallen_at'] is None and referee.fallen(g, world):
        s['fallen_at'] = g['t']
    s['passes'] += 1
    _paired, averaged, address = referee.passing()
    averaged(ctypes.byref(world.passing), K, address(s['torque']), address(s['power']))
    s['watts'] += K * (world.drawn() - s['watts'])
    for hook in s['each']:
        hook(s)


def told(s):
    """Her state as the page hears it (`sprung.state`), her epoch and her wear beside it, as its
    last window left it."""
    return dict(sprung.state(s['g'], s['torque'], s['power'], s['watts'], s['ratio'], s['fallen_at'],
                             s['says'], s['refused'], s['headroom']), epoch=s['epoch'], wear=s['wear_told'],
                shutdown=s['shutdown'])


class Her:

    """Her session (`session`) as `apart.paced` steps it: her time, her epoch, a command applied,
    a pass - its wear's seconds left out of the pace -, her state told at her ratio and headroom."""

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

    def said(self, ratio, headroom=1.0):
        if ratio is not None:
            self.s['ratio'], self.s['headroom'] = ratio, headroom
        return told(self.s)


def run(commands, states, shown, heard=None, every=sprung.STACK_EVERY, walks=False, realtime=True, ber=0.0):
    """The worker: she stands on the loop, paced to the clock (`realtime`, or flat out without it);
    commands heard - each put on `heard` at its pass -, her state said. Laid, she stands at rest
    unless `walks`. Run apart (`apart.paced`), no further than its lead past what her page has
    `shown`. Her fake camera shoots for her vision's panel (`physics.CAMERA`); her planner and her vision (`vision.Apart`) spawned warm; placed a core apart (`placed`)."""
    placed.placed()
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 1.0, 0.0
    s = session(every, walks, heard.put if heard is not None else None, ber=ber)
    s['g']['planner'], s['g']['vision'] = planner.Planner(), vision.Apart()
    s['g']['planner'].start()
    s['g']['vision'].start()
    if os.environ.get(TELEMETRY):
        from machine.process import introspect
        introspect.attach(s, int(os.environ[TELEMETRY]))
    apart.paced(Her(s), commands, states, shown=shown, realtime=realtime, say_s=sprung.SAY_S,
                rest_s=sprung.SAY_S / 4.0, slice_s=sprung.SLICE_S)


class Running(apart.Apart):

    """Her worker process on the loop (`run`, apart): `send` it commands, read its `latest` state
    and the commands it has `heard`, tell it what her page has `shown` - from `shown`, none held
    back till told; laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False, realtime=True, shown=math.inf, ber=0.0):
        self._heard = apart.CONTEXT.Queue()
        super().__init__(run, self._heard, every, walks, realtime, ber, states=8, kept=(self._heard,),
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

