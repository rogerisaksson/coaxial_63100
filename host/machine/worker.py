"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import math
import multiprocessing
import queue
import time
from machine import ortho, physics, sprung
from machine.errors import MachineError
from machine.se2 import WALK_ON


def _start_mode(g, by, entry, at):
    from machine import se2
    name = entry[0]
    g['active_entry'] = name
    if name == 'hop':
        g['_jump_until'], g['_leap'] = at + 1.45, False
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
    from machine import latent, modal, se2, stride
    by, name = {e[0]: e for e in gym}, entry[0]
    if g.get('_jump_until', 0.0) > g['t'] and name != 'hop':
        if name == g.get('active_entry') and any(w > g['_jump_until'] for w, _ in g['labels']):
            return
        t_res = g['_jump_until'] + 0.02
        g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['_jump_until'] + 0.01]
        g['labels'] = [(w, n) for w, n in g['labels'] if w <= g['_jump_until'] + 0.01]
        g['active_entry'] = name
        if name in ('#204', 'jog'):
            sprung.said(g, name, se2._WALK_ON_WIRE, (name,), at=t_res)
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
    off_c = not striding and (max(abs(float(ref[modal.LIFT])), abs(float(ref[modal.SAGITTAL])), abs(float(ref[modal.LATERAL]))) > 0.15
                              or abs(float(ref[modal.AXIAL])) > 0.25 or spd > 0.04 or abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0)
    t0, cur_d = g['t'] + 0.001, sprung.doing(g)
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
    from machine import latent, modal, se2, stride, wbc
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
    if 'say' in command:
        sc = command['say']
        entry = (gym[int(sc)] if (isinstance(sc, int) or (isinstance(sc, str) and sc.isdigit()))
                 and 0 <= int(sc) < len(gym) else by.get(sc))
        if entry is not None:
            if 'twist' not in command and entry[0] != 'hop' and not (g.get('_twist_active') and entry[0] in se2.WASD_GAITS):
                g['_twist_active'], g['_twist_stop_until'], g['twist'] = False, 0.0, (0.0, 0.0, 0.0)
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
    """Whether the gynoid has lost balance or fallen."""
    cur_doing = sprung.doing(g)
    cur_tilt = sprung.tilt(g)
    cur_y = float(world.data.qpos[1])
    if cur_tilt > 75.0 or cur_y < 0.08:
        return True
    if cur_doing in ('seated', 'approach', 'lower', 'lean', 'thrust'):
        return cur_tilt > 45.0 or cur_y < 0.28
    if cur_doing in ('bed_rest', 'recline', 'sit_edge', 'sit_up'):
        return cur_tilt > 45.0 or cur_y < 0.25
    if cur_doing in ('floor_rest', 'kneel', 'cushion', 'land'):
        return cur_tilt > 75.0 or cur_y < 0.06
    if cur_doing not in sprung.STUNT_PHASES:
        return cur_y < sprung.DOWN_M or (cur_tilt > sprung.DOWN_DEG and cur_y < 0.65) or cur_tilt > 80.0
    return False


def _put(states, item):
    while states.full():
        try:
            states.get_nowait()
        except queue.Empty:
            break
    try:
        states.put_nowait(item)
    except queue.Full:
        pass


#: A pass of her world, s; her drives' torque, power and watts averaged over AVERAGE_S.
DT = 1.0 / sprung.RATE_HZ
K = DT / sprung.AVERAGE_S


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
             passes=0)
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
            s['fallen_at'] = None
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
        else:
            world.feed[:], world.rate[:] = 0.0, 0.0
            world.target[:] += 0.05 * (world.data.qpos[world.qadr] - world.target)
            g['t'] += DT
            world.advance()
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
    """Her state as the page hears it (`sprung.state`), her epoch beside it."""
    return dict(sprung.state(s['g'], s['torque'], s['power'], s['watts'], s['ratio'], s['fallen_at'],
                             s['says'], s['refused']), epoch=s['epoch'])


def run(commands, states, heard=None, every=sprung.STACK_EVERY, walks=False):
    """The worker: she stands on the loop, paced to the clock; commands heard - each put on
    `heard` at its pass -, her state said. Laid, she stands at rest unless `walks`."""
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = session(every, walks, heard.put if heard is not None else None)
    _put(states, told(s))
    wall0, sim0, said_at, spent, ran = time.perf_counter(), s['g']['t'], 0.0, 0.0, 0.0
    while True:
        epoch = s['epoch']
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                apply(s, command)
        except queue.Empty:
            pass
        began = time.perf_counter()
        if s['epoch'] == epoch:
            due = sim0 + (began - wall0)
            if due - s['g']['t'] > sprung.SLICE_S:
                due = s['g']['t'] + sprung.SLICE_S
                wall0, sim0 = began, due
            from_t = s['g']['t']
            while s['g']['t'] < due - 1e-9 and s['epoch'] == epoch:
                stepped(s)
            spent += time.perf_counter() - began
            ran += s['g']['t'] - from_t
        if s['epoch'] != epoch:
            wall0, sim0, said_at, spent, ran = time.perf_counter(), s['g']['t'], 0.0, 0.0, 0.0
            _put(states, told(s))
            continue
        if s['g']['t'] - said_at >= sprung.SAY_S:
            s['ratio'] = ran / spent if spent > 1e-6 else s['ratio']
            spent = ran = 0.0
            said_at = s['g']['t']
            _put(states, told(s))
        rest = sprung.SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """Her worker process on the loop (`run`): `send` it commands, read its `latest` state and
    the commands it has `heard`; laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states, self._heard = context.Queue(), context.Queue(maxsize=8), context.Queue()
        for each in (self._commands, self._states, self._heard):
            each.cancel_join_thread()
        self._process = context.Process(target=run, daemon=True,
                                        args=(self._commands, self._states, self._heard, every, walks))
        self._process.start()
        self._last = None

    def send(self, **command):
        """{'say': the gym's entry by its place, in place of what was to come} | {'walk': True}:
        her walk from rest | {'entries': [(label, wire, names)]} | {'gym': one of GYMS} |
        {'text': orthokinematics, begun now} | {'push': (x, y, z) N, 'seconds': s} |
        {'restart': True}: laid anew. Any other key is refused, said with her state."""
        self._commands.put(command)

    def latest(self, into=None):
        """The newest state the worker has said, or the last one if nothing new; every one said
        since appended to `into` if given."""
        try:
            while True:
                self._last = self._states.get_nowait()
                if into is not None:
                    into.append(self._last)
        except queue.Empty:
            pass
        return self._last

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

    def close(self):
        try:
            self._commands.put_nowait(None)
        except queue.Full:
            pass
        for _ in (0, 1):
            for each in (self._states, self._heard):
                while True:
                    try:
                        each.get_nowait()
                    except queue.Empty:
                        break
            if not _:
                self._process.join(timeout=1.0)
                if self._process.is_alive():
                    self._process.terminate()
