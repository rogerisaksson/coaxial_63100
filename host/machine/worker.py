"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import math
import multiprocessing
import queue
import time
from machine import ortho, physics, sprung
from machine.errors import MachineError
from machine.se2 import WALK_ON


def robot_body_ids(m, world):
    import numpy as np
    bids = set()
    for i in range(m.nbody):
        p = i
        while p > 0:
            if p == world.pelvis:
                bids.add(i)
                break
            p = m.body_parentid[p]
    upper = np.array(sorted(bids - set(world.soles.nonzero()[0])), dtype=int)
    if 'door' in [m.body(i).name for i in range(m.nbody)]:
        bids.add(m.body('door').id)
    return np.array(sorted(bids), dtype=int), upper


def _start_mode(g, by, entry, at):
    from machine import se2
    name, z = entry[0], float(g['world'].data.qpos[2])
    g['active_entry'] = name
    se2.align_chair_hop(g, name)
    iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
    tw = g.get('twist', (0.0, 0.0, 0.0))
    if name in ('ballistic_walk', 'jog', '#201'):
        return sprung.said(g, *entry, at=at, reset=True)
    if (name in ('turn_left', 'turn_right') and abs(tw[0]) < 0.05 and abs(tw[2]) > 0.05) or name == '#202':
        return sprung.said(g, *entry, at=at)
    if name == '#203' and '#202' in by:
        sprung.said(g, *by['#202'], at=at)
        return sprung.said(g, *by['#203'], at=at + 0.80)
    if name in ('#204', 'trip', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk',
                'turn_left', 'turn_right', 'circle_walk') and iw is not None:
        sprung.said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
        return sprung.said(g, *by[name], at=at + (0.42 if name in ('turn_left', 'turn_right', 'circle_walk') else 0.65))
    if name in ('sit_chair', 'lie_bed') and z < 0.25 and iw is not None and '#204' in by:
        sprung.said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
        sprung.said(g, *by['#204'], at=at + 0.65)
        return sprung.said(g, *by[name], at=at + (1.80 if name == 'sit_chair' else 1.50))
    sprung.said(g, *entry, at=at)


def heard_say(g, gym, entry):
    from machine import latent, modal, se2, stride
    by, name = {e[0]: e for e in gym}, entry[0]
    if name == g.get('active_entry') and (name == '#201' or sprung.doing(g) not in ('rest', '#201')
                                          or bool(len(g['tab']) and (g['tab'][:, latent.AT] >= g['t']).any())):
        return
    striding = (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08 or g['st'][latent.NOW][latent.GAIN] > 0.02)
    lph, lamp = float(g['st'][latent.NOW][latent.PHASE][modal.LATERAL]), float(g['st'][latent.NOW][latent.AMP][modal.LATERAL])
    back_now = (math.sin(lph) * (1.0 if lamp >= 0.0 else -1.0)) < -0.2
    spd, ref = math.hypot(float(g['world'].data.qvel[0]), float(g['world'].data.qvel[2])), g['out'][latent.REF]
    off_c = not striding and (max(abs(float(ref[modal.LIFT])), abs(float(ref[modal.SAGITTAL])), abs(float(ref[modal.LATERAL]))) > 0.15
                              or abs(float(ref[modal.AXIAL])) > 0.25 or spd > 0.04 or abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0)
    t0, cur_d = g['t'] + 0.001, sprung.doing(g)
    if cur_d in ('hold', 'load', 'throw', 'land', 'step_off', 'drop', 'absorb', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land') and len(g['tab']):
        g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['t'] + 1.5]
        t0, off_c = (float(g['tab'][-1, latent.AT]) if len(g['tab']) else g['t']) + 0.05, True
    else:
        sprung._dropped(g)
    same = (name in sprung._FWD_GAITS and not back_now) or (name in sprung._BACK_GAITS and back_now)
    if name in ('parry_shove', 'controlled_fall'):
        return sprung.said(g, *entry, at=t0, reset=True)
    if striding and name == 'hop' and '#201' in by:
        return se2.stride_hop(g, by, t0, spd, back_now, sprung._FWD_GAITS, sprung._BACK_GAITS, sprung.said)
    if striding and '#201' in by and not same:
        ts, tg = (1.20, 2.40) if spd > 0.45 else (0.55, 1.10)
        sprung.said(g, 'brake', sprung._BRAKE_BACK_WIRE if back_now else sprung._BRAKE_WIRE, ('brake',), at=t0, reset=True)
        sprung.said(g, *by['#201'], at=t0 + ts, reset=True)
        g['active_entry'] = name
        if name != '#201':
            g['_sync_cpg_at'] = t0 + tg
            _start_mode(g, by, entry, at=t0 + tg)
        return
    if striding and same:
        if g.get('active_entry') != name:
            g['active_entry'] = name
            sprung.said(g, '#204', se2._WALK_ON_WIRE, ('#204',), at=t0) if name == '#204' else sprung.said(g, *entry, at=t0)
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
    rmap = {'seated': ('rise_chair', 2.20), 'bed_rest': ('rise_bed', 3.60), 'floor_rest': ('rise_floor', 3.30)}
    if cur_d in rmap and name in (sprung._FWD_GAITS + sprung._BACK_GAITS) and rmap[cur_d][0] in by:
        rm, dt_r = rmap[cur_d]
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
    from machine import latent, modal, se2, stride
    unknown = sorted(set(command) - set(sprung.HEARD))
    if unknown:
        raise MachineError('the loop hears %s: not %s' % (', '.join(sprung.HEARD), ', '.join(unknown)))
    if 'scenario' in command:
        se2.reset_for_scenario(g, command['scenario'], sprung._seen, sprung._dropped, sprung.tilt, sprung.doing, sprung.WEP_S)
    if ('say' in command or command.get('walk') or command.get('entries')) and g['st'][latent.NOW][latent.GAIN] < 1e-6:
        g['st'][latent.THETA] = 0.0
    if 'twist' in command:
        se2.heard_twist(g, gym, command['twist'], 'say' in command, sprung.said, sprung._dropped, heard_say, WALK_ON)
    by = {e[0]: e for e in gym}
    if 'say' in command:
        sc = command['say']
        entry = (gym[int(sc)] if (isinstance(sc, int) or (isinstance(sc, str) and sc.isdigit()))
                 and 0 <= int(sc) < len(gym) else by.get(sc))
        if entry is not None:
            if 'twist' not in command and entry[0] != 'hop':
                g['_twist_active'], g['twist'] = False, (0.0, 0.0, 0.0)
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
        force = command['push']
        g['world'].push(force, command.get('seconds', sprung.SHOVE_S))
        mag, t0 = (math.hypot(*force) if hasattr(force, '__iter__') else float(force)), g['t'] + 0.001
        mode, ph = (('controlled_fall', 'cushion') if mag > 120.0
                    else (('parry_shove', 'catch') if mag >= 35.0 else (None, None)))
        if mode and mode in by and sprung.doing(g) not in ('lower', 'seated', 'bed_rest', 'floor_rest'):
            sprung._dropped(g)
            g['_twist_active'], g['twist'], g['target_heading'] = False, (0.0, 0.0, 0.0), None
            if g['st'][latent.NOW][latent.GAIN] > 0.02:
                lph = float(g['st'][latent.NOW][latent.PHASE][modal.LATERAL])
                lamp = float(g['st'][latent.NOW][latent.AMP][modal.LATERAL])
                back_now = (math.sin(lph) * (1.0 if lamp >= 0.0 else -1.0)) < -0.2
                sprung.said(g, ph, sprung._BRAKE_BACK_WIRE if back_now else sprung._BRAKE_WIRE, (ph,), at=t0, reset=True)
                t0 += 1.10
            sprung.said(g, *by[mode], at=t0, reset=True)
            g['active_entry'] = mode


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


def _apply_cmd(command, g, gym, says, refused, fallen_at):
    try:
        if command.get('gym') in sprung.GYMS:
            gym = sprung.entries(name=command['gym'])
            says, refused = [command['gym']] + [entry[0] for entry in gym], None
        if 'text' in command:
            command, refused = dict(command, entries=sprung.entries(command['text'])), None
        heard(g, gym, command)
        if 'scenario' in command:
            fallen_at = None
    except (SyntaxError, MachineError) as error:
        refused = str(error)
    return gym, says, refused, fallen_at


def _step_slice(g, due, dt, fallen_at, torque, power, watts, k):
    world = g['world']
    while g['t'] < due - 1e-9:
        try:
            if fallen_at is None:
                sprung.passed(g, dt)
            else:
                world.feed[:], world.rate[:] = 0.0, 0.0
                world.target[:] += 0.05 * (world.data.qpos[world.qadr] - world.target)
                g['t'] += dt
                world.advance()
        except MachineError:
            fallen_at = g['t']
        if fallen_at is None and _is_fallen(g, world):
            fallen_at = g['t']
        tau = world.data.ctrl
        torque += k * (tau - torque)
        power += k * (tau * world.data.qvel[world.vadr] - power)
        watts += k * (world.drawn() - watts)
    return fallen_at, watts


def run(commands, states, every=sprung.STACK_EVERY, walks=False):
    """The worker: she stands on the loop, paced to the clock; commands heard, her state said.
    Laid, she stands at rest unless `walks`."""
    import numpy as np
    physics.ROOM, physics.SIGHT = 1.0, 0.0

    def anew():
        g = sprung.laid(every)
        if walks:
            sprung.walked(g)
        return g

    gym, g = sprung.entries(), anew()
    says, refused = [sprung.GYMS[0]] + [entry[0] for entry in gym], None
    dt, k = 1.0 / sprung.RATE_HZ, (1.0 / sprung.RATE_HZ) / sprung.AVERAGE_S
    torque, power = np.zeros(len(g['world'].peak)), np.zeros(len(g['world'].peak))
    watts, fallen_at = 0.0, None
    wall0, sim0 = time.perf_counter(), g['t']
    told, ratio, spent, ran = 0.0, 1.0, 0.0, 0.0
    _put(states, sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
    while True:
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                if command.get('restart'):
                    g, fallen_at, told = anew(), None, 0.0
                    torque[:], power[:], watts, refused = 0.0, 0.0, 0.0, None
                    wall0, sim0 = time.perf_counter(), g['t']
                    _put(states, sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
                gym, says, refused, fallen_at = _apply_cmd(
                    command, g, gym, says, refused, fallen_at)
        except queue.Empty:
            pass
        began = time.perf_counter()
        due = sim0 + (began - wall0)
        if due - g['t'] > sprung.SLICE_S:
            due = g['t'] + sprung.SLICE_S
            wall0, sim0 = began, due
        if fallen_at is not None and g['t'] - fallen_at >= sprung.RECOVER_S:
            g, fallen_at, told = anew(), None, 0.0
            torque[:], power[:], watts = 0.0, 0.0, 0.0
            wall0, sim0, due = time.perf_counter(), g['t'], g['t']
            _put(states, sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
        from_t = g['t']
        fallen_at, watts = _step_slice(g, due, dt, fallen_at, torque, power, watts, k)
        spent += time.perf_counter() - began
        ran += g['t'] - from_t
        if g['t'] - told >= sprung.SAY_S:
            ratio = ran / spent if spent > 1e-6 else ratio
            spent = ran = 0.0
            told = g['t']
            _put(states, sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
        rest = sprung.SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """Her worker process on the loop (`run`): `send` it commands, read its `latest` state;
    laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states = context.Queue(), context.Queue(maxsize=8)
        self._commands.cancel_join_thread()
        self._states.cancel_join_thread()
        self._process = context.Process(target=run, daemon=True,
                                        args=(self._commands, self._states, every, walks))
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

    def close(self):
        try:
            self._commands.put_nowait(None)
        except queue.Full:
            pass
        for _ in (0, 1):
            while True:
                try:
                    self._states.get_nowait()
                except queue.Empty:
                    break
            if not _:
                self._process.join(timeout=1.0)
                if self._process.is_alive():
                    self._process.terminate()


