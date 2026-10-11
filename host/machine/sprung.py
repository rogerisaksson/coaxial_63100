"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = Running(worker=sprung.run)      # `machine.running`: this module's worker, her process
    body.send(say=2)                       # the gym's third entry: a payload injected, a behaviour begun
    body.send(text='(stream_inject ..)')   # a text's payloads and behaviours, begun now
    body.send(push=(0.0, 0.0, 80.0), seconds=0.12); body.send(restart=True)
    body.send(gym='examples')              # the gym's other text (GYMS)
    sprung.entries()                       # [(label, wire, names)]: machine/moves.lisp's

    g = sprung.laid()                      # the same loop without the process: one dict a run holds
    sprung.said(g, label, wire, names)     # rows begun now
    s, loads, ask, out = sprung.passed(g)  # a pass, her world a millisecond on

A pass: she is sensed (`wbc.sense`), her latent state is stepped (`machine.latent`) and rendered
on her (`machine.modal`), the whole-body loop solved (`wbc.step`) - every `every` passes, its
answer held between -, its torques fed forward on the drives' PD round a reference integrated
from its accelerations, and her world advanced. The worker paces that to the wall clock and
says her state as `machine.running`'s does, her modes with it. Down, she is laid anew.
"""
import math
import os
import queue
import time

from machine.running import AVERAGE_S, RATE_HZ, RECOVER_S, SAY_S, SLICE_S

#: The drives' PD under the loop (`physics.SERVO`'s gains times SERVO_SHARE: kp, kd) round a
#: reference integrated from the loop's accelerations, leaking to the joints' angles over
#: LEAK_S; a standing leg's at STANDING_KP of kp. War emergency power: a drive the loop asks
#: past its clamp has its clamp at its stack's peak WEP_HOLD_S past its last need, WEP_S of it
#: at once, filled again WEP_FILL as fast as it is spent. As tools/sim/wbc.py measured them.
SERVO_SHARE, LEAK_S, STANDING_KP = (0.5, 1.0), 0.2, 0.1
WEP_HOLD_S, WEP_S, WEP_FILL = 0.6, 5.0, 0.1
#: The drives' own loop under the stack: 'pd' that PD; 'eso' each drive's disturbance observed
#: at ESO_WO rad/s (Han's observer, `parts.Eso`'s law on arrays) and taken off its torque, no
#: PD; 'none' the torque as asked.
INNER, ESO_WO = 'eso', 100.0
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
#: On the page the loop is solved every STACK_EVERY passes: solved every one a pass costs
#: 3.1 ms, a third of real time; every fourth 1.0 ms, her crouch and her weight's shift the same
#: to the third digit.
STACK_EVERY = 4
#: She is down with her pelvis tilted past DOWN_DEG or under DOWN_M.
DOWN_DEG, DOWN_M = 50.0, 0.45
#: The gym's texts, machine/NAME.lisp: her moves; the language's first texts as given.
GYMS = ('moves', 'examples')


def entries(text=None, name=GYMS[0]):
    """[(label, wire, names)]: the gym's entries of `text` (machine/`name`.lisp's) - its
    payloads, each '#' and its id, then its behaviours by name, a name a row."""
    from machine import ortho
    if text is None:
        with open(os.path.join(os.path.dirname(__file__), name + '.lisp'), encoding='utf-8') as f:
            text = f.read()
    tables, phases = ortho.compiled(text)
    size = ortho.ROW.size
    stream = [tables[''][i:i + size] for i in range(0, len(tables['']), size)]
    labels = ['#%d' % ortho.rows(wire)[0][0] for wire in stream]
    return ([(label, wire, (label,)) for label, wire in zip(labels, stream)]
            + [(name, tables[name], phases[name] + ('rest',)) for name in phases])


def _loads(world, force):
    """Each sole's load now, N: the floor's contacts on its foot and toes, unfiltered."""
    m, d, out = world.model, world.data, [0.0, 0.0]
    if d.ncon:
        on = world.soles[m.geom_bodyid[d.contact.geom]].any(axis=1)
        for side in (0, 1):
            for i in on[:, side].nonzero()[0].tolist():
                world._mj.mj_contactForce(m, d, i, force)
                out[side] += float(force[0])
    return out


def laid(every=1, inner=None):
    """She stands on the floor on the loop, at rest: one dict of what a run holds - her world
    and body, the drives' loop under the stack (`inner`, INNER with none), her statics, her
    latent state, an empty table; the loop solved every `every` passes."""
    import numpy as np
    from machine import gait, latent, modal, physics, wbc
    from machine.drives import kind
    from machine.figure import JOINTS
    world, body = physics.World(), wbc.Body()
    pose = gait.stand()
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    world.reset(pose, where=(0.0, 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet']), 0.0))
    inner = inner or INNER
    world.gains[:] *= np.array(SERVO_SHARE) * (inner == 'pd')
    world.gains[~body.driven] = 0.0
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    m = body.m
    fr = modal.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]),
                     float(m.body_subtreemass[body.torso]), float(np.mean(s['q'][list(body.knees)])),
                     0.001 * every)
    rest = modal.rest(fr)
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'st': latent.state(rest),
         'out': np.zeros(latent.OUT), 'a': np.zeros(modal.ASK), 'tab': np.zeros((0, latent.TABLE)),
         'labels': [], 'posture': s['q'].copy(),
         'bend': modal.crouch([j for j, d in zip(JOINTS, body.driven) if d]),
         'legs': [[i for i, j in enumerate(JOINTS) if j.startswith(side) and kind(j) in LEG_KINDS]
                  for side in ('left_', 'right_')],
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(),
         'joints': np.flatnonzero(body.driven), 'hold': np.zeros(len(body.clamp)), 'left': WEP_S,
         'ref_q': world.data.qpos[body.qact].copy(), 'ref_v': np.zeros(len(body.qact)),
         'force': np.zeros(6), 't': float(world.data.time), 'n': 0, 'every': every, 's': s,
         'loads': [0.0, 0.0], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([world.data.qpos[body.qact], np.zeros(len(body.qact)), np.zeros(len(body.qact))]),
         'jeff': np.diag(s['M'])[body.act].copy()}
    world.clock = lambda: g['t']
    return g


def said(g, label, wire, names=(), at=None):
    """`wire`'s rows (`ortho.compiled`) begun at `at` s - now - in her table, what has passed
    dropped; `names` what she is doing a row, else `label`."""
    import numpy as np
    from machine import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] > g['t']], new])
    g['labels'] = g['labels'][-16:] + [(float(when), names[i] if i < len(names) else label)
                                       for i, when in enumerate(new[:, latent.AT])]


def doing(g):
    """What she is doing now: the name of her table's row that last came, 'rest' with none."""
    come = [(when, name) for when, name in g['labels'] if when <= g['t'] + 1e-9]
    return max(come)[1] if come else 'rest'


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a
    `dt` on. The loop is solved on every `g['every']`th, its answer held between."""
    import numpy as np
    from machine import latent, modal, physics, wbc
    world, body = g['world'], g['body']
    if g['n'] % g['every'] == 0:
        s = g['s'] = wbc.sense(body, world.data.qpos, world.data.qvel)
        loads = g['loads'] = _loads(world, g['force'])
        y = modal.sensed(s, loads, g['y'])
        out = latent.step(g['st'], g['tab'], g['t'], dt * g['every'], g['out'])
        ask = g['ask'] = modal.given(modal.ask(out, y, g['fr'], g['a']), y, g['fr'], g['posture'],
                                     g['bend'], body.knees)
        g['got'] = wbc.step(body, s, dict(ask, wep=g['left'] > 0.0))
        g['jeff'] = np.diag(s['M'])[body.act]
        for k, leg in enumerate(g['legs']):
            world.gains[leg, 0] = g['free_kp'][leg] * (1.0 + (STANDING_KP - 1.0) * float(ask['stance'][k]))
    got, held, eso, wo = g['got'], float(g['hold'].any()), g['eso'], g['wo']
    # each drive's disturbance, observed about the acceleration the loop planned for it
    e = world.data.qpos[body.qact] + physics.WOUND * world.boxes.delta[body.driven] - eso[0]
    eso += dt * np.array([eso[1] + 3.0 * wo * e, got['qacc'][body.act] + eso[2] + 3.0 * wo * wo * e,
                          wo ** 3 * e])
    world.feed[body.driven] = got['tau'] - g['jeff'] * eso[2]
    g['hold'] = np.where(got['over'], WEP_HOLD_S, np.maximum(0.0, g['hold'] - dt))
    g['left'] = min(WEP_S, g['left'] + dt * (WEP_FILL * (1.0 - held) - held))
    world.limit[g['joints']] = np.where(g['hold'] > 0.0, body.wep, g['clamp'][g['joints']])
    leak = dt / LEAK_S
    g['ref_v'] += got['qacc'][body.act] * dt + leak * (world.data.qvel[body.act] - g['ref_v'])
    g['ref_q'] += g['ref_v'] * dt + leak * (world.data.qpos[body.qact] - g['ref_q'])
    world.target[body.driven] = g['ref_q']
    g['t'] += dt
    g['n'] += 1
    world.advance()
    return g['s'], g['loads'], g['ask'], g['out']


def tilt(g):
    """Her pelvis's tilt from upright, deg."""
    q = g['world'].data.qpos
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[4] ** 2 + q[6] ** 2)))))


def state(g, torque, power, watts, ratio, fallen_at, says=()):
    """Her state as the page hears it (`machine.running`'s keys), and her modes: each (name,
    asked, as she is, N/m, N s/m); `says` the gym's labels."""
    from machine import latent, modal, ortho
    from machine.figure import JOINTS
    world, out = g['world'], g['out']
    d = world.data
    z = modal.read(g['y'], g['fr'])[0]
    angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'peak': dict(zip(JOINTS, world.peak.tolist())), 'watts': float(watts),
            'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'speed': float(d.qvel[2]), 'velocity': (float(d.qvel[0]), float(d.qvel[2])),
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': 'fallen' if fallen_at is not None else doing(g), 'fallen': fallen_at is not None,
            'slips': 0, 'stir': 0.0, 'stirs': (0.0, 0.0, 0.0), 'swing': 0.0,
            'loads': tuple(g['loads']), 'heat': {j: (25.0, 0.0, 1.0, 1) for j in JOINTS},
            'props': world.props(), 'armed': None, 'rig': None, 'doing': doing(g), 'errand': None,
            'lamp': False, 'seen': None, 'buses': [], 'style': {}, 'sway': 0.0,
            'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'says': tuple(says),
            'modes': [(name, float(out[latent.REF][i]), float(z[i]), float(out[latent.KS][i]),
                       float(out[latent.DS][i])) for i, name in enumerate(ortho.MODES)]}


def _heard(g, gym, command):
    """A command on her: an entry said, a text's payloads and behaviours begun, a push."""
    if 'say' in command and 0 <= int(command['say']) < len(gym):
        said(g, *gym[int(command['say'])])
    for entry in entries(command['text']) if 'text' in command else ():
        said(g, *entry)
    if 'push' in command:
        g['world'].push(command['push'], command.get('seconds', 0.1))


def run(commands, states, every=STACK_EVERY):
    """The worker: she stands on the loop, paced to the clock; commands heard, her state said."""
    import numpy as np
    from machine import physics
    from machine.errors import MachineError
    physics.ROOM = physics.SIGHT = 0.0
    gym, g = entries(), laid(every)
    says = [GYMS[0]] + [entry[0] for entry in gym]
    dt = 1.0 / RATE_HZ
    k = dt / AVERAGE_S
    torque, power = np.zeros(len(g['world'].peak)), np.zeros(len(g['world'].peak))
    watts, fallen_at = 0.0, None
    wall0, sim0 = time.perf_counter(), g['t']
    told, ratio, spent, ran = 0.0, 1.0, 0.0, 0.0
    while True:
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                if command.get('restart'):
                    g, fallen_at, told = laid(every), None, 0.0
                    wall0, sim0 = time.perf_counter(), g['t']
                if command.get('gym') in GYMS:
                    gym = entries(name=command['gym'])
                    says = [command['gym']] + [entry[0] for entry in gym]
                _heard(g, gym, command)
        except queue.Empty:
            pass
        began = time.perf_counter()
        due = sim0 + (began - wall0)
        if due - g['t'] > SLICE_S:
            due = g['t'] + SLICE_S
            wall0, sim0 = began, due
        if fallen_at is not None and g['t'] - fallen_at >= RECOVER_S:
            g, fallen_at, told = laid(every), None, 0.0
            wall0, sim0, due = time.perf_counter(), g['t'], g['t']
        from_t, world = g['t'], g['world']
        while g['t'] < due - 1e-9:
            try:
                if fallen_at is None:
                    passed(g, dt)
                else:
                    world.feed[:] = 0.0
                    g['t'] += dt
                    world.advance()
            except MachineError:
                fallen_at = g['t']
            if fallen_at is None and (tilt(g) > DOWN_DEG or world.data.qpos[1] < DOWN_M):
                fallen_at = g['t']
            tau = world.data.ctrl
            torque += k * (tau - torque)
            power += k * (tau * world.data.qvel[world.vadr] - power)
            watts += k * (world.drawn() - watts)
        spent += time.perf_counter() - began
        ran += g['t'] - from_t
        if g['t'] - told >= SAY_S:
            ratio = ran / spent if spent > 1e-6 else ratio
            spent = ran = 0.0
            told = g['t']
            try:
                states.put_nowait(state(g, torque, power, watts, ratio, fallen_at, says))
            except queue.Full:
                pass
        rest = SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)
