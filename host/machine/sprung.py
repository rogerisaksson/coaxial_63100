"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running()                # her process: on the loop she walks from rest (WALK)
    body.send(say=2)                       # the gym's third entry: a payload injected, a behaviour begun
    body.send(text='(stream_inject ..)')   # a text's payloads and behaviours, begun now
    body.send(push=(0.0, 0.0, 80.0), seconds=0.12); body.send(restart=True)
    body.send(gym='examples')              # the gym's other text (GYMS), read anew each time
    now = body.latest()                    # her state as last said, None before the first
    body.close()
    sprung.entries()                       # [(label, wire, names)]: machine/moves.lisp's

    g = sprung.laid()                      # the same loop without the process: one dict a run holds
    sprung.said(g, label, wire, names)     # rows begun now
    s, loads, ask, out = sprung.passed(g)  # a pass, her world a millisecond on

A pass: sensed (`wbc.sense`), latent state stepped (`machine.latent`), whole body solved
(`wrench.step`), torques fed forward with ESO disturbance, and world advanced. The worker
runs at RATE_HZ, catches up to SLICE_S, and publishes state every SAY_S. Down, she recovers.
"""
import math
import multiprocessing
import os
import queue
import time

#: The loop's rate, Hz. The most simulated time one slice catches up, s; how often her state
#: goes out, s. Each drive's torque and power as said, meaned over AVERAGE_S. Down, she is
#: laid anew RECOVER_S on.
RATE_HZ, SLICE_S, SAY_S, AVERAGE_S, RECOVER_S = 1000.0, 0.05, 1.0 / 60.0, 0.05, 3.0
#: The shoves, newtons for SHOVE_S: a nudge 0.13 m/s on her, a push, a shove.
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
#: What the worker hears (`Running.send`).
HEARD = ('say', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading')
#: Her walk from rest: her moves' payloads (GYMS[0]) and the second each is said at.
WALK = (('#201', 0.5), ('#202', 2.0), ('#203', 5.0), ('#204', 9.0))
#: Her braking stride when asked to stop or switch from a walk: capture-point stepping kept
#: live (drive > STIRS) while lateral and sagittal momentum damp before #201 locks her soles.
BRAKE = ('(stream_inject (sequence_id 200) (blend_duration 450ms)'
         ' (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0))'
         ' (impedance (stiffness mode_axial_compression 8000) (damping mode_axial_compression 1000)'
         ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600)'
         ' (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)'
         ' (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
         ' (cpg_config (enable true) (frequency 0.83Hz) (amplitude mode_lateral_shift 0.28)'
         ' (amplitude mode_sagittal_drive 0.04) (amplitude mode_limb_lift 0.8) (amplitude mode_foot_rock 0.0)'
         ' (amplitude mode_pelvis_list 0.15) (amplitude mode_pelvis_turn 0.1) (amplitude mode_arm_swing 0.1)'
         ' (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)'
         ' (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))'
         ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')

#: The drives' PD under the loop (SERVO gains * SERVO_SHARE: kp, kd); WEP temporary boost.
SERVO_SHARE, LEAK_S, STANDING_KP = (0.5, 1.0), 0.2, 0.1
WEP_HOLD_S, WEP_S, WEP_FILL = 0.6, 5.0, 0.5
#: The drives' own loop under the stack: 'pd' that PD; 'eso' each drive's disturbance observed.
INNER, ESO_WO = 'eso', 100.0
#: The whole-body step under her modes: 'wrench' the step without an active set (wrench.step).
SOLVER = 'wrench'
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
#: A knee straighter than KNEE_SOFT rad is sprung back by its own drive, KNEE_NM at straight.
KNEE_SOFT, KNEE_NM = 0.1, 40.0
#: On the page the loop is solved every STACK_EVERY passes.
STACK_EVERY = 3
#: She is down with her pelvis tilted past DOWN_DEG or under DOWN_M.
DOWN_DEG, DOWN_M = 50.0, 0.45
#: The gym's texts, machine/NAME.lisp: her moves; the language's first texts as given.
GYMS = ('moves', 'examples')
#: Modes and phases for stunts (sitting, lying, hopping, controlled fall) - not treated as accidental falls.
STUNT_MODES = {'sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor',
               'rise_floor', 'hop_down_chair', 'hop_up_chair', 'stairs_down',
               'parry_shove', 'controlled_fall'}
STUNT_PHASES = {'approach', 'lower', 'seated', 'lean_forward', 'thrust', 'sit_edge', 'recline',
                'bed_rest', 'sit_up', 'swing_out', 'kneel', 'floor_rest', 'push_arms',
                'half_kneel', 'cushion', 'land', 'step_off', 'drop', 'absorb', 'crouch_load',
                'propulsion', 'flight_tuck', 'seat_land', 'descent'}


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


def _seen(body, world, limbs):
    """`wbc.sense`'s of her as she is, her hips' joints and her shoulders' reach beside it."""
    from machine import wbc
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    return s


def laid(every=1, inner=None, solver=None):
    """She stands on the floor on the loop, at rest: one dict of what a run holds - her world
    and body, the drives' loop under the stack (`inner`, INNER with none), her statics, her
    latent state, an empty table; the loop solved every `every` passes by `solver` (SOLVER
    with none)."""
    import numpy as np
    from machine import figure, latent, modal, physics, wbc, wrench
    from machine.drives import kind
    from machine.figure import JOINTS
    world, body = physics.World(), wbc.Body()
    pose = figure.stand()
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    world.reset(pose, where=(0.0, 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet']), 0.0))
    inner = inner or INNER
    world.gains[:] *= np.array(SERVO_SHARE) * (inner == 'pd')
    world.gains[~body.driven] = 0.0
    m = body.m
    names = [j for j, d in zip(JOINTS, body.driven) if d]
    limbs = {'hips': [m.body(side + 'thigh').id for side in ('left_', 'right_')],
             'shoulders': [names.index(side + 'shoulder') for side in ('left_', 'right_')]}
    s = _seen(body, world, limbs)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    fr = modal.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]),
                     float(m.body_subtreemass[body.torso]),
                     float(m.body_subtreemass[m.body('left_upper_arm').id]),
                     float(np.mean(s['q'][list(body.knees)])), 0.001 * every)
    rest = modal.rest(fr)
    rest[latent.K][modal.AXIAL] = 8000.0
    rest[latent.D][modal.AXIAL] = 1000.0
    rest[latent.K][modal.LATERAL] = 4500.0
    rest[latent.D][modal.LATERAL] = 500.0
    rest[latent.K][modal.SAGITTAL] = 1500.0
    rest[latent.D][modal.SAGITTAL] = 350.0
    st = latent.state(rest)
    tab = np.zeros((0, latent.TABLE))
    out = latent.step(st.copy(), tab, 0.0, 0.001 * every, np.zeros(latent.OUT))
    a = np.zeros(modal.ASK)
    a[modal.STANCE] = 1.0
    half_w = 0.5 * float(fr[modal.WEIGHT])
    robot_bids = set()
    for i in range(m.nbody):
        p = i
        while p > 0:
            if p == world.pelvis:
                robot_bids.add(i)
                break
            p = m.body_parentid[p]
    sole_bids = set(world.soles.nonzero()[0])
    upper_bids = np.array(sorted(robot_bids - sole_bids), dtype=int)
    door_b = m.body('door').id if 'door' in [m.body(i).name for i in range(m.nbody)] else None
    if door_b is not None:
        robot_bids.add(door_b)
    robot_ids = np.array(sorted(robot_bids), dtype=int)
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'st': st,
         'out': out, 'a': a, 'tab': tab,
         'labels': [], 'posture': s['q'].copy(), 'limbs': limbs,
         'bend': modal.crouch(names), 'arms': modal.swing(names),
         'legs': [[i for i, j in enumerate(JOINTS) if j.startswith(side) and kind(j) in LEG_KINDS]
                  for side in ('left_', 'right_')],
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(),
         'joints': np.flatnonzero(body.driven), 'hold': np.zeros(len(body.clamp)), 'left': WEP_S,
         'ref_q': world.data.qpos[body.qact].copy(), 'ref_v': np.zeros(len(body.qact)),
         'force': np.zeros(6), 't': float(world.data.time), 'n': 0, 'every': every, 's': s,
         'loads': [half_w, half_w], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([world.data.qpos[body.qact], np.zeros(len(body.qact)), np.zeros(len(body.qact))]),
         'jeff': np.diag(s['M'])[body.act].copy(),
         'robot_ids': robot_ids,
         'upper_ids': upper_bids,
         'soles_l': np.flatnonzero(world.soles[:, 0]),
         'soles_r': np.flatnonzero(world.soles[:, 1]),
         'stumble_until': 0.0,
         'heading': float(fr[modal.HEADING]),
         'scenario': 'flat',
         'solve': {'stack': wbc.step, 'wrench': wrench.step}[solver or SOLVER]}
    world.clock = lambda: g['t']
    return g


def said(g, label, wire, names=(), at=None, reset=False):
    """`wire`'s rows (`ortho.compiled`) begun at `at` s - now - in her table, what has passed
    dropped; `names` what she is doing a row, else `label`; `reset` fills unsaid of the first
    with her rest."""
    import numpy as np
    from machine import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    if reset and len(new):
        mask = new[0, latent.SAID] > 0.5
        new[0, latent.VALS] = np.where(mask, new[0, latent.VALS], g['rest'])
        new[0, latent.SAID] = 1.0
    if g['st'][latent.NOW][latent.GAIN] < 1e-6 and len(new):
        ph = new[0, latent.SAID][latent.PHASE] > 0.5
        g['st'][latent.NOW][latent.PHASE] = np.where(ph, new[0, latent.VALS][latent.PHASE],
                                                     g['st'][latent.NOW][latent.PHASE])
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] >= g['t']], new])
    g['labels'] = g['labels'][-16:] + [(float(when), names[i] if i < len(names) else label)
                                       for i, when in enumerate(new[:, latent.AT])]


def walked(g):
    """Her walk from rest (WALK) said from now, in place of what was to come."""
    by = {entry[0]: entry for entry in entries()}
    _dropped(g)
    for label, at in WALK:
        said(g, *by[label], at=g['t'] + at)


def _dropped(g):
    """Her table's rows still to come dropped, and what they were to be called."""
    g['tab'] = g['tab'][:0]
    g['labels'] = [(when, name) for when, name in g['labels'] if when <= g['t']]


def doing(g):
    """What she is doing now: the name of her table's row that last came, 'rest' with none."""
    come = [(when, name) for when, name in g['labels'] if when <= g['t'] + 1e-9]
    return max(come)[1] if come else 'rest'


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a
    `dt` on. The loop is solved on every `g['every']`th, its answer held between."""
    import numpy as np
    from machine import impedance, latent, modal, physics, wbc
    world, body = g['world'], g['body']
    if g['n'] % g['every'] == 0:
        s = g['s'] = _seen(body, world, g['limbs'])
        loads = g['loads'] = _loads(world, g['force'])
        y = modal.sensed(s, loads, g['y'])
        q = world.data.qpos[3:7]
        pelvis_yaw = 2.0 * math.atan2(float(q[2]), float(q[0]))
        d_yaw = math.atan2(math.sin(pelvis_yaw - g['heading']), math.cos(pelvis_yaw - g['heading']))
        g['heading'] += 0.005 * g['every'] * d_yaw
        s_hd, c_hd = math.sin(g['heading']), math.cos(g['heading'])
        g['fr'][modal.HEADING] = g['heading']
        g['fr'][modal.ROT] = np.array([[s_hd, 0.0, c_hd], [c_hd, 0.0, -s_hd], [0.0, 1.0, 0.0]]).ravel()
        if g['st'][latent.NOW][latent.GAIN] < 1e-6:
            g['st'][latent.THETA] = 0.0
        out = latent.step(g['st'], g['tab'], g['t'], dt * g['every'], g['out'])
        d = world.data
        if d.ncon:
            c_bids = world.model.geom_bodyid[d.contact.geom[:d.ncon]]
            b0, b1 = c_bids[:, 0], c_bids[:, 1]
            upper_hit = ((np.isin(b0, g['upper_ids']) & (b1 > 0) & ~np.isin(b1, g['robot_ids'])) |
                         (np.isin(b1, g['upper_ids']) & (b0 > 0) & ~np.isin(b0, g['robot_ids'])))
            if upper_hit.any():
                out[latent.SWAY][modal.DRIVE] = 0.0
                out[latent.REF][modal.DRIVE] = 0.0
                g['a'][modal.GOING] = 0.0
            aloft = g['a'][modal.STANCE] < 0.5
            hit_l = aloft[0] and (
                (np.isin(b0, g['soles_l']) & (b1 > 0) & ~np.isin(b1, g['robot_ids'])) |
                (np.isin(b1, g['soles_l']) & (b0 > 0) & ~np.isin(b0, g['robot_ids']))).any()
            hit_r = aloft[1] and (
                (np.isin(b0, g['soles_r']) & (b1 > 0) & ~np.isin(b1, g['robot_ids'])) |
                (np.isin(b1, g['soles_r']) & (b0 > 0) & ~np.isin(b0, g['robot_ids']))).any()
            if hit_l or hit_r:
                g['stumble_until'] = g['t'] + 0.35
        if g['t'] < g['stumble_until']:
            out[latent.SWAY][modal.LIFT] = max(float(out[latent.SWAY][modal.LIFT]), 2.8)
            out[latent.REF][modal.LIFT] = max(float(out[latent.REF][modal.LIFT]), 0.6)
        ask = g['ask'] = modal.given(impedance.ask(out, y, g['fr'], g['a']), y, g['fr'], g['posture'],
                                     g['bend'], g['arms'], body.knees)
        g['got'] = g['solve'](body, s, dict(ask, wep=g['left'] > 0.0))
        g['jeff'] = np.diag(s['M'])[body.act]
        for k, leg in enumerate(g['legs']):
            world.gains[leg, 0] = g['free_kp'][leg] * (1.0 + (STANDING_KP - 1.0) * float(ask['stance'][k]))
    got, held, eso, wo = g['got'], float(g['hold'].any()), g['eso'], g['wo']
    # a knee's soft stop, and what it adds to the acceleration planned for it
    sprung_back = np.zeros(len(body.act))
    qk = world.data.qpos[body.qact][list(body.knees)]
    sprung_back[list(body.knees)] = (
        KNEE_NM * np.clip(1.0 - qk / KNEE_SOFT, 0.0, 3.0)
        + 2400.0 * np.maximum(0.0, -qk)
    )
    # each drive's disturbance, observed about the acceleration the loop planned for it
    e = world.data.qpos[body.qact] + physics.WOUND * world.boxes.delta[body.driven] - eso[0]
    eso += dt * np.array([eso[1] + 3.0 * wo * e,
                          got['qacc'][body.act] + sprung_back / g['jeff'] + eso[2] + 3.0 * wo * wo * e,
                          wo ** 3 * e])
    world.feed[body.driven] = got['tau'] + sprung_back - g['jeff'] * eso[2]
    g['hold'] = np.where(got['over'], WEP_HOLD_S, np.maximum(0.0, g['hold'] - dt))
    g['left'] = min(WEP_S, max(0.0, g['left'] + dt * (WEP_FILL * (1.0 - held) - held)))
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


def state(g, torque, power, watts, ratio, fallen_at, says=(), refused=None):
    """Her state as the page hears it - her stage 'walk' while she strides, 'stand' or 'fallen'
    - and her modes: each (name, asked, as she is, N/m, N s/m); `says` the gym's labels,
    `refused` why its last text or command was."""
    from machine import latent, modal, ortho, stride
    from machine.figure import JOINTS
    world, out = g['world'], g['out']
    d = world.data
    z = modal.read(g['y'], g['fr'], g['a'][modal.STANCE])[0]
    angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'peak': dict(zip(JOINTS, world.peak.tolist())), 'watts': float(watts),
            'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'speed': float(d.qvel[2]), 'velocity': (float(d.qvel[0]), float(d.qvel[2])),
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': ('fallen' if fallen_at is not None
                      else 'walk' if abs(float(out[latent.SWAY][modal.DRIVE])) > stride.STIRS else 'stand'),
            'fallen': fallen_at is not None,
            'loads': tuple(g['loads']), 'props': world.props(), 'doing': doing(g),
            'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'says': tuple(says), 'refused': refused,
            'scenario': g.get('scenario', 'flat'),
            'modes': [(name, float(out[latent.REF][i]), float(z[i]), float(out[latent.KS][i]),
                       float(out[latent.DS][i])) for i, name in enumerate(ortho.MODES)]}


def _start_mode(g, by, entry, at):
    from machine import ortho
    name, z = entry[0], float(g['world'].data.qpos[2])
    iw = by['init_walk'][1][:ortho.ROW.size] if 'init_walk' in by else None
    if name == 'ballistic_walk':
        said(g, *entry, at=at, reset=True)
    elif name == '#202':
        said(g, *by['#202'], at=at)
    elif name == '#203' and '#202' in by:
        said(g, *by['#202'], at=at)
        said(g, *by['#203'], at=at + 0.80)
    elif name in ('#204', 'trip', 'jog', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk'):
        if iw is not None:
            said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
            said(g, *by[name], at=at + 0.65)
        else:
            said(g, *by[name], at=at)
    elif name in ('sit_chair', 'hop_up_chair') and z < 0.80 and iw is not None and '#204' in by:
        said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
        said(g, *by['#204'], at=at + 0.65)
        said(g, *by[name], at=at + 1.80)
    elif name in ('lie_bed', 'lie_floor') and z < 0.65 and iw is not None and '#204' in by:
        said(g, 'init_walk', iw, ('init_walk',), at=at, reset=True)
        said(g, *by['#204'], at=at + 0.65)
        said(g, *by[name], at=at + 1.50)
    elif name == '#201':
        said(g, *entry, at=at, reset=True)
    else:
        said(g, *entry, at=at)


def _heard_say(g, gym, entry):
    from machine import latent, modal, ortho, stride
    by = {e[0]: e for e in gym}
    drive = float(g['out'][latent.SWAY][modal.DRIVE])
    striding = abs(drive) > stride.STIRS
    back_now = float(g['st'][latent.NOW][latent.PHASE][modal.LATERAL]) < -0.1
    spd = math.hypot(float(g['world'].data.qvel[0]), float(g['world'].data.qvel[2]))
    unbalanced = abs(float(g['loads'][0]) - float(g['loads'][1])) > 90.0
    off_center = not striding and (
        abs(float(g['out'][latent.REF][modal.LIFT])) > 0.15
        or abs(float(g['out'][latent.REF][modal.SAGITTAL])) > 0.15
        or abs(float(g['out'][latent.REF][modal.LATERAL])) > 0.15
        or abs(float(g['out'][latent.REF][modal.AXIAL])) > 0.25
        or spd > 0.04
        or unbalanced
    )
    fwd_gaits = ('#204', 'trip', 'jog', 'ballistic_walk', 'stairs_walk', 'stairs_down',
                 'ice_walk', 'slope_walk', 'strafe_left', 'strafe_right', 'turn_left',
                 'turn_right', 'circle_walk', 'turn_back_left', 'turn_back_right')
    t0 = g['t'] + 0.001
    if doing(g) in ('down', 'hold', 'load', 'throw', 'land') and len(g['tab']):
        g['tab'] = g['tab'][g['tab'][:, latent.AT] <= g['t'] + 1.5]
        t0 = (float(g['tab'][-1, latent.AT]) if len(g['tab']) else g['t']) + 0.05
        off_center = True
    else:
        _dropped(g)
    same_gait = (entry[0] in fwd_gaits and not back_now) or (entry[0] == '#205' and back_now)
    in_apa = doing(g) == '#202'
    if entry[0] in ('parry_shove', 'controlled_fall'):
        said(g, *entry, at=t0, reset=True)
        return
    if striding and '#201' in by and not same_gait:
        brk = BRAKE.replace('108deg', '-72deg') if back_now else BRAKE
        said(g, 'brake', ortho.compiled(brk)[0][''], ('brake',), at=t0, reset=True)
        said(g, *by['#201'], at=t0 + 1.20, reset=True)
        if entry[0] != '#201':
            _start_mode(g, by, entry, at=t0 + 2.40)
        return
    if striding and same_gait:
        if g.get('active_entry') == entry[0]:
            return
        g['active_entry'] = entry[0]
        said(g, *entry, at=t0)
        return
    if in_apa and entry[0] in (('#203',) + fwd_gaits):
        said(g, *by['#203'], at=t0, reset=True)
        if entry[0] != '#203' and entry[0] in by:
            said(g, *by[entry[0]], at=t0 + 0.90)
        return
    if doing(g) in ('over', 'touch') and entry[0] != 'one_sole':
        touch = ('(stream_inject (sequence_id 200) (blend_duration 500ms)'
                 ' (latent_targets (mode_axial_compression 0.0) (mode_lateral_shift 0.0)'
                 '                 (mode_limb_lift 0.0) (mode_pelvis_list 0.0))'
                 ' (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 800)'
                 '            (stiffness mode_lateral_shift 6000) (damping mode_lateral_shift 600)'
                 '            (stiffness mode_pelvis_list 1500) (damping mode_pelvis_list 300))'
                 ' (cpg_config (enable false))'
                 ' (task_constraints (maintain_zmp (polygon support_polygon))))')
        said(g, 'touch', ortho.compiled(touch)[0][''], ('touch',), at=t0)
        said(g, *by['#201'], at=t0 + 0.80, reset=True)
        if entry[0] != '#201':
            _start_mode(g, by, entry, at=t0 + 2.00)
        return
    if off_center and '#201' in by and entry[0] not in (('#201',) + tuple(STUNT_MODES)):
        said(g, *by['#201'], at=t0, reset=True)
        _start_mode(g, by, entry, at=t0 + 1.35)
        return
    if not striding:
        _start_mode(g, by, entry, at=t0)
        return
    said(g, *entry, at=t0)


def _heard(g, gym, command):
    """A command on her: an entry said in place of what was to come, her walk from rest, a
    text's entries begun, a push. One the loop does not know is refused: nothing else runs
    her."""
    from machine import latent, modal, ortho, stride
    from machine.errors import MachineError
    unknown = sorted(set(command) - set(HEARD))
    if unknown:
        raise MachineError('the loop hears %s: not %s' % (', '.join(HEARD), ', '.join(unknown)))
    if ('say' in command or command.get('walk') or command.get('entries')) and g['st'][latent.NOW][latent.GAIN] < 1e-6:
        g['st'][latent.THETA] = 0.0
    if 'say' in command:
        say_cmd = command['say']
        entry = None
        if isinstance(say_cmd, int) or (isinstance(say_cmd, str) and say_cmd.isdigit()):
            idx = int(say_cmd)
            if 0 <= idx < len(gym):
                entry = gym[idx]
        elif isinstance(say_cmd, str):
            for e in gym:
                if e[0] == say_cmd:
                    entry = e
                    break
        if entry is not None:
            _heard_say(g, gym, entry)
    if command.get('walk'):
        by = {e[0]: e for e in gym}
        if '#204' in by:
            _start_mode(g, by, by['#204'], at=g['t'] + 0.001)
        else:
            walked(g)
    for entry in command.get('entries', ()):
        said(g, *entry)
    if 'scenario' in command:
        scen = command['scenario']
        g['scenario'] = scen
        g['world'].scenario(scen)
    if 'heading' in command:
        g['heading'] = float(command['heading'])
    if 'push' in command:
        force = command['push']
        secs = command.get('seconds', SHOVE_S)
        g['world'].push(force, secs)
        mag = math.hypot(*force) if hasattr(force, '__iter__') else float(force)
        t0 = g['t'] + 0.001
        by = {e[0]: e for e in gym}
        if mag > 120.0 and 'controlled_fall' in by:
            _dropped(g)
            said(g, *by['controlled_fall'], at=t0, reset=True)
        elif mag >= 35.0 and 'parry_shove' in by:
            _dropped(g)
            said(g, *by['parry_shove'], at=t0, reset=True)


def run(commands, states, every=STACK_EVERY, walks=False):
    """The worker: she stands on the loop, paced to the clock; commands heard, her state said.
    Laid, she stands at rest unless `walks`."""
    import numpy as np
    from machine import physics
    from machine.errors import MachineError
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    def anew():
        g = laid(every)
        if walks:
            walked(g)
        return g
    def _put(item):
        while states.full():
            try:
                states.get_nowait()
            except queue.Empty:
                break
        try:
            states.put_nowait(item)
        except queue.Full:
            pass

    gym, g = entries(), anew()
    says, refused = [GYMS[0]] + [entry[0] for entry in gym], None
    dt = 1.0 / RATE_HZ
    k = dt / AVERAGE_S
    torque, power = np.zeros(len(g['world'].peak)), np.zeros(len(g['world'].peak))
    watts, fallen_at = 0.0, None
    wall0, sim0 = time.perf_counter(), g['t']
    told, ratio, spent, ran = 0.0, 1.0, 0.0, 0.0
    _put(state(g, torque, power, watts, ratio, fallen_at, says, refused))
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
                    _put(state(g, torque, power, watts, ratio, fallen_at, says, refused))
                try:
                    if command.get('gym') in GYMS:
                        gym = entries(name=command['gym'])
                        says, refused = [command['gym']] + [entry[0] for entry in gym], None
                    if 'text' in command:
                        command, refused = dict(command, entries=entries(command['text'])), None
                    _heard(g, gym, command)
                except (SyntaxError, MachineError) as error:
                    refused = str(error)
        except queue.Empty:
            pass
        began = time.perf_counter()
        due = sim0 + (began - wall0)
        if due - g['t'] > SLICE_S:
            due = g['t'] + SLICE_S
            wall0, sim0 = began, due
        if fallen_at is not None and g['t'] - fallen_at >= RECOVER_S:
            g, fallen_at, told = anew(), None, 0.0
            torque[:], power[:], watts = 0.0, 0.0, 0.0
            wall0, sim0, due = time.perf_counter(), g['t'], g['t']
            _put(state(g, torque, power, watts, ratio, fallen_at, says, refused))
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
            if fallen_at is None and doing(g) not in STUNT_PHASES and (tilt(g) > DOWN_DEG or world.data.qpos[1] < DOWN_M):
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
            _put(state(g, torque, power, watts, ratio, fallen_at, says, refused))
        rest = SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """Her worker process on the loop (`run`): `send` it commands, read its `latest` state;
    laid, she stands at rest unless `walks`."""

    def __init__(self, every=STACK_EVERY, walks=False):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states = context.Queue(), context.Queue(maxsize=8)
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
        self._commands.put(None)
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
