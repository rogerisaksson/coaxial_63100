"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running(); body.send(say=2); now = body.latest(); body.close()
    g = sprung.laid(); s, loads, ask, out = sprung.passed(g)
"""
import math
import os
from machine import edge, estimate, flight, grip, ortho

RATE_HZ, SLICE_S, SAY_S, AVERAGE_S, RECOVER_S = 1000.0, 0.05, 1.0 / 60.0, 0.05, 3.0
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
#: Her stage (`staged`): walking past WALK_M_S, or with a sole off the floor within STEP_S.
WALK_M_S, STEP_S = 0.1, 1.0
HEARD = ('say', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading', 'twist')
WALK = (('#201', 0.5), ('#202', 2.0), ('#203', 5.0), ('#204', 9.0))
BRAKE = ('(stream_inject (sequence_id 200) (blend_duration 500ms) (latent_targets (mode_axial_compression 0.04)'
         ' (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0)'
         ' (mode_stance_width 0.0)) (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)'
         ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300) (stiffness mode_sagittal_shift 0)'
         ' (damping mode_sagittal_shift 260) (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
         ' (cpg_config (enable true) (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.04)'
         ' (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.0)'
         ' (amplitude mode_pelvis_turn 0.0) (amplitude mode_arm_swing 0.0) (amplitude mode_stance_width 0.0)'
         ' (phase_offset mode_lateral_shift mode_sagittal_drive 108deg) (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))'
         ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')
_TOUCH = ('(stream_inject (sequence_id 200) (blend_duration 500ms) (latent_targets (mode_axial_compression 0.0)'
          ' (mode_lateral_shift 0.0) (mode_limb_lift 0.0) (mode_pelvis_list 0.0)) (impedance'
          ' (stiffness mode_axial_compression 6000) (damping mode_axial_compression 800) (stiffness mode_lateral_shift 6000)'
          ' (damping mode_lateral_shift 600) (stiffness mode_pelvis_list 1500) (damping mode_pelvis_list 300))'
          ' (cpg_config (enable false)) (task_constraints (maintain_zmp (polygon support_polygon))))')
_BRAKE_WIRE, _BRAKE_BACK_WIRE, _TOUCH_WIRE = ortho.compiled(BRAKE)[0][''], ortho.compiled(BRAKE.replace('108deg', '-108deg'))[0][''], ortho.compiled(_TOUCH)[0]['']
SERVO_SHARE, LEAK_S, STANDING_KP, WEP_HOLD_S, WEP_S, WEP_FILL = (0.5, 1.0), 0.2, 0.1, 1.0, 8.0, 0.8
INNER, ESO_WO, SOLVER = 'eso', 100.0, 'wrench'
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
_Q_LIM = {'hip_yaw': (-0.44, 0.44), 'hip_roll': (-0.61, 0.61), 'hip': (-2.0, 0.78), 'knee': (0.05, 2.70),
          'ankle': (-0.78, 0.73), 'ankle_roll': (-0.26, 0.26), 'waist': (-0.78, 0.78), 'spine': (-0.44, 1.31),
          'spine_roll': (-0.52, 0.52), 'shoulder': (-1.13, 2.62), 'elbow': (0.0, 2.62), 'neck': (-0.35, 0.45), 'head': (-0.85, 0.85)}
KNEE_SOFT, KNEE_NM, STACK_EVERY, DOWN_DEG, DOWN_M = 0.1, 40.0, 3, 50.0, 0.45
GYMS = ('gynoid_mesomorph_medium', 'examples')
STUNT_MODES = {'sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor', 'rise_floor', 'hop_down_chair',
               'hop_up_chair', 'stairs_down', 'parry_shove', 'controlled_fall'}
STUNT_PHASES = {'approach', 'lower', 'seated', 'lean_forward', 'thrust', 'sit_edge', 'recline', 'bed_rest', 'sit_up',
                'swing_out', 'kneel', 'floor_rest', 'push_arms', 'half_kneel', 'cushion', 'land', 'step_off', 'drop',
                'absorb', 'crouch_load', 'propulsion', 'flight_tuck', 'seat_land', 'descent'}
_FWD_GAITS = ('#204', 'trip', 'jog', 'ballistic_walk', 'stairs_walk', 'stairs_down', 'ice_walk', 'slope_walk',
              'strafe_left', 'strafe_right', 'turn_left', 'turn_right', 'circle_walk')
_BACK_GAITS = ('#205', 'turn_back_left', 'turn_back_right')


def entries(text=None, name=GYMS[0]):
    """[(label, wire, names)]: the gym's entries of `text` (machine/`name`.lisp's)."""
    if text is None:
        with open(os.path.join(os.path.dirname(__file__), name + '.lisp'), encoding='utf-8') as f:
            text = f.read()
    tables, phases = ortho.compiled(text)
    sz = ortho.ROW.size
    stream = [tables[''][i:i + sz] for i in range(0, len(tables['']), sz)]
    return ([('#%d' % ortho.rows(w)[0][0], w, ('#%d' % ortho.rows(w)[0][0],)) for w in stream]
            + [(n, tables[n], phases[n] + ('rest',)) for n in phases])


def laid(every=1, inner=None, solver=None):
    """She stands on the floor on the loop, at rest: one dict of what a run holds."""
    import numpy as np
    from machine import figure, heat, latent, modal, physics, servos, wbc, wrench
    from machine.drives import kind, peak
    from machine.figure import JOINTS
    world, body, pose, inner = physics.World(), wbc.Body(), figure.stand(), (inner or INNER)
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    rest_y = 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(pose, where=(0.0, rest_y, 0.0))
    world.gains[:] *= np.array(SERVO_SHARE) * (inner == 'pd')
    world.gains[~body.driven] = 0.0
    m, names = body.m, [j for j, d in zip(JOINTS, body.driven) if d]
    limbs = {'hips': [m.body(s0 + 'thigh').id for s0 in ('left_', 'right_')], 'shoulders': [names.index(s0 + 'shoulder') for s0 in ('left_', 'right_')]}
    sense, est = edge.laid(world), estimate.laid(body, world.data.qpos[0:3])
    s = _sensed(body, est, sense, limbs)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    fr = modal.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]), float(m.body_subtreemass[body.torso]),
                     float(m.body_subtreemass[m.body('left_upper_arm').id]), float(np.mean(s['q'][list(body.knees)])), 0.001 * every)
    rest = modal.rest(fr)
    rest[latent.K][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (8000.0, 4500.0, 1500.0)
    rest[latent.D][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (1000.0, 500.0, 350.0)
    st, tab = latent.state(rest), np.zeros((0, latent.TABLE))
    out = latent.step(st.copy(), tab, 0.0, 0.001 * every, np.zeros(latent.OUT))
    joints = np.flatnonzero(body.driven)
    a, hw, q0 = np.zeros(modal.ASK), 0.5 * float(fr[modal.WEIGHT]), sense[edge.Q][joints]
    a[modal.STANCE] = 1.0
    legs = [[i for i, j in enumerate(JOINTS) if j.startswith(s0) and kind(j) in LEG_KINDS] for s0 in ('left_', 'right_')]
    wep_lim = np.array([max(float(servos.SERVO[kind(j)][0]), float(peak(j)), 1.45 * float(body.clamp[i])) for i, j in enumerate(names)])
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'rest_y': rest_y, 'st': st, 'out': out, 'a': a, 'tab': tab,
         'labels': [], 'posture': s['q'].copy(), 'limbs': limbs, 'bend': modal.crouch(names), 'arms': modal.swing(names),
         'cervical': [names.index(j) for j in ('neck', 'head') if j in names], 'waist_idx': names.index('waist') if 'waist' in names else None,
         'head_idx': names.index('head') if 'head' in names else None, '_look_yaw': 0.0,
         'legs': legs, 'leg_joints': np.concatenate(legs), 'leg_kp': world.gains[np.concatenate(legs), 0].copy(),
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(), 'wep_lim': wep_lim, 'joints': joints,
         'hold': np.zeros(len(body.clamp)), 'left': WEP_S, 'ref_q': q0.copy(), 'ref_v': np.zeros(len(q0)),
         'sense': sense, 'est': est, 'grip': grip.laid(),
         'grip_at': np.array([[m.body(s0 + part).id for part in ('foot', 'toes')] for s0 in ('left_', 'right_')]),
         'q_lo': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[0] for j in names]), 'q_hi': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[1] for j in names]),
         't': float(world.data.time), 'n': 0, 'every': every, 's': s, 'loads': [hw, hw], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([q0, np.zeros(len(q0)), np.zeros(len(q0))]), 'jeff': np.diag(s['M'])[body.act].copy(),
         '_brake_wire': _BRAKE_WIRE, '_stand_entry': next((e for e in entries() if e[0] == '#201'), None),
         'heading': float(fr[modal.HEADING]), 'twist': (0.0, 0.0, 0.0),
         'heat': heat.Heat.for_joints(JOINTS), 'temp': np.full(len(JOINTS), 25.0), 'scenario': 'flat',
         'solve': {'stack': wbc.step, 'wrench': wrench.step}[solver or SOLVER]}
    world.clock = lambda: g['t']
    return g


def _sensed(body, est, sense, limbs):
    """`wbc.sense`'s view of her at her lay, as her sensors and her estimate have her."""
    from machine import wbc
    qpos, qvel = estimate.step(est, body, sense, 0.0)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    return s


def said(g, label, wire, names=(), at=None, reset=False):
    import numpy as np
    from machine import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    if reset and len(new):
        new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
        new[0, latent.SAID] = 1.0
    t_eat = g['t'] - 0.001 * ((g['n'] - 1) % g['every'] + 1) + 1e-6 if g.get('n', 0) > 0 else -1.0
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] > t_eat], new])
    g['labels'] = g['labels'][-16:] + [(float(w), names[i] if i < len(names) else label) for i, w in enumerate(new[:, latent.AT])]


def walked(g):
    by = {e[0]: e for e in entries()}
    _dropped(g)
    for label, at in WALK:
        said(g, *by[label], at=g['t'] + at)


def _dropped(g):
    g['tab'], g['labels'], g['_sync_cpg_at'], g['_jump_until'] = g['tab'][:0], [(w, n) for w, n in g['labels'] if w <= g['t']], None, 0.0


def doing(g):
    """What she is doing now: the name of her table's row that last came, 'rest' with none."""
    come = [(w, n) for w, n in g['labels'] if w <= g['t'] + 1e-9]
    return max(come)[1] if come else 'rest'


def reset_for_scenario(g, scen):
    import numpy as np
    from machine import figure, latent, modal
    world, body, d = g['world'], g['body'], g['world'].data
    if (math.hypot(float(d.qpos[0]), float(d.qpos[2])) > 0.08 or abs(float(d.qpos[1]) - g['rest_y']) > 0.02
            or tilt(g) > 2.0 or doing(g) != 'rest' or len(g['tab']) > 0):
        world.reset(figure.stand(), where=(0.0, g['rest_y'], 0.0))
        g['est'], g['grip'] = estimate.laid(body, d.qpos[0:3]), grip.laid()
        s = g['s'] = _sensed(body, g['est'], edge.read(world, g['sense']), g['limbs'])
        hw, q0 = 0.5 * float(g['fr'][modal.WEIGHT]), g['sense'][edge.Q][g['joints']]
        g['loads'], g['heading'], g['target_heading'] = [hw, hw], 0.0, None
        modal.sensed(s, g['loads'], g['y'])
        g['fr'][modal.HEADING], g['fr'][modal.FLOOR], g['fr'][modal.GROUND] = 0.0, g.get('floor0', 0.0), 0.0
        g['fr'][modal.FLOOR_AT] = g['y'][modal.SOLE].reshape(2, 3)[:, [0, 2]].mean(axis=0)
        g['fr'][modal.ROT] = np.array([0, 0, 1, 1, 0, 0, 0, 1, 0], float)
        g['st'] = latent.state(g['rest'])
        g['out'] = latent.step(g['st'].copy(), g['tab'][:0], g['t'], 0.001 * g['every'], np.zeros(latent.OUT))
        g['a'][:], g['a'][modal.STANCE], g['ref_q'][:], g['ref_v'][:], g['hold'][:] = 0.0, 1.0, q0, 0.0, 0.0
        g['left'], g['eso'][:] = WEP_S, [q0, 0.0 * q0, 0.0 * q0]
        g['heat'], g['temp'][:] = type(g['heat']).for_joints(), 25.0
        g['_twist_active'], g['_twist_stop_until'], g['twist'], g['twist_actual'] = False, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        g['active_entry'], g['n'], g['_look_yaw'], g['_last_way'] = None, 0, 0.0, 1.0
        for idx in (g.get('waist_idx'), g.get('head_idx')):
            if idx is not None:
                g['posture'][idx] = 0.0
        _dropped(g)
    g['scenario'], g['bore'] = scen, (False, False)
    world.scenario(scen)


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a `dt` on:
    read through her edge, her law's pass (`machine.law`), her drives told through her edge."""
    import numpy as np
    from machine import law
    world, y, ev = g['world'], g['sense'], g['every']
    if g['n'] % ev == 0:
        edge.read(world, y)
        law.control(g, dt)
        edge.drive(world, g['leg_joints'], kp=g['leg_kp'])
        bears = tuple(n >= flight.TOUCH_N for n in g['loads'])
        if any(b and not n for b, n in zip(g.get('bore', (False, False)), bears)):
            g['lifted'] = g['t']
        g['bore'] = bears
        if g['n'] > 0:
            g['temp'] = np.asarray(g['heat'].step_joints(world.data.ctrl, world.data.qvel[world.vadr], dt * ev))
    else:
        edge.felt(world, y)
    law.drives(g, dt)
    edge.drive(world, g['joints'], **g['cmd'])
    g['t'], g['n'] = g['t'] + dt, g['n'] + 1
    world.advance()
    return g['s'], g['loads'], g['ask'], g['out']


def tilt(g):
    """Her pelvis's tilt from upright, deg, as the world has it: the referee's, not her law's."""
    q = g['world'].data.qpos
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[4] ** 2 + q[6] ** 2)))))


def staged(g, fallen_at):
    """Her stage as measured: fallen; in the air, neither sole bearing `flight.TOUCH_N`; walking,
    her pelvis faster than WALK_M_S or a sole off the floor within STEP_S; else standing. Read off
    her gait's amplitude it said stand at 0.9 m/s."""
    if fallen_at is not None:
        return 'fallen'
    if max(g['loads']) < flight.TOUCH_N:
        return 'air'
    d = g['world'].data
    going = math.hypot(float(d.qvel[0]), float(d.qvel[2])) > WALK_M_S
    return 'walk' if going or g['t'] - g.get('lifted', -math.inf) < STEP_S else 'stand'


def state(g, torque, power, watts, ratio, fallen_at, says=(), refused=None):
    """Her state as the page hears it: stage, modes, joint torque/power/temp, physical velocity, and
    her law's horizon - her centre of mass, her capture point, her soles, which stand (world)."""
    from machine import latent, modal
    from machine.figure import JOINTS
    world, out, d, hd = g['world'], g['out'], g['world'].data, float(g['heading'])
    z = modal.read(g['y'], g['fr'], g['a'][modal.STANCE])[0]
    angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
    vxz = (float(d.qvel[0]), float(d.qvel[2]))
    y, om = g['y'], float(g['fr'][modal.OMEGA])
    horizon = {'com': tuple(y[modal.COM].tolist()), 'xi': (float(y[modal.COM][0] + y[modal.VCOM][0] / om),
                                                           float(y[modal.COM][2] + y[modal.VCOM][2] / om)),
               'soles': tuple(tuple(s) for s in y[modal.SOLE].reshape(2, 3).tolist()),
               'stance': tuple(g['a'][modal.STANCE].tolist())}
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'temp': dict(zip(JOINTS, g['temp'].tolist())), 'peak': dict(zip(JOINTS, world.peak.tolist())),
            'watts': float(watts), 'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'heading': hd, 'twist': tuple(g.get('twist', (0.0, 0.0, 0.0))),
            'twist_actual': tuple(g.get('twist_actual', (0.0, 0.0, 0.0))),
            'speed': float(vxz[0] * math.sin(hd) + vxz[1] * math.cos(hd)), 'velocity': vxz,
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': staged(g, fallen_at),
            'fallen': fallen_at is not None, 'loads': tuple(g['loads']), 'props': world.props(),
            'doing': doing(g), 'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'says': tuple(says), 'refused': refused,
            'scenario': g.get('scenario', 'flat'), 'horizon': horizon,
            'modes': [(n, float(out[latent.REF][i]), float(z[i]), float(out[latent.KS][i]),
                       float(out[latent.DS][i])) for i, n in enumerate(ortho.MODES)]}


def _heard_say(g, gym, entry):
    from machine import worker
    return worker.heard_say(g, gym, entry)


def _heard(g, gym, command):
    from machine import worker
    return worker.heard(g, gym, command)


def run(*args, **kwargs):
    from machine.worker import run as _run
    return _run(*args, **kwargs)


class Running:

    """Her worker process on the loop (`worker.Running`)."""

    def __new__(cls, *args, **kwargs):
        from machine.worker import Running as _Running
        return _Running(*args, **kwargs)


