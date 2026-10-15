"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running(); body.send(say=2); now = body.latest(); body.close()
    g = sprung.laid(); s, loads, ask, out = sprung.passed(g)
"""
import math
import os
from machine import ortho

RATE_HZ, SLICE_S, SAY_S, AVERAGE_S, RECOVER_S = 1000.0, 0.05, 1.0 / 60.0, 0.05, 3.0
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
HEARD = ('say', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading', 'twist')
WALK = (('#201', 0.5), ('#202', 2.0), ('#203', 5.0), ('#204', 9.0))
BRAKE = ('(stream_inject (sequence_id 200) (blend_duration 500ms) (latent_targets (mode_axial_compression 0.04)'
         ' (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0)'
         ' (mode_stance_width 0.0)) (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)'
         ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300) (stiffness mode_sagittal_shift 0)'
         ' (damping mode_sagittal_shift 260) (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
         ' (cpg_config (enable true) (frequency 0.85Hz) (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.04)'
         ' (amplitude mode_limb_lift 0.7) (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.0)'
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


def _loads(world, force):
    m, d, out = world.model, world.data, [0.0, 0.0]
    if d.ncon:
        b = m.geom_bodyid[d.contact.geom[:d.ncon]]
        env = m.body_rootid[b] != m.body_rootid[18]
        on = (world.soles[b[:, 0]] & env[:, 1:2]) | (world.soles[b[:, 1]] & env[:, 0:1])
        for side in (0, 1):
            for i in on[:, side].nonzero()[0].tolist():
                world._mj.mj_contactForce(m, d, i, force)
                out[side] += float(force[0])
    return out


def _seen(body, world, limbs):
    from machine import wbc
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    return s


def laid(every=1, inner=None, solver=None):
    """She stands on the floor on the loop, at rest: one dict of what a run holds."""
    import numpy as np
    from machine import figure, heat, latent, modal, physics, wbc, worker, wrench
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
    s = _seen(body, world, limbs)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    fr = modal.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]), float(m.body_subtreemass[body.torso]),
                     float(m.body_subtreemass[m.body('left_upper_arm').id]), float(np.mean(s['q'][list(body.knees)])), 0.001 * every)
    rest = modal.rest(fr)
    rest[latent.K][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (8000.0, 4500.0, 1500.0)
    rest[latent.D][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (1000.0, 500.0, 350.0)
    st, tab = latent.state(rest), np.zeros((0, latent.TABLE))
    out = latent.step(st.copy(), tab, 0.0, 0.001 * every, np.zeros(latent.OUT))
    a, hw, q0 = np.zeros(modal.ASK), 0.5 * float(fr[modal.WEIGHT]), world.data.qpos[body.qact]
    a[modal.STANCE] = 1.0
    robot_ids, upper_bids = worker.robot_body_ids(m, world)
    wep_lim = np.array([max(float(physics.SERVO[kind(j)][0]), float(peak(j)), 1.45 * float(body.clamp[i])) for i, j in enumerate(names)])
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'rest_y': rest_y, 'st': st, 'out': out, 'a': a, 'tab': tab,
         'labels': [], 'posture': s['q'].copy(), 'limbs': limbs, 'bend': modal.crouch(names), 'arms': modal.swing(names),
         'cervical': [names.index(j) for j in ('neck', 'head') if j in names], 'waist_idx': names.index('waist') if 'waist' in names else None,
         'head_idx': names.index('head') if 'head' in names else None, '_look_yaw': 0.0,
         'legs': [[i for i, j in enumerate(JOINTS) if j.startswith(s0) and kind(j) in LEG_KINDS] for s0 in ('left_', 'right_')],
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(), 'wep_lim': wep_lim, 'joints': np.flatnonzero(body.driven),
         'hold': np.zeros(len(body.clamp)), 'left': WEP_S, 'ref_q': q0.copy(), 'ref_v': np.zeros(len(q0)), 'force': np.zeros(6),
         'q_lo': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[0] for j in names]), 'q_hi': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[1] for j in names]),
         't': float(world.data.time), 'n': 0, 'every': every, 's': s, 'loads': [hw, hw], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([q0, np.zeros(len(q0)), np.zeros(len(q0))]), 'jeff': np.diag(s['M'])[body.act].copy(), 'robot_ids': robot_ids,
         'upper_ids': upper_bids, 'obstacle_ids': np.array([m.body(n).id for n in ('wall', 'tree')], dtype=int),
         'sill_ids': np.array([m.body(n).id for n in ('sill', 'rug')], dtype=int), '_brake_wire': _BRAKE_WIRE,
         '_stand_entry': next((e for e in entries() if e[0] == '#201'), None), 'soles_l': np.flatnonzero(world.soles[:, 0]),
         'soles_r': np.flatnonzero(world.soles[:, 1]), 'stumble_until': 0.0, 'heading': float(fr[modal.HEADING]), 'twist': (0.0, 0.0, 0.0),
         'heat': heat.Heat.for_joints(JOINTS), 'temp': np.full(len(JOINTS), 25.0), 'scenario': 'flat',
         'solve': {'stack': wbc.step, 'wrench': wrench.step}[solver or SOLVER]}
    world.clock = lambda: g['t']
    return g


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
        s = g['s'] = _seen(body, world, g['limbs'])
        hw, q0 = 0.5 * float(g['fr'][modal.WEIGHT]), d.qpos[body.qact]
        g['loads'], g['heading'], g['target_heading'] = [hw, hw], 0.0, None
        modal.sensed(s, g['loads'], g['y'])
        g['fr'][modal.HEADING], g['fr'][modal.FLOOR], g['fr'][modal.ROT] = 0.0, g.get('floor0', 0.0), np.array([0, 0, 1, 1, 0, 0, 0, 1, 0], float)
        g['st'] = latent.state(g['rest'])
        g['out'] = latent.step(g['st'].copy(), g['tab'][:0], g['t'], 0.001 * g['every'], np.zeros(latent.OUT))
        g['a'][:], g['a'][modal.STANCE], g['ref_q'][:], g['ref_v'][:], g['hold'][:] = 0.0, 1.0, q0, 0.0, 0.0
        g['left'], g['stumble_until'], g['eso'][:] = WEP_S, 0.0, [q0, 0.0 * q0, 0.0 * q0]
        g['heat'], g['temp'][:] = type(g['heat']).for_joints(), 25.0
        g['_twist_active'], g['_twist_stop_until'], g['twist'], g['twist_actual'] = False, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        g['active_entry'], g['n'], g['_chair_elevated'], g['_chair_cleared'], g['_look_yaw'], g['_last_way'] = None, 0, False, False, 0.0, 1.0
        for idx in (g.get('waist_idx'), g.get('head_idx')):
            if idx is not None:
                g['posture'][idx] = 0.0
        _dropped(g)
    g['scenario'] = scen
    world.scenario(scen)


def _step_control(g, dt):
    import numpy as np
    from machine import flight, impedance, latent, modal, se2, support, wbc
    world, body, d, ev = g['world'], g['body'], g['world'].data, g['every']
    s = g['s'] = _seen(body, world, g['limbs'])
    g['loads'] = _loads(world, g['force'])
    y = modal.sensed(s, g['loads'], g['y'])
    pyaw, thd, cur_doing = 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3])), g.get('target_heading'), doing(g)
    if thd is not None:
        g['heading'] += max(-1.6 * dt * ev, min(1.6 * dt * ev, se2.wrap_angle(thd - g['heading'])))
    elif abs(g.get('twist', (0.0, 0.0, 0.0))[2]) < 0.02:
        g['heading'] += (0.02 if cur_doing == 'brake' else 0.005) * ev * se2.wrap_angle(pyaw - g['heading'])
    sh, ch = math.sin(g['heading']), math.cos(g['heading'])
    g['fr'][modal.HEADING], g['fr'][modal.ROT] = g['heading'], np.array([[sh, 0.0, ch], [ch, 0.0, -sh], [0.0, 1.0, 0.0]]).ravel()
    if g.get('_sync_cpg_at') is not None and g['t'] + 1e-9 >= g['_sync_cpg_at']:
        g['_sync_cpg_at'], g['st'][latent.THETA] = None, 0.0
        fut = g['tab'][(g['tab'][:, latent.AT] <= g['t'] + 1e-9) & (g['tab'][:, latent.AT] > g['t'] - dt * ev + 1e-9)]
        if len(fut):
            ph = fut[0, latent.SAID][latent.PHASE] > 0.5
            g['st'][latent.NOW][latent.PHASE] = np.where(ph, fut[0, latent.VALS][latent.PHASE], g['st'][latent.NOW][latent.PHASE])
    out = latent.step(g['st'], g['tab'], g['t'], dt * ev, g['out'])
    cw = -1.0 if (math.sin(float(out[latent.PSI][modal.LATERAL]) - float(out[latent.PSI][modal.DRIVE])) * (1.0 if float(out[latent.SWAY][modal.LATERAL]) >= 0.0 else -1.0) < -0.5) else 1.0
    if g.get('_last_way') is not None and cw != g['_last_way'] and (g['st'][latent.NOW][latent.GAIN] > 0.05 or abs(float(out[latent.SWAY][modal.DRIVE])) > 0.02):
        g['st'][latent.THETA] = (float(g['st'][latent.THETA]) + math.pi) % math.tau
        out = latent.step(g['st'], g['tab'], g['t'], 0.0, g['out'])
    g['_last_way'] = cw
    se2.step_physical_twist(g, dt * ev, out)
    cur_doing = doing(g)
    flight.check_contacts(g, world, out, cur_doing)
    if g['t'] < max(g['stumble_until'], g.get('parry_until', 0.0)):
        se2.boost_stumble(g, out)
    ae, tw = g.get('active_entry'), g.get('twist', (0.0, 0.0, 0.0))
    backing = ((cur_doing in _BACK_GAITS or ae in _BACK_GAITS or tw[0] < -0.05)
               and cur_doing not in ('rest', '#201', 'brake') and cur_doing not in STUNT_PHASES and g.get('_jump_until', 0.0) <= g['t'])
    tl = ((-1.0 if (cur_doing == 'turn_back_right' or ae == 'turn_back_right' or tw[2] < -0.05) else 1.0) if backing
          else (max(-0.35, min(0.35, 0.35 * tw[2] / se2.YAW_RATE)) if abs(tw[2]) > 0.03 and cur_doing not in ('rest', '#201') else 0.0))
    g['_look_yaw'] += min(1.0, (4.5 if backing else 5.0) * dt * ev) * (tl - g.get('_look_yaw', 0.0))
    for idx, k in ((g.get('waist_idx'), 0.38), (g.get('head_idx'), 0.72)):
        if idx is not None:
            g['posture'][idx] = k * g['_look_yaw'] if cur_doing not in STUNT_PHASES else 0.0
    imp = impedance.ask(out, y, g['fr'], g['a'])
    if backing and abs(g['_look_yaw']) > 1e-3 and cur_doing not in STUNT_PHASES and g.get('waist_idx') is not None:
        imp[modal.HEADED][1] = g['posture'][g['waist_idx']]
    ask = g['ask'] = modal.given(imp, y, g['fr'], g['posture'], g['bend'], g['arms'], body.knees)
    support.shape_hop_and_support(g, ask, out, cur_doing)
    s_solve, cv = s, list(g.get('cervical', ()))
    if (cur_doing in ('load', 'throw', 'land') or ae == 'hop') and ae not in ('hop_up_chair', 'hop_down_chair', 'controlled_fall'):
        cv = cv + list(g['limbs']['shoulders'])
    no_stride = (((g['a'][modal.STRIDES] < 0.5 and float(g['st'][latent.TO][latent.GAIN]) < 0.5
                   and not (len(g['tab']) and (g['tab'][:, latent.VALS][:, latent.GAIN] >= 0.5).any()))
                  or g.get('_jump_until', 0.0) > g['t']) and ae not in ('sit_chair', 'rise_chair', 'hop_up_chair', 'hop_down_chair'))
    if len(cv) and no_stride:
        acv, M = body.act[cv], s['M'].copy()
        M[:6, acv] = M[acv, :6] = 0.0
        s_solve = dict(s, M=M)
    wb = bool(g.get('_twist_active') or g.get('_leap') or cur_doing in STUNT_PHASES)
    der = np.where((g['hold'] > 0.0) | wb, g['wep_lim'] / np.maximum(1.0, body.wep), 1.0)
    g['got'] = g['solve'](body, s_solve, dict(ask, wep=g['left'] > 0.0, derate=der))
    if len(cv) and no_stride:
        cv_p = cv + ([1] if cur_doing == 'land' and g.get('_land_touch_t') is not None and not g.get('_leap') else [])
        acv = body.act[cv_p]
        qdd_cv = np.clip(3.2 * wbc.POSTURE_KD[0] * (ask['posture'][cv_p] - s['q'][cv_p]) - 1.8 * wbc.POSTURE_KD[1] * s['v'][acv], -130.0, 130.0)
        g['got']['qacc'][acv], g['got']['over'][cv_p] = qdd_cv, False
        g['got']['tau'][cv_p] = s['M'][acv, :][:, acv] @ qdd_cv + s['h'][acv]
    g['jeff'] = np.diag(s['M'])[body.act]
    for k, leg in enumerate(g['legs']):
        world.gains[leg, 0] = g['free_kp'][leg] * (1.0 + (STANDING_KP - 1.0) * float(ask['stance'][k]))
    if g['n'] > 0:
        g['temp'] = np.asarray(g['heat'].step_joints(d.ctrl, d.qvel[world.vadr], dt * ev))


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a `dt` on."""
    import numpy as np
    from machine import physics
    world, body = g['world'], g['body']
    if g['n'] % g['every'] == 0:
        _step_control(g, dt)
    got, held, eso, wo, ct = g['got'], float(g['hold'].any()), g['eso'], g['wo'], tilt(g)
    ys, sb = (max(0.0, min(1.0, (85.0 - ct) / 20.0)) if ct > 65.0 else 1.0), np.zeros(len(body.act))
    in_hop = (g.get('_jump_until', 0.0) > g['t'] or doing(g) in ('load', 'throw', 'land')) and not g.get('_leap')
    qk, qvk = world.data.qpos[body.qact][list(body.knees)], world.data.qvel[body.act][list(body.knees)]
    sb[list(body.knees)] = (KNEE_NM * np.clip(1.0 - qk / KNEE_SOFT, 0.0, 3.0) + 2400.0 * np.maximum(0.0, (0.0 if in_hop else 0.02) - qk)
                            - (0.0 if in_hop else 35.0) * np.clip((0.20 - qk) / 0.15, 0.0, 1.5) * np.minimum(0.0, qvk)) * ys
    qa, qpos_act = got['qacc'][body.act] * ys, world.data.qpos[body.qact]
    if not in_hop:
        qa = np.clip(qa, -250.0, 250.0)
    q_m = qpos_act + physics.WOUND * world.boxes.delta[body.driven]
    e = q_m - eso[0]
    eso += dt * np.array([eso[1] + 3.0 * wo * e, qa + sb / g['jeff'] + eso[2] + 3.0 * wo * wo * e, wo ** 3 * e])
    if not in_hop:
        eso[0], z_lim = np.clip(eso[0], q_m - 0.08, q_m + 0.08), 0.35 * body.clamp / np.maximum(0.05, g['jeff'])
        eso[2] = np.clip(eso[2], -z_lim, z_lim)
    if ys < 1.0:
        eso[1:] *= ys
    world.feed[body.driven] = got['tau'] * ys + sb - g['jeff'] * eso[2]
    g['hold'] = np.where(got['over'], WEP_HOLD_S, np.maximum(0.0, g['hold'] - dt))
    g['left'] = min(WEP_S, max(0.0, g['left'] + dt * (WEP_FILL * (1.0 - held) - held)))
    wb = bool(g.get('_twist_active') or g.get('_leap') or doing(g) in STUNT_PHASES)
    world.limit[g['joints']] = np.where(g['hold'] > 0.0, g['wep_lim'] if wb else body.wep, g['clamp'][g['joints']])
    leak, lo, hi = dt / LEAK_S, g['q_lo'], g['q_hi']
    g['ref_v'] += qa * dt + leak * (world.data.qvel[body.act] - g['ref_v'])
    if not in_hop:
        g['ref_v'] = np.clip(g['ref_v'], -18.0, 18.0)
    g['ref_q'] += g['ref_v'] * dt + leak * (qpos_act - g['ref_q'])
    if in_hop:
        g['ref_q'][list(body.knees)] = np.maximum(0.0, g['ref_q'][list(body.knees)])
        cv = g.get('cervical', ())
        if len(cv) >= 2:
            g['ref_q'][cv[0]], g['ref_q'][cv[1]] = np.clip(g['ref_q'][cv[0]], -0.35, 0.45), np.clip(g['ref_q'][cv[1]], -0.85, 0.85)
    else:
        g['ref_v'] = np.where(((g['ref_q'] <= lo) & (g['ref_v'] < 0.0)) | ((g['ref_q'] >= hi) & (g['ref_v'] > 0.0)), 0.0, g['ref_v'])
        g['ref_q'] = np.clip(g['ref_q'], lo, hi)
    if ys < 1.0:
        g['ref_v'], g['ref_q'] = g['ref_v'] * ys, qpos_act + (g['ref_q'] - qpos_act) * ys
    world.target[body.driven] = g['ref_q']
    g['t'], g['n'] = g['t'] + dt, g['n'] + 1
    world.advance()
    if g['every'] == 1:
        g['temp'] = np.asarray(g['heat'].step_joints(world.data.ctrl, world.data.qvel[world.vadr], dt))
    return g['s'], g['loads'], g['ask'], g['out']


def tilt(g):
    """Her pelvis's tilt from upright, deg."""
    q = g['world'].data.qpos
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[4] ** 2 + q[6] ** 2)))))


def state(g, torque, power, watts, ratio, fallen_at, says=(), refused=None):
    """Her state as the page hears it: stage, modes, joint torque/power/temp, and physical velocity."""
    from machine import latent, modal, stride
    from machine.figure import JOINTS
    world, out, d, hd = g['world'], g['out'], g['world'].data, float(g['heading'])
    z = modal.read(g['y'], g['fr'], g['a'][modal.STANCE])[0]
    angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
    vxz = (float(d.qvel[0]), float(d.qvel[2]))
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'temp': dict(zip(JOINTS, g['temp'].tolist())), 'peak': dict(zip(JOINTS, world.peak.tolist())),
            'watts': float(watts), 'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'heading': hd, 'twist': tuple(g.get('twist', (0.0, 0.0, 0.0))),
            'twist_actual': tuple(g.get('twist_actual', (0.0, 0.0, 0.0))),
            'speed': float(vxz[0] * math.sin(hd) + vxz[1] * math.cos(hd)), 'velocity': vxz,
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': ('fallen' if fallen_at is not None
                      else 'walk' if abs(float(out[latent.SWAY][modal.DRIVE])) > stride.STIRS else 'stand'),
            'fallen': fallen_at is not None, 'loads': tuple(g['loads']), 'props': world.props(),
            'doing': doing(g), 'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'says': tuple(says), 'refused': refused,
            'scenario': g.get('scenario', 'flat'),
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


