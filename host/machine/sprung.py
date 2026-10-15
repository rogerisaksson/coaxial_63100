"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running()                # her process: on the loop she walks from rest (WALK)
    body.send(say=2)                       # the gym's third entry: a payload injected, a behaviour begun
    body.send(text='(stream_inject ..)'); body.send(push=(0.0, 0.0, 80.0), seconds=0.12)
    now = body.latest(); body.close()
    g = sprung.laid(); s, loads, ask, out = sprung.passed(g)
"""
import math
import os

from machine import ortho

RATE_HZ, SLICE_S, SAY_S, AVERAGE_S, RECOVER_S = 1000.0, 0.05, 1.0 / 60.0, 0.05, 3.0
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
HEARD = ('say', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading', 'twist')
WALK = (('#201', 0.5), ('#202', 2.0), ('#203', 5.0), ('#204', 9.0))
BRAKE = ('(stream_inject (sequence_id 200) (blend_duration 500ms)'
         ' (latent_targets (mode_axial_compression 0.04) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0)'
         ' (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0) (mode_stance_width 0.0))'
         ' (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)'
         ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)'
         ' (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)'
         ' (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
         ' (cpg_config (enable true) (frequency 0.85Hz) (amplitude mode_lateral_shift 0.28)'
         ' (amplitude mode_sagittal_drive 0.04) (amplitude mode_limb_lift 0.7) (amplitude mode_foot_rock 0.0)'
         ' (amplitude mode_pelvis_list 0.0) (amplitude mode_pelvis_turn 0.0) (amplitude mode_arm_swing 0.0)'
         ' (amplitude mode_stance_width 0.0)'
         ' (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)'
         ' (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))'
         ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')
_TOUCH = ('(stream_inject (sequence_id 200) (blend_duration 500ms)'
          ' (latent_targets (mode_axial_compression 0.0) (mode_lateral_shift 0.0)'
          ' (mode_limb_lift 0.0) (mode_pelvis_list 0.0))'
          ' (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 800)'
          ' (stiffness mode_lateral_shift 6000) (damping mode_lateral_shift 600)'
          ' (stiffness mode_pelvis_list 1500) (damping mode_pelvis_list 300))'
          ' (cpg_config (enable false)) (task_constraints (maintain_zmp (polygon support_polygon))))')
_BRAKE_WIRE = ortho.compiled(BRAKE)[0]['']
_BRAKE_BACK_WIRE = ortho.compiled(BRAKE.replace('108deg', '-108deg'))[0]['']
_TOUCH_WIRE = ortho.compiled(_TOUCH)[0]['']

SERVO_SHARE, LEAK_S, STANDING_KP = (0.5, 1.0), 0.2, 0.1
WEP_HOLD_S, WEP_S, WEP_FILL = 0.6, 5.0, 0.5
INNER, ESO_WO, SOLVER = 'eso', 100.0, 'wrench'
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
KNEE_SOFT, KNEE_NM, STACK_EVERY = 0.1, 40.0, 3
DOWN_DEG, DOWN_M = 50.0, 0.45
GYMS = ('gynoid_mesomorph_medium', 'examples')
STUNT_MODES = {'sit_chair', 'rise_chair', 'lie_bed', 'rise_bed', 'lie_floor',
               'rise_floor', 'hop_down_chair', 'hop_up_chair', 'stairs_down',
               'parry_shove', 'controlled_fall'}
STUNT_PHASES = {'approach', 'lower', 'seated', 'lean_forward', 'thrust', 'sit_edge', 'recline',
                'bed_rest', 'sit_up', 'swing_out', 'kneel', 'floor_rest', 'push_arms',
                'half_kneel', 'cushion', 'land', 'step_off', 'drop', 'absorb', 'crouch_load',
                'propulsion', 'flight_tuck', 'seat_land', 'descent'}
_FWD_GAITS = ('#204', 'trip', 'jog', 'ballistic_walk', 'stairs_walk', 'stairs_down',
              'ice_walk', 'slope_walk', 'strafe_left', 'strafe_right', 'turn_left',
              'turn_right', 'circle_walk')
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
    """She stands on the floor on the loop, at rest: one dict of what a run holds."""
    import numpy as np
    from machine import figure, latent, modal, physics, wbc, worker, wrench
    from machine.drives import kind
    from machine.figure import JOINTS
    world, body, pose = physics.World(), wbc.Body(), figure.stand()
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    rest_y = 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(pose, where=(0.0, rest_y, 0.0))
    inner = inner or INNER
    world.gains[:] *= np.array(SERVO_SHARE) * (inner == 'pd')
    world.gains[~body.driven] = 0.0
    m, names = body.m, [j for j, d in zip(JOINTS, body.driven) if d]
    limbs = {'hips': [m.body(s0 + 'thigh').id for s0 in ('left_', 'right_')],
             'shoulders': [names.index(s0 + 'shoulder') for s0 in ('left_', 'right_')]}
    s = _seen(body, world, limbs)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    fr = modal.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]),
                     float(m.body_subtreemass[body.torso]),
                     float(m.body_subtreemass[m.body('left_upper_arm').id]),
                     float(np.mean(s['q'][list(body.knees)])), 0.001 * every)
    rest = modal.rest(fr)
    rest[latent.K][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (8000.0, 4500.0, 1500.0)
    rest[latent.D][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (1000.0, 500.0, 350.0)
    st, tab = latent.state(rest), np.zeros((0, latent.TABLE))
    out = latent.step(st.copy(), tab, 0.0, 0.001 * every, np.zeros(latent.OUT))
    a, hw = np.zeros(modal.ASK), 0.5 * float(fr[modal.WEIGHT])
    a[modal.STANCE] = 1.0
    robot_ids, upper_bids = worker.robot_body_ids(m, world)
    q0 = world.data.qpos[body.qact]
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'rest_y': rest_y, 'st': st,
         'out': out, 'a': a, 'tab': tab, 'labels': [], 'posture': s['q'].copy(), 'limbs': limbs,
         'bend': modal.crouch(names), 'arms': modal.swing(names),
         'cervical': [names.index(j) for j in ('neck', 'head') if j in names],
         'waist_idx': names.index('waist') if 'waist' in names else None,
         'head_idx': names.index('head') if 'head' in names else None, '_look_yaw': 0.0,
         'legs': [[i for i, j in enumerate(JOINTS) if j.startswith(s0) and kind(j) in LEG_KINDS]
                  for s0 in ('left_', 'right_')],
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(),
         'joints': np.flatnonzero(body.driven), 'hold': np.zeros(len(body.clamp)), 'left': WEP_S,
         'ref_q': q0.copy(), 'ref_v': np.zeros(len(q0)), 'force': np.zeros(6),
         't': float(world.data.time), 'n': 0, 'every': every, 's': s, 'loads': [hw, hw],
         'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([q0, np.zeros(len(q0)), np.zeros(len(q0))]),
         'jeff': np.diag(s['M'])[body.act].copy(), 'robot_ids': robot_ids, 'upper_ids': upper_bids,
         'obstacle_ids': np.array([m.body(n).id for n in ('wall', 'tree')], dtype=int),
         'sill_ids': np.array([m.body(n).id for n in ('sill', 'rug')], dtype=int),
         '_brake_wire': _BRAKE_WIRE, '_stand_entry': next((e for e in entries() if e[0] == '#201'), None),
         'soles_l': np.flatnonzero(world.soles[:, 0]), 'soles_r': np.flatnonzero(world.soles[:, 1]),
         'stumble_until': 0.0, 'heading': float(fr[modal.HEADING]), 'twist': (0.0, 0.0, 0.0),
         'temp': np.full(len(JOINTS), 25.0), 'scenario': 'flat',
         'solve': {'stack': wbc.step, 'wrench': wrench.step}[solver or SOLVER]}
    world.clock = lambda: g['t']
    return g


def said(g, label, wire, names=(), at=None, reset=False):
    """`wire`'s rows (`ortho.compiled`) begun at `at` s - now - in her table."""
    import numpy as np
    from machine import latent
    new = latent.table(wire, g['t'] + 0.001 if at is None else at, g['rest'])
    if reset and len(new):
        new[0, latent.VALS] = np.where(new[0, latent.SAID] > 0.5, new[0, latent.VALS], g['rest'])
        new[0, latent.SAID] = 1.0
    g['tab'] = np.concatenate([g['tab'][g['tab'][:, latent.AT] >= g['t']], new])
    g['labels'] = g['labels'][-16:] + [(float(w), names[i] if i < len(names) else label)
                                       for i, w in enumerate(new[:, latent.AT])]


def walked(g):
    """Her walk from rest (WALK) said from now, in place of what was to come."""
    by = {e[0]: e for e in entries()}
    _dropped(g)
    for label, at in WALK:
        said(g, *by[label], at=g['t'] + at)


def _dropped(g):
    """Her table's rows still to come dropped, and what they were to be called."""
    g['tab'], g['labels'], g['_sync_cpg_at'] = g['tab'][:0], [(w, n) for w, n in g['labels'] if w <= g['t']], None


def doing(g):
    """What she is doing now: the name of her table's row that last came, 'rest' with none."""
    come = [(w, n) for w, n in g['labels'] if w <= g['t'] + 1e-9]
    return max(come)[1] if come else 'rest'


def _check_contacts(g, world, out, cur_doing):
    import numpy as np
    from machine import latent, modal
    d = world.data
    g['_upper_hit'] = False
    if not d.ncon:
        return
    cb = world.model.geom_bodyid[d.contact.geom[:d.ncon]]
    b0, b1, obs, up = cb[:, 0], cb[:, 1], g['obstacle_ids'], g['upper_ids']
    if ((np.isin(b0, up) & np.isin(b1, obs)) | (np.isin(b1, up) & np.isin(b0, obs))).any():
        g['_upper_hit'] = True
        out[latent.SWAY][modal.DRIVE] = min(float(out[latent.SWAY][modal.DRIVE]), 0.04)
        out[latent.REF][modal.DRIVE] = g['a'][modal.GOING] = 0.0
        d.qvel[0], d.qvel[2], d.qvel[3:6] = d.qvel[0] * 0.85, min(float(d.qvel[2]), 0.05), d.qvel[3:6] * 0.85
        if cur_doing not in ('brake', '#201', 'rest') and abs(float(g['st'][latent.NOW][latent.AMP][modal.DRIVE])) > 0.06:
            _dropped(g)
            said(g, 'brake', g['_brake_wire'], ('brake',), at=g['t'] + 0.001, reset=True)
            if g['_stand_entry'] is not None:
                said(g, *g['_stand_entry'], at=g['t'] + 1.20, reset=True)
            g['active_entry'] = '#201'
    aloft, sills = g['a'][modal.STANCE] < 0.5, g['sill_ids']
    hl = aloft[0] and ((np.isin(b0, g['soles_l']) & np.isin(b1, sills)) | (np.isin(b1, g['soles_l']) & np.isin(b0, sills))).any()
    hr = aloft[1] and ((np.isin(b0, g['soles_r']) & np.isin(b1, sills)) | (np.isin(b1, g['soles_r']) & np.isin(b0, sills))).any()
    if hl or hr:
        g['stumble_until'] = g['t'] + 0.95
        d.qvel[0], d.qvel[2], d.qvel[3:6] = d.qvel[0] * 0.85, min(float(d.qvel[2]), 0.35), d.qvel[3:6] * 0.85


def _step_control(g, dt):
    import numpy as np
    from machine import impedance, latent, modal, se2, wbc
    world, body, d, ev = g['world'], g['body'], g['world'].data, g['every']
    s = g['s'] = _seen(body, world, g['limbs'])
    y = modal.sensed(s, _loads(world, g['force']), g['y'])
    g['loads'] = _loads(world, g['force'])
    pyaw = 2.0 * math.atan2(float(d.qpos[5]), float(d.qpos[3]))
    thd = g.get('target_heading')
    cur_doing = doing(g)
    if thd is not None:
        g['heading'] += max(-1.6 * dt * ev, min(1.6 * dt * ev, se2.wrap_angle(thd - g['heading'])))
    else:
        g['heading'] += (0.02 if cur_doing == 'brake' else 0.005) * ev * se2.wrap_angle(pyaw - g['heading'])
    sh, ch = math.sin(g['heading']), math.cos(g['heading'])
    g['fr'][modal.HEADING] = g['heading']
    g['fr'][modal.ROT] = np.array([[sh, 0.0, ch], [ch, 0.0, -sh], [0.0, 1.0, 0.0]]).ravel()
    if g.get('_sync_cpg_at') is not None and g['t'] + 1e-9 >= g['_sync_cpg_at']:
        g['_sync_cpg_at'], g['st'][latent.THETA] = None, 0.0
        fut = g['tab'][(g['tab'][:, latent.AT] <= g['t'] + 1e-9) & (g['tab'][:, latent.AT] > g['t'] - dt * ev + 1e-9)]
        if len(fut):
            ph = fut[0, latent.SAID][latent.PHASE] > 0.5
            g['st'][latent.NOW][latent.PHASE] = np.where(ph, fut[0, latent.VALS][latent.PHASE], g['st'][latent.NOW][latent.PHASE])
    out = latent.step(g['st'], g['tab'], g['t'], dt * ev, g['out'])
    se2.step_physical_twist(g, dt * ev, out)
    cur_doing = doing(g)
    _check_contacts(g, world, out, cur_doing)
    if g['t'] < g['stumble_until']:
        se2.boost_stumble(g, out)
    if world.push_until > 0.0 and d.time < world.push_until + 0.6 and cur_doing not in ('lower', 'seated'):
        d.qvel[0], d.qvel[2], d.qvel[3:6] = d.qvel[0] * 0.94, d.qvel[2] * 0.94, d.qvel[3:6] * 0.92
    ae, tw = g.get('active_entry'), g.get('twist', (0.0, 0.0, 0.0))
    backing = ((cur_doing in _BACK_GAITS or ae in _BACK_GAITS or tw[0] < -0.05)
               and cur_doing not in ('rest', '#201', 'brake') and cur_doing not in STUNT_PHASES)
    tl = (-1.0 if (cur_doing == 'turn_back_right' or ae == 'turn_back_right' or tw[2] < -0.05) else 1.0) if backing else 0.0
    g['_look_yaw'] += min(1.0, (4.5 if backing else 5.0) * dt * ev) * (tl - g.get('_look_yaw', 0.0))
    if g.get('waist_idx') is not None:
        g['posture'][g['waist_idx']] = 0.38 * g['_look_yaw'] if cur_doing not in STUNT_PHASES else 0.0
    if g.get('head_idx') is not None:
        g['posture'][g['head_idx']] = 0.72 * g['_look_yaw'] if cur_doing not in STUNT_PHASES else 0.0
    imp = impedance.ask(out, y, g['fr'], g['a'])
    if abs(g['_look_yaw']) > 1e-3 and cur_doing not in STUNT_PHASES and g.get('waist_idx') is not None:
        imp[modal.HEADED][1] = g['posture'][g['waist_idx']]
    ask = g['ask'] = modal.given(imp, y, g['fr'], g['posture'], g['bend'], g['arms'], body.knees)
    s_solve, cv = s, g.get('cervical', ())
    no_stride = ((g['a'][modal.STRIDES] < 0.5 and float(g['st'][latent.TO][latent.GAIN]) < 0.5
                  and not (len(g['tab']) and (g['tab'][:, latent.VALS][:, latent.GAIN] >= 0.5).any()))
                 or g.get('_jump_until', 0.0) > g['t'])
    if len(cv) and no_stride:
        acv, M = body.act[cv], s['M'].copy()
        M[:6, acv] = M[acv, :6] = 0.0
        s_solve = dict(s, M=M)
    g['got'] = g['solve'](body, s_solve, dict(ask, wep=g['left'] > 0.0))
    if len(cv) and no_stride:
        acv = body.act[cv]
        qdd_cv = 4.0 * wbc.POSTURE_KD[0] * (ask['posture'][cv] - s['q'][cv]) - 2.0 * wbc.POSTURE_KD[1] * s['v'][acv]
        g['got']['qacc'][acv], g['got']['over'][cv] = qdd_cv, False
        g['got']['tau'][cv] = s['M'][acv, :][:, acv] @ qdd_cv + s['h'][acv]
    g['jeff'] = np.diag(s['M'])[body.act]
    for k, leg in enumerate(g['legs']):
        world.gains[leg, 0] = g['free_kp'][leg] * (1.0 + (STANDING_KP - 1.0) * float(ask['stance'][k]))


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a `dt` on."""
    import numpy as np
    from machine import physics
    world, body = g['world'], g['body']
    if g['n'] % g['every'] == 0:
        _step_control(g, dt)
    got, held, eso, wo = g['got'], float(g['hold'].any()), g['eso'], g['wo']
    ct = tilt(g)
    ys = max(0.0, min(1.0, (85.0 - ct) / 20.0)) if ct > 65.0 else 1.0
    sb = np.zeros(len(body.act))
    qk = world.data.qpos[body.qact][list(body.knees)]
    sb[list(body.knees)] = (KNEE_NM * np.clip(1.0 - qk / KNEE_SOFT, 0.0, 3.0) + 2400.0 * np.maximum(0.0, -qk)) * ys
    qa = got['qacc'][body.act] * ys
    e = world.data.qpos[body.qact] + physics.WOUND * world.boxes.delta[body.driven] - eso[0]
    eso += dt * np.array([eso[1] + 3.0 * wo * e, qa + sb / g['jeff'] + eso[2] + 3.0 * wo * wo * e, wo ** 3 * e])
    if ys < 1.0:
        eso[1:] *= ys
    world.feed[body.driven] = got['tau'] * ys + sb - g['jeff'] * eso[2]
    g['hold'] = np.where(got['over'], WEP_HOLD_S, np.maximum(0.0, g['hold'] - dt))
    g['left'] = min(WEP_S, max(0.0, g['left'] + dt * (WEP_FILL * (1.0 - held) - held)))
    world.limit[g['joints']] = np.where(g['hold'] > 0.0, body.wep, g['clamp'][g['joints']])
    leak, qpos_act = dt / LEAK_S, world.data.qpos[body.qact]
    g['ref_v'] += qa * dt + leak * (world.data.qvel[body.act] - g['ref_v'])
    g['ref_q'] += g['ref_v'] * dt + leak * (qpos_act - g['ref_q'])
    g['ref_q'][list(body.knees)] = np.maximum(0.0, g['ref_q'][list(body.knees)])
    cv = g.get('cervical', ())
    if len(cv) >= 2:
        g['ref_q'][cv[0]] = np.clip(g['ref_q'][cv[0]], -0.35, 0.45)
        g['ref_q'][cv[1]] = np.clip(g['ref_q'][cv[1]], -0.85, 0.85)
    if ys < 1.0:
        g['ref_v'] *= ys
        g['ref_q'] = qpos_act + (g['ref_q'] - qpos_act) * ys
    world.target[body.driven] = g['ref_q']
    g['t'], g['n'] = g['t'] + dt, g['n'] + 1
    world.advance()
    tau, qv = world.data.ctrl, world.data.qvel[world.vadr]
    g['temp'] = np.clip(g['temp'] + dt * (45.0 * (tau / np.maximum(1.0, g['clamp'])) ** 2
                        + 0.08 * np.abs(tau * qv) - 0.35 * (g['temp'] - 25.0)), 25.0, 125.0)
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
    """A command on her: an entry said, her walk from rest, a text's entries begun, or a push."""
    from machine import worker
    return worker.heard(g, gym, command)


def run(*args, **kwargs):
    from machine.worker import run as _run
    return _run(*args, **kwargs)


class Running:

    """Her worker process on the loop (`run`): `send` it commands, read its `latest` state;
    laid, she stands at rest unless `walks`."""

    def __new__(cls, *args, **kwargs):
        from machine.worker import Running as _Running
        return _Running(*args, **kwargs)


