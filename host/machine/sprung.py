"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running(); body.send(say=2); now = body.latest(); body.close()
    g = sprung.laid(); s, loads, ask, out = sprung.passed(g)
"""
import math
import os
from machine import mechatronics, edge, estimate, grip, leap, ortho, se2, situation, supervisor, touch
from machine.se2 import WEP_S, _dropped, doing, said

LEAK_S, STANDING_KP, WEP_HOLD_S, WEP_FILL = se2.LEAK_S, se2.STANDING_KP, se2.WEP_HOLD_S, se2.WEP_FILL
KNEE_SOFT, KNEE_NM, STUNT_MODES, STUNT_PHASES = se2.KNEE_SOFT, se2.KNEE_NM, se2.STUNT_MODES, se2.STUNT_PHASES
_FWD_GAITS, _BACK_GAITS = se2._FWD_GAITS, se2._BACK_GAITS

RATE_HZ, SLICE_S, SAY_S, AVERAGE_S, RECOVER_S = 1000.0, 0.05, 1.0 / 60.0, 0.05, 3.0
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
#: Her stage (`staged`): walking past WALK_M_S, or with a sole off the floor within STEP_S.
WALK_M_S, STEP_S = 0.1, 1.0
HEARD = ('say', 'at', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading', 'twist',
         'grip', 'charge', 'floor')
STREAM_NAMES = {201: 'stand', 202: 'sway', 203: 'step', 204: 'walk', 205: 'walk_back'}
ALIASES = {'#201': 'stand', '#202': 'sway', '#203': 'step', '#204': 'walk', '#205': 'walk_back', 'back': 'walk_back'}
WALK = (('stand', 0.5), ('sway', 2.0), ('step', 5.0), ('walk', 9.0))
BRAKE = ('(stream_inject (sequence_id 200) (blend_duration 500ms) (latent_targets (mode_axial_compression 0.04)'
         ' (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0)'
         ' (mode_stance_width 0.0)) (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 800)'
         ' (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600) (stiffness mode_sagittal_shift 0)'
         ' (damping mode_sagittal_shift 260) (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))'
         ' (cpg_config (enable true) (amplitude mode_lateral_shift 0.12) (amplitude mode_sagittal_drive 0.0)'
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
SERVO_SHARE = (0.5, 1.0)
INNER, ESO_WO, SOLVER = 'eso', 100.0, 'wrench'
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
_Q_LIM = {'hip_yaw': (-0.44, 0.44), 'hip_roll': (-0.61, 0.61), 'hip': (-2.0, 0.78), 'knee': (0.05, 2.70),
          'ankle': (-0.78, 0.73), 'ankle_roll': (-0.26, 0.26), 'waist': (-0.78, 0.78), 'spine': (-0.44, 1.31),
          'spine_roll': (-0.52, 0.52), 'shoulder': (-1.13, 2.62), 'elbow': (0.0, 2.62), 'neck': (-0.35, 0.45), 'head': (-0.85, 0.85)}
STACK_EVERY, DOWN_DEG, DOWN_M = 3, 50.0, 0.45
GYMS = (mechatronics.ROBOT, 'examples')


def entries(text=None, name=GYMS[0], texts=None, suite=None):
    """[(label, wire, names)]: the gym's entries of `text` - machine/`name`.lisp's, hers with the
    ortho-DSP suite's stunts played on it (`maneuver.texts`) -, of her texts `texts` ({key: text})
    and the suite `suite` where given, else their files'."""
    if text is None:
        from machine import maneuver
        text = ortho.source(os.path.join(os.path.dirname(__file__), name + '.lisp'), texts) + (
            '\n' + maneuver.texts(suite) if name == GYMS[0] else '')
    tables, phases = ortho.compiled(text, texts=texts)
    sz = ortho.ROW.size
    stream = [tables[''][i:i + sz] for i in range(0, len(tables['']), sz)]
    return ([(STREAM_NAMES.get(ortho.rows(w)[0][0], '#%d' % ortho.rows(w)[0][0]), w,
              (STREAM_NAMES.get(ortho.rows(w)[0][0], '#%d' % ortho.rows(w)[0][0]),)) for w in stream]
            + [(n, tables[n], phases[n] + ('rest',)) for n in phases])


def laid(every=1, inner=None, solver=None, texts=None, suite=None):
    """She stands on the floor on the loop, at rest: one dict of what a run holds - her gym's
    entries of her texts `texts` and the suite `suite` where given (`entries`), both kept."""
    import numpy as np
    from machine import figure, heat, latent, modal, motion, physics, servos, wbc, wrench
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
    y[modal.CELLS] = sense[edge.CELLS]
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
    sit, gym = situation.text(), entries(texts=texts, suite=suite)
    from tools.cores import bodied
    sensed = bodied.on_body(wbc.sense, 'sense')
    if (solver or SOLVER) == 'c':
        from tools.cores.wbc import wrench_step as solve_step
    elif (solver or SOLVER) == 'wrench':
        from tools.cores import step as c_step
        solve_step = c_step.wrench_laid(body, sensed)
    else:
        solve_step = {'stack': wbc.step}[solver or SOLVER]
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'rest_y': rest_y, 'st': st, 'out': out, 'a': a, 'tab': tab,
         'labels': [], 'posture': s['q'].copy(), 'limbs': limbs, 'bend': modal.crouch(names), 'arms': modal.swing(names),
         'cervical': [names.index(j) for j in ('neck', 'head') if j in names], 'waist_idx': names.index('waist') if 'waist' in names else None,
         'head_idx': names.index('head') if 'head' in names else None, '_look_yaw': 0.0,
         'legs': legs, 'leg_joints': np.concatenate(legs), 'leg_kp': world.gains[np.concatenate(legs), 0].copy(),
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(), 'wep_lim': wep_lim, 'joints': joints,
         'hold': np.zeros(len(body.clamp)), 'left': WEP_S, 'ref_q': q0.copy(), 'ref_v': np.zeros(len(q0)),
         'sense': sense, 'est': est, 'grip': grip.laid(), 'grip_prior': grip.laid(), 'situations': sit, 'into': np.zeros(len(sit['names'])),
         'grip_seen': np.zeros((2, grip.SEEN)),
         'grip_at': np.array([[m.body(s0 + part).id for part in ('foot', 'toes')] for s0 in ('left_', 'right_')]),
         'q_lo': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[0] for j in names]), 'q_hi': np.array([_Q_LIM.get(kind(j), (-2.0, 2.0))[1] for j in names]),
         't': float(world.data.time), 'n': 0, 'every': every, 's': s, 'loads': [hw, hw], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([q0, np.zeros(len(q0)), np.zeros(len(q0))]), 'jeff': np.diag(s['M'])[body.act].copy(), 'surety': 1.0,
         '_brake_wire': _BRAKE_WIRE, '_stand_entry': next((e for e in gym if e[0] == 'stand'), None),
         'gym': gym, 'texts': texts, 'suite': suite,
         'heading': float(fr[modal.HEADING]), 'twist': (0.0, 0.0, 0.0),
         'heat': heat.Heat.for_joints(JOINTS), 'temp': np.full(len(JOINTS), 25.0), 'scenario': 'flat',
         'touched': touch.laid(), 'twisted': se2.twist_laid(), 'words': se2.words_laid(),
         'solve': solve_step, 'sensed': sensed, 'rate_hz': RATE_HZ}
    motion.laid(g)
    world.clock = lambda: g['t']
    return g


def _sensed(body, est, sense, limbs):
    """`wbc.sense`'s view of her at her lay, as her sensors and her estimate have her."""
    from machine import wbc
    qpos, qvel = estimate.step(est, body, sense, 0.0)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']]
    return s


def walked(g):
    by = {e[0]: e for e in g['gym']}
    _dropped(g)
    for label, at in WALK:
        said(g, *by[label], at=g['t'] + at)


def reset_for_scenario(g, scen):
    """Scene `scen` laid round her, she laid anew where it has her (`floor.lay`) unless she stands
    there at rest already."""
    from machine import floor, modal
    d, (x0, z0) = g['world'].data, floor.lay(scen)
    g['fr'][modal.MAP_N] = 0.0
    if (math.hypot(float(d.qpos[0]) - x0, float(d.qpos[2]) - z0) > 0.08 or abs(float(d.qpos[1]) - g['rest_y']) > 0.02
            or tilt(g) > 2.0 or doing(g) != 'rest' or len(g['tab']) > 0):
        relaid(g, where=(x0, z0))
    g['scenario'], g['bore'] = scen, (False, False)
    g['world'].scenario(scen)


def relaid(g, where=(0.0, 0.0), yaw=0.0):
    """She stands anew at `where` (x, z, m) facing `yaw` (rad), still, every state of hers laid
    with her: her estimate and grip, her senses, her frame's heading and rotation, her latent
    state, her heat, nothing asked."""
    import numpy as np
    from machine import figure, latent, modal
    world, body, d = g['world'], g['body'], g['world'].data
    sh, ch = math.sin(yaw), math.cos(yaw)
    world.reset(figure.stand(), where=(float(where[0]), g['rest_y'], float(where[1])),
                turn=(math.cos(0.5 * yaw), 0.0, math.sin(0.5 * yaw), 0.0))
    g['est'], g['grip'], g['grip_prior'] = estimate.laid(body, d.qpos[0:3]), grip.laid(), grip.laid()
    g['grip_seen'][:] = 0.0
    s = g['s'] = _sensed(body, g['est'], edge.read(world, g['sense']), g['limbs'])
    hw, q0 = 0.5 * float(g['fr'][modal.WEIGHT]), g['sense'][edge.Q][g['joints']]
    g['loads'], g['heading'], g['target_heading'], g['has_heading'] = [hw, hw], yaw, yaw, 0.0
    modal.sensed(s, g['loads'], g['y'])
    g['y'][modal.CELLS] = g['sense'][edge.CELLS]
    g['fr'][modal.HEADING], g['fr'][modal.FLOOR], g['fr'][modal.GROUND] = yaw, g.get('floor0', 0.0), 0.0
    g['fr'][modal.FLOOR_AT] = g['y'][modal.SOLE].reshape(2, 3)[:, [0, 2]].mean(axis=0)
    g['fr'][modal.ROT] = np.array([sh, 0.0, ch, ch, 0.0, -sh, 0.0, 1.0, 0.0])
    g['st'][:] = latent.state(g['rest'])
    g['out'][:] = latent.step(g['st'].copy(), g['tab'][:0], g['t'], 0.001 * g['every'], np.zeros(latent.OUT))
    g['a'][:], g['a'][modal.STANCE], g['ref_q'][:], g['ref_v'][:], g['hold'][:] = 0.0, 1.0, q0, 0.0, 0.0
    g['left'], g['eso'][:], g['surety'] = WEP_S, [q0, 0.0 * q0, 0.0 * q0], 1.0
    g['heat'], g['temp'][:] = type(g['heat']).for_joints(), 25.0
    g['_twist_active'], g['_twist_stop_until'], g['twist'], g['asked'], g['twist_actual'] = (
        False, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    g['active_entry'], g['n'], g['_halt_at'] = None, 0, None
    g['twisted'][se2.LOOK_YAW], g['twisted'][se2.TURN_DC] = 0.0, 0.0
    g['touched'][:] = touch.REST       # what she felt stood in the world laid away
    g['_w_back'], g['_backing_yaw'] = 0.0, 0.0
    supervisor.reset(g['bus'])
    for idx in (g.get('waist_idx'), g.get('head_idx')):
        if idx is not None:
            g['posture'][idx] = 0.0
    _dropped(g)


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a `dt` on:
    read through her edge, her motion pass (`machine.motion`), her drives told through her edge."""
    import numpy as np
    from machine import motion
    world, y, ev = g['world'], g['sense'], g['every']
    if g['n'] % ev == 0:
        edge.read(world, y)
        motion.control(g, dt)
        edge.drive(world, g['leg_joints'], kp=g['leg_kp'])
        bears = tuple(n >= leap.TOUCH_N for n in g['loads'])
        if any(b and not n for b, n in zip(g.get('bore', (False, False)), bears)):
            g['lifted'] = g['t']
        g['bore'] = bears
        if g['n'] > 0:
            g['temp'] = np.asarray(g['heat'].step_joints(world.data.ctrl, world.data.qvel[world.vadr], dt * ev))
    else:
        edge.felt(world, y)
    motion.drives(g, dt)
    edge.drive(world, g['joints'], **g['cmd'])
    g['t'], g['n'] = g['t'] + dt, g['n'] + 1
    world.advance()
    return g['s'], g['loads'], g['ask'], g['out']


def tilt(g):
    """Her pelvis's tilt from upright, deg, as the world has it: the referee's, not her law's."""
    q = g['world'].data.qpos
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[4] ** 2 + q[6] ** 2)))))


def staged(g, fallen_at):
    """Her stage as measured: fallen; in the air, neither sole bearing `leap.TOUCH_N`; walking,
    her pelvis faster than WALK_M_S or a sole off the floor within STEP_S; else standing. Read off
    her gait's amplitude it said stand at 0.9 m/s."""
    if fallen_at is not None:
        return 'fallen'
    if max(g['loads']) < leap.TOUCH_N:
        return 'air'
    d = g['world'].data
    going = math.hypot(float(d.qvel[0]), float(d.qvel[2])) > WALK_M_S
    return 'walk' if going or g['t'] - g.get('lifted', -math.inf) < STEP_S else 'stand'


def state(g, torque, power, watts, ratio, fallen_at, says=(), refused=None, headroom=1.0):
    """Her state as the page hears it: stage, modes, joint torque/power/temp, physical velocity, and
    her law's horizon - her centre of mass, her capture point, her soles, which stand (world),
    her shoulders' range; `ratio` her pace to the wall, `headroom` her process's flat out."""
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
                'stance': tuple(g['a'][modal.STANCE].tolist()), 'shoulder': tuple(_Q_LIM['shoulder'])}
    buses = {b.name: (b.frames, b.comm_errors, b.tx_bytes) for b in (getattr(world, 'limb_buses', None) or ())}
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'temp': dict(zip(JOINTS, g['temp'].tolist())), 'peak': dict(zip(JOINTS, world.peak.tolist())),
            'watts': float(watts), 'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'heading': hd, 'twist': tuple(g.get('twist', (0.0, 0.0, 0.0))),
            'twist_actual': tuple(g.get('twist_actual', (0.0, 0.0, 0.0))),
            'speed': float(vxz[0] * math.sin(hd) + vxz[1] * math.cos(hd)), 'velocity': vxz,
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': staged(g, fallen_at),
            'fallen': fallen_at is not None, 'loads': tuple(g['loads']), 'props': world.props(), 'dents': world.dents(),
            'doing': doing(g), 'entry': g.get('active_entry'),
            'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'headroom': min(headroom, 99.0), 'says': tuple(says), 'refused': refused,
            'buses': buses,
            'scenario': g.get('scenario', 'flat'), 'situation': situation.told(g['situations'], g['into']), 'horizon': horizon,
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


def emulated_limb(state, say, limb_cls):
    emu = limb_cls(state['emu_port'])
    state['origin'] = emu.origin
    say('ok', 'boards', '%d STM32H753 nodes on Renode (%s, %s)' % (len(emu.rigs), emu.limb, emu.origin.port))
    return emu


def boot_humanoid(state, boot, say, ahead_cls, limb_cls):
    import time
    from machine import figure, modes
    from machine.routines import TYPES
    drives = sum(len(s.actuators) for s in TYPES['gynoid'].body)
    with boot('LINKING HUMANOID') as step:
        step(0.15, 'MUJOCO & LISP')
        body = state['body'] = Running(realtime=not state.get('max', False), shown=0.0)
        say('ok', 'body', '%.1f kg, %d drives, MuJoCo at 1 kHz in its own process%s'
            % (figure.mass(), drives, ', flat out' if state.get('max') else ', paced to the wall'))
        say('ok', 'tier', modes.told(os.cpu_count() or 1, drives))
        if state.get('emu_port'):
            step(0.30, 'RENODE EXTREMITY BUS')
            state['emu'] = emulated_limb(state, say, limb_cls)
        step(0.45, 'SHADER, RASTER & MESHES')
        ahead = state['ahead'] = ahead_cls('coaxial.graphics.gynoid', 'render')
        name = ahead.name or 'a process of its own, dots'
        say('ok', 'drawing', name)
        step(0.90, 'SETTLING STAND')
        for _ in range(300):
            first = body.latest()
            if first is not None:
                state['playback'].push([first])
                break
            time.sleep(0.01)
        step()
    return body, name


class Running:

    """Her worker process on the loop (`worker.Running`)."""

    def __new__(cls, *args, **kwargs):
        from machine.worker import Running as _Running
        return _Running(*args, **kwargs)


