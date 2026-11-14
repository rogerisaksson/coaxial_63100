"""The gynoid on her latent loop, in a process of her own: the page's gym for orthokinematics.

    body = sprung.Running(); body.send(say=2); now = body.latest(); body.close()
    g = sprung.laid(); s, loads, ask, out = sprung.passed(g)
"""
import functools
import math
import os
from machine.body import mechatronics, peripherals
from machine.way import planner
from machine.eyes import camera, vision
from machine.senses import edge, estimate, field, grip, inekf, momentum, observer, situation, touch
from machine.moves import leap
from machine.texts import ortho
from machine.law import se2, supervisor
from machine.law.se2 import WEP_S, _dropped, doing, said
from machine.process import joints, page
from machine.process.sensed import sensed
from machine.world import steps

LEAK_S, STANDING_KP, WEP_HOLD_S, WEP_FILL = se2.LEAK_S, se2.STANDING_KP, se2.WEP_HOLD_S, se2.WEP_FILL
#: Her registry's looks by `physics.SIGHT` off or on: her stand-in the world's boxes, the whole room's at a look;
#: her fake camera's things (`camera.Camera`, under CAMERA or SIGHT: a look from her head as her invariant EKF
#: places it LOOK_S apart into her map apart), told a look later, what its frustum shows.
KNEE_SOFT, KNEE_NM, STUNT_MODES, STUNT_PHASES = se2.KNEE_SOFT, se2.KNEE_NM, se2.STUNT_MODES, se2.STUNT_PHASES
_FWD_GAITS, _BACK_GAITS = se2._FWD_GAITS, se2._BACK_GAITS

_sprung_bus = peripherals.bus()
RATE_HZ, SLICE_S, SAY_S, AVERAGE_S = round(1.0 / _sprung_bus['lockstep_s'], 1), 0.05, 1.0 / 60.0, 0.05
SHOVES, SHOVE_S = {'nudge': 38.0, 'push': 80.0, 'shove': 120.0}, 0.12
#: Her state as her page hears it (`page`): her stage, her tilt, her things named.
named, tilt, staged, state = page.named, page.tilt, page.staged, page.state
WALK_M_S, STEP_S, RECOVER_S = page.WALK_M_S, page.STEP_S, page.RECOVER_S
HEARD = ('say', 'at', 'walk', 'entries', 'gym', 'text', 'push', 'seconds', 'restart', 'scenario', 'heading', 'twist',
         'grip', 'charge', 'floor', 'link')
STREAM_NAMES = {201: 'stand', 202: 'sway', 203: 'step', 204: 'walk', 205: 'walk_back'}
ALIASES = {'#201': 'stand', '#202': 'sway', '#203': 'step', '#204': 'walk', '#205': 'walk_back', 'back': 'walk_back'}
WALK = (('stand', 0.5), ('sway', 2.0), ('step', 5.0), ('walk', 9.0))
BRAKE = (('(stream_inject (sequence_id 200) (blend_duration %dms) (latent_targets (mode_axial_compression 0.04)'
          ' (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0) (mode_pelvis_turn 0.0)'
          ' (mode_stance_width 0.0)) (impedance (stiffness mode_axial_compression %g)'
          ' (damping mode_axial_compression %g) (stiffness mode_lateral_shift %g)'
          ' (damping mode_lateral_shift %g) (stiffness mode_sagittal_shift 0)'
          ' (damping mode_sagittal_shift %g) (stiffness mode_foot_rock %g) (damping mode_foot_rock %g))'
          ' (cpg_config (enable true) (amplitude mode_lateral_shift 0.12) (amplitude mode_sagittal_drive 0.0)'
          ' (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.0)'
          ' (amplitude mode_pelvis_turn 0.0) (amplitude mode_arm_swing 0.0) (amplitude mode_stance_width 0.0)'
          ' (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)'
          ' (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))'
          ' (task_constraints (maintain_zmp (polygon dynamic_footprint))))')
         % (round(1e3 * se2.BRAKE_S), se2._K_WALK, round(1.3 * se2._D_WALK, -1), se2._K_WALK, se2._D_WALK,
            round(0.4 * se2._D_WALK, -1), se2._K_ROCK, se2._D_ROCK))
_TOUCH = (('(stream_inject (sequence_id 200) (blend_duration %dms) (latent_targets (mode_axial_compression 0.0)'
           ' (mode_lateral_shift 0.0) (mode_limb_lift 0.0) (mode_pelvis_list 0.0)) (impedance'
           ' (stiffness mode_axial_compression %g) (damping mode_axial_compression %g)'
           ' (stiffness mode_lateral_shift %g) (damping mode_lateral_shift %g)'
           ' (stiffness mode_pelvis_list %g) (damping mode_pelvis_list %g))'
           ' (cpg_config (enable false)) (task_constraints (maintain_zmp (polygon support_polygon))))')
          % (round(1e3 * se2.STAND_S), 2.0 * se2._K_WALK, round(1.3 * se2._D_WALK, -1), 2.0 * se2._K_WALK,
             se2._D_WALK, 0.5 * se2._K_WALK, 0.5 * se2._D_WALK))
_BRAKE_WIRE, _BRAKE_BACK_WIRE, _TOUCH_WIRE = ortho.compiled(BRAKE)[0][''], ortho.compiled(BRAKE.replace('108deg', '-108deg'))[0][''], ortho.compiled(_TOUCH)[0]['']
SERVO_SHARE = (0.5, 1.0)
INNER, ESO_WO, SOLVER = 'eso', _sprung_bus['eso_wo'], 'wrench'
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')
STACK_EVERY, DOWN_DEG, DOWN_M = _sprung_bus['law_div'], 50.0, 0.45
GYMS = (mechatronics.ROBOT, 'examples')


def entries(text=None, name=GYMS[0], texts=None, suite=None):
    """[(label, wire, names)]: the gym's entries of `text` - lisp/humanoid/`name`.lisp's, hers with the
    ortho-DSP suite's stunts played on it (`maneuver.texts`) and the kernel's clocked gaits (`clocked.texts`) -, of
    her texts `texts` ({key: text}) and the suite `suite` where given, else their files'."""
    if text is None:
        from machine.moves import clocked
        from machine.texts import maneuver
        text = ortho.source(os.path.join(os.path.dirname(ortho._FILE), name + '.lisp'), texts) + (
            '\n' + maneuver.texts(suite) + '\n' + clocked.texts() if name == GYMS[0] else '')
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
    from machine.body import bends, figure, gait, heat, servos
    from machine.law import latent, melt, modal, motion, statics
    from machine.world import physics
    from machine.eyes import registry
    from machine.stack import wbc, wrench
    from machine.body.drives import kind, peak, wep_scale
    from machine.body.figure import JOINTS
    world, body, pose, inner = physics.World(), wbc.Body(), figure.stand(), (inner or INNER)
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    rest_y = 1.0 - min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(pose, where=(0.0, rest_y, 0.0))
    world.gains[:] *= np.array(SERVO_SHARE) * (inner == 'pd')
    world.gains[~body.driven] = 0.0
    m, names = body.m, [j for j, d in zip(JOINTS, body.driven) if d]
    limbs = {'hips': [m.body(s0 + 'thigh').id for s0 in ('left_', 'right_')], 'shoulders': [names.index(s0 + 'shoulder') for s0 in ('left_', 'right_')],
             'elbows': [names.index(s0 + 'elbow') for s0 in ('left_', 'right_')]}
    sense, est = edge.laid(world), estimate.laid(body, world.data.qpos[0:3])
    s = sensed(body, est, sense, limbs)
    y = modal.sensed(s, (0.0, 0.0), np.zeros(modal.SENSED))
    y[modal.CELLS] = sense[edge.CELLS]
    fr = statics.frame(y, body.mass, float(m.body_subtreemass[m.body('left_hip_yaw').id]), float(m.body_subtreemass[body.torso]),
                     float(m.body_subtreemass[m.body('left_upper_arm').id]), float(m.body_subtreemass[m.body('left_forearm').id]),
                     float(np.mean(s['q'][list(body.knees)])), 0.001 * every)
    rest = statics.rest(fr)
    modes_g = gait.table('modes')
    k_g = float(fr[modal.WEIGHT] / fr[modal.H0])
    k_ax = float(round(modes_g['rest_axial_w'] ** 2 * k_g))
    k_lat, k_sag = modes_g['rest_lateral_share'] * k_ax, modes_g['rest_sagittal_share'] * k_ax
    d_ax = float(round(2.0 * modes_g['rest_axial_zeta'] * math.sqrt(k_ax * body.mass)))
    d_sag = float(round(2.0 * modes_g['rest_sagittal_zeta'] * math.sqrt(k_sag * body.mass)))
    rest[latent.K][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (k_ax, k_lat, k_sag)
    rest[latent.D][[modal.AXIAL, modal.LATERAL, modal.SAGITTAL]] = (
        d_ax, modes_g['rest_lateral_d_share'] * d_ax, d_sag)
    st, tab = latent.state(rest), np.zeros((0, latent.TABLE))
    out = latent.step(st.copy(), tab, 0.0, 0.001 * every, np.zeros(latent.OUT))
    joints = np.flatnonzero(body.driven)
    a, hw, q0 = np.zeros(modal.ASK), 0.5 * float(fr[modal.WEIGHT]), sense[edge.Q][joints]
    a[modal.STANCE] = 1.0
    legs = [[i for i, j in enumerate(JOINTS) if j.startswith(s0) and kind(j) in LEG_KINDS] for s0 in ('left_', 'right_')]
    wep_lim = np.array([max(float(servos.SERVO[kind(j)][0]), float(peak(j)), wep_scale(j) * float(body.clamp[i])) for i, j in enumerate(names)])
    from machine.body import defrobot
    drv = (defrobot.read() or {}).get('drives', {})
    rng = np.array([[math.radians(drv[j]['range'][0]), math.radians(drv[j]['range'][1])] if 'range' in drv.get(j, {})
                    else [-np.inf, np.inf] for j in names])
    sit, gym = situation.text(), entries(texts=texts, suite=suite)
    from tools.cores import bodied
    sensing = bodied.on_body(wbc.sense, 'sense')
    from tools.cores import step as c_step
    picked = solver or SOLVER
    if picked == 'c':
        from tools.cores.wbc import wrench_step as solve_step
    elif picked == 'wrench':
        solve_step = c_step.wrench_laid(body, sensing)
    else:
        solve_step = {'stack': wbc.step}[picked]
    g = {'world': world, 'body': body, 'y': y, 'fr': fr, 'rest': rest, 'rest_y': rest_y, 'st': st, 'out': out, 'a': a, 'tab': tab,
         'labels': [], 'posture': s['q'].copy(), 'limbs': limbs, 'bend': bends.crouch(names), 'arms': bends.swing(names),
         'reaches': bends.reach(names), 'folds': bends.fold(names),
         'cervical': [names.index(j) for j in ('neck', 'head') if j in names], 'waist_idx': names.index('waist') if 'waist' in names else None,
         'head_idx': names.index('head') if 'head' in names else None, '_look_yaw': 0.0,
         'legs': legs, 'leg_joints': np.concatenate(legs), 'leg_kp': world.gains[np.concatenate(legs), 0].copy(),
         'free_kp': world.gains[:, 0].copy(), 'clamp': world.limit.copy(), 'wep_lim': wep_lim, 'joints': joints,
         'hold': np.zeros(len(body.clamp)), 'left': WEP_S, 'rng': rng, 'ref_q': q0.copy(), 'ref_v': np.zeros(len(q0)),
         'sense': sense, 'est': est, 'grip': grip.laid(), 'grip_prior': grip.laid(), 'situations': sit, 'into': np.zeros(len(sit['names'])),
         'grip_seen': np.zeros((2, grip.SEEN)), 'grip_settled': np.zeros(2),
         'grip_at': np.array([[m.body(s0 + part).id for part in ('foot', 'toes')] for s0 in ('left_', 'right_')]),
         'q_lo': rng[:, 0].copy(), 'q_hi': rng[:, 1].copy(),
         't': float(world.data.time), 'n': 0, 'every': every, 's': s, 'loads': [hw, hw], 'ask': {}, 'got': {}, 'wo': ESO_WO * (inner == 'eso'),
         'eso': np.array([q0, np.zeros(len(q0)), np.zeros(len(q0))]), 'jeff': np.diag(s['M'])[body.act].copy(), 'surety': 1.0,
         '_brake_wire': _BRAKE_WIRE, '_stand_entry': next((e for e in gym if e[0] == 'stand'), None),
         'gym': gym, 'texts': texts, 'suite': suite,
         'heading': float(fr[modal.HEADING]), 'twist': (0.0, 0.0, 0.0), 'asked': (0.0, 0.0, 0.0), 'governed': (0.0, 0.0, 0.0),
         'heat': heat.Heat.for_joints(JOINTS), 'temp': np.full(len(JOINTS), 25.0), 'scenario': 'flat',
         'touched': touch.laid(), 'twisted': se2.twist_laid(), 'words': se2.words_laid(),
         'solves': (solve_step, c_step.chain_laid(body, sensing, picked == 'stack')), 'sensed': sensing, 'told': np.zeros(len(ortho.MANIFOLDS)), 'weighed_w': observer.SOLES.copy(), 'emancipated': np.zeros(2), 'unanchored': np.ones(2), 'let_cap': np.full(2, np.inf), 'freed': np.zeros(2), 'emancipation': {'iota': np.zeros(2)}, 'held_ref': out[latent.REF][melt.LEGGED].copy(), 'q_open': s['q'].copy(), 'high': np.zeros(2), 'sat': 0.0, 'sat_off': 0.0, 'seat_on': 0.0, 'seat_near': 0.0, 'seat_f': np.zeros(4), 'knee_was': np.zeros(2), 'axial_off': 0.0, 'axial_was': float(out[latent.REF][modal.AXIAL]), 'rate_hz': RATE_HZ, 'observer': momentum.laid(body), 'field': field.laid(), 'inekf': inekf.laid(world.data.qpos[0:3], world.data.qpos[3:7]), 'registry': registry.laid(),
         'things': (), 'things_at': -math.inf,
         'eyes': int(physics.SIGHT > 0.0), 'keep': camera.Camera(world) if physics.CAMERA or physics.SIGHT else None,
         'planner': planner.Here(), 'vision': vision.Here()}
    motion.laid(g)
    g['beat'] = _beat().laid(g)
    world.clock = lambda: g['t']
    return g


def walked(g):
    by = {e[0]: e for e in g['gym']}
    _dropped(g)
    for label, at in WALK:
        said(g, *by[label], at=g['t'] + at)


def reset_for_scenario(g, scen):
    """Scene `scen` laid round her, she laid anew where it has her (`floor.lay`) unless she stands
    there at rest already."""
    from machine.world import floor
    from machine.law import modal
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
    from machine.body import figure
    from machine.law import latent, melt, modal
    world, body, d = g['world'], g['body'], g['world'].data
    sh, ch = math.sin(yaw), math.cos(yaw)
    world.reset(figure.stand(), where=(float(where[0]), g['rest_y'], float(where[1])),
                turn=(math.cos(0.5 * yaw), 0.0, math.sin(0.5 * yaw), 0.0))
    g['est'], g['grip'], g['grip_prior'] = estimate.laid(body, d.qpos[0:3]), grip.laid(), grip.laid()
    from machine.eyes import registry
    g['inekf'], g['registry'], g['things'], g['things_at'] = inekf.laid(d.qpos[0:3], d.qpos[3:7]), registry.laid(), (), -math.inf
    g['beat'] = _beat().laid(g)     # her world's link and her filter laid anew
    g['grip_seen'][:], g['grip_settled'][:] = 0.0, 0.0
    s = g['s'] = sensed(body, g['est'], edge.read(world, g['sense']), g['limbs'])
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
    g['_twist_active'], g['_twist_stop_until'], g['twist'], g['asked'], g['governed'], g['twist_actual'] = (
        False, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    g['active_entry'], g['n'], g['_halt_at'], g['_jog_until'] = None, 0, None, 0.0
    g['twisted'][se2.LOOK_YAW], g['twisted'][se2.TURN_DC] = 0.0, 0.0
    g['touched'][:] = touch.REST       # what she felt stood in the world laid away
    g['_w_back'], g['_backing_yaw'] = 0.0, 0.0
    g['emancipated'], g['unanchored'], g['let_cap'], g['freed'] = np.zeros(2), np.ones(2), np.full(2, np.inf), np.zeros(2)
    g['emancipation'] = {'iota': np.zeros(2)}
    g['held_ref'] = g['out'][latent.REF][melt.LEGGED].copy()
    g['q_open'] = s['q'].copy()
    g['high'] = np.zeros(2)
    g['sat'], g['sat_off'], g['seat_on'], g['seat_near'] = 0.0, 0.0, 0.0, 0.0
    g['seat_f'] = np.zeros(4)
    g['knee_was'] = np.zeros(2)
    g['axial_off'], g['axial_was'] = 0.0, float(g['out'][latent.REF][modal.AXIAL])
    supervisor.reset(g['bus'])
    for idx in (g.get('waist_idx'), g.get('head_idx')):
        if idx is not None:
            g['posture'][idx] = 0.0
    _dropped(g)
    g['labels'] = []
    g.pop('_post_stunt', None)


def looked(g):
    """A look (`vision.LOOKS`): her eyes' map's things, or the world's boxes as their stand-in, into her registry
    at `g['t']`, her things its boxes (`vision.looked`); the step's at the kernel's SDF refresh, an errand's when
    told. Her vision apart, the step's joined in her law's pass instead (`passed`)."""
    vision.looked(g)


@functools.cache
def _law():
    """Her law's modules a pass reads, imported at her first: not by the page, nor again each pass."""
    from machine.law import modal, motion, schema
    return modal, motion, schema


@functools.cache
def _beat():
    """Her pass's C (`tools.cores.beat`), imported at her first lay."""
    from tools.cores import beat
    return beat


def passed(g, dt=0.001):
    """A pass: (sensed, her soles' loads, the loop's ask, the latent out) - and her world a `dt` on: felt through
    her edge, her motion pass at her law's (`machine.law.motion`), her drives' pass and her law's joints told her
    boards in one C call, her felt in it between her law's passes (`tools.cores.beat`), a program over them; her
    world's step's first half begun beside her law (`steps.ahead`), joined here where her pass raised before it."""
    import numpy as np
    modal, motion, schema = _law()
    world, y, ev = g['world'], g['sense'], g['every']
    try:
        felt, step, both, _b, _kept = g['beat']
        if g['n'] % ev == 0:
            felt(dt)
            edge.contacted(world, y)
            if g['t'] - g.get('things_at', -math.inf) >= schema.REFRESH_S:
                (vision.look if g['vision'].apart else looked)(g)
            steps.ahead(world)
            try:
                motion.control(g, dt)
            finally:
                vision.joined(g)
            edge.drive(world, g['leg_joints'], kp=g['leg_kp'])
            bears = tuple(n >= leap.TOUCH_N for n in g['loads'])
            if any(b and not n for b, n in zip(g.get('bore', (False, False)), bears)):
                g['lifted'] = g['t']
            g['bore'] = bears
            if g['n'] > 0:
                g['temp'] = np.asarray(g['heat'].step_joints(world.data.ctrl, world.data.qvel[world.vadr], dt * ev))
        else:
            step = both
        motion.drives(g, dt, step)
        joints.played(g, world)
        # a bus refusing her setpoints longer than her balance's horizon 1/omega is no longer sporadic
        edge.held(world, 1.0 / (float(g['fr'][modal.OMEGA]) * dt))
        g['t'], g['n'] = g['t'] + dt, g['n'] + 1
        world.advance()
        return g['s'], g['loads'], g['ask'], g['out']
    finally:
        steps.behind(world)


def _heard(g, gym, command):
    from machine.process import worker
    return worker.heard(g, gym, command)


def run(*args, **kwargs):
    from machine.process.worker import run as _run
    return _run(*args, **kwargs)


def emulated_limb(state, say, limb_cls):
    emu = limb_cls(state['emu_port'])
    state['origin'] = emu.origin
    say('ok', 'boards', '%d STM32H753 nodes on Renode (%s, %s)' % (len(emu.rigs), emu.limb, emu.origin.port))
    return emu


def boot_humanoid(state, boot, say, ahead_cls, limb_cls):
    import time
    from machine.body import figure
    from machine.rig import modes
    from machine.rig.routines import TYPES
    drives = sum(len(s.actuators) for s in TYPES['gynoid'].body)
    with boot('LINKING HUMANOID') as step:
        step(0.15, 'MUJOCO & LISP')
        body = state['body'] = Running(realtime=not state.get('max', False), shown=0.0, ber=edge.LINE_BER)
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
        from machine.process.worker import Running as _Running
        return _Running(*args, **kwargs)


