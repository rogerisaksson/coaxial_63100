"""The dual-domain kernel's ops by the mechanism each feeds on the robot's loop.

The kernel's texts lisp/humanoid/kernel/, read by `machine.texts.dsp`; its protocol docs/todo/protocol.md.
"""
import functools
import itertools
import math

from machine import libm

#: Kernel posture keys mapped onto the robot's canonical movement terms (`term_rows.TERMS`).
_ORIGIN_MOVES = {
    'talocrural': ('ankle', 'dorsiflexion'), 'tibiofemoral': ('knee', 'flexion'),
    'coxofemoral': ('hip', 'flexion'), 'l4-l5-lumbar': ('trunk', 'flexion'),
    'c7-t12-thoracic': ('trunk', 'flexion'), 'glenohumeral': ('shoulder', 'flexion'),
    'cubital': ('elbow', 'flexion'), 'c1-c2-atlas': ('neck', 'flexion'),
    'c1-c2-atlas-yaw': ('head', 'rotation'), 'iliac-coronal': ('trunk', 'lateral-flexion'),
    'calcaneus': ('foot', 'eversion'), 'glenohumeral-ext-rot': ('shoulder', 'lateral-rotation'),
    'glenohumeral-abduction': ('shoulder', 'abduction'), 'glenohumeral-abduct': ('shoulder', 'abduction'),
    'coxofemoral-abduct': ('hip', 'abduction'), 'coxofemoral-ext-rot': ('hip', 'lateral-rotation'),
}

#: Each op's mechanism, the node its params land in.
MECHANISMS = {
    'evaluate-lie-articulated-dynamics': 'dynamics', 'compute-operational-space-projector': 'nullspace',
    'reallocate-nullspace-projector': 'nullspace', 'synthesize-joint-torques': 'torque-allocation',
    'modulate-nullspace-inertia-metric': 'thermal-metric', 'regulate-dc-bus-regeneration': 'dc-bus',
    'interpolate-adjoint-cones': 'wrench-homotopy', 'sequence-wrench-homotopy': 'wrench-homotopy',
    'configure-balance-feedback': 'balance-feedback', 'project-nullspace-posture': 'nullspace-posture',
    'enable-generalized-momentum-observer': 'momentum-observer', 'map-wrench-to-manifold-weights': 'manifold-weights',
    'bind-stiffness-to-support-share': 'impedance-melt', 'drive-admittance-vector-field': 'admittance-field',
    'inject-qp-barrier-constraint': 'dcm-barrier', 'configure-tangential-unlatch-law': 'capture-step',
    'drive-appendage-admittance': 'appendage-admittance', 'modulate-barrier-floor': 'barrier-floor',
    'bind-adaptive-substrate-notch': 'substrate-notch',
    'solve-grasp-wrench-decomposition': 'grasp', 'monitor-tactile-vibrometry': 'slip-reflex',
    'couple-external-rigid-body-to-cwc': 'object-coupling', 'release-bimanual-coupling': 'object-release',
    'bind-multiband-sdf-manifolds': 'sdf-bands', 'solve-eikonal-geodesic-flow': 'geodesic',
    'apply-lorentz-steering-field': 'obstacle-curl', 'modulate-swing-landing-attractor': 'foothold',
    'drive-nullspace-gauge-posture': 'aperture', 'adapt-to-surface-normal': 'surface-normal',
    'fit-oriented-obstacle-boxes': 'obstacle-boxes', 'weigh-traversability-layers': 'costmap',
    'search-costed-cell-geodesic': 'cell-search', 'spawn-wayfinder-process': 'wayfinder-apart',
    'fit-local-ground-manifold': 'ground-manifold', 'modulate-target-obstacle-permeability-cone': 'docking-funnel',
    'integrate-se2-clothoid-streamline': 'clothoid', 'decouple-subtarsal-from-pelvic-footprint-on-target': 'sub-overhang',
    'bind-stereo-vision-frontend': 'stereo', 'run-right-invariant-ekf': 'invariant-ekf',
    'update-dual-quadric-registry': 'quadric-registry', 'diffuse-unobserved-object-covariance': 'memory-decay',
    'bind-tactile-to-registry-writeback': 'tactile-writeback', 'drive-information-gain-gaze': 'active-gaze',
    'bifurcate-moving-objects': 'moving-objects', 'project-registry-to-control-buses': 'affordance-routing',
    'compute-affordance-compatibility-tensor': 'affordances', 'register-funnel-preimage-graph': 'funnels',
    'synthesize-funnel-trajectory': 'funnel-chain', 'bind-funnel-transition-executors': 'funnel-executors',
    'modulate-nullspace-oscillator': 'respiration', 'inject-stochastic-com-drift': 'postural-sway',
    'apply-zero-velocity-dither': 'stiction-dither',
    'bind-reachability-foliation': 'reachability', 'configure-sliding-manifold-regime': 'coulomb-slide',
    'drive-hamiltonian-step-cycle': 'step-energy', 'splice-somatic-object-interaction': 'somatic-shove',
    'compute-dynamic-voltage-torque-envelope': 'bemf-envelope', 'compute-smooth-capsule-distances': 'clearance-field',
    'inflate-capsules-by-stopping-distance': 'braking-cocoon', 'inject-second-order-ecbf-constraints': 'ecbf-shield',
    'modulate-ecbf-permeability-mask': 'ecbf-permeability', 'attach-dynamic-capsule-to-schema': 'payload-capsule',
    'configure-self-collision-barriers': 'self-collision', 'bind-nullspace-evasion-synergies': 'limb-retraction',
    'maintain-out-of-frustum-ecbf-anchors': 'blindspot-memory', 'bind-unexpected-wrench-recoil': 'surprise-reflex',
    'modulate-wayfinder-aperture-width': 'payload-aperture', 'run-quadrature-goertzel-probe': 'phase-margin',
    'bind-meromorphic-lax-matrix': 'higgs-field', 'solve-hitchin-fibration-map': 'hitchin-base',
    'integrate-schlesinger-wrench-transport': 'schlesinger', 'apply-twistor-sphere-rotation': 'twistor',
    'synthesize-1jet-posture-bundle': 'posture-jet', 'evaluate-1jet-modal-expansion': 'modal-pose',
    'modulate-unloaded-limb-jet': 'limb-hang', 'warp-modal-basis-submatrix': 'appendage-brace',
    'bind-coupled-torus-oscillators': 'torus-clock', 'solve-virtual-limb-length-matching': 'limb-lengths',
    'configure-multi-limb-wrench-allocation': 'virtual-truss', 'splice-fifth-peristaltic-oscillator': 'axial-peristalsis',
    'execute-nonholonomic-body-roll': 'body-roll', 'bind-substrate-dependent-roll-law': 'substrate-roll',
    'execute-topological-leg-inversion': 'leg-inversion',
    'compute-complement-cone-feasibility': 'release-gate', 'drive-causal-unloading-cascade': 'unloading-cascade',
    'blend-closed-to-open-chain-impedance': 'open-chain', 'inject-open-chain-root-wrench-into-cwc': 'cantilever',
    'evaluate-constitutive-hysteresis': 'work-loop', 'augment-invariant-ekf-contact-model': 'sinking-anchor',
    'enforce-patch-pressure-barrier': 'patch-pressure', 'gate-homotopy-by-bekker-compaction': 'tamp-commit',
    'modulate-swing-extraction-contour': 'posthole-extraction', 'couple-port-hamiltonian-to-membrane': 'membrane-lock',
    'modulate-rolling-berm-clearance': 'berm-clearance', 'compensate-winkler-depression-basin': 'winkler-basin',
    'compensate-membrane-catenary-tension': 'catenary-tension', 'place-foot-in-compacted-knee-imprint': 'imprint-step',
    'execute-harmonic-seat-drop-to-feet': 'seat-drop-kip',
    'bind-5torus-locomotion-oscillator': 'axial-torus-clock', 'modulate-proximal-balljoint-turnout': 'coxa-turnout',
    'configure-peristaltic-stick-slip-cycle': 'stick-slip', 'drive-terrestrial-elevation-ratchet': 'grade-ratchet',
    'bind-posterior-crural-datum-probe': 'calf-datum', 'inject-elliptical-seat-rim-barrier': 'seat-rim',
    'execute-coupled-so3-cantilever-sweep': 'bed-pendulum', 'execute-zero-translation-bed-roll': 'bed-roll',
    'modulate-longitudinal-bed-datum': 'bed-datum', 'inject-nullspace-posture-delta': 'posture-delta',
    'inject-two-body-stability-barrier': 'fixture-cone', 'execute-bevel-arch-vault-homotopy': 'bevel-vault',
    'execute-appendage-replant-ratchet': 'hand-replant', 'execute-overhang-triceps-dip-egress': 'overhang-dip',
    'bind-vault-contingency-reflex': 'vault-abort', 'drive-froude-continuum-morph': 'froude-homotopy',
    'bind-zero-wrench-flight-connection': 'flight-connection', 'solve-adaptive-deceleration-cascade': 'brake-horizon',
    'execute-ssc-ballistic-launch': 'ssc-launch', 'bind-braking-pitch-counterweight': 'brake-pitch-arrest',
    'modulate-touchdown-elevation-datum': 'touchdown-datum',
    'apply-laplace-innovation-shrinkage': 'static-collapse', 'solve-tri-imu-kinematic-chain': 'tri-imu',
    'compute-map-simplex-manifold-weights': 'simplex-prior', 'gate-manifold-simplex-by-patch-speed': 'support-stillness',
    'read-floor-friction-from-slides': 'floor-friction', 'run-active-proprioceptive-weight-probe': 'weight-probe', 'set-limb-health-mask': 'limb-health',
    'reconfigure-underactuated-operational-space': 'fault-partition',
    'enforce-zero-net-yaw-wrench-allocation': 'yaw-nullifier', 'guard-underactuated-appendage-envelope': 'towed-limb'}
#: The kernel's ops of many, each held under its node by the param naming it.
KEYED = {'inject-adjoint-cone': ('adjoint-cones', 'id'), 'orient-manifold-normal': ('manifold-normals', 'patch')}
#: The kernel's queries of the robot's world, each a Law it resolves as it goes.
QUERIES = ('query-nearest-affordance', 'slot-surface-normal', 'slot-estimated-mu', 'query-upcoming-obstacle-span-x',
           'query-upcoming-obstacle-height-z')


@functools.cache
def _read(robot):
    from machine.texts import dsp
    return dsp.read(dsp.KERNEL)


def read():
    """Every name of the kernel's texts, evaluated (`dsp.read`) for the active humanoid build."""
    from machine.body import mechatronics
    return _read(mechatronics.ROBOT)


def said(stage, key):
    """A kernel stage's `key` as its text defaults it."""
    from machine.texts import dsp
    return dsp.value(read()[stage].keys[key], read())


def node(stage, at):
    """A kernel modifier's node `at` as it lays it on the robot's nominal gait."""
    from machine.texts import dsp
    return dsp.value([stage, 'nominal-biped-gait'], read())[at]


@functools.cache
def _transitions(robot):
    from machine.texts import dsp
    got = _read(robot)
    bound = (name for name, stage in got.items()
             if isinstance(stage, dsp.Stage) and 'bind-funnel-transition-executors' in str(stage.body))
    return {(t['from'], t['to']): t['execute'] for name in bound for t in node(name, 'funnel-executors')['transitions']}


def transitions():
    """Every funnel transition the kernel's modifiers bind (`bind-funnel-transition-executors`): (from, to) ->
    its executor, a pipeline of the kernel's stages."""
    from machine.body import mechatronics
    return _transitions(mechatronics.ROBOT)


def transition(frm, to, at):
    """A funnel transition's node `at`: its executor (`transitions`) run on the robot's nominal gait."""
    from machine.texts import dsp
    return dsp.run(transitions()[(frm, to)], read()['nominal-biped-gait'])[at]


def clothoid_kappa():
    """1/m: curvature bound from the active build's walk turning arc (`walk_m_s / yaw_rad_s`)."""
    from machine.body import gait
    tw = gait.table('twist')
    return round(tw['yaw_rad_s'] / tw['walk_m_s'], 2)


def clothoid_dkappa():
    """1/(m s): clothoid curvature rate taking up the walk's arc over one step (`0.5 / walk_hz`)."""
    from machine.body import gait
    tw = gait.table('twist')
    return round((tw['yaw_rad_s'] / tw['walk_m_s']) / (0.5 / tw['walk_hz']), 2)


def _laid_origin(origin):
    from machine.body import figure
    from machine.body.term_rows import ABSENT, TERMS
    deg = figure.stand()
    deg.update({k: 0.0 for k in deg if k.endswith(('_hip', '_knee', '_ankle', '_shoulder', '_elbow'))})
    for name, rad in origin.items():
        if _ORIGIN_MOVES[name] in ABSENT:
            continue
        kind, sign = TERMS[_ORIGIN_MOVES[name]][:2]
        d = sign * math.degrees(rad)
        if kind == 'spine':
            deg['spine'] += d
        else:
            deg.update({j: d for j in ([kind] if kind in deg else ['left_' + kind, 'right_' + kind])})
    return deg


def crural_hang_hz(talocrural):
    """Hz: small-angle natural frequency of the active build's shank, foot and toes hung from its knee."""
    from machine.body import build, compile as body, figure
    deg = _laid_origin({'talocrural': float(talocrural)})
    placed, built = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT), build.segments()
    _kx, ky, kz = placed['left_shank'][0]
    moment = inertia = 0.0
    for name in ('left_shank', 'left_foot', 'left_toes'):
        kg, centre, about = built[name]
        _px, py, pz = figure.add(placed[name][0], figure.apply(placed[name][1], centre))
        ry, rz = py - ky, pz - kz
        moment += kg * -ry
        inertia += about[0] + kg * (ry * ry + rz * rz)
    return round(math.sqrt(body.G_M_S2 * moment / inertia) / math.tau, 3)


@functools.cache
def _roll_radius_cached(_robot, origin_items, r_sh, r_el, l_hip, l_add, l_sh):
    import numpy as np
    from machine.body import figure
    from machine.body.term_rows import TERMS
    deg = _laid_origin(dict(origin_items))
    deg.update(right_shoulder=math.degrees(r_sh), right_elbow=math.degrees(r_el),
               left_hip=TERMS[('hip', 'flexion')][1] * math.degrees(l_hip),
               left_hip_roll=TERMS[('hip', 'adduction')][1] * math.degrees(l_add),
               left_shoulder=math.degrees(l_sh))
    segments = {s[0] for s in figure.SEGMENTS if not s[0].endswith('fingers')}
    points, radii = [], []
    for name, (at, here) in figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT).items():
        part = name.split('_', 1)[1] if name.startswith(('left_', 'right_')) else name
        caps = [(r, e) for p, r, a, b in figure.BODY if name in segments and p == part for e in (a, b)]
        boxes = [(0.0, tuple(c[i] + s[i] * size[i] for i in range(3)))
                 for p, _shape, size, c in figure.CONTACTS if name in segments and p == part
                 for s in itertools.product((-1.0, 1.0), repeat=3)]
        for r, e in caps + boxes:
            points.append(figure.add(at, figure.apply(here, e)))
            radii.append(r)
    pts, rad = np.array(points), np.array(radii)
    centre = np.array(figure.com(deg, (0.0, 0.0, 0.0), figure.IDENT))
    theta = np.linspace(0.0, 0.5 * math.pi, 1801)
    up = np.stack([libm.sin(theta), np.zeros_like(theta), libm.cos(theta)], axis=1)
    h = up @ centre - (up @ pts.T - rad).min(axis=1)
    quarter = float(np.sum(0.5 * (h[1:] + h[:-1]) * np.diff(theta)))
    return round(quarter / (0.5 * math.pi), 4)


def roll_radius_m(env):
    """m: effective torso roll radius over a quarter turn onto the side in the supine cantilever posture."""
    from machine.body import mechatronics
    from machine.texts import dsp
    origin = env['groundwork-1jet-posture-atlas']['postures']['floor-supine-flat']['origin-q0']
    roll = dsp.value(['xform-continuous-ground-roll', 'nominal-biped-gait'], env)['body-roll']
    slot, lever = roll['ipsilateral-limb-clearance'], roll['contralateral-momentum-driver']
    return _roll_radius_cached(
        mechatronics.ROBOT, tuple(sorted(origin.items())),
        float(slot['glenohumeral-flexion-rad']), float(slot['cubital-flexion-rad']),
        float(lever['coxofemoral-flexion-rad']), float(lever['coxofemoral-adduction-rad']),
        float(lever['glenohumeral-cross-reach-rad']))


@functools.cache
def _launch_v_cached(_robot, depth_mm, legs, svpwm):
    import numpy as np
    from machine.body import compile as body, drives, figure
    n, straight, depth = 4000, figure.THIGH + figure.SHANK, 1e-3 * depth_mm
    x = np.linspace(straight - depth, straight, n + 1)[:-1]
    cos = (figure.THIGH * figure.THIGH + figure.SHANK * figure.SHANK - x * x) / (2.0 * figure.THIGH * figure.SHANK)
    lever = figure.THIGH * figure.SHANK * np.sqrt(1.0 - cos * cos) / x
    m, g = figure.mass(), body.G_M_S2
    big = m + legs * drives.armature('left_knee') / (lever * lever)
    dx = float(x[1] - x[0])
    k_v = drives.kt('left_knee') / drives.of('left_knee')[1].r
    emf_k, peak_k, v_lim = drives.emf('left_knee'), drives.peak('left_knee'), svpwm * drives.PACK_V
    energy, v = np.zeros(n), np.zeros(n)
    for k in range(n - 1):
        lo = max(-peak_k, min(0.0, k_v * (-v_lim - emf_k * (-v[k] / lever[k]))))
        energy[k + 1] = max(0.0, energy[k] + (-legs * lo / lever[k] - m * g) * dx)
        v[k + 1] = math.sqrt(2.0 * energy[k + 1] / big[k + 1])
    acc = np.gradient(v, dx) * v
    idx = int(np.argmax(np.append(m * (acc[:-1] + g) <= 0.0, True)))
    return float(v[idx]), g


def jump_launch_mm(env, depth_mm, dz_ref=1.0, dx_ref=0.0, legs=2.0, vx_mps=None):
    """(apex_mm, span_mm): knee back-EMF ballistic launch apex and horizontal span for the active build."""
    from machine.body import mechatronics
    from machine.texts import dsp
    svpwm = float(dsp.value(['update-asymmetric-bemf-bounds', 'nominal-biped-gait'], env)['bemf-envelope'][
        'svpwm-modulation-limit'])
    v_leave, g = _launch_v_cached(mechatronics.ROBOT, round(float(depth_mm), 4), float(legs), round(svpwm, 6))
    if vx_mps is not None:
        vz, vx = v_leave, float(vx_mps)
    else:
        ang = libm.arctan2(4.0 * float(dz_ref), float(dx_ref))
        vz, vx = v_leave * float(libm.sin(ang)), v_leave * float(libm.cos(ang))
    return round(1e3 * vz * vz / (2.0 * g), 1), round(1e3 * 2.0 * vx * vz / g, 1)


@functools.cache
def _vault_height_cached(_robot, origin_items, col_items, a):
    import numpy as np
    from machine.body import figure
    origin, col = dict(origin_items), dict(col_items)
    deg = _laid_origin({j: origin.get(j, 0.0) + a * col.get(j, 0.0) for j in set(origin) | set(col)})
    fists = 0.5 * (np.array(figure.fist(deg, side='left')) + np.array(figure.fist(deg, side='right'))) - (
        0.0, figure.FIST_R, 0.0)
    centre = np.array(figure.com(deg, (0.0, 0.0, 0.0), figure.IDENT)) - fists
    cy, cz = float(centre[1]), float(centre[2])
    norm = math.sqrt(cy * cy + cz * cz)
    return -(cy * float(fists[1]) + cz * float(fists[2])) / norm


def vault_stroke_mm(env, a0, a1):
    """mm: fist-hung pelvic vertical stroke between axial modal coordinates `a0` and `a1`."""
    from machine.body import mechatronics
    dip = env['high-vault-1jet-posture-atlas']['postures']['carpal-dip-suspension']
    origin = tuple(sorted(dip['origin-q0'].items()))
    col = tuple(sorted(dip['modal-basis-columns']['mode-axial-compression'].items()))
    h0 = _vault_height_cached(mechatronics.ROBOT, origin, col, round(float(a0), 6))
    h1 = _vault_height_cached(mechatronics.ROBOT, origin, col, round(float(a1), 6))
    return round(1e3 * (h0 - h1), 1)


#: Pure build-parameterized functions and special forms exposed in `machine.texts.dsp`.
FUNCS = {
    'robot-clothoid-kappa': clothoid_kappa,
    'robot-clothoid-dkappa': clothoid_dkappa,
    'robot-crural-hang-hz': crural_hang_hz,
}


def special(val_fn):
    """Build-parameterized special forms `(args, env) -> value` for `dsp.SPECIAL`."""
    return {
        'robot-roll-radius-m': lambda _a, env: roll_radius_m(env),
        'robot-vault-stroke-mm': lambda a, env: vault_stroke_mm(env, val_fn(a[0], env), val_fn(a[1], env)),
        'robot-jump-apex-mm': lambda a, env: jump_launch_mm(env, val_fn(a[0], env))[0],
        'robot-jump-angled-apex-mm': lambda a, env: jump_launch_mm(env, *(val_fn(x, env) for x in a))[0],
        'robot-jump-angled-span-mm': lambda a, env: jump_launch_mm(env, *(val_fn(x, env) for x in a))[1],
        'robot-leap-apex-mm': lambda a, env: jump_launch_mm(
            env, val_fn(a[0], env), legs=1.0, vx_mps=val_fn(a[1], env))[0],
        'robot-leap-span-mm': lambda a, env: jump_launch_mm(
            env, val_fn(a[0], env), legs=1.0, vx_mps=val_fn(a[1], env))[1],
    }
