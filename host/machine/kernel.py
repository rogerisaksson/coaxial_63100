"""The dual-domain kernel's ops by the mechanism each feeds on her loop.

Its texts machine/ortho_kernel/, read by `machine.dsp`; its protocol docs/todo/protocol.md.
"""
import functools

#: Each op's mechanism, the node its params land in.
MECHANISMS = {
    'evaluate-lie-articulated-dynamics': 'dynamics', 'compute-operational-space-projector': 'nullspace',
    'reallocate-nullspace-projector': 'nullspace', 'synthesize-joint-torques': 'torque-allocation',
    'modulate-nullspace-inertia-metric': 'thermal-metric', 'regulate-dc-bus-regeneration': 'dc-bus',
    'interpolate-adjoint-cones': 'wrench-homotopy', 'sequence-wrench-homotopy': 'wrench-homotopy',
    'configure-balance-feedback': 'balance-feedback', 'rebalance-subpatch-symmetry': 'subpatch',
    'project-nullspace-posture': 'nullspace-posture',
    'enable-generalized-momentum-observer': 'momentum-observer', 'map-wrench-to-manifold-weights': 'manifold-weights',
    'bind-stiffness-to-support-share': 'impedance-melt', 'drive-admittance-vector-field': 'admittance-field',
    'inject-qp-barrier-constraint': 'dcm-barrier', 'configure-tangential-unlatch-law': 'capture-step',
    'drive-appendage-admittance': 'appendage-admittance', 'modulate-barrier-floor': 'barrier-floor',
    'bind-adaptive-substrate-notch': 'substrate-notch',
    'solve-grasp-wrench-decomposition': 'grasp', 'monitor-tactile-vibrometry': 'slip-reflex',
    'couple-external-rigid-body-to-cwc': 'object-coupling', 'release-bimanual-coupling': 'object-release',
    'reorient-coupled-quadric': 'object-pose',
    'bind-multiband-sdf-manifolds': 'sdf-bands', 'solve-eikonal-geodesic-flow': 'geodesic',
    'apply-lorentz-steering-field': 'obstacle-curl', 'modulate-swing-landing-attractor': 'foothold',
    'drive-nullspace-gauge-posture': 'aperture', 'adapt-to-surface-normal': 'surface-normal',
    'configure-compliant-obstacle-filter': 'compliant-obstacles',
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
    'evaluate-constitutive-hysteresis': 'work-loop', 'augment-invariant-ekf-contact-model': 'sinking-anchor',
    'enforce-patch-pressure-barrier': 'patch-pressure', 'gate-homotopy-by-bekker-compaction': 'tamp-commit',
    'modulate-swing-extraction-contour': 'posthole-extraction', 'couple-port-hamiltonian-to-membrane': 'membrane-lock',
    'modulate-rolling-berm-clearance': 'berm-clearance', 'compensate-winkler-depression-basin': 'winkler-basin',
    'compensate-membrane-catenary-tension': 'catenary-tension', 'place-foot-in-compacted-knee-imprint': 'imprint-step',
    'execute-harmonic-seat-drop-to-feet': 'seat-drop-kip'}
#: The kernel's ops of many, each held under its node by the param naming it.
KEYED = {'inject-adjoint-cone': ('adjoint-cones', 'id'), 'orient-manifold-normal': ('manifold-normals', 'patch')}


@functools.cache
def read():
    """Every name of the kernel's texts, evaluated (`dsp.read`)."""
    from machine import dsp
    return dsp.read(dsp.KERNEL)


def said(stage, key):
    """A kernel stage's `key` as its text defaults it."""
    from machine import dsp
    return dsp.value(read()[stage].keys[key], read())


def node(stage, at):
    """A kernel modifier's node `at` as it lays it on her nominal gait."""
    from machine import dsp
    return dsp.value([stage, 'nominal-biped-gait'], read())[at]


@functools.cache
def transitions():
    """Every funnel transition the kernel's modifiers bind (`bind-funnel-transition-executors`): (from, to) ->
    its executor, a pipeline of the kernel's stages."""
    from machine import dsp
    got = read()
    bound = (name for name, stage in got.items()
             if isinstance(stage, dsp.Stage) and 'bind-funnel-transition-executors' in str(stage.body))
    return {(t['from'], t['to']): t['execute'] for name in bound for t in node(name, 'funnel-executors')['transitions']}


def transition(frm, to, at):
    """A funnel transition's node `at`: its executor (`transitions`) run on her nominal gait."""
    from machine import dsp
    return dsp.run(transitions()[(frm, to)], read()['nominal-biped-gait'])[at]
