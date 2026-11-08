"""A handover spec's knobs: the texts edited and the law's numbers set as its `key=value` pairs say.

    texts = knobs.arranged(handover.parsed('sea=143,track=12,edge=0.1'))   # the texts given (tune.given), or None

`ax` the walk's axial target, `rock`/`drive` the walk stream's, `k`/`c` the stride spring's, `heel`, `heelrate`,
`flatk`, `fold`, `dip`, `ls`, `rise`, `edge` the stride's and rocker's, `mhz`/`mdrive`/`mturn`/`max` the march's,
`pace`/`pace_fast` the swing's, `rund` the run's drive, `sea` the ankle's series spring, `wind` the board's wound-ahead
ask, `track` the rotor's torque loop, `wfold` the stack's knee fold weight (tools/sim/handover.py).
"""
import math
import os

GAIT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lisp', 'humanoid', 'gait_base.lisp')
POSTURES_PATH = os.path.join(os.path.dirname(GAIT_PATH), 'postures_base.lisp')


def literal(src, key, start=0):
    """(where the number after `key` begins and ends in `src`, from `start`, and its value)."""
    j = src.find(key, start) + len(key)
    z = j
    while src[z] in '0123456789.-':
        z += 1
    return j, z, float(src[j:z])


def arranged(spec):
    """The texts and the stride's numbers as `spec` has them: the texts edited (`tune.given`), or None."""
    from machine.law import rocker, se2, stride, vault
    from machine.texts import ortho, tune
    texts = tune.texts()
    GAIT, POSTURES = (next(k for k in texts if os.path.normcase(os.path.abspath(k)) == os.path.normcase(p)) for p in (GAIT_PATH, POSTURES_PATH))
    gait, postures = texts[GAIT], texts[POSTURES]
    edits = {}
    if 'ax' in spec:
        j, z, _ = literal(se2.WALK_ON, '(mode_axial_compression ')
        se2.WALK_ON = se2.WALK_ON[:j] + spec['ax'] + se2.WALK_ON[z:]
        se2._WALK_ON_WIRE = ortho.compiled(se2.WALK_ON)[0]['']
        j, z, _ = literal(postures, '(mode_axial_compression ', postures.find('(defbehavior init_walk'))
        edits[(POSTURES, j, z)] = float(spec['ax'])
    walk = gait.find('(sequence_id 204)')
    if 'rock' in spec:
        j, z, was = literal(gait, '(amplitude mode_foot_rock ', walk)
        edits[(GAIT, j, z)] = round(was * float(spec['rock']), 3)
    if 'drive' in spec:
        j, z, _ = literal(gait, '(amplitude mode_sagittal_drive ', walk)
        edits[(GAIT, j, z)] = float(spec['drive'])
    jog = postures.find('(defbehavior jog')
    for key, name in (('mhz', '(frequency '), ('mdrive', '(amplitude mode_sagittal_drive '), ('mturn', '(amplitude mode_pelvis_turn '),
                      ('max', '(mode_axial_compression ')):
        if key in spec:
            at = jog
            for _phase in range(2):
                j, z, _ = literal(postures, name, at)
                edits[(POSTURES, j, z)] = float(spec[key])
                at = z
    if 'k' in spec:
        j, z, _ = literal(gait, '(stiffness mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec['k'])
        j, z, _ = literal(gait, '(damping mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec.get('c', 0.2 * float(spec['k'])))
    # a stride constant is bound by name where rocker and vault import it: set in each (the C table takes stride's)
    for key, name, scale in (('heel', 'HEEL_RAD', math.pi / 180.0), ('heelrate', 'HEEL_RAD_S', 1.0), ('flatk', 'FLAT_K', 1.0),
                             ('fold', 'FOLD_RAD', 1.0), ('dip', 'DIP_M', 1.0), ('ls', 'LAND_SHARE', 1.0), ('rise', 'RISE_U', 1.0),
                             ('edge', 'EDGE_RAD', 1.0)):
        if key in spec:
            for mod in (stride, rocker, vault):
                if hasattr(mod, name):
                    setattr(mod, name, scale * float(spec[key]))
    if 'swing' in spec:
        stride.SWING = float(spec['swing'])
        stride.PHI_W = math.pi * stride.SWING
    if 'swing_fast' in spec:
        stride.SWING_FAST = float(spec['swing_fast'])
    if 'pace_fast' in spec:
        stride.PACE_FAST_M_S = float(spec['pace_fast'])
    if 'rund' in spec:
        from machine.law import rhythm
        rhythm.R['run_drive'] = float(spec['rund'])
    if 'sea' in spec:
        from machine.body import drives
        drives.SERIES['ankle'] = float(spec['sea'])
    if 'wind' in spec:
        from machine.body import drives
        drives.AHEAD_WIND = float(spec['wind'])
    if 'track' in spec:
        from machine.body import gearbox
        gearbox.TRACK = float(spec['track'])
    if 'rates' in spec:
        from machine.body import gearbox
        gearbox.RATE_S = float(spec['rates'])
    if 'wfold' in spec:
        from machine.stack import wrench
        wrench.W_FOLD = float(spec['wfold'])
    return tune.given(edits) if edits else None
