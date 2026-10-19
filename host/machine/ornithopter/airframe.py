"""The ornithopter's airframe: its numbers (`ornithopter.lisp`), its wings' strokes, its MuJoCo XML."""
import math
import os

from machine import aerolisp, quad
from machine.aerobatics import HOVER
from machine.quad import inflow

LISP_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'ornithopter.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
CARD, COOLING = MANIFEST['routines']['ornithopter_dune'], DYNAMICS['cooling']
_FW, _AF, _DL = AERO['flapping_wings'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
WING_SPAN, WING_CHORD = _FW['wing_span'], _FW['wing_chord']
SHOULDER_X, FORE_Z, AFT_Z, SHOULDER_Y = (
    _FW['shoulder_x'], _FW['fore_z'], _FW['aft_z'], _FW['shoulder_y'])
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), WING_SPAN
IDLE_RAD_S, GOV_RAD_S, AFT_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, AFT_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
STROKE_DEG, PITCH_DEG, FOLD_DEG = (round(math.degrees(x), 6) for x in (
    _FW['stroke_limit'], _FW['pitch_limit'], _FW['sweep_fold_limit']))
K_THRUST, K_LIFT, K_DRAG = _FW['k_thrust'], _FW['k_lift'], _FW['k_drag']
K_PITCH, K_ROLL, K_YAW = _FW['k_pitch'], _FW['k_roll'], _FW['k_yaw']
INFLOW_FLOOR, RHO, BODY_CDA = _FW['inflow_floor'], _AF['air_density'], _AF['parasite_cda']
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (KINEMATICS['envelope'][k] for k in (
    'floor_height', 'hover_height', 'top_height'))
FLAP_ROCK_HZ, FLAP_ROCK_RAD = (KINEMATICS['oscillator'][k] for k in (
    'tictoc_frequency', 'tictoc_amplitude'))
RIDGE_SOAR, THERMAL_SPIRAL, FOLD_ENTRY, SANDSTORM_DIVE, DIVE_PIRO = (STUNTS[k] for k in (
    'ridge_soar', 'thermal_spiral', 'fold_entry', 'sandstorm_dive', 'dive_piro'))
WING_ROCK, BANK_ROCK, DUNE_SLASH, CORIOLIS_STORM, DESERT_UNWIND, UNWIND = (STUNTS[k] for k in (
    'wing_rock', 'bank_rock', 'dune_slash', 'coriolis_storm', 'desert_unwind', 'unwind'))
#: The crank's gear from a drive's turn to a beat: 9.7 Hz at the governor, a 2 kg flapper's.
#: A pair's lift pulses BEAT_PULSE of its mean twice a beat, the pairs in antiphase: their
#: sum steady, a pitching moment at the beat.
GEAR_RATIO, BEAT_PULSE = _FW['gear_ratio'], _FW['beat_pulse']
PULL_MAX = (GOV_RAD_S * STUNT_BOOST) ** 2 * K_THRUST / MASS_KG
#: Its laps' sphere: 2.0 and 1.7 struck gate 7, 1.5 lapped in 24 s whole.
COURSE_SPHERE = 1.5 * GRAVITY


def drive_loads(controls):
    """External load scales `(scale_fore, scale_aft)` on the FORE and AFT 63100 wing drives."""
    stroke, pitch_t, roll_d, yaw_t = controls
    return (min(_DL['main_ceiling'], _DL['main_base'] + _DL['main_collective'] * (
                stroke * stroke + _DL['main_cyclic'] * (pitch_t * pitch_t + roll_d * roll_d))),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['main_collective'] * (
                0.85 * stroke * stroke + 0.5 * _DL['tail_pitch'] * yaw_t * yaw_t)))


def wing_kinematics(controls, speeds):
    """`(beat_hz, stroke_deg, fold_deg)` of the 4 articulated wings for `controls` and `speeds`."""
    stroke, pitch_t, _roll_d, _yaw_t = controls
    w_mean = 0.5 * (float(speeds[0]) + float(speeds[1]))
    beat_hz = w_mean / (math.tau * GEAR_RATIO)
    stroke_deg = stroke * STROKE_DEG
    folded = max(0.0, min(1.0, -stroke - 0.10)) if stroke < -0.10 else (
        0.35 if w_mean < 120.0 else 0.12 * abs(pitch_t))
    return beat_hz, stroke_deg, folded * FOLD_DEG


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into an ornithopter stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


def mjcf(things=()):
    """The 4-wing Dune ornithopter and `things` (`grounds.solids`) as MuJoCo XML."""
    m = ('<geom name="carapace" type="box" pos="0 0.01 0.08" size="0.065 0.048 0.19" contype="1" '
         'conaffinity="1"/><geom name="fore_wings" type="box" pos="0 %g %g" size="%g 0.008 %g" '
         'contype="1" conaffinity="1"/><geom name="aft_wings" type="box" pos="0 %g %g" '
         'size="%g 0.008 %g" contype="1" conaffinity="1"/><geom name="boom" type="capsule" '
         'fromto="0 0.018 -0.08 0 0.018 -0.64" size="0.013" contype="1" conaffinity="1"/>'
         '<geom name="vtail" type="box" pos="0 0.04 -0.62" size="0.11 0.045 0.03" contype="1" '
         'conaffinity="1"/>' % (SHOULDER_Y, FORE_Z, 0.85 * WING_SPAN, 0.5 * WING_CHORD,
                                SHOULDER_Y + 0.01, AFT_Z, 0.75 * WING_SPAN, 0.45 * WING_CHORD))
    for k, (sx, sz) in enumerate((sx, sz) for sx in (-SKID_X, SKID_X) for sz in STRUT_Z):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -SKID_M, sz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody>' '</mujoco>' % ((head, SKID_M + FOOT_M, MASS_KG) + INERTIA + (m,)))


def _wing_drag(stroke, pitch_t, roll_d, w_eq):
    return (0.32 + 1.55 * (stroke * stroke + 0.48 * (pitch_t * pitch_t + roll_d * roll_d))) * (
        K_DRAG * w_eq * w_eq)


def _wing_cda(stroke=0.0):
    """Total carapace and articulated 4-wing drag area $C_D A$."""
    return BODY_CDA + 0.08 * WING_SPAN * WING_CHORD * (1.0 + 0.5 * abs(float(stroke)))


def _flap_inflow(along, v_horiz, w_eq):
    """Combined axial inflow and translational flapping lift factor (`K_LIFT`)."""
    tip = max(w_eq * WING_SPAN, 8.0)
    return inflow(along, w_eq) * (1.0 + K_LIFT * min(0.35, v_horiz / tip))
