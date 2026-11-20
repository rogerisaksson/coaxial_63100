"""The ornithopter's airframe: its numbers (`ornithopter.lisp`), its wings' strokes, its MuJoCo XML."""
import math

from machine.aircraft import aerolisp, compile as aircraft_compile, quad, text
from machine.aircraft.aerobatics import HOVER
from machine.aircraft.quad import inflow

LISP_PATH = text('ornithopter')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS = MANIFEST['streams'], MANIFEST['stunts']
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
PROFILE_DRAG, INDUCED_DRAG, CYCLIC_DRAG = (
    _FW['profile_drag_share'], _FW['induced_drag_share'], _FW['cyclic_drag_share'])
K_PITCH, K_ROLL, K_YAW = _FW['k_pitch'], _FW['k_roll'], _FW['k_yaw']
INFLOW_FLOOR, RHO, BODY_CDA = _FW['inflow_floor'], _AF['air_density'], _AF['parasite_cda']
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (KINEMATICS['envelope'][k] for k in (
    'floor_height', 'hover_height', 'top_height'))
WAKE_FLOOR_M_S, FLAP_ROCK_HZ, FLAP_ROCK_RAD = (
    KINEMATICS['vertical']['wake_floor'],
    *(KINEMATICS['oscillator'][k] for k in ('tictoc_frequency', 'tictoc_amplitude')))
SANDSTORM_DIVE = STUNTS['sandstorm_dive']
#: The crank's gear from a drive's turn to a beat: 9.7 Hz at the governor, a 2 kg flapper's.
#: A pair's lift pulses BEAT_PULSE of its mean twice a beat, the pairs in antiphase: their
#: sum steady, a pitching moment at the beat.
GEAR_RATIO, BEAT_PULSE = _FW['gear_ratio'], _FW['beat_pulse']
PULL_MAX = (GOV_RAD_S * STUNT_BOOST) ** 2 * K_THRUST / MASS_KG


def drive_loads(controls, _share=1.0):
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
        INFLOW_FLOOR if w_mean < 0.625 * IDLE_RAD_S else 0.12 * abs(pitch_t))
    return beat_hz, stroke_deg, folded * FOLD_DEG


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into an ornithopter stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


_ORNI_MJ = aircraft_compile.part('mujoco')['ornithopter']


def mjcf(things=()):
    """The 4-wing Dune ornithopter and `things` (`grounds.solids`) as MuJoCo XML."""
    g = _ORNI_MJ
    m = ('<geom name="carapace" type="box" pos="%g %g %g" size="%g %g %g" contype="1" '
         'conaffinity="1"/><geom name="fore_wings" type="box" pos="%g %g %g" size="%g %g %g" '
         'contype="1" conaffinity="1"/><geom name="aft_wings" type="box" pos="%g %g %g" '
         'size="%g %g %g" contype="1" conaffinity="1"/><geom name="boom" type="capsule" '
         'fromto="%g %g %g %g %g %g" size="%g" contype="1" conaffinity="1"/>'
         '<geom name="vtail" type="box" pos="%g %g %g" size="%g %g %g" contype="1" '
         'conaffinity="1"/>' % (*g['carapace_pos'], *g['carapace_size'],
                                *g['fore_pos'], *g['fore_size'],
                                *g['aft_pos'], *g['aft_size'],
                                *g['boom_from'], *g['boom_to'], g['boom_r'],
                                *g['vtail_pos'], *g['vtail_size'])
         + quad.skids(SKID_X, SKID_M, STRUT_Z, FOOT_M))
    return quad.mjcf(things, m, (0.0, SKID_M + FOOT_M, 0.0), MASS_KG, INERTIA)


def _wing_drag(stroke, pitch_t, roll_d, w_eq):
    return (PROFILE_DRAG + INDUCED_DRAG * (stroke * stroke + CYCLIC_DRAG * (pitch_t * pitch_t + roll_d * roll_d))) * (
        K_DRAG * w_eq * w_eq)


def _wing_cda(stroke=0.0):
    """Total carapace and articulated 4-wing drag area $C_D A$."""
    return BODY_CDA + 0.5 * K_LIFT * WING_SPAN * WING_CHORD * (1.0 + 0.5 * abs(float(stroke)))


def _flap_inflow(along, v_horiz, w_eq):
    """Combined axial inflow and translational flapping lift factor (`K_LIFT`)."""
    tip = max(w_eq * WING_SPAN, WAKE_FLOOR_M_S)
    return inflow(along, w_eq) * (1.0 + K_LIFT * min(INFLOW_FLOOR, v_horiz / tip))
