"""The helicopter's airframe: its numbers (`helicopter.lisp`), its MuJoCo XML, its rotors, its drives."""
import math
import os

from machine import aerolisp, quad
from machine.aerobatics import HOVER
from machine.quad import inflow

LISP_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'helicopter.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())
QUAD_LISP_PATH, QUAD_MANIFEST = quad.QUAD_LISP_PATH, quad.MANIFEST

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
CARD, COOLING = MANIFEST['routines']['helicopter_3d'], DYNAMICS['cooling']
_MR, _TR, _AF, _DL = AERO['main_rotor'], AERO['tail_rotor'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
DISC_M, ROTOR_R, TAIL_M, TAIL_R = _MR['disc_height'], _MR['radius'], _TR['boom_arm'], _TR['radius']
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), ROTOR_R
IDLE_RAD_S, GOV_RAD_S, TAIL_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, TAIL_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
COLL_DEG, CYC_DEG, TAIL_DEG = (round(math.degrees(x), 6) for x in (
    _MR['collective_limit'], _MR['cyclic_limit'], _TR['pitch_limit']))
K_THRUST, K_CYC, K_DRAG = _MR['k_thrust'], _MR['k_cyclic'], _MR['k_drag']
PROFILE_DRAG, INDUCED_DRAG, CYCLIC_DRAG = (
    _MR['profile_drag_share'], _MR['induced_drag_share'], _MR['cyclic_drag_share'])
INFLOW_FLOOR, K_TAIL, RHO, BODY_CDA = (
    _MR['inflow_floor'], _TR['k_thrust'], _AF['air_density'], _AF['parasite_cda'])
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (
    KINEMATICS['envelope'][k] for k in ('floor_height', 'hover_height', 'top_height'))
PULL_MAX = K_THRUST * (GOV_RAD_S * STUNT_BOOST) ** 2 / MASS_KG
COURSE_SPHERE = 2.0 * GRAVITY
TICTOC_HZ, TICTOC_RAD = (
    KINEMATICS['oscillator'][k] for k in ('tictoc_frequency', 'tictoc_amplitude'))
FUNNEL, SPIRAL, FLIP_IN, INVERTED, INV_PIRO, TICTOC, AILERON_TICTOC, PIRO_FLIP, CHAOS, HURRICANE, UNWIND = (
    STUNTS[k] for k in ('funnel', 'spiral', 'flip_in', 'inverted', 'inv_piro', 'tictoc',
                        'aileron_tictoc', 'piro_flip', 'chaos', 'hurricane', 'unwind'))


def drive_loads(controls):
    """The two drives' load scales `(main, tail)` for `controls`."""
    c, p, r, t = controls
    return (min(_DL['main_ceiling'], _DL['main_base'] + _DL['main_collective'] * (
                c * c + _DL['main_cyclic'] * (p * p + r * r))),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['tail_pitch'] * t * t))


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into a stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


def mjcf(things=()):
    """The helicopter and `things` (`grounds.solids`) as MuJoCo XML."""
    m = ('<geom name="canopy" type="box" pos="0 0 0.08" size="0.06 0.05 0.18" contype="1" '
         'conaffinity="1"/><geom name="disc" type="cylinder" pos="0 %g 0" size="%g 0.008" '
         'euler="90 0 0" contype="1" conaffinity="1"/><geom name="boom" type="capsule" '
         'fromto="0 0.012 -0.05 0 0.012 %g" size="0.014" contype="1" conaffinity="1"/>'
         '<geom name="fin" type="box" pos="0 0.02 %g" size="0.01 0.08 0.03" contype="1" '
         'conaffinity="1"/>' % (DISC_M, ROTOR_R, -TAIL_M, -TAIL_M))
    for k, (sx, sz) in enumerate((sx, sz) for sx in (-SKID_X, SKID_X) for sz in STRUT_Z):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -SKID_M, sz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((head, SKID_M + FOOT_M, MASS_KG) + INERTIA + (m,)))


def _rotor_drag(coll, cyc_p, cyc_r, w):
    return (PROFILE_DRAG + INDUCED_DRAG * (coll * coll + CYCLIC_DRAG * (
        cyc_p * cyc_p + cyc_r * cyc_r))) * K_DRAG * w * w


def _rotor_inflow(along, v_horiz, w_m):
    return inflow(along, w_m) * (1.0 + 0.65 * min(0.35, v_horiz / max(w_m * ROTOR_R, 8.0)))
