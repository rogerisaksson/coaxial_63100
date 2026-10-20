"""The aerobat's airframe: its numbers (`aerobat.lisp`), its wing, its MuJoCo XML, its drives."""
import math
import os

from machine import aerolisp, airfield, flightpack, pull, quad
from machine.aerobat import form
from machine.aerobatics import HOVER
from machine.wingborne import PITCH_REST, Wing

LISP_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'aerobat.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
#: The card: the routine's, its opening off the runway, its circuit and its final laid lines
#: (`airfield.opening`, `circuit`, `approach`).
_ROUTINE, COOLING = MANIFEST['routines']['aerobat_3d'], DYNAMICS['cooling']
_LIFT, _LAND = ([stage[0] for stage in _ROUTINE].index(name) for name in ('lift', 'land'))
CARD = (_ROUTINE[:_LIFT + 1] + (('knife pass', airfield.opening, 0.0, 0.0, 'knifed'),
                                ('rolling harrier', airfield.rolling_harrier, 0.0, 0.0, 'passed'))
        + _ROUTINE[_LIFT + 1:_LAND] + (('circuit', airfield.circuit, 0.0, 0.0, 'circled'),
                                       ('final', airfield.approach, 0.0, 0.0, 'flared'))
        + _ROUTINE[_LAND:])
_AS, _AF, _DL = AERO['aero_surfaces'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
WING_SPAN, WING_CHORD, WING_AREA = _AS['wing_span'], _AS['wing_chord'], _AS['wing_area']
PROP_R, PROP_Z, TAIL_ARM, STAB_AREA, FIN_AREA = (
    _AS['prop_radius'], _AS['prop_z'], _AS['tail_arm'], _AS['stab_area'], _AS['fin_area'])
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), 0.5 * WING_SPAN
STAB_SPAN = math.sqrt(STAB_AREA * WING_SPAN / WING_CHORD)
#: What touches the runway: the drawn Extra's tail, wing tips and boxes (`form.extents`).
FORM = form.extents(HALF_M / form.HALF)
GEAR_TAIL_Z, GEAR_MAIN_Z = ((z * math.cos(PITCH_REST) + SKID_M) / math.sin(PITCH_REST) for z in STRUT_Z)
IDLE_RAD_S, GOV_RAD_S, VECT_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, VECT_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
ELEV_DEG, AIL_DEG, RUD_DEG = (round(math.degrees(x), 6) for x in (
    _AS['elevator_limit'], _AS['aileron_limit'], _AS['rudder_limit']))
K_THRUST, K_DRAG, CL_ALPHA, CD0, CD_INDUCED = (
    _AS['k_thrust'], _AS['k_drag'], _AS['cl_alpha'], _AS['cd0'], _AS['cd_induced'])
BLADE_PITCH, INFLOW_J0, PROP_INERTIA = _AS['blade_pitch'], _AS['inflow_j0'], _AS['prop_inertia']
K_PITCH, K_ROLL, K_YAW, PROPWASH_SHARE, INFLOW_FLOOR = (
    _AS['k_pitch'], _AS['k_roll'], _AS['k_yaw'], _AS['propwash_share'], _AS['inflow_floor'])
RHO, BODY_CDA = _AF['air_density'], _AF['parasite_cda']
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL = _OBS['turn_bandwidth'], _OBS['pull_bandwidth']
FLOOR_M, HOVER_M = (KINEMATICS['envelope'][k] for k in ('floor_height', 'hover_height'))
WATERFALL_HZ, WATERFALL_RAD = (KINEMATICS['oscillator'][k] for k in (
    'tictoc_frequency', 'tictoc_amplitude'))
DIVE_M_S = 4.0
#: Both boards turn a 17x8 stage on the 63100's can; the propeller's top on the flight's clamp
#: and a full pack's volts, as the page's drives have it.
ROTOR = pull.Rotor(*pull.ROTOR[:4], pull.CAN_J + PROP_INERTIA, pull.ROTOR.b, K_DRAG)
W_TOP = pull.ceiling(flightpack.open_volts(1.0), pull.I_MAX, ROTOR)
#: Its course on all it has: on 1.5 g its climb to the hoop had 0.5 and it struck gate 2 in a
#: gusty air of eight.
PULL_MAX = COURSE_SPHERE = K_THRUST / MASS_KG * W_TOP ** 2
#: The tail in the propeller's wash (`slipstream`): its surfaces' moments a pascal, as quick at
#: the hover as on the head governed at GOV_RAD_S, where they were the propwash's alone.
A_DISC = math.pi * PROP_R ** 2
K_PITCH_Q, K_ROLL_Q = (k * GOV_RAD_S ** 2 * A_DISC / (MASS_KG * GRAVITY) for k in (K_PITCH, K_ROLL))
PROP_BASE, PROP_LOAD, PROP_SURF = _DL['main_base'], _DL['main_collective'], _DL['main_cyclic']
#: The air across the span meets the fuselage's side, its form's (0.159 m^2 at 1.4 m; 0.163
#: flown at 1.6 m), and the fin, m^2: at half the side a knife-edge at 12 m/s flew 39 deg nose-up.
SIDE_AREA = form.side(HALF_M / form.HALF) + FIN_AREA
WING = Wing(MASS_KG, GRAVITY, RHO, WING_AREA, CL_ALPHA, WING_SPAN, STAB_AREA, FIN_AREA, TAIL_ARM, SIDE_AREA)
ROTATE_M_S, RUNWAY_M, RUNWAY_RAD = WING.rotate, WING.runway(HOVER['lean']), airfield.RUNWAY_RAD


def drive_loads(controls, share=1.0):
    """The PROP and VECT drives' loads over their rotor's `drag w^2`, `(prop, vect)`: the single
    tractor propeller's torque at its thrust's `share` of the static and its surfaces, the vector
    drive's by its own."""
    _pitch, elev, ail, rud = controls
    return (min(_DL['main_ceiling'], PROP_BASE + PROP_LOAD * share + PROP_SURF * (elev * elev + ail * ail)),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['tail_pitch'] * (0.5 * share * share + rud * rud)))


def surface_angles(controls):
    """`(elev_deg, ail_deg, rud_deg)` of the 3D elevator, ailerons, and rudder for `controls`."""
    return controls[1] * ELEV_DEG, controls[2] * AIL_DEG, controls[3] * RUD_DEG


def row_from_payload(payload, base=None):
    """An aerobat stunt row of a `CompiledAeroPayload`."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


#: Its wheels' drag on its strip, of their load: on 0.4 and 0.5 it nosed over, its tail up on its
#: take-off's run, in 5 of six tours.
ROLL_MU = 0.3
#: The tail on asphalt, its fin 1 cm past the elevator, softer than the wheels: at their 0.03 s a
#: touch read 37-143 N, 7-27 N m against the ailerons' 5.3.
TAIL_ON = ' priority="1" solref="0.1 1" friction="0.4 0.005 0.0001"'


def mjcf(things=()):
    """The 3D aerobatic monoplane and `things` (`airfield.solids`) as MuJoCo XML: the drawn
    Extra's boxes, its prop's disc, its wheels."""
    boxes = [('fuselage%d' % k, box, '') for k, box in enumerate(FORM['fuselage'])] + [
        ('wing', FORM['wing'], ''), ('hstab', FORM['stab'], TAIL_ON), ('vfin', FORM['fin'], TAIL_ON)]
    m = ''.join('<geom name="%s" type="box" pos="%g %g %g" size="%g %g %g" contype="1" conaffinity="1"%s/>'
                % ((name,) + tuple(box[0]) + tuple(box[1]) + (on,)) for name, box, on in boxes)
    m += ('<geom name="prop" type="cylinder" pos="0 %g 0" size="%g 0.008" euler="90 0 0" contype="1" '
          'conaffinity="1"/>' % (PROP_Z, PROP_R))
    wheels = ((STRUT_Z[0], GEAR_TAIL_Z, 0.45), (STRUT_Z[1], GEAR_MAIN_Z, 1.0))
    for k, (sx, (by, bz, x_sc)) in enumerate((sx, w) for sx in (-SKID_X, SKID_X) for w in wheels):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'priority="1" solref="0.03 1" friction="%g 0.002 0.0001"/>'
              % (k, x_sc * sx, by, bz, FOOT_M, ROLL_MU))
    pos = (-RUNWAY_M * math.sin(RUNWAY_RAD), SKID_M + FOOT_M, -RUNWAY_M * math.cos(RUNWAY_RAD))
    return quad.mjcf(things, m, pos, MASS_KG, INERTIA)


def _prop_drag(share, elev, ail, w):
    """The propeller's torque, N m, at `w` rad/s, its thrust `share` of the static, its surfaces
    at `elev` and `ail`."""
    return (PROP_BASE + PROP_LOAD * share + PROP_SURF * (elev * elev + ail * ail)) * K_DRAG * w * w


def _aero_cda(elev=0.0, ail=0.0):
    cl_eff = 0.12 * CL_ALPHA * PROPWASH_SHARE * math.hypot(elev, ail)
    return BODY_CDA + WING_AREA * (CD0 + CD_INDUCED * cl_eff * cl_eff)


def _aero_inflow(along, v_horiz, w_p):
    """The propeller's thrust's share of the static at `w_p` rad/s, the air `along` its axis and
    `v_horiz` across: none at INFLOW_J0 of its pitch's speed, more edgewise."""
    return (max(0.0, 1.0 - along / (INFLOW_J0 * max(1.0, BLADE_PITCH * abs(w_p) / math.tau)))
            * (1.0 + 0.04 * CL_ALPHA * min(0.35, v_horiz / max(w_p * PROP_R, 8.0))))


def prop_speed(thrust, along, v_horiz):
    """The propeller's speed for `thrust`, N, the air `along` its axis and `v_horiz` across, rad/s:
    the root of `K_THRUST w^2 _aero_inflow` = thrust, edgewise where the static's root has it."""
    c, need = along * math.tau / (INFLOW_J0 * BLADE_PITCH), max(0.0, thrust) / K_THRUST
    w = 0.5 * (c + math.sqrt(c * c + 4.0 * need))
    edge = 1.0 + 0.04 * CL_ALPHA * min(0.35, v_horiz / max(w * PROP_R, 8.0))
    return 0.5 * (c + math.sqrt(c * c + 4.0 * need / edge))


def slipstream(along, thrust):
    """The dynamic pressure on the tail in the propeller's wash, Pa: the air along the nose and
    the thrust over its disc."""
    return 0.5 * RHO * max(0.0, along) ** 2 + max(0.0, thrust) / A_DISC
