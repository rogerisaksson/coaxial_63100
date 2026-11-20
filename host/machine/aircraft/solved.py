"""A craft's 3-axis attitude, heading and rate loops solved of its own blocks (`solver.loops`).

    solved_axes = solved.solve(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, stunts)
    imp = solved.impedance(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, stunts)

Rotors on arms turn a frame by what each can shed or add of its hover's thrust and drag, a main rotor
and tail rotor by cyclic and anti-torque spare thrust, flapping wings and a monoplane's surfaces by
their moments at the governed speeds less full-load drive reaction torque. Tilt (`pitch`, `roll`) is
one thrust-direction loop at the least of its physical ceilings (`loops.stiffest`); heading (`yaw`) is
a first-order rate loop on multirotors (`loops.rate`) and a second-order attitude loop at the softest
stiffness meeting its card's turn and course cascade needs (`loops.design`) on 3D attitude crafts.
"""
import math

from machine.aircraft import polars
from machine.solver import loops

#: A loop's dead time in its law's steps: one of transport and half a one of hold.
DEAD_STEPS = 1.5
#: A craft's rotational axes, each with its principal inertia's key in `rigid_body` (y is up: heading).
AXES = (('pitch', 'inertia_xx'), ('roll', 'inertia_zz'), ('yaw', 'inertia_yy'))
TILT_AXES = AXES[:2]


def _rotor_top_and_slew(ro, gov, pull_cfg, ref_radius, hover):
    sr = ro['disc_radius'] / ref_radius
    sr2 = sr * sr
    j_tot = pull_cfg['can_j'] + pull_cfg['prop_j'] * sr2 * sr2 * sr
    kt_mot = 1.5 * pull_cfg['pairs'] * pull_cfg['lam']
    b_visc, drag = pull_cfg['b'], ro['k_drag']
    w_held = (math.sqrt(b_visc * b_visc + 4.0 * drag * kt_mot * pull_cfg['i_max']) - b_visc) / (2.0 * drag)
    base = 0.5 * kt_mot * pull_cfg['i_wep'] / j_tot
    least = 0.5 * kt_mot * min(pull_cfg['i_wep'], pull_cfg['i_max']) / j_tot
    frac = min(1.0, max(0.0, (hover - gov['idle_speed']) / max(1e-6, pull_cfg['w_slew_rad_s'] - gov['idle_speed'])))
    return w_held, least + frac * (base - least)


def drive(aero, dynamics, pull_cfg, ref_radius):
    """`{axis: (moment N m, slew_rate N m/s)}` available for control about `pitch`, `roll` and `yaw`."""
    body, gov = dynamics['rigid_body'], dynamics['governor']
    if 'rotors' in aero:
        ro = aero['rotors']
        hover = math.sqrt(body['mass'] * body['gravity'] / (ro['rotors'] * ro['k_thrust']))
        w_held, w_slew = _rotor_top_and_slew(ro, gov, pull_cfg, ref_radius, hover)
        idle = gov['idle_speed']
        spare = ro['k_thrust'] * min(hover * hover - idle * idle, w_held * w_held - hover * hover)
        turn = 2.0 * ro['k_thrust'] * hover * w_slew
        arms = [sum(abs(at[k]) for at in polars.layout(int(ro['rotors']))) * ro['arm_length'] for k in (1, 0)]
        yaw_arm = ro['rotors'] * ro['k_drag'] / ro['k_thrust']
        return {'pitch': (spare * arms[0], turn * arms[0]),
                'roll': (spare * arms[1], turn * arms[1]),
                'yaw': (spare * yaw_arm, turn * yaw_arm)}
    w_m2, w_t2 = gov['main_speed'] * gov['main_speed'], gov['tail_speed'] * gov['tail_speed']
    if 'main_rotor' in aero:
        mr, tr = aero['main_rotor'], aero['tail_rotor']
        cyclic = mr['k_cyclic'] * w_m2
        q_main = (mr['profile_drag_share'] + mr['induced_drag_share']
                  * (1.0 + mr['cyclic_drag_share'])) * mr['k_drag'] * w_m2
        yaw = max(0.0, tr['boom_arm'] * tr['k_thrust'] * w_t2 - q_main)
        return {'pitch': (cyclic, 0.0), 'roll': (cyclic, 0.0), 'yaw': (yaw, 0.0)}
    if 'flapping_wings' in aero:
        fw = aero['flapping_wings']
        w_sq = 0.5 * (w_m2 + w_t2)
        q_main = (fw['profile_drag_share'] + fw['induced_drag_share']
                  * (1.0 + fw['cyclic_drag_share'])) * fw['k_drag'] * w_sq
        yaw = max(0.0, fw['k_yaw'] * w_t2 - q_main)
        return {'pitch': (fw['k_pitch'] * w_m2, 0.0),
                'roll': (fw['k_roll'] * w_m2, 0.0),
                'yaw': (yaw, 0.0)}
    as_, dl = aero['aero_surfaces'], aero['drive_load']
    q_main = (dl['main_base'] + dl['main_collective'] + dl['main_cyclic']) * as_['k_drag'] * w_m2
    yaw = max(0.0, as_['k_yaw'] * w_t2 - q_main)
    return {'pitch': (as_['k_pitch'] * w_m2, 0.0),
            'roll': (as_['k_roll'] * w_m2, 0.0),
            'yaw': (yaw, 0.0)}


def solve(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, stunts=None):
    """`{axis: (Loop, needs, ceilings)}` for `pitch`, `roll` and `yaw` on unit inertia."""
    body, gov = dynamics['rigid_body'], dynamics['governor']
    zeta = float(targets['damping_ratio'])
    mf, cf, ff = float(targets['motion_factor']), float(targets['cascade_factor']), float(targets['fall_factor'])
    delay_rad = math.radians(float(targets['delay_phase_deg']))
    excursion = math.radians(float(targets['excursion_deg']))
    dead_s = DEAD_STEPS * gov['step_time']

    motion = math.tau * kinematics['oscillator']['tictoc_frequency']
    along = math.sqrt(kinematics['horizontal']['spot_kp'])
    tilt_needs = {'motion': loops.stiffness(1.0, mf * motion),
                  'cascade': loops.stiffness(1.0, cf * along)}
    left = loops.margin(zeta) - delay_rad
    dead_stiff = loops.stiffness(1.0, loops.natural(dead_s, zeta, left))
    tilt_ceilings = {'dead time': dead_stiff}
    drv = drive(aero, dynamics, pull_cfg, ref_radius)
    for loop, key in TILT_AXES:
        moment, rate = drv[loop]
        tilt_ceilings[loop + ' authority'] = moment / body[key] / excursion
        if rate:
            tilt_ceilings['rotor slew'] = min(tilt_ceilings.get('rotor slew', math.inf),
                                              loops.stiffness(1.0, rate / moment))
    tilt = loops.stiffest(1.0, zeta, tilt_needs, tilt_ceilings)

    horiz = kinematics['horizontal']
    orb_turn = horiz['orbit_speed'] / horiz['orbit_radius']
    card_turn = max((math.radians(abs(s.get('turn', 0.0))) for s in (stunts or {}).values()), default=orb_turn)
    yaw_turn = max(orb_turn, card_turn)
    yaw_moment, yaw_rate = drv['yaw']
    yaw_auth = yaw_moment / body['inertia_yy'] / excursion
    if 'rotors' in aero:
        yaw_needs = {'motion': mf * yaw_turn, 'cascade': orb_turn}
        yaw_ceilings = {'dead time': loops.rate_natural(dead_s, 0.5 * math.pi - delay_rad),
                        'yaw authority': math.sqrt(yaw_auth)}
        if yaw_rate:
            yaw_ceilings['rotor slew'] = yaw_rate / yaw_moment
        yaw = loops.rate(1.0, yaw_needs, yaw_ceilings)
    else:
        yaw_needs = {'motion': loops.stiffness(1.0, mf * yaw_turn),
                     'cascade': loops.stiffness(1.0, cf * ff * along)}
        yaw_ceilings = {'dead time': dead_stiff, 'yaw authority': yaw_auth}
        yaw = loops.design(1.0, zeta, yaw_needs, yaw_ceilings)

    return {'pitch': (tilt, tilt_needs, tilt_ceilings),
            'roll': (tilt, tilt_needs, tilt_ceilings),
            'yaw': (yaw, yaw_needs, yaw_ceilings)}


def impedance(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, stunts=None):
    """Compiled `{'stiffness': ..., 'damping': ...}` table solved of a vehicle's own blocks."""
    axes = solve(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, stunts)
    kp_p, kd_p = round(axes['pitch'][0].kp, 1), round(axes['pitch'][0].kd, 1)
    kp_r, kd_r = round(axes['roll'][0].kp, 1), round(axes['roll'][0].kd, 1)
    kp_y, kd_y = round(axes['yaw'][0].kp, 1), round(axes['yaw'][0].kd, 1)
    kr, dr = (round(loops.bessel_rate(kd_p), 1), kd_p) if 'rotors' in aero else (0.0, 0.0)
    return {
        'stiffness': {'attitude_pitch': kp_p, 'attitude_roll': kp_r, 'attitude_yaw': kp_y,
                      'pitch_rate': kr, 'roll_rate': kr},
        'damping': {'attitude_pitch': kd_p, 'attitude_roll': kd_r, 'attitude_yaw': kd_y,
                    'pitch_rate': dr, 'roll_rate': dr},
    }
