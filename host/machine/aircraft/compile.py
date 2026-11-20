"""Static physical, aerodynamic and cybernetic meta-compiler for aircraft and drones.

    blk = compile.blocks()          # {name: (kind, dict)} for AeroLisp `defvehicle` / `use`
    pk = compile.part('pack')       # 15S LiPo flight pack (`aircraft.flightpack`)
    pl = compile.part('pull')       # outrunner, propeller and inverter shell (`aircraft.pull`)
    ak = compile.part('aerokin')    # vacuum trajectory & attitude handoff (`aircraft.aerokin`)
    wb = compile.part('wingborne')  # stall, rotation and harrier transition (`aircraft.wingborne`)
    mj = compile.part('mujoco')     # MuJoCo contact impedances, friction arms and airframe geoms

Reads declarative craft geometry (`crafts.toml`), motor/inverter/propeller datasheets (`motors.toml`,
`inverters.toml`, `propellers.toml`), materials (`materials.toml`), control targets (`control.toml`)
and atmosphere/sensors (`world.toml`), solving polars, inertias, loops and dynamic MuJoCo solver
configs via `machine.solver` and caching in `.cache/compiled/`.
"""
import math
import os

from machine.aircraft import polars, solved
from machine.body import mechatronics
from machine.solver import aero as solver_aero, contacts, drivetrain, loops
from motor.constants import AIR_GAMMA, AIR_GAS_CONSTANT, KELVIN_AT_ZERO_C

_ENV = mechatronics.environment(None)
_mat = mechatronics.materials()
_LIB = mechatronics.library()
_CRAFTS = mechatronics.crafts()
_SHARED = _CRAFTS['shared']
_MOT = _LIB['motors'][_SHARED['motor']]
_INV = _LIB['inverters'][_SHARED['inverter']]
GRAVITY_M_S2 = float(_ENV['g_m_s2'])
#: Main-rotor hover tip Mach number for a 550-class collective-pitch 3D helicopter.
HELI_TIP_MACH = 0.65


def _rotor(mot, inv, mat):
    """The crafts' rotor as its motor and its inverter give it: its can's inertia (kg m^2), a ring of the
    motor's mass at its can's radius, and the torque (N m) its flight current holds."""
    r = 0.5 * float(mot['can_m'])
    can_j = round(float(mat['rotor_ring_share']) * float(mot['kg']) * r * r, 7)
    return can_j, drivetrain.peak_torque(mot['pole_pairs'], mot['flux_vs'], inv['flight_amps'])


def _step_s(inv):
    """The world's step, s, decimated from the inverter's switching carrier (`solver.drivetrain`)."""
    return drivetrain.control_rates(inv['pwm_hz'])[1]


def _hover(qc, can_j, prop_j, rot_nm, q_kt, g):
    """The quadcopter hovering: its rotors' speed (rad/s) and how long they take to it at the rotor's
    torque (s) - solved here once, for its blocks and its parts alike."""
    w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
    spin_s = round(math.sqrt(0.5) * math.pi * (can_j + prop_j) * w_hover / rot_nm, 2)
    return w_hover, spin_s


def _pull_cfg(qc, mot, inv, fp, can_j, prop_j, w_hover):
    vf = mechatronics.v_frac(inv)
    v_min = round(float(fp['series']) * float(fp['cell_v'][0][1]) * 0.6)
    v_max = math.ceil(float(inv['volts']) * 1.1 + 0.5)
    w_nl = 3.0 * drivetrain.noload_rad_s(v_max, mot['pole_pairs'], mot['flux_vs'])
    return {'rotors': int(qc['rotors']), 'can_j': can_j, 'prop_j': prop_j,
            'r': float(mot['ohm']), 'lq': float(mot['lq_h']),
            'lam': float(mot['flux_vs']), 'pairs': float(mot['pole_pairs']), 'b': float(mot['viscous_nms']),
            'i_max': float(inv['flight_amps']), 'i_wep': float(inv['wep_amps']), 'v_frac': vf,
            'volts': (float(v_min), float(v_max), 0.5), 'charges': 41, 'over_rad_s': float(round(w_nl, -3)),
            'halvings': 40, 'w_slew_rad_s': float(round(math.sqrt(2.0) * w_hover, -2))}


ROTOR_CAN_J, ROTOR_MAX_NM = _rotor(_MOT, _INV, _mat)
MAX_SYS_K = round(0.16 * float(_INV['pwm_hz']), 1)
#: The share of the force vector's turn rate a craft's frame is fed while the vector is pushed down in a
#: descent (`aerokin.FORCE_SHARE`): the flight law's own, a quarter.
FORCE_SHARE = 0.25


def _orni_mjcf(or_, mat):
    """Derived MuJoCo collision geometry for the 4-wing ornithopter from its airframe dimensions."""
    ws, wc = float(or_['wing_span_m']), float(or_['wing_chord_m'])
    shx, shy = float(or_['shoulder_x_m']), float(or_['shoulder_y_m'])
    fz, az = float(or_['fore_z_m']), float(or_['aft_z_m'])
    saft, sfore = float(or_['strut_aft_m']), float(or_['strut_fore_m'])
    l_sk = sfore - saft
    wing_half = solver_aero.disc_half_height(0.75 * ws)
    boom_r = round(0.5 * float(mat['tubes']['stock'][8][0]) * 1e-3, 4)
    return {
        'carapace_pos': (0.0, round(0.2 * shy, 3), fz),
        'carapace_size': (shx, shy, round(0.8 * l_sk, 3)),
        'fore_pos': (0.0, shy, fz),
        'fore_size': (round(0.85 * ws, 4), wing_half, round(0.5 * wc, 4)),
        'aft_pos': (0.0, round(1.2 * shy, 3), az),
        'aft_size': (round(0.75 * ws, 4), wing_half, round(0.45 * wc, 4)),
        'boom_from': (0.0, round(0.4 * shy, 3), saft),
        'boom_to': (0.0, round(0.4 * shy, 3), round(saft - ws, 3)),
        'boom_r': boom_r,
        'vtail_pos': (0.0, round(0.85 * shy, 3), round(saft - 0.95 * ws, 3)),
        'vtail_size': (round(0.20 * ws, 3), shy, round(0.35 * wc, 3)),
    }


def _control_and_traj_blocks(crafts, inv, ar, hover, amb, aero_blk, body_blk, pull_cfg, ctl, env, lib):
    step_s, vf = _step_s(inv), mechatronics.v_frac(inv)
    he, or_, ae = crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    w_hover, spin_s = hover
    idle = float(round(w_hover / math.sqrt(6.0)))

    a_sound = math.sqrt(AIR_GAMMA * AIR_GAS_CONSTANT * (amb + KELVIN_AT_ZERO_C))
    h_d = polars.rotor_spec(he, lib, 'main_rotor')['diameter_m']
    h_mw = round(HELI_TIP_MACH * a_sound / (0.5 * h_d), 1)
    h_tw = float(round(vf * h_mw))
    gov_3g = 5.0 * round(w_hover * math.sqrt(3.0) / 5.0)
    o_mw = a_mw = gov_3g
    o_tw = float(round(o_mw * (1.0 - 0.5 * (float(or_['fore_z_m']) + float(or_['aft_z_m'])) / float(or_['wing_span_m']))))
    a_pw = float(aero_blk['unlimited_3d_monoplane'][1]['propwash_share'])
    a_tw = float(round(a_mw * math.sqrt(0.5 * (1.0 + a_pw) * (1.0 - float(ae['cd0'])))))

    settle_kp = round(0.5 / spin_s, 2)
    orb_r, orb_v = float(ar['orbit_r_m']), float(ar['orbit_m_s'])
    spot_kp = round(0.5 * (1.0 + vf) * float(ar['hover_lean_m_s2']) / (2.0 * orb_r), 2)

    i_fl, i_wep = float(inv['flight_amps']), float(inv['wep_amps'])
    heli_boost = solver_aero.stunt_boost(i_fl, i_wep, HELI_TIP_MACH)
    sub_boost = solver_aero.stunt_boost(i_fl, i_wep)
    yaw_thresh = round(float(ctl['motion_factor']) * (orb_v / orb_r), 1)

    h_tr = h_tw / h_mw
    o_tr = o_tw / o_mw
    heli_gov = {'idle_speed': idle, 'step_time': step_s, 'main_speed': h_mw, 'tail_speed': h_tw,
                'stunt_boost': heli_boost, 'stunt_yaw_threshold': yaw_thresh,
                'tail_ratio': round(h_tr * h_tr, 2)}
    quad_gov = {'idle_speed': idle, 'step_time': step_s, 'spin_time': spin_s}
    orni_gov = {**heli_gov, 'main_speed': o_mw, 'tail_speed': o_tw,
                'stunt_boost': sub_boost, 'tail_ratio': round(o_tr * o_tr, 2)}
    aero_gov = {**heli_gov, 'main_speed': a_mw, 'tail_speed': a_tw,
                'stunt_boost': sub_boost, 'tail_ratio': round(vf * a_tw / a_mw, 2)}

    std_env = {'floor_height': float(ar['floor_m']), 'hover_height': float(ar['hover_m']),
               'high_height': float(ar['high_m']), 'top_height': float(ar['top_m']),
               'least_pull': float(ar['least_pull']), 'hover_lean': float(ar['hover_lean_m_s2']),
               'sky_lean': float(ar['hover_lean_m_s2'])}
    ae_env = {**std_env, 'floor_height': ar['ae_floor_m'],
              'hover_height': ar['ae_hover_m'], 'high_height': ar['ae_high_m'], 'top_height': ar['ae_top_m']}
    vert_blk = {'climb_speed': float(ar['climb_m_s']), 'descent_speed': float(ar['descent_m_s']),
                'wake_floor': 2.0 * float(ar['climb_m_s']), 'settle_kp': settle_kp, 'settle_kd': 2.0 * settle_kp}
    horiz_blk = {'orbit_radius': orb_r, 'orbit_speed': orb_v, 'lap_duration': math.tau * orb_r / orb_v,
                 'spot_kp': spot_kp, 'spot_kd': round(2.0 * math.sqrt(spot_kp) / vf, 2)}
    hold_blk = {'height_tol': float(ar['height_tol_m']), 'speed_tol': float(ar['speed_tol_m_s']),
                'spot_tol': float(ar['spot_tol_m']), 'tilt_tol': float(ar['tilt_tol_rad'])}
    tt_hz, tt_base = float(ar['tictoc_hz']), float(ar['tictoc_deg'])
    o_cl = float(aero_blk['dune_4wing_flapper'][1]['k_lift'])
    osc_base = {'tictoc_frequency': tt_hz, 'tictoc_amplitude': math.radians(tt_base)}
    osc_heli = {'tictoc_frequency': tt_hz,
                'tictoc_amplitude': math.radians(tt_base + 0.5 * float(he['collective_deg']))}
    osc_orni = {'tictoc_frequency': round(tt_hz * (1.0 + 0.5 * o_cl), 2),
                'tictoc_amplitude': math.radians(tt_base + 0.125 * float(or_['pitch_deg']))}
    osc_aero = {'tictoc_frequency': tt_hz,
                'tictoc_amplitude': math.radians(tt_base + 0.6 * float(ae['stall_deg']))}

    ref_r = aero_blk['quad_x_16x14'][1]['disc_radius']

    def imp_of(ae_names, rb_name, gov_d, env_d, osc_d):
        ae_d = {aero_blk[n][0]: aero_blk[n][1] for n in ae_names}
        dy_d = {'rigid_body': body_blk[rb_name][1], 'governor': gov_d}
        ki_d = {'envelope': env_d, 'vertical': vert_blk, 'horizontal': horiz_blk,
                'oscillator': osc_d, 'hold': hold_blk}
        return ('impedance', solved.impedance(ae_d, dy_d, ki_d, ctl, pull_cfg, ref_r))

    quad_imp = imp_of(('standard_airframe', 'quad_x_16x14'), 'quad_x_body', quad_gov, std_env, osc_base)
    base_imp = imp_of(('standard_airframe', 'quad_x_16x14'), 'base_airframe_2kg', quad_gov, std_env, osc_base)
    heli_imp = imp_of(('standard_airframe', 'swashplate_550_3d', 'tail_110_antitorque', 'swashplate_tail_load'),
                      'heli_pod_boom_body', heli_gov, std_env, osc_heli)
    orni_imp = imp_of(('standard_airframe', 'dune_4wing_flapper', 'ornithopter_wing_load'),
                      'ornithopter_carapace_body', orni_gov, std_env, osc_orni)
    aero_imp = imp_of(('monoplane_airframe', 'unlimited_3d_monoplane', 'aerobat_prop_load'),
                      'aerobat_monoplane_body', aero_gov, ae_env, osc_aero)

    obs_f, ff = float(ctl['observer_factor']), float(ctl['fall_factor'])
    eso_left = loops.margin(1.0) - math.radians(float(ctl['delay_phase_deg']))
    turn_ceil = loops.natural(2.0 * solved.DEAD_STEPS * step_s, 1.0, eso_left)
    w_tilt = math.sqrt(quad_imp[1]['stiffness']['attitude_pitch'])
    turn_bw = round(loops.observer(w_tilt, obs_f, turn_ceil), 1)
    pull_bw = round(loops.observer(float(ar['descent_m_s']) / orb_r, obs_f, turn_ceil), 1)
    wind_bw = round(pull_bw / ff, 1)
    calm_turb = float(env['weather']['kinds']['calm'][5])

    return {
        'base_governor': ('governor', {'idle_speed': idle, 'step_time': step_s}),
        'quad_direct_governor': ('governor', quad_gov),
        'heli_3d_governor': ('governor', heli_gov),
        'ornithopter_governor': ('governor', orni_gov),
        'aerobat_3d_governor': ('governor', aero_gov),
        'base_attitude_impedance': base_imp,
        'quad_attitude_impedance': quad_imp,
        'heli_3d_impedance': heli_imp,
        'ornithopter_impedance': orni_imp,
        'aerobat_3d_impedance': aero_imp,
        'eso_3d_observers': ('observers', {'turn_bandwidth': turn_bw, 'pull_bandwidth': pull_bw,
                                           'wind_bandwidth': wind_bw,
                                           'wind_regularization': round(2.0 * obs_f * step_s, 2),
                                           'min_air_speed': round(2.0 * calm_turb, 2)}),
        'standard_arena_envelope': ('envelope', std_env),
        'aerobat_envelope': ('envelope', ae_env),
        'vertical_climb_brake': ('vertical', vert_blk),
        'orbit_circle': ('horizontal', horiz_blk),
        'base_tictoc_oscillator': ('oscillator', osc_base),
        'heli_tictoc_oscillator': ('oscillator', osc_heli),
        'ornithopter_flap_oscillator': ('oscillator', osc_orni),
        'aerobat_harrier_oscillator': ('oscillator', osc_aero),
        'hold_tolerances': ('hold', hold_blk),
    }


def _runtime_configs(env, crafts, mat, lib, inv, ar, aero_blk, pull_cfg, quad_imp, ctl, g):
    sh = crafts['shared']
    gm, qc, ae = sh['gimbal'], crafts['quadcopter'], crafts['aerobat']
    fp, ins, w = env['flight_pack'], env['instruments'], env['weather']
    vf, step_s = mechatronics.v_frac(inv), _step_s(inv)
    kp_q = quad_imp['stiffness']['attitude_pitch']
    yaw_d_q = quad_imp['damping']['attitude_yaw']
    force_a = float(round((2.0 / 3.0) * kp_q, -2))
    force_k = float(round(math.sqrt(0.5 * force_a)))
    nose_k = round(yaw_d_q / float(ctl['cascade_factor']), 1)
    ar_wing = float(ae['wing_span_m']) / float(ae['wing_chord_m'])
    m_ae, orb_v = float(ae['mass_kg']), float(ar['orbit_m_s'])
    mpc_points = 2
    mpc_sag = round(0.5 * float(ar['floor_m']), 2)
    mpc_zeta = round(float(ctl['damping_ratio']) / math.sqrt(2.0), 1)
    mpc_dt = round(0.5 / float(ins['gnss'][0]), 3)
    kf_raw, _ = contacts.penalty(m_ae, g, mpc_sag, mpc_zeta, mpc_dt, mpc_points)
    kf = float(round(kf_raw, -1))
    _, cf_raw = contacts.penalty(m_ae, g, m_ae * g / kf, mpc_zeta, 0.0, mpc_points)
    skim_m, foot_r = float(ae['tail_contact_m']), float(sh['foot_r_m'])
    mpc_w = contacts.bryson_mpc(mpc_sag, 0.5, math.radians(float(ctl['excursion_deg'])), skim_m)
    n_s, n_p = int(fp['series']), int(fp.get('parallel', 1))
    fp_ah = float(fp.get('ah', round(n_p * float(fp['cell_ah']), 4)))
    fp_ohm = float(fp.get('ohm', round(n_s * float(fp['cell_ohm']) / n_p + float(fp.get('lead_ohm', 0.0)), 6)))

    mu_dry, roll_mu = float(env['mu_dry']), float(ae['roll_mu'])
    tail_mu = round(0.5 * mu_dry, 2)
    floor_solref = round(5.0 * step_s, 4)
    skid_s = contacts.solref_period(0.05 * foot_r, g, floor_solref)
    wheel_s = contacts.solref_period(foot_r * foot_r, g, floor_solref)
    tail_s = contacts.solref_period(0.08 * skim_m, g, floor_solref)
    grip_s = contacts.solref_period(0.02 * skim_m, g, floor_solref)
    skid_arms = contacts.rolling_arms(foot_r, mu_dry, rolling=False)
    wheel_arms = contacts.rolling_arms(foot_r, roll_mu, rolling=True)
    a_pd = polars.rotor_spec(ae, lib, 'propeller', 'prop_diameter_m', 'prop_pitch_m')['diameter_m']
    inf_fl = float(sh.get('inflow_floor', solver_aero.WAKE_INFLOW_FLOOR))
    return {
        'defaults': (float(qc['mass_kg']) * g, orb_v),
        'pack': {'cells': n_s, 'ah': fp_ah, 'ohm': fp_ohm,
                 'reserve': float(fp['reserve']), 'cell_v': tuple((float(a), float(b)) for a, b in fp['cell_v'])},
        'pull': pull_cfg,
        'aerokin': {'nose_k': nose_k, 'pace_s': 1.0, 'home_k': round(orb_v / float(ar['orbit_r_m']), 2),
                    'headroom': round(2.0 * vf - 1.0, 2), 'max_tilt_rad': math.pi / 3.0, 'keep': 0.2,
                    'force_w': float(round(0.8 * force_k)), 'force_a': force_a, 'force_k': force_k,
                    'force_share': FORCE_SHARE},
        'wingborne': {'alpha_rad': math.radians(ar['alpha_run_deg']), 'tail_up': 0.4,
                      'climb_rad': math.radians(ar['climb_deg']),
                      'airborne_m': round(ar['ae_hover_m'] - 0.5 * float(ar['floor_m']), 2), 'run_g': 1.2, 'climb_v': 1.3,
                      'alpha_max': math.radians(float(ae['stall_deg'])),
                      'stall_rad': math.radians(float(ae['stall_deg'])),
                      'stall_blend': math.radians(float(ae['stall_blend_deg'])),
                      'cd_lift': round(1.0 / (math.pi * solver_aero.oswald(ar_wing) * ar_wing), 2),
                      'pitch_rest': math.radians(float(ae['pitch_rest_deg'])),
                      'harrier_rad': math.radians(ar['harrier_deg']), 'run_ease': 0.15, 'whole': (0.4, 0.6),
                      'grip': mu_dry, 'grip_s': grip_s,
                      'wheels_m': mpc_sag,
                      'flare_s': round(2.0 * float(ar['descent_m_s']) / g, 1),
                      'touch_m_s': round(float(ar['descent_m_s']) / 8.0, 2)},
        'aerobat': {'dive_m_s': float(ar['climb_m_s']), 'roll_mu': roll_mu,
                    'tail_mu': tail_mu,
                    'tail_s': tail_s, 'wheel_s': wheel_s,
                    'touch_m': skim_m, 'fast_m_s': (orb_v, round(0.8 * ar['v_stall'], 1)),
                    'lined_m': (float(ar['top_m']), float(ar['orbit_r_m'])),
                    'lined_rad_s': math.radians(ar['alpha_run_deg']), 'lined_m_s': float(ar['nose_m_s']),
                    'authority': round(math.sqrt(0.5), 1),
                    'gust_slow_s': float(w['kinds']['gusty'][4]),
                    'gust_rms_s': round(2.0 * float(w['eddy_s']), 1),
                    'gust_floor': round(1.5 * float(w['kinds']['constant'][5]), 2),
                    'path_pole': inf_fl, 'push_s': round(5.0 / (0.01 / step_s), 1)},
        'mpc': {'k_floor': kf, 'c_floor': float(round(cf_raw)),
                'sag_m': mpc_sag, 'zeta': mpc_zeta, 'dt_s': mpc_dt, 'points': mpc_points,
                'skim_m': skim_m, 'touch_m': round(contacts.touch(m_ae, g, kf, 2.5), 2),
                'weights': mpc_w},
        'mujoco': {'timestep': step_s, 'gravity': g,
                   'skid_solref_s': skid_s, 'skid_mu': mu_dry,
                   'skid_torsion_m': skid_arms[0], 'skid_roll_m': skid_arms[1],
                   'tail_solref_s': tail_s, 'tail_mu': tail_mu,
                   'wheel_solref_s': wheel_s, 'wheel_mu': roll_mu,
                   'wheel_torsion_m': wheel_arms[0], 'wheel_roll_m': wheel_arms[1],
                   'prop_half_m': solver_aero.disc_half_height(a_pd),
                   'rotor_half_m': aero_blk['quad_x_16x14'][1]['disc_half_height'],
                   'ornithopter': _orni_mjcf(crafts['ornithopter'], mat)},
        'gimbal': {'mount': tuple(float(x) for x in gm['mount']), 'follow': float(ar['climb_m_s']),
                   'nose_m_s': float(ar['nose_m_s']), 'tilt': math.radians(float(gm['tilt_deg'])),
                   'tilts': tuple(math.radians(float(x)) for x in gm['tilts_deg']),
                   'look': 1.1, 'lean': 0.35, 'most': math.radians(float(ar['gimbal_most_deg'])),
                   'lean_m_s': orb_v},
        'instruments': {'att_rad': math.radians(float(ins['att_deg'])), 'att_s': float(ins['att_s']),
                        'gyro': math.radians(float(ins['gyro_deg_s_rt_hz'])),
                        'acc': float(ins['acc_ug_rt_hz']) * 1e-6 * g, 'gnss': tuple(float(x) for x in ins['gnss']),
                        'star': (float(ins['star'][0]), float(ins['star'][1]),
                                 math.radians(float(ins['star'][2]) / 3600.0)),
                        'lose_s': float(ins['lose_s']), 'lost_s': tuple(float(x) for x in ins['lost_s']),
                        'keep_s': float(ins['keep_s']), 'gravity': g},
        'weather': {'air': {k: tuple(float(x) for x in v) for k, v in w['kinds'].items()},
                    'kind_s': float(w['kind_s']), 'ease_s': float(w['ease_s']), 'turn_deg': float(w['turn_deg']),
                    'gust_s': tuple(float(x) for x in w['gust_s']),
                    'gust_share': tuple(float(x) for x in w['gust_share']), 'gust_deg': float(w['gust_deg']),
                    'eddy_s': float(w['eddy_s']), 'eddy_disc_s': float(w['eddy_disc_s']),
                    'eddy_disc': float(w['eddy_disc']), 'eddy_up': float(w['eddy_up']),
                    'ref_m': float(w['ref_m']), 'shear': float(w['shear']), 'shear_m': float(w['shear_m']),
                    'shear_most': float(w['shear_most']), 'rise_m': float(w['rise_m']), 'discs': int(qc['rotors'])},
    }


def _build_aircraft(crafts=None):
    """Every craft's blocks and parts compiled of the specs, or of `crafts` in the spec's place: a craft
    may also change the shared motor or inverter."""
    env, mat, lib = mechatronics.environment(None), mechatronics.materials(), mechatronics.library()
    ctl = mechatronics.control()
    crafts = mechatronics.crafts() if crafts is None else crafts
    sh = crafts['shared']
    mot, inv = lib['motors'][sh['motor']], lib['inverters'][sh['inverter']]
    can_j, rot_nm = _rotor(mot, inv, mat)
    rho, g, amb = float(env['air_kg_m3']), float(env['g_m_s2']), float(env['ambient_c'])
    ar = polars.arena(crafts, rho, g, lib)
    aero_blk = polars.aero_blocks(crafts, mat, inv, rho, g, lib)
    body_blk = polars.body_and_cooling_blocks(crafts, mat, mot, inv, env, g, lib)
    prop_j = polars.prop_inertia(crafts['quadcopter'], mat, lib)
    hover = _hover(crafts['quadcopter'], can_j, prop_j, rot_nm, aero_blk['quad_x_16x14'][1]['k_thrust'], g)
    pull_cfg = _pull_cfg(crafts['quadcopter'], mot, inv, env['flight_pack'], can_j, prop_j, hover[0])
    ctrl_blk = _control_and_traj_blocks(crafts, inv, ar, hover, amb, aero_blk, body_blk, pull_cfg, ctl, env, lib)
    blocks = {**aero_blk, **body_blk, **ctrl_blk}
    return {'blocks': blocks, **_runtime_configs(
        env, crafts, mat, lib, inv, ar, aero_blk, pull_cfg, ctrl_blk['quad_attitude_impedance'][1], ctl, g)}


def compile_spec(spec: str | os.PathLike[str] | dict):
    """Compile an external `crafts.toml` path or resolved craft dict, caching file paths by content hash."""
    if isinstance(spec, dict):
        return _build_aircraft(spec)
    p = os.path.abspath(spec)
    srcs = mechatronics.craft_sources(p) + (os.path.abspath(__file__), os.path.abspath(polars.__file__))
    key = 'aircraft_' + mechatronics.cache_id(p)
    return mechatronics.compiled(key, srcs, lambda: _build_aircraft(mechatronics.crafts(p)))


def _compiled():
    srcs = mechatronics.craft_sources() + (os.path.abspath(__file__), os.path.abspath(polars.__file__))
    return mechatronics.compiled('aircraft', srcs, _build_aircraft)


def blocks():
    """Compiled `{block_name: (block_type, dict)}` registry for AeroLisp `defvehicle` / `use`."""
    return {k: (v[0], dict(v[1])) for k, v in _compiled()['blocks'].items()}


def part(name: str):
    """Compiled runtime dictionary `name` (`pack`, `pull`, `aerokin`, `wingborne`, `aerobat`,
    `mpc`, `mujoco`, `gimbal`, `instruments`)."""
    return dict(_compiled()[name])


def weather():
    """Compiled atmospheric boundary-layer wind and gust configuration (`aircraft.weather`)."""
    return dict(_compiled()['weather'])


def defaults():
    """`(hover_thrust_n, nominal_speed_m_s)` default scaling for `AeroLispCompiler`."""
    return tuple(_compiled()['defaults'])
