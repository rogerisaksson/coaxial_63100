"""Static physical, aerodynamic and cybernetic meta-compiler for aircraft and drones.

    blk = compile.blocks()      # {name: (kind, dict)} for AeroLisp `defvehicle` / `use`
    pk = compile.part('pack')       # 15S LiPo flight pack (`aircraft.flightpack`)
    pl = compile.part('pull')       # outrunner, propeller and inverter shell (`aircraft.pull`)
    ak = compile.part('aerokin')    # vacuum trajectory & attitude handoff (`aircraft.aerokin`)
    wb = compile.part('wingborne')  # stall, rotation and harrier transition (`aircraft.wingborne`)

Reads declarative craft geometry and nondimensional polars (`crafts.toml`), motor/inverter
datasheets (`motors.toml`, `inverters.toml`), heatsink materials (`materials.toml`) and
atmosphere/sensors (`world.toml`), caching the compiled parameterization in `.cache/compiled/`.
"""
import math

from machine.aircraft import polars
from machine.body import mechatronics

_ENV = mechatronics.environment(None)
_mat = mechatronics.materials()
_LIB = mechatronics.library()
_CRAFTS = mechatronics.crafts()
_SHARED = _CRAFTS['shared']
_MOT = _LIB['motors'][_SHARED['motor']]
_INV = _LIB['inverters'][_SHARED['inverter']]
GRAVITY_M_S2 = float(_ENV['g_m_s2'])
ROTOR_MAX_NM = 1.5 * float(_MOT['pole_pairs']) * float(_MOT['flux_vs']) * float(_INV['flight_amps'])
ROTOR_CAN_J = round(float(_mat['rotor_ring_share']) * float(_MOT['kg']) * (0.5 * float(_MOT['can_m'])) ** 2, 7)
MAX_SYS_K = round(0.16 * float(_INV['pwm_hz']), 1)
#: The share of the force vector's turn rate a craft's frame is fed while the vector is pushed down in a
#: descent (`aerokin.FORCE_SHARE`): the flight law's own, a quarter.
FORCE_SHARE = 0.25


def _imp(kp_p, kd_p, kp_r, kd_r, kp_y, kd_y, kr_p=0.0, dr_p=0.0):
    return ('impedance', {
        'stiffness': {'attitude_pitch': kp_p, 'attitude_roll': kp_r, 'attitude_yaw': kp_y,
                      'pitch_rate': kr_p, 'roll_rate': kr_p},
        'damping': {'attitude_pitch': kd_p, 'attitude_roll': kd_r, 'attitude_yaw': kd_y,
                    'pitch_rate': dr_p, 'roll_rate': dr_p},
    })


def _control_and_traj_blocks(crafts, inv, ar, can_j, rot_max_nm, rho, g, amb):
    step_s, vf = round(100.0 / float(inv['pwm_hz']), 6), mechatronics.v_frac(inv)
    qc, he, or_, ae = crafts['quadcopter'], crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    q_kt = polars.kt(rho, qc['ct'], qc['diameter_m'])
    w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
    idle = float(round(w_hover / math.sqrt(6.0)))
    spin_s = round(math.sqrt(0.5) * math.pi * (can_j + float(qc['prop_kgm2'])) * w_hover / rot_max_nm, 2)
    kp_q = float(round(2.0 * vf / (spin_s * spin_s), -1))
    kd_q = float(round(math.sqrt(kp_q)))
    kr_q, dyaw_q = float(round(0.67 * kp_q)), float(round(0.25 * kd_q))

    kp_b = float(round(kp_q * 13.0 / 15.0, -1))
    kd_b = float(round(math.sqrt(1.5 * kp_b)))
    kd_h, kp_yh = float(round(kd_b * 1.1)), float(round(0.54 * kp_b, -1))
    kd_yh = float(round(1.35 * math.sqrt(kp_yh)))
    pulse = float(or_['beat_pulse'])
    kp_o = 5.0 * round(kp_b * (1.0 + 0.06 * pulse) / 5.0)
    kp_yo = 5.0 * round(kp_yh * (1.0 + 0.1 * pulse) / 5.0)
    cdi = polars.cd_induced(ae)
    gyr_ae = float(ae['gyration_m'][2]) / float(ae['gyration_m'][1])
    kp_ap = float(round(kp_b * (1.0 + 0.5 * cdi), -1))
    kp_ar = 5.0 * round(kp_b * (1.0 + 0.5 * cdi * gyr_ae) / 5.0)
    kp_ay = float(round(kp_yh * (1.0 + 0.8 * cdi), -1))

    a_sound = math.sqrt(1.4 * 287.05 * (amb + 273.15))
    h_d, ht_d = float(he['diameter_m']), float(he['tail_diameter_m'])
    h_mw = round(0.65 * a_sound / (0.5 * h_d), 1)
    h_tw = float(round(h_mw * (ht_d / h_d) ** (1.0 / 30.0)))
    gov_3g = 5.0 * round(w_hover * math.sqrt(3.0) / 5.0)
    o_mw = a_mw = gov_3g
    o_tw = float(round(o_mw * (1.0 - 0.5 * (float(or_['fore_z_m']) + float(or_['aft_z_m'])) / float(or_['wing_span_m']))))
    a_tw = float(round(a_mw * math.sqrt(0.5 * (1.0 + float(ae['propwash_share'])) * (1.0 - float(ae['cd0'])))))

    f_step = 1.0 / step_s
    settle_kp = round(1.024 / (2.0 * spin_s), 2)
    orb_r, orb_v = float(ar['orbit_r_m']), float(ar['orbit_m_s'])
    spot_kp = round(0.5 * (1.0 + vf) * float(ar['hover_lean_m_s2']) / (2.0 * orb_r), 2)

    heli_gov = {'idle_speed': idle, 'step_time': step_s, 'main_speed': h_mw, 'tail_speed': h_tw,
                'stunt_boost': 1.12, 'stunt_yaw_threshold': 2.5,
                'tail_ratio': round((h_tw / h_mw) ** 2, 2)}
    std_env = {'floor_height': float(ar['floor_m']), 'hover_height': float(ar['hover_m']),
               'high_height': float(ar['high_m']), 'top_height': float(ar['top_m']),
               'least_pull': float(ar['least_pull']), 'hover_lean': float(ar['hover_lean_m_s2']),
               'sky_lean': float(ar['hover_lean_m_s2'])}
    tt_hz, tt_base = float(ar['tictoc_hz']), float(ar['tictoc_deg'])
    return {
        'base_governor': ('governor', {'idle_speed': idle, 'step_time': step_s}),
        'quad_direct_governor': ('governor', {'idle_speed': idle, 'step_time': step_s, 'spin_time': spin_s}),
        'heli_3d_governor': ('governor', heli_gov),
        'ornithopter_governor': ('governor', {**heli_gov, 'main_speed': o_mw, 'tail_speed': o_tw,
                                              'stunt_boost': 1.14, 'tail_ratio': round((o_tw / o_mw) ** 2, 2)}),
        'aerobat_3d_governor': ('governor', {**heli_gov, 'main_speed': a_mw, 'tail_speed': a_tw,
                                             'stunt_boost': 1.14, 'tail_ratio': round(vf * a_tw / a_mw, 2)}),
        'base_attitude_impedance': _imp(kp_b, kd_b, kp_b, kd_b, 0.0, 0.5 * kd_b),
        'quad_attitude_impedance': _imp(kp_q, kd_q, kp_q, kd_q, 0.0, dyaw_q, kr_q, kd_q),
        'heli_3d_impedance': _imp(kp_b, kd_h, kp_b, kd_h, kp_yh, kd_yh),
        'ornithopter_impedance': _imp(kp_o, kd_h, kp_o, kd_h, kp_yo, kd_yh),
        'aerobat_3d_impedance': _imp(kp_ap, kd_h, kp_ar, kd_h, kp_ay, kd_yh),
        'eso_3d_observers': ('observers', {'turn_bandwidth': round(0.07 * f_step, 1),
                                           'pull_bandwidth': round(0.02 * f_step, 1),
                                           'wind_bandwidth': round(0.01 * f_step, 1),
                                           'wind_regularization': round(20.0 * step_s, 2), 'min_air_speed': 0.2}),
        'standard_arena_envelope': ('envelope', std_env),
        'aerobat_envelope': ('envelope', {**std_env, 'floor_height': ar['ae_floor_m'],
                                          'hover_height': ar['ae_hover_m'], 'high_height': ar['ae_high_m'],
                                          'top_height': ar['ae_top_m']}),
        'vertical_climb_brake': ('vertical', {'climb_speed': float(ar['climb_m_s']),
                                              'descent_speed': float(ar['descent_m_s']),
                                              'wake_floor': 2.0 * float(ar['climb_m_s']),
                                              'settle_kp': settle_kp, 'settle_kd': 2.0 * settle_kp}),
        'orbit_circle': ('horizontal', {'orbit_radius': orb_r, 'orbit_speed': orb_v,
                                        'lap_duration': math.tau * orb_r / orb_v,
                                        'spot_kp': spot_kp, 'spot_kd': round(2.105 * math.sqrt(spot_kp), 2)}),
        'base_tictoc_oscillator': ('oscillator', {'tictoc_frequency': tt_hz,
                                                  'tictoc_amplitude': math.radians(tt_base)}),
        'heli_tictoc_oscillator': ('oscillator', {
            'tictoc_frequency': tt_hz,
            'tictoc_amplitude': math.radians(tt_base + 0.5 * float(he['collective_deg'])),
        }),
        'ornithopter_flap_oscillator': ('oscillator', {
            'tictoc_frequency': round(tt_hz * (1.0 + 0.5 * float(or_['cl_glide'])), 2),
            'tictoc_amplitude': math.radians(tt_base + 0.125 * float(or_['pitch_deg'])),
        }),
        'aerobat_harrier_oscillator': ('oscillator', {
            'tictoc_frequency': tt_hz,
            'tictoc_amplitude': math.radians(tt_base + 0.6 * float(ae['stall_deg'])),
        }),
        'hold_tolerances': ('hold', {'height_tol': float(ar['height_tol_m']), 'speed_tol': float(ar['speed_tol_m_s']),
                                     'spot_tol': float(ar['spot_tol_m']), 'tilt_tol': float(ar['tilt_tol_rad'])}),
    }


def _runtime_configs(env, crafts, mot, inv, ar, can_j, g):
    sh = crafts['shared']
    gm, qc, ae = sh['gimbal'], crafts['quadcopter'], crafts['aerobat']
    fp, ins, w = env['flight_pack'], env['instruments'], env['weather']
    rho, vf, step_s = float(env['air_kg_m3']), mechatronics.v_frac(inv), round(100.0 / float(inv['pwm_hz']), 6)
    q_kt = polars.kt(rho, qc['ct'], qc['diameter_m'])
    w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
    rot_nm = 1.5 * float(mot['pole_pairs']) * float(mot['flux_vs']) * float(inv['flight_amps'])
    spin_s = round(math.sqrt(0.5) * math.pi * (can_j + float(qc['prop_kgm2'])) * w_hover / rot_nm, 2)
    kp_q = float(round(2.0 * vf / (spin_s * spin_s), -1))
    force_a = float(round(0.67 * kp_q, -2))
    force_k = float(round(math.sqrt(0.5 * force_a)))
    v_min = round(float(fp['series']) * float(fp['cell_v'][0][1]) * 0.6)
    v_max = math.ceil(float(inv['volts']) * 1.1 + 0.5)
    w_nl = float(mot['kv']) * (math.tau / 60.0) * v_max * 1.75
    ar_wing = float(ae['wing_span_m']) / float(ae['wing_chord_m'])
    m_ae, orb_v = float(ae['mass_kg']), float(ar['orbit_m_s'])
    kf = float(round(20.0 * m_ae * g, -1))
    n_s, n_p = int(fp['series']), int(fp.get('parallel', 1))
    fp_ah = float(fp.get('ah', round(n_p * float(fp['cell_ah']), 4)))
    fp_ohm = float(fp.get('ohm', round(n_s * float(fp['cell_ohm']) / n_p + float(fp.get('lead_ohm', 0.0)), 6)))
    return {
        'defaults': (float(qc['mass_kg']) * g, orb_v),
        'pack': {'cells': n_s, 'ah': fp_ah, 'ohm': fp_ohm,
                 'reserve': float(fp['reserve']), 'cell_v': tuple((float(a), float(b)) for a, b in fp['cell_v'])},
        'pull': {'rotors': int(qc['rotors']), 'can_j': can_j, 'prop_j': float(qc['prop_kgm2']),
                 'r': float(mot['ohm']), 'lq': float(mot['lq_h']),
                 'lam': float(mot['flux_vs']), 'pairs': float(mot['pole_pairs']), 'b': float(mot['viscous_nms']),
                 'i_max': float(inv['flight_amps']), 'i_wep': float(inv['wep_amps']), 'v_frac': vf,
                 'volts': (float(v_min), float(v_max), 0.5), 'charges': 41, 'over_rad_s': float(round(w_nl, -3)),
                 'halvings': 40, 'w_slew_rad_s': float(round(math.sqrt(2.0) * w_hover, -2))},
        'aerokin': {'nose_k': 1.0, 'pace_s': 1.0, 'home_k': round(orb_v / float(ar['orbit_r_m']), 2),
                    'headroom': round(2.0 * vf - 1.0, 2), 'max_tilt_rad': math.pi / 3.0, 'keep': 0.2,
                    'force_w': float(round(0.8 * force_k)), 'force_a': force_a, 'force_k': force_k,
                    'force_share': FORCE_SHARE},
        'wingborne': {'alpha_rad': math.radians(ar['alpha_run_deg']), 'tail_up': 0.4,
                      'climb_rad': math.radians(ar['climb_deg']),
                      'airborne_m': round(ar['ae_hover_m'] - 0.5 * float(ar['floor_m']), 2), 'run_g': 1.2, 'climb_v': 1.3,
                      'alpha_max': math.radians(float(ae['stall_deg'])),
                      'stall_rad': math.radians(float(ae['stall_deg'])),
                      'stall_blend': math.radians(float(ae['stall_blend_deg'])),
                      'cd_lift': round(1.0 / (math.pi * 0.91 * ar_wing), 2),
                      'pitch_rest': math.radians(float(ae['pitch_rest_deg'])),
                      'harrier_rad': math.radians(ar['harrier_deg']), 'run_ease': 0.15, 'whole': (0.4, 0.6),
                      'grip': float(env['mu_dry']), 'grip_s': round(25.0 * step_s, 3),
                      'wheels_m': round(0.5 * float(ar['floor_m']), 2),
                      'flare_s': round(2.0 * float(ar['descent_m_s']) / g, 1),
                      'touch_m_s': round(float(ar['descent_m_s']) / 8.0, 2)},
        'aerobat': {'dive_m_s': float(ar['climb_m_s']), 'roll_mu': float(ae['roll_mu']),
                    'tail_mu': round(0.5 * float(env['mu_dry']), 2),
                    'tail_s': round(50.0 * step_s, 2), 'wheel_s': round(15.0 * step_s, 2),
                    'touch_m': float(ae['tail_contact_m']), 'fast_m_s': (orb_v, round(0.8 * ar['v_stall'], 1)),
                    'lined_m': (float(ar['top_m']), float(ar['orbit_r_m'])),
                    'lined_rad_s': math.radians(ar['alpha_run_deg']), 'lined_m_s': float(ar['nose_m_s']),
                    'authority': round(math.sqrt(0.5), 1),
                    'gust_slow_s': float(w['kinds']['gusty'][4]),
                    'gust_rms_s': round(2.0 * float(w['eddy_s']), 1),
                    'gust_floor': round(1.5 * float(w['kinds']['constant'][5]), 2),
                    'path_pole': float(sh['inflow_floor']), 'push_s': round(5.0 / (0.01 / step_s), 1)},
        'mpc': {'k_floor': kf, 'c_floor': float(round(math.sqrt(0.5 * m_ae * kf))),
                'skim_m': float(ae['tail_contact_m']), 'touch_m': round(m_ae * g / (2.5 * kf), 2)},
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


def _build_aircraft():
    env, mat, lib, crafts = (mechatronics.environment(None), mechatronics.materials(),
                             mechatronics.library(), mechatronics.crafts())
    sh = crafts['shared']
    mot, inv = lib['motors'][sh['motor']], lib['inverters'][sh['inverter']]
    g, rho, amb = float(env['g_m_s2']), float(env['air_kg_m3']), float(env['ambient_c'])
    ar = polars.arena(crafts, rho, g)
    can_j = round(float(mat['rotor_ring_share']) * float(mot['kg']) * (0.5 * float(mot['can_m'])) ** 2, 7)
    rot_nm = 1.5 * float(mot['pole_pairs']) * float(mot['flux_vs']) * float(inv['flight_amps'])
    blk = {**polars.aero_blocks(crafts, inv, rho), **polars.body_and_cooling_blocks(crafts, mat, inv, env, g),
           **_control_and_traj_blocks(crafts, inv, ar, can_j, rot_nm, rho, g, amb)}
    return {'blocks': blk, **_runtime_configs(env, crafts, mot, inv, ar, can_j, g)}


def _compiled():
    srcs = mechatronics.craft_sources() + (__file__, polars.__file__, mechatronics.__file__)
    return mechatronics.compiled('aircraft', srcs, _build_aircraft)


def defaults() -> tuple[float, float]:
    """Base hover thrust (N) and horizontal orbit speed (m/s) for AeroLisp translation."""
    h, v = _compiled()['defaults']
    return float(h), float(v)


def blocks():
    """Fresh copy of all statically derived AeroLisp vehicle blocks."""
    return {k: (kind, {sk: dict(sv) if isinstance(sv, dict) else sv for sk, sv in val.items()})
            for k, (kind, val) in _compiled()['blocks'].items()}


def part(name):
    """A fresh copy of one compiled part: 'pack', 'pull', 'aerokin', 'wingborne', 'aerobat', 'mpc', 'gimbal' or
    'instruments' - each the module of its name's (`aircraft.se3mpc` the mpc's, `aircraft.senses` the instruments')."""
    return dict(_compiled()[name])


def weather():
    out = dict(_compiled()['weather'])
    out['air'] = dict(out['air'])
    return out
