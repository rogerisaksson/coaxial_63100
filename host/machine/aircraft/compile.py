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


def _kt(rho, ct, d):
    return round(float(ct) * rho * (float(d) ** 4) / (math.tau ** 2), 8)


def _kq(rho, cp, d):
    return round(float(cp) * rho * (float(d) ** 5) / (math.tau ** 3), 9)


def _gyr(mass, gyr):
    m = float(mass)
    return tuple(round(m * float(r) ** 2, 6) for r in gyr)


def _imp(kp_p, kd_p, kp_r, kd_r, kp_y, kd_y, kr_p=0.0, dr_p=0.0):
    return ('impedance', {
        'stiffness': {'attitude_pitch': kp_p, 'attitude_roll': kp_r, 'attitude_yaw': kp_y,
                      'pitch_rate': kr_p, 'roll_rate': kr_p},
        'damping': {'attitude_pitch': kd_p, 'attitude_roll': kd_r, 'attitude_yaw': kd_y,
                    'pitch_rate': dr_p, 'roll_rate': dr_p},
    })


def _aero_blocks(crafts, rho):
    sh = crafts['shared']
    qc, q15 = crafts['quadcopter'], crafts['quadcopter_15in']
    he, or_, ae = crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    q_d, q_arm = float(qc['diameter_m']), float(qc['arm_m'])
    q_kt, q_kq = _kt(rho, qc['ct'], q_d), _kq(rho, qc['cp'], q_d)
    s15 = float(q15['diameter_m']) / q_d
    j0, inf_fl = float(sh['inflow_j0']), float(sh['inflow_floor'])
    q_rot = {'arm_length': q_arm, 'disc_height': float(qc['disc_height_m']), 'disc_radius': 0.5 * q_d,
             'disc_half_height': float(qc['disc_half_m']), 'k_thrust': q_kt, 'k_drag': q_kq,
             'inflow_j0': j0, 'blade_pitch': float(qc['pitch_m'])}
    h_d, ht_d = float(he['diameter_m']), float(he['tail_diameter_m'])
    h_r, ht_r = 0.5 * h_d, 0.5 * ht_d
    h_kt, h_kq = round(_kt(rho, he['ct'], h_d), 7), _kq(rho, he['cp'], h_d)
    h_kcyc, ht_kt = round(h_kt * float(he['cyclic_share']) * h_r, 6), round(_kt(rho, he['tail_ct'], ht_d), 6)

    o_span, o_chord, o_gear = float(or_['wing_span_m']), float(or_['wing_chord_m']), float(or_['gear_ratio'])
    o_stroke = math.radians(float(or_['stroke_deg']))
    o_arc = o_span * o_stroke
    o_dyn = rho * (4.0 * o_span * o_chord) * (o_arc ** 2) / (o_gear ** 2)
    o_kt, o_kq = round(float(or_['ct_stroke']) * o_dyn, 7), round(float(or_['cp_stroke']) * o_dyn * o_arc, 9)

    a_span, a_chord = float(ae['wing_span_m']), float(ae['wing_chord_m'])
    a_area, a_pd = round(a_span * a_chord, 6), float(ae['prop_diameter_m'])
    a_kt, a_kq = round(_kt(rho, ae['ct'], a_pd), 7), _kq(rho, ae['cp'], a_pd)
    a_stab, a_fin, a_arm = float(ae['stab_area_m2']), float(ae['fin_area_m2']), float(ae['tail_arm_m'])
    return {
        'standard_airframe': ('airframe', {'air_density': rho, 'parasite_cda': float(sh['parasite_cda'])}),
        'monoplane_airframe': ('airframe', {'air_density': rho, 'parasite_cda': float(ae['parasite_cda'])}),
        'quad_x_16x14': ('rotors', q_rot),
        'quad_x_15x13': ('rotors', {**q_rot, 'disc_radius': 0.5 * float(q15['diameter_m']),
                                    'disc_half_height': q_rot['disc_half_height'] * s15,
                                    'blade_pitch': q_rot['blade_pitch'] * s15,
                                    'k_thrust': q_kt * (s15 ** 4), 'k_drag': q_kq * (s15 ** 5)}),
        'swashplate_550_3d': ('main_rotor', {
            'disc_height': float(he['disc_height_m']), 'radius': h_r,
            'collective_limit': math.radians(float(he['collective_deg'])),
            'cyclic_limit': math.radians(float(he['cyclic_deg'])),
            'k_thrust': h_kt, 'k_cyclic': h_kcyc, 'k_drag': h_kq,
            'profile_drag_share': float(he['profile_drag_share']),
            'induced_drag_share': float(he['induced_drag_share']),
            'cyclic_drag_share': float(he['cyclic_drag_share']),
            'inflow_j0': j0, 'blade_pitch': float(he['pitch_m']), 'inflow_floor': inf_fl,
        }),
        'tail_110_antitorque': ('tail_rotor', {
            'boom_arm': float(he['tail_boom_m']), 'radius': ht_r,
            'pitch_limit': math.radians(float(he['tail_pitch_deg'])), 'k_thrust': ht_kt,
        }),
        'swashplate_tail_load': ('drive_load', {k: float(v) for k, v in he['drive_load'].items()}),
        'dune_4wing_flapper': ('flapping_wings', {
            'wing_span': o_span, 'wing_chord': o_chord, 'shoulder_x': float(or_['shoulder_x_m']),
            'fore_z': float(or_['fore_z_m']), 'aft_z': float(or_['aft_z_m']),
            'shoulder_y': float(or_['shoulder_y_m']), 'stroke_limit': o_stroke,
            'pitch_limit': math.radians(float(or_['pitch_deg'])),
            'sweep_fold_limit': math.radians(float(or_['sweep_fold_deg'])),
            'k_thrust': o_kt, 'k_lift': float(or_['cl_glide']),
            'k_pitch': round(o_kt * float(or_['pitch_lever_share']), 7),
            'k_roll': round(o_kt * float(or_['roll_lever_share']), 7),
            'k_yaw': round(o_kt * float(or_['yaw_lever_share']), 7),
            'k_drag': o_kq, 'inflow_floor': inf_fl, 'gear_ratio': o_gear, 'beat_pulse': float(or_['beat_pulse']),
        }),
        'ornithopter_wing_load': ('drive_load', {k: float(v) for k, v in or_['drive_load'].items()}),
        'unlimited_3d_monoplane': ('aero_surfaces', {
            'wing_span': a_span, 'wing_chord': a_chord, 'wing_area': a_area,
            'prop_radius': 0.5 * a_pd, 'prop_z': float(ae['prop_z_m']),
            'tail_arm': a_arm, 'stab_area': a_stab, 'fin_area': a_fin,
            'k_thrust': a_kt, 'k_drag': a_kq, 'blade_pitch': float(ae['prop_pitch_m']),
            'inflow_j0': j0, 'prop_inertia': float(ae['prop_kgm2']),
            'cl_alpha': float(ae['cl_alpha']), 'cd0': float(ae['cd0']), 'cd_induced': float(ae['cd_induced']),
            'elevator_limit': math.radians(float(ae['elevator_deg'])),
            'aileron_limit': math.radians(float(ae['aileron_deg'])),
            'rudder_limit': math.radians(float(ae['rudder_deg'])),
            'k_pitch': round(float(ae['cm_elev']) * a_stab * a_arm, 8),
            'k_roll': round(float(ae['cl_ail']) * a_area * 0.25 * a_span, 8),
            'k_yaw': round(float(ae['cn_rud']) * a_fin * a_arm, 8),
            'propwash_share': float(ae['propwash_share']), 'inflow_floor': inf_fl,
        }),
        'aerobat_prop_load': ('drive_load', {k: float(v) for k, v in ae['drive_load'].items()}),
    }


def _body_and_cooling_blocks(crafts, mat, sink, g):
    qc, qh = crafts['quadcopter'], crafts['quadcopter_heavy']
    he, hh = crafts['helicopter'], crafts['helicopter_heavy']
    or_, oh = crafts['ornithopter'], crafts['ornithopter_heavy']
    ae, ah = crafts['aerobat'], crafts['aerobat_heavy']
    q_m, q_arm = float(qc['mass_kg']), float(qc['arm_m'])
    q_c = round(q_m / 4.0 * q_arm * q_arm, 6)
    qh_c = round(float(qh['mass_kg']) / 4.0 * q_arm * q_arm, 6)
    base_rb = {'mass': q_m, 'gravity': g, 'inertia_xx': q_c, 'inertia_yy': 2.0 * q_c, 'inertia_zz': q_c,
               'skid_height': float(qc['skid_h_m']), 'foot_radius': float(crafts['shared']['foot_r_m'])}
    quad_rb = {**base_rb, 'hub_x': float(qc['hub_m'][0]), 'hub_y': float(qc['hub_m'][1]), 'hub_z': float(qc['hub_m'][2])}

    def pod(cfg, rnd=True):
        ix, iy, iz = (round(x, 3) if rnd else x for x in _gyr(cfg['mass_kg'], cfg['gyration_m']))
        return {**base_rb, 'mass': float(cfg['mass_kg']), 'inertia_xx': ix, 'inertia_yy': iy, 'inertia_zz': iz,
                'skid_height': float(cfg['skid_h_m']), 'skid_span': float(cfg['skid_span_m']),
                'strut_aft': float(cfg['strut_aft_m']), 'strut_fore': float(cfg['strut_fore_m'])}

    sink_m = float(sink['sink_mass_kg'])
    sink_cap = round(sink_m * float(mat['aluminium_housing_j_k_kg']) + float(sink['spreader_j_k']), 1)
    alu = {'sink_mass': sink_m, 'sink_capacity': sink_cap,
           **{k: float(sink[k]) for k in ('r_switches_to_sink', 'r_shunts_to_sink',
                                          'r_sink_to_air_natural', 'r_sink_to_air_forced')}}
    fan = {k: float(sink[k]) for k in ('fan_temp_on', 'fan_temp_full', 'fan_idle_duty',
                                       'fan_max_airflow', 'fan_max_rpm')}
    return {
        'base_airframe_2kg': ('rigid_body', base_rb),
        'quad_x_body': ('rigid_body', quad_rb),
        'quad_x_body_heavy': ('rigid_body', {**quad_rb, 'mass': float(qh['mass_kg']),
                                             'inertia_xx': qh_c, 'inertia_yy': 2.0 * qh_c, 'inertia_zz': qh_c}),
        'heli_pod_boom_body': ('rigid_body', pod(he)),
        'heli_pod_boom_body_heavy': ('rigid_body', pod(hh, False)),
        'ornithopter_carapace_body': ('rigid_body', pod(or_)),
        'ornithopter_carapace_body_heavy': ('rigid_body', pod(oh, False)),
        'aerobat_monoplane_body': ('rigid_body', pod(ae)),
        'aerobat_monoplane_body_heavy': ('rigid_body', pod(ah, False)),
        'aluminium_finned_block': ('cooling', alu),
        'temp_controlled_fan': ('cooling', fan),
        'finned_aluminium_fan_sink': ('cooling', {**alu, **fan}),
    }


def _control_and_traj_blocks(crafts, inv, can_j, rot_max_nm, rho, g):
    sh = crafts['shared']
    ar, step_s = sh['arena'], float(sh['step_s'])
    qc, he, or_, ae = crafts['quadcopter'], crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    q_kt = _kt(rho, qc['ct'], qc['diameter_m'])
    w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
    idle = float(round(w_hover / math.sqrt(6.0)))
    spin_s = round(math.sqrt(0.5) * math.pi * (can_j + float(qc['prop_kgm2'])) * w_hover / rot_max_nm, 2)
    kp_q = float(round(2.0 * float(inv['v_frac']) / (spin_s * spin_s), -1))
    kd_q = float(round(math.sqrt(kp_q)))
    kr_q, dyaw_q = float(round(0.67 * kp_q)), float(round(0.25 * kd_q))

    kp_b = float(round(kp_q * 13.0 / 15.0, -1))
    kd_b = float(round(math.sqrt(1.5 * kp_b)))
    kd_h, kp_yh = float(round(kd_b * 1.1)), float(round(0.54 * kp_b, -1))
    kd_yh = float(round(1.35 * math.sqrt(kp_yh)))

    h_mw = round(float(he['head_tip_m_s']) / (0.5 * float(he['diameter_m'])), 1)
    o_mw, a_mw = float(or_['drive_rad_s']), float(ae['drive_rad_s'])
    f_step = 1.0 / step_s
    settle_kp = round(1.024 / (2.0 * spin_s), 2)
    orb_r, orb_v = float(ar['orbit_r_m']), float(ar['orbit_m_s'])
    spot_kp = round(float(ar['hover_lean_m_s2']) / (2.0 * orb_r) - 0.05, 2)

    heli_gov = {'idle_speed': idle, 'step_time': step_s, 'main_speed': h_mw,
                'tail_speed': float(round(h_mw * float(he['tail_tip_ratio']))),
                'stunt_boost': 1.12, 'stunt_yaw_threshold': 2.5, 'tail_ratio': float(he['tail_drive_ratio'])}
    std_env = {'floor_height': float(ar['floor_m']), 'hover_height': float(ar['hover_m']),
               'high_height': float(ar['high_m']), 'top_height': float(ar['top_m']),
               'least_pull': float(ar['least_pull']), 'hover_lean': float(ar['hover_lean_m_s2']),
               'sky_lean': float(ar['hover_lean_m_s2'])}
    tt_hz = float(ar['tictoc_hz'])
    return {
        'base_governor': ('governor', {'idle_speed': idle, 'step_time': step_s}),
        'quad_direct_governor': ('governor', {'idle_speed': idle, 'step_time': step_s, 'spin_time': spin_s}),
        'heli_3d_governor': ('governor', heli_gov),
        'ornithopter_governor': ('governor', {**heli_gov, 'main_speed': o_mw,
                                              'tail_speed': float(round(o_mw * float(or_['aft_speed_ratio']))),
                                              'stunt_boost': 1.14, 'tail_ratio': float(or_['aft_drive_ratio'])}),
        'aerobat_3d_governor': ('governor', {**heli_gov, 'main_speed': a_mw,
                                             'tail_speed': float(round(a_mw * float(ae['vect_speed_ratio']))),
                                             'stunt_boost': 1.14, 'tail_ratio': float(ae['vect_drive_ratio'])}),
        'base_attitude_impedance': _imp(kp_b, kd_b, kp_b, kd_b, 0.0, 0.5 * kd_b),
        'quad_attitude_impedance': _imp(kp_q, kd_q, kp_q, kd_q, 0.0, dyaw_q, kr_q, kd_q),
        'heli_3d_impedance': _imp(kp_b, kd_h, kp_b, kd_h, kp_yh, kd_yh),
        'ornithopter_impedance': _imp(kp_b + 5.0, kd_h, kp_b + 5.0, kd_h, kp_yh + 5.0, kd_yh),
        'aerobat_3d_impedance': _imp(kp_b + 10.0, kd_h, kp_b + 15.0, kd_h, kp_yh + 10.0, kd_yh),
        'eso_3d_observers': ('observers', {'turn_bandwidth': round(0.07 * f_step, 1),
                                           'pull_bandwidth': round(0.02 * f_step, 1),
                                           'wind_bandwidth': round(0.01 * f_step, 1),
                                           'wind_regularization': round(20.0 * step_s, 2), 'min_air_speed': 0.2}),
        'standard_arena_envelope': ('envelope', std_env),
        'aerobat_envelope': ('envelope', {**std_env, 'floor_height': float(ae['floor_m']),
                                          'hover_height': float(ae['hover_m']), 'high_height': float(ae['high_m']),
                                          'top_height': float(ae['top_m'])}),
        'vertical_climb_brake': ('vertical', {'climb_speed': float(ar['climb_m_s']),
                                              'descent_speed': float(ar['descent_m_s']),
                                              'settle_kp': settle_kp, 'settle_kd': 2.0 * settle_kp}),
        'orbit_circle': ('horizontal', {'orbit_radius': orb_r, 'orbit_speed': orb_v,
                                        'lap_duration': math.tau * orb_r / orb_v,
                                        'spot_kp': spot_kp, 'spot_kd': round(2.105 * math.sqrt(spot_kp), 2)}),
        'base_tictoc_oscillator': ('oscillator', {'tictoc_frequency': tt_hz,
                                                  'tictoc_amplitude': math.radians(float(ar['tictoc_deg']))}),
        'heli_tictoc_oscillator': ('oscillator', {'tictoc_frequency': tt_hz,
                                                  'tictoc_amplitude': math.radians(float(he['tictoc_deg']))}),
        'ornithopter_flap_oscillator': ('oscillator', {'tictoc_frequency': float(or_['tictoc_hz']),
                                                       'tictoc_amplitude': math.radians(float(or_['tictoc_deg']))}),
        'aerobat_harrier_oscillator': ('oscillator', {'tictoc_frequency': tt_hz,
                                                      'tictoc_amplitude': math.radians(float(ae['tictoc_deg']))}),
        'hold_tolerances': ('hold', {'height_tol': float(ar['height_tol_m']), 'speed_tol': float(ar['speed_tol_m_s']),
                                     'spot_tol': float(ar['spot_tol_m']), 'tilt_tol': float(ar['tilt_tol_rad'])}),
    }


def _runtime_configs(env, crafts, mot, inv, can_j, g):
    sh = crafts['shared']
    ar, gm, qc, ae = sh['arena'], sh['gimbal'], crafts['quadcopter'], crafts['aerobat']
    fp, ins, w = env['flight_pack'], env['instruments'], env['weather']
    rho = float(env['air_kg_m3'])
    q_kt = _kt(rho, qc['ct'], qc['diameter_m'])
    w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
    rot_nm = 1.5 * float(mot['pole_pairs']) * float(mot['flux_vs']) * float(inv['flight_amps'])
    spin_s = round(math.sqrt(0.5) * math.pi * (can_j + float(qc['prop_kgm2'])) * w_hover / rot_nm, 2)
    kp_q = float(round(2.0 * float(inv['v_frac']) / (spin_s * spin_s), -1))
    force_a = float(round(1.33 * kp_q, -2))
    force_k = float(round(math.sqrt(0.5 * force_a)))
    v_min = math.floor(float(fp['series']) * float(fp['cell_v'][0][1]) * 0.6)
    v_max = math.ceil(float(inv['volts']) * 1.1 + 0.5)
    w_nl = float(mot['kv']) * (math.tau / 60.0) * v_max * 1.75
    ar_wing = float(ae['wing_span_m']) / float(ae['wing_chord_m'])
    m_ae = float(ae['mass_kg'])
    kf = float(round(20.0 * m_ae * g, -1))
    return {
        'defaults': (float(qc['mass_kg']) * g, float(ar['orbit_m_s'])),
        'pack': {'cells': int(fp['series']), 'ah': float(fp['ah']), 'ohm': float(fp['ohm']),
                 'reserve': float(fp['reserve']), 'cell_v': tuple((float(a), float(b)) for a, b in fp['cell_v'])},
        'pull': {'can_j': can_j, 'prop_j': float(qc['prop_kgm2']), 'r': float(mot['ohm']), 'lq': float(mot['lq_h']),
                 'lam': float(mot['flux_vs']), 'pairs': float(mot['pole_pairs']), 'b': float(mot['viscous_nms']),
                 'i_max': float(inv['flight_amps']), 'i_wep': float(inv['wep_amps']), 'v_frac': float(inv['v_frac']),
                 'volts': (float(v_min), float(v_max), 0.5), 'charges': 41, 'over_rad_s': float(round(w_nl, -3)),
                 'halvings': 40, 'w_slew_rad_s': float(round(math.sqrt(2.0) * w_hover, -2))},
        'aerokin': {'nose_k': 1.0, 'pace_s': 1.0, 'home_k': round(float(ar['orbit_m_s']) / float(ar['orbit_r_m']), 2),
                    'headroom': round(float(inv['v_frac']) - 0.05, 2), 'max_tilt_rad': math.pi / 3.0, 'keep': 0.2,
                    'force_w': float(round(0.8 * force_k)), 'force_a': force_a, 'force_k': force_k,
                    'force_share': round(1.0 - float(env['kinetic_share']), 2)},
        'wingborne': {'alpha_rad': math.radians(float(ae['alpha_run_deg'])), 'tail_up': 0.4,
                      'climb_rad': math.radians(float(ae['climb_deg'])),
                      'airborne_m': round(float(ae['hover_m']) - 0.05, 2), 'run_g': 1.2, 'climb_v': 1.3,
                      'alpha_max': math.radians(float(ae['stall_deg'])),
                      'stall_rad': math.radians(float(ae['stall_deg'])),
                      'stall_blend': math.radians(float(ae['stall_blend_deg'])),
                      'cd_lift': round(1.0 / (math.pi * 0.91 * ar_wing), 2),
                      'pitch_rest': math.radians(float(ae['pitch_rest_deg'])),
                      'harrier_rad': math.radians(float(ae['harrier_deg'])), 'run_ease': 0.15, 'whole': (0.4, 0.6),
                      'grip': float(env['mu_dry']), 'grip_s': round(25.0 * float(sh['step_s']), 3),
                      'wheels_m': round(0.5 * float(ar['floor_m']), 2),
                      'flare_s': round(2.0 * float(ar['descent_m_s']) / g, 1),
                      'touch_m_s': round(float(ar['descent_m_s']) / 8.0, 2)},
        'aerobat': {'dive_m_s': float(ar['climb_m_s']), 'roll_mu': float(ae['roll_mu']),
                    'touch_m': float(ae['tail_contact_m']), 'fast_m_s': (3.0, 7.0), 'lined_m': (1.5, 2.0),
                    'lined_rad_s': math.radians(10.0), 'lined_m_s': 1.0, 'authority': 0.7, 'gust_slow_s': 5.0,
                    'gust_rms_s': 3.0, 'gust_floor': 0.3, 'path_pole': 0.35, 'push_s': 1.0},
        'mpc': {'k_floor': kf, 'c_floor': float(round(math.sqrt(0.5 * m_ae * kf))),
                'skim_m': float(ae['tail_contact_m'])},
        'gimbal': {'mount': tuple(float(x) for x in gm['mount']), 'follow': float(ar['climb_m_s']),
                   'nose_m_s': float(gm['nose_m_s']), 'tilt': math.radians(float(gm['tilt_deg'])),
                   'tilts': tuple(math.radians(float(x)) for x in gm['tilts_deg']),
                   'look': 1.1, 'lean': 0.35, 'most': math.radians(float(gm['most_deg'])),
                   'lean_m_s': float(ar['orbit_m_s'])},
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
                    'shear_most': float(w['shear_most']), 'rise_m': float(w['rise_m']), 'discs': int(w['discs'])},
    }


def _build_aircraft():
    env, mat, lib, crafts = (mechatronics.environment(None), mechatronics.materials(),
                             mechatronics.library(), mechatronics.crafts())
    sh = crafts['shared']
    mot, inv = lib['motors'][sh['motor']], lib['inverters'][sh['inverter']]
    g, rho = float(env['g_m_s2']), float(env['air_kg_m3'])
    can_j = round(float(mat['rotor_ring_share']) * float(mot['kg']) * (0.5 * float(mot['can_m'])) ** 2, 7)
    rot_nm = 1.5 * float(mot['pole_pairs']) * float(mot['flux_vs']) * float(inv['flight_amps'])
    blk = {**_aero_blocks(crafts, rho), **_body_and_cooling_blocks(crafts, mat, inv['heatsink'], g),
           **_control_and_traj_blocks(crafts, inv, can_j, rot_nm, rho, g)}
    return {'blocks': blk, **_runtime_configs(env, crafts, mot, inv, can_j, g)}


def _compiled():
    return mechatronics.compiled('aircraft', mechatronics.craft_sources() + (__file__,), _build_aircraft)


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
