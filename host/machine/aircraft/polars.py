"""Static aerodynamic polars, arena kinematics, rigid-body inertia and heatsink blocks."""
import math

from machine.body import mechatronics


def kt(rho, ct, d):
    return round(float(ct) * rho * (float(d) ** 4) / (math.tau ** 2), 8)


def kq(rho, cp, d):
    return round(float(cp) * rho * (float(d) ** 5) / (math.tau ** 3), 9)


def _gyr(mass, gyr):
    m = float(mass)
    return tuple(round(m * float(r) ** 2, 6) for r in gyr)


def arena(crafts, rho, g):
    sh, qc, ae = crafts['shared'], crafts['quadcopter'], crafts['aerobat']
    foot_r, q_skid = float(sh['foot_r_m']), float(qc['skid_h_m'])
    q_arm, q_d, q_m = float(qc['arm_m']), float(qc['diameter_m']), float(qc['mass_kg'])
    a_disc = float(qc['rotors']) * math.pi * (0.5 * q_d) ** 2
    v_h = math.sqrt(q_m * g / (2.0 * rho * a_disc))
    floor_m = round(q_skid - foot_r, 1)
    hover_m = round(q_arm, 2)
    top_m = round(5.0 * hover_m, 2)
    climb_m_s = round(v_h, 1)
    gm = sh['gimbal']
    most_deg = float(round(float(gm['tilts_deg'][1]) + float(gm['tilt_deg']), -1))
    lean = float(round(g * math.tan(math.radians(most_deg))))
    nose_m_s = round(0.25 * v_h, 1)
    spot_tol = round(0.5 * q_arm, 2)
    stall = float(ae['stall_deg'])
    rest_rad = math.radians(float(ae['pitch_rest_deg']))
    ae_floor = round(abs(float(ae['strut_aft_m'])) * math.sin(rest_rad)
                     - (float(ae['skid_h_m']) + foot_r) * math.cos(rest_rad), 2)
    a_span, a_chord = float(ae['wing_span_m']), float(ae['wing_chord_m'])
    a_top = float(round(4.0 * a_span + 2.0 * a_chord))
    v_stall = math.sqrt(2.0 * float(ae['mass_kg']) * g
                        / (rho * a_span * a_chord * float(ae['cl_alpha']) * math.radians(stall)))
    return {
        'floor_m': floor_m, 'hover_m': hover_m, 'high_m': round(2.5 * hover_m, 2), 'top_m': top_m,
        'least_pull': round(0.15 * g, 1), 'hover_lean_m_s2': lean,
        'gimbal_most_deg': most_deg, 'nose_m_s': nose_m_s,
        'climb_m_s': climb_m_s, 'descent_m_s': round(0.5 * v_h, 1),
        'orbit_r_m': round(5.0 * q_d, 1), 'orbit_m_s': round(0.75 * v_h, 1),
        'tictoc_hz': round(0.5 * climb_m_s / (top_m + floor_m), 2), 'tictoc_deg': 45.0,
        'height_tol_m': round(2.0 * foot_r, 3), 'speed_tol_m_s': spot_tol,
        'spot_tol_m': spot_tol, 'tilt_tol_rad': round(math.pi / 6.0, 1),
        'ae_floor_m': ae_floor, 'ae_hover_m': round(ae_floor + 0.5 * q_arm, 1),
        'ae_high_m': a_top, 'ae_top_m': a_top,
        'alpha_run_deg': round(2.0 * stall / 3.0, 1), 'climb_deg': round(0.8 * stall, 1),
        'harrier_deg': round(stall + 0.125 * float(ae['stall_blend_deg']), 1), 'v_stall': v_stall,
    }


def cd_induced(ae, env=None):
    e = float((mechatronics.environment(None) if env is None else env)['kinetic_share'])
    ar = float(ae['wing_span_m']) / float(ae['wing_chord_m'])
    return float(ae.get('cd_induced', round(1.0 / (math.pi * e * ar), 3)))


def _heli_drag_shares(he, inf_fl):
    prof = float(he.get('profile_drag_share', inf_fl))
    ind = float(he.get('induced_drag_share', round(2.0 - prof, 2)))
    cyc = float(he.get('cyclic_drag_share', round((float(he['cyclic_deg']) / float(he['collective_deg'])) ** 2, 1)))
    return prof, ind, cyc


def _drive_loads(he, or_, ae, inv, inf_fl, env=None):
    prof, ind, cyc = _heli_drag_shares(he, inf_fl)
    pulse, cl_g = float(or_['beat_pulse']), float(or_['cl_glide'])
    cd0, cdi, pw = float(ae['cd0']), cd_induced(ae, env), float(ae['propwash_share'])
    wep_ratio = float(inv['wep_amps']) / float(inv['flight_amps'])
    ae_tp = round(1.0 + 1.5 * pw * (cd0 + cdi), 2)
    return {
        'swashplate_tail_load': ('drive_load', {
            'main_base': round(1.2 * prof, 2), 'main_collective': ind,
            'main_cyclic': round(1.1 * cyc, 2), 'main_ceiling': round(1.0 + 0.5 * cyc, 2),
            'tail_base': round(prof / math.sqrt(2.0), 2),
            'tail_pitch': round(1.0 + 0.5 * cyc, 2), 'tail_ceiling': 1.0,
        }),
        'ornithopter_wing_load': ('drive_load', {
            'main_base': round(inf_fl + 0.5 * cl_g, 2),
            'main_collective': round(1.0 + 2.0 * pulse + 0.5 * cl_g, 2),
            'main_cyclic': round(inf_fl + (2.0 / 3.0) * pulse, 2),
            'main_ceiling': round(1.0 + (5.0 / 6.0) * pulse, 2),
            'tail_base': round(inf_fl + 0.3 * cl_g, 2),
            'tail_pitch': round(1.0 + pulse + 0.3 * cl_g, 2),
            'tail_ceiling': round(1.0 + (2.0 / 3.0) * pulse, 2),
        }),
        'aerobat_prop_load': ('drive_load', {
            'main_base': round(inf_fl - 0.3 * cd0, 2),
            'main_collective': round(0.75 * pw + 0.3 * cdi, 2),
            'main_cyclic': round(pw - 1.5 * cdi, 2),
            'main_ceiling': round(wep_ratio / 0.6, 2),
            'tail_base': round(inf_fl - pw * cd0, 2), 'tail_pitch': ae_tp, 'tail_ceiling': ae_tp,
        }),
    }


def aero_blocks(crafts, inv, rho, env=None):
    sh = crafts['shared']
    qc, q15 = crafts['quadcopter'], crafts['quadcopter_15in']
    he, or_, ae = crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    q_d, q_arm = float(qc['diameter_m']), float(qc['arm_m'])
    q_kt, q_kq = kt(rho, qc['ct'], q_d), kq(rho, qc['cp'], q_d)
    s15 = float(q15['diameter_m']) / q_d
    j0, inf_fl = float(sh['inflow_j0']), float(sh['inflow_floor'])
    q_rot = {'rotors': int(qc['rotors']), 'arm_length': q_arm, 'disc_height': float(qc['disc_height_m']),
             'disc_radius': 0.5 * q_d, 'disc_half_height': float(qc['disc_half_m']), 'k_thrust': q_kt,
             'k_drag': q_kq, 'inflow_j0': j0, 'blade_pitch': float(qc['pitch_m'])}
    h_d, ht_d = float(he['diameter_m']), float(he['tail_diameter_m'])
    h_r, ht_r = 0.5 * h_d, 0.5 * ht_d
    h_kt, h_kq = round(kt(rho, he['ct'], h_d), 7), kq(rho, he['cp'], h_d)
    cyc_lim = math.radians(float(he['cyclic_deg']))
    h_kcyc = round(h_kt * 0.5 * math.tan(cyc_lim) * j0 * h_r, 6)
    ht_kt = round(kt(rho, he['tail_ct'], ht_d), 6)

    o_span, o_chord, o_gear = float(or_['wing_span_m']), float(or_['wing_chord_m']), float(or_['gear_ratio'])
    o_stroke = math.radians(float(or_['stroke_deg']))
    o_pitch = math.radians(float(or_['pitch_deg']))
    o_sweep = math.radians(float(or_['sweep_fold_deg']))
    o_arc = o_span * o_stroke
    o_dyn = rho * (4.0 * o_span * o_chord) * (o_arc ** 2) / (o_gear ** 2)
    o_kt, o_kq = round(float(or_['ct_stroke']) * o_dyn, 7), round(float(or_['cp_stroke']) * o_dyn * o_arc, 9)
    sh_x, fore_z, aft_z = float(or_['shoulder_x_m']), float(or_['fore_z_m']), float(or_['aft_z_m'])
    p_arm = 0.5 * (fore_z - aft_z) * math.cos(o_sweep) * math.cos(o_pitch)
    r_arm = sh_x * math.cos(o_sweep)

    a_span, a_chord = float(ae['wing_span_m']), float(ae['wing_chord_m'])
    a_area, a_pd = round(a_span * a_chord, 6), float(ae['prop_diameter_m'])
    a_kt, a_kq = round(kt(rho, ae['ct'], a_pd), 7), kq(rho, ae['cp'], a_pd)
    a_stab, a_fin, a_arm = float(ae['stab_area_m2']), float(ae['fin_area_m2']), float(ae['tail_arm_m'])
    h_prof, h_ind, h_cyc = _heli_drag_shares(he, inf_fl)
    o_pulse, o_cl = float(or_['beat_pulse']), float(or_['cl_glide'])
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
            'cyclic_limit': cyc_lim, 'k_thrust': h_kt, 'k_cyclic': h_kcyc, 'k_drag': h_kq,
            'profile_drag_share': h_prof, 'induced_drag_share': h_ind, 'cyclic_drag_share': h_cyc,
            'inflow_j0': j0, 'blade_pitch': float(he['pitch_m']), 'inflow_floor': inf_fl,
        }),
        'tail_110_antitorque': ('tail_rotor', {
            'boom_arm': float(he['tail_boom_m']), 'radius': ht_r,
            'pitch_limit': math.radians(float(he['tail_pitch_deg'])), 'k_thrust': ht_kt,
        }),
        'dune_4wing_flapper': ('flapping_wings', {
            'wing_span': o_span, 'wing_chord': o_chord, 'shoulder_x': sh_x,
            'fore_z': fore_z, 'aft_z': aft_z, 'shoulder_y': float(or_['shoulder_y_m']),
            'stroke_limit': o_stroke, 'pitch_limit': o_pitch, 'sweep_fold_limit': o_sweep,
            'k_thrust': o_kt, 'k_lift': o_cl,
            'k_pitch': round(o_kt * p_arm, 7), 'k_roll': round(o_kt * r_arm, 7),
            'k_yaw': round(o_kt * sh_x, 7), 'k_drag': o_kq,
            'inflow_floor': inf_fl, 'gear_ratio': o_gear, 'beat_pulse': o_pulse,
            'profile_drag_share': round(inf_fl - 0.1 * o_pulse, 2),
            'induced_drag_share': round(1.0 + 1.5 * o_pulse + 0.625 * o_cl, 2),
            'cyclic_drag_share': round(1.6 * o_pulse, 2),
        }),
        'unlimited_3d_monoplane': ('aero_surfaces', {
            'wing_span': a_span, 'wing_chord': a_chord, 'wing_area': a_area,
            'prop_radius': 0.5 * a_pd, 'prop_z': float(ae['prop_z_m']),
            'tail_arm': a_arm, 'stab_area': a_stab, 'fin_area': a_fin,
            'k_thrust': a_kt, 'k_drag': a_kq, 'blade_pitch': float(ae['prop_pitch_m']),
            'inflow_j0': j0, 'prop_inertia': float(ae['prop_kgm2']),
            'cl_alpha': float(ae['cl_alpha']), 'cd0': float(ae['cd0']), 'cd_induced': cd_induced(ae, env),
            'elevator_limit': math.radians(float(ae['elevator_deg'])),
            'aileron_limit': math.radians(float(ae['aileron_deg'])),
            'rudder_limit': math.radians(float(ae['rudder_deg'])),
            'k_pitch': round(float(ae['cm_elev']) * a_stab * a_arm, 8),
            'k_roll': round(float(ae['cl_ail']) * a_area * 0.25 * a_span, 8),
            'k_yaw': round(float(ae['cn_rud']) * a_fin * a_arm, 8),
            'propwash_share': float(ae['propwash_share']), 'inflow_floor': inf_fl,
        }),
        **_drive_loads(he, or_, ae, inv, inf_fl, env),
    }


def body_and_cooling_blocks(crafts, mat, inv, env, g):
    sink = inv['heatsink']
    qc, qh = crafts['quadcopter'], crafts['quadcopter_heavy']
    he, hh = crafts['helicopter'], crafts['helicopter_heavy']
    or_, oh = crafts['ornithopter'], crafts['ornithopter_heavy']
    ae, ah = crafts['aerobat'], crafts['aerobat_heavy']
    q_n, q_m, q_arm = float(qc['rotors']), float(qc['mass_kg']), float(qc['arm_m'])
    q_c = round(q_m / q_n * q_arm * q_arm, 6)
    qh_c = round(float(qh['mass_kg']) / q_n * q_arm * q_arm, 6)
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
    fan_on, fan_full, fan_idle = mechatronics.fan_curve(inv, sink, float(env['ambient_c']))
    fan = {'fan_temp_on': fan_on, 'fan_temp_full': fan_full, 'fan_idle_duty': fan_idle,
           'fan_max_airflow': float(sink['fan_max_airflow']), 'fan_max_rpm': float(sink['fan_max_rpm'])}
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
