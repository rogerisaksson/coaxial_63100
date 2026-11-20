"""Static aerodynamic polars, arena kinematics, composite inertia, and heatsink blocks."""
import math

from machine.body import mechatronics
from machine.solver import aero as solver_aero, inertia as solver_inertia

#: Helmbold 3D finite-wing lift-curve slope, re-exported from `machine.solver.aero`.
lift_slope = solver_aero.lift_slope
#: Foam-carbon wing and tail areal density (kg/m^2) and flapping carbon-membrane areal density (kg/m^2).
WING_SKIN_KG_M2, MEMBRANE_KG_M2 = 0.55, 0.25


def kt(rho, ct, d):
    d_f = float(d)
    d2 = d_f * d_f
    return round(float(ct) * rho * (d2 * d2) / (math.tau * math.tau), 8)


def kq(rho, cp, d):
    d_f = float(d)
    d2 = d_f * d_f
    t = math.tau
    return round(float(cp) * rho * (d2 * d2 * d_f) / (t * t * t), 9)


def layout(rotors):
    """Where `rotors` rotors stand round a frame's middle, in arms: `(x right, z ahead)` each, a square's
    corners for four."""
    return tuple((round(math.sqrt(2.0) * math.sin((2 * k - 1) * math.pi / rotors), 12),
                  round(math.sqrt(2.0) * math.cos((2 * k - 1) * math.pi / rotors), 12))
                 for k in range(rotors))


def rotor_spec(cfg, lib, key='propeller', d_key='diameter_m', p_key='pitch_m'):
    """Resolve propeller or rotor datasheet `cfg[key]` from `lib['propellers']` with inline overrides."""
    base = dict(lib['propellers'][cfg[key]]) if key in cfg else {}
    d = float(cfg.get(d_key, base['diameter_m']))
    p = float(cfg[p_key]) if p_key in cfg else (float(base['pitch_m']) if 'pitch_m' in base else None)
    chord = float(cfg['chord_m']) if 'chord_m' in cfg else (float(base['chord_m']) if 'chord_m' in base else None)
    blades = int(cfg.get('blades', base.get('blades', 2)))
    return {'diameter_m': d, 'pitch_m': p, 'chord_m': chord, 'blades': blades}


def prop_inertia(cfg, mat, lib=None, key='propeller', d_key='diameter_m'):
    """Propeller rotational inertia, kg m^2: explicit override or thin carbon shell (`solver.aero`)."""
    if 'prop_kgm2' in cfg:
        return float(cfg['prop_kgm2'])
    lb = mechatronics.library() if lib is None else lib
    spec = rotor_spec(cfg, lb, key, d_key)
    digits = 5 if d_key == 'diameter_m' else 4
    return round(solver_aero.blade_inertia(
        spec['diameter_m'], mat['cf_kg_m3'], mat['min_wall_m'], blades=spec['blades'],
        chord_m=spec['chord_m']), digits)


def span_efficiency(ae):
    """Whole-airframe span efficiency `e`, derived via `solver.aero.airframe_oswald` unless overridden."""
    if 'span_efficiency' in ae:
        return float(ae['span_efficiency'])
    return solver_aero.airframe_oswald(ae['wing_span_m'], ae['wing_chord_m'], ae['cd0'])


def propwash(ae, prop_d):
    """Slipstream tail lift immersion share, derived via `solver.aero.propwash_share` unless overridden."""
    if 'propwash_share' in ae:
        return float(ae['propwash_share'])
    return solver_aero.propwash_share(prop_d, ae['wing_span_m'], ae['wing_chord_m'], ae['stab_area_m2'])


def _gyr(mass, gyr):
    m = float(mass)
    return tuple(round(m * float(r) * float(r), 6) for r in gyr)


def _craft_inertia(kind, cfg, mat, mot, lib=None):
    """Principal `(ixx, iyy, izz)` at `cfg['mass_kg']` from structural tubes, wings, skids, and drives."""
    if 'gyration_m' in cfg:
        return tuple(round(x, 3) for x in _gyr(cfg['mass_kg'], cfg['gyration_m']))
    lb = mechatronics.library() if lib is None else lib
    m = float(cfg['mass_kg'])
    rho_cf, wall = float(mat['cf_kg_m3']), float(mat['min_wall_m'])
    sx, sh = float(cfg['skid_span_m']), float(cfg['skid_h_m'])
    saft, sfore = float(cfg['strut_aft_m']), float(cfg['strut_fore_m'])
    l_sk, z_sk = sfore - saft, 0.5 * (sfore + saft)
    if kind == 'helicopter':
        tr = rotor_spec(cfg, lb, 'tail_rotor', 'tail_diameter_m')
        l_b, rt = float(cfg['tail_boom_m']), 0.5 * tr['diameter_m']
        h_can = float(mot['height_m'])
        m_tail = 2.0 * solver_aero.blade_inertia(
            2.0 * rt, rho_cf, wall, blades=tr['blades'], chord_m=tr['chord_m']) / max(1e-6, rt * rt) + 0.012
        parts = [
            solver_inertia.tube(0.006, wall, l_b + saft, rho_cf, 'z', (0.0, 0.0, 0.5 * (saft - l_b))),
            solver_inertia.disc(m_tail, rt, 'x', (0.0, 0.0, -l_b)),
            solver_inertia.tube(0.005, wall, l_sk, rho_cf, 'z', (sx, -sh, z_sk)),
            solver_inertia.tube(0.005, wall, l_sk, rho_cf, 'z', (-sx, -sh, z_sk)),
            solver_inertia.tube(0.005, wall, 2.0 * sx, rho_cf, 'x', (0.0, -0.5 * sh, sfore)),
            solver_inertia.tube(0.005, wall, 2.0 * sx, rho_cf, 'x', (0.0, -0.5 * sh, saft)),
        ]
        ixx, iyy, izz = solver_inertia.composite(parts, m, (1.6 * sx, 1.5 * h_can, 0.7 * l_sk), trim_axis=2)
    elif kind == 'ornithopter':
        ws, wc = float(cfg['wing_span_m']), float(cfg['wing_chord_m'])
        shx, shy = float(cfg['shoulder_x_m']), float(cfg['shoulder_y_m'])
        fz, az = float(cfg['fore_z_m']), float(cfg['aft_z_m'])
        m_wing = solver_inertia.tube(0.004, wall, ws, rho_cf, 'x')[0] + MEMBRANE_KG_M2 * ws * wc
        parts = [
            *(solver_inertia.box(m_wing, 0.8 * ws, 0.01, wc, (s * (shx + 0.30 * ws), shy, zw))
              for s in (-1.0, 1.0) for zw in (fz, az)),
            solver_inertia.tube(0.006, wall, ws, rho_cf, 'z', (0.0, 0.0, sfore - 0.5 * ws)),
            solver_inertia.tube(0.005, wall, l_sk, rho_cf, 'z', (sx, -sh, z_sk)),
            solver_inertia.tube(0.005, wall, l_sk, rho_cf, 'z', (-sx, -sh, z_sk)),
        ]
        ixx, iyy, izz = solver_inertia.composite(parts, m, (2.0 * shx, sh, 1.2 * l_sk), trim_axis=2)
    else:
        ws, wc = float(cfg['wing_span_m']), float(cfg['wing_chord_m'])
        pz, ta = float(cfg['prop_z_m']), float(cfg['tail_arm_m'])
        sa, fa = float(cfg['stab_area_m2']), float(cfg['fin_area_m2'])
        stab_b, fin_h = math.sqrt(sa * ws / wc), math.sqrt(2.0 * fa)
        a_prop = rotor_spec(cfg, lb, 'propeller', 'prop_diameter_m', 'prop_pitch_m')
        d_can, h_can, pd = float(mot['can_m']), float(mot['height_m']), a_prop['diameter_m']
        m_prop = prop_inertia(cfg, mat, lb, 'propeller', 'prop_diameter_m') / max(1e-6, 0.05 * pd * pd)
        parts = [
            solver_inertia.box(float(mot['kg']) + m_prop, d_can, h_can, d_can, (0.0, 0.55 * pz, 0.0)),
            solver_inertia.tube(0.005, wall, ws, rho_cf, 'x', (0.0, 0.0, 0.02)),
            solver_inertia.box(WING_SKIN_KG_M2 * ws * wc, 0.80 * ws, wc, 0.03, (0.0, 0.0, 0.02)),
            solver_inertia.tube(0.017, wall, abs(saft), rho_cf, 'y', (0.0, 0.5 * saft, 0.0)),
            solver_inertia.box(WING_SKIN_KG_M2 * (sa + fa), stab_b, 0.20, fin_h, (0.0, -ta, -0.25 * fin_h)),
            solver_inertia.box(0.055, 0.03, 0.06, sh, (sx, sfore, 0.75 * sh)),
            solver_inertia.box(0.055, 0.03, 0.06, sh, (-sx, sfore, 0.75 * sh)),
        ]
        ixx, iyy, izz = solver_inertia.composite(parts, m, (0.14, 0.20, 0.20), trim_axis=1)
    return round(ixx, 3), round(iyy, 3), round(izz, 3)


def arena(crafts, rho, g, lib=None):
    lb = mechatronics.library() if lib is None else lib
    sh, qc, ae = crafts['shared'], crafts['quadcopter'], crafts['aerobat']
    foot_r, q_skid = float(sh['foot_r_m']), float(qc['skid_h_m'])
    q_d = rotor_spec(qc, lb)['diameter_m']
    q_arm, q_m = float(qc['arm_m']), float(qc['mass_kg'])
    r_q = 0.5 * q_d
    a_disc = float(qc['rotors']) * math.pi * r_q * r_q
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
                        / (rho * a_span * a_chord * lift_slope(a_span, a_chord) * math.radians(stall)))
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


def cd_induced(ae):
    """A wing's drag due to lift by its lift coefficient squared, 1 / (pi e AR): `e` the airframe's own span
    efficiency (`span_efficiency`), its fuselage's and its viscous drag due to lift in it."""
    ar = float(ae['wing_span_m']) / float(ae['wing_chord_m'])
    return float(ae.get('cd_induced', round(1.0 / (math.pi * span_efficiency(ae) * ar), 3)))


def _heli_drag_shares(he, inf_fl):
    prof = float(he.get('profile_drag_share', inf_fl))
    ind = float(he.get('induced_drag_share', round(2.0 - prof, 2)))
    ratio = float(he['cyclic_deg']) / float(he['collective_deg'])
    cyc = float(he.get('cyclic_drag_share', round(ratio * ratio, 1)))
    return prof, ind, cyc


def _drive_loads(he, ae, inv, inf_fl, o_cl, o_pulse, a_pw):
    prof, ind, cyc = _heli_drag_shares(he, inf_fl)
    cd0, cdi = float(ae['cd0']), cd_induced(ae)
    wep_ratio = float(inv['wep_amps']) / float(inv['flight_amps'])
    ae_tp = round(1.0 + 1.5 * a_pw * (cd0 + cdi), 2)
    return {
        'swashplate_tail_load': ('drive_load', {
            'main_base': round(1.2 * prof, 2), 'main_collective': ind,
            'main_cyclic': round(1.1 * cyc, 2), 'main_ceiling': round(1.0 + 0.5 * cyc, 2),
            'tail_base': round(prof / math.sqrt(2.0), 2),
            'tail_pitch': round(1.0 + 0.5 * cyc, 2), 'tail_ceiling': 1.0,
        }),
        'ornithopter_wing_load': ('drive_load', {
            'main_base': round(inf_fl + 0.5 * o_cl, 2),
            'main_collective': round(1.0 + 2.0 * o_pulse + 0.5 * o_cl, 2),
            'main_cyclic': round(inf_fl + (2.0 / 3.0) * o_pulse, 2),
            'main_ceiling': round(1.0 + (5.0 / 6.0) * o_pulse, 2),
            'tail_base': round(inf_fl + 0.3 * o_cl, 2),
            'tail_pitch': round(1.0 + o_pulse + 0.3 * o_cl, 2),
            'tail_ceiling': round(1.0 + (2.0 / 3.0) * o_pulse, 2),
        }),
        'aerobat_prop_load': ('drive_load', {
            'main_base': round(inf_fl - 0.3 * cd0, 2),
            'main_collective': round(0.75 * a_pw + 0.3 * cdi, 2),
            'main_cyclic': round(a_pw - 1.5 * cdi, 2),
            'main_ceiling': round(wep_ratio / 0.6, 2),
            'tail_base': round(inf_fl - a_pw * cd0, 2), 'tail_pitch': ae_tp, 'tail_ceiling': ae_tp,
        }),
    }


def aero_blocks(crafts, mat, inv, rho, g=None, lib=None):
    if g is None:
        g = float(mechatronics.environment(None)['g_m_s2'])
    lb = mechatronics.library() if lib is None else lib
    sh = crafts['shared']
    qc, q15 = crafts['quadcopter'], crafts['quadcopter_15in']
    he, or_, ae = crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    j0 = float(sh.get('inflow_j0', solver_aero.ZERO_THRUST_SLIP))
    inf_fl = float(sh.get('inflow_floor', solver_aero.WAKE_INFLOW_FLOOR))

    qp = rotor_spec(qc, lb)
    q_d, q_p, q_arm = qp['diameter_m'], qp['pitch_m'], float(qc['arm_m'])
    q_ct, q_cp = ((float(qc['ct']), float(qc['cp'])) if 'ct' in qc
                  else solver_aero.propeller_bemt(q_d, q_p, blades=qp['blades']))
    q_kt, q_kq = kt(rho, q_ct, q_d), kq(rho, q_cp, q_d)
    q_half = float(qc.get('disc_half_m', solver_aero.disc_half_height(q_d)))
    q_rot = {'rotors': int(qc['rotors']), 'arm_length': q_arm, 'disc_height': float(qc['disc_height_m']),
             'disc_radius': 0.5 * q_d, 'disc_half_height': q_half, 'k_thrust': q_kt,
             'k_drag': q_kq, 'inflow_j0': j0, 'blade_pitch': q_p}

    qp15 = rotor_spec(q15, lb)
    q15_d, q15_p = qp15['diameter_m'], qp15['pitch_m']
    q15_ct, q15_cp = ((float(q15['ct']), float(q15['cp'])) if 'ct' in q15
                      else solver_aero.propeller_bemt(q15_d, q15_p, blades=qp15['blades']))
    q15_rot = {**q_rot, 'disc_radius': 0.5 * q15_d,
               'disc_half_height': solver_aero.disc_half_height(q15_d),
               'blade_pitch': q15_p,
               'k_thrust': kt(rho, q15_ct, q15_d), 'k_drag': kq(rho, q15_cp, q15_d)}

    mr = rotor_spec(he, lb, 'main_rotor')
    tr = rotor_spec(he, lb, 'tail_rotor', 'tail_diameter_m')
    h_d, ht_d = mr['diameter_m'], tr['diameter_m']
    h_r, ht_r = 0.5 * h_d, 0.5 * ht_d
    coll_lim = math.radians(float(he['collective_deg']))
    cyc_lim = math.radians(float(he['cyclic_deg']))
    tail_lim = math.radians(float(he['tail_pitch_deg']))
    h_ct, h_cp = ((float(he['ct']), float(he['cp'])) if 'ct' in he
                  else solver_aero.rotor_bemt(h_d, coll_lim, float(he['main_gear']),
                                              blades=mr['blades'], chord_m=mr['chord_m']))
    ht_ct = (float(he['tail_ct']) if 'tail_ct' in he
             else solver_aero.rotor_bemt(ht_d, tail_lim, float(he['tail_gear']),
                                         step_up=True, blades=tr['blades'], chord_m=tr['chord_m'])[0])
    h_kt, h_kq = round(kt(rho, h_ct, h_d), 7), kq(rho, h_cp, h_d)
    h_kcyc = round(h_kt * 0.5 * math.tan(cyc_lim) * j0 * h_r, 6)
    ht_kt = round(kt(rho, ht_ct, ht_d), 6)
    h_pitch = float(he.get('pitch_m', round(0.75 * math.pi * h_d * math.tan(coll_lim), 4)))

    o_span, o_chord, o_gear = float(or_['wing_span_m']), float(or_['wing_chord_m']), float(or_['gear_ratio'])
    o_stroke = math.radians(float(or_['stroke_deg']))
    o_pitch = math.radians(float(or_['pitch_deg']))
    o_sweep = math.radians(float(or_['sweep_fold_deg']))
    o_arc = o_span * o_stroke
    o_dyn = rho * (4.0 * o_span * o_chord) * (o_arc * o_arc) / (o_gear * o_gear)
    o_ct, o_cp, o_cl = ((float(or_['ct_stroke']), float(or_['cp_stroke']), float(or_['cl_glide']))
                        if 'ct_stroke' in or_
                        else solver_aero.flapping_polar(o_span, o_chord, o_stroke, o_pitch, o_sweep, o_gear))
    o_kt, o_kq = round(o_ct * o_dyn, 7), round(o_cp * o_dyn * o_arc, 9)
    sh_x, fore_z, aft_z = float(or_['shoulder_x_m']), float(or_['fore_z_m']), float(or_['aft_z_m'])
    p_arm = 0.5 * (fore_z - aft_z) * math.cos(o_sweep) * math.cos(o_pitch)
    r_arm = sh_x * math.cos(o_sweep)
    o_pulse = float(or_.get('beat_pulse', solver_aero.flapping_beat_pulse(o_stroke, o_pitch)))

    ap = rotor_spec(ae, lb, 'propeller', 'prop_diameter_m', 'prop_pitch_m')
    a_span, a_chord = float(ae['wing_span_m']), float(ae['wing_chord_m'])
    a_area, a_pd, a_pp = round(a_span * a_chord, 6), ap['diameter_m'], ap['pitch_m']
    a_ct, a_cp = ((float(ae['ct']), float(ae['cp'])) if 'ct' in ae
                  else solver_aero.propeller_bemt(a_pd, a_pp, blades=ap['blades']))
    a_kt, a_kq = round(kt(rho, a_ct, a_pd), 7), kq(rho, a_cp, a_pd)
    a_stab, a_fin, a_arm = float(ae['stab_area_m2']), float(ae['fin_area_m2']), float(ae['tail_arm_m'])
    a_span_eff, a_pw = span_efficiency(ae), propwash(ae, a_pd)
    el_lim = math.radians(float(ae['elevator_deg']))
    ai_lim = math.radians(float(ae['aileron_deg']))
    ru_lim = math.radians(float(ae['rudder_deg']))
    if 'cm_elev' in ae:
        a_kp = round(float(ae['cm_elev']) * a_stab * a_arm, 8)
        a_kr = round(float(ae['cl_ail']) * a_area * 0.25 * a_span, 8)
        a_ky = round(float(ae['cn_rud']) * a_fin * a_arm, 8)
    else:
        w_hover = math.sqrt(float(qc['mass_kg']) * g / (float(qc['rotors']) * q_kt))
        a_mw = 5.0 * round(w_hover * math.sqrt(3.0) / 5.0)
        a_tw = float(round(a_mw * math.sqrt(0.5 * (1.0 + a_pw) * (1.0 - float(ae['cd0'])))))
        a_kp, a_kr, a_ky = solver_aero.control_moments(
            a_span, a_chord, a_stab, a_fin, a_arm, a_pd, ae['prop_z_m'], el_lim, ai_lim, ru_lim,
            a_pw, ae['flap_share'], ae['aileron_share'],
            math.radians(float(ae['vector_deg'])), float(ae['mass_kg']) * g, a_mw, a_tw)
    h_prof, h_ind, h_cyc = _heli_drag_shares(he, inf_fl)
    return {
        'standard_airframe': ('airframe', {'air_density': rho, 'parasite_cda': float(sh['parasite_cda'])}),
        'monoplane_airframe': ('airframe', {'air_density': rho, 'parasite_cda': float(ae['parasite_cda'])}),
        'quad_x_16x14': ('rotors', q_rot),
        'quad_x_15x13': ('rotors', q15_rot),
        'swashplate_550_3d': ('main_rotor', {
            'disc_height': float(he['disc_height_m']), 'radius': h_r,
            'collective_limit': coll_lim,
            'cyclic_limit': cyc_lim, 'k_thrust': h_kt, 'k_cyclic': h_kcyc, 'k_drag': h_kq,
            'profile_drag_share': h_prof, 'induced_drag_share': h_ind, 'cyclic_drag_share': h_cyc,
            'inflow_j0': j0, 'blade_pitch': h_pitch, 'inflow_floor': inf_fl,
        }),
        'tail_110_antitorque': ('tail_rotor', {
            'boom_arm': float(he['tail_boom_m']), 'radius': ht_r,
            'pitch_limit': tail_lim, 'k_thrust': ht_kt,
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
            'k_thrust': a_kt, 'k_drag': a_kq, 'blade_pitch': a_pp,
            'inflow_j0': j0, 'prop_inertia': prop_inertia(ae, mat, lb, 'propeller', 'prop_diameter_m'),
            'cl_alpha': lift_slope(a_span, a_chord), 'cd0': float(ae['cd0']), 'cd_induced': cd_induced(ae),
            'span_efficiency': a_span_eff,
            'elevator_limit': el_lim, 'aileron_limit': ai_lim, 'rudder_limit': ru_lim,
            'k_pitch': a_kp, 'k_roll': a_kr, 'k_yaw': a_ky,
            'propwash_share': a_pw, 'inflow_floor': inf_fl,
        }),
        **_drive_loads(he, ae, inv, inf_fl, o_cl, o_pulse, a_pw),
    }


def body_and_cooling_blocks(crafts, mat, mot, inv, env, g, lib=None):
    lb = mechatronics.library() if lib is None else lib
    sink = inv['heatsink']
    qc, he, or_, ae = crafts['quadcopter'], crafts['helicopter'], crafts['ornithopter'], crafts['aerobat']
    q_n, q_m, q_arm = float(qc['rotors']), float(qc['mass_kg']), float(qc['arm_m'])
    q_c = round(q_m / q_n * q_arm * q_arm, 6)
    qh_c = round(float(qc['heavy_kg']) / q_n * q_arm * q_arm, 6)
    base_rb = {'mass': q_m, 'gravity': g, 'inertia_xx': q_c, 'inertia_yy': 2.0 * q_c, 'inertia_zz': q_c,
               'skid_height': float(qc['skid_h_m']), 'foot_radius': float(crafts['shared']['foot_r_m'])}
    quad_rb = {**base_rb, 'hub_x': float(qc['hub_m'][0]), 'hub_y': float(qc['hub_m'][1]), 'hub_z': float(qc['hub_m'][2])}

    def pod(kind, cfg, frame='mass_kg'):
        """A craft's body on its frame, or on its heavy one (`heavy_kg`): its radii of gyration kept."""
        base_i = _craft_inertia(kind, cfg, mat, mot, lb)
        scale = 1.0 if frame == 'mass_kg' else float(cfg[frame]) / float(cfg['mass_kg'])
        ix, iy, iz = (x if frame == 'mass_kg' else round(x * scale, 6) for x in base_i)
        return {**base_rb, 'mass': float(cfg[frame]), 'inertia_xx': ix, 'inertia_yy': iy, 'inertia_zz': iz,
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
        'quad_x_body_heavy': ('rigid_body', {**quad_rb, 'mass': float(qc['heavy_kg']),
                                             'inertia_xx': qh_c, 'inertia_yy': 2.0 * qh_c, 'inertia_zz': qh_c}),
        'heli_pod_boom_body': ('rigid_body', pod('helicopter', he)),
        'heli_pod_boom_body_heavy': ('rigid_body', pod('helicopter', he, 'heavy_kg')),
        'ornithopter_carapace_body': ('rigid_body', pod('ornithopter', or_)),
        'ornithopter_carapace_body_heavy': ('rigid_body', pod('ornithopter', or_, 'heavy_kg')),
        'aerobat_monoplane_body': ('rigid_body', pod('aerobat', ae)),
        'aerobat_monoplane_body_heavy': ('rigid_body', pod('aerobat', ae, 'heavy_kg')),
        'aluminium_finned_block': ('cooling', alu),
        'temp_controlled_fan': ('cooling', fan),
        'finned_aluminium_fan_sink': ('cooling', {**alu, **fan}),
    }
