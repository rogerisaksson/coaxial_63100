"""Static meta-compiler for the robot's fieldbus, thermal SOA, cooling, harness, eyes, world and sysid.

    b = peripherals.bus()         # fieldbus wire timing, SEA observer share and wear window
    t = peripherals.thermal()     # board SOA derating and trip recovery envelope
    c = peripherals.cooling()     # finned aluminium heatsink and fan parameters
    h = peripherals.harness()     # pelvic safety harness stiffness and damping
    e = peripherals.eyes()        # stereo camera optics, SGM penalties and gimbal geometry
    w = peripherals.world()       # contact solver, simulation plant switches and virtual slew
    i = peripherals.identify()    # on-line wear filter, ridge correction, stand campaign and sysid

Reads declarative hardware specifications (`world.toml`, `inverters.toml`, `materials.toml`,
`<robot>.toml`) and caches the compiled parameterization in `.cache/compiled/`, re-running
automatically on first run or whenever any input `.toml` or compiler file changes.
"""
import math
from typing import Any

from machine.body import build, compile as _compile, drives, figure, mechatronics


def _build_peripherals():
    env, rbt, lib = mechatronics.environment(), mechatronics.robot(), mechatronics.library()
    mat = mechatronics.materials()
    amb = float(env['ambient_c'])
    bus_raw = rbt['bus']
    bus_hz, law_div = drives.BUS_HZ, float(drives.LAW_DIV)
    rate_s = (law_div + 2.0) / bus_hz
    thermal_s = round(10.0 / bus_hz, 4)
    walk_hz = _compile.gait()['twist']['walk_hz']
    bus_cfg = {
        'baud': int(bus_raw['baud']), 'turn_s': float(bus_raw['turn_s']),
        'lockstep_s': 1.0 / bus_hz, 'law_div': int(law_div), 'eso_wo': round(0.1 * bus_hz, 1),
        'max_sys_k': round(0.08 * bus_hz * bus_hz, -3),
        'thermal_s': thermal_s,
        'spin_s': round(2.0 / bus_hz, 4), 'stall_s': round(6000.0 * thermal_s, 1),
        'threads_a_limb': 2, 'line_ber': float(bus_raw['line_ber']),
        'accel_s': rate_s, 'rate_s': rate_s,
        'observe_x': round(2.0 + 1.0 / law_div, 1),
        'play_nm': round(0.25 * drives.mesh('left_knee'), 1),
        'wear_window_s': float(math.ceil(4.0 / walk_hz)),
    }

    inv_k_name = mechatronics.primary_inverter(rbt, lib)
    inv_k = lib['inverters'][inv_k_name]
    t_sw, t_lam, tj_max = float(inv_k['ceiling_c'][0]), float(inv_k['ceiling_c'][1]), float(inv_k['tj_max_c'])
    sw_jk = float(inv_k['switch_j_k'])
    lam_jk, into_kw = mechatronics.laminate_j_k(inv_k, mat), mechatronics.into_k_w(inv_k, mat)
    tau_sw = sw_jk * into_kw
    tau_lam = lam_jk * drives.INVERTERS[inv_k_name][3]
    margin = round((t_lam - amb) / (t_sw - amb + (t_sw - t_lam) / math.sqrt(2.0)), 2)
    trip_rec_s = round(10.0 * math.sqrt(2.0) * tau_lam, -2)
    thermal_cfg = {
        'haste': round(0.1 / thermal_s, 1),
        'throttle_at': round(1.0 - (t_sw - t_lam) / (tj_max + amb), 2),
        'lookahead_s': round(math.sqrt(2.0) * tau_sw, 1),
        'recover_per_s': round(1.0 / (lam_jk + into_kw), 2),
        'trip_margin': margin, 'trip_recover_per_s': (1.0 - margin) / trip_rec_s,
    }

    inv_air = lib['inverters'][mechatronics.crafts()['shared']['inverter']]
    hs = inv_air['heatsink']
    sink_m = float(hs['sink_mass_kg'])
    sink_cap = round(sink_m * float(mat['aluminium_housing_j_k_kg']) + float(hs['spreader_j_k']), 1)
    fan_on, fan_full, fan_idle = mechatronics.fan_curve(inv_air, hs, amb)
    cooling_cfg = {
        'sink_mass': sink_m, 'sink_capacity': sink_cap,
        'switch_pkg_j_k': float(hs['switch_pkg_j_k']), 'shunt_pkg_j_k': float(hs['shunt_pkg_j_k']),
        'r_switches_to_sink': float(hs['r_switches_to_sink']),
        'r_shunts_to_sink': float(hs['r_shunts_to_sink']),
        'r_sink_to_air_natural': float(hs['r_sink_to_air_natural']),
        'r_sink_to_air_forced': float(hs['r_sink_to_air_forced']),
        'fan_temp_on': fan_on, 'fan_temp_full': fan_full,
        'fan_idle_duty': fan_idle, 'fan_max_airflow': float(hs['fan_max_airflow']),
        'fan_max_rpm': float(hs['fan_max_rpm']), 'ram_air_share': float(hs['ram_air_share']),
    }

    st = _compile.subtrees()
    m_dressed = st['weight'] / _compile.G_M_S2
    j_sprung = st['trunk'][2] + 2.0 * st['leg'][2]
    zeta, w_h = round(math.sqrt(0.5), 1), math.sqrt(12.0) * st['omega']
    kp_p = float(round(m_dressed * w_h * w_h, -3))
    kd_p = float(round(2.0 * zeta * math.sqrt(kp_p * m_dressed), -2))
    kp_t = float(round(j_sprung * w_h * w_h, -2))
    kd_t = float(round(2.0 * zeta * math.sqrt(kp_t * j_sprung), -1))

    ey = rbt['eyes']
    w_px, h_px = int(ey['resolution'][0]), int(ey['resolution'][1])
    ch = 2 * int(math.sqrt(64.0 * h_px / w_px) // 2) + 1
    cw = 2 * int((65 // ch) // 2) + 1
    n_bits = cw * ch - 1
    p1 = int(round(n_bits / 3.0, -1))
    eyes_cfg = {
        'w': w_px, 'h': h_px,
        'fovy_deg': float(ey['fovy_deg']), 'baseline_m': float(ey['baseline_m']),
        'left_at': tuple(float(x) for x in ey['left_at']), 'down_deg': float(ey['down_deg']),
        'squint': {s: tuple(float(x) for x in v) for s, v in ey['squint_deg'].items()},
        'rods': {s: tuple(float(x) for x in v) for s, v in ey['rods'].items()},
        'play': {s: tuple(float(x) for x in v) for s, v in ey['play_deg'].items()},
        'lever': {s: float(v) for s, v in ey['lever_m'].items()},
        'church': {s: float(v) for s, v in ey['church'].items()},
        'census': (cw, ch), 'window': cw, 'quantum': 1,
        'p1': p1, 'p2': 12 * p1, 'texture': 1.5, 'unique': round(1.0 + 3.0 / p1, 2),
    }

    stacks = tuple(rbt['stacks'].values())
    pwm_hz = max(float(lib['inverters'][s[2]]['pwm_hz']) for s in stacks)
    min_eff = min(float(lib['gearboxes'][s[1]]['efficiency']) for s in stacks)
    min_ceil = min(float(lib['inverters'][s[2]]['ceiling_c'][0]) for s in stacks)
    p_air_mpa = float(env['air_kg_m3']) * 287.05 * (amb + 273.15) * 1e-6
    sole, pad, lace = rbt['elastomers']['sole'], rbt['elastomers']['pad'], rbt['garments']['lace']
    e_sole, h_sole = float(sole['modulus_mpa']), float(sole['thick_m'])
    a_sole = 2.0 * figure.CONTACTS[0][2][0] * (figure.BALL + figure.HEEL)
    k_sole = (e_sole * 1e6) * a_sole / h_sole
    e_pad, h_pad = float(pad['modulus_mpa']), figure.PAD_M
    r_pad = sum(p[4] for p in figure.PADS) / len(figure.PADS)
    k_pad = (e_pad * 1e6) * (1.0 + r_pad / h_pad) * (math.tau * r_pad * h_pad) / h_pad
    k_lace = float(lace['break_n']) * figure.SCALE / (float(lace['stretch']) * float(lace['length_m']))
    solref_floor = round(5.0 / bus_hz, 4)
    world_cfg = {
        'step_s': 1.0 / bus_hz,
        'substeps': max(1, int(round(pwm_hz / (10.0 * bus_hz)))),
        'cone': 'elliptic',
        'impratio': round(1.0 / (1.0 - min_eff), 1),
        'stand_m': round(1.5 * figure.standing() / figure.SCALE, 1) * figure.SCALE,
        'stand_solref_s': solref_floor,
        'sole_s': max(solref_floor, round(math.tau * math.sqrt(m_dressed / k_sole), 2)),
        'sole_damp': round(2.0 * float(sole['loss']), 2),
        'sole_soft': round(e_sole / (e_sole + p_air_mpa), 1),
        'sole_width_m': h_sole,
        'pad_s': max(solref_floor, round(math.tau * math.sqrt((m_dressed / (2 * len(figure.PADS))) / k_pad), 2)),
        'pad_damp': round(2.0 * float(pad['loss']), 2),
        'pad_soft': round(e_pad / (e_pad + p_air_mpa), 1),
        'lace_s': max(solref_floor, round(math.tau * math.sqrt(0.5 * st['unsprung_kg'] / k_lace), 2)),
        'touched_n': float(round(st['foot'][0] * _compile.G_M_S2)),
        'reflected': 1.0, 'placed': 1.0, 'backdrive': 1.0, 'skeleton': 0.0, 'wound': 1.0,
        'boxed': 0.0, 'room': 0.0, 'camera': 0.0, 'sight': 0.0, 'envelope': 1.0,
        'soa_rds': float(round(pwm_hz / bus_hz)),
        'warm_c': round(amb + 0.75 * (min_ceil - amb), 1),
        'virtual_span_deg': float(round(max(hi - lo for lo, hi in drives.TRAVEL.values()) * 0.9, -1)),
        'virtual_slew_deg_s': float(round(drives.speed('left_shoulder'), -2)),
    }
    return {'bus': bus_cfg, 'thermal': thermal_cfg, 'cooling': cooling_cfg,
            'harness': (kp_p, kd_p, kp_t, kd_t), 'eyes': eyes_cfg, 'world': world_cfg}


def _compiled():
    srcs = (mechatronics.robot_sources() + mechatronics.craft_sources()
            + (__file__, mechatronics.__file__, _compile.__file__, drives.__file__, figure.__file__, build.__file__))
    return mechatronics.compiled('peripherals_' + mechatronics.ROBOT, srcs, _build_peripherals)


def bus():
    """Fieldbus wire timing, SEA observer bandwidth and wear window parameters."""
    return dict(_compiled()['bus'])


def thermal():
    """Board SOA derating and trip recovery envelope derived from inverter RC network."""
    return dict(_compiled()['thermal'])


def cooling():
    """Finned aluminium heatsink and fan parameters derived from flight thermal budget."""
    return dict(_compiled()['cooling'])


def harness():
    """Overhead pelvic safety harness stiffness and damping derived from sprung body inertia."""
    return tuple(_compiled()['harness'])


def eyes() -> dict[str, Any]:
    """Stereo camera resolution, optics, SGM penalties and gimbal geometry (`[eyes]`)."""
    return dict(_compiled()['eyes'])


def world() -> dict[str, Any]:
    """Contact solver parameters, HIL plant switches and virtual slew derived from body and bus."""
    return dict(_compiled()['world'])


def _service() -> dict[str, Any]:
    from machine.body import heat
    env, rbt, lib = mechatronics.environment(), mechatronics.robot(), mechatronics.library()
    stacks = tuple(rbt['stacks'].values())
    eff = min(float(lib['gearboxes'][s[1]]['efficiency']) for s in stacks)
    fatigue_cycles = min(float(lib['gearboxes'][s[1]]['l10_cycles']) for s in stacks)
    max_bl = max(float(lib['gearboxes'][s[1]]['backlash_deg']) for s in stacks)
    loss_band = round(5.0 * (1.0 - eff), 2)
    mesh_arm = max(0.2, round(drives.mesh('left_shoulder'), 1))
    pose, origin = figure.stand(), (0.0, figure.standing(), 0.0)
    ext = figure.frames(dict(pose, left_shoulder=90.0, left_elbow=0.0, left_wrist=0.0), origin, figure.IDENT)
    segs = build.segments()
    arm_names = ['left_upper_arm', 'left_forearm', 'left_hand', 'left_fingers']
    m_ar, r_ar, _ = _compile.crb(segs, ext, arm_names, ext['left_upper_arm'][0], (1.0, 0.0, 0.0))
    l_arm = figure.UPPER_ARM + figure.FOREARM + abs(figure.FIST_AT[1])
    tau_cont = heat.held('left_shoulder', drives.PACK_V)[0]
    load_kg = max(0.5, float(round((tau_cont - m_ar * _compile.G_M_S2 * r_ar) / (_compile.G_M_S2 * l_arm))))
    walk_hz = _compile.gait()['twist']['walk_hz']
    win_s = float(math.ceil(4.0 / walk_hz))
    trend_h = round(win_s * round(3600.0 / win_s) / 3600.0, 1)
    return {
        'friction': [loss_band, round(0.5 * mesh_arm, 2)],
        'viscous': [loss_band, float(env['joint_damping'])],
        'torque': round(1.5 * (1.0 - eff), 2),
        'play': round(1.0 - drives.RIGID_PLAY_DEG / max_bl, 2),
        'wind_up': loss_band,
        'zero': round(math.radians(max_bl) + _compile.ENC_RAD, 2),
        'load_kg': load_kg,
        'hours': round(fatigue_cycles / (3600.0 * walk_hz), -2),
        'trend_h': trend_h,
    }


def service(overrides=None) -> dict[str, Any]:
    """Service wear alarm bands and maintenance interval from gearbox, encoder and thermal limits."""
    srcs = (mechatronics.robot_sources()
            + (__file__, _compile.__file__, drives.__file__, figure.__file__, build.__file__))
    out = dict(mechatronics.compiled('service_' + mechatronics.ROBOT, srcs, _service))
    if overrides:
        out.update(overrides)
    return out


def _identify() -> dict[str, Any]:
    env, rbt, lib = mechatronics.environment(), mechatronics.robot(), mechatronics.library()
    segs, svc, b_cfg = build.segments(), service(), bus()
    stacks = tuple(rbt['stacks'].values())
    eff = min(float(lib['gearboxes'][s[1]]['efficiency']) for s in stacks)
    mesh_rad_s = min(float(lib['gearboxes'][s[1]]['mesh_rad_s']) for s in stacks)
    max_bl = max(float(lib['gearboxes'][s[1]]['backlash_deg']) for s in stacks)
    zeta = drives.zeta('neck')
    bus_hz = round(1.0 / float(b_cfg['lockstep_s']), 1)
    law_div, win_s = int(b_cfg['law_div']), float(b_cfg['wear_window_s'])
    mesh_arm = max(0.2, round(drives.mesh('left_shoulder'), 1))

    m_u = max(0.02, round(segs['left_forearm'][0], 1))
    h_u = max(0.004, round(m_u * 2.0 * (abs(figure.FIST_AT[1]) + figure.FIST_R), 3))
    f_u = max(0.01, round(0.1 * mesh_arm, 2))
    b_u = max(0.01, round(0.5 * f_u, 2))
    kt_u = max(0.005, round(0.1 * (1.0 - eff), 3))
    j_u = max(0.002, round(float(svc['torque']) * drives.armature('neck'), 2))

    f_rotor = math.sqrt(drives.wind('neck') / drives.armature('neck')) / math.tau
    smooth_hz = max(2.0, float(round(zeta * f_rotor)))
    every = max(1, int(round(bus_hz / (10.0 * smooth_hz))))
    sample_hz = bus_hz / every
    rows_at = max(10, int(round(sample_hz)))

    sigmas = 2.0
    prior = float(every)
    load_pace = round(0.5 * prior / math.sqrt(win_s), 2)
    drift_pace = round(sigmas / math.sqrt(law_div * win_s), 2)
    inside = round(1.0 - float(svc['torque']), 2)
    wound_rows = max(5, int(round(0.25 * sample_hz)))
    df = round(1.0 / (every * smooth_hz), 4)
    tones = (tuple(round(p * df, 4) for p in (23, 61, 137)),
             tuple(round(p * df, 4) for p in (31, 79, 113)))
    free_rad = max(0.2, round(math.radians(abs(float(rbt['stops']['spine'][0]))), 1))
    margin = round(0.5 * (1.0 - eff), 3)
    swing = round(0.5 * (1.0 - float(svc['torque']) - 3.0 * margin), 2)
    l_hem = max(0.04, abs(figure.FIST_AT[1]) + 0.01)
    return {
        'wear': {
            'unit': {'m': m_u, 'hx': h_u, 'hy': h_u, 'hz': h_u, 'f': f_u, 'b': b_u, 'kt': kt_u},
            'pace': {'m': load_pace, 'hx': load_pace, 'hy': load_pace, 'hz': load_pace,
                     'f': drift_pace, 'b': drift_pace, 'kt': drift_pace},
            'prior': prior, 'noise': float(svc['friction'][1]), 'sigmas': sigmas,
            'every': every, 'window_s': win_s, 'box_windows': law_div,
            'hung_kg': round(0.5 * m_u, 3), 'alone_share': round(win_s / (win_s + 1.0), 2),
        },
        'identified': {
            'units': {'m': m_u, 'hx': h_u, 'hy': h_u, 'hz': h_u, 'J': j_u, 'b': b_u, 'f': f_u},
            'ridge': zeta, 'kept': round(1.0 - float(env['kinetic_share']), 2),
        },
        'campaign': {
            'wound_nm': mesh_arm, 'wound_rows': wound_rows, 'smooth_hz': smooth_hz,
            'inside': inside, 'rotor_rows': wound_rows, 'rotor_nm': round(0.5 * mesh_arm, 2),
            'tones': tones, 'swing': swing, 'margin': margin, 'free_rad': free_rad,
            'ramp_s': round(5.0 / smooth_hz, 2), 'pose': (('shoulder', 'flexion', 90.0),),
        },
        'sysid': {
            'rows_at': rows_at, 'independent': 1e-3, 'none': 1e-9, 'tie': 1e-9,
            'trust': round(1.0 - eff, 2), 'stick': round(12.5 * mesh_rad_s, 2),
            'sprung_stick': inside, 'lever_m': max(0.05, round(0.5 * (figure.BALL + figure.HEEL), 1)),
            'stop_deg': round(4.0 * max_bl, 1),
            'hem_acc_max': float(round(_compile.G_M_S2 / l_hem, -1)),
        },
    }


def identify() -> dict[str, Any]:
    """Compiled on-line wear, ridge correction, stand campaign and Newton-Euler sysid parameters."""
    srcs = (mechatronics.robot_sources()
            + (__file__, _compile.__file__, drives.__file__, figure.__file__, build.__file__))
    return dict(mechatronics.compiled('identify_' + mechatronics.ROBOT, srcs, _identify))
