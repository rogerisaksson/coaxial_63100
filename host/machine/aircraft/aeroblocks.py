"""Physical, dynamic and kinematic block schemas and static scaling for AeroLisp."""
import math

from machine.aircraft import compile as aircraft_compile, polars, solved
from machine.body import mechatronics


def _sp(s):
    return tuple(s.split())


AERO_BLOCKS = {
    'main_rotor': _sp('disc_height radius collective_limit cyclic_limit k_thrust k_cyclic k_drag '
                      'profile_drag_share induced_drag_share cyclic_drag_share inflow_j0 '
                      'blade_pitch inflow_floor'),
    'tail_rotor': _sp('boom_arm radius pitch_limit k_thrust'),
    'rotors': _sp('rotors arm_length disc_height disc_radius disc_half_height k_thrust k_drag '
                  'inflow_j0 blade_pitch'),
    'flapping_wings': _sp('wing_span wing_chord shoulder_x fore_z aft_z shoulder_y '
                          'stroke_limit pitch_limit sweep_fold_limit k_thrust k_lift '
                          'k_pitch k_roll k_yaw k_drag inflow_floor gear_ratio beat_pulse '
                          'profile_drag_share induced_drag_share cyclic_drag_share'),
    'aero_surfaces': _sp('wing_span wing_chord wing_area prop_radius prop_z k_thrust tail_arm stab_area fin_area '
                         'k_drag cl_alpha cd0 cd_induced elevator_limit aileron_limit '
                         'rudder_limit k_pitch k_roll k_yaw propwash_share inflow_floor blade_pitch '
                         'inflow_j0 prop_inertia span_efficiency'),
    'airframe': ('air_density', 'parasite_cda'),
    'drive_load': _sp('main_base main_collective main_cyclic main_ceiling tail_base '
                      'tail_pitch tail_ceiling'),
}
DYN_BLOCKS = {
    'rigid_body': _sp('mass gravity inertia_xx inertia_yy inertia_zz skid_height foot_radius '
                      'skid_span strut_aft strut_fore hub_x hub_y hub_z'),
    'cooling': _sp('sink_mass sink_capacity r_switches_to_sink r_shunts_to_sink '
                   'r_sink_to_air_natural r_sink_to_air_forced fan_temp_on fan_temp_full '
                   'fan_idle_duty fan_max_airflow fan_max_rpm'),
    'governor': _sp('idle_speed main_speed tail_speed stunt_boost stunt_yaw_threshold '
                    'tail_ratio spin_time step_time'),
    'observers': _sp('turn_bandwidth pull_bandwidth wind_bandwidth wind_regularization '
                     'min_air_speed'),
}
DYN_REQUIRED = {
    'rigid_body': _sp('mass gravity inertia_xx inertia_yy inertia_zz skid_height foot_radius'),
    'cooling': _sp('sink_capacity r_switches_to_sink r_shunts_to_sink r_sink_to_air_natural '
                   'r_sink_to_air_forced fan_temp_on fan_temp_full'),
    'governor': ('idle_speed',),
    'observers': ('turn_bandwidth', 'pull_bandwidth', 'wind_bandwidth'),
}
DYN_MODES = _sp('attitude_pitch attitude_roll attitude_yaw pitch_rate roll_rate')
KIN_BLOCKS = {
    'envelope': _sp('floor_height hover_height high_height top_height least_pull '
                    'hover_lean sky_lean'),
    'vertical': _sp('climb_speed descent_speed wake_floor settle_kp settle_kd'),
    'horizontal': _sp('orbit_radius orbit_speed lap_duration spot_kp spot_kd'),
    'oscillator': ('tictoc_frequency', 'tictoc_amplitude'),
    'hold': _sp('height_tol speed_tol spot_tol tilt_tol'),
}
ALL_BLOCK_KINDS = set(AERO_BLOCKS) | set(DYN_BLOCKS) | {'impedance'} | set(KIN_BLOCKS)
STUNT_FIELDS = _sp('height pace climb push speed slide surge sway turn nose x z home rail '
                   'lean roll flip light wing plan skim knife lined')
GATES = ('', 'fit', 'held')


def _scale_pow(out, inherited, explicit, s, pairs):
    for k, p in pairs:
        if k not in explicit and k in inherited:
            out[k] = inherited[k] * s ** p


def _derive_flapping(out, inherited, explicit):
    dims = ('wing_span', 'wing_chord', 'stroke_limit', 'gear_ratio')
    if any(k in explicit for k in dims) and all(inherited.get(k, 0.0) > 0.0 for k in dims):
        sb, sc = out['wing_span'] / inherited['wing_span'], out['wing_chord'] / inherited['wing_chord']
        st, sg = out['stroke_limit'] / inherited['stroke_limit'], out['gear_ratio'] / inherited['gear_ratio']
        dyn = (sb ** 3) * sc * (st ** 2) / (sg ** 2)
        if 'k_thrust' not in explicit and 'k_thrust' in inherited:
            out['k_thrust'] = inherited['k_thrust'] * dyn
        if 'k_drag' not in explicit and 'k_drag' in inherited:
            out['k_drag'] = inherited['k_drag'] * dyn * sb * st
    if inherited.get('k_thrust', 0.0) > 0.0 and 'k_thrust' in out:
        kt_r = out['k_thrust'] / inherited['k_thrust']
        arms = ('fore_z', 'aft_z', 'shoulder_x', 'sweep_fold_limit', 'pitch_limit')
        if kt_r != 1.0 or any(k in explicit for k in arms):
            c_sw0 = math.cos(inherited.get('sweep_fold_limit', 0.0))
            c_sw1 = math.cos(out.get('sweep_fold_limit', 0.0))
            p0 = (inherited.get('fore_z', 0.0) - inherited.get('aft_z', 0.0)) * c_sw0 * math.cos(
                inherited.get('pitch_limit', 0.0))
            p1 = (out.get('fore_z', 0.0) - out.get('aft_z', 0.0)) * c_sw1 * math.cos(
                out.get('pitch_limit', 0.0))
            y0, y1 = inherited.get('shoulder_x', 0.0), out.get('shoulder_x', 0.0)
            for k, a0, a1 in (('k_pitch', p0, p1), ('k_roll', y0 * c_sw0, y1 * c_sw1), ('k_yaw', y0, y1)):
                if k not in explicit and k in inherited and a0 != 0.0:
                    out[k] = inherited[k] * kt_r * (a1 / a0)


def _derive_surfaces(out, inherited, explicit):
    if 'wing_area' not in explicit and 'wing_span' in out and 'wing_chord' in out:
        out['wing_area'] = out['wing_span'] * out['wing_chord']
    resized = ('wing_span' in explicit or 'wing_chord' in explicit) and out.get('wing_chord', 0.0) > 0.0
    if resized and 'cd_induced' not in explicit and 'span_efficiency' in out:
        out['cd_induced'] = polars.cd_induced({'wing_span_m': out['wing_span'], 'wing_chord_m': out['wing_chord'],
                                               'span_efficiency': out['span_efficiency']})
    if resized and 'cl_alpha' not in explicit and 'cl_alpha' in inherited:
        out['cl_alpha'] = polars.lift_slope(out['wing_span'], out['wing_chord'])
    if 'prop_radius' in explicit and inherited.get('prop_radius', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['prop_radius'] / inherited['prop_radius'],
                   (('blade_pitch', 1), ('k_thrust', 4), ('k_drag', 5), ('prop_inertia', 5)))
    for k, triggers, keys, lim in (
            ('k_pitch', ('stab_area', 'tail_arm', 'elevator_limit'), ('stab_area', 'tail_arm'), 'elevator_limit'),
            ('k_roll', ('wing_span', 'wing_chord', 'wing_area', 'aileron_limit'), ('wing_area', 'wing_span'),
             'aileron_limit'),
            ('k_yaw', ('fin_area', 'tail_arm', 'rudder_limit'), ('fin_area', 'tail_arm'), 'rudder_limit')):
        if any(t in explicit for t in triggers) and k not in explicit and k in inherited:
            denom = inherited.get(keys[0], 0.0) * inherited.get(keys[1], 0.0) * math.sin(inherited.get(lim, 0.0))
            if denom > 0.0:
                out[k] = inherited[k] * (out[keys[0]] * out[keys[1]] * math.sin(out[lim])) / denom


def _derive_rotors(out, inherited, explicit):
    if 'disc_radius' in explicit and inherited.get('disc_radius', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['disc_radius'] / inherited['disc_radius'],
                   (('disc_half_height', 1), ('blade_pitch', 1), ('k_thrust', 4), ('k_drag', 5)))


def _derive_main_rotor(out, inherited, explicit):
    if 'radius' in explicit and inherited.get('radius', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['radius'] / inherited['radius'],
                   (('blade_pitch', 1), ('k_thrust', 4), ('k_drag', 5)))
    if ('cyclic_limit' in explicit or 'collective_limit' in explicit) and 'cyclic_drag_share' not in explicit:
        c_cy0, c_co0 = inherited.get('cyclic_limit', 0.0), inherited.get('collective_limit', 0.0)
        if c_cy0 > 0.0 and c_co0 > 0.0 and out.get('collective_limit', 0.0) > 0.0 and 'cyclic_drag_share' in inherited:
            out['cyclic_drag_share'] = (inherited['cyclic_drag_share']
                                        * ((out['cyclic_limit'] / out['collective_limit']) / (c_cy0 / c_co0)) ** 2)
    if any(k in explicit for k in ('radius', 'cyclic_limit', 'inflow_j0', 'k_thrust')):
        if 'k_cyclic' not in explicit and 'k_cyclic' in inherited:
            c0 = (inherited.get('k_thrust', 0.0) * math.tan(inherited.get('cyclic_limit', 0.0))
                  * inherited.get('inflow_j0', 0.0) * inherited.get('radius', 0.0))
            if c0 > 0.0:
                c1 = out['k_thrust'] * math.tan(out['cyclic_limit']) * out['inflow_j0'] * out['radius']
                out['k_cyclic'] = inherited['k_cyclic'] * (c1 / c0)


def _derive_tail_rotor(out, inherited, explicit):
    if 'radius' in explicit and inherited.get('radius', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['radius'] / inherited['radius'], (('k_thrust', 4),))


def _rigid_axes(d, planar_z):
    if all(d.get(k, 0.0) > 0.0 for k in ('hub_x', 'hub_y', 'hub_z')):
        return d['hub_x'], d['hub_y'], d['hub_z']
    span = 0.5 * abs(d.get('strut_fore', 0.0) - d.get('strut_aft', 0.0))
    h = d.get('skid_height', 0.0)
    if d.get('skid_span', 0.0) > 0.0 and h > 0.0 and span > 0.0:
        return (d['skid_span'], span, h) if planar_z else (d['skid_span'], h, span)
    return None


def _derive_rigid_body(out, inherited, explicit):
    m_r = (out['mass'] / inherited['mass']
           if 'mass' in explicit and inherited.get('mass', 0.0) > 0.0 else 1.0)
    geom = ('hub_x', 'hub_y', 'hub_z', 'skid_span', 'skid_height', 'strut_aft', 'strut_fore')
    planar_z = inherited.get('inertia_zz', 0.0) > inherited.get('inertia_yy', 0.0)
    ax0 = _rigid_axes(inherited, planar_z) if any(k in explicit for k in geom) else None
    ax1 = _rigid_axes(out, planar_z) if ax0 is not None else None
    if ax0 is not None and ax1 is not None:
        (x0, y0, z0), (x1, y1, z1) = ax0, ax1
        for k, n0, n1 in (('inertia_xx', y0 * y0 + z0 * z0, y1 * y1 + z1 * z1),
                          ('inertia_yy', x0 * x0 + z0 * z0, x1 * x1 + z1 * z1),
                          ('inertia_zz', x0 * x0 + y0 * y0, x1 * x1 + y1 * y1)):
            if k not in explicit and k in inherited and n0 > 0.0:
                out[k] = inherited[k] * m_r * (n1 / n0)
    elif m_r != 1.0:
        _scale_pow(out, inherited, explicit, m_r,
                   (('inertia_xx', 1), ('inertia_yy', 1), ('inertia_zz', 1)))


def _derive_cooling(out, inherited, explicit):
    if 'sink_mass' in explicit and inherited.get('sink_mass', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['sink_mass'] / inherited['sink_mass'],
                   (('sink_capacity', 1),))


def _derive_governor(out, inherited, explicit):
    if ('main_speed' in explicit or 'tail_speed' in explicit) and 'tail_ratio' not in explicit:
        m0, t0 = inherited.get('main_speed', 0.0), inherited.get('tail_speed', 0.0)
        if m0 > 0.0 and t0 > 0.0 and out.get('main_speed', 0.0) > 0.0 and 'tail_ratio' in inherited:
            out['tail_ratio'] = inherited['tail_ratio'] * (out['tail_speed'] / out['main_speed']) / (t0 / m0)


def _derive_observers(out, inherited, explicit):
    if 'turn_bandwidth' in explicit and inherited.get('turn_bandwidth', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['turn_bandwidth'] / inherited['turn_bandwidth'],
                   (('pull_bandwidth', 1), ('wind_bandwidth', 1)))
    if ('wind_bandwidth' in explicit or 'turn_bandwidth' in explicit) and 'wind_regularization' not in explicit:
        w0, w1 = inherited.get('wind_bandwidth', 0.0), out.get('wind_bandwidth', 0.0)
        if w0 > 0.0 and w1 > 0.0 and 'wind_regularization' in inherited:
            out['wind_regularization'] = inherited['wind_regularization'] * w0 / w1


def _derive_envelope(out, inherited, explicit):
    if 'hover_height' in explicit and inherited.get('hover_height', 0.0) > 0.0:
        _scale_pow(out, inherited, explicit, out['hover_height'] / inherited['hover_height'],
                   (('high_height', 1), ('top_height', 1)))
    if 'hover_lean' in explicit and 'sky_lean' not in explicit:
        if inherited.get('hover_lean', 0.0) > 0.0 and 'sky_lean' in inherited:
            out['sky_lean'] = inherited['sky_lean'] * out['hover_lean'] / inherited['hover_lean']


def _derive_vertical(out, _inherited, explicit):
    if 'settle_kd' not in explicit and 'settle_kp' in out:
        out['settle_kd'] = 2.0 * out['settle_kp']
    if 'wake_floor' not in explicit and 'climb_speed' in out:
        out['wake_floor'] = 2.0 * out['climb_speed']


def _derive_horizontal(out, inherited, explicit):
    if 'lap_duration' not in explicit and 'orbit_radius' in out and out.get('orbit_speed', 0.0) > 0.0:
        out['lap_duration'] = math.tau * out['orbit_radius'] / out['orbit_speed']
    if 'orbit_radius' in explicit and 'spot_kp' not in explicit:
        r0, r1 = inherited.get('orbit_radius', 0.0), out.get('orbit_radius', 0.0)
        if r0 > 0.0 and r1 > 0.0 and 'spot_kp' in inherited:
            out['spot_kp'] = inherited['spot_kp'] * r0 / r1
    if ('spot_kp' in explicit or 'orbit_radius' in explicit) and 'spot_kd' not in explicit:
        k0, k1 = inherited.get('spot_kp', 0.0), out.get('spot_kp', 0.0)
        if k0 > 0.0 and k1 > 0.0 and 'spot_kd' in inherited:
            out['spot_kd'] = inherited['spot_kd'] * math.sqrt(k1 / k0)


_BLOCK_DERIVERS = {
    'rotors': _derive_rotors,
    'main_rotor': _derive_main_rotor,
    'tail_rotor': _derive_tail_rotor,
    'flapping_wings': _derive_flapping,
    'aero_surfaces': _derive_surfaces,
    'rigid_body': _derive_rigid_body,
    'cooling': _derive_cooling,
    'governor': _derive_governor,
    'observers': _derive_observers,
    'envelope': _derive_envelope,
    'vertical': _derive_vertical,
    'horizontal': _derive_horizontal,
}


def _derive_block(kind, out, inherited, explicit):
    fn = _BLOCK_DERIVERS.get(kind)
    if fn is not None:
        fn(out, inherited, explicit)


def kv_block(rows, allowed, c, req=None, base=None, blocks=None, kind=None):
    """Parse and statically derive `(field value)` and `(use block)` pairs for a vehicle block."""
    inherited = dict(base or {})
    out, explicit = dict(inherited), set()
    for r in rows:
        if not isinstance(r, list) or len(r) != 2 or not isinstance(r[0], str):
            raise SyntaxError(f'expected (field value), not {r!r}')
        k, v = r
        if k == 'use':
            if not isinstance(v, str) or not blocks or v not in blocks or blocks[v][0] != kind:
                raise SyntaxError(f'unknown {kind} block {v!r}')
            inherited.update(blocks[v][1])
            out.update(blocks[v][1])
        elif k in allowed:
            out[k] = c._val(v)
            explicit.add(k)
        else:
            raise SyntaxError(f'no field {k!r}')
    _derive_block(kind, out, inherited, explicit)
    miss = [k for k in (allowed if req is None else req) if k not in out]
    if miss:
        raise SyntaxError('missing fields: ' + ', '.join(miss))
    return out


def copy_base(form, base=None):
    """Copy `base` section dictionary under the manifest form's declared name."""
    if len(form) < 2 or not isinstance(form[1], str):
        raise SyntaxError(f'manifest form requires a name: {form!r}')
    return {**{k: (dict(v) if isinstance(v, dict) else v) for k, v in (base or {}).items()},
            'name': form[1]}


def dyn_impedance(rows, c, base=None, blocks=None):
    """Parse `(stiffness|damping mode value)` and `(use block)` rows for attitude/rate impedance."""
    b = base or {}
    out = {'stiffness': dict(b.get('stiffness') or {m: 0.0 for m in DYN_MODES}),
           'damping': dict(b.get('damping') or {m: 0.0 for m in DYN_MODES})}
    for r in rows:
        if isinstance(r, list) and len(r) == 2 and r[0] == 'use':
            if not isinstance(r[1], str) or not blocks or r[1] not in blocks or blocks[r[1]][0] != 'impedance':
                raise SyntaxError(f'unknown impedance block {r!r}')
            for p in ('stiffness', 'damping'):
                out[p].update(blocks[r[1]][1][p])
            continue
        if not isinstance(r, list) or len(r) != 3 or not all(isinstance(x, str) for x in r[:2]):
            raise SyntaxError(f'expected (stiffness|damping mode value), not {r!r}')
        prop, mode, val = r
        if prop not in out or (mode != 'global' and mode not in DYN_MODES):
            raise SyntaxError(f'invalid dynamics impedance {r!r}')
        num = c._val(val)
        for m in (DYN_MODES if mode == 'global' else (mode,)):
            out[prop][m] = num
    return out


def _apply_sec_block(b, out, amap, c, blocks, sec):
    h = b[0] if isinstance(b, list) and b else None
    if h == 'use':
        if len(b) != 2 or not isinstance(b[1], str) or b[1] not in blocks:
            raise SyntaxError(f'unknown {sec} block {b!r}')
        k, val = blocks[b[1]]
        if k not in amap and (sec, k) != ('dynamics', 'impedance'):
            raise SyntaxError(f'unknown {sec} block {b[1]!r}')
        out[k] = (dyn_impedance([b], c, out.get('impedance'), blocks)
                  if k == 'impedance' else {**(out.get(k) or {}), **val})
        return
    if not isinstance(h, str) or (h not in amap and (sec, h) != ('dynamics', 'impedance')):
        raise SyntaxError(f'no {sec} block {h!r}')
    out[h] = (dyn_impedance(b[1:], c, out.get('impedance'), blocks) if h == 'impedance'
              else kv_block(b[1:], amap[h], c, (), out.get(h), blocks, h))


_ROTOR_MAX_NM = aircraft_compile.ROTOR_MAX_NM


def compile_section(form, c, base, blocks, sec):
    """Compile a `defaerodynamics`, `defdynamics`, or `defkinematics` section and update symbols."""
    out = copy_base(form, base)
    amap = AERO_BLOCKS if sec == 'aerodynamics' else DYN_BLOCKS if sec == 'dynamics' else KIN_BLOCKS
    rmap = DYN_REQUIRED if sec == 'dynamics' else ({'rotors': AERO_BLOCKS['rotors'][1:]}
                                                   if sec == 'aerodynamics' else {})
    for b in form[2:]:
        _apply_sec_block(b, out, amap, c, blocks, sec)
    for k, v in out.items():
        if k in amap:
            miss = [f for f in rmap.get(k, amap[k]) if f not in v]
            if miss:
                raise SyntaxError('missing fields: ' + ', '.join(miss))
    if sec == 'aerodynamics':
        kind = next((k for k in ('rotors', 'main_rotor', 'flapping_wings', 'aero_surfaces') if k in out), None)
        req = (('airframe',) if out['name'] == 'shared' and not kind else ('rotors', 'airframe')
               if kind == 'rotors' else (kind, 'airframe', 'drive_load')
               if kind in ('flapping_wings', 'aero_surfaces')
               else ('main_rotor', 'tail_rotor', 'airframe', 'drive_load'))
        if any(b not in out for b in req):
            raise SyntaxError('missing aerodynamics blocks')
        if kind:
            tau = _ROTOR_MAX_NM if kind == 'rotors' else out['drive_load']['main_ceiling']
            w2 = tau / max(out[kind]['k_drag'], 1e-9)
            c.symbols['MAX_SYS_THRUST'] = (out[kind]['k_thrust']
                                           * (out[kind].get('rotors', 4.0) if kind == 'rotors' else 1.0) * w2)
        return out
    if sec == 'dynamics':
        if any(b not in out for b in ('rigid_body', 'governor', 'impedance', 'observers')):
            raise SyntaxError('missing dynamics blocks')
        w = out['rigid_body']['mass'] * out['rigid_body']['gravity']
        c.symbols.update(HOVER_THRUST=w, INVERTED_THRUST=-w)
        return out
    if any(b not in out for b in KIN_BLOCKS):
        raise SyntaxError('missing kinematics blocks')
    h = out['horizontal']
    turn = math.degrees(h['orbit_speed'] / h['orbit_radius'])
    c.symbols.update(LAP_S=h['lap_duration'], ORBIT_SPEED=h['orbit_speed'],
                     FUNNEL_TURN_DEG_S=-turn, UNWIND_TURN_DEG_S=turn)
    return out


def solve_vehicle_impedance(out, dyn_rows, pull_cfg=None):
    """Solve `out['dynamics']['impedance']` of a `defvehicle`'s own blocks, keeping explicit overrides."""
    aero, dyn, kin = out['aerodynamics'], out['dynamics'], out['kinematics']
    if not (aero and dyn and kin and 'quad_x_16x14' in out['blocks']):
        return
    pinned = set()
    for b in dyn_rows:
        if isinstance(b, list) and b and b[0] == 'impedance':
            for r in b[1:]:
                if isinstance(r, list) and len(r) == 3 and r[0] in ('stiffness', 'damping'):
                    for m in (DYN_MODES if r[1] == 'global' else (r[1],)):
                        pinned.add((r[0], m))
    ref_r = out['blocks']['quad_x_16x14'][1]['disc_radius']
    pcfg = aircraft_compile.part('pull') if pull_cfg is None else pull_cfg
    imp = solved.impedance(aero, dyn, kin, mechatronics.control(), pcfg, ref_r, out.get('stunts'))
    for prop in ('stiffness', 'damping'):
        for mode, val in imp[prop].items():
            if (prop, mode) not in pinned:
                dyn['impedance'][prop][mode] = val
