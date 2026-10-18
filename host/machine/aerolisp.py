"""Flight intent and modular vehicle blocks (`defblock`/`use`) compiled from Lisp S-expressions."""
import math
from typing import Any
import os
import re
import struct
from dataclasses import dataclass, field

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_MODES = ('mode_collective_thrust mode_attitude_pitch mode_attitude_roll '
          'mode_yaw_rate mode_pitch_rate mode_roll_rate').split()
LATENT_SYMBOL_MAP = {m: i for i, m in enumerate(_MODES)}
K_MODES = 6
TASKS = {'maintain_altitude': 0, 'limit_vertical_ascent': 1}
UNITS = {'': 1.0, 'deg': math.pi / 180.0, 'rad': 1.0, 'rad_s': 1.0, 'deg_s': 1.0, 'Hz': 1.0,
         'm': 1.0, 's': 1.0, 'ms': 1e-3, 'm_s': 1.0, 'm_s2': 1.0, 'kg': 1.0}
_VALUE = re.compile(r'^([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)([A-Za-z_]\w*)?$')
_TOKEN = re.compile(r'[()]|[^\s()]+')
_PACK_FMT = 'I I 6f 6f 6f I f 6f I 2f'


@dataclass
class CompiledAeroPayload:

    """One stream compiled: aero_payload_t's fields plus CPG phase offsets."""

    sequence_id: int
    blend_duration_ms: int
    latent_targets: list
    stiffness: list
    damping: list
    cpg_enabled: int
    cpg_frequency: float
    cpg_amplitudes: list
    task_constraints_mask: int
    task_constraints: list = field(default_factory=lambda: [math.nan, math.nan])
    cpg_phase_offsets: list = field(default_factory=lambda: [0.0] * K_MODES)

    def pack(self):
        return struct.pack(
            _PACK_FMT, self.sequence_id, self.blend_duration_ms, *self.latent_targets,
            *self.stiffness, *self.damping, self.cpg_enabled, self.cpg_frequency,
            *self.cpg_amplitudes, self.task_constraints_mask, *self.task_constraints)


class AeroLispCompiler:

    """Streams compiled in the order given, each laid over the one before it."""

    def __init__(self, symbols=None):
        self.latent_dim = K_MODES
        self.symbols = {'MAX_SYS_K': 8000.0, **(symbols or {})}
        self.state = None

    def _val(self, v):
        if not isinstance(v, (str, int, float)):
            raise SyntaxError(f'expected scalar, not {v!r}')
        t = str(v)
        if t == 'ignore':
            return math.nan
        if t in self.symbols:
            return float(self.symbols[t])
        m = _VALUE.match(t)
        if not m or (m.group(2) or '') not in UNITS:
            raise SyntaxError(f'{t!r} is no value')
        return float(m.group(1)) * UNITS[m.group(2) or '']

    def _mode(self, name):
        if name not in LATENT_SYMBOL_MAP:
            raise SyntaxError(f'no mode {name!r}')
        return LATENT_SYMBOL_MAP[name]

    def compile(self, sexpr):
        w, z6 = self.state, [0.0] * K_MODES
        tgt = list(w.latent_targets) if w else [math.nan] * K_MODES
        imp = {'stiffness': list(w.stiffness) if w else list(z6),
               'damping': list(w.damping) if w else list(z6)}
        for name, v in sexpr.get('latent_targets', []):
            tgt[self._mode(name)] = self._val(v)
        for p, name, v in sexpr.get('impedance', []):
            if p not in imp:
                raise SyntaxError(f'expected stiffness or damping, not {p!r}')
            for k in (range(K_MODES) if name == 'global' else (self._mode(name),)):
                imp[p][k] = self._val(v)
        cpg = sexpr.get('cpg_config') or {}
        on = False if 'halt_oscillators' in (sexpr.get('cpg_override') or ()) else bool(
            cpg.get('enable', bool(w and w.cpg_enabled)))
        hz = self._val(cpg['frequency']) if 'frequency' in cpg else (w.cpg_frequency if w else 0.0)
        amp, ph = (list(w.cpg_amplitudes), list(w.cpg_phase_offsets)) if w else (list(z6), list(z6))
        for _w, name, v in cpg.get('amplitudes', []):
            amp[self._mode(name)] = self._val(v)
        for e in cpg.get('phase_offsets', []):
            ph[self._mode(e[1])] = (ph[self._mode(e[2])] if len(e) == 4 else 0.0) + self._val(e[-1])
        mask, vals = 0, [math.nan, math.nan]
        for k, v in sexpr.get('task_constraints', []):
            if k not in TASKS:
                raise SyntaxError(f'no task constraint {k!r}')
            mask |= 1 << TASKS[k]
            vals[TASKS[k]] = self._val(v)
        self.state = CompiledAeroPayload(
            int(sexpr.get('sequence_id', (w.sequence_id + 1) if w else 0)),
            int(round(1e3 * self._val(sexpr.get('blend_duration', '0ms')))),
            tgt, imp['stiffness'], imp['damping'], int(on), hz if on else 0.0,
            amp if on else list(z6), mask, vals, ph if on else list(z6))
        return self.state


def _trees(text):
    st = [[]]
    for w in _TOKEN.findall(re.sub(r';[^\n]*', '', text)):
        if w == '(':
            st.append([])
        elif w == ')':
            if len(st) < 2:
                raise SyntaxError('a ) with no ( before it')
            st[-2].append(st.pop())
        else:
            st[-1].append(w)
    if len(st) != 1:
        raise SyntaxError(f'{len(st) - 1} ( never closed')
    return st[0]


def _parse_tuples(rows, n):
    for r in rows:
        if not isinstance(r, list) or len(r) != n or not all(isinstance(x, str) for x in r):
            raise SyntaxError(f'expected {n}-tuple of atoms, not {r!r}')
    return [tuple(r) for r in rows]


def _parse_cpg(rows):
    out: dict[str, Any] = {'amplitudes': [], 'phase_offsets': []}
    for r in rows:
        if not isinstance(r, list) or not r or not all(isinstance(x, str) for x in r):
            raise SyntaxError(f'invalid cpg_config row {r!r}')
        if (r[0] == 'amplitude' and len(r) == 3) or (r[0] == 'phase_offset' and len(r) in (3, 4)):
            out[r[0] + 's'].append(tuple(r))
        elif r[0] == 'enable' and len(r) == 2 and r[1] in ('true', 'false'):
            out['enable'] = r[1] == 'true'
        elif r[0] == 'frequency' and len(r) == 2:
            out['frequency'] = r[1]
        else:
            raise SyntaxError(f'invalid cpg_config entry {r!r}')
    return out


def _parse_seq_id(r):
    if len(r) != 1 or not isinstance(r[0], str) or not r[0].lstrip('-+').isdigit():
        raise SyntaxError(f'sequence_id must be an integer, not {r!r}')
    return int(r[0])


def _parse_cpg_override(rows):
    if any(r != ['halt_oscillators'] for r in rows):
        raise SyntaxError(f'expected (halt_oscillators) in cpg_override, not {rows!r}')
    return tuple(r[0] for r in rows)


_BLOCKS = {
    'sequence_id': _parse_seq_id, 'blend_duration': lambda r: _parse_tuples([r], 1)[0][0],
    'latent_targets': lambda r: _parse_tuples(r, 2), 'impedance': lambda r: _parse_tuples(r, 3),
    'task_constraints': lambda r: _parse_tuples(r, 2), 'cpg_config': _parse_cpg,
    'cpg_override': _parse_cpg_override,
}


def _stream_dict(form):
    if not isinstance(form, list) or not form or form[0] != 'stream_inject':
        raise SyntaxError(f'a stream is (stream_inject ...), not {form!r}')
    s = {}
    for b in form[1:]:
        h = b[0] if isinstance(b, list) and b else None
        if not isinstance(h, str) or h not in _BLOCKS:
            raise SyntaxError(f'no block {h if h is not None else b!r}')
        s[h] = _BLOCKS[h](b[1:])
    return s


def read(text):
    return [_stream_dict(f) for f in _trees(text)]


def compiled(text, symbols=None):
    c = AeroLispCompiler(symbols)
    return [c.compile(s) for s in read(text)]


def _sp(s):
    return tuple(s.split())


AERO_BLOCKS = {
    'main_rotor': _sp('disc_height radius collective_limit cyclic_limit k_thrust k_cyclic k_drag '
                      'profile_drag_share induced_drag_share cyclic_drag_share inflow_j0 '
                      'blade_pitch inflow_floor'),
    'tail_rotor': _sp('boom_arm radius pitch_limit k_thrust'),
    'rotors': _sp('arm_length disc_height disc_radius disc_half_height k_thrust k_drag '
                  'inflow_j0 blade_pitch'),
    'flapping_wings': _sp('wing_span wing_chord shoulder_x fore_z aft_z shoulder_y '
                          'stroke_limit pitch_limit sweep_fold_limit k_thrust k_lift '
                          'k_pitch k_roll k_yaw k_drag inflow_floor gear_ratio beat_pulse'),
    'aero_surfaces': _sp('wing_span wing_chord wing_area prop_radius prop_z k_thrust tail_arm stab_area fin_area '
                         'k_drag cl_alpha cd0 cd_induced elevator_limit aileron_limit '
                         'rudder_limit k_pitch k_roll k_yaw propwash_share inflow_floor'),
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
    'vertical': _sp('climb_speed descent_speed settle_kp settle_kd'),
    'horizontal': _sp('orbit_radius orbit_speed lap_duration spot_kp spot_kd'),
    'oscillator': ('tictoc_frequency', 'tictoc_amplitude'),
    'hold': _sp('height_tol speed_tol spot_tol tilt_tol'),
}
ALL_BLOCK_KINDS = set(AERO_BLOCKS) | set(DYN_BLOCKS) | {'impedance'} | set(KIN_BLOCKS)
STUNT_FIELDS = _sp('height pace climb push speed slide surge sway turn nose x z home rail '
                   'lean roll flip light wing plan skim knife')
GATES = ('', 'fit', 'held')


def _kv_block(rows, allowed, c, req=None, base=None, blocks=None, kind=None):
    out = dict(base or {})
    for r in rows:
        if not isinstance(r, list) or len(r) != 2 or not isinstance(r[0], str):
            raise SyntaxError(f'expected (field value), not {r!r}')
        k, v = r
        if k == 'use':
            if not isinstance(v, str) or not blocks or v not in blocks or blocks[v][0] != kind:
                raise SyntaxError(f'unknown {kind} block {v!r}')
            out.update(blocks[v][1])
        elif k in allowed:
            out[k] = c._val(v)
        else:
            raise SyntaxError(f'no field {k!r}')
    miss = [k for k in (allowed if req is None else req) if k not in out]
    if miss:
        raise SyntaxError('missing fields: ' + ', '.join(miss))
    return out


def _copy_base(form, base=None):
    if len(form) < 2 or not isinstance(form[1], str):
        raise SyntaxError(f'manifest form requires a name: {form!r}')
    return {**{k: (dict(v) if isinstance(v, dict) else v) for k, v in (base or {}).items()},
            'name': form[1]}


def _dyn_impedance(rows, c, base=None, blocks=None):
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
        out[k] = (_dyn_impedance([b], c, out.get('impedance'), blocks)
                  if k == 'impedance' else {**(out.get(k) or {}), **val})
        return
    if not isinstance(h, str) or (h not in amap and (sec, h) != ('dynamics', 'impedance')):
        raise SyntaxError(f'no {sec} block {h!r}')
    out[h] = (_dyn_impedance(b[1:], c, out.get('impedance'), blocks) if h == 'impedance'
              else _kv_block(b[1:], amap[h], c, (), out.get(h), blocks, h))


def _compile_section(form, c, base, blocks, sec):
    out = _copy_base(form, base)
    amap = AERO_BLOCKS if sec == 'aerodynamics' else DYN_BLOCKS if sec == 'dynamics' else KIN_BLOCKS
    rmap = DYN_REQUIRED if sec == 'dynamics' else {}
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
            c.symbols['MAX_SYS_THRUST'] = out[kind]['k_thrust'] * (4.0 if kind == 'rotors' else 1.0) * 770.0 ** 2
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
    c.symbols.update(LAP_S=h['lap_duration'], FUNNEL_TURN_DEG_S=-turn, UNWIND_TURN_DEG_S=turn)
    return out


def payload_to_row(p, base_row, hover_thrust=19.62, orbit_speed=3.0):
    """Translate a `CompiledAeroPayload` `p` into a stunt row dict overlaid on `base_row`."""
    row, t, nan = dict(base_row), p.latent_targets, math.isnan
    for bit, i in ((1, 0), (2, 1)):
        if p.task_constraints_mask & bit and not nan(p.task_constraints[i]):
            row['height'] = float(p.task_constraints[i])
    if not nan(t[0]) and t[0] > 1.5 * hover_thrust:
        row['pace'] = max(row.get('pace', 1.5), 100.0)
    if not nan(t[3]):
        row['turn'] = math.degrees(t[3])
    for i, k in ((4, 'flip'), (5, 'roll')):
        if not nan(t[i]):
            row[k] = t[i] / math.tau if abs(t[i]) > 1e-6 else 0.0
    if not (nan(t[1]) or nan(t[2])) and 0.1 < abs(t[1]) < 1.2 and abs(row.get('turn', 0.0)) > 10.0:
        row.update(slide=orbit_speed, home=0.0)
    if p.cpg_enabled and abs(row.get('flip', 0.0)) <= 1e-6 and abs(row.get('roll', 0.0)) <= 1e-6:
        row['nose'] = -3.0 if p.cpg_amplitudes[2] > 0.2 >= p.cpg_amplitudes[1] else -2.0
    elif (not nan(t[0]) and t[0] < 0.0) or (not nan(t[1]) and abs(t[1]) > 2.0):
        row['nose'] = -1.0
    elif not nan(t[1]) and abs(t[1]) <= 0.5 and not p.cpg_enabled:
        row['nose'] = 1.0
    return row


def _compile_phase_or_stage(row, tag, stunts, c, beh_c=None, prev_row=None):
    if not isinstance(row, list) or len(row) < 3 or row[0] != tag or not isinstance(row[1], str):
        raise SyntaxError(f'expected ({tag} NAME ...), not {row!r}')
    s_keys = {'stunt', 'duration', 'blend' if tag == 'phase' else 'ramp', 'gate'}
    o_keys = set(_BLOCKS) - {'sequence_id'} if tag == 'phase' else set()
    props: dict[str, Any] = {'name': row[1], 'gate': ''}
    sp, name = {}, row[1]
    for item in row[2:]:
        h = item[0] if isinstance(item, list) and item else None
        if isinstance(h, str) and h in o_keys:
            sp[h] = _BLOCKS[h](item[1:])
        elif not isinstance(h, str) or h not in s_keys or len(item) != 2 or not isinstance(item[1], str):
            raise SyntaxError(f'no {tag} field {item!r}')
        elif h == 'stunt':
            if item[1] not in stunts:
                raise SyntaxError(f'no stunt {item[1]!r} in {tag} {name!r}')
            props['stunt'] = item[1]
        elif h == 'gate':
            if item[1] not in GATES:
                raise SyntaxError(f'no gate {item[1]!r}')
            props['gate'] = item[1]
        else:
            props[h] = c._val(item[1])
    if 'duration' not in props or ('stunt' not in props and not sp):
        raise SyntaxError(f'{tag} {name!r} requires stunt and duration')
    if tag == 'phase' and beh_c is not None:
        if 'stunt' in props:
            beh_c.state = None
        if 'blend' in props and 'blend_duration' not in sp:
            sp['blend_duration'] = f"{props['blend']:g}s"
        p = beh_c.compile(sp) if sp else None
        if p is not None and 'blend_duration' in sp and 'blend' not in props:
            props['blend'] = p.blend_duration_ms * 1e-3
        base = dict(stunts[props['stunt']] if 'stunt' in props
                    else (prev_row or stunts.get('hover') or next(iter(stunts.values()), {})))
        props.update(payload=p, stunt=props.get('stunt', 'stream'),
                     row=payload_to_row(p, base, c.symbols.get('HOVER_THRUST', 19.62)) if p is not None else base)
    return props


def _include_manifest(form, out, c, base_dir, seen):
    if len(form) != 2 or not isinstance(form[1], str):
        raise SyntaxError(f'expected (include "filename.lisp"), not {form!r}')
    raw = form[1].strip('"\'')
    path = os.path.normpath(os.path.join(base_dir, raw))
    if path in seen or not os.path.isfile(path):
        raise SyntaxError(f'cannot include {raw!r}')
    out['includes'].append(os.path.basename(path))
    with open(path, encoding='utf-8') as f:
        for sub in _trees(f.read()):
            if not isinstance(sub, list) or not sub:
                raise SyntaxError(f'invalid form {sub!r} in {raw!r}')
            _apply_manifest_form(sub, out, c, os.path.dirname(path), seen | {path})


def _sec_of(k):
    return 'aerodynamics' if k in AERO_BLOCKS else 'kinematics' if k in KIN_BLOCKS else 'dynamics'


def _compile_vehicle(form, out, c):
    if len(form) < 3 or not isinstance(form[1], str):
        raise SyntaxError(f'defvehicle requires a name and blocks: {form!r}')
    name, blocks = form[1], out['blocks']
    out['vehicle'] = name
    by_sec = {'aerodynamics': [], 'dynamics': [], 'kinematics': []}
    for sub in form[2:]:
        sh = sub[0] if isinstance(sub, list) and sub else None
        if sh == 'use' and len(sub) == 2 and isinstance(sub[1], str) and sub[1] in blocks:
            by_sec[_sec_of(blocks[sub[1]][0])].append(sub)
        elif sh in ALL_BLOCK_KINDS:
            by_sec[_sec_of(sh)].append(sub)
        elif sh in by_sec:
            by_sec[sh].extend(sub[1:])
        else:
            raise SyntaxError(f'unknown defvehicle clause {sub!r}')
    for sec, rows in by_sec.items():
        if rows:
            out[sec] = _compile_section(['def' + sec, name] + rows, c, out[sec], blocks, sec)


def _apply_manifest_form(form, out, c, base_dir, seen):
    h, stunts, blocks = form[0], out['stunts'], out['blocks']
    if h == 'include':
        return _include_manifest(form, out, c, base_dir, seen)
    if h == 'defblock':
        if len(form) < 4 or not all(isinstance(x, str) for x in form[1:3]) or form[1] not in ALL_BLOCK_KINDS:
            raise SyntaxError(f'no block kind in {form!r}')
        k, name, rows = form[1], form[2], form[3:]
        schema = AERO_BLOCKS.get(k) or DYN_BLOCKS.get(k) or KIN_BLOCKS.get(k)
        blocks[name] = (k, _dyn_impedance(rows, c, blocks=blocks) if k == 'impedance'
                        else _kv_block(rows, schema, c, (), blocks=blocks, kind=k))
        return
    if h == 'defvehicle':
        return _compile_vehicle(form, out, c)
    if h in ('defaerodynamics', 'defdynamics', 'defkinematics'):
        out[h[3:]] = _compile_section(form, c, out[h[3:]], blocks, h[3:])
        return
    if h == 'stream_inject':
        out['streams'].append(c.compile(_stream_dict(form)))
        return
    if h == 'defstunt':
        name, rows, base = _copy_base(form)['name'], form[2:], {}
        if rows and isinstance(rows[0], list) and rows[0] and rows[0][0] == 'base':
            if len(rows[0]) != 2 or not isinstance(rows[0][1], str) or rows[0][1] not in stunts:
                raise SyntaxError(f'unknown base stunt {rows[0][1:]!r} in {name!r}')
            base, rows = dict(stunts[rows[0][1]]), rows[1:]
        req = () if base else _sp('height lean pace slide turn flip roll nose light')
        base.update(_kv_block(rows, STUNT_FIELDS, c, req))
        stunts[name] = base
        return
    if h in ('defbehavior', 'defroutine'):
        if len(form) < 3 or not isinstance(form[1], str):
            raise SyntaxError(f'{h} requires a name and at least one subform')
        tag, beh_c = ('phase', AeroLispCompiler(c.symbols)) if h == 'defbehavior' else ('stage', None)
        items, prev = [], None
        for sub in form[2:]:
            step = _compile_phase_or_stage(sub, tag, stunts, c, beh_c, prev)
            prev = step.get('row')
            items.append(step)
        if h == 'defbehavior':
            out['behaviors'][form[1]] = tuple(dict(i, blend=i.get('blend', 0.0)) for i in items)
        else:
            out['routines'][form[1]] = tuple(
                (i['name'].replace('_', ' '), stunts[i['stunt']], i['duration'], i.get('ramp', 0.0), i['gate'])
                for i in items)
        return
    raise SyntaxError(f'unknown or malformed manifest form {h!r}')


def compile_manifest(text, symbols=None, base_dir=_BASE_DIR):
    c = AeroLispCompiler(symbols)
    out = {'includes': [], 'blocks': {}, 'vehicle': None, 'aerodynamics': {}, 'dynamics': {},
           'kinematics': {}, 'streams': [], 'stunts': {}, 'behaviors': {}, 'routines': {}}
    for f in _trees(text):
        if not isinstance(f, list) or not f:
            raise SyntaxError(f'invalid form {f!r}')
        _apply_manifest_form(f, out, c, base_dir, set())
    out.update(includes=tuple(out['includes']), streams=tuple(out['streams']))
    return out


def compile_aerodynamics(text, symbols=None, base_dir=_BASE_DIR):
    return compile_manifest(text, symbols, base_dir)['aerodynamics']


def compile_dynamics(text, symbols=None, base_dir=_BASE_DIR):
    return compile_manifest(text, symbols, base_dir)['dynamics']


def compile_kinematics(text, symbols=None, base_dir=_BASE_DIR):
    return compile_manifest(text, symbols, base_dir)
