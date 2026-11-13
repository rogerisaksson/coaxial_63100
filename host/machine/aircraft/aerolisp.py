"""Flight intent and modular vehicle blocks (`defblock`/`use`) compiled from Lisp S-expressions."""
from dataclasses import dataclass, field
import math
import os
import re
import struct
from typing import Any

from machine.aircraft.aeroblocks import (AERO_BLOCKS, ALL_BLOCK_KINDS, DYN_BLOCKS, GATES, KIN_BLOCKS,
                                         STUNT_FIELDS, compile_section, copy_base, dyn_impedance, kv_block)
from machine.body import mechatronics

_BASE_DIR = mechatronics.AIRCRAFT
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
            out[sec] = compile_section(['def' + sec, name] + rows, c, out[sec], blocks, sec)


def _apply_manifest_form(form, out, c, base_dir, seen):
    h, stunts, blocks = form[0], out['stunts'], out['blocks']
    if h == 'include':
        return _include_manifest(form, out, c, base_dir, seen)
    if h == 'defblock':
        if len(form) < 4 or not all(isinstance(x, str) for x in form[1:3]) or form[1] not in ALL_BLOCK_KINDS:
            raise SyntaxError(f'no block kind in {form!r}')
        k, name, rows = form[1], form[2], form[3:]
        schema = AERO_BLOCKS.get(k) or DYN_BLOCKS.get(k) or KIN_BLOCKS.get(k)
        blocks[name] = (k, dyn_impedance(rows, c, blocks=blocks) if k == 'impedance'
                        else kv_block(rows, schema, c, (), blocks=blocks, kind=k))
        return
    if h == 'defvehicle':
        return _compile_vehicle(form, out, c)
    if h in ('defaerodynamics', 'defdynamics', 'defkinematics'):
        out[h[3:]] = compile_section(form, c, out[h[3:]], blocks, h[3:])
        return
    if h == 'stream_inject':
        out['streams'].append(c.compile(_stream_dict(form)))
        return
    if h == 'defstunt':
        name, rows, base = copy_base(form)['name'], form[2:], {}
        if rows and isinstance(rows[0], list) and rows[0] and rows[0][0] == 'base':
            if len(rows[0]) != 2 or not isinstance(rows[0][1], str) or rows[0][1] not in stunts:
                raise SyntaxError(f'unknown base stunt {rows[0][1:]!r} in {name!r}')
            base, rows = dict(stunts[rows[0][1]]), rows[1:]
        req = () if base else ('height', 'lean', 'pace', 'slide', 'turn', 'flip', 'roll', 'nose', 'light')
        base.update(kv_block(rows, STUNT_FIELDS, c, req))
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
