"""Orthokinematics: her movement as text, compiled to rows of numbers her latent space is stepped by.

    (stream_inject
      (sequence_id 102)
      (blend_duration 800ms)
      (latent_targets (mode_axial_compression 0.25))
      (impedance (stiffness mode_lateral_shift 1200) (damping mode_lateral_shift 15))
      (cpg_config (enable true) (frequency 1.1Hz)
                  (amplitude mode_sagittal_drive 0.4) (amplitude mode_lateral_shift 0.35)
                  (phase_offset mode_lateral_shift mode_sagittal_drive 90deg))
      (task_constraints (maintain_zmp (polygon dynamic_footprint)) (limit_yaw_momentum 15.0)))

    (defbehavior hop
      (phase pre_load (duration 400ms) (latent_targets (mode_axial_compression 0.8))
                      (cpg_override (halt_oscillators)))
      (phase propulsion (duration 150ms) (latent_targets (mode_axial_compression -1.0))
                        (impedance (stiffness mode_axial_compression MAX_SYS_K))))

    tables, phases = ortho.compiled(TEXT)
    tables['']        # bytes: the payloads' rows in the order said, ROW.size each
    tables['hop']     # its phases' rows, each at its ms from the behaviour's start, then her rest
    phases['hop']     # ('pre_load', 'propulsion')
    ortho.rows(wire)  # [(id, at ms, blend ms, flags, the parameters ..)]

A text speaks of balance, rhythm and elasticity alone. Where her latent coordinates are asked
(`latent_targets`, -REACH to REACH a mode); how stiffly and how damped each is held there (`impedance`:
N/m and N s/m, `global` every mode, MAX_SYS_K the stiffest said); her rhythm (`cpg_config`: on
or off, its frequency, each mode's amplitude and its phase from another's; `cpg_override
(halt_oscillators)` is off); what must hold whatever is asked (`task_constraints`: her ZMP in
her soles' polygon, her yaw momentum's limit). It names no joint, no limb's side and no motor.

A row is numbers alone (`ROW`, little-endian, `ortho_row_t`): its id, when it is injected, how
long her state takes to blend to it, and one vector of parameters laid as her latent state is
(`machine.latent`). What a row leaves unsaid is NaN and stays as it was. A behaviour is rows in
a table, each a phase at its time; its last row, flagged REST, is her rest again. What a mode is
on her body and what her drives give are downstream's (`machine.modal`, `machine.wbc`).
"""
import math
import os
import re
import struct
from machine import mechatronics

#: Classification schema following the gynoid_<morphology>_<scale> convention:
#: - gynoid_ectomorph_petite: Small-framed and fine-boned (waif/gamine, < 165 cm); yields an
#:   extremely low limb moment of inertia, but minimal joint volume forces tendon-driven
#:   architectures or low-torque actuators (the Ava archetype).
#: - gynoid_ectomorph_tall: Slender, statuesque column form (> 175 cm); long lever arms provide
#:   extended reach but demand high structural flexural rigidity and strictly minimized distal
#:   mass at the knees and ankles.
#: - gynoid_mesomorph_medium: Anthropometric standard (160-168 cm); a pragmatic compromise
#:   between statistical human proportions and sufficient internal/external envelope so the
#:   structural chassis is robust rather than overly fragile,
#:   accommodating all structural 3D prints, actuators, harmonic drives, battery packs, TPU
#:   gel pads, and clothing (165 cm, 55.0 kg, 24 cm sole in machine.figure).
#: - gynoid_hourglass_full: Hyper-gynoid geometry (WHR <= 0.70); introduces a severe structural
#:   bottleneck for power buses and spinal actuators at the waist, while offering generous
#:   packaging volume for cells and high-torque hip actuators in the pelvis and thorax.
#: - gynoid_athletic_heavy: High-dynamic, muscular profile; expanded thigh, gluteal, and shoulder
#:   envelopes dictated by high-torque-density QDD actuators, structural stiffness under impact
#:   loads, and thermal dissipation requirements.
#: - gynoid_skeletal_ultralight: Hollowed truss or exoskeleton chassis stripped of synthetic soft
#:   tissue; reduces feminine cues strictly to pelvic geometry, Q-angle, and lumbar lordosis to
#:   minimize total system mass.
_FILE = os.path.join(os.path.dirname(__file__), mechatronics.ROBOT + '.lisp')

#: Her latent coordinates, a row's vectors in this order. Of her centre of mass over her soles'
#: middle: its height, 1 fully compressed; its place across, 1 over her first limb's sole;
#: along (the fifth), 1 at her toes' edge. Of her limbs against each other: the drive along her
#: way, 1 her first limb a stride behind and the other as far ahead; the lift, 1 her first
#: sole clear of the floor over the other; the width, 1 twice as wide as she stood. Her trunk's
#: pitch, 1 ahead. Her pelvis's list, 1 her first hip up, and its turn, 1 her other hip
#: forward; her arms' swing, 1 her first arm ahead; her soles' rock, 1 her first heel up and
#: the other's toes.
MODES = ('axial_compression', 'lateral_shift', 'sagittal_drive', 'pitch_attitude', 'sagittal_shift',
         'limb_lift', 'stance_width', 'pelvis_list', 'pelvis_turn', 'arm_swing', 'foot_rock')
N = len(MODES)
#: A row's parameters, one vector: each mode's target, stiffness, damping, amplitude and phase
#: (rad); then her rhythm's gain (0 off, 1 on) and Hz, her ZMP's polygon (0 none, 1 the
#: support polygon, 2 the dynamic footprint), her yaw momentum's limit, N m s.
Z, K, D, AMP, PHASE = (i * N for i in range(5))
GAIN, HZ, ZMP, YAW = (5 * N + i for i in range(4))
P = 5 * N + 4
#: A row on the wire: id, at ms, blend ms, flags as u32; the parameters as f32.
ROW = struct.Struct('<4I%df' % P)
#: A row's flag: every parameter back to her rest. A behaviour's last row, blended REST_MS.
REST, REST_MS = 1, 200
_CLEAR = float('nan')
#: A name that is a number (or `none` / `unsaid` to clear an inherited parameter back to NaN).
SYMBOLS = {'MAX_SYS_K': 8.0e4, 'true': 1.0, 'false': 0.0, 'support_polygon': 1.0, 'dynamic_footprint': 2.0,
           'none': _CLEAR, 'unsaid': _CLEAR, 'nan': _CLEAR}

_UNIT = re.compile(r'(-?\d+(?:\.\d+)?)(ms|s|Hz|deg)')
_SCALE = {'ms': ('s', 1e-3), 's': ('s', 1.0), 'Hz': ('Hz', 1.0), 'deg': ('rad', math.pi / 180.0)}
_WANTS = {'s': 'ms or s', 'Hz': 'Hz', 'rad': 'deg'}
_MODE = {'mode_' + m: (i,) for i, m in enumerate(MODES)}
_SPRUNG = {**_MODE, 'global': tuple(range(N))}
#: A row as it is compiled: [id, blend ms, its ms as a phase] and the parameters.
_ID, _BLEND, _LASTS, _HEAD = 0, 1, 2, 3
_HAS = ('(sequence_id N), (blend_duration 800ms), (duration 400ms) a phase, (latent_targets (MODE N) ..), '
        '(impedance (stiffness MODE|global N|MAX_SYS_K) (damping MODE|global N) ..), (cpg_config (enable '
        'true|false) (frequency 1.1Hz) (amplitude MODE N) (phase_offset MODE OF_MODE 90deg)), (cpg_override '
        '(halt_oscillators)) a phase, (task_constraints (maintain_zmp (polygon support_polygon|'
        'dynamic_footprint)) (limit_yaw_momentum N))')


def read(text):
    """`text` as nested lists: a name a str, a number or a SYMBOLS name a float, `400ms`
    ('s', 0.4), `1.1Hz` ('Hz', 1.1), `90deg` ('rad', 1.57..); ';' to a line's end a remark."""
    stack = [[]]
    for token in re.findall(r'[()]|"[^"]*"|[^\s()]+', re.sub(r';[^\n]*', '', text)):
        if token == '(':
            stack.append([])
        elif token != ')':
            stack[-1].append(_atom(token))
        elif len(stack) < 2:
            raise SyntaxError('a ) with no ( before it')
        else:
            done = stack.pop()
            stack[-1].append(done)
    if len(stack) != 1:
        raise SyntaxError('a ( never closed: (%s ..' % ' '.join(said(f) for f in stack[-1][:2]))
    return stack[0]


def _atom(token):
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return token[1:-1]
    unit = _UNIT.fullmatch(token)
    if unit:
        kind, scale = _SCALE[unit.group(2)]
        return (kind, float(unit.group(1)) * scale)
    if token in SYMBOLS:
        return SYMBOLS[token]
    try:
        return float(token)
    except ValueError:
        return token


def said(form):
    """A form back as text."""
    if isinstance(form, list):
        return '(%s)' % ' '.join(said(f) for f in form)
    if isinstance(form, tuple):
        return '%g %s' % (form[1], form[0])
    return '%g' % form if isinstance(form, float) else str(form)


def _number(v, low=-math.inf, high=math.inf):
    if v is _CLEAR:
        return _CLEAR
    if not isinstance(v, float) or not low <= v <= high or not math.isfinite(v):
        raise ValueError('%s is no number from %g to %g' % (said(v), low, high))
    return v


def _in(v, unit):
    if v is _CLEAR:
        return _CLEAR
    if not isinstance(v, tuple) or v[0] != unit:
        raise ValueError('%s wants %s' % (said(v), _WANTS[unit]))
    return v[1]


def _head(at, unit=None):
    def lay(r, v):
        val = _in(v, unit) if unit else _number(v, 0.0)
        r[at] = 0 if val is _CLEAR else int(round((1e3 if unit else 1.0) * val))
    return lay


def _mode(base, low=-math.inf, high=math.inf, modes=_MODE):
    def lay(r, mode, v):
        for k in modes[mode]:
            r[_HEAD + base + k] = _number(v, low, high)
    return lay


def _one(slot, low=-math.inf, high=math.inf, unit=None):
    def lay(r, v):
        r[_HEAD + slot] = _number(_in(v, unit) if unit else v, low, high)
    return lay


def _halt(r):
    r[_HEAD + GAIN] = 0.0


def _offset(r, mode, of, v):
    lead = r[_HEAD + PHASE + _MODE[of][0]]
    rad = _in(v, 'rad')
    r[_HEAD + PHASE + _MODE[mode][0]] = _CLEAR if rad is _CLEAR else (0.0 if math.isnan(lead) else lead) + rad


#: How far a mode's target goes, units either way: past 1 is past her soles' own reach - a
#: seat 3.5 axial units down and 3 sagittal back, the floor under her 6 - where a support, a
#: step or a fall bears it.
REACH = 6.0
#: What a clause says, by the names down to it: how it is laid in the row. A latent target's
#: last name is its mode.
_PAYLOAD = {
    ('sequence_id',): _head(_ID),
    ('blend_duration',): _head(_BLEND, 's'),
    ('latent_targets', 'MODE'): _mode(Z, -REACH, REACH),
    ('impedance', 'stiffness'): _mode(K, 0.0, modes=_SPRUNG),
    ('impedance', 'damping'): _mode(D, 0.0, modes=_SPRUNG),
    ('cpg_config', 'enable'): _one(GAIN, 0.0, 1.0),
    ('cpg_config', 'frequency'): _one(HZ, 0.0, unit='Hz'),
    ('cpg_config', 'amplitude'): _mode(AMP),
    ('cpg_config', 'phase_offset'): _offset,
    ('task_constraints', 'maintain_zmp', 'polygon'): _one(ZMP, 0.0, 2.0),
    ('task_constraints', 'limit_yaw_momentum'): _one(YAW, 0.0),
}
_PHASE = {**{path: lay for path, lay in _PAYLOAD.items() if path != ('sequence_id',)},
          ('duration',): _head(_LASTS, 's'), ('cpg_override', 'halt_oscillators'): _halt}


def _sayings(form, path=()):
    """A clause's leaves: (the names down to each, its values)."""
    path = path + (form[0],)
    leaves = [item for item in form[1:] if not isinstance(item, list)]
    if leaves or len(form) == 1:
        yield path, leaves
    for item in form[1:]:
        if isinstance(item, list):
            yield from _sayings(item, path)


def _merge(r, src, rules):
    """Lay non-empty fields of prototype row `src` onto row `r`."""
    if src[_LASTS] and ('duration',) not in rules:
        raise SyntaxError('(duration %dms): a stream_inject has no duration' % src[_LASTS])
    if src[_BLEND]:
        r[_BLEND] = src[_BLEND]
    if src[_LASTS]:
        r[_LASTS] = src[_LASTS]
    for k in range(_HEAD, _HEAD + P):
        if src[k] is not math.nan:
            r[k] = src[k]


def _use(r, clause, rules, traits):
    """Apply `(use TRAIT ..)` or `(extends TRAIT ..)` onto `r`."""
    if len(clause) < 2 or not all(isinstance(x, str) for x in clause[1:]):
        raise SyntaxError('%s: (%s TRAIT ..) wants a trait name' % (said(clause), clause[0]))
    for name in clause[1:]:
        if not traits or name not in traits:
            raise SyntaxError('%s: unknown trait %s' % (said(clause), name))
        _merge(r, traits[name], rules)


def _row(clauses, rules, base=None, traits=None, row_traits=()):
    """A row of a payload's or a phase's `clauses`, by `rules`."""
    r = list(base) if base else [0, 0, 0] + [math.nan] * P
    for tr in row_traits:
        _merge(r, tr, rules)
    for clause in clauses:
        if not isinstance(clause, list) or not clause:
            raise SyntaxError('%s: it has %s; a mode is %s' % (
                said(clause), _HAS, ', '.join(_MODE)))
        if clause[0] in ('use', 'extends'):
            _use(r, clause, rules, traits)
            continue
        try:
            for path, leaves in _sayings(clause):
                if path in rules:
                    rules[path](r, *leaves)
                else:
                    rules[path[:-1] + ('MODE',)](r, path[-1], *leaves)
        except KeyError:
            raise SyntaxError('%s: it has %s; a mode is %s' % (
                said(clause), _HAS, ', '.join(_MODE))) from None
        except (ValueError, TypeError, IndexError) as error:
            raise SyntaxError('%s: %s' % (said(clause), error)) from None
    return r


def _packed(r, at, flags=0):
    return ROW.pack(r[_ID], at, r[_BLEND], flags, *r[_HEAD:])


def _parents(name, spec, raw, traits):
    """(base_phases, row_traits) resolved left-to-right from `(extends BASE ..)` on a behaviour."""
    if len(spec) < 2 or not all(isinstance(x, str) for x in spec[1:]):
        raise SyntaxError('(defbehavior %s %s ..): wants a base or trait name' % (name, said(spec)))
    base, pre_traits, post_traits = [], [], []
    for p in spec[1:]:
        if p in raw:
            b_list = []
            for i, (b_name, b_row) in enumerate(raw[p]):
                r = [0, 0, 0] + [math.nan] * P
                for tr in pre_traits:
                    _merge(r, tr, _PHASE)
                if i < len(base):
                    _merge(r, base[i][1], _PHASE)
                for tr in post_traits:
                    _merge(r, tr, _PHASE)
                _merge(r, b_row, _PHASE)
                b_list.append((b_name, r))
            base, pre_traits, post_traits = b_list, pre_traits + post_traits, []
        elif p in traits:
            if base:
                for _b_name, b_row in base:
                    _merge(b_row, traits[p], _PHASE)
                post_traits.append(traits[p])
            else:
                pre_traits.append(traits[p])
        else:
            raise SyntaxError('(defbehavior %s (extends %s) ..): unknown base %s' % (name, p, p))
    return tuple(base), tuple(pre_traits + post_traits)


def _behavior(body, base=(), traits=None, row_traits=()):
    """(wire, names, raw) of a behaviour's `body` phases, optionally derived from `base` [(name, r)]."""
    if not body and not base:
        raise SyntaxError('a behaviour needs at least one (phase NAME ..)')
    for phase in body:
        if not isinstance(phase, list) or len(phase) < 2 or phase[0] != 'phase' or not isinstance(phase[1], str):
            raise SyntaxError('%s: a behaviour is phases, (phase NAME (duration 400ms) ..)' % said(phase)[:60])
    body_names = [p[1] for p in body]
    if len(set(body_names)) != len(body_names):
        raise SyntaxError('(phase %s ..): said twice' % body_names[0])
    base_map = dict(base)
    overlap = set(body_names) & set(base_map)
    if not body:
        plan = [(b_name, (), b_row) for b_name, b_row in base]
    elif overlap and set(body_names) >= set(base_map):
        plan = [(p[1], p[2:], base_map.get(p[1])) for p in body]
    elif overlap:
        by_name = {p[1]: p[2:] for p in body}
        plan = ([(b_name, by_name.get(b_name, ()), b_row) for b_name, b_row in base]
                + [(p[1], p[2:], None) for p in body if p[1] not in base_map])
    elif base and len(body) < len(base):
        raise SyntaxError('(phase %s ..): unknown phase of base (has %s)' % (body_names[0], ', '.join(base_map)))
    else:
        plan = [(p[1], p[2:], base[i][1] if i < len(base) else None) for i, p in enumerate(body)]
    wire, at, names, raw = b'', 0, [], []
    scope = {**(traits or {}), **base_map}
    for p_name, clauses, b_row in plan:
        r = _row(clauses, _PHASE, b_row, scope, () if b_row is not None else row_traits)
        if r[_LASTS] <= 0:
            raise SyntaxError('(phase %s ..): a phase lasts its (duration 400ms)' % p_name)
        wire, at, names = wire + _packed(r, at), at + r[_LASTS], names + [p_name]
        raw.append((p_name, r))
        scope[p_name] = r
    return wire + ROW.pack(0, at, REST_MS, REST, *[math.nan] * P), tuple(names), raw


def _forms(text, here, seen=(), done=None):
    """Top-level forms of `text`, expanding `(include FILE ..)` and `(extends FILE ..)` once."""
    if done is None:
        done = set(seen)
    for form in read(text):
        head = form[0] if isinstance(form, list) and form else None
        if head in ('include', 'extends'):
            if len(form) < 2 or not all(isinstance(x, str) and x for x in form[1:]):
                raise SyntaxError('%s: (%s FILE ..) wants a .lisp file name' % (said(form)[:60], head))
            for name in form[1:]:
                rel = name if name.endswith('.lisp') else name + '.lisp'
                path = os.path.normcase(os.path.abspath(os.path.join(here, rel)))
                if path in seen:
                    raise SyntaxError('(%s %s): circular include' % (head, name))
                if path in done:
                    continue
                if not os.path.isfile(path):
                    raise SyntaxError('(%s %s): no file %s' % (head, name, rel))
                done.add(path)
                with open(path, encoding='utf-8') as f:
                    yield from _forms(f.read(), os.path.dirname(path), seen + (path,), done)
        else:
            yield form


def _robot(form):
    """{name, body, modes, drives} of a (defrobot NAME (body (KEY N) ..) (mode MODE (KEY N) ..) ..
    (drive JOINT (KEY V ..) ..) ..): her manifest's numbers (tools/cores/manifest.py)."""
    if len(form) < 2 or not isinstance(form[1], str):
        raise SyntaxError('%s: (defrobot NAME ..) wants a name' % said(form)[:60])
    out = {'name': form[1], 'body': {}, 'modes': {}, 'drives': {}}
    for part in form[2:]:
        if not (isinstance(part, list) and part and part[0] in ('body', 'mode', 'drive')):
            raise SyntaxError('(defrobot %s ..) holds (body ..), (mode MODE ..) and (drive JOINT ..): not %s'
                              % (form[1], said(part)[:40]))
        if part[0] == 'mode' and part[1:2] and part[1] not in _MODE:
            raise SyntaxError('(defrobot %s (mode %s ..)): a mode is %s' % (form[1], part[1], ', '.join(_MODE)))
        named = {kv[0]: (kv[1] if len(kv) == 2 else tuple(kv[1:])) for kv in part[1 + (part[0] != 'body'):]}
        if part[0] == 'body':
            out['body'] = named
        else:
            out['modes' if part[0] == 'mode' else 'drives'][part[1]] = named
    return out


def robot(text=None, here=None):
    """Her manifest's (defrobot ..) in `text` (or `_FILE` and its includes), read: {name, body
    {key: n}, modes {mode: {key: n}}, drives {joint: {key: v}}}; the one named ROBOT where a
    text extends another robot's, else the first; None with none."""
    here = here or os.path.dirname(_FILE)
    if text is None:
        with open(_FILE, encoding='utf-8') as f:
            text = f.read()
    found = [_robot(f) for f in _forms(text, here) if isinstance(f, list) and f and f[0] == 'defrobot']
    return next((r for r in found if r['name'] == mechatronics.ROBOT), found[0] if found else None)


def rows(wire):
    """The rows of `wire`: [(id, at ms, blend ms, flags, the P parameters ..)]."""
    return list(ROW.iter_unpack(wire))


def compiled(text=None, here=None):
    """(tables, phases) of `text` (or `_FILE` if None): tables[''] its (stream_inject ..) payloads'
    rows in the order said, tables[NAME] each (defbehavior NAME ..)'s or (derive NAME BASE ..)'s,
    as bytes; phases[NAME] its phases' names."""
    here = here or os.path.dirname(_FILE)
    seen = ()
    if text is None:
        with open(_FILE, encoding='utf-8') as f:
            text = f.read()
        seen = (os.path.normcase(os.path.abspath(_FILE)),)
    tables, phases, raw, traits = {'': b''}, {}, {}, {}
    for form in _forms(text, here, seen):
        head = form[0] if isinstance(form, list) and form else None
        if head == 'defrobot':
            _robot(form)
            continue
        if head == 'defsituation':
            continue
        if head == 'stream_inject':
            tables[''] += _packed(_row(form[1:], _PAYLOAD, traits=traits), 0)
            continue
        if head == 'deftrait' and len(form) >= 3 and isinstance(form[1], str):
            ext = isinstance(form[2], list) and form[2] and form[2][0] in ('extends', 'use')
            if form[1] in traits and not (ext and form[1] in form[2][1:]):
                raise SyntaxError('(deftrait %s ..): said twice' % form[1])
            traits[form[1]] = _row(form[2:], _PHASE, traits=traits)
            continue
        if head == 'derive' and len(form) >= 3 and isinstance(form[1], str) and isinstance(form[2], str):
            if form[1] in tables and form[1] != form[2]:
                raise SyntaxError('(derive %s ..): said twice' % form[1])
            base, t_rows = _parents(form[1], ['extends', form[2]], raw, traits)
            tables[form[1]], phases[form[1]], raw[form[1]] = _behavior(form[3:], base, traits, t_rows)
            continue
        if head != 'defbehavior' or len(form) < 3 or not isinstance(form[1], str):
            raise SyntaxError('%s: a form is (include FILE ..), (defrobot NAME ..), (deftrait NAME ..), '
                              '(stream_inject ..), (defbehavior NAME ..), (derive NAME BASE ..) or '
                              '(defsituation NAME ..)' % said(form)[:60])
        ext = isinstance(form[2], list) and form[2] and form[2][0] in ('extends', 'use')
        if form[1] in tables and not (ext and form[1] in form[2][1:]):
            raise SyntaxError('(defbehavior %s ..): said twice' % form[1])
        base, t_rows = _parents(form[1], form[2], raw, traits) if ext else ((), ())
        tables[form[1]], phases[form[1]], raw[form[1]] = _behavior(
            form[3:] if ext else form[2:], base, traits, t_rows)
    return tables, phases

