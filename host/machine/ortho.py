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
(`latent_targets`, -1 to 1 a mode); how stiffly and how damped each is held there (`impedance`:
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
import re
import struct

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
#: A name that is a number.
SYMBOLS = {'MAX_SYS_K': 8.0e4, 'true': 1.0, 'false': 0.0, 'support_polygon': 1.0, 'dynamic_footprint': 2.0}

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
    for token in re.findall(r'[()]|[^\s()]+', re.sub(r';[^\n]*', '', text)):
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
    if not isinstance(v, float) or not low <= v <= high or not math.isfinite(v):
        raise ValueError('%s is no number from %g to %g' % (said(v), low, high))
    return v


def _in(v, unit):
    if not isinstance(v, tuple) or v[0] != unit:
        raise ValueError('%s wants %s' % (said(v), _WANTS[unit]))
    return v[1]


def _head(at, unit=None):
    def lay(r, v):
        r[at] = int(round(1e3 * _in(v, unit) if unit else _number(v, 0.0)))
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
    r[_HEAD + PHASE + _MODE[mode][0]] = (0.0 if math.isnan(lead) else lead) + _in(v, 'rad')


#: What a clause says, by the names down to it: how it is laid in the row. A latent target's
#: last name is its mode.
_PAYLOAD = {
    ('sequence_id',): _head(_ID),
    ('blend_duration',): _head(_BLEND, 's'),
    ('latent_targets', 'MODE'): _mode(Z, -1.0, 1.0),
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


def _row(clauses, rules):
    """A row of a payload's or a phase's `clauses`, by `rules`."""
    r = [0, 0, 0] + [math.nan] * P
    for clause in clauses:
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


def rows(wire):
    """The rows of `wire`: [(id, at ms, blend ms, flags, the P parameters ..)]."""
    return list(ROW.iter_unpack(wire))


def compiled(text):
    """(tables, phases) of `text`: tables[''] its (stream_inject ..) payloads' rows in the
    order said, tables[NAME] each (defbehavior NAME (phase NAME ..) ..)'s, as bytes;
    phases[NAME] its phases' names."""
    tables, phases = {'': b''}, {}
    for form in read(text):
        head = form[0] if isinstance(form, list) and form else None
        if head == 'stream_inject':
            tables[''] += _packed(_row(form[1:], _PAYLOAD), 0)
        elif head != 'defbehavior' or len(form) < 3 or not isinstance(form[1], str):
            raise SyntaxError('%s: a form is (stream_inject ..) or (defbehavior NAME (phase NAME ..) ..)'
                              % said(form)[:60])
        elif form[1] in tables:
            raise SyntaxError('(defbehavior %s ..): said twice' % form[1])
        else:
            wire, at, names = b'', 0, []
            for phase in form[2:]:
                if not isinstance(phase, list) or len(phase) < 2 or phase[0] != 'phase' or not isinstance(
                        phase[1], str):
                    raise SyntaxError('%s: a behaviour is phases, (phase NAME (duration 400ms) ..)'
                                      % said(phase)[:60])
                r = _row(phase[2:], _PHASE)
                if r[_LASTS] <= 0:
                    raise SyntaxError('(phase %s ..): a phase lasts its (duration 400ms)' % phase[1])
                wire, at, names = wire + _packed(r, at), at + r[_LASTS], names + [phase[1]]
            tables[form[1]] = wire + ROW.pack(0, at, REST_MS, REST, *[math.nan] * P)
            phases[form[1]] = tuple(names)
    return tables, phases
