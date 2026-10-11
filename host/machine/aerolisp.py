"""Flight intent as text, compiled to the flat payloads the frame's core reads (aero/).

    payloads = aerolisp.compiled(TEXT, {'MAX_SYS_THRUST': 107.7})    # a payload a stream
    blob = payloads[0].pack()                                        # aero_payload_t's bytes

A stream is `(stream_inject ...)`: its sequence, its blend, and what it changes -

    (latent_targets (MODE VALUE) ..)                   `ignore` drops a mode: NaN
    (impedance (stiffness MODE K) (damping MODE D) ..) MODE `global`: every mode's
    (cpg_config (enable true) (frequency F) (amplitude MODE A) ..)
    (task_constraints (maintain_altitude H) (limit_vertical_ascent H))

What a stream does not name stands as the stream before left it; its task constraints alone
are its own. A value is a number with its unit - deg compiled to rad; rad, rad_s, Hz, m, s as
they are - or a symbol of the frame's, given with the text (`MAX_SYS_THRUST`). A mode, a block
or a unit there is not is refused. `;` begins a comment.
"""
import math
import re
import struct
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

#: The latent modes, each its place in a payload's rows.
LATENT_SYMBOL_MAP: Dict[str, int] = {
    'mode_collective_thrust': 0,
    'mode_attitude_pitch': 1,
    'mode_attitude_roll': 2,
    'mode_yaw_rate': 3,
    'mode_pitch_rate': 4,
    'mode_roll_rate': 5,
}
K_MODES = 6

#: The task constraints, each its bit in the mask and its value's place.
TASKS = {'maintain_altitude': 0, 'limit_vertical_ascent': 1}

#: A unit's factor to the payload's: rad, rad/s, Hz, m, s, m/s^2.
UNITS = {'': 1.0, 'deg': math.pi / 180.0, 'rad': 1.0, 'rad_s': 1.0, 'Hz': 1.0, 'm': 1.0,
         's': 1.0, 'ms': 1e-3, 'm_s2': 1.0}

_VALUE = re.compile(r'^([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)([A-Za-z_]\w*)?$')
_TOKEN = re.compile(r'[()]|[^\s()]+')


@dataclass
class CompiledAeroPayload:

    """One stream compiled: aero_payload_t's fields."""

    sequence_id: int
    blend_duration_ms: int
    latent_targets: List[float]
    stiffness: List[float]
    damping: List[float]
    cpg_enabled: int
    cpg_frequency: float
    cpg_amplitudes: List[float]
    task_constraints_mask: int
    task_constraints: List[float] = field(default_factory=lambda: [math.nan] * len(TASKS))

    def pack(self) -> bytes:
        """The payload as the core reads it: uint32, uint32, float[6] x 3, uint32, float,
        float[6], uint32, float[2]."""
        return struct.pack(
            'I I %df %df %df I f %df I %df' % ((K_MODES,) * 4 + (len(TASKS),)),
            self.sequence_id, self.blend_duration_ms, *self.latent_targets, *self.stiffness,
            *self.damping, self.cpg_enabled, self.cpg_frequency, *self.cpg_amplitudes,
            self.task_constraints_mask, *self.task_constraints)


class AeroLispCompiler:

    """Streams compiled in the order given, each laid over the one before it."""

    def __init__(self, symbols: Optional[Dict[str, float]] = None):
        self.latent_dim = K_MODES
        self.symbols = dict(symbols or {})
        self.state: Optional[CompiledAeroPayload] = None

    def _parse_value(self, val: Any) -> float:
        """A value in the payload's units: a number, a number and its unit, a symbol, or
        `ignore` - NaN."""
        text = str(val)
        if text == 'ignore':
            return math.nan
        if text in self.symbols:
            return float(self.symbols[text])
        found = _VALUE.match(text)
        if not found or (found.group(2) or '') not in UNITS:
            raise SyntaxError('%r is no value - a number and its unit (%s), `ignore`, or one '
                              'of %s' % (text, ', '.join(u for u in UNITS if u),
                                         ', '.join(self.symbols) or 'no symbols given'))
        return float(found.group(1)) * UNITS[found.group(2) or '']

    def _mode(self, name: str) -> int:
        if name not in LATENT_SYMBOL_MAP:
            raise SyntaxError('no mode %r - %s' % (name, ', '.join(LATENT_SYMBOL_MAP)))
        return LATENT_SYMBOL_MAP[name]

    def compile(self, sexpr: Dict[str, Any]) -> CompiledAeroPayload:
        """A stream's payload, laid over the last one compiled: its targets, impedances and
        oscillator where the stream names none; a first stream's over nothing tracked."""
        was = self.state
        targets = list(was.latent_targets) if was else [math.nan] * K_MODES
        rows = {'stiffness': list(was.stiffness) if was else [0.0] * K_MODES,
                'damping': list(was.damping) if was else [0.0] * K_MODES}
        for name, val in sexpr.get('latent_targets', []):
            targets[self._mode(name)] = self._parse_value(val)
        for prop, name, val in sexpr.get('impedance', []):
            if prop not in rows:
                raise SyntaxError('an impedance is a stiffness or a damping, not %r' % prop)
            spread = range(K_MODES) if name == 'global' else (self._mode(name),)
            for k in spread:
                rows[prop][k] = self._parse_value(val)
        cpg = sexpr.get('cpg_config') or {}
        on = bool(cpg.get('enable', bool(was and was.cpg_enabled)))
        hz = self._parse_value(cpg['frequency']) if 'frequency' in cpg else (
            was.cpg_frequency if was else 0.0)
        swing = list(was.cpg_amplitudes) if was else [0.0] * K_MODES
        for _word, name, val in cpg.get('amplitudes', []):
            swing[self._mode(name)] = self._parse_value(val)
        mask, values = 0, [math.nan] * len(TASKS)
        for kind, val in sexpr.get('task_constraints', []):
            if kind not in TASKS:
                raise SyntaxError('no task constraint %r - %s' % (kind, ', '.join(TASKS)))
            mask |= 1 << TASKS[kind]
            values[TASKS[kind]] = self._parse_value(val)
        self.state = CompiledAeroPayload(
            sequence_id=int(sexpr.get('sequence_id', 0)),
            blend_duration_ms=int(round(1e3 * self._parse_value(
                sexpr.get('blend_duration', '0ms')))),
            latent_targets=targets, stiffness=rows['stiffness'], damping=rows['damping'],
            cpg_enabled=int(on), cpg_frequency=hz if on else 0.0,
            cpg_amplitudes=swing if on else [0.0] * K_MODES,
            task_constraints_mask=mask, task_constraints=values)
        return self.state


def _trees(text):
    """The text's forms as nested lists of words."""
    stack: List[List[Any]] = [[]]
    for word in _TOKEN.findall(re.sub(r';[^\n]*', '', text)):
        if word == '(':
            stack.append([])
        elif word == ')':
            if len(stack) < 2:
                raise SyntaxError('a ) with no ( before it')
            done = stack.pop()
            stack[-1].append(done)
        else:
            stack[-1].append(word)
    if len(stack) != 1:
        raise SyntaxError('%d ( never closed' % (len(stack) - 1))
    return stack[0]


#: A stream's blocks, each how its rows are kept: one value, rows as tuples, a table.
_BLOCKS = {
    'sequence_id': lambda rows: int(rows[0]),
    'blend_duration': lambda rows: rows[0],
    'latent_targets': lambda rows: [tuple(row) for row in rows],
    'impedance': lambda rows: [tuple(row) for row in rows],
    'task_constraints': lambda rows: [tuple(row) for row in rows],
    'cpg_config': lambda rows: dict(
        {row[0]: row[1] == 'true' if row[0] == 'enable' else row[1]
         for row in rows if row[0] != 'amplitude'},
        amplitudes=[tuple(row) for row in rows if row[0] == 'amplitude']),
}


def read(text: str) -> List[Dict[str, Any]]:
    """Every stream of `text`, each as `AeroLispCompiler.compile` takes it."""
    streams = []
    for form in _trees(text):
        if not isinstance(form, list) or not form or form[0] != 'stream_inject':
            raise SyntaxError('a stream is (stream_inject ...), not %r' % (form,))
        stream = {}
        for block in form[1:]:
            if not isinstance(block, list) or not block or block[0] not in _BLOCKS:
                raise SyntaxError('no block %r - %s' % (
                    block[0] if isinstance(block, list) and block else block,
                    ', '.join(_BLOCKS)))
            stream[block[0]] = _BLOCKS[block[0]](block[1:])
        streams.append(stream)
    return streams


def compiled(text: str, symbols: Optional[Dict[str, float]] = None) -> List[CompiledAeroPayload]:
    """`text`'s streams as payloads, in its order, each laid over the one before it."""
    compiler = AeroLispCompiler(symbols)
    return [compiler.compile(stream) for stream in read(text)]
