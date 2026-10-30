"""The kernel's lowering (ast_to_soa_lowering.lisp): a gait into a bank of its SoA ABI, seeded.

    bank = lowering.lowered(g, live)          # gait `g` as a bank (float32, wbc_soa.h), its filters seeded
    v = lowering.field(bank, 'track-width-mm')  # a field read back, SI; a node's by `node=`
    y = lowering.biquad(bank, node, x)        # her loop's pass of a node's filter, its states advanced
    plan = lowering.scheduled(soa, live)      # a compile-to-soa's schedule, each gait its bank

The text's fold, on the gait it folded, over the bank of her nominal gait (`*nominal-soa-template*`,
TEMPLATE): each of its fields (`*abi-field-descriptors*`) a param of
the gait's nodes, else a key of the last stage its trace says it of, scaled to SI and clamped at
its bounds, written at its offset; a field her nominal gait says of several nodes is each node's,
four bytes a node's index on (NODES) - the text's `offset + 4 joint` for every field would lay a
node's track over its neighbours' (0x80 + 4 k on 0x84, 0x88). Then a biquad a node - a low-pass
where the node has a cut-off, a notch where a notch, else passing -, its coefficients the text's own
Tustin (`precompute-tustin-biquad`, pre-warped, at the schedule's rate), its states seeded on her
live value (`seed-bumpless-biquad-states`) for the float32 coefficients her loop runs: its first
output that value through their DC gain, its rest, no step at the swap. Then
the manifolds' weights and the body schema's capsules. Every section on whole 64-byte lines, a
column a node or capsule; her loop's side is C (wbc/inc/wbc_soa.h), the same offsets. A bank rides
`machine.process.rcu`'s table as its words (`words`).
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.texts import dsp, kernel
from machine.errors import MachineError

_TEXT = kernel.read()
#: The text's fields: (key, offset in bytes, SI scale, least, most).
FIELDS = tuple((str(k)[1:], int(at), scale, lo, hi) for k, at, scale, lo, hi in _TEXT['*abi-field-descriptors*'])
#: A line, bytes; a float, bytes; floats a line.
LINE, F32 = 64, 4
PER_LINE = LINE // F32
_NOMINAL = _TEXT['nominal-biped-gait']
#: The nodes a bank indexes: her nominal gait's, in its order (the text's `resolve-joint-soa-index`).
NODES = tuple(k for k in _NOMINAL if not k.startswith('_'))
#: The manifolds a bank weighs, the registry's order.
MANIFOLDS = tuple(_TEXT['anatomical-manifold-registry'])
_SCHEMA = _TEXT['anatomical-capsule-schema']
#: The body schema's capsules, its slots for payloads after them.
CAPSULES = int(_SCHEMA['memory-layout']['num-body-capsules'] + _SCHEMA['memory-layout']['num-payload-slots'])
#: A biquad's columns: its coefficients, then its states.
BIQUAD = ('b0', 'b1', 'b2', 'a1', 'a2', 's1', 's2')
#: A capsule's columns: its ends in its segment's frame, its radii, m.
CAPSULE = ('p0x', 'p0y', 'p0z', 'p1x', 'p1y', 'p1z', 'r0', 'r1')
#: A node's filter by the param naming its corner: the text's modes (`precompute-tustin-biquad`).
FILTERS = (('lpf-cutoff-hz', ':lowpass'), ('biquad-notch-hz', ':notch'))
#: The text's Q where a node names none: Butterworth's.
Q = 0.70710678
#: The schedule's rate, Hz: the room's compile-to-soa.
RATE = _TEXT['compile-complete-humanoid-soa']['sample-rate-hz']


def _param(key: str) -> str:
    """The gait's name of a field's key (`dsp.ALIASES`, the first of `dsp.SPLIT`)."""
    key = dsp.ALIASES.get(key, key)
    return dsp.SPLIT.get(key, (key,))[0]


def _number(v):
    return isinstance(v, float | int) and not isinstance(v, bool)


#: The fields each node's: those her nominal gait says of more than one node.
EACH = tuple(k for k, *_r in FIELDS if sum(_number(p.get(_param(k))) for p in _NOMINAL.values() if isinstance(p, dict)) > 1)


def _lined(n):
    """`n` floats' room on whole lines, bytes."""
    return -(-n // PER_LINE) * LINE


def _sections():
    out = [(k, at, len(NODES) if k in EACH else 1) for k, at, *_r in FIELDS]
    at = _lined(max(a + F32 * n for _k, a, n in out) // F32)
    for name, n in [(c, len(NODES)) for c in BIQUAD] + [('weight', len(MANIFOLDS))] + [(c, CAPSULES) for c in CAPSULE]:
        out.append((name, at, n))
        at += _lined(n)
    return tuple(out), at


#: Each section: (its name, its offset in bytes, its floats); a bank's size, bytes.
SECTIONS, SIZE = _sections()
AT = {name: at // F32 for name, at, _n in SECTIONS}


def _said(g, key):
    """[(node, value)] of field `key` in gait `g`: each node holding it, else the last stage its trace
    says it of (node None)."""
    name = _param(key)
    held = [(at, p[name]) for at, p in g.items() if not at.startswith('_') and isinstance(p, dict) and _number(p.get(name))]
    told = [keys[key] for _stage, keys in g.get('_trace', ()) if isinstance(keys, dict) and _number(keys.get(key))]
    return held or [(None, v) for v in told[-1:]]


def _index(node, key):
    if node not in NODES:
        raise MachineError('%s of %s: no node of the bank (%s)' % (key, node, ', '.join(NODES)))
    return NODES.index(node)


def _filter(params):
    """(mode, corner Hz, Q) of a node's params: its one filter, or None."""
    found = [(mode, params[p]) for p, mode in FILTERS if _number(params.get(p))]
    if len(found) > 1:
        raise MachineError('a node of %s: one filter a node' % ' and '.join(m for m, _f in found))
    return [(mode, fc, params.get('q-factor', Q)) for mode, fc in found][0] if found else None


def coefficients(mode, fc, q):
    """[b0 b1 b2 a1 a2] of the text's Tustin (`precompute-tustin-biquad`) at RATE, as it computes them."""
    return dsp.value(['precompute-tustin-biquad', mode, fc, q, RATE], _TEXT)


def seeded(b, x0):
    """[s1 s2] holding biquad `b` at rest on `x0`, the text's (`seed-bumpless-biquad-states`)."""
    return dsp.value(['seed-bumpless-biquad-states', *b, x0], _TEXT)


def _fields(bank, g):
    """`bank` with the fields gait `g` says written, the rest as they were."""
    for key, _at, scale, lo, hi in FIELDS:
        said = _said(g, key)
        if key not in EACH and len(said) > 1:
            raise MachineError('%s: said of %s, one field' % (key, ', '.join(n for n, _v in said)))
        for node, v in said:
            bank[AT[key] + (_index(node, key) if key in EACH else 0)] = min(hi, max(lo, v * scale))
    return bank


#: The bank the fold begins on: her nominal gait's fields.
TEMPLATE = _fields(np.zeros(SIZE // F32, np.float32), _NOMINAL)


def lowered(g, live=None):
    """Gait `g` (`dsp`'s, evaluated) as a bank, float32 (SIZE bytes): its fields over TEMPLATE's, a
    biquad a node seeded on `live` ({node: her value}, 0 where none), the manifolds' weights, the
    capsules."""
    live = live or {}
    bank = _fields(TEMPLATE.copy(), g)
    for k, node in enumerate(NODES):
        f = _filter(g.get(node, {}))
        b = tuple(np.asarray(coefficients(*f) if f else (1.0, 0.0, 0.0, 0.0, 0.0), np.float32).tolist())
        for name, v in zip(BIQUAD, b + tuple(seeded(b, live.get(node, 0.0)))):
            bank[AT[name] + k] = v
    weights = g['centroidal-balance']['manifold-weights']
    bank[AT['weight']:AT['weight'] + len(MANIFOLDS)] = [weights[m] for m in MANIFOLDS]
    for k, c in enumerate(_SCHEMA['capsules'].values()):
        for name, v in zip(CAPSULE, tuple(c['p0']) + tuple(c['p1']) + (c['r0-m'], c['r1-m'])):
            bank[AT[name] + k] = v
    return bank


def field(bank, key, node=None):
    """Field or column `key` of `bank`: a node's where it is each node's or a column's."""
    return float(bank[AT[key] + (0 if node is None else _index(node, key))])


def biquad(bank, node, x):
    """Her loop's pass of `node`'s biquad of `bank` on `x`, direct form II transposed in float32 as
    wbc_soa.c: its output, its states advanced in the bank."""
    k, f = _index(node, 'biquad'), np.float32
    b0, b1, b2, a1, a2 = (bank[AT[c] + k] for c in BIQUAD[:5])
    x = f(x)
    y = b0 * x + bank[AT['s1'] + k]
    bank[AT['s1'] + k] = b1 * x - a1 * y + bank[AT['s2'] + k]
    bank[AT['s2'] + k] = b2 * x - a2 * y
    return float(y)


def words(bank):
    """A bank as `machine.process.rcu`'s table of one column: its bytes as 8-byte words, (1, SIZE / 8)."""
    return bank.view(np.float64).reshape(1, -1)


def scheduled(soa, live=None):
    """[(segment, bank or None)] of a compile-to-soa (`dsp.schedule`): each gait's bank, a dwell
    none."""
    return [(seg, lowered(seg[-1], live) if isinstance(seg[-1], dict) else None) for seg in dsp.schedule(soa)]
