"""A patch played on her loop: its lisp read as written, its nodes on her voices, its morph as rows.

    got = patch.read(text)                        # {'states', 'modifiers', 'transitions', 'patches'}
    dry, wet, bands = patch.voiced(got, fr, std)  # {slot: value} on her walk's row `std`, {slot: band}
    wire, span = patch.morph(dry, wet, bands, got)  # the transition's rows, contiguous, on the bar grid
    at = patch.crossing(st, now)                  # her lateral voice's next zero crossing, s
    span = patch.window(st)                       # its double support, s, to wait in for the lock
    ok = patch.locked(y, fr, a)                   # both soles stand, her capture point in their boxes

A patch (machine/runway.lisp) is a dry state (defstate), a wet one made of it by a modifier's ops
(defmodifier), a transition between them (deftransition: a sync trigger, a clock slew, bands of
nodes on their curves) and a schedule in bars (defpatch). Its nodes reach her voices by NODES: a
row slot each, a conversion each; what has no voice is said so (VOICELESS). The morph is two bars
of rows a sixteenth apart (GRID), each saying every mapped slot at its time - the dry plus the
wet's change on its band's curve over the band's window - and the clock on its own, contiguous
curve samples (`ortho.CURVE`) her loop draws as one Catmull-Rom; its times run on the clock as it
slews. A bar is a stride, her clock's turn. Measured on her (findings/sequencer.md): the dry and
the wet at once, by a linear crossfade at any phase of the stride or by the morph locked to the
crossing, stand alike; the rock voice stiffened for the heel's transient felled her alone.
"""
import math
import re

from machine import modal, ortho, stride

#: The morph's grid, sixteenths of a bar.
GRID = 16
#: A step a beat: a bar (a stride, a turn of her clock) is two of the patch's beats.
BEATS = 2.0
#: The curves a band rides, u in [0, 1] -> the wet's share: an exponential with e^-SPAN_E of its
#: way left at the window's end, normalized there; the equal-power crossfade's energy share; an
#: audio taper's swell over SWELL_DB; Hermite's smoothstep.
SPAN_E, SWELL_DB = 4.0, 40.0
CURVES = {
    'exponential-fast': lambda u: (1.0 - math.exp(-SPAN_E * u)) / (1.0 - math.exp(-SPAN_E)),
    'equal-power': lambda u: math.sin(0.5 * math.pi * u) ** 2,
    'logarithmic-swell': lambda u: (10.0 ** (SWELL_DB / 20.0 * u) - 1.0) / (10.0 ** (SWELL_DB / 20.0) - 1.0),
    'hermite-s-curve': lambda u: u * u * (3.0 - 2.0 * u),
    'linear': lambda u: u,
}
#: A morph's rows' sequence id, after the takes' (300) and a modified row (301).
ROW_ID = 302

_TOKEN = re.compile(r'\s*(?:(;[^\n]*)|([()\[\]])|(->)|("(?:[^"\\]|\\.)*")|([^\s()\[\];"]+))')


def _tokens(text):
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m or m.end() == pos:
            if text[pos:].strip():
                raise SyntaxError('unreadable at %r' % text[pos:pos + 30])
            return
        pos = m.end()
        comment, paren, arrow, string, atom = m.groups()
        if not comment:
            yield paren or arrow or string or atom


def _atom(tok):
    if tok.startswith('"'):
        return tok[1:-1]
    if tok in ('true', 'false'):
        return tok == 'true'
    try:
        return float(tok)
    except ValueError:
        return tok


def forms(text):
    """The text's forms: lists, vectors as tuples, keywords and symbols as strings, numbers."""
    stack = [[]]
    for tok in _tokens(text):
        if tok in ('(', '['):
            stack.append([] if tok == '(' else ['[]'])
        elif tok in (')', ']'):
            if len(stack) == 1:
                raise SyntaxError('a %s closes nothing' % tok)
            done = stack.pop()
            stack[-1].append(tuple(done[1:]) if done[:1] == ['[]'] else done)
        else:
            stack[-1].append(_atom(tok))
    if len(stack) != 1:
        raise SyntaxError('%d forms left open' % (len(stack) - 1))
    return stack[0]


def _keyed(items):
    """{key: value} of ':key value' pairs, a value of pairs a dict, a bare form (op ..) under '_ops'."""
    out, k = {}, 0
    while k < len(items):
        item = items[k]
        if isinstance(item, list) and item and isinstance(item[0], str) and not item[0].startswith(':'):
            out.setdefault('_ops', []).append((item[0], _keyed(item[1:])))
            k += 1
            continue
        if not (isinstance(item, str) and item.startswith(':')):
            raise SyntaxError('a keyword wanted, %r found' % (item,))
        val = items[k + 1] if k + 1 < len(items) else True
        if isinstance(val, list) and val and isinstance(val[0], str) and val[0].startswith(':'):
            val = _keyed(val)
        out[item[1:]] = val.lstrip(':') if isinstance(val, str) else val
        k += 2
    return out


def read(text):
    """A patch's forms: {'states': {name: {node: {param: value}}}, 'modifiers': {name: [(op,
    params)]}, 'transitions': {name: {'endpoints', 'ops': [(op, params)]}}, 'patches': {name:
    {op: params}}}."""
    got = {'states': {}, 'modifiers': {}, 'transitions': {}, 'patches': {}}
    for form in forms(text):
        head, name, body = form[0], form[1], form[2:]
        if head == 'defstate':
            state = {}
            for clause in body:
                if clause[0] == ':kinematics':
                    state.update({clause[k][1:]: _keyed(clause[k + 1]) for k in range(1, len(clause), 2)})
                else:
                    state[clause[0][1:]] = _keyed(clause[1:])
            got['states'][name] = state
        elif head == 'defmodifier':
            if body[1][0] != '->':
                raise SyntaxError('(defmodifier %s [stream] (-> stream (op ..) ..))' % name)
            got['modifiers'][name] = [(op[0], _keyed(op[1:])) for op in body[1][2:]]
        elif head == 'deftransition':
            got['transitions'][name] = {'endpoints': _keyed(list(body[0])), 'ops': [(op[0], _keyed(op[1:])) for op in body[1:]]}
        elif head == 'defpatch':
            got['patches'][name] = {op[0]: _keyed(op[1:]) for op in body}
        else:
            raise SyntaxError('(%s ..): a patch has defstate, defmodifier, deftransition, defpatch' % head)
    return got


def wet_of(state, ops):
    """The state a modifier's `ops` make of `state` ({node: {param: value}}): each op's params
    on its node - the clock's quantized, a transient's node and its soft-clip node, the mid-side
    nodes, the sidechain on its target, the filter bank's on theirs."""
    wet = {node: dict(params) for node, params in state.items()}
    names = {'offset-rad': 'dc-offset-rad', 'ceiling-rad': 'headroom-rad'}
    for op, p in ops:
        if op == 'quantize-grid':
            wet['clock'].update({'bpm': p['target-bpm'], 'swing-gate': p.get('swing-gate', 0.0)})
            wet['clock']['phase-jitter-ms'] *= 1.0 - float(bool(p.get('strip-jitter')))
        elif op == 'shape-transients':
            clip = {k: p[k] for k in ('flexion-cushion-rad', 'compliance') if k in p}
            rest = {k: v for k, v in p.items() if k not in clip and k not in ('target', 'soft-clip-node')}
            wet.setdefault(p['target'], {}).update(rest)
            wet.setdefault(p['soft-clip-node'], {}).update(clip)
        elif op == 'mid-side-rebalance':
            for part in ('narrow-stereo', 'stereo-mod'):
                wet.setdefault(p[part]['node'], {}).update({k: v for k, v in p[part].items() if k != 'node'})
        elif op == 'route-sidechain':
            wet.setdefault(p['target'], {})['sidechain'] = {k: v for k, v in p.items() if k != 'target'}
        elif op == 'apply-filter-bank':
            for _f, q in p['_ops']:
                wet.setdefault(q['target'], {}).update({names.get(k, k): v for k, v in q.items() if k != 'target'})
        else:
            raise SyntaxError('(%s ..): a modifier has quantize-grid, shape-transients, mid-side-rebalance, '
                              'route-sidechain, apply-filter-bank' % op)
    return wet


Z, K, D, AMP, PHASE = ortho.Z, ortho.K, ortho.D, ortho.AMP, ortho.PHASE
#: A node's param on her voice: (node, param, row slot, rule). Rules: 'beats' a tempo's steps a
#: minute to her stride's Hz; 'unit' the value in the voice's unit; 'shift' the change from the
#: dry in the voice's unit, onto her walk's; 'ratio' her walk's times the value over the dry's;
#: 'inverse' times the dry's over the value; 'zeta' a damper for that ratio of critical on the
#: slot's spring; 'lpf_k', 'lpf_d' a second-order Butterworth low-pass at that cutoff, Hz; 'db'
#: her walk's amplitude times the gain's change; 'lag' her walk's phase less the lag; 'track' a
#: track, mm, as her width's share of the landings' floor APART_M, which her walk's 103 mm sits
#: on (`stride.meets`); 'milli' mm or ms to m or s.
NODES = (
    ('clock', 'bpm', ortho.HZ, 'beats'),
    ('tibiofemoral', 'flexion-cushion-rad', Z + modal.AXIAL, 'ratio'),
    ('tibiofemoral', 'compliance', K + modal.AXIAL, 'inverse'),
    ('sacroiliac-z', 'damping', D + modal.AXIAL, 'zeta'),
    ('iliac-coronal', 'lfo-amp-rad', AMP + modal.LIST, 'unit'),
    ('c7-t12-thoracic', 'dc-offset-rad', Z + modal.PITCH, 'shift'),
    ('c7-t12-thoracic', 'cutoff-hz', K + modal.PITCH, 'lpf_k'),
    ('c7-t12-thoracic', 'cutoff-hz', D + modal.PITCH, 'lpf_d'),
    ('glenohumeral', 'gain-db', AMP + modal.ARMS, 'db'),
    ('glenohumeral', 'lag-rad', PHASE + modal.ARMS, 'lag'),
    ('tarsal-coronal', 'track-width-mm', Z + modal.WIDTH, 'track'),
    ('sacroiliac-z', 'sidechain.duck-depth-mm', ortho.DUCK.start, 'milli'),
    ('sacroiliac-z', 'sidechain.attack-ms', ortho.DUCK.start + 1, 'milli'),
    ('sacroiliac-z', 'sidechain.release-ms', ortho.DUCK.start + 2, 'milli'),
)
#: What of a patch has no voice of hers, and what she has instead (findings/sequencer.md).
VOICELESS = {
    ('clock', 'swing-gate'): 'her limbs half a turn apart: 0.0005 of a step measured',
    ('clock', 'phase-jitter-ms'): 'her clock has no humanize: 1.1 ms a step measured',
    ('calcaneus', '*'): "her shoe's sole (mjcf.SOLE_S): 21.6 ms attack on a 9.5 ms sole, 24.6 on her 20; the rock "
                        "voice stiffened for it felled her",
    ('sacroiliac-z', 'vertical-bounce-mm'): "her height's own: 22.7 mm a step",
    ('iliac-coronal', 'wave'): 'a sine',
    ('iliac-coronal', 'width'): "the list's: read as the lateral sway's 1.55 beside a narrow width she widened to "
                                '230 mm and fell',
    ('l4-l5-lumbar', '*'): "her trunk has one pitch voice, the thoracic's",
    ('c1-c2-atlas', '*'): 'no head voice',
    ('craniofacial', '*'): 'no face',
}


def _rule(rule, v, v_dry, slot, std, fr, row):
    mass, mode = fr[modal.MASS], slot % modal.N
    return {
        'beats': lambda: v / (60.0 * BEATS),
        'unit': lambda: v / fr[modal.UNIT][mode],
        'shift': lambda: std[slot] + (v - v_dry) / fr[modal.UNIT][mode],
        'ratio': lambda: std[slot] * v / v_dry,
        'inverse': lambda: std[slot] * v_dry / v,
        'zeta': lambda: 2.0 * v * math.sqrt(row.get(K + mode, std[K + mode]) * mass[mode]),
        'lpf_k': lambda: mass[mode] * (math.tau * v) ** 2,
        'lpf_d': lambda: math.sqrt(2.0) * mass[mode] * math.tau * v,
        'db': lambda: std[slot] * 10.0 ** ((v - v_dry) / 20.0),
        'lag': lambda: std[slot] - (v - v_dry),
        'track': lambda: 1e-3 * v / stride.APART_M - 1.0,
        'milli': lambda: 1e-3 * v,
    }[rule]()


def _at(params, path):
    """A param by its dotted path in a node's params, None where it is not."""
    for key in path.split('.'):
        params = params.get(key) if isinstance(params, dict) else None
    return params


def voiced(got, fr, std, transition=None):
    """(dry, wet, bands) of a patch's transition: {slot: value} of its dry state and of its wet
    one on her walk's row `std` - the dry's params the reference of the wet's change, a cutoff's
    rule on a second-order slope alone, a lag's dry none - and {slot: (curve, from bar, to bar)}
    of the band each slot's node rides."""
    tr = got['transitions'][transition or next(iter(got['transitions']))]
    dry_state = got['states'][tr['endpoints']['dry-source']]
    wet_state = wet_of(dry_state, got['modifiers'][tr['endpoints']['wet-target'][0]])
    rows = {}
    for which, state in (('dry', dry_state), ('wet', wet_state)):
        row = rows[which] = {}
        for node, param, slot, rule in NODES:
            p, d = _at(state.get(node, {}), param), _at(dry_state.get(node, {}), param)
            if p is not None and not (rule.startswith('lpf') and state[node].get('slope-db') != 12.0):
                row[slot] = _rule(rule, p, 0.0 if d is None else d, slot, std, fr, row)
    rows['dry'].update({slot: float(std[slot]) for slot in set(rows['wet']) - set(rows['dry'])})
    bands = {}
    for op, p in tr['ops']:
        for _band, q in (p['_ops'] if op == 'band-split-crossfade' else ()):
            for node in q['nodes']:
                bands.update({slot: (q['curve'], q['window-bars'][0], q['window-bars'][-1])
                              for n, _param, slot, _rule in NODES if n == node.lstrip(':')})
    return rows['dry'], rows['wet'], bands


def clock_of(got, transition=None):
    """(curve, from bar, to bar, dry Hz, wet Hz) of a transition's clock slew."""
    tr = got['transitions'][transition or next(iter(got['transitions']))]
    p = next(q for op, q in tr['ops'] if op == 'pll-clock-slew')
    bpm = p['bpm-ramp']
    return p['curve'], p['window-bars'][0], p['window-bars'][-1], bpm[0] / (60.0 * BEATS), bpm[-1] / (60.0 * BEATS)


def morph(dry, wet, bands, clock, bars=2.0):
    """The transition's rows (`ortho.ROW`, contiguous curve samples a sixteenth of a bar apart) and
    their span, s: each sample every slot at the dry plus the wet's change on its band's curve, the
    clock's Hz on its own, the times as the clock slews (a sixteenth over its mean Hz)."""
    import numpy as np
    at = np.arange(int(bars * GRID) + 1) / GRID
    share = lambda c, b0, b1, b: CURVES[c](min(max((b - b0) / (b1 - b0), 0.0), 1.0))
    curve, c0, c1, hz0, hz1 = clock
    hz = np.array([hz0 + (hz1 - hz0) * share(curve, c0, c1, b) for b in at])
    ms = np.round(1e3 * np.concatenate([[0.0], np.cumsum(np.diff(at) / (0.5 * (hz[1:] + hz[:-1])))])).astype(int)
    gaps = np.diff(ms)
    rows = []
    for k, b in enumerate(at):
        p = [math.nan] * ortho.P
        p[ortho.GAIN], p[ortho.HZ] = 1.0, float(hz[k])
        for slot, band in bands.items():
            p[slot] = dry[slot] + (wet[slot] - dry[slot]) * share(*band, b)
        rows.append(ortho.ROW.pack(ROW_ID, int(ms[k]), int(gaps[min(k, len(gaps) - 1)]),
                                   ortho.CURVE * (k + 1 < len(at)), *p))
    return b''.join(rows), float(ms[-1]) * 1e-3


def crossing(st, now):
    """The time of her lateral voice's next zero crossing, s: her clock's angle and the voice's
    phase carried at her rhythm's rate (`latent` THETA, PHASE, HZ)."""
    from machine import latent
    hz = float(st[latent.NOW][latent.HZ])
    psi = float(st[latent.THETA] + st[latent.NOW][latent.PHASE][modal.LATERAL])
    return now + ((-psi) % math.pi) / (math.tau * hz)


def window(st):
    """The double support past each crossing, s: pi - 2 stride.PHI_W of her clock's turn (her lateral
    voice 108 deg past her drive's, pi - PHI_W; measured her soles both stand 0 to 0.6 rad past it)."""
    from machine import latent
    return (math.pi - 2.0 * stride.PHI_W) / (math.tau * float(st[latent.NOW][latent.HZ]))


def locked(y, fr, a):
    """Whether a transition may begin now: both soles stand and the ZMP her law asks (her centre of
    mass less its asked acceleration over omega squared) lies in their boxes (`poise.boxes`: a sole
    less poise.MARGIN_M round it). Her capture point runs ahead of them walking, never in them."""
    from machine import poise
    rot = fr[modal.ROT].reshape(3, 3)
    c = rot @ y[modal.COM]
    zmp = c[:2] - (rot @ a[modal.COM_ACC])[:2] / float(fr[modal.OMEGA]) ** 2
    lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
    return bool(a[modal.STANCE].min() > 0.5 and (zmp >= lo.min(axis=0)).all() and (zmp <= hi.max(axis=0)).all())
