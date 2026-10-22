"""The ortho-DSP suite (machine/ortho_dsp/) evaluated and voiced on her walk's row.

    got = dsp.read(dsp.SUITE)                    # every name the texts define, evaluated
    g = got['patch-sneaker-runway']              # a gait: {node: {param: value}}, its stages in '_trace'
    row, told = dsp.voiced(g, dry, fr)           # {slot: value} off her walk's row; {'twist', 'grip'}
    wire, span = dsp.morphed(m, dry, fr)         # a Morph's rows (`patch.morph`)
    plan = dsp.schedule(got['compile-omnidirectional-suite'])  # [(from bar, to bar, gait or Morph)]

A gait is the suite's manifold as nodes of params, a sidechain on the node it ducks; the runway's
dialect (runway.lisp, `tools/sim/identify.py`'s) says the same by ALIASES. Stages are functions of
a gait threaded by `->`; ops (GAIT_OPS), functions (FUNCS) and special forms (SPECIAL) are tables.
A gait moves her walk's row (`walk`) by its changes off a dry gait, the suite's nominal (VOICES,
RULES): the dry plays her walk to the last slot. What has no voice of hers is named with what she
has instead (VOICELESS), what she cannot do refused by name (REFUSED), anything else as unknown.
"""
import collections
import copy
import glob
import math
import operator
import os

from machine import modal, ortho, patch, se2
from machine.errors import MachineError

#: The suite's modules, read in their order.
SUITE = sorted(glob.glob(os.path.join(os.path.dirname(__file__), 'ortho_dsp', '*.lisp')))
#: An op on a param, `(:scale 1.2)`; a bare value sets. Clamped, an unset param takes the bound.
Op = collections.namedtuple('Op', 'name value')
OPS = {'set': lambda old, v: v, 'scale': operator.mul, 'add': operator.add,
       'clamp-min': lambda old, v: v if old is None else max(old, v),
       'clamp-max': lambda old, v: v if old is None else min(old, v)}
#: A stage: its parameters, its keys' default forms, its body; a transition's ops its body.
Stage = collections.namedtuple('Stage', 'name params keys body')
#: A transition between two gaits: its sync trigger, its clock's slew, its bands, its bars.
Morph = collections.namedtuple('Morph', 'source target sync clock bands bars')
#: The runway's nodes and params by the suite's names; a param said of two.
NODE_ALIASES = {'clock': 'oscillator', 'sacroiliac-z': 'com-dynamics'}
ALIASES = {'target-bpm': 'bpm', 'swing-gate': 'even-step-swing-offset', 'flexion-cushion-rad': 'stance-yield-rad',
           'lfo-amp-rad': 'lfo-amplitude-rad', 'wave': 'waveform', 'width': 'stereo-width',
           'vertical-bounce-mm': 'vertical-oscillation-mm', 'damping': 'damping-ratio',
           'offset-rad': 'dc-offset-rad', 'cutoff-hz': 'lpf-cutoff-hz', 'slope-db': 'lpf-slope-db',
           'lag-rad': 'phase-lag-rad', 'headroom-rad': 'angular-ceiling-rad', 'ceiling-rad': 'angular-ceiling-rad',
           'velocity-sensitivity': 'velocity-gain', 'duck-depth-mm': 'modulation-depth-mm',
           'key-input': 'trigger-source', 'saturation-mode': 'saturation'}
SPLIT = {'stance-duty-pwm': ('duty-factor-l', 'duty-factor-r')}
#: The runway's transient shaper's params that land on its soft-clip node, the knee.
CLIP = ('stance-yield-rad', 'compliance')
#: The runway's filter bank: each filter modulates its :target's params.
FILTERS = ('dc-bias', 'butterworth-lpf', 'phase-delay', 'hard-mute')
#: Her voices' slots.
Z, K, D, AMP, PHASE, N = ortho.Z, ortho.K, ortho.D, ortho.AMP, ortho.PHASE, ortho.N


def _node(key):
    return NODE_ALIASES.get(key.lstrip(':'), key.lstrip(':'))


def _pairs(args):
    """[(param, form)] of `:key form ..` args, the suite's names."""
    if len(args) % 2 or not all(isinstance(k, str) and k.startswith(':') for k in args[::2]):
        raise MachineError('keyword pairs wanted, %r found' % (args,))
    return [(ALIASES.get(str(k)[1:], str(k)[1:]), form) for k, form in zip(args[::2], args[1::2])]


def _plist(args, env):
    return {k: value(form, env) for k, form in _pairs(args)}


def _applied(old, v, where):
    try:
        return OPS[v.name](old, v.value) if isinstance(v, Op) else v
    except TypeError as error:
        raise MachineError('%s: (:%s %r) of %r' % (where, v.name, v.value, old)) from error


def _modulated(g, node, said):
    """A copy of gait `g`, `node`'s params `said` ({param: value or Op}) applied."""
    g = copy.deepcopy(g)
    at = g.setdefault(node, {})
    for param, v in said.items():
        for p in SPLIT.get(param, (param,)):
            at[p] = _applied(at.get(p), v, '%s.%s' % (node, p))
    return g


def _sidechain(g, args, env):
    said = _plist(args, env)
    node = _node(said.pop('target-node', said.pop('target', ':com-dynamics')))
    g = copy.deepcopy(g)
    held = g.setdefault(node, {}).setdefault('sidechain', {})
    held.update({p: _applied(held.get(p), v, '%s.sidechain.%s' % (node, p)) for p, v in said.items()})
    return g


def _transients(g, args, env):
    said = _plist(args, env)
    target, clip = _node(said.pop('target')), _node(said.pop('soft-clip-node', ':tibiofemoral'))
    g = _modulated(g, clip, {k: v for k, v in said.items() if k in CLIP})
    return _modulated(g, target, {k: v for k, v in said.items() if k not in CLIP})


def _rebalanced(g, args, env):
    for said in _plist(args, env).values():
        g = _modulated(g, _node(said.pop('node')), said)
    return g


def _filtered(g, args, env):
    for form in args:
        if form[0] not in FILTERS:
            raise MachineError('(%s ..): a filter bank has %s' % (form[0], ', '.join(FILTERS)))
        said = _plist(form[1:], env)
        g = _modulated(g, _node(said.pop('target')), said)
    return g


def _on(node):
    return lambda g, a, env: _modulated(g, node, _plist(a, env))


#: The ops on a gait: (gait, raw args, env) -> a new gait.
GAIT_OPS = {
    'modulate-oscillator': _on('oscillator'), 'quantize-grid': _on('oscillator'),
    'set-plantar-boundary': _on('plantar-boundary'), 'modulate-plantar-boundary': _on('plantar-boundary'),
    'modulate-node': lambda g, a, env: _modulated(g, _node(a[0]), _plist(a[1:], env)),
    'enforce-wrench-cone': _on('wrench-cone'),
    'route-envelope-sidechain': _sidechain, 'route-sidechain': _sidechain,
    'shape-transients': _transients, 'mid-side-rebalance': _rebalanced, 'apply-filter-bank': _filtered,
}


def _param(g, key):
    """A gait's param by its name alone, the one node that has it."""
    key = ALIASES.get(key.lstrip(':'), key.lstrip(':'))
    found = [params[key] for node, params in g.items() if node != '_trace' and key in params]
    if len(found) != 1:
        raise MachineError('(get-param gait :%s): %d nodes have it' % (key, len(found)))
    return found[0]


#: The lisp's functions, their arguments evaluated.
FUNCS = {'+': lambda *a: sum(a), '-': lambda a, *b: a - sum(b) if b else -a, '*': lambda *a: math.prod(a),
         '/': operator.truediv, 'abs': abs, 'max': max, 'min': min, 'not': operator.not_,
         '>': operator.gt, '<': operator.lt, '>=': operator.ge, '<=': operator.le, 'eq': operator.eq,
         'get-param': _param, 'bars': lambda b0, b1, item: (b0, b1, item),
         **{name: (lambda p: lambda g: g['oscillator'][p])(p) for name, p in (
             ('get-bpm', 'bpm'), ('get-phase-diff', 'inter-limb-phase-rad'), ('get-duty-l', 'duty-factor-l'),
             ('get-duty-r', 'duty-factor-r'))},
         'compile-to-soa': lambda *a: {k.lstrip(':'): v for k, v in zip(a[::2], a[1::2])}}


def _let(args, env):
    env = dict(env)
    names = args[0]
    for name, form in zip(names[::2], names[1::2]):
        env[name] = value(form, env)
    return [value(form, env) for form in args[1:]][-1]


def _case(args, env):
    key = value(args[0], env)
    for clause in args[1:]:
        if clause[0] in ('otherwise', key) or (isinstance(clause[0], list) and key in clause[0]):
            return value(clause[1], env)
    raise MachineError('(case %s ..): no clause for %r' % (args[0], key))


def _threaded(args, env):
    g = value(args[0], env)
    for form in args[1:]:
        g = called(form[0], [g], form[1:], env)
    return g


#: The lisp's special forms, their arguments unevaluated.
SPECIAL = {'let': _let, 'case': _case, '->': _threaded,
           'if': lambda a, env: value(a[1] if value(a[0], env) else (a[2] if len(a) > 2 else None), env),
           'and': lambda a, env: all(value(x, env) for x in a), 'or': lambda a, env: any(value(x, env) for x in a)}


def value(x, env):
    """`x` evaluated in `env`: a vector's items (a ramp `[a -> b]` its ends), a symbol's binding,
    a form by its head - a special form, an op, a keyword list, a function, a stage called."""
    if isinstance(x, tuple):
        return tuple(value(e, env) for e in x if e != '->')
    if isinstance(x, str) and not x.startswith(':'):
        if x not in env:
            raise MachineError('%s: not defined' % x)
        return env[x]
    if not isinstance(x, list) or not x:
        return x
    head = x[0]
    if head in SPECIAL:
        return SPECIAL[head](x[1:], env)
    if head.startswith(':'):
        return Op(head[1:], value(x[1], env)) if head[1:] in OPS else _plist(x, env)
    if head in FUNCS:
        return FUNCS[head](*(value(a, env) for a in x[1:]))
    lead = next((k for k, a in enumerate(x[1:]) if isinstance(a, str) and a.startswith(':')), len(x) - 1)
    return called(head, [value(a, env) for a in x[1:lead + 1]], x[lead + 1:], env)


def called(name, given, args, env):
    """Stage or op `name` called on `given` (its leading values, a gait first) and keyword `args`."""
    if name in GAIT_OPS:
        return GAIT_OPS[name](given[0], args, env)
    stage = env.get(name)
    if not isinstance(stage, Stage):
        raise MachineError('(%s ..): no stage, op or function of that name' % name)
    keys = dict(_pairs(args))
    unknown = set(keys) - set(stage.keys)
    if unknown or len(given) != len(stage.params):
        raise MachineError('(%s ..): %s, %d of %d given' % (name, ', '.join(sorted(unknown)) or 'its keys',
                                                            len(given), len(stage.params)))
    inner = dict(env)
    inner.update(zip(stage.params, given))
    inner.update({k: value(keys.get(k, form), env if k in keys else inner) for k, form in stage.keys.items()})
    if stage.name == 'transition':
        return _morph(stage.body, inner)
    g = [value(form, inner) for form in stage.body][-1]
    return dict(g, _trace=g.get('_trace', []) + [(name, {k: inner[k] for k in stage.keys})])


def _morph(ops, env):
    by = {op[0]: op[1:] for op in ops}
    ends = _plist(by.pop('endpoints'), env) if 'endpoints' in by else {}
    sync, clock = _plist(by.get('sync-trigger', []), env), _plist(by.get('pll-clock-slew', []), env)
    bands = [dict(_plist(band[1:], env), band=band[0]) for band in by.get('band-split-crossfade', [])]
    return Morph(ends.get('dry-source', env.get('source-state')), ends.get('wet-target', env.get('target-state')),
                 sync, clock, bands, sync.get('duration-bars', env.get('bars', 2.0)))


def _state(clauses):
    g = {}
    for clause in clauses:
        nodes = ([(clause[k], clause[k + 1]) for k in range(1, len(clause), 2)] if clause[0] == ':kinematics'
                 else [(clause[0], clause[1:])])
        for node, params in nodes:
            g[_node(node)] = {k: value(v, {}) for k, v in _pairs(params)}
    return dict(g, _trace=[])


def _stage(form, kind=None):
    params = list(form[2])
    cut = params.index('&key') if '&key' in params else len(params)
    keys = {k[0]: k[1] for k in params[cut + 1:]}
    return Stage(kind or form[1], tuple(params[:cut]), keys, form[3:])


#: What each top-level form defines: (env, form) -> {name: thing}.
DEFINE = {
    'defpackage': lambda env, f: {}, 'in-package': lambda env, f: {},
    'defconstants': lambda env, f: {f[1]: {k: v for k, v in _pairs(f[2:])}},
    'defstate': lambda env, f: {f[1]: _state(f[2:])},
    'deftransducer': lambda env, f: {f[1]: _stage(f)}, 'defmodifier': lambda env, f: {f[1]: _stage(f)},
    'deflimiter': lambda env, f: {f[1]: _stage(f)},
    'deftransition': lambda env, f: ({f[1]: _morph([['endpoints', *f[2]]] + list(f[3:]), env)}
                                     if f[2] and str(f[2][0]).startswith(':') else {f[1]: _stage(f, 'transition')}),
    'defpatch': lambda env, f: {f[1]: value(f[2], env)},
}


def read(paths):
    """{name: what it defines} of lisp files `paths` in order: constants as {param: value},
    states and patches as gaits, stages, transitions, a schedule as compile-to-soa's keys."""
    env = {}
    for path in paths:
        with open(path, encoding='utf-8') as f:
            for form in patch.forms(f.read()):
                if form[0] not in DEFINE:
                    raise MachineError('(%s ..): a patch file has %s' % (form[0], ', '.join(DEFINE)))
                env.update(DEFINE[form[0]](env, form))
    return env


def schedule(soa):
    """[(from bar, to bar, gait or Morph)] of a compile-to-soa's schedule."""
    return [tuple(seg) for seg in soa['schedule']]


#: A param's voice: (node, param, slot, rule); a param a rule moves its slot by (RULES).
VOICES = (
    ('oscillator', 'bpm', ortho.HZ, 'ratio'),
    ('com-dynamics', 'base-height-z-mm', Z + modal.AXIAL, 'drop'),
    ('com-dynamics', 'damping-ratio', D + modal.AXIAL, 'ratio'),
    ('tibiofemoral', 'rebound-q', D + modal.AXIAL, 'inverse'),
    ('tibiofemoral', 'compliance', K + modal.AXIAL, 'inverse'),
    ('tarsal-coronal', 'track-width-mm', Z + modal.WIDTH, 'width'),
    ('tarsal-sagittal', 'floor-clearance-z-mm', AMP + modal.LIFT, 'ratio'),
    ('iliac-coronal', 'lfo-amplitude-rad', AMP + modal.LIST, 'ratio'),
    ('c7-t12-thoracic', 'dc-offset-rad', Z + modal.PITCH, 'shift'),
    ('glenohumeral', 'gain-db', AMP + modal.ARMS, 'db'),
    ('glenohumeral', 'phase-shift-rad', PHASE + modal.ARMS, 'phase'),
    ('glenohumeral', 'phase-lag-rad', PHASE + modal.ARMS, 'lag'),
    ('com-dynamics', 'sidechain.modulation-depth-mm', ortho.DUCK.start, 'milli'),
    ('com-dynamics', 'sidechain.attack-ms', ortho.DUCK.start + 1, 'milli'),
    ('com-dynamics', 'sidechain.release-ms', ortho.DUCK.start + 2, 'milli'),
)
#: Rules (slot's value so far, value, the dry's, the mode's unit) -> the slot's value: 'ratio'
#: her walk's times the value over the dry's, 'inverse' times the dry's over it, 'shift' her
#: walk's plus the change in her unit, 'drop' a height lowered, mm, as her legs' shortening,
#: 'width' her width's share moved by the ratio (`stride.meets` floors it), 'db' a gain's
#: change, 'phase' and 'lag' a phase's, rad, 'milli' mm or ms as m or s.
RULES = {'ratio': lambda base, v, vd, unit: base * v / vd,
         'inverse': lambda base, v, vd, unit: base * vd / v,
         'shift': lambda base, v, vd, unit: base + (v - vd) / unit,
         'drop': lambda base, v, vd, unit: base + 1e-3 * (vd - v) / unit,
         'width': lambda base, v, vd, unit: (1.0 + base) * v / vd - 1.0,
         'db': lambda base, v, vd, unit: base * 10.0 ** ((v - vd) / 20.0),
         'phase': lambda base, v, vd, unit: base + (v - vd),
         'lag': lambda base, v, vd, unit: base - (v - vd),
         'milli': lambda base, v, vd, unit: 1e-3 * v}
#: A param her voices read off others: (node, param, its sources, how). Her lift is her swing's
#: height over the floor she steps to: stairs told raise it by their rise.
DERIVED = (
    ('tarsal-sagittal', 'floor-clearance-z-mm', ('swing-clearance-z-mm', 'step-elevation-z-mm'),
     lambda n: n['swing-clearance-z-mm'] - abs(n.get('step-elevation-z-mm', 0.0))),
)
#: Params whose unsaid value is a value: a lag unsaid is none.
UNSAID = {('glenohumeral', 'phase-lag-rad'): 0.0}
#: Params told her as commands, not voiced: her twist (`se2`) and her floor's grip (`grip`). A
#: stride is her capture point's: said as her drive's amplitude (x1.54) 713 -> 759 mm for 1 220
#: asked, and the catwalk take, a forward walk and a catwalk loop fell at 6.2-10.3 s.
TOLD = (('oscillator', 'direction-angle-rad'), ('oscillator', 'yaw-rate-radps'),
        ('tarsal-sagittal', 'stride-length-mm'), ('wrench-cone', 'mu-static'))
_SOLE, _GATE = "her shoe's sole (mjcf.SOLE_S); the rock stiffened for its transient felled her", 'her gate: 0.6 stands'
_CLOCK, _GUARD = 'her clock has no humanize: 1.1 ms a step', 'stride.guarded holds her swing off the standing sole'
_ARMS = 'her arms swing in pitch alone'
#: What has no voice of hers, and what she has instead (findings/sequencer.md); '*' a node's all.
VOICELESS = {
    ('oscillator', 'even-step-swing-offset'): 'her limbs half a turn apart: 0.0005 of a step measured',
    ('oscillator', 'phase-jitter-ms'): _CLOCK, ('oscillator', 'strip-jitter'): _CLOCK,
    ('oscillator', 'duty-factor-l'): _GATE, ('oscillator', 'duty-factor-r'): _GATE,
    ('plantar-boundary', '*'): "her shoe (mjcf.SOLE_S)", ('calcaneus', '*'): _SOLE,
    ('tarsal-sagittal', 'strike-angle-rad'): _SOLE,
    ('tarsal-sagittal', 'step-elevation-z-mm'): 'her floor, told (`floor.told_stairs`)',
    ('com-dynamics', 'topology'): "her pendulum's vault; a spring's (SLIP) refused",
    ('com-dynamics', 'vertical-oscillation-mm'): "her height's own: 22.7 mm a step",
    ('com-dynamics', 'sidechain.*'): 'her duck: a touchdown latches it, a linear attack, an exponential release',
    ('tibiofemoral', 'stance-yield-rad'): "her knee's give is her axial spring's under load (compliance)",
    ('tibiofemoral', 'work-mode'): "her drives' work", ('tibiofemoral', 'energy-injection-j'): "her drives' work",
    ('tarsal-coronal', 'swing-circumduction-mm'): _GUARD, ('tarsal-coronal', 'circumduction-window-phase'): _GUARD,
    ('tarsal-coronal', 'circumduction-shape'): _GUARD,
    ('tarsal-coronal', 'pan-spread'): "her landings' spread is her capture point's",
    ('iliac-coronal', 'waveform'): 'a sine',
    ('iliac-coronal', 'stereo-width'): "as the sway's 1.55 by a narrow track she widened to 230 mm and fell",
    ('l4-l5-lumbar', '*'): "her trunk has one pitch voice, the thoracic's",
    ('c7-t12-thoracic', 'q-factor'): "her posture's spring", ('c7-t12-thoracic', 'zmp-counter-bias-x-mm'): 'her law',
    ('c7-t12-thoracic', 'lpf-cutoff-hz'): "her trunk held on her posture's spring (1.59 Hz): softened by 2.2 of 8 Hz "
                                          'it bent 0.85 rad for 0.16', ('c7-t12-thoracic', 'lpf-slope-db'): 'her posture',
    ('glenohumeral', 'abduction-rad'): _ARMS, ('glenohumeral', 'elbow-flexion-rad'): _ARMS,
    ('c1-c2-atlas', '*'): 'no head voice', ('craniofacial', '*'): 'no face',
    ('wrench-cone', 'mu-kinetic'): 'her friction pyramid has one mu (wbc.MU)',
    ('wrench-cone', 'max-shear-slew-nps'): 'her friction pyramid has no slew',
}
#: Her walk's top speed ahead, m/s: past it her twist picks her jog (`se2.GAIT_BOXES`).
WALK_TOP = next(box[1] for box in se2.GAIT_BOXES if box[6] == 'walk')
#: A centre of mass on a spring, sinking onto its stance and flying between: a run's, a gallop's.
SLIP = (':ballistic-slip', ':double-peak-slip')
#: What she cannot do: (node, the test on its params and the dry's, why).
REFUSED = (
    ('oscillator', lambda n, d: n.get('inter-limb-phase-rad') != d.get('inter-limb-phase-rad'),
     'inter-limb-phase-rad: her legs half a turn apart on one clock'),
    ('oscillator', lambda n, d: min(n.get('duty-factor-l', 1.0), n.get('duty-factor-r', 1.0)) < 0.5,
     'a duty under 0.5: a flight phase'),
    ('oscillator', lambda n, d: n.get('duty-factor-l') != n.get('duty-factor-r'),
     'duty-factor-l and -r apart: one gate for both legs'),
    ('com-dynamics', lambda n, d: n.get('topology') in SLIP,
     'a spring-mass topology: a run sinks onto its stance and flies, her centre vaults'),
    ('tarsal-coronal', lambda n, d: n.get('crossover-weave') is True,
     'crossover-weave: her landings never cross'),
)


def _flat(g):
    """{(node, param): value} of a gait, a sidechain's params dotted, the derived ones beside."""
    out = {}
    for node, params in g.items():
        if node == '_trace':
            continue
        for k, v in params.items():
            out.update({(node, '%s.%s' % (k, j)): w for j, w in v.items()} if isinstance(v, dict) else {(node, k): v})
    for node, param, sources, how in DERIVED:
        at = g.get(node, {})
        out[(node, param)] = how(at) if sources[0] in at else None
    return out


def _covered(node, param):
    return ((node, param) in VOICELESS or (node, param.split('.')[0] + '.*') in VOICELESS
            or (node, '*') in VOICELESS or (node, param) in TOLD
            or any(n == node and p == param for n, p, _s, _r in VOICES)
            or any(n == node and param in s for n, _p, s, _h in DERIVED))


def walk(fr):
    """Her walk's row as her loop holds it: what the walk says (`gait.said`), her rest where it
    says nothing (`modal.rest`, of her statics `fr`)."""
    import numpy as np
    from machine import gait
    said = gait.said()
    return np.where(np.isnan(said), modal.rest(fr), said)


def voiced(g, dry, fr):
    """({slot: value} of her walk's row (`walk`) moved by gait `g`'s changes off gait `dry` -
    only the slots it moves -, {'twist': (vx, vy, wz) told her, within her walk's box ahead,
    'grip': her floor's mu told, 'asked': the twist asked where it is past the box})."""
    for node, test, why in REFUSED:
        if test(g.get(node, {}), dry.get(node, {})):
            raise MachineError('refused: ' + why)
    now, was = _flat(g), _flat(dry)
    changed = {k for k in set(now) | set(was) if now.get(k, UNSAID.get(k)) != was.get(k, UNSAID.get(k))}
    unknown = sorted('%s.%s' % k for k in changed if not _covered(*k))
    if unknown:
        raise MachineError('no voice of hers, none named: ' + ', '.join(unknown))
    std, row = walk(fr), {}
    for node, param, slot, rule in VOICES:
        v, vd = now.get((node, param)), was.get((node, param), UNSAID.get((node, param)))
        if v is None or v == vd:
            continue
        mode = slot % N if slot < 5 * N else 0
        row[slot] = RULES[rule](row.get(slot, float(std[slot])), v, vd, fr[modal.UNIT][mode])
    bad = sorted(slot for slot, v in row.items() if not math.isfinite(v))
    if bad:
        raise MachineError('voiced past her: slots %s' % bad)
    osc, dry_osc = g.get('oscillator', {}), dry.get('oscillator', {})
    stride_of = lambda x: x.get('tarsal-sagittal', {}).get('stride-length-mm', 1.0)
    speed = se2.WALK_SPEED * stride_of(g) / stride_of(dry) * osc.get('bpm', 1.0) / dry_osc.get('bpm', 1.0)
    heading = osc.get('direction-angle-rad', 0.0)
    asked = (speed * math.cos(heading), speed * math.sin(heading), osc.get('yaw-rate-radps', 0.0))
    told = {'twist': (min(asked[0], math.nextafter(WALK_TOP, 0.0)),) + asked[1:]}
    told.update({'grip': g['wrench-cone']['mu-static']} if 'wrench-cone' in g else {})
    told.update({'asked': asked} if told['twist'] != asked else {})
    return row, told


def morphed(m, dry, fr):
    """(rows, span s) of Morph `m` on her loop (`patch.morph`): each slot either end moves on its
    node's band, her clock from one end's Hz to the other's on the clock's slew."""
    src, _told = voiced(m.source, dry, fr)
    dst, _told = voiced(m.target, dry, fr)
    std = walk(fr)
    slots = (set(src) | set(dst)) - {ortho.HZ}
    src, dst = ({k: x.get(k, float(std[k])) for k in slots | {ortho.HZ}} for x in (src, dst))
    bands = {slot: (b['curve'].lstrip(':'), b['window-bars'][0], b['window-bars'][-1])
             for b in m.bands for node in b['nodes'] for n, _p, slot, _r in VOICES if n == _node(node) and slot in slots}
    loose = sorted({n for n, _p, slot, _r in VOICES if slot in slots - set(bands)})
    if loose:
        raise MachineError('voiced in no band of the morph: ' + ', '.join(loose))
    c = m.clock
    clock = (c['curve'].lstrip(':'), c['window-bars'][0], c['window-bars'][-1], src[ortho.HZ], dst[ortho.HZ])
    return patch.morph(src, dst, bands, clock, m.bars)
