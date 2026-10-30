"""The ortho-DSP suite's lisp (lisp/humanoid/suite/) evaluated: its states, stages and patches.

    got = dsp.read(dsp.SUITE)                    # every name the texts define, evaluated
    g = got['patch-sneaker-runway']              # a gait: {node: {param: value}}
    plan = dsp.schedule(got['compile-omnidirectional-suite'])  # [(from bar, to bar, gait or Morph)]

A gait is the suite's manifold as nodes of params, a sidechain on the node it ducks; the runway's
dialect (runway.lisp, `tools/sim/identify.py`'s) says the same by ALIASES. Stages - transducers,
modifiers, limiters, maneuvers, a patch with parameters - are functions of a gait threaded by `->`.
A gait keeps the stages and transitions it went through in
order ('_trace') and a maneuver's acts ('_events': an envelope, a contact's threshold and the acts
it releases). `machine.texts.voicing` plays a gait on her walk, `machine.texts.maneuver` a maneuver on her.
"""
import collections
import copy
import math
import operator
import struct
from typing import Any

from machine.texts import kept, ortho, patch
from machine.texts.kernel import KEYED, MECHANISMS, QUERIES
from machine.texts.packages import modules
from machine.errors import MachineError

#: The suite's modules, read each after those it uses; the dual-domain kernel's (docs/todo/protocol.md).
SUITE = modules('suite')
KERNEL = modules('kernel')
#: An op on a param, `(:scale 1.2)`; a bare value sets. Clamped, an unset param takes the bound;
#: scaled or added, it keeps the op for her own value.
Op = collections.namedtuple('Op', 'name value')
OPS = {'set': lambda old, v: v, 'scale': operator.mul, 'add': operator.add,
       'clamp-min': lambda old, v: v if old is None else max(old, v),
       'clamp-max': lambda old, v: v if old is None else min(old, v)}
#: A law or a datum her world resolves as she goes, `(:wrench-proportional :max-share 0.18)`,
#: `(:active-obstacle-handle-se3)`: its name and its params ({key: value}) or values (a tuple).
Law = collections.namedtuple('Law', 'name args')
#: Stage forms threaded from the gait they are given later: `(-> (lock-horizon-gaze) ..)`.
Pipeline = collections.namedtuple('Pipeline', 'forms env')
#: A stage: its parameters, its keys' default forms, its body; a transition's ops its body.
Stage = collections.namedtuple('Stage', 'name params keys body')
#: A function of the text's Common Lisp (`defun`): its lambda list, its body.
Defun = collections.namedtuple('Defun', 'name params body')
#: A lambda list's words.
LAMBDA = ('&optional', '&key')
#: A transition from a gait (to a gait): its sync trigger, its clock's slew, its bands, its bars,
#: its other acts as events.
Morph = collections.namedtuple('Morph', 'source target sync clock bands bars acts')
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
FILTERS = ('dc-bias', 'butterworth-lpf', 'phase-delay', 'hard-mute', 'compressor-gain', 'brickwall-limit')


def node(key):
    """A node's name, the suite's."""
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
        kept = isinstance(v, Op) and old is None and v.name in ('scale', 'add')
        return v if kept or not isinstance(v, Op) else OPS[v.name](old, v.value)
    except TypeError as error:
        raise MachineError('%s: (:%s %r) of %r' % (where, v.name, v.value, old)) from error


def _modulated(g, node, said):
    """A copy of gait `g`, `node`'s params `said` ({param: value or Op}) applied: that node copied,
    the rest - its trace's transitions among them - shared."""
    g = dict(g)
    at = g[node] = copy.deepcopy(g.get(node, {}))
    for param, v in said.items():
        for p in SPLIT.get(param, (param,)):
            at[p] = _applied(at.get(p), v, '%s.%s' % (node, p))
    return g


def _sidechain(g, args, env):
    said = _plist(args, env)
    at = node(said.pop('target-node', said.pop('target', ':com-dynamics')))
    g = dict(g)
    g[at] = copy.deepcopy(g.get(at, {}))
    held = g[at].setdefault('sidechain', {})
    held.update({p: _applied(held.get(p), v, '%s.sidechain.%s' % (at, p)) for p, v in said.items()})
    return g


def _transients(g, args, env):
    said = _plist(args, env)
    target, clip = node(said.pop('target')), node(said.pop('soft-clip-node', ':tibiofemoral'))
    g = _modulated(g, clip, {k: v for k, v in said.items() if k in CLIP})
    return _modulated(g, target, {k: v for k, v in said.items() if k not in CLIP})


def _rebalanced(g, args, env):
    for said in _plist(args, env).values():
        g = _modulated(g, node(said.pop('node')), said)
    return g


def _filtered(g, args, env):
    for form in args:
        if form[0] not in FILTERS:
            raise MachineError('(%s ..): a filter bank has %s' % (form[0], ', '.join(FILTERS)))
        said = _plist(form[1:], env)
        g = _modulated(g, node(said.pop('target')), said)
    return g


def _on(node):
    return lambda g, a, env: _modulated(g, node, _plist(a, env))


def _keyed(args):
    """How many of `args` lead as keyword pairs."""
    return next((k for k in range(0, len(args), 2) if not (isinstance(args[k], str) and args[k].startswith(':'))),
                len(args))


def _acts(args, env):
    """({param: value} of the leading keyword pairs, [the events of the act forms after them])."""
    lead = _keyed(args)
    for form in args[lead:]:
        if form[0] not in EVENTS:
            raise MachineError('(%s ..): an act is one of %s' % (form[0], ', '.join(EVENTS)))
    return _plist(args[:lead], env), [(form[0], *_acts(form[1:], env)) for form in args[lead:]]


def _event(op):
    return lambda g, a, env: dict(g, _events=g.get('_events', []) + [(op, *_acts(a, env))])


#: A maneuver's acts, each an event: (op, its params, the acts it releases).
EVENTS = tuple('''trigger-one-shot-envelope on-contact-threshold crossfade-support-manifold stage-sequential-manifold
    configure-contact-compliance on-manifold-equilibrium cascade-actuator-gates latch-current-encoders
    stage-coupled-pendulum absorb-and-rebound-manifold cascade-impedance-envelope seek-auxiliary-manifold'''.split())
#: The anti-spasm insert's ops (anti_spasm_supervisor.lisp), each its node's params: her loop's
#: (`machine.law.supervisor`).
GUARDS = {'apply-velocity-noise-gate': 'velocity-gate', 'filter-derivative-channel': 'derivative-channel',
          'limit-torque-transients': 'torque-transients', 'configure-jacobian-solver': 'jacobian-solver',
          'enforce-joint-boundary-buffer': 'joint-boundary', 'configure-contact-comparators': 'contact-comparators',
          'enforce-bumpless-handover': 'bumpless-handover', 'apply-zero-crossing-governor': 'zero-crossing',
          'monitor-energy-tank': 'energy-tank'}
def _targeted(g, args, env):
    said = _plist(args, env)
    return _modulated(g, node(said.pop('target')), said)


def _keyed_on(at, key):
    def op(g, args, env):
        said = _plist(args, env)
        g = dict(g)
        g[at] = dict(g.get(at, {}), **{str(said.pop(key)).lstrip(':'): said})
        return g
    return op


def _gated(op):
    """A gate: its forms threaded on the gait unless its condition is false, the nodes they moved
    held as gated by it (`_gates`) where her world decides it as she goes."""
    def gate(g, args, env):
        cond = value(args[0], env)
        if cond is False or cond is None:
            return g
        out = g
        for form in args[1:]:
            out = _into(out, form, env)
        moved = sorted(k for k in out if not k.startswith('_') and out[k] is not g.get(k))
        return dict(out, _gates=g.get('_gates', []) + [(op, cond, moved)])
    return gate


#: The ops on a gait: (gait, raw args, env) -> a new gait.
GAIT_OPS = {**{op: _event(op) for op in EVENTS}, **{op: _on(at) for op, at in GUARDS.items()},
    **{op: _on(at) for op, at in MECHANISMS.items()}, **{op: _keyed_on(*k) for op, k in KEYED.items()},
    **{op: _gated(op) for op in ('when-modifier', 'when-funnel-active', 'when-target-affordance-occluded',
                                 'when-funnel-transition')},
    'engage-sub-oscillator': _targeted, 'sample-and-hold-terrain': _targeted,
    'modulate-oscillator': _on('oscillator'), 'quantize-grid': _on('oscillator'),
    'set-plantar-boundary': _on('plantar-boundary'), 'modulate-plantar-boundary': _on('plantar-boundary'),
    'modulate-node': lambda g, a, env: _modulated(g, node(a[0]), _plist(a[1:], env)),
    'enforce-wrench-cone': _on('wrench-cone'),
    'route-envelope-sidechain': _sidechain, 'route-sidechain': _sidechain,
    'shape-transients': _transients, 'mid-side-rebalance': _rebalanced, 'apply-filter-bank': _filtered,
}


def _param(g, key):
    """A gait's param by its name alone, the one node that has it."""
    key = ALIASES.get(key.lstrip(':'), key.lstrip(':'))
    found = [params[key] for at, params in g.items() if not at.startswith('_') and key in params]
    if len(found) != 1:
        raise MachineError('(get-param gait :%s): %d nodes have it' % (key, len(found)))
    return found[0]


#: A value as a Common Lisp type holds it (`coerce`).
COERCE = {'single-float': lambda x: struct.unpack('<f', struct.pack('<f', x))[0], 'double-float': float}
#: The language's constants, every text's.
CONSTANTS = {'pi': math.pi}
#: The lisp's functions, their arguments evaluated; arithmetic on a Law of her world a Law.
FUNCS = {**{k: (lambda k, f: lambda *a: Law(k, a) if Law in map(type, a) else f(*a))(k, f) for k, f in {
             '+': lambda *a: sum(a), '-': lambda a, *b: a - sum(b) if b else -a, '*': lambda *a: math.prod(a),
             '/': operator.truediv, 'abs': abs, 'max': max, 'min': min}.items()}, 'not': operator.not_, 'tan': math.tan,
         'values': lambda *a: a, 'coerce': lambda x, kind: COERCE[kind](x), 'elt': lambda seq, k: seq[int(k)],
         '>': operator.gt, '<': operator.lt, '>=': operator.ge, '<=': operator.le, 'eq': operator.eq,
         'get-param': _param, 'bars': lambda b0, b1, item: (b0, b1, item), 'oneshot': lambda g: ('oneshot', g),
         'dwell-ms': lambda ms: ('dwell', 1e-3 * ms),
         **{name: (lambda p: lambda g: g['oscillator'][p])(p) for name, p in (
             ('get-bpm', 'bpm'), ('get-phase-diff', 'inter-limb-phase-rad'), ('get-duty-l', 'duty-factor-l'),
             ('get-duty-r', 'duty-factor-r'), ('get-duty', 'duty-factor-l'))},
         **{name: lambda *a: {k.lstrip(':'): v for k, v in zip(a[::2], a[1::2])}
            for name in ('compile-to-soa', 'engage-virtual-dissipative-load')},
         **{name: (lambda name: lambda *a: Law(name, a))(name)
            for name in QUERIES}}


def _bindings(names):
    """A let's (name, form) pairs, `[a 1 b 2]` or Common Lisp's `((a 1) (b 2))`."""
    return [tuple(b) for b in names] if isinstance(names, list) else list(zip(names[::2], names[1::2]))


def _bind(env, name, got):
    """`env` with `name` bound to `got`; a vector of names, `[a b c]`, each to its item of it."""
    env.update(zip(name, got) if isinstance(name, tuple) else ((name, got),))


def _let(args, env):
    env = dict(env)
    for name, form in _bindings(args[0]):
        _bind(env, name, value(form, env))
    return [value(form, env) for form in args[1:]][-1]


def _call(fn, args, env):
    """Defun `fn` on its evaluated `args`, positional; an optional one missing takes its default."""
    inner = dict(env)
    for k, p in enumerate(q for q in fn.params if q not in LAMBDA):
        name, default = (p[0], p[1]) if isinstance(p, list) else (p, None)
        inner[name] = args[k] if k < len(args) else value(default, inner)
    return [value(form, inner) for form in fn.body][-1]


def _clause(args, env):
    """A case's form its key picks."""
    key = value(args[0], env)
    for clause in args[1:]:
        if clause[0] in ('otherwise', key) or (isinstance(clause[0], list) and key in clause[0]):
            return clause[1]
    raise MachineError('(case %s ..): no clause for %r' % (args[0], key))


def _case(args, env):
    return value(_clause(args, env), env)


def _given(form, env):
    """([the values of a call's leading arguments], [its keyword args and forms after them])."""
    lead = next((k for k, a in enumerate(form[1:]) if isinstance(a, str) and a.startswith(':')), len(form) - 1)
    return [value(a, env) for a in form[1:lead + 1]], form[lead + 1:]


def _into(g, form, env):
    """Gait `g` through `form`: a stage or op called on it, a let's forms in its bindings, a case's
    clause, a thread of its own seed."""
    if form[0] == '->':
        return value(form, env)
    if form[0] in ('case', 'ecase'):
        return _into(g, _clause(form[1:], env), env)
    if form[0] == 'let':
        inner = dict(env)
        for name, bound in _bindings(form[1]):
            _bind(inner, name, value(bound, inner))
        for each in form[2:]:
            g = _into(g, each, inner)
        return g
    given, rest = ([], form[1:]) if form[0] in GAIT_OPS else _given(form, env)
    return _then(g, form[0], called(form[0], [g] + given, rest, env))


def _short(form, env):
    """Whether `form` calls a stage short of the gait it threads."""
    stage = env.get(form[0]) if isinstance(form, list) and form and isinstance(form[0], str) else None
    return isinstance(stage, Stage) and len(_given(form, env)[0]) < len(stage.params)


def _threaded(args, env):
    if _short(args[0], env):
        return Pipeline(tuple(args), env)
    g = value(args[0], env)
    for form in args[1:]:
        g = _into(g, form, env)
    return g


def run(pipeline, g):
    """Gait `g` through a pipeline's forms."""
    for form in pipeline.forms:
        g = _into(g, form, pipeline.env)
    return g


def _then(g, name, got):
    """Gait `g` after stage `name` gave `got`: that gait, or `g` with the transition it gave traced."""
    return dict(g, _trace=g['_trace'] + [(name, got)]) if isinstance(got, Morph) else got


def _datum(x):
    """A quoted form as data, its lists as tuples."""
    return tuple(_datum(e) for e in x) if isinstance(x, (list, tuple)) else x


def _pairs_of(args):
    return len(args) % 2 == 0 and _keyed(args) == len(args)


def _law_args(args, env):
    """A law's params as {key: value} where its args are keyword pairs, else their values."""
    return _plist(args, env) if _pairs_of(args) else tuple(value(a, env) for a in args)


def _quasi(x, env):
    """A quasiquoted form as data, its unquoted forms evaluated."""
    if isinstance(x, list) and x[:1] == ['unquote']:
        return value(x[1], env)
    return tuple(_quasi(e, env) for e in x) if isinstance(x, (list, tuple)) else x


#: The lisp's special forms, their arguments unevaluated.
SPECIAL = {'let': _let, 'let*': _let, 'case': _case, 'ecase': _case, '->': _threaded,
           'quote': lambda a, env: _datum(a[0]),
           'quasiquote': lambda a, env: _quasi(a[0], env),
           'if': lambda a, env: value(a[1] if value(a[0], env) else (a[2] if len(a) > 2 else None), env),
           'and': lambda a, env: all(value(x, env) for x in a),
           'or': lambda a, env: next(filter(None, (value(x, env) for x in a)), None)}


def value(x, env) -> Any:
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
    if not isinstance(head, str):
        return tuple(value(e, env) for e in x)
    if head in SPECIAL:
        return SPECIAL[head](x[1:], env)
    if head.startswith(':'):
        if head[1:] in OPS:
            return Op(head[1:], value(x[1], env))
        return _plist(x, env) if _pairs_of(x) else Law(head[1:], _law_args(x[1:], env))
    if head in FUNCS:
        return FUNCS[head](*(value(a, env) for a in x[1:]))
    if isinstance(env.get(head), Defun):
        return _call(env[head], [value(a, env) for a in x[1:]], env)
    return called(head, *_given(x, env), env)


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
        return _morph(stage.body, inner, stage.params)
    g = [value(form, inner) for form in stage.body][-1]
    return dict(g, _trace=g.get('_trace', []) + [(name, {k: inner[k] for k in stage.keys})])


#: A transition's own ops; any other of its ops is an act, an event.
MORPH_OPS = ('endpoints', 'sync-trigger', 'pll-clock-slew', 'band-split-crossfade')


def _morph(ops, env, params=()):
    """A transition's Morph, its ends its first two params' gaits where no endpoints say."""
    by = {op[0]: op[1:] for op in ops}
    ends = _plist(by.pop('endpoints'), env) if 'endpoints' in by else {}
    sync, clock = _plist(by.get('sync-trigger', []), env), _plist(by.get('pll-clock-slew', []), env)
    bands = [dict(_plist(band[1:], env), band=band[0]) for band in by.get('band-split-crossfade', [])]
    named = [env[p] for p in params] + [None, None]
    acts = [(op[0], *_acts(op[1:], env)) for op in ops if op[0] not in MORPH_OPS]
    return Morph(ends.get('dry-source', named[0]), ends.get('wet-target', named[1]),
                 sync, clock, bands, sync.get('duration-bars', env.get('bars', 2.0)), acts)


def _records(params):
    """Whether a clause's params are records, `(:kind :name params..)` each, one or more."""
    return len(params) > 0 and all(isinstance(p, list) and len(p) > 1 and str(p[1]).startswith(':') for p in params)


def _state(clauses):
    """A state's nodes of params; a clause of a single form, that form as data (a capsule
    schema's pairs); a clause of records, {name: params} (a posture atlas's, `defcontact-patch`'s)."""
    g = {}
    for clause in clauses:
        nodes = ([(clause[k], clause[k + 1]) for k in range(1, len(clause), 2)] if clause[0] == ':kinematics'
                 else [(clause[0], clause[1:])])
        for at, params in nodes:
            g[node(at)] = ({str(c[1]).lstrip(':'): _plist(c[2:], {}) for c in params} if _records(params)
                           else _datum(params[0]) if len(params) == 1
                           else {p: value(v, {}) for k, v in _pairs(params) for p in SPLIT.get(k, (k,))})
    return dict(g, _trace=[], _events=[])


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
    'deflimiter': lambda env, f: {f[1]: _stage(f)}, 'defmaneuver': lambda env, f: {f[1]: _stage(f)},
    'deftransition': lambda env, f: ({f[1]: _morph([['endpoints', *f[2]]] + list(f[3:]), env)}
                                     if f[2] and str(f[2][0]).startswith(':') else {f[1]: _stage(f, 'transition')}),
    'defpatch': lambda env, f: {f[1]: _stage(f) if isinstance(f[2], tuple) else value(f[2], env)},
    'defprimitive': lambda env, f: {f[1]: _stage(f)},
    'defcontact-patch': lambda env, f: {f[1]: {str(c[1]).lstrip(':'): _plist(c[2:], env) for c in f[2:]}},
    'define-residual-grammar': lambda env, f: {f[1]: tuple(_plist(c, env) for c in f[2:])},
    'defdaemon': lambda env, f: {f[1]: dict(_pairs(f[3:]), params=f[2])},
    'defparameter': lambda env, f: {f[1]: value(f[2], env)}, 'defun': lambda env, f: {f[1]: Defun(f[1], f[2], f[3:])},
}


#: The reads kept (`kept.loaded`): a session reads one suite five times over - a load of 59 texts'
#: 6 ms, against 145 ms to evaluate them.
_READ: dict = {}


def read(paths, texts=None):
    """{name: what it defines} of lisp files `paths` in order, each `texts`' ({key: text}) where given
    (`ortho.source`): constants as {param: value}, states and patches as gaits, stages, transitions,
    a schedule as compile-to-soa's keys, functions as Defuns. Each call its own copy, the same texts
    evaluated once a process."""
    try:
        sources = tuple((path, ortho.source(path, texts)) for path in paths)
    except (SyntaxError, OSError):  # a text missing: refused where the reading meets it, uncached
        return _evaluated((path, ortho.source(path, texts)) for path in paths)
    return kept.loaded(_READ, sources, _evaluated)


def _evaluated(sources):
    """The env of `sources` ((path, text) ..) in order: each form's definitions over the last's."""
    env: dict[str, Any] = dict(CONSTANTS)
    for _path, text in sources:
        for form in patch.forms(text):
            if form[0] not in DEFINE:
                raise MachineError('(%s ..): a patch file has %s' % (form[0], ', '.join(DEFINE)))
            env.update(DEFINE[form[0]](env, form))
    return env


def schedule(soa):
    """[(from bar, to bar, gait or Morph), ('oneshot', gait), ('dwell', s)] of a compile-to-soa's
    schedule."""
    return [tuple(seg) for seg in soa['schedule']]
