"""The autotuner (meta_ast_autotuner.lisp) on the suite's texts: its grammar, walk, reach and step.

    gr = tune.grammar()                              # {class: Residual}, the text's order
    texts = tune.texts()                             # {path: text} of the suite as on disk
    table = tune.walked('patch-sneaker-runway', gr, texts)   # [(Literal, class)] the root reaches
    cols, refused = tune.reach('patch-sneaker-runway', table, texts, fr)  # what each moves of her
    texts = tune.spliced(texts, lits, values, digits)  # values written at the literals' spans
    d = tune.step(J, r, scales)                      # the damped least-squares step, in their scales

A literal is a keyword's number in a text (`:compliance 0.80`), or an op's said of the keyword
before its form (`:stride-length-mm (:scale 0.82)`), where it stands. A root reaches its own
definition, those it names, theirs; a stunt (`maneuver.PLAYED`) its patch and those of the stunts it
rises from. A literal moves her where her voices change as it does - a gait's row, twist and grip
(`voicing.voiced`), a played stunt's phases (`maneuver.phases`) -, by evaluation, nothing run: the
rest are inert, their Jacobian's columns zero. The step's numbers are the text's (`knobs`); what
the text names that her loop does another way is named (INSTEAD).
"""
import collections
import math
import os
import re

from machine import dsp, maneuver, ortho, patch, voicing
from machine.errors import MachineError

#: The autotuner's text.
TEXT = os.path.join(os.path.dirname(__file__), 'meta_ast_autotuner.lisp')
#: A residual class: its telemetry, the keys it matches (the suite's names), the text's sign, the
#: invariant bounding it or None.
Residual = collections.namedtuple('Residual', 'name telemetry keys sign invariant')
#: A literal: its file and span, the definition it is in, the node or stage it is said of, its key
#: (the suite's name), the op it is of or None, its value.
Literal = collections.namedtuple('Literal', 'path start end symbol node key op value')
#: The text's names of the manifold's invariants (core_manifold.lisp's physical-invariants).
NAMES = {'min-normal-fz-ratio': 'min-normal-force-ratio'}
#: The text's primitives and daemon, each by what does it here.
PRIMITIVES = {'introspect-active-ast': 'tune.walked', 'compute-adjoint-ast-deltas': 'tune.step',
              'apply-homoiconic-rewrite': 'tune.spliced',
              'atomic-rcu-soa-swap': 'suite.played: a row laid on her walk, a morph at her lateral crossing, both soles locked',
              'sync-mutated-packages-to-disk': 'tune.written', 'universal-autotune-daemon': 'tools/sim/tune.py, a run a window'}
#: What the text names that her loop does another way, or not.
INSTEAD = {'ortho-dsp.supervisor': "no module of the suite: her drives' energy is the world's (World.drawn)",
           'adjoint-sign': "the measured Jacobian's signs; the text's are not read",
           'binding-deadband': "none said: a step under the text's float precision writes nothing",
           'substrate-eigenfreq-hz': 'no substrate mode on a rigid floor',
           'energy-tank-bleed-joules': "her drives' energy a metre: her loop keeps no energy tank"}
_NUMBER = re.compile(r'[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$')


def _keyword(tok):
    return isinstance(tok, str) and tok.startswith(':')


def texts(paths=None):
    """{path: text} of the suite's files, or `paths`', as on disk."""
    return {path: dsp.text(path) for path in paths or dsp.SUITE}


def literals(path, text):
    """[Literal] of a text: each keyword's number, and each op's number said of the keyword before
    its form, in the text's order."""
    out, frames, symbol, said = [], [], None, None
    for tok, start, end in patch.spans(text):
        if tok in ('(', '['):
            frames.append([said if _keyword(said) else None, None, 0, None])
        elif tok in (')', ']'):
            frames.pop()
        elif frames:
            owner, head, n, last = frames[-1]
            symbol = tok if len(frames) == 1 and n == 1 else symbol
            if _NUMBER.match(tok) and _keyword(last):
                key, op = (owner[1:], last[1:]) if last[1:] in dsp.OPS and n == 1 and owner else (last[1:], None)
                out.append(Literal(path, start, end, symbol, (owner or head or '').lstrip(':'),
                                   dsp.ALIASES.get(key, key), op, float(tok)))
            frames[-1] = [owner, head if n else tok, n + 1, tok]
        said = tok
    return out


def grammar(path=TEXT):
    """{class: Residual} of the text's define-residual-grammar in its order; a key two classes name
    refused."""
    form = next(f for f in patch.forms(dsp.text(path)) if f[0] == 'define-residual-grammar')
    out, seen = {}, {}
    for clause in form[2:]:
        said = dict(zip(clause[::2], clause[1::2]))
        name = str(said[':residual-class'])[1:]
        keys = tuple(dsp.ALIASES.get(str(k)[1:], str(k)[1:]) for k in said[':match-ast-keys'])
        twice = sorted(set(keys) & set(seen))
        if twice:
            raise MachineError('%s: %s named by %s too' % (name, ', '.join(twice), seen[twice[0]]))
        seen.update(dict.fromkeys(keys, name))
        bound = said.get(':hard-invariant')
        out[name] = Residual(name, said[':telemetry-error'][1:], keys, said[':adjoint-sign'],
                             bound and NAMES.get(bound[1:], bound[1:]))
    return out


def knobs(path=TEXT):
    """{(definition, key): number} of the text's own numbers: its step's damping and trust region,
    its tolerance, its precision, its table's size."""
    return {(lit.symbol, lit.key): lit.value for lit in literals(path, dsp.text(path))}


def primitives(path=TEXT):
    """The names the text's defprimitive and defdaemon forms define, in its order."""
    return [f[1] for f in patch.forms(dsp.text(path)) if f[0] in ('defprimitive', 'defdaemon')]


def _atoms(x):
    return [x] if isinstance(x, str) else [a for e in x for a in _atoms(e)] if isinstance(x, (list, tuple)) else []


def starts(root):
    """The suite's names `root` begins at: a stunt's patch and those of the stunts it rises from
    (`maneuver.PLAYED`), else itself."""
    out = []
    while root in maneuver.PLAYED:
        out.append(maneuver.PLAYED[root][0])
        root = maneuver.PLAYED[root][1]
    return out or [root]


def reached(roots, texts):
    """The names of the definitions `roots` reach in `texts`: theirs, those they name, and so on."""
    defined = {f[1]: f for text in texts.values() for f in patch.forms(text) if len(f) > 1 and isinstance(f[1], str)}
    missing = sorted(set(roots) - set(defined))
    if missing:
        raise MachineError('%s: no definition of the suite' % ', '.join(missing))
    seen, todo = set(), list(roots)
    while todo:
        name = todo.pop()
        if name in defined and name not in seen:
            seen.add(name)
            todo.extend(_atoms(defined[name][2:]))
    return seen


def walked(root, gr, texts):
    """The binding table: [(Literal, class)] of each literal in a definition `root` reaches whose key
    a class of `gr` names, in the texts' order; past the text's max-nodes refused."""
    keys = {k: name for name, r in gr.items() for k in r.keys}
    names = reached(starts(root), texts)
    table = [(lit, keys[lit.key]) for path, text in texts.items() for lit in literals(path, text)
             if lit.symbol in names and lit.key in keys]
    most = knobs()[('introspect-active-ast', 'max-nodes')]
    if len(table) > most:
        raise MachineError('%s: %d literals, the table holds %d' % (root, len(table), most))
    return table


def kind(root, got):
    """What `root` is: 'stunt' she plays (`maneuver.PLAYED`), 'schedule' (a compile-to-soa's), 'gait'
    of the suite; anything else refused."""
    g = got.get(root)
    found = next((name for name, holds in (('stunt', root in maneuver.PLAYED),
                                           ('schedule', isinstance(g, dict) and 'schedule' in g),
                                           ('gait', isinstance(g, dict))) if holds), None)
    if found is None:
        raise MachineError('%s: no gait, schedule or stunt she plays' % root)
    return found


def _phased(phases):
    out = {}
    for k, p in enumerate(phases):
        out.update({(k, 'ms'): p.ms, (k, 'blend'): p.blend, (k, 'zmp'): p.zmp})
        out.update({(k, 'target', mode): v for mode, v in p.targets.items()})
        out.update({(k, 'spring', mode, j): v for mode, kd in p.springs.items() for j, v in enumerate(kd)})
    return out


def _voiced(g, got, fr):
    row, told = voicing.voiced(g, got['nominal-biped-manifold'], fr)
    out: dict = {('slot', k): v for k, v in row.items()}
    out.update({('twist', k): v for k, v in enumerate(told['twist'])})
    out.update({('grip',): told['grip']} if 'grip' in told else {})
    return out


def _morphed(m, got, fr):
    wire, span = voicing.morphed(m, got['nominal-biped-manifold'], fr)
    out: dict = {('row', k): float(v) for k, v in enumerate(v for row in ortho.ROW.iter_unpack(wire) for v in row)}
    out[('span',)] = span
    return out


def _scheduled(root, got, fr):
    """A schedule's segments voiced in order, a morph by its rows, to the first she cannot play -
    as it plays -: a refusal, a one-shot or a dwell (the gym's)."""
    out = {}
    for k, seg in enumerate(dsp.schedule(got[root])):
        try:
            said = (_morphed if isinstance(seg[-1], dsp.Morph) else _voiced)(seg[2], got, fr) if len(seg) == 3 else None
        except MachineError:
            said = None
        if said is None:
            break
        out.update({(k,) + key: v for key, v in said.items()})
    return out


#: How a root's voices are read, by its kind (`kind`).
VOICED = {'stunt': lambda root, got, fr: _phased(maneuver.phases(root, None, got)), 'schedule': _scheduled,
          'gait': lambda root, got, fr: _voiced(got[root], got, fr)}


def voices(root, got, fr):
    """{voice: value} of what `root` moves of her (VOICED): a played stunt's phases
    (`maneuver.phases`), a schedule's segments, a gait's voiced row, twist and grip (`voicing.voiced`,
    of her statics `fr`)."""
    return VOICED[kind(root, got)](root, got, fr)


def scale(lit, got):
    """A literal's scale: its size, or its key's in the suite's manifold where that is larger; a
    scaling op's its own, a ratio."""
    if lit.op == 'scale':
        return abs(lit.value)
    said = [abs(v) for node, params in got['nominal-biped-manifold'].items() if not node.startswith('_')
            for k, v in params.items() if k == lit.key and isinstance(v, float)]
    return max([abs(lit.value)] + said)


def _same(a, b):
    return a == b or (isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b))


def _slope(up, down, h):
    try:
        return (up - down) / (2.0 * h)
    except TypeError:
        return math.nan


def reach(root, table, texts, fr):
    """({i: {voice: d voice / d literal}} of the literals of `table` that move her, {i: why} of those
    refused): literal i its scale times the text's trust region up and down, the suite re-read, her
    voices compared - a voice there one way alone d nan. The rest are inert."""
    got = dsp.read(dsp.SUITE, texts)
    base = voices(root, got, fr)
    trust = knobs()[('compute-adjoint-ast-deltas', 'trust-region-scale')]
    cols, refused = {}, {}
    for i, (lit, _cls) in enumerate(table):
        h = trust * scale(lit, got)
        if not h:
            refused[i] = 'no scale: 0, and 0 or none in the manifold'
            continue
        try:
            up, down = (voices(root, dsp.read(dsp.SUITE, spliced(texts, [lit], [lit.value + s * h])), fr) for s in (1, -1))
        except MachineError as error:
            refused[i] = str(error)
            continue
        col = {k: _slope(up.get(k), down.get(k), h) for k in set(base) | set(up) | set(down) if not _same(up.get(k), down.get(k))}
        cols.update({i: col} if col else {})
    return cols, refused


def number(v, digits=None):
    """A literal's text: `digits` after the point, its trailing zeros cut to one; else in full."""
    if digits is None:
        return repr(float(v))
    s = ('%.*f' % (int(digits), v)).rstrip('0')
    return s + '0' if s.endswith('.') else s


def spliced(texts, lits, values, digits=None):
    """`texts` with each literal of `lits` its value of `values` at its span (`number`'s)."""
    out = dict(texts)
    for lit, v in sorted(zip(lits, values), key=lambda pair: -pair[0].start):
        out[lit.path] = out[lit.path][:lit.start] + number(v, digits) + out[lit.path][lit.end:]
    return out


def step(J, r, scales, held=(), shrink=1.0):
    """Each literal's step against residuals `r` (`J` their Jacobian, a column a literal): the damped
    least squares min |r + J d|^2 + lambda |d|^2 in the literals' `scales`, each move within
    `shrink` of the text's trust region of its scale, no measure of `held` (rows of its Jacobian)
    growing by them (`qp.solve`; d = 0 always holds)."""
    import numpy as np
    from machine import qp
    k = knobs()
    s = np.asarray(scales, float)
    a, rows = np.asarray(J, float) * s, [np.asarray(row, float) * s for row in held]
    rows = [row for row in rows if np.all(np.isfinite(row)) and np.any(row)]
    n, trust = a.shape[1], shrink * k[('compute-adjoint-ast-deltas', 'trust-region-scale')]
    x = qp.solve(a.T @ a + k[('compute-adjoint-ast-deltas', 'damping-lambda')] * np.eye(n), a.T @ r,
                 np.vstack([np.eye(n), -np.eye(n)] + rows),
                 np.concatenate([np.full(2 * n, trust), np.zeros(len(rows))]))[0]
    return x * s


def written(texts):
    """[path] of each file of `texts` unlike its disk's, written whole and swapped in at once."""
    changed = [path for path, text in texts.items() if text != dsp.text(path)]
    for path in changed:
        with open(path + '.tmp', 'w', encoding='utf-8', newline='') as f:
            f.write(texts[path])
        os.replace(path + '.tmp', path)
    return changed
