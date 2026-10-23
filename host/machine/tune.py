"""The autotuner (meta_ast_autotuner.lisp) on her texts: its grammar, walk, reach and step.

    gr = tune.grammar()                              # {class: Residual}, the text's order
    texts = tune.texts()                             # {key: text}: every text her loop reads
    table = tune.walked('walk', gr, texts)           # [(Literal, class)] the root reaches
    cols, refused = tune.reach('walk', table, texts, fr)   # what each moves of her
    texts = tune.spliced(texts, lits, values, digits)  # values written at the literals' spans
    d = tune.step(J, r, scales)                      # the damped least-squares step, in their scales

Her texts are the suite's and her gym's, its includes (`ortho.sources`). A literal is a keyword's
number in the suite's dialect (`:compliance 0.80`) or an op's said of the keyword before its form
(`:stride-length-mm (:scale 0.82)`); in her gym's, a clause's number (`(stiffness MODE 3000)`), the
suite's name it is the one voicing reads into that clause's slot of her row (`ortho.SLOTS`,
`voicing.VOICES`): either where it stands. A root - a gait, a schedule, a stunt she plays, a
behaviour of her gym, her walk its streams (WHOLE) - reaches its definitions, those they name,
theirs. A literal moves her where her voices change as it does, by evaluation, nothing run: the
rest are inert, their Jacobian's columns zero. The step's numbers are the text's (`knobs`); what the
text names that her loop does another way is named (INSTEAD).
"""
import collections
import functools
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
#: A literal: its file and span, the definition it is in, the node or clause it is said of, its key
#: (the suite's name), the op it is of or None, its value, its slot of her row where her gym's.
Literal = collections.namedtuple('Literal', 'path start end symbol node key op value slot', defaults=(None,))
#: The text's names of the manifold's invariants (core_manifold.lisp's physical-invariants).
NAMES = {'min-normal-fz-ratio': 'min-normal-force-ratio'}
#: The text's primitives and daemon, each by what does it here.
PRIMITIVES = {'introspect-active-ast': 'tune.walked', 'compute-adjoint-ast-deltas': 'tune.step',
              'apply-homoiconic-rewrite': 'tune.spliced',
              'atomic-rcu-soa-swap': 'suite.played: a row laid on her walk, a morph at her lateral crossing, both soles locked',
              'sync-mutated-packages-to-disk': 'tune.written', 'universal-autotune-daemon': 'tools/sim/tune.py, a run a window'}
#: What the text names that her loop does another way, or not.
INSTEAD = {'adjoint-sign': "the measured Jacobian's signs; the text's are not read",
           'binding-deadband': "none said: a step under the text's float precision writes nothing",
           'substrate-eigenfreq-hz': "no substrate mode on a rigid floor; fitted, her vertical mode read her step's rate"}
#: A root that is several definitions: her walk, its streams as her loop composes them (`gait.said`).
WHOLE = {'walk': ('stand', 'sway', 'step', 'walk')}
#: The forms of her gym that are knobs: a behaviour's, a trait's, a derivation's, a stream's - not
#: her manifest's numbers nor a situation's caps.
KNOBS = ('defbehavior', 'deftrait', 'derive', 'stream_inject')
#: Where her gym's payload clauses begin: a definition's own, a stream's, a phase's.
PAYLOADS = KNOBS + ('phase',)
#: Her texts' names of her modes (`ortho.MODES`).
MODES = {'mode_' + m for m in ortho.MODES}
_NUMBER = re.compile(r'[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$')
_UNITS = re.compile(r'([-+]?(?:\d+\.?\d*|\.\d+))(ms|s|Hz|deg)$')


def _keyword(tok):
    return isinstance(tok, str) and tok.startswith(':')


def _numeral(tok):
    """A token's number as written, its unit cut (`108deg` -> `108`), or None."""
    m = _UNITS.match(tok)
    return m.group(1) if m else tok if _NUMBER.match(tok) else None


def texts():
    """{key: text} of every text her loop reads, as on disk: the suite's and her gym's."""
    return dict(ortho.sources(), **{path: ortho.source(path) for path in dsp.SUITE})


def _named(form):
    """A top-level form's name: a stream's by its sequence id (`sprung.STREAM_NAMES`), else its
    second word."""
    from machine import sprung
    if form[0] == 'stream_inject':
        ids = [int(x[1]) for x in form[1:] if isinstance(x, list) and x[:1] == ['sequence_id']]
        return sprung.STREAM_NAMES.get(ids[0], '#%d' % ids[0]) if ids else None
    return form[1] if len(form) > 1 and isinstance(form[1], str) else None


@functools.lru_cache(maxsize=None)
def _laid(clause):
    """The slots of her row a payload clause lays: compiled alone in a stream (`ortho.compiled`), its
    number 1, the parameters it says."""
    import numpy as np
    try:
        row = ortho.rows(ortho.compiled('(stream_inject (sequence_id 1) %s)' % clause)[0][''])[0]
    except (SyntaxError, ValueError):
        return ()
    return tuple(int(i) for i in np.flatnonzero(np.isfinite(np.array(row[4:], float))))


def _bound(slots):
    """(slot, the suite's name voicing reads into it - a derived param's source first) of one slot,
    else None."""
    voiced = [p for _n, p, s, _r in voicing.VOICES if (s,) == slots]
    derived = [src for _n, p, srcs, _h in voicing.DERIVED if p in voiced for src in srcs]
    names = [n.split('.')[-1] for n in derived + voiced]
    return (slots[0], names[0]) if names else None


def _tops(text):
    """[(the top-level form, its tokens (tok, start, end))] of a text."""
    out, depth, toks = [], 0, []
    for tok, start, end in patch.spans(text):
        toks.append((tok, start, end))
        depth += (tok in ('(', '[')) - (tok in (')', ']'))
        if depth == 0 and toks:
            out.append((patch.forms(text[toks[0][1]:end])[0] if toks[0][0] in ('(', '[') else None, toks))
            toks = []
    return [(form, toks) for form, toks in out if isinstance(form, list) and form]


def _suite_literals(path, name, toks):
    out, frames, said = [], [], None
    for tok, start, end in toks:
        if tok in ('(', '['):
            frames.append([said if _keyword(said) else None, None, 0, None])
        elif tok in (')', ']'):
            frames.pop()
        elif frames:
            owner, head, n, last = frames[-1]
            if _NUMBER.match(tok) and _keyword(last):
                key, op = (owner[1:], last[1:]) if last[1:] in dsp.OPS and n == 1 and owner else (last[1:], None)
                out.append(Literal(path, start, end, name, (owner or head or '').lstrip(':'),
                                   dsp.ALIASES.get(key, key), op, float(tok)))
            frames[-1] = [owner, head if n else tok, n + 1, tok]
        said = tok
    return out


def _gym_literals(path, name, toks):
    out, frames = [], []
    for tok, start, end in toks:
        if tok in ('(', '['):
            frames.append([])
        elif tok in (')', ']'):
            frames.pop()
        elif frames:
            frames[-1].append(tok)
            num = _numeral(tok)
            if num is None:
                continue
            heads = tuple(f[0] for f in frames)
            under = heads[max(i for i, h in enumerate(heads) if h in PAYLOADS) + 1:-1]
            clause = ''.join('(%s ' % h for h in under) + '(%s)' % ' '.join(frames[-1][:-1] + ['1' + tok[len(num):]]) + ')' * len(under)
            bound = _bound(_laid(clause))
            node = ' '.join(under + (heads[-1],) + tuple(t for t in frames[-1][1:-1] if t in MODES))
            out += [Literal(path, start, start + len(num), name, node, bound[1], None, float(num), bound[0])] if bound else []
    return out


def literals(path, text):
    """[Literal] of a text in its order: the suite's keyword numbers and ops' numbers; her gym's
    clauses' numbers in its knobs' forms (KNOBS), each the suite's name its slot is voiced by."""
    out = []
    for form, toks in _tops(text):
        out += (_gym_literals(path, _named(form), toks) if form[0] in KNOBS
                else _suite_literals(path, _named(form), toks))
    return out


def grammar(path=TEXT):
    """{class: Residual} of the text's define-residual-grammar in its order; a key two classes name
    refused."""
    form = next(f for f in patch.forms(ortho.source(path)) if f[0] == 'define-residual-grammar')
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
    return {(lit.symbol, lit.key): lit.value for lit in literals(path, ortho.source(path))}


def primitives(path=TEXT):
    """The names the text's defprimitive and defdaemon forms define, in its order."""
    return [f[1] for f in patch.forms(ortho.source(path)) if f[0] in ('defprimitive', 'defdaemon')]


def _atoms(x):
    return [x] if isinstance(x, str) else [a for e in x for a in _atoms(e)] if isinstance(x, (list, tuple)) else []


def starts(root):
    """The definitions `root` begins at: a stunt's patch and those of the stunts it rises from
    (`maneuver.PLAYED`), a whole's parts (WHOLE), else itself."""
    out = []
    while root in maneuver.PLAYED:
        out.append(maneuver.PLAYED[root][0])
        root = maneuver.PLAYED[root][1]
    return out or list(WHOLE.get(root, (root,)))


def reached(roots, texts):
    """The names of the definitions `roots` reach in `texts`: theirs, those they name, and so on."""
    defined = {}
    for text in texts.values():
        defined.update({_named(f): f for f in patch.forms(text) if isinstance(f, list) and f and _named(f)})
    missing = sorted(set(roots) - set(defined))
    if missing:
        raise MachineError('%s: no definition among her texts' % ', '.join(missing))
    seen, todo = set(), list(roots)
    while todo:
        name = todo.pop()
        if name in defined and name not in seen:
            seen.add(name)
            todo.extend(_atoms(defined[name][1:]))
    return seen


def walked(root, gr, texts):
    """The binding table: [(Literal, class)] of each literal in a definition `root` reaches whose key
    a class of `gr` names, in the texts' order - the master insert's not: its numbers are hers, said
    otherwise refused (`supervisor.checked`) -; past the text's max-nodes refused."""
    keys = {k: name for name, r in gr.items() for k in r.keys}
    names = reached(starts(root), texts) - reached(['master-anti-spasm-insert'], texts)
    table = [(lit, keys[lit.key]) for path, text in texts.items() for lit in literals(path, text)
             if lit.symbol in names and lit.key in keys]
    most = knobs()[('introspect-active-ast', 'max-nodes')]
    if len(table) > most:
        raise MachineError('%s: %d literals, the table holds %d' % (root, len(table), most))
    return table


def kind(root, got, gym=()):
    """What `root` is: a 'stunt' she plays (`maneuver.PLAYED`), a 'schedule' (a compile-to-soa's), a
    'gait' of the suite (a state or patch: a trace), a 'behaviour' among her gym's entries `gym`;
    anything else refused."""
    g = got.get(root)
    found = next((name for name, holds in (('stunt', root in maneuver.PLAYED),
                                           ('schedule', isinstance(g, dict) and 'schedule' in g),
                                           ('gait', isinstance(g, dict) and '_trace' in g),
                                           ('behaviour', root in {e[0] for e in gym})) if holds), None)
    if found is None:
        raise MachineError('%s: no gait, schedule, stunt or behaviour of hers' % root)
    return found


def _phased(phases):
    out = {}
    for k, p in enumerate(phases):
        out.update({(k, 'ms'): p.ms, (k, 'blend'): p.blend, (k, 'zmp'): p.zmp})
        out.update({(k, 'target', mode): v for mode, v in p.targets.items()})
        out.update({(k, 'spring', mode, j): v for mode, kd in p.springs.items() for j, v in enumerate(kd)})
    return out


def _voiced(g, got, fr, gym):
    row, told = voicing.voiced(g, got['nominal-biped-manifold'], fr, gym)
    out: dict = {('slot', k): v for k, v in row.items()}
    out.update({('twist', k): v for k, v in enumerate(told['twist'])})
    out.update({('grip',): told['grip']} if 'grip' in told else {})
    return out


def _wired(wire):
    return {('row', k): float(v) for k, v in enumerate(v for row in ortho.ROW.iter_unpack(wire) for v in row)}


def _morphed(m, got, fr, gym):
    wire, span = voicing.morphed(m, got['nominal-biped-manifold'], fr, gym)
    out: dict = _wired(wire)
    out[('span',)] = span
    return out


def _segment(seg, got, fr, gym):
    """A schedule's segment voiced (a morph by its rows); a one-shot or a dwell, the gym's, refused."""
    if len(seg) != 3:
        raise MachineError("a %s: the gym's" % seg[0])
    return _morphed(seg[2], got, fr, gym) if isinstance(seg[2], dsp.Morph) else _voiced(seg[2], got, fr, gym)


def _scheduled(root, got, fr, gym):
    """A schedule's segments voiced in order to the first she cannot play, as it plays."""
    out = {}
    for k, seg in enumerate(dsp.schedule(got[root])):
        try:
            said = _segment(seg, got, fr, gym)
        except MachineError:
            break
        out.update({(k,) + key: v for key, v in said.items()})
    return out


def _behaved(root, got, fr, gym):
    """A behaviour's rows: her walk's row as her loop holds it (`voicing.walk`) where it is a whole of
    streams (WHOLE), else its entry's wire."""
    if root in WHOLE:
        return {('slot', k): float(v) for k, v in enumerate(voicing.walk(fr, gym))}
    return _wired({e[0]: e for e in gym}[root][1])


#: How a root's voices are read, by its kind (`kind`).
VOICED = {'stunt': lambda root, got, fr, gym: _phased(maneuver.phases(root, None, got)), 'schedule': _scheduled,
          'gait': lambda root, got, fr, gym: _voiced(got[root], got, fr, gym), 'behaviour': _behaved}


def voices(root, got, fr, gym):
    """{voice: value} of what `root` moves of her (VOICED), of the suite `got` and her gym's entries
    `gym`: a played stunt's phases (`maneuver.phases`), a schedule's segments, a gait's voiced row,
    twist and grip (`voicing.voiced`, her statics `fr`), a behaviour's rows."""
    return VOICED[kind(root, got, gym)](root, got, fr, gym)


def scale(lit, got, row):
    """A literal's scale: its size, or where larger the manifold's of its key (the suite's) or her
    row's in its slot (her gym's, `row` her walk's row as her loop holds it); a scaling op's its own,
    a ratio."""
    if lit.op == 'scale':
        return abs(lit.value)
    said = ([abs(float(row[lit.slot]))] if lit.slot is not None else
            [abs(v) for node, params in got['nominal-biped-manifold'].items() if not node.startswith('_')
             for k, v in params.items() if k == lit.key and isinstance(v, float)])
    return max([abs(lit.value)] + [s for s in said if math.isfinite(s)])


def _same(a, b):
    return a == b or (isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b))


def _slope(up, down, h):
    try:
        return (up - down) / (2.0 * h)
    except TypeError:
        return math.nan


def evaluated(texts):
    """(the suite, her gym's entries) of her texts `texts`: what her loop plays from them."""
    from machine import sprung
    got = dsp.read(dsp.SUITE, texts)
    return got, sprung.entries(texts=texts, suite=got)


def _voices_of(root, texts, lit, fr, gym):
    """`root`'s voices of `texts` moved at `lit`: her gym re-read where it is her gym's, else `gym`."""
    got, gym = evaluated(texts) if lit.slot is not None else (dsp.read(dsp.SUITE, texts), gym)
    return voices(root, got, fr, gym)


def reach(root, table, texts, fr):
    """({i: {voice: d voice / d literal}} of the literals of `table` that move her, {i: why} of those
    refused): literal i its scale times the text's trust region up and down, the suite and - for
    her gym's literals - her gym re-read, her voices compared, a voice there one way alone d nan.
    The rest are inert."""
    got, gym = evaluated(texts)
    base, row = voices(root, got, fr, gym), voicing.walk(fr, gym)
    trust = knobs()[('compute-adjoint-ast-deltas', 'trust-region-scale')]
    cols, refused = {}, {}
    for i, (lit, _cls) in enumerate(table):
        h = trust * scale(lit, got, row)
        if not h:
            refused[i] = 'no scale: 0, and 0 or none in the manifold or her row'
            continue
        try:
            up, down = (_voices_of(root, spliced(texts, [lit], [lit.value + s * h]), lit, fr, gym) for s in (1, -1))
        except (MachineError, SyntaxError, ValueError) as error:
            refused[i] = str(error)
            continue
        col = {k: _slope(up.get(k), down.get(k), h) for k in set(base) | set(up) | set(down) if not _same(up.get(k), down.get(k))}
        cols.update({i: col} if col else {})
    return cols, refused


def number(v, digits=None):
    """A literal's text, positional - her gym's reader takes no exponent: `digits` after the point,
    else the fewest that read back as it; its trailing zeros cut to one."""
    import numpy as np
    s = np.format_float_positional(float(v), trim='0') if digits is None else ('%.*f' % (int(digits), v)).rstrip('0')
    return s + '0' if s.endswith('.') else s


def spliced(texts, lits, values, digits=None):
    """`texts` with each literal of `lits` its value of `values` at its span (`number`'s)."""
    out = dict(texts)
    for lit, v in sorted(zip(lits, values), key=lambda pair: -pair[0].start):
        out[lit.path] = out[lit.path][:lit.start] + number(v, digits) + out[lit.path][lit.end:]
    return out


def given(state, base=None, digits=None):
    """Her texts (`base`, else `texts()`) with `state`'s literals ({(path, start, end): value}) spliced."""
    lits = [Literal(p, a, z, None, None, None, None, v) for (p, a, z), v in state.items()]
    return spliced(texts() if base is None else base, lits, [lit.value for lit in lits], digits)


def setting(arg):
    """((path, start, end), value) of a PATH:START:END=VALUE, a run's `--set`."""
    where, value = arg.rsplit('=', 1)
    path, start, end = where.rsplit(':', 2)
    return (path, int(start), int(end)), float(value)


def step(J, r, scales, held=(), shrink=1.0, slack=None):
    """Each literal's step against residuals `r` (`J` their Jacobian, a column a literal): the damped
    least squares min |r + J d|^2 + lambda |d|^2 in the literals' `scales`, each move within
    `shrink` of the text's trust region of its scale, no measure of `held` (rows of its Jacobian)
    growing by them past its `slack` (0 where none) (`qp.solve`; d = 0 always holds)."""
    import numpy as np
    from machine import qp
    k = knobs()
    s = np.asarray(scales, float)
    a = np.asarray(J, float) * s
    pairs = [(np.asarray(row, float) * s, b) for row, b in zip(held, [0.0] * len(held) if slack is None else slack)]
    rows, room = [row for row, _b in pairs if np.all(np.isfinite(row)) and np.any(row)], [
        b for row, b in pairs if np.all(np.isfinite(row)) and np.any(row)]
    n, trust = a.shape[1], shrink * k[('compute-adjoint-ast-deltas', 'trust-region-scale')]
    x = qp.solve(a.T @ a + k[('compute-adjoint-ast-deltas', 'damping-lambda')] * np.eye(n), a.T @ r,
                 np.vstack([np.eye(n), -np.eye(n)] + rows),
                 np.concatenate([np.full(2 * n, trust), room]))[0]
    return x * s


def written(texts):
    """[path] of each file of `texts` unlike its disk's, written whole and swapped in at once."""
    changed = [path for path, text in texts.items() if text != ortho.source(path)]
    for path in changed:
        with open(path + '.tmp', 'w', encoding='utf-8', newline='') as f:
            f.write(texts[path])
        os.replace(path + '.tmp', path)
    return changed
