"""A traced program in C11 (`tools.cores.trace`): each statement a loop nest at its layout.

Its buffers laid in one workspace by when each lives (`layout`); an elementwise tree per element,
elementwise temps read once folded into their reader (`fused`), an einsum summed in the order numpy's own
result shows (`orders`), a matmul numpy's BLAS's (`wbc_matmul`), a MuJoCo function called by its
prototype, a gather or a scatter through separable index tables.

    source, info = emit.c(prog, 'sense')        # the C; each buffer's place, the blocks' spans
    source, info = emit.c(one, 'regressor', orders, batch=True)   # one state's program over n of them

    void name([long n, ]const double *in, double *out, double *w, double *const *ext,
              const wbc_blas_t *blas, const void *model, void *data, void (*const *call)(void))

The inputs one block in their order, the outputs one in theirs (`blocks`; over n states each
array's n together), the workspace, each extern's address, numpy's BLAS, her model and data, the
MuJoCo functions `prog.calls` names. A program with a solve an int: 1 where numpy raises
LinAlgError, nothing after it run; else 0.
"""
import itertools

import numpy as np

from tools.cores.layout import blocks, cstrides, fused, lay, leaves, same_ref
from tools.cores.orders import letters, ordered
from tools.cores.trace import Ref, Untraced

#: An elementwise node's C, by numpy's ufunc: a format over its children's C; 'power' C's pow (numpy's
#: npy_pow), 'power0' numpy's power at an exponent of stride 0 (`trace_np.power`).
C_OF = {'add': '(%s + %s)', 'subtract': '(%s - %s)', 'multiply': '(%s * %s)', 'divide': '(%s / %s)',
        'negative': '(-%s)', 'positive': '(%s)', 'sign': 'npsign(%s)', 'maximum': 'npmaximum(%s, %s)',
        'minimum': 'npminimum(%s, %s)', 'sqrt': 'sqrt(%s)', 'absolute': 'fabs(%s)', 'square': '(%s * %s)',
        'reciprocal': '(1.0 / %s)', 'power': 'pow(%s, %s)', 'power0': 'nppow(%s, %s)',
        'sin': 'sin(%s)', 'cos': 'cos(%s)', 'tan': 'tan(%s)', 'arcsin': 'asin(%s)', 'arccos': 'acos(%s)',
        'arctan': 'atan(%s)', 'arctan2': 'atan2(%s, %s)', 'exp': 'exp(%s)', 'log': 'log(%s)',
        'hypot': 'hypot(%s, %s)', 'floor': 'floor(%s)', 'ceil': 'ceil(%s)',
        'where': '(%s != 0.0 ? %s : %s)', 'clip': 'npclip(%s, %s, %s)', 'nan_to_num': 'npnan_to_num(%s)',
        'isnan': '(isnan(%s) ? 1.0 : 0.0)', 'greater': '(%s > %s ? 1.0 : 0.0)', 'greater_equal': '(%s >= %s ? 1.0 : 0.0)',
        'less': '(%s < %s ? 1.0 : 0.0)', 'less_equal': '(%s <= %s ? 1.0 : 0.0)', 'equal': '(%s == %s ? 1.0 : 0.0)',
        'not_equal': '(%s != %s ? 1.0 : 0.0)', 'logical_and': '(%s != 0.0 && %s != 0.0 ? 1.0 : 0.0)',
        'logical_or': '(%s != 0.0 || %s != 0.0 ? 1.0 : 0.0)', 'logical_not': '(%s == 0.0 ? 1.0 : 0.0)',
        'invert': '(%s == 0.0 ? 1.0 : 0.0)'}
def separable(T):
    """(base, terms) with T == base + sum over its axes of each term's (('stride', s) or ('table', t))
    at that axis's index - None where it is not."""
    T = np.asarray(T, dtype=np.int64)
    base = int(T.flat[0]) if T.size else 0
    terms, total = [], np.full(T.shape, base, dtype=np.int64)
    for d, n in enumerate(T.shape):
        line = T[(0,) * d + (slice(None),) + (0,) * (T.ndim - d - 1)] - base
        steps = np.diff(line)
        if n == 1 or (steps == steps[0]).all():
            terms.append(('stride', int(steps[0]) if n > 1 else 0))
        else:
            terms.append(('table', line))
        total = total + line.reshape((1,) * d + (n,) + (1,) * (T.ndim - d - 1))
    return (base, terms) if np.array_equal(total, T) else None


class Writer:

    """The C being written: its tables and its body, a statement at a time."""

    def __init__(self, laid):
        self.laid, self.tables, self.body, self.named = laid, [], [], {}
        self.given: list | None = None
        self.found: list = []

    def table(self, values, ctype):
        values = np.ascontiguousarray(values, dtype=np.int64 if ctype == 'long' else float)
        key = (ctype, values.shape, values.tobytes())
        if key not in self.named:
            name = self.named[key] = 't%d' % len(self.tables)
            fmt = (lambda x: '%d' % x) if ctype == 'long' else (lambda x: _lit(float(x)))
            items = ', '.join(fmt(x) for x in values.ravel())
            self.tables.append('static const %s %s[%d] = {%s};' % (ctype, name, max(1, values.size), items or '0'))
        return self.named[key]

    def at(self, buffer, offset, terms, idx):
        """A buffer's element at `offset` plus each axis's term at index `idx[d]`."""
        parts = [str(int(offset)) if offset else '']
        for (kind, x), i in zip(terms, idx):
            if kind == 'stride' and x:
                parts.append('%d * %s' % (x, i) if x != 1 else i)
            elif kind == 'table':
                parts.append('%s[%s]' % (self.table(x, 'long'), i))
        address = ' + '.join(p for p in parts if p) or '0'
        if buffer.role == 'in':
            return 'in%d[%s]' % (self.laid['in'][buffer.id], address)
        if buffer.role == 'extern':
            return 'e%d[%s]' % (self.laid['ext'][buffer.id], address)
        if buffer.id in self.laid['out']:
            return 'out%d[%s]' % (self.laid['out'][buffer.id], address)
        return 'w[%d + %s]' % (self.laid['at'][buffer.id], address)

    def pointer(self, r):
        """The address of a Ref's first element."""
        b = r.buffer
        if b.role == 'in':
            return 'in%d + %d' % (self.laid['in'][b.id], r.offset)
        if b.role == 'extern':
            return 'e%d + %d' % (self.laid['ext'][b.id], r.offset)
        if b.id in self.laid['out']:
            return 'out%d + %d' % (self.laid['out'][b.id], r.offset)
        return 'w + %d' % (self.laid['at'][b.id] + r.offset)

    def ref(self, r, idx):
        return self.at(r.buffer, r.offset, [('stride', s) for s in r.strides], idx)

    def loops(self, shape, inner):
        """`inner` (C lines over index names i0..) within a loop over each axis of `shape`."""
        idx = ['i%d' % d for d in range(len(shape))]
        lines, pad = [], '  '
        for d, n in enumerate(shape):
            if n == 1:
                lines.append(pad + '{ const long i%d = 0;' % d)
            else:
                lines.append(pad + 'for (long i%d = 0; i%d < %d; i%d++) {' % (d, d, n, d))
            pad += '  '
        lines += [pad + x for x in inner(idx)]
        lines += ['  ' + '  ' * d + '}' for d in reversed(range(len(shape)))]
        return lines


def _lit(x):
    """A double's exact C literal."""
    if x != x:
        return '(0.0 / 0.0)'
    if x in (float('inf'), float('-inf')):
        return '(%s1.0 / 0.0)' % ('-' if x < 0 else '')
    return '%s' % float.hex(x) if x != 0.0 or np.signbit(x) is False else ('-0.0' if np.signbit(x) else '0.0')


def _tree(w, node, idx):
    kind = node[0]
    if kind == 'ref':
        return w.ref(node[1], idx)
    if kind == 'scalar':
        return _lit(node[1])
    if kind == 'const':
        arr, strides = node[1], node[2]
        return '%s[%s]' % (w.table(arr, 'double'), ' + '.join(['0'] + ['%d * %s' % (s, i) for s, i in zip(strides, idx) if s]))
    if kind not in C_OF:
        raise Untraced('np.%s in C' % kind)
    kids = [_tree(w, k, idx) for k in node[1:]]
    if kind == 'square':
        kids = kids * 2
    return C_OF[kind] % tuple(kids)


def _einsum(w, st, order):
    """An einsum in `order` (`orders.ordered`): each run along its inner summed letter in its lanes,
    added to the output in turn over the outer ones, unrolled."""
    (perm, inner), (ops, out, _c), refs = order, letters(st), st.args[1]
    sizes = dict(zip(''.join(ops), sum((list(np.shape(v)) for v in st.args[2]), [])))
    if any(isinstance(r, tuple) for r in refs):
        raise Untraced('einsum on a constant operand (%s)' % st.where)

    def run(place):
        terms = []
        for j in range(sizes[perm[-1]]):
            at = dict(place, **{perm[-1]: str(j)})
            factors = [w.ref(r, [at[x] for x in lets]) for lets, r in zip(ops, refs)]
            terms.append(' * '.join(factors) if len(factors) < 3 else '(' + ' * '.join(factors[:2]) + ') * ' + factors[2])
        return _summed(terms, inner)

    def element(idx):
        place, outer = dict(zip(out, idx)), perm[:-1]
        if not outer:
            got = run(place)
            return ['%s = %s;' % (w.ref(st.out, idx), got if inner == 'seq' else '0.0 + ' + got)]
        acc = '0.0'
        for combo in itertools.product(*[range(sizes[x]) for x in outer]):
            acc = '(%s + %s)' % (acc, run(dict(place, **{x: str(v) for x, v in zip(outer, combo)})))
        return ['%s = %s;' % (w.ref(st.out, idx), acc)]
    return w.loops(st.out.shape, element)


def _summed(terms, order):
    """A run of terms summed in `order` (`orders.summed`): a chain from zero, or SIMD lanes across."""
    if order == 'seq':
        expr = '0.0'
        for t in terms:
            expr = '(%s + %s)' % (expr, t)
        return expr
    v, J = int(order[5:]), len(terms)
    lanes, t = ['0.0'] * v, 0
    while J - t >= 4 * v:
        lanes = ['((((%s + %s) + %s) + %s) + %s)' % (lanes[k], terms[t + 3 * v + k], terms[t + 2 * v + k],
                                                   terms[t + v + k], terms[t + k]) for k in range(v)]
        t += 4 * v
    while t < J:
        lanes = ['(%s + %s)' % (lanes[k], terms[t + k] if t + k < J else '0.0') for k in range(v)]
        t += v
    return '(%s + %s)' % (lanes[0], lanes[1]) if v == 2 else '((%s + %s) + (%s + %s))' % (
        lanes[0], lanes[2], lanes[1], lanes[3])


def _reduce(w, _prog, st):
    """numpy's reduction in its own order: the source's axes outer to inner by stride, neighbours its
    iterator coalesces one; along an innermost reduced run a sum pairwise (npsum_s), else each in
    turn; a sum from zero, a product from one (or `initial`), min and max from the first."""
    op, src, axes, initial = st.args
    out, nd = st.out, len(src.shape)
    oaxes = [d for d in range(nd) if d not in axes] if len(out.shape) < nd else list(range(nd))
    ostride = [0] * nd
    for d, step in zip(oaxes, out.strides):
        ostride[d] = 0 if d in axes else step
    groups = []
    for d in sorted((d for d in range(nd) if src.shape[d] > 1), key=lambda d: (-src.strides[d], d)):
        n, step, ostep, cut = src.shape[d], src.strides[d], ostride[d], d in axes
        if groups and groups[-1][3] == cut and groups[-1][1] == step * n and groups[-1][2] == ostep * n:
            groups[-1] = [groups[-1][0] * n, step, ostep, cut]
        else:
            groups.append([n, step, ostep, cut])

    def first(idx):
        if op in ('add', 'multiply') or initial is not None:
            start = {'add': 0.0, 'multiply': 1.0}.get(op) if initial is None else float(initial)
            return ['%s = %s;' % (w.ref(out, idx), _lit(start))]
        at = ['0'] * nd
        for d, i in zip(oaxes, idx):
            at[d] = '0' if d in axes else i
        return ['%s = %s;' % (w.ref(out, idx), w.ref(src, at))]
    lines = w.loops(out.shape, first)
    groups = groups or [[1, 0, 0, True]]
    idx = ['g%d' % k for k in range(len(groups))]

    def place(r, k, which):
        return w.at(r.buffer, r.offset, [('stride', g[which]) for g in groups[:k]], idx[:k])
    pad, k = '  ', len(groups) - 1
    for j, (n, _s, _o, _c) in enumerate(groups[:-1]):
        lines.append(pad + 'for (long g%d = 0; g%d < %d; g%d++) {' % (j, j, n, j))
        pad += '  '
    n, step, _o, cut = groups[-1]
    if cut and op == 'add':
        o = place(out, k, 2)
        lines.append(pad + '%s = %s + npsum_s(&%s, %d, %d);' % (o, o, place(src, k, 1), n, step))
    else:
        o, x = place(out, k + 1, 2), place(src, k + 1, 1)
        rule = {'add': '%s + %s', 'multiply': '%s * %s', 'minimum': 'npminimum(%s, %s)',
                'maximum': 'npmaximum(%s, %s)'}[op]
        lines += [pad + 'for (long g%d = 0; g%d < %d; g%d++) {' % (k, k, n, k), pad + '  %s = %s;' % (o, rule % (o, x)),
                  pad + '}']
    for j in reversed(range(len(groups) - 1)):
        lines.append('  ' + '  ' * j + '}')
    return lines


def _viewed(w, x, strides):
    """A side's wbc_view_t: a Ref's first element, or a constant's table at its first element's place."""
    at = w.pointer(x) if not isinstance(x, tuple) else w.table(x[1], 'double') + (
        ' + %d' % x[2] if len(x) > 2 and x[2] else '')
    return '(wbc_view_t){%s, %d, %d}' % (at, strides[0], strides[1])


def _matmul(w, _prog, st):
    """numpy's a @ b: `wbc_matmul` on its operands' views, numpy's dispatch and BLAS."""
    (a, sa), (b, sb), (om, op), (m, n, p) = st.args
    return ['  wbc_matmul(blas, %s, %s, %s, %d, %d, %d, %d, %d);' % (_viewed(w, a, sa), _viewed(w, b, sb),
                                                                   w.pointer(st.out), om, op, m, n, p)]


def _solved(w, _prog, st):
    """linalg's solve, or inv where b is None: `wbc_solve` or `wbc_inv`, numpy's dgesv on Fortran-ordered
    copies, into its C-ordered result - where numpy raises LinAlgError the function's 1, nothing
    after it run."""
    (a, sa), b, n, nrhs, scratch = st.args
    call = ('wbc_inv(blas, %s, %d, %s, %s)' % (_viewed(w, a, sa), n, w.pointer(st.out), w.pointer(scratch))
            if b is None else 'wbc_solve(blas, %s, %s, %d, %d, %s, %s)' % (
                _viewed(w, a, sa), _viewed(w, *b), n, nrhs, w.pointer(st.out), w.pointer(scratch)))
    return ['  if (%s)' % call, '  {', '    return 1;', '  }']


def _fill(w, _prog, st):
    b = st.out.buffer
    place = 'out%d' % w.laid['out'][b.id] if b.id in w.laid['out'] else 'w + %d' % w.laid['at'][b.id]
    return ['  memset(%s, 0, %d * sizeof(double));' % (place, b.size)]


def _elementwise(w, _prog, st):
    """Each element of its tree: refused where it reads what it writes but at another element."""
    written = None
    for leaf in leaves(st.args):
        if leaf[1].buffer is st.out.buffer and not same_ref(leaf[1], st.out):
            written = st.out.addresses() if written is None else written
            read = leaf[1].addresses()
            if np.isin(read, written).any() and not np.array_equal(read, written):
                raise Untraced('an elementwise write over what it reads, elsewhere in it (%s)' % st.where)
    return w.loops(st.out.shape, lambda idx: ['%s = %s;' % (w.ref(st.out, idx), _tree(w, st.args, idx))])


def _contracted(w, _prog, st):
    order = w.given[len(w.found)] if w.given is not None else ordered(st)
    w.found.append(order)
    return _einsum(w, st, order)


def _gathered(w, _prog, st):
    src, table = st.args
    base, terms = _split(table, st.where)
    return _copy(w, st.out.shape, (st.out.buffer, st.out.offset, [('stride', s) for s in st.out.strides]),
                 (src, base, terms))


def _scattered(w, _prog, st):
    table, src, pick = st.args
    base, terms = _split(table, st.where)
    if src is st.out.buffer:
        raise Untraced('a scatter from its own buffer (%s)' % st.where)
    from_ = (pick,) if src is None else (src,) + tuple(_split(pick, st.where))
    return _copy(w, table.shape, (st.out.buffer, base, terms), from_)


def _called(w, prog, st):
    """A MuJoCo function through its prototype: her model, her data, buffers, NULL, ints."""
    name, kinds = st.args
    args = []
    for kind in kinds:
        if kind[0] in ('model', 'data'):
            args.append(kind[0])
        elif kind[0] == 'null':
            args.append('NULL')
        elif kind[0] == 'int':
            args.append('%d' % kind[1])
        else:
            if kind[1].buffer.role == 'in' and not kind[2]:
                raise Untraced('%s writes her input (%s)' % (name, st.where))
            args.append(w.pointer(kind[1]))
    return ['  ((%s_t)call[%d])(%s);' % (name, [c[0] for c in prog.calls].index(name), ', '.join(args))]


def _copy(w, shape, dst, src):
    """dst = src over `shape`, each (buffer, base, terms) or a constant (values,)."""
    def inner(idx):
        rhs = ('%s[%s]' % (w.table(src[0], 'double'), ' + '.join(['0'] + ['%d * %s' % (s, i) for s, i in
                                                                       zip(cstrides(shape), idx)]))
               if len(src) == 1 else w.at(src[0], src[1], src[2], idx))
        return ['%s = %s;' % (w.at(dst[0], dst[1], dst[2], idx), rhs)]
    return w.loops(shape, inner)


def _split(T, where):
    """(base, terms) of a Ref or a separable table."""
    if isinstance(T, Ref):
        return T.offset, [('stride', x) for x in T.strides]
    got = separable(T)
    if got is None:
        raise Untraced('an index table not separable by axis (%s)' % where)
    return got


#: Each statement's C by its kind: a writer of (the Writer, the program, the statement).
WRITERS = {'fill': _fill, 'ew': _elementwise, 'einsum': _contracted, 'gather': _gathered, 'scatter': _scattered,
           'matmul': _matmul, 'reduce': _reduce, 'mj': _called, 'solve': _solved}


def c(prog, name, orders=None, batch=False):
    """(C11 source of `void name(...)` (the module's), its layout: places, the workspace's doubles,
    each einsum's order, the input and output blocks' spans) - `orders` each einsum's where known,
    else read off its own values. `batch`: `prog` one state's, its arrays' first axis 1, the C
    `void name(long n, ...)` running it over n states, each its own slice of every input and output,
    its workspace reused."""
    statements = fused(prog)
    laid, top = lay(prog, statements)
    solves = any(st.kind == 'solve' for st in statements)
    w = Writer(laid)
    w.given = list(orders) if orders is not None else None
    for k, st in enumerate(statements):
        w.body.append('  /* %d %s %s */' % (k, st.kind, st.where.replace('*/', '* /')))
        if st.kind != 'mj' and st.out.buffer.role == 'in':
            raise Untraced('her code writes an input (%s)' % st.where)
        if st.kind == 'mj' or 0 not in (st.args[0].shape if st.kind == 'scatter' else st.out.shape):
            w.body += WRITERS[st.kind](w, prog, st)
        elif st.kind == 'einsum':
            w.found.append(None if w.given is None else w.given[len(w.found)])
    found = w.found
    for k, r in enumerate(prog.outs):
        if laid['out'].get(r.buffer.id) == k:
            continue
        w.body.append('  /* out %d */' % k)
        w.body += w.loops(r.shape, lambda idx, r=r, k=k: ['out%d[%s] = %s;' % (
            k, ' + '.join(['0'] + ['%d * %s' % (s, i) for s, i in zip(cstrides(r.shape), idx)]), w.ref(r, idx))])
    head = ['/* %s: generated by tools.cores.emit from her numpy, not by hand. */' % name,
            '#include <math.h>', '#include <string.h>', '#include "wbc_blas.h"', '#include "wbc_np.h"', '',
            '#ifdef _WIN32', '#define API __declspec(dllexport)', '#else', '#define API', '#endif', '']
    if prog.calls:
        head += ['typedef double mjtNum;', 'typedef struct mjModel_ mjModel;', 'typedef struct mjData_ mjData;']
        head += ['typedef void (*%s_t)(%s);' % call for call in prog.calls] + ['']
    head += w.tables + ['', 'API %s %s(%sconst double *in, double *out, double *w, double *const *ext, '
                        'const wbc_blas_t *blas, const void *model, void *data, void (*const *call)(void))'
                        % ('int' if solves else 'void', name, 'long n, ' if batch else ''), '{']
    done = ['  return 0;'] if solves else []
    ins, outs = blocks([b.root.shape for b in prog.ins]), blocks([r.shape for r in prog.outs])
    names = ['in%d' % k for k in range(len(prog.ins))] + ['out%d' % k for k in range(len(prog.outs))]
    if batch:
        if any(b.root.shape[:1] != (1,) for b in prog.ins) or any(r.shape[:1] != (1,) for r in prog.outs):
            raise Untraced('a batch of one state: every input and output its first axis 1')
        if prog.externs or prog.calls:
            raise Untraced('a batch of states on memory of one: externs, MuJoCo')
        head += ['  for (long s = 0; s < n; s++)', '  {']
        head += ['  const double *in%d = in + n * %d + s * %d;' % (k, a, b - a) for k, (a, b) in enumerate(ins[0])]
        head += ['  double *out%d = out + n * %d + s * %d;' % (k, a, b - a) for k, (a, b) in enumerate(outs[0])]
        head += ['  (void)%s;' % x for x in names]
        tail = ['  }'] + done + ['}', '']
    else:
        head += ['  const double *in%d = in + %d;' % (k, a) for k, (a, _b) in enumerate(ins[0])]
        head += ['  double *out%d = out + %d;' % (k, a) for k, (a, _b) in enumerate(outs[0])]
        head += ['  double *e%d = ext[%d];' % (k, k) for k in range(len(prog.externs))]
        head += ['  (void)%s;' % x for x in names + ['e%d' % k for k in range(len(prog.externs))]]
        tail = done + ['}', '']
    head += ['  (void)%s;' % x for x in ('w', 'ext', 'blas', 'model', 'data', 'call')]
    return '\n'.join(head + w.body + tail), {'laid': laid, 'doubles': top, 'orders': found, 'ins': ins, 'outs': outs,
                                             'solves': solves}
