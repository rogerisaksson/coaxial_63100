"""A traced program in C11 (`tools.cores.trace`): each statement a loop nest at its layout.

Its buffers laid in one workspace by when each lives; an elementwise tree per element, elementwise
temps read once folded into their reader (`fused`), an einsum summed in the order numpy's own
result shows (`ordered`), a gather or a scatter through separable index tables.

    source, laid = emit.c(prog, 'regressor')    # the C, and each buffer's place in the workspace
    source, laid = emit.c(one, 'regressor', orders, batch=True)   # one state's program over n of them
"""
import numpy as np

from tools.cores.trace import Ref, Untraced

#: An elementwise node's C, by numpy's ufunc: a format over its children's C.
C_OF = {'add': '(%s + %s)', 'subtract': '(%s - %s)', 'multiply': '(%s * %s)', 'divide': '(%s / %s)',
        'negative': '(-%s)', 'positive': '(%s)', 'sign': 'npsign(%s)', 'maximum': 'npmax(%s, %s)',
        'minimum': 'npmin(%s, %s)', 'sqrt': 'sqrt(%s)', 'absolute': 'fabs(%s)', 'square': '(%s * %s)',
        'sin': 'sin(%s)', 'cos': 'cos(%s)', 'tan': 'tan(%s)', 'arcsin': 'asin(%s)', 'arccos': 'acos(%s)',
        'arctan': 'atan(%s)', 'arctan2': 'atan2(%s, %s)', 'exp': 'exp(%s)', 'log': 'log(%s)',
        'hypot': 'hypot(%s, %s)', 'floor': 'floor(%s)', 'ceil': 'ceil(%s)'}
#: The orders an einsum's terms may be summed in, as numpy 2's sum_of_products adds them: a plain
#: sum into its zeroed output, or SIMD lanes of 2 (SSE) or 4 (AVX2) summed across at the end.
ORDERS = ('seq', 'lanes2', 'lanes4')


def same(a, b):
    """Whether `a` and `b` hold the same doubles, bit for bit."""
    a, b = np.ascontiguousarray(a, dtype=float), np.ascontiguousarray(b, dtype=float)
    return a.shape == b.shape and np.array_equal(a.view(np.int64), b.view(np.int64))


def summed(P, order):
    """`P` (..., J)'s terms summed along their last axis in `order`, numpy's arithmetic."""
    J = P.shape[-1]
    if order == 'seq':
        acc = np.zeros(P.shape[:-1])
        for j in range(J):
            acc = acc + P[..., j]
        return acc
    v = int(order[5:])
    lanes = [np.zeros(P.shape[:-1]) for _ in range(v)]
    t = 0
    while J - t >= 4 * v:
        for k in range(v):
            lanes[k] = (((lanes[k] + P[..., t + 3 * v + k]) + P[..., t + 2 * v + k]) + P[..., t + v + k]) + P[..., t + k]
        t += 4 * v
    while t < J:
        for k in range(v):
            lanes[k] = lanes[k] + (P[..., t + k] if t + k < J else 0.0)
        t += v
    across = lanes[0] + lanes[1] if v == 2 else (lanes[0] + lanes[2]) + (lanes[1] + lanes[3])
    return 0.0 + across


def _letters(st):
    subscripts = st.args[0].replace(' ', '')
    ins, out = subscripts.split('->')
    ops = ins.split(',')
    contracted = sorted(set(''.join(ops)) - set(out))
    if len(contracted) != 1 or any(len(set(o)) != len(o) for o in ops):
        raise Untraced('einsum %r: one index summed, none repeated in an operand (%s)' % (subscripts, st.where))
    return ops, out, contracted[0]


def products(st):
    """The einsum's terms (out..., J), numpy's products left to right."""
    ops, out, c = _letters(st)
    full = out + c
    factors = []
    for letters, value in zip(ops, st.args[2]):
        axes = [letters.index(x) for x in full if x in letters]
        moved = np.transpose(value, axes)
        factors.append(moved.reshape([value.shape[letters.index(x)] if x in letters else 1 for x in full]))
    P = factors[0]
    for f in factors[1:]:
        P = P * f
    return P


def ordered(st):
    """The order numpy summed an einsum's terms in, as its result shows: the first of ORDERS its
    values are bit for bit; a plain sum where the terms cannot tell (one term, or zeros)."""
    P = products(st)
    found = [o for o in ORDERS if same(summed(P, o), st.value)]
    if not found:
        raise Untraced('einsum %r in no order numpy sums in (%s)' % (st.args[0], st.where))
    return 'seq' if 'seq' in found else found[0]


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
        if buffer.id in self.laid['out']:
            return 'out%d[%s]' % (self.laid['out'][buffer.id], address)
        return 'w[%d + %s]' % (self.laid['at'][buffer.id], address)

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
    ops, out, c = _letters(st)
    refs = st.args[1]
    J = dict(zip(''.join(ops), sum((list(np.shape(v)) for v in st.args[2]), [])))[c]

    def inner(idx):
        place = dict(zip(out, idx))
        terms = []
        for j in range(J):
            factors = []
            for letters, r in zip(ops, refs):
                if isinstance(r, tuple):
                    raise Untraced('einsum on a constant operand (%s)' % st.where)
                sub = [place[x] if x != c else str(j) for x in letters]
                factors.append(w.ref(r, sub))
            terms.append(' * '.join(factors) if len(factors) < 3 else '(' + ' * '.join(factors[:2]) + ') * ' + factors[2])
        target = w.ref(st.out, idx)
        if order == 'seq':
            expr = '0.0'
            for t in terms:
                expr = '(%s + %s)' % (expr, t)
            return ['%s = %s;' % (target, expr)]
        v = int(order[5:])
        lanes, t = ['0.0'] * v, 0
        while J - t >= 4 * v:
            lanes = ['((((%s + %s) + %s) + %s) + %s)' % (lanes[k], terms[t + 3 * v + k], terms[t + 2 * v + k],
                                                       terms[t + v + k], terms[t + k]) for k in range(v)]
            t += 4 * v
        while t < J:
            lanes = ['(%s + %s)' % (lanes[k], terms[t + k] if t + k < J else '0.0') for k in range(v)]
            t += v
        across = '(%s + %s)' % (lanes[0], lanes[1]) if v == 2 else '((%s + %s) + (%s + %s))' % (
            lanes[0], lanes[2], lanes[1], lanes[3])
        return ['%s = 0.0 + %s;' % (target, across)]
    return w.loops(st.out.shape, inner)


def _copy(w, shape, dst, src):
    """dst = src over `shape`, each (buffer, base, terms) or a constant (values,)."""
    def inner(idx):
        rhs = ('%s[%s]' % (w.table(src[0], 'double'), ' + '.join(['0'] + ['%d * %s' % (s, i) for s, i in
                                                                       zip(_cstrides(shape), idx)]))
               if len(src) == 1 else w.at(src[0], src[1], src[2], idx))
        return ['%s = %s;' % (w.at(dst[0], dst[1], dst[2], idx), rhs)]
    return w.loops(shape, inner)


def _cstrides(shape):
    out, step = [], 1
    for n in reversed(shape):
        out.append(step)
        step *= n
    return list(reversed(out))


def _split(T, where):
    """(base, terms) of a Ref or a separable table."""
    if isinstance(T, Ref):
        return T.offset, [('stride', x) for x in T.strides]
    got = separable(T)
    if got is None:
        raise Untraced('an index table not separable by axis (%s)' % where)
    return got


def _leaves(node):
    """An elementwise tree's ('ref', Ref) leaves."""
    if node[0] == 'ref':
        yield node
    elif node[0] not in ('scalar', 'const'):
        for k in node[1:]:
            yield from _leaves(k)


def _same(a, b):
    return (a.buffer is b.buffer and a.offset == b.offset and a.strides == b.strides and a.shape == b.shape)


def _put(node, target, tree):
    """`node` with each leaf reading `target` replaced by `tree`."""
    if node[0] == 'ref':
        return tree if _same(node[1], target) else node
    if node[0] in ('scalar', 'const'):
        return node
    return (node[0],) + tuple(_put(k, target, tree) for k in node[1:])


def fused(prog):
    """`prog`'s statements with each elementwise temp written once and read by one later elementwise
    statement alone, element for element, folded into that statement's tree - none of what it read
    written between. The per-element arithmetic numpy's, its temps gone."""
    sts = list(prog.statements)
    writes, reads = {}, {}
    for k, st in enumerate(sts):
        writes.setdefault(st.out.buffer.id, []).append(k)
        for b in _touched(st)[1:]:
            reads.setdefault(b.id, []).append(k)
    outs = {r.buffer.id for r in prog.outs}
    gone = set()
    for k, st in enumerate(sts):
        b = st.out.buffer
        if (st.kind != 'ew' or b.role != 'temp' or b.id in outs or writes.get(b.id) != [k]
                or len(set(reads.get(b.id, []))) != 1 or st.out.offset != 0 or b.size != int(np.prod(st.out.shape))):
            continue
        j = reads[b.id][0]
        use = sts[j]
        if use.kind != 'ew' or use.out.shape != st.out.shape:
            continue
        hits = [leaf for leaf in _leaves(use.args) if leaf[1].buffer is b]
        if not hits or any(not _same(leaf[1], st.out) for leaf in hits):
            continue
        read = {leaf[1].buffer.id for leaf in _leaves(st.args)}
        if any(sts[i].out.buffer.id in read for i in range(k + 1, j) if i not in gone):
            continue
        sts[j] = type(use)(use.kind, use.out, _put(use.args, st.out, st.args), use.value, use.where)
        for leaf in _leaves(st.args):
            reads.setdefault(leaf[1].buffer.id, []).append(j)
        gone.add(k)
    return [st for k, st in enumerate(sts) if k not in gone]


def lay(prog, statements=None):
    """Each buffer's place: inputs by their order; an output that is its whole buffer in C order the
    caller's own array; the rest in one workspace, a place free once the statement after its last
    has run; (places, the workspace's doubles)."""
    last, first = {}, {}
    statements = prog.statements if statements is None else statements
    for k, st in enumerate(statements):
        for b in _touched(st):
            first.setdefault(b.id, k)
            last[b.id] = k
    end = len(statements)
    outs = {}
    for k, r in enumerate(prog.outs):
        last[r.buffer.id] = end
        b = r.buffer
        if b.role != 'in' and r.offset == 0 and r.strides == tuple(_cstrides(r.shape)) and b.size == int(np.prod(r.shape)):
            outs.setdefault(b.id, k)
    starts = {}
    for bid, k in first.items():
        starts.setdefault(k, []).append(bid)
    at, free, top, ending = {}, [], 0, {}
    for k in range(end + 1):
        for bid in ending.pop(k, ()):
            free = _merged(free + [(at[bid], prog.buffers[bid].size)])
        for bid in starts.get(k, ()):
            b = prog.buffers[bid]
            if b.role == 'in' or bid in outs:
                continue
            spot = next((s for s in free if s[1] >= b.size), None)
            if spot is not None:
                free.remove(spot)
                at[bid] = spot[0]
                if spot[1] > b.size:
                    free = _merged(free + [(spot[0] + b.size, spot[1] - b.size)])
            else:
                at[bid] = top
                top += b.size
            ending.setdefault(last[bid] + 1, []).append(bid)
    return {'at': at, 'in': {b.id: k for k, b in enumerate(prog.ins)}, 'out': outs}, top


def _merged(free):
    """Free places sorted, the touching ones one."""
    out = []
    for start, size in sorted(free):
        if out and out[-1][0] + out[-1][1] == start:
            out[-1] = (out[-1][0], out[-1][1] + size)
        else:
            out.append((start, size))
    return out


def _touched(st):
    out = [st.out.buffer]
    if st.kind == 'ew':
        stack = [st.args]
        while stack:
            node = stack.pop()
            if node[0] == 'ref':
                out.append(node[1].buffer)
            elif node[0] not in ('scalar', 'const'):
                stack.extend(node[1:])
    elif st.kind == 'einsum':
        out += [r.buffer for r in st.args[1] if not isinstance(r, tuple)]
    elif st.kind in ('gather', 'scatter') and st.args[1 if st.kind == 'scatter' else 0] is not None:
        out.append(st.args[1 if st.kind == 'scatter' else 0])
    return out


def c(prog, name, orders=None, batch=False):
    """(C11 source of `void name(const double *const *in, double *const *out, double *w)`, its layout:
    places, the workspace's doubles, each einsum's order) - `orders` each einsum's where known, else
    read off its own values. `batch`: `prog` one state's, its arrays' first axis 1, the C
    `void name(long n, ...)` running it over n states, each its own slice of every input and output,
    its workspace reused."""
    statements = fused(prog)
    laid, top = lay(prog, statements)
    w = Writer(laid)
    given, orders = list(orders) if orders is not None else None, []
    for k, st in enumerate(statements):
        w.body.append('  /* %d %s %s */' % (k, st.kind, st.where.replace('*/', '* /')))
        if any(b.role == 'in' for b in [st.out.buffer]):
            raise Untraced('her code writes an input (%s)' % st.where)
        if 0 in (st.args[0].shape if st.kind == 'scatter' else st.out.shape):
            continue
        if st.kind == 'fill':
            b = st.out.buffer
            place = 'out%d' % laid['out'][b.id] if b.id in laid['out'] else 'w + %d' % laid['at'][b.id]
            w.body.append('  memset(%s, 0, %d * sizeof(double));' % (place, b.size))
        elif st.kind == 'ew':
            if any(leaf[1].buffer is st.out.buffer and not _same(leaf[1], st.out) for leaf in _leaves(st.args)):
                raise Untraced('an elementwise write over what it reads, elsewhere in it (%s)' % st.where)
            w.body += w.loops(st.out.shape, lambda idx, st=st: ['%s = %s;' % (w.ref(st.out, idx), _tree(w, st.args, idx))])
        elif st.kind == 'einsum':
            order = given[len(orders)] if given is not None else ordered(st)
            orders.append(order)
            w.body += _einsum(w, st, order)
        elif st.kind == 'gather':
            src, table = st.args
            base, terms = _split(table, st.where)
            w.body += _copy(w, st.out.shape, (st.out.buffer, st.out.offset, [('stride', s) for s in st.out.strides]),
                            (src, base, terms))
        elif st.kind == 'scatter':
            table, src, pick = st.args
            base, terms = _split(table, st.where)
            if src is st.out.buffer:
                raise Untraced('a scatter from its own buffer (%s)' % st.where)
            from_ = (pick,) if src is None else (src,) + tuple(_split(pick, st.where))
            w.body += _copy(w, table.shape, (st.out.buffer, base, terms), from_)
    for k, r in enumerate(prog.outs):
        if laid['out'].get(r.buffer.id) == k:
            continue
        w.body.append('  /* out %d */' % k)
        w.body += w.loops(r.shape, lambda idx, r=r, k=k: ['out%d[%s] = %s;' % (
            k, ' + '.join(['0'] + ['%d * %s' % (s, i) for s, i in zip(_cstrides(r.shape), idx)]), w.ref(r, idx))])
    head = ['/* %s: generated by tools.cores.emit from her numpy, not by hand. */' % name,
            '#include <math.h>', '#include <string.h>', '#include "wbc_np.h"', '',
            '#ifdef _WIN32', '#define API __declspec(dllexport)', '#else', '#define API', '#endif', ''] + w.tables + [
        '', 'API void %s(%sconst double *const *in, double *const *out, double *w)' % (name, 'long n, ' if batch else ''),
        '{']
    if batch:
        if any(b.root.shape[:1] != (1,) for b in prog.ins) or any(r.shape[:1] != (1,) for r in prog.outs):
            raise Untraced('a batch of one state: every input and output its first axis 1')
        head += ['  for (long s = 0; s < n; s++)', '  {']
        head += ['  const double *in%d = in[%d] + s * %d;' % (k, k, b.size) for k, b in enumerate(prog.ins)]
        head += ['  double *out%d = out[%d] + s * %d;' % (k, k, int(np.prod(r.shape))) for k, r in enumerate(prog.outs)]
        tail = ['  }', '}', '']
    else:
        head += ['  const double *in%d = in[%d];' % (k, k) for k in range(len(prog.ins))]
        head += ['  double *out%d = out[%d];' % (k, k) for k in range(len(prog.outs))]
        tail = ['}', '']
    head += ['  (void)w;']
    return '\n'.join(head + w.body + tail), {'laid': laid, 'doubles': top, 'orders': orders}
