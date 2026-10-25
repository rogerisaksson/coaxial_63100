"""numpy's functions as her trace takes them: each numpy's own result, a statement.

Its functions over `tools.cores.trace`'s arrays: einsum, cross, matmul, stack, vstack and concatenate; the reductions - sum, mean, min, max - in
numpy's own order (`emit`'s 'reduce'), any and all; where, clip, nan_to_num, power and astype elementwise; full and
full_like; diag, repeat and roll copies; linalg's norm, solve and inv.
"""
import numpy as np

from tools.cores import trace
from tools.cores.trace import Ref, Traced, Untraced, operand, owner, where

trace.hide(__file__)
#: wbc_blas.c's MOST: the largest solve its dgesv lays on the stack.
GESV_MOST = 128
#: numpy's power where its exponent's stride is 0 (DOUBLE_power), by the exponent: an elementwise tree of x.
FAST_POWER = {-1.0: lambda x: ('reciprocal', x), 0.0: lambda x: ('positive', ('scalar', 1.0)),
              0.5: lambda x: ('sqrt', x), 1.0: lambda x: ('positive', x), 2.0: lambda x: ('square', x)}


def _einsum(prog, subscripts, *operands, **kwargs):
    if kwargs.get('optimize', False) or set(kwargs) - {'optimize'}:
        raise Untraced('np.einsum %r with %s at %s' % (subscripts, kwargs, where()))
    concrete = [x.view if isinstance(x, Traced) else np.asarray(x) for x in operands]
    got = np.einsum(subscripts, *concrete)
    buf = prog.buffer(owner(got), 'temp')
    result = Traced(prog, buf, got)
    refs = [x.ref() if isinstance(x, Traced) else ('const', np.array(x)) for x in operands]
    prog.add('einsum', result.ref(), (subscripts, refs, [np.array(c, copy=True) for c in concrete]), got)
    return result


def _cross(prog, a, b, axisa=-1, axisb=-1, axisc=-1, axis=None):
    """np.cross as numpy computes it: each component a product less a product, in its own buffer laid
    as numpy lays it, the vector's axis moved back where it asked."""
    if axis is not None:
        axisa = axisb = axisc = axis
    ca, cb = (x.view if isinstance(x, Traced) else np.asarray(x) for x in (a, b))
    got = np.cross(ca, cb, axisa=axisa, axisb=axisb, axisc=axisc)
    buf = prog.buffer(owner(got), 'temp')
    result = Traced(prog, buf, got)
    am, bm = (_moved(prog, x, ax) for x, ax in ((a, axisa), (b, axisb)))
    cm = np.moveaxis(got, axisc, -1)
    for c, (i, j) in enumerate(((1, 2), (2, 0), (0, 1))):
        part = prog.ref(buf, cm[..., c])
        shape = part.shape
        tree = ('subtract', ('multiply', _comp(prog, am, i, shape), _comp(prog, bm, j, shape)),
                ('multiply', _comp(prog, am, j, shape), _comp(prog, bm, i, shape)))
        prog.add('ew', part, tree)
    return result


def _moved(prog, x, ax):
    if isinstance(x, Traced):
        return Traced(prog, x.buffer, np.moveaxis(x.view, ax, -1))
    return np.moveaxis(np.asarray(x, dtype=float), ax, -1)


def _comp(prog, x, k, shape):
    return operand(prog, x[..., k] if isinstance(x, Traced) else x[..., k], shape)


def matmul(prog, a, b):
    """numpy's a @ b of 1-d and 2-d operands: a 'matmul' statement, its core dims (m, n, p), each side
    (its Ref or constant, its strides with a 1-d operand's missing one 0), numpy's own dispatch on
    them in C (`wbc_matmul`)."""
    ca, cb = (x.view if isinstance(x, Traced) else np.asarray(x, dtype=float) for x in (a, b))
    if ca.ndim > 2 or cb.ndim > 2 or not ca.ndim or not cb.ndim:
        raise Untraced('a matmul of %d-d and %d-d at %s' % (ca.ndim, cb.ndim, where()))
    got = np.matmul(ca, cb)
    got = got if isinstance(got, np.ndarray) else np.array(got)
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    m, n, p = (1 if ca.ndim == 1 else ca.shape[0]), ca.shape[-1], (1 if cb.ndim == 1 else cb.shape[1])
    if m > 1 and n > 1 and p > 1:
        (a, ca), (b, cb) = _blased(prog, a, ca), _blased(prog, b, cb)

    def side(x, c, left):
        held, st = (x.ref(), x.ref().strides) if isinstance(x, Traced) else _laid(c)
        return held, ((0, st[0]) if left else (st[0], 0)) if c.ndim == 1 else st
    out = result.ref()
    o = (0, 0) if got.ndim == 0 else (((out.strides[0], 0) if p == 1 else (0, out.strides[0])) if got.ndim == 1
                                      else out.strides)
    prog.add('matmul', out, (side(a, ca, True), side(b, cb, False), o, (m, n, p)))
    return result


def _blased(prog, x, c):
    """(x, its array) as numpy 2.5's matrix @ matrix takes it: itself where BLAS takes its strides
    either way (is_blasable2d), else a C-ordered copy of its own for gemm - her array's an 'ew' copy."""
    (s0, s1), (d0, d1), z = c.strides, c.shape, c.itemsize
    if (s1 == z and s0 % z == 0 and s0 // z >= d1) or (s0 == z and s1 % z == 0 and s1 // z >= d0):
        return x, c
    got = np.ascontiguousarray(c, dtype=float)
    if not isinstance(x, Traced):
        return got, got
    return _filled(prog, x, got), got


def _laid(c):
    """(a constant operand as numpy laid it, its strides): C-ordered itself; else the span of memory
    under it, its first element's place and its strides kept - a transposed one gemm's TRANS, a
    strided one numpy's own loop, as numpy's call was."""
    if c.flags.c_contiguous or any(s % c.itemsize for s in c.strides):
        arr = np.ascontiguousarray(c, dtype=float)
        return ('const', arr), tuple(s // 8 for s in arr.strides)
    st = tuple(s // c.itemsize for s in c.strides)
    at = Ref(None, 0, st, c.shape).addresses()
    span = np.zeros(int(at.max() - at.min()) + 1)
    span[at - at.min()] = c
    return ('const', span, -int(at.min())), st


def _gesv(prog, a, b, got):
    """numpy's linalg on (n, n) and (n,) or (n, k) - b None its identity -, `got` its own result: a
    'solve' statement (`wbc_solve`, `wbc_inv`), its dgesv's scratch a buffer of its own."""
    ca, cb = np.asarray(_view(a)), None if b is None else np.asarray(_view(b))
    n = len(ca)
    if (ca.ndim != 2 or not 0 < n <= GESV_MOST or (cb is not None and cb.ndim not in (1, 2))
            or got.dtype != np.float64 or not got.flags.c_contiguous):
        raise Untraced('linalg of %s and %s at %s' % (ca.shape, None if cb is None else cb.shape, where()))
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    nrhs = n if cb is None else 1 if cb.ndim == 1 else cb.shape[1]
    scratch = prog.buffer(np.empty(2 * n * (n + nrhs)), 'temp')

    def side(x, c):
        held = x.ref() if isinstance(x, Traced) else ('const', np.ascontiguousarray(c, dtype=float))
        st = held.strides if isinstance(held, Ref) else tuple(k // 8 for k in held[1].strides)
        return held, st + (0,) * (2 - len(st))
    prog.add('solve', result.ref(), (side(a, ca), None if b is None else side(b, cb), n, nrhs,
                                     Ref(scratch, 0, (1,), (scratch.size,))))
    return result


def power(prog, x, y, out=None):
    """numpy's power to float64 (DOUBLE_power): where its loop sees the exponent at stride 0 - a
    number, a 0-d array, one broadcast whole - its fast paths by its value (FAST_POWER), else C's
    pow, numpy's npy_pow (UCRT's both here); an exponent of every element C's pow; one broadcast
    along some axes, or one element in all, numpy's iterator's to choose: Untraced."""
    cx, cy = _view(x), _view(y)
    if np.result_type(cx, cy) != np.float64:
        raise Untraced('np.power to %s at %s' % (np.result_type(cx, cy), where()))
    shape = np.broadcast_shapes(np.shape(cx), np.shape(cy))
    size = int(np.prod(shape))
    whole = np.ndim(cy) == 0 or (np.size(cy) == 1 and size > 1)
    if whole and not isinstance(y, Traced):
        fast = FAST_POWER.get(float(np.ravel(cy)[0]))
        return trace._ufunc(prog, np.power, (x, y), out, lambda ox, oy: ('power', ox, oy) if fast is None else fast(ox))
    if not whole and (size < 2 or 0 in [s for s, k in zip(np.broadcast_to(cy, shape).strides, shape) if k > 1]):
        raise Untraced('np.power of %s to %s at %s' % (shape, np.shape(cy), where()))
    return trace._ufunc(prog, np.power, (x, y), out, lambda ox, oy: ('power0' if whole else 'power', ox, oy))


def truth(x, op, axis=None, out=None, keepdims=False, **kw):
    """numpy's any (`op` 'maximum') or all ('minimum'): a reduction of x's truth, 0 or 1, from 0 or 1
    - exact in any order -, its result bools."""
    if not isinstance(x, Traced) or out is not None or kw:
        raise Untraced('np.%s with %s at %s' % ({'maximum': 'any'}.get(op, 'all'), sorted(kw), where()))
    return reduced(x.prog, op, x if x.dtype == bool else np.not_equal(x, 0.0), axis, op == 'minimum', keepdims)


def astype(x, dtype, order='K', casting='unsafe', subok=True, copy=True):
    """numpy's astype to float64: a copy laid as numpy lays it, or `x` where numpy copies none - a
    bool's 0.0 and 1.0 its doubles already; to another dtype Untraced."""
    if np.dtype(dtype) != np.float64:
        raise Untraced('.astype(%s) at %s' % (dtype, where()))
    got = x.view.astype(dtype, order, casting, subok, copy)
    return x if got is x.view else _filled(x.prog, x, got)


def full(prog, real, shape, fill_value, *rest, **kw):
    """np.full of her fill: numpy's own array, her fill copied to each element; of a fill not hers,
    numpy's own - a constant."""
    if not isinstance(fill_value, Traced):
        return real(shape, fill_value, *rest, **kw)
    return _filled(prog, fill_value, real(shape, fill_value.view, *rest, **kw))


def full_like(prog, real, a, fill_value, *rest, **kw):
    """np.full_like of her fill, as `full`, laid as numpy lays `a`'s; of a fill not hers numpy's own."""
    got = real(_view(a), _view(fill_value), *rest, **kw)
    return _filled(prog, fill_value, got) if isinstance(fill_value, Traced) else got


def _filled(prog, x, got):
    """`got` (numpy's own, of `x` copied or cast) as a buffer of its own: an 'ew' copy of `x`, a bool
    as float its 0.0 or 1.0, a float as a bool Untraced."""
    if got.dtype != np.float64 and not (got.dtype == bool and x.dtype == bool):
        raise Untraced('%s laid as %s at %s' % (x.dtype, got.dtype, where()))
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    prog.add('ew', result.ref(), ('positive', operand(prog, x, got.shape)))
    return result


def _int(x):
    """`x`, an int as numpy reads it: Untraced where it is her state."""
    if not isinstance(x, (int, np.integer)):
        raise Untraced('a count or an axis of %s at %s' % (type(x).__name__, where()))
    return int(x)


def _ints(x):
    """`x`, an int or a sequence of them (`_int`)."""
    return tuple(_int(v) for v in x) if isinstance(x, (tuple, list)) else _int(x)


def _diag(prog, v, k=0):
    """np.diag: of a 1-d a square matrix, `v` down its k-th diagonal (zeros, then a scatter); of a 2-d
    its k-th diagonal, numpy's read-only view."""
    k = _int(k)
    if v.ndim == 2:
        return v._view(v.view.diagonal(k))
    if v.ndim != 1:
        raise Untraced('np.diag of %d-d at %s' % (v.ndim, where()))
    got = np.diag(v.view, k)
    result = Traced(prog, prog.buffer(got, 'zeros'), got)
    prog.add('fill', result.ref(), (0,))
    prog.add('scatter', Ref(result.buffer, 0, (), ()), (np.diagonal(result.ref().addresses(), k), v.buffer, v.ref()))
    return result


def _repeat(prog, a, repeats, axis=None):
    """np.repeat by a static count: a gather through numpy's own repeat of `a`'s addresses."""
    repeats, axis = _int(repeats), None if axis is None else _int(axis)
    got = np.repeat(a.view, repeats, axis)
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    prog.add('gather', result.ref(), (a.buffer, np.repeat(a.ref().addresses(), repeats, axis)))
    return result


def _roll(prog, a, shift, axis=None):
    """np.roll by a static shift: a gather through numpy's own roll of `a`'s addresses - of every
    axis in their flattened order, numpy's own result C-ordered."""
    shift, axis = _ints(shift), None if axis is None else _ints(axis)
    got = np.roll(a.view, shift, axis)
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    table, out = np.roll(a.ref().addresses(), shift, axis), result.ref()
    if axis is None:
        if not got.flags.c_contiguous:
            raise Untraced('np.roll laid apart at %s' % where())
        table, out = table.ravel(), Ref(out.buffer, out.offset, (1,), (got.size,))
    prog.add('gather', out, (a.buffer, table))
    return result


def _placed(prog, func, arrays, *rest, **kw):
    """np.stack, vstack or concatenate: numpy's own result, each array copied to its place in it."""
    concrete = [x.view if isinstance(x, Traced) else np.asarray(x, dtype=float) for x in arrays]
    got = func(concrete, *rest, **kw)
    result = Traced(prog, prog.buffer(owner(got), 'temp'), got)
    tags = func([np.full(c.shape, k) for k, c in enumerate(concrete)], *rest, **kw)
    addresses = result.ref().addresses()
    for k, (x, c) in enumerate(zip(arrays, concrete)):
        table = addresses[tags == k].reshape(c.shape)
        if isinstance(x, Traced):
            prog.add('scatter', Ref(result.buffer, 0, (), ()), (table, x.buffer, x.ref()))
        else:
            prog.add('scatter', Ref(result.buffer, 0, (), ()), (table, None, np.array(c)))
    return result


def reduced(prog, op, x, axis=None, initial=None, keepdims=False):
    """numpy's `op` ('add', 'multiply', 'minimum', 'maximum') reduced over `axis` (None every one): a
    'reduce' statement, numpy's own result; one from an `initial` of her state, or one numpy refuses,
    Untraced."""
    ufunc = getattr(np, op)
    if isinstance(initial, Traced):
        raise Untraced('a reduction from an initial of her state at %s' % where())
    kw = {} if initial is None else {'initial': initial}
    if not isinstance(x, Traced):
        return ufunc.reduce(np.asarray(x, dtype=float), axis=axis, keepdims=keepdims, **kw)
    nd = x.ndim
    axes = tuple(range(nd)) if axis is None else tuple(sorted({a % nd for a in (axis if isinstance(axis, tuple)
                                                                                 else (axis,))}))
    if any(s < 0 for s in x.ref().strides):
        raise Untraced('a reduction over a reversed view at %s' % where())
    try:
        got = np.array(ufunc.reduce(x.view, axis=axes, keepdims=keepdims, **kw))
    except ValueError as e:
        raise Untraced('%s at %s' % (e, where())) from e
    result = Traced(prog, prog.buffer(got, 'temp'), got)
    prog.add('reduce', result.ref(), (op, x.ref(), axes, initial), got)
    return result


def mean(prog, x, axis=None, keepdims=False):
    """numpy's mean: its sum over `axis` divided by the count summed."""
    total = reduced(prog, 'add', x, axis, None, keepdims)
    return total / float(x.size // max(1, total.size))


def _node(prog, name, inputs, got):
    """An elementwise node of numpy's `name` over `inputs`, numpy's own result `got`."""
    got = np.array(got)
    result = Traced(prog, prog.buffer(got, 'temp'), got)
    prog.add('ew', result.ref(), (name,) + tuple(operand(prog, x, got.shape) for x in inputs))
    return result


def _view(x):
    return x.view if isinstance(x, Traced) else x


def _where(prog, condition, *picks):
    if len(picks) != 2:
        raise Untraced('np.where of one argument at %s: its shape her state' % where())
    return _node(prog, 'where', (condition,) + picks, np.where(_view(condition), *map(_view, picks)))


def _clip(prog, x, lo, hi, **kw):
    if kw:
        raise Untraced('np.clip with %s at %s' % (sorted(kw), where()))
    lo, hi = (-np.inf if lo is None else lo), (np.inf if hi is None else hi)
    return _node(prog, 'clip', (x, lo, hi), np.clip(_view(x), _view(lo), _view(hi)))


def _nan_to_num(prog, x, **kw):
    if kw:
        raise Untraced('np.nan_to_num with %s at %s' % (sorted(kw), where()))
    return _node(prog, 'nan_to_num', (x,), np.nan_to_num(_view(x)))


def _norm(prog, x, ord=None, axis=None, keepdims=False):
    """linalg's norm as numpy takes it: a vector's sqrt of its dot with itself, along an axis the
    sqrt of the sum of its squares."""
    if ord is not None or keepdims or (axis is None and x.ndim != 1):
        raise Untraced('np.linalg.norm with ord %r, axis %r of %d-d at %s' % (ord, axis, x.ndim, where()))
    if axis is None:
        return np.sqrt(matmul(prog, x, x))
    return np.sqrt(reduced(prog, 'add', x * x, axis))


def _reducing(op):
    def rule(prog, x, axis=None, keepdims=False, initial=None, **kw):
        if kw:
            raise Untraced('np.%s with %s at %s' % (op, sorted(kw), where()))
        return reduced(prog, op, x, axis, initial, keepdims)
    return rule


def _solve(prog, a, b):
    """linalg's solve of (n, n) and (n,) or (n, k): numpy's dgesv on Fortran-ordered copies in C,
    into its result C-ordered as numpy's; LinAlgError where numpy raises it."""
    return _gesv(prog, a, b, np.linalg.solve(_view(a), _view(b)))


def _inv(prog, a):
    """linalg's inv: its solve against the identity."""
    return _gesv(prog, a, None, np.linalg.inv(_view(a)))


FUNCTIONS = {'einsum': _einsum, 'cross': _cross,
             'stack': lambda prog, arrays, *a, **k: _placed(prog, np.stack, arrays, *a, **k),
             'vstack': lambda prog, arrays, *a, **k: _placed(prog, np.vstack, arrays, *a, **k),
             'concatenate': lambda prog, arrays, *a, **k: _placed(prog, np.concatenate, arrays, *a, **k),
             'sum': lambda prog, x, axis=None, keepdims=False: reduced(prog, 'add', x, axis, None, keepdims),
             'prod': lambda prog, x, axis=None, keepdims=False: reduced(prog, 'multiply', x, axis, None, keepdims),
             'mean': lambda prog, x, axis=None, keepdims=False: mean(prog, x, axis, keepdims),
             'min': _reducing('minimum'), 'amin': _reducing('minimum'), 'max': _reducing('maximum'),
             'amax': _reducing('maximum'), 'where': _where, 'clip': _clip, 'nan_to_num': _nan_to_num, 'norm': _norm,
             'shape': lambda prog, x: x.shape, 'ndim': lambda prog, x: x.ndim, 'size': lambda prog, x: x.size,
             'ravel': lambda prog, x, order='C': x.reshape(-1),
             'any': lambda prog, x, *a, **k: truth(x, 'maximum', *a, **k),
             'all': lambda prog, x, *a, **k: truth(x, 'minimum', *a, **k),
             'full_like': lambda prog, *a, **k: full_like(prog, np.full_like, *a, **k),
             'diag': _diag, 'repeat': _repeat, 'roll': _roll, 'solve': _solve, 'inv': _inv}
