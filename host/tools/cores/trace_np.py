"""numpy's functions as her trace takes them: each numpy's own result, a statement.

Its functions over `tools.cores.trace`'s arrays: einsum, cross, matmul, stack, vstack and concatenate; the reductions - sum, mean, min, max - in
numpy's own order (`emit`'s 'reduce'); where, clip and nan_to_num elementwise; linalg's norm.
"""
import numpy as np

from tools.cores import trace
from tools.cores.trace import Ref, Traced, Untraced, operand, owner, where

trace.hide(__file__)


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

    def side(x, c, left):
        held = x.ref() if isinstance(x, Traced) else ('const', np.ascontiguousarray(c, dtype=float))
        st = held.strides if isinstance(held, Ref) else tuple(k // 8 for k in held[1].strides)
        return held, ((0, st[0]) if left else (st[0], 0)) if c.ndim == 1 else st
    out = result.ref()
    o = (0, 0) if got.ndim == 0 else (((out.strides[0], 0) if p == 1 else (0, out.strides[0])) if got.ndim == 1
                                      else out.strides)
    prog.add('matmul', out, (side(a, ca, True), side(b, cb, False), o, (m, n, p)))
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
    'reduce' statement, numpy's own result."""
    ufunc = getattr(np, op)
    kw = {} if initial is None else {'initial': initial}
    if not isinstance(x, Traced):
        return ufunc.reduce(np.asarray(x, dtype=float), axis=axis, keepdims=keepdims, **kw)
    nd = x.ndim
    axes = tuple(range(nd)) if axis is None else tuple(sorted({a % nd for a in (axis if isinstance(axis, tuple)
                                                                                 else (axis,))}))
    if any(s < 0 for s in x.ref().strides):
        raise Untraced('a reduction over a reversed view at %s' % where())
    got = np.array(ufunc.reduce(x.view, axis=axes, keepdims=keepdims, **kw))
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


FUNCTIONS = {'einsum': _einsum, 'cross': _cross,
             'stack': lambda prog, arrays, *a, **k: _placed(prog, np.stack, arrays, *a, **k),
             'vstack': lambda prog, arrays, *a, **k: _placed(prog, np.vstack, arrays, *a, **k),
             'concatenate': lambda prog, arrays, *a, **k: _placed(prog, np.concatenate, arrays, *a, **k),
             'sum': lambda prog, x, axis=None, keepdims=False: reduced(prog, 'add', x, axis, None, keepdims),
             'prod': lambda prog, x, axis=None, keepdims=False: reduced(prog, 'multiply', x, axis, None, keepdims),
             'mean': lambda prog, x, axis=None, keepdims=False: mean(prog, x, axis, keepdims),
             'min': _reducing('minimum'), 'amin': _reducing('minimum'), 'max': _reducing('maximum'),
             'amax': _reducing('maximum'), 'where': _where, 'clip': _clip, 'nan_to_num': _nan_to_num, 'norm': _norm,
             'shape': lambda prog, x: x.shape, 'ndim': lambda prog, x: x.ndim, 'size': lambda prog, x: x.size}
