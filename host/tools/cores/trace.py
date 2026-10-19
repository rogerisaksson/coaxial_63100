"""Her numpy traced on her own arrays: each operation numpy's own result, at its layout.

A statement over dense buffers and their affine views, what `tools.cores.emit` writes in C and
`tools.cores.held` holds bit for bit.

    prog = trace.traced(fn, args, inputs=(1, 2))    # fn(*args), the arrays at `inputs` traced
    prog.statements, prog.buffers, prog.ins, prog.outs

A statement: 'fill' a buffer laid by np.zeros; 'ew' an elementwise tree (IEEE per element, any loop
order the same); 'einsum' a contraction, its order numpy's to be found (`emit.ordered`); 'gather' and
'scatter' a copy through index tables. A branch, a float or a write into a plain array on her state
cannot be traced and raises: what is compiled is a function of its arrays alone.
"""
import sys
import threading

import numpy as np

#: The ufuncs an elementwise tree takes, by numpy's name.
UFUNCS = ('add', 'subtract', 'multiply', 'divide', 'negative', 'positive', 'sign', 'maximum', 'minimum',
          'sqrt', 'absolute', 'square', 'sin', 'cos', 'tan', 'arcsin', 'arccos', 'arctan', 'arctan2', 'exp',
          'log', 'hypot', 'floor', 'ceil', 'greater', 'greater_equal', 'less', 'less_equal', 'equal',
          'not_equal', 'logical_and', 'logical_or', 'logical_not')
_HERE = (__file__,)


class Untraced(TypeError):

    """What her code did to a traced array that a program of its arrays alone cannot hold."""


class Buffer:

    """A dense allocation: `root` numpy's own array over it, `role` 'in', 'zeros' or 'temp'."""

    def __init__(self, k, root, role):
        self.id, self.role, self.root = k, role, root if role == 'in' else None
        lo = root.__array_interface__['data'][0] + sum(min(s, 0) * (n - 1) for s, n in zip(root.strides, root.shape))
        self.start, self.size, self.dtype, self.shape = lo, root.size, root.dtype, root.shape


class Ref:

    """An affine view of a buffer: element `offset` and `strides`, `shape`."""

    def __init__(self, buffer, offset, strides, shape):
        self.buffer, self.offset, self.strides, self.shape = buffer, offset, tuple(strides), tuple(shape)

    def addresses(self):
        """Each element's address in its buffer, an int array of `shape`."""
        out = np.full(self.shape, self.offset, dtype=np.int64)
        for d, (n, s) in enumerate(zip(self.shape, self.strides)):
            out += (np.arange(n, dtype=np.int64) * s).reshape((1,) * d + (n,) + (1,) * (len(self.shape) - d - 1))
        return out

    def broadcast(self, shape):
        """This view broadcast to `shape` as numpy broadcasts: a new axis or a 1 strides 0."""
        lead = len(shape) - len(self.shape)
        strides = [0] * lead + [0 if n == 1 and m != 1 else s for n, m, s in zip(self.shape, shape[lead:],
                                                                               self.strides)]
        return Ref(self.buffer, self.offset, strides, shape)


class Statement:

    """One operation: `kind`, the view it writes `out`, its `args`, numpy's values after it `value`,
    where in her code it stood `where`."""

    def __init__(self, kind, out, args, value, where):
        self.kind, self.out, self.args, self.value, self.where = kind, out, args, value, where


class Program:

    """Her function as statements over buffers: its inputs and its outputs (Refs)."""

    def __init__(self):
        self.buffers, self.statements, self.ins, self.outs = [], [], [], []

    def buffer(self, root, role):
        b = Buffer(len(self.buffers), root, role)
        self.buffers.append(b)
        return b

    def ref(self, buffer, view):
        """`view` (numpy's, over `buffer.root`) as an affine Ref."""
        size = view.dtype.itemsize
        off, rem = divmod(view.__array_interface__['data'][0] - buffer.start, size)
        if rem or any(s % size for s in view.strides):
            raise Untraced('a view off its elements')
        return Ref(buffer, off, [s // size for s in view.strides], view.shape)

    def add(self, kind, out, args, value=None):
        kept = None if value is None else np.array(value, copy=True)
        self.statements.append(Statement(kind, out, args, kept, _where()))


def _where():
    """file:line of the frame in her code nearest this statement."""
    f = sys._getframe(2)
    while f is not None and (f.f_code.co_filename in _HERE or 'numpy' in f.f_code.co_filename):
        f = f.f_back
    return '%s:%d' % (f.f_code.co_filename, f.f_lineno) if f is not None else '?'


def _owner(a):
    while isinstance(a.base, np.ndarray):
        a = a.base
    return a


def _binary(uf):
    """(a op b, b op a) of a numpy ufunc, `a` traced."""
    return (lambda self, other: uf(self, other)), (lambda self, other: uf(other, self))


def _inplace(uf):
    """a op= b: the ufunc into `a`."""
    def done(self, other):
        uf(self, other, out=self)
        return self
    return done


class Traced:

    """An array of her state: numpy's own value (`view`, over its buffer's root) and the program
    that writes it."""

    __array_priority__ = 1000

    def __init__(self, prog, buffer, view):
        self.prog, self.buffer, self.view = prog, buffer, view

    shape = property(lambda self: self.view.shape)
    ndim = property(lambda self: self.view.ndim)
    size = property(lambda self: self.view.size)
    dtype = property(lambda self: self.view.dtype)
    T = property(lambda self: self.transpose())

    def ref(self):
        return self.prog.ref(self.buffer, self.view)

    def __len__(self):
        return len(self.view)

    def __iter__(self):
        return (self[k] for k in range(len(self)))

    def __bool__(self):
        raise Untraced('a branch on her state at %s' % _where())

    def __float__(self):
        raise Untraced('a float of her state at %s' % _where())

    __int__ = __index__ = __float__

    def __array__(self, dtype=None, copy=None):
        raise Untraced('her state written into a plain array at %s' % _where())

    def __repr__(self):
        return 'Traced(%s)' % (self.shape,)

    def _view(self, view):
        if _owner(view) is not _owner(self.view):
            raise Untraced('a view that is a copy')
        return Traced(self.prog, self.buffer, view)

    def transpose(self, *axes):
        return self._view(self.view.transpose(*axes))

    def reshape(self, *shape):
        got = self.view.reshape(*shape)
        if _owner(got) is _owner(self.view):
            return self._view(got)
        raise Untraced('a reshape numpy copies at %s' % _where())

    def __getitem__(self, key):
        _static(key)
        got = self.view[key]
        if not isinstance(got, np.ndarray):
            got = self.view[key + (Ellipsis,) if isinstance(key, tuple) else (key, Ellipsis)]
        if _owner(got) is _owner(self.view):
            return Traced(self.prog, self.buffer, got)
        return _gathered(self.prog, self, key)

    def __setitem__(self, key, value):
        _static(key)
        if isinstance(value, (tuple, list)) and any(isinstance(v, Traced) for v in _flat(value)):
            for k, v in enumerate(value):
                self[_sub(key, k, self.view)] = v
            return
        target = _indexed(self, key)
        shape = target.shape
        if isinstance(value, Traced):
            if value.buffer is self.buffer:
                raise Untraced('a write from the array it writes at %s' % _where())
            self.view[key] = value.view
            self.prog.add('scatter', Ref(self.buffer, 0, (), ()), (target, value.buffer, value.ref().broadcast(shape)))
        else:
            self.view[key] = value
            const = np.array(np.broadcast_to(np.asarray(value, dtype=self.dtype), shape))
            self.prog.add('scatter', Ref(self.buffer, 0, (), ()), (target, None, const))

    __add__, __radd__ = _binary(np.add)
    __sub__, __rsub__ = _binary(np.subtract)
    __mul__, __rmul__ = _binary(np.multiply)
    __truediv__, __rtruediv__ = _binary(np.divide)
    __gt__, __lt__ = _binary(np.greater)[0], _binary(np.less)[0]
    __ge__, __le__ = _binary(np.greater_equal)[0], _binary(np.less_equal)[0]
    __and__, __or__ = _binary(np.logical_and)[0], _binary(np.logical_or)[0]
    __iadd__, __isub__ = _inplace(np.add), _inplace(np.subtract)
    __imul__, __itruediv__ = _inplace(np.multiply), _inplace(np.divide)

    def __neg__(self):
        return np.negative(self)

    def __abs__(self):
        return np.absolute(self)

    def __pow__(self, other):
        """numpy's `**` on an array: a square by 2."""
        if isinstance(other, (int, float)) and float(other) == 2.0:
            return np.square(self)
        raise Untraced('** %r at %s' % (other, _where()))

    def __array_ufunc__(self, ufunc, method, *inputs, **kwargs):
        if method != '__call__' or ufunc.__name__ not in UFUNCS or set(kwargs) - {'out'}:
            raise Untraced('np.%s.%s at %s' % (ufunc.__name__, method, _where()))
        return _ufunc(self.prog, ufunc, inputs, kwargs.get('out'))

    def __array_function__(self, func, types, args, kwargs):
        rule = FUNCTIONS.get(func.__name__)
        if rule is None:
            raise Untraced('np.%s at %s' % (func.__name__, _where()))
        return rule(self.prog, *args, **kwargs)


def _flat(value):
    for v in value:
        if isinstance(v, (tuple, list)):
            yield from _flat(v)
        else:
            yield v


def _static(key):
    for k in (key if isinstance(key, tuple) else (key,)):
        if isinstance(k, Traced):
            raise Untraced('an index of her state at %s' % _where())


def _sub(key, k, view):
    """`key` narrowed to its first axis's `k`th: a[key][k] as a key of a."""
    probe = np.arange(view.size).reshape(view.shape)[key]
    return np.unravel_index(probe[k], view.shape) if probe[k].ndim == 0 else _ix(probe[k], view.shape)


def _ix(flat, shape):
    return tuple(np.asarray(i) for i in np.unravel_index(flat, shape))


def _basic(k):
    return k is None or k is Ellipsis or isinstance(k, (slice, int, np.integer))


def _indexed(src, key):
    """The elements `src[key]` names in its buffer: a Ref where `key` is basic, else an int table of
    their addresses - the basic part applied first, the table no larger than what it names."""
    key = key if isinstance(key, tuple) else (key,)
    if all(_basic(k) for k in key):
        return src.prog.ref(src.buffer, src.view[key + (Ellipsis,) if Ellipsis not in key else key])
    if any(isinstance(k, (int, np.integer)) for k in key):
        return src.ref().addresses()[key]
    part = src.view[tuple(k if _basic(k) else slice(None) for k in key)]
    rest = tuple(k if k is Ellipsis else slice(None) if _basic(k) else np.asarray(k) for k in key)
    return src.prog.ref(src.buffer, part).addresses()[rest]


def _gathered(prog, src, key):
    """A copy numpy makes of `src[key]`: its new buffer, laid as numpy lays it, the addresses it reads."""
    got = src.view[key]
    buf = prog.buffer(_owner(got), 'temp')
    out = Traced(prog, buf, got)
    table = _indexed(src, key)
    prog.add('gather', out.ref(), (src.buffer, table if isinstance(table, Ref) else np.asarray(table)))
    return out


def _operand(prog, x, shape):
    """An elementwise operand: ('ref', Ref broadcast to `shape`), ('const', its array, its strides
    broadcast to `shape`) or ('scalar', x)."""
    if isinstance(x, Traced):
        return ('ref', x.ref().broadcast(shape))
    if isinstance(x, np.ndarray) and x.ndim:
        arr = np.ascontiguousarray(x, dtype=float)
        return ('const', arr, tuple(s // arr.itemsize for s in np.broadcast_to(arr, shape).strides))
    return ('scalar', float(x))


def _ufunc(prog, ufunc, inputs, out):
    concrete = [x.view if isinstance(x, Traced) else x for x in inputs]
    if out is not None:
        target = out[0] if isinstance(out, tuple) else out
        if not isinstance(target, Traced):
            raise Untraced('her state written into a plain array at %s' % _where())
        ufunc(*concrete, out=target.view)
        got, result = target.view, target
    else:
        got = ufunc(*concrete)
        if not isinstance(got, np.ndarray):
            got = np.array(got)
        buf = prog.buffer(_owner(got), 'temp')
        result = Traced(prog, buf, got)
    tree = (ufunc.__name__,) + tuple(_operand(prog, x, got.shape) for x in inputs)
    prog.add('ew', result.ref(), tree)
    return result


def _einsum(prog, subscripts, *operands, **kwargs):
    if kwargs.get('optimize', False) or set(kwargs) - {'optimize'}:
        raise Untraced('np.einsum %r with %s at %s' % (subscripts, kwargs, _where()))
    concrete = [x.view if isinstance(x, Traced) else np.asarray(x) for x in operands]
    got = np.einsum(subscripts, *concrete)
    buf = prog.buffer(_owner(got), 'temp')
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
    buf = prog.buffer(_owner(got), 'temp')
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
    return _operand(prog, x[..., k] if isinstance(x, Traced) else x[..., k], shape)


def _zeros(prog, shape, dtype=float):
    root = np.zeros(shape, dtype=dtype)
    buf = prog.buffer(root, 'zeros')
    result = Traced(prog, buf, root)
    prog.add('fill', result.ref(), (0,))
    return result


FUNCTIONS = {'einsum': _einsum, 'cross': _cross}
#: numpy's array makers her code lays a traced buffer with, called from her code.
MAKERS = {'zeros': _zeros}
#: One trace at a time: numpy's makers patched by one alone.
_ONE = threading.Lock()


def traced(fn, args, inputs, kwargs=None):
    """(the Program, fn's own result) of `fn(*args, **kwargs)`, the arrays at `inputs` traced: numpy's
    makers patched for her code's own calls in this thread while it runs - another thread's, numpy's."""
    prog = Program()
    args = list(args)
    for k in inputs:
        root = np.ascontiguousarray(args[k], dtype=float)
        buf = prog.buffer(root.copy(), 'in')
        args[k] = Traced(prog, buf, buf.root)
        prog.ins.append(buf)
    with _ONE:
        kept = {name: getattr(np, name) for name in MAKERS}
        tracing = threading.get_ident()

        def maker(name):
            real = kept[name]

            def made(*a, **k):
                caller = sys._getframe(1).f_code.co_filename
                if threading.get_ident() != tracing or 'numpy' in caller or caller in _HERE:
                    return real(*a, **k)
                return MAKERS[name](prog, *a, **k)
            return made
        try:
            for name in MAKERS:
                setattr(np, name, maker(name))
            got = fn(*args, **(kwargs or {}))
        finally:
            for name, real in kept.items():
                setattr(np, name, real)
    for out in (got if isinstance(got, tuple) else (got,)):
        if not isinstance(out, Traced):
            raise Untraced('an output not of her traced arrays: %r' % type(out))
        prog.outs.append(out.ref())
    return prog, got
