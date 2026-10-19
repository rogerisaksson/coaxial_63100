"""Her numpy compiled on her memory: numpy's own until its C is built and held, then the C.

A function of her arrays, and of her body with its MuJoCo traced (`trace_mj`): each array it is
given an extern, read and written in its own memory - a dict's float arrays too, an entry of it
rebound written back into the array there; a number it is given traced; what else it is given or
reads of her body a constant of the trace, the same objects or it stays numpy's.

    sense = bodied.on_body(wbc.sense, 'sense')               # her body first, the arrays at (1, 2)
    felt = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3))   # the rest of its arguments constants
    s = sense(b, qpos, qvel)                                 # wbc.sense(b, qpos, qvel), bit for bit

At its first call traced on copies - her body's a twin of it - and written in C; built in a thread,
gcc's process its work; at the first call after, held bit for bit on copies at that call's
arguments and fresh draws, its result and every argument after; then the C. Not held, it stays
numpy's own, said once on stderr.
"""
import ctypes
import hashlib
import sys
import threading

import numpy as np

from tools.cores import emit, held, layout, trace, trace_mj

#: The builds under way in this process by their source's digest, a thread each.
_BUILDING = {}
_LOCK = threading.Lock()


def on_body(fn, name, inputs=(1, 2), fresh=2, seed=1, body=0):
    """`fn(..., b, ...)`, her body at `body`, the arrays at `inputs` (Compiled)."""
    return Compiled(fn, name, inputs, body, fresh, seed)


def on_args(fn, name, inputs, fresh=2, seed=1):
    """`fn(...)` of the arrays, numbers and dicts of arrays at `inputs` (Compiled)."""
    return Compiled(fn, name, inputs, None, fresh, seed)


class Compiled:

    """`fn` compiled on her memory: numpy's own until its C is held, then the C."""

    def __init__(self, fn, name, inputs, body, fresh, seed):
        self.fn, self.name, self.inputs, self.body, self.fresh, self.seed = fn, name, list(inputs), body, fresh, seed
        self.made: tuple | None = None
        self.job: dict | None = None
        self.c: Bound | None = None
        self.why: Exception | None = None

    @property
    def last(self):
        """(its C's last result, the address of the block it lies in), None before the C."""
        return None if self.c is None else self.c.last

    def __call__(self, *args):
        c = self.c
        kept = None if c is None else c.fits(args)
        if kept is None and self.why is None:
            try:
                self._advance(args)
            except (trace.Untraced, held.Parted, RuntimeError, OSError) as e:
                self.why, self.c = e, None
                print('%s stays numpy: %s' % (self.name, e), file=sys.stderr)
            c = self.c
            kept = None if c is None else c.fits(args)
        return self.fn(*args) if c is None or kept is None else c(kept)

    def wait(self, *args):
        """Built and held at `args` now: its C (Bound), or why it stays numpy raised."""
        self(*args)
        if self.job is not None:
            self.job['done'].wait()
        self(*args)
        if self.why is not None:
            raise self.why
        if self.c is None:
            raise trace.Untraced("%s: its constants not the body's" % self.name)
        return self.c

    def _advance(self, args):
        if self.made is None:
            self._trace(args)
            return
        prog, info, twin, specs = self.made
        job = self.job or {}
        if self.c is None:
            if not job['done'].is_set():
                return
            if 'why' in job:
                raise job['why']
            lib, cfn = job['lib']
            c = Bound(lib, cfn, prog, info, specs, self.body)
            self._hold(c, args, twin)
            self.c = c
        if self.body is not None:
            b = args[self.body]
            if any(getattr(b, k, None) is not v for k, v in prog.consts.items()):
                self.made = self.job = self.c = None
                return
            self.c.bind(b)
        if self.c.fits(args) is None:
            raise trace.Untraced('%s given what it was not traced on' % self.name)

    def _trace(self, args):
        """Traced on copies - her body's a twin -, written in C, its build begun."""
        given, twin, specs = list(args), None, []
        if self.body is not None:
            twin = given[self.body] = trace_mj.twin(args[self.body])

        def wrap(prog, got):
            got = list(got)
            if twin is not None:
                got[self.body] = trace_mj.Body(prog, twin)
            for k in self.inputs:
                got[k] = _tree_in(prog, k, (), args[k], specs, True)
            return got
        prog, _got = trace.traced(self.fn, given, (), wrap=wrap, after=lambda prog, got: _rebound(prog, got, specs))
        specs.extend(('guard', k, (), args[k]) for k in range(len(args)) if k not in self.inputs and k != self.body)
        source, info = emit.c(prog, self.name)
        self.made, self.job = (prog, info, twin, specs), building(source, self.name, held.matmuls(prog))

    def _hold(self, c, args, twin):
        """`c` held bit for bit against `fn` on copies, at the call's arguments then `fresh` draws: its
        result and every argument after."""
        rng = np.random.default_rng(self.seed)
        if twin is not None:
            c.bind(twin)
        for d in range(1 + self.fresh):
            ours = [_copy_in(args[k], rng if d else None, True) for k in self.inputs]
            theirs = [_copy_in(x, None, True) for x in ours]
            mine, other = list(args), list(args)
            if twin is not None:
                mine[self.body] = other[self.body] = twin
            for k, x, y in zip(self.inputs, ours, theirs):
                mine[k], other[k] = x, y
            with np.errstate(all='ignore'):
                want = self.fn(*mine)
            kept = c.fits(other)
            if kept is None:
                raise held.Parted('%s: its own copies do not fit it' % self.name)
            on = 'its copies of %s' % ("the call's arguments" if d == 0 else 'fresh draws %d' % d)
            held.alike(trace.leaves_of(c.prog.shape, want), trace.leaves_of(c.prog.shape, c(kept)), self.name, on)
            held.alike(dict(enumerate(_leaves_in(ours))), dict(enumerate(_leaves_in(theirs))), self.name + "'s arguments", on)


def _tree_in(prog, k, path, v, specs, top):
    """Argument `k`'s `v` at `path` as traced: an array (or at the top a number) an extern over a copy
    of it; a dict or a list of arrays its own of each; anything else itself, pinned (`specs`)."""
    if isinstance(v, dict):
        return {key: _tree_in(prog, k, path + (key,), x, specs, False) for key, x in v.items()}
    if isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
        return type(v)(_tree_in(prog, k, path + (i,), x, specs, False) for i, x in enumerate(v))
    if (isinstance(v, np.ndarray) and v.dtype.kind in 'fb') or (top and np.asarray(v).dtype.kind in 'fbiu'):
        name = 'a%d%s' % (k, ''.join('/%s' % p for p in path))
        traced = prog.extern(name, np.array(v, dtype=float))
        specs.append(('array', k, path, name, traced))
        return traced
    specs.append(('pin', k, path, v))
    return v


def _rebound(prog, args, specs):
    """A dict entry fn rebound to an array of its own written back into the extern it was."""
    for spec in [s for s in specs if s[0] == 'array' and s[2]]:
        _kind, k, path, name, extern = spec
        node = args[k]
        for p in path[:-1]:
            node = node[p]
        now = node[path[-1]]
        if now is extern:
            continue
        if not isinstance(now, trace.Traced) or now.shape != extern.shape:
            raise trace.Untraced('%s rebound to a %s' % (name, type(now).__name__))
        prog.add('ew', extern.ref(), ('positive', ('ref', now.ref())))


def _copy_in(v, rng, top):
    """`v` with each array `_tree_in` makes an extern copied (or drawn afresh from `rng`), the rest itself."""
    if isinstance(v, dict):
        return {key: _copy_in(x, rng, False) for key, x in v.items()}
    if isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
        return type(v)(_copy_in(x, rng, False) for x in v)
    if (isinstance(v, np.ndarray) and v.dtype.kind in 'fb') or (top and np.asarray(v).dtype.kind in 'fbiu'):
        a = np.array(v)
        if rng is not None:
            a = rng.normal(size=a.shape) > 0.0 if a.dtype == bool else rng.normal(size=a.shape).astype(a.dtype)
        return a if isinstance(v, np.ndarray) or a.ndim else a[()]
    return v


def _leaves_in(trees):
    """The arrays `_copy_in` copies, in order."""
    out = []
    for v in trees:
        if isinstance(v, dict):
            out += _leaves_in(list(v.values()))
        elif isinstance(v, (list, tuple)) and any(isinstance(x, (dict, list, tuple, np.ndarray)) for x in v):
            out += _leaves_in(list(v))
        elif isinstance(v, np.ndarray):
            out.append(v)
    return out


def building(source, name, matmul):
    """The build of `source` under way or done, {'done': its Event, 'lib': (the library, its function)
    or 'why'}: one thread a source in this process, the library loaded there - a new one's first load
    40 ms of the walking process."""
    key = hashlib.sha1(source.encode()).hexdigest()
    with _LOCK:
        job = _BUILDING.get(key)
        if job is None:
            job = _BUILDING[key] = {'done': threading.Event()}
            threading.Thread(target=_build, args=(job, source, name, matmul), name='held ' + name, daemon=True).start()
    return job


def _build(job, source, name, matmul):
    try:
        job['lib'] = held.loaded(held.built(source, name, matmul), name)
    except (RuntimeError, OSError) as e:
        job['why'] = e
    job['done'].set()


class Bound:

    """A build's C on her memory: its workspace laid once; her body's externs, her model's, her data's
    and MuJoCo's addresses where bound; its arguments' addressed at each call; fn's result laid back
    of a fresh output block a call."""

    def __init__(self, lib, cfn, prog, info, specs=(), body=None):
        self.lib, self.cfn, self.prog, self.at = lib, cfn, prog, body
        self.consts = list(prog.consts.items())
        index = {e.name: i for i, e in enumerate(prog.externs)}
        written = {b.id for st in prog.statements for b in layout.writes(st)}
        self.args = [(index[s[3]], s[1], s[2], s[4].buffer.id in written, s[4].shape) for s in specs if s[0] == 'array']
        self.pins = [s[1:] for s in specs if s[0] in ('pin', 'guard')]
        self.table = (ctypes.c_void_p * max(1, len(prog.externs)))()
        self.words = np.frombuffer(self.table, dtype=np.uint64)
        spans, self.total = info['outs']
        self.spans = [a for a, _b in spans]
        self.made = maker(prog.shape, [(a, b, r.shape, r.buffer.dtype) for (a, b), r in zip(spans, prog.outs)])
        self.work = np.empty(max(1, info['doubles']))
        self.lapack = ctypes.addressof(held.blas()) if held.matmuls(prog) else None
        self.funcs = (ctypes.c_void_p * max(1, len(prog.calls)))(*trace_mj.functions(prog))
        self.body, self.kept, self.model, self.data = None, [], None, None
        self.last: tuple | None = None

    def bind(self, b):
        """Her body `b` addressed - its externs, her model, her data."""
        arrays, self.model, self.data = trace_mj.bound(self.prog, b)
        for i, arr in arrays:
            self.words[i] = arr.ctypes.data
        self.kept, self.body = [arr for _i, arr in arrays], b

    def fits(self, args):
        """The arrays converted for this call, its arguments' externs addressed; None where `args` are
        not as traced - her body not the one bound, a constant not the same, an array not its shape."""
        if self.at is not None:
            b = args[self.at]
            if b is not self.body or any(getattr(b, k) is not v for k, v in self.consts):
                return None
        for k, path, v in self.pins:
            a = args[k]
            for p in path:
                a = a[p]
            if a is not v and (isinstance(v, np.ndarray) or type(a) is not type(v) or a != v):
                return None
        kept = []
        for i, k, path, written, shape in self.args:
            a = args[k]
            for p in path:
                a = a[p]
            if type(a) is not np.ndarray or a.dtype != np.float64 or not a.flags.c_contiguous:
                if written:
                    return None
                a = np.array(a, dtype=np.float64, order='C')
                kept.append(a)
            if a.shape != shape:
                return None
            self.words[i] = a.__array_interface__['data'][0]
        return kept

    def __call__(self, kept):
        flat = np.empty(self.total) if self.total else None
        address = None if flat is None else flat.ctypes.data
        self.cfn(None, address, self.work.ctypes.data, ctypes.addressof(self.table), self.lapack, self.model,
                 self.data, ctypes.addressof(self.funcs))
        got = self.made(flat)
        self.last = (got, address)
        return got

    def place(self, path):
        """Where in its output block the leaf at `path` (keys and indices into fn's result) starts."""
        node = self.prog.shape
        for key in path:
            kind, body = node
            node = dict(body)[key] if kind == 'dict' else body[key]
        if node[0] != 'leaf':
            raise KeyError(path)
        return self.spans[node[1]]


def maker(shape, outs):
    """fn's result laid back of an output block, a function of it: each leaf a view of the block at
    its span (a 0-d one numpy's scalar, a bool one numpy's bools), its dicts, lists and tuples as
    traced (`trace.unflatten`)."""
    consts, lines = [], []
    for k, (a, b, s, dtype) in enumerate(outs):
        view = 'f[%d]' % a if not s else 'f[%d:%d]' % (a, b) if len(s) == 1 else 'f[%d:%d].reshape(%r)' % (a, b, tuple(s))
        lines.append('    l%d = %s' % (k, view + ' != 0.0' if dtype == bool else view))

    def put(x):
        consts.append(x)
        return 'c[%d]' % (len(consts) - 1)

    def code(node):
        kind, body = node
        if kind == 'leaf':
            return 'l%d' % body
        if kind == 'dict':
            return '{%s}' % ', '.join('%s: %s' % (put(k), code(v)) for k, v in body)
        if kind in ('list', 'tuple'):
            items = [code(v) for v in body]
            return '[%s]' % ', '.join(items) if kind == 'list' else '(%s)' % ''.join(x + ', ' for x in items)
        return put(body)
    text = 'def made(f):\n%s\n    return %s\n' % ('\n'.join(lines), code(shape))
    space: dict = {'c': consts}
    exec(text, space)
    return space['made']
