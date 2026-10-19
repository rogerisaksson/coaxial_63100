"""Her numpy compiled on her body: numpy's own until its C is built and held, then the C.

A function of her body and her arrays, her MuJoCo traced (`trace_mj`).

    sense = bodied.on_body(wbc.sense, 'sense')
    s = sense(b, qpos, qvel)                # wbc.sense(b, qpos, qvel), bit for bit

At its first call traced on a twin of her body and written in C; built in a thread, gcc's process
its work and not this interpreter; at the first call after, held bit for bit on the twin at that
call's arrays and fresh draws; then the C on her own body - her data's arrays and her scratch
addressed once, the constants the trace read the same objects or it is traced again. Not held, it
stays numpy's own, said once on stderr.
"""
import ctypes
import hashlib
import sys
import threading

import numpy as np

from tools.cores import emit, held, trace, trace_mj

#: The builds under way in this process by their source's digest, a thread each.
_BUILDING = {}
_LOCK = threading.Lock()


def on_body(fn, name, inputs=(1, 2), fresh=2, seed=1):
    """`fn(b, *arrays)` - her body first, the arrays at `inputs` - compiled on her body (OnBody)."""
    return OnBody(fn, name, inputs, fresh, seed)


class OnBody:

    """`fn` on her body: numpy's own until its C is held, then the C."""

    def __init__(self, fn, name, inputs, fresh, seed):
        self.fn, self.name, self.inputs, self.fresh, self.seed = fn, name, list(inputs), fresh, seed
        self.made: tuple | None = None
        self.job: dict | None = None
        self.c: Bound | None = None
        self.why: Exception | None = None

    def __call__(self, *args):
        c = self.c
        if c is not None and c.body is args[0] and c.fits(args):
            return c([args[k] for k in self.inputs])
        if self.why is None:
            try:
                self._advance(args)
            except (trace.Untraced, held.Parted, RuntimeError, OSError) as e:
                self.why, self.c = e, None
                print('%s stays numpy: %s' % (self.name, e), file=sys.stderr)
        c = self.c
        if c is not None and c.body is args[0] and c.fits(args):
            return c([args[k] for k in self.inputs])
        return self.fn(*args)

    @property
    def last(self):
        """(its C's last result, the address of the block it lies in), None before the C."""
        return None if self.c is None else self.c.last

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
        b = args[0]
        if self.made is None:
            twin = trace_mj.twin(b)
            prog, _got = trace.traced(self.fn, [twin] + list(args[1:]), self.inputs, wrap=trace_mj.wrap(0))
            source, info = emit.c(prog, self.name)
            self.made, self.job = (prog, info, twin), building(source, self.name, held.matmuls(prog))
            return
        prog, info, twin = self.made
        job = self.job or {}
        if self.c is None:
            if not job['done'].is_set():
                return
            if 'why' in job:
                raise job['why']
            lib, cfn = job['lib']
            c = Bound(lib, cfn, prog, info, self.inputs)
            self._hold(c, args, twin)
            self.c = c
        if any(getattr(b, k, None) is not v for k, v in self.c.consts):
            self.made = self.job = self.c = None
            return
        shapes = [np.shape(args[k]) for k in self.inputs]
        if shapes != self.c.shapes:
            raise trace.Untraced('%s given %s, traced %s' % (self.name, shapes, self.c.shapes))
        self.c.bind(b)

    def _hold(self, c, args, twin):
        """`c` held bit for bit against `fn` on the twin: at the call's arrays, then `fresh` draws."""
        prog = c.prog
        c.bind(twin)
        rng = np.random.default_rng(self.seed)
        draws = [[np.asarray(args[k], dtype=float) for k in self.inputs]]
        draws += [[rng.normal(size=np.shape(args[k])) for k in self.inputs] for _ in range(self.fresh)]
        for d, arrays in enumerate(draws):
            full = [twin] + list(args[1:])
            for k, a in zip(self.inputs, arrays):
                full[k] = a
            want = trace.leaves_of(prog.shape, self.fn(*full))
            held.alike(want, trace.leaves_of(prog.shape, c(arrays)), self.name,
                       'its twin at %s' % ("the call's arrays" if d == 0 else 'fresh arrays %d' % d))


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

    """A build's C bound to her body: its input block and workspace laid once, her externs', her
    model's, her data's and MuJoCo's addresses; fn's result laid back of a fresh output block a call."""

    def __init__(self, lib, cfn, prog, info, inputs=()):
        self.lib, self.cfn, self.prog, self.inputs = lib, cfn, prog, list(inputs)
        self.consts = list(prog.consts.items())
        self.shapes = [b.root.shape for b in prog.ins]
        spans, total = info['ins']
        self.block = np.empty(max(1, total))
        self.views = [self.block[a:b].reshape(s) for (a, b), s in zip(spans, self.shapes)]
        spans, self.total = info['outs']
        self.spans = [a for a, _b in spans]
        self.made = maker(prog.shape, [(a, b, r.shape) for (a, b), r in zip(spans, prog.outs)])
        self.work = np.empty(max(1, info['doubles']))
        self.lapack = ctypes.addressof(held.blas()) if held.matmuls(prog) else None
        self.funcs = (ctypes.c_void_p * max(1, len(prog.calls)))(*trace_mj.functions(prog))
        self.body = self.kept = self.ext = None
        self.fixed: tuple = ()
        self.last: tuple | None = None

    def bind(self, b):
        """Her body `b` addressed - its externs, her model, her data; None, none."""
        arrays, model, data = trace_mj.bound(self.prog, b) if b is not None else ([], None, None)
        self.kept = arrays
        self.ext = (ctypes.c_void_p * max(1, len(arrays)))(*[a.ctypes.data for a in arrays])
        self.fixed = (self.block.ctypes.data, self.work.ctypes.data, ctypes.addressof(self.ext), self.lapack,
                      model, data, ctypes.addressof(self.funcs))
        self.body = b

    def fits(self, args):
        """Whether the constants the trace read are `args`' body's and its arrays the shapes traced."""
        b = args[0]
        for k, v in self.consts:
            if getattr(b, k) is not v:
                return False
        return [np.shape(args[k]) for k in self.inputs] == self.shapes

    def __call__(self, arrays):
        for view, a in zip(self.views, arrays):
            view[...] = a
        flat = np.empty(self.total)
        i, w, e, lapack, m, d, f = self.fixed
        address = flat.ctypes.data
        self.cfn(i, address, w, e, lapack, m, d, f)
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
    its span (a 0-d one numpy's scalar), its dicts, lists and tuples as traced (`trace.unflatten`)."""
    consts, lines = [], []
    for k, (a, b, s) in enumerate(outs):
        view = 'f[%d]' % a if not s else 'f[%d:%d]' % (a, b) if len(s) == 1 else 'f[%d:%d].reshape(%r)' % (a, b, tuple(s))
        lines.append('    l%d = %s' % (k, view))

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
