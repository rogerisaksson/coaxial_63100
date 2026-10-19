"""Her numpy compiled to C and held bit for bit against numpy's own run of it.

Traced on her arrays (`trace`), written in C (`emit`), built once a source; held on the arrays it
was traced on, then on fresh ones.

    fn_c = held.compiled(fn, args, inputs, 'regressor')   # fn_c(*arrays at `inputs`) its C
    fn_c = held.batched(fn, args, inputs, 'regressor')    # the same over any number of states
    fn_c = held.batched(fn, args, inputs, 'regressor', made_of=(module, lay))   # found again untraced
"""
import ctypes
import hashlib
import itertools
import json
import os
import pickle
import subprocess
import sys

import numpy as np

from tools.cores import build, emit, trace
from tools.cores.wbc import FLAGS, INCLUDES


#: Generated C as her chain's library is built, at -O3: a loop nest a statement, numpy's arithmetic
#: the same at every level (ISO C contracts nothing), the regressor 8.0 -> 5.95 ms at 400 states.
GENERATED = FLAGS + ('-O3',)
#: The states a batched build is traced and held on, at most: numpy's orders read and its bits held
#: at as many, one state's program the same at any number (a first build 1.42 GB at 400).
STATES = 64
#: A build apart's commit, booked in the relay's ledger: a first build peaks 1.36 GB, gcc 0.79 of it.
APART_GB = 1.5
_count = itertools.count()


class Parted(AssertionError):

    """Her C and her numpy apart: what was compiled is not hers."""


def digest(*parts):
    """A key of what a build is made of: strings, bytes, arrays, modules' sources, nested."""
    h = hashlib.sha1()

    def feed(x):
        if isinstance(x, (list, tuple)):
            for y in x:
                feed(y)
        elif isinstance(x, dict):
            for k in sorted(x):
                feed(k)
                feed(x[k])
        elif isinstance(x, np.ndarray):
            h.update(str((x.dtype, x.shape)).encode())
            h.update(np.ascontiguousarray(x).tobytes())
        elif hasattr(x, '__file__'):
            with open(x.__file__, 'rb') as f:
                h.update(f.read())
        else:
            h.update(repr(x).encode())
    feed(parts)
    for module in (trace, emit):
        feed(module)
    with open(__file__, 'rb') as f:
        h.update(f.read())
    features = getattr(getattr(getattr(np, '_core', None), '_multiarray_umath', None), '__cpu_features__', {})
    feed((np.__version__, sorted(k for k, on in features.items() if on), GENERATED))
    return h.hexdigest()[:16]


def _build(source, name):
    folder = os.path.join(build.OUT, 'gen')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, '%s_%s.c' % (name, hashlib.sha1(source.encode()).hexdigest()[:12]))
    if not os.path.exists(path):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(source)
    found = build.cached([path], INCLUDES, name, GENERATED)[0]
    lib = ctypes.CDLL(found)
    fn = getattr(lib, name)
    fn.restype = None
    return lib, fn, found


def compiled(fn, args, inputs, name, fresh=2, seed=1):
    """`fn(*args)` compiled for the arrays at `inputs` (their shapes as traced, the rest of `args` as
    they are): a callable of those arrays returning fn's own (the C's), held against numpy's own on
    the traced arrays and on `fresh` draws of new ones - Parted where any element differs."""
    inputs = list(inputs)
    prog, _got = trace.traced(fn, args, inputs)
    source, info = emit.c(prog, name)
    lib, cfn, _found = _build(source, name)
    cfn.argtypes = [ctypes.c_void_p] * 3
    shapes = [b.root.shape for b in prog.ins]
    work = np.empty(max(1, info['doubles']))

    def run(*arrays):
        ins = [np.ascontiguousarray(a, dtype=float) for a in arrays]
        if [a.shape for a in ins] != shapes:
            raise ValueError('%s was compiled for %s, given %s' % (name, shapes, [a.shape for a in ins]))
        outs = [np.empty(r.shape) for r in prog.outs]
        ip = (ctypes.c_void_p * len(ins))(*[a.ctypes.data for a in ins])
        op = (ctypes.c_void_p * len(outs))(*[o.ctypes.data for o in outs])
        cfn(ip, op, work.ctypes.data)
        return outs[0] if len(outs) == 1 else tuple(outs)

    return _held(run, fn, args, inputs, shapes, fresh, seed, name, lib, source, info, prog)


def batched(fn, args, inputs, name, fresh=2, seed=1, made_of=None):
    """`fn(*args)` compiled as one state's program run over any number: the arrays at `inputs` each
    a state along their first axis, fn's outputs too; traced at one state for its program and at the
    states given for numpy's orders, held against numpy's own run at those states and on `fresh` draws.
    `made_of` what fn's arithmetic is made of beyond its arrays' shapes (its modules, its constants):
    a build kept under it is loaded again untraced."""
    inputs = list(inputs)
    rest = [tuple(np.shape(args[k])[1:]) for k in inputs]
    note = None if made_of is None else _note(name, made_of, rest)
    run = None if note is None else found(name, made_of, rest)
    if run is not None:
        return run
    one, args = list(args), list(args)
    for k in inputs:
        one[k] = np.ascontiguousarray(args[k][:1], dtype=float)
        args[k] = np.ascontiguousarray(args[k][:STATES], dtype=float)
    prog, _ = trace.traced(fn, one, inputs)
    many, _ = trace.traced(fn, args, inputs)
    kinds = [(st.kind, st.args[0] if st.kind == 'einsum' else None) for st in prog.statements]
    if kinds != [(st.kind, st.args[0] if st.kind == 'einsum' else None) for st in many.statements]:
        raise Parted('%s: one state\'s program is not each of many states\'' % name)
    orders = [emit.ordered(st) for st in many.statements if st.kind == 'einsum']
    source, info = emit.c(prog, name, orders, batch=True)
    lib, _cfn, made = _build(source, name)
    outs = [tuple(r.shape[1:]) for r in prog.outs]
    run = _batched_run(lib, name, rest, outs, info['doubles'])
    shapes = [np.shape(args[k]) for k in inputs]
    held = _held(run, fn, args, inputs, shapes, fresh, seed, name, lib, source, info, prog)
    if note is not None:
        with open(note + '.%d' % os.getpid(), 'w', encoding='utf-8') as f:
            json.dump({'lib': made, 'ins': rest, 'outs': outs, 'doubles': info['doubles']}, f)
        os.replace(note + '.%d' % os.getpid(), note)
    return held


def _note(name, made_of, rest):
    return os.path.join(build.OUT, 'gen', '%s_%s.json' % (name, digest(made_of, [tuple(r) for r in rest])))


def kept(name, made_of, rest):
    """What was kept of a batched build under `made_of` for states of `rest`: its note, None where none
    is - a note whose 'lib' is None a build this host could not make."""
    note = _note(name, made_of, rest)
    if not os.path.exists(note):
        return None
    with open(note, encoding='utf-8') as f:
        return json.load(f)


def refused(name, made_of, rest, why):
    """A build this host cannot make kept as such: not tried again for the same arithmetic."""
    note = _note(name, made_of, rest)
    os.makedirs(os.path.dirname(note), exist_ok=True)
    with open(note + '.%d' % os.getpid(), 'w', encoding='utf-8') as f:
        json.dump({'lib': None, 'why': str(why)}, f)
    os.replace(note + '.%d' % os.getpid(), note)


def apart(module, fn, payload):
    """`module.fn(*payload)` in a process of its own - a build's trace holds its own interpreter, not
    the caller's -, `payload` pickled beside the builds, the process booked in the relay's ledger
    (APART_GB) while it runs. (the process, running; its booking) - its words in gen/<its payload>.log."""
    from tools.dev import focus
    folder = os.path.join(build.OUT, 'gen')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, 'payload_%d_%d.pkl' % (os.getpid(), next(_count)))
    with open(path, 'wb') as f:
        pickle.dump(payload, f)
    host = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    code = ('import os, pickle, sys\nfrom %s import %s as f\nwith open(sys.argv[1], "rb") as g:\n    a = pickle.load(g)\n'
            'os.remove(sys.argv[1])\nf(*a)\n' % (module, fn))
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1',
               PYTHONPATH=os.pathsep.join([host] + [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]))
    with open(path[:-4] + '.log', 'w', encoding='utf-8') as log:
        proc = subprocess.Popen([sys.executable, '-c', code, path], cwd=host, env=env, stdout=log,
                                stderr=subprocess.STDOUT)
    booking = os.path.join(focus.LEDGER, 'apart-%d-%d' % (os.getpid(), proc.pid))
    try:
        focus.booked(booking, APART_GB, proc.pid)
    except OSError:
        pass
    return proc, booking


def found(name, made_of, rest):
    """A batched build kept under `made_of` for states of `rest`, loaded untraced; None where none is."""
    kept_ = kept(name, made_of, rest)
    if kept_ is None or kept_['lib'] is None or not os.path.exists(kept_['lib']):
        return None
    with open(_note(name, made_of, rest), encoding='utf-8') as f:
        kept_ = json.load(f)
    return _batched_run(ctypes.CDLL(kept_['lib']), name, [tuple(r) for r in kept_['ins']],
                        [tuple(r) for r in kept_['outs']], kept_['doubles'])


def _batched_run(lib, name, rest, outs, doubles):
    """A batched build's callable: its states' arrays in, fn's outputs over as many states out, a tuple."""
    cfn = getattr(lib, name)
    cfn.argtypes, cfn.restype = [ctypes.c_long] + [ctypes.c_void_p] * 3, None
    work = np.empty(max(1, doubles))

    def run(*arrays):
        ins = [np.ascontiguousarray(a, dtype=float) for a in arrays]
        n = len(ins[0])
        if [a.shape for a in ins] != [(n,) + tuple(r) for r in rest]:
            raise ValueError('%s takes states of %s, given %s' % (name, rest, [a.shape for a in ins]))
        got = [np.empty((n,) + tuple(r)) for r in outs]
        ip = (ctypes.c_void_p * len(ins))(*[a.ctypes.data for a in ins])
        op = (ctypes.c_void_p * len(got))(*[o.ctypes.data for o in got])
        cfn(n, ip, op, work.ctypes.data)
        return tuple(got)
    run.lib = lib
    return run


def _held(run, fn, args, inputs, shapes, fresh, seed, name, lib, source, info, prog):
    """`run` held bit for bit against `fn` on the traced arrays and `fresh` draws of new ones."""
    rng = np.random.default_rng(seed)
    draws = [[args[k] for k in inputs]] + [[rng.normal(size=s) for s in shapes] for _ in range(fresh)]
    for d, arrays in enumerate(draws):
        full = list(args)
        for k, a in zip(inputs, arrays):
            full[k] = a
        want = fn(*full)
        got = run(*arrays)
        for k, (g, w_) in enumerate(zip(got if isinstance(got, tuple) else (got,), want if isinstance(want, tuple) else (want,))):
            if not emit.same(g, w_):
                apart = np.ascontiguousarray(g).view(np.int64) != np.ascontiguousarray(w_, dtype=float).view(np.int64)
                raise Parted('%s output %d on %s: %d of %d elements apart, the first at %s' % (
                    name, k, 'its traced arrays' if d == 0 else 'fresh arrays %d' % d, int(apart.sum()), apart.size,
                    np.unravel_index(int(np.argmax(apart)), apart.shape)))
    run.lib, run.source, run.info, run.prog = lib, source, info, prog
    return run
