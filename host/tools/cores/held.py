"""The robot's numpy compiled to C and held bit for bit against numpy's own run of it.

Traced on the robot's arrays (`trace`), written in C (`emit`), built once a source; held on the arrays it
was traced on, then on fresh ones.

    fn_c = held.compiled(fn, args, inputs, 'name')        # fn_c(*arrays at `inputs`) its C
    fn_c = held.batched(fn, args, inputs, 'regressor')    # the same over any number of states
    fn_c = held.batched(fn, args, inputs, 'regressor', made_of=(module, lay))   # found again untraced
    fn_c = bodied.on_body(wbc.sense, 'sense')             # fn(b, *arrays) on the robot's body (`bodied`)
"""
from concurrent.futures import ThreadPoolExecutor
import ctypes
import functools
import hashlib
import itertools
import json
import os
import pickle
import subprocess
import sys
import threading

import numpy as np

from tools import REPO
from tools.cores import build, emit, layout, orders, trace, trace_mj, trace_np
from tools.cores.wbc import FLAGS, INCLUDES

#: Generated C as the robot's chain's library is built, at -O3: a loop nest a statement, numpy's arithmetic
#: the same at every level (ISO C contracts nothing), the regressor 8.0 -> 5.95 ms at 400 states.
GENERATED = FLAGS + ('-O3',)
#: The states a batched build is traced and held on, at most: numpy's orders read and its bits held
#: at as many, one state's program the same at any number (a first build 1.42 GB at 400).
STATES = 64
#: A build apart's commit, booked in the relay's ledger: a first build peaks 1.36 GB, gcc 0.79 of it.
APART_GB = 1.5
#: numpy's matmul in C (`wbc_matmul`), built beside a program that has one.
BLAS_C = os.path.join(REPO, 'wbc', 'src', 'wbc_blas.c')
#: The robot's libm's reduction past 2^22 (wbc_libm.c's lm_reduce_far, its arrays' runs left out), built beside a
#: program that calls its sin, cos or tan (FAR); the rest of the libm a program inlines from its header.
LIBM_C, FAR, FAR_ONLY = os.path.join(REPO, 'wbc', 'src', 'wbc_libm.c'), ('wbc_sin(', 'wbc_cos(', 'wbc_tan('), '-DWBC_LIBM_FAR_ONLY'
_count = itertools.count()
#: The builds of this process by their source's digest, WIDTH threads building them in turn.
_BUILDING = {}
_LOCK = threading.Lock()
_QUEUE: list = []
#: The threads building now; at most WIDTH: one beside the robot's loop, a warm's as many as it is given (`widened`).
_RUNNING, WIDTH = [0], [1]
_LOADERS = ThreadPoolExecutor(max_workers=4, thread_name_prefix='held-load')


@functools.cache
def _include_bytes():
    """Header digest bytes for `INCLUDES`, read once per process (`build.headers`)."""
    return build.headers(INCLUDES)


def _cached_path(source, name, matmul):
    """Existing built library path for `(source, name, matmul)` if already on disk, else `None`."""
    folder = os.path.join(build.OUT, 'gen')
    key = hashlib.sha1(source.encode()).hexdigest()[:12]
    src = os.path.join(folder, '%s_%s.c' % (name, key))
    if not os.path.exists(src):
        return None
    far = any(call in source for call in FAR)
    sources = [src] + ([BLAS_C] if matmul else []) + ([LIBM_C] if far else [])
    extra = GENERATED + ((FAR_ONLY,) if far else ())
    digest = hashlib.sha1(' '.join(build.FLAGS + list(extra)).encode())
    digest.update(str(bool(os.environ.get('COAXIAL_GCOV'))).encode())
    digest.update((build._host() if any(flag.endswith('=native') for flag in extra) else '').encode())
    for path in sources:
        if not os.path.exists(path):
            return None
        with open(path, 'rb') as f:
            digest.update(f.read())
    digest.update(_include_bytes())
    ext = '.dll' if os.name == 'nt' else '.so'
    lib = os.path.join(build.OUT, '%s_%s_%s%s' % (name, key, digest.hexdigest()[:12], ext))
    return lib if os.path.exists(lib) else None


class Parted(AssertionError):

    """The robot's C and its numpy apart: what was compiled is not its own."""


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
    for module in (trace, trace_np, trace_mj, emit, layout, orders):
        feed(module)
    with open(__file__, 'rb') as f:
        h.update(f.read())
    features = getattr(getattr(getattr(np, '_core', None), '_multiarray_umath', None), '__cpu_features__', {})
    feed((np.__version__, sorted(k for k, on in features.items() if on), GENERATED))
    return h.hexdigest()[:16]


def built(source, name, matmul, heads=None):
    """The library of `source` built once, its path: the C kept beside the builds, numpy's matmul
    built with it where it has one and the libm's far reduction where it calls its sin, cos or tan, named by its source - a process builds any number at once; `heads`
    its headers' bytes as a round of builds read them (`build.headers`)."""
    folder = os.path.join(build.OUT, 'gen')
    os.makedirs(folder, exist_ok=True)
    key = hashlib.sha1(source.encode()).hexdigest()[:12]
    path = os.path.join(folder, '%s_%s.c' % (name, key))
    if not os.path.exists(path):
        with open(path + '.%d' % os.getpid(), 'w', encoding='utf-8', newline='\n') as f:
            f.write(source)
        os.replace(path + '.%d' % os.getpid(), path)
    far = any(call in source for call in FAR)
    return build.cached([path] + ([BLAS_C] if matmul else []) + ([LIBM_C] if far else []), INCLUDES, '%s_%s' % (name, key),
                        GENERATED + ((FAR_ONLY,) if far else ()), heads)[0]


def loaded(path, name, batch=False, solves=False):
    """(the library at `path`, its `name` typed: [long n, ]in, out, w, ext, blas, model, data, call -
    an int where it `solves`, 1 where numpy raises LinAlgError)."""
    lib = ctypes.CDLL(path)
    fn = getattr(lib, name)
    fn.restype = ctypes.c_int if solves else None
    fn.argtypes = ([ctypes.c_long] if batch else []) + [ctypes.c_void_p] * 8
    return lib, fn


def building(source, name, matmul, solves=False):
    """The build of `source` under way or done, {'done': its Event, 'lib': (the library, its function)
    or 'why'}: cached libraries on disk load across `_LOADERS`, while new sources compile on `WIDTH` threads."""
    key = hashlib.sha1(source.encode()).hexdigest()
    with _LOCK:
        job = _BUILDING.get(key)
        if job is not None:
            return job
        job = _BUILDING[key] = {'done': threading.Event(), 'cached': False}
        ready = _cached_path(source, name, matmul)
        if ready is None:
            _QUEUE.append((job, source, name, matmul, solves))
            _started()
            return job
        job['cached'] = True

    def _load():
        try:
            job['lib'] = loaded(ready, name, solves=solves)
        except OSError:
            with _LOCK:
                job['cached'] = False
                _QUEUE.append((job, source, name, matmul, solves))
                _started()
            return
        except Exception as e:
            job['why'] = e
        job['done'].set()

    _LOADERS.submit(_load)
    return job


def settled_cached(wait_s=2.0):
    """Wait for already-built cached libraries queued so far without blocking on fresh GCC builds."""
    with _LOCK:
        jobs = [job for job in _BUILDING.values() if job.get('cached') and not job['done'].is_set()]
    return all(job['done'].wait(wait_s) for job in jobs)


def settled(wait_s=None):
    """Every build queued in this process so far waited for, `wait_s` s each at most: whether all are done."""
    return all(job['done'].wait(wait_s) for job in list(_BUILDING.values()))


def widened(width):
    """This process's builds `width` at once from now, those queued too: a warm's (`tools.sim.warmed`), each
    gcc's process on a core of the host's - one at a time, the 37 of a pull took the robot 16 minutes."""
    with _LOCK:
        WIDTH[0] = max(1, int(width))
        _started()


def _started():
    """A thread more for the queue while fewer than WIDTH build (under `_LOCK`)."""
    while _RUNNING[0] < min(WIDTH[0], len(_QUEUE)):
        _RUNNING[0] += 1
        threading.Thread(target=_builds, name='held builds', daemon=True).start()


def unbuilt():
    """This process's builds asked and not yet done: each's {'done': ...} to wait on."""
    with _LOCK:
        return [job for job in _BUILDING.values() if not job['done'].is_set()]


def _builds():
    """This process's builds, a thread's one after another while any are queued - the glue's extension first
    (`quick`), so a part held is called through it and no build of it waits in the robot's loop. A build's fault
    of any kind is its job's `why` and its job told done: a fault this thread did not take (a symbol missing,
    an AttributeError) ended it, left that build's waiters and every build after it waiting for good."""
    from tools.cores import quick
    try:
        quick.module()
        heads, fault = build.headers(INCLUDES), None
    except Exception as e:  # the builds' own headers unread: each build queued told so, none left waiting
        heads, fault = None, e
    while True:
        with _LOCK:
            if not _QUEUE:
                _RUNNING[0] -= 1
                return
            job, source, name, matmul, solves = _QUEUE.pop(0)
        try:
            if fault is not None:
                raise fault
            job['lib'] = loaded(built(source, name, matmul, heads), name, solves=solves)
        except Exception as e:  # the build's numpy runs on; its why told
            job['why'] = e
        finally:
            job['done'].set()


def matmuls(prog):
    """Whether `prog` calls numpy's BLAS or LAPACK: wbc_blas.c built beside it, numpy's table passed."""
    return any(st.kind in ('matmul', 'solve') for st in prog.statements)


@functools.lru_cache(maxsize=None)
def blas():
    """numpy's own BLAS laid once (`motion.numpy_blas`), every build's matmuls'."""
    from tools.cores import motion
    return motion.numpy_blas()


def compiled(fn, args, inputs, name, fresh=2, seed=1):
    """`fn(*args)` compiled for the arrays at `inputs` (`bodied.on_args`), built and held now on the
    traced arrays and `fresh` draws: a callable of those arrays returning fn's own result, its C's -
    Parted where any element differs."""
    from tools.cores import bodied
    inputs = list(inputs)
    fn_c = bodied.on_args(fn, name, inputs, fresh, seed)
    c = fn_c.wait(*args)

    def run(*arrays):
        full = list(args)
        for k, a in zip(inputs, arrays):
            full[k] = a
        return fn_c(*full)
    run.lib, run.prog = c.lib, c.prog
    return run


def batched(fn, args, inputs, name, fresh=2, seed=1, made_of=None):
    """`fn(*args)` compiled as one state's program run over any number: the arrays at `inputs` each
    a state along their first axis, fn's outputs too; traced at one state for its program and at as
    many states drawn at random for numpy's orders - an order is numpy's algorithm, states alike
    leave it untold -, held against numpy's own run at the states given and on `fresh` draws.
    `made_of` what fn's arithmetic is made of beyond its arrays' shapes (its modules, its constants):
    a build kept under it is loaded again untraced. The run returns fn's outputs, a tuple."""
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
    rng, drawn = np.random.default_rng(seed + 1), list(args)
    for k in inputs:
        drawn[k] = rng.normal(size=args[k].shape)
    many, _ = trace.traced(fn, drawn, inputs)
    kinds = [(st.kind, st.args[0] if st.kind == 'einsum' else None) for st in prog.statements]
    if kinds != [(st.kind, st.args[0] if st.kind == 'einsum' else None) for st in many.statements]:
        raise Parted('%s: one state\'s program is not each of many states\'' % name)
    found_ = [orders.ordered(st) for st in many.statements if st.kind == 'einsum']
    source, info = emit.c(prog, name, found_, batch=True)
    made, matmul = built(source, name, matmuls(prog)), matmuls(prog)
    outs = [tuple(r.shape[1:]) for r in prog.outs]
    run = _batched_run(made, name, rest, outs, info['doubles'], matmul)
    shapes = [np.shape(args[k]) for k in inputs]
    held(run, fn, args, inputs, shapes, fresh, seed, name, prog, flat=True)
    run.source, run.info, run.prog = source, info, prog
    if note is not None:
        with open(note + '.%d' % os.getpid(), 'w', encoding='utf-8') as f:
            json.dump({'lib': made, 'ins': rest, 'outs': outs, 'doubles': info['doubles'], 'matmul': matmul}, f)
        os.replace(note + '.%d' % os.getpid(), note)
    return run


def _note(name, made_of, rest):
    return os.path.join(build.OUT, 'gen', '%s_%s.json' % (name, digest(made_of, [tuple(r) for r in rest])))


def kept(name, made_of, rest):
    """What was kept of a batched build under `made_of` for states of `rest`: its note, None where none
    is - a note whose 'lib' is None a build this host could not make."""
    note = _note(name, made_of, rest)
    if not os.path.exists(note):
        return None
    with open(note, encoding='utf-8') as f:
        info = json.load(f)
    return info if info.get('lib') is None or os.path.exists(info['lib']) else None


def refused(name, made_of, rest, why):
    """A build this host cannot make kept as such: not tried again for the same arithmetic."""
    note = _note(name, made_of, rest)
    os.makedirs(os.path.dirname(note), exist_ok=True)
    with open(note + '.%d' % os.getpid(), 'w', encoding='utf-8') as f:
        json.dump({'lib': None, 'why': str(why)}, f)
    os.replace(note + '.%d' % os.getpid(), note)


def apart(module, fn, payload, key):
    """`module.fn(*payload)` in a process of its own - a build's trace holds its own interpreter, not
    the caller's -, `payload` pickled beside the builds, the process booked in the relay's ledger
    (APART_GB) while it runs: begun only where the machine's commit holds it and no live process
    builds `key` (`focus.admitted`). (the process, running; its booking), or None - its words in
    gen/<its payload>.log."""
    from tools.dev import census, focus
    folder = os.path.join(build.OUT, 'gen')
    os.makedirs(folder, exist_ok=True)
    marker = os.path.join(folder, 'building_%s.json' % key)

    def busy():
        try:
            with open(marker, encoding='utf-8') as f:
                return census.alive(json.load(f)['pid'])
        except (OSError, ValueError, KeyError):
            return False
    path = os.path.join(folder, 'payload_%d_%d.pkl' % (os.getpid(), next(_count)))
    with open(path, 'wb') as f:
        pickle.dump(payload, f)
    host = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    code = ('import os, pickle, sys\nfrom %s import %s as f\nwith open(sys.argv[1], "rb") as g:\n    a = pickle.load(g)\n'
            'os.remove(sys.argv[1])\ntry:\n    f(*a)\nfinally:\n    try:\n        os.remove(sys.argv[2])\n'
            '    except OSError:\n        pass\n' % (module, fn))
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1',
               PYTHONPATH=os.pathsep.join([host] + [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]))
    def start():
        with open(path[:-4] + '.log', 'w', encoding='utf-8') as log:
            proc = subprocess.Popen([sys.executable, '-c', code, path, marker], cwd=host, env=env, stdout=log,
                                    stderr=subprocess.STDOUT)
        with open(marker, 'w', encoding='utf-8') as f:
            json.dump({'pid': proc.pid}, f)
        return proc
    got = focus.admitted(APART_GB, start, 'apart', busy)
    if got is None:
        os.remove(path)
    return got


def found(name, made_of, rest):
    """A batched build kept under `made_of` for states of `rest`, loaded untraced; None where none is."""
    kept_ = kept(name, made_of, rest)
    if kept_ is None or kept_['lib'] is None or not os.path.exists(kept_['lib']):
        return None
    return _batched_run(kept_['lib'], name, [tuple(r) for r in kept_['ins']], [tuple(r) for r in kept_['outs']],
                        kept_['doubles'], kept_['matmul'])


def _batched_run(path, name, rest, outs, doubles, matmul):
    """A batched build's callable: its states' arrays in, fn's outputs over as many states out, a tuple
    of views of one block."""
    lib, cfn = loaded(path, name, batch=True)
    work = np.empty(max(1, doubles))
    lapack = ctypes.addressof(blas()) if matmul else None
    spans, total = layout.blocks(outs)

    def run(*arrays):
        ins = [np.ascontiguousarray(a, dtype=float) for a in arrays]
        n = len(ins[0])
        if [a.shape for a in ins] != [(n,) + tuple(r) for r in rest]:
            raise ValueError('%s takes states of %s, given %s' % (name, rest, [a.shape for a in ins]))
        block, flat = np.concatenate([a.ravel() for a in ins]), np.empty(max(1, n * total))
        cfn(n, block.ctypes.data, flat.ctypes.data, work.ctypes.data, None, lapack, None, None, None)
        return tuple(flat[n * a:n * b].reshape((n,) + tuple(r)) for (a, b), r in zip(spans, outs))
    run.lib = lib
    return run


def held(run, fn, args, inputs, shapes, fresh, seed, name, prog, flat=False):
    """`run` held bit for bit against `fn` on the traced arrays and `fresh` draws of new ones: each of
    its result's arrays (`flat`: run's own a tuple of them) - Parted where any element differs."""
    rng = np.random.default_rng(seed)
    draws = [[args[k] for k in inputs]] + [[rng.normal(size=s) for s in shapes] for _ in range(fresh)]
    for d, arrays in enumerate(draws):
        full = list(args)
        for k, a in zip(inputs, arrays):
            full[k] = a
        want = trace.leaves_of(prog.shape, fn(*full))
        got = run(*arrays)
        alike(want, dict(enumerate(got)) if flat else trace.leaves_of(prog.shape, got), name,
              'its traced arrays' if d == 0 else 'fresh arrays %d' % d)


def alike(want, got, name, on):
    """Parted where an array of `got` is not its of `want` bit for bit."""
    for k in sorted(want):
        g, w_ = np.ascontiguousarray(got[k], dtype=float), np.ascontiguousarray(want[k], dtype=float)
        if g.shape != w_.shape:
            raise Parted('%s output %d on %s: %s, numpy\'s %s' % (name, k, on, g.shape, w_.shape))
        if not orders.same(g, w_):
            apart_ = g.view(np.int64) != w_.view(np.int64)
            raise Parted('%s output %d on %s: %d of %d elements apart, the first at %s' % (
                name, k, on, int(apart_.sum()), apart_.size, np.unravel_index(int(np.argmax(apart_)), apart_.shape)))
