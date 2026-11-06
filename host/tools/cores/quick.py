"""Her parts' glue in C (quick.c), an extension of this interpreter: leaves laid, C called.

    walk = quick.walker(b, todo, slow)   # glue.walk's walk in C, Python's own (`slow`) where it gives up
    call = quick.caller(cfn)             # call(*cfn's eight pointers): its int, the interpreter let go
    data = quick.addressing()            # an ndarray's first element's address, read the quickest way here
    run = quick.runner(b)                # run(args): Bound b's whole call in C, MISS or SLOW where it is not

Her law's parts are handed fresh arrays every pass, 207 leaves a law pass: walked in Python each was
0.8 us of the 12 us a part's glue took, its C 7 us; ctypes' call 1 us over its eight pointers. A part's
call - its head's checks, its walk, its C and its result's views - one call in C (quick_run.c): six
layers of Python a call were 6 us of glue, 160 us a law pass. Built once a source with the host's
compiler against this interpreter's and numpy's headers, loaded as an extension module; where none can
be built - no headers, no compiler - its callers keep their Python, said once on stderr. No float is
touched here but a bool result's `!= 0.0`, an array's layout read: the bits the same.
"""
import ctypes
import functools
import importlib.machinery
import importlib.util
import os
import sys
import sysconfig
import threading

import numpy as np

from tools.cores import build
from tools.cores.arrays import address

#: The extension's sources, beside this: its walk and calls, its runner.
SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'quick.c')
SOURCES = (SOURCE, os.path.join(os.path.dirname(SOURCE), 'quick_run.c'))
#: What a runner gives back where a call's arguments are not as traced, and where Python's walk must say why.
MISS, SLOW = object(), object()
#: A check's kind and a result's leaf and node (quick_run.c's enums).
CHECKS = {'has': 0, 'keys': 1, 'length': 2}
VIEW, NUMBER, BOOLS, BOOL, GIVEN = range(5)
NODES = {'leaf': 0, 'dict': 1, 'list': 2, 'tuple': 3}
#: A leaf's row (quick.c): its argument, first key, keys, slot, place, a float its own, dimensions and
#: each one's length, MAXD at most - a leaf of more Python's always.
MAXD = 8
WIDTH = 7 + MAXD
_LOADING = threading.Lock()
_BUILT: list = []


def module():
    """The extension, built and loaded once a process, laid with numpy's ndarray and float64; None
    where it cannot be built here."""
    with _LOADING:
        if not _BUILT:
            _BUILT.append(_loaded())
        return _BUILT[0]


def _interpreter():
    """What the extension links against: on Windows this interpreter's own DLL, elsewhere nothing."""
    if os.name != 'nt':
        return []
    path = ctypes.create_unicode_buffer(32768)
    ctypes.windll.kernel32.GetModuleFileNameW(ctypes.c_void_p(sys.dllhandle), path, len(path))
    return [path.value]


def _loaded():
    """The extension built (numpy's version in its flags: a build its headers') and loaded, or None."""
    try:
        extra = ('-O2', '-DQUICK_NUMPY=%s' % np.__version__, *_interpreter())
        path = build.cached(list(SOURCES), [sysconfig.get_paths()['include'], np.get_include()], 'quick', extra)[0]
        loader = importlib.machinery.ExtensionFileLoader('_quick', path)
        made = importlib.util.module_from_spec(importlib.util.spec_from_loader('_quick', loader))
        loader.exec_module(made)
        made.laid(np.ndarray, np.dtype(np.float64))
        return made
    except (RuntimeError, OSError, ImportError, AttributeError) as e:
        print('quick stays python: %s' % str(e)[:300], file=sys.stderr)
        return None


def addressing():
    """An ndarray's first element's address: the extension's read, else `tools.cores.arrays.address`."""
    q = module()
    return address if q is None else q.data


def caller(cfn):
    """`cfn` (a part's C, ctypes' function of eight pointers) a call of them: its int, 0 where it returns
    none, the interpreter let go as ctypes lets it; None where the extension is not built."""
    q = module()
    if q is None:
        return None
    return functools.partial(q.call, ctypes.cast(cfn, ctypes.c_void_p).value, int(cfn.restype is not None))


def walker(b, todo, slow):
    """Bound `b`'s walk over its leaves `todo` in C (`glue.walk`): each leaf laid there - a float its own
    scalar, a float64 array of its shape in C's order its address - or handed to `b.leaf` and the walk
    on after it; a path not there False; any other error `slow`'s, Python's own walk, to raise. None where
    the extension is not built."""
    q = module()
    if q is None:
        return None
    rows, keys, places = np.zeros((max(1, len(todo)), WIDTH), dtype=np.int64), [], []
    for r, (j, i, k, path, written, shape) in enumerate(todo):
        nd = len(shape) if len(shape) <= MAXD else -1
        rows[r, :7] = (k, len(keys), len(path), i, j, int(not shape and not written), nd)
        rows[r, 7:7 + max(nd, 0)] = shape[:max(nd, 0)]
        keys.extend(path)
        places.append((j, k, path))
    own_at = np.array(b.own_at, dtype=np.uint64)
    ctx = q.context(rows.ctypes.data, len(todo), tuple(keys), ctypes.addressof(b.table), b._sbuf.ctypes.data,
                    own_at.ctypes.data)
    seen, leaf, step, n = b.seen, b.leaf, q.walk, len(todo)

    def walk(args):
        at = 0
        while True:
            r = step(args, ctx, at)
            if r == n:
                return True
            if r == -1:
                return False
            if r < -1:
                seen[:] = [None] * len(seen)
                return slow(args)
            j, k, path = places[r]
            try:
                a = args[k]
                for p in path:
                    a = a[p]
                seen[j] = None
                if not leaf(j, a):
                    return False
            except (KeyError, IndexError, TypeError):
                return False
            at = r + 1
    walk.own_at, walk.context, walk.places = own_at, ctx, tuple((j, k, tuple(path)) for j, k, path in places)
    return walk


def runner(b, singular):
    """Bound `b`'s whole call in C (quick_run.c), laid on it as it is now - its body, model and data bound -: run(args)
    its result, `MISS` where `args` are not as traced, `SLOW` where Python's walk must say why; `singular` raised
    where its C says so. None where the extension is not built, or `b` reads another part's block (`b.sources`)."""
    q = module()
    walk = b.walk
    if q is None or b.sources or getattr(walk, 'context', None) is None:
        return None
    path = lambda p: tuple(p)
    pins = tuple((k, path(p), v, isinstance(v, np.ndarray)) for k, p, v in b.pins)
    checks = tuple((CHECKS[kind], k, path(p), rest[0], rest[-1]) for kind, k, p, *rest in b.checks)
    leaves = []
    for a, _b, shape, dtype, given in b.outs:
        if given is not None:
            leaves.append((GIVEN, 0, given[0], (), path(given[1])))
        else:
            kind = (BOOL if dtype == bool else NUMBER) if not shape else (BOOLS if dtype == bool else VIEW)
            leaves.append((kind, a, 0, tuple(shape), ()))
    blocks = tuple(b.blocks)
    ctx = q.runner(b, b.body, -1 if b.at is None else b.at, tuple(b.consts), pins, checks, np.array_equal, walk.context,
                   b.seen, b.leaf, walk.places, ctypes.cast(b.cfn, ctypes.c_void_p).value, b.cfn.restype is not None,
                   tuple(b.fixed), blocks, b.turns, b.total, tuple(leaves), _node(b.prog.shape), singular, MISS, SLOW)
    return functools.partial(q.run, ctx)


def _node(node):
    """A result's tree (`trace` shape) as quick_run.c reads it: (0 leaf, k), (1 dict, ((key, node), ...)), (2 list, ...),
    (3 tuple, ...), (4 a constant, it)."""
    kind, body = node
    if kind == 'leaf':
        return (0, body)
    if kind == 'dict':
        return (1, tuple((k, _node(v)) for k, v in body))
    if kind in ('list', 'tuple'):
        return (NODES[kind], tuple(_node(v) for v in body))
    return (4, body)
