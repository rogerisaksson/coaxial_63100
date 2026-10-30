"""Her parts' glue in C (quick.c), an extension of this interpreter: leaves laid, C called.

    walk = quick.walker(b, todo, slow)   # glue.walk's walk in C, Python's own (`slow`) where it gives up
    call = quick.caller(cfn)             # call(*cfn's eight pointers): its int, the interpreter let go
    data = quick.addressing()            # an ndarray's first element's address, read the quickest way here

Her law's parts are handed fresh arrays every pass, 207 leaves a law pass: walked in Python each was
0.8 us of the 12 us a part's glue took, its C 7 us; ctypes' call 1 us over its eight pointers. Built
once a source with the host's compiler against this interpreter's and numpy's headers, loaded as an
extension module; where none can be built - no headers, no compiler - its callers keep their Python,
said once on stderr. No float is touched here, an array's layout alone read: the bits the same.
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
from tools.cores.wbc import address

#: The extension's source, beside this.
SOURCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'quick.c')
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
        path = build.cached([SOURCE], [sysconfig.get_paths()['include'], np.get_include()], 'quick', extra)[0]
        loader = importlib.machinery.ExtensionFileLoader('_quick', path)
        made = importlib.util.module_from_spec(importlib.util.spec_from_loader('_quick', loader))
        loader.exec_module(made)
        made.laid(np.ndarray, np.dtype(np.float64))
        return made
    except (RuntimeError, OSError, ImportError, AttributeError) as e:
        print('quick stays python: %s' % str(e)[:300], file=sys.stderr)
        return None


def addressing():
    """An ndarray's first element's address: the extension's read, else `tools.cores.wbc.address`."""
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
    walk.own_at = own_at
    return walk
