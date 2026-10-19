"""Her MuJoCo as a trace sees it (`tools.cores.trace`): her body's model, data and scratch, her calls.

    prog, got = trace.traced(wbc.sense, (t, qpos, qvel), (1, 2), wrap=lambda prog, a: [Body(prog, t)] + a[1:])
    arrays, model, data = bound(prog, b)               # at call time, her own body's (t = twin(b))

Her data's arrays and her body's scratch (SCRATCH) are externs: memory MuJoCo's and hers, addressed
at call time. A MuJoCo function her code calls runs as it is and stands an 'mj' statement: in C the
function itself, mujoco.dll's, by its prototype in mujoco.h - the binding's arguments the C's, in
their order. What else her code reads of her body is a constant of the trace (`prog.consts`), the
same objects or traced again; of her model and her data nothing else is read.
"""
import copy
import ctypes
import functools
import glob
import os
import re

import numpy as np

from tools.cores import trace
from tools.cores.trace import Traced, Untraced

#: Her body's own scratch a MuJoCo call writes into (`machine.wbc.Body`).
SCRATCH = ('_M', '_jp', '_jr', '_am')
#: MjData's float arrays a holder may keep a view of (`quad.Sky._xmat`, `_vel`, `_xfrc`).
DATA_ARRAYS = ('qpos', 'qvel', 'qacc', 'act', 'ctrl', 'qfrc_applied', 'xfrc_applied',
               'xpos', 'xquat', 'xmat', 'xipos', 'ximat', 'site_xpos', 'site_xmat',
               'subtree_com', 'cdof', 'qfrc_bias', 'qfrc_passive')
trace.hide(__file__)


def _of(b):
    """(MjModel, MjData, mujoco) of any MuJoCo holder `b` (`wbc.Body`, `quad.Sky`, `physics.World`)."""
    m = getattr(b, 'm', None) or b.model
    d = getattr(b, 'd', None) or b.data
    mj = getattr(b, 'mj', None) or b._mj
    return m, d, mj


def _in_data(d, arr):
    """(array name on `d`, byte offset in it) if `arr` is a view of one of `d`'s arrays, else None."""
    at = arr.ctypes.data
    for attr in DATA_ARRAYS:
        base = getattr(d, attr)
        lo = base.ctypes.data
        if lo <= at < lo + base.nbytes:
            return attr, at - lo
    return None


class Model:

    """Her model as traced: its sizes read, its parameters not - identified, they move mid-run."""

    def __init__(self, m):
        self.raw = m

    def __getattr__(self, name):
        got = getattr(self.raw, name)
        if isinstance(got, int):
            return got
        raise Untraced("her model's %s read at %s: a parameter, not a size" % (name, trace.where()))


class Data:

    """Her data as traced: each of its float arrays an extern named 'd.<name>', nothing else read."""

    def __init__(self, prog, d):
        self.prog, self.raw = prog, d

    def __getattr__(self, name):
        got = getattr(self.raw, name)
        if isinstance(got, np.ndarray) and got.dtype == np.float64:
            return self.prog.extern('d.' + name, got)
        raise Untraced("her data's %s read at %s: her state, not an array of it" % (name, trace.where()))


class Mj:

    """MuJoCo as traced: each mj_ function run as it is and recorded ('mj'), its constants its own."""

    def __init__(self, prog, mj):
        self.prog, self.raw = prog, mj

    def __getattr__(self, name):
        real = getattr(self.raw, name)
        if not name.startswith('mj_'):
            return real
        prog = self.prog

        def call(*args):
            params = prototype(name)
            if len(params) != len(args):
                raise Untraced('%s takes %d, given %d at %s' % (name, len(params), len(args), trace.where()))
            kinds = [_kind(a, p, name) for a, p in zip(args, params)]
            real(*[a.raw if isinstance(a, (Model, Data)) else a.view if isinstance(a, Traced) else a for a in args])
            if name not in [c[0] for c in prog.calls]:
                prog.calls.append((name, ', '.join(params)))
            prog.add('mj', None, (name, kinds))
        return call


class Body:

    """Her body as traced: its model, data, MuJoCo and scratch traced, the rest constants read."""

    def __init__(self, prog, b):
        m, d, mj = _of(b)
        self.prog, self.raw = prog, b
        self.m = self.model = Model(m)
        self.d = self.data = Data(prog, d)
        self.mj = self._mj = Mj(prog, mj)
        for name in SCRATCH:
            if hasattr(b, name):
                setattr(self, name, prog.extern('b.' + name, getattr(b, name)))

    def __getattr__(self, name):
        got = getattr(self.raw, name)
        if callable(got):
            raise Untraced("her body's %s called at %s: its reads not traced" % (name, trace.where()))
        if isinstance(got, np.ndarray) and got.dtype == np.float64:
            found = _in_data(self.d.raw, got)
            if found is not None:
                attr, off = found
                ext = getattr(self.d, attr)
                return Traced(self.prog, ext.buffer,
                              np.ndarray(got.shape, np.float64, buffer=ext.view, offset=off, strides=got.strides))
        self.prog.consts[name] = got
        return got


def _kind(a, param, name):
    """An argument as the C passes it, checked against its parameter in mujoco.h."""
    if 'mjModel' in param:
        if isinstance(a, Model):
            return ('model',)
    elif 'mjData' in param:
        if isinstance(a, Data):
            return ('data',)
    elif 'mjtNum' in param and ('*' in param or '[' in param):
        const = param.startswith('const')
        if a is None:
            return ('null',)
        if isinstance(a, Traced):
            r = a.ref()
            if not _contiguous(r):
                raise Untraced('%s given a view not C-ordered at %s' % (name, trace.where()))
            if not const and r.buffer.role == 'in':
                raise Untraced('%s writes her input at %s' % (name, trace.where()))
            return ('buf', r, const)
    elif re.fullmatch(r'int\s+\w+', param):
        if isinstance(a, (bool, int, np.integer)):
            return ('int', int(a))
    else:
        raise Untraced('%s: a parameter of %r' % (name, param))
    raise Untraced('%s given a %s for %r at %s' % (name, type(a).__name__, param, trace.where()))


def _contiguous(r):
    step = 1
    for n, s in reversed(list(zip(r.shape, r.strides))):
        if n != 1 and s != step:
            return False
        step *= n
    return True


def copy_state(dst, src):
    """`dst`'s MjData and scratch copied from `src`'s (`mj_copyData`)."""
    m, d, mj = _of(src)
    mj.mj_copyData(_of(dst)[1], m, d)
    for name in SCRATCH:
        if hasattr(src, name) and hasattr(dst, name):
            getattr(dst, name)[:] = getattr(src, name)


def twin(b):
    """Her body with a MjData and scratch of its own: traced and held on, hers untouched."""
    m, d, mj = _of(b)
    t = copy.copy(b)
    td = mj.MjData(m)
    mj.mj_copyData(td, m, d)
    t.d = t.data = td
    t.m = t.model = m
    t.mj = t._mj = mj
    for name in SCRATCH:
        if hasattr(b, name):
            setattr(t, name, np.zeros_like(getattr(b, name)))
    for name, val in list(vars(b).items()):
        if isinstance(val, np.ndarray) and val.dtype == np.float64:
            found = _in_data(d, val)
            if found is not None:
                attr, off = found
                setattr(t, name, np.ndarray(val.shape, np.float64, buffer=getattr(td, attr),
                                            offset=off, strides=val.strides))
    return t


def bound(prog, b):
    """Her body `b` as the C addresses it: ([(each of her body's externs' place in `prog.externs`, its
    array)], her model's address, her data's) - Untraced where an array is not as traced."""
    m, d, _mj = _of(b)
    arrays = []
    for i, e in enumerate(prog.externs):
        kind, _dot, name = e.name.partition('.')
        if kind not in ('d', 'b') or not _dot:
            continue
        arr = getattr(d if kind == 'd' else b, name)
        if arr.shape != e.shape or arr.dtype != np.float64 or not arr.flags.c_contiguous:
            raise Untraced('%s is %s %s, traced %s' % (e.name, arr.dtype, arr.shape, e.shape))
        arrays.append((i, arr))
    return arrays, m._address, d._address


def functions(prog):
    """Each function `prog.calls` names, mujoco.dll's address of it, in their order."""
    lib = _library()
    return [ctypes.cast(getattr(lib, name), ctypes.c_void_p).value for name, _params in prog.calls]


def prototype(name):
    """`name`'s parameters as mujoco.h declares them; Untraced where it returns anything."""
    ret, params = _prototypes().get(name, (None, None))
    if ret != 'void':
        raise Untraced('%s: %s in mujoco.h' % (name, 'not declared' if ret is None else 'returns ' + ret))
    return params


@functools.lru_cache(maxsize=None)
def _prototypes():
    import mujoco
    found = glob.glob(os.path.join(os.path.dirname(mujoco.__file__), 'include', '**', 'mujoco.h'), recursive=True)
    if not found:
        raise Untraced('no mujoco.h beside mujoco')
    with open(found[0], encoding='utf-8') as f:
        text = re.sub(r'//[^\n]*', '', f.read())
    out = {}
    for ret, name, params in re.findall(r'MJAPI\s+([\w\s\*]+?)\s*\b(mj_\w+)\s*\(([^;]*?)\)\s*;', text):
        out[name] = (' '.join(ret.split()), [' '.join(p.split()) for p in params.split(',') if p.strip()])
    return out


@functools.lru_cache(maxsize=None)
def _library():
    import mujoco
    here = os.path.dirname(mujoco.__file__)
    found = [p for pattern in ('mujoco.dll', 'libmujoco.so*', 'libmujoco*.dylib')
             for p in sorted(glob.glob(os.path.join(here, pattern)))]
    if not found:
        raise Untraced('no MuJoCo library beside mujoco')
    return ctypes.CDLL(found[0])
