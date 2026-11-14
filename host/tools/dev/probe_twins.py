#!/usr/bin/env python3
"""A runner probe, never merged: where her motion.control's C parts from its python twins on this host - its
processor, numpy's dispatch and OpenBLAS's core, exact_host's probes one by one, and in test_tick's twin run each
pass's first hook whose answer or written arrays part, and the passes whose solve parts most."""
import collections
import ctypes
import glob
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path[:0] = [HOST, os.path.join(HOST, 'tests')]

import numpy as np  # noqa: E402


def processor():
    """The CPU's model and wide vector sets, numpy's dispatch, numpy's OpenBLAS's core and config."""
    try:
        with open('/proc/cpuinfo', encoding='utf-8') as f:
            lines = f.read().splitlines()
        model = next((x.split(':', 1)[1].strip() for x in lines if x.startswith('model name')), '?')
        flags = next((x.split(':', 1)[1].split() for x in lines if x.startswith('flags')), [])
        wide = sorted(f for f in flags if f.startswith(('avx512', 'avx10', 'amx')))
        print('cpu:', model, '|', ' '.join(wide) or 'no avx512', '| fma' if 'fma' in flags else '')
    except OSError:
        import platform
        print('cpu:', platform.processor())
    feats = np._core._multiarray_umath.__cpu_features__
    print('numpy', np.__version__, 'on:', ' '.join(sorted(k for k, on in feats.items() if on and k.startswith(
        ('AVX512', 'X86_V', 'AVX2', 'FMA3')))))
    found = sorted(glob.glob(os.path.join(os.path.dirname(np.__file__), os.pardir, 'numpy.libs', '*scipy_openblas64*')))
    if found:
        lib = ctypes.CDLL(os.path.abspath(found[0]))
        for name in ('scipy_openblas_get_corename64_', 'scipy_openblas_get_config64_'):
            fn = getattr(lib, name, None)
            if fn is not None:
                fn.restype = ctypes.c_char_p
                print('openblas %s: %s' % (name, fn().decode()))


def exact_parts(draws=1000):
    """exact_host's probes one at a time: how many draws each parts from numpy here."""
    from tools.cores import motion as c_motion, wbc
    lib = wbc.library()
    lib.wbh_np_probe.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    lib.wbh_np_probe.restype = None
    lib.wbh_np_libm.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
    lib.wbh_np_libm.restype = None
    rng = np.random.default_rng(3)
    a = rng.normal(size=(draws, 9)) * rng.choice([1e-3, 1.0, 1e3], size=(draws, 1))
    b = rng.normal(size=(draws, 9))
    wants = {0: (a.reshape(draws, 3, 3) @ b[:, :3, None])[:, :, 0],
             1: (a[:, :4].reshape(draws, 2, 2) @ b[:, :2, None])[:, :, 0],
             2: (a[:, :6].reshape(draws, 2, 3) @ np.transpose(b.reshape(draws, 3, 3), (0, 2, 1))).reshape(draws, 6),
             3: (a[:, :6].reshape(draws, 2, 3) @ b.reshape(draws, 3, 3)).reshape(draws, 6),
             4: np.array([x[:3] @ y[:3] for x, y in zip(a, b)])[:, None],
             5: np.array([np.einsum('nia,ia->n', x[None, :6].reshape(1, 2, 3), y[:6].reshape(2, 3))[0]
                          for x, y in zip(a, b)])[:, None]}
    for which, want in wants.items():
        out = np.zeros((draws, 9))
        lib.wbh_np_probe(which, a.ctypes.data, b.ctypes.data, out.ctypes.data, draws)
        print('probe %d: %d of %d draws part' % (which, int(np.any(out[:, :want.shape[1]] != want, axis=1).sum()),
                                                 draws))
    x = np.concatenate([a[:, 0] * 0.3, rng.uniform(-8.0, 8.0, 100 * draws)])
    ins = (x, rng.normal(size=len(x)), np.clip(x, -1.0, 1.0), np.clip(x, -50.0, 50.0), np.abs(x) + 1e-3)
    got, few, laid = np.zeros((len(x), 10)), 2 * draws, np.ascontiguousarray(np.stack(ins, axis=1))
    lib.wbh_np_libm(len(x), laid.ctypes.data, got.ctypes.data)
    libm = [(np.sin, (0,)), (np.cos, (0,)), (np.tan, (0,)), (np.arcsin, (2,)), (np.arccos, (2,)),
            (np.arctan2, (0, 1)), (np.exp, (3,)), (np.log, (4,)), (np.hypot, (0, 1))]
    for j, (fn, at) in enumerate(libm):
        scalars = np.array([fn(*[np.float64(ins[i][k]) for i in at]) for k in range(few)])
        print('libm %s: arrays %d of %d, scalars %d of %d part' % (
            fn.__name__, int((got[:, j] != fn(*[ins[i] for i in at])).sum()), len(x),
            int((got[:few, j] != scalars).sum()), few))
    print('pow 2: %d of %d part' % (int((got[:few, 9] != np.array([np.float64(v) ** 2 for v in x[:few]])).sum()),
                                    few))
    print('exact_host:', c_motion.exact_host())


def _arrays(obj, out, depth=0):
    """Every array and number in `obj` (arrays, numbers, tuples, lists, dicts by sorted key), copied, in order."""
    if isinstance(obj, np.ndarray):
        out.append(obj.copy())
    elif isinstance(obj, (bool, int, float, np.number)):
        out.append(np.array(float(obj)))
    elif isinstance(obj, (tuple, list)) and depth < 3:
        for o in obj:
            _arrays(o, out, depth + 1)
    elif isinstance(obj, dict) and depth < 3:
        for k in sorted(obj, key=str):
            _arrays(obj[k], out, depth + 1)
    return out


def _parted(xs, ys, names=None):
    """'name maxdiff, ...' of the entries of xs and ys whose bits differ, None where none."""
    out = []
    for j, (x, y) in enumerate(zip(xs, ys)):
        if x.shape != y.shape or x.dtype != y.dtype or x.tobytes() != y.tobytes():
            try:
                d = float(np.nanmax(np.abs(x.astype(float) - y.astype(float)))) if x.shape == y.shape else np.nan
            except (TypeError, ValueError):
                d = np.nan
            out.append('%s %.3g' % (names[j] if names else j, d))
    if len(xs) != len(ys):
        out.append('len %d/%d' % (len(xs), len(ys)))
    return ', '.join(out[:6]) or None


def twins():
    """test_tick's twin run, each hook of both sessions recorded: each pass's first parting, the worst solves."""
    import gc
    import test_tick as tt
    from machine.law import motion
    from machine.process import worker
    from tools.cores import motion as c_motion
    from tools.sim import told
    skip = tt.CONTROL_SKIP | set(c_motion.HOOKS)
    watch = tt.CONTROL_ARRAYS + ('seat_f',)
    log, gs, pair, real = {1: [], 2: []}, {}, [], motion.control
    tally = collections.Counter()
    firsts, hooks, solves, arrays_apart = [], collections.Counter(), [], collections.Counter()

    def record(sid, name, ret):
        g = gs[sid]
        log[sid].append((name, _arrays(ret, []), [np.array(g[k]).copy() for k in watch if k in g],
                         [k for k in watch if k in g]))

    class Wrapped:
        """A hook (or a hook object's method) whose every call is recorded."""

        def __init__(self, obj, name, sid):
            object.__setattr__(self, '_o', obj)
            object.__setattr__(self, '_n', name)
            object.__setattr__(self, '_s', sid)

        def __call__(self, *a, **k):
            r = self._o(*a, **k)
            record(self._s, self._n, r)
            return r

        def __getattr__(self, attr):
            v = getattr(self._o, attr)
            return Wrapped(v, self._n + '.' + attr, self._s) if callable(v) and not isinstance(v, type) else v

        def __setattr__(self, attr, value):
            setattr(self._o, attr, value)

    def wrap(g, sid):
        gs[sid] = g
        for name in c_motion.HOOKS:
            v = g.get(name)
            if isinstance(v, tuple):
                g[name] = tuple(Wrapped(x, '%s[%d]' % (name, i), sid) if callable(x) else x for i, x in enumerate(v))
            elif v is not None:
                g[name] = Wrapped(v, name, sid)

    def beside(g, dt):
        if not pair or g is not pair[0]:
            return real(g, dt)
        g2 = pair[1]
        tt._sync_control(g, g2, skip)
        log[1].clear()
        log[2].clear()
        real(g, dt)
        real(g2, dt)
        n, chained, t = tally['passes'], int(g.get('chained', 0)), float(g.get('t', 0.0))
        tally['passes'] += 1
        tally['chained %d' % chained] += 1
        first = None
        for i, (a, b) in enumerate(zip(log[1], log[2])):
            if a[0] != b[0]:
                first = (i, a[0] + '|' + b[0], 'order')
                break
            ret, state = _parted(a[1], b[1]), _parted(a[2], b[2], a[3])
            if ret or state:
                first = (i, a[0], 'ret[%s] state[%s]' % (ret, state))
                break
        if first is None and len(log[1]) != len(log[2]):
            first = (min(len(log[1]), len(log[2])), 'calls %d/%d' % (len(log[1]), len(log[2])), 'count')
        law = [(k, g[k], g2[k]) for k in tt.CONTROL_ARRAYS] + [
            (k, np.array(float(g.get(k, 0.0))), np.array(float(g2.get(k, 0.0)))) for k in tt.CONTROL_NUMBERS]
        sol = [('seat_f', g['seat_f'], g2['seat_f']), ('tau', g['drives'][1]['tau'], g2['drives'][1]['tau']),
               ('qacc', g['drives'][1]['qacc'], g2['drives'][1]['qacc']), ('feed', g['cmd']['feed'], g2['cmd']['feed'])]
        worst = (0.0, '')
        for k, a, b in law + sol:
            if a.tobytes() != b.tobytes():
                arrays_apart[k] += 1
                d = np.abs(a.astype(float) - b.astype(float))
                d = float(np.nanmax(d)) if np.any(~np.isnan(d)) else 0.0
                if k in ('seat_f', 'tau', 'qacc', 'feed'):
                    worst = max(worst, (d, k))
        if first is not None:
            tally['parted passes'] += 1
            hooks[first[1]] += 1
            if len(firsts) < 12:
                firsts.append((n, t, chained) + first)
        if worst[0] > 0.0:
            got = _parted(_arrays(g.get('got'), []), _arrays(g2.get('got'), []),
                          [str(k) for k in sorted(g.get('got', {}), key=str)])
            solves.append((worst[0], worst[1], n, t, chained, first[1] if first else None, got))
    motion.control = beside
    try:
        s1, s2 = worker.session(walks=True), worker.session(walks=True)
        c_motion.twins(s2['g'])
        gc.collect()
        wrap(s1['g'], 1)
        wrap(s2['g'], 2)
        pair[:] = [s1['g'], s2['g']]
        for _ in range(900):
            worker.stepped(s1)
        r1, r2 = told.Run('bed_behind'), told.Run('bed_behind')
        c_motion.twins(r2.s['g'])
        gc.collect()
        wrap(r1.s['g'], 1)
        wrap(r2.s['g'], 2)
        pair[:] = [r1.s['g'], r2.s['g']]
        r1.tell(say='lie_bed')
        r1.go(8.2)
    finally:
        motion.control = real
    print('passes:', dict(tally))
    print('arrays apart (passes):', dict(arrays_apart))
    print('first parting hooks:', hooks.most_common(12))
    for f in firsts:
        print('  first parting: pass %d t %.3f chained %d call %d %s %s' % f)
    for s in sorted(solves, reverse=True)[:10]:
        print('  solve apart %.3g in %s: pass %d t %.3f chained %d first %s got[%s]' % s)


if __name__ == '__main__':
    processor()
    exact_parts()
    twins()
