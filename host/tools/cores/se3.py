"""se3/ through ctypes and its numpy twin, the same bits: a pose's step onto weighted points.

    planes = se3.laid(points, goals, weights)            # the arena: eleven planes of a stride, zeros past the last
    out = se3.read(se3.step(planes, cfg, r, t))          # the C's step, its fields by name
    out = se3.read(se3.twin(planes, cfg, r, t))          # numpy's own, bit for bit

A pose (r, t) lays a body's points on their goals; a step lessens the weighted squares of the misses about the
points' centroid (se3.h). The C has no branch and checks nothing: `step` holds its arena to its shape here. Goals
and t are about one anchor (`laid`'s): far from the world's origin a float keeps its grain so.
"""
import ctypes
import functools
import os

import numpy as np

from tools import REPO
from tools.cores.build import TARGETS, cached, native

SOURCES = [os.path.join(REPO, 'se3', name) for name in ('test/harness.c', 'src/se3_sums.c', 'src/se3_step.c')]
INCLUDES = [os.path.join(REPO, 'se3', 'inc')]

#: se3.h's sizes: a sum's lanes, the arena's planes and those of them given, a cache line of bytes; a step's turn
#: at most, rad.
LANES, PLANES, GIVEN, LINE, TURN = 16, 11, 7, 64, 1.0
#: se3_cfg_t and se3_out_t as flat arrays of their precision, no padding: each field's name and its numbers.
CFG = ('damp_turn', 'damp_shift', 'soft', 'most_turn', 'most_shift')
OUT = (('weight', 1), ('spread', 1), ('centre', 3), ('second', 6), ('turn', 3), ('shift', 3), ('cost', 1), ('at', 3),
       ('curve', 9), ('turned', 3), ('shifted', 3), ('r', 9), ('t', 3))
SUMS, SIZE = 18, sum(n for _name, n in OUT)
#: The series' ratios (se3_step.c), the floor's share of the curvature's trace and the least a sum is divided by.
SINE, COSINE = (6.0, 20.0, 42.0, 72.0, 110.0, 156.0, 210.0), (12.0, 30.0, 56.0, 90.0, 132.0, 182.0, 240.0)
SHARE, TINY, LEAST = {np.float32: 1e-4, np.float64: 1e-6}, {np.float32: 1e-30, np.float64: 1e-60}, 1e-150


def kind(single=False):
    """The lanes' precision: numpy's float32 where `single`, else float64."""
    return np.float32 if single else np.float64


def built(single=False, isa=''):
    """The compiler's flags for the core: the aero core's (-O2, no errno) on target `isa` (`build.TARGETS`) or this
    host's own (`build.native`), nothing contracted; SE3_SINGLE for float lanes."""
    target = (('-ffp-contract=off',) + TARGETS[isa]) if isa else native()
    return (('-O2', '-fno-math-errno') + target + (('-DSE3_SINGLE',) if single else ())
            + (('-Wl,--export-all-symbols',) if os.name == 'nt' else ()))


@functools.lru_cache(maxsize=None)
def library(single=False, isa=''):
    """The core built for a precision and a target, typed once a process; its structs' sizes held to these."""
    name = 'se3%s%s' % ('f' if single else 'd', '_' + isa if isa else '')
    lib = ctypes.CDLL(cached(SOURCES, INCLUDES, name, list(built(single, isa)))[0])
    lib.se3_size.argtypes, lib.se3_size.restype = [ctypes.c_uint], ctypes.c_uint
    lib.se3_step.argtypes = [ctypes.c_void_p, ctypes.c_size_t] + [ctypes.c_void_p] * 4
    lib.se3_step.restype = None
    for run in (lib.seh_steps, lib.seh_sums):
        run.argtypes, run.restype = lib.se3_step.argtypes + [ctypes.c_ulong], None
    item = np.dtype(kind(single)).itemsize
    sizes = [lib.se3_size(which) for which in range(4)]
    if sizes != [item, len(CFG) * item, SUMS * item, SIZE * item]:
        raise RuntimeError('se3.h is not this module\'s layout: %s' % sizes)
    return lib


def cfg(single=False, **told):
    """se3_cfg_t as an array: no damping, the squares alone, a step half a radian and a quarter metre at most,
    but as `told` (CFG)."""
    values = dict(damp_turn=0.0, damp_shift=0.0, soft=0.0, most_turn=0.5, most_shift=0.25)
    values.update(told)
    return np.array([values[name] for name in CFG], kind(single))


def laid(points, goals, weights, single=False, anchor=(0.0, 0.0, 0.0)):
    """The arena of n `points` (n, 3) in their body, their `goals` (n, 3) about `anchor` and their `weights` (n,):
    seven planes given and four the step's own, each a stride of whole lanes, one block on a cache line's boundary,
    zeros past the last point."""
    dt = kind(single)
    p, w = np.asarray(points, float).reshape(-1, 3), np.asarray(weights, float).ravel()
    g = np.asarray(goals, float).reshape(-1, 3) - np.asarray(anchor, float)
    stride = max(LANES, -(-len(w) // LANES) * LANES)
    block = np.zeros(PLANES * stride + LINE // np.dtype(dt).itemsize, dt)
    lead = (-block.ctypes.data % LINE) // block.itemsize
    planes = block[lead:lead + PLANES * stride]
    for k, column in enumerate((p[:, 0], p[:, 1], p[:, 2], g[:, 0], g[:, 1], g[:, 2], w)):
        planes[k * stride:k * stride + len(w)] = column
    return planes


def _held(planes, config, r, t):
    """(the stride, the cfg, r and t as the arena's precision) of an arena in its shape, else a ValueError."""
    dt = planes.dtype.type
    if (dt not in SHARE or planes.ndim != 1 or len(planes) % (PLANES * LANES) or not len(planes)
            or not planes.flags.c_contiguous or planes.ctypes.data % LINE):
        raise ValueError('an arena is eleven planes of whole lanes on a cache line (se3.laid)')
    return (len(planes) // PLANES, np.ascontiguousarray(config, dt), np.ascontiguousarray(r, dt).reshape(9),
            np.ascontiguousarray(t, dt).reshape(3))


def step(planes, config, r, t, isa=''):
    """se3_step from the pose (`r` 3 by 3, `t`) on `planes` (`laid`) under `config` (`cfg`): se3_out_t as an array
    (`read`), on target `isa` where one is named."""
    stride, config, r, t = _held(planes, config, r, t)
    out = np.zeros(SIZE, planes.dtype)
    library(planes.dtype == np.float32, isa).se3_step(planes.ctypes.data, stride, config.ctypes.data, r.ctypes.data,
                                                     t.ctypes.data, out.ctypes.data)
    return out


def paced(planes, config, r, t, calls, sums=False):
    """What a step costs, s: the least of five runs of `calls` steps in C (harness.c), or of its `sums` alone."""
    import time
    stride, config, r, t = _held(planes, config, r, t)
    lib, out = library(planes.dtype == np.float32), np.zeros(SIZE, planes.dtype)
    run, took = (lib.seh_sums if sums else lib.seh_steps), []
    for _ in range(5):
        t0 = time.perf_counter()
        run(planes.ctypes.data, stride, config.ctypes.data, r.ctypes.data, t.ctypes.data, out.ctypes.data, calls)
        took.append((time.perf_counter() - t0) / calls)
    return min(took)


def read(out):
    """A step's fields by name (OUT), views of `out`: its matrices 3 by 3."""
    fields, at = {}, 0
    for name, n in OUT:
        fields[name] = out[at] if n == 1 else out[at:at + n].reshape((3, 3) if n == 9 else (n,))
        at += n
    return fields


def _lanes(values, dt):
    """A plane's sum as the C keeps it: sixteen lanes down the blocks, then by halves (se3_sums.c's tree)."""
    lanes = np.zeros(LANES, dt)
    for row in values.reshape(-1, LANES):
        lanes = lanes + row
    eight = lanes[:8] + lanes[8:]
    four = eight[:4] + eight[4:]
    return (four[0] + four[2]) + (four[1] + four[3])


def _summed(planes, stride, soft, r, t, dt):
    """se3_summed: (weight, spread, centre, second, turn, shift, cost)."""
    px, py, pz, gx, gy, gz, pw = (planes[k * stride:(k + 1) * stride] for k in range(GIVEN))
    half, one = dt(0.5), dt(1.0)
    mx = ((((r[0] * px) + (r[1] * py)) + (r[2] * pz)) + t[0]) - gx
    my = ((((r[3] * px) + (r[4] * py)) + (r[5] * pz)) + t[1]) - gy
    mz = ((((r[6] * px) + (r[7] * py)) + (r[8] * pz)) + t[2]) - gz
    m2 = ((mx * mx) + (my * my)) + (mz * mz)
    w = (half * (pw + np.abs(pw))) / (one + (m2 * soft))
    weight = _lanes(w, dt)
    inverse = weight / ((weight * weight) + dt(TINY[dt]))
    centre = [_lanes(w * px, dt) * inverse, _lanes(w * py, dt) * inverse, _lanes(w * pz, dt) * inverse]
    spread = _lanes(w * (((px * px) + (py * py)) + (pz * pz)), dt)
    lx, ly, lz = px - centre[0], py - centre[1], pz - centre[2]
    dx = ((r[0] * lx) + (r[1] * ly)) + (r[2] * lz)
    dy = ((r[3] * lx) + (r[4] * ly)) + (r[5] * lz)
    dz = ((r[6] * lx) + (r[7] * ly)) + (r[8] * lz)
    wx, wy, wz = w * dx, w * dy, w * dz
    second = [_lanes(v, dt) for v in (wx * dx, wx * dy, wx * dz, wy * dy, wy * dz, wz * dz)]
    turn = [_lanes(v, dt) for v in ((wy * mz) - (wz * my), (wz * mx) - (wx * mz), (wx * my) - (wy * mx))]
    shift = [_lanes(v, dt) for v in (w * mx, w * my, w * mz)]
    return weight, spread, centre, second, turn, shift, half * _lanes(w * m2, dt)


def _times(a, b):
    """a b, 3 by 3 row-major, each sum in se3_step.c's order."""
    return [((a[3 * i] * b[j]) + (a[3 * i + 1] * b[3 + j])) + (a[3 * i + 2] * b[6 + j])
            for i in range(3) for j in range(3)]


def _turn(h, b, spread, dt):
    """se3_step.c's turn: h (h h + floor^2)^-1 (-b) in double, its L D L' and h last."""
    d = np.float64
    xx, xy, xz, yy, yz, zz = d(h[0]), d(h[1]), d(h[2]), d(h[4]), d(h[5]), d(h[8])
    ground = ((d(SHARE[dt]) * np.abs((xx + yy) + zz)) + (d(np.finfo(dt).eps) * d(spread))) + d(LEAST)
    f2 = ground * ground
    a = [(((xx * xx) + (xy * xy)) + (xz * xz)) + f2, ((xx * xy) + (xy * yy)) + (xz * yz),
         ((xx * xz) + (xy * yz)) + (xz * zz), (((xy * xy) + (yy * yy)) + (yz * yz)) + f2,
         ((xy * xz) + (yy * yz)) + (yz * zz), (((xz * xz) + (yz * yz)) + (zz * zz)) + f2]
    g = [d(0.0) - d(b[0]), d(0.0) - d(b[1]), d(0.0) - d(b[2])]
    i0 = d(1.0) / a[0]
    l10, l20 = a[1] * i0, a[2] * i0
    i1 = d(1.0) / (a[3] - (l10 * a[1]))
    e = a[4] - (l20 * a[1])
    l21 = e * i1
    i2 = d(1.0) / ((a[5] - (l20 * a[2])) - (l21 * e))
    f1 = g[1] - (l10 * g[0])
    y2 = ((g[2] - (l20 * g[0])) - (l21 * f1)) * i2
    y1 = (f1 * i1) - (l21 * y2)
    y0 = ((g[0] * i0) - (l10 * y1)) - (l20 * y2)
    return [dt(((xx * y0) + (xy * y1)) + (xz * y2)), dt(((xy * y0) + (yy * y1)) + (yz * y2)),
            dt(((xz * y0) + (yz * y1)) + (zz * y2))]


def _capped(v, cap, dt):
    """se3_step.c's capped: `v` no longer than `cap`, its own bits inside."""
    size = np.sqrt(((v[0] * v[0]) + (v[1] * v[1])) + (v[2] * v[2]))
    held = (dt(0.5) * (cap + np.abs(cap))) + dt(TINY[dt])
    scale = held / (size if np.signbit(held - size) else held)
    return [v[0] * scale, v[1] * scale, v[2] * scale]


def _taken(r, t, centre, w, v, dt):
    """se3_step.c's taken: (r_out, t_out), the turn's series, Newton's and Schulz's step onto SO(3)."""
    half, one, three = dt(0.5), dt(1.0), dt(3.0)
    xx, yy, zz, xy, xz, yz = w[0] * w[0], w[1] * w[1], w[2] * w[2], w[0] * w[1], w[0] * w[2], w[1] * w[2]
    u = (xx + yy) + zz
    a = b = one
    for k in range(len(SINE) - 1, -1, -1):
        a = one - ((u * (one / dt(SINE[k]))) * a)
        b = one - ((u * (one / dt(COSINE[k]))) * b)
    b = b * half
    by = [one - (b * (yy + zz)), (b * xy) - (a * w[2]), (b * xz) + (a * w[1]),
          (b * xy) + (a * w[2]), one - (b * (xx + zz)), (b * yz) - (a * w[0]),
          (b * xz) - (a * w[1]), (b * yz) + (a * w[0]), one - (b * (xx + yy))]
    raw = _times(by, r)
    gram = [half * ((three * dt(i == j)) - (((raw[i] * raw[j]) + (raw[3 + i] * raw[3 + j])) + (raw[6 + i] * raw[6 + j])))
            for i in range(3) for j in range(3)]
    r_out = _times(raw, gram)
    t_out = [(t[i] + ((((r[3 * i] - r_out[3 * i]) * centre[0]) + ((r[3 * i + 1] - r_out[3 * i + 1]) * centre[1]))
                      + ((r[3 * i + 2] - r_out[3 * i + 2]) * centre[2]))) + v[i] for i in range(3)]
    return r_out, t_out


def twin(planes, config, r, t):
    """`step` in numpy: every sum and product in the C's order and precision, so the same bits."""
    stride, config, r, t = _held(planes, config, r, t)
    dt, zero = planes.dtype.type, planes.dtype.type(0.0)
    damp_turn, damp_shift, soft, most_turn, most_shift = config
    weight, spread, centre, second, turn, shift, cost = _summed(planes, stride, soft, r, t, dt)
    at = [(((r[3 * i] * centre[0]) + (r[3 * i + 1] * centre[1])) + (r[3 * i + 2] * centre[2])) + t[i] for i in range(3)]
    sxx, sxy, sxz, syy, syz, szz = second
    curve = [(syy + szz) + damp_turn, zero - sxy, zero - sxz, zero - sxy, (sxx + szz) + damp_turn, zero - syz,
             zero - sxz, zero - syz, (sxx + syy) + damp_turn]
    turned = _turn(curve, turn, spread, dt)
    held = weight + damp_shift
    inverse = held / ((held * held) + dt(TINY[dt]))
    shifted = [zero - (b * inverse) for b in shift]
    turned = _capped(turned, dt(TURN) if np.signbit(dt(TURN) - most_turn) else most_turn, dt)
    shifted = _capped(shifted, most_shift, dt)
    r_out, t_out = _taken(r, t, centre, turned, shifted, dt)
    return np.array([weight, spread] + centre + second + turn + shift + [cost] + at + curve + turned + shifted
                    + r_out + t_out, dt)
