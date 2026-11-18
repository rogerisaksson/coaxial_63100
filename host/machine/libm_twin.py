"""The robot's own elementary functions in numpy, wbc_libm.h's twin: one set of bits on every machine.

    from machine import libm_twin
    y = libm_twin.sin(x)   # cos, tan, sincos, exp, expm1, tanh, log, arctan, arctan2, arcsin, arccos, hypot

What holds the C to its bits (test_libm) and what `machine.libm` falls back on where no compiler builds its
extension: 81 us a call, no way to call it from the robot's loop - `machine.libm` is what its code calls.

The platform's libm rounds its last bit its own way: Windows' UCRT and Linux's glibc part in numpy's and math's sin,
cos, tan, exp, log, arctan2, arccos and hypot (np.tanh and sqrt alone agree), the robot's estimators took it in
their first pass and its walk fell on one machine and stood on the other. Each function here is IEEE 754's basic
operations and square root alone, wbc_libm.h's step for step (numpy's elementwise operations over an array, no FMA
anywhere), so that each agrees with the C to the bit; the polynomials' coefficients and the constants are
tools/cores/libm.py's (machine/libm_tables.py). An argument of sin, cos or tan past 2^22 is reduced by 2/pi's bits
with Python's integers, as lm_reduce_far does with words of 32. Errors stay under one unit in the last place.
"""
import os
from math import ldexp

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm_tables as T

MAGIC = 6755399441055744.0
SPLIT = 134217729.0
FAR = ldexp(1.0, 22)
SIGN, MAG = 0x8000000000000000, 0x7FFFFFFFFFFFFFFF
U = np.uint64
#: 2^(32k - 190): the weight of the far reduction's fraction word k.
WEIGHT = tuple(ldexp(1.0, 32 * k - 190) for k in range(6))
WORDS = (0, 0) + T.TWO_OVER_PI_WORDS


def _given(x):
    """x as a float64 array of at least one dimension, and whether it came as a scalar."""
    a = np.asarray(x, dtype=np.float64)
    return np.atleast_1d(a), a.ndim == 0


def _out(y, scalar):
    return y[0] if scalar else y


def _bits(x):
    return np.ascontiguousarray(x, dtype=np.float64).view(U)


def _double(u):
    return np.ascontiguousarray(u, dtype=U).view(np.float64)


def _copysign(a, b):
    return _double((_bits(a) & U(MAG)) | (_bits(b) & U(SIGN)))


def _two_prod(a, b):
    p, ga, gb = a * b, SPLIT * a, SPLIT * b
    ah, bh = ga - (ga - a), gb - (gb - b)
    al, bl = a - ah, b - bh
    return p, (((ah * bh - p) + ah * bl) + al * bh) + al * bl


def _e(c, z, powers=None):
    """c[0] + c[1] z + ... by Estrin's pairs: the lower half joined to the upper by z's power past it, as
    wbc_libm.h's lm_e4 .. lm_e14 (z^2, z^4, z^8 squared from z alike)."""
    if powers is None:
        z2 = z * z
        z4 = z2 * z2
        powers = {1: z, 2: z2, 4: z4, 8: z4 * z4}
    n = len(c)
    if n == 1:
        return c[0]
    if n == 2:
        return c[0] + c[1] * z
    m = 1
    while 2 * m < n:
        m *= 2
    return _e(c[:m], z, powers) + powers[m] * _e(c[m:], z, powers)


def _reduce(x):
    """lm_reduce: x - k pi/2 as h + lo for |x| < 2^22, k's low two bits q."""
    t = x * T.TWO_OVER_PI[0] + MAGIC
    k = t - MAGIC
    a = x - k * T.PIO2[0]
    w = -(k * T.PIO2[1])
    b = a + w
    bb = b - a
    lo = ((a - (b - bb)) + (w - bb)) - k * T.PIO2[2]
    h = b + lo
    return h, lo - (h - b), k - 4.0 * (((0.25 * k - 0.375) + MAGIC) - MAGIC)


def _reduce_far(x):
    """lm_reduce_far for one double: the window of 2/pi's bits times x's integer significand, held to 192 bits."""
    u = int(np.float64(x).view(U))
    m, ue = (u & 0xFFFFFFFFFFFFF) | (1 << 52), (u >> 52) & 0x7FF
    at = ue - 1013 if ue > 1013 else 0
    wi, sh = at >> 5, at & 31
    big = 0
    for k in range(6):
        big = (big << 32) | ((((WORDS[wi + k] << 32) | WORDS[wi + k + 1]) << sh) >> 32 & 0xFFFFFFFF)
    p = (m * big) & ((1 << 192) - 1)
    q, f = p >> 190, p & ((1 << 190) - 1)
    neg = (f >> 189) & 1
    if neg:
        f, q = ((1 << 190) - f) & ((1 << 190) - 1), q + 1
    h = lo = 0.0
    for k in range(5, -1, -1):
        v = float((f >> (32 * k)) & 0xFFFFFFFF) * WEIGHT[k]
        s = h + v
        bb = s - h
        lo = lo + ((h - (s - bb)) + (v - bb))
        h = s + lo
        lo = lo - (h - s)
    p, pe = _two_prod(h, T.PI_2[0])
    pe = pe + (h * T.PI_2[1] + lo * T.PI_2[0])
    h = p + pe
    lo = pe - (h - p)
    flip = bool(neg) != (x < 0.0)
    q = float(((-q) if x < 0.0 else q) & 3)
    if not abs(x) < float('inf'):
        return x - x, 0.0, q
    return (-h if flip else h), (-lo if flip else lo), q


def _reduced(x):
    """lm_reduced over an array: the near reduction, its lanes past 2^22 (or not finite) the far one's."""
    h, lo, q = _reduce(x)
    for i in np.flatnonzero(~(np.abs(x) < FAR)):
        h[i], lo[i], q[i] = _reduce_far(float(x[i]))
    return h, lo, q


def _ksin(x, y):
    z = x * x
    v = z * x
    t = (z * (0.5 * y - v * _e(T.SIN[1:], z)) - y) - v * T.SIN[0]
    s = x - t
    return s, (x - s) - t


def _kcos(x, y):
    z = x * x
    hz = 0.5 * z
    w = 1.0 - hz
    t = ((1.0 - w) - hz) + (z * (z * _e(T.COS, z)) - x * y)
    c = w + t
    return c, (w - c) + t


def _quadrant(s, c, q):
    """lm_quadrant: q in 0..3 as a double, s or c by its parity, signed by its half."""
    neg = q >= 2.0
    v = np.where((q - np.where(neg, 2.0, 0.0)) > 0.5, c, s)
    return np.where(neg, -v, v)


def _next(q):
    """q + 1 mod 4, as lm_cos_at takes it."""
    q1 = q + 1.0
    return q1 - np.where(q1 >= 4.0, 4.0, 0.0)


def _trig(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        h, lo, q = _reduced(x)
        (s, sl), (c, cl) = _ksin(h, lo), _kcos(h, lo)
    return x, scalar, q, s, sl, c, cl


def sin(x):
    x, scalar, q, s, _sl, c, _cl = _trig(x)
    return _out(np.where(x == 0.0, x, _quadrant(s, c, q)), scalar)


def cos(x):
    _x, scalar, q, s, _sl, c, _cl = _trig(x)
    return _out(_quadrant(s, c, _next(q)), scalar)


def sincos(x):
    """(sin x, cos x) from one reduction."""
    x, scalar, q, s, _sl, c, _cl = _trig(x)
    return _out(np.where(x == 0.0, x, _quadrant(s, c, q)), scalar), _out(_quadrant(s, c, _next(q)), scalar)


def tan(x):
    """s/c on an even quadrant, -c/s on an odd one, each a sum of two; the quotient corrected by its residual."""
    x, scalar, q, s, sl, c, cl = _trig(x)
    with np.errstate(all='ignore'):
        odd = (q - np.where(q >= 2.0, 2.0, 0.0)) > 0.5
        n, nl, d, dl = np.where(odd, -c, s), np.where(odd, -cl, sl), np.where(odd, s, c), np.where(odd, sl, cl)
        t = n / d
        p, pe = _two_prod(t, d)
        t = t + (((n - p) - pe) + (nl - t * dl)) / d
    return _out(np.where(x == 0.0, x, t), scalar)


def _exp_core(xc):
    """lm_exp_core for x held in range: x = k ln2 + r + rr, e^r = 1 + r + q; k's bits and floor(k/2)'s."""
    t = xc * T.INV_LN2[0] + MAGIC
    n = t - MAGIC
    rh, rl = xc - n * T.LN2[0], -(n * T.LN2[1])
    r = rh + rl
    return r, (r * r) * _e(T.EXP, r), (rh - r) + rl, _bits(t), _bits((0.5 * n - 0.25) + MAGIC)


def _pow2(j, b):
    """lm_pow2: j + b laid into the exponent's field."""
    return _double((j + U(b)) << U(52))


def exp(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        r, q, rr, k, k1 = _exp_core(np.where(np.abs(x) > 746.0, _copysign(np.full_like(x, 746.0), x), x))
        s = 1.0 + r
        s = s + (((1.0 - s) + r) + (q + rr * (1.0 + r)))
        y = (s * _pow2(k1, 1023)) * _pow2(k - k1, 1023)
    return _out(y, scalar)


def expm1(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        r, q, rr, k, k1 = _exp_core(np.where(x > 746.0, 746.0, np.where(x < -40.0, -40.0, x)))
        f, b = _pow2(k1, 1023), -_pow2(U(0) - (k - k1), 1023)
        h = f + b
        bb = h - f
        lo = (f - (h - bb)) + (b - bb)
        ar = f * r
        s = h + ar
        bb = s - h
        s = (s + (((h - (s - bb)) + (ar - bb)) + (lo + f * (q + rr * (1.0 + r))))) * _pow2(k - k1, 1023)
    return _out(_copysign(s, x), scalar)


def tanh(x):
    """wbc_tanh: E / (E + 2), E = e^(2|x|) - 1 as expm1 sums it kept in two parts; the quotient one division and its
    residual by Dekker's product; |x| held to 22."""
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        a = np.abs(x)
        r, q, rr, k, k1 = _exp_core(2.0 * np.where(a > 22.0, 22.0, a))
        f, b = _pow2(k1, 1023), -_pow2(U(0) - (k - k1), 1023)
        h = f + b
        bb = h - f
        lo = (f - (h - bb)) + (b - bb)
        ar = f * r
        s = h + ar
        bb = s - h
        c = ((h - (s - bb)) + (ar - bb)) + (lo + f * (q + rr * (1.0 + r)))
        nh = s + c
        bb = nh - s
        nl = (s - (nh - bb)) + (c - bb)
        g = -2.0 * b
        dh = nh + g
        bb = dh - nh
        dl = ((nh - (dh - bb)) + (g - bb)) + nl
        qh = nh / dh
        p, pe = _two_prod(qh, dh)
        y = qh + ((((nh - p) - pe) + nl) - qh * dl) / dh
        y = np.where(np.isnan(x), x, _copysign(y, x))
    return _out(y, scalar)


def log(x):
    """fdlibm's: x = 2^k (1 + f), s = f/(2 + f), log(1 + f) by R(s^2) in one of fdlibm's two orders."""
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        sub = x < ldexp(1.0, -1022)
        u = _bits(np.where(sub, x * ldexp(1.0, 54), x))
        hx = (u >> U(32)) & U(0xFFFFF)
        i = (hx + U(0x95F64)) & U(0x100000)
        f = _double((u & U(0x000FFFFFFFFFFFFF)) | ((i ^ U(0x3FF00000)) << U(32))) - 1.0
        dk = (_double(((u >> U(52)) + (i >> U(20))) | U(0x4330000000000000)) - 4503599627371519.0) - np.where(
            sub, 54.0, 0.0)
        s = f / (2.0 + f)
        z = s * s
        w = z * z
        hf = (0.5 * f) * f
        g = T.LOG
        r = z * (g[0] + w * (g[2] + w * (g[4] + w * g[6]))) + w * (g[1] + w * (g[3] + w * g[5]))
        m0 = _double((u & U(0x000FFFFFFFFFFFFF)) | U(0x3FF0000000000000))
        y = np.where((m0 >= float.fromhex('0x1.6147Bp0')) & (m0 < float.fromhex('0x1.6B851p0')),
                     dk * T.LN2[0] - ((hf - (s * (hf + r) + dk * T.LN2[1])) - f),
                     dk * T.LN2[0] - ((s * (f - r) - dk * T.LN2[1]) - f))
        y = np.where(x < 0.0, np.nan, y)
        y = np.where(x == 0.0, -np.inf, y)
        y = np.where(x == np.inf, x, y)
        y = np.where(np.isnan(x), x, y)
    return _out(y, scalar)


def _atan_u(num, den):
    """lm_atan_u: fdlibm's range (c = 1, 1/2 or 0) and u + ul = (num - c den)/(den + c num)."""
    t = num / den
    one, half = t >= 0.6875, t >= 0.4375
    c = np.where(one, 1.0, np.where(half, 0.5, 0.0))
    cn = c * num
    n = num - c * den
    dh = den + cn
    bb = dh - den
    dl = (den - (dh - bb)) + (cn - bb)
    u = n / dh
    p, pe = _two_prod(u, dh)
    return u, (((n - p) - pe) - u * dl) / dh, one, half


def _atan(num, den, swap, turn):
    """lm_atan_tail and lm_atan_turn: arctan(c) + arctan(u + ul), from pi/2 less if swapped, from pi if turned."""
    u, ul, one, half = _atan_u(num, den)
    c = T.ATAN_C
    z = u * u
    lo = (np.where(one, c[1], np.where(half, c[3], 0.0)) + ul) + (u * z) * _e(T.ATAN, z)
    hi = np.where(one, c[0], np.where(half, c[2], 0.0))
    h = np.where(swap, T.GRID_PI_2[0] - hi, hi)
    lo = np.where(swap, T.GRID_PI_2[1] - lo, lo)
    v = np.where(swap, -u, u)
    h = np.where(turn, T.PI[0] - h, h)
    lo = np.where(turn, T.PI[1] - lo, lo)
    v = np.where(turn, -v, v)
    s = h + v
    return s + (((h - s) + v) + lo)


def arctan(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        a = np.abs(x)
        inv = a > 1.0
        den = np.where(inv, np.where(a > ldexp(1.0, 900), ldexp(1.0, 900), a), 1.0)
        y = _copysign(_atan(np.where(inv, 1.0, a), den, inv, False), x)
    return _out(y, scalar)


def arctan2(y, x):
    """Annex F's quadrants and zeros; the smaller magnitude over the larger, scaled into [2^-900, 2^900]."""
    y, sy = _given(y)
    x, sx = _given(x)
    y, x = np.broadcast_arrays(y, x)
    with np.errstate(all='ignore'):
        ax, ay = np.abs(x), np.abs(y)
        lo, hi = np.where(ay > ax, ax, ay), np.where(ay > ax, ay, ax)
        sc = np.where(hi > ldexp(1.0, 900), ldexp(1.0, -200), np.where(hi < ldexp(1.0, -900), ldexp(1.0, 200), 1.0))
        s = _atan(np.where(hi == np.inf, 0.0, lo * sc), np.where(hi == np.inf, 1.0, hi * sc), ay > ax, x < 0.0)
        s = np.where(hi == 0.0, np.where(_copysign(np.ones_like(x), x) < 0.0, T.PI[0], 0.0), s)
        s = _copysign(np.where(lo == np.inf, np.where(x > 0.0, T.PI_4[0], T.PI_3_4[0]), s), y)
        s = np.where(np.isnan(x), x + y, s)
        s = np.where(np.isnan(y), x + y, s)
    return _out(s, sy and sx)


def _arc(x):
    """asin's and acos's common part, fdlibm's: z, s = sqrt|z|, r = z R(z), s's high half w and c = (z - w^2)/(s + w)."""
    a = np.abs(x)
    z = np.where(a < 0.5, x * x, 0.5 * (1.0 - a))
    s = np.sqrt(np.abs(z))
    r = z * _e(T.ASIN, z)
    g = SPLIT * s
    w = g - (g - s)
    return a, s, r, w, (z - w * w) / (s + w)


def arcsin(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        a, s, r, w, c = _arc(x)
        p2, p4 = T.PI_2, T.PI_4
        y = np.where(a >= 0.975, p2[0] - (2.0 * (s + s * r) - p2[1]),
                     p4[0] - (((2.0 * s) * r - (p2[1] - 2.0 * c)) - (p4[0] - 2.0 * w)))
        y = np.where(x < 0.0, -y, y)
        y = np.where(a > 1.0, np.nan, y)
        y = np.where(a < 0.5, x + x * r, y)
    return _out(y, scalar)


def arccos(x):
    x, scalar = _given(x)
    with np.errstate(all='ignore'):
        a, s, r, w, c = _arc(x)
        y = np.where(x < 0.0, T.PI[0] - 2.0 * (s + (r * s - T.PI_2[1])), 2.0 * (w + (r * s + c)))
        y = np.where(x == 1.0, 0.0, y)
        y = np.where(a > 1.0, np.nan, y)
        y = np.where(a < 0.5, T.PI_2[0] - (x - (T.PI_2[1] - x * r)), y)
    return _out(y, scalar)


def hypot(x, y):
    """sqrt(x^2 + y^2) scaled by a power of two, the root's rounding corrected by Dekker's squares' residual."""
    x, sx = _given(x)
    y, sy = _given(y)
    x, y = np.broadcast_arrays(x, y)
    with np.errstate(all='ignore'):
        ax, ay = np.abs(x), np.abs(y)
        hi, lo = np.where(ax > ay, ax, ay), np.where(ax > ay, ay, ax)
        big, small = hi > ldexp(1.0, 500), hi < ldexp(1.0, -500)
        sc = np.where(big, ldexp(1.0, -600), np.where(small, ldexp(1.0, 600), 1.0))
        hi, lo = hi * sc, lo * sc
        (hh, he), (ll, le) = _two_prod(hi, hi), _two_prod(lo, lo)
        h = np.sqrt(hh + ll)
        qq, qe = _two_prod(h, h)
        h = (h + (((hh - qq) + ll) + ((he + le) - qe)) / (2.0 * h)) * np.where(
            big, ldexp(1.0, 600), np.where(small, ldexp(1.0, -600), 1.0))
        h = np.where(hi == 0.0, lo, h)
        h = np.where(ax == np.inf, np.inf, h)
        h = np.where(ay == np.inf, np.inf, h)
    return _out(h, sx and sy)
