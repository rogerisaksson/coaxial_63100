#!/usr/bin/env python3
"""The robot's libm's numbers: its polynomials' coefficients and constants, for its C and its Python.

    python tools/cores/libm.py            # each kernel's degree and its error over its interval
    python tools/cores/libm.py --write    # ... laid into wbc/inc/wbc_libm_tables.h and machine/libm_tables.py

Each kernel is the polynomial interpolating its function at Chebyshev's nodes, solved at 60 digits in `decimal`
(within a small factor of the best one's error, no iteration), each coefficient its nearest double; its error is
read at 4000 points of its interval against the function itself. The tables hold nearest doubles, or a high part
on a grid with the rest beside it, so that the sums wbc_libm.h makes of them are exact (no table is indexed by
data: a gather is no vector instruction on every machine, nor one every compiler's tuning lays):
- pi/2 in three parts, 31 bits, 31 bits and the rest: k times either high part is exact for |k| < 2^22;
- ln 2 in two, the high part of 32 bits: k times it is exact for every k exp and log meet;
- arctan(1/2) and pi/4 on the grid of 2^-51, as pi/2 and pi are, so that their differences are exact;
- 2/pi's bits in words of 32, for the reduction of arguments past 2^22.
"""
import argparse
import functools
import math
import os
import sys
from decimal import Decimal, getcontext, localcontext
from fractions import Fraction

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

from tools import REPO

getcontext().prec = 60
C_PATH = os.path.join(REPO, 'wbc', 'inc', 'wbc_libm_tables.h')
PY_PATH = os.path.join(HOST, 'machine', 'libm_tables.py')
NOTICE = "The robot's libm's coefficients and tables: written by host/tools/cores/libm.py, not by hand."
D = Decimal
#: 2/pi's words: past the highest exponent's window (971 + 2 + 192 bits) and a word to shift through.
WORDS = 38


def _atan_small(x, digits=58):
    total, term, k, x2 = D(0), x, 0, x * x
    while abs(term) > abs(x) * D(10) ** -digits:
        total += term / (2 * k + 1) * (-1 if k % 2 else 1)
        term *= x2
        k += 1
    return total


def atan(x):
    """arctan at 60 digits: halved till under 1/8, its series, doubled back."""
    x, n = D(x), 0
    while abs(x) > D('0.125'):
        x = x / (1 + (1 + x * x).sqrt())
        n += 1
    return _atan_small(x) * (2 ** n)


PI = 16 * _atan_small(D(1) / 5) - 4 * _atan_small(D(1) / 239)
LN2 = D(2).ln()


def sin(x):
    x = D(x)
    total, term, k = D(0), x, 1
    while abs(term) > abs(x) * D(10) ** -58:
        total += term
        term *= -x * x / ((2 * k) * (2 * k + 1))
        k += 1
    return total


def cos(x):
    x = D(x)
    total, term, k = D(0), D(1), 1
    while abs(term) > D(10) ** -58:
        total += term
        term *= -x * x / ((2 * k - 1) * (2 * k))
        k += 1
    return total


def asin(x):
    x = D(x)
    return atan(x / (1 - x * x).sqrt()) if abs(x) < 1 else (PI / 2 if x > 0 else -PI / 2)


def nearest(q):
    """q's nearest double."""
    return float(Fraction(q))


def bits(q, n):
    """q rounded to n significant bits, a double."""
    q = Fraction(q)
    if q == 0:
        return 0.0
    e = 0
    while abs(q) >= 2 ** (e + 1):
        e += 1
    while abs(q) < 2 ** e:
        e -= 1
    scale = Fraction(2) ** (n - 1 - e)
    return float(Fraction(round(q * scale)) / scale)


def grid(q, e):
    """q rounded to a multiple of 2^e, a double."""
    scale = Fraction(2) ** -e
    return float(Fraction(round(Fraction(q) * scale)) / scale)


def split(q, hi):
    """[q's high part, the rest's nearest double]."""
    return [hi, nearest(Fraction(q) - Fraction(hi))]


def ln(q):
    """log of a Fraction at 60 digits."""
    return (D(q.numerator) / D(q.denominator)).ln()


def fit(g, a, b, n):
    """The degree-n polynomial through g at Chebyshev's nodes of [a, b]: its coefficients, the lowest first."""
    a, b = D(a), D(b)
    xs = [(a + b) / 2 + (b - a) / 2 * cos(PI * (2 * k + 1) / (2 * (n + 1))) for k in range(n + 1)]
    rows = [[x ** j for j in range(n + 1)] + [g(x)] for x in xs]
    for c in range(n + 1):
        p = max(range(c, n + 1), key=lambda r: abs(rows[r][c]))
        rows[c], rows[p] = rows[p], rows[c]
        for r in range(n + 1):
            if r != c:
                f = rows[r][c] / rows[c][c]
                rows[r] = [u - f * v for u, v in zip(rows[r], rows[c])]
    return [rows[j][n + 1] / rows[j][j] for j in range(n + 1)]


def horner(c, x):
    q = D(c[-1])
    for k in range(len(c) - 2, -1, -1):
        q = D(c[k]) + x * q
    return q


#: Under this a kernel's function is its limit at 0: a node of an odd count of them falls at the middle of a
#: symmetric interval, 1e-63 as `cos` reads it, where e^r - 1 - r holds no digit at 60.
TINY = D(10) ** -25
SQ = (PI / 4) ** 2
#: Each kernel: (its name, the function its polynomial stands for, its interval, its degree, how its error is
#: read, over what). SIN and COS on |r| <= pi/4; EXP on exp's residual |r| <= ln2/2 (and its low part's room);
#: LOG as fdlibm's R(z) = z g(z), z = s^2, s = f/(2 + f) for 1 + f in [sqrt(2)/2, sqrt(2)); ATAN on arctan's
#: residual tangent |u| <= 7/16; ASIN on asin's and acos's z <= 1/4.
LH = (D(2).sqrt() - 1) / (D(2).sqrt() + 1)
KERNELS = (
    ('SIN', lambda z: (sin(z.sqrt()) / z.sqrt() - 1) / z if z > TINY else D(-1) / 6, 0, SQ, 6,
     lambda c, r: (r + r * (r * r) * horner(c, r * r) - sin(r)) / sin(r), (D('1e-9'), PI / 4)),
    ('COS', lambda z: (cos(z.sqrt()) - 1 + z / 2) / (z * z) if z > TINY else D(1) / 24, 0, SQ, 5,
     lambda c, r: ((1 - r * r / 2) + (r * r) ** 2 * horner(c, r * r) - cos(r)) / cos(r), (D(0), PI / 4)),
    ('EXP', lambda r: ((r.exp() - 1 - r) / (r * r)) if abs(r) > TINY else D(1) / 2, -LN2 / 2 - D(2) ** -30,
     LN2 / 2 + D(2) ** -30, 10,
     lambda c, r: (1 + r + r * r * horner(c, r) - r.exp()) / r.exp(), (-LN2 / 2, LN2 / 2)),
    ('LOG', lambda z: ((((1 + z.sqrt()) / (1 - z.sqrt())).ln() / z.sqrt() - 2) / z) if z > TINY else D(2) / 3, 0,
     LH * LH, 6,
     lambda c, s: (2 * s + s * (s * s) * horner(c, s * s) - ((1 + s) / (1 - s)).ln()) / ((1 + s) / (1 - s)).ln(),
     (D('1e-9'), LH)),
    ('ATAN', lambda z: (atan(z.sqrt()) / z.sqrt() - 1) / z if z > TINY else D(-1) / 3, 0, (D(7) / 16) ** 2, 11,
     lambda c, u: (u + u * (u * u) * horner(c, u * u) - atan(u)) / atan(u), (D('1e-9'), D(7) / 16)),
    ('ASIN', lambda z: (asin(z.sqrt()) / z.sqrt() - 1) / z if z > TINY else D(1) / 6, 0, D('0.25'), 13,
     lambda c, s: (s + s * (s * s) * horner(c, s * s) - asin(s)) / asin(s), (D('1e-9'), D('0.5'))),
)


def kernels(read=True):
    """{name: coefficients as doubles}, and each one's worst error over its interval (a Decimal) where read."""
    out, worst = {}, {}
    for name, g, a, b, n, err, span in KERNELS:
        c = [nearest(x) for x in fit(g, a, b, n)]
        lo, hi = span
        if read:
            worst[name] = max(abs(err(c, lo + (hi - lo) * (2 * k + 1) / 8000)) for k in range(4000))
        out[name] = c
    return out, worst


@functools.lru_cache(maxsize=None)
def pi_wide():
    """pi at 420 digits (Machin's): wide enough to reduce any double by pi/2 and keep 60 digits of the rest."""
    with localcontext() as ctx:
        ctx.prec = 420
        return 16 * _atan_small(D(1) / 5, 415) - 4 * _atan_small(D(1) / 239, 415)


def two_over_pi():
    """2/pi's first WORDS words of 32 bits after its point, at 420 digits."""
    with localcontext() as ctx:
        ctx.prec = 420
        whole = int((2 / pi_wide()) * (D(2) ** (32 * WORDS)))
    return [(whole >> (32 * (WORDS - 1 - i))) & 0xFFFFFFFF for i in range(WORDS)]


def _quarter(x):
    """(x - k pi/2 at 60 digits, k mod 4) for a double x of any size: k its nearest integer to 2x/pi."""
    with localcontext() as ctx:
        ctx.prec = 420
        half = pi_wide() / 2
        k = (D(x) / half).to_integral_value()
        r = D(x) - k * half
    return +r, int(k) % 4


def _expm1(x):
    x = D(x)
    if abs(x) >= D('0.01'):
        return x.exp() - 1
    total, term, k = D(0), x, 1
    while abs(term) > abs(x) * D(10) ** -58:
        total += term
        k += 1
        term = term * x / k
    return total


def _atan2(y, x):
    if y == 0.0:
        return D(math.copysign(1.0, y)) * (PI if math.copysign(1.0, x) < 0.0 else D(0))
    y, x = D(y), D(x)
    if x == 0:
        return PI / 2 if y > 0 else -PI / 2
    a = atan(y / x)
    return a if x > 0 else (a + PI if y >= 0 else a - PI)


def reference(name, a, b=None):
    """A function of the libm at the doubles a (and b), finite and with a finite value, to 60 digits: the reference
    test_libm and the bench read units in the last place against."""
    if name in ('sin', 'cos', 'tan'):
        r, q = _quarter(a)
        s, c = sin(r), cos(r)
        s, c = ((s, c), (c, -s), (-s, -c), (-c, s))[q]
        return {'sin': s, 'cos': c, 'tan': s / c}[name]
    one = {'exp': lambda x: D(x).exp(), 'expm1': _expm1, 'log': lambda x: D(x).ln(), 'arctan': atan, 'arcsin': asin,
           'arccos': lambda x: PI / 2 - asin(x)}
    if name in one:
        return one[name](a)
    if b is None:
        raise ValueError('%s takes two arguments' % name)
    return _atan2(a, b) if name == 'arctan2' else (D(a) ** 2 + D(b) ** 2).sqrt()


def ulps(y, q):
    """|y - q| in units in the last place of q's nearest double (the subnormals' step at the least)."""
    f = float(q)
    e = math.frexp(abs(f))[1] if f != 0.0 else -1073
    return abs(D(y) - q) / (D(2) ** max(e - 53, -1074))


def tables():
    """[(name, its C type, values)]: the constants."""
    pio2 = Fraction(PI / 2)
    p1 = bits(pio2, 31)
    p2 = bits(pio2 - Fraction(p1), 31)
    at = atan(D(1) / 2)
    t = [('PIO2', [p1, p2, nearest(pio2 - Fraction(p1) - Fraction(p2))]),
         ('TWO_OVER_PI', [nearest(2 / PI)]),
         ('PI_2', split(PI / 2, nearest(PI / 2))),
         ('PI_4', split(PI / 4, nearest(PI / 4))),
         ('PI', split(PI, nearest(PI))),
         ('PI_3_4', [nearest(3 * PI / 4)]),
         ('GRID_PI_2', split(PI / 2, grid(PI / 2, -51))),
         ('ATAN_C', split(PI / 4, grid(PI / 4, -51)) + split(at, grid(at, -51))),
         ('INV_LN2', [nearest(1 / LN2)]),
         ('LN2', split(LN2, bits(LN2, 32)))]
    out: list = [(name, 'double', values) for name, values in t]
    out.append(('TWO_OVER_PI_WORDS', 'uint32_t', two_over_pi()))
    return out


def _rows(items, per):
    return [', '.join(items[i:i + per]) for i in range(0, len(items), per)]


def laid(read=True):
    """(the C header's text, the Python module's text, each kernel's worst error where read)."""
    ks, worst = kernels(read)
    every = [(name, 'double', values) for name, values in ks.items()] + tables()
    c = ['/** ' + NOTICE + ' */', '#ifndef WBC_LIBM_TABLES_H', '#define WBC_LIBM_TABLES_H', '',
         '#include <stdint.h>', '']
    py = ['"""' + NOTICE + '"""', 'H = float.fromhex', '']
    for name, ctype, values in every:
        if ctype == 'double':
            ci, pi = [v.hex() for v in values], ["'%s'" % v.hex() for v in values]
            head, tail = '%s = tuple(map(H, (' % name, ')))'
        else:
            ci, pi = ['0x%xU' % v for v in values], ['0x%x' % v for v in values]
            head, tail = '%s = (' % name, ')'
        c.append('static const %s lm_%s[%d] = {' % (ctype, name.lower(), len(values)))
        c += ['  ' + row + ',' for row in _rows(ci, 4)] + ['};']
        py.append(head)
        py += ['    ' + row + ',' for row in _rows(pi, 4)] + [tail]
    c += ['', '#endif', '']
    return '\n'.join(c), '\n'.join(py) + '\n', worst


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--write', action='store_true', help='lay the C header and the Python module')
    args = parser.parse_args(argv)
    c, py, worst = laid()
    for name, e in worst.items():
        print('%-8s worst %.3g' % (name, float(e)))
    if args.write:
        for path, text in ((C_PATH, c), (PY_PATH, py)):
            with open(path, 'w', encoding='utf-8', newline='\n') as f:
                f.write(text)
            print('written', path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
