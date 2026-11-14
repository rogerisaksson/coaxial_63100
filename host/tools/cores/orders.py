"""The order numpy sums an einsum's terms in, read off its own result (`tools.cores.emit`'s einsums).

    perm, inner = orders.ordered(st)    # the summed letters outer to inner; an order of ORDERS

Each run along the innermost summed index summed in `inner` (numpy's sum_of_products), added to the
output in turn over the outer ones.
"""
import itertools
import math
from fractions import Fraction

import numpy as np

from tools.cores.trace import Ref, Untraced

#: The orders an einsum's terms may be summed in, as numpy 2's sum_of_products adds them: a plain
#: sum into its zeroed output, or SIMD lanes of 2 (SSE) or 4 (AVX2) summed across at the end; on an
#: Arm each product fused into its add (NEON's npyv_muladd a fused multiply-add: a contiguous pair
#: of operands 'lanes2fma', 'nji,nj->ni' 'seqfma'; strided ones and three plain), its unfused first.
ORDERS = ('seq', 'lanes2', 'lanes4', 'seqfma', 'lanes2fma')
#: The draws, and their seed, an einsum's order is read on where its own result fits more than one:
#: an order is numpy's algorithm, terms alike or zero leave it untold (her capsules' ends, most 0
#: across, summed 'seq' where numpy's AVX2 sums in lanes).
DRAWS, SEED = 3, 5


def same(a, b):
    """Whether `a` and `b` hold the same doubles, bit for bit."""
    a, b = np.ascontiguousarray(a, dtype=float), np.ascontiguousarray(b, dtype=float)
    return a.shape == b.shape and np.array_equal(a.view(np.int64), b.view(np.int64))


def _fma(x, y, z):
    """x * y + z rounded once, as C's fma: exact in rationals, rounded to nearest; an exact zero's sign IEEE's."""
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return x * y + z
    exact = Fraction(x) * Fraction(y) + Fraction(z)
    if exact == 0:
        return x * y + z
    try:
        return float(exact)
    except OverflowError:
        return math.copysign(math.inf, exact)


_FUSED = np.frompyfunc(_fma, 3, 1)


def fused(a, b, c):
    """a * b + c each rounded once (C's fma), as numpy's NEON adds a product."""
    return np.asarray(_FUSED(a, b, c), dtype=float)


def summed(A, B, order):
    """The terms A * B (..., J) summed along their last axis in `order`, numpy's arithmetic: each product
    rounded and added, or fused into its add (an order ending 'fma')."""
    J, fma = A.shape[-1], order.endswith('fma')

    def add(a, b, acc):
        return fused(a, b, acc) if fma else acc + a * b
    if order.startswith('seq'):
        acc = np.zeros(A.shape[:-1])
        for j in range(J):
            acc = add(A[..., j], B[..., j], acc)
        return acc
    v = int(order[5])
    lanes = [np.zeros(A.shape[:-1]) for _ in range(v)]
    t = 0
    while J - t >= 4 * v:
        for k in range(v):
            for u in (3, 2, 1, 0):
                lanes[k] = add(A[..., t + u * v + k], B[..., t + u * v + k], lanes[k])
        t += 4 * v
    while t < J:
        for k in range(v):
            lanes[k] = add(A[..., t + k], B[..., t + k], lanes[k]) if t + k < J else lanes[k] + 0.0
        t += v
    across = lanes[0] + lanes[1] if v == 2 else (lanes[0] + lanes[2]) + (lanes[1] + lanes[3])
    return 0.0 + across


def letters(st):
    """(each operand's letters, the output's, the letters summed) of an einsum statement."""
    subscripts = st.args[0].replace(' ', '')
    ins, out = subscripts.split('->')
    ops = ins.split(',')
    contracted = sorted(set(''.join(ops)) - set(out))
    if not contracted or any(len(set(o)) != len(o) for o in ops):
        raise Untraced('einsum %r: an index summed, none repeated in an operand (%s)' % (subscripts, st.where))
    return ops, out, contracted


def products(st, perm, values=None):
    """The einsum's terms (out..., the summed letters of `perm` in its order) of its operands' `values`
    (its own where None) as (A, B), broadcast alike: numpy's products left to right, the last factor B
    (1 of one operand) - a term A * B, or B's product fused into its add."""
    ops, out, _c = letters(st)
    full = out + ''.join(perm)
    factors = []
    for lets, value in zip(ops, st.args[2] if values is None else values):
        axes = [lets.index(x) for x in full if x in lets]
        moved = np.transpose(value, axes)
        factors.append(moved.reshape([value.shape[lets.index(x)] if x in lets else 1 for x in full]))
    A = factors[0]
    for f in factors[1:-1]:
        A = A * f
    A, B = np.broadcast_arrays(A, factors[-1] if len(factors) > 1 else np.ones(1))
    return A, B


def grouped(A, B, n, inner):
    """A * B (..., n summed axes)'s terms: each run along the last summed in `inner`, added to zero in
    turn over the outer ones in order."""
    if n == 1:
        return summed(A, B, inner)
    acc = np.zeros(A.shape[:-n])
    for idx in itertools.product(*[range(m) for m in A.shape[-n:-1]]):
        at = (Ellipsis,) + idx + (slice(None),)
        acc = acc + summed(A[at], B[at], inner)
    return acc


def _drawn(r, v, rng):
    """Normal draws laid as operand `v` (its Ref `r`, a constant's own array) was: numpy's loop over
    them its loop over it."""
    strides = r.strides if isinstance(r, Ref) else [s // v.itemsize for s in v.strides]
    at = Ref(None, 0, strides, v.shape).addresses()
    lo = int(at.min(initial=0))
    span = rng.normal(size=int(at.max(initial=0)) - lo + 1)
    return np.lib.stride_tricks.as_strided(span[-lo:], v.shape, [s * span.itemsize for s in strides])


def _fits(st, values, value, found):
    """The (permutation, order)s of `found` whose sum of the terms of `values` is `value` bit for bit."""
    return [(perm, o) for perm, o in found if same(grouped(*products(st, perm, values), len(perm), o), value)]


def ordered(st):
    """(the summed letters outer to inner, the inner run's order) numpy summed an einsum's terms in,
    as its result shows - where it fits more than one, as its result on DRAWS laid as its operands
    are shows: the first permutation with an order of ORDERS bit for bit, a plain sum where the
    terms cannot tell; the fused orders tried where no unfused one fits (an Arm's)."""
    _ops, _out, contracted = letters(st)
    perms = list(itertools.permutations(contracted))
    for fma in (False, True):
        found = _fits(st, None, st.value, [(p, o) for p in perms for o in ORDERS if o.endswith('fma') == fma])
        if found:
            break
    rng = np.random.default_rng(SEED)
    for _ in range(DRAWS if len(found) > 1 else 0):
        draws = [_drawn(r, v, rng) for r, v in zip(st.args[1], st.args[2])]
        found = _fits(st, draws, np.einsum(st.args[0], *draws), found) or found
    if not found:
        raise Untraced('einsum %r in no order numpy sums in (%s)' % (st.args[0], st.where))
    perm = found[0][0]
    return perm, 'seq' if (perm, 'seq') in found else found[0][1]
