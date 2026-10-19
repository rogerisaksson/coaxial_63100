"""The order numpy sums an einsum's terms in, read off its own result (`tools.cores.emit`'s einsums).

    order = orders.ordered(st)      # 'seq', 'lanes2' or 'lanes4': the first whose sums are numpy's bits
"""
import numpy as np

from tools.cores.trace import Untraced

#: The orders an einsum's terms may be summed in, as numpy 2's sum_of_products adds them: a plain
#: sum into its zeroed output, or SIMD lanes of 2 (SSE) or 4 (AVX2) summed across at the end.
ORDERS = ('seq', 'lanes2', 'lanes4')


def same(a, b):
    """Whether `a` and `b` hold the same doubles, bit for bit."""
    a, b = np.ascontiguousarray(a, dtype=float), np.ascontiguousarray(b, dtype=float)
    return a.shape == b.shape and np.array_equal(a.view(np.int64), b.view(np.int64))


def summed(P, order):
    """`P` (..., J)'s terms summed along their last axis in `order`, numpy's arithmetic."""
    J = P.shape[-1]
    if order == 'seq':
        acc = np.zeros(P.shape[:-1])
        for j in range(J):
            acc = acc + P[..., j]
        return acc
    v = int(order[5:])
    lanes = [np.zeros(P.shape[:-1]) for _ in range(v)]
    t = 0
    while J - t >= 4 * v:
        for k in range(v):
            lanes[k] = (((lanes[k] + P[..., t + 3 * v + k]) + P[..., t + 2 * v + k]) + P[..., t + v + k]) + P[..., t + k]
        t += 4 * v
    while t < J:
        for k in range(v):
            lanes[k] = lanes[k] + (P[..., t + k] if t + k < J else 0.0)
        t += v
    across = lanes[0] + lanes[1] if v == 2 else (lanes[0] + lanes[2]) + (lanes[1] + lanes[3])
    return 0.0 + across


def letters(st):
    """(each operand's letters, the output's, the one letter summed) of an einsum statement."""
    subscripts = st.args[0].replace(' ', '')
    ins, out = subscripts.split('->')
    ops = ins.split(',')
    contracted = sorted(set(''.join(ops)) - set(out))
    if len(contracted) != 1 or any(len(set(o)) != len(o) for o in ops):
        raise Untraced('einsum %r: one index summed, none repeated in an operand (%s)' % (subscripts, st.where))
    return ops, out, contracted[0]


def products(st):
    """The einsum's terms (out..., J), numpy's products left to right."""
    ops, out, c = letters(st)
    full = out + c
    factors = []
    for lets, value in zip(ops, st.args[2]):
        axes = [lets.index(x) for x in full if x in lets]
        moved = np.transpose(value, axes)
        factors.append(moved.reshape([value.shape[lets.index(x)] if x in lets else 1 for x in full]))
    P = factors[0]
    for f in factors[1:]:
        P = P * f
    return P


def ordered(st):
    """The order numpy summed an einsum's terms in, as its result shows: the first of ORDERS its
    values are bit for bit; a plain sum where the terms cannot tell (one term, or zeros)."""
    P = products(st)
    found = [o for o in ORDERS if same(summed(P, o), st.value)]
    if not found:
        raise Untraced('einsum %r in no order numpy sums in (%s)' % (st.args[0], st.where))
    return 'seq' if 'seq' in found else found[0]
