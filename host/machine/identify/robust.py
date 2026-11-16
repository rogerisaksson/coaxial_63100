"""A linear fit as Huber's M-estimate: least squares reweighed off each group's own residual scale.

    x, w = robust.fitted(A, b, group)   # A x = b, each row's weight of its residual against its group's
    c = robust.huber(0.95)              # the threshold, in scales, of that efficiency on normal residuals

Least squares weighs every row's residual alike, so a few rows a model cannot place - a pendulum the
robot does not measure striking its stop, an impulse a step off its time - pull every parameter they
touch. Huber's estimate is least squares within `huber(EFFICIENCY)` of a row's scale and a straight line
beyond it, each group's scale its median absolute residual over the normal's (`SPREAD`).
"""
from statistics import NormalDist

#: An M-estimate's efficiency on normal residuals, of least squares': test_sysid's world off the figure
#: read the robot's right hip yaw's friction 11.6 % off by least squares, its shank's hem striking its
#: stops through the leg's stage (`campaign.hemming`), and 1.7 % so (18.4 and 1.6 % at its servo's
#: 523 N m/rad), every drive's within 2.9 %; the world's own hems in its rows read it 3.7 and 2.8 %.
EFFICIENCY = 0.95
#: A normal residual's standard deviation over its median absolute value.
SPREAD = 1.0 / NormalDist().inv_cdf(0.75)


def huber(efficiency=EFFICIENCY):
    """Huber's threshold, in scales, whose estimate has `efficiency` of least squares' on normal
    residuals: (2 P - 1)^2 / (2 P - 1 - 2 c p + 2 c^2 (1 - P)), P and p the normal's at c - bisected
    to the float, the efficiency rising with c."""
    unit = NormalDist()

    def of(c):
        inside = 2.0 * unit.cdf(c) - 1.0
        return inside * inside / (inside - 2.0 * c * unit.pdf(c) + 2.0 * c * c * (1.0 - unit.cdf(c)))
    lo, hi = 0.0, 1.0
    while of(hi) < efficiency:
        lo, hi = hi, 2.0 * hi
    while True:
        mid = 0.5 * (lo + hi)
        if not lo < mid < hi:
            return mid
        lo, hi = (mid, hi) if of(mid) < efficiency else (lo, mid)


def fitted(A, b, group, efficiency=EFFICIENCY):
    """(x, w) of A x = b, `A` (M, k), `b` (M,), by Huber's estimate: least squares reweighed, a row's
    weight min(1, c s / |r|) of its residual r against its group's scale s (`group` (M,) a row's; SPREAD
    times its median absolute residual, a group without one at weight 1), c `huber(efficiency)`; reweighed
    until the rows weighed down and their residuals' signs repeat, at most k times, each a solve of the
    normal equations - `A`'s columns scaled alike, its Gram well conditioned."""
    import numpy as np
    c, w, side, x = huber(efficiency), np.ones(len(b)), None, np.zeros(A.shape[1])
    groups = [np.flatnonzero(group == g) for g in np.unique(group)]
    for _ in range(max(1, A.shape[1])):
        x = np.linalg.solve(A.T @ (A * w[:, None]), A.T @ (w * b))
        r = b - A @ x
        s = np.zeros(len(b))
        for at in groups:
            s[at] = SPREAD * np.median(np.abs(r[at]))
        w = np.where(s > 0.0, np.minimum(1.0, c * s / np.maximum(np.abs(r), np.finfo(float).tiny)), 1.0)
        now = np.sign(r) * (w < 1.0)
        if side is not None and np.array_equal(now, side):
            break
        side = now
    return x, w
