"""The walk's curves, pure: an ease, Hermite polynomials, Catmull-Rom, a pitched foot.

The ends' conditions to any order (value, rate, acceleration, jerk); the periodic curve
through a stride's samples; the ankle of a foot pitched about a point of its sole.
"""
import math


def eased(x):
    """0 to 1 over 0 to 1 with no jerk at either end (smootherstep)."""
    x = min(1.0, max(0.0, x))
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


def pivot(x, y, dx, dy, pitch):
    """The ankle, from a point (x, y) on the foot and the ankle's offset (dx, dy) from it on a flat
    foot, the foot pitched `pitch` degrees toes-up about that point."""
    c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    return x + dx * c - dy * s, y + dx * s + dy * c


def through(samples, p):
    """The periodic Catmull-Rom curve through `samples` (a stride's) at phase `p`."""
    n = len(samples)
    x = (p % 1.0) * n
    i, u = int(x), x - int(x)
    a, b, c, d = (samples[(i + k) % n] for k in (-1, 0, 1, 2))
    return 0.5 * (2.0 * b + (c - a) * u + (2.0 * a - 5.0 * b + 4.0 * c - d) * u * u
                  + (3.0 * (b - c) + d - a) * u * u * u)


#: Hermite bases, u^0..u^7 coefficients per end condition in order (value, rate, acceleration,
#: jerk) at u 0 then at u 1: the quintic from three, the septic from four.
BASES = {3: ((1, 0, 0, -10, 15, -6), (0, 1, 0, -6, 8, -3), (0, 0, 0.5, -1.5, 1.5, -0.5),
              (0, 0, 0, 10, -15, 6), (0, 0, 0, -4, 7, -3), (0, 0, 0, 0.5, -1, 0.5)),
          4: ((1, 0, 0, 0, -35, 84, -70, 20), (0, 1, 0, 0, -20, 45, -36, 10),
              (0, 0, 0.5, 0, -5, 10, -7.5, 2), (0, 0, 0, 1 / 6, -2 / 3, 1, -2 / 3, 1 / 6),
              (0, 0, 0, 0, 35, -84, 70, -20), (0, 0, 0, 0, -15, 39, -34, 10),
              (0, 0, 0, 0, 2.5, -7, 6.5, -2), (0, 0, 0, 0, -1 / 6, 0.5, -0.5, 1 / 6))}


def hermite(start, end, u):
    """The polynomial from `start` at u 0 to `end` at u 1, each (value, rate, acceleration[,
    jerk]) over u."""
    powers = [u ** k for k in range(2 * len(start))]
    return sum(w * sum(c * p for c, p in zip(basis, powers))
               for w, basis in zip(list(start) + list(end), BASES[len(start)]))


def ends(f, q, h, span):
    """Per coordinate of f at q: (value, rate, acceleration, jerk) over `span`, differenced one
    side, steps of `h` (negative, behind)."""
    return [(a, (4.0 * b - 3.0 * a - c) / (2.0 * h) * span, (a - 2.0 * b + c) / h ** 2 * span ** 2,
             (3.0 * (b - c) + d - a) / h ** 3 * span ** 3)
            for a, b, c, d in zip(*(f(q + k * h) for k in range(4)))]


def septic(x):
    """0 to 1 over 0 to 1, its first three derivatives nothing at either end."""
    x = min(1.0, max(0.0, x))
    return x ** 4 * (35.0 - 84.0 * x + 70.0 * x * x - 20.0 * x ** 3)
