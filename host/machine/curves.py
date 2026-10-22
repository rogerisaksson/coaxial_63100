"""The ortho-DSP suite's curves as shares of the way, and as quintics for a row's blend on her loop.

    share = curves.CURVES['equal-power'](u)              # the curve itself, by the suite's name, u in [0, 1]
    c1, c2, c3, c4, c5 = curves.QUINTICS['equal_power']  # its blend's coefficients on u..u^5 (`latent.SHAPES`)

A morph's bands take the curves (`patch.morph`), her rows' blends the quintics (`ortho.SHAPES`).
"""
import math

#: An exponential with e^-SPAN_E of its way left at its end; an audio taper's swell over SWELL_DB.
SPAN_E, SWELL_DB = 4.0, 40.0
#: Samples a quintic is fitted on, u in [0, 1].
FIT_N = 65


def _swell(u):
    return (10.0 ** (SWELL_DB / 20.0 * u) - 1.0) / (10.0 ** (SWELL_DB / 20.0) - 1.0)


#: The curves by the suite's names: an exponential, the equal-power crossfade's energy share, an
#: audio taper's swell and its mirror settling, Hermite's smoothstep, a line.
CURVES = {
    'exponential-fast': lambda u: (1.0 - math.exp(-SPAN_E * u)) / (1.0 - math.exp(-SPAN_E)),
    'equal-power': lambda u: math.sin(0.5 * math.pi * u) ** 2,
    'logarithmic-swell': _swell,
    'logarithmic-settle': lambda u: 1.0 - _swell(1.0 - u),
    'hermite-s-curve': lambda u: u * u * (3.0 - 2.0 * u),
    'linear': lambda u: u,
}


def _damped(a):
    """An exponential approach from rest, `a` its rate over the way: a critically damped step's
    response, (1 - (1 + a u) e^-au) normalized to 1 at its end."""
    return lambda u: (1.0 - (1.0 + a * u) * math.exp(-a * u)) / (1.0 - (1.0 + a) * math.exp(-a))


def _from_rest(f):
    """f's least squares quintic on u in [0, 1], still at 0 and 1 at 1: its coefficients on u..u^5,
    c2..c4 of the normal equations of u^k - u^5 against f - u^5 (Cramer's rule)."""
    us = [i / (FIT_N - 1.0) for i in range(FIT_N)]
    cols = [[u ** k - u ** 5 for u in us] for k in range(2, 5)]
    rhs = [f(u) - u ** 5 for u in us]
    a = [[sum(p * q for p, q in zip(ci, cj)) for cj in cols] for ci in cols]
    b = [sum(p * q for p, q in zip(ci, rhs)) for ci in cols]

    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    whole = det(a)
    c = [det([[b[r] if j == i else a[r][j] for j in range(3)] for r in range(3)]) / whole for i in range(3)]
    return (0.0, c[0], c[1], c[2], 1.0 - sum(c))


_LINE, _JERK = (1.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 10.0, -15.0, 6.0)
_SETTLE = _from_rest(_damped(SWELL_DB / 20.0 * math.log(10.0)))
#: Each curve as a row's blend on her loop (`ortho.SHAPES`, in this order), a quintic's coefficients
#: on u..u^5, leaving still: a line, Hermite's smoothstep and the minimum jerk's exact - a soft
#: landing Hermite's quintic, unaccelerated at both ends -; the equal-power and the swell least
#: squares; the exponential and the settle as a critically damped step's response (`_damped`) -
#: leaving at their own slope, a rise's preload asked her centre of mass 1.8 m/s at its first pass
#: and lifted her off the chair in 50 ms.
QUINTICS = {
    'linear': _LINE, 'linear_fast': _LINE, 'hermite_soft_landing': _JERK, 'hermite_s_curve': (0.0, 3.0, -2.0, 0.0, 0.0),
    'minimum_jerk_deceleration': _JERK, 'equal_power': _from_rest(CURVES['equal-power']),
    'exponential_fast': _from_rest(_damped(SPAN_E)), 'logarithmic_swell': _from_rest(_swell),
    'logarithmic_settle': _SETTLE, 'logarithmic_brake': _SETTLE,
}
#: Each blend's greatest |S''| on [0, 1], its acceleration in its way over its time squared
#: (`maneuver._paced`).
BENT = {name: max(abs(sum((k + 1) * k * c[k] * (i / (FIT_N - 1.0)) ** (k - 1) for k in range(1, 5))) for i in range(FIT_N))
        for name, c in QUINTICS.items()}
