"""Where a swinging foot lands across, data-oriented: a row a leg, a step as machine.cyclic's.

    s = state()                         # a leg a row, at rest, the defaults on every row
    x, swapping, catch, off = landing(s, u)  # u: rows of q, sign, xi, standing, sep, rate, omega

The capture point xi - the centre of mass plus its speed over omega, sqrt(g/h) - is where a foot
must stand to stop her. Walking, it runs out from the standing foot toward the swinging one along
XI_NOM by the swing's progress (measured, 2026-09-26), and the foot lands `sep` out: the walk's
own. What the capture point is off that course, the ankle holds while it stays within `margin`
of the foot's middle - out, the course has used most of that by mid-swing: at 0.65 strides/s
25 mm off at 0.7 of the swing doubled and the foot landed short (2026-09-26) - and the rest
grows to the landing by e^(omega tau), the inverted pendulum's; the foot
lands `gain` of that further out - a little past the capture point, to bring her back: shoved
120 N, the foot landed 24 mm short of its mark and the capture point 50 mm past it, more than
the sole holds; at 1.3 the step back grew by |1 - gain| e^(omega T), 1.35 a step (2026-09-26). Asked across the standing foot it lands no nearer than `cross`; asked `swap`
across, she swaps: this foot down at once, the other steps out (`machine.walker`, the side step).
The lateral is latched from `latch_u` of the swing: moved to the last, the foot landed 25 mm off
it.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402
from numpy.typing import NDArray  # noqa: E402

from machine.gait import TOE_OFF  # noqa: E402

#: The capture point's course out from the standing foot toward the swinging one, m, by the swing's
#: progress: the walk on this law at 0.65, 0.85 and 0.9 strides/s, meaned (2026-09-26, spread
#: +-0.016). Scaled to the feet's separation, the wide first steps were expected to sway 8-14 cm
#: and the feet were put down crossed.
U_NOM = (0.0, 0.14, 0.28, 0.41, 0.54, 0.67, 0.80, 0.93, 1.0)
XI_NOM = (0.000, 0.006, 0.013, 0.026, 0.033, 0.034, 0.042, 0.050, 0.051)

#: The row: its parameters, then its memory - the lateral latched and whether, the swap asked.
PARAMS = ('margin', 'gain', 'dead', 'cross', 'swap', 'from_u', 'latch_u', 'catch')
CAPTURE = np.dtype([(n, 'f8') for n in PARAMS + ('x', 'latched', 'swapping')])

#: The sole's half width the ankle holds within, m; the landing's gain on what is off; the dead
#: band, m; the nearest across, the crossing that swaps, the swing's progress a swap is asked
#: within; the progress the lateral is latched from; a landing this far off the walk's is a
#: catch: `state`'s defaults, in PARAMS' order.
MARGIN, GAIN, DEAD, CROSS, SWAP, FROM_U, LATCH_U, CATCH = (0.035, 1.1, 0.01, 0.02, 0.03, 0.12,
                                                            0.7, 0.04)


def state(rows=2, **params) -> NDArray:
    """A row a leg, `params` over the defaults on every row, nothing latched."""
    s = np.zeros(rows, CAPTURE)
    defaults = dict(zip(PARAMS, (MARGIN, GAIN, DEAD, CROSS, SWAP, FROM_U, LATCH_U, CATCH)))
    for name, value in dict(defaults, **params).items():
        s[name] = value
    return s


def landing(s: NDArray, u) -> tuple:
    """u: q (the leg's phase), sign (1 her left), xi, standing (the other foot's x), sep, rate
    (strides/s), omega - a row a leg. (x the lateral to land at, swapping, catch, off - what
    the capture point is off the walk's course, m, out toward the swinging foot) a row."""
    q, sign, xi, standing, sep, rate, omega = (np.asarray(v, float) for v in u)
    swinging = q >= TOE_OFF
    prog = np.where(swinging, (q - TOE_OFF) / (1.0 - TOE_OFF), 1.0)
    tau = (1.0 - prog) * (1.0 - TOE_OFF) / np.maximum(rate, 0.1)
    nom = np.interp(prog, U_NOM, XI_NOM)
    off = sign * (xi - standing) - nom
    e = np.sign(off) * np.maximum(0.0, np.abs(off) - s['dead'])
    held = np.clip(e, -(s['margin'] + nom), np.maximum(0.0, s['margin'] - nom))
    across = sep + s['gain'] * (held + (e - held) * np.exp(omega * tau))
    fresh = prog < s['latch_u']
    s['swapping'] = np.where(prog < s['from_u'], 0.0, s['swapping'])
    ask = (across < -s['swap']) & swinging & (prog >= s['from_u']) & (prog < 1.0 - s['from_u'])
    s['swapping'] = np.where(ask, 1.0, s['swapping'])
    across = np.maximum(across, s['cross'])
    keep = (s['latched'] == 1.0) & ~fresh
    s['x'] = np.where(keep, s['x'], standing + sign * across)
    s['latched'] = np.where(fresh, 0.0, 1.0)
    catch = (s['swapping'] == 1.0) | (np.abs(across - sep) > s['catch'])
    return s['x'].copy(), s['swapping'] == 1.0, catch, off
