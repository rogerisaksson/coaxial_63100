"""Her limbs on all fours: a torus clock, a truss, a limb bent to a length.

    lag, duty, bpm, k = fours.clock_of(g)              # a gait's torus clock as the kernel's texts lay it
    phi = fours.clock(phi, lag, hz, k, dt)             # (n,) rad: each limb turned, pulled to its lag
    lag = fours.toward(lag, to, s)                     # a morph's lags, s of the way on the torus
    on = fours.bearing(phi, duty)                      # (n,) 1 while a limb stands
    lags = fours.fifth(lag, duty, together)            # (n + 1,): her limbs', then the axial patch's
    f = fours.truss(points, wrench, metric, told)      # (n, 3) N: the wrench met, each edge squeezed as told
    kappa = fours.bend(upper, lower, length)           # rad: a two-link limb's middle joint at that length

The kernel's multimodal texts (machine/ortho_kernel/multimodal_*.lisp): `configure-4torus-symplectic-clock`,
`couple-fifth-limb-axial-peristalsis`, `project-virtual-truss-internal-wrenches`,
`equalize-heterogeneous-limb-kinematics`. A limb's phase is 0 as it lands and it stands for the first `duty` of a
turn; its lag, rad, how long after the first limb it lands - a text's offset [LH LF RH RF], of a turn. Flat arrays
and arithmetic: no branch, no loop, no dict.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

TAU = 2.0 * np.pi


def clock_of(g):
    """(lags (4,) rad, duty, bpm, coupling 1/s) of gait `g`'s torus clock (`bind-coupled-torus-oscillators`)."""
    at = g['torus-clock']
    return (TAU * np.array([c.args['phase-offset'] for c in at['limb-channels']]), at['duty-cycle'], at['bpm'],
            at['sync-stiffness-k'])


def clock(phi, lag, hz, k, dt):
    """`phi` (n,) rad a tick `dt` on: each limb turned at `hz` and pulled by `k` (1/s) toward lagging the first
    by its `lag` (n,) - phi_a' = 2 pi hz + k sum_b sin(phi_b - phi_a + lag_b - lag_a)."""
    pull = np.sin(phi[None, :] - phi[:, None] + lag[None, :] - lag[:, None]).sum(axis=1)
    return (phi + (TAU * hz + k * pull) * dt) % TAU


def toward(lag, to, s):
    """Lags `s` of the way from `lag` to `to` along the torus's shortest arc: taken as numbers, the trot's
    [0 .5 .5 0] to the crawl's [0 .25 .5 .75] turns the right fore three quarters and one limb bears alone."""
    return lag + s * np.angle(np.exp(1j * (to - lag)))


def bearing(phi, duty):
    """(n,) 1.0 for a limb in its stance, the first `duty` of its turn."""
    return (phi % TAU < TAU * duty).astype(float)


def fifth(lag, duty, together):
    """(n + 1,) lags: her limbs' drawn to her first's by `together` (1: advanced as one), and the axial patch's,
    landing as her first limb lifts - antiphase to their swing (`splice-fifth-peristaltic-oscillator`)."""
    limbs = lag + together * (lag[0] - lag)
    return np.append(limbs, limbs[0] + TAU * duty)


def held(com, newtons, up):
    """(6,) what her contacts give to hold `newtons` at `com` against gravity along -`up`: force, moment about the
    origin."""
    force = newtons * np.asarray(up, float)
    return np.concatenate([force, np.cross(com, force)])


def grasp(points):
    """(6, 3n): forces at `points` (n, 3) to their force and moment about the origin."""
    n = len(points)
    g = np.zeros((6, 3 * n))
    g[:3] = np.tile(np.eye(3), n)
    g[3:] = np.cross(points[:, None, :], np.eye(3)[None]).transpose(2, 0, 1).reshape(3, 3 * n)
    return g


def squeezes(points):
    """(m, 3n): each edge's squeeze, N, of forces at `points` (n, 3), an edge a pair a < b (`np.triu_indices`):
    half the two forces' difference along it from a to b - its contacts pressed apart, the floor's reactions
    pushing them together."""
    n = len(points)
    a, b = np.triu_indices(n, 1)
    e = points[b] - points[a]
    e = e / np.linalg.norm(e, axis=1)[:, None]
    out = np.zeros((len(a), n, 3))
    out[np.arange(len(a)), a] = 0.5 * e
    out[np.arange(len(a)), b] = -0.5 * e
    return out.reshape(len(a), 3 * n)


def truss(points, wrench, metric, told):
    """(n, 3) N: forces at `points` (n, 3) meeting `wrench` (`held`), each edge squeezed (`squeezes`) `told` (m,)
    as near as the points allow, the rest the least of sum_a metric_a |f_a|^2 - the kernel's virtual truss on her
    contacts' costs (`modulate-nullspace-inertia-metric`)."""
    root = np.repeat(1.0 / np.sqrt(metric), 3)
    a, e = grasp(points) * root, squeezes(points) * root
    inv = np.linalg.pinv(a)
    x = inv @ wrench
    free = np.eye(len(root)) - inv @ a
    x = x + free @ np.linalg.pinv(e @ free) @ (told - e @ x)
    return (root * x).reshape(-1, 3)


def bend(upper, lower, length):
    """A two-link limb's middle joint's flexion, rad (0 straight), that sets its ends `length` apart: the law of
    cosines, clipped to straight and to folded flat."""
    c = (length * length - upper * upper - lower * lower) / (2.0 * upper * lower)
    return np.arccos(np.clip(c, -1.0, 1.0))
