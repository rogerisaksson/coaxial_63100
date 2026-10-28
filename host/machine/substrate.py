"""What she stands on when it gives: a contact's work loop, a patch's area, a power law, a bed.

    w_in, w_out, eta = substrate.loop(fz, zdot, dt)          # J, J, 1: a contact's compression and rebound work
    plastic, elastic, harmonic = substrate.regime(eta, ceiling, floor)   # 1 at its regime
    zeta = substrate.damping(eta)                            # 1: a linear contact's damping ratio returning eta
    t = substrate.stroke(m, k)                               # s: a mass's compression stroke on a spring
    area = substrate.footprint(a, b, r, step)                # m^2: a union of flat capsules, rastered
    k = substrate.tangent(force, sink, n)                    # N/m: a power law's d F / d z at that load and sinkage
    theta = substrate.phase(z, zdot, omega)                  # rad: an oscillating bed's phase, its bottom pi
    z, zdot = substrate.bed(z, zdot, force, mass, k, d, dt)  # a bed's deflection a tick on

The kernel's deformable texts (lisp/humanoid/kernel/deformable_*.lisp). z up, a bed's or a substrate's surface from
where it was touched; a force on it positive down. Flat arrays and arithmetic: no branch, no loop, no dict.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np


def loop(fz, zdot, dt):
    """(J into the substrate as it is pressed down, J back as it rises, their ratio) of normal force `fz` (..., n) N
    and surface speed `zdot` (..., n) m/s a tick `dt` apart (`evaluate-constitutive-hysteresis`'s eta_rest)."""
    w_in = dt * (fz * np.clip(-zdot, 0.0, None)).sum(axis=-1)
    w_out = dt * (fz * np.clip(zdot, 0.0, None)).sum(axis=-1)
    return w_in, w_out, w_out / w_in


def regime(eta, ceiling, floor):
    """(3, ...): 1 at the regime of energy return `eta` - plastic under `ceiling`, harmonic over `floor`,
    viscoelastic between, both ends its (`identify-rheological-work-loop`)."""
    eta = np.asarray(eta, float)
    plastic, harmonic = np.heaviside(ceiling - eta, 0.0), np.heaviside(eta - floor, 0.0)
    return np.stack([plastic, 1.0 - plastic - harmonic, harmonic])


def damping(eta):
    """The damping ratio of a linear spring-damper contact that returns `eta` of an impact's energy: eta =
    exp(-2 pi zeta / sqrt(1 - zeta^2)) inverted."""
    lost = -np.log(eta)
    return lost / np.sqrt(4.0 * np.pi * np.pi + lost * lost)


def stroke(m, k):
    """s: mass `m` pressing a spring `k` from its touch to its deepest, a quarter of its period."""
    return 0.5 * np.pi * np.sqrt(m / k)


def footprint(a, b, r, step):
    """m^2: the area of flat capsules from `a` to `b` (n, 2) m, `r` (n,) m round, as a raster of `step` m: a disc
    where its ends meet."""
    lo, hi = np.minimum(a, b).min(axis=0) - r.max(), np.maximum(a, b).max(axis=0) + r.max()
    x, y = np.meshgrid(np.arange(lo[0], hi[0], step), np.arange(lo[1], hi[1], step))
    p = np.stack([x.ravel(), y.ravel()], axis=1)[:, None, :]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(axis=2) / np.maximum((ab * ab).sum(axis=1), 1e-30), 0.0, 1.0)
    near = np.linalg.norm(p - a - t[:, :, None] * ab, axis=2)
    return (near <= r).any(axis=1).sum() * step * step


def tangent(force, sink, n):
    """N/m: d F / d z of a power law F ~ z^n (Bekker's sigma = (k_c / b + k_phi) z^n over its patch) at `force` N
    and `sink` m: n F / z, whatever its moduli."""
    return n * force / sink


def phase(z, zdot, omega):
    """rad: a bed's phase oscillating at `omega` rad/s, atan2(zdot / omega, z) (`couple-port-hamiltonian-to-membrane`'s
    theta_mem): its top 0, its bottom pi."""
    return np.arctan2(zdot / omega, z)


def bed(z, zdot, force, mass, k, d, dt):
    """(z, zdot) of a bed - `mass` kg on a spring `k` N/m and a damper `d` N s/m - a tick `dt` on, `force` N pressing
    it down: semi-implicit Euler."""
    zdot = zdot - dt * (k * z + d * zdot + force) / mass
    return z + dt * zdot, zdot
