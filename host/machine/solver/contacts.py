"""Compliant penalty contact impedance solved from mass, gravity, sag and step stability.

    k, c = contacts.penalty(mass, gravity, sag_m, zeta, dt, points)
    d = contacts.touch(mass, gravity, k, points)

A dynamic trajectory or contact solver models a floor or support by penalty springs `k` (N/m) and
dampers `c` (N s/m) at `points` parallel bearing points: `k` sets the static sag `m g / k`, bounded
above by symplectic Euler stability `4 m / (points dt^2)` over a rollout step `dt`, and `c` damps
the supported mass at ratio `zeta` across the bearing points (`points c = 2 zeta sqrt(m points k)`).
"""
import math


def penalty(mass, gravity, sag_m, zeta, dt=0.0, points=1):
    """`(stiffness N/m, damping N s/m)` per contact point for `mass` under `gravity` sagging `sag_m`,
    damped at system ratio `zeta` across `points` parallel contacts, under `dt`'s step ceiling."""
    ceiling = math.inf if dt <= 0.0 else 4.0 * mass / (points * dt * dt)
    kp = min(mass * gravity / sag_m, ceiling)
    kd = 2.0 * zeta * math.sqrt(mass * kp / points)
    return kp, kd


def touch(mass, gravity, stiffness, points=1.0):
    """Static contact deflection, m, of `mass` under `gravity` shared over `points` contacts of `stiffness`."""
    return mass * gravity / (points * stiffness)
