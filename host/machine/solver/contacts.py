"""Compliant penalty contact stiffness, damping, MuJoCo solref/friction, and Bryson MPC weights.

    kf, cf = contacts.penalty(mass, gravity, sag_m, zeta, dt, points)
    touch_m = contacts.touch(mass, gravity, kf, points)
    t_s = contacts.solref_period(sag_m, gravity, floor_s)
    mu_t, mu_r = contacts.rolling_arms(foot_r_m, mu, rolling)
    weights = contacts.bryson_mpc(sag_m, spot_m, exc_rad, skim_m)

A soft floor contact of `points` bearing points holds a machine of `mass` kg under `gravity` m/s^2
within a static sag `sag_m`, bounded above by the explicit symplectic step ceiling `4 m / (n dt^2)`,
and damped at ratio `zeta` across the `points` dampers in parallel. Bryson's inverse-square rule
turns physical position, attitude, and clearance tolerances into predictive rollout cost weights.
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


def solref_period(sag_m, gravity, floor_s=0.0):
    """MuJoCo `solref` oscillation period `2 pi sqrt(sag_m / g)`, s, bounded below by `floor_s`."""
    return max(float(floor_s), round(math.tau * math.sqrt(float(sag_m) / float(gravity)), 2))


def rolling_arms(foot_r_m, mu, rolling=False):
    """`(torsional_m, rolling_m)` Hertzian contact friction arms from sphere/wheel radius `foot_r_m`."""
    r = float(foot_r_m)
    torsion = round((float(mu) * r / 2.25) if rolling else (r / 3.0), 3)
    return torsion, round(r / 150.0, 4)


def bryson_mpc(sag_m, spot_m, exc_rad, skim_m):
    """SE(3) MPPI cost weights `{x, y, z, speed, belly, nose, spin, reserve, touch, crash, ease,
    thrust, skim}` in `mppi.h`'s order via Bryson's inverse-square tolerance rule."""
    w_xz = round(1.0 / (float(spot_m) * float(spot_m)), 1)
    w_y = round(1.0 / (float(sag_m) * float(sag_m)), 1)
    w_att = float(round(1.0 / (float(exc_rad) * float(exc_rad))))
    sk = float(skim_m)
    w_skim = float(round(4.0 / (sk * sk), -3))
    return {
        'x': w_xz, 'y': w_y, 'z': w_xz, 'speed': 1.0,
        'belly': w_att, 'nose': w_att, 'spin': round(float(sag_m), 2),
        'reserve': round(15.0 * w_xz, 1),
        'touch': 100.0 * w_y, 'crash': 1000.0 * w_y,
        'ease': 2.0, 'thrust': 1.0, 'skim': w_skim,
    }
