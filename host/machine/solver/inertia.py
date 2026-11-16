"""The inertia a drive sees of its machine: off the mass matrix, free or with a contact held.

    j = inertia.seen(mass, dof)             # every other coordinate free to give
    j = inertia.seen(mass, dof, held)       # ... with a contact's rows allowed no motion (planted soles)
    w = inertia.fall(gravity, j)            # rad/s it falls at where gravity tips it

No formula a joint: the machine's own mass matrix says what a drive's acceleration costs it. A torque
at one coordinate accelerates it by (M^-1)_ii while the others give way, so it sees 1 / (M^-1)_ii; a
contact that holds some motions (`held`, its Jacobian's rows) takes those ways of giving from it, and
the same drive sees more - an ankle a twentieth of a kg m^2 with its foot in the air and the whole
body over it with the sole planted. Pure functions of arrays: any machine's matrix, however laid.
"""
import math


def seen(mass, dof, held=None):
    """The inertia (kg m^2 at a hinge, kg along a slide) the drive at coordinate `dof` sees of the mass
    matrix `mass`: 1 / (M^-1)_ii, with the motions `held` (a constraint's Jacobian, rows by the
    coordinates) taken out of what may give; inf where the held motions leave the coordinate none."""
    import numpy as np
    give = np.linalg.inv(np.asarray(mass, dtype=float))
    if held is not None and len(held):
        across = np.asarray(held, dtype=float) @ give
        give = give - across.T @ np.linalg.pinv(across @ np.asarray(held, dtype=float).T) @ across
    own = float(give[dof, dof])
    return 1.0 / own if own > np.finfo(float).eps * float(np.abs(np.diag(give)).max()) else math.inf


def fall(gravity, inertia):
    """How fast, rad/s, an inertia falls where gravity tips it with the stiffness `gravity` (N m/rad, an
    inverted pendulum's m g h): the rate its upset grows at, sqrt(K / J)."""
    return math.sqrt(gravity / inertia)
