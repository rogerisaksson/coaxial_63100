"""The inertia a drive sees of its machine, and rigid-body composite inertia from geometry.

    j = inertia.seen(mass, dof)                     # every other coordinate free to give
    j = inertia.seen(mass, dof, held)               # ... with a contact's rows allowed no motion
    w = inertia.fall(gravity, j)                    # rad/s an upset grows where gravity tips j
    part = inertia.box(mass, lx, ly, lz, center)    # (mass, center, (ixx, iyy, izz)) of a solid box
    part = inertia.tube(r, wall, L, rho, axis, c)   # ... of a thin-walled hollow circular tube
    part = inertia.disc(mass, r, axis, center)      # ... of a thin circular rotor disc
    ixx, iyy, izz = inertia.composite(parts, mass)  # parallel-axis principal moments balanced at CoM

No formula a joint: the machine's own mass matrix says what a drive's acceleration costs, and the
parallel-axis sum over elementary boxes, tubes, and discs gives a craft's principal inertias without
hand-entered radii of gyration. Pure functions of numbers and arrays.
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


def box(mass, lx, ly, lz, center=(0.0, 0.0, 0.0)):
    """`(mass, center, (ixx, iyy, izz))` of a uniform rectangular box of side lengths `(lx, ly, lz)`."""
    m, x, y, z = float(mass), float(lx), float(ly), float(lz)
    x2, y2, z2 = x * x, y * y, z * z
    return (m, (float(center[0]), float(center[1]), float(center[2])),
            (m * (y2 + z2) / 12.0, m * (x2 + z2) / 12.0, m * (x2 + y2) / 12.0))


def tube(radius, wall, length, rho_kg_m3, axis, center=(0.0, 0.0, 0.0)):
    """`(mass, center, (ixx, iyy, izz))` of a hollow circular tube along `'x'`, `'y'`, or `'z'`."""
    r, L = float(radius), float(length)
    ri = max(0.0, r - float(wall))
    r2, ri2 = r * r, ri * ri
    m = math.pi * (r2 - ri2) * L * float(rho_kg_m3)
    j_along = 0.5 * m * (r2 + ri2)
    j_perp = m * (0.25 * (r2 + ri2) + L * L / 12.0)
    return (m, (float(center[0]), float(center[1]), float(center[2])),
            tuple(j_along if a == axis else j_perp for a in 'xyz'))


def disc(mass, radius, axis='y', center=(0.0, 0.0, 0.0)):
    """`(mass, center, (ixx, iyy, izz))` of a thin circular disc of `radius` normal to `axis`."""
    m, r = float(mass), float(radius)
    j_axis = 0.5 * m * r * r
    j_diam = 0.5 * j_axis
    return (m, (float(center[0]), float(center[1]), float(center[2])),
            tuple(j_axis if a == axis else j_diam for a in 'xyz'))


def composite(parts, total_mass=None, pod_size=(0.12, 0.12, 0.18), trim_axis=2):
    """Principal `(ixx, iyy, izz)` about the composite CoM of `parts`; when `total_mass` exceeds the
    structural parts' sum, the remaining drive and pack mass sits in a central pod trimmed along
    `trim_axis` so the composite CoM lies on the main thrust line."""
    items = list(parts)
    m_struct = sum(p[0] for p in items)
    if total_mass is not None and float(total_mass) > m_struct:
        m_pod = float(total_mass) - m_struct
        c_pod = [0.0, 0.0, 0.0]
        c_pod[trim_axis] = -sum(p[0] * p[1][trim_axis] for p in items) / m_pod
        items.append(box(m_pod, pod_size[0], pod_size[1], pod_size[2], tuple(c_pod)))
    mt = sum(p[0] for p in items)
    com = tuple(sum(p[0] * p[1][a] for p in items) / mt for a in range(3))
    ixx = iyy = izz = 0.0
    for m, (x, y, z), (jx, jy, jz) in items:
        dx, dy, dz = x - com[0], y - com[1], z - com[2]
        ixx += jx + m * (dy * dy + dz * dz)
        iyy += jy + m * (dx * dx + dz * dz)
        izz += jz + m * (dx * dx + dy * dy)
    return ixx, iyy, izz
