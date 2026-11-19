"""The bus's multidrop chain marked on the robot's prints: a ring and dot in, a ring and cross out.

    tris = chain.mark('in', at, normal, up)    # raised off the face through `at` that faces `normal`

Where the bus comes into a node - a T-cross's arm, a guard's wall, the foot's box - a ring with a dot at its middle
(the arrow's point coming at the viewer); where it goes on along the chain, a ring with a cross (its tail going
away): the vector's notation, so the chain's order reads off the robot's prints, node by node, each mark where the
hose itself goes in or out. Each RING_M round, its stroke STROKE_M, PROUD_M off its face, in the robot's accent
(ACCENT): what a print's first layers take, or its last - a guard's face (`covered`), a mark -, one change of
filament a print.
"""
import math

#: A mark's ring's radius, its stroke's width, the dot's radius, its height off the face, m.
RING_M, STROKE_M, DOT_M, PROUD_M = 0.003, 0.0007, 0.0008, 0.0004
#: The robot's accent: a lime green, across the wheel from the drives' magenta.
ACCENT = (176, 226, 36)


def _square(points, at, u, v, n, ink):
    """[(tri, colour)]: the 2-d outline `points` [(a, b)] at `at` in the face's axes (u, v), raised along n."""
    import numpy as np
    flat = np.asarray(points, float)
    lo = at + flat[:, :1] * u + flat[:, 1:] * v
    hi = lo + n * PROUD_M
    tris = []
    for i in range(len(flat)):
        j = (i + 1) % len(flat)
        tris += [(np.array((lo[i], lo[j], hi[j])), ink), (np.array((lo[i], hi[j], hi[i])), ink)]
    mid_hi = hi.mean(axis=0)
    tris += [(np.array((mid_hi, hi[i], hi[(i + 1) % len(flat)])), ink) for i in range(len(flat))]
    return tris


def mark(kind, at, normal, up=(0.0, 1.0, 0.0)):
    """[(tri, colour)]: the bus's `kind` ('in' or 'out') mark on the face through `at` facing `normal`, its cross's
    arms square to `up` laid on the face."""
    import numpy as np
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    u = np.asarray(up, float) - n * float(np.dot(up, n))
    u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-9 else np.cross(n, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    at = np.asarray(at, float)
    tris = []
    for k in range(32):                                  # the ring, a stroke wide
        t0, t1 = 2.0 * math.pi * k / 32, 2.0 * math.pi * (k + 1) / 32
        quad = [(r * math.cos(t), r * math.sin(t)) for r, t in ((RING_M, t0), (RING_M, t1), (RING_M - STROKE_M, t1),
                                                                  (RING_M - STROKE_M, t0))]
        tris += _square(quad, at, u, v, n, ACCENT)
    if kind == 'in':
        tris += _square([(DOT_M * math.cos(t), DOT_M * math.sin(t)) for t in np.linspace(0.0, 2.0 * math.pi, 16,
                                                                                         endpoint=False)],
                        at, u, v, n, ACCENT)
    else:
        arm = RING_M - STROKE_M
        for t in (0.25 * math.pi, 0.75 * math.pi):       # the cross, its arms to the ring
            c, s, w = math.cos(t), math.sin(t), 0.5 * STROKE_M
            tris += _square([(arm * c - w * s, arm * s + w * c), (-arm * c - w * s, -arm * s + w * c),
                             (-arm * c + w * s, -arm * s - w * c), (arm * c + w * s, arm * s - w * c)],
                            at, u, v, n, ACCENT)
    return tris
