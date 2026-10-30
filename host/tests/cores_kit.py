"""What the suites of her C cores share: her poses drawn, results told apart in numpy's order."""


def apart(got, want):
    from tools.cores.orders import same
    return any(not same(got[k], want[k]) for k in want)


def configurations(b, rows, rng, poses):
    """(qpos, qvel) within the stops, the pelvis anywhere, for `poses` draws."""
    import numpy as np
    for _ in range(poses):
        q = np.zeros(b.m.nq)
        quat = rng.normal(size=4)
        q[:3], q[3:7] = rng.normal(size=3), quat / np.linalg.norm(quat)
        for k, row in enumerate(rows[1:]):
            lo, hi = row['stop'] or (-1.5, 1.5)
            q[7 + k] = rng.uniform(lo, hi)
        v = np.zeros(b.m.nv)
        v[:b.n] = rng.normal(size=b.n)
        yield q, v
