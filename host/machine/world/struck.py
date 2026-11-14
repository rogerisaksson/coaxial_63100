"""What of her touches what is not hers, the world's truth: what struck, how hard, how fast.

    struck.touching(world)        # {her body's name: [N on it, the fastest it closes on what it touches, m/s]}

Her world's contacts between a body of hers (her pelvis's tree) and one not hers, each its normal force
(`mj_contactForce`) and its point's speed into the other (`mj_objectVelocity`, the body's turn about its centre
of mass), summed and the fastest kept a body of hers - as no sense of hers tells it: the landing analyzer's
(`machine.analyzers.landing`) and the fall probe's (`tools/sim/falls.py`), introspect's `ground` layer.
"""
import functools


@functools.cache
def _hers(m):
    """(her root's id, each body's name) of the world's model `m`."""
    return int(m.body_rootid[m.body('pelvis').id]), tuple(m.body(k).name for k in range(m.nbody))


def touching(world):
    """{her body's name: [the normal force on it, N; the fastest any point of it closes on what it touches,
    m/s]} this pass, of `world` (`physics.World`): 121 us with her two soles down, twenty contacts."""
    import numpy as np
    m, d, mj = world.model, world.data, world._mj
    root, names = _hers(m)
    con, f6 = d.contact, np.zeros(6)
    b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
    h1 = m.body_rootid[b1] == root
    at = np.flatnonzero(h1 != (m.body_rootid[b2] == root))
    if not len(at):
        return {}
    mine = np.where(h1, b1, b2)[at]
    bodies, which = np.unique(mine, return_inverse=True)
    w = np.empty((len(bodies), 6))
    for k, b in enumerate(bodies):
        mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_BODY, int(b), w[k], 0)
    w = w[which]
    v = w[:, 3:] + np.cross(w[:, :3], con.pos[at] - d.xipos[mine])
    closing = np.where(h1[at], 1.0, -1.0) * np.einsum('ij,ij->i', v, con.frame[at, :3])
    normal = np.empty(len(at))
    for k, i in enumerate(at):
        mj.mj_contactForce(m, d, int(i), f6)
        normal[k] = f6[0]
    force, fastest = np.zeros(len(bodies)), np.full(len(bodies), -np.inf)
    np.add.at(force, which, normal)
    np.maximum.at(fastest, which, closing)
    return {names[b]: [float(f), float(c)] for b, f, c in zip(bodies, force, fastest)}
