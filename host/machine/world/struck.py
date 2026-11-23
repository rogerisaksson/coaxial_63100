"""What of the robot touches what is not its own, the world's truth: what struck, how hard, how fast.

    struck.touching(world)        # {the robot's body's name: [N on it, the fastest it closes on what it touches, m/s]}

The robot's world's contacts between a body of its own (its pelvis's tree) and one not the
robot's own, each its normal force (`mj_contactForce`) and its point's speed into the other (`mj_objectVelocity`,
the body's turn about its centre of mass), summed and the fastest kept a body of the robot's own - as no sense of
the robot's own tells it: the landing analyzer's (`machine.analyzers.landing`) and the fall probe's
(`tools/sim/falls.py`), introspect's `ground` layer.
"""
def touching(world):
    """{the robot's body's name: [the normal force on it, N; the fastest any point of it closes on what it
    touches, m/s]} this pass, of `world` (`physics.World`): 121 us with the robot's two soles down, twenty
    contacts."""
    import numpy as np
    m, d, mj = world.model, world.data, world._mj
    # the robot's root off the world's pelvis, and the names of the bodies touching alone: cached by the world's
    # model, they kept the model of every world laid anew
    root = m.body_rootid[world.pelvis]
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
    return {m.body(int(b)).name: [float(f), float(c)] for b, f, c in zip(bodies, force, fastest)}
