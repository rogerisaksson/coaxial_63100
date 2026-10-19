"""Her Newton-Euler regressor: tau = Y(q, v, a) theta, her drives' torques at N states of her model.

Numpy's, its definition, on `sysid.laid`'s layout, and compiled from it to C bit for bit
(`tools.cores.held`), no port between them.

    xpos, xmat, axis, anchor = regressor.posed(lay, body, qpos)     # her frames, MuJoCo's
    Y = regressor.regressor(lay, body, qpos, qvel, qacc, a0, wacc)  # numpy
    Y = regressor.in_c(lay, body, qpos, qvel, qacc, a0, wacc)       # its C, what sysid calls
"""


def posed(lay, body, qpos):
    """(xpos (N, bodies, 3), xmat (N, bodies, 3, 3), axis (N, hinges, 3), anchor (N, hinges, 3))
    her links' frames and her hinges' axes, world, at `qpos` (N, nq)."""
    import numpy as np
    m, d, mj = body.m, body.d, body.mj
    bodies, jnt, n = lay['bodies'], lay['jnt'], len(qpos)
    xpos, xmat = np.zeros((n, len(bodies), 3)), np.zeros((n, len(bodies), 3, 3))
    axis, anchor = np.zeros((n, len(jnt), 3)), np.zeros((n, len(jnt), 3))
    for k in range(n):
        d.qpos[:] = qpos[k]
        mj.mj_kinematics(m, d)
        xpos[k], xmat[k] = d.xpos[bodies], d.xmat[bodies].reshape(-1, 3, 3)
        axis[k], anchor[k] = d.xaxis[jnt], d.xanchor[jnt]
    return xpos, xmat, axis, anchor


def regressor(lay, body, qpos, qvel, qacc, a0, wacc=None, frames=None):
    """Y (N, hinges + 6, P) at N states of her law's model: `qpos` (N, nq) and `qvel`, `qacc`
    (N, nv) in MuJoCo's convention, `a0` (N, 3) her pelvis's origin's acceleration less gravity,
    world; every hinge's row, as its torque would be in a world without gravity, then her
    pelvis's six: the force on her, world, and the moment about her pelvis's origin in its
    frame - what the floor and whatever else touches her must supply. `wacc` (N, driven) each
    motor's acceleration through its gearbox, the armature's column: the joint's own without."""
    import numpy as np
    hinge, parent, span, above = lay['hinge'], lay['parent'], lay['span'], lay['above']
    n, nb, nh = len(qpos), len(lay['bodies']), len(hinge)
    xpos, xmat, axis, anchor = posed(lay, body, qpos) if frames is None else frames
    R, p = xmat[:, 0], xpos[:, 0]
    w = np.einsum('nij,nj->ni', R, qvel[:, 3:6])
    al = np.einsum('nij,nj->ni', R, qacc[:, 3:6])
    vp = qvel[:, 0:3]
    # Plucker at the world's origin: (w, u) a body's point there and (al, a) its rate; each
    # hinge's axis (sw, su).
    V, A = np.zeros((nb, 2, n, 3)), np.zeros((nb, 2, n, 3))
    V[0], A[0] = (w, vp - np.cross(w, p)), (al, a0 - np.cross(w, vp) - np.cross(al, p))
    sw, su = axis, np.cross(anchor, axis)
    for k in range(1, nb):
        (wv, uv), (wa, ua) = V[parent[k]], A[parent[k]]
        for h in range(span[k, 0], span[k, 1]):
            dof = hinge[h]
            qd, qdd = qvel[:, dof:dof + 1], qacc[:, dof:dof + 1]
            dsw, dsu = np.cross(wv, sw[:, h]), np.cross(wv, su[:, h]) + np.cross(uv, sw[:, h])
            wa, ua = wa + sw[:, h] * qdd + dsw * qd, ua + su[:, h] * qdd + dsu * qd
            wv, uv = wv + sw[:, h] * qd, uv + su[:, h] * qd
        V[k], A[k] = (wv, uv), (wa, ua)
    Y = np.zeros((n, nh + 6, len(lay['names'])))
    force, moment = slice(nh, nh + 3), slice(nh + 3, nh + 6)
    for k in range(nb):
        Rb, pb = xmat[:, k], xpos[:, k]
        (wv, uv), (wa, ua) = V[k], A[k]
        # In the body's frame at its origin: its spin, its angular and its classical acceleration.
        om = np.einsum('nji,nj->ni', Rb, wv)
        alb = np.einsum('nji,nj->ni', Rb, wa)
        ac = np.einsum('nji,nj->ni', Rb, ua + np.cross(wa, pb) + np.cross(wv, uv + np.cross(wv, pb)))
        f, nn = np.zeros((n, 3, 10)), np.zeros((n, 3, 10))
        f[:, :, 0] = ac
        for a in range(3):
            e = np.zeros(3)
            e[a] = 1.0
            f[:, :, 1 + a] = np.cross(alb, e) + np.cross(om, np.cross(om, e))
            nn[:, :, 1 + a] = np.cross(e, ac)
        nn[:, :, 4:10] = _tensor(alb) + np.cross(om[:, :, None], _tensor(om), axis=1)
        # To the world, about its origin; each hinge between the body and her pelvis takes its row.
        fw = np.einsum('nij,njc->nic', Rb, f)
        nw = np.einsum('nij,njc->nic', Rb, nn) + np.cross(pb[:, :, None], fw, axis=1)
        cols, rows_ = slice(10 * k, 10 * k + 10), np.flatnonzero(above[k])
        Y[:, force, cols] = fw
        Y[:, moment, cols] = np.einsum('nji,njc->nic', R, nw - np.cross(p[:, :, None], fw, axis=1))
        Y[:, rows_, cols] = (np.einsum('nhi,nic->nhc', sw[:, rows_], nw)
                             + np.einsum('nhi,nic->nhc', su[:, rows_], fw))
    at = lay['drive']
    for i, (h, dof) in enumerate(zip(lay['driven'], lay['dof'])):
        Y[:, h, at + 3 * i] = qacc[:, dof] if wacc is None else wacc[:, i]
        Y[:, h, at + 3 * i + 1] = qvel[:, dof]
        Y[:, h, at + 3 * i + 2] = np.sign(qvel[:, dof])
    return Y


def _tensor(w):
    """L(w) (N, 3, 6): L(w) [Ixx Ixy Ixz Iyy Iyz Izz] = I w."""
    import numpy as np
    L = np.zeros((len(w), 3, 6))
    L[:, 0, 0], L[:, 0, 1], L[:, 0, 2] = w[:, 0], w[:, 1], w[:, 2]
    L[:, 1, 1], L[:, 1, 3], L[:, 1, 4] = w[:, 0], w[:, 1], w[:, 2]
    L[:, 2, 2], L[:, 2, 4], L[:, 2, 5] = w[:, 0], w[:, 1], w[:, 2]
    return L


def in_c(lay, body, qpos, qvel, qacc, a0, wacc=None, frames=None, wait=False):
    """`regressor` compiled from its numpy (`tools.cores.held`): one state's program run over the
    states, numpy's arithmetic operation for operation, its einsums' orders read off numpy's own;
    held bit for bit as it is built, a build a layout kept. A layout not yet built is built in a
    process of its own (`wait` till it is), numpy's own meanwhile and where this host builds none -
    her bits the same either way; built in this one, its trace held her world 0.88 s."""
    import numpy as np
    arrays = [np.ascontiguousarray(f, dtype=np.float64) for f in (posed(lay, body, qpos) if frames is None else frames)]
    arrays += [np.ascontiguousarray(x, dtype=np.float64) for x in (qvel, qacc, a0) + (() if wacc is None else (wacc,))]
    run = _compiled(lay, wacc is not None, arrays, wait)
    if run is None:
        return regressor(lay, body, qpos, qvel, qacc, a0, wacc, tuple(arrays[:4]))
    return run(*arrays)[0]


#: Each layout's build, by the layout it was built for: (layout, its C or None, its build's process,
#: when one was last sought), the latest KEPT layouts' - a session lays its own. A build not begun -
#: the machine's commit short, another process building it - sought again RETRY_S on.
_BUILT: dict = {}
KEPT, RETRY_S = 8, 30.0


def _compiled(lay, given, arrays, wait):
    import os
    import sys
    import time
    from tools.cores import held
    key = (id(lay), given, tuple(a.shape[1:] for a in arrays))
    made_of = (sys.modules[__name__], lay, given)
    if key in _BUILT and _BUILT[key][0] is not lay:
        del _BUILT[key]
    if key not in _BUILT:
        while len(_BUILT) >= KEPT:
            del _BUILT[next(iter(_BUILT))]
        _BUILT[key] = (lay, held.found('regressor', made_of, key[2]), None, -RETRY_S)
    _lay, run, child, sought = _BUILT[key]
    while run is None:
        if child is not None:
            if not wait and child[0].poll() is None:
                break
            child[0].wait()
            try:
                os.remove(child[1])
            except OSError:
                pass
            child, run = None, held.found('regressor', made_of, key[2])
            break
        if not wait and time.monotonic() - sought < RETRY_S:
            break
        sought, run = time.monotonic(), held.found('regressor', made_of, key[2])
        if run is not None or held.kept('regressor', made_of, key[2]) is not None:
            break
        child = held.apart(__name__, 'build', (lay, given, [a[:held.STATES].copy() for a in arrays]),
                           held.digest(made_of, list(key[2])))
        if child is None and wait:
            time.sleep(1.0)
    _BUILT[key] = (lay, run, child, sought)
    return run


def build(lay, given, arrays):
    """A layout's regressor built and kept (`held.batched`) - a build apart's own work; where this
    host builds none, kept as none."""
    import sys
    from tools.cores import held

    def per(xpos, xmat, axis, anchor, qvel, qacc, a0, *wacc):
        import numpy as np
        return regressor(lay, None, np.zeros((len(qvel), 1)), qvel, qacc, a0, wacc[0] if wacc else None,
                         (xpos, xmat, axis, anchor))
    made_of = (sys.modules[__name__], lay, given)
    try:
        held.batched(per, arrays, range(len(arrays)), 'regressor', made_of=made_of)
    except Exception as why:    # a build apart's edge: what it raised kept as its refusal, not tried again
        print('machine.regressor: no C on this host, numpy\'s own (%s: %s)' % (type(why).__name__, why),
              file=sys.stderr)
        held.refused('regressor', made_of, [a.shape[1:] for a in arrays], '%s: %s' % (type(why).__name__, why))
