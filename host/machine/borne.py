"""What bears her off her soles as her skin feels it: her law's support, her step's contacts.

    prop, contacts = borne.read(b, s, y, weight)    # `wbc.sense`'s s, `machine.edge`'s y

Each segment above her soles pushed up (`edge.TOUCH`) at its touch's centre (`edge.AT`), placed
by her own model as her estimate has her - a seat, a bed, the floor under a knee or a hand -:
prop, the share of her weight it bears and the box its points span, world (share, low (3,),
high (3,)); contacts, each such point, its force now (3,), its Jacobian (3, n) and that's
drift (3,), whose force the step finds (`machine.wrench`). Its sensed force fed to the step
instead, a stiff seat's a pass late, the step cancelled the seat's stiffness a pass after it
pushed - 695, 250, 88, 539, 131 N on it 30 ms apart, her soles' loads swinging 0 to 286 N.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import edge, touch

#: A segment bears that is pushed up past this, N: her skin's touch.
BEARS_N = edge.TOUCHED_N


def read(b, s, y, weight):
    """(prop, contacts) of `y`'s touch above her soles: her model `b`'s data posed as `wbc.sense`
    left it for `s`, `weight` hers, N."""
    m, d, n = b.m, b.d, b.n
    ids = getattr(b, 'skin', None)
    if ids is None:
        ids = b.skin = np.array([m.body(name).id for name in edge.SKIN])
    pushes, at = y[edge.TOUCH].reshape(-1, 3), y[edge.AT].reshape(-1, 3)
    up, low, high, contacts = 0.0, np.full(3, np.inf), np.full(3, -np.inf), []
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for k in touch.ABOVE:
        if not pushes[k].any():
            continue
        R = d.xmat[ids[k]].reshape(3, 3)
        f, p = R @ pushes[k], d.xpos[ids[k]] + R @ at[k]
        if f[1] <= BEARS_N:
            continue
        up += f[1]
        low, high = np.minimum(low, p), np.maximum(high, p)
        b.mj.mj_jac(m, d, jp, jr, p, ids[k])
        J = jp[:, :n].copy()
        b.mj.mj_jacDot(m, d, jp, jr, p, ids[k])
        contacts.append({'p': p, 'f': f, 'J': J, 'dJv': jp[:, :n] @ s['v']})
    held = up > 0.0
    return (up / weight, np.where(held, low, 0.0), np.where(held, high, 0.0)), contacts
