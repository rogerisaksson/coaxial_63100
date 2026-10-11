"""The quad's collective and torques shared over its four rotors, by hand or as her whole-body law.

    f = thrusts.shared(top, thrust, torque)        # the tilt kept, the collective next, the heading last
    f = thrusts.stacked(top, thrust, torque)       # the same as levels under bounds (`machine.qp.stack`)

Both give the four rotors' thrusts, N, none under nothing nor over `top` / 4. Where nothing
binds they agree to the mN; at a cap the tilt is kept and the collective gives way, alike
(`flying.STACKED`, docs/findings/quad.md).
"""
from machine import qp, quad


def stacked(top, thrust, torque):
    import numpy as np
    """The collective `thrust` and `torque`, N m about the frame's axes, shared over the
    rotors as levels of her whole-body law (`machine.qp.stack`): the tilt, the collective,
    the heading, the rotors alike, each in what the ones above leave; every rotor between
    nothing and its cap, `top` the four's at most."""
    cap, n, arm = top / 4.0, len(quad.ROTOR_AT), quad.ARM_M
    # rows of unit scale: at K_DRAG / K_THRUST the heading's row fell under the tie-break
    tilt = np.array([[-z / arm for _x, z in quad.ROTOR_AT], [x / arm for x, _z in quad.ROTOR_AT]])
    none = np.zeros((0, n))
    levels = [(tilt, np.array([torque[0], torque[2]]) / arm, none, np.zeros(0), 1.0),
              (np.ones((1, n)), np.array([thrust]), none, np.zeros(0), 1.0),
              (np.array([quad.SPIN], float), np.array([torque[1] * quad.K_THRUST / quad.K_DRAG]),
               none, np.zeros(0), 1.0),
              (np.eye(n), np.full(n, thrust / 4.0), none, np.zeros(0), 1.0)]
    f, held, _slack = qp.stack(np.zeros((0, n)), np.zeros(0),
                               np.vstack([-np.eye(n), np.eye(n)]),
                               np.concatenate([np.zeros(n), np.full(n, cap)]), levels,
                               ())
    if f is None:
        return shared(top, thrust, torque)
    return [max(0.0, min(cap, float(x))) for x in f]

def shared(top, thrust, torque):
    """The collective `thrust` and `torque` shared over the rotors by hand - an X, its
    diagonals spun alike - none under nothing nor over its top: the tilt's torque kept, the
    collective next, the heading's what they leave."""
    reach, cap = 4.0 * quad.ARM_M * quad.ARM_M, top / 4.0
    parts = [-torque[0] * z / reach + torque[2] * x / reach for x, z in quad.ROTOR_AT]
    each = max(-min(parts), min(thrust / 4.0, cap - max(parts)))
    room = max(0.0, min(min(each + part, cap - each - part) for part in parts))
    yaw = max(-room, min(room, torque[1] * quad.K_THRUST / (4.0 * quad.K_DRAG)))
    return [max(0.0, min(cap, each + part + yaw * s)) for part, s in zip(parts, quad.SPIN)]
