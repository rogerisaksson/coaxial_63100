"""The robot's law's sense of it at its lay (wbc.sense), as its sensors and its estimate have it."""
from machine.senses import estimate


def sensed(body, est, sense, limbs):
    """`wbc.sense`'s view of the robot at its lay, as its sensors and its estimate have it."""
    from machine.stack import wbc
    qpos, qvel = estimate.step(est, body, sense, 0.0)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'], s['elbows'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']], s['q'][limbs['elbows']]
    return s
