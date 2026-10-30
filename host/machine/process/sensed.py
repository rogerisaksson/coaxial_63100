"""Her law's sense of her at her lay (wbc.sense), as her sensors and her estimate have her."""
from machine.senses import estimate


def sensed(body, est, sense, limbs):
    """`wbc.sense`'s view of her at her lay, as her sensors and her estimate have her."""
    from machine.stack import wbc
    qpos, qvel = estimate.step(est, body, sense, 0.0)
    s = wbc.sense(body, qpos, qvel)
    s['hips'], s['arms'], s['elbows'] = body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']], s['q'][limbs['elbows']]
    return s
