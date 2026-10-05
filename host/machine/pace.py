"""Her pace on the one law, for the director: taken up from her stand, a row of her way asked.

    director.pace = 0.0                  # asked on: the walk's row (`gaits.between`)
    pace.take(director, bus, out)        # standing still: the law has her from here
    out = pace.step(director, dt)        # every pass of it

`director.pace` is the row asked of her way - -1 her stand, 0 the walk's, 0.5 her jog, 1 the
run's -, None the law off and the walker hers. The row she goes on follows it no faster than
she does (`gaits.toward`). Standing settled on both feet, the law (`machine.going`) takes her
where she is: its legs standing where hers are, the pelvis at its height, every setpoint eased
from the one before. Down, the director's fall and get-up are hers as ever, and risen she stands
for the law again.
"""
import math

from machine import figure, gait, gaits, going, walkplan

#: The law takes her SETTLED_S into a standing keyframe; its setpoints ease in over BLEND_S.
SETTLED_S, BLEND_S = 0.3, 0.3


def settled(director):
    """Whether she stands SETTLED_S into a standing keyframe of the arrival's."""
    a, t = director.arrival, director.arrival.t
    for stage, span, _frame in a.frames[1:]:
        if t <= span:
            return stage == 'stand' and t >= SETTLED_S
        t -= span
    return a.frames[-1][0] == 'stand'


def stage(director):
    """Her stage for the page: the director's, on the law the gait of the row she goes on."""
    if director.stage != 'go':
        return director.stage
    k = director.k[0]
    return 'stand' if k <= -1.0 else 'walk' if k <= 0.25 else 'jog' if k <= 0.5 else 'run'


def take(director, bus, out):
    """The law taking her as she stands on both feet, her setpoints `out` before it."""
    law = director.going = going.Going(director.machine, gaits.STAND)
    pel, now = director._pelvis(bus)
    law.heading = director.walker.heading
    for side, sign in walkplan.SIDES:
        angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in figure.LEG)
        ankle, _foot = figure.foot_of(sign, pel, now, angles)
        law.legs[side] = {'stands': True, 't': 1.0, 'flat': (ankle[0], gait.ANKLE_H, ankle[2]),
                          'landed': 0.0, 'off': None,
                          'was': tuple(math.radians(out[side + k]) for k in figure.LEG)}
    law.y = {'t': 1.0, 'span': 1.0, 'landed': pel[1], 'rate': 0.0,
             'from': (pel[1], 0.0, 0.0), 'to': (pel[1], 0.0, 0.0)}
    law.last, law.pace = dict(out), 0.0
    director.stage, director.k, director.blend, director.age = 'go', (-1.0, 0.0), dict(out), 0.0


def step(director, dt):
    """{joint: degrees} of her going on the law this pass, eased in from the director's last."""
    d = director
    k, on = d.k
    d.k = gaits.toward(k, on, -1.0 if d.pace is None else d.pace, dt)
    d.going.ask = gaits.between(d.k[0])
    out = d.going.step(dt)
    if d.blend is not None:
        d.age += dt
        k = gait.eased(d.age / BLEND_S)
        out = {j: d.blend.get(j, v) + (v - d.blend.get(j, v)) * k for j, v in out.items()}
        if d.age >= BLEND_S:
            d.blend = None
    return out
