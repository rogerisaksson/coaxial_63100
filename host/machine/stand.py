"""Standing: a point under each sole the support, a quick step whenever the capture point leaves it.

As on stilts (the user, 2026-10-04).

    frames = stand.needed(director, bus, dt, out)   # a step's keyframes, or None
    director.arrival.play(frames)                    # the step, then standing again

The capture point - the centre of mass plus its speed over omega (`machine.capture`) - is read
along and across; the support is the segment between the feet's points, one point once a foot
has hung unloaded HANG_S; the sole's shape is nothing to it. Out of the support by STEP_M,
DWELL_S or more after the last step, the foot bearing less steps: its point put down past the
capture point as it will be when the foot comes down - what it is out grown by e^(omega T), the
inverted pendulum's - by GAIN of that: on its own side, no nearer the other foot than CLEAR_M
across;
past the other foot, crossing over CROSS_M ahead of it; the pelvis lowered for the leg to reach
it and by what the stance foot stands above the floor; lifted LIFT_M over LIFT_S and down over
DOWN_S, her centre of mass held where it is going meanwhile and brought over the two points'
middle over STOOD_S stood again.
The feet stay where they land. Each step is the arrival's keyframes (`machine.arrival`): the
legs by IK, the centre of mass fed back through the pelvis, the stepping foot its `swing`.
"""
import math

from machine import arrival, figure, gait, stance, walkplan
from machine.figure import LEG, add, apply

#: Out of the support by STEP_M, m - past the sole's edge: at 3 cm a nudge along her way had
#: her step and fall, 51.5 % against 100 (2026-10-04) -, she steps, DWELL_S or more after the
#: last step began; the point lands GAIN of what it is out past the capture point as it will be
#: when the foot comes down (`machine.capture`'s 1.1, landed 2 x the brick's step went 31 cm
#: and tipped her over,
#: 2026-10-04); no nearer the other foot's point than CLEAR_M across; crossing over, CROSS_M
#: ahead of it; the landing within LUNGE of a leg's reach along the floor, the pelvis lowered
#: to reach it. A foot under `stance.LANDED_N` for HANG_S hangs, no support: unloaded to 31 N
#: by a shove's first 30 ms, the far foot was stepped to its own side, a no-op that cost the
#: catch (2026-10-04).
STEP_M, DWELL_S, GAIN, CLEAR_M, CROSS_M, LUNGE, HANG_S = 0.06, 0.4, 0.1, 0.16, 0.25, 0.8, 0.1

#: A step no further than STEP_MAX_M from where the foot stood: micro-steps, as many as it takes
#: (the user), none a lunge.
STEP_MAX_M = 0.1

#: A step: the foot lifted LIFT_M over LIFT_S, down over DOWN_S or at DOWN_M_S from higher;
#: stood again over STOOD_S, the landed leg eased into stance over it.
LIFT_M, LIFT_S, DOWN_S, DOWN_M_S, STOOD_S = 0.04, 0.1, 0.12, 0.6, 0.3

#: The sole's centre, m ahead of the ankle: the point a foot stands on.
POINT_Z = arrival.FEET_Z


def _nearest(xi, a, b):
    """The point of the segment a-b (x, z) nearest `xi`."""
    dx, dz = b[0] - a[0], b[1] - a[1]
    span = dx * dx + dz * dz
    u = max(0.0, min(1.0, ((xi[0] - a[0]) * dx + (xi[1] - a[1]) * dz) / span)) if span else 0.0
    return (a[0] + u * dx, a[1] + u * dz)


def _view(director, bus):
    """(pelvis, turn, heading rad, {side: (ankle, point (x, z), load N)}) world, as the loop
    read it: each foot's ankle and the point it stands on."""
    pel, turn = director._pelvis(bus)
    feet = {}
    for side, sign in walkplan.SIDES:
        angles = tuple(math.radians(bus.get(side + k + '.deg', 0.0)) for k in LEG)
        ankle, foot = figure.foot_of(sign, pel, turn, angles)
        point = add(ankle, apply(foot, (0.0, -gait.ANKLE_H, POINT_Z)))
        feet[side] = (ankle, (point[0], point[2]), bus['pelvis.pose.%s_load' % side])
    return pel, turn, math.atan2(turn[0][2], turn[2][2]), feet


def out(director, bus, dt=0.0):
    """(the capture point (x, z), the point of the support nearest it, how far out it is, m -
    nan with both feet hanging -, {side: (ankle, point, load)}, the heading rad, the pelvis,
    its turn); `dt` on, each foot's hang timed."""
    pel, turn, h, feet = _view(director, bus)
    for side, _sign in walkplan.SIDES:
        director.hang[side] = (director.hang.get(side, 0.0) + dt
                               if feet[side][2] < stance.LANDED_N else 0.0)
    points = [feet[side][1] for side, _sign in walkplan.SIDES
              if director.hang[side] < HANG_S]
    omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
    v = director.arrival.v
    xi = (bus['pelvis.pose.com_x'] + v[0] / omega, bus['pelvis.pose.com_z'] + v[1] / omega)
    p = _nearest(xi, points[0], points[-1]) if points else xi
    return xi, p, math.dist(xi, p) if points else math.nan, feet, h, pel, turn


def _reached(hip, ankle):
    """(`ankle` within LUNGE of a leg's reach of `hip` along the floor, how far the pelvis must
    come down for the leg to reach it, m)."""
    dx, dz = ankle[0] - hip[0], ankle[2] - hip[2]
    flat = math.hypot(dx, dz)
    k = min(1.0, LUNGE * gait.REACH / flat) if flat else 1.0
    ankle = (hip[0] + dx * k, ankle[1], hip[2] + dz * k)
    up = math.sqrt(gait.REACH ** 2 - (flat * k) ** 2)
    return ankle, max(0.0, hip[1] - ankle[1] - up)


def needed(director, bus, dt, out_):
    """A step's keyframes if one is due now, else None; `out_` is this pass's setpoints."""
    director.calm += dt
    xi, p, off, feet, h, pel, turn = out(director, bus, dt)
    if not off > STEP_M or director.calm < DWELL_S:
        return None
    director.calm = 0.0
    ahead, left = (math.sin(h), math.cos(h)), (math.cos(h), -math.sin(h))
    side, sign = min(walkplan.SIDES, key=lambda ss: feet[ss[0]][2])
    other = feet[walkplan._OTHER[side]][1]
    omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
    was = feet[side][0]
    grown = (1.0 + GAIN) * math.exp(omega * (LIFT_S + max(
        DOWN_S, (max(was[1], gait.ANKLE_H) + LIFT_M - gait.ANKLE_H) / DOWN_M_S)))
    land = (p[0] + grown * (xi[0] - p[0]), p[1] + grown * (xi[1] - p[1]))
    across = (land[0] - other[0]) * left[0] + (land[1] - other[1]) * left[1]
    along = (land[0] - other[0]) * ahead[0] + (land[1] - other[1]) * ahead[1]
    if sign * ((xi[0] - other[0]) * left[0] + (xi[1] - other[1]) * left[1]) < 0.0:
        along = max(along, CROSS_M)
    else:
        across = sign * max(sign * across, CLEAR_M)
    land = (other[0] + across * left[0] + along * ahead[0],
            other[1] + across * left[1] + along * ahead[1])
    mine = feet[side][1]
    far = math.dist(land, mine)
    if far > STEP_MAX_M:
        land = (mine[0] + (land[0] - mine[0]) * STEP_MAX_M / far,
                mine[1] + (land[1] - mine[1]) * STEP_MAX_M / far)
    ankle, lunge = _reached(figure.hip(sign, pel, turn), (
        land[0] - POINT_Z * ahead[0], gait.ANKLE_H, land[1] - POINT_Z * ahead[1]))
    now = dict(director._now(bus, out_), swing=sign)
    # Lowered for the reach now, for a stance foot above the floor only once stood again: the
    # pelvis dropped 6 cm on the brick's leg as the other reached down, the landing bounced
    # 0-600 N and she went on over the landed foot (2026-10-04).
    base = dict(now, tilt=0.0, pelvis=(pel[0], pel[1] - lunge, pel[2]))
    drop = max(lunge, feet[walkplan._OTHER[side]][0][1] - gait.ANKLE_H)
    mid = ((was[0] + ankle[0]) / 2.0, max(was[1], ankle[1]) + LIFT_M, (was[2] + ankle[2]) / 2.0)
    v = director.arrival.v
    com = (bus['pelvis.pose.com_x'] + v[0] * (LIFT_S + DOWN_S),
           bus['pelvis.pose.com_z'] + v[1] * (LIFT_S + DOWN_S))
    down = dict(base, **{side: (ankle, 0.0)})
    stood = arrival.over(dict(down, swing=0.0, pelvis=(pel[0], pel[1] - drop, pel[2])),
                         (other[0] + land[0]) / 2.0,
                         (other[1] + land[1]) / 2.0)
    down_s = max(DOWN_S, (mid[1] - ankle[1]) / DOWN_M_S)
    return [('tread', 0.0, now),
            ('tread', LIFT_S, arrival.over(dict(base, **{side: (mid, 0.0)}), *com)),
            ('tread', down_s, arrival.over(down, *com)),
            ('stand', STOOD_S, stood), ('stand', math.inf, stood)]
