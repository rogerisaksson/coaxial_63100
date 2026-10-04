"""Standing: a point under each sole the support, a quick step whenever the capture point leaves.

As on stilts (the user, 2026-10-04), by one law in the capture point's plane (`machine.dcm`).

    frames = stand.needed(director, bus, dt, out)   # a step's keyframes, or None
    director.arrival.play(frames)                    # the step, then standing again
    stand.retarget(director, bus)                    # every pass of it: the step aimed anew

The capture point - the centre of mass plus its speed over omega (`machine.capture`) - is read
along and across; the support is the segment between the feet's points, one point once a foot
has hung unloaded HANG_S; the sole's shape is nothing to it. `dcm.due` says when a step is
due, `dcm.landing` where the foot bearing less lands, the other standing; within LUNGE of its
leg's reach: lifted LIFT_M over
LIFT_S and down over DOWN_S, her centre of mass held where it is going meanwhile and brought
over the two points' middle over STOOD_S stood again. The feet stay where they land. Each step
is the arrival's keyframes (`machine.arrival`): the legs by IK, the centre of mass fed back
through the pelvis, the stepping foot its `swing`, landed where it bears as it comes down
(`land`).
"""
import math

from machine import arrival, dcm, figure, gait, stance, walkplan
from machine.figure import LEG, add, apply

#: The landing within LUNGE of a leg's reach along the floor, the pelvis lowered to reach it. A
#: foot bearing under `stance.LANDED_N` for HANG_S where the centre of mass asks HANG_SHARE of
#: her weight of it hangs, no support: by its load alone, unloaded to 31 N by a shove's first
#: 30 ms, the far foot was no support and she stepped inward for nothing (2026-10-04).
LUNGE, HANG_S, HANG_SHARE = 0.8, 0.15, 0.45

#: A step in flight is aimed anew every pass until FREEZE_S before it comes down: called 30 ms
#: into a 120 ms push it landed 5 cm short of the capture point, and the next was called as it
#: touched (100 N from behind, 2026-10-04).
FREEZE_S = 0.06

#: A step: the foot lifted LIFT_M over LIFT_S, down over DOWN_S or at DOWN_M_S from higher;
#: stood again over STOOD_S, the landed leg eased into stance over it. Stood 4 cm lower after
#: a step the hop stayed, 0-1114 N a foot (2026-10-04).
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
    a, b = feet['left'][1], feet['right'][1]
    com = (bus['pelvis.pose.com_x'], bus['pelvis.pose.com_z'])
    q = _nearest(com, a, b)
    u = math.dist(q, a) / max(1e-6, math.dist(a, b))
    for side, share in (('left', 1.0 - u), ('right', u)):
        director.hang[side] = (director.hang.get(side, 0.0) + dt if feet[side][2] < stance.LANDED_N
                               and share >= HANG_SHARE else 0.0)
    points = [feet[side][1] for side, _sign in walkplan.SIDES
              if director.hang[side] < HANG_S]
    omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
    v = director.arrival.v
    xi = (bus['pelvis.pose.com_x'] + v[0] / omega, bus['pelvis.pose.com_z'] + v[1] / omega)
    p = _nearest(xi, points[0], points[-1]) if points else xi
    return xi, p, math.dist(xi, p) if points else math.nan, feet, h, pel, turn


def _plane(p, q, h):
    """World (x, z) `p` in her plane, complex: her left real, ahead imaginary, from `q`."""
    dx, dz = p[0] - q[0], p[1] - q[1]
    return complex(dx * math.cos(h) - dz * math.sin(h), dx * math.sin(h) + dz * math.cos(h))


def _world(z, q, h):
    """Her plane's `z` back in the world, (x, z)."""
    return (q[0] + z.real * math.cos(h) + z.imag * math.sin(h),
            q[1] - z.real * math.sin(h) + z.imag * math.cos(h))


def _within(land, hip):
    """`land` (x, z) no further than LUNGE of a leg's reach from `hip` along the floor."""
    far = math.hypot(land[0] - hip[0], land[1] - hip[2])
    k = min(1.0, LUNGE * gait.REACH / far) if far else 1.0
    return (hip[0] + (land[0] - hip[0]) * k, hip[2] + (land[1] - hip[2]) * k)


def needed(director, bus, dt, out_):
    """A step's keyframes if one is due now (`dcm.step`), else None; `out_` is this pass's
    setpoints."""
    xi, q, _off, feet, h, pel, turn = out(director, bus, dt)
    omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
    if not dcm.due(_plane(xi, q, h)):
        return None
    # The foot that steps: one hanging, else the one bearing less - the other stands. Lifted
    # bearing the more, the leg drew up under her and she dropped, the pelvis rolling 27 deg
    # in 0.2 s (100 N from a side); the farther from the landing, it crossed the nearer
    # (2026-10-04).
    side, sign = min(walkplan.SIDES, key=lambda ss: (director.hang[ss[0]] < HANG_S,
                                                     feet[ss[0]][2]))
    other = feet[walkplan._OTHER[side]][1]
    at = dcm.landing(_plane(xi, q, h), _plane(other, q, h), omega, dcm.STEP_S, sign)
    if at is None:
        return None
    was = feet[side][0]
    land = _within(_world(at, q, h), figure.hip(sign, pel, turn))
    director.stepping = {'side': side, 'sign': sign, 'land': land, 'q': q, 'h': h}
    ahead = (math.sin(h), math.cos(h))
    ankle = (land[0] - POINT_Z * ahead[0], gait.ANKLE_H, land[1] - POINT_Z * ahead[1])
    now = dict(director._now(bus, out_), swing=sign)
    base = dict(now, tilt=0.0)
    # Stood again, the pelvis down by what the stance foot stands above the floor: dropped as
    # the other reached down off the brick, the landing bounced 0-600 N and she went on over
    # the landed foot (2026-10-04).
    drop = max(0.0, feet[walkplan._OTHER[side]][0][1] - gait.ANKLE_H)
    mid = ((was[0] + ankle[0]) / 2.0, max(was[1], ankle[1]) + LIFT_M, (was[2] + ankle[2]) / 2.0)
    v = director.arrival.v
    com = (bus['pelvis.pose.com_x'] + v[0] * (LIFT_S + DOWN_S),
           bus['pelvis.pose.com_z'] + v[1] * (LIFT_S + DOWN_S))
    down = dict(base, land=1.0, **{side: (ankle, 0.0)})
    stood = arrival.over(dict(down, swing=0.0, pelvis=(pel[0], pel[1] - drop, pel[2])),
                         (other[0] + land[0]) / 2.0,
                         (other[1] + land[1]) / 2.0)
    down_s = max(DOWN_S, (mid[1] - ankle[1]) / DOWN_M_S)
    return [('tread', 0.0, now),
            ('tread', LIFT_S, arrival.over(dict(base, **{side: (mid, 0.0)}), *com)),
            ('tread', down_s, arrival.over(down, *com)),
            ('stand', STOOD_S, stood), ('stand', math.inf, stood)]


def retarget(director, bus):
    """The step in flight aimed where the law has the capture point now (`dcm.landing`, the
    time left to it)."""
    a, step = director.arrival, director.stepping
    side, sign, q, h = step['side'], step['sign'], step['q'], step['h']
    left_s = sum(span for _stage, span, _frame in a.frames[1:3]) - a.t
    if left_s < FREEZE_S or side in a.pinned:
        return
    xi, _q, _off, feet, _h, pel, turn = out(director, bus)
    omega = math.sqrt(9.81 / max(0.3, bus['pelvis.pose.com_y']))
    at = dcm.landing(_plane(xi, q, h), _plane(feet[walkplan._OTHER[side]][1], q, h), omega,
                     left_s, sign)
    if at is None:
        return
    land = _within(_world(at, q, h), figure.hip(sign, pel, turn))
    dx, dz = land[0] - step['land'][0], land[1] - step['land'][1]
    step['land'] = land
    # The lifted mark half the way, the landing and the standing after it all the way, her
    # weight over the two points' middle half of it.
    stood = a.frames[3][2]
    for frame, k in ((a.frames[1][2], 0.5), (a.frames[2][2], 1.0), (stood, 1.0)):
        (x, y, z), pitch = frame[side]
        frame[side] = ((x + k * dx, y, z + k * dz), pitch)
    x, y, z = stood['pelvis']
    stood['pelvis'] = (x + dx / 2.0, y, z + dz / 2.0)
    a.coms[3:] = [(c0[0] + dx / 2.0, c0[1], c0[2] + dz / 2.0) for c0 in a.coms[3:]]
