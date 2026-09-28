"""Where a swinging foot goes: the plan's landing, a catch put down on the capture point, a side step.

`landings(w, ..)` sets each foot not held on its target, clear of the other foot (`clear`), within
the leg's reach (`reach`), and says how far the pelvis must come down for it to reach; a catch or
a side step (`sidestep`) takes over when the capture point runs off. `w` is the `walker.Walker`.
"""
import math

from machine import capture, figure, gait, stance, walkplan


#: Where a swinging foot lands across is `machine.capture`'s: on the capture point as it will be
#: at the landing. Put down from the pelvis instead, a shove toward the standing foot took the
#: swinging one across it and she fell in 0.3 s; the capture point past the standing foot ran
#: 109 -> 425 mm in 0.42 s once the other lifted (2026-09-26).

#: A swap is a side step outside the plan's phase: the swinging foot is put down where it is,
#: flat, lowered over DOWN_S, and once it bears BEARS_N for DWELL_S the standing one steps out to
#: where the capture point will be when it lands, foreseen with no ankle to help, SIDE_AHEAD_M
#: ahead of where the pelvis will be, and there it stays - put 5 cm behind, the walk began again
#: on it in its push-off and the heel's rise threw her 3 cm up; moved on with the pelvis once
#: down, the leg could not reach it and lifted it, and the weight came over 0.3 s late
#: (2026-09-26) - lifted SIDE_LIFT_M at the middle of SIDE_S seconds and set down as softly
#: as lifted - an arc of sine struck at 0.9 m/s, 2100 N, and the leg threw her 18 cm up
#: (2026-09-26); the phase SIDE_HURRY faster meanwhile. Left to the plan, the swapped foot landed
#: 0.24 s after the shove and the other lifted 0.12 s later still, the capture point 20 cm out by
#: then; dropped at once it struck 1200 N and bounced; the other lifted on 60 N of one pass and
#: she hung in the air; the step out sized once, with the ankle's help, was 11 cm of the 20
#: needed (2026-09-26).
DOWN_S, DWELL_S, SIDE_AHEAD_M, SIDE_LIFT_M, SIDE_S, SIDE_HURRY = 0.1, 0.02, 0.1, 0.06, 0.2, 0.3


#: The swapped foot is put down DOWN_AHEAD_M ahead of the pelvis, m: where it was, the body walked
#: on past it, the leg could not reach back, and the foot left the floor (2026-09-26). One behind
#: the pelvis is put down where it is, DOWN_BEHIND_M behind at most: sent 15 cm ahead from 30
#: behind, it never came down (2026-09-27).
DOWN_AHEAD_M, DOWN_BEHIND_M = 0.15, 0.25


#: No side step in the first SIDE_AGAIN_S of a walk begun again: none until its blend was done,
#: one asked in it went the way the capture point had left (2026-09-27).
SIDE_AGAIN_S = 0.15


#: The side step is over once the stepped-out foot, within DOWN_M of the floor, bears BEARS_N
#: and more than the other; if the other still bears her SIDE_GIVE_S on, she has come back to
#: it, and the walk begins again on that one. Ended as the stepped-out foot touched, the plan
#: lifted the one that bore her; struck by the other leg 4 cm up, 2400 N ended the step with the
#: foot in the air (2026-09-26). The stepped-out foot goes out first and on from SIDE_OUT_FIRST
#: of the step: on at once, its shank struck the other leg's (2026-09-27).
SIDE_GIVE_S, DOWN_M, SIDE_OUT_FIRST = 0.4, 0.01, 0.25


#: The swapped foot is put down SIDE_CLEAR_M across from the other's line at least: put down
#: where it was, 3 cm across, the other's toes struck its heel going by, 1800 N (2026-09-27).
SIDE_CLEAR_M = 0.12


#: A catch the leg cannot reach standing is a stomp: the swinging foot put down at once on the
#: capture point and STOMP_PAST_M past it, flat, the pelvis lowered to reach, the walk begun
#: again on it once it bears - as a swap's foot is put down.
#: Left to the plan, the foot touched at its reach, lifted again, and chased the capture point
#: to 66 cm out (2026-09-27).
STOMP_PAST_M = 0.05


#: The capture point is the centre of mass plus the pelvis's sideways speed over omega, not the
#: centre of mass's own: the swinging leg's speed is in that, and a wide step's put the capture
#: point 6 cm out past where it went - the foot chased itself (2026-09-26).

#: No foot waits at toe-off for the capture point to come over the other: held there 0.3 s
#: while the legs drove the pelvis toward the standing foot, the pelvis moved 3 mm; walking on,
#: the plan froze while the body went 24 cm past the standing foot, the leg could not reach and
#: she sank 17 cm (2026-09-26).


#: A swinging foot lands on the capture point along the walk as it does across it: FORE_K of the
#: body's speed over the plan's, over omega, FORE_M at most, on from the plan's spot. Slowed to a
#: stop by a knee folding under her, the next foot came down where the plan had it, ahead of a
#: body going nowhere, and she fell backwards (2026-09-27).
FORE_K, FORE_M = 1.0, 0.25


def put_down(ball_z, pel_z):
    """Where a foot is put down on: DOWN_AHEAD_M ahead of the pelvis, or further if it is; where
    it is behind, DOWN_BEHIND_M behind at most."""
    z = ball_z - gait.BALL
    return max(z, pel_z + DOWN_AHEAD_M) if z > pel_z else max(z, pel_z - DOWN_BEHIND_M)


#: A swinging foot's sole is kept GAP_M off the other's, edge to edge across the walk, the other
#: along its own heading, wherever they overlap along the walk, eased in over OVERLAP_M of them
#: coming to: pushed out on its own side, by the walker's own kinematics; a catch, a side step, a
#: turn may pass nearer than the walk's path. Kept 90 mm off the other's ball, centre to centre,
#: the foot skimming 25 mm over the floor met the other's heel, 24 mm nearer toed out 8 degrees:
#: -8 mm, 340 N. Eased in over 30 mm, 10-15 ms at the swing's speed, the toes met the other's
#: heel's corner: over 100 mm they pass 12 mm apart (2026-09-28).
GAP_M, OVERLAP_M = 0.01, 0.1


#: A leg's plan led SWING_LEAD_S through its swing and its landing's roll: a drive lags a
#: setpoint on the move; its hip 5 degrees behind caught up into the floor (2026-09-28).
SWING_LEAD_S = 0.02


def led(q):
    """Whether a leg at phase `q` is swinging or landing, its plan led."""
    return q % 1.0 >= gait.TOE_OFF or q % 1.0 < gait.SETTLE


def clear(at, sign, other, ankle):
    """`at`, a swinging ankle's target (world), on side `sign`, clear of the other foot, its ball
    at `other` and its ankle at `ankle`: the soles' overlap along the walk from the other's heel
    to its toes."""
    dx, dz = other[0] - ankle[0], other[2] - ankle[2]
    norm = math.hypot(dx, dz) or 1.0
    dx, dz = dx / norm, dz / norm
    nx, nz = (dz, -dx) if sign * dz > 0.0 else (-dz, dx)
    w = figure.SOLE_HALF
    lo = (ankle[0] - gait.HEEL * dx + w * nx, ankle[2] - gait.HEEL * dz + w * nz)
    hi = (other[0] + figure.TOE_M * dx + w * nx, other[2] + figure.TOE_M * dz + w * nz)
    z0, z1 = max(lo[1], at[2] - gait.HEEL), min(hi[1], at[2] + gait.BALL + figure.TOE_M)
    near = max(0.0, min(1.0, (z1 - z0 + OVERLAP_M) / OVERLAP_M))
    span = hi[1] - lo[1] or 1.0
    edges = (lo[0] + (hi[0] - lo[0]) * max(0.0, min(1.0, (z - lo[1]) / span)) for z in (z0, z1))
    short = max(sign * (e - at[0]) for e in edges) + w + GAP_M
    if short <= 0.0 or near <= 0.0:
        return at
    return (at[0] + sign * short * near, at[1], at[2])


def reach(hip, at, reach):
    """`at`, an ankle's target, within `reach` of the hip: its step shortened first, then the
    ankle held up; (the target, how far the pelvis must come down for it to reach as asked)."""
    dx, dy, dz = (a - h for a, h in zip(at, hip))
    if dx * dx + dy * dy + dz * dz <= reach * reach:
        return at, 0.0
    room = reach * reach - dx * dx - dy * dy
    if room >= 0.0:
        return (at[0], at[1], hip[2] + math.copysign(math.sqrt(room), dz)), 0.0
    dx = max(-0.9 * reach, min(0.9 * reach, dx))
    low = -math.sqrt(reach * reach - dx * dx)
    return (hip[0] + dx, hip[1] + low, hip[2]), low - dy


def landings(w, dt, bus, qs, legs, balls, pel, turn_now, planned_z, spread, length, xi,
              omega, ankles):
    """({side: its ankle's target} for each foot not held, how far the pelvis must come down
    for them to reach, {side: x, where it stands or last stood}, what the capture point is
    off the walk's course a row): across on the capture point (`machine.capture`), round the
    standing foot; on, the plan's, shortened to reach; in a side step (`_sidestep`), its
    own."""
    standing = [(w.anchor[o] if o in w.anchor else balls[o])[0] for o in ('right', 'left')]
    sep = 2.0 * (walkplan.TRACK_M + spread)
    x, swapping, catch, off = capture.landing(w.capture, (
        qs, (1.0, -1.0), (xi, xi), standing, (sep, sep), (w.rate, w.rate),
        (omega, omega)))
    loads = {side: bus['pelvis.pose.%s_load' % side] for side, _sign in walkplan.SIDES}
    sidestep(w, dt, swapping, loads, balls, legs, pel, ankles)
    if w.side is not None:
        w.capture['swapping'][:] = 0.0
    w.hurry = SIDE_HURRY if w.side is not None else 0.0
    # No catch in the first strides: their landings are off the walk's own by design.
    w.catching = w.side is not None or (w.held is None and bool(catch.any()))
    out, lower, feet_x = {}, 0.0, {}
    fore = max(-FORE_M, min(FORE_M, FORE_K * (w.v_on - length * w.rate) / omega))
    for i, ((side, sign), q, (ankle, _tw, _pi, _to)) in enumerate(zip(walkplan.SIDES, qs, legs)):
        step = w.side
        if step is not None and side == step['down'] and q >= gait.TOE_OFF:
            step['down_s'] = min(DOWN_S, step['down_s'] + dt)
            x0, y0, z0 = step['at']
            at = (x0, y0 + (gait.ANKLE_H - y0) * gait.eased(step['down_s'] / DOWN_S), z0)
            at = clear(at, sign, balls[walkplan._OTHER[side]], ankles[walkplan._OTHER[side]])
            at, short = reach(figure.hip(sign, pel, turn_now), at, stance.SWING_REACH * gait.REACH)
            out[side], lower = at, max(lower, short)
            w.stood[side] = feet_x[side] = x0
            continue
        if step is not None and side == step['out'] and step['stage'] == 'out':
            # Foreseen once as it begins, with no ankle to help and no gain - on the capture
            # point, the body to come to rest over the foot. Followed as the capture point
            # ran, the foot chased the swing of its own leg's mass, 16 cm past (2026-09-26).
            if step['x_out'] is None:
                xs, _swap, _catch, _off = capture.landing(capture.state(1, margin=0.0, gain=1.0), (
                    (1.0 - SIDE_S * w.rate,), (sign,), (xi,), (balls[step['down']][0],),
                    (sep,), (w.rate,), (omega,)))
                step['x_out'] = float(xs[0])
            u = min(1.0, step['since'] / SIDE_S)
            # Eased on from where it stood: sent SIDE_AHEAD_M ahead at once while it bore
            # 1400 N, it dragged and threw her 1.5 cm up, the other foot off the floor
            # (2026-09-26).
            on = gait.eased(max(0.0, (u - SIDE_OUT_FIRST) / (1.0 - SIDE_OUT_FIRST)))
            at = (step['x_from'] + (step['x_out'] - step['x_from']) * gait.eased(u),
                  gait.ANKLE_H + SIDE_LIFT_M * math.sin(math.pi * u) ** 2,
                  step['z_from'] + (step['z_land'] - step['z_from']) * on)
            at = clear(at, sign, balls[walkplan._OTHER[side]], ankles[walkplan._OTHER[side]])
            at, short = reach(figure.hip(sign, pel, turn_now), at, stance.SWING_REACH * gait.REACH)
            out[side], lower = at, max(lower, short)
            feet_x[side] = w.stood.get(side, balls[side][0])
            continue
        if side in w.anchor:
            w.stood[side] = feet_x[side] = w.anchor[side][0]
            continue
        feet_x[side] = w.stood.get(side, balls[side][0])
        u = (q - gait.TOE_OFF) / (1.0 - gait.TOE_OFF) if q >= gait.TOE_OFF else 0.0
        # Aimed at its ball: toed out, the ball is off the ankle's line, 12 mm at 6 degrees.
        at = (float(x[i]) + sign * (walkplan.WIDEN_M * math.sin(math.pi * u) ** 2 - gait.BALL
                                    * math.sin(math.radians(gait.TOE_OUT_DEG))), ankle[1],
              planned_z + ankle[2] + fore)
        at = clear(at, sign, balls[walkplan._OTHER[side]], ankles[walkplan._OTHER[side]])
        hip = figure.hip(sign, pel, turn_now)
        at, short = reach(hip, at, stance.SWING_REACH * gait.REACH)
        if (w.side is None and catch[i] and short > 0.0 and u >= capture.FROM_U
                and (w.held is None or w.age >= SIDE_AGAIN_S)):
            # Where the capture point will be as the foot comes down, DOWN_S on, no ankle to
            # help: put down past where it was, the foot came in under her as it ran on.
            xs, _swap, _catch, _off = capture.landing(
                capture.state(1, margin=0.0, gain=1.0),
                ((1.0 - DOWN_S * w.rate,), (sign,), (xi,), (standing[i],), (sep,),
                 (w.rate,), (omega,)))
            w.side = {'down': side, 'out': 'right' if side == 'left' else 'left',
                         'stage': 'down', 'since': 0.0, 'stomp': True,
                         'at': (float(xs[0]) + sign * STOMP_PAST_M, ankle[1],
                                put_down(balls[side][2], pel[2])),
                         'x_out': None, 'borne': 0.0, 'down_s': 0.0}
            w.hurry, w.catching = SIDE_HURRY, True
        out[side], lower = at, max(lower, short)
    return out, lower, feet_x, off


def sidestep(w, dt, swapping, loads, balls, legs, pel, ankles):
    """The side step's stages: begun on a swap the law asks while the other foot bears -
    `down`, the swinging foot put down flat where it is, SIDE_CLEAR_M across at least;
    `out` once it bears BEARS_N for DWELL_S, the other foot stepping; over when that one
    bears her, or the first still does SIDE_GIVE_S on - the walk begun again on whichever
    (`step`), and none begun while that blends in. A stomp (`_landings`) is the `down`
    alone, the walk begun again on that foot. Resumed at the phase left, the
    plan lifted the foot she had just caught herself on; asked as the blend began, a second
    side step put the foot down in the air (2026-09-26)."""
    if w.side is None and (w.held is None or w.age >= SIDE_AGAIN_S):
        for i, ((side, sign), (ankle, _tw, _pi, _to)) in enumerate(zip(walkplan.SIDES, legs)):
            other = 'right' if side == 'left' else 'left'
            if swapping[i] and loads[other] > stance.LANDED_N and side not in w.anchor:
                ball, there = balls[side], balls[other][0]
                across = max(sign * (ball[0] - there), SIDE_CLEAR_M)
                w.side = {'down': side, 'out': other, 'stage': 'down', 'since': 0.0,
                             'stomp': False,
                             'at': (there + sign * across, ankle[1],
                                    put_down(ball[2], pel[2])),
                             'x_out': None, 'borne': 0.0, 'down_s': 0.0}
                break
    if w.side is None:
        return
    if w.side['stage'] == 'down':
        # Borne only with the ankle near the floor: pitched from its swing, the foot's toes
        # took 1000 N 9 cm up and the walk began again on them (2026-09-27).
        down = w.side['down']
        took = loads[down] > stance.BEARS_N and ankles[down][1] < gait.ANKLE_H + DOWN_M
        w.side['borne'] = w.side['borne'] + dt if took else 0.0
        if w.side['borne'] >= DWELL_S and w.side['stomp']:
            w.resume = w.side['down']
        elif w.side['borne'] >= DWELL_S:
            w.side['stage'], w.side['since'], w.side['borne'] = 'out', 0.0, 0.0
            w.side['x_from'] = balls[w.side['out']][0]
            w.side['z_from'] = balls[w.side['out']][2] - gait.BALL
            w.side['z_land'] = pel[2] + w.v_on * SIDE_S + SIDE_AHEAD_M
    else:
        w.side['since'] += dt
        out, down = w.side['out'], w.side['down']
        took = (loads[out] > max(stance.BEARS_N, loads[down])
                and ankles[out][1] < gait.ANKLE_H + DOWN_M)
        w.side['borne'] = w.side['borne'] + dt if took else 0.0
        # The step stays this pass, the walk begun again at the next (`step`): let go at
        # once, the plan's stale phase swung the foot she stood on 0.8 m out for a pass, and
        # the blend began from that (2026-09-26).
        if w.side['since'] >= 0.5 * SIDE_S and w.side['borne'] >= DWELL_S:
            w.resume = out
        elif w.side['since'] >= SIDE_GIVE_S and loads[down] > max(stance.BEARS_N, loads[out]):
            w.resume = down
