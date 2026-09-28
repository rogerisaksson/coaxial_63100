"""The standing feet: where each is held, the stance legs' joints, and the phase's pace.

`anchor(w, ..)` holds a foot where it landed; `legs(w, ..)` solves each leg's joints, a stance leg
carrying the pelvis, a swing leg reaching; `advance(w, ..)` moves the phase on at its pace, pulled
to where the pelvis stands over the stance balls; `reachable(w, ..)` keeps the pelvis's target
within the stance legs' reach. `w` is the `walker.Walker`.
"""
import math

from machine import figure, gait, walkplan
from machine.figure import LEG, mul, t


#: A swinging foot is reached for no further than SWING_REACH of a leg's: asked further, its step
#: is shortened first and the pelvis lowered the rest, LOWER_M at most, LOWER_M_S a second down
#: and RAISE_M_S up - lowered at once, the standing knee bent before the body had sunk and the
#: foot left the floor; raised as fast, the leg threw her 13 cm into the air (2026-09-26).
#: Latched 10 mm low at every landing - the trailing leg sags 9 mm into it, its ankle 2 degrees
#: under the push-off's torque - and raised at 0.1 m/s, the pelvis rushed up at 0.22 m/s to a
#: dead stop, 4.6 then -3.8 m/s2 at her ears; at 0.03 the rise 0.15 m/s, the stop -1 m/s2, the
#: stir 1.37 -> 1.28 mm (2026-09-27). RAISE_S lifts the rest in proportion: latched lower at each
#: landing, she sank into a crouch and stayed (2026-09-28).
SWING_REACH, LOWER_M, LOWER_M_S, RAISE_M_S, RAISE_S = 0.985, 0.12, 0.4, 0.03, 2.0


#: The phase pulled to the body, strides a second per stride of error.
PULL = 5.0


#: A standing foot does not lift while the other bears under LANDED_N, LAND_WAIT_S at most: its
#: phase pinned PIN short of toe-off, a stride growing carried it 2e-5 a pass past a 1e-6 pin
#: (2026-09-26).
LAND_WAIT_S, PIN = 0.3, 1e-3


#: From standing on the left foot: the stride grows from BEGIN of a stride's to all of it over
#: RAMP_S, the setpoints ease from the stand's over BLEND_S; the phase starts at BEGIN_AT, the left
#: foot flat, the right lifted into its swing - begun at its toe-off, the plan tipped a flat foot
#: onto its ball and it threw her (2026-09-25).
BEGIN, RAMP_S, BLEND_S, BEGIN_AT = 0.1, 2.5, 0.3, 0.30


#: From standing, the phase runs at EASE_IN of the cadence at first, and is pulled to the body
#: from PULL_FROM of the walk's stride: pulled only past half, it ran ahead of her first step and
#: the front leg, straightened for a pelvis not yet over it, lifted her (2026-09-26).
EASE_IN, PULL_FROM = 0.8, 0.2


#: The pull slows the phase to LEAD of its pace at most, and hurries it to 1/LEAD: pulled to a
#: body standing still, the phase stood still with it, one foot in the air for a second, and she
#: toppled off the other; thrown forward off a short landing, it raced to 3.5 strides/s
#: (2026-09-26).
LEAD = 0.5


#: To a stop (`halt`): over HALT_S the stride shrinks to HALT of a stride's, the first stride's
#: (`arrival.FIRST`), and the pace to HALT_PACE of the cadence. Down to 0.1 in 2 s, the hips rose
#: with the short stride's reach and threw her off the floor, 0.3 s on no foot; at the pace kept,
#: she came onto the front foot at 0.55 m/s and on over its ball (2026-09-26).
HALT, HALT_S, HALT_PACE = 0.6, 1.5, 0.7


#: A sole bearing this much has landed, N; bearing BEARS_N it is all stance, up to BEARS_UNTIL
#: of a stride past its toe-off.
LANDED_N, BEARS_N, BEARS_UNTIL = 60.0, 250.0, 0.06


FLAT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def anchor(w, bus, qs, balls, height, pel):
    """A foot is held where it landed until it leaves: landed once its sole bears, or ACCEPT
    into its stance whatever it bears. Held from the phase alone, a foot still in the air
    was taken to bear her, and she fell when the other left (2026-09-25)."""
    for (side, _sign), q in zip(walkplan.SIDES, qs):
        if q >= gait.TOE_OFF or (w.side is not None and side == w.side['out']
                                 and w.side['stage'] == 'out'):
            w.anchor.pop(side, None)
        elif side not in w.anchor and (bus['pelvis.pose.%s_load' % side] > LANDED_N
                                          or q >= walkplan.ACCEPT):
            w.anchor[side] = balls[side]
            # The height's target starts from where the body is, up again (RAISE_M_S):
            # landed with the body 3 cm low over the leaning leg, both legs pushed to the
            # plan's height and threw her 5 cm into the air (2026-09-26); let go of the first
            # 15 mm, the walk's first landing hopped off the front foot (2026-09-27).
            w.lowered = max(w.lowered, min(LOWER_M, height - pel[1]))
        w.was_q[side] = q


def legs(w, out, bus, qs, legs, feet, held, swings, target, turn, turn_now, pel):
    """Each leg's joints into `out`, deg. A stance leg carries the pelvis to where it should
    be; a swing leg reaches from where the pelvis is, as it is turned. A leg still bearing is
    a stance leg whatever the phase says: reached from the pelvis as it stood while it bore,
    the rear heel's rise lifted her and the front foot hung off the floor (2026-09-26). In a
    side step the foot that bears carries the pelvis on and up but not across - the body a
    free pendulum on it, as the step out was foreseen - and holds the attitude; the other
    reaches from where the pelvis is; both feet flat. Carried across to the plan's target
    20 cm off, the leg chattered on the floor, then threw her up; its attitude let go, the
    torso fell over the hip in 60 ms; its height let go, she sank; the stepped-out foot
    pitched as the plan's swing had it struck the floor on its edge, 2200 N, 5 cm before its
    ankle was down (2026-09-26)."""
    for (side, sign), q, (_ankle, _tw, _pi, toes), foot in zip(walkplan.SIDES, qs, legs, feet):
        b = walkplan.carried(q)
        at = held[side] if side in held else swings[side]
        if w.side is not None and (side == w.side['out'] or q >= gait.TOE_OFF):
            foot, toes = FLAT, 0.0
        if q < gait.TOE_OFF + BEARS_UNTIL and bus['pelvis.pose.%s_load' % side] > LANDED_N:
            b = max(b, min(1.0, bus['pelvis.pose.%s_load' % side] / BEARS_N))
        if w.side is not None:
            bears = w.side['out'] if w.side['stage'] == 'down' else w.side['down']
            b = 1.0 if side == bears else 0.0
            hip_from = (pel[0], target[1], target[2]) if b else pel
        else:
            hip_from = tuple(b * a + (1.0 - b) * c for a, c in zip(target, pel))
        reach = turn if b >= 1.0 else mul(walkplan.turned(tuple(b * c for c in walkplan.vee(mul(turn, t(turn_now))))), turn_now)
        for k, v in zip(LEG, figure.leg(sign, hip_from, reach, at, foot)):
            out[side + k] = math.degrees(v)
        out[side + '_foot'] = toes


def advance(w, dt, length, balls, pel, bus):
    """The phase on by a pass: carried across a change of stride, at its pace, pulled to the
    body."""
    # A change of stride carries the phase with it, each stance foot kept where it stands under
    # her (`gait.planted`: L (STANCE_AT - q) ahead of the hip), weighed. Left to the pull, a
    # stride shortened to a stop sped her up, bounced her off the floor and threw her
    # (2026-09-26).
    if w.length_was is not None and abs(length - w.length_was) > 1e-12:
        shift = weight = 0.0
        for q in (w.phase, (w.phase + 0.5) % 1.0):
            b = walkplan.carried(q)
            shift += b * (1.0 - w.length_was / length) * (gait.STANCE_AT - q)
            weight += b
        if weight > 0.0:
            w.phase = (w.phase + shift / weight) % 1.0
    w.length_was = length
    # The phase pulled to where the pelvis stands over each stance ball.
    err = weight = 0.0
    for side, q in (('left', w.phase), ('right', (w.phase + 0.5) % 1.0)):
        if 0.05 < q < gait.TOE_OFF - 0.05:
            b = walkplan.carried(q)
            err += b * ((length * gait.STANCE_AT + gait.BALL - (balls[side][2] - pel[2]))
                        / length - q)
            weight += b
    # A short stride makes a centimetre a large phase: no pull under PULL_FROM of the walk's.
    pull = PULL * max(0.0, min(1.0, (w.scale - PULL_FROM) / (1.0 - PULL_FROM)))
    # From standing the phase starts slow: at full cadence from rest, the rear heel's push threw
    # her up instead of on (2026-09-25).
    pace = w.cadence * (1.0 if w.held is None else
                           EASE_IN + (1.0 - EASE_IN) * gait.eased(w.age / RAMP_S))
    pace *= 1.0 + w.hurry
    if w.halting is not None:
        pace *= 1.0 + (HALT_PACE - 1.0) * gait.eased(w.halting / HALT_S)
    rate = max(LEAD * pace, min(pace / LEAD, pace + pull * (err / weight if weight else 0.0)))
    step = dt * rate
    # A standing foot lifts only once the other bears, LAND_WAIT_S at most: lifted on the
    # clock while the other still reached for the floor, she stood on neither (2026-09-26).
    for side, offset in (('left', 0.0), ('right', 0.5)):
        q = (w.phase + offset) % 1.0
        other = 'right' if side == 'left' else 'left'
        if (q < gait.TOE_OFF <= q + step + PIN and w.waited < LAND_WAIT_S
                and bus['pelvis.pose.%s_load' % other] < LANDED_N):
            step, w.waited = gait.TOE_OFF - PIN - q, w.waited + dt
            break
    else:
        w.waited = 0.0
    w.rate = rate
    w.phase = (w.phase + step) % 1.0


def reachable(w, target, turn, held, qs):
    """The pelvis's target, lowered to where each stance leg reaches it."""
    # Never higher than a stance leg reaches (`gait.REACH`): a foot landed on a longer stride
    # than the plan's now - shortening to a stop - held her up on a straight leg, and its
    # heel's rise threw her off the floor (2026-09-26).
    reach = gait.REACH
    for (side, sign), q in zip(walkplan.SIDES, qs):
        if side in held and walkplan.carried(q) > 0.0:
            hip = figure.hip(sign, target, turn)
            d2 = (hip[0] - held[side][0]) ** 2 + (hip[2] - held[side][2]) ** 2
            over = hip[1] - held[side][1] - math.sqrt(max(0.0, reach * reach - d2))
            if over > 0.0:
                target = (target[0], target[1] - walkplan.carried(q) * over, target[2])
    return target
