"""Her legs and centre of mass sat or sitting down: the knee and ankle a seat asks, where they put her.

    knee, ankle = seated.landed(hips, edge, top, trunk)   # her hips `hips` m up onto a seat `top` up, `edge` behind
    z = seated.com(knee, ankle, trunk, modes, body)     # her centre of mass in her sagittal unit

Her figure's links and segments (`machine.body.figure`); her modes' units her manifest's (`defrobot.read`).
"""
import math

from machine.body import figure
from machine.moves import leap
from machine.law import poise

#: Halvings of her thigh's length that find a landing's hips (`landed`): 1e-13 m.
HALVINGS = 40
#: Her soles' back edge ahead of her ankles, m: her ZMP's margin inside her heels (`poise.EXTENT`).
HEELS = leap.MIDDLE - float(poise.EXTENT[0])
#: Her thigh's capsules (`figure.BODY`): radius, from, to, its frame - down it -y, ahead z.
THIGHS = tuple((r, a, b) for seg, r, a, b in figure.BODY if seg == 'thigh')
#: Her hand's capsule (`figure.BODY`): radius, its middle in the hand's frame.
HAND = next((r, a) for seg, r, a, _b in figure.BODY if seg == 'hand')
#: Her thigh's underside at her knee over the floor, m, her shank plumb on a flat sole: a seat over it her
#: feet do not set her on (`acts._sought`).
POPLITEAL = figure.ANKLE_H + figure.SHANK - THIGHS[-1][0]
#: Newton's steps that set her fist where it is asked (`fisted`): within 1e-16 m from her arm hanging; the
#: turn its slope is read over, rad.
NEWTON, NUDGE = 12, 1e-6
#: Her fist's centre on the seat she sits on, up off her pelvis's middle, m (`sat`).
ON = figure.FIST_R + figure.SEAT_Y - figure.SEAT_R


def axial(knee, modes, body):
    """Her height's mode at a knee's bend, rad: her legs' shortening (`modal.shortened`) as her
    posture bends her knees for it (`impedance._poses`), a leg's mass off hers (`modal.frame`)."""
    hips_by = body['mass'] / (body['mass'] - modes['mode_limb_lift'][3])
    return ((math.cos(0.5 * math.radians(body['knee'])) - math.cos(0.5 * knee)) * (figure.THIGH + figure.SHANK)
            / (modes['mode_axial_compression'][0] * hips_by))


def knee(hips, ankle):
    """The knee's bend, rad, her hip joints `hips` m over the floor at, the ankle `ankle` bent."""
    rise = (hips - figure.ANKLE_H - figure.SHANK * math.cos(ankle)) / figure.THIGH
    return ankle + math.acos(max(-1.0, min(1.0, rise)))


def hips(knee, ankle):
    """Her hip joint's place over her ankle, (ahead, up) m, her knee and ankle bent `knee`, `ankle`."""
    return (figure.SHANK * math.sin(ankle) - figure.THIGH * math.sin(knee - ankle),
            figure.SHANK * math.cos(ankle) + figure.THIGH * math.cos(knee - ankle))


def posed(knee, ankle, trunk):
    """{joint: deg}: her knees and ankles bent `knee`, `ankle`, her pelvis and trunk `trunk` rad forward
    - her pitch turns both (`impedance._poses`) -, her soles flat, the rest of her as she stands."""
    deg = figure.stand()
    for side in ('left_', 'right_'):
        deg[side + 'hip'] = math.degrees(ankle - knee - trunk)
        deg[side + 'knee'], deg[side + 'ankle'] = math.degrees(knee), -math.degrees(ankle)
    return deg


def sagittal(p, knee, ankle, trunk):
    """A point `p` of her `posed` (her pelvis's frame, her pelvis at its origin turned `trunk` forward)
    as (ahead of her ankles, up off the floor), m."""
    hx, hy = hips(knee, ankle)
    return (p[2] + figure.HIP_DROP * math.sin(trunk) + hx, p[1] + figure.HIP_DROP * math.cos(trunk) + hy + figure.ANKLE_H)


def ahead(knee, ankle, trunk):
    """Her centre of mass ahead of her ankles, m, `posed`: her figure's segments (`figure.com`), within
    3 mm of her model's. Her manifest's masses lumped, each leg at its thigh's and shank's middles, put
    it 4 mm off: her hips stopped 6 mm over a seat they were asked 5.7 mm into."""
    return sagittal(figure.com(posed(knee, ankle, trunk), (0.0, 0.0, 0.0), figure.rx(trunk)), knee, ankle, trunk)[0]


def com(knee, ankle, trunk, modes, body):
    """Her centre of mass as `ahead` has it in her sagittal unit, off her soles' middle - her
    sagittal mode's zero (`modal.reading`, `wbc`'s sole point). Off her stand's instead, 2.9 cm
    behind it, a descent asked her 2.4 cm ahead of its knee and ankle: her ankle bent 0.41 for 0.32
    and her hips stopped 21 mm over the seat."""
    return (ahead(knee, ankle, trunk) - leap.MIDDLE) / modes['mode_sagittal_shift'][0]


def clear(knee, ankle, edge, top):
    """m her thighs clear a seat's front edge `edge` m behind her ankles, its top `top` m up, her
    knee and ankle bent `knee`, `ankle` - negative, in it: her thigh's capsules (THIGHS) in her
    sagittal plane, the edge a line across her."""
    hx, hy = hips(knee, ankle)
    ux, uy = (figure.SHANK * math.sin(ankle) - hx) / figure.THIGH, (figure.SHANK * math.cos(ankle) - hy) / figure.THIGH
    ex, ey = -edge - hx, top - figure.ANKLE_H - hy
    gaps = []
    for r, a, b in THIGHS:
        ax, ay, bx, by = -a[1] * ux - a[2] * uy, -a[1] * uy + a[2] * ux, -b[1] * ux - b[2] * uy, -b[1] * uy + b[2] * ux
        t = max(0.0, min(1.0, ((ex - ax) * (bx - ax) + (ey - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
        gaps.append(math.hypot(ex - ax - t * (bx - ax), ey - ay - t * (by - ay)) - r)
    return min(gaps)


def landed(hips, edge, top, trunk):
    """(knee, ankle), rad, that land her hip joints `hips` m over the floor as far behind her ankles
    as sets her centre of mass at her soles' back edge (HEELS), her trunk `trunk` rad forward -
    halved for till HALVINGS -, as her thighs clear the edge of a seat `top` m up and `edge` m
    behind her ankles (`clear`), and no further than a buttock past it (`figure.SEAT_Z`). Over her
    ankles her hips met a seat 0.19 m behind her soles' middle 2.6 cm onto it, her pelvis alone on
    its edge, and it bore 0.39 of her: 20 docks of 25 never borne. A buttock past an edge 0.12 m
    behind her soles' middle her thighs were 14 mm into it: they met it 2-5 cm over the seat, one
    first, her pelvis hung off it and fell back. At the suite's ankle, 0.32, her centre of mass
    stood 7 mm inside her heels' edge, her balance held it 28 mm behind her ankles and her hips
    stopped 21 mm over the seat; landed a buttock's length behind the seen edge whatever her centre
    of mass, she rose 9 times of 14."""
    lo, hi = 0.0, figure.THIGH
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        knee, ankle = legs(hips, mid)
        back = ahead(knee, ankle, trunk) < HEELS or clear(knee, ankle, edge, top) < 0.0
        lo, hi = (lo, mid) if back else (mid, hi)
    return legs(hips, min(0.5 * (lo + hi), edge - figure.SEAT_Z[0]))


def legs(hips, behind):
    """(knee, ankle), rad, that put her hip joints `hips` m over the floor and `behind` m behind her
    ankles: her leg's two links from the ankle (`figure.SHANK`, `THIGH`), its knee by the cosines."""
    up, shank, thigh = hips - figure.ANKLE_H, figure.SHANK, figure.THIGH
    knee = math.acos(max(-1.0, min(1.0, (behind * behind + up * up - shank * shank - thigh * thigh) / (2.0 * shank * thigh))))
    return knee, math.atan2(-behind, up) + math.atan2(thigh * math.sin(knee), shank + thigh * math.cos(knee))


def sat(seat):
    """Her hip joints' height sat on a surface `seat` m up: her buttocks under her pelvis, her hips
    under its middle (`figure`)."""
    return seat - figure.SEAT_Y + figure.SEAT_R - figure.HIP_DROP



def braced(trunk):
    """Her shoulders' flexion, rad (back negative), that sets her fists on the seat she sits on behind her,
    her spine `trunk` rad forward (back negative), her elbows as she stands - halved for till
    HALVINGS: sat, her pelvis's middle SEAT_R - SEAT_Y over the seat's top (`sat`)."""
    lo, hi, deg = -0.5 * math.pi, 0.0, dict(figure.stand(), spine=math.degrees(trunk))
    r, (hx, hy, hz) = HAND
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        deg['left_shoulder'] = math.degrees(mid)
        at, here = figure.frames(deg, (0.0, figure.SEAT_R - figure.SEAT_Y, 0.0), figure.IDENT)['left_hand']
        low = at[1] + sum(here[1][k] * c for k, c in enumerate((hx, hy, hz))) - r
        lo, hi = (lo, mid) if low < 0.0 else (mid, hi)
    return 0.5 * (lo + hi)


def fisted(up, ahead, trunk):
    """(shoulder, elbow) flexion, rad, that set her fist `up` m over and `ahead` m ahead of her pelvis's middle,
    her spine `trunk` rad forward (`figure.fist`, her right's mirroring her left's): NEWTON steps from her arm
    hanging, its slope read NUDGE apart, her elbow no straighter than straight."""
    deg = dict(figure.stand(), spine=math.degrees(trunk))
    q = (0.0, math.radians(deg['left_elbow']))

    def off(q):
        deg['left_shoulder'], deg['left_elbow'] = math.degrees(q[0]), math.degrees(q[1])
        at = figure.fist(deg)
        return at[1] - up, at[2] - ahead
    for _ in range(NEWTON):
        e = off(q)
        (a, b), (c, d) = (((f[0] - e[0]) / NUDGE, (f[1] - e[1]) / NUDGE)
                          for f in (off((q[0] + NUDGE, q[1])), off((q[0], q[1] + NUDGE))))
        det = a * d - b * c
        q = (q[0] - (d * e[0] - c * e[1]) / det, max(0.0, q[1] - (a * e[1] - b * e[0]) / det))
    return q


def placed(up, trunk):
    """(shoulder, elbow, aside, levers) of her fist set `up` m over her pelvis's middle under her shoulder, her
    spine `trunk` rad forward: its flexions, rad (`fisted`); how far aside of her middle it stands, m; and the
    reach along her way from her shoulder and her elbow to it, m - what a push up on her fist turns them by."""
    deg = dict(figure.stand(), spine=math.degrees(trunk))
    shoulder = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0]
    flexed = fisted(up, shoulder[2], trunk)
    deg['left_shoulder'], deg['left_elbow'] = (math.degrees(v) for v in flexed)
    at, elbow = figure.fist(deg), figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_forearm'][0]
    return flexed + (abs(at[0]), (abs(at[2] - shoulder[2]), abs(at[2] - elbow[2])))


def reaches(top):
    """Whether her legs reach the floor from her hips on a seat `top` m up (`sat`), plumb and straight: past
    it nothing her soles stand on bears her up onto it, her hands no help."""
    return sat(top) - figure.ANKLE_H <= figure.THIGH + figure.SHANK
