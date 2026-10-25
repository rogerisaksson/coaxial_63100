"""Her legs and centre of mass sat or sitting down: the knee and ankle a seat asks, where they put her.

    knee, ankle = seated.landed(hips, edge, top, trunk)   # her hips `hips` m up onto a seat `top` up, `edge` behind
    z = seated.com(knee, ankle, trunk, modes, body)     # her centre of mass in her sagittal unit
    shoulder, elbow = seated.reached(knee, ankle, trunk, (ahead, up))   # her arm putting her fist there

Her figure's links and segments (`machine.figure`); her modes' units her manifest's (`defrobot.read`).
"""
import math

from machine import drives, figure, leap, poise, terms
from machine.errors import MachineError

#: Halvings of her thigh's length that find a landing's hips (`landed`): 1e-13 m.
HALVINGS = 40
#: Her hands' joints where her build holds them (`drives.passive`), deg: her wrists, her fingers curled.
HELD = {j: held[2] for j in ('left_wrist', 'left_gripper', 'right_wrist', 'right_gripper') for held in (drives.passive(j),) if held}
#: Newton's steps that put her hand on a point (`reached`), their difference, deg, and how near, m.
REACHES, DEG, NEAR = 20, 1e-5, 1e-9
#: Her soles' back edge ahead of her ankles, m: her ZMP's margin inside her heels (`poise.EXTENT`).
HEELS = leap.MIDDLE - float(poise.EXTENT[0])
#: Her thigh's capsules (`figure.BODY`): radius, from, to, its frame - down it -y, ahead z.
THIGHS = tuple((r, a, b) for seg, r, a, b in figure.BODY if seg == 'thigh')
#: Her hand's capsule (`figure.BODY`): radius, its middle in the hand's frame.
HAND = next((r, a) for seg, r, a, _b in figure.BODY if seg == 'hand')


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


def ischia(trunk):
    """Her ischia - her buttocks' middle (`figure.SEAT_Z`) - ahead of her hip joints, m, her pelvis
    `trunk` rad forward."""
    return figure.apply(figure.rx(trunk), (0.0, figure.SEAT_Y, 0.5 * sum(figure.SEAT_Z)))[2] + figure.HIP_DROP * math.sin(trunk)


def reached(knee, ankle, trunk, at, side='left'):
    """(shoulder, elbow), rad, of her `side` arm putting her hand's lowest point (`figure.palm`) `at`
    (ahead of her ankles, up off the floor, m), `posed` `knee`, `ankle`, `trunk`, her hands as her build
    holds them (HELD): Newton's steps from her arm as she stands, her elbow half bent - from her stand's 10 deg
    a fist ahead of her shoulder was reached with the elbow 33 deg past straight. Past her arm or her elbow's
    range (`terms`), refused."""
    deg, turn, joints = {**posed(knee, ankle, trunk), **HELD}, figure.rx(trunk), (side + '_shoulder', side + '_elbow')
    _kind, _sign, low, high = terms.TERMS[('elbow', 'flexion')]

    def where(q):
        deg.update(zip(joints, q))
        return sagittal(figure.palm(deg, (0.0, 0.0, 0.0), turn, side), knee, ankle, trunk)

    q = [deg[joints[0]], 0.5 * (low + high)]
    for _ in range(REACHES):
        p = where(q)
        miss = (at[0] - p[0], at[1] - p[1])
        if math.hypot(*miss) < NEAR:
            break
        (a, b), (c, d) = ([(where([q[0] + DEG * (k == 0), q[1] + DEG * (k == 1)])[i] - p[i]) / DEG for k in (0, 1)]
                          for i in (0, 1))
        det = a * d - b * c
        q = [q[0] + (d * miss[0] - b * miss[1]) / det, q[1] + (a * miss[1] - c * miss[0]) / det]
    p = where(q)
    off = math.hypot(at[0] - p[0], at[1] - p[1])
    if off >= NEAR or not low <= q[1] <= high:
        raise MachineError('refused: her hand %.3f m ahead of her ankles and %.3f up, her arm %.3f m off it, her elbow %.0f deg'
                           % (at[0], at[1], off, q[1]))
    return math.radians(q[0]), math.radians(q[1])


def braced(trunk):
    """Her shoulders' flexion, rad (back negative), that sets her fists on the seat she sits on behind her,
    her pelvis and trunk `trunk` rad forward (back negative), her elbows as she stands - halved for till
    HALVINGS: sat, her pelvis's middle SEAT_R - SEAT_Y over the seat's top (`sat`)."""
    lo, hi, deg = -0.5 * math.pi, 0.0, figure.stand()
    r, (hx, hy, hz) = HAND
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        deg['left_shoulder'] = math.degrees(mid)
        at, here = figure.frames(deg, (0.0, figure.SEAT_R - figure.SEAT_Y, 0.0), figure.rx(trunk))['left_hand']
        low = at[1] + sum(here[1][k] * c for k, c in enumerate((hx, hy, hz))) - r
        lo, hi = (lo, mid) if low < 0.0 else (mid, hi)
    return 0.5 * (lo + hi)
