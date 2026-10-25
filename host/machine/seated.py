"""Her legs and centre of mass sat or sitting down: the knee and ankle a seat asks, where they put her.

    knee, ankle = seated.landed(hips, trunk)            # her hips `hips` m up, her trunk `trunk` forward
    z = seated.com(knee, ankle, trunk, modes, body)     # her centre of mass in her sagittal unit

Her figure's links and segments (`machine.figure`); her modes' units her manifest's (`ortho.robot`).
"""
import math

from machine import figure, leap, poise

#: Halvings of her thigh's length that find a landing's hips (`landed`): 1e-13 m.
HALVINGS = 40
#: Her soles' back edge ahead of her ankles, m: her ZMP's margin inside her heels (`poise.EXTENT`).
HEELS = leap.MIDDLE - float(poise.EXTENT[0])


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


def ahead(knee, ankle, trunk):
    """Her centre of mass ahead of her ankles, m, her knees and ankles bent `knee`, `ankle`, her
    pelvis and trunk `trunk` rad forward - her pitch turns both (`impedance._poses`) -, her soles
    flat, the rest of her as she stands: her figure's segments (`figure.com`), within 3 mm of her
    model's. Her manifest's masses lumped, each leg at its thigh's and shank's middles, put it 4 mm
    off: her hips stopped 6 mm over a seat they were asked 5.7 mm into."""
    deg = figure.stand()
    for side in ('left_', 'right_'):
        deg[side + 'hip'] = math.degrees(ankle - knee - trunk)
        deg[side + 'knee'], deg[side + 'ankle'] = math.degrees(knee), -math.degrees(ankle)
    return figure.com(deg, (0.0, 0.0, 0.0), figure.rx(trunk))[2] + figure.HIP_DROP * math.sin(trunk) + hips(knee, ankle)[0]


def com(knee, ankle, trunk, modes, body):
    """Her centre of mass as `ahead` has it in her sagittal unit, off her soles' middle - her
    sagittal mode's zero (`modal.reading`, `wbc`'s sole point). Off her stand's instead, 2.9 cm
    behind it, a descent asked her 2.4 cm ahead of its knee and ankle: her ankle bent 0.41 for 0.32
    and her hips stopped 21 mm over the seat."""
    return (ahead(knee, ankle, trunk) - leap.MIDDLE) / modes['mode_sagittal_shift'][0]


def landed(hips, trunk):
    """(knee, ankle), rad, that land her hip joints `hips` m over the floor as far behind her ankles
    as sets her centre of mass at her soles' back edge (HEELS), her trunk `trunk` rad forward -
    halved for till HALVINGS. So she sits 0.29 legs back and upright (a human's 0.31-0.47, within 7
    deg), on a seat over her knees her thighs on its front edge and her ischia on it (`borne`, two).
    Landed where her thighs cleared the edge and no further than a buttock past it she sat on the
    edge, 0.23 legs and 13 deg forward; over her ankles her hips met a seat 0.19 m behind her soles'
    middle 2.6 cm onto it, her pelvis alone on its edge, and it bore 0.39 of her: 20 docks of 25
    never borne. At the suite's ankle, 0.32, her centre of mass stood 7 mm inside her heels' edge,
    her balance held it 28 mm behind her ankles and her hips stopped 21 mm over the seat; landed a
    buttock's length behind the seen edge whatever her centre of mass, she rose 9 times of 14."""
    lo, hi = 0.0, figure.THIGH
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        knee, ankle = legs(hips, mid)
        lo, hi = (lo, mid) if ahead(knee, ankle, trunk) < HEELS else (mid, hi)
    return legs(hips, 0.5 * (lo + hi))


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
