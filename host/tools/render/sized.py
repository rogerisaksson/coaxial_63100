"""A joint's printed clamps sized from its loads: each band, neck and yoke the least that holds.

A slit band holds its drum by friction: two screws across its slit (PINCH) pull a hoop in its layers that grips the
drum's torque (`knee_loads.FRICTION`), so its wall is that hoop over its grip at the printed polymer's strength - and
never thinner than hides those screws' heads in it, only its slit seen (SKIN): no lug, no flange, nothing tightened
from outside. A neck - a saddle's waist, a web, a foot - carries all its joint's loads at once in its layers,
as knee_loads' output arm does: the swing bending it across its depth, the lateral across its width, the axial and
the torsion; its depth is the least that holds for its width. A yoke spans two arms under the joint's axial load at
its middle. Every load at the structural margin (`knee_loads.loads`); nothing thinner than the least printed wall
(`knee_loads.WALL`); mm.
"""
import math

from tools.render import knee_loads as kl

#: The pinch screws across a band's slit, and the skin over their heads hid in its wall either side, mm: the elbow's
#: band as approved (9 mm round an M4's 7 mm head), its screws unseen and only its slit showing.
PINCH, SKIN = 2, 1.0
#: The screws a pinch may take, smallest first: preload at 70 % of 12.9's proof on the stress area, N, and the head's
#: radius, mm - an M2 (2.07 mm2, a 3.8 mm head), an M2.5 (3.39 mm2, 4.5 mm) and an M3 (5.03 mm2, 5.5 mm) before
#: knee_loads' own.
PINCHES = (tuple((0.7 * 970.0 * area, head) for area, head in ((2.07, 1.9), (3.39, 2.25), (5.03, 2.75)))
           + tuple((preload, head) for preload, head, _tall in kl.SCREWS.values()))
#: Their threads, mm, as PINCHES lists them.
THREADS = (2.0, 2.5, 3.0) + tuple(float(name[1:]) for name in kl.SCREWS)


def torque(joint):
    """A drive's own output torque at the margin, N mm: its motor's at the board's amps through its gearbox, no
    more than the gearbox takes momentarily - what the band round its housing holds."""
    from machine.body import drives, structural
    s = drives.of(joint)[1]
    return structural.MARGIN * 1e3 * min(s.kt_motor * drives.ratio(joint) * s.efficiency * s.amps, s.shock)


def _hoop(torque_nmm, r_mm):
    """The hoop a slit band's screws pull to grip `torque_nmm` on a drum `r_mm` round, N."""
    return torque_nmm / (2.0 * math.pi * kl.FRICTION * r_mm)


def pinch(torque_nmm, r_mm):
    """The head radius, mm, of the smallest screw PINCH of which across a band's slit pull the hoop gripping
    `torque_nmm` on a drum `r_mm` round."""
    hoop = _hoop(torque_nmm, r_mm)
    return next((head for preload, head in PINCHES if PINCH * preload >= hoop), PINCHES[-1][1])


def band(torque_nmm, r_mm, grip_mm):
    """The least wall of a slit band gripping `torque_nmm` on a drum `r_mm` round over `grip_mm` of it, mm: its hoop's
    at the printed polymer's strength, and no thinner than hides its pinch screws' heads."""
    from machine.body import structural
    hoop = _hoop(torque_nmm, r_mm) / (grip_mm * structural.PAHT_CF[2])
    return max(kl.WALL, hoop, 2.0 * (pinch(torque_nmm, r_mm) + SKIN))


def rim(torque_nmm, r_mm, count):
    """(The least wall of a printed rim, mm, the thread of its screws, mm): `count` screws `r_mm` round carrying
    `torque_nmm` across it, the smallest of THREADS whose bearing in its hole - two threads deep - holds at the printed
    polymer's strength, its hole hid in the wall (SKIN either side); no thinner than the least printed wall."""
    from machine.body import structural
    load = torque_nmm / (count * r_mm)
    thread = next((d for d in THREADS if 2.0 * d * d * structural.PAHT_CF[2] >= load), THREADS[-1])
    return max(kl.WALL, thread + 2.0 * SKIN), thread


def neck(width_mm, joint='knee'):
    """The least depth of a neck `width_mm` along the joint's axis carrying all its loads at once, mm."""
    from machine.body import structural
    ld, allow = kl.loads(joint), structural.PAHT_CF[2]

    def stress(d):
        normal = (6.0 * ld['swing'] / (width_mm * d * d) + 6.0 * ld['lateral'] / (d * width_mm ** 2)
                  + ld['axial'] / (width_mm * d))
        lo, hi = sorted((width_mm, d))
        shear = ld['torsion'] * (3.0 * hi + 1.8 * lo) / (hi * hi * lo * lo)     # a solid rectangle's, at its flat
        return normal / 2.0 + math.hypot(normal / 2.0, shear)
    return float(next(d for d in range(int(kl.WALL), 400) if stress(d) <= allow))


def yoke(span_mm, width_mm, joint='knee'):
    """The least thickness of a plate `width_mm` wide across a `span_mm` between two arms, the joint's axial load at
    its middle, mm."""
    from machine.body import structural
    moment = kl.loads(joint)['axial'] * span_mm / 4.0
    return max(kl.WALL, math.sqrt(6.0 * moment / (width_mm * structural.PAHT_CF[2])))
