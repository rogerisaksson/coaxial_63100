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


def hooped(torque_nmm, r_mm, grip_mm, wall_mm):
    """How many times over a slit band's wall `wall_mm` carries the hoop gripping `torque_nmm` on a drum `r_mm` round
    over `grip_mm` of it, at the printed polymer's strength."""
    from machine.body import structural
    return wall_mm * grip_mm * structural.PAHT_CF[2] / _hoop(torque_nmm, r_mm)


def leant(torque_nmm, tilt_nmm, r_mm, grip_mm):
    """The torque whose hoop a slit band's screws pull to grip `torque_nmm` on a drum `r_mm` round and stay shut
    under a moment `tilt_nmm` across the drum's axis, N mm: the tilt asks the hoop 6 M / (pi L) - less, and the
    band's pressure on the drum's far edge falls to nothing and the band opens -, added as the torque of that hoop."""
    return torque_nmm + 6.0 * tilt_nmm / (math.pi * grip_mm) * 2.0 * math.pi * kl.FRICTION * r_mm


def held(torque_nmm, r_mm):
    """How many times over a band's PINCH screws - the size `pinch` picks - pull the hoop gripping `torque_nmm`."""
    hoop = _hoop(torque_nmm, r_mm)
    return PINCH * next((p for p, _h in PINCHES if PINCH * p >= hoop), PINCHES[-1][0]) / hoop


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


def tube(bend_nmm, twist_nmm=0.0, pull_n=0.0, least=0.0):
    """(Outer radius, wall, its worst stress MPa) of the slimmest stock carbon tube - none under `least` mm round -
    whose wall holds a bending, a twist and a pull at once, mm: the stock table's (`materials.toml` [tubes]), its
    principal stress at the carbon's allowable; the stoutest where none holds."""
    from machine.body import mechatronics, structural
    found = (0.0, 0.0, 0.0)
    for d, wall in sorted((float(d), 1e3 * float(w)) for d, w in mechatronics.materials()['tubes']['stock']):
        r, bore = d / 2.0, d / 2.0 - wall
        second, area = math.pi / 4.0 * (r ** 4 - bore ** 4), math.pi * (r * r - bore * bore)
        normal, shear = bend_nmm * r / second + pull_n / area, twist_nmm * r / (2.0 * second)
        found = (r, wall, normal / 2.0 + math.hypot(normal / 2.0, shear))
        if r >= least and found[2] <= structural.CARBON[2]:
            break
    return found


def section(outline):
    """(Area mm2, its middle, (second moments about the middle along the first axis, along the second, their
    product) mm4, the widest circle it holds, mm) of a solid section: its outline [(u, v)] mm, in order round."""
    import numpy as np
    p = np.asarray(outline, float)
    q = np.roll(p, -1, axis=0)
    a = p[:, 0] * q[:, 1] - q[:, 0] * p[:, 1]
    turn = 1.0 if a.sum() > 0 else -1.0
    area = 0.5 * turn * a.sum()
    mid = turn * ((p + q) * a[:, None]).sum(axis=0) / (6.0 * area)
    uu = turn * ((p[:, 0] ** 2 + p[:, 0] * q[:, 0] + q[:, 0] ** 2) * a).sum() / 12.0 - area * mid[0] ** 2
    vv = turn * ((p[:, 1] ** 2 + p[:, 1] * q[:, 1] + q[:, 1] ** 2) * a).sum() / 12.0 - area * mid[1] ** 2
    uv = turn * ((p[:, 0] * q[:, 1] + 2.0 * p[:, 0] * p[:, 1] + 2.0 * q[:, 0] * q[:, 1] + q[:, 0] * p[:, 1]) * a
                 ).sum() / 24.0 - area * mid[0] * mid[1]
    lo, hi = p.min(axis=0), p.max(axis=0)
    grid = np.stack(np.meshgrid(np.arange(lo[0], hi[0], 0.5), np.arange(lo[1], hi[1], 0.5)), axis=-1).reshape(-1, 2)
    edge = q - p
    t = np.clip(((grid[:, None, :] - p[None]) * edge[None]).sum(axis=2) / (edge ** 2).sum(axis=1), 0.0, 1.0)
    away = np.linalg.norm(grid[:, None, :] - (p[None] + t[..., None] * edge[None]), axis=2).min(axis=1)
    rise = np.where(edge[:, 1] == 0.0, 1.0, edge[:, 1])
    cut = p[None, :, 0] + edge[None, :, 0] * (grid[:, None, 1] - p[None, :, 1]) / rise[None]
    over = ((p[None, :, 1] > grid[:, None, 1]) != (q[None, :, 1] > grid[:, None, 1])) & (grid[:, None, 0] < cut)
    return float(area), (float(mid[0]), float(mid[1])), (float(uu), float(vv), float(uv)), 2.0 * float(
        away[over.sum(axis=1) % 2 == 1].max())


def borne(outline, bend, twist=0.0, pull=0.0):
    """The highest principal stress, MPa, in a solid section - its outline [(u, v)] mm - bent by `bend` (the moment
    whose stress grows along u, the one along v) N mm, each either way, twisted by `twist` N mm and pulled by `pull`
    N at once: the bending at its corners; the twist's shear where its widest circle D touches its side, as for
    any solid section without a re-entrant corner - T C / K, K = A^4 / (40 Ip), C = D (1 + 0.15 q) / (1 + q),
    q = pi^2 D^4 / (16 A^2) (a rectangle's own within 3 %) -; the two taken at one point."""
    area, mid, (uu, vv, uv), wide = section(outline)
    det = uu * vv - uv * uv
    normal = max(abs(((su * bend[0] * vv - sv * bend[1] * uv) * (u - mid[0])
                      + (sv * bend[1] * uu - su * bend[0] * uv) * (v - mid[1])) / det)
                 for u, v in outline for su in (1.0, -1.0) for sv in (1.0, -1.0)) + abs(pull) / area
    q = math.pi ** 2 * wide ** 4 / (16.0 * area ** 2)
    shear = twist * wide * (1.0 + 0.15 * q) / (1.0 + q) * 40.0 * (uu + vv) / area ** 4
    return normal / 2.0 + math.hypot(normal / 2.0, shear)


def yoke(span_mm, width_mm, joint='knee'):
    """The least thickness of a plate `width_mm` wide across a `span_mm` between two arms, the joint's axial load at
    its middle, mm."""
    from machine.body import structural
    moment = kl.loads(joint)['axial'] * span_mm / 4.0
    return max(kl.WALL, math.sqrt(6.0 * moment / (width_mm * structural.PAHT_CF[2])))
