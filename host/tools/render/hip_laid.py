"""The hip's layout: its three drives about its middle, what stands between them, sized from its loads.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/hip_laid.py

The hip's frame: its middle the origin, x out to the leg's side, y up, z ahead; the left's. The pitch drive stands
where `skeleton.cans` lays it, on the hip's own axis. The yaw drive stands over it on its own axis, RAISE_M higher
than the build lays it: the thigh's tube, folded to the deepest bend the scenarios ask (POSED), passes under its
gearbox. The roll drive stands behind and over the roll's axis, its own parallel to it and ROLL_AT off: a pinion on
its output's hub turns a toothed sector (GEARS, the build's ratio) - under it nothing, so the thigh swings back past.
The pitch drive's motor is clamped in the cradle (a slit band, `sized.band`), the cradle's back a saddle a steel plate
is screwed onto: the sector at its top, a hollow pin (KINGPIN) welded in its middle. The pin turns in two needle
bearings (NEEDLE) in the yaw's block under the roll drive - the cradle hung from behind alone, since a bearing ahead
of the hip stands in the thigh's way however its strut is led (the thigh folds to 142 degrees and turns out while
folded) - and the block clamps the roll drive's motor. The block's two cheeks reach forward past the pinion to a
crown screwed under the yaw drive's hub. The thigh hangs from the pitch drive's output ring: a slit band round it,
a plate over its hub's face screwed to the hub, an arm down the hip's outer side and a jog in under the cradle to
the thigh's tube's spigot - a neck, as a thigh bone's, round what the roll turns.

`loads` are the hip's as the scenarios stood through loaded it (MEASURED), at the structural margin, no moment under
its drive's own peak. `checks` holds each member to them by hand: the arm, the jog, the cheeks, the hub's and the
plate's screws, the needle bearings, the pin, the teeth.
"""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import sized as ks  # noqa: E402

#: The poses a hip stood in through the scenarios it stood through, both builds, the right's mirrored
#: (tools/sim/legload.py --all, 99 runs): {pitch's cell: (roll from, to, yaw from, to)}, each a cell's lower edge,
#: deg, a cell `legload.CELL_DEG` wide - pitch negative folded forward, roll positive out to the side, yaw positive toes out.
#: Folded past 100 degrees a hip turns out 15 at the most and in 35; standing it turns out 40 (the bed's side step).
POSED = {-145: (-35, 15, -15, 0), -140: (-35, 15, -15, 0), -135: (-30, 15, -15, 20), -130: (-30, 15, -15, 25),
         -125: (-30, 10, -15, 25), -120: (-30, 10, -10, 20), -115: (-30, 10, -10, 20), -110: (-30, 10, -10, 20),
         -105: (-30, 10, -10, 20), -100: (-30, 5, -10, 20), -95: (-30, 5, -10, 20), -90: (-30, 5, -10, 25),
         -85: (-30, 5, -10, 25), -80: (-30, 5, -10, 25), -75: (-25, 5, -10, 25), -70: (-25, 5, -15, 25),
         -65: (-20, 10, -15, 25), -60: (-20, 10, -15, 25), -55: (-20, 10, -25, 25), -50: (-20, 10, -25, 25),
         -45: (-20, 15, -25, 25), -40: (-25, 20, -30, 25), -35: (-25, 25, -25, 25), -30: (-20, 25, -25, 15),
         -25: (-20, 35, -25, 25), -20: (-20, 40, -25, 25), -15: (-15, 40, -30, 25), -10: (-15, 20, -30, 25),
         -5: (-15, 20, -30, 25), 0: (-15, 20, -25, 25), 5: (-15, 20, -25, 25), 10: (-15, 20, -25, 25),
         15: (-15, 15, -25, 25), 20: (-15, 20, -25, 25), 25: (-15, 5, -25, 25), 30: (-15, 5, -25, 25),
         35: (-15, 5, -25, 25), 40: (-15, 5, -25, 25), 45: (-15, 5, -25, 25), 50: (-15, 5, 0, 25),
         55: (-10, -5, 20, 25), 60: (-10, -5, 20, 25), 65: (-10, -5, 25, 25)}
#: The farthest a hip folded and swung back through them, deg (sitting down; a leg laid on the bed).
TURNED = (-142.0, 66.0)
#: The most a hip carried through those scenarios, a peak at least 2 ms above its half: the force along x, y and z,
#: N (1048 sitting down, 1326 and 1074 lying down on the bed), and the moment about them, N m (215 about the pitch's
#: axis on the bed - past its gearbox's shock, its rotor's inertia through the gears -, 97 about the yaw's sitting
#: down, 136 about the roll's strafing).
MEASURED = {'force': (1048.0, 1326.0, 1074.0), 'moment': (215.0, 97.0, 136.0)}
#: How much higher the yaw drive stands than the build lays it, m; the roll drive's axis off the roll's, (x, y) m -
#: the gears' centres apart; the gears: module mm, the pinion's teeth, the sector's as a whole wheel's.
RAISE_M, ROLL_AT, GEARS = 0.013, (0.0, 0.080), (4.0, 15, 25)
#: Steel: the roll's plate's thickness and the air it and the pinion keep, m; the pin's outside and bore radii, m;
#: a drawn-cup needle bearing HK 3020: its outside radius and length, m, its static rating, N; the housing's wall, m.
PLATE_M, AIR_M, KINGPIN, NEEDLE, HOUSED_M = 0.010, 0.002, (0.015, 0.009), (0.0185, 0.020, 27.5e3), 0.006
#: A case-hardened steel's tooth root at the margin, a quenched and tempered steel's yield, MPa.
ROOT_MPA, YIELD_MPA = 500.0, 650.0
#: The cradle's saddle behind its band at the roll's axis, its half height, m; the plate's stem's half width.
SADDLE_M, SADDLE_Y, STEM_M = 0.003, 0.044, 0.012
#: The crown under the yaw's hub: thickness, the hub's flange's radius, m; the cheeks on it either side of the yaw's
#: ring: from x, to x, up to y; how far ahead of the yaw's axis the crown reaches, round its hub's screws' heads.
CROWN_M, HUB_R, CHEEK, CROWN_Z = 0.014, 0.028, (0.036, 0.066, 0.104), 0.0245
#: The thigh's neck: the plate over the pitch's hub's face (thickness, radius); the arm's inner face off the
#: gearbox's end (inboard of it: past the band), its thickness, where it widens from its hub's round (the reach
#: from the hip's axis) and its z behind and ahead of that axis; the jog's reach (from, to), its z and how
#: much of its back top corner is cut off; m.
HUB_PLATE, ARM_X, ARM_M, ARM_FROM, ARM_Z = (0.014, 0.036), -0.004, 0.030, 0.058, (-0.058, 0.006)
JOG, JOG_Z, JOG_CUT = (0.106, 0.156), (-0.034, 0.0), 0.014
#: The arm's back cut away at its foot, where the ankle's pre-curved leaf lies with the knee folded to its travel's
#: end: how far ahead of the arm's back the cut stands at the foot and how far up the arm it runs out, m. Measured
#: with the arm uncut, the knee at 155 deg (`leg_clamped.folded`): the leaf's front 12.3 mm ahead of the arm's back
#: at its foot, out of that back 29 mm up, the leaf across the arm's whole width - 12 mm inside it, 2.3 mm clear at
#: 152.5; the cut that line and 3 mm, crawling back turning the knee 0.2 deg past its travel at 5.8 mm a degree.
ARM_CUT = (0.016, 0.038)


def loads():
    """The hip's loads at the margin: {'force': (x, y, z) N, 'moment': (x, y, z) N mm}, each moment no less than its
    drive's own peak."""
    from machine.body import drives, structural
    k = structural.MARGIN
    peaks = [drives.peak(j) for j in ('hip', 'hip_yaw', 'hip_roll')]
    return {'force': tuple(k * f for f in MEASURED['force']),
            'moment': tuple(k * 1e3 * max(got, own) for got, own in zip(MEASURED['moment'], peaks))}


def _cans(joint):
    """{part: (from, to, radius)} of a drive's stack along its own axis, m, as `skeleton.cans` lays it."""
    from machine.body import drives, skeleton
    letter = {'left_hip': 0, 'left_hip_yaw': 1, 'left_hip_roll': 2}[joint]
    parts = [(float(at[letter]) - half, float(at[letter]) + half, float(r)) for j, _rides, at, _axis, r, half
             in skeleton.cans() if j == joint]
    return {p: s for (p, *_), s in zip(drives.along(joint), parts)}


def stacks():
    """{'pitch', 'yaw', 'roll': {part: (from, to, radius)}}, m: the pitch drive along x where the build lays it; the
    yaw drive along y over the hip's middle, RAISE_M up; the roll drive along z behind the cradle's plate, its
    gearbox's face AIR_M behind it."""
    from machine.body.figure import SEGMENTS
    pitch, yaw, roll = (_cans('left_' + j) for j in ('hip', 'hip_yaw', 'hip_roll'))
    drop = next(s for s in SEGMENTS if s[0] == 'left_thigh')[3][1]            # the hip's middle in the pelvis's frame
    yaw = {p: (a - drop + RAISE_M, b - drop + RAISE_M, r) for p, (a, b, r) in yaw.items()}
    face = plate()[1] - AIR_M
    back = max(b for _a, b, _r in roll.values())
    return {'pitch': pitch, 'yaw': yaw, 'roll': {p: (a - back + face, b - back + face, r) for p, (a, b, r) in roll.items()}}


def walls():
    """{'cradle', 'thigh', 'roll': a slit band's wall, m}: round the pitch drive's motor and its output ring, each
    gripping the pitch's moment; round the roll drive's motor, gripping its own torque."""
    from machine.body import drives
    pitch, _yaw, roll = (_cans('left_' + j) for j in ('hip', 'hip_yaw', 'hip_roll'))
    swing = loads()['moment'][0]
    (m0, m1, can), (g0, g1, ring) = pitch['motor'], pitch['gear']
    split = g0 + em.OUTPUT_SHARE * (g1 - g0)
    own = ks.torque('hip_roll') * drives.ratio('hip') / drives.ratio('hip_roll')   # its gearbox's, before the gears
    return {'cradle': 1e-3 * ks.band(swing, 1e3 * can, 1e3 * (m1 - m0 - 0.004)),
            'thigh': 1e-3 * ks.band(swing, 1e3 * ring, 1e3 * (g1 - split - 0.0025)),
            'roll': 1e-3 * ks.band(own, 1e3 * roll['motor'][2], 1e3 * (roll['motor'][1] - roll['motor'][0] - 0.004))}


def plate():
    """(z of its front face, z of its back): the roll's steel plate behind the cradle's saddle, m."""
    can = _cans('left_hip')['motor'][2]
    front = -(can + walls()['cradle'] + SADDLE_M)
    return front, front - PLATE_M


def teeth():
    """(pinion's pitch radius, sector's, tip height), m: the roll's gears, their centres ROLL_AT apart."""
    module, small, big = GEARS
    assert abs(0.5e-3 * module * (small + big) - math.hypot(*ROLL_AT)) < 1e-9, 'the gears mesh at their centres'
    return 0.5e-3 * module * small, 0.5e-3 * module * big, 1e-3 * module


def needles():
    """((z from, to) of the front needle bearing, of the back one), m: the front right behind the roll drive's
    gearbox's face, the back at the roll drive's board's far end - as far apart as the roll drive is long."""
    roll = stacks()['roll']
    front, back = max(b for _a, b, _r in roll.values()), min(a for a, _b, _r in roll.values())
    return (front - AIR_M - NEEDLE[1], front - AIR_M), (back, back + NEEDLE[1])


def checks():
    """{member: (its stress or load, what it may bear)} at the hip's loads, MPa or N: the arm and the jog bent and
    twisted at once, the cheeks bent, the hub's screws' tension against their preload, the plate's, each needle
    bearing's load against its static rating, the pin bent, the pinion's tooth's root."""
    from machine.body import structural
    ld, allow = loads(), structural.PAHT_CF[2]
    (fx, fy, fz), (mx, my, mz) = ld['force'], ld['moment']
    pitch, yaw = stacks()['pitch'], stacks()['yaw']
    out = {}
    wide, thick = 1e3 * (ARM_Z[1] - ARM_Z[0]), 1e3 * ARM_M
    bent = 6.0 * mz / (wide * thick ** 2) + 6.0 * mx / (thick * wide ** 2) + fy / (wide * thick)
    shear = my * (3.0 * wide + 1.8 * thick) / (wide * wide * thick * thick)
    out['arm'] = (bent / 2.0 + math.hypot(bent / 2.0, shear), allow)
    reach = 1e3 * (pitch['gear'][1] + ARM_X + 0.5 * ARM_M) + 9.0                # the arm's middle to the tube's line
    deep, across = 1e3 * (JOG[1] - JOG[0]), 1e3 * (JOG_Z[1] - JOG_Z[0])
    out['jog'] = (6.0 * (mz + fy * reach) / (across * deep ** 2) + 6.0 * my / (deep * across ** 2), allow)
    # The arm's foot where its back is cut away deepest (ARM_CUT), beside the jog: the two one section across the
    # tube's line - each (across x, deep z, its middle off that line) -, bent about both and twisted at once.
    past, near = kl.tube()[0] + 1e3 * em.COLLAR_M, reach - 0.5 * thick          # the jog: past the tube's line, to the arm
    slab = ((thick, 1e3 * (JOG_Z[1] - ARM_Z[0] - ARM_CUT[0]), reach), (near + past, across, 0.5 * (near - past)))
    area = sum(b * h for b, h, _x in slab)
    x_mid, z_mid = (sum(b * h * q for b, h, q in rows) / area for rows in (slab, [(b, h, 0.5 * h) for b, h, _x in slab]))
    about_z = sum(h * b ** 3 / 12.0 + b * h * (x - x_mid) ** 2 for b, h, x in slab)
    about_x = sum(b * h ** 3 / 12.0 + b * h * (0.5 * h - z_mid) ** 2 for b, h, _x in slab)
    far_x = max(abs(x + side * 0.5 * b - x_mid) for b, _h, x in slab for side in (-1.0, 1.0))
    far_z = max(z_mid, max(h for _b, h, _x in slab) - z_mid)
    bent = (mz + fy * x_mid) * far_x / about_z + mx * far_z / about_x + fy / area
    twist = [(max(b, h), min(b, h)) for b, h, _x in slab]
    stiff = [a * s ** 3 * (1.0 / 3.0 - 0.21 * s / a * (1.0 - s ** 4 / (12.0 * a ** 4))) for a, s in twist]
    shear = max(my * j / sum(stiff) * (3.0 * a + 1.8 * s) / (a * a * s * s) for (a, s), j in zip(twist, stiff))
    out['arm foot'] = (bent / 2.0 + math.hypot(bent / 2.0, shear), allow)
    up = 1e3 * min(a for a, _b, _r in yaw.values())                             # the hub's face over the hip's middle
    tilt = math.hypot(mx + fz * up, mz + fx * up)                               # what tilts the yaw's hub, N mm
    slab, wide = 1e3 * CROWN_M, 2e3 * CHEEK[1]                                 # the crown and its two cheeks: a section
    rib, each = 1e3 * CHEEK[2] - up, 1e3 * (CHEEK[1] - CHEEK[0])
    areas = ((wide * slab, 0.5 * slab), (2.0 * each * rib, slab + 0.5 * rib))
    mid = sum(a * y for a, y in areas) / sum(a for a, _y in areas)
    inertia = (wide * slab ** 3 / 12.0 + areas[0][0] * (mid - areas[0][1]) ** 2
               + 2.0 * each * rib ** 3 / 12.0 + areas[1][0] * (mid - areas[1][1]) ** 2)
    out['cheeks'] = (tilt * max(mid, slab + rib - mid) / inertia, allow)
    count, circle = kl.HUB
    out['hub screws'] = (2.0 * tilt / (count * circle) + fy / count, kl.SCREWS['M5'][0])
    lever = 2e3 * (SADDLE_Y - 0.006)                                            # the plate's screws, top and bottom
    out['plate screws'] = ((mx + fy * 1e3 * abs(plate()[0])) / lever / 2.0, kl.SCREWS['M6'][0])
    (f0, f1), (b0, b1) = needles()
    span, out_by = 1e3 * (0.5 * (f0 + f1) - 0.5 * (b0 + b1)), -1e3 * 0.5 * (f0 + f1)
    front = math.hypot(mx / span + fy * (out_by + span) / span, my / span + fx * (out_by + span) / span)
    back = math.hypot(mx / span + fy * out_by / span, my / span + fx * out_by / span)
    out['front needle'], out['back needle'] = (front, NEEDLE[2]), (back, NEEDLE[2])
    r, bore = 1e3 * KINGPIN[0], 1e3 * KINGPIN[1]
    out['pin'] = (math.hypot(mx + fy * out_by, my + fx * out_by) * 32.0 * 2.0 * r
                  / (math.pi * ((2.0 * r) ** 4 - (2.0 * bore) ** 4)), YIELD_MPA)
    small, _big, _tip = teeth()
    form = 0.29                                                                 # Lewis, 15 teeth
    out['tooth'] = (mz / (1e3 * teeth()[1]) / (1e3 * PLATE_M * GEARS[0] * form), ROOT_MPA)
    return out


def main(argv=None):
    ld = loads()
    print('loads at the margin: force %s N, moment %s N m' % (
        [round(f) for f in ld['force']], [round(1e-3 * m) for m in ld['moment']]))
    print('walls: ' + ', '.join('%s %.1f mm' % (k, 1e3 * v) for k, v in walls().items()))
    for part, stack in stacks().items():
        print('%-5s %s' % (part, '  '.join('%s %.1f..%.1f r %.1f' % (p, 1e3 * a, 1e3 * b, 1e3 * r)
                                             for p, (a, b, r) in stack.items())))
    print('plate z %.1f..%.1f, needles %s mm' % (1e3 * plate()[0], 1e3 * plate()[1],
                                                 [[round(1e3 * v, 1) for v in n] for n in needles()]))
    for member, (got, may) in checks().items():
        print('%-13s %8.1f of %8.1f  x%.2f' % (member, got, may, may / got))
    return 0


if __name__ == '__main__':
    sys.exit(main())
