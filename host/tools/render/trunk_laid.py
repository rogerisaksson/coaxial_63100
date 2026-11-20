"""The trunk, a first draft, laid: the calf's drive pair on the chest over one cardan joint.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_laid.py

The trunk as the leg's calf is built: the calf's two drives back to back on one axis across the back
(`knee_clamped.laid`), their inverters between them, on the chest; under them one classic cardan joint, low by the
hips - its cross's pitch pin in the pelvis's yoke, its roll journals in the chest's, each yoke holding its pins
either side -; on each drive's output ring a lever clamped, as the calf's, its pin out past the gable
(`trunk_rods.lever`); from each lever's pin a rod - a cartridge of leaves, as the calf's - down the back past the
joint to a pin on the pelvis's cross. Both drives one way bow the
chest, one against the other lean it; nothing turns the chest about its own length - no waist's drive in this
draft. The cross is turned a quarter from the build's order, the pitch before the roll: the drives ride the chest
and the roll's axis turns with them, so no bow takes the roll's lever from the rods.
All in the joint's frame: its middle the origin, x left, y up, z ahead; the chest's own frame turned with it
(`turn`). `cranked` solves the two cranks at a pose, `able` tells what the pair turns there - the pitch alone and
the roll alone, each drive within its own torque (`drive`) -, how near a rod pulls to its crank's dead centre and
how far the rod leans out of its two pins' squares; `reached` tells those over the build's whole travel; `pulled`
the rods' pulls on the pelvis's pins that hold a moment of the chest's.
A draft's open ends: leant 35 deg and bowed back 30, near its crank's dead centre a rod's leaves pass the drive's
output ring 2.0 mm off and its lever's band 2.2 - the search below kept the rod's line their half width off the
drive's drum, not their width off its end nor their bow, so the leaves were narrowed and the band set in from the
gable (`trunk_rods`); kept so, the search's least share falls to 0.51 -; leant 35 deg the chest's rear yoke's arm's
edge comes within 0.1 mm of the pelvis's yoke, into its notch at the plate's inner edge, and the pair's cage within
0.5 mm, onto the plate's outer face by its eye - the chest's yokes and the cage not yet shaped by their loads -:
under the 2 mm kept everywhere else (`trunk_clamped.aired`); the pelvis's yoke stands on the web alone, 18 mm deep
between the hips' yokes' sweeps, and the trunk's moment passes through it; the leaves' give along a rod is the
calf's, not laid for the trunk; the cardan's cross's block and a rod's two collars are steel; the joint stands
RAISED_M over where the build's software bends the trunk, the pitch before the roll, the waist's turn gone - the
build's spec and what runs on it are untouched; no screws, no bus.
"""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: How far over the build's torso's anchor the joint's middle stands, m: 175 mm over the pelvis's middle. From
#: there the chest's drives clear the hips' yaw drives' tops bowed 85 deg and leant 35 (5.5 mm; 0.6 at 160 mm),
#: the pelvis's pins and the rods' low ends the hips' links by 32 mm; and frame by frame through the twelve
#: scenarios that bow or sit, the drives keep 89 mm or more off the hips' links as each scenario holds them.
RAISED_M = 0.055
#: A drive's hub on the chest (x: its crank's pin's middle, outboard of its gable and its crank's plate; y; z), a
#: crank's radius, m, and its bearing at rest, deg from straight back toward up; the pelvis's pin's middle (x, y,
#: z), m. A search over these eight (differential evolution, 27 poses of the travel, the calf's 77 N m a drive)
#: for the most of the least share of the build's 108 N m of pitch and 144 of roll, laid with the drives under the
#: joint and the rods up - this layout is its mirror, the cross turned with it: the same pitch, roll, lever, pull
#: and lean at every pose, to the last bit. Free, 0.69; each rod's line kept its leaves' half width and 2 mm out
#: from its own drive's drum and crank where it passed beside them - leant 35 deg and bowed back 30 it swung in over
#: them -, the hubs 87 mm over the joint - the cage then keeps off the pelvis's eyes leant -, 0.625, four seeds
#: alike, on the edge of the least lever. A pair
#: whose rods' far end rides the body the roll's axis does not stand with reaches no such travel: the bow takes
#: the roll's lever.
HUB, CRANK, BALL = (0.093, 0.087, 0.003), (0.080, 37.5), (0.095, -0.015, -0.085)
#: The least share of its crank's radius a rod pulls at: 25 deg off the crank's dead centre.
LEVER = 0.42


def joint():
    """(The joint's middle, the left hip's) in the pelvis's frame, m."""
    from machine.body.figure import SEGMENTS
    at = {s[0]: s[3] for s in SEGMENTS}
    return (at['torso'][0], at['torso'][1] + RAISED_M, at['torso'][2]), at['left_thigh']


def drive():
    """A drive's own torque, N m: half the ankle's pitch, which the calf's two share."""
    from machine.body import drives
    return 0.5 * drives.peak('left_ankle')


def travel():
    """((the roll's least, its most), (the pitch's)), deg: the build's own."""
    from machine.body import drives
    return drives.TRAVEL['spine_roll'], drives.TRAVEL['spine']


def turn(roll=0.0, pitch=0.0):
    """(The cross's, the chest's) turns, 3 by 3 each, at the trunk's angles, deg: about x - a bow ahead positive -,
    then about the cross's z."""
    import numpy as np
    (cr, sr), (cp, sp) = ((math.cos(math.radians(a)), math.sin(math.radians(a))) for a in (roll, pitch))
    about_x = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])
    return about_x, about_x @ np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])


def _spoke(bearing):
    """A crank's way from its hub at `bearing`, rad from straight back toward up: a unit (x, y, z)."""
    import numpy as np
    return np.array((0.0, math.sin(bearing), -math.cos(bearing)))


def cranked(roll=0.0, pitch=0.0):
    """[(the crank's bearing, rad, its pin, the pelvis's pin)] the left's then the right's, the joint's frame, the
    trunk at `roll` and `pitch`, deg: each crank where its rod - as long as at rest - holds it, on the rest's side
    of its dead centre; None where a rod cannot reach."""
    import numpy as np
    chest, (r, rest) = turn(roll, pitch)[1], (CRANK[0], math.radians(CRANK[1]))
    out = []
    for side in (1.0, -1.0):
        hub, ball = np.array(HUB) * (side, 1.0, 1.0), np.array(BALL) * (side, 1.0, 1.0)
        long = float(np.linalg.norm(hub + r * _spoke(rest) - ball))
        was, now = ball - hub, chest.T @ ball - hub                  # the pelvis's pin seen from the chest
        way = math.copysign(1.0, math.remainder(rest - math.atan2(was[1], -was[2]), math.tau))
        reach = math.hypot(now[1], now[2])
        share = (r * r + reach * reach + now[0] * now[0] - long * long) / (2.0 * r * reach)
        if abs(share) > 1.0:
            return None
        bearing = math.atan2(now[1], -now[2]) + way * math.acos(share)
        out.append((bearing, chest @ (hub + r * _spoke(bearing)), ball))
    return out


def _statics(roll, pitch):
    """(cranked's, each rod's way from its pelvis's pin to its crank's, each crank's torque a newton of its rod,
    [(the pitch, the roll) a newton of each rod turns]) at a pose, deg; None where a rod cannot reach or pulls
    through its crank's hub."""
    import numpy as np
    got = cranked(roll, pitch)
    if got is None:
        return None
    x = np.array((1.0, 0.0, 0.0))
    cross, chest = turn(roll, pitch)
    axis, own = cross @ np.array((0.0, 0.0, 1.0)), chest @ x
    ways, arms, turns = [], [], []
    for side, (_bearing, pin, ball) in zip((1.0, -1.0), got):
        along = (pin - ball) / np.linalg.norm(pin - ball)
        arm = float(np.cross(pin - chest @ (np.array(HUB) * (side, 1.0, 1.0)), along) @ own)
        if abs(arm) < 1e-9:
            return None
        ways.append(along)
        arms.append(arm)
        turns.append((float(np.cross(ball, along) @ x), float(np.cross(ball, along) @ axis)))
    return got, ways, arms, turns


def able(roll=0.0, pitch=0.0):
    """{'pitch', 'roll': N m, each alone with both drives within `drive`; 'lever': the least share of its crank's
    radius a rod pulls at; 'lean': deg, the most a rod stands out of its crank's pin's square or the pelvis's
    pin's; 'pull': N, the most a rod bears; 'cranks': deg from rest, the left's and the right's} at a pose, deg;
    None where a rod cannot reach or pulls through its crank's hub."""
    import numpy as np
    got = _statics(roll, pitch)
    if got is None:
        return None
    (cranks, ways, arms, turns), most = got, drive()
    own, x, rest = turn(roll, pitch)[1] @ np.array((1.0, 0.0, 0.0)), np.array((1.0, 0.0, 0.0)), math.radians(CRANK[1])
    share = np.linalg.inv(np.array([(p / a, r / a) for (p, r), a in zip(turns, arms)]).T)
    return {'pitch': most / float(np.abs(share[:, 0]).max()), 'roll': most / float(np.abs(share[:, 1]).max()),
            'lever': min(abs(a) for a in arms) / CRANK[0], 'pull': max(most / abs(a) for a in arms),
            'lean': max(abs(math.degrees(math.asin(float(w @ way)))) for w in ways for way in (x, own)),
            'cranks': [math.degrees(math.remainder(b - rest, math.tau)) for b, _p, _ball in cranks]}


def pulled(roll, pitch, moment):
    """[(the force, N, the left rod pulls the pelvis's pin with), (the right's)] in the joint's frame, the trunk at
    `roll` and `pitch`, deg, the rods holding the chest's `moment` (pitch, roll), N m - each at most a drive's own
    torque at the margin on its crank (`structural.MARGIN`), the two scaled back together where one would pass it;
    None where a rod cannot reach."""
    import numpy as np
    from machine.body import structural
    got = _statics(roll, pitch)
    if got is None:
        return None
    _cranks, ways, arms, turns = got
    pulls = np.linalg.solve(np.array(turns).T, np.asarray(moment, float))
    most = structural.MARGIN * drive()
    pulls = pulls * min(1.0, min(most / abs(a) / max(abs(f), 1e-9) for f, a in zip(pulls, arms)))
    return [f * w for f, w in zip(pulls, ways)]


def reached(steps=(5, 9)):
    """{(roll, pitch): `able` there} over the build's travel, deg: `steps` poses across the roll's, across the
    pitch's."""
    import numpy as np
    (r0, r1), (p0, p1) = travel()
    return {(float(a), float(b)): able(float(a), float(b))
            for a in np.linspace(r0, r1, steps[0]) for b in np.linspace(p0, p1, steps[1])}


def main():
    from machine.body import drives
    got = reached()
    print('the joint %.0f mm over the pelvis\'s middle; a drive %.1f N m; the build\'s spine %.0f N m of pitch, %.0f of roll'
          % (1e3 * joint()[0][1], drive(), drives.peak('spine'), drives.peak('spine_roll')))
    pitches = sorted({p for _r, p in got})
    print('roll   ' + ''.join('%14s' % ('pitch %.0f' % p) for p in pitches) + '    (pitch / roll alone, N m)')
    for roll in sorted({r for r, _p in got}):
        print('%5.1f  ' % roll + ''.join('%14s' % ('none' if v is None else '%.0f / %.0f' % (v['pitch'], v['roll']))
                                         for v in (got[(roll, p)] for p in pitches)))
    held = [v for v in got.values() if v is not None]
    print('%d of %d poses reached; the least pitch %.0f N m, roll %.0f; a rod pulls %.0f N at the most, at %.2f of its '
          'crank\'s radius at the least; leans %.0f deg at the most; the cranks turn %.0f to %.0f deg from rest'
          % (len(held), len(got), min(v['pitch'] for v in held), min(v['roll'] for v in held),
             max(v['pull'] for v in held), min(v['lever'] for v in held), max(v['lean'] for v in held),
             min(min(v['cranks']) for v in held), max(max(v['cranks']) for v in held)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
