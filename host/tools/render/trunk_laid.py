"""The trunk, a first draft, laid: a cardan joint at mid back, the calf's drive pair, a rod each.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_laid.py

The trunk as the leg's calf is built, turned up. A classic cardan joint at mid back, RAISED_M over the build's
torso's anchor: its cross's roll journal in the pelvis's tower, its pitch pin in the chest's fork - the build's
order, the roll's axis standing with the pelvis. Under it, on a column off the pelvis, the calf's own two drives
back to back on one axis across the back (`knee_clamped.laid`), their inverters between them; on each drive's hub,
outboard of its gable, a crank; from each crank's pin a rod - a cartridge of leaves, as the calf's - up the back
to a pin on the chest's cross: the two arms the chest's fork reaches back with, a little over the joint. Both
drives one way bow the chest, one against the other lean it; nothing turns the chest about its own length - no
waist's drive in this draft.
All in the joint's frame: its middle the origin, x left, y up, z ahead. `cranked` solves the two cranks at a pose,
`able` tells what the pair turns there - the pitch alone and the roll alone, each drive within its own torque
(`drive`) -, how near a rod pulls to its crank's dead centre and how far the rod leans out of its two pins'
squares; `reached` tells those over the build's whole travel. Drives on the chest's side reach no such travel:
the roll's axis stands with the pelvis, and rods along the chest lose the roll as the chest bows.
A draft's open ends: the rods' ends - a plain fork and eye hold a rod square to its pin, and the roll leans the
rod (`able`'s 'lean') more than the calf's spherical eye gives; leant to its travel's end and bowed back, the low
side's rod runs in over its own drive's gearbox (`trunk_clamped.aired`); the leaves' give along a rod is the
calf's, not laid for the trunk; the neck's and the head's boards' bay stands on the chest's tube where the
pelvis's tower stands, and has to move; the joint stands RAISED_M over where the build's software bends the trunk
and the waist's turn is gone - the build's spec and what runs on it are untouched; no screws, no bus; its prints
lie on their beds (`prints.trunk`) and the column is shaped by its loads (`trunk_shaped`). The tower and the fork,
printed, do not carry a rod's pull at the margin on its least crank lever (3.5 kN): the tower holds the cross's
journal behind it only, the fork's walls take the rod's lean across them at the travel's corner (2.2 kN, 136 MPa
in 13 mm of print) - a yoke either side of the cross, and a chest's cross no lean bends, are the next draft's.
"""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: How far over the build's torso's anchor the joint's middle stands, m: mid back - the pair's cage clear of the
#: hips' yaw drives under it (their tops 121 mm over the pelvis's middle), the cranks' pins swinging 13 mm over
#: those tops at their lowest.
RAISED_M = 0.178
#: A drive's hub (x: its crank's pin's middle, outboard of its gable and its crank's plate; y; z), a crank's
#: radius, m, and its bearing at rest, deg from straight back toward up; the chest's pin's middle (x, y, z), m.
#: A search over these eight (differential evolution, 27 poses of the travel, the calf's 77 N m a drive, the
#: pins no lower than those 13 mm and no farther back than 100 mm) for the most of the least share of the build's
#: 108 N m of pitch and 144 of roll: 0.72 with the chest's pin 5 mm over the joint, 0.69 at 10 mm (here), 0.62 at
#: 20; the joint 18 mm lower 0.62; the cranks' pins in over the gearboxes, as the calf's, a rod's line through
#: its own drive at the travel's ends. A pair on the chest's side (rods down to the pelvis): no geometry reaches
#: the travel with rod ends that tilt 30 deg, 0.45 of it with ends that tilt freely. With the cranks' pins
#: 91 mm out - a rod's end, leant as the roll leans it, clear of its crank's plate - sought again, three seeds
#: alike: 0.69, the chest's pins 6 mm over the joint and 8 mm in from the cranks'.
HUB, CRANK, BALL = (0.091, -0.086, 0.004), (0.078, -44.0), (0.083, 0.006, -0.085)
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
    """(The cross's, the chest's) turns, 3 by 3 each, at the trunk's angles, deg: about z, then about the rolled
    x - a bow ahead positive."""
    import numpy as np
    (cr, sr), (cp, sp) = ((math.cos(math.radians(a)), math.sin(math.radians(a))) for a in (roll, pitch))
    about_z = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    return about_z, about_z @ np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])


def _spoke(bearing):
    """A crank's way from its hub at `bearing`, rad from straight back toward up: a unit (x, y, z)."""
    import numpy as np
    return np.array((0.0, math.sin(bearing), -math.cos(bearing)))


def cranked(roll=0.0, pitch=0.0):
    """[(the crank's bearing, rad, its pin, the chest's pin's middle)] the left's then the right's, the joint's
    frame, the trunk at `roll` and `pitch`, deg: each crank where its rod - as long as at rest - holds it, on the
    rest's side of its dead centre; None where a rod cannot reach."""
    import numpy as np
    chest, (r, rest) = turn(roll, pitch)[1], (CRANK[0], math.radians(CRANK[1]))
    out = []
    for side in (1.0, -1.0):
        hub, ball = np.array(HUB) * (side, 1.0, 1.0), np.array(BALL) * (side, 1.0, 1.0)
        long = float(np.linalg.norm(hub + r * _spoke(rest) - ball))
        was, now = ball - hub, chest @ ball - hub
        way = math.copysign(1.0, math.remainder(rest - math.atan2(was[1], -was[2]), math.tau))
        reach = math.hypot(now[1], now[2])
        share = (r * r + reach * reach + now[0] * now[0] - long * long) / (2.0 * r * reach)
        if abs(share) > 1.0:
            return None
        bearing = math.atan2(now[1], -now[2]) + way * math.acos(share)
        out.append((bearing, hub + r * _spoke(bearing), chest @ ball))
    return out


def able(roll=0.0, pitch=0.0):
    """{'pitch', 'roll': N m, each alone with both drives within `drive`; 'lever': the least share of its crank's
    radius a rod pulls at; 'lean': deg, the most a rod stands out of its crank's pin's square or the chest's
    pin's; 'pull': N, the most a rod bears; 'cranks': deg from rest, the left's and the right's} at a pose, deg;
    None where a rod cannot reach or pulls through its crank's hub."""
    import numpy as np
    got = cranked(roll, pitch)
    if got is None:
        return None
    x, about = np.array((1.0, 0.0, 0.0)), np.array((0.0, 0.0, 1.0))
    axis, most, rest = turn(roll, pitch)[0] @ x, drive(), math.radians(CRANK[1])
    turns, lever, lean, pull, cranks = [], 1.0, 0.0, 0.0, []
    for side, (bearing, pin, ball) in zip((1.0, -1.0), got):
        along = (ball - pin) / np.linalg.norm(ball - pin)
        arm = float(np.cross(pin - np.array(HUB) * (side, 1.0, 1.0), along) @ x)      # the crank's torque a newton
        if abs(arm) < 1e-9:
            return None
        turns.append((float(np.cross(ball, along) @ axis) / arm, float(np.cross(ball, along) @ about) / arm))
        lever, pull = min(lever, abs(arm) / CRANK[0]), max(pull, most / abs(arm))
        lean = max(lean, *(abs(math.degrees(math.asin(float(along @ pin_way)))) for pin_way in (x, axis)))
        cranks.append(math.degrees(math.remainder(bearing - rest, math.tau)))
    share = np.linalg.inv(np.array(turns).T)             # the drives' torques a newton metre of pitch, of roll
    return {'pitch': most / float(np.abs(share[:, 0]).max()), 'roll': most / float(np.abs(share[:, 1]).max()),
            'lever': lever, 'lean': lean, 'pull': pull, 'cranks': cranks}


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
