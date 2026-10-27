#!/usr/bin/env python3
"""Her posture as what bears her holds her (`modal.poses`, `modal.seat`; the kernel's
`canonical_posture_bundle.lisp`): the seat's chain against her landing, the blend over what bears her,
the atlas read, her ask sat on the chair."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def _seated(hips, behind, trunk):
    """(y, fr, knee, ankle) of her sensed sat, her soles flat on the floor, her hip joints `hips` m up and
    `behind` m behind her ankles (`seated.legs`), her pelvis pitched `trunk` rad forward, her way along
    the world's z."""
    import numpy as np
    from machine import modal, seated
    from machine.figure import ANKLE_H
    from machine.leap import MIDDLE
    knee, ankle = seated.legs(hips, behind)
    hx, hy = seated.hips(knee, ankle)
    y, fr = np.zeros(modal.SENSED), np.zeros(modal.FRAME)
    fr[modal.ROT] = (0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    y[modal.QUAT] = (math.cos(0.5 * trunk), math.sin(0.5 * trunk), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    y[modal.SOLE] = (0.09, 0.0, 0.0, -0.09, 0.0, 0.0)
    y[modal.HIP] = (0.09, ANKLE_H + hy, hx - MIDDLE, -0.09, ANKLE_H + hy, hx - MIDDLE)
    return y, fr, knee, ankle


def test_the_seat_holds_her_legs_as_she_sits(report):
    """`modal.seat`: her hips where a seat lands them (`seated.legs`: 0.40-0.55 m up, 0.10-0.30 m behind
    her ankles), her pelvis pitched -0.3 to 0.3 rad, her knee as landed - each leg's hip and ankle the
    landing's (`seated.posed`)."""
    import numpy as np
    from machine import modal, rounds, seated
    worst = 0.0
    for hips in (0.40, 0.47, 0.55):
        for behind in (0.10, 0.20, 0.30):
            for trunk in (-0.3, 0.0, 0.3):
                y, fr, knee, ankle = _seated(hips, behind, trunk)
                pose = np.zeros(len(rounds.DRIVEN))
                pose[modal.CHAIN[:, 1]] = knee
                deg = seated.posed(knee, ankle, trunk)
                want = np.radians([[deg[side + j] for j in ('hip', 'knee', 'ankle')] for side in ('left_', 'right_')])
                worst = max(worst, float(np.abs(modal.seat(y, fr, pose) - want).max()))
    report.check("sat, each leg's hip and ankle the landing's, 27 seats and pitches", worst < 1e-9, '%.1e rad' % worst)


def test_her_legs_as_what_bears_her_holds_them(report):
    """`modal.poses` over what bears her (`ortho.MANIFOLDS`): her soles alone, the crouch's pose bit for
    bit (her walk's), her side, her back and her hands the same; the seat alone, `modal.seat`'s legs, the
    rest of her the crouch's; half on each, half way; her knees as asked whatever bears her; a let-go
    leg's hip level less her pelvis's pitch (HUNG), the other's as it was."""
    import numpy as np
    from machine import modal, ortho, rounds
    names = rounds.DRIVEN
    y, fr, knee, _ankle = _seated(0.47, 0.2, 0.1)
    bend, arms, reaches, folds = columns = (modal.crouch(names), modal.swing(names), modal.reach(names), modal.fold(names))
    posture, knees = np.linspace(-0.3, 0.3, len(names)), tuple(modal.CHAIN[:, 1])
    posture[list(knees)] = 0.0
    a = np.zeros(modal.ASK)
    a[modal.BEND], a[modal.SWUNG], a[modal.REACHED], a[modal.ELBOWED] = knee, 0.1, -0.2, 0.3
    crouch = posture + bend * a[modal.BEND] + arms * a[modal.SWUNG] + reaches * a[modal.REACHED] + folds * a[modal.ELBOWED]
    hot, seat_at = np.eye(len(ortho.MANIFOLDS)), ortho.MANIFOLDS.index('ischial')

    def posed(w, let_go=(0.0, 0.0)):
        a[modal.LET_GO] = let_go
        return modal.poses(a, y, fr, posture, *columns, knees, w)[1]

    seat = modal.seat(y, fr, crouch)
    alone = {m: posed(hot[k]) for k, m in enumerate(ortho.MANIFOLDS)}
    rest = np.ones(len(names), bool)
    rest[modal.CHAIN.ravel()] = False
    report.check('her soles, her side, her back and her hands alone: the crouch bit for bit',
                 all(np.array_equal(alone[m], crouch) for m in ('plantar', 'trochanter', 'dorsal', 'palmar')))
    sat = alone['ischial']
    report.check("the seat alone: her legs the seat's, her knees and the rest of her the crouch's",
                 np.allclose(sat[modal.CHAIN], seat, rtol=0.0, atol=1e-12) and np.array_equal(sat[rest], crouch[rest])
                 and np.array_equal(sat[list(knees)], crouch[list(knees)]),
                 'legs %.1e rad off' % float(np.abs(sat[modal.CHAIN] - seat).max()))
    half = posed(0.5 * (hot[0] + hot[seat_at]))
    report.check('half on her soles and half on the seat: her legs half way',
                 np.allclose(half[modal.CHAIN], 0.5 * (crouch[modal.CHAIN] + seat), rtol=0.0, atol=1e-12))
    hung = posed(hot[seat_at], (1.0, 0.0))
    want = modal.HUNG + modal.HUNG_BY * modal.pitched(y, fr[modal.ROT].reshape(3, 3))[0]
    report.check("a leg let go: its hip level less her pelvis's pitch, the other's the seat's",
                 np.allclose(hung[modal.HIPS[0]], want, rtol=0.0, atol=1e-12)
                 and np.array_equal(hung[modal.HIPS[1]], sat[modal.HIPS[1]]),
                 '%s for %s' % (np.round(hung[modal.HIPS[0]], 4), np.round(want, 4)))


def test_the_posture_atlas_reads_whole(report):
    """The kernel's 1-jet posture atlas (`canonical_posture_bundle.lisp`) and its specializers
    (`posture_specializers.lisp`): seven postures, each its origin, its three modal columns and its hang
    law - the seat's shank plumb, its soles unloaded -; the blend, the reconstruction, the synthesizer
    and the five specializers its stages."""
    from machine import dsp
    got = dsp.read(dsp.KERNEL)
    atlas = got['canonical-posture-soa-atlas']
    layout, postures = atlas['memory-layout'], atlas['postures']
    report.check('seven postures, each its origin, three columns and a hang law',
                 len(postures) == layout['num-canonical-postures'] == 7.0
                 and all(len(p['modal-basis-columns']) == layout['num-modal-columns'] and 'origin-q0' in p
                         and 'gravity-hang-law' in p for p in postures.values()), str(sorted(postures)))
    hang = postures['ischial-seated']['gravity-hang-law']
    report.check("the seat's hang: its shank plumb, on its soles unloaded",
                 hang['enabled'] is True and hang['dangling-q0']['tibiofemoral'] == 1.5708
                 and ':plantar-bilateral' in hang['trigger-on-unloaded-patch'], str(hang))
    stages = ('blend-manifold-posture-jet', 'reconstruct-modal-pose', 'xform-posture-manifold-synthesizer',
              'specialize-posture-dangling-crural-hang', 'specialize-posture-appendage-brace',
              'specialize-posture-seated-archetype', 'specialize-posture-recumbent-archetype',
              'specialize-posture-protective-breakfall')
    report.check('the blend, the reconstruction, the synthesizer and the five specializers its stages',
                 all(isinstance(got.get(k), dsp.Stage) for k in stages),
                 str([k for k in stages if not isinstance(got.get(k), dsp.Stage)]))


def test_sat_her_legs_are_asked_as_the_seat_holds_them(report):
    """Down onto the chair and up (`chair`), sat - the seat 0.8 of what bears her or more, felt -: her ask's hips
    and ankles (`modal.poses`) nearer her legs than half what the crouch's column asks of her knee
    (`modal.crouch`, hip and ankle half the knee: 25-29 deg off them, findings/seat.md)."""
    import numpy as np
    from machine import modal, ortho
    from tools.sim import scenarios
    seat, legs = ortho.MANIFOLDS.index('ischial'), modal.CHAIN[:, [0, 2]]
    knees, offs = modal.CHAIN[:, [1, 1]], []

    def laid(s):
        g = s['g']
        if g['n'] % g['every'] == 1 and 'posture' in g.get('ask', {}) and g['weighed_w'][seat] >= 0.8:
            q, posture = np.asarray(g['s']['q']), g['posture']
            crouch = posture[legs] + g['bend'][legs] * (q[knees] - posture[knees])
            offs.append((np.abs(np.asarray(g['ask']['posture'])[legs] - q[legs]), np.abs(crouch - q[legs])))
    got = scenarios.ran('chair', laid=laid)
    asked, crouched = (np.degrees(np.mean([o[k] for o in offs], axis=0)) if offs else np.full((2, 2), np.nan) for k in (0, 1))
    report.check("sat, her hips and ankles asked nearer her legs than half the crouch's, and she rose",
                 got['met'] and bool(np.all(asked < 0.5 * crouched)),
                 '%d passes sat; asked off (hip, ankle) %s deg, the crouch %s' % (
                     len(offs), np.round(asked, 1).tolist(), np.round(crouched, 1).tolist()))


ROSTER = [test_the_seat_holds_her_legs_as_she_sits, test_her_legs_as_what_bears_her_holds_them,
          test_the_posture_atlas_reads_whole, test_sat_her_legs_are_asked_as_the_seat_holds_them]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
