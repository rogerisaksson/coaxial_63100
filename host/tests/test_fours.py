#!/usr/bin/env python3
"""Her limbs on all fours (`machine.moves.fours`; the kernel's multimodal_*.lisp): the texts read whole, the torus clock
locked to each symmetry and turned between them, the fifth limb bearing through her limbs' swing, her legs levelled
to her arms on her figure, the truss meeting her weight."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The texts' symmetries (`specialize-quad-phase-topology`), its limbs [LH LF RH RF].
SYMMETRIES = (':crawl-lateral-sequence', ':diagonal-trot', ':ipsilateral-pace', ':synchronous-bound')
#: A tick, s; a phase met, rad.
DT, MET = 1e-3, 1e-6


def _gait(symmetry):
    from machine.texts import dsp, kernel
    return dsp.value(['specialize-quad-phase-topology', 'nominal-biped-gait', ':gait-symmetry', symmetry], kernel.read())


def _turned(phi, lag, hz, k, seconds):
    """`phi` after `seconds` of the clock, and its phases each tick (ticks, n)."""
    import numpy as np
    from machine.moves import fours
    out = []
    for _ in range(int(round(seconds / DT))):
        phi = fours.clock(phi, lag, hz, k, DT)
        out.append(phi)
    return phi, np.array(out)


def _off(phi, lag):
    """How far, rad, each limb is from lagging the first by its lag."""
    import numpy as np
    return np.abs(np.angle(np.exp(1j * (phi + lag - phi[0] - lag[0]))))


def test_the_texts_read_whole(report):
    """The three texts read with the kernel (`dsp.read`): the two postures their origins and three columns each; the
    clock, the equalizer, the truss, the fifth limb, the two gaits and five specializers stages; the schedule a
    trot, a morph, a crawl and the crab's scoot, each its clock as the texts compose it."""
    from machine.texts import dsp, kernel
    from machine.moves import fours
    got = kernel.read()
    postures = got['quadrupedal-posture-extensions']['postures']
    report.check('two postures, each its origin and three columns',
                 sorted(postures) == ['pronograde-quadruped', 'supine-inverted-quadruped']
                 and all(len(p['modal-basis-columns']) == 3 and 'origin-q0' in p for p in postures.values()))
    stages = ('configure-4torus-symplectic-clock', 'equalize-heterogeneous-limb-kinematics',
              'project-virtual-truss-internal-wrenches', 'couple-fifth-limb-axial-peristalsis',
              'xform-gait-pronograde-quadruped', 'xform-gait-supine-inverted-crab', 'specialize-quad-phase-topology',
              'specialize-pronograde-contact-interface', 'specialize-supine-peristaltic-underbed-scoot',
              'specialize-quad-steep-grade-scramble', 'specialize-transition-biped-to-quadruped',
              'patch-pronograde-dog-trot', 'patch-low-table-hands-and-knees-crawl', 'patch-supine-crab-and-underbed-scoot')
    report.check('the core, the gaits, the specializers and the patches its stages',
                 all(isinstance(got.get(k), dsp.Stage) for k in stages),
                 str([k for k in stages if not isinstance(got.get(k), dsp.Stage)]))
    plan = dsp.schedule(got['compile-multimodal-locomotion-soa'])
    clocks = [(a, b, *(fours.clock_of(g)[:2] if isinstance(g, dict) else ('morph', g.bars))) for a, b, g in plan]
    crab = plan[3][2]
    want = [(0.0, 8.0, (0.0, 0.5, 0.5, 0.0), 0.52), (8.0, 10.0, 'morph', 2.0),
            (10.0, 18.0, (0.0, 0.25, 0.5, 0.75), 0.78), (18.0, 26.0, (0.0, 0.5, 0.5, 0.0), 0.52)]
    said = [(a, b, tuple(round(x / fours.TAU, 6) for x in c) if not isinstance(c, str) else c, d) for a, b, c, d in clocks]
    report.check('the schedule: a trot, a morph, a crawl, the crab; the crab its fifth limb told',
                 said == want and crab['axial-peristalsis']['ground-dwell-duty'] == 0.48
                 and crab['axial-peristalsis']['simultaneous-limb-advance-on-axial-stance'] is True,
                 str(said))


def test_each_symmetry_locks_from_anywhere(report):
    """The torus clock (`fours.clock`, the text's coupling 8.5/s) from 64 random phases a symmetry: each limb within
    1e-6 rad of its lag; then a turn of it bears as the symmetry pairs her limbs - the crawl never fewer than three,
    landing left hind, left fore, right hind, right fore; the trot's diagonals as one, the pace's sides as one, the
    bound's girdles a twentieth of a turn apart."""
    import numpy as np
    from machine.moves import fours
    rng = np.random.default_rng(0)
    for symmetry in SYMMETRIES:
        lag, duty, _bpm, k = fours.clock_of(_gait(symmetry))
        worst = slowest = 0.0
        for _ in range(64):
            phi, path = _turned(rng.uniform(0.0, fours.TAU, 4), lag, 1.0, k, 1.5)
            met = [n for n, p in enumerate(path) if _off(p, lag).max() < MET]
            worst, slowest = max(worst, _off(phi, lag).max()), max(slowest, DT * (met[0] + 1) if met else math.inf)
        _phi, path = _turned(phi, lag, 1.0, k, 1.0)
        on = np.array([fours.bearing(p, duty) for p in path])
        lands = np.argmax((np.diff(on, axis=0) > 0), axis=0)
        paired = {':crawl-lateral-sequence': on.sum(axis=1).min() == 3
                  and list(np.argsort((lands - lands[0]) % len(on))) == [0, 1, 2, 3],
                  ':diagonal-trot': np.array_equal(on[:, 0], on[:, 3]) and np.array_equal(on[:, 1], on[:, 2]),
                  ':ipsilateral-pace': np.array_equal(on[:, 0], on[:, 1]) and np.array_equal(on[:, 2], on[:, 3]),
                  ':synchronous-bound': abs((lands[2] - lands[0]) * DT - 0.05) < 2 * DT}[symmetry]
        report.check('%s: locked in %.2f s at most, %.0e rad off; it bears as it pairs her limbs' % (
            symmetry[1:], slowest, worst), worst < MET and bool(paired),
            'bearing %d-%d limbs, duty %.2f' % (on.sum(axis=1).min(), on.sum(axis=1).max(), duty))


def test_a_morph_turns_her_lags_continuously(report):
    """The schedule's morph, the trot to the crawl over its 2 bars (`phase-locked-multiband-morph`, its s-curve), her
    lags turned along the torus's shortest arc (`fours.toward`) and her duty ramped under the clock, a beat half a
    turn: no limb's phase steps past twice what the clock turns it in a tick, her limbs trail their lags by at most
    the fastest lag's rate over the clock's pull (4 x 8.5/s), and two or more bear throughout."""
    import numpy as np
    from machine.texts import dsp, kernel
    from machine.moves import fours
    plan = dsp.schedule(kernel.read()['compile-multimodal-locomotion-soa'])
    (lag0, duty0, bpm, k), (lag1, duty1, _b, _k), bars = fours.clock_of(plan[0][2]), fours.clock_of(plan[2][2]), plan[1][2].bars
    hz = bpm / 120.0
    seconds = bars * 4.0 * 60.0 / bpm
    phi, _path = _turned(-lag0, lag0, hz, k, 2.0)
    steps, trail, fewest = 0.0, 0.0, 4.0
    for n in range(int(round(seconds / DT))):
        u = (n + 1) * DT / seconds
        s = u * u * (3.0 - 2.0 * u)
        lag, duty = fours.toward(lag0, lag1, s), duty0 + s * (duty1 - duty0)
        nxt = fours.clock(phi, lag, hz, k, DT)
        steps = max(steps, float(np.abs(np.angle(np.exp(1j * (nxt - phi)))).max()))
        phi = nxt
        trail, fewest = max(trail, float(_off(phi, lag).max())), min(fewest, float(fours.bearing(phi, duty).sum()))
    rate = 1.5 * float(np.abs(fours.toward(lag0, lag1, 1.0) - lag0).max()) / seconds
    report.check('trot to crawl over %.2f s: a tick turns a limb %.4f rad at most, they trail by %.4f rad (%.4f), %d '
                 'bear at least' % (seconds, steps, trail, rate / (4.0 * k), fewest),
                 steps < 2.0 * fours.TAU * hz * DT and trail <= 1.05 * rate / (4.0 * k) and fewest >= 2.0)


def test_the_fifth_bears_while_her_limbs_swing(report):
    """The crab's scoot (`specialize-supine-peristaltic-underbed-scoot`): its fifth limb locked antiphase to her
    limbs' swing (`fours.fifth`) bears through every limb's swing when they advance as one, its flag; under the trot
    its topology lays, through half of it - one diagonal's swing."""
    import numpy as np
    from machine.texts import dsp, kernel
    from machine.moves import fours
    crab = dsp.schedule(kernel.read()['compile-multimodal-locomotion-soa'])[3][2]
    lag, duty, _bpm, k = fours.clock_of(crab)
    axial = crab['axial-peristalsis']
    covered = {}
    for together in (float(axial['simultaneous-limb-advance-on-axial-stance']), 0.0):
        lags = fours.fifth(lag, duty, together)
        phi, _path = _turned(-lags, lags, 1.0, k, 1.0)
        _phi, path = _turned(phi, lags, 1.0, k, 1.0)
        on = np.array([np.append(fours.bearing(p[:4], duty), fours.bearing(p[4:], axial['ground-dwell-duty'])) for p in path])
        swing = 1.0 - on[:, :4]
        covered[together] = float((swing * on[:, 4:]).sum() / swing.sum())
    report.check('advanced as one, the fifth bears through %.3f of their swing; under the trot %.3f' % (
        covered[1.0], covered[0.0]), covered[1.0] > 0.99 and abs(covered[0.0] - 0.5) < 0.01)


def _stance(origin, pitch, hind_point):
    """Her figure on all fours (deg, turn, contacts (4, 3) [LH LF RH RF] on the floor, centre of mass, knee rad, frames):
    the text's ankle, elbow, spine and neck; each limb plumb from its hip or shoulder; the knee the equalizer bends
    (`fours.bend`) for her shoulders `pitch` rad over her hips; the contacts a sole's `hind_point` and a fist's
    lowest."""
    import numpy as np
    from machine.body import figure
    from machine.moves import fours
    from machine.body.term_rows import TERMS

    def told(part, term, rad):
        kind, sign = TERMS[(part, term)][:2]
        return kind, sign * math.degrees(rad)
    deg = figure.stand()
    for name, part, term in (('talocrural', 'ankle', 'dorsiflexion'), ('cubital', 'elbow', 'flexion'),
                             ('c1-c2-atlas', 'neck', 'flexion')):
        kind, d = told(part, term, origin.get(name, 0.0))
        deg.update({kind if kind == 'neck' else side + '_' + kind: d for side in ('left', 'right')})
    deg['spine'] = told('trunk', 'flexion', origin.get('l4-l5-lumbar', 0.0) + origin.get('c7-t12-thoracic', 0.0))[1]
    deg.update({side + j: 0.0 for side in ('left', 'right') for j in ('_hip', '_knee', '_shoulder')})
    fr = figure.frames(deg, (0, 0, 0), figure.IDENT)
    hips = 0.5 * np.add(fr['left_thigh'][0], fr['right_thigh'][0])
    trunk = 0.5 * np.add(fr['left_upper_arm'][0], fr['right_upper_arm'][0]) - hips
    alpha = math.atan2(trunk[1], trunk[2]) - pitch
    turn = figure.rx(alpha)

    def fist(s):
        deg.update({'left_shoulder': s, 'right_shoulder': s})
        return np.subtract(figure.fist(deg, (0, 0, 0), turn), figure.frames(deg, (0, 0, 0), turn)['left_upper_arm'][0])
    lo, hi = 0.0, 180.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if fist(mid)[2] < 0.0 else (lo, mid)
    fore = -fist(lo)[1] + figure.FIST_R
    contact = np.add(fr['left_foot'][0], figure.apply(fr['left_foot'][1], hind_point)) - np.array(fr['left_shank'][0])
    kappa = float(fours.bend(figure.THIGH, math.hypot(contact[1], contact[2]), fore - math.hypot(*trunk[1:]) * math.sin(pitch))
                  + math.atan2(contact[1], contact[2]) + 0.5 * math.pi)
    deg.update({side + '_knee': told('knee', 'flexion', kappa)[1] for side in ('left', 'right')})
    fr = figure.frames(deg, (0, 0, 0), figure.IDENT)
    u = np.add(fr['left_foot'][0], figure.apply(fr['left_foot'][1], hind_point)) - np.array(fr['left_thigh'][0])
    deg.update({side + '_hip': math.degrees(math.atan2(u[1], u[2]) - alpha + 0.5 * math.pi) for side in ('left', 'right')})
    fr = figure.frames(deg, (0, 0, 0), turn)
    points = [p for side in ('left', 'right') for p in (
        np.add(fr[side + '_foot'][0], figure.apply(fr[side + '_foot'][1], hind_point)),
        np.subtract(figure.fist(deg, (0, 0, 0), turn, side), (0.0, figure.FIST_R, 0.0)))]
    return deg, turn, np.array(points), np.array(figure.com(deg, (0, 0, 0), turn)), kappa, fr


def _share(points, com):
    """Her legs' share of her weight as her centre's place along her stance makes it."""
    hind, fore = points[[0, 2], 2].mean(), points[[1, 3], 2].mean()
    return (fore - com[2]) / (fore - hind)


def test_her_legs_levelled_to_her_arms(report):
    """On her figure, the text's pronograde origin on her toes and fists with her limbs plumb: the knee bent by the
    equalizer (`fours.bend`) for the dog trot's trunk pitch sets her soles and fists on one floor to 1e-9 m and her
    shoulders over her hips at that pitch, the knee within her travel; her hips' height, her legs' share and how far
    back the text's share wants her centre as that stance makes them."""
    import numpy as np
    from machine.body import drives
    from machine.texts import kernel
    from machine.body.figure import SOLE_BALL
    got = kernel.read()
    origin = got['quadrupedal-posture-extensions']['postures']['pronograde-quadruped']['origin-q0']
    told = kernel.node('xform-gait-pronograde-quadruped', 'limb-lengths')
    share = kernel.node('xform-gait-pronograde-quadruped', 'virtual-truss')['sagittal-load-distribution']
    pitch = told['nominal-thorax-pitch-rad']
    deg, _turn, points, com, kappa, fr = _stance(origin, pitch, SOLE_BALL)
    hips = 0.5 * np.add(fr['left_thigh'][0], fr['right_thigh'][0])
    sh = 0.5 * np.add(fr['left_upper_arm'][0], fr['right_upper_arm'][0])
    floor = points[:, 1].max() - points[:, 1].min()
    at = math.atan2(sh[1] - hips[1], sh[2] - hips[2])
    low, high = drives.TRAVEL['knee']
    legs, length = _share(points, com), points[[1, 3], 2].mean() - points[[0, 2], 2].mean()
    report.check('one floor, %.0e m; shoulders at %.3f rad; knee %.2f rad (the text %.2f), hips %.3f m up (%.3f), her '
                 'legs %.2f of her: %.2f wants her centre %.3f m back' % (
                     floor, at, kappa, origin['tibiofemoral'], hips[1] - points[:, 1].mean(), told['nominal-pelvis-z-m'],
                     legs, share['hind-to-fore-weight-ratio'], (share['hind-to-fore-weight-ratio'] - legs) * length),
                 floor < 1e-9 and abs(at - pitch) < 1e-9 and low <= deg['left_knee'] <= high)


def test_the_truss_meets_her_weight_and_squeezes_nothing(report):
    """`fours.truss` on her levelled pronograde stance under her weight: the wrench met to 1e-9, every edge squeezed
    0 N as told; her legs' share her centre's place alone, the same to 1e-9 N under her drives' held torques as the
    metric (`heat.held`: arms 22.3 N m, legs 43.0); the scramble's 32 N wedge on all six edges, her hands a riser up."""
    import numpy as np
    from machine.body import figure, heat
    from machine.moves import fours
    from machine.texts import kernel
    from machine.law import modal
    origin = kernel.read()['quadrupedal-posture-extensions']['postures']['pronograde-quadruped']['origin-q0']
    _deg, _turn, points, com, _kappa, _fr = _stance(origin, -0.06, figure.SOLE_BALL)
    weight = fours.held(com, figure.mass() * modal.G, (0.0, 1.0, 0.0))
    torques = np.array([heat.held(j)[0] for j in ('left_hip', 'left_shoulder', 'right_hip', 'right_shoulder')])
    shares = []
    for metric in (np.ones(4), 1.0 / torques ** 2):
        f = fours.truss(points, weight, metric, np.zeros(6))
        met = float(np.abs(fours.grasp(points) @ f.ravel() - weight).max())
        squeezed = float(np.abs(fours.squeezes(points) @ f.ravel()).max())
        shares.append((f[0, 1] + f[2, 1]) / weight[1])
        report.check('metric %s: met to %.0e, squeezed %.0e N, her legs %.4f of her' % (
            np.round(metric / metric.max(), 2).tolist(), met, squeezed, shares[-1]), met < 1e-9 and squeezed < 1e-9)
    report.check('the metric moves no weight between her legs and arms: %.0e of her' % abs(shares[1] - shares[0]),
                 abs(shares[1] - shares[0]) < 1e-9 and abs(shares[0] - _share(points, com)) < 1e-9)
    wedge = kernel.node('specialize-quad-steep-grade-scramble', 'virtual-truss')['nullspace-internal-wrench-law']
    three, told = points[:3], np.full(3, wedge['edge-tension-target-n'])
    f = fours.truss(three, weight, np.ones(3), told)
    four = fours.truss(points, weight, np.ones(4), np.full(6, told[0]))
    report.check('three bearing, her right fore aloft: every edge squeezed %.0f N to %.0e; four on one plane, %.1f N '
                 'short (a braced quadrilateral\'s self-stress)' % (
                     told[0], float(np.abs(fours.squeezes(three) @ f.ravel() - told).max()),
                     float(np.abs(fours.squeezes(points) @ four.ravel() - told[0]).max())),
                 np.abs(fours.squeezes(three) @ f.ravel() - told).max() < 1e-9)


ROSTER = [test_the_texts_read_whole, test_each_symmetry_locks_from_anywhere, test_a_morph_turns_her_lags_continuously,
          test_the_fifth_bears_while_her_limbs_swing, test_her_legs_levelled_to_her_arms,
          test_the_truss_meets_her_weight_and_squeezes_nothing]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
