#!/usr/bin/env python3
"""Her gaits off her feet (`machine.crawl`; the kernel's terrestrial_multimodal_gaits.lisp,
terrestrial_gait_specializers.lisp, terrestrial_locomotion_patches.lisp): the texts read whole, her hinges sideways,
a sled and its anchors, the four origins on her figure, the supine ratchet on her drives."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_groundwork import MOVES, TOUCH, _bearing, _laid, _parts

#: The terrestrial texts, machine/ortho_kernel/, and the overlay their patches use that no text defined.
TEXTS = ('terrestrial_multimodal_gaits.lisp', 'terrestrial_gait_specializers.lisp',
         'terrestrial_locomotion_patches.lisp', 'substrate_overlay.lisp')
#: Samples of the vault's half turn; a hinge's angle stepped, rad.
HALF, NUDGE = 100001, 1e-4


def _plan():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-terrestrial-multimodal-suite-soa'])


def _patch(name):
    """The patch of the registry or the floor texts a name says: itself, or the one whose frames hold it."""
    from machine import kernel
    got = kernel.read()
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    name = name.lstrip(':')
    return patches[name] if name in patches else next(v for v in patches.values() if ':' + name in v['frames'])


def test_the_texts_read_whole(report):
    """The texts read with the kernel (`kernel.read`): four postures, each its origin and three columns; every
    manifold they name a patch; the stages; the schedule four segments of 8 bars - the dog's trot, her back's crawl
    with its stair ratchet, the scoot and its hip drag, the crab to her right - each on the overlay, the crab's outer
    pair half a turn ahead of the inner."""
    from machine import dsp, kernel
    got = kernel.read()
    atlas = got['terrestrial-1jet-posture-atlas']['postures']
    report.check('four postures, each its origin and three columns',
                 len(atlas) == 4 and all(len(p['modal-basis-columns']) == 3 and 'origin-q0' in p for p in atlas.values()))
    folder = os.path.join(os.path.dirname(dsp.__file__), 'ortho_kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    named = {m for t in texts for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx))(?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch' % len(named), all(_patch(m) for m in named))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages' % (len(stages) - 1),
                 all(isinstance(got[n], dsp.Stage) for n in stages if n != 'compile-terrestrial-multimodal-suite-soa'))
    plan = _plan()
    clocks = [(b0, b1, g['axial-torus-clock']['nominal-phases'], g['axial-torus-clock']['limb-duty'],
               g['axial-torus-clock']['desired-se2-twist']) for b0, b1, g in plan]
    report.check('the schedule: %s' % '; '.join('%g-%g %s %.2f %s' % c for c in clocks),
                 [c[:2] for c in clocks] == [(0.0, 8.0), (8.0, 16.0), (16.0, 24.0), (24.0, 32.0)]
                 and clocks[0][2] == (0.0, 0.5, 0.5, 0.0, 0.0) and clocks[2][2] == (0.0, 0.0, 0.0, 0.0, 0.5)
                 and clocks[3][2] == (0.5, 0.5, 0.0, 0.0, 0.0) and clocks[3][4][1] < 0.0
                 and all(any(n == 'apply-universal-deformable-overlay' for n, _x in g['_trace']) for _a, _b, g in plan))


def _stroke(deg, joint, at):
    """1: the share along her left-right of the way point `at(deg)` goes as `joint` turns."""
    import numpy as np
    lo, hi = dict(deg), dict(deg)
    lo[joint] -= math.degrees(NUDGE)
    hi[joint] += math.degrees(NUDGE)
    way = np.subtract(at(hi), at(lo))
    return abs(way[0]) / float(np.linalg.norm(way))


def test_her_hinges_sideways(report):
    """`crawl.coronal` of the schedule's twists - the dog and her back's crawl 0, the scoot's 0.275, the crab's 1 -
    and the turnout it asks, saturating on her hip's turn. In the crab's origin her knee drives her ankle sideways
    more as her hip turns out (its abduction alone, her 25 deg, the text's 45); her elbow the same whatever it asks:
    her shoulder only flexes."""
    from machine import crawl, drives, figure, kernel
    plan = _plan()
    kappa = [float(crawl.coronal(g['axial-torus-clock']['desired-se2-twist'])) for _a, _b, g in plan]
    most = kernel.said('align-hinge-joints-to-se2-vector', 'max-coxa-turnout-rad')
    stop = math.radians(drives.TRAVEL['hip_yaw'][1])
    report.check('kappa %s; at most %.3f rad of hip turnout, her %.3f: past it beyond kappa %.3f' % (
        [round(k, 3) for k in kappa], most, stop, stop / most),
        kappa[:2] == [0.0, 0.0] and abs(kappa[3] - 1.0) < 1e-12
        and abs(float(crawl.turnout(plan[3][2]['axial-torus-clock']['desired-se2-twist'], most)) - most) < 1e-12)
    crab = kernel.read()['terrestrial-1jet-posture-atlas']['postures']['terrestrial-coronal-crab']['origin-q0']
    deg = _laid(crab)

    def ankle(d):
        return figure.frames(d, (0.0, 0.0, 0.0), figure.IDENT)['left_foot'][0]

    def fist(d):
        return figure.fist(d)
    knees, elbows = [], []
    for yaw in (0.0, drives.TRAVEL['hip_yaw'][1], math.degrees(crab['coxofemoral-ext-rot'])):
        turned = dict(deg, left_hip_yaw=yaw, right_hip_yaw=yaw)
        knees.append(_stroke(turned, 'left_knee', ankle))
        elbows.append(_stroke(turned, 'left_elbow', fist))
    report.check('sideways: her knee %s of its stroke at hip turnout 0, %g, %.0f deg; her elbow %.3f' % (
        ', '.join('%.3f' % k for k in knees), drives.TRAVEL['hip_yaw'][1], math.degrees(crab['coxofemoral-ext-rot']),
        elbows[0]),
        knees[0] < knees[1] < knees[2] and max(elbows) - min(elbows) < 1e-9)


def test_a_sled_and_its_anchors(report):
    """`crawl.held` is the share of the vault's half turn where `crawl.slip` is at most 1, to 1e-4; on the patches'
    own frictions the scoot's and the stairs' anchors hold part of it, the hip drag's and the army crawl's none: an
    anchor carrying under mu_sled / (mu_anchor + mu_sled) of her drags no sled."""
    import numpy as np
    from machine import crawl, dsp, kernel
    phase = np.linspace(0.0, np.pi, HALF)[1:-1]
    worst = max(abs(float(np.mean(crawl.slip(phase, g, a, s) <= 1.0)) - float(crawl.held(g, a, s)))
                for g, a, s in ((0.75, 0.7, 0.55), (0.65, 0.7, 0.65), (0.95, 0.6, 0.6)))
    report.check('held the share where the anchors hold, within %.0e' % worst, worst < 1e-4)
    plan = _plan()
    army = dsp.value(['specialize-pronograde-knee-vs-bear-mode', 'nominal-biped-gait', ':mode',
                      ':forearm-belly-army-crawl'], kernel.read())['stick-slip']
    said = []
    for name, node in (('scoot', kernel.node('xform-gait-ischial-butt-scoot', 'stick-slip')),
                       ('stairs', plan[1][2]['stick-slip']), ('hip drag', plan[2][2]['stick-slip']), ('army', army)):
        a = float(np.mean([_patch(p)['default-mu'] for p in node['anchors']]))
        s = float(np.mean([_patch(p)['default-mu'] for p in node['sleds']]))
        gamma = node['pelvic-unweighting-gain']
        said.append((name, gamma, float(crawl.held(gamma, a, s)), s / (a + s)))
    report.check('of the half turn the anchors hold: %s' % '; '.join(
        '%s at %.2f %.3f (it needs %.3f)' % x for x in said),
        all((x[2] > 0.0) == (x[1] > x[3]) for x in said))


def test_the_origins_on_her_figure(report):
    """The four origins as her movements: each joint hers or one she lacks, each angle against her stops; laid rigid
    on a hard floor (`groundwork.rest`, pitched), every rest her centre over what touches, the rest where its coupled
    patches bear best and the float of their worst part: the dog alone on its own."""
    from machine import kernel
    from machine.term_rows import ABSENT, TERMS
    atlas = kernel.read()['terrestrial-1jet-posture-atlas']['postures']
    told = [(name, n, rad) for name, p in atlas.items() for n, rad in p['origin-q0'].items()]
    lacks = sorted({'%s %s' % (p[12:], n) for p, n, _r in told if MOVES[n] in ABSENT})
    stops = {m: (-math.inf if t[2] is None else t[2], math.inf if t[3] is None else t[3]) for m, t in TERMS.items()}
    out = sorted({'%s %s %.0f deg' % (p[12:], n, math.degrees(r)) for p, n, r in told if MOVES[n] in TERMS
                  and not stops[MOVES[n]][0] <= math.degrees(r) <= stops[MOVES[n]][1]})
    report.check('she lacks: %s; past her stops: %s' % ('; '.join(lacks), '; '.join(out) or 'none'), bool(lacks))
    rests, bears = [], {}
    for name, p in atlas.items():
        coupled = p['coupled-manifold']
        phi, part, worst, margin, touching, every = _bearing(_laid(p['origin-q0']), _parts(coupled))
        rests += every
        bears[name] = worst < TOUCH
        report.check('%s: best at %+.0f deg on %s, its %s %.0f mm up' % (
            name[12:], math.degrees(phi), ', '.join(touching), part, 1e3 * worst), True)
    report.check('every rest her centre over what touches; on its own patches: %s' % [n[12:] for n, b in bears.items() if b],
                 min(rests) >= -TOUCH and [n for n, b in bears.items() if b] == ['terrestrial-pronograde-dog'])


def test_the_supine_ratchet_on_her_drives(report):
    """Her back's stair climb: a riser's work, the legs' share the text gives them; her shoulder's held torque
    (`heat.held`) against the text's cap on its extension; the descent's skim height as a drop."""
    from machine import figure, heat
    ratchet = _plan()[1][2]['grade-ratchet']
    rise = 1e-3 * ratchet['elevation-delta-mm']
    cap = ratchet['supine-ascent-law']['max-glenohumeral-extension-torque-nm']
    skim = 1e-3 * ratchet['supine-descent-law']['skim-clearance-z-mm']
    weight = figure.mass() * 9.81
    held = heat.held('left_shoulder')[0]
    report.check('a %.0f mm riser %.1f J; her shoulder holds %.1f N m for ever, the text caps it at %.0f; its skim a '
                 '%.0f mm drop, %.2f m/s, %.1f J' % (1e3 * rise, weight * rise, held, cap, 1e3 * skim,
                                                      math.sqrt(2.0 * 9.81 * skim), weight * skim), held < cap)


ROSTER = [test_the_texts_read_whole, test_her_hinges_sideways, test_a_sled_and_its_anchors,
          test_the_origins_on_her_figure, test_the_supine_ratchet_on_her_drives]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
