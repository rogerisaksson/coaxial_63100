#!/usr/bin/env python3
"""Walk to run, a brake, a jump and a leap (`machine.ballistic`; the kernel's ballistic_*.lisp): the texts read
whole, their regimes on her leg, her run's stance on her knees, her jumps and leap from her drives, her brake on a
floor's friction and her DC link."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The ballistic texts, machine/ortho_kernel/.
TEXTS = ('ballistic_and_velocity_core.lisp', 'ballistic_locomotion_specializers.lisp',
         'ballistic_lifecycle_patches.lisp')
#: The regimes the velocity spectrum names, slowest first.
REGIMES = (':normal-walk', ':brisk-walk', ':sprangmarsch', ':slip-run')


def _plan():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-ballistic-velocity-lifecycle-soa'])


def _on(form):
    """Her nominal gait through one of the texts' forms, `[stage, keys..]`."""
    from machine import dsp, kernel
    return dsp.value([form[0], 'nominal-biped-gait', *form[1:]], kernel.read())


def _regime(regime):
    """(m/s, BPM, the stance's share of a stride) of a regime on her leg."""
    from machine import ballistic
    g = _on(['xform-continuous-velocity-spectrum', ':regime', regime])
    morph = g['froude-homotopy']
    law = morph['duty-factor-law'].args
    beta = float(ballistic.duty(morph['froude-index'], law['froude-knots'], law['duty-values']))
    return morph['target-speed-mps'], g['oscillator']['bpm'], beta


def test_the_texts_read_whole(report):
    """The texts read with the kernel: four postures, each an origin and three columns; every manifold they name a
    patch; their stages; the schedule's walk, morphs, double-time, leap, run, brake, walk and jump; the brisk walk's
    leap held on her world's obstacle ahead."""
    from machine import dsp, kernel
    got, plan = kernel.read(), _plan()
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    folder = os.path.join(os.path.dirname(dsp.__file__), 'ortho_kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    postures = got['ballistic-1jet-posture-atlas']['postures']
    report.check('%d postures, each an origin and 3 columns: %s; the header\'s split-leap has none' % (
        len(postures), ', '.join(postures)),
        len(postures) == 4 and all(len(p['modal-basis-columns']) == 3 and p['origin-q0'] for p in postures.values()))
    named = {m for t in texts for m in re.findall(r':([a-z0-9-]+-bilateral)(?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch: %s' % (len(named), ', '.join(sorted(named))),
                 named <= set(patches), str(sorted(named - set(patches))))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages' % (len(stages) - 1), all(isinstance(got[n], dsp.Stage) for n in stages
                                                      if n != 'compile-ballistic-velocity-lifecycle-soa'))
    shots = [seg[1] for seg in plan if seg[0] == 'oneshot']
    morphs = [seg for seg in plan if seg[0] != 'oneshot' and isinstance(seg[2], dsp.Morph)]
    launched = [g['ssc-launch']['topology'] for g in shots if 'ssc-launch' in g]
    report.check('the schedule: %d segments, %d morphs; its leap %s, its brake, its jump %s' % (
        len(plan), len(morphs), launched[0], launched[-1]),
        [seg[0] for seg in plan if seg[0] == 'oneshot'] == ['oneshot'] * 3 and len(morphs) == 2
        and launched[0] == ':unilateral-split-leap' and launched[-1] == ':bilateral-in-phase'
        and 'brake-horizon' in shots[1])
    brisk = plan[2][2]
    gate = brisk['_gates'][0]
    apex = brisk['ssc-launch']['desired-apex-z-m']
    report.check('the brisk walk\'s leap held on her world (%s): its apex a law of the obstacle ahead' % gate[1].name,
                 isinstance(gate[1], dsp.Law) and isinstance(apex, dsp.Law) and 'ssc-launch' in gate[2]
        and 'query-upcoming-obstacle-height-z' in str(apex))


def test_her_regimes_on_her_leg(report):
    """Each regime's Froude number on her leg and the duty the text's Hermite map gives it: double support walking,
    flight running. Her run's stance: one sole bears her weight over a step through a stance of its share (a
    half-sine's peak), her knee holds it at the run origin's bend within its peak."""
    from machine import ballistic, drives, figure, kernel, somatic
    rows = [(r, *_regime(r)) for r in REGIMES]
    fr = [float(ballistic.froude(v, somatic.LEG_M)) for _r, v, _b, _d in rows]
    report.check('on her %.3f m leg: %s' % (somatic.LEG_M, '; '.join(
        '%s %.2f m/s %.1f BPM Fr %.3f duty %.3f' % (r[1:], v, b, f, d) for (r, v, b, d), f in zip(rows, fr))),
        all(a < b for a, b in zip(fr, fr[1:])) and rows[0][3] > 0.5 > rows[-1][3])
    _r, _v, bpm, beta = rows[-1]
    step = 60.0 / bpm
    stance = 2.0 * beta * step
    weight = figure.mass() * ballistic.G
    peak = 0.5 * math.pi * weight * step / stance
    flight = step - stance
    knee = kernel.read()['ballistic-1jet-posture-atlas']['postures']['locomotion-slip-run']['origin-q0']['tibiofemoral']
    reach = math.sqrt(figure.THIGH ** 2 + figure.SHANK ** 2 + 2.0 * figure.THIGH * figure.SHANK * math.cos(knee))
    lever = figure.THIGH * figure.SHANK * math.sin(knee) / reach
    report.check('her run: a %.0f ms step, %.0f ms stance, %.0f ms flight; a sole peaks at %.0f N, her knee %.1f N m '
                 'at the origin\'s %.2f rad (peak %.1f)' % (1e3 * step, 1e3 * stance, 1e3 * flight, peak,
                                                          peak * lever, knee, drives.peak('left_knee')),
                 flight > 0.0 and peak * lever < drives.peak('left_knee'))


def test_her_jumps_and_leap_from_her_drives(report):
    """Each jump's apex the text asks her knees' launch from its crouch (`ballistic.launch`): she leaves where her
    back-EMF ends the push, her knees near their no-load speed; her ankles' heel rise, the text's handoff, reaches a
    little past it before their rotors are spun. Its broad jump at the text's angle, the platform under her apex by
    its margin, the leap one knee's from its stance at her double-time march, the gap leap's floor her apex less
    its margin."""
    from machine import ballistic, drives, figure, kernel
    no_load = math.radians(drives.speed('left_knee'))
    ankle = figure.BALL * math.radians(drives.speed('left_ankle'))
    shots = [seg[1] for seg in _plan() if seg[0] == 'oneshot']
    platform = _on(['specialize-jump-onto-elevated-platform'])
    told = {'default': kernel.node('drive-stretch-shortening-ballistic-impulse', 'ssc-launch'),
            'standing': kernel.node('xform-standing-bilateral-jump', 'ssc-launch'), 'lifecycle': shots[2]['ssc-launch'],
            'platform': platform['ssc-launch']}
    rows = []
    for name, node in told.items():
        depth = node['phase-1-countermovement']['crouch-depth-m']
        v, t, w = ballistic.launch(depth)
        h, x = node['desired-apex-z-m'], node['desired-range-x-m']
        angle = math.atan2(4.0 * h, x)
        apex, span = ballistic.parabola(v * math.sin(angle), v * math.cos(angle))
        rows.append((name, depth, v, t, w, h, x, float(apex), float(span)))
    report.check('her jumps: %s; her ankles reach %.2f m/s' % ('; '.join(
        '%s from %.0f mm leaves at %.2f m/s in %.0f ms, knees %.1f of %.1f rad/s: %.0f up %.0f ahead (told %.0f, %.0f)'
        % (n, 1e3 * d, v, 1e3 * t, w, no_load, 1e3 * a, 1e3 * s, 1e3 * h, 1e3 * x)
        for n, d, v, t, w, h, x, a, s in rows), ankle),
        all(abs(a - h) < 1e-3 and abs(s - x) < 1e-3 and w > 0.9 * no_load for _n, _d, _v, _t, w, h, x, a, s in rows))
    up = platform['touchdown-datum']['expected-delta-z-mm']
    report.check('her platform %.0f mm, %.0f under her apex' % (up, 1e3 * platform['ssc-launch']['desired-apex-z-m'] - up),
                 0.0 < up < 1e3 * platform['ssc-launch']['desired-apex-z-m'])
    leap = shots[0]['ssc-launch']
    v, _t, _w = ballistic.launch(leap['phase-1-countermovement']['crouch-depth-m'], legs=1.0)
    apex, span = ballistic.parabola(v, _regime(':sprangmarsch')[0])
    gap = kernel.read()['specialize-leap-over-sdf-gap-or-hurdle'].keys
    report.check('her leap on one knee: %.2f m/s up, %.0f mm, %.0f ahead at %.2f m/s (told %.0f, %.0f); a gap '
                 'leap\'s apex floor %.0f' % (v, 1e3 * apex, 1e3 * span, _regime(':sprangmarsch')[0],
                                              1e3 * leap['desired-apex-z-m'], 1e3 * leap['desired-range-x-m'],
                                              1e3 * apex - gap['safety-margin-z-mm']),
                 abs(1e3 * apex - 1e3 * leap['desired-apex-z-m']) < 1.0
                 and abs(1e3 * span - 1e3 * leap['desired-range-x-m']) < 1.0)


def test_her_brake(report):
    """From her run to the walk the brake asks: each step's stance brakes her by its cap of the floor's friction
    (`ballistic.braked`, `ballistic.steps`) - within the text's two steps on a dry floor and her soles' own, its
    stutter's steps on a slick one -; what her speed gives her DC link over two steps under what her packs take."""
    from machine import ballistic, dclink, figure, kernel
    v, bpm, _beta = _regime(':slip-run')
    node = _on(['xform-substrate-gated-rapid-brake'])
    brake, walk = node['brake-horizon'], node['brake-horizon']['target-speed-mps']
    high, low = brake['high-mu-aggressive-stop'], brake['low-mu-stutter-abs-stop']
    cap = brake['cone-utilization-cap']
    mu = kernel.read()['anatomical-manifold-registry']['plantar-bilateral']['default-mu']
    dry = [int(ballistic.steps(v, walk, ballistic.braked(m, cap, 60.0 / bpm))) for m in (0.65, mu)]
    stutter = min(float(ballistic.braked(0.35, cap, 60.0 / bpm / low['boost-cadence-bpm-scale'])),
                  low['limit-per-step-dv-mps'])
    slick = int(ballistic.steps(v, walk, stutter))
    report.check('%.2f to %.2f m/s at %.1f BPM: %d steps at mu 0.65, %d at her soles\' %.2f (at most %d); %d stutter '
                 'steps at 0.35 (at least %d)' % (v, walk, bpm, dry[0], dry[1], mu, high['max-braking-steps'], slick,
                                                  low['min-braking-steps']),
                 max(dry) <= high['max-braking-steps'] and slick >= low['min-braking-steps'])
    energy = 0.5 * figure.mass() * (v * v - walk * walk)
    watts = energy / (dry[0] * 60.0 / bpm)
    taken = dclink.CHARGE_A * dclink.ONSET_V
    report.check('her %.0f J over %d steps, %.0f W; her packs take %.0f W at %.1f V, the brake\'s ceiling %.1f under '
                 'her clamp %.1f' % (energy, dry[0], watts, taken, dclink.ONSET_V, brake['bus-voltage-ceiling-v'],
                                     dclink.CLAMP_V),
                 watts < taken and dclink.ONSET_V < brake['bus-voltage-ceiling-v'] < dclink.CLAMP_V)


ROSTER = [test_the_texts_read_whole, test_her_regimes_on_her_leg, test_her_jumps_and_leap_from_her_drives,
          test_her_brake]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
