#!/usr/bin/env python3
"""What she stands on when it gives (`machine.substrate`; the kernel's deformable_substrate_core.lisp,
deformable_groundwork_specializers.lisp, deformable_groundwork_patches.lisp): the texts read whole, the work loop's
regimes as a contact's damping, her patches' pressures, the tamp's gate, a bed under her still centre, the kip."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The deformable texts, lisp/humanoid/kernel/.
TEXTS = ('deformable_substrate_core.lisp', 'deformable_groundwork_specializers.lisp',
         'deformable_groundwork_patches.lisp')
#: A tick of an impact or a bed, s; her patches' raster, m; the identifier's window, s (its text's comment).
DT, STEP, WINDOW_S = 1e-6, 5e-4, 0.035
#: A power law's exponent at most (Bekker's n for snow and soil runs under it).
N_MOST = 2.0


def _plan():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-deformable-groundwork-suite-soa'])


def _impact(m, k, zeta, v0):
    """(normal force, surface speed) each tick of mass `m` striking a linear spring-damper contact `k` at `v0`,
    damped at `zeta`, gravity aside, till it leaves - the contact pulling where its damper outruns its spring."""
    import numpy as np
    d = 2.0 * zeta * math.sqrt(k * m)
    z, v, out = 0.0, -v0, []
    while z <= 0.0 or v <= 0.0:
        f = -k * z - d * v
        out.append((f, v))
        v += DT * f / m
        z += DT * v
    return np.array(out).T


def test_the_texts_read_whole(report):
    """The three texts read with the kernel (`kernel.read`): the substrate's state, 9 slots of 8 patches; every
    manifold they name a patch; the core, the specializers and the patches stages; the schedule the mattress, the snow
    and the trampoline, each up again on her walk's clock; the regimes, ceilings, tamp and kip as the runs lay them."""
    from machine import dsp, kernel, mechatronics
    got = kernel.read()
    state = got['rheological-substrate-soa-state']
    report.check('the state: %d slots of %d patches' % (len(state['slot-schema']),
                                                        state['memory-layout']['num-monitored-patches']),
                 len(state['slot-schema']) == 9 and state['memory-layout']['num-monitored-patches'] == 8.0)
    folder = os.path.join(mechatronics.LISP, 'humanoid', 'kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    named = {m for t in texts for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx))(?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch' % len(named), named <= set(patches), str(sorted(named - set(patches))))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages' % (len(stages) - 1),
                 all(isinstance(got[n], dsp.Stage) for n in stages if n != 'compile-deformable-groundwork-suite-soa'))
    plan = _plan()
    runs = [seg[1]['_trace'][-1][0] for seg in plan]
    report.check('the schedule: %s, each up on her clock' % ', '.join(runs),
                 [seg[0] for seg in plan] == ['oneshot'] * 3
                 and runs == ['patch-groundwork-on-viscoelastic-mattress', 'patch-groundwork-in-deep-snow',
                              'patch-groundwork-on-trampoline']
                 and all(any(n == 'ignite-cyclic-clock' for n, _x in seg[1]['_trace']) for seg in plan))
    classes = plan[0][1]['work-loop']['regime-classifier']
    snow, base = plan[1][1], plan[0][1]
    report.check('plastic under %.2f, harmonic over %.2f; %.0f kPa a patch, %.0f in snow; snow tamped at %.2f of '
                 'her weight to %.0f N/mm; the kip %.0f mm up' % (
                     classes[0]['when-eta-below'], classes[2]['when-eta-above'],
                     base['patch-pressure']['pressure-ceiling-kpa'], snow['patch-pressure']['pressure-ceiling-kpa'],
                     snow['tamp-commit']['stage-1-pre-tamp']['preload-weight-cap'],
                     snow['tamp-commit']['stage-2-commit-trigger']['require-dF-dz-above-npmm'],
                     1e3 * plan[2][1]['seat-drop-kip']['target-aerial-clearance-m']),
                 classes[1]['when-eta-between'] == (classes[0]['when-eta-below'], classes[2]['when-eta-above']))


def test_the_work_loop_names_a_damping(report):
    """`substrate.loop` on her striking a linear contact returns e^2 = exp(-2 pi zeta / sqrt(1 - zeta^2)) of the
    energy to 1e-3, so the text's regimes are damping ratios (`substrate.damping`): an elastic foam damped at 0.5,
    springing back whole, reads plastic. Her stroke, touch to deepest, fits the identifier's 35 ms only on a contact
    stiffer than her mass times (pi / 2 / 0.035)^2."""
    import numpy as np
    from machine import figure, kernel, substrate
    classes = _plan()[0][1]['work-loop']['regime-classifier']
    ceiling, floor = classes[0]['when-eta-below'], classes[2]['when-eta-above']
    soft = kernel.node('xform-continuous-ground-roll', 'substrate-roll')['soft-substrate-compensation']
    m, k = figure.mass(), 1e3 * soft['active-below-npmm']
    names = ('plastic', 'viscoelastic', 'harmonic')
    for zeta, want in ((substrate.damping(floor), None), (substrate.damping(ceiling), None), (0.04, 'harmonic'),
                       (0.15, 'viscoelastic'), (0.5, 'plastic')):
        fz, zdot = _impact(m, k, float(zeta), 1.0)
        _w_in, _w_out, eta = substrate.loop(fz, zdot, DT)
        e2 = math.exp(-2.0 * math.pi * zeta / math.sqrt(1.0 - zeta * zeta))
        got = names[int(np.argmax(substrate.regime(eta, ceiling, floor)))]
        report.check('damped %.3f: %.4f of the energy back (%.4f)%s' % (
            zeta, eta, e2, ', ' + got if want else ''), abs(eta - e2) < 1e-3 and got == (want or got))
    stiff = m * (0.5 * math.pi / WINDOW_S) ** 2
    report.check('her %.1f kg strokes %.0f ms at %.0f N/mm: within %.0f ms only past %.1f N/mm' % (
        m, 1e3 * substrate.stroke(m, k), k / 1e3, 1e3 * WINDOW_S, stiff / 1e3),
        abs(substrate.stroke(m, stiff) - WINDOW_S) < 1e-12)


def _patches():
    """{patch: m^2} of hers: a sole and its toes (their boxes' undersides), a fist, a shin, a forearm and its fist, a
    thigh (capsules flat on their segments' x-y planes), her buttocks (the seat's capsules on its x-z plane)."""
    import numpy as np
    from machine import figure, substrate

    def flat(caps, plane=(0, 1)):
        a, b = (np.array([[c[k][i] for i in plane] for c in caps]) for k in (1, 2))
        return substrate.footprint(a, b, np.array([c[0] for c in caps]), STEP)

    def of(part):
        return [(r, a, b) for p, r, a, b in figure.BODY if p == part]
    fist = (figure.FIST_R, (figure.FIST_AT[0], figure.FIST_AT[1] - figure.FOREARM), (figure.FIST_AT[0], figure.FIST_AT[1] - figure.FOREARM))
    return {'sole': 2.0 * 0.045 * (figure.BALL + figure.HEEL + figure.TOE_M),
            'fist': flat([(figure.FIST_R, (0.0, 0.0), (0.0, 0.0))]), 'shin': flat(of('shank')),
            'forearm': flat(of('forearm') + [fist]), 'thigh': flat(of('thigh')),
            'buttocks': flat([c for c in of('pelvis') if c[0] == figure.SEAT_R], (0, 2))}


def test_her_patches_under_her_weight(report):
    """`substrate.footprint` a disc's and a capsule's areas to 1 %; her patches' areas against the snowshoe's
    multipliers; her pressure standing, landing a step (her walk's bound), kneeling, on all fours' fists (the text's
    share), on her forearms and sat, against the texts' ceilings."""
    import numpy as np
    from machine import figure, kernel, substrate
    from tools.sim.looks import LANDS
    r, length = 0.03, 0.2
    disc = substrate.footprint(np.zeros((1, 2)), np.zeros((1, 2)), np.array([r]), STEP)
    capsule = substrate.footprint(np.zeros((1, 2)), np.array([[0.0, length]]), np.array([r]), STEP)
    report.check('a disc %.2f of pi r^2, a capsule %.2f of 2 r L + pi r^2' % (
        disc / (math.pi * r * r), capsule / (2.0 * r * length + math.pi * r * r)),
        abs(disc / (math.pi * r * r) - 1.0) < 0.01 and abs(capsule / (2.0 * r * length + math.pi * r * r) - 1.0) < 0.01)
    area = _patches()
    plan = _plan()
    snowshoe = {s['broad-snowshoe-patch']: s['area-multiplier'] for s in plan[1][1]['patch-pressure']['area-foliation-synergies']}
    report.check('hers, cm^2: %s; shin to sole %.2f (the text %.1f), forearm to fist %.2f (%.1f), thighs to buttocks '
                 '%.2f (%.1f)' % (', '.join('%s %.0f' % (k, 1e4 * v) for k, v in area.items()),
                                  area['shin'] / area['sole'], snowshoe[':anterior-tibial-bilateral'],
                                  area['forearm'] / area['fist'], snowshoe[':olecranon-ulnar-sphinx'],
                                  2.0 * area['thigh'] / area['buttocks'], snowshoe[':posterior-femoral-bilateral']), True)
    weight = figure.mass() * 9.81
    fore = 1.0 - kernel.node('xform-kneeling-to-hands-and-knees', 'virtual-truss')['sagittal-load-distribution'][
        'hind-to-fore-weight-ratio']
    ceilings = (plan[1][1]['patch-pressure']['pressure-ceiling-kpa'], kernel.said('regulate-mohr-coulomb-patch-pressure',
                'max-contact-pressure-kpa'), plan[0][1]['patch-pressure']['pressure-ceiling-kpa'])
    said = []
    for name, newtons, a in (('standing', weight, 2.0 * area['sole']), ('a landing', LANDS * weight, area['sole']),
                             ('kneeling', weight, 2.0 * area['shin']), ('fists', fore * weight, 2.0 * area['fist']),
                             ('forearms', weight, 2.0 * area['forearm']), ('sat', weight, area['buttocks'])):
        kpa = newtons / a / 1e3
        said.append('%s %.1f%s' % (name, kpa, '' if kpa <= min(ceilings) else ' past %g' % max(c for c in ceilings if c < kpa)))
    report.check('kPa (ceilings %s): %s' % ('/'.join('%g' % c for c in ceilings), '; '.join(said)), True)


def test_a_tamp_commits_only_where_snow_bears_her(report):
    """`substrate.tangent` is a power law's d F / d z, n F / z, to 1e-6 whatever its moduli: the tamp's stiffness
    gate opens under its pre-load only at a sinkage under n F / k, any n to 2: in the first 2 % of the sinkage the
    tamp allows."""
    import numpy as np
    from machine import figure, substrate
    rng = np.random.default_rng(0)
    worst = 0.0
    for scale, n, z in zip(rng.uniform(1e4, 1e7, 16), rng.uniform(0.5, N_MOST, 16), rng.uniform(1e-3, 0.3, 16)):
        h = 1e-7 * z
        numeric = scale * ((z + h) ** n - (z - h) ** n) / (2.0 * h)
        worst = max(worst, abs(substrate.tangent(scale * z ** n, z, n) / numeric - 1.0))
    report.check('n F / z a power law\'s stiffness, within %.0e' % worst, worst < 1e-6)
    tamp = _plan()[1][1]['tamp-commit']
    pre, deepest = tamp['stage-1-pre-tamp']['preload-weight-cap'], tamp['stage-1-pre-tamp']['max-allowable-sinkage-m']
    gate = 1e3 * tamp['stage-2-commit-trigger']['require-dF-dz-above-npmm']
    weight = figure.mass() * 9.81
    opens = [1e3 * n * pre * weight / gate for n in (1.0, N_MOST)]
    at = [substrate.tangent(pre * weight, deepest, n) / gate for n in (1.0, N_MOST)]
    report.check('its %.0f N/mm opens under %.2f of her weight at %.1f-%.1f mm sunk, under all of it at %.1f-%.1f; at '
                 'its %.0f mm %.3f-%.3f of the gate' % (
                     gate / 1e3, pre, *opens, *[1e3 * n * weight / gate for n in (1.0, N_MOST)], 1e3 * deepest, *at),
                 max(opens) < 0.02 * 1e3 * deepest)


def test_a_bed_under_her_still_centre(report):
    """`substrate.phase` 0 at a bed's top, pi at its bottom. Her centre held still, her push on a bed is her weight,
    constant: its energy about its loaded rest falls by its own damper's alone (`substrate.bed`, 1e-3), the
    harmonic regime's most damped bed keeping what it keeps after the text's 1.5 periods; the kip's apex time and
    energy, her hips and knees at its tuck's speeds within their peak."""
    import numpy as np
    from machine import bemf, drives, figure, substrate
    report.check('phase: top %.3f, bottom %.3f rad' % (substrate.phase(1.0, 0.0, 1.0), substrate.phase(-1.0, 0.0, 1.0)),
                 substrate.phase(1.0, 0.0, 1.0) == 0.0 and abs(substrate.phase(-1.0, 0.0, 1.0) - math.pi) < 1e-12)
    plan = _plan()
    zeta = float(substrate.damping(plan[0][1]['work-loop']['regime-classifier'][2]['when-eta-above']))
    m, k, dt = figure.mass(), 5e3, 1e-5
    weight = m * 9.81
    kept = []
    for share in (1.0, 0.2):
        mass = share * m
        d = 2.0 * zeta * math.sqrt(k * (m + mass))
        rest = -weight / k
        z, v = rest - 0.05, 0.0
        start, lost = 0.5 * k * (z - rest) ** 2, 0.0
        for _ in range(int(round(1.5 * 2.0 * math.pi * math.sqrt(m / k) / dt))):
            z, v = substrate.bed(z, v, weight, mass, k, d, dt)
            lost += d * v * v * dt
        left = 0.5 * k * (z - rest) ** 2 + 0.5 * mass * v * v
        kept.append((share, left / start, abs(start - left - lost) / start))
    report.check('damped %.3f with her on it: a bed of %s her mass keeps %s of its energy, its damper the rest to %.0e' % (
        zeta, ' and '.join('%.1f' % s for s, _l, _e in kept), ' and '.join('%.2g' % l for _s, l, _e in kept),
        max(e for _s, _l, e in kept)), max(e for _s, _l, e in kept) < 1e-3)
    kip = plan[2][1]['seat-drop-kip']
    h, tuck = kip['target-aerial-clearance-m'], kip['midair-fast-leg-tuck']
    slews = (tuck['coxofemoral-tuck-slew-radps'], tuck['tibiofemoral-tuck-slew-radps'])
    joints = [bemf.DRIVEN.index(j) for j in ('left_hip', 'left_knee')]
    hi = [float(bemf.bounds(np.full(len(bemf.DRIVEN), s), drives.PACK_V)[1][j]) for s, j in zip(slews, joints)]
    report.check('the kip: %.0f mm up in %.0f ms, %.1f J; hip %.1f, knee %.1f rad/s leave %.0f, %.0f N m of %.0f '
                 '(%.0f V, %.1f rad/s at most)' % (
                     1e3 * h, 1e3 * math.sqrt(2.0 * h / 9.81), weight * h, *slews, *hi, bemf.PEAK[joints[0]],
                     drives.PACK_V, bemf.REACH * drives.PACK_V / bemf.E[joints[0]]),
                 min(h_ - bemf.PEAK[j] for h_, j in zip(hi, joints)) >= 0.0)


ROSTER = [test_the_texts_read_whole, test_the_work_loop_names_a_damping, test_her_patches_under_her_weight,
          test_a_tamp_commits_only_where_snow_bears_her, test_a_bed_under_her_still_centre]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
