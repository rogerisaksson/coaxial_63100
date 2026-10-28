#!/usr/bin/env python3
"""Her gaits off her feet (`machine.crawl`; the kernel's terrestrial_multimodal_gaits.lisp,
terrestrial_gait_specializers.lisp, terrestrial_locomotion_patches.lisp): the texts read whole, her hinges sideways,
both crabs holding her, a sled and its anchors, the four origins on her figure, the supine ratchet on her
drives; the degraded scoot and crawl (degraded_scoot_and_crawl_*.lisp) read whole, on her figure, a dead leg of hers
and her sleds."""
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
    manifold they name a patch; the stages; the schedule five segments of 8 bars - the dog's trot, her back's crawl
    with its stair ratchet, the scoot and its hip drag, the crab on her back to her right and belly down to her left -
    each on the overlay, each crab's outer pair half a turn ahead of the inner."""
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
                 [c[:2] for c in clocks] == [(8.0 * n, 8.0 * n + 8.0) for n in range(5)]
                 and clocks[0][2] == (0.0, 0.5, 0.5, 0.0, 0.0) and clocks[2][2] == (0.0, 0.0, 0.0, 0.0, 0.5)
                 and clocks[3][2] == (0.5, 0.5, 0.0, 0.0, 0.0) and clocks[3][4][1] < 0.0
                 and clocks[4][2] == (0.0, 0.0, 0.5, 0.5, 0.0) and clocks[4][4][1] > 0.0
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
    """`crawl.coronal` of the schedule's twists - the dog and her back's crawl 0, the scoot's 0.275, the crab's 1 - and
    the turnout they ask, every heading: her hips' at most their stop; her shoulders', a turn she lacks - they only
    flex. In the crab's origin her abduction moves her ankle sideways more than her knee, her hip's turn least; her
    trunk's lateral bend carries her shoulders sideways for her hands."""
    import numpy as np
    from machine import crawl, drives, figure, kernel
    from machine.term_rows import ABSENT
    plan = _plan()
    kappa = [float(crawl.coronal(g['axial-torus-clock']['desired-se2-twist'])) for _a, _b, g in plan]
    stop = math.radians(drives.TRAVEL['hip_yaw'][1])
    asked = [g['coxa-turnout'] for _a, _b, g in plan]
    heading = np.linspace(0.0, 2.0 * np.pi, 361)
    twists = np.stack([np.cos(heading), np.sin(heading), np.zeros_like(heading)], axis=1)
    most = max(float(crawl.turnout(twists, a['max-hip-external-rot-rad']).max()) for a in asked)
    report.check('kappa %s; her hips turned %.3f rad at most, her stop %.3f; her shoulders asked %s, lacking' % (
        [round(k, 3) for k in kappa], most, stop, sorted({a['max-shoulder-external-rot-rad'] for a in asked})),
        kappa[:2] == [0.0, 0.0] and abs(kappa[3] - 1.0) < 1e-12 and most <= stop + 5e-4
        and ('shoulder', 'lateral-rotation') in ABSENT)
    atlas = kernel.read()['terrestrial-1jet-posture-atlas']['postures']
    deg = _laid(atlas['terrestrial-coronal-crab']['origin-q0'])
    belly = _laid(dict(atlas['terrestrial-pronograde-dog']['origin-q0'], **{'coxofemoral-ext-rot': stop}))

    def ankle(d):
        return figure.frames(d, (0.0, 0.0, 0.0), figure.IDENT)['left_foot'][0]

    def shoulder(d):
        return figure.frames(d, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0]
    shares = {j: _stroke(deg, 'left_' + j, ankle) for j in ('hip_roll', 'knee', 'hip_yaw')}
    down = {j: _stroke(belly, 'left_' + j, ankle) for j in ('hip_roll', 'knee', 'hip_yaw')}
    low, high = drives.TRAVEL['spine_roll']
    bend = abs(shoulder(dict(deg, spine_roll=high))[0] - shoulder(dict(deg, spine_roll=low))[0])
    report.check("sideways of their strokes on her back: abduction %.2f, knee %.2f, hip turn %.2f, elbow %.2f; her "
                 "trunk's bend carries her shoulders %.2f m. Belly down: abduction %.2f, knee %.2f, hip turn %.2f" % (
                     shares['hip_roll'], shares['knee'], shares['hip_yaw'], _stroke(deg, 'left_elbow', figure.fist),
                     bend, down['hip_roll'], down['knee'], down['hip_yaw']),
                 shares['hip_roll'] > shares['knee'] > shares['hip_yaw'] and down['hip_roll'] > down['knee'])


def test_her_crabs_hold_her(report):
    """Each crab laid rigid on a hard floor - on her back the crab's origin on her soles and fists, its straight arms
    within 1 deg of plumb; belly down the dog's, turned out to her hip's stop, on her toes and fists - her legs
    bearing the share the crab tells to 0.01; there every drive's static torque - her contacts' push along the floor's
    up through each joint, by virtual work - within what it holds for ever (`heat.held`)."""
    import numpy as np
    from machine import figure, groundwork, heat, kernel
    from test_groundwork import TURN, _part, _skin
    atlas, plan = kernel.read()['terrestrial-1jet-posture-atlas']['postures'], _plan()
    belly = dict(atlas['terrestrial-pronograde-dog']['origin-q0'],
                 **{'coxofemoral-ext-rot': plan[4][2]['coxa-turnout']['max-hip-external-rot-rad']})
    phi = np.linspace(-np.pi, np.pi, TURN, endpoint=False)
    up, weight = groundwork.pitched(phi), figure.mass() * 9.81
    for name, origin, sole, at in (('on her back', atlas['terrestrial-coronal-crab']['origin-q0'], 'foot', plan[3][2]),
                                   ('belly down', belly, 'toes', plan[4][2])):
        deg = _laid(origin)
        points, radii, of, centre = _skin(deg)
        names = np.array([_part(s) for s in of])
        gap = groundwork.clear(points, radii, up)
        rests = groundwork.settled(groundwork.rest(points, radii, centre, up))
        worst = np.array([gap[rests][:, names == p].min(axis=1) for p in (sole, 'hand')]).max(axis=0)
        k = rests[int(np.argmin(worst))]
        u, ahead, touch = up[k], np.array([0.0, -np.sin(phi[k]), np.cos(phi[k])]), gap[k] < TOUCH
        feet, hands = ((points[(names == p) & touch] @ ahead).mean() for p in (sole, 'hand'))
        legs, told = (hands - centre @ ahead) / (hands - feet), at['coxa-turnout']['hind-to-fore-thermal-load-bias']
        arm = np.subtract(figure.fist(deg), figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0])
        tilt = math.degrees(math.atan2(arm @ ahead, -(arm @ u)))
        torques = {}
        for joint, part, share in (('hip', sole, legs), ('hip_roll', sole, legs), ('hip_yaw', sole, legs),
                                   ('knee', sole, legs), ('ankle', sole, legs), ('shoulder', 'hand', 1.0 - legs),
                                   ('elbow', 'hand', 1.0 - legs)):
            on = np.array([s == 'left_' + part for s in of]) & touch
            moved = [_skin(dict(deg, **{'left_' + joint: deg['left_' + joint] + sign * math.degrees(NUDGE)}))[0][on]
                     .mean(axis=0) for sign in (-1.0, 1.0)]
            torques[joint] = 0.5 * share * weight * float(u @ np.subtract(moved[1], moved[0])) / (2.0 * NUDGE)
        held = {j: heat.held('left_' + j)[0] for j in torques}
        report.check('%s: on her %s at %+.0f deg, pelvis %.0f mm up, arms %.1f deg off plumb, legs %.3f (told %.2f); '
                     'static N m (held) %s' % (
                         name, ', '.join(sorted({_part(of[i]) for i in np.flatnonzero(touch)})), math.degrees(phi[k]),
                         1e3 * gap[k][names == 'pelvis'].min(), tilt, legs, told,
                         ', '.join('%s %.1f (%.1f)' % (j, abs(t), held[j]) for j, t in torques.items())),
                     worst.min() < TOUCH and abs(legs - told) < 0.01 and (abs(tilt) < 1.0 or sole == 'toes')
                     and all(abs(t) <= held[j] for j, t in torques.items()))


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
    patches bear best and the float of their worst part: the dog and the crab on their own."""
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
                 min(rests) >= -TOUCH
                 and [n for n, b in bears.items() if b] == ['terrestrial-pronograde-dog', 'terrestrial-coronal-crab'])


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


#: The degraded scoot and crawl texts; the limb each of their suite's four segments breaks.
DEGRADED = ('degraded_scoot_and_crawl_core.lisp', 'degraded_scoot_and_crawl_specializers.lisp',
            'degraded_scoot_and_crawl_patches.lisp')
BROKEN = (None, ':hind-left', None, ':hind-left')


def _degraded():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-scoot-and-lowcrawl-suite-soa'])


def test_the_degraded_texts_read_whole(report):
    """The degraded scoot and crawl read with the kernel: two postures, each its origin and three columns; its frog
    piston on her shank; every manifold they name a patch; the stages; the schedule four segments of 8 bars, the
    second and fourth her left leg in High-Z."""
    from machine import dsp, figure, kernel
    got = kernel.read()
    atlas = got['scoot-and-lowcrawl-1jet-atlas']['postures']
    report.check('two postures, each its origin and three columns',
                 len(atlas) == 2 and all(len(p['modal-basis-columns']) == 3 and 'origin-q0' in p for p in atlas.values()))
    frog = got['low-crawl-manifold-extensions']['medial-crural-frog-piston']['cop-bounds-mm']
    r = max(c[1] for c in figure.BODY if c[0] == 'shank')
    report.check('its frog piston %g by %g mm on her shank %.0f by %.0f' % (
        2 * frog['x'][1], 2 * frog['y'][1], 1e3 * figure.SHANK, 2e3 * r),
        2e-3 * frog['x'][1] <= figure.SHANK and 1e-3 * frog['y'][1] <= r)
    folder = os.path.join(os.path.dirname(dsp.__file__), 'ortho_kernel')
    texts = []
    for name in DEGRADED:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions'],
               **got['low-crawl-manifold-extensions']}
    named = {m for x in texts for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx|frog-piston))(?![a-z0-9-])', x)}
    report.check('%d manifolds named, each a patch' % len(named), named <= set(patches))
    stages = [n for x in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', x)]
    report.check('%d stages' % (len(stages) - 1),
                 all(isinstance(got[n], dsp.Stage) for n in stages if n != 'compile-scoot-and-lowcrawl-suite-soa'))
    plan = _degraded()
    broke = tuple((g.get('limb-health') or {}).get('limb') for _a, _b, g in plan)
    report.check('the schedule %s, broken %s' % ([(b0, b1) for b0, b1, _g in plan], broke),
                 [(b0, b1) for b0, b1, _g in plan] == [(8.0 * n, 8.0 * n + 8.0) for n in range(4)] and broke == BROKEN)


def test_the_degraded_origins_on_her_figure(report):
    """The scoot's and the low crawl's origins as her movements: her shoulders' abduction she lacks, nothing past her
    stops - the frog's 75 deg turnout hers at her hip yaw's 25."""
    from machine import kernel
    from machine.term_rows import ABSENT, TERMS
    atlas = kernel.read()['scoot-and-lowcrawl-1jet-atlas']['postures']
    told = [(name, n, rad) for name, p in atlas.items() for n, rad in p['origin-q0'].items()]
    lacks = sorted({'%s %s' % (p, n) for p, n, _r in told if MOVES[n] in ABSENT})
    stops = {m: (-math.inf if x[2] is None else x[2], math.inf if x[3] is None else x[3]) for m, x in TERMS.items()}
    out = sorted({'%s %s %.0f deg' % (p, n, math.degrees(r)) for p, n, r in told if MOVES[n] in TERMS
                  and not stops[MOVES[n]][0] - 1e-6 <= math.degrees(r) <= stops[MOVES[n]][1] + 1e-6})
    report.check('she lacks: %s; past her stops: %s' % ('; '.join(lacks), '; '.join(out) or 'none'),
                 bool(lacks) and not out)


def test_a_dead_leg_and_her_sleds(report):
    """Her left hip and knee in High-Z (`crawl.partitioned`, her law's model): her active mass matrix less what they
    take positive definite, their accelerations her model's own; her dead knee's backdrive against her shank's weight
    held level; the escort's roll lifting her far hip; the arm her yaw asks; where her anchors hold each sled of the
    suite, on her patches' friction."""
    import numpy as np
    from machine import crawl, drives, dsp, kernel, observer, wbc
    b = wbc.Body()
    b.mj.mj_forward(b.m, b.d)
    M = np.zeros((b.m.nv, b.m.nv))
    b.mj.mj_fullM(b.m, b.d, M)
    h = b.d.qfrc_bias.copy()
    free = np.zeros(b.m.nv, bool)
    for j in ('left_hip_yaw', 'left_hip_roll', 'left_hip', 'left_knee'):
        free[b.m.jnt_dofadr[b.m.joint(j).id]] = True
    Mbar, A, off = crawl.partitioned(M, h, free)
    qdd = np.linalg.solve(M, -h)
    least = float(np.linalg.eigvalsh(Mbar).min())
    report.check('her active block %d by %d, its least eigenvalue %.3g; her dead leg %.1g off her model' % (
        Mbar.shape[0], Mbar.shape[1], least, float(np.abs(A @ qdd[~free] + off - qdd[free]).max())),
        least > 0.0 and np.allclose(A @ qdd[~free] + off, qdd[free]))
    parts = [b.m.body(n).id for n in ('left_shank', 'left_foot', 'left_toes')]
    mass = sum(b.m.body_mass[p] for p in parts)
    centre = sum(b.m.body_mass[p] * b.d.xipos[p] for p in parts) / mass
    held, hangs = drives.backdrive('left_knee'), mass * wbc.G * np.linalg.norm(centre - b.d.xanchor[b.m.joint('left_knee').id])
    report.check('her dead knee holds %.2f N m, her shank and foot %.2f kg level turn it %.1f: it folds' % (held, mass, hangs),
                 held < hangs)
    pelvis = b.m.body('pelvis').id
    R = b.d.xmat[pelvis].reshape(3, 3)
    ischia = [b.d.xpos[pelvis] + R @ np.array(p) for sh, seg, p, _n in observer.POINTS
              if seg == 'pelvis' and sh[0][0] == 'ischial-bilateral']
    hips = [b.d.xanchor[b.m.joint(j).id] for j in ('left_hip', 'right_hip')]
    across = max(abs(i[0] - hips[1][0]) for i in ischia)
    tow = kernel.said('escort-passive-towed-limb', 'coronal-unweight-tilt-rad')
    report.check('a %.2f rad roll onto one buttock lifts her far hip %.1f mm, %.0f mm across (the text 35)' % (
        tow, 1e3 * crawl.lifted(across, tow), 1e3 * across), abs(crawl.lifted(across, tow) - 0.035) < 0.002)
    shoulders = [b.d.xanchor[b.m.joint(j).id] for j in ('left_shoulder', 'right_shoulder')]
    hands, heel = 0.5 * abs(shoulders[0][0] - shoulders[1][0]), 0.5 * abs(hips[0][0] - hips[1][0]) - 0.065
    report.check('her arm on the dead side pushes %.2f of the other for no yaw, her heel %.0f mm off her midline' % (
        crawl.boost(hands, heel), 1e3 * heel), abs(crawl.boost(hands, heel) - 1.20) < 0.01)
    got = kernel.read()
    mu = {k: v['default-mu'] for name in ('anatomical-manifold-registry', 'groundwork-manifold-extensions',
                                          'low-crawl-manifold-extensions') for k, v in got[name].items()}
    seal = dsp.value(['specialize-low-crawl-frog-piston-topology', 'nominal-biped-gait', ':drive-topology',
                      ':arms-only-seal-drag'], got)['stick-slip']
    said = []
    for name, node in (('scoot', _degraded()[0][2]['stick-slip']), ('low crawl', _degraded()[2][2]['stick-slip']),
                       ('seal drag', seal)):
        a = float(np.mean([mu[p.lstrip(':')] for p in node['anchors']]))
        s = float(np.mean([mu[p.lstrip(':')] for p in node['sleds']]))
        gamma = node['pelvic-unweighting-gain']
        said.append((name, gamma, float(crawl.held(gamma, a, s)), s / (a + s)))
    report.check('of the half turn her anchors hold: %s' % '; '.join(
        '%s at %.2f %.3f (it needs %.3f)' % x for x in said), all(x[2] > 0.0 for x in said))


ROSTER = [test_the_texts_read_whole, test_her_hinges_sideways, test_her_crabs_hold_her, test_a_sled_and_its_anchors,
          test_the_origins_on_her_figure, test_the_supine_ratchet_on_her_drives, test_the_degraded_texts_read_whole,
          test_the_degraded_origins_on_her_figure, test_a_dead_leg_and_her_sleds]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
