#!/usr/bin/env python3
"""At a seat's or a bed's edge (`machine.bedside`; the kernel's bedside_*.lisp): the texts read whole, what meets a
fixture behind her, a stool without a back, the bed's pendulum on her body, her roll held on the bed's line."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_groundwork import LANDMARKS, TOUCH, _laid, _part, _skin

#: The bedside texts, lisp/humanoid/kernel/.
TEXTS = ('bedside_and_stool_core.lisp', 'bedside_maneuver_specializers.lisp', 'bedside_choreography_patches.lisp')
#: The texts' landmarks by the segment of hers that carries each.
CARRIED = {**LANDMARKS, 'shank-crural': 'shank', 'glenohumeral': 'upper_arm', 'cranial-vault': 'head'}
#: Her capsules sampled this often along their axes, m; heights stepped, m; samples round a turn; a law pass, s.
STEP_M, RISE_M, TURN, PASS_S = 0.005, 0.01, 7200, 1e-3
#: A sole's or toes' box as a lattice of this many a side; the stool's lean and the cot's elbow fold stepped, rad.
LATTICE, LEAN_STEP, FOLD_STEP = 7, 1e-3, 0.01


def _plan():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-complete-stool-and-bed-lifecycle-soa'])


def _shots():
    """The schedule's three gaits: the stool, the bed lain along, the roll and the rise."""
    return [seg[1] for seg in _plan() if seg[0] == 'oneshot']


def _patches():
    from machine import kernel
    got = kernel.read()
    return {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}


def _keys(shot, stage):
    """The keys of a shot's last call of `stage`."""
    return [keys for name, keys in shot['_trace'] if name == stage][-1]


def test_the_texts_read_whole(report):
    """The texts read with the kernel: their stages; every manifold they name a patch; every landmark they probe, prop
    or anchor on a segment of hers, those on no patch of her registry named; the stool, 3 s, the bed, 2 s, the roll;
    the pendulum a quarter turn each way."""
    from machine import dsp, kernel, mechatronics
    got, patches, plan, shots = kernel.read(), _patches(), _plan(), _shots()
    folder = os.path.join(mechatronics.LISP, 'humanoid', 'kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    named = {m for t in texts
             for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx))(?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch or a landmark of hers: %s' % (
        len(named), ', '.join(sorted(named - set(patches)))),
        all(m[:-len('-bilateral')] in CARRIED for m in named - set(patches)))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages' % (len(stages) - 1), all(isinstance(got[n], dsp.Stage) for n in stages
                                                      if n != 'compile-complete-stool-and-bed-lifecycle-soa'))
    report.check('the schedule: %s' % ', '.join(seg[1]['_trace'][-1][0] if seg[0] == 'oneshot' else '%g s' % seg[1]
                                               for seg in plan),
                 [seg[1] for seg in plan if seg[0] == 'dwell'] == [3.0, 2.0]
                 and [g['_trace'][-1][0] for g in shots] == ['patch-turn-and-sit-high-stool',
                                                             'patch-dismount-stool-and-mount-bed-longside',
                                                             'patch-bed-roll-side-stomach-back-and-rise'])
    frames = {f.lstrip(':') for p in patches.values() for f in p['frames']}
    used = {str(f).lstrip(':') for g in shots for f in g['calf-datum']['probe-capsules']} | {
        str(f).lstrip(':') for g in shots[1:] for f in (
            *g['bed-roll']['shrimp-counter-translation']['hip-escape-anchor-frames'],
            g['bed-pendulum']['cantilever-seesaw-coupling']['upper-body-prop-frame'],
            g['bed-datum']['cranial-capsule'])}
    landmark = {f: re.sub(r'-(l|r|bilateral)$', '', f) for f in used | frames}
    patched = {landmark[f] for f in frames}
    report.check('probed, propped on, anchored: %s; on no patch of her registry her %s' % (
        ', '.join('%s (her %s)' % (f, CARRIED[landmark[f]]) for f in sorted(used)),
        ', '.join(sorted({CARRIED[landmark[f]] for f in used if landmark[f] not in patched}))),
        all(landmark[f] in CARRIED for f in used))
    turns = [(g['bed-pendulum']['direction'], g['bed-pendulum']['target-yaw-rotation-rad']) for g in shots[1:]]
    report.check('the pendulum: %s' % '; '.join('%s %+.4f rad' % t for t in turns),
                 turns == [(':ingress-to-longitudinal-supine', 1.570796), (':egress-to-transverse-seated', -1.570796)])


def _dense(deg):
    """(points, radii, parts) of her skin posed `deg`, her soles on the floor: a capsule of `figure.BODY` a sphere every
    STEP_M along it, a sole's and toes' box a lattice at 0; her fingers left out."""
    import itertools
    import numpy as np
    from machine import figure
    segments = {s[0] for s in figure.SEGMENTS if not s[0].endswith('fingers')}
    points, radii, of = [], [], []
    for name, (at, here) in figure.frames(deg, (0.0, figure.standing(), 0.0), figure.IDENT).items():
        for r, a, b in [(r, a, b) for p, r, a, b in figure.BODY if name in segments and p == _part(name)]:
            for s in np.linspace(0.0, 1.0, int(math.ceil(math.dist(a, b) / STEP_M)) + 1):
                points.append(np.add(at, figure.apply(here, tuple(np.add(a, s * np.subtract(b, a))))))
                radii.append(r)
                of.append(_part(name))
        for size, c in [(size, c) for p, _s, size, c in figure.CONTACTS if name in segments and p == _part(name)]:
            for s in itertools.product(np.linspace(-1.0, 1.0, LATTICE), repeat=3):
                points.append(np.add(at, figure.apply(here, tuple(c[i] + s[i] * size[i] for i in range(3)))))
                radii.append(0.0)
                of.append(_part(name))
    return np.array(points), np.array(radii), np.array(of)


def _face(name):
    """(bottom, top) m of the lodge's box `name` as furnished (`room.furnish`)."""
    from machine import room
    xml = next(b for b in room.furnish('') if b.startswith('<body name="%s"' % name))
    mid = float(re.findall(r'pos="\S+ (\S+) ', xml)[0]) + sum(float(y) for y in re.findall(r' pos="0 (\S+) 0"', xml))
    half = float(re.findall(r'size="\S+ (\S+) ', xml)[0])
    return mid - half, mid + half


def test_what_meets_a_fixture_behind_her(report):
    """Standing, her skin's reach behind her hips by height (`bedside.behind`): heels, calves, thighs, buttocks. The
    chairs' seat edges and the lodge bed's side meet her calves, on no patch of hers; an edge over her knee her thighs'
    backs, the stool's probe. From each turn's standoff the stool and the bed meet her within the text's seek."""
    import numpy as np
    from machine import bedside, figure, floor
    points, radii, of = _dense(figure.stand())
    heights = np.arange(0.0, 1.0, RISE_M)
    reach = bedside.behind(points, radii, heights, heights + RISE_M)
    first = of[np.argmax(reach, axis=1)]
    cuts = [0] + [k for k in range(1, len(first)) if first[k] != first[k - 1]] + [len(first)]
    runs = [(a, b) for a, b in zip(cuts[:-1], cuts[1:]) if np.isfinite(reach[a:b].max())]
    report.check('behind her hips: %s' % '; '.join('%s %.2f-%.2f m, %.0f mm' % (
        first[a], heights[a], heights[b - 1], 1e3 * reach[a:b].max()) for a, b in runs),
        [first[a] for a, _b in runs] == ['foot', 'shank', 'thigh', 'pelvis', 'torso'])
    (h0, h1), (s0, s1), (t0, t1) = runs[:3]
    thick = 2.0 * floor.CHAIR_SEAT[1]
    faces = (('the chair', floor.CHAIR_SEAT_H - thick, floor.CHAIR_SEAT_H),
             ('the low chair', floor.CHAIR_LOW_H - thick, floor.CHAIR_LOW_H), ('the lodge bed', *_face('bed')))
    met = [(name, lo, hi, of[int(np.argmax(each))], 1e3 * each.max())
           for (name, lo, hi), each in zip(faces, bedside.behind(points, radii, *zip(*[f[1:] for f in faces])))]
    report.check('; '.join('%s %.2f-%.2f m meets her %s %.0f mm behind' % m for m in met),
                 [m[3] for m in met] == ['shank'] * 3)
    patches, shots = _patches(), _shots()
    stool, bed = (_keys(g, 'xform-stand-and-turn-at-fixture') for g in shots[:2])
    thighs = next(n for n, p in patches.items() if set(p['frames']) == set(stool['probe-capsules']))
    report.check('her calves %.2f-%.2f m on no patch; the stool probes her thighs %.2f-%.2f m, her %s' % (
        heights[s0], heights[s1 - 1], heights[t0], heights[t1 - 1], thighs),
        not any(f.startswith(':shank') for p in patches.values() for f in p['frames'])
        and bed['probe-capsules'] == (':shank-crural-l', ':shank-crural-r'))
    heel, seek = reach[h0:h1].max(), shots[0]['calf-datum']['max-search-travel-m']
    travel = [(n, 1e-3 * keys['standoff-clearance-mm'] + heel - r) for n, keys, r in (
        ('the stool', stool, reach[t0:t1].max(axis=1).min()), ('the bed', bed, 1e-3 * met[2][4]))]
    report.check('her heels %.0f mm behind; from the turn %s; the seek %.0f' % (
        1e3 * heel, ', '.join('%s meets her in %.0f mm' % (n, 1e3 * t) for n, t in travel), 1e3 * seek),
        all(t < seek for _n, t in travel))


def _seated(bias=0.0, spine=0.0):
    """Her figure on the kernel's canonical ischial origin, legs hung as the stool's hang sets them, her lumbar `bias`
    rad more, her spine `spine` deg more."""
    from machine import kernel
    seated = kernel.read()['canonical-posture-soa-atlas']['postures']['ischial-seated']['origin-q0']
    hang = kernel.node('xform-sit-on-chair-or-stool', 'limb-hang')['override-origin']
    deg = _laid(dict(seated, **hang, **{'l4-l5-lumbar': seated['l4-l5-lumbar'] + bias}))
    return dict(deg, spine=deg['spine'] + spine)


def _seat(deg):
    """(her centre's z, her buttocks' z from .. to, m) of her figure posed `deg`: her pelvis's lowest skin."""
    import numpy as np
    points, radii, of, centre = _skin(deg)
    on = np.array([_part(s) == 'pelvis' for s in of])
    low = points[:, 1] - radii
    touch = on & (low - low[on].min() < TOUCH)
    return centre[2], points[touch, 2].min(), points[touch, 2].max()


def _hung(talocrural):
    """(kg m, kg m^2): her shank, foot and toes as built hung from her knee, her ankle at `talocrural` rad - their mass
    times their centre's depth under it, their inertia about it."""
    import numpy as np
    from machine import build, figure
    placed, built = figure.frames(_laid({'talocrural': talocrural}), (0.0, 0.0, 0.0), figure.IDENT), build.segments()
    knee = np.array(placed['left_shank'][0])
    moment = inertia = 0.0
    for name in ('left_shank', 'left_foot', 'left_toes'):
        kg, centre, about = built[name]
        r = np.array(figure.add(placed[name][0], figure.apply(placed[name][1], centre))) - knee
        moment += kg * -r[1]
        inertia += about[0] + kg * (r[1] ** 2 + r[2] ** 2)
    return moment, inertia


def test_a_stool_without_a_back(report):
    """The rim's barrier (`bedside.halted`) at her capture point's gamma halts the slide as exp(-gamma t), never past.
    Seated, legs hung, her centre is ahead of her buttocks, still at her spine's stop: she cannot lean off backward.
    Her centre over the patch's stool, her ischia keep its rim's margin as far as the text leans her. Her shank hangs
    at the text's frequency; on her toes she perches higher."""
    import numpy as np
    from machine import barrier, bedside, drives, figure, kernel, observer
    from machine.term_rows import TERMS
    shot = _shots()[0]
    speed = _keys(shot, 'xform-sit-on-chair-or-stool')['slide-speed-mps']
    seat = _keys(shot, 'specialize-with-stool-active-core-balance')['seat-diameter-mm']
    gap = [0.5e-3 * seat]
    for _ in range(int(2.0 / PASS_S)):
        gap.append(gap[-1] - PASS_S * float(bedside.halted(gap[-1], speed, barrier.GAMMA)))
    gap = np.array(gap)
    near = int(np.argmax(gap < speed / barrier.GAMMA))
    tail = gap[near:] / gap[near]
    report.check('its slide at %.2f m/s from the rim: %.1f mm short at %.0f ms, then exp(-%.2f t) at gamma %g, %.2f '
                 'mm short at 2 s' % (speed, 1e3 * gap[near], 1e3 * near * PASS_S,
                                      -np.log(tail[-1]) / (len(tail) - 1) / PASS_S, barrier.GAMMA, 1e3 * gap[-1]),
                 gap.min() > 0.0 and np.all(np.diff(gap) < 0.0)
                 and abs(tail[-1] - (1.0 - barrier.GAMMA * PASS_S) ** (len(tail) - 1)) < 1e-12)
    bias = kernel.node('enforce-backless-stool-rim-zcbf', 'seat-rim')['on-backless-stool-detected'][
        'self-balancing-lumbar-pitch-bias-rad']
    upright, biased, back = _seat(_seated()), _seat(_seated(bias)), _seat(_seated(spine=drives.TRAVEL['spine'][0]))
    report.check('her centre %.0f mm ahead of her buttocks (%.0f long), %.0f leaning the text\'s %+.3f rad, %.0f at '
                 'her spine\'s %g deg' % (1e3 * (upright[0] - upright[2]), 1e3 * (upright[2] - upright[1]),
                                          1e3 * (biased[0] - biased[2]), bias, 1e3 * (back[0] - back[2]),
                                          drives.TRAVEL['spine'][0]), upright[0] > upright[2] and back[0] > back[2])
    ischia = next(p for _s, seg, p, _n in observer.POINTS if seg == 'pelvis' and p is not None and p[1] < 0.0)[2]
    margin = shot['seat-rim']['perimeter-margin-m']
    leans = np.arange(0.0, 0.2, LEAN_STEP)
    inside = np.array([0.5e-3 * seat - (_seat(_seated(b))[0] - ischia) for b in leans])
    most = leans[inside >= margin].max()
    report.check('her centre over a %.0f mm stool, her ischia %.1f mm inside its rim upright, %.1f leaning; its margin '
                 '%.1f: she leans at most %.3f rad, the text %.3f' % (
                     seat, 1e3 * inside[0], 1e3 * float(np.interp(bias, leans, inside)), 1e3 * margin, most, bias),
                 inside[0] >= margin and bias <= most)
    hang = kernel.node('xform-sit-on-chair-or-stool', 'limb-hang')
    moment, inertia = _hung(hang['override-origin']['talocrural'])
    swing, told = float(bedside.pendulum(moment, inertia)), hang['passive-pendulum-dynamics']['eigenfreq-hz']
    report.check('her shank hangs %.3f kg m, %.4f kg m^2 about her knee: %.3f Hz, the text %.2f' % (
        moment, inertia, swing, told), abs(swing - told) < 5e-3)
    points, radii, of = _dense(figure.stand())
    toes = TERMS[('ankle', 'plantarflexion')]
    raised = _dense(dict(figure.stand(), left_ankle=toes[1] * toes[3], right_ankle=toes[1] * toes[3],
                         left_foot=-toes[3], right_foot=-toes[3]))
    up = (points[:, 1] - radii).min() - (raised[0][:, 1] - raised[1]).min()
    perch = (points[of == 'pelvis', 1] - radii[of == 'pelvis']).min()
    report.check('her buttocks %.3f m up, on her toes at her %g deg stop %.3f' % (perch, toes[3], perch + up), up > 0.0)


def _deep():
    """m: her hips in from a bed's edge sat her deepest, the edge at her calves."""
    import numpy as np
    points, radii, of, _c = _skin(_seated())
    shank = np.array([_part(x) == 'shank' for x in of])
    return (points[shank, 2] - radii[shank]).min()


def _rolled(deg, side):
    """(her centre's height over a hard floor, rolled from her back onto `side`, round a turn; her skin across the
    floor from her centre at her back, side and front, (3, 2) m; the samples) of her figure posed `deg`."""
    import numpy as np
    from machine import groundwork
    points, radii, _of, centre = _skin(deg)
    theta = np.linspace(0.0, 2.0 * np.pi, TURN + 1)
    up = groundwork.rolled(theta, side)
    across = np.cross(up[[0, TURN // 4, TURN // 2]], [0.0, 1.0, 0.0])
    s, c = across @ points.T, across @ centre
    spans = np.stack([(s - radii).min(axis=1) - c, (s + radii).max(axis=1) - c], axis=1)
    return groundwork.rest(points, radii, centre, up), spans, theta


def _half(origin, fold=0.0):
    """m: her half width lying on her back posed as the groundwork atlas's `origin`, her elbows `fold` rad more, and
    on her side."""
    import numpy as np
    from machine import kernel
    posed = kernel.read()['groundwork-1jet-posture-atlas']['postures'][origin]['origin-q0']
    spans = _rolled(_laid(dict(posed, cubital=posed['cubital'] + fold)), -1.0)[1]
    return np.abs(spans[0]).max(), np.abs(spans[1]).max()


def _stroke():
    """m: her pelvis across her planted shoulders (the text's glenohumeral anchors, their joints' middle) at her
    spine's lateral stop."""
    from machine import figure, drives
    at = [figure.frames(dict(figure.stand(), spine_roll=r), (0.0, 0.0, 0.0), figure.IDENT) for r in (
        0.0, drives.TRAVEL['spine_roll'][1])]
    straight, bent = (0.5 * (a['left_upper_arm'][0][0] + a['right_upper_arm'][0][0]) for a in at)
    return abs(bent - straight)


def test_the_bed_pendulum_on_her_body(report):
    """Lying straight, on the lodge bed her pelvis lies between the pillow's clearance past her crown and its foot
    less her legs. Sat her deepest on its side the pendulum lays her there: within the bed, inside the roll's margins
    only on its line, one stroke of her escape away. Her hip holds a leg out straight."""
    import numpy as np
    from machine import build, figure, heat, kernel, room
    points, radii, _of, _c = _skin(_laid({}))
    crown, soles = (points[:, 1] + radii).max(), -(points[:, 1] - radii).min()
    length, width = 2.0 * room.BED[2], 2.0 * room.BED[0]
    clear = kernel.node('specialize-with-bed-headboard-pillow-align', 'bed-datum')['headboard-wall-margin-m']
    report.check('her crown %.3f m over her pelvis, her soles %.3f under: on the %.1f m bed her pelvis %.3f-%.3f m '
                 'from its head' % (crown, soles, length, crown + clear, length - soles),
                 crown + clear < length - soles)
    deep, half = _deep(), _half('floor-supine-flat')[0]
    margin = _shots()[1]['bed-roll']['mattress-drop-off-barrier']['min-capsule-to-edge-margin-m']
    report.check('sat her deepest her hips %.3f m in, %.3f off its line; on her back %.3f m wide, %.0f mm inside the '
                 'edge, inside its %.0f mm margins within %.1f mm of the line; her stroke %.3f' % (
                     deep, 0.5 * width - deep, 2.0 * half, 1e3 * (deep - half), 1e3 * margin,
                     1e3 * (0.5 * width - margin - half), _stroke()),
                 deep > half and 0.5 * width - margin - half >= 0.0 and 0.5 * width - deep < _stroke())
    built, held, out = build.segments(), heat.held('left_hip')[0], []
    for knee in (0.0, -kernel.node('xform-sit-on-chair-or-stool', 'limb-hang')['override-origin']['tibiofemoral']):
        placed = figure.frames(dict(figure.stand(), left_hip=-90.0, left_knee=-math.degrees(knee), left_ankle=0.0),
                               (0.0, 0.0, 0.0), figure.IDENT)
        hip = placed['left_thigh'][0][2]
        out.append(9.81 * sum(built[n][0] * (figure.add(placed[n][0], figure.apply(placed[n][1], built[n][1]))[2] - hip)
                              for n in ('left_thigh', 'left_shank', 'left_foot', 'left_toes')))
    report.check('a leg out straight %.1f N m at her hip, its knee hung %.1f; it holds %.1f' % (out[0], out[1], held),
                 max(out) < held)


def test_her_roll_held_on_the_bed(report):
    """`bedside.drift` is R theta for a cylinder. Her roll wave's posture rolled from her back onto her right drifts
    her the text's radius times the quarter turn. Her room on the bed at rest; the escape (`bedside.escape`) brings
    her onto its line through that quarter in strokes of hers; the texts' strokes within hers at her shoulders; on the
    cot her elbows folded as little as fits her between its margins."""
    import numpy as np
    from machine import bedside, kernel, room
    from test_groundwork import _roll_postures
    theta = np.linspace(0.0, 2.0, 2001)
    worst = np.abs(bedside.drift(np.full_like(theta, 0.2), theta) - 0.2 * theta).max()
    report.check('a cylinder of 0.2 m drifts R theta within %.0e' % worst, worst < 1e-12)
    shots = _shots()
    told, barrier = (shots[1]['bed-roll'][k] for k in ('shrimp-counter-translation', 'mattress-drop-off-barrier'))
    h, _spans, theta = _rolled(_roll_postures()[2], -1.0)
    quarter = bedside.drift(h, theta)[TURN // 4]
    report.check('her centre %.3f m up on her back, %.3f on her side; she drifts %.3f m the quarter: radius %.3f, the '
                 'text %.3f' % (h[0], h[TURN // 4], quarter, quarter / (0.5 * np.pi), told['effective-torso-radius-m']),
                 abs(told['effective-torso-radius-m'] - quarter / (0.5 * np.pi)) < 5e-4)
    half = (*_half('floor-supine-flat'), _half('floor-prone-flat')[0])
    width, margin = barrier['bed-width-m'], barrier['min-capsule-to-edge-margin-m']
    stray = [0.5 * width - margin - x for x in half]
    report.check('on the %.1f m bed, its %.0f mm margins, she may stray %s mm on her back / side / front, %s wide' % (
        width, 1e3 * margin, ' / '.join('%.0f' % (1e3 * x) for x in stray),
        ' / '.join('%.0f' % (2e3 * x) for x in half)),
        width == 2.0 * room.BED[0] and min(stray) >= 0.0)
    rate = kernel.node('xform-continuous-ground-roll', 'body-roll')['nominal-omega-radps']
    gain, stroke, off = told['centerline-restoring-gain'], _stroke(), [0.5 * width - _deep()]
    scoot, dt = 0.0, (theta[1] - theta[0]) / rate
    for k in range(TURN // 4):
        v = float(bedside.escape(h[k], rate, off[-1], gain))
        off.append(off[-1] + dt * (h[k] * rate + v))
        scoot += dt * abs(v)
    report.check('the escape at %g /s from %.0f mm off: %.1f mm after the quarter (%.2f s), her pelvis scooting '
                 '%.3f m, %d strokes of her %.0f mm' % (gain, 1e3 * off[0], 1e3 * abs(off[-1]), TURN // 4 * dt, scoot,
                                                        math.ceil(scoot / stroke), 1e3 * stroke),
                 abs(off[-1]) < 0.01 * off[0])
    cot = kernel.read()['specialize-with-shrimp-centerline-recentering'].keys
    narrow = kernel.node('specialize-with-shrimp-centerline-recentering', 'bed-roll')['mattress-drop-off-barrier']
    strokes = (told['pelvic-scoot-stroke-mm'], cot['shrimp-stroke-mm'],
               round(1e3 * shots[2]['stick-slip']['stroke-amplitude-m'], 6))
    fold = dict(kernel.node('specialize-with-shrimp-centerline-recentering', 'posture-delta')[
        'delta-angles'])[':cubital']
    room = 1e-3 * cot['cot-width-mm'] - 2.0 * narrow['min-capsule-to-edge-margin-m']
    folded, short = (2.0 * _half('floor-supine-flat', f)[0] for f in (fold, fold - FOLD_STEP))
    report.check('the strokes %s mm, hers %.0f; on the %.0f mm cot her elbows %.2f rad more: %.0f mm wide, its %.0f '
                 'between margins, %.0f a step less' % (', '.join('%.0f' % x for x in strokes), 1e3 * stroke,
                                                         cot['cot-width-mm'], fold, 1e3 * folded, 1e3 * room,
                                                         1e3 * short),
                 max(strokes) <= round(1e3 * stroke) and cot['shrimp-stroke-mm'] == round(1e3 * stroke)
                 and folded <= room < short)


ROSTER = [test_the_texts_read_whole, test_what_meets_a_fixture_behind_her, test_a_stool_without_a_back,
          test_the_bed_pendulum_on_her_body, test_her_roll_held_on_the_bed]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
