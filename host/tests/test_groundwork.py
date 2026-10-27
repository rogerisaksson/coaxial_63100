#!/usr/bin/env python3
"""Her body on the floor (`machine.groundwork`; the kernel's floor_groundwork_core.lisp,
ground_rolling_and_transitions.lisp, floor_lifecycle_patches.lisp): the texts read whole, the roll's ring and girdles,
her roll on a hard floor and out of a mattress's trough, the six ground postures laid on her figure."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The groundwork texts, machine/ortho_kernel/.
TEXTS = ('floor_groundwork_core.lisp', 'ground_rolling_and_transitions.lisp', 'floor_lifecycle_patches.lisp')
#: The texts' joints as her movements (`term_rows.TERMS`, or `ABSENT`): her trunk one joint, lumbar and thoracic
#: summed.
MOVES = {'talocrural': ('ankle', 'dorsiflexion'), 'tibiofemoral': ('knee', 'flexion'),
         'coxofemoral': ('hip', 'flexion'), 'l4-l5-lumbar': ('trunk', 'flexion'),
         'c7-t12-thoracic': ('trunk', 'flexion'), 'glenohumeral': ('shoulder', 'flexion'),
         'cubital': ('elbow', 'flexion'), 'c1-c2-atlas': ('neck', 'flexion'), 'c1-c2-atlas-yaw': ('head', 'rotation'),
         'iliac-coronal': ('trunk', 'lateral-flexion'), 'calcaneus': ('foot', 'eversion'),
         'glenohumeral-ext-rot': ('shoulder', 'lateral-rotation'), 'glenohumeral-abduction': ('shoulder', 'abduction')}
#: The texts' landmarks by the segment of hers that carries each; her wrist's patch her fist (her hands bear on their
#: fists, `borne`).
LANDMARKS = {'calcaneus': 'foot', 'metatarsal-head': 'foot', 'femoral-diaphysis-dorsal': 'thigh',
             'femoral-diaphysis-ventral': 'thigh', 'ischial-tuberosity': 'pelvis', 'greater-trochanter': 'pelvis',
             'sacral': 'pelvis', 'iliac-anterior-spine': 'pelvis', 'l4-l5-lumbar': 'torso', 'c7-t12-thoracic': 'torso',
             'sternal-pectoral': 'torso', 'occipital': 'head', 'olecranon': 'forearm', 'ulna-distal': 'hand',
             'carpal-heel': 'hand', 'palmar-carpal': 'hand', 'tibial-tuberosity': 'shank', 'tarsal-dorsum': 'foot'}
#: Samples round a turn; a part within TOUCH m of the floor touches it; the text's cranium clear of the floor in the
#: roll (`drive-helical-axial-roll-wave`'s comment), m.
TURN, TOUCH, CRANIUM_M = 7200, 1e-3, 0.035


def _laid(origin):
    """{joint: deg} of her figure posed as a text's origin ({name: rad}): her hips, knees, ankles, shoulders and
    elbows as told, both sides, else straight; a movement she lacks left out."""
    from machine import figure
    from machine.term_rows import ABSENT, TERMS
    deg = figure.stand()
    deg.update({k: 0.0 for k in deg if k.endswith(('_hip', '_knee', '_ankle', '_shoulder', '_elbow'))})
    for name, rad in origin.items():
        if MOVES[name] in ABSENT:
            continue
        kind, sign = TERMS[MOVES[name]][:2]
        d = sign * math.degrees(rad)
        if kind == 'spine':
            deg['spine'] += d
        else:
            deg.update({j: d for j in ([kind] if kind in deg else ['left_' + kind, 'right_' + kind])})
    return deg


def _part(segment):
    return segment.split('_', 1)[1] if segment.startswith(('left_', 'right_')) else segment


def _skin(deg, turn=None):
    """(points (n, 3), radii (n,), their segments, her centre) of her figure posed `deg`, the pelvis at the origin
    turned `turn`: each capsule of `figure.BODY` its two ends, each sole's and toes' box its corners at 0; her
    fingers left out, her fists what her hands bear on."""
    import itertools
    import numpy as np
    from machine import figure
    turn = turn or figure.IDENT
    segments = {s[0] for s in figure.SEGMENTS if not s[0].endswith('fingers')}
    points, radii, of = [], [], []
    for name, (at, here) in figure.frames(deg, (0.0, 0.0, 0.0), turn).items():
        caps = [(r, e) for p, r, a, b in figure.BODY if name in segments and p == _part(name) for e in (a, b)]
        boxes = [(0.0, tuple(c[i] + s[i] * size[i] for i in range(3)))
                 for p, _shape, size, c in figure.CONTACTS if name in segments and p == _part(name)
                 for s in itertools.product((-1.0, 1.0), repeat=3)]
        for r, e in caps + boxes:
            points.append(np.add(at, figure.apply(here, e)))
            radii.append(r)
            of.append(name)
    return np.array(points), np.array(radii), of, np.array(figure.com(deg, (0.0, 0.0, 0.0), turn))


def _parts(manifolds):
    """Her segments' names, side-less, that carry the landmarks of `manifolds` (their frames)."""
    from machine import kernel
    got = kernel.read()
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    frames = [f.lstrip(':') for m in manifolds for f in patches[m.lstrip(':')]['frames']]
    return sorted({LANDMARKS[f[:-2] if f.endswith(('-l', '-r')) else f] for f in frames})


def _pitched(deg):
    """(pitches round a turn, each point's skin over the floor at each (`groundwork.clear`), her centre over it, her
    margin at a pitch's index - her centre's over what touches along the floor, m, negative past it -, what touches
    at an index, the points' parts) of her figure posed `deg`."""
    import numpy as np
    from machine import groundwork
    points, radii, of, centre = _skin(deg)
    phi = np.linspace(-np.pi, np.pi, TURN, endpoint=False)
    up = groundwork.pitched(phi)
    gap = groundwork.clear(points, radii, up)

    def margin(k):
        ahead = np.array([0.0, -np.sin(phi[k]), np.cos(phi[k])])
        along, c = points[gap[k] < TOUCH] @ ahead, centre @ ahead
        return min(c - along.min(), along.max() - c)

    def touching(k):
        return sorted({_part(of[i]) for i in np.flatnonzero(gap[k] < TOUCH)})
    return phi, gap, groundwork.rest(points, radii, centre, up), margin, touching, np.array([_part(s) for s in of])


def _bearing(deg, parts):
    """Her figure posed `deg` at its rest, pitched about her x axis, where `parts` bear best: (pitch rad, the worst of
    them and its float m, her centre's margin m, what touches, every rest's margin)."""
    import numpy as np
    from machine import groundwork
    phi, gap, h, margin, touching, names = _pitched(deg)
    rests = groundwork.settled(h)
    each = np.array([gap[rests][:, names == p].min(axis=1) for p in parts])
    k = int(np.argmin(each.max(axis=0)))
    return (phi[rests[k]], parts[int(np.argmax(each[:, k]))], float(each[:, k].max()), margin(rests[k]),
            touching(rests[k]), [margin(r) for r in rests])


def _placed(deg, part):
    """Her figure posed `deg`, pitched nearest upright with `part` on the floor: (her centre's margin m, what
    touches)."""
    import numpy as np
    phi, gap, _h, margin, touching, names = _pitched(deg)
    k = int(np.argmin(np.where(gap[:, names == part].min(axis=1) < TOUCH, np.abs(phi), np.inf)))
    return margin(k), touching(k)


def test_the_texts_read_whole(report):
    """The three texts read with the kernel (`kernel.read`): six postures, each its origin and three columns; three
    patches more, every manifold the texts name one of the registry's or theirs; the core, the transforms, the
    specializers and the patches stages; the schedule four oneshots 1.5 s apart, the last igniting her walk's clock;
    the roll's node as the default roll lays it, rolling her right."""
    from machine import dsp, kernel
    got = kernel.read()
    atlas = got['groundwork-1jet-posture-atlas']['postures']
    report.check('six postures, each its origin and three columns',
                 sorted(atlas) == ['floor-ischial-carpal-tripod', 'floor-ischial-unbraced', 'floor-prone-flat',
                                   'floor-supine-flat', 'kneeling-quadruped-braced', 'kneeling-tall-biped']
                 and all(len(p['modal-basis-columns']) == 3 and 'origin-q0' in p for p in atlas.values()))
    folder = os.path.join(os.path.dirname(dsp.__file__), 'ortho_kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    named = {m for t in texts for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx))(?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch: the registry\'s 8 and the texts\' 3' % len(named),
                 len(got['groundwork-manifold-extensions']) == 3 and named <= set(patches),
                 str(sorted(named - set(patches))))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages: the core, the transforms, the specializers, the patches' % (len(stages) - 1),
                 all(isinstance(got[n], dsp.Stage) for n in stages if n != 'compile-complete-groundwork-soa'))
    plan = dsp.schedule(got['compile-complete-groundwork-soa'])
    phases = [seg[1]['_trace'][-1][0] for seg in plan if seg[0] == 'oneshot']
    clock = [x for name, x in plan[-1][1]['_trace'] if name == 'ignite-cyclic-clock']
    report.check('the schedule: descent, sit and supine, the rolls, the rise, 1.5 s between; her clock ignited last',
                 [seg[0] for seg in plan] == ['oneshot', 'dwell'] * 3 + ['oneshot']
                 and [seg[1] for seg in plan if seg[0] == 'dwell'] == [1.5] * 3
                 and phases == ['patch-descent-to-knees-and-all-fours', 'patch-transition-to-buttocks-and-supine',
                                'patch-roll-supine-to-prone-and-back', 'patch-rise-from-prone-to-standing']
                 and len(clock) == 1 and isinstance(clock[0], dsp.Morph) and clock[0].bars == 1.0, str(phases))
    roll = kernel.node('xform-continuous-ground-roll', 'body-roll')
    law = kernel.node('xform-continuous-ground-roll', 'substrate-roll')
    report.check('the roll: 3.14159 rad pelvis first at 1.45 rad/s, the right arm overhead, the left limbs across; '
                 'hard over 380 N/mm, soft under 65',
                 (roll['total-angle-rad'], roll['axial-wave-leader'], roll['nominal-omega-radps'])
                 == (3.14159, ':pelvic-led', 1.45)
                 and (roll['ipsilateral-limb-clearance']['side'], roll['contralateral-momentum-driver']['side'])
                 == (':right', ':left') and len(roll['circumferential-manifold-ring']) == 5
                 and (law['hard-substrate-compensation']['active-above-npmm'],
                      law['soft-substrate-compensation']['active-below-npmm']) == (380.0, 65.0), str(roll))


def test_the_ring_and_the_girdles(report):
    """The roll's ring (`groundwork.ring`) whole on one patch at each quarter, its shares summing to 1 round a turn;
    the girdles (`groundwork.girdles`) of the schedule's two rolls - 180 deg pelvis first, 360 deg thorax first - turn
    her waist by the text's lag at most, within its stops, the leader ahead throughout, both done at
    (angle + lag) / omega."""
    import numpy as np
    from machine import dsp, drives, groundwork, kernel
    theta = np.linspace(0.0, 4.0 * np.pi, 2 * TURN)
    w = groundwork.ring(theta)
    report.check('the ring: one patch whole a quarter, shares summing to 1 within %.0e' % np.abs(w.sum(axis=1) - 1).max(),
                 np.array_equal(groundwork.ring(groundwork.QUARTERS[:4]), np.eye(5)[:4])
                 and np.abs(w.sum(axis=1) - 1.0).max() < 1e-12)
    got = kernel.read()
    rolls = [x for name, x in dsp.schedule(got['compile-complete-groundwork-soa'])[4][1]['_trace']
             if name == 'xform-continuous-ground-roll']
    node = kernel.node('xform-continuous-ground-roll', 'body-roll')
    omega, lag = node['nominal-omega-radps'], node['inter-girdle-phase-lag-rad']
    low, high = drives.TRAVEL['waist']
    for roll in rolls:
        total, lead = roll['angle-rad'], (':pelvic-led', ':thoracic-led').index(roll['leader'])
        done = (total + lag) / omega
        t = np.linspace(0.0, done + 1.0, 4001)
        girdles = groundwork.girdles(t, omega, total, lag, lead)
        twist = girdles[lead] - girdles[1 - lead]
        report.check('%.0f deg, %s: done in %.2f s, her waist turned %.1f deg at most (%g..%g)' % (
            math.degrees(total), roll['leader'][1:], done, math.degrees(twist.max()), low, high),
            abs(twist.max() - lag) < 1e-12 and twist.min() >= 0.0 and math.degrees(lag) <= high
            and np.all(girdles[:, t >= done] == total) and np.all(girdles[:, t < done - 1e-9].min(axis=0) < total))


def _rolled(deg, side):
    """(her centre's height round a turn on her back rolled onto `side`, each point's skin over the floor, her
    segments, the samples) of her figure posed `deg`."""
    import numpy as np
    from machine import groundwork
    points, radii, of, centre = _skin(deg)
    theta = np.linspace(0.0, 2.0 * np.pi, TURN, endpoint=False)
    up = groundwork.rolled(theta, side)
    return groundwork.rest(points, radii, centre, up), groundwork.clear(points, radii, up), of, theta


def _roll_postures():
    """Her supine origin; the roll's right arm overhead (its slot); the slot and the left limbs across (its
    cantilever, her shoulder's only axis its flexion)."""
    from machine import kernel
    from machine.term_rows import TERMS
    origin = kernel.read()['groundwork-1jet-posture-atlas']['postures']['floor-supine-flat']['origin-q0']
    roll = kernel.node('xform-continuous-ground-roll', 'body-roll')
    slot, lever = roll['ipsilateral-limb-clearance'], roll['contralateral-momentum-driver']
    sides = _laid(origin)
    slotted = dict(sides, right_shoulder=math.degrees(slot['glenohumeral-flexion-rad']),
                   right_elbow=math.degrees(slot['cubital-flexion-rad']))
    across = dict(slotted, left_hip=TERMS[('hip', 'flexion')][1] * math.degrees(lever['coxofemoral-flexion-rad']),
                  left_hip_roll=TERMS[('hip', 'adduction')][1] * math.degrees(lever['coxofemoral-adduction-rad']),
                  left_shoulder=math.degrees(lever['glenohumeral-cross-reach-rad']))
    return sides, slotted, across


def test_her_roll_on_a_hard_floor(report):
    """`groundwork.rest` is her centre's height over her lowest skin, her figure turned any way (16 random turns,
    1e-12 m). Rolled onto her right on a hard floor: her arms at her sides, her right fist is what she lies on, side
    down; the roll's overhead slot lowers her climb onto her side, the cantilever lowers it again and moves her centre
    toward the roll; her head clears the floor by the text's 35 mm round the turn in the slot."""
    import numpy as np
    from machine import figure, groundwork
    rng = np.random.default_rng(0)
    deg = _roll_postures()[1]
    points, radii, _of, centre = _skin(deg)
    worst = 0.0
    for q in rng.normal(size=(16, 4)):
        turn = figure.quat(*(q / np.linalg.norm(q)))
        world, _r, _o, at = _skin(deg, turn)
        h = groundwork.rest(points, radii, centre, np.array(turn)[1][None])[0]
        worst = max(worst, abs(h - (at[1] - (world[:, 1] - radii).min())))
    report.check('her centre over her lowest skin, any turn: within %.0e m' % worst, worst < 1e-12)
    weight = figure.mass() * 9.81
    side = TURN // 4
    climbs, held = [], []
    for name, posed in zip(('her arms at her sides', 'the slot', 'the slot and the cantilever'), _roll_postures()):
        h, gap, of, theta = _rolled(posed, -1.0)
        top = int(np.argmax(h[:side + 1]))
        torque = weight * np.gradient(h, theta)[:top + 1].max(initial=0.0)
        lowest = sorted({of[i] for i in np.flatnonzero(gap[side] < TOUCH)})
        head = float(gap[:, [s == 'head' for s in of]].min())
        centre = _skin(posed)[3]
        climbs.append(weight * (h[top] - h[0]))
        held.append((lowest, head, centre[0]))
        report.check('%s: onto her side %.1f J, %.1f N m at most; on it she lies on %s; her head %.0f mm clear, her '
                     'centre %+.3f m across' % (name, climbs[-1], torque, ', '.join(lowest), 1e3 * head, centre[0]),
                     True)
    report.check('side down her arms at her sides lie under her: her right fist', held[0][0] == ['right_hand'])
    report.check('the slot climbs less (%.1f J for %.1f), the cantilever less again (%.1f), her centre %.0f mm toward '
                 'the roll' % (climbs[1], climbs[0], climbs[2], -1e3 * held[2][2]),
                 climbs[2] < climbs[1] < climbs[0] and held[2][2] < held[1][2])
    report.check("in the slot her head clears the floor by %.0f mm, the text's %.0f" % (1e3 * held[1][1], 1e3 * CRANIUM_M),
                 held[1][1] >= CRANIUM_M)


def test_out_of_a_mattress_trough(report):
    """The roll law's branches (`groundwork.substrate`) never both on, the hard over 380 N/mm, the soft under 65; her
    sinkage her weight over the stiffness; the text's law (`groundwork.escape`) at the start of a roll half her
    sinkage times her weight. Its 110 N heel push about her right buttock's edge, lying on her back, against that
    trough: the stiffness at which they are equal, and how high the cantilever holds that heel."""
    import numpy as np
    from machine import figure, groundwork, kernel
    law = kernel.node('xform-continuous-ground-roll', 'substrate-roll')
    above = law['hard-substrate-compensation']['active-above-npmm'] * 1e3
    below = law['soft-substrate-compensation']['active-below-npmm'] * 1e3
    push = law['soft-substrate-compensation']['sinkage-trough-escape-boost']['contralateral-heel-pushoff-n']
    k = np.geomspace(1e3, 1e6, 601)
    hard, soft = groundwork.substrate(k, above, below)
    report.check('hard over %.0f N/mm, soft under %.0f, never both' % (above / 1e3, below / 1e3),
                 not np.any(hard * soft) and np.array_equal(hard, (k > above).astype(float))
                 and np.array_equal(soft, (k < below).astype(float)))
    weight = figure.mass() * 9.81
    sink = groundwork.sinkage(weight, np.array([below, above]))
    sides, _slotted, across = _roll_postures()
    trough = groundwork.escape(weight, _rolled(sides, -1.0)[0][0], sink[0], 0.0)
    report.check('her %.1f N sinks %.2f mm at %.0f N/mm, %.2f at %.0f; the trough holds her with %.2f N m' % (
        weight, 1e3 * sink[0], below / 1e3, 1e3 * sink[1], above / 1e3, trough),
        abs(sink[0] * below - weight) < 1e-9 and abs(trough - 0.5 * weight * sink[0]) < 1e-12)
    points, radii, of, _centre = _skin(sides)
    gap = groundwork.clear(points, radii, groundwork.rolled(0.0, -1.0))[0]
    edge = points[gap < TOUCH, 0].min()
    lever = figure.frames(sides, (0.0, 0.0, 0.0), figure.IDENT)['left_foot'][0][0] - edge
    equal = weight * weight / (2.0 * push * lever)
    lifted = groundwork.clear(*_skin(across)[:2], groundwork.rolled(0.0, -1.0))[0]
    heel = float(lifted[[s == 'left_foot' for s in _skin(across)[2]]].min())
    report.check('the heel %.0f N at %.3f m from her right buttock: %.1f N m, %.0f times the trough at %.0f N/mm; '
                 'equal at %.2f N/mm. The cantilever holds that heel %.0f mm up' % (
                     push, lever, push * lever, push * lever / trough, below / 1e3, equal / 1e3, 1e3 * heel),
                 equal < below)


def test_the_postures_on_her_figure(report):
    """The six origins and the roll's and the bridge's angles as her movements (`term_rows`): each joint one of hers
    or one she lacks, each angle against her stops. Each origin laid rigid on a hard floor (`groundwork.rest`,
    pitched): every rest her centre over what touches; the rest where its coupled patch bears best, the float of its
    worst part and her centre's margin. Her supine rests on its dorsal patch, her pelvis and head bearing; the tall
    kneel rests on her shins at no point of its origin, and at the least of its axial column that sets her centre
    over them."""
    from machine import drives, kernel
    from machine.term_rows import ABSENT, TERMS
    got = kernel.read()
    atlas = got['groundwork-1jet-posture-atlas']['postures']
    roll = kernel.node('xform-continuous-ground-roll', 'body-roll')
    bridge = kernel.node('route-90-90-hip-inversion-bridge', 'leg-inversion')['hip-90-90-rotational-pathway']
    told = [(name, n, rad) for name, p in atlas.items() for n, rad in p['origin-q0'].items()]
    told += [('roll', 'glenohumeral', roll['ipsilateral-limb-clearance']['glenohumeral-flexion-rad'])]
    unknown = [n for _p, n, _r in told if n not in MOVES]
    lacks = sorted({'%s %s %.2f' % (p, n, r) for p, n, r in told if n in MOVES and MOVES[n] in ABSENT})
    stops = {m: (-math.inf if t[2] is None else t[2], math.inf if t[3] is None else t[3]) for m, t in TERMS.items()}
    out = sorted({'%s %s %.0f deg' % (p, n, math.degrees(r)) for p, n, r in told if n in MOVES and MOVES[n] in TERMS
                  and not stops[MOVES[n]][0] <= math.degrees(r) <= stops[MOVES[n]][1]})
    turn = math.degrees(bridge['lead-hip-internal-rot-rad'])
    report.check('every joint hers or one she lacks: %s; past her stops: %s; the 90/90 bridge turns a hip %.0f deg, '
                 'hers %g' % ('; '.join(lacks), '; '.join(out) or 'none', turn, drives.TRAVEL['hip_yaw'][1]),
                 not unknown, str(unknown))
    rests, laid = [], {}
    for name, p in atlas.items():
        coupled = p['coupled-manifold'] if isinstance(p['coupled-manifold'], tuple) else (p['coupled-manifold'],)
        laid[name] = _bearing(_laid(p['origin-q0']), _parts(coupled))
        phi, part, worst, margin, touching, every = laid[name]
        rests += every
        report.check('%s: best at %+.0f deg on %s, its %s %.0f mm up, her centre %.0f mm inside' % (
            name, math.degrees(phi), ', '.join(touching), part, 1e3 * worst, 1e3 * margin), True)
    report.check('every rest her centre over what touches (%.1f mm at least)' % (1e3 * min(rests)), min(rests) >= -TOUCH)
    supine = laid['floor-supine-flat']
    report.check('her supine on its dorsal patch: her pelvis and head bear, her %s %.1f mm up' % (supine[1], 1e3 * supine[2]),
                 {'pelvis', 'head'} <= set(supine[4]) and supine[3] > 0.0)
    kneel = atlas['kneeling-tall-biped']
    column = kneel['modal-basis-columns']['mode-axial-compression']

    steps = []
    for a in [0.05 * n for n in range(21)]:
        origin = dict(kneel['origin-q0'], **{j: kneel['origin-q0'][j] + a * v for j, v in column.items()})
        phi, _shank, worst, margin, _touching, _every = _bearing(_laid(origin), ['shank'])
        steps.append((a, origin['tibiofemoral'], phi, worst, margin))
    found = next((s for s in steps if s[3] < TOUCH), None)
    past, on = _placed(_laid(kneel['origin-q0']), 'shank')
    report.check('the tall kneel: on her %s her centre %.0f mm past them, no rest; %.2f of its axial column - knee '
                 '%.0f deg, pitched %+.0f deg - sets it %.0f mm inside' % (
                     ' and '.join(on), -1e3 * past, found[0], math.degrees(found[1]), math.degrees(found[2]),
                     1e3 * found[4]) if found else 'the tall kneel: on her shins nowhere along its axial column',
                 steps[0][3] > TOUCH and past < 0.0 and found is not None)


ROSTER = [test_the_texts_read_whole, test_the_ring_and_the_girdles, test_her_roll_on_a_hard_floor,
          test_out_of_a_mattress_trough, test_the_postures_on_her_figure]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
