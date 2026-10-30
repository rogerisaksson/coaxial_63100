"""The kernel's clocked sled gaits on her loop: a segment's posture, clock and ratchet as her phases.

    text = clocked.text('scoot_floor')      # (defbehavior scoot_floor (extends scoot_floor) ..): her descent, its strokes
    texts = clocked.texts()                 # every gait of hers the kernel's clock plays (CLOCKED), for her gym

A segment (`dsp.schedule`) says its posture - `blend-manifold-posture-jet`'s atlas, the posture there its sled bears,
`reconstruct-modal-pose`'s vector over that posture's modal columns -, its clock (`drive-5torus-ehresmann-clock`: its bpm,
its limbs' and its sled's duties, its twist) and its ratchet (`modulate-stick-slip-sled-ratchet`: anchors, sleds, the
vault's share, the stroke). A turn of the clock is her anchors' stance - her weight on them the vault's share, her
posture rowed its stroke along its sagittal column the way her twist goes, her sled sliding as far - then her sled's,
all of her on it, her posture rowed back as her anchors are carried. A posture is hers as `acts` reads a joint's
target: her knee her height (`seated.axial`), her trunk her pitch, her shoulders and elbows her arms' reach and fold. A bar is BAR turns. What her body has not - a
bearing patch she has no manifold for, a joint in High-Z - is refused by name.
"""
import math
import os

from machine.texts import acts, dsp, kernel, maneuver, ortho
from machine.body import figure, mechatronics
from machine.moves import seated
from machine.errors import MachineError

#: Each gait of hers the kernel's clock plays: its patch, the segment of its schedule, her own text it extends,
#: that text's phases it follows and the one its strokes use.
CLOCKED = {name: ('compile-scoot-and-lowcrawl-suite-soa', at, 'scoot_floor', ('kneel', 'squat', 'tip', 'catch', 'sit'),
                  'sit') for name, at in (('scoot_floor', 0), ('scoot_back', 1))}
#: A bar's turns of the clock.
BAR = 4
#: Her told manifold for each of the kernel's patches, by the body that bears.
TOLD = {':plantar-bilateral': 'plantar', ':metatarsal-equinus-bilateral': 'plantar',
        ':ulnar-carpal-shove-bilateral': 'palmar', ':palmar-carpal-bilateral': 'palmar',
        ':ischial-bilateral': 'ischial', ':posterior-femoral-bilateral': 'ischial',
        ':ventral-prone-continuum': 'dorsal', ':dorsal-supine-continuum': 'dorsal'}
#: A posture's modal columns in `reconstruct-modal-pose`'s order.
COLUMNS = ('mode-axial-compression', 'mode-sagittal-shift', 'mode-pitch-attitude')
#: The limb each of her posture's modes moves, by the manifold it bears on: a limb told to bear nothing is released
#: (`centroidal-balance`'s `auto-release-unloaded-limbs`), its modes' stiffness at `passive-cone-atten-db`. Held at
#: her rest's, her let-go soles bore 371 N of her 305 as her knees opened and threw her off her seat.
LIMB = {'mode_axial_compression': 'plantar', 'mode_arm_reach': 'palmar', 'mode_arm_fold': 'palmar',
        'mode_pitch_attitude': 'ischial'}
#: The modes a seat releases (`xform-balance-biped-to-seated`: seated, her knees and ankles released): her height
#: is what bears her pelvis's. Sprung while her soles anchored, her legs pushed her pelvis off the floor, 0.16 ->
#: 0.29 m, and her trunk over backward to -51 deg in a stroke.
SEATED = ('mode_axial_compression',)
#: Her manifold each kind of the clock's limbs bears on. With no axial duty her sled never bears alone: as her
#: hands, the fore limbs, are carried her soles hold. Carried together, her seat bore her alone, her trunk went
#: over backward, -108 deg, at the backward scoot's second stroke.
LIMBED = {':hind': 'plantar', ':fore': 'palmar'}
#: Her hip joints' height sat on the floor, m (`seated.sat`).
SAT = seated.sat(0.0)


def _sat(name='scoot_floor', phase='sit', mode='mode_pitch_attitude'):
    """Her own text's `mode` target in `phase` of `name` (lisp/humanoid/motion.lisp)."""
    forms = ortho.read(ortho.source(os.path.join(mechatronics.HUMANOID, 'motion.lisp')))
    body = next(f for f in forms if isinstance(f, list) and f[:2] == ['defbehavior', name])
    said = next(c for p in body if isinstance(p, list) and p[:2] == ['phase', phase] for c in p if c and c[0] == 'latent_targets')
    return float(next(t[1] for t in said[1:] if t[0] == mode))


#: Her trunk's pitch sat on the floor, her text's sit's (units of her pitch mode): her tripod's - her fists on the floor
#: behind her, her arms straight (`seated.braced`), her shoulders over her seat. On the kernel's pitch, 24 deg forward,
#: her arms hung straight down under her shoulders, her hands 2-9 cm behind them, and she fell back off her seat.
SAT_PITCH = _sat()


def _told(patches, share):
    """{her manifold: share} of `share` over the kernel's `patches`, evenly."""
    out = {}
    for p in patches:
        if p not in TOLD:
            raise MachineError('refused: %s bears her nowhere she has a manifold for' % p)
        out[TOLD[p]] = out.get(TOLD[p], 0.0) + share / len(patches)
    return out


def _manifolds(told):
    return ' '.join('(%s %.3f)' % (m, told.get(m, 0.0)) for m in ('plantar', 'ischial', 'palmar', 'dorsal'))


def _segment(name):
    """(the segment's state, its posture's origin and modal columns, its anchors) of gait `name` (CLOCKED). Its
    limbs' health (`limb-health`) is the kernel's robot's: hers is her drives' own. Its High-Z limb's soles
    dropped, her hands bore its 0.72 alone and her trunk went over, -110 deg."""
    patch, at = CLOCKED[name][:2]
    g = dsp.schedule(kernel.read()[patch])[at][2]
    anchors = g['stick-slip']['anchors']
    sled = g['stick-slip']['sleds']
    postures = dsp.read(dsp.KERNEL)[g['posture-jet']['posture-atlas']]['postures']
    posture = next(p for p in postures.values() if set(sled) <= set(p['coupled-manifold']))
    return g, posture['origin-q0'], posture['modal-basis-columns'], anchors


def turns(name):
    """(her turns of the clock, s a turn, s its stroke - her anchors' stance, the clock's limb duty -, m a stroke)
    of gait `name`'s segment (CLOCKED)."""
    patch, at = CLOCKED[name][:2]
    b0, b1, g = dsp.schedule(kernel.read()[patch])[at]
    clock = g['axial-torus-clock']
    turn = 60.0 / clock['bpm']
    return (int(round((b1 - b0) * BAR)), turn, turn * clock['limb-duty'],
            g['stick-slip']['stroke-amplitude-m'])


def posed(q0, columns, m):
    """{joint: rad} of a posture's origin `q0` its modal vector `m` along its `columns` on."""
    return {j: v + sum(m[k] * columns[c].get(j, 0.0) for k, c in enumerate(COLUMNS)) for j, v in q0.items()}


def hers(q, modes, body):
    """{her mode: target} of a kernel posture `q` sat on the floor: her knee's bend her height (`seated.axial`), her
    trunk her sit's (SAT_PITCH), her fists on the floor behind her at it, her arms straight (`seated.braced`): her
    tripod of hands, shoulders and seat - on the kernel's shoulders and elbows her palms stayed off the floor, her
    stroke waited on them and she tipped off her seat. Her pelvis's pitch read off her legs and hip instead swung her
    trunk 60 deg back over a stroke and threw her. Her centre of
    mass is where they put it, no spring of its own: her sagittal reads her soles, and sat on the scoot's posture it is
    0.63 m behind them, past its 6 units."""
    trunk = SAT_PITCH * modes['mode_pitch_attitude'][0]
    return {'mode_axial_compression': seated.axial(q['tibiofemoral'], modes, body), 'mode_pitch_attitude': SAT_PITCH,
            'mode_arm_reach': seated.braced(trunk) / modes['mode_arm_reach'][0], 'mode_arm_fold': 0.0}


def chord(knee):
    """m her hip joints sit behind her ankle, sat on the floor (SAT), her knee bent `knee`."""
    up = SAT - figure.ANKLE_H
    return math.sqrt(max(figure.THIGH ** 2 + figure.SHANK ** 2 + 2.0 * figure.THIGH * figure.SHANK * math.cos(knee) - up * up, 0.0))


def rowed(q0, columns, m, metres):
    """The modal units along a posture's sagittal column that row her hips `metres` along her way over her soles."""
    at = lambda s: -chord(posed(q0, columns, (m[0], s, m[2]))['tibiofemoral'])
    return metres / ((at(m[1] + 1e-3) - at(m[1] - 1e-3)) / 2e-3)


def _springs(targets, told, released, modes):
    """{mode: (stiffness, damping)} of `targets` at her rest's, a mode whose limb is `told` to bear nothing, or a
    seat's (SEATED) her seat is told to bear, at the `released` share of it (`acts.sprung`)."""
    held = {m: (told.get(LIMB[m], 0.0) > 0.0) * (1.0 - (m in SEATED) * (told.get('ischial', 0.0) > 0.0)) for m in targets}
    return {m: acts.sprung(m, released + (1.0 - released) * held[m], 1.0, modes) for m in targets}


def _targets(targets):
    return ' '.join('(%s %.3f)' % kv for kv in sorted(targets.items()))


def _impedance(springs):
    return ' '.join('(stiffness %s %g) (damping %s %g)' % (mode, k, mode, d) for mode, (k, d) in sorted(springs.items()))


def _phase(name, use, ms, targets, springs, slide, told, wait=''):
    lt, imp = _targets(targets), _impedance(springs)
    return ('  (phase %s (use %s)%s\n    (duration %dms) (blend_duration %dms)\n    (latent_targets %s)\n'
            '    (impedance %s)\n    (task_constraints (maintain_zmp (polygon 0)) (slide %.3f) (contact_manifolds %s)))'
            % (name, use, wait and ' (wait %s)' % wait, ms, ms, lt, imp, slide, _manifolds(told)))


def text(name):
    """(defbehavior NAME (extends BASE) ..) of gait `name` as the kernel's clock plays it (CLOCKED): her own phases,
    sat with her legs carried out ahead to the stroke's start, bent - their weight her counterweight (stunts/crawl:
    knees 34-124 deg, the centre of mass 2-5 cm off the hips; folded at 165 deg, hers went 5.5 cm behind her pelvis and
    she tipped back as her hands lifted) -, then each turn her sled's - her anchors carried to the stroke's start, her legs while her hands bear their stroke's
    share, then her hands while her legs stay let go; her soles holding where it never bears alone (LIMBED), their
    modes at the stroke's end - and its stroke, then her rest on her sled. Sat, her knees are past the stroke's fold
    (146 deg): stroked first, they opened and rowed her back 4 cm; her hands carried with her legs before they were
    let go, her seat alone held her capture point 0.22 m outside it and she went over backward; her soles told to bear
    again as her hands went, her legs stood half stretched and threw her back."""
    patch, at, base, own, used = CLOCKED[name]
    g, q0, columns, anchors = _segment(name)
    sled = g['stick-slip']
    n, turn, stroke, metres = turns(name)
    modes, body = maneuver._manifest()
    m = g['modal-pose']['modal-vector-m3']
    way = math.copysign(1.0, g['axial-torus-clock']['desired-se2-twist'][0])
    half = 0.5 * way * rowed(q0, columns, m, metres)
    ahead, behind = (hers(posed(q0, columns, (m[0], m[1] + s, m[2])), modes, body) for s in (half, -half))
    gain = sled['pelvic-unweighting-gain']
    anchored = _told(anchors, gain)
    for k, v in _told(sled['sleds'], 1.0 - gain).items():
        anchored[k] = anchored.get(k, 0.0) + v
    alone = g['axial-torus-clock']['axial-duty'] > 0.0
    held = {} if alone else {m: v for m, v in anchored.items() if m == LIMBED[':hind']}
    borne = dict(held, **_told(sled['sleds'], 1.0 - sum(held.values())))
    held_back = {mode: (ahead if LIMB[mode] in held else behind)[mode] for mode in behind}
    cone = g['centroidal-balance']['nullspace-projector']
    released = 10.0 ** (cone['passive-cone-atten-db'] / 20.0) if cone['auto-release-unloaded-limbs'] else 1.0
    legs = {mode: target for mode, target in held_back.items() if LIMB[mode] == LIMBED[':hind']}
    sat = '  (phase %s (latent_targets %s) (impedance %s))' % (used, _targets(legs), _impedance(_springs(legs, {}, 1.0, modes)))
    rows, last = [sat if p == used else '  (phase %s)' % p for p in own], used
    for t in range(1, n + 1):
        for limb, other, phase in ((LIMBED[':hind'], LIMBED[':fore'], 'sled'), (LIMBED[':fore'], LIMBED[':hind'], 'reach')):
            carried = {mode: target for mode, target in held_back.items() if LIMB[mode] == limb}
            bearing = dict(borne, **{other: anchored.get(other, 0.0)})
            rows.append(_phase('%s_%d' % (phase, t), last, 1e3 * (turn - stroke), carried,
                               _springs(carried, bearing, 1.0, modes), 0.0, bearing))
            last = '%s_%d' % (phase, t)
        rows.append(_phase('stroke_%d' % t, last, 1e3 * stroke, ahead, _springs(ahead, anchored, released, modes),
                           way * metres / stroke, anchored, 'palmar'))
        last = 'stroke_%d' % t
    rows.append('  (phase floor_rest (use %s) (duration 60s) (blend_duration 1000ms))' % last)
    return '(defbehavior %s (extends %s)\n%s)\n' % (name, base, '\n'.join(rows))


def texts():
    """Every gait of hers the kernel's clock plays (CLOCKED) and her body can, as one text for her gym."""
    out = []
    for name in CLOCKED:
        try:
            out.append(text(name))
        except MachineError:
            continue
    return '\n'.join(out)
