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

from machine import acts, dsp, figure, kernel, maneuver, seated
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
#: Her manifold each kind of the clock's limbs bears on: her hands, the fore limbs, anchor her; her legs, stretched
#: out, rest. Her soles anchoring too, her legs pushed from her sit's knees, 146 deg, and kicked her over backward.
LIMBED = {':hind': 'plantar', ':fore': 'palmar'}
#: Her lower legs' share of her weight (`figure.SEGMENTS`: shank, foot, toes): what her soles bear stretching her
#: legs out along the floor, light enough to slide.
LEGS = sum(s[5] for s in figure.SEGMENTS if s[0].split('_')[-1] in ('shank', 'foot', 'toes'))
#: Her arms' modes' units, rad (`maneuver._manifest`); Newton's steps, its finite step and its bound a step, deg,
#: laying `palmed` - the bound her arm reach's unit: unbounded, the forward scoot's palm asked a 17.68-unit reach.
ARM_UNIT, FOLD_UNIT = (maneuver._manifest()[0][m][0] for m in ('mode_arm_reach', 'mode_arm_fold'))
NEWTON, STEP, REACH_DEG = 16, 1e-3, math.degrees(ARM_UNIT)
#: Her hip joints' height sat on the floor, m (`seated.sat`).
SAT = seated.sat(0.0)


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
    spine's bend over her pelvis on the seat her pitch, her shoulders and elbows her arms' reach and fold. Her pelvis's
    pitch read off her legs and hip instead swung her trunk 60 deg back over a stroke and threw her. Her centre of
    mass is where they put it, no spring of its own: her sagittal reads her soles, and sat on the scoot's posture it is
    0.63 m behind them, past its 6 units."""
    knee, trunk = q['tibiofemoral'], q['l4-l5-lumbar'] + q['c7-t12-thoracic']
    return {'mode_axial_compression': seated.axial(knee, modes, body),
            'mode_pitch_attitude': trunk / modes['mode_pitch_attitude'][0],
            'mode_arm_reach': q['glenohumeral'] / modes['mode_arm_reach'][0],
            'mode_arm_fold': q['cubital'] / modes['mode_arm_fold'][0]}


def palmed(z, lean):
    """{her arm's mode: target} that put her palm on the floor `z` m along her way from her pelvis, sat on it (SAT),
    her trunk leant `lean` rad: her shoulder and elbow by Newton's steps on `figure.palm`, over her stand's. Turned
    about her shoulder a stroke's metres over her arm's length, her palm pressed 7 cm into the floor and lifted 4 cm
    off it."""
    pelvis, rest = (0.0, SAT + figure.HIP_DROP, 0.0), figure.stand()
    at = lambda q: figure.palm(dict(rest, spine=math.degrees(lean), left_shoulder=q[0], left_elbow=q[1]), pelvis)
    q = [0.0, rest['left_elbow']]
    for _ in range(NEWTON):
        p = at(q)
        r = (p[1], p[2] - z)
        d = [[(at([q[0] + STEP * (k == 0), q[1] + STEP * (k == 1)])[i] - p[i]) / STEP for k in (0, 1)] for i in (1, 2)]
        det = d[0][0] * d[1][1] - d[0][1] * d[1][0]
        dq = [(d[1][1] * r[0] - d[0][1] * r[1]) / det, (d[0][0] * r[1] - d[1][0] * r[0]) / det]
        q = [q[0] - max(-REACH_DEG, min(REACH_DEG, dq[0])), max(0.0, q[1] - max(-REACH_DEG, min(REACH_DEG, dq[1])))]
    return {'mode_arm_reach': math.radians(q[0] - rest['left_shoulder']) / ARM_UNIT,
            'mode_arm_fold': math.radians(q[1] - rest['left_elbow']) / FOLD_UNIT}


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


def _phase(name, use, ms, targets, springs, slide, told):
    lt, imp = _targets(targets), _impedance(springs)
    return ('  (phase %s (use %s)\n    (duration %dms) (blend_duration %dms)\n    (latent_targets %s)\n'
            '    (impedance %s)\n    (task_constraints (maintain_zmp (polygon 0)) (slide %.3f) (contact_manifolds %s)))'
            % (name, use, ms, ms, lt, imp, slide, _manifolds(told)))


def text(name):
    """(defbehavior NAME (extends BASE) ..) of gait `name` as the kernel's clock plays it (CLOCKED): her own phases,
    a bar stretching her legs out under her upright trunk (`stretch`: her soles their LEGS, her sled and hands
    sliding her way as her knees open - a stance sole is held still: her hands held, her knees stopped at 124-135
    deg; her soles bearing their anchors' share, her legs threw her over backward), then each turn a stroke - her hands its
    anchors (LIMBED) bearing its vault, her palms on the floor from a stroke behind her to beside her (`palmed`), her
    legs let go - and her sled's, her palms sliding back, then her rest
    on her sled."""
    patch, at, base, own, used = CLOCKED[name]
    g, q0, columns, anchors = _segment(name)
    sled = g['stick-slip']
    n, turn, stroke, metres = turns(name)
    modes, body = maneuver._manifest()
    m = g['modal-pose']['modal-vector-m3']
    way = math.copysign(1.0, g['axial-torus-clock']['desired-se2-twist'][0])
    half = 0.5 * way * rowed(q0, columns, m, metres)
    ahead, behind = (hers(posed(q0, columns, (m[0], m[1] + s, m[2])), modes, body) for s in (half, -half))
    straight = seated.axial(math.radians(figure.STAND_KNEE), modes, body)
    behind['mode_axial_compression'] = straight
    lean = ahead['mode_pitch_attitude'] * modes['mode_pitch_attitude'][0]
    behind.update(palmed(way * metres, lean))
    ahead = dict(behind, mode_pitch_attitude=ahead['mode_pitch_attitude'], **palmed(0.0, lean))
    hands = tuple(p for p in anchors if TOLD[p] == LIMBED[':fore'])
    gain = sled['pelvic-unweighting-gain']
    anchored = dict(_told(sled['sleds'], 1.0 - gain), **_told(hands, gain))
    borne = _told(sled['sleds'], 1.0)
    stretched = dict(_told(sled['sleds'], 1.0 - LEGS - gain * len(hands) / len(anchors)),
                     **_told(hands, gain * len(hands) / len(anchors)), plantar=LEGS)
    cone = g['centroidal-balance']['nullspace-projector']
    released = 10.0 ** (cone['passive-cone-atten-db'] / 20.0) if cone['auto-release-unloaded-limbs'] else 1.0
    rows = ['  (phase %s)' % p for p in own]
    upright = dict(behind, mode_pitch_attitude=0.0)
    rows.append(_phase('stretch', used, 1e3 * BAR * turn, upright, _springs(upright, stretched, released, modes),
                       way * chord(math.radians(figure.STAND_KNEE)) / (BAR * turn), stretched))
    last = 'stretch'
    for t in range(1, n + 1):
        rows.append(_phase('stroke_%d' % t, last, 1e3 * stroke, ahead, _springs(ahead, anchored, released, modes),
                           way * metres / stroke, anchored))
        rows.append(_phase('sled_%d' % t, 'stroke_%d' % t, 1e3 * (turn - stroke), behind,
                           _springs(behind, borne, released, modes), 0.0, borne))
        last = 'sled_%d' % t
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
