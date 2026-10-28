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

#: Each gait of hers the kernel's clock plays: its patch, the segment of its schedule, her own text's phases it
#: follows and the one its strokes use.
CLOCKED = {name: ('compile-scoot-and-lowcrawl-suite-soa', at, ('kneel', 'squat', 'tip', 'catch', 'sit'), 'sit')
           for name, at in (('scoot_floor', 0), ('scoot_back', 1))}
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
#: A limb under this authority is in High-Z (`partition-fault-tolerant-dynamics`' floor).
FREEWHEEL = kernel.node('partition-fault-tolerant-dynamics', 'fault-partition')['freewheel-threshold']
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
    """(the segment's state, its posture's origin and modal columns) of gait `name` (CLOCKED)."""
    patch, at = CLOCKED[name][:2]
    g = dsp.schedule(kernel.read()[patch])[at][2]
    health = g.get('limb-health') or {}
    if health.get('authority', 1.0) < FREEWHEEL:
        raise MachineError('refused: %s in %s; her step has no joint in High-Z' % (health['limb'], health['fault-mode']))
    sled = g['stick-slip']['sleds']
    postures = dsp.read(dsp.KERNEL)[g['posture-jet']['posture-atlas']]['postures']
    posture = next(p for p in postures.values() if set(sled) <= set(p['coupled-manifold']))
    return g, posture['origin-q0'], posture['modal-basis-columns']


def turns(name):
    """(her turns of the clock, s a turn, s its stroke, m a stroke) of gait `name`'s segment (CLOCKED)."""
    patch, at = CLOCKED[name][:2]
    b0, b1, g = dsp.schedule(kernel.read()[patch])[at]
    clock = g['axial-torus-clock']
    turn = 60.0 / clock['bpm']
    return (int(round((b1 - b0) * BAR)), turn, turn * clock['limb-duty'] / (clock['limb-duty'] + clock['axial-duty']),
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


def chord(knee):
    """m her hip joints sit behind her ankle, sat on the floor (SAT), her knee bent `knee`."""
    up = SAT - figure.ANKLE_H
    return math.sqrt(max(figure.THIGH ** 2 + figure.SHANK ** 2 + 2.0 * figure.THIGH * figure.SHANK * math.cos(knee) - up * up, 0.0))


def rowed(q0, columns, m, metres):
    """The modal units along a posture's sagittal column that row her hips `metres` along her way over her soles."""
    at = lambda s: -chord(posed(q0, columns, (m[0], s, m[2]))['tibiofemoral'])
    return metres / ((at(m[1] + 1e-3) - at(m[1] - 1e-3)) / 2e-3)


def _springs(targets, told, released, modes):
    """{mode: (stiffness, damping)} of `targets` at her rest's, a mode whose limb is `told` to bear nothing at the
    `released` share of it (`acts.sprung`)."""
    return {m: acts.sprung(m, 1.0 if told.get(LIMB[m], 0.0) > 0.0 else released, 1.0, modes) for m in targets}


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
    """(defbehavior NAME (extends NAME) ..) of gait `name` as the kernel's clock plays it (CLOCKED): her own phases,
    then each turn of its segment a stroke and her sled's, then her rest on her sled."""
    patch, at, own, used = CLOCKED[name]
    g, q0, columns = _segment(name)
    sled = g['stick-slip']
    n, turn, stroke, metres = turns(name)
    modes, body = maneuver._manifest()
    m = g['modal-pose']['modal-vector-m3']
    way = math.copysign(1.0, g['axial-torus-clock']['desired-se2-twist'][0])
    half = 0.5 * way * rowed(q0, columns, m, metres)
    ahead, behind = (hers(posed(q0, columns, (m[0], m[1] + s, m[2])), modes, body) for s in (half, -half))
    gain = sled['pelvic-unweighting-gain']
    anchored = _told(sled['anchors'], gain)
    for k, v in _told(sled['sleds'], 1.0 - gain).items():
        anchored[k] = anchored.get(k, 0.0) + v
    borne = _told(sled['sleds'], 1.0)
    cone = g['centroidal-balance']['nullspace-projector']
    released = 10.0 ** (cone['passive-cone-atten-db'] / 20.0) if cone['auto-release-unloaded-limbs'] else 1.0
    rows, last = ['  (phase %s)' % p for p in own], used
    for t in range(1, n + 1):
        rows.append(_phase('stroke_%d' % t, last, 1e3 * stroke, ahead, _springs(ahead, anchored, released, modes),
                           way * metres / stroke, anchored))
        rows.append(_phase('sled_%d' % t, 'stroke_%d' % t, 1e3 * (turn - stroke), behind,
                           _springs(behind, borne, released, modes), 0.0, borne))
        last = 'sled_%d' % t
    rows.append('  (phase floor_rest (use %s) (duration 60s) (blend_duration 1000ms))' % last)
    return '(defbehavior %s (extends %s)\n%s)\n' % (name, name, '\n'.join(rows))


def texts():
    """Every gait of hers the kernel's clock plays (CLOCKED) and her body can, as one text for her gym."""
    out = []
    for name in CLOCKED:
        try:
            out.append(text(name))
        except MachineError:
            continue
    return '\n'.join(out)
