"""The kernel's scoot segment on the robot's loop: its descent to the floor and seat, legs out ahead.

    text = clocked.text('scoot_floor')      # (defbehavior scoot_floor (extends scoot_floor) ..): the robot's descent, its seat
    texts = clocked.texts()                 # every gait of its own the kernel's segment sits (CLOCKED), for its gym

A segment (`dsp.schedule`) says its posture - `blend-manifold-posture-jet`'s atlas, the posture there its sled
bears, `reconstruct-modal-pose`'s vector over that posture's modal columns -, its clock
(`drive-5torus-ehresmann-clock`: its bpm, its limbs' and its sled's duties, its twist) and its ratchet
(`modulate-stick-slip-sled-ratchet`: anchors, sleds, the vault's share, the stroke): the robot's seat holds
the robot's legs where a stroke would start, its anchors' modes there. The robot's strokes are its
twist's (`machine.moves.scoot`), not the clock's. A posture is the robot's own as `acts` reads a joint's
target: its knee its height (`seated.axial`), its trunk its pitch, its shoulders and elbows its arms' reach
and fold. A bar is BAR turns. What the robot's body has not - a bearing patch it has no manifold for, a joint
in High-Z - is refused by name.
"""
import math
import os

from machine.texts import acts, dsp, kernel, maneuver, ortho
from machine.body import figure, mechatronics
from machine.moves import seated
from machine.errors import MachineError

#: Each gait of the robot's own the kernel's segment sits: its patch, the segment of its schedule, the robot's
#: own text it extends, that text's phases it follows and the one it sits the robot in. The backward scoot's
#: segment is the robot's twist's way now, S (`scoot`).
CLOCKED = {'scoot_floor': ('compile-scoot-and-lowcrawl-suite-soa', 0, 'scoot_floor', ('kneel', 'squat', 'tip', 'catch',
                                                                                      'sit'), 'sit')}
#: The programs each binds to its phases ({phase: the loop's word}, laid as it is said: `g['programs']`): the
#: robot's scoot from its rest, sat on its fists till its twist (its law's own rest
#: crept on elbows bent 45 deg under its fists, their 25 N m, and laid it down 4.6 s on).
BOUND = {'scoot_floor': {'floor_rest': {'joints': 'scoot'}}}
#: A bar's turns of the clock.
BAR = 4
#: The robot's told manifold for each of the kernel's patches, by the body that bears.
TOLD = {':plantar-bilateral': 'plantar', ':metatarsal-equinus-bilateral': 'plantar',
        ':ulnar-carpal-shove-bilateral': 'palmar', ':palmar-carpal-bilateral': 'palmar',
        ':ischial-bilateral': 'ischial', ':posterior-femoral-bilateral': 'ischial',
        ':ventral-prone-continuum': 'dorsal', ':dorsal-supine-continuum': 'dorsal'}
#: A posture's modal columns in `reconstruct-modal-pose`'s order.
COLUMNS = ('mode-axial-compression', 'mode-sagittal-shift', 'mode-pitch-attitude')
#: The limb each of the robot's posture's modes moves, by the manifold it bears on: a limb told to bear nothing is
#: released (`centroidal-balance`'s `auto-release-unloaded-limbs`), its modes' stiffness at
#: `passive-cone-atten-db`. Held at the robot's rest's, its let-go soles bore 371 N of its 305 as its knees opened
#: and threw it off its seat.
LIMB = {'mode_axial_compression': 'plantar', 'mode_arm_reach': 'palmar', 'mode_arm_fold': 'palmar',
        'mode_pitch_attitude': 'ischial'}
#: The modes a seat releases (`xform-balance-biped-to-seated`: seated, the robot's knees and ankles released):
#: its height is what bears its pelvis's. Sprung while the robot's soles anchored, its legs pushed its pelvis off
#: the floor, 0.16 -> 0.29 m, and its trunk over backward to -51 deg in a stroke.
SEATED = ('mode_axial_compression',)
#: The robot's manifold each kind of the clock's limbs bears on. With no axial duty the robot's sled never
#: bears alone: as its hands, the fore limbs, are carried its soles hold. Carried together, the robot's seat
#: bore it alone, its trunk went over backward, -108 deg, at the backward scoot's second stroke.
LIMBED = {':hind': 'plantar', ':fore': 'palmar'}
#: The robot's hip joints' height sat on the floor, m (`seated.sat`).
SAT = seated.sat(0.0)


def _sat(name='scoot_floor', phase='sit', mode='mode_pitch_attitude'):
    """The robot's own text's `mode` target in `phase` of `name` (lisp/humanoid/motion.lisp)."""
    forms = ortho.read(ortho.source(os.path.join(mechatronics.HUMANOID, 'motion.lisp')))
    body = next(f for f in forms if isinstance(f, list) and f[:2] == ['defbehavior', name])
    said = next(c for p in body if isinstance(p, list) and p[:2] == ['phase', phase] for c in p if c and c[0] == 'latent_targets')
    return float(next(t[1] for t in said[1:] if t[0] == mode))


#: The robot's trunk's pitch sat on the floor, its text's sit's (units of its pitch mode): its tripod's - its
#: fists on the floor behind it, its arms straight (`seated.braced`), its shoulders over its seat. On the
#: kernel's pitch, 24 deg forward, the robot's arms hung straight down under its shoulders, its hands 2-9 cm
#: behind them, and it fell back off its seat.
SAT_PITCH = _sat()


def _told(patches, share):
    """{the robot's manifold: share} of `share` over the kernel's `patches`, evenly."""
    out = {}
    for p in patches:
        if p not in TOLD:
            raise MachineError('refused: %s bears the robot\'s nowhere it has a manifold for' % p)
        out[TOLD[p]] = out.get(TOLD[p], 0.0) + share / len(patches)
    return out


def _segment(name):
    """(the segment's state, its posture's origin and modal columns, its anchors) of gait `name` (CLOCKED). The
    gait's limbs' health (`limb-health`) is the kernel's robot's: its own is its drives' own. The
    gait's High-Z limb's soles dropped, the robot's hands bore its 0.72 alone and its trunk went over, -110
    deg."""
    patch, at = CLOCKED[name][:2]
    g = dsp.schedule(kernel.read()[patch])[at][2]
    anchors = g['stick-slip']['anchors']
    sled = g['stick-slip']['sleds']
    postures = dsp.read(dsp.KERNEL)[g['posture-jet']['posture-atlas']]['postures']
    posture = next(p for p in postures.values() if set(sled) <= set(p['coupled-manifold']))
    return g, posture['origin-q0'], posture['modal-basis-columns'], anchors


def turns(name):
    """(the robot's turns of the clock, s a turn, s its stroke - the robot's anchors' stance, the clock's limb
    duty -, m a stroke) of gait `name`'s segment (CLOCKED)."""
    patch, at = CLOCKED[name][:2]
    b0, b1, g = dsp.schedule(kernel.read()[patch])[at]
    clock = g['axial-torus-clock']
    turn = 60.0 / clock['bpm']
    return (int(round((b1 - b0) * BAR)), turn, turn * clock['limb-duty'],
            figure.SCALE * g['stick-slip']['stroke-amplitude-m'])


def phases(name):
    """The robot's limbs' phases on the clock of gait `name`'s segment (CLOCKED), of a turn: [LH LF RH RF
    axial]."""
    patch, at = CLOCKED[name][:2]
    return tuple(float(p) for p in dsp.schedule(kernel.read()[patch])[at][2]['axial-torus-clock']['nominal-phases'])


def posed(q0, columns, m):
    """{joint: rad} of a posture's origin `q0` its modal vector `m` along its `columns` on."""
    return {j: v + sum(m[k] * columns[c].get(j, 0.0) for k, c in enumerate(COLUMNS)) for j, v in q0.items()}


def robot_own(q, modes, body):
    """{the robot's mode: target} of a kernel posture `q` sat on the floor: its knee's bend its height
    (`seated.axial`), its trunk its sit's (SAT_PITCH), its fists on the floor behind the robot
    at it, the robot's arms straight (`seated.braced`): its tripod of hands, shoulders and seat - on the
    kernel's shoulders and elbows its palms stayed off the floor, its stroke waited on them and the
    robot tipped off its seat. The robot's pelvis's pitch read off its legs and hip instead swung its trunk 60
    deg back over a stroke and threw it. The robot's centre of mass is where they put it, no spring of its own: the
    robot's sagittal reads its soles, and sat on the scoot's posture it is 0.63 m behind them, past its 6
    units."""
    trunk = SAT_PITCH * modes['mode_pitch_attitude'][0]
    return {'mode_axial_compression': seated.axial(q['tibiofemoral'], modes, body), 'mode_pitch_attitude': SAT_PITCH,
            'mode_arm_reach': seated.braced(trunk) / modes['mode_arm_reach'][0], 'mode_arm_fold': 0.0}


def chord(knee):
    """m the robot's hip joints sit behind its ankle, sat on the floor (SAT), its knee bent `knee`."""
    up = SAT - figure.ANKLE_H
    return math.sqrt(max(figure.THIGH ** 2 + figure.SHANK ** 2 + 2.0 * figure.THIGH * figure.SHANK * math.cos(knee) - up * up, 0.0))


def rowed(q0, columns, m, metres):
    """The modal units along a posture's sagittal column that row the robot's hips `metres` along its way over its
    soles."""
    at = lambda s: -chord(posed(q0, columns, (m[0], s, m[2]))['tibiofemoral'])
    return metres / ((at(m[1] + 1e-3) - at(m[1] - 1e-3)) / 2e-3)


def _springs(targets, told, released, modes):
    """{mode: (stiffness, damping)} of `targets` at the robot's rest's, a mode whose limb is `told` to bear
    nothing, or a seat's (SEATED) its seat is told to bear, at the `released` share of it
    (`acts.sprung`)."""
    held = {m: (told.get(LIMB[m], 0.0) > 0.0) * (1.0 - (m in SEATED) * (told.get('ischial', 0.0) > 0.0)) for m in targets}
    return {m: acts.sprung(m, released + (1.0 - released) * held[m], 1.0, modes) for m in targets}


def _targets(targets):
    return ' '.join('(%s %.3f)' % kv for kv in sorted(targets.items()))


def _impedance(springs):
    return ' '.join('(stiffness %s %g) (damping %s %g)' % (mode, k, mode, d) for mode, (k, d) in sorted(springs.items()))


def text(name):
    """(defbehavior NAME (extends BASE) ..) of gait `name` as the kernel's segment sits the robot (CLOCKED):
    its own phases, sat with its legs carried out ahead to the stroke's start, bent - their weight its
    counterweight (stunts/crawl: knees 34-124 deg, the centre of mass 2-5 cm off the hips; folded at 165 deg,
    its own went 5.5 cm behind its pelvis and it tipped back as its hands lifted) -, then its rest on its
    seat. The robot's strokes are its twist's, not the clock's (`joints.scooted`, `machine.moves.scoot`): the
    segment's 32 clocked strokes waited on fists that never bore from the third, its trunk upright, and it sat
    still (backward 0.05 m in 9 s, the crawl take's 5.4)."""
    _patch, _at, base, own, used = CLOCKED[name]
    g, q0, columns, anchors = _segment(name)
    sled = g['stick-slip']
    _n, _turn, _stroke, metres = turns(name)
    modes, body = maneuver._manifest()
    m = g['modal-pose']['modal-vector-m3']
    way = math.copysign(1.0, g['axial-torus-clock']['desired-se2-twist'][0])
    half = 0.5 * way * rowed(q0, columns, m, metres)
    ahead, behind = (robot_own(posed(q0, columns, (m[0], m[1] + s, m[2])), modes, body) for s in (half, -half))
    gain = sled['pelvic-unweighting-gain']
    anchored = _told(anchors, gain)
    for k, v in _told(sled['sleds'], 1.0 - gain).items():
        anchored[k] = anchored.get(k, 0.0) + v
    alone = g['axial-torus-clock']['axial-duty'] > 0.0
    held = {} if alone else {m: v for m, v in anchored.items() if m == LIMBED[':hind']}
    held_back = {mode: (ahead if LIMB[mode] in held else behind)[mode] for mode in behind}
    legs = {mode: target for mode, target in held_back.items() if LIMB[mode] == LIMBED[':hind']}
    sat = '  (phase %s (latent_targets %s) (impedance %s))' % (used, _targets(legs), _impedance(_springs(legs, {}, 1.0, modes)))
    rows = [sat if p == used else '  (phase %s)' % p for p in own]
    rows.append('  (phase floor_rest (use %s) (duration 60s) (blend_duration 1000ms))' % used)
    return '(defbehavior %s (extends %s)\n%s)\n' % (name, base, '\n'.join(rows))


def texts():
    """Every gait of the robot's own the kernel's segment sits (CLOCKED) and its body can, as one text for its
    gym."""
    out = []
    for name in CLOCKED:
        try:
            out.append(text(name))
        except MachineError:
            continue
    return '\n'.join(out)
