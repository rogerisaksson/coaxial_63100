"""A gait of the ortho-DSP suite (`machine.texts.dsp`) voiced on her walk's row.

    row, told = voicing.voiced(g, dry, fr)       # {slot: value} off her walk's row; {'twist', 'grip'}
    wire, span = voicing.morphed(m, dry, fr)     # a Morph's rows (`patch.morph`)

A gait moves her walk's row (`walk`) by its changes off a dry gait, the suite's nominal (VOICES,
RULES): the dry plays her walk to the last slot. What has no voice of hers is named with what she
has instead (VOICELESS), what she cannot do refused by name (REFUSED), anything else as unknown.
"""
import math

from machine.texts import dsp, ortho, patch
from machine.law import modal, se2, supervisor
from machine.errors import MachineError

#: Her voices' slots.
Z, K, D, AMP, PHASE, N = ortho.Z, ortho.K, ortho.D, ortho.AMP, ortho.PHASE, ortho.N

#: A param's voice: (node, param, slot, rule); a param a rule moves its slot by (RULES).
VOICES = (
    ('oscillator', 'bpm', ortho.HZ, 'ratio'),
    ('com-dynamics', 'base-height-z-mm', Z + modal.AXIAL, 'drop'),
    ('com-dynamics', 'damping-ratio', D + modal.AXIAL, 'ratio'),
    ('tibiofemoral', 'rebound-q', D + modal.AXIAL, 'inverse'),
    ('tibiofemoral', 'compliance', K + modal.AXIAL, 'inverse'),
    ('tarsal-coronal', 'track-width-mm', Z + modal.WIDTH, 'width'),
    ('tarsal-sagittal', 'floor-clearance-z-mm', AMP + modal.LIFT, 'ratio'),
    ('iliac-coronal', 'lfo-amplitude-rad', AMP + modal.LIST, 'ratio'),
    ('c7-t12-thoracic', 'dc-offset-rad', Z + modal.PITCH, 'shift'),
    ('glenohumeral', 'gain-db', AMP + modal.ARMS, 'db'),
    ('glenohumeral', 'phase-shift-rad', PHASE + modal.ARMS, 'phase'),
    ('glenohumeral', 'phase-lag-rad', PHASE + modal.ARMS, 'lag'),
    ('com-dynamics', 'sidechain.modulation-depth-mm', ortho.DUCK.start, 'milli'),
    ('com-dynamics', 'sidechain.attack-ms', ortho.DUCK.start + 1, 'milli'),
    ('com-dynamics', 'sidechain.release-ms', ortho.DUCK.start + 2, 'milli'),
)
#: Rules (slot's value so far, value, the dry's, the mode's unit) -> the slot's value: 'ratio'
#: her walk's times the value over the dry's, 'inverse' times the dry's over it, 'shift' her
#: walk's plus the change in her unit, 'drop' a height lowered, mm, as her legs' shortening,
#: 'width' her width's share moved by the ratio (`stride.meets` floors it), 'db' a gain's
#: change, 'phase' and 'lag' a phase's, rad, 'milli' mm or ms as m or s.
RULES = {'ratio': lambda base, v, vd, unit: base * v / vd,
         'inverse': lambda base, v, vd, unit: base * vd / v,
         'shift': lambda base, v, vd, unit: base + (v - vd) / unit,
         'drop': lambda base, v, vd, unit: base + 1e-3 * (vd - v) / unit,
         'width': lambda base, v, vd, unit: (1.0 + base) * v / vd - 1.0,
         'db': lambda base, v, vd, unit: base * 10.0 ** ((v - vd) / 20.0),
         'phase': lambda base, v, vd, unit: base + (v - vd),
         'lag': lambda base, v, vd, unit: base - (v - vd),
         'milli': lambda base, v, vd, unit: 1e-3 * v}
#: A param her voices read off others: (node, param, its sources, how). Her lift is her swing's
#: height over the floor she steps to: stairs told raise it by their rise.
DERIVED = (
    ('tarsal-sagittal', 'floor-clearance-z-mm', ('swing-clearance-z-mm', 'step-elevation-z-mm'),
     lambda n: n['swing-clearance-z-mm'] - abs(n.get('step-elevation-z-mm', 0.0))),
)
#: Params whose unsaid value is a value: a lag unsaid is none.
UNSAID = {('glenohumeral', 'phase-lag-rad'): 0.0}
#: Params told her as commands, not voiced: her twist (`se2`) and her floor's grip (`grip`). A
#: stride is her capture point's: said as her drive's amplitude (x1.54) 713 -> 759 mm for 1 220
#: asked, and the catwalk take, a forward walk and a catwalk loop fell at 6.2-10.3 s.
TOLD = (('oscillator', 'direction-angle-rad'), ('oscillator', 'yaw-rate-radps'),
        ('tarsal-sagittal', 'stride-length-mm'), ('wrench-cone', 'mu-static'))
_SOLE, _GATE = "her shoe's sole (mjcf.SOLE_S); the rock stiffened for its transient felled her", 'her gate: 0.6 stands'
_CLOCK, _GUARD = 'her clock has no humanize: 1.1 ms a step', 'stride.guarded holds her swing off the standing sole'
_ARMS = 'her arms swing in pitch alone'
#: What has no voice of hers, and what she has instead (findings/sequencer.md); '*' a node's all.
VOICELESS = {
    ('oscillator', 'even-step-swing-offset'): 'her limbs half a turn apart: 0.0005 of a step measured',
    ('oscillator', 'phase-jitter-ms'): _CLOCK, ('oscillator', 'strip-jitter'): _CLOCK,
    ('oscillator', 'duty-factor-l'): _GATE, ('oscillator', 'duty-factor-r'): _GATE,
    ('plantar-boundary', '*'): "her shoe (mjcf.SOLE_S)", ('calcaneus', '*'): _SOLE,
    ('tarsal-sagittal', 'strike-angle-rad'): _SOLE,
    ('tarsal-sagittal', 'step-elevation-z-mm'): 'her floor, told (`floor.told_stairs`)',
    ('com-dynamics', 'topology'): "her pendulum's vault; a spring's (SLIP) refused",
    ('com-dynamics', 'vertical-oscillation-mm'): "her height's own: 22.7 mm a step",
    ('com-dynamics', 'sidechain.*'): 'her duck: a touchdown latches it, a linear attack, an exponential release',
    ('tibiofemoral', 'stance-yield-rad'): "her knee's give is her axial spring's under load (compliance)",
    ('tibiofemoral', 'work-mode'): "her drives' work", ('tibiofemoral', 'energy-injection-j'): "her drives' work",
    ('tarsal-coronal', 'swing-circumduction-mm'): _GUARD, ('tarsal-coronal', 'circumduction-window-phase'): _GUARD,
    ('tarsal-coronal', 'circumduction-shape'): _GUARD,
    ('tarsal-coronal', 'pan-spread'): "her landings' spread is her capture point's",
    ('iliac-coronal', 'waveform'): 'a sine',
    ('iliac-coronal', 'stereo-width'): "as the sway's 1.55 by a narrow track she widened to 230 mm and fell",
    ('l4-l5-lumbar', '*'): "her trunk has one pitch voice, the thoracic's",
    ('c7-t12-thoracic', 'q-factor'): "her posture's spring", ('c7-t12-thoracic', 'zmp-counter-bias-x-mm'): 'her law',
    ('c7-t12-thoracic', 'lpf-cutoff-hz'): "her trunk held on her posture's spring (1.59 Hz): softened by 2.2 of 8 Hz "
                                          'it bent 0.85 rad for 0.16', ('c7-t12-thoracic', 'lpf-slope-db'): 'her posture',
    ('glenohumeral', 'abduction-rad'): _ARMS, ('glenohumeral', 'elbow-flexion-rad'): _ARMS,
    ('c1-c2-atlas', '*'): 'no head voice', ('craniofacial', '*'): 'no face',
    ('wrench-cone', 'mu-kinetic'): 'her friction pyramid has one mu (wbc.MU)',
    ('wrench-cone', 'max-shear-slew-nps'): 'her friction pyramid has no slew',
}
#: Her walk's top speed ahead, m/s: past it her twist picks her jog (`se2.GAIT_BOXES`).
WALK_TOP = next(box[1] for box in se2.GAIT_BOXES if box[6] == 'walk')
#: A centre of mass on a spring, sinking onto its stance and flying between: a run's, a gallop's.
SLIP = (':ballistic-slip', ':double-peak-slip')
#: What she cannot do: (node, the test on its params and the dry's, why).
REFUSED = (
    ('oscillator', lambda n, d: n.get('inter-limb-phase-rad') != d.get('inter-limb-phase-rad'),
     'inter-limb-phase-rad: her legs half a turn apart on one clock'),
    ('oscillator', lambda n, d: min(n.get('duty-factor-l', 1.0), n.get('duty-factor-r', 1.0)) < 0.5,
     'a duty under 0.5: a flight phase'),
    ('oscillator', lambda n, d: n.get('duty-factor-l') != n.get('duty-factor-r'),
     'duty-factor-l and -r apart: one gate for both legs'),
    ('com-dynamics', lambda n, d: n.get('topology') in SLIP,
     'a spring-mass topology: a run sinks onto its stance and flies, her centre vaults'),
    ('tarsal-coronal', lambda n, d: n.get('crossover-weave') is True,
     'crossover-weave: her landings never cross'),
)


def _flat(g):
    """{(node, param): value} of a gait, a sidechain's params dotted, the derived ones beside."""
    out = {}
    for node, params in g.items():
        if node.startswith('_'):
            continue
        for k, v in params.items():
            out.update({(node, '%s.%s' % (k, j)): w for j, w in v.items()} if isinstance(v, dict) else {(node, k): v})
    for node, param, sources, how in DERIVED:
        at = g.get(node, {})
        out[(node, param)] = how(at) if sources[0] in at else None
    return out


def _covered(node, param):
    return ((node, param) in VOICELESS or (node, param.split('.')[0] + '.*') in VOICELESS
            or (node, '*') in VOICELESS or (node, param) in TOLD
            or any(n == node and p == param for n, p, _s, _r in VOICES)
            or any(n == node and param in s for n, _p, s, _h in DERIVED) or supervisor.named(node, param))


def walk(fr, gym=None):
    """Her walk's row as her loop holds it: what the walk says (`gait.said`, of her gym's entries
    `gym` where given), her rest where it says nothing (`modal.rest`, of her statics `fr`)."""
    import numpy as np
    from machine.law import gait
    said = gait.said(gym)
    return np.where(np.isnan(said), modal.rest(fr), said)


def voiced(g, dry, fr, gym=None):
    """({slot: value} of her walk's row (`walk`, of `gym`) moved by gait `g`'s changes off gait
    `dry` - only the slots it moves -, {'twist': (vx, vy, wz) told her, within her walk's box ahead,
    'grip': her floor's mu told, 'asked': the twist asked where it is past the box})."""
    for node, test, why in REFUSED:
        if test(g.get(node, {}), dry.get(node, {})):
            raise MachineError('refused: ' + why)
    supervisor.checked(g)
    now, was = _flat(g), _flat(dry)
    changed = {k for k in set(now) | set(was) if now.get(k, UNSAID.get(k)) != was.get(k, UNSAID.get(k))}
    unknown = sorted('%s.%s' % k for k in changed if not _covered(*k))
    if unknown:
        raise MachineError('no voice of hers, none named: ' + ', '.join(unknown))
    std, row = walk(fr, gym), {}
    for node, param, slot, rule in VOICES:
        v, vd = now.get((node, param)), was.get((node, param), UNSAID.get((node, param)))
        if v is None or v == vd:
            continue
        mode = slot % N if slot < 5 * N else 0
        row[slot] = RULES[rule](row.get(slot, float(std[slot])), v, vd, fr[modal.UNIT][mode])
    bad = sorted(slot for slot, v in row.items() if not math.isfinite(v))
    if bad:
        raise MachineError('voiced past her: slots %s' % bad)
    osc, dry_osc = g.get('oscillator', {}), dry.get('oscillator', {})
    stride_of = lambda x: x.get('tarsal-sagittal', {}).get('stride-length-mm', 1.0)
    speed = se2.WALK_SPEED * stride_of(g) / stride_of(dry) * osc.get('bpm', 1.0) / dry_osc.get('bpm', 1.0)
    heading = osc.get('direction-angle-rad', 0.0)
    asked = (speed * math.cos(heading), speed * math.sin(heading), osc.get('yaw-rate-radps', 0.0))
    told = {'twist': (min(asked[0], math.nextafter(WALK_TOP, 0.0)),) + asked[1:]}
    told.update({'grip': g['wrench-cone']['mu-static']} if 'wrench-cone' in g else {})
    told.update({'asked': asked} if told['twist'] != asked else {})
    return row, told


def morphed(m, dry, fr, gym=None):
    """(rows, span s) of Morph `m` on her loop (`patch.morph`), her walk `gym`'s: each slot either end
    moves on its node's band, her clock from one end's Hz to the other's on the clock's slew."""
    src, _told = voiced(m.source, dry, fr, gym)
    dst, _told = voiced(m.target, dry, fr, gym)
    std = walk(fr, gym)
    slots = (set(src) | set(dst)) - {ortho.HZ}
    src, dst = ({k: x.get(k, float(std[k])) for k in slots | {ortho.HZ}} for x in (src, dst))
    bands = {slot: (b['curve'].lstrip(':'), b['window-bars'][0], b['window-bars'][-1])
             for b in m.bands for node in b['nodes'] for n, _p, slot, _r in VOICES if n == dsp.node(node) and slot in slots}
    loose = sorted({n for n, _p, slot, _r in VOICES if slot in slots - set(bands)})
    if loose:
        raise MachineError('voiced in no band of the morph: ' + ', '.join(loose))
    c = m.clock
    clock = (c['curve'].lstrip(':'), c['window-bars'][0], c['window-bars'][-1], src[ortho.HZ], dst[ortho.HZ])
    return patch.morph(src, dst, bands, clock, m.bars)
