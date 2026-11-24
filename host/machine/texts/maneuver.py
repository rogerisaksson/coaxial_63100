"""The ortho-DSP suite's maneuvers (`dsp`) as the robot's texts: its sits, lies, rises as said.

    text = maneuver.text('sit_chair', (0.47, 0.17, 0.0, 0.0))   # the suite's patch, a seat seen (`furniture.seen`)
    label, wire, names = maneuver.entry('sit_chair', (0.47, 0.17, 0.0, 0.0))    # its gym entry
    texts = maneuver.texts()                  # every stunt the suite plays (PLAYED), its own seats
    programs = maneuver.programs('lie_bed')   # {phase: the joint program the robot's boards play from its first tick}

A stunt plays a suite patch (PLAYED), the height of the furniture the robot's eyes see its key; its acts
in order (`dsp`'s '_events') are phases of its text (`machine.texts.acts`), named by their stage of
the stunt, each paced no quicker than its springs follow it, its arms held, what bears it
carried till its soles are given back. The robot's errand docks its heels the patch's standoff
off the furniture's edge (`standoff`, `tour`); its units and rest stiffness are its manifest's
(`defrobot.read`); what has no phase of its own is named (PASSED), any other act refused. An act's
`:joint-program` binds a program of the robot's joints (`machine.process.joints`) to its first phase: said,
the stunt lays it on its run (`said`) and its boards play it from that
phase's first tick over the law's phases.
"""
import functools
import math

from machine.texts import curves, dsp, ortho
from machine.body import defrobot, figure
from machine.law import impedance, supervisor
from machine.moves import seated
from machine.stack import wrench
from machine.texts.acts import ACTS, GRAIN_MS, LEGS, NODES, SUPPORT, sprung
from machine.errors import MachineError

#: The robot's legs' modes: at the furniture's gain through a cross-fade onto it, back toward their rest
#: through a pan to its soles (`_legs`).
LEGS_MODES = tuple(m for node in LEGS for m in NODES[node])

#: Each stunt of the robot's own the suite plays: its patch, the stunt it rises from (its patch played on that
#: one's gait, as the suite's lifecycle plays it), its phases' names by the stunt's stages; its chair the
#: rigid one's: the confident sit's upright trunk stands on a foot staggered under the seat's edge
#: (`pivot-single-anchor-arc`'s footprint), and its errand docks its feet together - upright, its hips landed
#: on its front edge and it slid off it; its bed's is its gym's (motion.lisp): the suite's side-lie
#: (side_lie.lisp) rolls it onto its trochanter, past its modes (`acts._pendulum`), and its supine sleep's
#: dock, 70 mm, fell it off the mattress's edge 3 of 3 where the chair's 85 held.
PLAYED = {
    'sit_chair': ('patch-sit-rigid-chair', None, {'descent': 'lower', 'contact': 'seated', 'rest': 'seated',
                                                  'recline': 'seated'}),
    'rise_chair': ('patch-rise-chair', 'sit_chair', {'recline': 'scoot', 'wake': 'plant', 'preload': 'lean',
                                                     'contact': 'thrust', 'drive': 'stand'}),
    'lie_bed': ('patch-bed-lean', None, {'descent': 'sit_edge', 'contact': 'sit_edge', 'recline': 'bed_rest'}),
    'leg_bed': ('patch-leg-bed', None, {'descent': 'sit_edge', 'contact': 'sit_edge', 'recline': 'bed_rest'}),
    'rise_bed': ('patch-rise-bed', 'lie_bed', {'recline': 'sit_up', 'wake': 'plant', 'preload': 'lean', 'contact': 'thrust',
                                              'drive': 'stand'}),
    'rise_floor': ('patch-rise-bed', 'lie_bed', {'recline': 'sit_up', 'wake': 'plant', 'preload': 'lean',
                                                 'contact': 'thrust', 'drive': 'stand'}),
}
#: The key a stunt's patch takes the seen furniture's height by, mm; the key it takes the edge the robot sat
#: on by (`side`): a bed's end lain along straight back, its side rolled -90 deg onto (bed_lean.lisp).
SEEN, SIDE = 'seat-z-mm', 'bed-side'
#: The key a patch takes the robot's popliteal height by, mm (`seated.POPLITEAL`): a seat over it hangs its
#: feet off the floor.
POPLITEAL = 'popliteal-mm'
#: The ischia's height over a seat bearing them, m (`figure`'s seat capsules).
ISCHIA = -figure.SEAT_Y + figure.SEAT_R


@functools.lru_cache(maxsize=1)
def _reach():
    """m, the robot's pelvis's floor on its legs: flat soles, the knees at their drives' range (its
    manifest's); a seat asking its pelvis under it has no leg path - the deep landing is a tip onto a known
    seat, protocol item 12 - and its stunt is refused by name (`text`)."""
    r = defrobot.read()
    if r is None:
        raise MachineError('no (defrobot ..) in the robot\'s manifest')
    knee = math.radians(next(v['range'][1] for k, v in r['drives'].items() if 'knee' in k))
    return figure.pelvis_at(knee)
#: A sit's or a lie's rest, s: the text holds it till a rise is said.
REST_S = 60.0
#: Trace entries that are no act of a text, and what does them instead.
PASSED = {'dock-terminal-stance': 'the robot\'s errand docks its heels its standoff off the edge (`standoff`, `tour`)',
          'pivot-single-anchor-arc': 'the robot\'s errand docks it (`tour`)',
          'ignite-cyclic-clock': 'the robot\'s walk said after its rise', 'lock-horizon-gaze': 'no head voice'}


@functools.lru_cache(maxsize=8)
def _manifest_for(_robot, _path):
    r = defrobot.read()
    if r is None:
        raise MachineError('no (defrobot ..) in the robot\'s manifest')
    return {m: (v['unit'], v['stiffness'], v['damping'], v['mass']) for m, v in r['modes'].items()}, dict(r['body'])


def _manifest():
    """({mode: (unit, stiffness, damping, mass)}, the robot's body's numbers) of its manifest
    (`defrobot.read`)."""
    return _manifest_for(defrobot.mechatronics.ROBOT, ortho._FILE)


def side(seen):
    """The edge of the furniture `seen` (`furniture.seen`) the robot backs onto: ':short' where that face is
    narrower than the furniture is deep across it - a bed's end -, ':long' else; None with no depth seen
    (the patch's own, `*bed-side*`)."""
    if not seen or len(seen) < 6 or seen[4] is None or seen[5] is None:
        return None
    return ':short' if seen[5] < seen[4] else ':long'


def _played(name, got, seat, edge=None, popliteal=seated.POPLITEAL):
    """(the gait the suite's patch for stunt `name` makes of the gait it rises from - the stunt's it
    rises from, else the suite's nominal manifold -, a seat `seat` m up its key, the edge `edge` the robot sat
    on (`side`) its key, how many of its acts are that gait's)."""
    patch_name, under = PLAYED[name][:2]
    base = _played(under, got, seat, edge, popliteal)[0] if under else got['nominal-biped-manifold']
    keys = [':' + SEEN, 1e3 * seat] if seat is not None and SEEN in got[patch_name].keys else []
    keys += [':' + SIDE, edge] if edge is not None and SIDE in got[patch_name].keys else []
    keys += [':' + POPLITEAL, 1e3 * popliteal] if POPLITEAL in got[patch_name].keys else []
    g = dsp.called(patch_name, [base] + [got['nominal-biped-manifold']] * (len(got[patch_name].params) - 1), keys, got)
    if not isinstance(g, dict):
        raise MachineError('%s: a patch of a gait, not of a transition' % patch_name)
    supervisor.checked(g)
    for step, info in g['_trace']:
        if isinstance(info, dsp.Morph) and step not in PASSED:
            raise MachineError('refused: %s, no phase of the robot\'s own for it' % step)
    return g, len(base['_events'])


def phases(name, seen=None, got=None):
    """[Phase] of stunt `name`, its furniture `seen` (`_phases`): its phases alone."""
    return [p for p, _bound in _phases(name, seen, got)]


def _phases(name, seen=None, got=None):
    """[(Phase, the joint program its first tick begins or None)] of stunt `name`, its furniture `seen` (its
    top, its near edge behind the robot's soles' middle, its
    middle aside, its turn, its depth, its width; None: its patch's own seat at its dock's standoff, no depth), its acts in
    order - those of the stunt it rises from played first, the robot's place and support theirs -, a phase a contact
    releases laid at the descent's start; an act's `:joint-program` on its first phase, the robot's boards' program from
    that tick (`machine.process.joints`)."""
    given, got = got, got if got is not None else dsp.read(dsp.SUITE)
    modes, body = _manifest()
    pressed = impedance.PROP_ON * body['weight'] / modes['mode_axial_compression'][1]
    no_palms = (seen is not None and len(seen) > 5 and seen[5] is not None and seated.reaches(seen[0])
                and 0.5 * seen[5] - abs(seen[2]) - seated.placed(seated.ON - pressed, 0.0)[2] < 0.0)
    pop = max(seated.POPLITEAL, seen[0]) if seen is not None and no_palms else seated.POPLITEAL
    g, first = _played(name, got, seen[0] if seen else None, side(seen), pop)
    told = next((info[SEEN] for step, info in g['_trace'] if isinstance(info, dict) and SEEN in info), None)
    at = {'knee': math.radians(body['knee']), 'ankle': 0.5 * math.radians(body['knee']),
          'seat': None if told is None else 1e-3 * told,
          'behind': seen[1] if seen else standoff(name, given) + 0.5 * (figure.BALL + figure.HEEL),
          'aside': seen[2] if seen else 0.0, 'turn': seen[3] if seen else 0.0, 'deep': seen[4] if seen else None,
          'across': seen[5] if seen and len(seen) > 5 else None,
          'reclines': any(op == 'stage-sequential-manifold' for op, _p, _t in g['_events'][first:])}
    out, own, bound = [], [], []
    for k, (op, p, then) in enumerate(g['_events']):
        if op not in ACTS:
            raise MachineError('refused: %s, no phase of the robot\'s own for it' % op)
        made = [_paced(_armed(_placed(_held(phase, p), p, modes), modes), modes)
                for phase in ACTS[op](p, then, at, modes, body)]
        out, own = out + made, own + [k >= first] * len(made)
        bound += [_program(p, at) if i == 0 else None for i in range(len(made))]
    out = _supported([p._replace(ms=GRAIN_MS) if k + 1 < len(out) and out[k + 1].waits and p.stage == 'descent' else p
                      for k, p in enumerate(out)], modes)
    return [(p, b) for p, mine, b in zip(out, own, bound) if mine]


def _held(p, stage):
    """`p`, its trunk's springs (pitch and shift) the stage's `:trunk-hold` times stiffer: a lean on the fists
    behind the robot held (at the bed's end on the rest's springs it fell 58 -> 5-16 deg; held 4 it stayed
    28-40 over 2 s on fists 0.22 m behind; 6 pressed the hands 200-700 N and wrecked it)."""
    hold = float(stage.get('trunk-hold', 1.0)) if hasattr(stage, 'get') else 1.0
    if hold == 1.0:
        return p
    held = {m: (kd[0] * hold, kd[1] * math.sqrt(hold)) for m, kd in p.springs.items() if m in TRUNK_MODES}
    return p._replace(springs={**p.springs, **held})


#: The modes `:trunk-hold` stiffens.
TRUNK_MODES = ('mode_pitch_attitude', 'mode_sagittal_shift')


def _placed(p, stage, modes):
    """`p`, the stage's `:limb-back-m` as its drive's target with its lift's sign, the drive sprung: the leg
    the lift lets go placed on the bed, its ankle that far behind where its knee had it (`cascade.opening`)."""
    back = stage.get('limb-back-m') if hasattr(stage, 'get') else None
    if back is None:
        return p
    lift = float(stage.get('counterweight-limb-sweep', {}).get('limb-lift', 0.0))
    drive = (1.0 if lift >= 0.0 else -1.0) * figure.SCALE * float(back) / modes['mode_sagittal_drive'][0]
    return p._replace(targets={**p.targets, 'mode_sagittal_drive': drive},
                      springs={'mode_sagittal_drive': sprung('mode_sagittal_drive', 1.0, 1.0, modes), **p.springs})


def _armed(p, modes):
    """`p`, the robot's arms held where they hang as a pressing contact's point is (`wrench.W_STILL`) unless
    it springs them itself: no reaction wheel. Let go to their rest in a rise, they swung 454 deg/s through 74
    deg."""
    return p._replace(springs={'mode_arm_swing': sprung('mode_arm_swing', wrench.W_STILL / wrench.W_POSTURE, 1.0, modes),
                               **p.springs})


def _paced(p, modes):
    """`p` no quicker than the springs follow its curve: its time stretched as far as brings the curve's
    bandwidth, the root of its greatest |S''| over its time (`curves.BENT`), under each of its targets'
    natural frequency, the root of its mode's stiffness over its mass (`defrobot.read`). The suite's 350 ms
    preload asked the robot's centre of mass 11.7 m/s^2 and its drives carried 0.20 of the ask: the seat let
    it go 0.16 s in and a sole rose; at 575 ms it stood 6 times of 6."""
    wn = min((math.sqrt(modes[m][1] / modes[m][3]) for m in p.targets), default=math.inf)
    least = 1e3 * math.sqrt(curves.BENT[p.shape]) / wn
    return p._replace(ms=max(p.ms, least), blend=max(p.blend, least))


def _supported(ps, modes):
    """`ps`, each after a cross-fade on the support it gave the robot - the furniture's, its ZMP let go, till
    one gives its soles back - where its stunts' own (`stance_hold`) would hold its polygon; a preload before
    a cross-fade onto its soles pans what bears it to the share that fade waits for (the handover's liftoff),
    its ZMP in its support as they take it and its legs' springs back from the furniture's gain toward their
    rest by that share (`_panned`, `_legs`). Left to the fade, the seat bore 0.9-1.1 of the robot through the
    lean and one sole took it at the thrust, 33/156 N; panned with its legs at the seat's -24 dB, one sole
    took it through the lean and rolled its 13 deg. A phase whose pan puts all of the robot on its soles (a
    move's landing: off a seat over its popliteal height, `chair-dismount`) gives its ZMP to its support and
    its legs their rest as a fade onto them does: let go till the rise's fade, it bore one sole at a time
    (141/0, 0/389 N), stepping at 14 s, knees 73-77 deg."""
    out, zmp, weighs, legs = [], None, None, None
    for p, then in zip(ps, ps[1:] + [None]):
        weighs = p.weighs if p.weighs is not None else weighs
        soled = weighs is not None and weighs[0] >= 1.0 and p.zmp is None and zmp != SUPPORT
        zmp = p.zmp if p.zmp is not None else SUPPORT if soled else zmp
        legs = p.springs if all(m in p.springs for m in LEGS_MODES) else legs
        panned = _panned(p, then, weighs)
        share = panned[0] if panned else 1.0 if soled else None
        springs = {**_legs(legs, share, modes), **p.springs} if share is not None and legs else p.springs
        out.append(p._replace(zmp=SUPPORT if panned else zmp, weighs=panned or weighs, springs=springs))
    return out


def _legs(seat, plantar, modes):
    """The legs' modes' springs `plantar` of the way from the furniture's gain on them (`seat`, its
    cross-fade's springs) to their rest (`modes`)."""
    return {m: sprung(m, seat[m][0] / modes[m][1] + plantar * (1.0 - seat[m][0] / modes[m][1]), 1.0, modes)
            for m in LEGS_MODES}


def _panned(p, then, weighs):
    """A preload's pan: the robot's soles at the share the cross-fade `then` waits for, what bore it off its
    hands the rest as it was split, its hands none - its arms hang in the lean (`acts._envelope`); None where
    `p` is no such preload."""
    soles = then.waits[1] if then is not None and then.waits and then.waits[0] == 'soles' and weighs else None
    if p.stage != 'preload' or soles is None or weighs[0] >= 1.0:
        return None
    hands = weighs[ortho.PALMAR]
    return (soles,) + tuple(w * (k != ortho.PALMAR) * (1.0 - soles) / (1.0 - weighs[0] - hands)
                            for k, w in enumerate(weighs[1:], 1))


def _form(name, p, last):
    """The text's phase of `p`, named `name`: the `last` of a sit or a lie held REST_S."""
    lines = ['(duration %dms) (blend_duration %dms)' % (round(1e3 * REST_S if last else p.ms), round(p.blend))]
    lines += ['(wait %s)' % ' '.join(map(str, p.waits))] if p.waits else []
    lines += ['(latch)'] if p.latch else []
    lines += ['(shape %s)' % p.shape] if p.shape != 'linear' else []
    lines += ['(latent_targets %s)' % ' '.join('(%s %.4f)' % kv for kv in p.targets.items())] if p.targets else []
    lines += ['(impedance %s)' % ' '.join('(stiffness %s %.2f) (damping %s %.2f)' % (m, k, m, d)
                                          for m, (k, d) in p.springs.items())] if p.springs else []
    held = (['(maintain_zmp (polygon %g))' % p.zmp] if p.zmp is not None else []) + (
        ['(contact_manifolds %s)' % ' '.join('(%s %g)' % kv for kv in zip(ortho.MANIFOLDS, p.weighs))] if p.weighs else []) + (
        ['(slide %g)' % p.slide] if p.slide is not None else [])
    lines += ['(task_constraints %s)' % ' '.join(held)] if held else []
    return '  (phase %s\n    %s)' % (name, '\n    '.join(lines))


def _program(p, at):
    """What an act's `:joint-program` says of the robot's joints (`machine.process.joints`, the loop's word):
    the program, the groups its law keeps while it plays (`:joint-keep`, the player's KEPT), and where the act
    gives a stroke's gain up the furniture (`:stroke-m`) and where the program lands its middle off its edge
    (`:landed-m`), the strokes that bring its middle to the furniture's middle by the depth its eyes saw
    (`at['deep']`); None with no program."""
    name = p.get('joint-program')
    if name is None:
        return None
    strokes = (max(0, math.floor((0.5 * at['deep'] / figure.SCALE - p.get('landed-m', 0.0)) / p['stroke-m'] + 0.5))
               if 'stroke-m' in p and at['deep'] is not None else None)
    kept = str(p.get('joint-keep', '')).lstrip(':')
    return {'joints': str(name).lstrip(':').replace('-', '_'), **({'keep': kept} if kept else {}),
            **({} if strokes is None else {'strokes': strokes})}


def _named(name, seen=None, got=None):
    """([phase name], [Phase], [its joint program or None]) of stunt `name`, its furniture `seen`: its phases
    (`_phases`) named by their stage (PLAYED), a stage's run by what each does; a seat asking the robot's
    pelvis under its legs' floor (`_reach`) refused by name - the suite's bed patch at a 0.14 m floor mattress
    asked mode_axial_compression 4.10 where its legs reach 3.4 and it froze mid-descent at 0.505 m, every
    joint still."""
    if seen is not None and seen[0] + ISCHIA < _reach():
        raise MachineError('%s: a seat %.2f m up asks the robot\'s pelvis %.3f m, under its legs %.3f '
                           '(the deep landing, protocol item 12)' % (name, seen[0], seen[0] + ISCHIA, _reach()))
    named, made = PLAYED[name][2], _phases(name, seen, got)
    ps, bound = [p for p, _b in made], [b for _p, b in made]
    unnamed = sorted({p.stage for p in ps} - set(named))
    if unnamed:
        raise MachineError('%s: no phase of the robot\'s own named for %s' % (name, ', '.join(unnamed)))
    names = [named[p.stage] for p in ps]
    names = [n if k + 1 == len(ps) or names[k + 1] != n else '%s_%s' % (n, ps[k].what.lstrip(':').replace('-', '_'))
             for k, n in enumerate(names)]
    return names, ps, bound


def text(name, seen=None, got=None):
    """(defbehavior NAME ..) of stunt `name` as the suite plays it (PLAYED), its furniture `seen` (`_named`)."""
    return _text(name, *_named(name, seen, got)[:2])


def _text(name, names, ps):
    return '(defbehavior %s (extends stance_hold)\n%s)\n' % (
        name, '\n'.join(_form(n, p, k == len(ps) - 1 and not PLAYED[name][1]) for k, (n, p) in enumerate(zip(names, ps))))


def programs(name, seen=None, got=None):
    """{phase: the loop's word for a joint program} of stunt `name`, its furniture `seen` (`_program`): what the
    robot's boards play from that phase's first tick (`machine.process.joints`), its strokes by the furniture
    seen."""
    names, _ps, bound = _named(name, seen, got)
    return _bound(names, bound)


def _bound(names, bound):
    return {n: b for n, b in zip(names, bound) if b is not None}


@functools.lru_cache(maxsize=None)
def _suite():
    return dsp.read(dsp.SUITE)


def _aligned(name, got):
    g = _played(name, got, None)[0]
    return next((q for step, info in g['_trace'] if step == 'dock-terminal-stance'
                 for op, q, _then in info.acts if op == 'align-terminal-footprint'), {})


def _footprint(name, got):
    return 1e-3 * figure.SCALE * _aligned(name, got).get('sagittal-standoff-mm', 0.0)


@functools.lru_cache(maxsize=None)
def _standoff(name):
    return _footprint(name, _suite())


def standoff(name, got=None):
    """m, the robot's heels off the edge of the furniture stunt `name` docks it at, of the suite `got` (else
    its files'): its patch's terminal footprint (`dock-terminal-stance`), 0 where it docks at none."""
    return _standoff(name) if got is None else _footprint(name, got)


@functools.lru_cache(maxsize=None)
def face(name):
    """The face of the furniture stunt `name` docks the robot at first where more than one is its own to
    reach, its patch's terminal footprint's (`dock-terminal-stance`): ':short' a bed's end, ':long' its side,
    ':any' whichever its walk reaches soonest; None where it docks at none."""
    return _aligned(name, _suite()).get('fixture-face')


def entry(name, seen=None, got=None, texts=None):
    """(label, wire, names) of stunt `name`, its furniture `seen`, of the suite `got` and the robot's texts
    `texts` (else their files'): its gym's entry (`sprung.entries`)."""
    return _entry(name, text(name, seen, _suite() if got is None else got), texts)


def _entry(name, source, texts):
    tables, named = ortho.compiled('(include "gait_base.lisp")\n' + source, texts=texts)
    return name, tables[name], named[name] + ('rest',)


def said(g, item, at=None, thing=None):
    """A gym entry `item` as the robot's session says it: a stunt (PLAYED) compiled for the furniture seen
    nearest `at` (slot `thing` where given, `furniture.seen`); unseen, the robot looks till a sighting is its
    own (`registry.TAKEN` of HIT a look), else refused by name. The joint programs laid on `g['programs']`
    (`machine.process.joints`), the seat's top m on `g['seat']` (`melt.step`, inf outside stunts)."""
    from machine.way import furniture
    from machine.process import sprung
    g['seat'] = math.inf
    if item[0] not in PLAYED:
        return item
    from machine.law import modal, se2, stride
    # on the floor (`se2.GROUNDED`) the floor is the seat, top 0: a rise there is refused by leg reach
    floor = float(sprung.doing(g) in se2.GROUNDED)
    verb = PLAYED[item[0]][1] or item[0]
    seen, edge, reach, sat_slot = (((0.0, 0.0, 0.0, 0.0, 0.0), 0.0, stride.STRIDE_M, None) if floor
                                   else furniture.at_hand(g, item[0], verb, at, thing))
    if seen is None:
        raise MachineError('%s: nothing among the robot\'s things affords it' % item[0])
    if edge > reach:
        raise MachineError('%s plays where the robot stands: its furniture is %.1f m off, past the robot %.2f '
                           '(a stride and the standoff) - an errand walks there' % (item[0], edge, reach))
    g['sat_thing'] = sat_slot if PLAYED[item[0]][1] is None and not floor else None
    names, ps, bound = _named(item[0], seen, _suite() if g['suite'] is None else g['suite'])
    programs_ = _bound(names, bound)
    if item[0] == 'rise_chair' and 'y' in g and 'loads' in g and 'fr' in g:
        wt = float(g['fr'][modal.WEIGHT])
        if wt > 0.0 and float(g['y'][modal.PROP]) < 0.05 and sum(g['loads']) >= 0.82 * wt:
            programs_ = {k: v for k, v in programs_.items() if v.get('joints') != 'chair_dismount'}
    g['programs'], g['seat'] = programs_, float(seen[0])
    return _entry(item[0], _text(item[0], names, ps), g['texts'])


def _joined(got):
    return '\n'.join(text(name, None, got) for name in PLAYED)


@functools.lru_cache(maxsize=1)
def _texts():
    return _joined(_suite())


def texts(got=None):
    """Every stunt the suite plays (PLAYED), its patches' own seats, as one text for the gym, of the
    suite `got` (else its files')."""
    return _texts() if got is None else _joined(got)
