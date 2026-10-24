"""The ortho-DSP suite's maneuvers (`machine.dsp`) as her texts: her sits, lies and rises as it says.

    text = maneuver.text('sit_chair', (0.47, 0.17, 0.0, 0.0))   # the suite's patch, a seat seen (`furniture.seen`)
    label, wire, names = maneuver.entry('sit_chair', (0.47, 0.17, 0.0, 0.0))    # its gym entry
    texts = maneuver.texts()                  # every stunt the suite plays (PLAYED), its own seats

A stunt plays a suite patch (PLAYED), the height of the furniture her eyes see its key; its acts
in order (`dsp`'s '_events') are phases of her text (`machine.acts`), named by their stage of the
stunt, each paced no quicker than her springs follow it, her arms held, what bears her carried
till her soles are given back. Her errand docks her heels the patch's standoff off the furniture's
edge (`standoff`, `tour`). Her units and rest stiffness are her manifest's (`ortho.robot`). What has
no phase of hers is named (PASSED), any other act refused.
"""
import functools
import math

from machine import curves, dsp, figure, ortho, supervisor, wrench
from machine.acts import ACTS, GRAIN_MS, sprung
from machine.errors import MachineError

#: Each stunt of hers the suite plays: its patch, the stunt it rises from (its patch played on that
#: one's gait, as the suite's lifecycle plays it), her phases' names by the stunt's stages. Her chair
#: the rigid one's: the confident sit's upright trunk stands on a foot staggered under the seat's
#: edge (`pivot-single-anchor-arc`'s footprint), and her errand docks her feet together - upright,
#: her hips landed on its front edge and she slid off it. Her bed's is her gym's (motion.lisp): the
#: suite's side-lie (side_lie.lisp) rolls her onto her trochanter, past her modes (`acts._pendulum`), and
#: its supine sleep's dock, 70 mm, fell her off the mattress's edge 3 of 3 where the chair's 85 held.
PLAYED = {
    'sit_chair': ('patch-sit-rigid-chair', None, {'descent': 'lower', 'contact': 'seated', 'rest': 'seated'}),
    'rise_chair': ('patch-rise-chair', 'sit_chair', {'wake': 'plant', 'preload': 'lean', 'contact': 'thrust',
                                                     'drive': 'stand'}),
}
#: The key a stunt's patch takes the seen furniture's height by, mm.
SEEN = 'seat-z-mm'
#: A sit's or a lie's rest, s: her text holds it till a rise is said.
REST_S = 60.0
#: Trace entries that are no act of a text, and what does them instead.
PASSED = {'dock-terminal-stance': 'her errand docks her heels its standoff off the edge (`standoff`, `tour`)',
          'pivot-single-anchor-arc': 'her errand docks her (`tour`)',
          'ignite-cyclic-clock': 'her walk said after her rise', 'lock-horizon-gaze': 'no head voice'}


def _manifest():
    """({mode: (unit, stiffness, damping, mass)}, her body's numbers) of her manifest (`ortho.robot`)."""
    r = ortho.robot()
    if r is None:
        raise MachineError('no (defrobot ..) in her manifest')
    return {m: (v['unit'], v['stiffness'], v['damping'], v['mass']) for m, v in r['modes'].items()}, r['body']


def _played(name, got, seat):
    """(the gait the suite's patch for stunt `name` makes of the gait it rises from - the stunt's it
    rises from, else the suite's nominal manifold -, a seat `seat` m up its key, how many of its acts
    are that gait's)."""
    patch_name, under = PLAYED[name][:2]
    base = _played(under, got, seat)[0] if under else got['nominal-biped-manifold']
    keys = [':' + SEEN, 1e3 * seat] if seat is not None and SEEN in got[patch_name].keys else []
    g = dsp.called(patch_name, [base] + [got['nominal-biped-manifold']] * (len(got[patch_name].params) - 1), keys, got)
    if not isinstance(g, dict):
        raise MachineError('%s: a patch of a gait, not of a transition' % patch_name)
    supervisor.checked(g)
    for step, info in g['_trace']:
        if isinstance(info, dsp.Morph) and step not in PASSED:
            raise MachineError('refused: %s, no phase of hers for it' % step)
    return g, len(base['_events'])


def phases(name, seen=None, got=None):
    """[Phase] of stunt `name`, its furniture `seen` (its top, its near edge behind her soles' middle;
    None: its patch's own seat at its dock's standoff), its acts in order - those of the stunt it rises from played first, her
    place and support theirs -, a phase a contact releases laid at the descent's start."""
    given, got = got, got if got is not None else dsp.read(dsp.SUITE)
    (g, first), (modes, body) = _played(name, got, seen[0] if seen else None), _manifest()
    told = next((info[SEEN] for step, info in g['_trace'] if isinstance(info, dict) and SEEN in info), None)
    at = {'knee': math.radians(body['knee']), 'ankle': 0.5 * math.radians(body['knee']),
          'seat': None if told is None else 1e-3 * told,
          'behind': seen[1] if seen else standoff(name, given) + 0.5 * (figure.BALL + figure.HEEL),
          'aside': seen[2] if seen else 0.0, 'turn': seen[3] if seen else 0.0}
    out, own = [], []
    for k, (op, p, then) in enumerate(g['_events']):
        if op not in ACTS:
            raise MachineError('refused: %s, no phase of hers for it' % op)
        made = [_paced(_armed(phase, modes), modes) for phase in ACTS[op](p, then, at, modes, body)]
        out, own = out + made, own + [k >= first] * len(made)
    out = _supported([p._replace(ms=GRAIN_MS) if k + 1 < len(out) and out[k + 1].waits and p.stage == 'descent' else p
                      for k, p in enumerate(out)])
    return [p for p, mine in zip(out, own) if mine]


def _armed(p, modes):
    """`p`, her arms held where they hang as a pressing contact's point is (`wrench.W_STILL`)
    unless it springs them itself: no reaction wheel. Let go to their rest in her rise, they swung
    454 deg/s through 74 deg."""
    return p._replace(springs={'mode_arm_swing': sprung('mode_arm_swing', wrench.W_STILL / wrench.W_POSTURE, 1.0, modes),
                               **p.springs})


def _paced(p, modes):
    """`p` no quicker than her springs follow its curve: its time stretched as far as brings the
    curve's bandwidth, the root of its greatest |S''| over its time (`curves.BENT`), under each of
    its targets' natural frequency, the root of its mode's stiffness over its mass (`ortho.robot`).
    The suite's 350 ms preload asked her centre of mass 11.7 m/s^2 and her drives carried 0.20 of
    the ask: the seat let her go 0.16 s in and a sole rose; at 575 ms she stood 6 times of 6."""
    wn = min((math.sqrt(modes[m][1] / modes[m][3]) for m in p.targets), default=math.inf)
    least = 1e3 * math.sqrt(curves.BENT[p.shape]) / wn
    return p._replace(ms=max(p.ms, least), blend=max(p.blend, least))


def _supported(ps):
    """`ps`, each after a cross-fade on the support it gave her - the furniture's, her ZMP let go,
    till one gives her soles back - where her stunts' own (`stance_hold`) would hold her polygon."""
    out, zmp, weighs = [], None, None
    for p in ps:
        zmp = p.zmp if p.zmp is not None else zmp
        weighs = p.weighs if p.weighs is not None else weighs
        out.append(p._replace(zmp=zmp, weighs=weighs))
    return out


def _form(name, p, last):
    """Her text's phase of `p`, named `name`: the `last` of a sit or a lie held REST_S."""
    lines = ['(duration %dms) (blend_duration %dms)' % (round(1e3 * REST_S if last else p.ms), round(p.blend))]
    lines += ['(wait %s)' % ' '.join(map(str, p.waits))] if p.waits else []
    lines += ['(latch)'] if p.latch else []
    lines += ['(shape %s)' % p.shape] if p.shape != 'linear' else []
    lines += ['(latent_targets %s)' % ' '.join('(%s %.4f)' % kv for kv in p.targets.items())] if p.targets else []
    lines += ['(impedance %s)' % ' '.join('(stiffness %s %.2f) (damping %s %.2f)' % (m, k, m, d)
                                          for m, (k, d) in p.springs.items())] if p.springs else []
    pan = ' (contact_manifolds %s)' % ' '.join('(%s %g)' % kv for kv in zip(ortho.MANIFOLDS, p.weighs)) if p.weighs else ''
    lines += ['(task_constraints (maintain_zmp (polygon %g))%s)' % (p.zmp, pan)] if p.zmp is not None else []
    return '  (phase %s\n    %s)' % (name, '\n    '.join(lines))


def text(name, seen=None, got=None):
    """(defbehavior NAME ..) of stunt `name` as the suite plays it (PLAYED), its furniture `seen`."""
    named, ps = PLAYED[name][2], phases(name, seen, got)
    unnamed = sorted({p.stage for p in ps} - set(named))
    if unnamed:
        raise MachineError('%s: no phase of hers named for %s' % (name, ', '.join(unnamed)))
    names = [named[p.stage] for p in ps]
    names = [n if k + 1 == len(ps) or names[k + 1] != n else '%s_%s' % (n, ps[k].what.lstrip(':').replace('-', '_'))
             for k, n in enumerate(names)]
    return '(defbehavior %s (extends stance_hold)\n%s)\n' % (
        name, '\n'.join(_form(n, p, k == len(ps) - 1 and not PLAYED[name][1]) for k, (n, p) in enumerate(zip(names, ps))))


@functools.lru_cache(maxsize=None)
def _suite():
    return dsp.read(dsp.SUITE)


def _footprint(name, got):
    g = _played(name, got, None)[0]
    return next((1e-3 * q['sagittal-standoff-mm'] for step, info in g['_trace'] if step == 'dock-terminal-stance'
                 for op, q, _then in info.acts if op == 'align-terminal-footprint'), 0.0)


@functools.lru_cache(maxsize=None)
def _standoff(name):
    return _footprint(name, _suite())


def standoff(name, got=None):
    """m, her heels off the edge of the furniture stunt `name` docks her at, of the suite `got` (else
    its files'): its patch's terminal footprint (`dock-terminal-stance`), 0 where it docks her at none."""
    return _standoff(name) if got is None else _footprint(name, got)


def entry(name, seen=None, got=None, texts=None):
    """(label, wire, names) of stunt `name`, its furniture `seen`, of the suite `got` and her texts
    `texts` (else their files'): her gym's entry (`sprung.entries`)."""
    tables, named = ortho.compiled('(include "gait_base.lisp")\n' + text(name, seen, _suite() if got is None else got),
                                   texts=texts)
    return name, tables[name], named[name] + ('rest',)


def said(g, item, at=None):
    """A gym entry `item` as her session says it: a stunt she plays (PLAYED) compiled for the furniture
    she sees nearest `at` (`furniture.seen`), of her session's suite and texts; else itself."""
    from machine import furniture
    if item[0] not in PLAYED:
        return item
    return entry(item[0], furniture.seen(g, item[0], at), g['suite'], g['texts'])


def _joined(got):
    return '\n'.join(text(name, None, got) for name in PLAYED)


@functools.lru_cache(maxsize=1)
def _texts():
    return _joined(_suite())


def texts(got=None):
    """Every stunt the suite plays (PLAYED), its patches' own seats, as one text for her gym, of the
    suite `got` (else its files')."""
    return _texts() if got is None else _joined(got)
