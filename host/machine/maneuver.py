"""The ortho-DSP suite's maneuvers (`machine.dsp`) as her texts: her sits, lies and rises as it says.

    text = maneuver.text('sit_chair', (0.47, 0.17, 0.0, 0.0))   # the suite's patch, a seat seen (`furniture.seen`)
    label, wire, names = maneuver.entry('sit_chair', (0.47, 0.17, 0.0, 0.0))    # its gym entry
    texts = maneuver.texts()                  # every stunt the suite plays (PLAYED), its own seats

A stunt plays a suite patch (PLAYED), the height of the furniture her eyes see its key; its acts
in order (`dsp`'s '_events') are phases of her text (ACTS), named by their stage of the stunt as her
stunts' phases are. They are her law's own quantities: an envelope's knee is her height (`modal`'s
axial, her legs' shortening), where a seat is seen the knee and ankle that land her hips on it (her
hips over a surface sat on: `figure.SEAT_Y`, `SEAT_R`, `HIP_DROP`) her centre of mass at her
soles' back edge, its counterweight her trunk's pitch, her centre of mass where its posture puts it
(`seated.com`). Her errand docks her heels the patch's standoff off the furniture's edge (`standoff`,
`tour`). A contact's threshold is her support's own (`impedance.PROP_ON`):
the acts it releases are laid from the descent's start and wait till the seat bears her
(`ortho.CONTACT`, `waits.borne_back`), the seat met where it is. A cross-fade is the furniture
taking her - her ZMP let go, her legs at its post-contact gain, her trunk settled: she rocks back
onto it, the support its till her soles take her back - or her soles, her ZMP engaged; a
cascade or a gate a phase a node, its modes' stiffness at its dB, -inf none (no turn of her trunk
asked, no reaction wheel of her arms); a recline her trunk's ramp. Her units and rest stiffness
are her manifest's (`ortho.robot`). What has no phase of hers is named (PASSED), any other act
refused.
"""
import collections
import functools
import math

from machine import curves, dsp, figure, impedance, leap, ortho, seated, supervisor, wrench
from machine.errors import MachineError

#: Each stunt of hers the suite plays: its patch, the stunt it rises from (its patch played on that
#: one's gait, as the suite's lifecycle plays it), her phases' names by the stunt's stages. Her chair
#: the rigid one's: the confident sit's upright trunk stands on a foot staggered under the seat's
#: edge (`pivot-single-anchor-arc`'s footprint), and her errand docks her feet together - upright,
#: her hips landed on its front edge and she slid off it. Her bed's patches are not played yet: her
#: step holds each point a bed bears still, her pelvis could not roll back on it and lying back
#: stopped at her spine's 0.6 rad (docs/TODO.md).
PLAYED = {
    'sit_chair': ('patch-sit-rigid-chair', None, {'descent': 'lower', 'contact': 'seated', 'rest': 'seated'}),
    'rise_chair': ('patch-rise-chair', 'sit_chair', {'wake': 'plant', 'preload': 'lean', 'contact': 'thrust',
                                                     'drive': 'stand'}),
}
#: The key a stunt's patch takes the seen furniture's height by, mm.
SEEN = 'seat-z-mm'
#: Her modes by the suite's nodes: what a node's stiffness or gate moves. Her head has none.
NODES = {':calcaneus': ('mode_foot_rock',), ':talocrural': ('mode_foot_rock',),
         ':tibiofemoral': ('mode_axial_compression', 'mode_limb_lift'),
         ':coxofemoral': ('mode_sagittal_shift', 'mode_lateral_shift', 'mode_stance_width', 'mode_pelvis_list',
                          'mode_pelvis_turn', 'mode_sagittal_drive'),
         ':l4-l5-lumbar': ('mode_pitch_attitude',), ':c7-t12-thoracic': ('mode_pitch_attitude',),
         ':glenohumeral': ('mode_arm_swing',), ':c1-c2-atlas': (), ':craniofacial': ()}
#: Her legs' nodes: a furniture's post-contact gain moves these.
LEGS = (':calcaneus', ':talocrural', ':tibiofemoral', ':coxofemoral')
#: Her centre of mass in her sagittal unit off where she stands, her ankles under her hips
#: (`figure.stand`, `seated.com`): over her heels' back edge, over her soles. Sat on a seat at her heels
#: her hips are 0.27 m behind her ankles, her knees over her toes (her ankle 0.3 rad): at her
#: heels' edge her trunk is 40 deg forward, over her ankles 69 (her manifest's masses: legs 11.9
#: kg, pelvis 4.7, trunk 14.6 kg 0.40 m up); the suite's eccentric hand-over, 0.68 rad, puts it at
#: her heels' edge.
#: An envelope by its mode: its stage, where her centre of mass goes.
ENVELOPES = {':eccentric-feathered-zero': 'descent', ':eccentric-damping': 'descent', ':eccentric-arc-glide': 'descent',
             ':momentum-preload': 'preload', ':concentric-drive': 'drive'}
#: A cross-fade onto these buses is her soles taking her again: her ZMP engaged, no furniture's gain.
PLANTAR = (':plantar-bilateral',)
#: Her ZMP's words (`modal`): let go, in her support.
NONE, SUPPORT = 0.0, 1.0
#: A sit's or a lie's rest, s: her text holds it till a rise is said. A row's grain, ms: the wire's.
REST_S, GRAIN_MS = 60.0, 1
#: Trace entries that are no act of a text, and what does them instead.
PASSED = {'dock-terminal-stance': 'her errand docks her heels its standoff off the edge (`standoff`, `tour`)',
          'pivot-single-anchor-arc': 'her errand docks her (`tour`)',
          'ignite-cyclic-clock': 'her walk said after her rise', 'lock-horizon-gaze': 'no head voice'}
#: A phase of her text: its stage, what it does, ms, blend ms, {mode: target}, {mode: (stiffness,
#: damping)}, ZMP or None, the event it waits for or None, whether it latches her where she is, its
#: blend's shape (`ortho.SHAPES`).
Phase = collections.namedtuple('Phase', 'stage what ms blend targets springs zmp waits latch shape',
                               defaults=(False, 'linear'))


def _manifest():
    """({mode: (unit, stiffness, damping, mass)}, her body's numbers) of her manifest (`ortho.robot`)."""
    r = ortho.robot()
    if r is None:
        raise MachineError('no (defrobot ..) in her manifest')
    return {m: (v['unit'], v['stiffness'], v['damping'], v['mass']) for m, v in r['modes'].items()}, r['body']


def _curve(p):
    """The shape of an act's curve (`ortho.SHAPES`), linear unsaid."""
    shape = p.get('curve', ':linear').lstrip(':').replace('-', '_')
    if shape not in ortho.SHAPES:
        raise MachineError('refused: a %s curve; her blends are %s' % (shape, ', '.join(ortho.SHAPES)))
    return shape


def _envelope(p, at, modes, body):
    """An envelope: a phase to its targets along its curve over its time, its counterweight her
    trunk's pitch. A descent over a seen seat bends her knee and ankle as land her hips on it
    (`seated.landed`), pressed in as far as it bears her support's share at her height's stiffness
    (`impedance.PROP_ON`, 5.7 mm): its curve lands them, the contact released (`waits.borne_back`),
    her centre of mass in her soles till then - pressed 40 mm, her hips met it at 0.3 m/s, it bore
    2.5 of her weight and threw her back onto her soles. It turns her pelvis square to the seat's
    edge - 0.05-0.1 rad off one thigh met it 1 s before the other and 11 docks of 20 fell -, and
    its arms' counterweight is her one arm mode's, one ahead and one behind (`mode_arm_swing`), the
    one on the seat's side ahead: hanging at the descent's trunk her hands are 4.5 cm under her
    buttocks and 0.27 m out, a 0.22 m seat's side 2 cm in - 3 cm off its middle one met it first,
    held (`_armed`), and propped her up off it or rolled her (34 docks of 50). Sat, a preload's
    knee and ankle are where the seat and her soles hold them, her pelvis square to her soles, her
    arms hanging: swung 25 deg either way through her rise's lean, a sole rose 58 mm and she fell."""
    stage = ENVELOPES[p['mode']]
    joints = dict(p.get('joint-targets', ()))
    trunk = p.get('axial-counterweight', {}).get('dc-offset-rad', 0.0)
    seen = stage == 'descent' and at['seat'] is not None
    sat = stage == 'preload' and at['seat'] is not None
    pressed = impedance.PROP_ON * body['weight'] / modes['mode_axial_compression'][1]
    told = (at['knee'], at['ankle']) if sat else (joints.get(':tibiofemoral', at['knee']), joints.get(':talocrural', at['ankle']))
    at['knee'], at['ankle'] = seated.landed(seated.sat(at['seat']) - pressed, at['behind'] - leap.MIDDLE, at['seat'], trunk) if seen else told
    at['leaned'] = (p['duration-ms'], _curve(p))
    # her height only where it bends her knee: a preload's asked her seat's modelled 10 mm under where
    # she sat, her legs let go and her soles bore nothing in 20 ms
    targets = {'mode_axial_compression': seated.axial(at['knee'], modes, body)} if seen or ':tibiofemoral' in joints else {}
    targets['mode_sagittal_shift'] = seated.com(at['knee'], at['ankle'], trunk, modes, body)
    seat = {'mode_arm_swing': math.copysign(p.get('appendage-counterweight', {}).get('flexion-rad', 0.0), at['aside']),
            'mode_pelvis_turn': at['turn']}
    targets.update({m: v * seen / modes[m][0] for m, v in seat.items()} if seen or sat else {})
    targets.update({'mode_pitch_attitude': trunk / modes['mode_pitch_attitude'][0]} if 'axial-counterweight' in p else {})
    # held at her rest's springs, as `_paced` paces it: on her stand's sagittal 300 her centre of mass
    # ended 2.1 cm behind a descent's, her hips 2.8 cm back, a thigh on the seat's edge and not her buttocks
    springs = {m: _sprung(m, 1.0, 1.0, modes) for m in targets if m != 'mode_arm_swing'}
    return [Phase(stage, p['mode'], p['duration-ms'], p['duration-ms'], targets, springs, None, None, shape=_curve(p))]


def _contact(p, then, at, modes, body):
    """A contact's threshold: a seat's, her support's own (`impedance.PROP_ON`) - the acts it releases
    wait for it, the first laid at the descent's start -; her soles', its share of her: the acts
    it releases wait till her soles bear it (`ortho.SOLES`)."""
    plantar = p['sensor-bus'] in PLANTAR
    if not plantar and p['trigger-fz-ratio'] != impedance.PROP_ON:
        raise MachineError('refused: a seat bears her from %g of her weight (impedance.PROP_ON), not %g'
                           % (impedance.PROP_ON, p['trigger-fz-ratio']))
    out = [phase for op, q, more in then for phase in ACTS[op](q, more, at, modes, body)]
    return [out[0]._replace(waits=('soles', p['trigger-fz-ratio']) if plantar else ('contact',))] + out[1:] if out else out


def _crossfade(p, then, at, modes, body):
    """A cross-fade: the furniture taking her - her ZMP let go, her knees held where her hips met
    the seat (bent on, her feet came off the floor), her legs at its post-contact gain -, then her
    trunk settled on it, her centre of mass where she sits so (`seated.com`; left at her heels her law
    folded her over it and stood her up), back as it leaned over her descent, its time and curve;
    or her soles, her ZMP engaged. Settled as the seat took her, or after it in its 0.32 s, her centre
    of mass asked 11 cm back at 5.4 m/s^2, her soles pushed her hips 2 cm up off it."""
    plantar, ms, shape = p['to-bus'] in PLANTAR, p['transfer-time-ms'], _curve(p)
    seat = not plantar and at['seat'] is not None
    at['knee'] = seated.knee(seated.sat(at['seat']), at['ankle']) if seat else at['knee']
    gain = 1.0 if plantar else 10.0 ** (at.get('legs-db', 0.0) / 20.0)
    springs = {m: _sprung(m, gain, 1.0, modes) for node in LEGS for m in NODES[node]}
    fade = Phase('contact', 'crossfade', ms, ms, {'mode_axial_compression': seated.axial(at['knee'], modes, body)} if seat else {},
                 springs, SUPPORT if plantar else NONE, None, shape=shape)
    settle, (leaned, curve) = p.get('thoracic-settle-rad'), at.get('leaned', (ms, shape))
    return [fade] + ([Phase('contact', 'settle', leaned, leaned, {
        'mode_pitch_attitude': settle / modes['mode_pitch_attitude'][0],
        'mode_sagittal_shift': seated.com(at['knee'], at['ankle'], settle, modes, body)}, {}, None, None, shape=curve)]
        if settle is not None and seat else [])


def _sprung(mode, gain, zeta, modes):
    """(stiffness, damping) of `mode` at `gain` of her rest stiffness, its damping ratio `zeta`
    times her rest's: damping as the root of the stiffness."""
    _unit, k, d, _mass = modes[mode]
    return k * gain, d * zeta * math.sqrt(gain)


def _cascade(p, then, at, modes, body):
    """An impedance cascade: a phase a node in its order, its modes at its dB and damping ramp - her
    arms held where they rest (`_resting`)."""
    db, (z0, z1) = p['stiffness-atten-db'], p['damping-ratio-ramp']
    gain = 0.0 if db == -math.inf else 10.0 ** (db / 20.0)
    return [Phase('rest', node, p['node-stagger-ms'], p['node-stagger-ms'], {},
                  {m: _resting(m, gain, z1 / z0, modes) for m in NODES[node]}, None, None)
            for node in p['propagation-order']]


def _resting(mode, gain, zeta, modes):
    """(stiffness, damping) of `mode` let go to `gain` of her rest's: her arms' own, held where they
    rest as a pressing contact's point is (`wrench.W_STILL`) - gravity's compensation, high-Z: no
    drive of her balance - every other mode's at `gain` (`_sprung`)."""
    held = mode == 'mode_arm_swing'
    return _sprung(mode, wrench.W_STILL / wrench.W_POSTURE if held else gain, 1.0 if held else zeta, modes)


def _gates(p, then, at, modes, body):
    """Gates a node at a time: high-Z her modes' springs none, closed-loop her rest's."""
    on = p['bridge-state'] == ':pwm-active-closed-loop'
    ms = p.get('torque-ramp-ms', p.get('torque-decay-ms'))
    return [Phase('wake' if on else 'rest', node, p['stagger-ms'], ms, {},
                  {m: _sprung(m, 1.0, 1.0, modes) if on else _resting(m, 0.0, 1.0, modes) for m in NODES[node]},
                  None, None) for node in p['nodes']]


def _equilibrium(p, then, at, modes, body):
    """Her rest on the furniture: the acts it releases wait till it bears her (`impedance.PROP_ON`)
    and its dwell after."""
    if p['min-distributed-fz-ratio'] != impedance.PROP_ON:
        raise MachineError('refused: the furniture bears her from %g of her weight (impedance.PROP_ON), not %g'
                           % (impedance.PROP_ON, p['min-distributed-fz-ratio']))
    return [Phase('rest', 'equilibrium', p['dwell-verify-ms'], p['dwell-verify-ms'], {}, {}, None, ('contact',))] + [
        phase for op, q, more in then for phase in ACTS[op](q, more, at, modes, body)]


def _recline(p, then, at, modes, body):
    """A recline: her trunk's ramp to its end, her centre of mass where she lies then (`seated.com`; left
    where she sat her law slid her hips to the edge), her pelvis's turn's, her legs' sweep's knee."""
    sweep = p.get('counterweight-limb-sweep', {})
    at['knee'] = sweep.get('tibiofemoral-flex-rad', (at['knee'],))[-1]
    trunk = p['torso-pitch-ramp'][-1]
    targets = {'mode_axial_compression': seated.axial(at['knee'], modes, body),
               'mode_pitch_attitude': trunk / modes['mode_pitch_attitude'][0],
               'mode_sagittal_shift': seated.com(at['knee'], at['ankle'], trunk, modes, body),
               'mode_pelvis_turn': p.get('axial-yaw-ramp', (0.0,))[-1] / modes['mode_pelvis_turn'][0]}
    return [Phase('recline', 'recline', p['duration-ms'], p['duration-ms'], targets, {}, None, None)]


def _latch(p, then, at, modes, body):
    """A latch: her vector where she is read at its time, the reference her gains come back on with
    no error (`ortho.LATCH`, `waits.latched_back`)."""
    return [Phase('wake', 'latch', GRAIN_MS, GRAIN_MS, {}, {}, None, None, True)]


def _compliance(p, then, at, modes, body):
    """A furniture's compliance: its legs' gain after the contact; its spring and notch the world's."""
    at['legs-db'] = p.get('post-contact-limb-gain-db', 0.0)
    return []


#: Each act: (its params, the acts it releases, her running place, her modes, her body) -> phases.
ACTS = {'trigger-one-shot-envelope': lambda p, then, at, modes, body: _envelope(p, at, modes, body),
        'on-contact-threshold': _contact, 'crossfade-support-manifold': _crossfade,
        'cascade-impedance-envelope': _cascade, 'cascade-actuator-gates': _gates,
        'on-manifold-equilibrium': _equilibrium, 'stage-sequential-manifold': _recline,
        'configure-contact-compliance': _compliance, 'latch-current-encoders': _latch}


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
    got = got if got is not None else dsp.read(dsp.SUITE)
    (g, first), (modes, body) = _played(name, got, seen[0] if seen else None), _manifest()
    told = next((info[SEEN] for step, info in g['_trace'] if isinstance(info, dict) and SEEN in info), None)
    at = {'knee': math.radians(body['knee']), 'ankle': 0.5 * math.radians(body['knee']),
          'seat': None if told is None else 1e-3 * told,
          'behind': seen[1] if seen else standoff(name) + 0.5 * (figure.BALL + figure.HEEL),
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
    return p._replace(springs={'mode_arm_swing': _sprung('mode_arm_swing', wrench.W_STILL / wrench.W_POSTURE, 1.0, modes),
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
    out, zmp = [], None
    for p in ps:
        zmp = p.zmp if p.zmp is not None else zmp
        out.append(p._replace(zmp=zmp))
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
    lines += ['(task_constraints (maintain_zmp (polygon %g)))' % p.zmp] if p.zmp is not None else []
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
