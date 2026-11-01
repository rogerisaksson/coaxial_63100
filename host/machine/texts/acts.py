"""The suite's acts as phases of her text (`machine.texts.maneuver`).

    phases = acts.ACTS[op](params, released, at, modes, body)   # an act's phases of her text

They are her law's own quantities: an envelope's knee is her height (`modal`'s axial), where a
seat is seen the knee and ankle that land her hips on it (`figure.SEAT_Y`, `SEAT_R`, `HIP_DROP`),
her centre of mass where its posture puts it (`seated.com`). A contact's threshold is her
support's own (`impedance.PROP_ON`): the acts it releases are laid from the descent's start and
wait till the seat bears her (`waits.borne_back`). A cross-fade is the furniture taking her - her
ZMP let go, her legs at its post-contact gain, what bears her panned to it (`ortho.MANIFOLDS`) - or
her soles; a cascade or a gate a phase a node, its modes' stiffness at its dB (no reaction wheel of
her arms); a recline her trunk's ramp; a pendulum her pelvis listed and turned onto her hip, refused
past her modes' reach; a seek onto her carpal heels' manifold her fists set on a seat over her shins.
"""
import collections
import math

from machine.body import drives, figure
from machine.law import impedance, somatic
from machine.texts import kernel, ortho
from machine.moves import leap, seated
from machine.stack import wrench
from machine.errors import MachineError

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
#: A row's grain, ms: the wire's.
GRAIN_MS = 1
#: A phase of her text: its stage, what it does, ms, blend ms, {mode: target}, {mode: (stiffness,
#: damping)}, ZMP or None, the event it waits for or None, whether it latches her where she is, its
#: blend's shape (`ortho.SHAPES`), what bears her panned (`ortho.MANIFOLDS`), her slide over it m/s.
Phase = collections.namedtuple('Phase', 'stage what ms blend targets springs zmp waits latch shape weighs slide',
                               defaults=(False, 'linear', None, None))
#: A support bus's contact manifold (`ortho.MANIFOLDS`).
BUSES = {':plantar-bilateral': 0, ':ischial-bilateral': 1, ':trochanter-lateral': 2, ':scapular-dorsal': 3,
         ':occipital': 3, ':palmar-carpal-bilateral': 4}
#: A contact wavefront's frames as her support buses.
WAVEFRONT = {':greater-trochanter-r': ':trochanter-lateral', ':greater-trochanter-l': ':trochanter-lateral',
             ':ischial-sacral': ':ischial-bilateral', ':scapular-dorsal': ':scapular-dorsal', ':occipital': ':occipital'}
#: Her pelvis's half-width across, m: her hips' height over what she lies on her side on (`figure.BODY`).
LAIN = next(0.5 * abs(b[0] - a[0]) + r for seg, r, a, b in figure.BODY if seg == 'pelvis' and a[0] != b[0])
#: Her hands' frames a seat's manifold bears on (`_sought`), her carpal heels; the modes that set them.
CARPAL, ARMS = (':carpal-heel-l', ':carpal-heel-r'), ('mode_arm_reach', 'mode_arm_fold')
#: The kernel's seek of her arms down onto what her hands find (`specialize-with-tactile-arm-sweep`), m/s.
SEEK_M_S = -kernel.node('specialize-with-tactile-arm-sweep', 'appendage-admittance')['seek-velocity-mps'][2]
#: What her shoulder's and her elbow's drives give at the most, N m (`drives.peak`).
PEAKS = (drives.peak('left_shoulder'), drives.peak('left_elbow'))


def _weighs(pan):
    """Each manifold's weight of a pan ((bus weight) ..), None unsaid."""
    return None if pan is None else tuple(sum(w for bus, w in pan if BUSES[bus] == k) for k in range(len(ortho.MANIFOLDS)))


def _curve(p):
    """The shape of an act's curve (`ortho.SHAPES`), linear unsaid."""
    shape = p.get('curve', ':linear').lstrip(':').replace('-', '_')
    if shape not in ortho.SHAPES:
        raise MachineError('refused: a %s curve; her blends are %s' % (shape, ', '.join(ortho.SHAPES)))
    return shape


def _envelope(p, at, modes, body):
    """An envelope: a phase to its targets along its curve over its time, its counterweight her
    trunk's pitch, what bears her panned as it says (`manifold-pan`). A descent over a seen seat
    lands her hips on it (`seated.landed`) pressed 5.7 mm (`impedance.PROP_ON`; 40 mm, it bore 2.5
    of her and threw her off), her pelvis square to its edge (0.05-0.1 rad off, 11 docks of 20
    fell), her arms one ahead and one behind, the seat's side's ahead (hanging, a hand met it in 34
    docks of 50). Sat, a preload's knee and ankle are where the seat and her soles hold them, her
    arms hanging - her fists off a seat they bore on (`_sought`): swung 25 deg either way, a sole rose
    58 mm and she fell; it slides her forward off the seat as the kernel's egress has it
    (`somatic.EGRESS_M_S`)."""
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
    # a preload to her soles' middle, no further: 5 cm past it, the seat let her go at 0.54 m/s
    com = seated.com(at['knee'], at['ankle'], trunk, modes, body)
    targets['mode_sagittal_shift'] = min(com, 0.0) if sat else com
    reclines = seen and bool(at.get('reclines'))
    seat = {'mode_arm_swing': 0.0 if reclines else math.copysign(p.get('appendage-counterweight', {}).get('flexion-rad', 0.0), at['aside']),
            'mode_pelvis_turn': at['turn']}
    at['swung'] = seat['mode_arm_swing'] if seen else at.get('swung', 0.0)
    targets.update({m: v * seen / modes[m][0] for m, v in seat.items()} if seen or sat else {})
    targets.update({'mode_arm_reach': -2.8, 'mode_arm_fold': 1.8} if reclines else {})
    targets.update(dict.fromkeys(ARMS, 0.0) if sat and at.get('palmed') else {})
    targets.update({'mode_pitch_attitude': trunk / modes['mode_pitch_attitude'][0]} if 'axial-counterweight' in p else {})
    # held at her rest's springs, as `_paced` paces it: on her stand's sagittal 300 her centre of mass
    # ended 2.1 cm behind a descent's, her hips 2.8 cm back, a thigh on the seat's edge and not her buttocks
    springs = {m: sprung(m, 1.0, 1.0, modes) for m in targets if m != 'mode_arm_swing'}
    return [Phase(stage, p['mode'], p['duration-ms'], p['duration-ms'], targets, springs, None, None, shape=_curve(p),
                  weighs=_weighs(p.get('manifold-pan')), slide=somatic.EGRESS_M_S if sat else None)]


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
    or her soles, her ZMP engaged and her slide over the seat ended. Settled as the seat took her, or
    after it in its 0.32 s, her centre of mass asked 11 cm back at 5.4 m/s^2, her soles pushed her hips
    2 cm up off it."""
    plantar, ms, shape = p['to-bus'] in PLANTAR, p['transfer-time-ms'], _curve(p)
    seat = not plantar and at['seat'] is not None
    at['knee'] = seated.knee(seated.sat(at['seat']), at['ankle']) if seat else at['knee']
    gain = 1.0 if plantar else 10.0 ** (at.get('legs-db', 0.0) / 20.0)
    springs = {m: sprung(m, gain, 1.0, modes) for node in LEGS for m in NODES[node]}
    left = p.get('source-residual-weight', 0.0)
    fade = Phase('contact', 'crossfade', ms, ms, {'mode_axial_compression': seated.axial(at['knee'], modes, body)} if seat else {},
                 springs, SUPPORT if plantar else NONE, None, shape=shape,
                 weighs=_weighs(((p['from-bus'], left), (p['to-bus'], 1.0 - left))), slide=0.0 if plantar else None)
    settle, (leaned, curve) = p.get('thoracic-settle-rad'), at.get('leaned', (ms, shape))
    at['weighs'], at['trunk'] = fade.weighs, settle if settle is not None and seat else at.get('trunk', 0.0)
    return [fade] + ([Phase('contact', 'settle', leaned, leaned, {
        'mode_pitch_attitude': settle / modes['mode_pitch_attitude'][0],
        'mode_sagittal_shift': seated.com(at['knee'], at['ankle'], settle, modes, body)}, {}, None, None, shape=curve)]
        if settle is not None and seat else [])


def sprung(mode, gain, zeta, modes):
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
    """(stiffness, damping) of `mode` let go to `gain` of her rest's, her arms' held where they rest as
    a pressing contact's point is (`wrench.W_STILL`)."""
    held = mode == 'mode_arm_swing'
    return sprung(mode, wrench.W_STILL / wrench.W_POSTURE if held else gain, 1.0 if held else zeta, modes)


def _gates(p, then, at, modes, body):
    """Gates a node at a time: high-Z her modes' springs none, closed-loop her rest's, what bears her
    panned where they say so (a wake's pan to her soles, 0.85, capped the seat at 0.15 of her as she sat
    upright: the handover's lean pans her, `maneuver._panned`)."""
    on = p['bridge-state'] == ':pwm-active-closed-loop'
    ms = p.get('torque-ramp-ms', p.get('torque-decay-ms'))
    return [Phase('wake' if on else 'rest', node, p['stagger-ms'], ms, {},
                  {m: sprung(m, 1.0, 1.0, modes) if on else _resting(m, 0.0, 1.0, modes) for m in NODES[node]},
                  None, None, weighs=_weighs(p.get('manifold-pan'))) for node in p['nodes']]


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
    where she sat her law slid her hips to the edge), her pelvis's turn's, her legs' sweep's knee; what bears
    her panned as it says (`manifold-pan`), her fists on the seat behind her where it pans her hands
    (`seated.braced`), held there as a pressing contact's point is (`wrench.W_STILL`), hanging where it pans them
    none; her centre of mass on her
    rest's spring: on the seat's -24 dB her trunk came back upright from -13 deg as her pelvis rolled under it."""
    sweep = p.get('counterweight-limb-sweep', {})
    at['knee'] = sweep.get('tibiofemoral-flex-rad', (at['knee'],))[-1]
    trunk, weighs = p['torso-pitch-ramp'][-1], _weighs(p.get('manifold-pan'))
    spine = max(trunk, math.radians(drives.TRAVEL['spine'][0]))
    supine = weighs is not None and weighs[0] < 0.12
    com = seated.com(at['knee'], at['ankle'], spine, modes, body)
    targets = {'mode_axial_compression': seated.axial(at['knee'], modes, body),
               'mode_pitch_attitude': trunk / modes['mode_pitch_attitude'][0],
               'mode_sagittal_shift': min(com, -1.2) if supine else com,
               'mode_pelvis_turn': p.get('axial-yaw-ramp', (0.0,))[-1] / modes['mode_pelvis_turn'][0]}
    if 'limb-lift' in sweep:
        targets['mode_limb_lift'] = sweep['limb-lift']
    palmed = weighs is not None and weighs[ortho.PALMAR] > 0.0
    targets.update({'mode_arm_reach': seated.braced(spine) * palmed / modes['mode_arm_reach'][0],
                    'mode_arm_fold': 0.2 * palmed,
                    'mode_arm_swing': 0.0} if weighs is not None else {})
    sag = (0.0, modes['mode_sagittal_shift'][2]) if supine else sprung('mode_sagittal_shift', 1.0, 1.0, modes)
    springs = {'mode_sagittal_shift': sag,
               'mode_pitch_attitude': sprung('mode_pitch_attitude', 1.0, 1.0, modes),
               'mode_pelvis_turn': sprung('mode_pelvis_turn', 1.0, 1.0, modes),
               'mode_limb_lift': sprung('mode_limb_lift', 1.0, 1.0, modes),
               **({'mode_axial_compression': (0.0, 0.0)} if weighs is not None and weighs[BUSES[':scapular-dorsal']] >= 0.5 else {}),
               **({'mode_arm_reach': sprung('mode_arm_reach', wrench.W_STILL / wrench.W_POSTURE if palmed else 1.0, 1.0, modes),
                   'mode_arm_fold': sprung('mode_arm_fold', 1.0, 1.0, modes)}
                  if weighs is not None else {})}
    chain = p.get('contact-chain', ('recline',))
    what = str(chain[-1]) if len(chain) > 1 else 'recline'
    return [Phase('recline', what, p['duration-ms'], p['duration-ms'], targets, springs, None, None, weighs=weighs)]


def _sought(p, then, at, modes, body):
    """A seek onto an auxiliary manifold: the kernel's carpal seat boost (its `splice-auxiliary-manifold` on her
    carpal heels) on a seat over her shins (`seated.POPLITEAL`), reached by its arm sweep's seek
    (`drive-appendage-admittance`): her fists lifted a fist's height over it under her shoulders, set on it pressed as her
    hips are (`impedance.PROP_ON`) at the kernel's seek (SEEK_M_S), told the weight law's share as they come
    down - what bore her the rest as it was split -, held where they bear once each does (`ortho.PALMED`). On
    the lodge's 0.47 m chair both bore, 9.7 cm onto it, 1 mm off her model: swung back from her descent's
    counterweight one met the seat's front face 3 cm under its top, and told nothing as they came down the
    wait for them never ended. A seat under her shins: none. Refused by name where she cannot: her legs
    reaching no floor from it (`seated.reaches`), her fists no room on the seat she sees where she sits -
    its width less its middle aside of hers (`furniture.seen`; landed 0.151 m aside, a fist met none and the
    wait never ended) -, or their push past her arm drives' peak (PEAKS)."""
    if tuple(p['anatomical-frames']) != CARPAL or p['weight-law'].name != 'wrench-proportional':
        raise MachineError('refused: a manifold on %s, %s; her hands bear on %s by a wrench-proportional share'
                           % (' '.join(p['anatomical-frames']), p['weight-law'].name, ' '.join(CARPAL)))
    if at['seat'] is None or at['seat'] <= seated.POPLITEAL:
        return []
    share, trunk = p['weight-law'].args['max-share'], at.get('trunk', 0.0)
    pressed = impedance.PROP_ON * body['weight'] / modes['mode_axial_compression'][1]
    _s, _e, aside, levers = seated.placed(seated.ON - pressed, trunk)
    room = math.inf if at['across'] is None else 0.5 * at['across'] - abs(at['aside']) - aside
    torques = tuple(0.5 * share * body['weight'] * lever for lever in levers)
    faults = [why for why, bad in (
        ('her legs reach no floor from her hips on a seat %.2f m up' % at['seat'], not seated.reaches(at['seat'])),
        ('a seat %.2f m wide, its middle %.2f m aside of hers, leaves her fists %.2f m aside no room'
         % (at['across'] or 0.0, at['aside'], aside), room < 0.0),
        ('her fists bearing %.2f of her ask her shoulder %.1f, her elbow %.1f N m past their peak %.1f, %.1f'
         % ((share,) + torques + PEAKS), any(t > k for t, k in zip(torques, PEAKS)))) if bad]
    if faults:
        raise MachineError('refused: %s' % '; '.join(faults))
    deg = dict(figure.stand(), spine=math.degrees(trunk))
    ahead = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0][2]
    on, over = seated.ON, 2.0 * figure.FIST_R
    reach = max(math.dist(figure.fist(dict(deg, left_shoulder=math.degrees(s)))[1:], (on + over, ahead))
                for s in (at.get('swung', 0.0), -at.get('swung', 0.0)))
    lift, down = 1e3 * reach / SEEK_M_S, 1e3 * (over + pressed) / SEEK_M_S

    def arms(up):
        shoulder, elbow = seated.fisted(up, ahead, trunk)
        return {'mode_arm_swing': 0.0, 'mode_arm_reach': shoulder / modes['mode_arm_reach'][0],
                'mode_arm_fold': (elbow - math.radians(deg['left_elbow'])) / modes['mode_arm_fold'][0]}
    pan = tuple(share if k == ortho.PALMAR else w * (1.0 - share) for k, w in enumerate(at['weighs']))
    at['palmed'] = share
    rest = {m: sprung(m, 1.0, 1.0, modes) for m in ARMS}
    held = {m: sprung(m, wrench.W_STILL / wrench.W_POSTURE, 1.0, modes) for m in ARMS}
    return [Phase('contact', 'carpal-reach', lift, lift, arms(on + over), rest, None, None),
            Phase('contact', 'carpal-seek', down, down, arms(on - pressed), rest, None, None, weighs=pan),
            Phase('contact', p['patch-id'], GRAIN_MS, GRAIN_MS, {}, held, None, ('palmar',), weighs=pan)]


def _pendulum(p, then, at, modes, body):
    """A coupled pendulum: her trunk down onto her hip while her legs swing up onto the mattress as
    its counterweight, her pelvis turned to lie along the bed - her hip bearing her (the wavefront's
    first, `BUSES`), her ZMP let go, her legs at the furniture's gain -, one phase over its time: her
    pelvis's list toward the fulcrum's side and its turn to the bed's axis, her hips as high as lying
    on her side puts them (LAIN over the fulcrum's touchdown), her trunk straight along it. A slew past
    her modes' reach (`ortho.REACH`) is refused: the text's 1.57 rad rolls her base onto her hip,
    where her list mode tilts her pelvis against her standing legs - at its reach, 0.9 rad, her soles
    bore all of her beside the bed, her neck swung 1 363 deg/s."""
    lever = p['coupled-lever-dynamics']
    side = -1.0 if str(p['fulcrum-node']).endswith('-r') else 1.0
    at['knee'] = seated.knee(1e-3 * p['fulcrum-touchdown-z-mm'] + LAIN, at['ankle'])
    turned = {'mode_pelvis_list': lever['torso-list-slew-rad'][-1], 'mode_pelvis_turn': lever['pelvic-yaw-slew-rad'][-1]}
    for m, v in turned.items():
        if abs(v) > ortho.REACH * modes[m][0]:
            raise MachineError('refused: a pendulum of %g rad on %s; her modes reach %g (ortho.REACH)'
                               % (v, m, ortho.REACH * modes[m][0]))
    targets = {m: side * v / modes[m][0] for m, v in turned.items()}
    targets.update({
        'mode_axial_compression': seated.axial(at['knee'], modes, body), 'mode_pitch_attitude': 0.0,
        'mode_sagittal_shift': 0.0})
    gain = 10.0 ** (at.get('legs-db', 0.0) / 20.0)
    springs = {m: sprung(m, gain, 1.0, modes) for node in LEGS for m in NODES[node]}
    springs.update({m: sprung(m, 1.0, 1.0, modes) for m in targets})
    first = str(p['contact-wavefront'][0])
    return [Phase('contact', 'pendulum', p['total-duration-ms'], p['total-duration-ms'], targets, springs, NONE, None,
                  shape=_curve(p), weighs=_weighs(((WAVEFRONT[first], 1.0),)))]


def _latch(p, then, at, modes, body):
    """A latch: her vector where she is read at its time, the reference her gains come back on with
    no error (`ortho.LATCH`, `waits.latched_back`)."""
    return [Phase('wake', 'latch', GRAIN_MS, GRAIN_MS, {}, {}, None, None, True)]


def _compliance(p, then, at, modes, body):
    """A furniture's compliance: its legs' gain after the contact, kept where it says none (a mattress's
    notch); its spring and notch the world's."""
    at['legs-db'] = p.get('post-contact-limb-gain-db', at.get('legs-db', 0.0))
    return []


#: Each act: (its params, the acts it releases, her running place, her modes, her body) -> phases.
ACTS = {'trigger-one-shot-envelope': lambda p, then, at, modes, body: _envelope(p, at, modes, body),
        'on-contact-threshold': _contact, 'crossfade-support-manifold': _crossfade,
        'cascade-impedance-envelope': _cascade, 'cascade-actuator-gates': _gates,
        'on-manifold-equilibrium': _equilibrium, 'stage-sequential-manifold': _recline,
        'configure-contact-compliance': _compliance, 'latch-current-encoders': _latch,
        'stage-coupled-pendulum': _pendulum, 'seek-auxiliary-manifold': _sought}
