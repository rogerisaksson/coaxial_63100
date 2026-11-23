"""The robot's motion pass: what its sensors read, via its latent loop, to what its drives are told.

The world is never seen: the robot's array is `machine.senses.edge`'s.

    motion.control(g, dt)    # a pass of the loop, every `g['every']`: the robot's ask solved, its legs' stiffness
    motion.drives(g, dt, step)    # every pass: each drive's setpoint, feed and clamp, `g['cmd']`; `step` its pass's C

`g` is `sprung.laid`'s; its `sense` the edge's array, its `est` the robot's estimate
(`machine.senses.estimate`).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.law import barrier, brace, composer, impedance, latent, melt, modal, schema, se2, somatic, stride, supervisor, waits
from machine.body import bemf, capsules, figure, servos
from machine.senses import estimate, field, grip, inekf, observer, situation, touch
from machine.moves import cascade, emancipation, leap, seated
from machine.stack import chain
from machine.senses import edge
from machine.texts import ortho
from machine.eyes import vision
from machine.way import cocoon, wayfinder
from machine.law.upper import step_modal


#: The robot's arms' modes, its hold's (`ask['hold']`).
ARMED = np.array([modal.ARMS, modal.REACH, modal.ELBOWS])
#: The robot's axial stiffness in its latent state, where it goes and where it comes from; past HOP_K
#: (midpoint between resting axial stiffness and knee-lever throw stiffness, N/m) a hop's.
_AXIAL_K = tuple(block.start + latent.K.start + modal.AXIAL for block in (latent.TO, latent.FROM))
_L_CR = 0.5 * modal.LEG_M * libm.sin(math.radians(abs(figure.SQUAT['left_ankle'])))
HOP_K = round(0.5 * (0.625 * modal.LIMB_KD[0] * figure.mass() + servos.SERVO['knee'][1] / (_L_CR * _L_CR)), -2)
#: The robot's surety its brace takes it over below (`bw`): a stumble.
BRACE_SURETY = 0.58
#: A joint none of what bears the robot needs, of its clamp: the suite's passive cone (-32 dB). The robot's
#: ankles and knees are its soles' (`ortho.MANIFOLDS`), the rest of its let go as it lies (dorsal) or on its
#: side (trochanter): lying back on the bed, its law held to its soles, its legs swung 483 deg/s.
RELEASED = libm.power(10.0, -32.0 / 20.0)
DISTAL = ('knee', 'ankle', 'ankle_roll')
def axial_k(st):
    """The robot's axial stiffness asked in its latent `st`, the stiffer of where it goes and where it
    comes from, N/m."""
    return max(float(st[_AXIAL_K[0]]), float(st[_AXIAL_K[1]]))


def seen(g, dt):
    """`wbc.sense`'s view of the robot as its sensors and its estimate have it, `dt` s on: both in C bit
    for bit once built (`g['estimated']`, `laid`; `g['sensed']`, `sprung.laid`)."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = g['estimated'](g['est'], body, g['sense'], dt)
    s = g['sensed'](body, qpos, qvel)
    observer.step(g['observer'], s, qpos, dt)
    shares = np.append(g['observer']['soles'] * g['unanchored'],
                       g['observer']['normal'][observer.ROWS[1:4]].sum()) / float(g['fr'][modal.WEIGHT])
    inekf.stepped(g['inekf'], body, g['sense'], shares, dt, g['inekf_points'], g['inekf_step'])
    s['hips'], s['arms'], s['elbows'], s['qpos'] = (body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']],
                                                    s['q'][limbs['elbows']], qpos.copy())
    palmar = g['palmar'] = min(float(g['told'][ortho.PALMAR]), float(g['st'][latent.TO][latent.MANIFOLD.start + ortho.PALMAR]))
    # what bears the robot and, sat, its seat's cone in C on its contacts before any is a dict
    # (`emancipation.seated`)
    s['prop'], s['contacts'], s['palms'] = g['borne'](body, s, g['sense'], float(g['fr'][modal.WEIGHT]), palmar,
                                                      g['a'][modal.LET_GO], g)
    return s


def _step_latent(g, dt):
    """The robot's latent state stepped: `waits.woken_rows` and `latent.step` in C (`g['waits']`,
    `g['latent']`)."""
    g['waits'].woken(g['tab'], g['t'], dt)
    return g['latent'](g['st'], g['tab'], g['t'], dt, g['out'])


def surety(g, dt, ct=math.nan):
    """The robot's surety field stepped `dt` s on its arrays (`surety.field`, in C): `g['surety']` in
    [0, 1], `g['surety_vec']` (s_ax, s_lat, d_ax, d_lat), `g['surety_probe']`."""
    cur, vec = g['surety_field'](float(g['surety']), dt, ct, float(g['t']))
    s_ax, s_lat, d_ax, d_lat, g['surety_probe'] = vec.tolist()
    g['surety'], g['surety_vec'] = cur, (s_ax, s_lat, d_ax, d_lat)
    return cur


def _chained(g, _s, _ask):
    """Whether the robot's closed chains held it at its last pass: its rise's gate works then alone."""
    return g.get('chained', 0)


def _loose(_g, _s, _ask):
    """The robot's rise's gate at rest: no chain of its own closed."""
    return 0.0


@composer.capability(_chained, _loose)
def gated(g, s, ask):
    """The robot's rise's gate (`chain.outside`) on its state `s` and its `ask`, a part composed by its
    chains."""
    return chain.outside(s, ask, g['loads'], float(g['fr'][modal.OMEGA]), float(g['fr'][modal.WEIGHT]), g['told'])


def control(g, dt):
    """A pass of the robot's latent loop on what it senses: `g['got']` solved, `g['leg_kp']` its legs'
    stiffness by their stance."""
    body, ev, y_in = g['body'], g['every'], g['sense']
    g['t_law'] = g['t']
    s = g['s'] = seen(g, dt * ev)
    g['grip'] = g['felt'](g['grip'], g['grip_seen'], g['grip_settled'], body.d.xmat[g['grip_at']].reshape(2, 2, 3, 3),
                          y_in[edge.TOUCH].reshape(-1, 3),
                          g['est']['slip'], float(g['fr'][modal.WEIGHT]), dt * ev, g['grip_prior'])
    g['loads'] = [float(v) for v in y_in[edge.LOAD]]
    y = g['modal_sensed'](s, g['loads'], g['y'])
    y[modal.CELLS] = y_in[edge.CELLS]
    y[modal.GRIP] = g['held'] = g['outside'](g['grip'], g['situations']['grips'])
    g['into'] = g['read'](g['situations'], g['readings'](
        y, g['fr'], g['a'], g['grip'], y_in[edge.TOUCH].reshape(-1, 3), float(g['fr'][modal.WEIGHT])))
    waits.backs(g, dt * ev)
    out = _step_latent(g, dt * ev)
    # the robot's legs released as told and as what else bears its allows (`emancipation`): weighed, its hands told,
    # unfelt, kept them standing
    told = g['told'] = out[latent.MANIFOLDED].copy()
    weighed = g['weighed_w'] = g['weighed'](g['observer'], told, float(g['fr'][modal.WEIGHT]), dt * ev)
    out[latent.HELD] *= weighed[0]
    melt.step(g, out, told, weighed, dt * ev)
    emancipation.step(g, s, out, told, dt * ev)
    unfelt, lean = float(weighed[0] - observer.said(told)[0]), float(told[ortho.FURNITURE].sum())
    g['seek'](g['field'], field.due(g['tab'], g['t'], dt * ev), y, lean, float(g['st'][latent.TO][latent.Z][modal.AXIAL]), dt * ev, out)
    se2.step_physical_twist(g, dt * ev, out)
    sits = g['situations']
    g['gripped'](out, g['held'], g['a'][modal.STANCE], sits['caps'], g['into'], sits['floors'], sits['period'], g['eased'], dt * ev)
    touch.step(g, out, dt * ev)
    dq, imp = step_modal(g, s, y, y_in, out)
    w_back = float(g.get('_w_back', 0.0))
    imp[modal.HEADED][1] += w_back * (float(g.get('_backing_yaw', 0.0)) - imp[modal.HEADED][1])
    vision.joined(g)  # the robot's look's things, asked before this pass (`sprung.passed`): the gauge reads them
    gauge = wayfinder.gauged(g, s)
    imp[modal.HEADED][1] += gauge
    sr = surety(g, dt * ev)
    bw = libm.sq(max(0.0, 1.0 - libm.quart(sr / BRACE_SURETY)))
    sw = 1.0 - bw
    _, _, d_ax, d_lat = g['surety_vec']
    w_hop = float((axial_k(g['st']) > HOP_K) & (g['st'][latent.TO][latent.AMP][modal.DRIVE] < 0.1))
    q_post = g['sured'](imp, g['posture'], dq, g['brace'], sw, bw, d_ax, d_lat)
    # the robot's legs as what bears it holds them, as far as it is sure of it: toppling, the
    # tuck's (`brace`)
    posed = g['poses'](imp, y, g['fr'], q_post, g['bend'], g['arms'], g['reaches'], g['folds'], body.knees,
                       weighed * sw, g.get('_twist_clear', 0.0) * sw)
    ask = g['ask'] = modal.given(imp, y, posed, body.knees, g['bend'], g['fr'], g['pitched'])
    ask['let_cap'], ask['sat'] = g['let_cap'], max(g['sat'], float(g['a'][modal.BORNE]) * float(g['a'][modal.LET_GO].max()))
    ask['com_acc'] = emancipation.settled(emancipation.counterweighted(ask['com_acc'], g), g, s)
    emancipation.opened(g, s, ask, dt * ev)
    somatic.asked(g, s, out, told, ask)
    bears = np.array(g['loads']) >= leap.TOUCH_N
    isch, pal = emancipation.SEAT_ROWS[0], latent.MANIFOLD.start + ortho.PALMAR
    pelvis_up = sum(float(c['f'][1]) * float(int(c.get('body', -1)) == body.pelvis) for c in s.get('contacts', ()))
    isch_w = min(1.0, float(told[isch]) / max(0.5 * float(told.sum()), observer.TINY))
    borne_w = min(1.0, pelvis_up / max(impedance.PROP_ON * float(g['fr'][modal.WEIGHT]), observer.TINY))
    pal_due = max(float(told[ortho.PALMAR]), float(g['st'][latent.TO][pal]), float(g['tab'][:, pal].max(initial=0.0)))
    upright = float((~bool(bears.all())) & (pal_due <= 0.0) & (float(ask.get('reclined', 0.0)) <= 0.5)
                    & (float(g['st'][latent.TO][latent.ZMP]) < 0.5) & (abs(float(out[latent.SLID])) <= 1e-6))
    ask['sat'] = max(ask['sat'], isch_w * borne_w * upright)
    g['observer']['slide'][:] = ask['slide']
    held, rest = emancipation.held(g, s, bears, unfelt)
    g['support'] = held
    ask['com_acc'] = barrier.barred(ask['com_acc'], s['com'], s['vcom'], float(g['fr'][modal.OMEGA]), held, rest, g['ringed'])
    barrier.ahead(g['ringed'], g['ringed_ahead'])
    ask['posture'][g['head_idx']] -= gauge
    ask['posture'], g['schema'] = schema.shielded(body, g['clear'](g['schema_body'], s, g['schema_at']), s, ask,
                                                  float(y_in[edge.LINK][0]), g['shield'])
    ask['mu'] = g['sloped'](g['grip'], g['situations']['grips'])
    ask['hold'] = max(1.0, float((g['out'][latent.KS][ARMED] / g['rest'][latent.K][ARMED]).max()))
    w_gait = min(1.0, max(0.0, (0.05 - float(g['st'][latent.NOW][latent.GAIN])) / 0.05))
    w_drive = min(1.0, abs(float(g['st'][latent.TO][latent.AMP][modal.DRIVE])) / stride.STIRS)
    foot = min(1.0, max(0.0, float(g['out'][latent.HELD]) - 1.0))
    # a sole up while borne is no stride: counted one, lying back the robot's neck let go and flexed at 1034
    # deg/s
    rest = (1.0 - float(g['a'][modal.STRIDES]) * (1.0 - float(g['a'][modal.BORNE]))) * w_gait
    w_base = sw * max(w_hop, rest)
    # the robot's neck holds pitch through a brake and its blend into stand (`NOW[GAIN]`), and when seated
    g_on = max(float(g['st'][latent.TO][latent.GAIN]), float(g['st'][latent.NOW][latent.GAIN]))
    brake = foot * (1.0 - w_drive) * g_on
    w_neck = sw * max(w_hop, max(rest, brake))
    w_up = np.array([w_neck, w_base, w_hop * w_base, w_hop * w_base])
    M = g['braced'].scaled(s['M'], w_up)
    lo, hi = g['bounds'](s['v'][body.act], float(y_in[edge.LINK][0]))
    # the robot's rise's gate keeps closed chains only as they leave it, never taking it into them
    ask['outside'] = gated(g, s, ask)
    ask['lean'] = chain.rise()[0] * ask['outside']
    chained = g['chained'] = chain.chained(ask)
    # thighs join the cone only while seated upright with hands bearing (soles, ischia, palms)
    sat = any(int(c.get('body', -1)) == body.pelvis for c in s.get('contacts', ()))
    told = (g.get('palmar', 0.0) > 0.0) & (float(ask.get('reclined', 0.0)) <= 0.5)
    legs = body.legs[:int(sat & told) * len(body.legs)]
    # the step's inputs kept for the robot's introspection (`borne.named`, chain_trace's replay), its answer
    # `got`
    g['solve_in'] = (dict(s, M=M, contacts=chain.bearing(s, g['loads'], ask['stance'], chained, g['emancipation']['iota'],
                                                          legs)),
                     dict(ask, wep=bool((g['left'] > 0.0) & (sr > 0.25)), derate=g['_derate'], lo=lo, hi=hi))
    g['got'] = g['solves'][chained](body, *g['solve_in'])
    got = g['got']
    g['seat_f'][:] = 0.0
    for c, f in zip(s.get('contacts', ()), got.get('borne', ())):
        g['seat_f'][c.get('corner', 0)] += float(f[1]) * float('corner' in c)
    g['jeff'], g['leg_kp'] = g['braced'].blend(s['M'], M, s['q'], s['v'], s['h'], ask['posture'], got['qacc'], got['tau'],
                                               got['over'], sw, bw, d_ax, d_lat, w_hop, w_up, np.array(ask['stance'], float))
    _told(g)
    g['released'](g['release'], g['weighed_w'], g['a'], g['distal'], g['distal_side'])


def released(release, w, a, distal, side):
    """`release` (in place), each joint's clamp's share: whole as far as what bears the robot needs it - a `distal`
    joint the robot's soles' (`w`, what bears it weighed), the rest also as it lies neither back nor
    on its side -, a let-go leg's (`a`'s, by its `side`) kept whole; numpy's alone, so compiled
    (`g['released']`)."""
    kept = np.maximum(w[0], a[modal.LET_GO][side])
    release[:] = 1.0 - (1.0 - RELEASED) * (1.0 - (distal * kept + (1.0 - distal) * np.maximum(1.0 - w[2] - w[3], kept)))


def _told(g):
    """The solve's answer and the robot's drives' inertia told its drives' step (`g['drives']`'s
    arrays, `laid`): what it reads every pass till the robot's law's next."""
    got, kept = g['got'], g['drives'][1]
    kept['qacc'][:] = got['qacc']
    kept['tau'][:] = got['tau']
    kept['jeff'][:] = g['jeff']
    kept['over'][:] = got['over']


def laid(g):
    """The drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`) pointing at `g`'s arrays once
    (`g['cmd']`), with its latent step, upper-body coupling, ask, surety, touch, twist, table waits, brace and
    borne read in C bit for bit (`tools.cores.motion`, `step`, `surety`, `touch`, `twist`, `wait`, `brace`,
    `borne`), and the robot's estimate, grip, situation, poses, EKF, barrier, field, weighed shares,
    clearances, wayfinder gap and cocoon's rooms compiled from numpy (`tools.cores.bodied`)."""
    from machine.body.figure import JOINTS
    from tools.cores import bodied, motion as c_motion, surety as c_surety, touch as c_touch, twist as c_twist
    from tools.cores import brace as c_brace, wait as c_wait, wbc as cores
    g['estimated'] = bodied.on_body(estimate.step, 'estimate', (0, 2), body=1)
    g['felt'] = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3, 4, 5, 6, 8))
    g['weighed'] = bodied.on_args(observer.weighed, 'weighed', (0, 1, 2, 3))
    g['clear'] = bodied.on_body(capsules.felt, 'clear', (1,))
    g['gap'] = bodied.on_shapes(wayfinder.gap, 'gap', (0, 1, 2, 3, 4, 5))
    g['rooms'] = bodied.on_shapes(cocoon.rooms_of, 'rooms', (0, 1, 2, 3, 4, 5))     # the cocoon's room along a way
    g['outside'] = bodied.on_args(grip.outside, 'outside', (0, 1))
    g['sloped'] = bodied.on_args(grip.sloped, 'sloped', (0, 1))     # each sole's slope (`ask['mu']`): numpy's 24 us
    g['readings'] = bodied.on_args(situation.readings, 'readings', (0, 1, 2, 3, 4, 5))
    g['read'] = bodied.on_args(situation.read, 'read', (0, 1))
    g['modes'] = bodied.on_args(modal.read, 'modes', (0, 1, 2))     # the robot's modes said to its page (`sprung.state`)
    g['pitched'] = bodied.on_args(modal.pitched, 'pitched', (0, 1))     # the robot's trunk reclined (`modal.given`)
    g['capped'] = bodied.on_args(somatic.capped, 'capped', tuple(range(11)))     # the robot's ask's caps (`somatic.asked`)
    g['told_on'] = bodied.on_args(emancipation.on_floor, 'told_on', tuple(range(6)))     # `emancipation.told_on`
    g['settling'] = bodied.on_args(emancipation.settling, 'settling', tuple(range(10)))     # `emancipation.settled`
    g['bounds'] = bodied.on_args(bemf.bounds, 'bounds', (0, 1))     # the robot's drives' torque bounds at their speeds
    g['gripped'] = bodied.on_args(situation.gripped, 'gripped', tuple(range(9)))
    g['sured'] = bodied.on_args(brace.sured, 'sured', tuple(range(8)))     # the robot's ask and posture by its surety
    g['released'] = bodied.on_args(released, 'released', (0, 1, 2, 3))     # the robot's joints' clamps' shares
    g['poses'] = bodied.on_args(modal.poses, 'poses', (0, 1, 2, 3, 4, 5, 6, 7, 9, 10))
    g['ringed'] = bodied.on_shapes(barrier.ringed, 'ringed', (0, 1, 2, 3, 4, 5))
    g['ringed_ahead'] = list(barrier.HULLS)
    g['modal_sensed'] = bodied.on_args(modal.sensed, 'modal_sensed', (0, 1, 2))     # the robot's sensing's arrays walked in C
    g['inekf_points'] = bodied.on_body(inekf.points, 'inekf_points', (2, 3))
    g['inekf_step'] = bodied.on_args(inekf.step, 'inekf', (0, 2, 3, 4))
    g['seek'] = bodied.on_args(field.seek, 'seek', (0, 1, 2, 3, 4, 6))
    g.update(cascade.compiled())
    g['kept'] = bodied.on_args(seated.kept, 'kept', (0, 1, 2, 3, 4, 5))
    g['shield'] = bodied.on_args(schema.shield, 'shield', (0, 1, 2, 3, 4, 5, 6))
    k, ask = c_motion.laid()
    g['impedance'] = functools.partial(c_motion.asked, k, ask)
    g['surety_field'] = c_surety.laid(k, g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'])
    laid_ = c_motion.latent_laid()
    g['latent'], g['couple'] = functools.partial(c_motion.stepped, laid_), functools.partial(c_motion.coupled, laid_)
    body, joints = g['body'], g['joints']
    from tools.cores import borne as c_borne
    g['borne'] = c_borne.laid(body)
    g['em_supports'] = c_borne.supported(g['borne'])     # `emancipation.supports` on its contacts, in C
    g['leaned'] = c_borne.leaned(g['borne'])     # `somatic.leaned` on its contacts, in C
    g['_skin_ids'] = np.array([body.m.body(n).id for n in edge.SKIN])
    g['schema_at'], g['schema_body'] = capsules.ids(body.m), capsules.Own(body)
    g['_tips'] = np.array(touch.TIPS)
    g['touched_step'] = c_touch.laid(k, g['sense'], body.d.xmat, body.d.xpos, g['_skin_ids'], g['_tips'], g['y'],
                                     g['fr'], g['a'], g['st'], g['out'], g['rest'], g['touched'])
    g['twist_step'] = c_twist.laid(k, g['sense'], g['y'], g['fr'], g['a'], g['st'], g['out'], g['posture'],
                                   g['twisted'], g['words'], g['waist_idx'], g['head_idx'])
    g['waits'] = c_wait.laid(k, g['a'], g['out'], g['y'], g['fr'], g['st'])
    g['_cv_sh'] = list(g['cervical']) + list(g['limbs']['shoulders'])
    g['_acv_sh'] = body.act[g['_cv_sh']]
    g['_derate'] = g['wep_lim'] / np.maximum(1.0, body.wep)
    names = [JOINTS[j] for j in joints]
    g['brace'] = np.array([[d.get(n, 0.0) for n in names] for d in (brace.TUCK, brace.AX, brace.LAT)])
    g['braced'] = c_brace.laid(body, g['brace'], g['_cv_sh'], g['_acv_sh'], g['legs'], g['free_kp'])
    n = len(joints)
    kept = {'joints': np.ascontiguousarray(joints, dtype=np.int32),
            'act': np.ascontiguousarray(body.act, dtype=np.int32), 'sb': np.zeros(n), 'feed': np.zeros(n),
            'limit': np.zeros(n), 'clamp_j': np.ascontiguousarray(g['clamp'][joints], dtype=np.float64),
            'qacc': np.zeros(body.n, dtype=np.float64), 'tau': np.zeros(n, dtype=np.float64),
            'jeff': np.ascontiguousarray(g['jeff'], dtype=np.float64), 'over': np.zeros(n, dtype=np.bool_),
            'release': np.ones(n)}
    g['release'], g['distal'] = kept['release'], np.array([float(nm.endswith(DISTAL)) for nm in names])
    g['distal_side'] = np.array([int(nm.startswith('right_')) for nm in names])
    g['got'] = {'qacc': kept['qacc'].copy(), 'tau': kept['tau'].copy(), 'over': kept['over'].copy()}
    k, bus, _arrays = g['bus'] = supervisor.bus(names, g['rate_hz'])
    stop_width, _stiff, _damp, stop_reach = body.buffers[1]
    p = cores.Drives(n=n, q_at=edge.Q.start, qd_at=edge.QD.start, qm_at=edge.QM.start, knee_nm=se2.KNEE_NM,
                     knee_soft=se2.KNEE_SOFT, leak_s=se2.LEAK_S, wep_hold_s=se2.WEP_HOLD_S,
                     bus_k=ctypes.addressof(k), bus=ctypes.addressof(bus), posture_kd=supervisor.POSTURE_KD,
                     stop_width=stop_width, stop_reach=stop_reach)
    p.knee[0], p.knee[1] = body.knees
    p.cervical[0], p.cervical[1] = g['cervical']
    for name, arr in (('joints', kept['joints']), ('act', kept['act'])):
        setattr(p, name, cores.pointer(arr, np.int32))
    for name, arr in (('y', g['sense']), ('clamp', body.clamp), ('wep', body.wep), ('wep_lim', g['wep_lim']),
                      ('clamp_j', kept['clamp_j']), ('q_lo', g['q_lo']), ('q_hi', g['q_hi']), ('eso', g['eso']),
                      ('ref_q', g['ref_q']), ('ref_v', g['ref_v']), ('hold', g['hold']), ('sb', kept['sb']),
                      ('feed', kept['feed']), ('limit', kept['limit']), ('qacc', kept['qacc']),
                      ('tau', kept['tau']), ('jeff', kept['jeff']), ('release', kept['release'])):
        setattr(p, name, cores.pointer(arr))
    p.over = cores.address(kept['over'])
    g['cmd'] = {'target': g['ref_q'], 'feed': kept['feed'], 'limit': kept['limit'].copy()}
    g['drives'] = (p, kept, cores.library().wbh_pass_drives)
    from tools.cores import tick
    g['tick'] = tick.laid(g, HOP_K, se2.WEP_S, se2.WEP_FILL)


def drives(g, dt, step):
    """Each driven joint's setpoint (`g['ref_q']`), feed and clamp (`g['cmd']`) this pass, its
    observer stepped on its motor's encoder - the robot's surety as it tilts or slips, a hop, which WEP
    limit is its own - in one C call, `step`'s (its address and call on the tick's arguments: `g['tick'][:2]`
    the tick alone, `tools.cores.tick`; the robot's pass's, `tools.cores.beat`, its law's joints told the
    robot's boards after it, the robot's felt before it where it is both), how far the robot rests limp told it
    (`se2.limp`); `drives_twin` its python."""
    _at, _tick, t, vec = g['tick']
    at, tick = step
    t.surety, t.left = g['surety'], g['left']
    tick(at, dt, g['t'], g['wo'], se2.limp(g))
    s_ax, s_lat, d_ax, d_lat, g['surety_probe'] = vec.tolist()
    g['surety'], g['surety_vec'], g['left'] = t.surety, (s_ax, s_lat, d_ax, d_lat), t.left


def drives_twin(g, dt):
    """`drives` in python: the step in C (`laid`), told here what of the robot it reads whole."""
    p, _kept, step = g['drives']
    held, wo = float(g['hold'].any()), g['wo']
    ys = surety(g, dt)
    drv = 1.0 - libm.cube(1.0 - ys)
    wo_s = wo * drv
    striding = float(g['a'][modal.STRIDES]) * min(max(float(g['out'][latent.HELD]), 0.0), 1.0)
    st0, st1 = float(g['a'][modal.STANCE][0]), float(g['a'][modal.STANCE][1])
    stood0 = (st0 * (1.0 - st1)) * striding
    stood1 = (st1 * (1.0 - st0)) * striding
    in_hop = int(axial_k(g['st']) > HOP_K)
    step(ctypes.byref(p), dt, wo_s, wo_s * wo_s * wo_s, drv, stood0, stood1, in_hop, 1)
    # a setpoint within its drive's range (the manifest's): asked compression 4.0 the pattern's
    # ankle ask of -77 deg wound the joint to 215.8 of its -45..42 and felled the robot at 3.0 s.
    np.minimum(np.maximum(g['ref_q'], g['rng'][:, 0], out=g['ref_q']), g['rng'][:, 1], out=g['ref_q'])
    # the robot's clamps told as the step left them, zero where it rests limp (`se2.LIMP`), into a
    # buffer laid once: rebound to its own product a pass, the clamps froze at the first pass's and a rise
    # from the low chair fell at 31.9 s that stood
    np.multiply(_kept['limit'], 1.0 - se2.limp(g), out=g['cmd']['limit'])
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
