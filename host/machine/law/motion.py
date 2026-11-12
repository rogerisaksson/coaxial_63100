"""Her motion pass: what her sensors read, through her latent loop, to what her drives are told.

The world is never seen: her array is `machine.senses.edge`'s.

    motion.control(g, dt)    # a pass of the loop, every `g['every']`: her ask solved, her legs' stiffness
    motion.drives(g, dt, step)    # every pass: each drive's setpoint, feed and clamp, `g['cmd']`; `step` her pass's C

`g` is `sprung.laid`'s; its `sense` the edge's array, its `est` her estimate (`machine.senses.estimate`).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import barrier, brace, composer, latent, melt, modal, schema, se2, somatic, stride, supervisor, waits
from machine.body import bemf, capsules
from machine.senses import estimate, field, grip, inekf, observer, situation, touch
from machine.moves import cascade, emancipation, leap, seated
from machine.stack import chain
from machine.senses import edge
from machine.texts import ortho
from machine.eyes import vision
from machine.way import wayfinder
from machine.law.upper import step_modal


#: Her arms' modes, her hold's (`ask['hold']`).
ARMED = np.array([modal.ARMS, modal.REACH, modal.ELBOWS])
#: Her axial stiffness in her latent state, where it goes and where it comes from; past HOP_K
#: (N/m) a hop's.
_AXIAL_K = tuple(block.start + latent.K.start + modal.AXIAL for block in (latent.TO, latent.FROM))
HOP_K = 15000.0
#: Her surety her brace takes her over below (`bw`): a stumble.
BRACE_SURETY = 0.58
#: A joint none of what bears her needs, of its clamp: the suite's passive cone (-32 dB). Her ankles
#: and knees are her soles' (`ortho.MANIFOLDS`), the rest of her let go as she lies (dorsal) or on
#: her side (trochanter): lying back on the bed, her law held to her soles, her legs swung 483 deg/s.
RELEASED = 10.0 ** (-32.0 / 20.0)
DISTAL = ('knee', 'ankle', 'ankle_roll')
def axial_k(st):
    """Her axial stiffness asked in her latent `st`, the stiffer of where it goes and where it comes
    from, N/m."""
    return max(float(st[_AXIAL_K[0]]), float(st[_AXIAL_K[1]]))


def seen(g, dt):
    """`wbc.sense`'s view of her as her sensors and her estimate have her, `dt` s on: both in C bit
    for bit once built (`g['estimated']`, `laid`; `g['sensed']`, `sprung.laid`)."""
    body, limbs = g['body'], g['limbs']
    qpos, qvel = g['estimated'](g['est'], body, g['sense'], dt)
    s = g['sensed'](body, qpos, qvel)
    observer.step(g['observer'], s, qpos, dt)
    inekf.stepped(g['inekf'], body, g['sense'], g['observer']['soles'] * g['unanchored'] / float(g['fr'][modal.WEIGHT]),
                  dt, g['inekf_points'], g['inekf_step'])
    s['hips'], s['arms'], s['elbows'], s['qpos'] = (body.d.xpos[limbs['hips']].copy(), s['q'][limbs['shoulders']],
                                                    s['q'][limbs['elbows']], qpos.copy())
    palmar = g['palmar'] = min(float(g['told'][ortho.PALMAR]), float(g['st'][latent.TO][latent.MANIFOLD.start + ortho.PALMAR]))
    # what bears her and, sat, her seat's cone in C on her contacts before any is a dict (`emancipation.seated`)
    s['prop'], s['contacts'], s['palms'] = g['borne'](body, s, g['sense'], float(g['fr'][modal.WEIGHT]), palmar,
                                                      g['a'][modal.LET_GO], g)
    return s


def gripped(out, held, stance, caps, into):
    """`out` in the situations she is in (`into` (S,), `situation.read`): each mode's sway and its
    sine scaled down to each's caps (S, N; `situation.text`) as far as she is in it; her drive's
    further to her going no faster than sqrt(2 STRIDE_M grip g) on her standing soles' grip
    (`held`) - a stride she can stop within: 0.77 m/s on ice (0.1), 1.9 at her law's own 0.6.
    Asked 0.69 on ice at 0.02 she ran on and fell at 3.0-3.4 s."""
    grip_now = ((stance + 1e-9) / (stance.sum() + 2e-9)) @ held
    sway = np.abs(out[latent.SWAY])
    under = into[:, None]
    capped = np.where(np.isnan(caps), 1.0, np.minimum(1.0, np.nan_to_num(caps) / np.maximum(sway, 1e-9)))
    k = np.min(1.0 - under + under * capped, axis=0, initial=1.0)
    turns = out[latent.TURNS]
    cruise = 2.0 / math.pi * stride.STRIDE_M * (sway[modal.DRIVE] * k[modal.DRIVE]) * np.sin(stride.swings(sway[modal.DRIVE] * k[modal.DRIVE], turns, 2.0 / math.pi * stride.STRIDE_M * (sway[modal.DRIVE] * k[modal.DRIVE]) * turns)) * turns
    k[modal.DRIVE] *= np.minimum(1.0, np.sqrt(2.0 * stride.STRIDE_M * grip_now * modal.G) / np.maximum(cruise, 1e-9))
    sine = out[latent.SWAY] * np.sin(out[latent.PSI])
    out[latent.REF] += (k - 1.0) * sine
    out[latent.RATE] += (k - 1.0) * out[latent.SWAY] * turns * np.cos(out[latent.PSI])
    out[latent.ACC] -= (k - 1.0) * sine * turns * turns
    out[latent.SWAY] *= k


def _step_latent(g, dt):
    """Her latent state stepped: `waits.woken_rows` and `latent.step` in C (`g['waits']`, `g['latent']`)."""
    g['waits'].woken(g['tab'], g['t'], dt)
    return g['latent'](g['st'], g['tab'], g['t'], dt, g['out'])


def surety(g, dt, ct=math.nan):
    """Her surety field stepped `dt` s on her arrays (`surety.field`, in C): `g['surety']` in
    [0, 1], `g['surety_vec']` (s_ax, s_lat, d_ax, d_lat), `g['surety_probe']`."""
    cur, vec = g['surety_field'](float(g['surety']), dt, ct, float(g['t']))
    s_ax, s_lat, d_ax, d_lat, g['surety_probe'] = vec.tolist()
    g['surety'], g['surety_vec'] = cur, (s_ax, s_lat, d_ax, d_lat)
    return cur


def _chained(g, _s, _ask):
    """Whether her closed chains held her at her last pass: her rise's gate works then alone."""
    return g.get('chained', 0)


def _loose(_g, _s, _ask):
    """Her rise's gate at rest: no chain of hers closed."""
    return 0.0


@composer.capability(_chained, _loose)
def gated(g, s, ask):
    """Her rise's gate (`chain.outside`) on her state `s` and her `ask`, a part composed by her chains."""
    return chain.outside(s, ask, g['loads'], float(g['fr'][modal.OMEGA]), float(g['fr'][modal.WEIGHT]), g['told'])


def control(g, dt):
    """A pass of her latent loop on what she senses: `g['got']` solved, `g['leg_kp']` her legs'
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
    # her legs released as told and as what else bears her allows (`emancipation`): weighed, her hands told, unfelt,
    # kept them standing
    told = g['told'] = out[latent.MANIFOLDED].copy()
    weighed = g['weighed_w'] = g['weighed'](g['observer'], told, float(g['fr'][modal.WEIGHT]), dt * ev)
    out[latent.HELD] *= weighed[0]
    melt.step(g, out, told, weighed, dt * ev)
    emancipation.step(g, s, out, told, dt * ev)
    unfelt, lean = float(weighed[0] - observer.said(told)[0]), float(told[ortho.FURNITURE].sum())
    g['seek'](g['field'], field.due(g['tab'], g['t'], dt * ev), y, lean, float(g['st'][latent.TO][latent.Z][modal.AXIAL]), dt * ev, out)
    se2.step_physical_twist(g, dt * ev, out)
    g['gripped'](out, g['held'], g['a'][modal.STANCE], g['situations']['caps'], g['into'])
    touch.step(g, out, dt * ev)
    dq, imp = step_modal(g, s, y, y_in, out)
    w_back = float(g.get('_w_back', 0.0))
    imp[modal.HEADED][1] += w_back * (float(g.get('_backing_yaw', 0.0)) - imp[modal.HEADED][1])
    vision.joined(g)  # her look's things, asked before this pass (`sprung.passed`): the gauge reads them
    gauge = wayfinder.gauged(g, s)
    imp[modal.HEADED][1] += gauge
    sr = surety(g, dt * ev)
    bw = max(0.0, 1.0 - (sr / BRACE_SURETY) ** 4) ** 2
    sw = 1.0 - bw
    _, _, d_ax, d_lat = g['surety_vec']
    w_hop = float((axial_k(g['st']) > HOP_K) & (g['st'][latent.TO][latent.AMP][modal.DRIVE] < 0.1))
    q_post = g['sured'](imp, g['posture'], dq, g['brace'], sw, bw, d_ax, d_lat)
    # her legs as what bears her, felt, holds them, as far as she is sure of it: toppling, the tuck's (`brace`)
    posed = g['poses'](imp, y, g['fr'], q_post, g['bend'], g['arms'], g['reaches'], g['folds'], body.knees,
                       weighed * sw, g.get('_twist_clear', 0.0) * sw)
    ask = g['ask'] = modal.given(imp, y, posed, body.knees, g['bend'], g['fr'], g['pitched'])
    ask['let_cap'], ask['sat'] = g['let_cap'], max(g['sat'], float(g['a'][modal.BORNE]) * float(g['a'][modal.LET_GO].max()))
    ask['com_acc'] = emancipation.settled(emancipation.counterweighted(ask['com_acc'], g), g, s)
    emancipation.opened(g, s, ask, dt * ev)
    somatic.asked(g, s, out, told, ask)
    g['observer']['slide'][:] = ask['slide']
    bears = np.array(g['loads']) >= leap.TOUCH_N
    held, rest = emancipation.held(g, s, bears, unfelt)
    g['support'] = held
    ask['com_acc'] = barrier.barred(ask['com_acc'], s['com'], s['vcom'], float(g['fr'][modal.OMEGA]), held, rest, g['ringed'])
    barrier.ahead(g['ringed'], g['ringed_ahead'])
    ask['posture'][g['head_idx']] -= gauge
    ask['posture'], g['schema'] = schema.shielded(body, g['clear'](g['schema_body'], s, g['schema_at']), s, ask,
                                                  float(y_in[edge.LINK][0]), g['shield'])
    ask['mu'] = g['held']
    ask['hold'] = max(1.0, float((g['out'][latent.KS][ARMED] / g['rest'][latent.K][ARMED]).max()))
    cv, acv = g['_cv_sh'], g['_acv_sh']
    w_gait = min(1.0, max(0.0, (0.05 - float(g['st'][latent.NOW][latent.GAIN])) / 0.05))
    w_drive = min(1.0, abs(float(g['st'][latent.TO][latent.AMP][modal.DRIVE])) / stride.STIRS)
    foot = min(1.0, max(0.0, float(g['out'][latent.HELD]) - 1.0))
    # a sole up while what bears her bears her is no stride: counted one, lying back her neck let
    # go 0.36 s and flexed 64 deg at 1034 deg/s
    rest = (1.0 - float(g['a'][modal.STRIDES]) * (1.0 - float(g['a'][modal.BORNE]))) * w_gait
    w_base = sw * max(w_hop, rest)
    # her neck holds her head's pitch itself also while her rhythm is told to run, held to her
    # landings' footprint, but told no drive - a brake: coupled, her step's rounds threw her head
    # 101-105 deg as her last landing let her strides go; her head's yaw turns with her. Seated
    # too: let go there, a rise's lean whipped her head 71-98 deg back and asked her neck 10 times
    # its clamp, the whole ask shrunk to 0.04 and she collapsed onto the seat
    brake = foot * (1.0 - w_drive) * float(g['st'][latent.TO][latent.GAIN])
    w_neck = sw * max(w_hop, max(rest, brake))
    w_up = np.array([w_neck, w_base, w_hop * w_base, w_hop * w_base])
    M = g['braced'].scaled(s['M'], w_up)
    lo, hi = g['bounds'](s['v'][body.act], float(y_in[edge.LINK][0]))
    # her rise's gate keeps her closed chains as they leave her, never takes her into them: her chair's seat is in her
    # upright step's cone, and held in her chains from her seat she sat on through every rise
    ask['outside'] = gated(g, s, ask)
    ask['lean'] = chain.rise()[0] * ask['outside']
    chained = g['chained'] = chain.chained(ask)
    # her legs' touch on the seat is the chains' alone while she sits - her pelvis bears, not reclined - and her hands
    # are told to bear: the kernel's cone for the upright step (soles, ischia, palms). The sit tells the palms from
    # `seated` on (4.2 s), after the landing, through the hands' vault: with the thighs in the cone she sat at the seat's
    # middle leaning 16 deg back, fists in the air till the scoot; without them upright, 4 cm back, fists on the seat
    sat = any(int(c.get('body', -1)) == body.pelvis for c in s.get('contacts', ()))
    told = (g.get('palmar', 0.0) > 0.0) & (float(ask.get('reclined', 0.0)) <= 0.5)
    legs = body.legs[:int(sat & told) * len(body.legs)]
    # the step's inputs kept for her introspection (`borne.named`, chain_trace's replay), its answer `got`
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
    """`release` (in place), each joint's clamp's share: whole as far as what bears her needs it - a `distal`
    joint her soles' (`w`, what bears her weighed), the rest also as she lies neither back nor on her side -, a let-go
    leg's (`a`'s, by its `side`) kept whole; numpy's alone, so compiled (`g['released']`)."""
    kept = np.maximum(w[0], a[modal.LET_GO][side])
    release[:] = 1.0 - (1.0 - RELEASED) * (1.0 - (distal * kept + (1.0 - distal) * np.maximum(1.0 - w[2] - w[3], kept)))


def _told(g):
    """The solve's answer and her drives' inertia told her drives' step (`g['drives']`'s arrays,
    `laid`): what it reads every pass till her law's next."""
    got, kept = g['got'], g['drives'][1]
    kept['qacc'][:] = got['qacc']
    kept['tau'][:] = got['tau']
    kept['jeff'][:] = g['jeff']
    kept['over'][:] = got['over']


def laid(g):
    """Her drive-side step laid in C (`wbc_pass.c`, `tools.cores.wbc.Drives`): the struct
    pointing at `g`'s arrays once - the observers, the setpoints and their rates, the holds, the
    knees' spring, the feed and the clamps her drives are told (`g['cmd']`) - and what it reads
    by, kept beside it. Her latent step, her upper body's coupling, her ask, her surety field, her
    touch's filter, her twist's step, her table's waits, her surety's brace on the solve and what bears her off her
    soles (`machine.senses.borne.read`) in C, her python's bit for bit (`tools.cores.motion`, `step`, `surety`, `touch`,
    `twist`, `wait`, `brace`, `borne`); her estimate, her grip's feel and floor, her
    situation, her drive's cap, her ask's poses, her invariant EKF, her barrier (a ring's length a part), her
    field's seek, what bears her weighed, her shield's clearances and her wayfinder's gap compiled from
    their numpy (`tools.cores.bodied`)."""
    from machine.body.figure import JOINTS
    from tools.cores import bodied, motion as c_motion, surety as c_surety, touch as c_touch, twist as c_twist
    from tools.cores import brace as c_brace, wait as c_wait, wbc as cores
    g['estimated'] = bodied.on_body(estimate.step, 'estimate', (0, 2), body=1)
    g['felt'] = bodied.on_args(grip.felt, 'felt', (0, 1, 2, 3, 4, 5, 6, 8))
    g['weighed'] = bodied.on_args(observer.weighed, 'weighed', (0, 1, 2, 3))
    g['clear'] = bodied.on_body(capsules.felt, 'clear', (1,))
    g['gap'] = bodied.on_shapes(wayfinder.gap, 'gap', (0, 1, 2, 3, 4, 5))
    g['outside'] = bodied.on_args(grip.outside, 'outside', (0, 1))
    g['readings'] = bodied.on_args(situation.readings, 'readings', (0, 1, 2, 3, 4, 5))
    g['read'] = bodied.on_args(situation.read, 'read', (0, 1))
    g['modes'] = bodied.on_args(modal.read, 'modes', (0, 1, 2))     # her modes said to her page (`sprung.state`)
    g['pitched'] = bodied.on_args(modal.pitched, 'pitched', (0, 1))     # her trunk reclined (`modal.given`)
    g['capped'] = bodied.on_args(somatic.capped, 'capped', tuple(range(11)))     # her ask's caps (`somatic.asked`)
    g['told_on'] = bodied.on_args(emancipation.on_floor, 'told_on', tuple(range(6)))     # `emancipation.told_on`
    g['settling'] = bodied.on_args(emancipation.settling, 'settling', tuple(range(10)))     # `emancipation.settled`
    g['bounds'] = bodied.on_args(bemf.bounds, 'bounds', (0, 1))     # her drives' torque bounds at their speeds
    g['gripped'] = bodied.on_args(gripped, 'gripped', (0, 1, 2, 3, 4))
    g['sured'] = bodied.on_args(brace.sured, 'sured', tuple(range(8)))     # her ask and posture by her surety
    g['released'] = bodied.on_args(released, 'released', (0, 1, 2, 3))     # her joints' clamps' shares
    g['poses'] = bodied.on_args(modal.poses, 'poses', (0, 1, 2, 3, 4, 5, 6, 7, 9, 10))
    g['ringed'] = bodied.on_shapes(barrier.ringed, 'ringed', (0, 1, 2, 3, 4, 5))
    g['ringed_ahead'] = list(barrier.HULLS)
    g['modal_sensed'] = bodied.on_args(modal.sensed, 'modal_sensed', (0, 1, 2))     # her sensing's arrays walked in C
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
    p = cores.Drives(n=n, q_at=edge.Q.start, qd_at=edge.QD.start, qm_at=edge.QM.start, knee_nm=se2.KNEE_NM,
                     knee_soft=se2.KNEE_SOFT, leak_s=se2.LEAK_S, wep_hold_s=se2.WEP_HOLD_S,
                     bus_k=ctypes.addressof(k), bus=ctypes.addressof(bus), posture_kd=supervisor.POSTURE_KD)
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
    observer stepped on its motor's encoder - her surety as she tilts or slips, a hop, which WEP
    limit is hers - in one C call, `step`'s (its address and call on the tick's arguments: `g['tick'][:2]` the tick
    alone, `tools.cores.tick`; her pass's, `tools.cores.beat`, her law's joints told her boards after it, her felt
    before it where it is both), how far she rests limp told it (`se2.limp`); `drives_twin` its python."""
    _at, _tick, t, vec = g['tick']
    at, tick = step
    t.surety, t.left = g['surety'], g['left']
    tick(at, dt, g['t'], g['wo'], se2.limp(g))
    s_ax, s_lat, d_ax, d_lat, g['surety_probe'] = vec.tolist()
    g['surety'], g['surety_vec'], g['left'] = t.surety, (s_ax, s_lat, d_ax, d_lat), t.left


def drives_twin(g, dt):
    """`drives` in python: the step in C (`laid`), told here what of her it reads whole."""
    p, _kept, step = g['drives']
    held, wo = float(g['hold'].any()), g['wo']
    ys = surety(g, dt)
    drv = 1.0 - (1.0 - ys) ** 3
    wo_s = wo * drv
    in_hop = int(axial_k(g['st']) > HOP_K)
    step(ctypes.byref(p), dt, wo_s, wo_s ** 3, drv, in_hop, 1)
    # a setpoint within its drive's range (the manifest's): asked compression 4.0 the pattern's
    # ankle ask of -77 deg wound the joint to 215.8 of its -45..42 and felled her at 3.0 s.
    np.minimum(np.maximum(g['ref_q'], g['rng'][:, 0], out=g['ref_q']), g['rng'][:, 1], out=g['ref_q'])
    # her clamps told as the step left them, zero where she rests limp (`se2.LIMP`), into a buffer laid
    # once: rebound to its own product a pass, the clamps froze at the first pass's and a rise from
    # the low chair fell at 31.9 s that stood
    np.multiply(_kept['limit'], 1.0 - se2.limp(g), out=g['cmd']['limit'])
    g['left'] = min(se2.WEP_S, max(0.0, g['left'] + dt * (se2.WEP_FILL * (1.0 - held) - held)))
