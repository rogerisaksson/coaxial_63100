"""Her step in closed chains: her tasks and the contacts' forces solved as one QP, within friction.

No sole standing and none asked to land - her legs let go of the floor onto a bed, a seat's edge, the
floor under her knees and hands -, her inverted pendulum is gone: the physics is a parallel chain with
contacts that stick, slide or lift (closed_chain_qp.lisp). The upright step (`machine.stack.wrench`)
holds every bearing point still at W_STILL and plans its push; on a yielding support both are asks it
cannot meet (lying back on a bed she spasmed at the foam's 1.9 Hz and rolled 19 deg). This step asks no
balance of what bears her. Its QP (`chain_qp`) takes her driven joints' accelerations and the contacts'
forces as one unknown, every other quantity affine in them through her dynamics: level 1 answers her
pelvis's and trunk's turns, a sole's swing and the twist told her with the anchors held still and the
forces least, under the laws no answer may break - a contact presses alone and never sinks into its
support faster than its give says, friction within its pyramid, her torques within their clamps -;
level 2 lays her posture in level 1's damped null space, its arm columns projected off her held hands;
her torques are those forces' - a plan that welded the contacts and a Gauss-Seidel that re-solved them
apart squeezed her fists 570 N into each other and starved them. wbc_chain.c is it in C, bit for bit;
`motion.control` picks it by her release alone (`chained`), never by a text's name, and hands it her
standing soles as contacts (`bearing`).
"""
import collections
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.senses.borne import CARPAL, LET
from machine.stack import chain_qp
from machine.stack.rounds import LEGS
from machine.stack.shrunk import box, shrunk
from machine.stack.wbc import HELD_K, LIMIT_S, STOP_RAD, SWING_KD, turn_error

#: The regime as its text says it (`closed-chain-regime`): level 2's damping m and level 1's on x, a contact's
#: softness m/s^2 a N, a sticking contact's slip bled 1/s, her sled's friction and an anchor's, a patch held from N;
#: a held contact's stick rows' weight, the shear and the normals regularised toward none (rad/s^2 a N); the tasks' weights - a
#: turn, a swing, her posture -; its own gains, a turn's and the posture's (kp, kd): lying down is slow, the gait's
#: asked tens of rad/s^2; a twist's weight and gain.
Regime = collections.namedtuple('Regime', 'lam task soft bleed mu anchor hold stick reg nreg turn swing posture tkp tkd pkp '
                                          'pkd twist vkd slide pin tilt')


def chained(ask):
    """1 where the step is hers: a leg let go, no sole standing, or borne reclined (`modal.given`), and no sole asked
    to land unless she lies on no sole (`motion.control`): lying on her side, her leg swung over the bed's edge, the
    upright step took her, shrank her torques to 0.01-0.09 of its ask and whipped her elbow 731 deg/s; this step swings
    it. A sole standing keeps a swing the upright step's: taken here, her rise off the chair never stood; and lying is
    something bearing her now (`propped`, `somatic.asked`), not her support's latch: tipping onto the floor's mattress
    she was reclined in the air with both soles asked to land, and taken here for 3 passes she landed beside it."""
    reclined, standing = ask.get('reclined', 0.0) > 0.5, any(ask['stance'])
    lying = reclined and bool(ask.get('propped', True))
    return int((not ask.get('swing') or (lying and not standing)) and (any(ask.get('let_go', ())) or not standing or reclined))


def bearing(s, loads, stance, chained, shed=(0.0, 0.0), legs=()):
    """Her contacts with each standing sole one more patch where the step is hers (`chained`): its load up (felt) as far
    as her text keeps its leg (`shed`, the release `emancipation` asks of each), its Jacobian's translation, a body
    below 0 - an anchor within its friction's pyramid, never planned as a push. Felt whole, the leg her text lifted took
    82 -> 112 N as the other sole unloaded to 22, that one lost its stance, and she fell off the bed's end at 10.35 s.
    Her legs' touch on what she sits on (`legs`, their bodies, given while she sits and her hands are told to bear -
    `motion`) is the chains' sled alone: the upright step bears her on the kernel's cone, her soles, her hands and her
    seat; with her thighs' points in it its seated balance never asked for her hands and the scoot began on none."""
    soles = [{'p': f['sole'], 'f': np.array([0.0, float(loads[k]) * (1.0 - float(shed[k])), 0.0]),
              'n': np.array([0.0, 1.0, 0.0]), 'J': np.ascontiguousarray(f['J6'][3:6]),
              'dJv': np.asarray(f['dJv'][3:6], float), 'patch': 0, 'side': k, 'body': -1 - k}
             for k, f in enumerate(s['feet']) if chained and stance[k]]
    return [c for c in s.get('contacts', ()) if chained or int(c.get('body', -1)) not in legs] + soles


def regime(said):
    """The `Regime` of the kernel's `closed-chain-regime` as evaluated."""
    return Regime(float(said['dls-damping-m']), float(said['task-damping']), float(said['contact-softness-m-s2-per-n']),
                  float(said['stick-bleed-per-s']), float(said['friction-mu']), float(said['anchor-mu']), float(said['hold-n']),
                  float(said['stick-weight']), float(said['force-reg']), float(said['normal-reg']), float(said['turn-weight']),
                  float(said['swing-weight']),
                  float(said['posture-weight']), float(said['turn-kp']), float(said['turn-kd']), float(said['posture-kp']),
                  float(said['posture-kd']), float(said['twist-weight']), float(said['twist-kd']), float(said['slide-m-s']),
                  float(said['pin-weight']), float(said['slide-tilt']))


def frame(f0, f1, f2):
    """A contact's frame (3, 3), rows: its normal - the way (f0, f1, f2) of what bears it, up where none -, two tangents."""
    norm = math.sqrt(f0 * f0 + f1 * f1 + f2 * f2)
    n0, n1, n2 = (f0 / norm, f1 / norm, f2 / norm) if norm > 0.0 else (0.0, 1.0, 0.0)
    e0, e1, e2 = (0.0, 0.0, 1.0) if abs(n0) > 0.9 else (1.0, 0.0, 0.0)
    t0, t1, t2 = n1 * e2 - n2 * e1, n2 * e0 - n0 * e2, n0 * e1 - n1 * e0
    tn = math.sqrt(t0 * t0 + t1 * t1 + t2 * t2)
    t0, t1, t2 = t0 / tn, t1 / tn, t2 / tn
    return np.array(((n0, n1, n2), (t0, t1, t2), (n1 * t2 - n2 * t1, n2 * t0 - n0 * t2, n0 * t1 - n1 * t0)))


def frames(f):
    """Each contact's `frame` of the normals of what bears it `f` (3k, `borne._normal`'s; her floor's up): (k, 3, 3).
    Of the felt forces instead, her fists' shear tilted it 36-44 deg off her seat's and her plan held them at 7-21 N."""
    return np.array([frame(float(f[3 * i]), float(f[3 * i + 1]), float(f[3 * i + 2])) for i in range(len(f) // 3)]).reshape(-1, 3, 3)


def ways(q):
    """Her pelvis's left, its up and its way, world (3, 3), of its quaternion `q` (w, x, y, z): its turn's rows about
    its up and its way, without its pitch about its left; its error the turn taking its left to the asked left."""
    w, x, y, z = (float(v) for v in q)
    return np.array([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)],
                     [2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)],
                     [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]])


def tasks(b, s, ask, regime):
    """(R (m, n), r (m), want (nd)): the tasks above her posture, weighed - her pelvis's turn about its own up and
    way, its error the turn taking its left to the asked left (`ways`: the log map's way and up parts carried
    0.04-0.09 rad of a 35 deg pitch mismatch, and the way part left her 17 deg off the bed's line), her trunk's whole
    as asked (`regime.turn`; the text's pitch is her trunk's, her pelvis's is her posture's to make, rolling on its
    ischia as a scoot leaves it: asked of the pelvis it hauled her forward 37 deg at the bed's handover), a sole
    asked to swing (`regime.swing`), her pelvis along its level way at the twist told her (`regime.twist`) -, and her
    driven joints' posture acceleration, at the regime's own gains."""
    n, v, act = b.n, s['v'], np.asarray(b.act, int)
    rows, rhs = [], []
    for j, (t, turn) in enumerate(zip(s['turns'], ask.get('turns', ()))):
        have = ways(t['quat'])
        err = np.cross(have[0], ways(np.asarray(turn, float))[0]) if j == 0 else turn_error(np.asarray(turn, float), t['quat'])
        Jr, w, dJr = (np.asarray(t[k], float) for k in ('Jr', 'w', 'dJr'))
        for ax in (np.eye(3) if j else have[1:]):
            rows.append(regime.turn * (ax[0] * Jr[0] + ax[1] * Jr[1] + ax[2] * Jr[2]))
            rhs.append(regime.turn * (ax[0] * (regime.tkp * float(err[0]) - regime.tkd * float(w[0]) - float(dJr[0]))
                                      + ax[1] * (regime.tkp * float(err[1]) - regime.tkd * float(w[1]) - float(dJr[1]))
                                      + ax[2] * (regime.tkp * float(err[2]) - regime.tkd * float(w[2]) - float(dJr[2]))))
    for k, (at, speed, acc, quat) in ask.get('swing', {}).items():
        f = s['feet'][k]
        twist = f['J6'] @ v
        err = turn_error(np.asarray(quat, float), f['quat'])
        a6 = np.concatenate([regime.tkp * err - regime.tkd * twist[:3],
                             np.asarray(acc, float) + SWING_KD[0] * (np.asarray(at, float) - f['sole'])
                             + SWING_KD[1] * (np.asarray(speed, float) - twist[3:])])
        for a in range(6):
            rows.append(regime.swing * np.asarray(f['J6'][a], float))
            rhs.append(regime.swing * (float(a6[a]) - float(f['dJv'][a])))
    vx, way = float(ask.get('twist', (0.0,))[0]), ways(s['turns'][0]['quat'])[2]
    hx, hz = float(way[0]), float(way[2])
    nh = math.sqrt(hx * hx + hz * hz)
    if vx != 0.0 and nh > 0.0:
        row = np.zeros(n)
        row[0], row[2] = hx / nh, hz / nh
        rows.append(regime.twist * row)
        rhs.append(regime.twist * (regime.vkd * (vx - (float(row[0]) * float(v[0]) + float(row[2]) * float(v[2])))))
    want = regime.pkp * (np.asarray(ask['posture'], float) - s['q']) - regime.pkd * v[act]
    # hung off a seat (`ask['hang']`, each leg's share): its knee, ankle and their roll (`rounds.LEGS`' last three) as
    # gravity swings them, damped at `ask['hang_rate']` - her drives hold none of it (`wrench._maps`' PASSIVE)
    rate = float(ask.get('hang_rate', 0.0))
    for k, leg in enumerate(LEGS):
        j = np.asarray(leg[3:])
        c = act[j]
        passive = -s['h'][c] / np.diag(s['M'])[c] - rate * v[c]
        want[j] = want[j] + float(ask.get('hang', (0.0, 0.0))[k]) * (passive - want[j])
    return (np.vstack(rows) if rows else np.zeros((0, n))), np.array(rhs, float), want


def stop_rows(b, s, nd):
    """(rows (2j, nd), bounds (2j)): her driven joints' stops on her plan's accelerations as her upright QP bounds them
    (`wbc`): each met no nearer than STOP_RAD, LIMIT_S ahead at the acceleration asked."""
    k = np.asarray(b.stops, int)
    rows = np.zeros((2 * len(k), nd))
    rows[np.arange(len(k)), k], rows[len(k) + np.arange(len(k)), k] = 1.0, -1.0
    ahead = s['q'][k] + s['v'][np.asarray(b.act, int)[k]] * LIMIT_S
    return rows, np.concatenate([2.0 * (b.stop_hi - STOP_RAD - ahead) / LIMIT_S ** 2,
                                 -2.0 * (b.stop_lo + STOP_RAD - ahead) / LIMIT_S ** 2])


def gives_of(b, contacts, regime, compliant, tc=None):
    """Each contact's give for the QP's stick rows and its law of no sinking (`chain_qp.level1`): None, all rigid,
    unless `compliant` - True each at the foam's `regime.soft`, 'sensed' each as her skin has it, its support's
    impedance over her mass (`borne.read`'s give, edge.GIVE), 'material' each as her world has what bears it, the
    python twin's measure alone (`tc` {her
    body's name: the time constant of what it touches}, read off her world: over the foam's, squared, of
    `regime.soft`; MuJoCo's default rigid)."""
    if not compliant:
        return None
    if compliant is True:
        return np.full(len(contacts), regime.soft)
    if compliant == 'sensed':
        # as much more as what bears her gives than her floor, as her skin feels it (`borne.read`'s give); a floor none
        return np.array([float(c.get('give', 0.0)) for c in contacts])
    from machine.world.floor import FOAM
    foam, out = float(FOAM.split('solref="')[1].split()[0]), []
    for c in contacts:
        k = int(c.get('body', 0))
        t = (tc or {}).get(b.m.body(k).name if k >= 0 else ('left', 'right')[-1 - k] + '_foot', 0.0)
        out.append(regime.soft * (t / foam) ** 2 if t > 0.02 + 1e-9 else 0.0)
    return np.array(out)


def step(b, s, ask, regime, stopped=False, compliant='sensed'):
    """{tau, qacc, bears, cop, over, slack, held, holds, modes, borne, scale, residual, foreseen, met, al}: `ask` as
    `wrench.step` takes it, no sole standing; `borne` each contact's force (k, 3), world; `modes` each contact's
    (`chain_qp.unknowns`: 1 a static sled, 2 a kinetic one); `scale` what her drives carry of the ask,
    1 where the QP met its laws; `residual` the complementarity's; `foreseen` the base's and passive joints'
    accelerations the QP counted on, `al` the contacts' in their frames (3k); `held` the rows it held, warm for the
    next pass (`b.chain_warm`); `holds` which
    contacts' normals it held (every touching one but a foot told free and those it let go last pass, `b.chain_let`:
    a held contact it would have pulled on, a let one it still lifts - `chain_qp.letting`; an anchor's tangents stick
    where its patch bears `regime.hold`). Her driven joints' accelerations, the
    contacts' forces and so her torques solved as one (`chain_qp`); `stopped`: her joints' stops among the laws
    (`stop_rows`); `compliant`: what she holds giving (`gives_of`). Where no answer meets the laws, the least squares
    alone and her torques shrunk as the upright step's: toward nothing asked of her joints, then toward 0."""
    n, M, h, v = b.n, s['M'], s['h'], s['v']
    contacts = s.get('contacts', ())
    A, H = np.asarray(b.act, int), np.asarray(b.held, int)
    P = np.flatnonzero((np.arange(n) < 6) | b.passive)
    given, k, nd = np.concatenate([A, H]), len(contacts), len(b.act)
    held = -HELD_K * v[H]
    J = np.vstack([c['J'] for c in contacts]) if k else np.zeros((0, n))
    dJv = np.concatenate([np.asarray(c['dJv'], float) for c in contacts]) if k else np.zeros(0)
    f = np.concatenate([np.asarray(c['f'], float) for c in contacts]) if k else np.zeros(0)
    patch = np.array([int(c.get('patch', 0)) for c in contacts], int)
    body = np.array([int(c.get('body', 0)) for c in contacts], int)
    Mpp, Mpg = np.ascontiguousarray(M[np.ix_(P, P)]), np.ascontiguousarray(M[np.ix_(P, given)])
    Minv = np.linalg.inv(Mpp)
    rhs0 = -h[P]
    Jp, Jg = np.ascontiguousarray(J[:, P]), np.ascontiguousarray(J[:, given])
    Jv, Q = (J @ v if k else np.zeros(0)), frames(np.concatenate([np.asarray(c['n'], float) for c in contacts]) if k else f)
    R, r, want = tasks(b, s, ask, regime)
    vx, way = float(ask.get('twist', (0.0,))[0]), ways(s['turns'][0]['quat'])[2]
    hx, hz = float(way[0]), float(way[2])
    nh, sign = math.sqrt(hx * hx + hz * hz), 1.0 if vx > 0.0 else -1.0
    slide = (sign * (hx / nh), 0.0, sign * (hz / nh)) if vx != 0.0 and nh > 0.0 else (0.0, 0.0, 0.0)
    felt = [math.sqrt(f[3 * i] * f[3 * i] + f[3 * i + 1] * f[3 * i + 1] + f[3 * i + 2] * f[3 * i + 2]) for i in range(k)]
    free = ask.get('free', (False, False))
    width, S, mu, bears, sled, mode, st = chain_qp.unknowns(patch, body, felt, Q, slide, free, regime, Jv)
    keys = chain_qp.keyed(patch, body)
    loose = [chain_qp.freed(int(patch[i]), int(body[i]), free) for i in range(k)]
    let = [not loose[i] and keys[i] in b.chain_let for i in range(k)]
    holds = [not loose[i] and not let[i] for i in range(k)]
    stick = [holds[i] and bears[i] and not sled[i] for i in range(k)]
    maps = chain_qp.affine(Minv, Mpg, rhs0, held, Jp, Jg, dJv, Q, S, np.ascontiguousarray(R[:, P]),
                           np.ascontiguousarray(R[:, given]), r, np.ascontiguousarray(M[np.ix_(A, np.concatenate([P, given]))]),
                           h[A], np.ascontiguousarray(J[:, A]), nd)
    gives = gives_of(b, contacts, regime, compliant, ask.get('_tc'))
    limit, lo, hi = box(b, ask)
    H1, g1, G1, Aq, bq = chain_qp.level1(maps, S, width, mu, holds, stick, mode, st, np.zeros(k) if gives is None else gives, Jv,
                                         f, Q, regime, (lo, hi), stop_rows(b, s, nd) if stopped else None, float(M[0, 0]))
    hands = [i for i in range(k) if holds[i] and bears[i] and CARPAL <= int(patch[i]) < LET]
    u, b.chain_warm, met = chain_qp.solved(H1, g1, G1, Aq, bq, maps, S, Jg, hands, want, regime, b.chain_warm)
    x, foreseen, z, tau1, residual, al = chain_qp.answer(u, maps, S, Q, nd)
    b.chain_let = chain_qp.letting(keys, holds, let, b.chain_warm[0], chain_qp.zrows(holds, width, mode), al)
    qdd = np.zeros(n)
    qdd[given], qdd[P] = np.concatenate([x, held]), foreseen
    if met or not (np.abs(tau1) > limit).any():
        scale, tau = 1.0, tau1
    else:
        zeta = u[nd:]
        tau0, qdd0 = maps.t0 + maps.Tz @ zeta, np.zeros(n)
        qdd0[given], qdd0[P] = np.concatenate([np.zeros(nd), held]), maps.p0 + maps.Pz @ zeta
        scale = shrunk(tau1, tau0, -limit, limit)
        tau, qdd = tau0 + scale * (tau1 - tau0), qdd0 + scale * (qdd - qdd0)
        s0 = shrunk(tau, np.zeros(nd), -limit, limit) if (np.abs(tau0) > limit).any() else 1.0
        if s0 < 1.0:
            tau, qdd = s0 * tau, s0 * qdd
        scale = s0 * scale
    past = np.abs(tau1) > b.clamp * (1.0 + 1e-6)
    over = past.copy()
    for p, q in b.pairs:
        over[p] = over[q] = past[p] or past[q]
    b.load = np.abs(tau1) / b.clamp
    return {'tau': np.clip(tau, lo, hi), 'qacc': qdd, 'bears': [0.0, 0.0], 'cop': [], 'over': over, 'slack': [],
            'held': b.chain_warm, 'holds': holds, 'modes': mode, 'borne': z.reshape(-1, 3), 'scale': scale,
            'residual': residual, 'foreseen': foreseen, 'met': met, 'al': al}
