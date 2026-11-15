"""The robot's legs' release in arrays alone: `emancipation`'s arithmetic, compiled by `motion.laid`.

The robot's stage reads and lays its session (`emancipation`); these take its arrays and give
arrays back, written nowhere else - so each is held bit for bit in C (`tools.cores.bodied`), numpy's own until it
is.

    kept, iota, om, xi = cascade.asked(was, sat, told, out, com, vcom, floor)       # how far each leg is asked
    eta, *parts, reach = cascade.leave(xi, pts, room, soles, vcom, shares, loads, weight)  # the physics' leave
    eta, gamma, *laid = cascade.released(was, iota, eta, kept, sat, told, a, y, com, loads, fr, held, out, dt)
    high, ref, posture, share, hold = cascade.opening(let, q, fr, a, y, out, high, ...)  # the robot's let-go legs' chains
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import impedance, latent, melt, modal, poise
from machine.law.stride import STRIDE_M
from machine.moves.seated import POPLITEAL, RESTS
from machine.texts import kernel, maneuver, ortho
from machine.senses import observer
from machine.stack import rounds

#: The gate, the cascade as the robot's text lays them (`xform-intentional-limb-emancipation`).
GATE = kernel.node('xform-intentional-limb-emancipation', 'release-gate')
OPEN = kernel.node('xform-intentional-limb-emancipation', 'open-chain')['open-chain-impedance']
CASCADE = kernel.node('xform-intentional-limb-emancipation', 'unloading-cascade')
#: A leg hung off a seat, the kernel's (`specialize-posture-dangling-crural-hang` of `xform-sit-on-chair-or-stool`):
#: its knee, ankle and their roll asked where they are, as gravity swings them. The kernel's reach foliation
#: (`modulate-kinematic-reachability`):
#: the robot's soles on its toes through its window, dangling from its detach (DANGLE); sat, the robot's reach
#: is the seat's height over its popliteal height (`seated.POPLITEAL`).
HANG = kernel.node('xform-sit-on-chair-or-stool', 'limb-hang')
#: A hung leg's passive pendulum's damping, 1/s - twice its ratio its eigenfrequency, the robot's shank's own
#: (test_bedside): its knee and ankle go as gravity swings them, damped so (`wrench._maps`).
HANG_RATE = (2.0 * HANG['passive-pendulum-dynamics']['damping-ratio']
             * 2.0 * np.pi * HANG['passive-pendulum-dynamics']['eigenfreq-hz'])
FOLIATION = kernel.node('modulate-kinematic-reachability', 'reachability')
DANGLE = (FOLIATION['equinus-morph-window']['rho-bounds'][1], FOLIATION['unloading-horizon']['detach-rho'])
#: The robot's Froude number's lockout; the least share of its weight what bears it besides a leg is
#: told; the least margin of its capture point inside it, N m (the robot's weight times its distance in).
FROUDE, ANCHORS, MARGIN_NM = GATE['max-allowable-froude'], GATE['min-complement-weight-sum'], GATE['min-residual-cone-margin']
#: The cascade's slew to release, s, and back; its windows in a leg's release - the robot's estimator's anchor let
#: go, its step's push shed -; the share of its weight a shed sole bears over its own leg's under
#: which the leg is free to move: a foot sliding on the floor bears its leg (the robot's legs 0.38 of it);
#: held to the text's 0.03 of it, lifted foot's, its heels never let its knees go.
SLEW_S, RELOCK_S = 1e-3 * CASCADE['slew-duration-ms'], 1e-3 * CASCADE['relock-slew-ms']
UNANCHOR, SHED = CASCADE['observer-unanchor-window'], CASCADE['qp-load-shed-window']
FREED = CASCADE['kinematic-release-fz-floor']
#: A leg moves once shed, from the shed window's end.
FREE = (SHED[1], 1.0)
#: The robot's leg, m: what its Froude number is over.
LEG_M = maneuver._manifest()[1]['leg']
#: The robot's comparator's release share (`observer.RELEASE`): soles told less of what is told are told
#: nothing.
TOLD_NOTHING = observer.RELEASE
PLANTAR, DORSAL = ortho.MANIFOLDS.index('plantar'), ortho.MANIFOLDS.index('dorsal')

#: The robot's legs' modes (`melt.LEGGED`): held where they were as a leg is asked released, till it is free - moved
#: by the robot's text while its leg stood on the gate's leave, the sled's knees opened under 294 N on the robot's
#: soles and threw it back.
LEGGED = melt.LEGGED
#: The robot's legs' joints, a row a leg (`rounds.LEGS`).
LEG_JOINTS = np.array(rounds.LEGS)
#: The directions the robot's support's margin is taken along: its error at most R (1 - cos(pi / DIRECTIONS)), 0.6
#: mm over the robot's 0.5 m reach.
DIRECTIONS = 64

_U = np.stack([np.cos(np.arange(DIRECTIONS) * 2.0 * np.pi / DIRECTIONS),
               np.sin(np.arange(DIRECTIONS) * 2.0 * np.pi / DIRECTIONS)], axis=1)


def smooth(x):
    """The smooth step of `x` over [0, 1]."""
    u = np.minimum(np.maximum(x, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def window(gamma, span):
    """How far `gamma` is through the window `span` (lo, hi), 0 to 1."""
    return np.minimum(np.maximum((gamma - span[0]) / (span[1] - span[0]), 0.0), 1.0)


def reached(point, pts, room, mask):
    """(k, DIRECTIONS) each lane's support's reach past `point` along each of `_U`: `margin`'s, of arrays alone."""
    return np.where(mask[:, None, :] > 0.0, (_U @ pts.T + room)[None], -np.inf).max(axis=2) - (_U @ point)[None]


def nothing(told):
    """`told_nothing` of arrays alone, a 0-d array."""
    return np.minimum(np.maximum(1.0 - observer.said(told)[PLANTAR] / TOLD_NOTHING, 0.0), 1.0)


def lift(out):
    """The robot's text's lift of a leg, a 0-d array: + the first of `impedance.LIMBS`, its rhythm aside (the
    robot's stride's clearance is no lift)."""
    return out[latent.REF][modal.LIFT] - out[latent.SWAY][modal.LIFT] * np.sin(out[latent.PSI][modal.LIFT])


def intent(told, out):
    """How far the robot's text asks each leg released, 0 to 1 (2,): its soles told nothing (`told_nothing`), or
    the leg lifted past its poise's lift (`impedance.LIMBS`, `lift`)."""
    return np.maximum(nothing(told), window(impedance.LIMBS * lift(out), (0.0, poise.LIFT_EPS)))


def leave(xi, pts, room, soles, vcom, shares, loads, weight):
    """`gate`'s arithmetic, of arrays alone: (eta, froude's, anchors', margin's, margin N m, the reach `reached`)."""
    froude = (vcom[0] * vcom[0] + vcom[2] * vcom[2]) / (modal.G * LEG_M)
    by_froude = np.exp(-(froude / FROUDE) ** 2)
    own = shares[PLANTAR] * loads / np.maximum(loads.sum(), observer.TINY)
    by_anchors = smooth((shares.sum() - own - ANCHORS) / (1.0 - ANCHORS))
    reach = reached(xi, pts, room, 1.0 - soles)
    m = reach.min(axis=1)
    by_margin = smooth((weight * m - MARGIN_NM) / MARGIN_NM)
    return by_froude * by_anchors * by_margin, by_froude, by_anchors, by_margin, weight * m, reach


def asked(was, sat, told, out, com, vcom, floor):
    """`step`'s ask, of arrays alone: (kept (2,), iota (2,), omega, the robot's capture point (2,)) - each leg
    kept released while its text tells its soles nothing off the floor, how far the robot's
    text asks it, the robot's pendulum's rate over `floor`."""
    kept = (1.0 - sat) * (observer.said(told)[PLANTAR] < 0.10) * (was > 0.85)
    iota = np.maximum(intent(told, out), kept)
    om = np.sqrt(modal.G / np.maximum(com[1] - floor, impedance.LOW_M))
    return kept, iota, om, com[[0, 2]] + vcom[[0, 2]] / om


def released(was, iota, eta, kept, sat, told, a, y, com, loads, fr, held, out, dt):
    """`step`'s cascade, of arrays alone: (eta, gamma, let-go, unanchored, let-go caps, freed, the robot's
    legs' held references, rates and accelerations) - `eta` the physics' leave (`gate`), aloft or kept
    opened."""
    weight = fr[modal.WEIGHT]
    aloft = (1.0 - sat) * (y[modal.LOWEST] - com[1] > -0.38) * (was > SHED[0]) * (observer.said(told)[PLANTAR] < 0.10)
    eta = np.maximum(eta, np.maximum(kept, aloft))
    # lain back, the robot's soles told nothing and neither leg lifted alone (the bed's side swings one, then the
    # other), its legs leave as one: a leg at a time, the first's push came onto the other's sole (76 ->
    # 214 N), its gate shut and relocked it, and the free leg threw the robot's round 115 deg on the bed
    alike = (observer.said(told)[PLANTAR] < 0.10) * (1.0 - window(np.abs(lift(out)), (0.0, poise.LIFT_EPS)))
    asked = iota * eta
    asked = asked + alike * np.clip(told[DORSAL] / 0.10, 0.0, 1.0) * (asked.max() - asked)
    gamma = was + np.minimum(np.maximum(asked - was, -dt / RELOCK_S), dt / SLEW_S)
    # lain back, the robot's soles told nothing and a leg already free: the other let go, its gate's leave or not.
    # Held by it, the robot's left leg stood alone under 0.06 of its told, pushed 368 N - its pelvis
    # 0.2 m up off the bed, what bore its unlatched - and swung as a stride's as its sole lifted, -13.6 rad/s
    lying = (observer.said(told)[PLANTAR] < TOLD_NOTHING) * np.clip(told[DORSAL] / 0.10, 0.0, 1.0) * (was.max() > 0.85)
    let_go = np.maximum(window(gamma, (0.0, SHED[0])), np.maximum(np.maximum(nothing(told) * a[modal.PROPPED], lying), sat))
    leg = weight * (1.0 - 1.0 / fr[modal.HIPS_BY])
    freed = np.maximum(window(gamma, FREE) * np.minimum(np.maximum(1.0 - (loads - leg) / (FREED * weight), 0.0), 1.0),
                       kept)
    hold = (iota * (1.0 - freed)).max() * (1.0 - freed.max())
    held = held + (1.0 - hold) * (out[latent.REF][LEGGED] - held)
    # a let-go foot the robot's text does not lift bears its leg below the knee resting (`seated.RESTS`): shed to
    # nothing as the robot sat, its plan stood its low chair on one resting shank (0/278 N)
    rests = (1.0 - window(impedance.LIMBS * lift(out), (0.0, poise.LIFT_EPS))) * RESTS * weight
    return (eta, gamma, let_go, 1.0 - window(gamma, UNANCHOR), np.maximum((1.0 - window(gamma, SHED)) * weight, rests),
            freed, held,
            out[latent.RATE][LEGGED] * (1.0 - hold), out[latent.ACC][LEGGED] * (1.0 - hold))


def opening(let, q, fr, a, y, out, high, iota, freed, told, sat, posture, bend, q_open, laid, q_lo, q_hi, emancipated,
            dt):
    """`opened`'s arithmetic, of arrays alone: (the robot's let-go legs' hip heights, its open chains'
    references, its posture's ask, its open chains' share of their stiffness, the robot's let-go legs'
    hold) - `posture` its ask's, `laid` its session's."""
    rot = fr[modal.ROT].reshape(3, 3)
    leg = modal._leg(y, rot)
    # a leg the robot's text lifts AND drives (`:limb-back-m`) is placed: raised 4 cm a lift unit to the
    # text's reach, its ankle `back` nearer the robot than its knee had it; undriven, as before to the bit
    back = impedance.LIMBS * STRIDE_M * out[latent.REF][modal.DRIVE]
    driven = (back != 0.0) * 1.0
    up = np.clip(impedance.LIMBS * out[latent.REF][modal.LIFT], 0.0, 1.0)
    up = up + driven * (np.clip(impedance.LIMBS * out[latent.REF][modal.LIFT], 0.0, ortho.REACH) - up)
    rising = np.logical_and(out[latent.RATE][modal.PITCH] > 0.05, out[latent.RATE][modal.AXIAL] > 0.0) * 1.0
    lifted = (1.0 - rising) * np.maximum(iota, freed * np.clip(told[DORSAL] / 0.10, 0.0, 1.0))
    # in bed as the robot's back is told to bear or its text lifts the leg - sat upright on a chair,
    # let go, it stays where the robot's seat put it: read as in bed there, the robot's hips went to 0.08 m
    # over its ankles, its left sole off the low chair's floor, and it rose on the
    # robot's right alone, its left leg up to 0.98 m, and fell
    bed = let * (1.0 - sat) * np.maximum(np.clip(told[DORSAL] / 0.10, 0.0, 1.0),
                                         window(impedance.LIMBS * lift(out), (0.0, poise.LIFT_EPS)))
    # where else than the robot's soles bears it upright (borne, its trunk not past
    # RECLINED_RAD back) on a seat its shank cannot reach the floor from - the kernel's reach
    # foliation from what bears it: its lowest over the floor as sensed (`modal.PROP_LO`) over the
    # robot's popliteal height, past its toes' window to its detach -, a let-go leg the robot's text neither
    # lifts nor places hangs: its knee and ankle the kernel's (HANG), off the robot's axial mode, its hip
    # where the seat holds it, on its open chain's own impedance. Read as lifted in bed, the robot's hips'
    # height over its ankles went to 0.08 m and its knees, on its axial mode, opened 90 -> 33 deg on a chair;
    # read from its knee's height, its knees up on the floor reached as far (rho 1) and hung its soles bearing
    # 20 and 91 N: it fell
    pitch = modal.pitched(y, rot)
    reach = smooth(((y[modal.PROP_LO][1] - fr[modal.FLOOR]) / POPLITEAL - DANGLE[0]) / (DANGLE[1] - DANGLE[0]))
    hung = (let * a[modal.BORNE] * (1.0 * (pitch[1] >= -modal.RECLINED_RAD)) * reach
            * (1.0 - window(impedance.LIMBS * lift(out), (0.0, poise.LIFT_EPS))) * (1.0 - driven))
    down = 0.42 * (1.0 - lifted) + 0.08 * lifted
    high = (let * (high + dt * bed * (down - 0.04 * up - high) / SLEW_S)
            + (1.0 - let) * np.sqrt(leg[:, modal.UP] ** 2 + leg[:, modal.LEFT] ** 2))
    cosine = np.cos(0.5 * fr[modal.KNEE]) - modal.AXIAL_M * out[latent.REF][modal.AXIAL] * fr[modal.HIPS_BY] / modal.LEG_M
    bent = (1.0 - bed * (1.0 - rising)) * (2.0 * np.arccos(np.clip(cosine, -1.0, 1.0)) - fr[modal.KNEE] - a[modal.BEND]) + 0.15 * bed * up
    said = posture[modal.CHAIN[:, 1]] + bend[modal.CHAIN[:, 1]] * bent
    # the placed knee solved from the text's knee `said` (the slewed reference carries dq and ran away straight):
    # the ankle as far ahead of the hip as that knee puts it at this height, less `back`, under the hip at most
    flexed = modal.FLEX[1] * said
    was = np.maximum(np.sqrt(np.maximum(modal.SHANK ** 2 + modal.THIGH ** 2 + 2.0 * modal.SHANK * modal.THIGH * np.cos(flexed)
                                        - high ** 2, 0.0)) - back, 0.0)
    placed = np.arccos(np.clip((was ** 2 + high ** 2 - modal.SHANK ** 2 - modal.THIGH ** 2)
                               / (2.0 * modal.SHANK * modal.THIGH), -1.0, 1.0)) / modal.FLEX[1]
    said = said + driven * (placed - said)
    # hung, its knee and ankle asked where they are - passive in the robot's step; asked at the kernel's origin (90
    # deg, -0.24 rad), its ask stood 25 deg off the ankle its sole set on the floor, and that much
    # jumped as the hang let go
    said = said + hung * (q[modal.CHAIN[:, 1]] - said)
    ref = q_open.copy()
    ref[modal.CHAIN[:, 1]] += dt * let * (said - ref[modal.CHAIN[:, 1]]) / SLEW_S
    knee = modal.FLEX[1] * ref[modal.CHAIN[:, 1]]
    behind = np.sqrt(np.maximum(modal.SHANK ** 2 + modal.THIGH ** 2 + 2.0 * modal.SHANK * modal.THIGH * np.cos(knee)
                                - high ** 2, 0.0))
    lean = np.arctan2(-behind, high) + np.arctan2(modal.THIGH * np.sin(knee), modal.SHANK + modal.THIGH * np.cos(knee))
    # the robot's pelvis's pitch as asked while its legs are to stand or rest - their weight, out ahead of
    # it, holds its pelvis there, the floor sit's - and as sensed (`modal.pitched`'s first, spelled
    # out) as its back is told to bear, its legs to lift: asked then, their weight hauled the
    # robot's pelvis to the ask, 37 deg forward at the bed's handover where the scoot had rolled it back, and the
    # robot's trunk with it off the robot's fists; by the legs' lift share the haul reloaded its soles and
    # the ask came back, round and round
    told_up = (1.0 - rising) * np.clip(told[DORSAL] / 0.10, 0.0, 1.0)
    qw, qx, qj, qk = y[modal.QUAT][0], y[modal.QUAT][1], y[modal.QUAT][2], y[modal.QUAT][3]
    up0, up1, up2 = 2.0 * (qx * qj - qw * qk), 1.0 - 2.0 * (qx * qx + qk * qk), 2.0 * (qj * qk + qw * qx)
    sensed = np.arctan2(up0 * rot[modal.ALONG][0] + up1 * rot[modal.ALONG][1] + up2 * rot[modal.ALONG][2], up1)
    aim = (1.0 - told_up) * (fr[modal.PITCH0][0] + modal.PITCH_RAD * out[latent.REF][modal.PITCH]) + told_up * sensed
    hip = modal.FLEX[0] * (knee - lean + aim)
    ankle = (1.0 - bed) * modal.FLEX[2] * lean + bed * laid[modal.CHAIN[:, 2]]
    chain = np.stack([hip + hung * (posture[modal.CHAIN[:, 0]] - hip), modal.FLEX[1] * knee,
                      ankle + hung * (q[modal.CHAIN[:, 2]] - ankle)], axis=1)
    # both legs at once, each element's ops as a leg's alone (their joints apart): 20 numpy calls where 10
    joints, on = LEG_JOINTS, let[:, None]
    target = (1.0 - bed[:, None]) * ref[joints] + bed[:, None] * laid[joints]
    target[:, 2:5] = chain
    target[:, 5] = target[:, 5] + hung * (q[joints][:, 5] - target[:, 5])
    ref[joints] += dt * on * (target - ref[joints]) / SLEW_S
    ref[joints] = np.clip(on * ref[joints] + (1.0 - on) * q[joints], q_lo[joints], q_hi[joints])
    posture = posture.copy()
    posture[joints] = on * ref[joints] + (1.0 - on) * posture[joints]
    return high, ref, posture, freed * (1.0 - bed * (1.0 - hung)), 1.0 - window(emancipated, SHED), hung


def compiled():
    """Each kernel here its C (`tools.cores.bodied`), keyed as the robot's session holds it
    (`machine.law.motion.laid`)."""
    from tools.cores import bodied
    return {'leave': bodied.on_shapes(leave, 'leave', tuple(range(8))),
            'em_asked': bodied.on_args(asked, 'em_asked', tuple(range(7))),
            'em_released': bodied.on_args(released, 'em_released', tuple(range(14))),
            'em_opening': bodied.on_args(opening, 'em_opening', tuple(range(19)))}
