"""Her legs' release in arrays alone: `emancipation`'s arithmetic, each compiled by `motion.laid`.

Her stage reads and lays her session (`emancipation`); these take her arrays and give arrays back, written nowhere
else - so each is held bit for bit in C (`tools.cores.bodied`), numpy's own until it is.

    kept, iota, om, xi = cascade.asked(was, sat, told, out, com, vcom, floor)       # how far each leg is asked
    eta, *parts, reach = cascade.leave(xi, pts, room, soles, vcom, shares, loads, weight)  # the physics' leave
    eta, gamma, *laid = cascade.released(was, iota, eta, kept, sat, told, a, y, com, loads, fr, held, out, dt)
    high, ref, posture, share, hold = cascade.opening(let, q, fr, a, y, out, high, ...)  # her let-go legs' chains
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.law import impedance, latent, melt, modal, poise
from machine.texts import kernel, maneuver, ortho
from machine.senses import observer
from machine.stack import rounds

#: The gate, the cascade as her text lays them (`xform-intentional-limb-emancipation`).
GATE = kernel.node('xform-intentional-limb-emancipation', 'release-gate')
OPEN = kernel.node('xform-intentional-limb-emancipation', 'open-chain')['open-chain-impedance']
CASCADE = kernel.node('xform-intentional-limb-emancipation', 'unloading-cascade')
#: Her Froude number's lockout; the least share of her weight what bears her besides a leg is told; the least margin
#: of her capture point inside it, N m (her weight times its distance in).
FROUDE, ANCHORS, MARGIN_NM = GATE['max-allowable-froude'], GATE['min-complement-weight-sum'], GATE['min-residual-cone-margin']
#: The cascade's slew to release, s, and back; its windows in a leg's release - her estimator's anchor let go, her
#: step's push shed -; the share of her weight a shed sole bears over its own leg's under which the leg is free to
#: move: a foot sliding on the floor bears its leg (her legs 0.38 of her); held to the text's 0.03 of her, lifted
#: foot's, her heels never let her knees go.
SLEW_S, RELOCK_S = 1e-3 * CASCADE['slew-duration-ms'], 1e-3 * CASCADE['relock-slew-ms']
UNANCHOR, SHED = CASCADE['observer-unanchor-window'], CASCADE['qp-load-shed-window']
FREED = CASCADE['kinematic-release-fz-floor']
#: A leg moves once shed, from the shed window's end.
FREE = (SHED[1], 1.0)
#: Her leg, m: what her Froude number is over.
LEG_M = maneuver._manifest()[1]['leg']
#: Her comparator's release share (`observer.RELEASE`): soles told less of what is told are told nothing.
TOLD_NOTHING = observer.RELEASE
PLANTAR, DORSAL = ortho.MANIFOLDS.index('plantar'), ortho.MANIFOLDS.index('dorsal')

#: Her legs' modes (`melt.LEGGED`): held where they were as a leg is asked released, till it is free - moved by her
#: text while its leg stood on the gate's leave, the sled's knees opened under 294 N on her soles and threw her back.
LEGGED = melt.LEGGED
#: Her legs' joints, a row a leg (`rounds.LEGS`).
LEG_JOINTS = np.array(rounds.LEGS)
#: The directions her support's margin is taken along: its error at most R (1 - cos(pi / DIRECTIONS)), 0.6 mm over
#: her 0.5 m reach.
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


def intent(told, out):
    """How far her text asks each leg released, 0 to 1 (2,): her soles told nothing (`told_nothing`), or the leg
    lifted past her poise's lift (`impedance.LIMBS`, its rhythm aside: her stride's clearance is no lift)."""
    ref = out[latent.REF][modal.LIFT] - out[latent.SWAY][modal.LIFT] * np.sin(out[latent.PSI][modal.LIFT])
    return np.maximum(nothing(told), window(impedance.LIMBS * ref, (0.0, poise.LIFT_EPS)))


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
    """`step`'s ask, of arrays alone: (kept (2,), iota (2,), omega, her capture point (2,)) - each leg kept released
    while her text tells her soles nothing off the floor, how far her text asks it, her pendulum's rate over `floor`."""
    kept = (1.0 - sat) * (observer.said(told)[PLANTAR] < 0.10) * (was > 0.85)
    iota = np.maximum(intent(told, out), kept)
    om = np.sqrt(modal.G / np.maximum(com[1] - floor, impedance.LOW_M))
    return kept, iota, om, com[[0, 2]] + vcom[[0, 2]] / om


def released(was, iota, eta, kept, sat, told, a, y, com, loads, fr, held, out, dt):
    """`step`'s cascade, of arrays alone: (eta, gamma, let-go, unanchored, let-go caps, freed, her legs' held
    references, rates and accelerations) - `eta` the physics' leave (`gate`), aloft or kept opened."""
    weight = fr[modal.WEIGHT]
    aloft = (1.0 - sat) * (y[modal.LOWEST] - com[1] > -0.38) * (was > SHED[0]) * (observer.said(told)[PLANTAR] < 0.10)
    eta = np.maximum(eta, np.maximum(kept, aloft))
    gamma = was + np.minimum(np.maximum(iota * eta - was, -dt / RELOCK_S), dt / SLEW_S)
    let_go = np.maximum(window(gamma, (0.0, SHED[0])), np.maximum(nothing(told) * a[modal.PROPPED], sat))
    leg = weight * (1.0 - 1.0 / fr[modal.HIPS_BY])
    freed = np.maximum(window(gamma, FREE) * np.minimum(np.maximum(1.0 - (loads - leg) / (FREED * weight), 0.0), 1.0),
                       kept)
    hold = (iota * (1.0 - freed)).max() * (1.0 - freed.max())
    held = held + (1.0 - hold) * (out[latent.REF][LEGGED] - held)
    return (eta, gamma, let_go, 1.0 - window(gamma, UNANCHOR), (1.0 - window(gamma, SHED)) * weight, freed, held,
            out[latent.RATE][LEGGED] * (1.0 - hold), out[latent.ACC][LEGGED] * (1.0 - hold))


def opening(let, q, fr, a, y, out, high, iota, freed, told, sat, posture, bend, q_open, laid, q_lo, q_hi, emancipated,
            dt):
    """`opened`'s arithmetic, of arrays alone: (her let-go legs' hip heights, her open chains' references, her
    posture's ask, its open chains' share of their stiffness, her let-go legs' hold) - `posture` her ask's,
    `laid` her session's."""
    rot = fr[modal.ROT].reshape(3, 3)
    leg = modal._leg(y, rot)
    up = np.clip(impedance.LIMBS * out[latent.REF][modal.LIFT], 0.0, 1.0)
    rising = np.logical_and(out[latent.RATE][modal.PITCH] > 0.05, out[latent.RATE][modal.AXIAL] > 0.0) * 1.0
    lifted = (1.0 - rising) * np.maximum(iota, freed * np.clip(told[DORSAL] / 0.10, 0.0, 1.0))
    bed = let * (1.0 - sat)
    down = 0.42 * (1.0 - lifted) + 0.08 * lifted
    high = (let * (high + dt * bed * (down - 0.04 * up - high) / SLEW_S)
            + (1.0 - let) * np.sqrt(leg[:, modal.UP] ** 2 + leg[:, modal.LEFT] ** 2))
    cosine = np.cos(0.5 * fr[modal.KNEE]) - modal.AXIAL_M * out[latent.REF][modal.AXIAL] * fr[modal.HIPS_BY] / modal.LEG_M
    bent = (1.0 - bed * (1.0 - rising)) * (2.0 * np.arccos(np.clip(cosine, -1.0, 1.0)) - fr[modal.KNEE] - a[modal.BEND]) + 0.15 * bed * up
    said = posture[modal.CHAIN[:, 1]] + bend[modal.CHAIN[:, 1]] * bent
    ref = q_open.copy()
    ref[modal.CHAIN[:, 1]] += dt * let * (said - ref[modal.CHAIN[:, 1]]) / SLEW_S
    knee = modal.FLEX[1] * ref[modal.CHAIN[:, 1]]
    behind = np.sqrt(np.maximum(modal.SHANK ** 2 + modal.THIGH ** 2 + 2.0 * modal.SHANK * modal.THIGH * np.cos(knee)
                                - high ** 2, 0.0))
    lean = np.arctan2(-behind, high) + np.arctan2(modal.THIGH * np.sin(knee), modal.SHANK + modal.THIGH * np.cos(knee))
    aim = fr[modal.PITCH0][0] + modal.PITCH_RAD * out[latent.REF][modal.PITCH]
    chain = np.stack([modal.FLEX[0] * (knee - lean + aim), modal.FLEX[1] * knee,
                      (1.0 - bed) * modal.FLEX[2] * lean + bed * laid[modal.CHAIN[:, 2]]], axis=1)
    # both legs at once, each element's ops as a leg's alone (their joints apart): 20 numpy calls where 10
    joints, on = LEG_JOINTS, let[:, None]
    target = (1.0 - bed[:, None]) * ref[joints] + bed[:, None] * laid[joints]
    target[:, 2:5] = chain
    ref[joints] += dt * on * (target - ref[joints]) / SLEW_S
    ref[joints] = np.clip(on * ref[joints] + (1.0 - on) * q[joints], q_lo[joints], q_hi[joints])
    posture = posture.copy()
    posture[joints] = on * ref[joints] + (1.0 - on) * posture[joints]
    return high, ref, posture, freed * (1.0 - bed), 1.0 - window(emancipated, SHED)


def compiled():
    """Each kernel here its C (`tools.cores.bodied`), keyed as her session holds it (`machine.law.motion.laid`)."""
    from tools.cores import bodied
    return {'leave': bodied.on_shapes(leave, 'leave', tuple(range(8))),
            'em_asked': bodied.on_args(asked, 'em_asked', tuple(range(7))),
            'em_released': bodied.on_args(released, 'em_released', tuple(range(14))),
            'em_opening': bodied.on_args(opening, 'em_opening', tuple(range(19)))}
