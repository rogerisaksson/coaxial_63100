"""The kernel's Step 4 on the robot's loop: its legs' reach, its slide on what bears it, its load told.

    somatic.asked(g, s, out, told, ask)    # a pass: the step's edge, slide, fz_cap and slid_cap asks laid

somatic_interaction_core.lisp. Reachability (`modulate-kinematic-reachability`): each leg's reach, its hip to
its sole over the robot's leg's length (LEG_M), rho; from RHO[0] to RHO[1] its sole's centre of
pressure goes onto its ball (the step's `edge`), by the share of the robot the furniture is told to bear -
standing, rho 1 and nothing borne, its soles are its own. The Coulomb slide
(`drive-tangential-coulomb-slide`): the robot's text's slide (`ortho.SLIDE`, m/s along its way,
forward +) names its regime, the robot's pressing contacts' normal load capped at that regime's share of the
robot's weight (CAPS: its sit's ingress, deeper, and its egress) and the slide they may take; the
slide itself the robot's step feels and follows (`wrench._patch`, `rounds.pushed`) - a point's velocity asked
of it lifted the robot off the seat, a drag assumed of a stuck seat fought it (seat findings). What
bears the robot off its soles is capped at its told share of what is told (`ortho.MANIFOLDS`: a pan's
wrench homotopy; nothing told, no cap), never under what the robot's balance needs of it (`lever`): a rise's
lean so hands the robot to its soles as its centre of mass comes over its heels;
uncapped, the seat bore all of it through the lean and the dock's lean never ended.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
# np.clip's own ufunc, its Python layers left out: the same bits, 3.7 -> 1.3 us a call
from numpy._core.umath import clip as CLIP  # pyright: ignore[reportAttributeAccessIssue]

from machine.texts import kernel, ortho
from machine.law import impedance, latent, modal, stride
from machine.moves import seated
from machine.senses.observer import TINY
from machine.stack.rounds import ROOM
from machine.body.figure import ANKLE_H, SHANK, THIGH

_REACH = kernel.node('modulate-kinematic-reachability', 'reachability')['equinus-morph-window']
#: The robot's leg's length, hip to sole, m; the reach the equinus morphs over, of it.
LEG_M = ANKLE_H + THIGH + SHANK
RHO = tuple(_REACH['rho-bounds'])
#: The robot's sit's ingress and its egress (`bind-funnel-transition-executors`): the Coulomb slide's regime
#: each way.
INGRESS, EGRESS = (kernel.transition(a, b, 'coulomb-slide')
                   for a, b in ((':bipedal-upright', ':ischial-seated'), (':ischial-seated', ':bipedal-upright')))
#: A pressing contact's load while the robot slides over it, of the robot's weight: sliding backward (deeper),
#: forward.
CAPS = np.array([INGRESS['normal-load-governor']['cap-fz-ratio'], EGRESS['normal-load-governor']['cap-fz-ratio']])
#: The egress slide along the robot's way, m/s: forward off the seat through a rise's lean (`acts`).
EGRESS_M_S = float(EGRESS['tangent-velocity-attractor'][0])
#: The seated butt-scoot's clock, turns a minute (`xform-locomotion-seated-butt-scoot`): the robot's heels step in
#: turn at it.
ROW_BPM = float(kernel.said('xform-locomotion-seated-butt-scoot', 'bpm'))


def asked(g, s, out, told, ask):
    """`ask` with the step's `edge` rolled onto the ball as each leg reaches, `slide` the robot's text's along
    the robot's way (world: the slide its contacts may take), `fz_cap`, what bears its told
    share of its weight, no less than its balance needs of it (`lever`), and `slid_cap`, a
    sliding contact's load's cap, that slide's regime's share (said, the cap stepped 260 -> 214 N on a seat
    feeling 190-270 and the lean struck), `hand_cap` what the robot's hands bear's, their told share - folded
    on the floor all of it, sat there (`g['sat']`) no more than its balance needs of them
    (`propped`) or their told share: `s` its sensed state (hips, feet, com, contacts), `out` the
    robot's latent's, `told` what bears it as told (`latent.MANIFOLDED`), the furniture's its shares
    between the robot's soles' and its hands' (`ortho`); and its fists kept on the seat they
    are told to bear on (`seated.kept`, `g['fists']` their latch)."""
    fr = g['fr']
    way, weight = fr[modal.ROT].reshape(3, 3)[stride.ALONG], fr[modal.WEIGHT]
    edge, ask['slide'], ask['fz_cap'], ask['hand_cap'], ask['slid_cap'] = g.get('capped', capped)(
        s['hips'], s['feet'][0]['sole'], s['feet'][1]['sole'], told, np.asarray(ask['edge'], float), fr[modal.ROT],
        out[latent.SLID], weight, g['sat'], lever(s, way, weight), propped(s, way))
    ask['edge'] = tuple(edge)
    ask['posture'], g['fists'] = g.get('kept', seated.kept)(g['y'], g['fr'], float(told[ortho.PALMAR]), ask['posture'], s['q'],
                                                        g.setdefault('fists', np.zeros(14)))
    # something bearing the robot now off its soles (`borne`'s prop at impedance.PROP_ON), not its
    # support's latch: the way chosen takes a swing to its closed chains only lying on it (`chain.chained`)
    ask['propped'] = float(s.get('prop', (0.0,))[0]) >= impedance.PROP_ON
    # the twist told the robot (WASD) where no program plays: the closed chains' step carries its pelvis along
    # it, the robot's heels stepping in turn - the crawl take's knees alternate (r -0.63) - at the seated butt-scoot's
    # clock, the one in its swing free for the robot's posture to set back (held, one stroke stalled at 0.235 m, its
    # limbs at their range)
    told = g.get('played') is None and any(float(v) != 0.0 for v in g.get('twist', ()))
    now = g.get('t', 0.0)
    phase, last = g.get('_row', (0.0, now))
    phase = (phase + (now - last) * ROW_BPM / 60.0) % 1.0 if told else 0.0
    g['_row'] = (phase, now)
    # none told, the robot's text's slide over what bears it along its way (its egress) the
    # robot's sled's twist, driven only as far as its soles do not yet bear what its legs weigh
    # (`seated.RESTS`): the closed chains' step drives what it sits on at it - the upright step's slide frees
    # its friction alone, 1.2 cm in 5.2 s -; driven on the robot's low chair's bearing soles it pushed the robot off
    # its seat
    soles = (g['loads'][0] + (0.0 + g['loads'][1])) / weight     # numpy's sum of the two, in its order
    driven = float(np.asarray(ask['slide'], float) @ way) * min(max(1.0 - soles / (2.0 * seated.RESTS), 0.0), 1.0)
    ask['twist'] = tuple(g['twist']) if told else (driven, 0.0, 0.0)
    ask['free'] = (told and phase >= 0.5, told and phase < 0.5)
    return ask


def capped(hips, sole0, sole1, told, edge, rot, along, weight, sat, lever_, propped_):
    """(each sole's edge (2,), the slide the robot's contacts may take (world), fz_cap, hand_cap, slid_cap):
    `asked`'s arithmetic of its law's arrays alone - `lever_` and `propped_` what bears it must take of its
    weight -, numpy's, so compiled (`motion.laid`'s g['capped']): in numpy its ask's caps took 70 of `asked`'s
    100 us."""
    rho = np.linalg.norm(hips - np.stack([sole0, sole1]), axis=1) / LEG_M
    equinus = told[ortho.FURNITURE].sum() * np.clip((rho - RHO[0]) / (RHO[1] - RHO[0]), 0.0, 1.0)
    local, said = np.zeros(3), told.sum()
    local[stride.ALONG] = along
    told_cap = np.where(said > 0.0, weight * told[ortho.FURNITURE].sum() / np.maximum(said, TINY), np.inf)
    told_hand = np.where(said > 0.0, weight * told[ortho.PALMAR] / np.maximum(said, TINY), 0.0)
    braced = np.maximum(told_hand, propped_ * weight)
    return (np.clip(edge + equinus, -1.0, 1.0), rot.reshape(3, 3).T @ local, np.maximum(told_cap, lever_ * weight),
            np.where(rho.mean() < THIGH / LEG_M, sat * braced + (1.0 - sat) * weight * (told[ortho.PALMAR] > 0.0),
                     told_hand),
            np.where(np.abs(along) > 0.0, np.where(along > 0.0, CAPS[1], CAPS[0]) * weight, np.inf))


def propped(s, way):
    """The share of the robot's weight its hands must take for its balance once sat, by the lever's
    rule: its capture point along its `way` behind its seat's contacts (`borne`'s body
    patches and its told seat lying on the floor, `emancipation.lying`) toward its hands' middle -
    the robot's pendulum's omega over its seat's height; all of it with no seat or hands. Folded on the
    floor the robot's hands took up to all of it as it sat too, carried it a millimetre over
    its seat and it bore 0 N; by the robot's centre of mass alone they let it roll back over the
    robot's buttocks onto the floor."""
    from machine.senses.borne import CARPAL
    seat = [c['p'] for c in s.get('contacts', ()) if c.get('patch', 0) < CARPAL]
    hands = [c['p'] for c in s.get('contacts', ()) if c.get('carpal')]
    if not len(seat) or not len(hands):
        return float(len(hands) > 0)
    middle = _middle(seat)
    at, palm = way @ middle, way @ _middle(hands)
    c, v = np.asarray(s['com'], float), np.asarray(s['vcom'], float)
    xi = c + v / np.sqrt(modal.G / max(float(c[1]) - float(middle[1]), ANKLE_H))
    return float(CLIP((at - way @ xi) / np.maximum(at - palm, 1e-3), 0.0, 1.0))


def lever(s, way, weight):
    """The share of the robot's weight the furniture bearing it off its soles must take for the
    robot's balance, by the lever's rule: its centre of mass along its `way` between its points'
    middle and the robot's soles' polygon's rear edge (ROOM[1] behind their middle), less what its hands
    bear (`borne.CARPAL`) of its `weight`; 0 with nothing borne, or its centre of mass over the
    robot's soles. Capped under it the step planned the robot's soles 20/19 N where 250 came and let it
    drop."""
    contacts = [c for c in s.get('contacts', ()) if not c.get('carpal')]
    hands = sum(max(float(c['f'][1]), 0.0) for c in s.get('contacts', ()) if c.get('carpal')) / weight
    if not len(contacts):
        return 0.0
    seat = way @ _middle([c['p'] for c in contacts])
    rear = way @ _middle([f['sole'] for f in s['feet']]) - ROOM[1]
    gap = rear - seat
    need = np.where(gap > 0.0, CLIP((rear - way @ np.asarray(s['com'])) / np.maximum(gap, TINY), 0.0, 1.0), 0.0)
    return np.maximum(need - hands, 0.0)


def _middle(points):
    """`points`' mean as numpy's `np.mean(points, axis=0)` takes it, without its layers: their sum a row at a time
    over their count - its wrappers were 4 us of each of the five a law pass seated."""
    return np.add.reduce(np.array(points), axis=0) / len(points)
