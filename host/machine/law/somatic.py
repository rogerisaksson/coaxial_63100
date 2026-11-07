"""The kernel's Step 4 on her loop: her legs' reach, her slide over what bears her, its load as told.

    somatic.asked(g, s, out, told, ask)    # a pass: the step's edge, slide, fz_cap and slid_cap asks laid

somatic_interaction_core.lisp. Reachability (`modulate-kinematic-reachability`): each leg's reach, its hip to its sole over her leg's length
(LEG_M), rho; from RHO[0] to RHO[1] her sole's centre of pressure goes onto its ball (the step's `edge`), by
the share of her the furniture is told to bear - standing, rho 1 and nothing borne, her soles are her own. The
Coulomb slide (`drive-tangential-coulomb-slide`): her text's slide (`ortho.SLIDE`, m/s along her way, forward
+) names its regime, her pressing contacts' normal load capped at that regime's share of her weight (CAPS: her
sit's ingress, deeper, and its egress) and the slide they may take; the slide itself her step feels and follows
(`wrench._patch`, `rounds.pushed`) - a point's velocity asked of it lifted her off the seat, a drag assumed of a
stuck seat fought her (seat findings). What bears her off her soles is capped at its told share of what is told
(`ortho.MANIFOLDS`: a pan's wrench homotopy; nothing told, no cap), never under what her balance needs of it
(`lever`): a rise's lean so hands her to her soles as her centre of mass comes over her heels; uncapped, the seat
bore all of her through the lean and the dock's lean never ended.
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
#: Her leg's length, hip to sole, m; the reach the equinus morphs over, of it.
LEG_M = ANKLE_H + THIGH + SHANK
RHO = tuple(_REACH['rho-bounds'])
#: Her sit's ingress and its egress (`bind-funnel-transition-executors`): the Coulomb slide's regime each way.
INGRESS, EGRESS = (kernel.transition(a, b, 'coulomb-slide')
                   for a, b in ((':bipedal-upright', ':ischial-seated'), (':ischial-seated', ':bipedal-upright')))
#: A pressing contact's load while she slides over it, of her weight: sliding backward (deeper), forward.
CAPS = np.array([INGRESS['normal-load-governor']['cap-fz-ratio'], EGRESS['normal-load-governor']['cap-fz-ratio']])
#: The egress slide along her way, m/s: forward off the seat through a rise's lean (`acts`).
EGRESS_M_S = float(EGRESS['tangent-velocity-attractor'][0])
#: The seated butt-scoot's clock, turns a minute (`xform-locomotion-seated-butt-scoot`): her heels step in turn at it.
ROW_BPM = float(kernel.said('xform-locomotion-seated-butt-scoot', 'bpm'))


def asked(g, s, out, told, ask):
    """`ask` with the step's `edge` rolled onto the ball as each leg reaches, `slide` her text's along her way
    (world: the slide her contacts may take), `fz_cap`, what bears her told share of her weight, no less than her
    balance needs of it (`lever`), and `slid_cap`, a sliding contact's load's cap, that slide's regime's share
    (said, the cap stepped 260 -> 214 N on a seat feeling 190-270 and the lean struck), `hand_cap` what her hands
    bear's, their told share - folded on the floor all of her, sat there (`g['sat']`) no more than her balance needs
    of them (`propped`) or their told share: `s` her sensed state (hips, feet, com, contacts), `out` her latent's, `told` what bears
    her as told (`latent.MANIFOLDED`), the furniture's its shares between her soles' and her hands' (`ortho`); and her
    fists kept on the seat they are told to bear on (`seated.kept`, `g['fists']` their latch)."""
    fr = g['fr']
    way, weight = fr[modal.ROT].reshape(3, 3)[stride.ALONG], fr[modal.WEIGHT]
    edge, ask['slide'], ask['fz_cap'], ask['hand_cap'], ask['slid_cap'] = g.get('capped', capped)(
        s['hips'], s['feet'][0]['sole'], s['feet'][1]['sole'], told, np.asarray(ask['edge'], float), fr[modal.ROT],
        out[latent.SLID], weight, g['sat'], lever(s, way, weight), propped(s, way))
    ask['edge'] = tuple(edge)
    ask['posture'], g['fists'] = g.get('kept', seated.kept)(g['y'], g['fr'], float(told[ortho.PALMAR]), ask['posture'], s['q'],
                                                        g.setdefault('fists', np.zeros(14)))
    # something bearing her now off her soles (`borne`'s prop at impedance.PROP_ON), not her support's latch: the way
    # chosen takes a swing to her closed chains only lying on it (`chain.chained`)
    ask['propped'] = float(s.get('prop', (0.0,))[0]) >= impedance.PROP_ON
    # the twist told her (WASD) where no program plays: the closed chains' step carries her pelvis along it, her heels
    # stepping in turn - the crawl take's knees alternate (r -0.63) - at the seated butt-scoot's clock, the one in its
    # swing free for her posture to set back (held, one stroke stalled at 0.235 m, its limbs at their range)
    told = g.get('played') is None and any(float(v) != 0.0 for v in g.get('twist', ()))
    now = g.get('t', 0.0)
    phase, last = g.get('_row', (0.0, now))
    phase = (phase + (now - last) * ROW_BPM / 60.0) % 1.0 if told else 0.0
    g['_row'] = (phase, now)
    # none told, her text's slide over what bears her along her way (her egress) her sled's twist, driven only as far as
    # her soles do not yet bear what her legs weigh (`seated.RESTS`): the closed chains' step drives what she sits on
    # at it - the upright step's slide frees its friction alone, 1.2 cm in 5.2 s -; driven on her low chair's
    # bearing soles it pushed her off its seat
    soles = float(np.sum(g['loads'])) / weight
    driven = float(np.asarray(ask['slide'], float) @ way) * min(max(1.0 - soles / (2.0 * seated.RESTS), 0.0), 1.0)
    ask['twist'] = tuple(g['twist']) if told else (driven, 0.0, 0.0)
    ask['free'] = (told and phase >= 0.5, told and phase < 0.5)
    return ask


def capped(hips, sole0, sole1, told, edge, rot, along, weight, sat, lever_, propped_):
    """(each sole's edge (2,), the slide her contacts may take (world), fz_cap, hand_cap, slid_cap): `asked`'s arithmetic
    of her law's arrays alone - `lever_` and `propped_` what bears her must take of her weight -, numpy's, so compiled
    (`motion.laid`'s g['capped']): in numpy her ask's caps took 70 of `asked`'s 100 us."""
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
    """The share of her weight her hands must take for her balance once sat, by the lever's rule: her capture point
    along her `way` behind her seat's contacts (`borne`'s body patches and her told seat lying on the floor,
    `emancipation.lying`) toward her hands' middle - her pendulum's omega over her seat's height; all of it with no
    seat or hands. Folded on the floor her hands took up to all of her as she sat too, carried her a millimetre over
    her seat and it bore 0 N; by her centre of mass alone they let her roll back over her buttocks onto the floor."""
    from machine.senses.borne import CARPAL
    seat = [c['p'] for c in s.get('contacts', ()) if c.get('patch', 0) < CARPAL]
    hands = [c['p'] for c in s.get('contacts', ()) if c.get('carpal')]
    if not len(seat) or not len(hands):
        return float(len(hands) > 0)
    at, palm = way @ np.mean(seat, axis=0), way @ np.mean(hands, axis=0)
    c, v = np.asarray(s['com'], float), np.asarray(s['vcom'], float)
    xi = c + v / np.sqrt(modal.G / max(float(c[1]) - float(np.mean(seat, axis=0)[1]), TINY))
    return float(CLIP((at - way @ xi) / np.maximum(at - palm, TINY), 0.0, 1.0))


def lever(s, way, weight):
    """The share of her weight the furniture bearing her off her soles must take for her balance, by the lever's
    rule: her centre of mass along her `way` between its points' middle and her soles' polygon's rear edge (ROOM[1]
    behind their middle), less what her hands bear (`borne.CARPAL`) of her `weight`; 0 with nothing borne, or her
    centre of mass over her soles. Capped under it the step planned her soles 20/19 N where 250 came and let her
    drop."""
    contacts = [c for c in s.get('contacts', ()) if not c.get('carpal')]
    hands = sum(max(float(c['f'][1]), 0.0) for c in s.get('contacts', ()) if c.get('carpal')) / weight
    if not len(contacts):
        return 0.0
    seat = way @ np.mean([c['p'] for c in contacts], axis=0)
    rear = way @ np.mean([f['sole'] for f in s['feet']], axis=0) - ROOM[1]
    gap = rear - seat
    need = np.where(gap > 0.0, CLIP((rear - way @ np.asarray(s['com'])) / np.maximum(gap, TINY), 0.0, 1.0), 0.0)
    return np.maximum(need - hands, 0.0)
