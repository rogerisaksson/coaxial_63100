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
from numpy._core.umath import clip as CLIP

from machine import kernel, latent, modal, ortho, stride
from machine.observer import TINY
from machine.rounds import ROOM
from machine.figure import ANKLE_H, SHANK, THIGH

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


def asked(g, s, out, told, ask):
    """`ask` with the step's `edge` rolled onto the ball as each leg reaches, `slide` her text's along her way
    (world: the slide her contacts may take), `fz_cap`, what bears her told share of her weight, no less than her
    balance needs of it (`lever`), and `slid_cap`, a sliding contact's load's cap, that slide's regime's share
    (said, the cap stepped 260 -> 214 N on a seat feeling 190-270 and the lean struck), `hand_cap` what her hands
    bear's, their told share - folded on the floor all of her, sat there (`g['sat']`) no more than her balance needs
    of them (`propped`) or their told share: `s` her sensed state (hips, feet, com, contacts), `out` her latent's, `told` what bears
    her as told (`latent.MANIFOLDED`), the furniture's its shares between her soles' and her hands' (`ortho`)."""
    rho = np.linalg.norm(np.asarray(s['hips']) - np.array([f['sole'] for f in s['feet']]), axis=1) / LEG_M
    equinus = float(told[ortho.FURNITURE].sum()) * CLIP((rho - RHO[0]) / (RHO[1] - RHO[0]), 0.0, 1.0)
    ask['edge'] = tuple(CLIP(np.asarray(ask['edge'], float) + equinus, -1.0, 1.0))
    rot = g['fr'][modal.ROT].reshape(3, 3)
    along, weight, said = out[latent.SLID], g['fr'][modal.WEIGHT], told.sum()
    local = np.zeros(3)
    local[stride.ALONG] = along
    ask['slide'] = rot.T @ local
    told_cap = np.where(said > 0.0, weight * told[ortho.FURNITURE].sum() / np.maximum(said, TINY), np.inf)
    ask['fz_cap'] = np.maximum(told_cap, lever(s, rot[stride.ALONG], weight) * weight)
    told_hand = np.where(said > 0.0, weight * told[ortho.PALMAR] / np.maximum(said, TINY), 0.0)
    braced = np.maximum(told_hand, propped(s, rot[stride.ALONG]) * weight)
    ask['hand_cap'] = np.where(rho.mean() < THIGH / LEG_M,
                               g['sat'] * braced + (1.0 - g['sat']) * weight * float(told[ortho.PALMAR] > 0.0), told_hand)
    ask['slid_cap'] = np.where(abs(along) > 0.0, np.where(along > 0.0, CAPS[1], CAPS[0]) * weight, np.inf)
    return ask


def propped(s, way):
    """The share of her weight her hands must take for her balance once sat, by the lever's rule: her capture point
    along her `way` behind her seat's contacts (`borne`'s body patches and her told seat lying on the floor,
    `emancipation.lying`) toward her hands' middle - her pendulum's omega over her seat's height; all of it with no
    seat or hands. Folded on the floor her hands took up to all of her as she sat too, carried her a millimetre over
    her seat and it bore 0 N; by her centre of mass alone they let her roll back over her buttocks onto the floor."""
    from machine.borne import CARPAL
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
