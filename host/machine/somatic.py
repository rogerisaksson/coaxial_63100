"""The kernel's Step 4 on her loop: her legs' reach, her slide over what bears her.

    somatic.asked(g, s, out, told, ask)    # a pass: the step's edge, slide and fz_cap asks laid

somatic_interaction_core.lisp. Reachability (`modulate-kinematic-reachability`): each leg's reach, its hip to its sole over her leg's length
(LEG_M), rho; from RHO[0] to RHO[1] her sole's centre of pressure goes onto its ball (the step's `edge`), by
the share of her the furniture is told to bear - standing, rho 1 and nothing borne, her soles are her own. The
Coulomb slide (`drive-tangential-coulomb-slide`): her text's slide (`ortho.SLIDE`, m/s along her way, forward
+) the step's pressing contacts' velocity, their normal load capped at CAP of her weight while it moves.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import kernel, latent, modal, stride
from machine.figure import ANKLE_H, SHANK, THIGH

_REACH = kernel.node('modulate-kinematic-reachability', 'reachability')['equinus-morph-window']
#: Her leg's length, hip to sole, m; the reach the equinus morphs over, of it.
LEG_M = ANKLE_H + THIGH + SHANK
RHO = tuple(_REACH['rho-bounds'])
#: A pressing contact's load while she slides over it, of her weight.
CAP = kernel.node('drive-tangential-coulomb-slide', 'coulomb-slide')['normal-load-governor']['cap-fz-ratio']


def asked(g, s, out, told, ask):
    """`ask` with the step's `edge` rolled onto the ball as each leg reaches, `slide` her text's along her way
    (world) and `fz_cap`: `s` her sensed state (hips, feet), `out` her latent's, `told` what bears her as told
    (`latent.MANIFOLDED`)."""
    rho = np.linalg.norm(np.asarray(s['hips']) - np.array([f['sole'] for f in s['feet']]), axis=1) / LEG_M
    equinus = float(told[1:].sum()) * np.clip((rho - RHO[0]) / (RHO[1] - RHO[0]), 0.0, 1.0)
    ask['edge'] = tuple(np.clip(np.asarray(ask['edge'], float) + equinus, -1.0, 1.0))
    local = np.zeros(3)
    local[stride.ALONG] = float(out[latent.SLID])
    ask['slide'] = g['fr'][modal.ROT].reshape(3, 3).T @ local
    ask['fz_cap'] = float(np.where(abs(local[stride.ALONG]) > 0.0, CAP * g['fr'][modal.WEIGHT], np.inf))
    return ask
