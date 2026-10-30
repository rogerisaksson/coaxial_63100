"""Her legs' springs melted by the share of her weight told off soles her text asks nothing of.

    melt.step(g, out, told, dt)                         # a pass: her legs' springs in `out` melted

The kernel's `bind-stiffness-to-support-share` on her loop.
"""
import math

from machine import acts, kernel, latent, ortho

#: In dB as that share, MELT_DB borne wholly, back within MELT_S as it unloads; her legs' modes (`acts.LEGS`).
#: Unmelted, tipped back from her squat onto the floor she rolled over her heels to -128 deg; melted where her soles
#: were told a share too, the low chair's sit held 1 of 7 where 6. Weighed as felt, her stiff legs bore her on her
#: squat's height while her hands bore 0.04-0.23 - under the 0.18 that counts them -, then dropped her 21 cm in 0.4 s.
_MELT = kernel.node('bind-wrench-driven-homotopy', 'impedance-melt')
MELT_DB, MELT_S = _MELT['full-support-atten-db'], 1e-3 * _MELT['unloaded-rebound-rate-ms']
LEGGED = sorted({ortho.MODES.index(m[len('mode_'):]) for node in acts.LEGS for m in acts.NODES[node]}
                - {ortho.MODES.index('pelvis_turn'), ortho.MODES.index('limb_lift')})


def melted(melt, share, was, dt):
    """(her legs' gain on their springs, the melt kept) of the share told off her soles `share`, it `was` the pass
    before, `dt` s on: her legs melt as it grows and stay so as it holds; as it falls they are back within MELT_S.
    Melted by the share as it fell, her legs were short of their push as her rise's thrust began: the low chair's
    rise 2 of 7 where 6; back as the share felt off her soles fell, every flicker of the seat's touch stiffened them
    and no sit held."""
    grows, falls = float(share > was), float(share < was)
    melt = grows * min(share, 1.0) + (1.0 - grows - falls) * melt + falls * melt * (1.0 - min(1.0, dt / MELT_S))
    return 10.0 ** (MELT_DB * melt / 20.0), melt


def step(g, out, told, dt):
    """Her legs' springs in her latent's `out` melted by her text's share off soles it tells nothing (`told`,
    `latent.MANIFOLDED`), `g`'s melt kept, `dt` s on: normalized (`observer.said`), the share leapt to 1 at its
    blend's first pass and she fell 22 cm in 0.25 s."""
    share = float(told[1:].sum()) / max(float(told.sum()), 1.0) * float(told[0] <= 0.0)
    gain, g['melt'] = melted(g.get('melt', 0.0), share, g.get('melt_was', 0.0), dt)
    g['melt_was'] = share
    out[latent.KS][LEGGED] *= gain
    out[latent.DS][LEGGED] *= math.sqrt(gain)
