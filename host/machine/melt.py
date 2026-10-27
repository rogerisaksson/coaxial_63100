"""Her legs' springs melted by the share of her weight her text tells off her soles: the kernel's
`bind-stiffness-to-support-share` on her loop.

    gain, melt = melt.melted(melt, share, was, dt)     # her legs' gain on their springs (LEGGED)
"""
import numpy as np

from machine import acts, kernel, ortho

#: Her legs' springs by the share of her weight borne off her soles (the kernel's `bind-stiffness-to-support-share`):
#: in dB as that share, MELT_DB borne wholly, back within MELT_S as it unloads; her legs' modes, a furniture's
#: post-contact gain's (`acts.LEGS`). The share is her text's (`observer.said`), as its phase blends it: her feet need
#: not bear, she falls onto what is to bear her as she falls onto a swinging foot. Weighed as felt, her stiff legs
#: bore her on her squat's height while her hands bore 0.04-0.23 - under the 0.18 that counts them -, then let go at
#: once and dropped her 21 cm in 0.4 s.
_MELT = kernel.node('bind-wrench-driven-homotopy', 'impedance-melt')
MELT_DB, MELT_S = _MELT['full-support-atten-db'], 1e-3 * _MELT['unloaded-rebound-rate-ms']
LEGGED = np.array(sorted({ortho.MODES.index(m[len('mode_'):]) for node in acts.LEGS for m in acts.NODES[node]}))


def melted(melt, share, was, dt):
    """(her legs' gain on their springs, the melt kept) of the share of her weight told off her soles `share`, it
    `was` the pass before, `dt` s on (MELT_DB, MELT_S): her legs melt as it grows and stay so as it holds; as it
    falls - her text handing her back to her soles - they are back within MELT_S. Melted by the share as it fell,
    her legs were short of their push as her rise's thrust began: the low chair's rise 2 of 7 where 6; back as
    the share felt off her soles fell, every flicker of the seat's touch stiffened them and no sit held."""
    grows, falls = float(share > was), float(share < was)
    melt = grows * min(share, 1.0) + (1.0 - grows - falls) * melt + falls * melt * (1.0 - min(1.0, dt / MELT_S))
    return 10.0 ** (MELT_DB * melt / 20.0), melt
