"""Her legs' springs melted by the share of her weight told off soles told nothing or but to rest.

    melt.step(g, out, told, weighed, dt)                # a pass: her legs' springs in `out` melted

The kernel's `bind-stiffness-to-support-share` on her loop.
"""
import math

from machine.texts import acts, kernel, ortho
from machine.law import composer, latent
from machine.moves.seated import POPLITEAL, RESTS

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


def melting(g, _out, told, _weighed, _dt):
    """Whether `step` works this pass, as its own zeros say: a share told off her soles while her soles are told none,
    her soles told some but no more than twice what her lower legs weigh resting, or her melt or her rest kept off zero.
    Else its share and its rest are 0, its melt held at 0 and her springs times 1.0 - nothing: its rest is no step
    (`composer`). Told nothing - her walk's -, it had been reckoned at work, 34 us a law pass."""
    said, soles = max(float(told.sum()), 1.0), float(told[0])
    return ((float(told[1:].sum()) != 0.0 and soles <= 0.0) or 0.0 < soles <= 2.0 * RESTS * said
            or g.get('melt', 0.0) != 0.0 or g.get('rested', 0.0) != 0.0
            or g.get('melt_was', 0.0) != 0.0 or g.get('rested_was', 0.0) != 0.0)


@composer.capability(melting, lambda *_a: None)
def step(g, out, told, weighed, dt):
    """Her legs' springs in her latent's `out` melted by her text's share off soles it tells nothing (`told`,
    `latent.MANIFOLDED`), `g`'s melt kept, `dt` s on: normalized (`observer.said`), the share leapt to 1 at its
    blend's first pass and she fell 22 cm in 0.25 s. Soles told no more than her lower legs weigh resting (RESTS a
    foot), resting on what bears them as her comparator feels (its plantar gate) from a seat no higher than her
    popliteal height (`g['seat']`, `maneuver.said`: over it her feet hang off the floor), melt by what it feels bear
    her off them (`weighed`, `observer.weighed`: none till it latches, none as it lets go), held as `melted` holds
    while her text alone moves her and back within MELT_S while a joint program does (`g['played']`): stiff there,
    on her low chair her soles bore 0.5 of her and her seat less than they; held under one, a bed's lie pushed off
    on legs at 0.6 of their gain and she stood where she was to lie. Melted on a higher seat's edge before its
    program, her 0.47 m chair's vault lifted her 5.5 cm for 7 and she sat back after it; the bed's lie (3 mm over)
    threw her 1.5 m off its edge."""
    said = max(float(told.sum()), 1.0)
    share = float(told[1:].sum()) / said * float(told[0] <= 0.0)
    rests = (float(0.0 < told[0] <= 2.0 * RESTS * said) * float(g.get('seat', math.inf) <= POPLITEAL)
             * float(g['observer']['gate'][0]) * float(weighed[1:].sum()))
    free, rested = float(g.get('played') is None) * float(told[1:].sum() > 0.0), g.get('rested', 0.0)
    gain, g['melt'] = melted(g.get('melt', 0.0), share, g.get('melt_was', 0.0), dt)
    g['rested'] = (free * melted(rested, rests, g.get('rested_was', 0.0), dt)[1]
                   + (1.0 - free) * rested * (1.0 - min(1.0, dt / MELT_S)))
    g['melt_was'], g['rested_was'] = share, rests
    gain = min(gain, 10.0 ** (MELT_DB * g['rested'] / 20.0))
    out[latent.KS][LEGGED] *= gain
    out[latent.DS][LEGGED] *= math.sqrt(gain)
