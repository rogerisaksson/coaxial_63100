"""Her standard gait and its modifiers: one walk, moved by named amounts, no gait a special.

    row = gait.modified(roll=0.86)   # one row (ortho.ROW) over her walk: its list's amplitude up 0.86
    std = gait.standard()            # the walk's parameters as the lisp's streams fold (stand, sway, step, walk)

A modifier is a named amount on one of the walk's parameters (MODIFIERS): its pace, its stride
(the drive's amplitude), the arms' swing, its sway, the pelvis's roll and turn, the soles' rock,
its crouch, its lean, its width. Laid over the walk she is on, a row says the moved parameters
alone, the standard's value and the amount; the rest stays the walk's. A gait take is its amounts
(`takes.amounts`), a word of her movement language (`tools.sim.normal`) an amount: a woman's
catwalk is roll +0.86, and so modified her walk is named a catwalk - its pelvis rolls 18.8 deg
for the take's 23.3 (sequencer.md). The patch and its modulation, docs/architecture/synth.md.
"""
import math

from machine import modal, ortho

#: A modifier's name and the parameter of the walk it moves.
MODIFIERS = {'pace': ortho.HZ, 'stride': ortho.AMP + modal.DRIVE, 'swing': ortho.AMP + modal.ARMS,
             'sway': ortho.AMP + modal.LATERAL, 'roll': ortho.AMP + modal.LIST, 'turn': ortho.AMP + modal.TURN,
             'rock': ortho.AMP + modal.ROCK, 'crouch': ortho.Z + modal.AXIAL, 'lean': ortho.Z + modal.PITCH,
             'width': ortho.Z + modal.WIDTH}
#: The walk's streams in the order they are said (`sprung.WALK`), folded into the standard.
STREAMS = ('stand', 'sway', 'step', 'walk')
#: A modified row's sequence id, after the texts' streams (200-205) and a take's rows (300).
ROW_ID = 301
#: A modified row's blend, s: the half period of her leg's swing, a take's entering too.
BLEND_S = math.pi * math.sqrt(modal.LEG_M / modal.G)


def said():
    """The walk's parameters (`ortho.P`): each as the last of the walk's streams that says it,
    nan where none does - her loop keeps what she had there, her rest (`modal.rest`) from a stand."""
    import numpy as np
    from machine import latent, sprung
    by = {entry[0]: entry for entry in sprung.entries()}
    std = np.full(ortho.P, math.nan)
    for name in STREAMS:
        for row in np.frombuffer(by[name][1], latent.WIRE):
            given = ~np.isnan(row['p'])
            std[given] = row['p'][given]
    return std


def standard():
    """The walk's parameters (`said`), 0 where none says one."""
    import numpy as np
    return np.nan_to_num(said(), nan=0.0)


def modified(blend_s=BLEND_S, **amounts):
    """One row (`ortho.ROW`) over her walk, blended over `blend_s`: each named modifier's
    parameter at the standard's value plus its amount, the rest unsaid."""
    std = standard()
    p = [math.nan] * ortho.P
    p[ortho.GAIN] = 1.0
    for name, amount in amounts.items():
        p[MODIFIERS[name]] = float(std[MODIFIERS[name]] + amount)
    return ortho.ROW.pack(ROW_ID, 0, int(round(1e3 * blend_s)), 0, *p)
