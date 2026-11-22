"""The calf's two boards wired: each a branch off the collar's T-cross, its phases on their lugs.

The calf's two inverters stand back to back in the cage, their component sides 2 mm apart. Turned alike, each
board's phase lugs would stand against the other's; half a turn apart (TURNS) each board's lugs face the other's low
parts. The trunk comes down the knee's hose to the T-cross on the collar's flange (`knee_hose`); its branch goes
through the flange into the cage and along it, inside the wall, to both boards' rims - a black pipe - and in at the
bearing the fitting stands at: the power pair and the shield's lead a lug each, the RS-485's pair in and out again
(`boards.wired`, a `tee`'s). Each board's three phase leads go from their lugs onto the band round its own motor.
"""
import math

from tools.render import boards
from tools.render import knee_mount as km
from tools.render import tees

#: How the calf's two boards lie turned about their axis, rad: half a turn apart.
TURNS = {'pitch': 0.0, 'roll': math.pi}
#: The branch's pipe's radius along the cage, and a phase slot's width across its lug (1 mm air a side), m.
BRANCH_R, SLOT_M = 0.003, 0.007


def slots(lay):
    """[(bearing, x0, x1, width)] of the six phase slots in the calf's cage (`knee_clamped.laid`), rad about the
    calf's axis from +y toward +z and m: each board's three from its inner face out to its motor's end of the cage;
    () with no model."""
    c0, c1 = lay['cage'][:2]
    out = []
    for which, span in (('roll', (c0, lay['roll'][0]['board'][1])), ('pitch', (lay['pitch'][0]['board'][0], c1))):
        out += [((TURNS[which] + math.atan2(z, y)) % (2.0 * math.pi), span[0], span[1], SLOT_M)
                for y, z, _top in boards.screws()[0]]
    return tuple(out)


def leads(lay, spot):
    """[(tri, colour)] in the knee's frame, the leg straight: the calf's two boards' leads as `knee_clamped.laid`
    (`lay`) lays the drives - their phases' and their branches of the bus, in at the bearing about the calf's axis
    the collar's fitting `spot` (y, z) stands at - and the branch's pipe from the collar's flange to the boards."""
    import numpy as np
    cy, cz = lay['pitch'][1]
    entry, out = math.atan2(spot[1] - cz, spot[0] - cy), []
    for which, band in (('pitch', lay['pitch_band'][-1][2]), ('roll', lay['collar'][0][2])):
        stack = lay[which][0]
        (b0, b1, _r), m0 = stack['board'], stack['motor'][0]
        laid = (b0, b1, (cy, cz), b1 if m0 >= b1 - 1e-9 else b0, TURNS[which])
        out += boards.leads(*laid, band) + boards.wired(*laid, entry, None, 0.0, True)
    far = lay['pitch'][0]['board'][0] + 0.5 * (lay['pitch'][0]['board'][1] - lay['pitch'][0]['board'][0])
    return out + km._rod(np.array([lay['collar'][2][0], spot[0], spot[1]]), np.array([far, spot[0], spot[1]]), BRANCH_R,
                         tees.PIPE_INK)
