"""The mocap takes as her texts: a take's modes at its frames, laid once, said as rows.

    wire, end_ms = takes.wire('stand_to_sit')  # the take's rows (ortho.ROW), its posture's modes said
    sprung.entries() includes takes.entries()  # 'lie_bed': sitting on the bed, then lying down

FILE holds each take's times (s, at her size) and her eleven modes at them, read off the take as
`modal.reading` reads hers (`tools/sim/stunts.py --reference`; the takes lie beside the repo).
STREAMED are the modes a take says, her posture's: the legs' shortening, the centre of mass over
the soles along, the trunk's pitch; her stance's and her arms' stay her balance's (the human's
feet, pelvis list, heels and arm swing each felled her when said). A take enters over ENTER_S
from wherever she is, its first row's blend; its frames then follow at their own pace, each
blended over one, contiguous, so her loop draws them as one curve (`latent.CONTIG`). TOLD names
the texts that are takes: a sequence of takes, their rows one after another, a phase name each
(`se2.STUNT_PHASES`) and the name its last row rests at.
"""
import math
import os

from machine import modal, ortho

#: Each take's '<key>_t' and '<key>_z' (frames, modes), at her size.
FILE = os.path.join(os.path.dirname(__file__), 'takes.npz')
#: The modes a take says.
STREAMED = (modal.AXIAL, modal.SAGITTAL, modal.PITCH)
#: A take's rows' sequence id, after the texts' streams (200-205).
TAKE_ID = 300
#: A take's first row's blend, s: the half period of her leg's swing, the window its measures
#: are smoothed over too.
ENTER_S = math.pi * math.sqrt(modal.LEG_M / modal.G)
#: Her texts that are takes: name: (((take, its rows' phase), ..), the last row's phase).
TOLD = {'lie_bed': ((('sitting_on_bed', 'sit_edge'), ('lying_down_from_sitting', 'recline')), 'bed_rest')}


def wire(key, at_ms=0):
    """A take's rows for her loop (`ortho.ROW`) from `at_ms` on: its first row blended over
    ENTER_S, the frames after it at the take's pace, each over one frame; its STREAMED modes
    said, the rest left as they are. And the ms its last frame's blend ends at, where a next
    take's rows join."""
    import numpy as np
    with np.load(FILE) as kept:
        t, z = kept[key + '_t'], kept[key + '_z'].astype(float)
    step = int(round(1e3 * (t[1] - t[0])))
    enter = int(round(1e3 * ENTER_S))
    at = [at_ms] + [at_ms + enter + int(round(1e3 * (t[k] - t[1]))) for k in range(1, len(t))]
    blend = [enter] + [step] * (len(t) - 1)
    rows = []
    for k in range(len(t)):
        p = [math.nan] * ortho.P
        for i in STREAMED:
            p[ortho.Z + i] = float(z[k, i])
        rows.append(ortho.ROW.pack(TAKE_ID, at[k], blend[k], 0, *p))
    return b''.join(rows), at[-1] + step


def told(name):
    """(label, wire, names) of the text `name` (TOLD): its takes' rows one after another, a
    row's phase its take's, the last row's the text's rest."""
    takes, rest = TOLD[name]
    wires, names, at = [], [], 0
    for key, phase in takes:
        w, at = wire(key, at)
        wires.append(w)
        names += [phase] * (len(w) // ortho.ROW.size)
    names[-1] = rest
    return name, b''.join(wires), tuple(names)


def entries():
    """[(label, wire, names)] of every text that is a take (TOLD)."""
    return [told(name) for name in TOLD]
