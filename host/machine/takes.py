"""The mocap takes as rows for her loop: a take's modes at its frames, laid once, said as rows.

    wire, end_ms = takes.wire('stand_to_sit')  # the take's rows (ortho.ROW), its posture's modes said

FILE holds each take's times (s, at her size) and her eleven modes at them, read off the take as
`modal.reading` reads hers (`tools/sim/stunts.py --reference`; the takes lie beside the repo).
STREAMED are the modes a take says, her posture's: the legs' shortening, the centre of mass over
the soles along, the trunk's pitch; her stance's and her arms' stay her balance's (the human's
feet, pelvis list, heels and arm swing each felled her when said). A take enters over ENTER_S
from wherever she is, its first row's blend; its frames then follow at their own pace, each
blended over one, contiguous, so her loop draws them as one curve (`latent.CONTIG`). The bench
tells a take (`scenarios._told`); her texts stay the lisp's: told the bed's takes she sat at
its edge as the human did and collapsed back from there (stunts.md).
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
