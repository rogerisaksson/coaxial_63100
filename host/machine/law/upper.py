"""Her upper body's yield and the loop's ask in C: support.couple and impedance.ask, bit for bit."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np

from machine.senses import edge


def step_modal(g, s, y, y_in, out):
    """Her upper body's yield and the loop's ask: `support.couple` and `impedance.ask` in C bit
    for bit (`g['couple']`, `g['impedance']`, `laid`)."""
    qd = np.ascontiguousarray(y_in[edge.QD][g['joints']], dtype=np.float64)
    dq = g['couple'](np.ascontiguousarray(s['q'], dtype=np.float64), qd, g['ref_q'], g['ref_v'], g['eso'][2], out)
    return dq, g['impedance'](out, y, g['fr'], g['a'])
