"""The robot's upper body yield and the loop's ask in C: support.couple and impedance.ask, bit for bit."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np

from machine.senses import edge


def step_modal(g, s, y, y_in, out):
    """The robot's upper body's yield and the loop's ask: `support.couple` and `impedance.ask` in C bit
    for bit (`g['couple']`, `g['impedance']`, `laid`)."""
    qd_buf = g.get('_qd_j')
    if qd_buf is not None and type(y_in) is np.ndarray and y_in.dtype == np.float64:
        qd = np.take(y_in[edge.QD], g['_joints_i'], out=qd_buf)
    else:
        qd = y_in[edge.QD][g['joints']]
        if not (type(qd) is np.ndarray and qd.dtype == np.float64 and qd.flags.c_contiguous):
            qd = np.ascontiguousarray(qd, dtype=np.float64)
    sq = s['q']
    if not (type(sq) is np.ndarray and sq.dtype == np.float64 and sq.flags.c_contiguous):
        sq = np.ascontiguousarray(sq, dtype=np.float64)
    return g['couple'](sq, qd, g['ref_q'], g['ref_v'], g['eso'][2], out), g['impedance'](out, y, g['fr'], g['a'])
