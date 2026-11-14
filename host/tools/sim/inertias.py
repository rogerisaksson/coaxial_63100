#!/usr/bin/env python3
"""Its figure's inertias as MuJoCo compiles them here: each body's iquat and inertia, bit for bit.

    python tools/sim/inertias.py              # a body a line: its iquat and principal inertia as hex
    python tools/sim/inertias.py --values     # the same as floats

Two hosts' lines diffed tell whether its figure is the same bits on both: its principal frames are its own
arithmetic (`mjcf.principal`, a Jacobi of SWEEPS), given MuJoCo to keep - its own diagonalisation of a full
inertia parted an Arm's aarch64 build from an x86's.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def lines(values=False):
    """A line a body of its law's model (`wbc.Body`): its index, iquat and inertia, hex or floats."""
    import numpy as np
    from machine.stack import wbc
    m = wbc.Body().m
    out = []
    for body in range(m.nbody):
        q, i = np.asarray(m.body_iquat[body], float), np.asarray(m.body_inertia[body], float)
        out.append('%d %s' % (body, ' '.join(repr(float(x)) for x in np.r_[q, i]) if values
                              else q.tobytes().hex() + ' ' + i.tobytes().hex()))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--values', action='store_true', help='floats, not their bits')
    args = p.parse_args(argv)
    print('\n'.join(lines(args.values)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
