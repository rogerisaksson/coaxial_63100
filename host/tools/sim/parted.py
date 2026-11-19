#!/usr/bin/env python3
"""A scenario's passes as bits: each law pass's digest of the world, one host's beside another's.

    python tools/sim/parted.py play_walk                  # a line a law pass: its count, its time, its digest
    python tools/sim/parted.py chair --wobble 2e-4 --until 3

Each law pass a digest (blake2b, 8 bytes) of MuJoCo's qpos, qvel, act and ctrl and of the robot's edge's array: two
hosts' lines side by side part at the first pass their arithmetic does - the pass to read for what parted them.
"""
import argparse
import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def digest(*arrays):
    """The arrays' bits, hashed short."""
    import numpy as np
    h = hashlib.blake2b(digest_size=8)
    for a in arrays:
        h.update(np.ascontiguousarray(a, dtype=np.float64).tobytes())
    return h.hexdigest()


def passes(name, wobble=0.0, until=6.0):
    """[(law pass, time, digest)] of scenario `name` laid turned `wobble` rad, to `until` s."""
    from tools.sim import scenarios
    rows, last = [], [None]

    def laid(s):
        g = s['g']
        if g.get('t_law') == last[0]:
            return
        last[0] = g.get('t_law')
        d = g['world'].data
        rows.append((len(rows), round(float(g['t']), 4), digest(d.qpos, d.qvel, d.act, d.ctrl, g['sense'])))
        if g['t'] >= until:
            raise StopIteration

    try:
        scenarios.ran(name, wobble, laid=laid)
    except StopIteration:
        pass
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('name')
    parser.add_argument('--wobble', type=float, default=0.0)
    parser.add_argument('--until', type=float, default=6.0)
    args = parser.parse_args([a for a in (sys.argv[1:] if argv is None else argv) if a != '--'])
    for n, t, d in passes(args.name, args.wobble, args.until):
        print('%d %.4f %s' % (n, t, d))
    return 0


if __name__ == '__main__':
    sys.exit(main())
