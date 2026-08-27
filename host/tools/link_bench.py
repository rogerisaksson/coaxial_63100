#!/usr/bin/env python3
"""How close this link runs to its own bitrate.

    python tools/link_bench.py --port COM4

Three shapes, because they cost differently: a ping that carries nothing, an
echo that loads both directions, and the ring's burst, which is one small
request for the largest reply the board sends - the shape a capture actually
runs in.

`floor` is the bitrate alone, `bytes * 10 / baud` for 8N1. Everything above
it is turnaround, driver latency and the gap that ends a frame, and none of
it grows with the payload - which is why a ping is nearly all overhead and a
full block is not. The board's own `max_rate_hz` is built on the same
arithmetic.

Nothing here judges the link. It reports what it measured against what the
bitrate allows.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, bench                     # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--rounds', type=int, default=20,
                        help='transactions per case; the median is reported')
    parser.add_argument('--simulated', action='store_true')
    args = parser.parse_args()

    # No analog supply is needed to measure a wire, and a ten-second run
    # with AFE_ON raised is current drawn for nothing.
    with Coaxial63100(port=args.port, power_afe=False,
                      simulated_device=args.simulated) as daq:
        print(daq)
        results = bench.run(daq.board, rounds=args.rounds)

    print()
    for line in bench.table(results):
        print(line)


if __name__ == '__main__':
    main()
