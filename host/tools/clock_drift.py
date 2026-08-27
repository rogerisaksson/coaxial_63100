#!/usr/bin/env python3
"""How far the board's counter runs against UTC.

    python tools/clock_drift.py --seconds 600

Three clocks and one reference. The board is reached through this PC
because that is the only clock it can talk to, and this PC is not a
reference either: measured 2026-08-27, six minutes after W32Time had
synced, it sat 947 ms behind UTC and was losing a further 25 ppm - Windows
had declined to step it, the offset being inside the 1 s
MaxAllowedPhaseOffset, and slewing was not catching up.

`sync(reference='utc')` measures that over the same window and takes it
out. Printed here with the floor beside it, because a rate is only worth
reading past the noise of the reference it was measured against: NTP is
good to about a millisecond a side, so 1 ppm needs a thousand seconds.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                           # noqa: E402
from coaxial.clock import NTP_SERVER, ntp_offset           # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--seconds', type=float, default=600.0)
    parser.add_argument('--ntp', default=NTP_SERVER)
    args = parser.parse_args()

    try:
        offset, trip = ntp_offset(args.ntp)
        print('this PC is %+.3f ms from UTC (%s, trip %.1f ms)'
              % (offset * 1e3, args.ntp, trip * 1e3))
    except Exception as why:
        print('no UTC reference: %s' % why)

    # The clock needs no analog supply, and leaving AFE_ON alone is one
    # less thing drawing current across a ten-minute window.
    with Coaxial63100(port=args.port, power_afe=False) as daq:
        print('%s, measuring for %.0f s ...' % (daq, args.seconds))
        sync = daq.set_time_from_pc(seconds=args.seconds,
                                    ntp_server=args.ntp)

    print()
    print('reference   %s%s' % (sync.reference,
                                '  (' + sync.note + ')' if sync.note else ''))
    print('SYSCLK      %.3f MHz measured, %.3f nominal'
          % (sync.hz / 1e6, sync.nominal_hz / 1e6))
    print('board       %+.2f ppm vs %s' % (sync.error_ppm, sync.reference))
    if sync.pc_ppm is not None:
        print('this PC     %+.2f ppm vs UTC' % sync.pc_ppm)
    print('floor       %.2f ppm - anything smaller is the reference, not the '
          'board' % sync.floor_ppm)
    print('verdict     %s'
          % ('measured' if abs(sync.error_ppm) > sync.floor_ppm
             else 'below the floor: bounded, not measured'))


if __name__ == '__main__':
    main()
