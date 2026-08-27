#!/usr/bin/env python3
"""How far the board's counter drifts against UTC, with the PC in between.

    python tools/clock_drift.py --seconds 600

Two oscillators and one reference. The board is measured against this PC
because that is the only clock it can reach; the PC is measured against NTP
because it is not a reference either - measured here with the Windows time
service stopped, it sat 931 ms off UTC and free-running.

So the board's error against UTC is the sum, and reporting the first
without the second is reporting the difference between two unqualified
oscillators as if it were one of them.

NTP offset noise on a LAN is about a millisecond, so resolving 1 ppm needs
roughly a thousand seconds between the two ends. Shorter runs bound it
rather than measure it, and this says which it managed.
"""
import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100                           # noqa: E402

NTP_EPOCH = 2208988800


def ntp_offset(host, rounds=8, timeout=3.0):
    """Best-of-N SNTP queries: this PC's clock against the server's.

    Min-filtered on the round trip, the way every NTP client does it - the
    shortest exchange has the least queueing in it, and its remaining error
    is half the asymmetry rather than all of the delay.
    """
    best = None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        for _ in range(rounds):
            first = time.time()
            sock.sendto(b'\x1b' + 47 * b'\0', (host, 123))
            data, _ = sock.recvfrom(48)
            last = time.time()

            server_rx = (struct.unpack('!I', data[32:36])[0]
                         + struct.unpack('!I', data[36:40])[0] / 2 ** 32
                         - NTP_EPOCH)
            server_tx = (struct.unpack('!I', data[40:44])[0]
                         + struct.unpack('!I', data[44:48])[0] / 2 ** 32
                         - NTP_EPOCH)
            trip = (last - first) - (server_tx - server_rx)
            offset = ((server_rx - first) + (server_tx - last)) / 2
            if best is None or trip < best[1]:
                best = (offset, trip)
    finally:
        sock.close()
    return best


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=600.0)
    parser.add_argument('--ntp', default='pool.ntp.org')
    parser.add_argument('--port', default='COM4')
    args = parser.parse_args(argv)

    with Coaxial63100(port=args.port, power_afe=False) as daq:
        if daq.simulated:
            print('no board answered - nothing to measure')
            return 1

        first_ntp, first_trip = ntp_offset(args.ntp)
        first_sync = daq.set_time_from_pc(seconds=3.0)
        started = time.time()
        print('start   PC vs NTP %+.3f ms (trip %.1f ms)   board vs PC %+.2f ppm'
              % (first_ntp * 1e3, first_trip * 1e3, first_sync.error_ppm))
        print('waiting %.0f s ...' % args.seconds)

        time.sleep(args.seconds)

        last_ntp, last_trip = ntp_offset(args.ntp)
        last_sync = daq.set_time_from_pc(seconds=3.0)
        elapsed = time.time() - started

    # The PC's own rate error: how much its offset from UTC grew.
    pc_ppm = (last_ntp - first_ntp) / elapsed * 1e6
    board_vs_pc = last_sync.error_ppm
    resolution = 1e-3 / elapsed * 1e6

    print('end     PC vs NTP %+.3f ms (trip %.1f ms)   board vs PC %+.2f ppm'
          % (last_ntp * 1e3, last_trip * 1e3, board_vs_pc))
    print()
    print('over %.0f s, and NTP offset noise of about a millisecond puts the '
          'floor at %.1f ppm:' % (elapsed, resolution))
    print('   PC against UTC      %+8.2f ppm' % pc_ppm)
    print('   board against PC    %+8.2f ppm' % board_vs_pc)
    print('   board against UTC   %+8.2f ppm' % (board_vs_pc + pc_ppm))
    if abs(pc_ppm) < resolution:
        print('   the PC term is inside the floor, so it bounds rather than '
              'measures')
    return 0


if __name__ == '__main__':
    sys.exit(main())
