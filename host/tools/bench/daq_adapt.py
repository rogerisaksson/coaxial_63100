"""The board filtering harder when the link cannot keep up.

A ladder of whole chains (boxcar, coefficients, decimation; decimating
harder without redesigning lets a fold in) goes down to the board, which
climbs it as its ring fills and comes back as the link catches up. A slow
link then costs bandwidth, not records. The test: the host stops reading,
then reads hard.

    python tools/bench/daq_adapt.py [--starve 6]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial import Coaxial63100  # noqa: E402
from coaxial.acquire import bessel  # noqa: E402
from coaxial.errors import RigError  # noqa: E402


def show(daq, what):
    """Where the board is on the ladder, and what its ring holds."""
    st = daq.state()
    capacity = st.get('capacity') or 1
    held = st.get('available') or 0
    bar = int(round(12.0 * held / capacity))
    print('  %-22s rung %d/%d  [%s%s] %4d/%-4d  dropped %-5d moves %d'
          % (what, st.get('rung', 0), max(1, st.get('rungs', 1)) - 1, '#' * bar,
             '-' * (12 - bar), held, capacity, st.get('dropped', 0), st.get('rung_changes', 0)))


def ladder(daq, names, args):
    """Whole chains, each slower than the last, for the measured sweep rate and this link."""
    fs = daq.sweep_rate(names)
    daq.shape()
    daq.configure(names, accumulate=1, digital=True)
    carries = daq.state()['max_rate_hz']
    rungs = bessel.ladder(fs=fs, max_rate_hz=carries, rungs=args.rungs, order=args.order)
    print('\n-- the ladder, %.0f sweeps a second and a link that carries %d --' % (fs, carries))
    for n, chain in enumerate(rungs):
        print('  rung %d: boxcar %4d x %-3d -> %6.1f records/s, cutoff %5.1f Hz, alias %.1f dB'
              % (n, chain['boxcar'], chain['decimate'], chain['out_rate'], chain['cutoff'],
                 chain['worst_alias_db']))
    return rungs


def climb(daq, names, rungs):
    """The task started on the ladder's first rung, free to climb."""
    daq.ladder(rungs)
    daq.configure(names, accumulate=rungs[0]['boxcar'], digital=True, adapt=True)
    daq.ladder(rungs)                   # configure forgets it: the stride moved
    daq.start()


def drain(daq, seconds, label):
    """Read hard for `seconds`, showing the ring."""
    began = time.time()
    while time.time() - began < seconds:
        for _ in range(20):
            if not daq.acquire():
                break
        show(daq, '%s %.1fs' % (label, time.time() - began))
        time.sleep(0.3)


def starve(daq, seconds):
    """Read nothing for `seconds`, showing the ring fill."""
    began = time.time()
    while time.time() - began < seconds:
        time.sleep(0.4)
        show(daq, 'starved %.1fs' % (time.time() - began))


def arguments(argv):
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--channels', default='Phase U,NTC,DC bus')
    p.add_argument('--rungs', type=int, default=4)
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--starve', type=float, default=4.0, help='seconds nobody reads')
    p.add_argument('--recover', type=float, default=6.0, help='seconds of reading hard after')
    return p.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    names = args.channels.split(',')
    try:
        with Coaxial63100(port=args.port, power_afe=True,
                          simulated_device=args.simulated) as rig:
            print('link: %s' % rig.origin.label)
            daq = rig.board.daq                # the ladder is the board's
            daq.stop()                         # a dead session's task refuses the rest

            climb(daq, names, ladder(daq, names, args))
            print('\n-- reading, so the ring stays empty --')
            drain(daq, 2.0, 'draining')
            print('\n-- the bottle: %.0f s with nobody reading --' % args.starve)
            starve(daq, args.starve)
            print('\n-- the fibre: reading hard again --')
            drain(daq, args.recover, 'draining')

            final = daq.state()
            daq.stop()
            daq.shape()
            print('\n  climbed and fell %d times, %d dropped in all'
                  % (final.get('rung_changes', 0), final.get('dropped', 0)))
            return 0
    except RigError as exc:
        print('  the board refused: %s' % exc)
        return 1


if __name__ == '__main__':
    sys.exit(main())
