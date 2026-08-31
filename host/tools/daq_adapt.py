"""The board filtering harder when the link cannot keep up.

A ladder of whole chains is sent down; the board climbs it when its ring
fills and comes back down when the link has caught up. What a slow link
costs is then BANDWIDTH rather than records - the passband narrows, the
filter follows it, and nothing is dropped or aliased on the way.

Every rung is a complete design - boxcar, coefficients, decimation -
because decimating harder without redesigning is exactly how a fold gets
in. The board cannot design anything; it chooses between designs.

    cd host
    python tools/daq_adapt.py               # watch it climb and fall
    python tools/daq_adapt.py --starve 6    # a longer bottle

The test is the starve: the host stops reading, the ring fills, and the
board has to answer. Then the host reads hard and it should come back.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, bessel                     # noqa: E402
from coaxial.errors import RigError                          # noqa: E402


def show(daq, what):
    """One line of where the board is, and what it is holding."""
    st = daq.state()
    capacity = st.get('capacity') or 1
    held = st.get('available') or 0
    bar = int(round(12.0 * held / capacity))
    print('  %-22s rung %d/%d  [%s%s] %4d/%-4d  dropped %-5d moves %d'
          % (what, st.get('rung', 0), max(1, st.get('rungs', 1)) - 1,
             '#' * bar, '-' * (12 - bar), held, capacity,
             st.get('dropped', 0), st.get('rung_changes', 0)))
    return st


def sweep_rate(daq, names, records=300, timeout=6.0):
    """What the poll loop manages, from a finite run - a free-running one
    would measure the rate the board substitutes for the link instead."""
    daq.shape()
    daq.configure(names, accumulate=1, digital=True, records=records,
                  interval_us=0)
    began = time.time()
    daq.start()
    while time.time() - began < timeout and not daq.state()['done']:
        time.sleep(0.005)
    span = time.time() - began
    st = daq.state()
    daq.stop()
    return (st['produced'] + st['dropped']) / max(span, 1e-6)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--channels', default='Phase U,NTC,DC bus')
    p.add_argument('--rungs', type=int, default=4)
    p.add_argument('--order', type=int, default=4)
    p.add_argument('--starve', type=float, default=4.0,
                   help='seconds the host stops reading, so the ring fills')
    p.add_argument('--recover', type=float, default=6.0,
                   help='seconds it reads hard afterwards')
    args = p.parse_args(argv)

    device = Coaxial63100(port=args.port, power_afe=True,
                          simulated_device=bool(args.simulated))
    daq = device.daq
    try:
        daq.open()
    except RigError as exc:
        print('  could not open the board: %s' % exc)
        return 1
    print('link: %s' % device.origin.label)

    names = args.channels.split(',')
    try:
        fs = sweep_rate(daq, names)
        daq.shape()
        daq.configure(names, accumulate=1, digital=True)
        carries = daq.state()['max_rate_hz']
        rungs = bessel.ladder(fs=fs, max_rate_hz=carries, rungs=args.rungs,
                              order=args.order)

        print('\n-- the ladder, %.0f sweeps a second and a link that carries '
              '%d --' % (fs, carries))
        for n, chain in enumerate(rungs):
            print('  rung %d: boxcar %4d x %-3d -> %6.1f records/s, cutoff '
                  '%5.1f Hz, alias %.1f dB'
                  % (n, chain['boxcar'], chain['decimate'], chain['out_rate'],
                     chain['cutoff'], chain['worst_alias_db']))

        daq.ladder(rungs)
        daq.configure(names, accumulate=rungs[0]['boxcar'], digital=True,
                      adapt=True)
        daq.ladder(rungs)              # configure forgets it: the stride moved
        daq.start()

        print('\n-- reading, so the ring stays empty --')
        began = time.time()
        while time.time() - began < 2.0:
            daq.acquire()
            time.sleep(0.01)
        show(daq, 'draining')

        print('\n-- the bottle: %.0f s with nobody reading --' % args.starve)
        began = time.time()
        while time.time() - began < args.starve:
            time.sleep(0.4)
            show(daq, 'starved %.1fs' % (time.time() - began))

        print('\n-- the fibre: reading hard again --')
        began = time.time()
        while time.time() - began < args.recover:
            for _ in range(20):
                if not daq.acquire():
                    break
            show(daq, 'draining %.1fs' % (time.time() - began))
            time.sleep(0.3)

        final = daq.state()
        daq.stop()
        print('\n  climbed and fell %d times, %d dropped in all'
              % (final.get('rung_changes', 0), final.get('dropped', 0)))
        return 0
    except RigError as exc:
        print('  the board refused: %s' % exc)
        return 1
    finally:
        try:
            daq.stop()
            daq.shape()
        except RigError:
            pass
        device.close()


if __name__ == '__main__':
    sys.exit(main())
