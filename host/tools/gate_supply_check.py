"""Is the gate drivers' supply actually up while AFE_ON is low?

THE CATCH. `Vgate` (PA5, 47k+10k over 10k from +15V7, ratio 6.70) is an ADC
channel, and the ADC's reference is fed by AFE_ON. So Vgate can only be read
while the AFE is ON - which is exactly when the inversion says the drivers are
unpowered. The repo's only Vgate measurement, 0.35 V, was taken that way and
therefore says nothing about the other state.

THE WAY ROUND: +15V7 has decoupling. If the supply is up while the AFE is off,
those capacitors are charged, and switching the AFE on and reading immediately
shows the charge as a decay before it settles.

    supply UP while AFE off   ->  first reading HIGH, then falling
    supply DOWN throughout    ->  first reading is the rest level, flat

It is an indirect measurement and does not prove a voltage level - it tells two
states apart. That is enough for the question being asked.

NOTE 2026-08-28: the STO chain also gates DCDC_ENABLE, so AFE_ON low is
necessary but not sufficient. A flat result means "no charge seen", which can
be either the STO chain holding the supply down or the AFE never having been
off - check for a stale process holding the port before believing it.
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from coaxial import Coaxial63100
from coaxial.errors import NoReplyError, RigError

CHANNEL = 'Vgate'


def volts(rig, name=CHANNEL):
    """Volts at the pin for a named channel, or None if the link stays quiet."""
    for _ in range(6):
        try:
            for row in rig.board.analog.read_all(nr_of_samples=1)['channels']:
                if row['signal'] == name:
                    return row['volts_at_pin']
            raise SystemExit('no channel is called %r - check 0x6D kind 0'
                             % name)
        except (NoReplyError, RigError):
            time.sleep(0.3)
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--off-seconds', type=float, default=8.0,
                   help='how long the AFE is held off before reading back')
    p.add_argument('--samples', type=int, default=12)
    a = p.parse_args()

    with Coaxial63100(port=a.port) as rig:
        rig.board.afe.enable()
        time.sleep(1.0)
        rest = volts(rig)
        print('rest level, AFE on throughout : %.3f V' % rest)

        print('switching the AFE off for %.0f s (the drivers should then have '
              'supply) ...' % a.off_seconds)
        rig.board.afe.disable()
        time.sleep(a.off_seconds)

        rig.board.afe.enable()          # no settle sleep: we want the first
        t0 = time.time()
        got = []
        for _ in range(a.samples):
            v = volts(rig)
            if v is None:
                break
            got.append((time.time() - t0, v))

        print('\nafter switching the AFE back on:')
        for dt, v in got:
            print('  %6.3f s   %.3f V   %+.3f V against the rest level'
                  % (dt, v, v - rest))

        if not got:
            raise SystemExit('no readings - the link stayed quiet')
        peak = max(v for _, v in got)
        rise = peak - rest
        print('\nhighest over the rest level: %+.3f V' % rise)
        if rise > 0.10:
            print('THE SUPPLY WAS UP while the AFE was off: %.3f V at the pin'
                  % peak)
            print('is %.1f V on +15V7 through the divider\'s 6.70.'
                  % (peak * 6.70))
        else:
            print('NO CHARGE SEEN. Either the supply was never up, or it had')
            print('discharged before the first reading - the first sample came')
            print('%.3f s after switching on.' % got[0][0])


if __name__ == '__main__':
    main()
