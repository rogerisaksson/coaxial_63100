#!/usr/bin/env python3
"""Walk the dead time down until the supply says stop.

    python tools/deadtime_trim.py                    # 46 down to 26 ns
    python tools/deadtime_trim.py --from 40 --to 30
    python tools/deadtime_trim.py --skew 2           # trim lead against lag

WHAT IT MEASURES. Dry switching draws almost nothing, so an over-current with
no load is shoot-through: both FETs of a leg conducting through the dead time.
The bench supply's OCP is the instrument - it trips, the DC link collapses,
and the step that did it is the one the board could not hold.

HOW IT SEES A TRIP. By the OBSERVER'S UPTIME going backwards, which is the
only thing that survives the event. The DC link was tried first and does not
work: the sample is taken with the stage down, and by then the supply has
recovered - it read 24.86 V at the very step that had just reset the board.
Measured 2026-08-29 with the bench OCP at 300 mA, and the reset was visible
only because the thermal nodes had gone to zero.

The uptime is not a threshold on anything (invariant 10): a counter that was
larger a minute ago and is smaller now says the board restarted, which is a
fact about the board rather than a judgement about volts.

LEAD AND LAG. `--skew` is DTG counts, positive lengthening the transition the
counter reaches counting up and shortening the other by the same, so the pair
still averages the dead time asked for. It exists because the two transitions
of a leg need not be symmetric - one gate turns off through a different
impedance than the other turns on. Nothing here has been on a scope.

The observer runs beside it: `drivers` and `phases` climbing faster at one
step than the last is the bridge heating from something the previous step did
not have, which is what approaching the limit looks like from inside.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import Coaxial63100, scaling                    # noqa: E402
from demos import SETTLE_S              # the board's own settle
from screen import say, steady                                       # noqa: E402



def sample(rig, params):
    """Stand down, measure, and hand back what the board said.

    The stage goes down FIRST and comes back up LAST: AFE_ON high removes the
    gate drivers' supply, so the only safe moment to have both is with MOE
    clear.
    """
    steady(rig.gates.disarm)
    got = {}
    if steady(rig.board.afe.enable) is not None:
        time.sleep(SETTLE_S)
        table = steady(rig.board.analog.read_all, nr_of_samples=32)
        if table is not None:
            for row in table['channels']:
                if row['signal'] in ('DC bus', '+5V'):
                    to = scaling.converter(row['unit'], row['differential'],
                                           signal=row['signal'], params=params)
                    got[row['signal']] = to(row['mean_raw'])
        state = steady(rig.board.thermal.state)
        spend = steady(rig.board.thermal.budget)
        if state is not None:
            got['ntc'] = state['ntc']
        if state is not None:
            got['uptime'] = state['seconds']
        if spend is not None:
            got['drivers'] = spend['used']['drivers']
            got['phases'] = spend['used']['phases']
        steady(rig.board.afe.disable)
        time.sleep(SETTLE_S)
    return got


def step(rig, params, nanoseconds, skew, seconds, every, legs):
    """One dead time, held for `seconds`. Returns the samples taken."""
    steady(rig.board.afe.disable)
    if steady(rig.gates.arm, bypass_sto=True, ignore_interlock=True) is None:
        return None

    held = steady(rig.gates.dead_time, nanoseconds, skew=skew)
    # ONE LEG IS THE SENSITIVE TEST. Dry switching draws almost nothing, so
    # with the supply's OCP wound down, a single leg's shoot-through is the
    # whole of the current there is - three legs would share the trip between
    # them and hide which one did it.
    load = dict(('Phase %s' % leg, 0.5 if leg in legs else 0.0)
                for leg in ('U', 'V', 'W'))
    if steady(rig.write, analog=load) is None:
        steady(rig.gates.disarm)
        return None

    taken, started = [], time.monotonic()
    while time.monotonic() - started < seconds:
        time.sleep(min(every, seconds - (time.monotonic() - started)))
        got = sample(rig, params)
        got['at'] = time.monotonic() - started
        got['held_ns'] = (held or {}).get('nanoseconds')
        taken.append(got)
        if time.monotonic() - started < seconds:
            steady(rig.gates.arm, bypass_sto=True, ignore_interlock=True)
            steady(rig.write, analog=load)

    steady(rig.write, analog=dict.fromkeys(load, 0.0))
    steady(rig.gates.disarm)
    return taken


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--from', dest='high', type=float, default=46.0)
    p.add_argument('--to', dest='low', type=float, default=26.0)
    p.add_argument('--step', type=float, default=4.0)
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--every', type=float, default=20.0)
    p.add_argument('--skew', type=int, default=0)
    p.add_argument('-P', '--phases', default='U',
                   help='legs to switch; one is the sensitive test')
    a = p.parse_args()

    with Coaxial63100(port=a.port, power_afe=False) as rig:
        params = steady(rig.board.analog.scaling)
        say('ok', 'trim', 'leg %s, %.0f down to %.0f ns in %.0f ns steps, '
            '%.0f s each' % (a.phases, a.high, a.low, a.step, a.seconds))

        wanted, uptime = a.high, None
        print()
        print('%7s %7s %6s  %8s %7s  %7s %7s'
              % ('ask ns', 'held', 'at s', 'DC bus', 'up s', 'drivers',
                 'phases'))

        while wanted >= a.low - 0.01:
            taken = step(rig, params, wanted, a.skew, a.seconds, a.every,
                         a.phases)
            if taken is None:
                say('fail', '%.0f ns' % wanted, 'the stage would not arm')
                break

            for got in taken:
                bus, up = got.get('DC bus'), got.get('uptime')
                print('%7.0f %7s %6.0f  %8s %7s  %6.1f %% %6.1f %%'
                      % (wanted, got.get('held_ns', '--'), got['at'],
                         '%.2f V' % bus if bus is not None else '--',
                         up if up is not None else '--',
                         100.0 * got.get('drivers', 0.0),
                         100.0 * got.get('phases', 0.0)))

                if up is not None and uptime is not None and up < uptime:
                    say('fail', '%s ns' % got.get('held_ns', wanted),
                        'the board restarted - uptime %d s after %d, so the '
                        'supply tripped under it' % (up, uptime))
                    return 1
                if up is not None:
                    uptime = up

            wanted -= a.step

        say('ok', 'trim', 'the whole sweep held')
    return 0


if __name__ == '__main__':
    sys.exit(main())
