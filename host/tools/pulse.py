"""One short pulse between two phases, then off. Nothing else.

    python tools/pulse.py                    # U at 2 %, V held low, W low
    python tools/pulse.py -d 0.05 -H V -L W  # V at 5 % against W
    python tools/pulse.py -n 20 --gap 0.05   # twenty in a row, armed once

The HIGH leg switches at the duty; the LOW leg and the third sit at
zero, which with MOE set is the low-side FET on - so a load between
the two sees the DC link for `duty` of every period. The pulse lasts
as long as the second compare write takes to land: 15.5 ms measured
2026-08-30, ~780 cycles at 50 kHz. It was 110 ms through rig.write(),
whose arm check and period lookup were three state reads at 31 ms
each. The protocol has no cycle-counted burst; ten cycles is firmware
work.

Like switch.py it turns the AFE off and bypasses the STO break before
arming - on this bench board AFE_ON high takes the supply off the gate
drivers - and reads no current: measuring and switching are mutually
exclusive here. It prints the gate state after the pulse and after the
disarm, so a fault latch, an overrun or a gate short shows.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coaxial import Coaxial63100                           # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from switch import PHASES                                  # noqa: E402

#: Seconds a compare write takes to land, measured 14.9-16.0 ms over the
#: probe's COM port: what an --on wait is shortened by.
LANDING = 0.015

SHOWN = ('pwm_enabled', 'sync_armed', 'fault', 'break_bypassed', 'updates',
         'overruns', 'duty', 'pins', 'worst_gap_cycles', 'gate_shorts')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('-d', '--duty', type=float, default=0.02)
    p.add_argument('-H', '--high', default='U', help='the leg that switches')
    p.add_argument('-L', '--low', default='V', help='the leg held low')
    p.add_argument('-n', '--count', type=int, default=1,
                   help='pulses in a row, armed once')
    p.add_argument('--gap', type=float, default=0.05,
                   help='seconds off between pulses')
    p.add_argument('--on', type=float, default=0.0,
                   help='seconds on per pulse - 0 is as short as the link '
                        'allows, about 15 ms; longer waits before the off '
                        'write, with its 15 ms landing counted in')
    a = p.parse_args()
    high, low = a.high.upper(), a.low.upper()
    if high not in PHASES or low not in PHASES or high == low:
        raise SystemExit('pick two different legs from U, V, W')
    zeros = (0, 0, 0)

    rig = Coaxial63100(port=a.port, power_afe=False).open()
    afe_was_on = rig.gates.state()['afe_on']
    try:
        rig.board.afe.disable()
        rig.gates.arm(bypass_sto=True, ignore_interlock=True)
        state = rig.gates.state()
        print('armed, dead time %d ns' % state['deadtime_ns'])
        # The raw compare write, twice, with nothing between: the stage
        # is armed by the line above, and rig.write()'s own arm check is
        # a 31 ms state read the pulse would be spent waiting for.
        ticks = [0, 0, 0]
        ticks[PHASES.index(high)] = int(a.duty * (state['period'] - 1))
        held = []
        for i in range(a.count):
            if i:
                time.sleep(a.gap)
            t0 = time.perf_counter()
            rig.board.gate_drivers.duty(ticks)
            t1 = time.perf_counter()
            if a.on > LANDING:
                # Sleep to 2 ms short, then spin: Windows sleeps in
                # ~15 ms steps, and 100 ms asked for came out 109.
                until = t1 + a.on - LANDING
                if until - time.perf_counter() > 0.002:
                    time.sleep(until - time.perf_counter() - 0.002)
                while time.perf_counter() < until:
                    pass
            rig.board.gate_drivers.duty(zeros)
            held.append(time.perf_counter() - t1)
            if a.count == 1:
                print('%.1f ms to land' % (1000 * (t1 - t0)))
        after = rig.gates.state()
        on = sorted(held)
        print('%s at %.1f %% against %s low, %d pulse%s: on %.1f ms min, '
              '%.1f median, %.1f max - ~%d cycles each at 50 kHz'
              % (high, 100 * a.duty, low, len(on), '' if len(on) == 1 else 's',
                 1000 * on[0], 1000 * on[len(on) // 2], 1000 * on[-1],
                 int(on[len(on) // 2] * 50000)))
        print('after:', {k: after[k] for k in SHOWN})
    finally:
        try:
            rig.board.gate_drivers.duty(zeros)
        except RigError:
            pass
        try:
            rig.gates.disarm()
        except RigError as exc:
            print('disarm:', exc)
        final = rig.gates.state()
        print('disarmed:', {k: final[k] for k in SHOWN})
        if afe_was_on:
            # The way it was found: a thermal view sharing the port
            # went blind for good when the pulse left the AFE off.
            rig.board.afe.enable()
            print('AFE back on')
        rig.close()
        print('off', flush=True)


if __name__ == '__main__':
    main()
