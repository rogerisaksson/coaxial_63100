"""Switch the gate drivers, now. Nothing else.

    python tools/switch.py                       # 50 %, 60 s, all three legs
    python tools/switch.py -d 0.25 -s 120        # one duty, two minutes
    python tools/switch.py --sweep 5,95 -p 10    # triangle, 10 s per sweep
    python tools/switch.py -P U,V                # only those legs
    python tools/switch.py --stop                # stop a run, disarmed

**Turns the AFE off and bypasses the STO break before arming**, because on
this bench board AFE_ON high takes the supply off the gate drivers - so
measuring and switching are mutually exclusive and switching is what this
tool is for. It reads no temperature and settles no baseline.

`--stop` exists because killing a switching run from outside leaves the
stage armed. It drops a file the run watches for, so the run exits through
its own `finally` and the gates go down.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from coaxial import Coaxial63100
from coaxial.errors import RigError

PHASES = ('U', 'V', 'W')
STOP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '.switch-stop')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('-d', '--duty', type=float, default=0.5)
    p.add_argument('-s', '--seconds', type=float, default=60.0)
    p.add_argument('-P', '--phases', default='U,V,W')
    p.add_argument('--sweep', help='LO,HI in percent, triangular')
    p.add_argument('-p', '--period', type=float, default=10.0,
                   help='seconds per sweep, with --sweep')
    p.add_argument('--stop', action='store_true',
                   help='ask a running switch.py to disarm and exit')
    p.add_argument('--keep-afe', action='store_true',
                   help='leave the AFE on, which leaves the drivers unpowered')
    p.add_argument('--keep-break', action='store_true',
                   help='leave the STO break in circuit')
    p.add_argument('--interlock', action='store_true',
                   help='honour the arming interlock')
    a = p.parse_args()

    if a.stop:
        open(STOP_FILE, 'w').close()
        print('stop requested - the run disarms and exits within 200 ms')
        return

    legs = [x.strip().upper() for x in a.phases.split(',') if x.strip()]
    for leg in legs:
        if leg not in PHASES:
            raise SystemExit('unknown phase %r - pick from %s'
                             % (leg, ', '.join(PHASES)))
    lo = hi = None
    if a.sweep:
        lo, hi = (float(x) / 100.0 for x in a.sweep.split(','))

    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)

    def write(rig, duty):
        rig.write(analog={'Phase ' + leg: duty for leg in legs})

    rig = Coaxial63100(port=a.port).open()
    try:
        if not a.keep_afe:
            rig.board.afe.disable()
        rig.arm_gate_drivers(bypass_sto=not a.keep_break,
                             ignore_interlock=not a.interlock)
        what = ('sweep %.0f-%.0f %% every %.0fs' % (lo * 100, hi * 100, a.period)
                if a.sweep else '%.0f %%' % (a.duty * 100))
        print('LIVE: %s at %s for %.0f s   (stop: python tools/switch.py --stop)'
              % ('+'.join(legs), what, a.seconds), flush=True)

        start = time.time()
        write(rig, lo if a.sweep else a.duty)
        while True:
            elapsed = time.time() - start
            if elapsed >= a.seconds:
                break
            if os.path.exists(STOP_FILE):
                os.remove(STOP_FILE)
                print('stopped after %.1f s' % elapsed, flush=True)
                break
            if a.sweep:
                x = (elapsed / a.period) % 1.0
                duty = lo + (hi - lo) * (2 * x if x < 0.5 else 2 * (1 - x))
                try:
                    write(rig, duty)
                except RigError:
                    pass          # a lost frame is not a reason to stop
                time.sleep(0.05)
            else:
                time.sleep(0.2)
    finally:
        try:
            write(rig, 0.0)
        except RigError:
            pass
        try:
            rig.disarm_gate_drivers()
        except RigError:
            pass
        rig.close()
        print('off', flush=True)


if __name__ == '__main__':
    main()
