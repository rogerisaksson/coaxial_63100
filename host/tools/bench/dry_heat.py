"""Dry switching's heat on the bench board: the NTC's rise a run, over PWM frequency and dead time.

    python tools/bench/dry_heat.py --port COM3 --points 50:60,25:60     # kHz:dead_ns, rested each
    python tools/bench/dry_heat.py --port COM3 --points 25:60,30:60,35:60 --stairs --limit 60
    python tools/bench/dry_heat.py --port COM3 --points 0:60            # armed at zero: no edge

All three legs at 50 %, so no volts across whatever is on the phases; the break in circuit
(AFE_ON low, PE15 high, the latch cleared). The thermistor needs AFE_ON, which takes this
board's drivers' supply: it is read before a run and 0.6 s after its gates. A frequency is
TIM1's ARR, written over SWD with the stage off - the firmware has no op for it - and put back
at the end; the dead time the gate device's, put back too. A rested run waits for the NTC nearly
still (and under --start), its drift taken off the rise; --stairs runs the points one behind the
other and stops at --limit. Nothing is armed above --volts or 65 C.
"""
import argparse
import re
import subprocess
import sys
import time
from contextlib import suppress

from coaxial import Coaxial63100
from coaxial.errors import RigError
from tools.target import build_and_flash

NTC_MAX_C = 65.0
#: TIM1's ARR and its 50 kHz: centre-aligned, 2 x (ARR + 1) ticks a period.
TIM1_ARR, ARR_50K = 0x4001002C, 2375
#: The reference up behind AFE_ON, and PE15 up behind its fall, s.
AFE_UP_S, PE15_UP_S = 0.6, 0.4
#: A rested start: the NTC's drift under this, K/s, read this far apart, s.
STILL_K_PER_S, STILL_S = 0.01, 15.0


def swd(*args):
    """One STM32_Programmer_CLI call on the running board, its output."""
    programmer = build_and_flash.find_programmer(build_and_flash.toolchain_path())
    if programmer is None:
        raise SystemExit('STM32_Programmer_CLI not found - see setup.ps1')
    done = subprocess.run([programmer, '-c', 'port=SWD', 'mode=HOTPLUG'] + list(args),
                          capture_output=True, text=True, errors='replace', timeout=60)
    return done.stdout + done.stderr


def arr(value=None):
    """TIM1's ARR, written first where one is given."""
    if value is not None:
        swd('-w32', '0x%08X' % TIM1_ARR, '0x%08X' % value)
    found = re.search(r'0x%08X : ([0-9A-Fa-f]{8})' % TIM1_ARR, swd('-r32', '0x%08X' % TIM1_ARR, '4'))
    return int(found.group(1), 16) if found else None


def ntc(rig):
    """(NTC C, link V), the AFE up."""
    return (rig.board.analog.ntc_temperature()['celsius'], rig.board.analog.dcbus_voltage()['volts'])


def rested(rig, under):
    """The NTC nearly still, and under `under` where one is given: its drift, K/s."""
    last = None
    while True:
        now, at = ntc(rig)[0], time.time()
        if last is not None:
            drift = (now - last[0]) / (at - last[1])
            if (under is None or now < under) and abs(drift) < STILL_K_PER_S:
                return drift
        last = (now, at)
        time.sleep(STILL_S)


def arm(rig, khz, dead_ns):
    """The stage on at `khz` and `dead_ns`, 50 % on every leg - or armed at zero for 0 kHz."""
    if rig.drive.state()['stage_enabled']:
        raise SystemExit('the stage is on: no register is written under it')
    want = (ARR_50K + 1) * 50 // khz - 1 if khz else ARR_50K
    if arr(want) != want:
        raise SystemExit('ARR did not take %d' % want)
    rig.gates.configure(dead_time_ns=dead_ns)
    rig.board.afe.off()
    time.sleep(PE15_UP_S)
    if not rig.board.afe.state().get('pe15'):
        raise SystemExit('PE15 low with the AFE off: nothing armed')
    rig.gates.clear()
    rig.gates.on(bypass_sto=False, ignore_interlock=True)
    if khz:
        rig.gates.write(ticks=((want + 1) // 2,) * 3)


def disarm(rig):
    with suppress(RigError):
        rig.gates.write(ticks=(0, 0, 0))
    rig.gates.off()


def main():
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--points', default='50:60', help='kHz:dead_ns, comma separated; 0 kHz no edge')
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--start', type=float, default=None, help='a rested run starts under this, C')
    p.add_argument('--stairs', action='store_true', help='no rest between the points')
    p.add_argument('--limit', type=float, default=NTC_MAX_C, help='--stairs stops at this NTC, C')
    p.add_argument('--volts', type=float, default=36.0, help='the link above which nothing is armed')
    a = p.parse_args()

    points = [tuple(int(x) for x in point.split(':')) for point in a.points.split(',')]
    rig = Coaxial63100(port=a.port, power_afe=True, fallback=False).open()
    armed, was_dead, table = False, None, []
    try:
        was_dead = rig.gates.dead_time()['nanoseconds']
        time.sleep(1.0)
        first = None
        for khz, dead_ns in points:
            drift = 0.0 if (a.stairs and first is not None) else rested(rig, a.start)
            before, link = ntc(rig)
            first = before if first is None else first
            if link > a.volts or before > NTC_MAX_C:
                raise SystemExit('link %.1f V, NTC %.1f C: nothing armed' % (link, before))
            arm(rig, khz, dead_ns)
            armed = True
            began = time.time()
            while time.time() - began < a.seconds:
                if not rig.drive.state()['stage_enabled']:
                    raise SystemExit('the stage went off %.1f s into %d kHz' % (time.time() - began, khz))
                time.sleep(1.0)
            disarm(rig)
            armed = False
            span = time.time() - began
            rig.board.afe.on()
            time.sleep(AFE_UP_S)
            after = ntc(rig)[0]
            took = rig.gates.dead_time()['nanoseconds']
            table.append((khz, took, link, before, after, after - before - drift * span, after - first))
            print('%3d kHz %3d ns %6.2f V: NTC %.2f -> %.2f C, %+.2f K less its drift in %.0f s'
                  % (khz, took, link, before, after, table[-1][5], span), flush=True)
            if a.stairs and after >= a.limit:
                print('enough: %.2f C at %d kHz' % (after, khz))
                break
    finally:
        if armed:
            with suppress(RigError):
                disarm(rig)
        with suppress(RigError):
            if not rig.drive.state()['stage_enabled']:
                arr(ARR_50K)
                if was_dead is not None:
                    rig.gates.configure(dead_time_ns=was_dead)
        with suppress(RigError):
            rig.board.afe.off()
        rig.close()
    print('\n kHz  dead ns  link V  before C  after C  rise K  over the first C')
    for row in table:
        print(' %3d  %7d  %6.2f  %8.2f  %7.2f  %6.2f  %16.2f' % row)
    return 0


if __name__ == '__main__':
    sys.exit(main())
