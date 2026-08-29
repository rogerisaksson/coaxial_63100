"""Four thermal states, one at a time, to equilibrium - for a thermal camera.

The differences between the states isolate each power term on its own, which
is the point: no single measurement can separate the drivers' loss from the
MCU's, but the difference between state 1 and state 4 can.

    1 passive  AFE off -> the drivers HAVE SUPPLY (the gate is inverted), no PWM
    2 afe      AFE on  -> drivers unpowered, sensors alive, no traffic
    3 traffic  AFE on  + DAQ at full tilt and data pumped off the board
    4 switch   AFE off + three legs at 50 %

WHY A FIXED DWELL AND NOT PLATEAU DETECTION
AFE_ON feeds the NTC, so in states 1 and 4 the board's own thermometer is
blind - and those are exactly the states where the drivers have supply. The
same dwell in all four is therefore the only consistent choice: tau is 6.8 min
measured, and 25 minutes is 3.7 tau = 97 % of the way to equilibrium.

TSEN IS NOT USED HERE
The A1335's TSEN measures its own die, not the board, and its self-heating is
reset every time AFE_ON breaks: measured 2026-08-28 it FELL 1.88 K during a
run that warmed the board. It also quantises at 0.125 K. The NTC has 30 mK and
a stable bias (it sits still in the drivers' hot spot), so it is the reference.
The camera is the truth.

The script holds each state and WAITS for a file before moving on, so the
camera measurement can take as long as it takes:

    python tools/thermal_calibrate.py --next
"""
import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from coaxial import Coaxial63100
from coaxial.errors import NoReplyError, RigError
from coaxial.thermal import STATE_IS as WHAT
from coaxial.thermal import STATES, tau_minutes

HERE = os.path.dirname(os.path.abspath(__file__))
NEXT_FILE = os.path.join(HERE, '.calib-next')


#: What the camera should look at, plus the reference the rest is measured
#: against. The last one carries the whole calibration.
POINTS = ('the middle half-bridge', 'buck + LDO', 'the MCU',
          'angle sensor / IMU', 'hot swap / DC link - the mask beside it',
          'DEAD SURFACE - mask well away from anything that warms')

#: How often the AFE may come on for a sample in an AFE-off state. SAMPLING
#: PERTURBS THE STATE - the gate is inverted, so the drivers lose supply for
#: as long as it lasts.
#:
#: Measured 2026-08-28: a sample is 0.42 s, so every 60 s is 0.7 % of the time
#: in the wrong state - nothing against tau 6.8 min, and 25 samples a state
#: instead of 5. Four samples 3 s apart spread 50 mK with no drift.
PEEK_EVERY_S = 60.0

#: How long the reference is given to come up before reading. 300 ms is
#: enough - see the spread above. It is three quarters of the sample's length,
#: so this is where speeding up would gain anything.
PEEK_SETTLE_S = 0.3


def insist(what, tries=8, pause=0.4):
    """Run `what()` until the link answers. Returns (ok, result).

    The link goes quiet now and then - FINDINGS has it open, and 600 requests
    ruled out four causes. A sample every 60 s hits it more often than one
    every 300 s, so every call here has to tolerate silence.
    """
    for _ in range(tries):
        try:
            return True, what()
        except (NoReplyError, RigError):
            time.sleep(pause)
    return False, None


def sensors(rig):
    """(ntc, spread_mK) while the AFE is on, else (None, None)."""
    ok, on = insist(rig.board.afe.is_on)
    if not ok or not on:
        return None, None
    ok, n = insist(rig.board.analog.ntc_temperature)
    return (n['celsius'], n['spread_millikelvin']) if ok else (None, None)


def peek(rig):
    """A sample inside an AFE-off state: on, read, off again.

    Leaves the AFE off, so the state is as it was.

    FAILING TO SWITCH IT BACK OFF IS WORSE THAN FAILING TO READ. A failed read
    costs one data point; a failed switch-off leaves the rig feeding a
    different state from the one it reports, silently. So the switch-off is
    more stubborn than the read, and raises if it still fails.
    """
    ok, was_on = insist(rig.board.afe.is_on)
    if not ok:
        return None, None
    try:
        if not was_on:
            if not insist(rig.board.afe.enable)[0]:
                return None, None
            time.sleep(PEEK_SETTLE_S)
        return sensors(rig)
    finally:
        if not was_on:
            if not insist(rig.board.afe.disable, tries=20, pause=0.5)[0]:
                raise SystemExit(
                    'COULD NOT SWITCH THE AFE BACK OFF after a sample. The '
                    'state is now wrong and everything measured after this is '
                    'worthless - stopping rather than reporting bad numbers.')


def announce(state, minutes, ntc, spread):
    bar = '=' * 68
    print('\n%s' % bar, flush=True)
    print('  READY FOR THE CAMERA - state %d/%d: %s'
          % (STATES.index(state) + 1, len(STATES), state.upper()), flush=True)
    print('  %s' % WHAT[state], flush=True)
    print('  held %.1f min (%.1f tau)' % (minutes, minutes / tau_minutes()),
          flush=True)
    if ntc is not None:
        print('  the board\'s own: NTC %.2f C  (spread %.0f mK)'
              % (ntc, spread), flush=True)
    else:
        print('  the board\'s own: blind (AFE off feeds the sensors)',
              flush=True)
    print('  measure:', flush=True)
    for i, point in enumerate(POINTS, 1):
        print('    %d. %s' % (i, point), flush=True)
    print('  next:  python tools/thermal_calibrate.py --next', flush=True)
    print('%s\n' % bar, flush=True)


def hold(state, dwell_s, poll_s=30.0):
    """Set the state, hold it, read at the end."""
    if state == 'switch':
        # Switch in chunks and sample between: switch.py owns the port while
        # it runs, so a sample means stopping it. A chunk is 5 min and a
        # sample ~1 s, so 0.3 % of the time not switching - against tau
        # 6.8 min that does not move the equilibrium.
        chunk = min(300.0, dwell_s)
        done = 0.0
        while done < dwell_s:
            this = min(chunk, dwell_s - done)
            subprocess.run([sys.executable, 'tools/switch.py', '--port', PORT,
                            '-P', 'U,V,W', '-d', '0.50', '-s', str(int(this))],
                           cwd=os.path.dirname(HERE), check=True)
            done += this
            # power_afe=False: the rig must not switch the AFE on at open. If
            # it does, the AFE is ON when the next chunk starts and the gate
            # drivers then have no supply - the gate is inverted. peek()
            # drives it itself and leaves it off.
            with Coaxial63100(port=PORT, power_afe=False) as rig:
                ntc, _spread = peek(rig)
            print('  switch %5.1f min   NTC %s  (sample)'
                  % (done / 60.0,
                     '%6.2f C' % ntc if ntc is not None else 'quiet'),
                  flush=True)
        with Coaxial63100(port=PORT, power_afe=False) as rig:
            top = peek(rig)
            print('  right after stop : NTC %s'
                  % ('%.2f C' % top[0] if top[0] else 'quiet'), flush=True)
            time.sleep(60.0)
            bulk = peek(rig)
            print('  +60 s (evened out): NTC %s'
                  % ('%.2f C' % bulk[0] if bulk[0] else 'quiet'), flush=True)
        return bulk

    with Coaxial63100(port=PORT) as rig:
        if state == 'passive':
            rig.board.afe.disable()
        else:
            rig.board.afe.enable()
            time.sleep(0.3)

        if state == 'traffic':
            rig.configure(accumulate=1, digital=True)
            rig.start()

        afe_off = (state == 'passive')
        start = last_peek = time.time()
        while (time.time() - start) < dwell_s:
            time.sleep(poll_s)
            if state == 'traffic':
                try:
                    rig.acquire()       # the traffic is the point
                except (NoReplyError, RigError):
                    pass
            mins = (time.time() - start) / 60.0

            if not afe_off:
                ntc, spread = sensors(rig)
            elif (time.time() - last_peek) >= PEEK_EVERY_S:
                ntc, spread = peek(rig)
                last_peek = time.time()
                print('  %-7s %5.1f min   NTC %s  (sample)'
                      % (state, mins,
                         '%6.2f C' % ntc if ntc is not None else 'quiet'),
                      flush=True)
                continue
            else:
                print('  %-7s %5.1f min   (AFE off, next sample in %.0f s)'
                      % (state, mins,
                         PEEK_EVERY_S - (time.time() - last_peek)), flush=True)
                continue

            print('  %-7s %5.1f min   NTC %6.2f C' % (state, mins, ntc),
                  flush=True)

        if state == 'traffic':
            try:
                rig.stop()
            except RigError:
                pass
        return peek(rig) if afe_off else sensors(rig)


def wait_for_next():
    if os.path.exists(NEXT_FILE):
        os.remove(NEXT_FILE)
    while not os.path.exists(NEXT_FILE):
        time.sleep(2.0)
    os.remove(NEXT_FILE)


def main():
    global PORT
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--minutes', type=float, default=25.0,
                   help='how long each state is held; 25 = 3.7 tau = 97 %%')
    p.add_argument('--only', help='these states only, comma separated')
    p.add_argument('--next', action='store_true',
                   help='tell a waiting run to move to the next state')
    a = p.parse_args()

    if a.next:
        open(NEXT_FILE, 'w').close()
        print('moving to the next state')
        return

    PORT = a.port
    want = a.only.split(',') if a.only else list(STATES)
    print('tau = %.1f min, holding %.0f min per state (%.1f tau)'
          % (tau_minutes(), a.minutes, a.minutes / tau_minutes()), flush=True)
    print('%d state(s) -> about %.0f min plus camera time\n'
          % (len(want), len(want) * a.minutes), flush=True)

    for state in want:
        print('--- %s: %s ---' % (state, WHAT[state]), flush=True)
        ntc, spread = hold(state, a.minutes * 60.0)
        announce(state, a.minutes, ntc, spread)
        wait_for_next()

    print('every state run', flush=True)


if __name__ == '__main__':
    main()
