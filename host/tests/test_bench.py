"""Does the board still go as fast as it did? Against a recorded baseline."""
import json
import os
import sys
import time

from coaxial import Coaxial63100
from coaxial.comm.session import open_session
from coaxial.errors import NoReplyError, RigError

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '.bench.json')

#: The share of the baseline below which a figure has regressed: wide enough
#: that a busy PC passes, narrow enough to catch a poll that started blocking
#: (the free-read defect cost far more).
SLACK = 0.70

#: Seconds per measurement. Long enough that the 5 s thermal observer sample lands
#: inside at least one of them, which is the point: its cost has to be in the
#: number rather than dodged by a short window.
WINDOW = 6.0


def round_trips(rig):
    """Modbus request/reply per second, on the cheapest call there is."""
    done, quiet, end = 0, 0, time.time() + WINDOW
    while time.time() < end:
        try:
            rig.board.afe.is_on()
            done += 1
        except (NoReplyError, RigError):
            quiet += 1
    return done / WINDOW, quiet


def per_second(rig, read, seconds=WINDOW):
    """Rate of a monotonic counter the board keeps."""
    first = read()
    t0 = time.time()
    time.sleep(seconds)
    return (read() - first) / (time.time() - t0)


def measure(rig):
    """Each figure in its own quiet window; the order is part of it."""
    got = {
        'angle_updates_per_s': per_second(
            rig, lambda: rig.board.angle.state()['updates']),
        'observer_steps_per_s': per_second(
            rig, lambda: rig.board.thermal.state()['steps']),
    }
    trips, quiet = round_trips(rig)
    got['round_trips_per_s'] = trips
    got['quiet_replies'] = quiet
    return got


def report(now, was):
    """(lines, failures). Higher is better for every field but the quiet one."""
    lines, bad = [], 0
    for name, value in sorted(now.items()):
        if name == 'quiet_replies':
            lines.append('  %-22s %10.0f   silent replies in %.0f s'
                         % (name, value, WINDOW))
            continue
        before = was.get(name) if was else None
        if not before:
            lines.append('  %-22s %10.1f   no baseline' % (name, value))
            continue
        share = value / before
        ok = share >= SLACK
        bad += not ok
        lines.append('  %-22s %10.1f   was %9.1f   %3.0f %%  %s'
                     % (name, value, before, 100.0 * share,
                        'ok' if ok else 'REGRESSED'))
    return lines, bad


def main():
    record = '--record' in sys.argv

    _session, origin = open_session()
    if not origin.real or origin.kind == 'emulator':
        print('no board answered - a benchmark against the stand-in or the emulator '
              'measures this PC, so there is nothing to say')
        print('0 passed, 0 failed')
        return 0

    # AFE_ON held, as the baseline was recorded.
    with Coaxial63100(port=origin.port, power_afe=True) as rig:
        print('  %s\n' % rig.origin.label)
        now = measure(rig)

    was = {}
    if os.path.exists(BASELINE):
        with open(BASELINE, encoding='utf-8') as handle:
            was = json.load(handle)

    lines, bad = report(now, was)
    print('\n'.join(lines))

    if record:
        with open(BASELINE, 'w', encoding='utf-8') as handle:
            json.dump(now, handle, indent=2, sort_keys=True)
        print('\nbaseline written to %s' % os.path.basename(BASELINE))
        return 0

    if not was:
        print('\nno baseline yet - run with --record on a board you trust')
        return 0

    print('\n%d passed, %d failed' % (len(lines) - bad, bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
