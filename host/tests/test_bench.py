"""Does the board still go as fast as it did? Against a recorded baseline.

WHY A BASELINE AND NOT A THRESHOLD. A number written into a test is a limit
somebody invented, and this tree does not carry those (invariant 10). A
baseline is different: it is what this board actually did on a day someone
wrote down, and the check is that today is not materially worse. It catches
the regression without pretending to know what fast is.

WHAT IT WATCHES, and why each one is here rather than a loop counter:

    round trips     the link, end to end. Everything else rides on it.
    angle updates   the main loop's own rate - the angle poll runs as fast as
                    SPI4 allows, so it falls the moment anything in the loop
                    starts blocking. Needs AFE_ON: it powers the A1335, and
                    with the rail off this reads 0 and means nothing.
    observer steps  the thermal model, which must keep its 10 Hz whatever the
                    sensors are doing. `steps`, not `seconds`: the latter is
                    wall clock, so its rate is 1.0 however slow the thermal observer
                    gets, and this check watched it for a day.

Measured 2026-08-28: the thermal observer's free-read path read two ADC channels -
one at 810.5 cycles - and did two SPI4 transactions on EVERY poll whenever
anything held AFE_ON, for an anchor whose gain is 0.05 Hz. That is the shape
of regression this exists to catch.

    python tests/test_bench.py             # check against the baseline
    python tests/test_bench.py --record    # write today down as the baseline
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import NoReplyError, RigError            # noqa: E402

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '.bench.json')

#: How much slower than the baseline is a regression. Wide enough that a busy
#: PC does not fail the board, narrow enough to catch a poll that started
#: blocking - the free-read defect above cost far more than this.
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
    """Each figure in its own quiet window, and the ORDER is part of it.

    The loop rates go first, over a link that is otherwise idle. Measured:
    taking them after the round-trip hammer gave 1404 angle updates a second
    against 16 728 - the polls sit behind `link_busy()`, so timing them while
    hammering the link times the traffic instead of the loop. Two true
    numbers about different things, and only one of them is the one asked
    for.
    """
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

    from coaxial_mcp.session import open_session   # noqa: E402
    _session, origin = open_session()
    if not origin.real:
        print('no board answered - a benchmark against the stand-in measures '
              'this PC, so there is nothing to say')
        print('0 passed, 0 failed')
        return 0

    from coaxial import Coaxial63100                # noqa: E402
    # AFE_ON HELD, and that is what the baseline was recorded under.
    # It powers the A1335, so the angle rate without it is 0 - and it
    # is also the condition the thermal observer's free-read path is measured
    # in, which is the regression this suite exists to catch. The rig
    # puts the rail back the way it found it.
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
