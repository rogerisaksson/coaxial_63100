#!/usr/bin/env python3
"""The board's attitude, redrawn in this terminal until you close it.

    python tools/show_orientation.py
    python tools/show_orientation.py --port COM4 --hz 20

Enables the rotation vector on the IMU, then reads it and draws the board.
The drawing itself is `coaxial.orientation`, which is pure and tested; this
file is the loop, the screen and the cable.

Nothing here judges an orientation. It shows the quaternion the part reported
and the angles that follow from it - invariant 10 applies to attitude exactly
as it applies to a voltage.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import orientation                            # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from coaxial_mcp.session import open_session               # noqa: E402
from screen import paint, say                             # noqa: E402

ROTATION_VECTOR = 0x05


def latest(board):
    """The board's shared record, or None if it could not be read.

    One round trip, and no SPI in it: the board polls the part from its own
    main loop and this reads what that wrote. Draining cargo by cargo from
    here cost 45 ms each and caught one frame in eight.
    """
    try:
        return board.imu.state()
    except RigError:
        return None


def capability(board):
    """The board's own entry for its IMU, or None if it reports none.

    Read from the board, not decided here: `channels` kind 4 is the parts
    list the firmware carries, so a board without the part says so itself
    and a board that grows one needs nothing changed on this side.
    """
    try:
        parts = board.system.channel_map()['parts']
    except (RigError, KeyError):
        return None

    for part in parts:
        if part['name'].startswith('BNO') or 'IMU' in part['what'].upper():
            return part
    return None


def preflight(board, part):
    """Say what is about to happen, and return whether AFE_ON was already on.

    The caller puts the AFE back the way it found it. Leaving a board powered
    because a view was closed with Ctrl+C is a change nobody asked for, and
    switching one off that was on before is worse.
    """
    say('ok', 'capability', '%s - %s, on %s'
        % (part['name'], part['what'], part['where']))

    was_on = board.afe.is_on()

    if part['power']:
        if was_on:
            say('ok', part['power'], 'already on, and left on afterwards')
        else:
            say('warn', part['power'],
                'off - on for this run, off again on the way out')
            board.afe.enable()
            # The part needs its supply up before it is reset, and the reset
            # is the next thing that happens. Enabling and configuring in the
            # same breath answered SERVER DEVICE FAILURE.
            time.sleep(0.3)
            say('ok', part['power'], 'on')

    return was_on


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='screen refreshes per second')
    parser.add_argument('--interval-us', type=int, default=10000,
                        help='what to ask the IMU for, in microseconds')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed. For checking the view against a board '
                             'without a terminal to close.')
    args = parser.parse_args(argv)

    session, origin = open_session(args.port,
                                   simulated=True if args.simulated else None)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))

    board = session.board

    part = capability(board)
    if part is None:
        say('fail', 'capability', 'this board reports no IMU among its parts')
        session.close()
        return 1

    try:
        afe_was_on = preflight(board, part)
    except RigError as exc:
        say('fail', part['power'] or 'supply',
            'could not power %s: %s' % (part['name'], exc))
        session.close()
        return 1

    # Stop the board's poll loop, configure, start it again. Both would
    # otherwise be masters on one SPI bus. The reset is not optional: measured,
    # a Set Feature onto a part that was already running took no effect at all
    # and the loop absorbed nothing afterwards.
    try:
        with board.imu.configuring():
            board.imu.reset()
            board.imu.feature(ROTATION_VECTOR, args.interval_us)
    except RigError as exc:
        say('fail', 'rotation vector', str(exc))
        session.close()
        return 1

    say('ok', 'rotation vector', 'every %d us' % args.interval_us)
    say('ok', 'poll loop', 'the board reads the part; this reads the board')
    say('wait', 'drawing', 'Ctrl+C stops it and undoes all of the above')

    period = 1.0 / max(args.hz, 0.5)
    quaternion = (0.0, 0.0, 0.0, 1.0)
    stale = 0
    frame = 0
    # The board's own counter of rotation vectors written. A reading that has
    # not moved and a link that has stopped look identical in the values.
    seen = -1

    # Only on a console: piped to a file the escapes are not interpreted and
    # every frame arrives with the cursor moves printed in it.
    console = sys.stdout.isatty()
    if console:
        if os.name == 'nt':
            os.system('')    # enables ANSI on a Windows console
        sys.stdout.write(chr(27) + '[2J')
    shown = []

    try:
        while True:
            record = latest(board)
            fresh = record['quaternion'] if record else None
            if fresh is None or record['updates'] == seen:
                stale += 1
            else:
                quaternion = (fresh['i'], fresh['j'], fresh['k'], fresh['real'])
                seen, stale = record['updates'], 0

            frame += 1
            note = ''
            if record:
                note = ('   loop %s   %d vectors, %d errors'
                        % (record['loop'], record['updates'],
                           record['errors']))
            lines = (['coaxial_63100 - board attitude   '
                      '(Ctrl+C to leave)' + note, ''] +
                     orientation.picture(quaternion, frame=frame,
                                         age=stale).split('\n'))
            sys.stdout.write(paint(shown, lines, console))
            sys.stdout.flush()
            shown = lines

            if args.frames and frame >= args.frames:
                break

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        # Everything this run started, put back: the report it enabled,
        # and the supply, but only if it was the one that switched it on.
        try:
            with board.imu.configuring():
                board.imu.feature(ROTATION_VECTOR, 0)
            sys.stdout.write('\n')
            say('ok', 'rotation vector', 'disabled')
            if not afe_was_on and part['power']:
                board.afe.disable()
                say('ok', part['power'], 'off again - it was off before this')
            else:
                say('ok', part['power'] or 'supply',
                    'left on, as it was found')
        except RigError as exc:
            sys.stdout.write('\n')
            say('fail', 'putting it back', str(exc))
        session.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
