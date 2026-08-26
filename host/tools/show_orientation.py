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

ROTATION_VECTOR = 0x05


def latest(board, deadline):
    """The most recent quaternion the part has sent, or None.

    Drains rather than taking the first: at 20 Hz on screen and 100 Hz on the
    wire the queue would otherwise show an orientation that is seconds old.
    """
    found = None
    while time.time() < deadline:
        try:
            got = board.imu.read()
        except RigError:
            return found
        if not got['cargo']:
            return found
        for report in got['reports']:
            if 'quaternion' in report:
                q = report['quaternion']
                found = (q['i'], q['j'], q['k'], q['real'])
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='screen refreshes per second')
    parser.add_argument('--interval-us', type=int, default=10000,
                        help='what to ask the IMU for, in microseconds')
    args = parser.parse_args(argv)

    session, origin = open_session(args.port)
    print('%s - %s' % (origin.label, 'simulated' if not origin.real else 'live'))

    board = session.board
    try:
        board.imu.feature(ROTATION_VECTOR, args.interval_us)
    except RigError as exc:
        print('could not enable the rotation vector: %s' % exc)
        session.close()
        return 1

    period = 1.0 / max(args.hz, 0.5)
    quaternion = (0.0, 0.0, 0.0, 1.0)
    stale = 0
    frame = 0

    # Cursor home rather than a clear: clearing the screen every frame makes
    # the whole picture blink at 20 Hz, and the blink reads as the board
    # jumping about rather than turning.
    home = chr(27) + '[H'
    if os.name == 'nt':
        os.system('')            # enables ANSI on a Windows console

    print(chr(27) + '[2J', end='')

    try:
        while True:
            fresh = latest(board, time.time() + period * 0.6)
            if fresh is None:
                stale += 1
            else:
                quaternion, stale = fresh, 0

            frame += 1
            sys.stdout.write(home)
            sys.stdout.write('coaxial_63100 - board attitude   '
                             '(Ctrl+C to leave)' + chr(27) + '[K\n\n')
            for line in orientation.picture(quaternion, frame=frame,
                                            age=stale).split('\n'):
                sys.stdout.write(line + chr(27) + '[K\n')
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            board.imu.feature(ROTATION_VECTOR, 0)      # leave it as found
        except RigError:
            pass
        session.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
