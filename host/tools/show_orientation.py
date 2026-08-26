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


def paint(shown, lines, console):
    """What to write to move the screen from `shown` to `lines`.

    Only the rows that differ, each addressed directly. Rewriting all of it
    every frame is what made the prompt flicker: at 20 Hz the terminal
    repaints two dozen unchanged rows, so the header and the caption blink
    along with the drawing they are not part of.
    """
    if not console:
        return '\n'.join(lines) + '\n'

    out = []
    for row in range(max(len(shown), len(lines))):
        was = shown[row] if row < len(shown) else None
        now = lines[row] if row < len(lines) else ''
        if now != was:
            out.append('%s[%d;1H%s%s[K' % (chr(27), row + 1, now, chr(27)))

    return ''.join(out)



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
            fresh = latest(board, time.time() + period * 0.6)
            if fresh is None:
                stale += 1
            else:
                quaternion, stale = fresh, 0

            frame += 1
            lines = (['coaxial_63100 - board attitude   '
                      '(Ctrl+C to leave)', ''] +
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
        try:
            board.imu.feature(ROTATION_VECTOR, 0)      # leave it as found
        except RigError:
            pass
        session.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
