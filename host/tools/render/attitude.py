#!/usr/bin/env python3
"""The attitude view's picture at a pose and a moment, to text and a PNG - no window.

    python tools/render/attitude.py --rpy 25 -15 30 --at 48 --png out.png
    python tools/render/attitude.py --craft 2 --png pass.png     # mid the second pass
    python tools/render/attitude.py --at 5.5 --size 150 44       # the flight at 5.5 s

The board at roll, pitch, yaw (degrees, the view's own derivation of an IMU reading);
the approach - gates, the bank, the stars, the craft - at `--at` seconds of its clock.
"""
import argparse
import math
import sys

from coaxial.draw import orientation
from coaxial.graphics import approach
from machine import ansi


def quaternion(roll, pitch, yaw):
    """(i, j, k, real) of roll, pitch, yaw in degrees, applied yaw, pitch, roll."""
    cr, sr = math.cos(math.radians(roll) / 2.0), math.sin(math.radians(roll) / 2.0)
    cp, sp = math.cos(math.radians(pitch) / 2.0), math.sin(math.radians(pitch) / 2.0)
    cy, sy = math.cos(math.radians(yaw) / 2.0), math.sin(math.radians(yaw) / 2.0)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def pass_at(n, width, height, step=0.05, within=3600.0):
    """The middle of the `n`th craft pass (1 the first), seconds; None past `within`."""
    seen, t, during = 0, 0.0, []
    while t < within:
        on = approach._pass(t) is not None
        if on:
            during.append(t)
        elif during:
            seen += 1
            if seen == n:
                return during[len(during) // 2]
            during = []
        t += step
    return None


def frame(roll=0.0, pitch=0.0, yaw=0.0, at=0.0, width=116, height=46, zoom=1.44 * 0.88,
          hud=True):
    """The view's picture as ANSI text."""
    q = orientation.attitude(quaternion(roll, pitch, yaw))
    return orientation.render(q, width=width, height=height, zoom=zoom, toon=True, wire=True,
                              colour=True, frame_on=hud, scroll=at)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--rpy', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=('ROLL', 'PITCH', 'YAW'), help='the board, degrees')
    parser.add_argument('--at', type=float, default=0.0, help="the approach's clock, s")
    parser.add_argument('--craft', type=int, help='mid the nth craft pass instead of --at')
    parser.add_argument('--size', type=int, nargs=2, default=(116, 46),
                        metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--zoom', type=float, default=1.44 * 0.88,
                        help="the view's own: its 1.44 over the frame's 0.88")
    parser.add_argument('--bare', action='store_true', help='no ground, no HUD')
    parser.add_argument('--png', help='also the picture as a PNG here')
    args = parser.parse_args(argv)
    width, height = args.size
    at = args.at
    if args.craft:
        at = pass_at(args.craft, width, height)
        if at is None:
            print('no pass %d within the hour' % args.craft, file=sys.stderr)
            return 1
    art = frame(*args.rpy, at=at, width=width, height=height, zoom=args.zoom,
                hud=not args.bare)
    print(art)
    if args.png:
        size = ansi.png(art, args.png)
        print('%s: %dx%d, t %.2f s' % (args.png, size[0], size[1], at), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
