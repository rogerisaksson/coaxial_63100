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

from coaxial import farm, orientation                      # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from coaxial import Coaxial63100                           # noqa: E402
from screen import (TO_MENU, Keys, banner, closing,  # noqa: E402
                    say, stamp_crosses)

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

    # The supply is Coaxial63100's: it brings AFE_ON up on the way in and
    # puts it back on the way out. The part is therefore already settled by
    # the time this runs, which matters - a reset issued before its supply
    # had come up answered SERVER DEVICE FAILURE.
    say('ok', part['power'] or 'supply',
        'on for this run, and put back the way it was found')


def canvas(args):
    """How big to draw, filling the window unless told otherwise.

    Big matters here more than anywhere else in this tree: a board is mostly
    flat, so what shows its components is each one covering several cells.
    At 34x15 none of them does and the board is a disc; at 150x60 they
    resolve and it looks like the reference this renderer is a port of.
    Four rows are left for the caption and the numbers above it, and
    two more for the blanks that keep the banner off the top rows,
    where the shell's status line sits.
    """
    if args.width and args.height:
        return args.width, args.height

    try:
        size = os.get_terminal_size()
        width, height = size.columns - 2, size.lines - 10
    except OSError:
        width, height = 100, 40         # not a terminal: still worth drawing

    return (args.width or max(40, width), args.height or max(16, height))


def workshop(args):
    """A pool of drawing processes, or None if this run is too short for one.

    Pure Python holds the GIL, so the frame is cut into bands and drawn by
    several processes - measured at 150x44, 137 ms becomes 45. Windows spawns
    a fresh interpreter per worker, which costs about two seconds for
    sixteen, so a run of a few frames is better off drawing them itself.
    """
    if args.frames and args.frames <= 4:
        return None
    if not args.photo:
        # The toon mesh draws in 12 ms single-process - measured, against
        # 108 for the photographic one - so a pool would cost more in spawn
        # time than it saves, and render() only draws the toon package when
        # no shop is passed.
        return None

    try:
        return farm.Farm(orientation.MODEL_MESH)
    except (OSError, ValueError) as exc:
        say('warn', 'drawing', 'one process only: %s' % exc)
        return None


def silent_part(record):
    """What to draw when the IMU has produced nothing at all.

    Not the model at identity: identity is a board lying exactly level, and
    a viewer cannot tell that from a part that has never spoken.
    """
    return [
        '  The IMU has reported %d rotation vectors.' % record['updates'],
        '',
        '  loop %s, %d errors, last error %s.'
        % (record['loop'], record['errors'], record['error']),
        '',
        '  Nothing is drawn because there is nothing to draw: the model at',
        '  identity is a board lying exactly level, which is a plausible',
        '  attitude and would look like a part that works.',
        '',
        '  imu.pins() and imu.product_id() need the loop HELD - call them',
        '  inside board.imu.configuring(), or they answer SERVER DEVICE',
        '  FAILURE and look like a dead part. See FINDINGS.',
    ]


def start_reporting(board, interval_us):
    """Ask the part for a rotation vector, and say whether it took.

    Stop the board's poll loop, configure, start it again: both would
    otherwise be masters on one SPI bus. The reset is not optional - measured,
    a Set Feature onto a part that was already running took no effect at all
    and the loop absorbed nothing afterwards.
    """
    board.imu.settled()          # a feature before 'running' is refused

    try:
        # No reset first. The poll loop brings the part up on its own, and
        # a reset immediately before a Set Feature is what stops the feature
        # taking: the write's wake handshake runs its own reset when the
        # acknowledge does not arrive, and right after a host reset it does
        # not, so the write lands on a part that has just restarted.
        # Measured 2026-08-27: reset then feature, 0 rotation vectors;
        # feature alone, 49.0 a second. See FINDINGS.
        with board.imu.configuring():
            pid = board.imu.product_id()
            board.imu.feature(ROTATION_VECTOR, interval_us)
    except RigError as exc:
        say('fail', 'rotation vector', str(exc))
        return None

    say('ok', 'rotation vector', 'every %d us' % interval_us)
    say('ok', 'poll loop', 'the board reads the part; this reads the board')
    return pid


def boxes(part, pid, record, q, rate):
    """The instrument boxes, every value off the target."""
    from screen import hud

    roll, pitch, yaw = orientation.euler_degrees(q)
    ident = []
    if pid:
        ident = [('firmware', '%s build %s' % (pid.get('sw_version'),
                                               pid.get('sw_build'))),
                 ('reset', str(pid.get('reset_cause_name')))]
    stream = []
    if record:
        feature = record.get('feature') or {}
        stream = [('loop', str(record.get('loop'))),
                  ('report', '0x%02X every %d us'
                   % (feature.get('report_id', 0),
                      feature.get('interval_us', 0))),
                  ('rate', '%.1f vectors/s' % rate),
                  ('errors', '%d  last %s'
                   % (record.get('errors', 0),
                      record.get('last_fault') or '-'))]
    return [hud(part['name'], ident or ['-']),
            hud('REPORT', stream or ['-']),
            hud('ATTITUDE', [
                ('q    i', '%+8.4f' % q[0]),
                ('     j', '%+8.4f' % q[1]),
                ('     k', '%+8.4f' % q[2]),
                ('  real', '%+8.4f' % q[3]),
                ('rpy', '%+6.1f %+6.1f %+6.1f' % (roll, pitch, yaw))])]


def put_back(board, part):
    """Everything this run started, undone.

    The report it enabled, and the supply - but the supply only if this run
    was what switched it on. Leaving a board powered because a view was
    closed is a change nobody asked for, and switching one off that was on
    before is worse.
    """
    done = []
    try:
        with board.imu.configuring():
            board.imu.feature(ROTATION_VECTOR, 0)
        done.append(('rotation vector', 'disabled - the part stops streaming'))
    except RigError as exc:
        done.append(('rotation vector', 'FAILED: %s' % exc))

    done.append(('IMU poll loop', 'running, as the board left it'))
    return done


def compose(origin, args, view, colour, console):
    """One frame on the stage: viewport left, instruments right, keys."""
    from screen import frame_of

    q = view['quaternion']
    if view['tare'] is not None:
        q = orientation.relative(q, view['tare'])

    tall = view['tall']
    art_w = min(max(24, view['wide'] - 40), 2 * tall + 14)
    art = orientation.render(
        q, width=art_w, height=tall,
        zoom=view['zoom'] * (0.88 if not args.photo else 1.0),
        shop=view['shop'], toon=not args.photo, colour=colour).splitlines()
    margin = min((len(l) - len(l.lstrip(' '))
                  for l in art if l.strip()), default=0)
    art = stamp_crosses([l[margin:] for l in art], art_w - margin)

    note = (('stale %d frames' % view['stale']) if view['stale'] else 'live')
    return frame_of(
        console, origin, 'BOARD ATTITUDE', '\n'.join(art),
        boxes(view['part'], view['pid'], view['record'], q, view['rate']),
        (('Q', 'CLOSE'), ('ESC', 'MENU'), ('Z', 'TARE'), ('WHEEL', 'ZOOM'),
         ('', note)))


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='screen refreshes per second')
    parser.add_argument('--interval-us', type=int, default=10000,
                        help='what to ask the IMU for, in microseconds')
    parser.add_argument('--width', type=int, default=0,
                        help='drawing width in characters. 0 fills the window')
    parser.add_argument('--height', type=int, default=0,
                        help='drawing height in rows, 0 to fill. Rows are '
                             'worth two columns, so this is what binds')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--photo', action='store_true',
                        help='the photographic renderer: the fine mesh and '
                             'the ten-step ramp, banded across processes. '
                             'The default is the toon drawing, which is '
                             '9x faster and reads better at terminal sizes')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed. For checking the view against a board '
                             'without a terminal to close.')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # power_afe SAID: the default went quiet-False when every connect
    # stopped flipping the rail, and this view inherited it - the part it
    # exists to show is AFE-powered, so it asks by name and puts it back.
    rig = Coaxial63100(port=args.port, power_afe=True,
                       simulated_device=bool(args.simulated)).open()
    origin, board = rig.origin, rig.board
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))

    part = capability(board)
    if part is None:
        say('fail', 'capability', 'this board reports no IMU among its parts')
        rig.close()
        return 1

    try:
        preflight(board, part)
    except RigError as exc:
        say('fail', part['power'] or 'supply',
            'could not power %s: %s' % (part['name'], exc))
        rig.close()
        return 1

    pid = start_reporting(board, args.interval_us)
    if pid is None:
        # ONE retry, after the loop has settled: the first launch after the
        # rail rises can catch the part mid-advertisement, and a view that
        # needs a second start by hand reads as broken.
        board.imu.settled()
        pid = start_reporting(board, args.interval_us)
    if pid is None:
        rig.close()
        return 1
    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')

    period = 1.0 / max(args.hz, 0.5)
    quaternion = (0.0, 0.0, 0.0, 1.0)
    stale = 0
    frame = 0
    # The report rate, measured off the target's own counter rather than
    # assumed from the interval - the part adopts what it can.
    rate, rate_seen, rate_at = 0.0, None, time.time()
    # The board's own counter of rotation vectors written. A reading that has
    # not moved and a link that has stopped look identical in the values.
    seen = -1

    # Only on a console: piped to a file the escapes are not interpreted and
    # every frame arrives with the cursor moves printed in it.
    console = sys.stdout.isatty()
    from screen import curtain, stage

    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # 1.0 is the fit: the board just filling the shorter axis. The wheel and
    # a right-drag move it, clamped so the model cannot be pushed through
    # the camera or shrunk to nothing.
    zoom = 1.0
    tare = None
    shop = workshop(args)
    if shop:
        say('ok', 'drawing', '%d processes, one band of the picture each'
            % shop.workers)

    try:
        with curtain(board_view) as live, Keys(console, mouse=True) as keys:
            while True:
                wide, tall = canvas(args)

                record = latest(board)
                fresh = record['quaternion'] if record else None
                if fresh is None or record['updates'] == seen:
                    stale += 1
                else:
                    quaternion = (fresh['i'], fresh['j'], fresh['k'],
                                  fresh['real'])
                    seen, stale = record['updates'], 0

                frame += 1
                now = time.time()
                if record and now - rate_at >= 1.0:
                    if rate_seen is not None:
                        rate = ((record['updates'] - rate_seen)
                                / (now - rate_at))
                    rate_seen, rate_at = record['updates'], now

                live.update(compose(origin, args, {
                    'part': part, 'pid': pid, 'record': record,
                    'quaternion': quaternion, 'rate': rate, 'tare': tare,
                    'stale': stale, 'frame': frame, 'zoom': zoom,
                    'shop': shop, 'wide': wide, 'tall': tall},
                    colour=console and not args.photo,
                    console=console), refresh=True)

                if args.frames and frame >= args.frames:
                    break
                leaving, moved = keys.poll()
                if leaving:
                    break
                if moved:
                    zoom = max(0.25, min(6.0, zoom * (1.0 + moved)))
                if any(t in 'zZ' for t in keys.taken()):
                    # TARE: the attitude from HERE is what draws. One press
                    # cancels the mounting offset (-1.9 deg roll on this
                    # bench) and the arbitrary yaw reference at once;
                    # pressing it again re-zeros.
                    tare = quaternion

                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\n')
        if shop:
            shop.close()
        done = put_back(board, part)
        rig.close()
        done.append((part['power'] or 'supply',
                     'back the way it was found'))
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
