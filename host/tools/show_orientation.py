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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import farm, orientation                      # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from screen import closing, say, TO_MENU                  # noqa: E402

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

ROTATION_VECTOR = 0x05

#: Degrees the attitude has to move before the picture redraws with it.
#: Under this the frame is held bit-identical, which is what stops the
#: part's resting wander from shimmering the shading. Sized under the
#: display's own resolution: at 150 columns one cell on the board's rim
#: is ~1 degree, so a change this small could not have moved a glyph.
DEADBAND_DEG = 0.35


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
    The stage takes four rows - title band, key bar, the viewport's two
    edges - and the -10 it reserved from the banner era left the drawing
    short OR, worse, taller than its panel: the viewport centres
    vertically, so the overflow CLIPPED both the top and the bottom of
    the model at once.
    """
    if args.width and args.height:
        return args.width, args.height

    try:
        size = os.get_terminal_size()
        width, height = size.columns - 2, size.lines - 4
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
        return _announced(farm.Farm(orientation.MODEL_MESH))
    except (OSError, ValueError) as exc:
        say('warn', 'drawing', 'one process only: %s' % exc)
        return None


def _announced(pool):
    say('ok', 'drawing', '%d processes, one band of the picture each'
        % pool.workers)
    return pool


def bands(args, step):
    """The staged engine's crew for the vector drawing, or None for a
    run too short to pay for spawning it. Measured 2026-08-30: the
    raster at 150x44 goes 9.9 ms to 5.0 with eight workers; at 220x60,
    14.8 to 5.8. The crew holds EVERY level of detail: built on the
    zoom-1 board alone it was never used - the view opens at zoom
    1.49, one level finer - and the finest level costs 29 ms alone.

    Six decimations - 1.2 s in parallel, 5.0 one after another - the
    shadow casters' decimation (0.86 s, which used to land on the
    FIRST FRAME after the strip) and the spawn, all reported to the
    boot strip's `step`."""
    from coaxial import crew, wireframe
    levels = len(wireframe.LODS)

    def landed(done, _total, divisions):
        step(0.15 + 0.45 * done / levels, 'DECIMATING GRID %d' % divisions)

    solids = wireframe._lods(landed)
    step(0.62, 'SHADOW CASTERS')
    wireframe._shadowmap((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    # The outline's exact index and its loops: 0.6 s once, here behind
    # the strip rather than as a hitch on the first frame.
    step(0.66, 'OUTLINE EDGES')
    wireframe._outline_source()
    if args.photo or (args.frames and args.frames <= 4):
        return None
    step(0.70, 'SPAWNING %d PROCESSES' % crew.MAX_WORKERS)
    try:
        return _announced(crew.Crew(solids, art=wireframe._face()))
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
    board.imu.settled()          # anything before 'running' is startup

    # The product id is DECORATION - the feature is the point. They were
    # one configuring() block, and an id refused during the part's boot
    # window took the whole view down with it: 'sometimes does not load'
    # was this line. An id that does not answer is a dash in the HUD.
    pid = {}
    try:
        with board.imu.configuring():
            pid = board.imu.product_id()
    except RigError:
        say('warn', 'product id', 'not answered - the HUD shows a dash')

    try:
        # No reset first. The poll loop brings the part up on its own, and
        # a reset immediately before a Set Feature is what stops the feature
        # taking: the write's wake handshake runs its own reset when the
        # acknowledge does not arrive, and right after a host reset it does
        # not, so the write lands on a part that has just restarted.
        # Measured 2026-08-27: reset then feature, 0 rotation vectors;
        # feature alone, 49.0 a second. See FINDINGS.
        with board.imu.configuring():
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
        if board.imu.state().get('loop') != 'running':
            # The loop is down - the rail dropped, and the part forgot
            # the report with it. Nothing to disable, and saying FAILED
            # on the way out over that taught nothing.
            return [('rotation vector', 'already gone with the rail')]
        with board.imu.configuring():
            board.imu.feature(ROTATION_VECTOR, 0)
        done.append(('rotation vector', 'disabled - the part stops streaming'))
    except RigError as exc:
        done.append(('rotation vector', 'FAILED: %s' % exc))

    done.append(('IMU poll loop', 'running, as the board left it'))
    return done


def _mirror_keys(flip):
    """One footer pair per axis. An INVERTED axis burns sodium on the bar,
    so a wild ride on the empirical dial always shows where it is."""
    from rich.text import Text

    for name, flipped in zip('XYZ', flip):
        yield (name, Text('INV', style='bold color(214) on grey15')
               if flipped else Text('+', style='keys'))


def bindings(typed, view, quaternion):
    """Apply one frame of keys to the view state, in place.

    X, Y and Z toggle MIRRORING of that axis - the empirical knob for
    finding how the part actually sits; the footer shows the state and
    the finding goes into MOUNT as code once it is reported. C toggles
    the coordinate system and the horizon. T tares: the attitude from
    HERE is what draws - one press cancels the mounting offset and the
    arbitrary yaw reference at once, and pressing again re-zeros.
    """
    for t in typed:
        if t in 'xX':
            view['flip'][0] = not view['flip'][0]
        elif t in 'yY':
            view['flip'][1] = not view['flip'][1]
        elif t in 'zZ':
            view['flip'][2] = not view['flip'][2]
        elif t in 'cC':
            view['frame_on'] = not view['frame_on']
        elif t in 'tT':
            view['tare'] = quaternion


def compose(origin, args, view, colour, console):
    """One frame on the stage: viewport left, instruments right, keys."""
    from screen import frame_of

    # Mirrors on the raw quaternion (the empirical knob), then the whole
    # derivation in one call: orientation.attitude carries the tare and
    # the mounting sandwich, proven by its own three checks.
    def flipped(raw):
        sx, sy, sz = view['flip']
        return (-raw[0] if sx else raw[0],
                -raw[1] if sy else raw[1],
                -raw[2] if sz else raw[2], raw[3])

    q = orientation.attitude(flipped(view['quaternion']),
                             flipped(view['tare'])
                             if view['tare'] is not None else None)

    tall = view['tall']
    # The FULL width the HUD leaves over: the old 2*tall+14 cap cropped a
    # zoomed model at the frame long before the window ran out of columns.
    art_w = max(24, view['wide'] - 42)
    art = orientation.render(
        q, width=art_w, height=tall,
        zoom=view['zoom'] * (0.88 if not args.photo else 1.0),
        shop=view['shop'], toon=not args.photo, wire=not args.photo,
        colour=colour, frame_on=view['frame_on'],
        crew=view.get('crew'), persist=view.get('persist')).splitlines()
    margin = min((len(l) - len(l.lstrip(' '))
                  for l in art if l.strip()), default=0)
    art = [l[margin:] for l in art]

    note = (('stale %d frames' % view['stale']) if view['stale'] else 'live')
    return frame_of(
        console, origin, 'BOARD ATTITUDE', '\n'.join(art),
        boxes(view['part'], view['pid'], view['record'], q, view['rate']),
        (tuple(_mirror_keys(view['flip']))
         + (('C', 'FRAME'), ('T', 'TARE'), ('WHEEL', 'ZOOM'),
            ('Q', 'EXIT'), ('ESC', 'MENU'), ('', note))))


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


def launch(args):
    """Everything before the first frame, behind ONE boot strip that
    rides the real milestones - link, six decimations, shadow casters,
    the part, its power, the rotation vector, the pool - so the strip
    ends where the view begins. None when a step refused, after
    saying which."""
    # power_afe SAID: the default went quiet-False when every connect
    # stopped flipping the rail, and this view inherited it - the part it
    # exists to show is AFE-powered, so it asks by name and puts it back.
    from screen import boot, open_rig
    rig = open_rig('LINKING BNO085', port=args.port, power_afe=True,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return None
    with boot('BRINGING THE PART UP') as step:
        origin, board = rig.origin, rig.board
        step(0.12, 'BNO085 LINKED')
        pool = bands(args, step)
        step(0.78, 'THE PART')
        part = capability(board)
        if part is None:
            say('fail', 'capability',
                'this board reports no IMU among its parts')
            rig.close()
            return None
        try:
            preflight(board, part)
        except RigError as exc:
            say('fail', part['power'] or 'supply',
                'could not power %s: %s' % (part['name'], exc))
            rig.close()
            return None
        step(0.86, 'ROTATION VECTOR')
        pid = start_reporting(board, args.interval_us)
        if pid is None:
            # ONE retry, after the loop has settled: the first launch
            # after the rail rises can catch the part mid-advertisement,
            # and a view that needs a second start by hand reads as
            # broken.
            board.imu.settled()
            pid = start_reporting(board, args.interval_us)
        if pid is None:
            rig.close()
            return None
        step(0.95, 'DRAWING')
        shop = workshop(args)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')
    return rig, origin, board, part, pid, pool, shop


def main(argv=None):
    args = parse_args(argv)

    started = launch(args)
    if started is None:
        return 1
    rig, origin, board, part, pid, pool, shop = started

    period = 1.0 / max(args.hz, 0.5)
    from screen import Freshness, run_view, stage

    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # zoom: 1.0 is the guaranteed fit at ANY attitude; 1.5 rests larger and
    # lets an axis tip clip in the extremes, which the eye forgives and the
    # wheel undoes. The wheel and a right-drag move it, clamped so the
    # model cannot be pushed through the camera or shrunk to nothing.
    view = {'zoom': 1.44,                # 77% of the 1.875 it rested at
            'quaternion': (0.0, 0.0, 0.0, 1.0), 'frame': 0}
    # `persist` holds the two frames before this one, so three can vote
    # per cell - wireframe._steady, one frame of latency for no blinks.
    state = {'tare': None, 'flip': [False, False, False],
             'frame_on': True, 'persist': {}}
    tally = Freshness()

    def draw():
        wide, tall = canvas(args)
        record = latest(board)
        fresh = record['quaternion'] if record else None
        if fresh is not None and record['updates'] != tally.seen:
            new = (fresh['i'], fresh['j'], fresh['k'], fresh['real'])
            # THE DEADBAND. The part's rotation vector wanders a few tenths
            # of a degree at rest, and every wander redrew the board:
            # measured, 1.7 % of the cells changed glyph and 80 % of the
            # rows changed tone between two frames that differed by noise
            # alone - the shimmer. Held under DEADBAND_DEG the picture is
            # bit-identical frame to frame, and a real turn passes the
            # band inside one frame. Nothing is lost: at this view's size
            # one cell on the board's rim is about a degree, so a change
            # under the band could not have moved a glyph anyway.
            if (tally.seen < 0 or orientation.angle_between(
                    new, view['quaternion']) >= DEADBAND_DEG):
                view['quaternion'] = new
            if tally.seen < 0:
                # TARE ONCE, on the first real sample: the resting picture
                # is the board as it lies, not its yaw history. T re-tares
                # whenever wanted.
                state['tare'] = view['quaternion']
        tally.take(record['updates'] if record else None)
        view['frame'] += 1
        shown = dict(state, part=part, pid=pid, record=record,
                     quaternion=view['quaternion'], rate=tally.rate,
                     stale=tally.stale, frame=view['frame'],
                     zoom=view['zoom'], shop=shop, crew=pool,
                     wide=wide, tall=tall)
        return compose(origin, args, shown,
                       colour=console and not args.photo, console=console)

    def on_input(typed, moved):
        if moved:
            view['zoom'] = max(0.25, min(6.0, view['zoom'] * (1.0 + moved)))
        bindings(typed, state, view['quaternion'])

    try:
        leaving = run_view(board_view, console, period, args.frames, draw,
                           on_input, mouse=True)
    finally:
        sys.stdout.write('\n')
        if shop:
            shop.close()
        if pool:
            pool.close()
        done = put_back(board, part)
        rig.close()
        done.append((part['power'] or 'supply',
                     'back the way it was found'))
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
