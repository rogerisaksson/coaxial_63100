#!/usr/bin/env python3
"""The board's attitude, redrawn in this terminal until you close it.

    python terminal/views/show_orientation.py
    python terminal/views/show_orientation.py --port COM4 --hz 20

Enables the rotation vector on the IMU, then reads it and draws the board.
The drawing itself is `coaxial.draw.orientation`, which is pure and tested; this
file is the loop, the screen and the cable.

Nothing here judges an orientation: it shows the quaternion the part reported
and the angles that follow from it (invariant 10).
"""
import argparse
import os
import sys
import threading
import time
from typing import Any

from rich.text import Text

from coaxial.comm.session import standing
from coaxial.draw import easel, orientation
from coaxial.graphics import card
from coaxial.errors import RigError
from terminal.loader import TO_MENU
from terminal.ui import ahead
from terminal.ui import console as _console, screen as _screen
from terminal.ui.console import WHEEL_STEP
from terminal.ui.screen import (PORT, FEED_HZ, FPS_CAP, Feed, Freshness, closing, mode_of, open_rig, run_view,
                                say)
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.views import drawn

_screen.CHATTER = False     # the boot bar replaced the scroll

ROTATION_VECTOR = 0x05

#: Degrees the attitude has to move before the picture redraws with it.
#: Under this the frame is held bit-identical, which is what stops the
#: part's resting wander from shimmering the shading. Sized under the
#: display's own resolution: at 150 columns one cell on the board's rim
#: is ~1 degree, so a change this small could not have moved a glyph.
DEADBAND_DEG = 0.35


def latest(board):
    """The board's shared record, or None if it could not be read."""
    try:
        return board.imu.state()
    except RigError:
        return None


def capability(board):
    """The board's own entry for its IMU, or None if it reports none."""
    try:
        parts = board.system.parts()
    except RigError:
        return None

    for part in parts:
        if part['name'].startswith('BNO') or 'IMU' in part['what'].upper():
            return part
    return None


def preflight(board, part):
    """Say what is about to happen."""
    say('ok', 'capability', '%s - %s, on %s'
        % (part['name'], part['what'], part['where']))

    # The supply is Coaxial63100's: it brings AFE_ON up on the way in and puts
    # it back on the way out.
    say('ok', part['power'] or 'supply',
        'on for this run, and put back the way it was found')


def canvas(args):
    """How big to draw, filling the window unless told otherwise."""
    if args.width and args.height:
        return args.width, args.height

    try:
        size = os.get_terminal_size()
        width, height = size.columns - 2, size.lines - 4
    except OSError:
        width, height = 100, 40         # not a terminal: still worth drawing

    return (args.width or max(40, width), args.height or max(16, height))


def silent_part(record):
    """What to draw when the IMU has produced nothing at all."""
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
    """Ask the part for a rotation vector, and say whether it took."""
    board.imu.settled()          # anything before 'running' is startup

    # The product id is decoration; the feature is what is needed.
    pid = {}
    try:
        with board.imu.configuring():
            pid = board.imu.product_id()
    except RigError:
        say('warn', 'product id', 'not answered - the HUD shows a dash')

    try:
        board.imu.configure({ROTATION_VECTOR: interval_us})
    except RigError as exc:
        say('fail', 'rotation vector', str(exc))
        return None

    say('ok', 'rotation vector', 'every %d us' % interval_us)
    say('ok', 'poll loop', 'the board reads the part; this reads the board')
    return pid


def boxes(part, pid, record, q, rate):
    """The instrument boxes, every value off the target; the record's `clock` the link's wall
    seconds a second of the board's - an emulated board awake runs at a fraction of real time, its
    reports with it."""

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
                      record.get('last_fault') or '-')),
                  ('clock', '%.2f of real time' % (1.0 / max(float(record.get('clock', 1.0)), 1e-9)))]
    return [hud(part['name'], ident or ['-']),
            hud('REPORT', stream or ['-']),
            hud('ATTITUDE', [
                ('q    i', '%+8.4f' % q[0]),
                ('     j', '%+8.4f' % q[1]),
                ('     k', '%+8.4f' % q[2]),
                ('  real', '%+8.4f' % q[3]),
                ('rpy', '%+6.1f %+6.1f %+6.1f' % (roll, pitch, yaw))])]


def put_back(board, part):
    """Everything this run started, undone."""
    done = []
    try:
        if board.imu.state().get('loop') != 'running':
            # The loop is down - the rail dropped, and the part forgot the
            # report with it.
            return [('rotation vector', 'already gone with the rail')]
        board.imu.configure({ROTATION_VECTOR: 0})
        done.append(('rotation vector', 'disabled - the part stops streaming'))
    except RigError as exc:
        done.append(('rotation vector', 'FAILED: %s' % exc))

    done.append(('IMU poll loop', 'running, as the board left it'))
    return done


def _lit(word):
    """One footer word in the live chip's colours."""

    return Text(word, style='chip.live')


def _mirror_keys(flip):
    """One footer pair per axis."""

    for name, flipped in zip('XYZ', flip):
        yield (name, Text('INV', style='bold color(214) on grey15')
               if flipped else Text('+', style='keys'))


def bindings(typed, view, quaternion):
    """Apply one frame of keys to the view state, in place."""
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

    # Mirrors on the raw quaternion (the empirical knob), then the whole
    # derivation in one call: orientation.attitude carries the tare and the
    # mounting sandwich, proven by its own three checks.
    def flipped(raw):
        sx, sy, sz = view['flip']
        return (-raw[0] if sx else raw[0],
                -raw[1] if sy else raw[1],
                -raw[2] if sz else raw[2], raw[3])

    q = orientation.attitude(flipped(view['quaternion']),
                             flipped(view['tare'])
                             if view['tare'] is not None else None)

    tall = view['tall']
    art_w = art_width(view['wide'])
    zoom = view['zoom'] * (TOON_ZOOM if not args.photo else 1.0)
    record, drawing = view['record'], view.get('drawing')
    if drawing is not None and drawn.handed(drawing, view['persist']):
        art = drawing['ahead'].drawn(q, art_w, tall, zoom=zoom, colour=colour,
                                     frame_on=view['frame_on'], scroll=view.get('scroll'),
                                     carried=drawing.pop('carried', None))
        # Drawn a frame ahead: this frame shows the pose asked the frame before, its numbers too.
        (q, record), drawing['asked'] = drawing['asked'] or (q, record), (q, record)
    else:
        art = orientation.render(
            q, width=art_w, height=tall, zoom=zoom,
            shop=view['shop'], toon=not args.photo, wire=not args.photo,
            colour=colour, frame_on=view['frame_on'],
            crew=view.get('crew'), persist=view.get('persist'),
            # One pose ahead: the crew rasters the pose just read while the
            # previous one is painted; wireframe._face_ahead has the numbers.
            scroll=view.get('scroll'), ahead=True)
    art = art.splitlines()
    margin = min((len(l) - len(l.lstrip(' '))
                  for l in art if l.strip()), default=0)
    art = [l[margin:] for l in art]

    # The tally's own words: the rule for what is stale lives in Freshness.
    note = view['note']
    return frame_of(
        console, origin, 'BOARD ATTITUDE', '\n'.join(art),
        boxes(view['part'], view['pid'], record, q, view['rate']),
        (tuple(_mirror_keys(view['flip']))
         + (('C', 'FRAME'), ('T', 'TARE'), ('WHEEL', 'ZOOM'),
            ('+ -', 'ZOOM'),
            # Lit while the view has the mouse, dark while the terminal has
            # it, the default, so a left-drag marks text.
            ('F', _lit('MOUSE') if _console.holding() else 'MOUSE'),
            ('Q', 'EXIT'), ('ESC', 'MENU'), ('', note))),
        dressed=False)


#: The zoom a view opens at: 1.0 is the guaranteed fit at any attitude; 1.44 rests larger and lets
#: an axis tip clip in the extremes - 77 % of the 1.875 it rested at. The toon's drawing a share of
#: it.
ZOOM, TOON_ZOOM = 1.44, 0.88

#: Columns the instruments take beside the drawing, and the fewest the drawing keeps.
HUD_COLUMNS, ART_LEAST = 42, 24


def art_width(wide):
    """The drawing's columns in a window `wide` across: the full width the HUD leaves over - a
    2*tall+14 cap cropped a zoomed model at the frame long before the window ran out."""
    return max(ART_LEAST, wide - HUD_COLUMNS)


def period_of(hz):
    """Seconds a frame, from the refresh asked for: FPS_CAP's at the least - a frame that costs
    more than its period never sleeps, and at 20 Hz a 52 ms frame held a core and most of the
    others - and one every two seconds at the most; a resting board costs almost nothing, its
    face held (`wireframe.FACE_SETTLE`)."""
    return 1.0 / max(0.5, min(float(hz), FPS_CAP))


def parse_args(argv):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--hz', type=float, default=FPS_CAP,
                        help='screen refreshes per second, at most %.0f'
                             % FPS_CAP)
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
    """Everything before the first frame, the drawing's process started first: its card, meshes
    and crew come up while this one opens the board. Its process closed where the start fails."""
    drawing = ahead.take(easel.__name__, 'render') if drawn.apart(args) else None
    # While the board opens, this process's own drawing readied where its card is warm: it draws
    # until its drawing's process is up.
    held = {}
    wide, tall = canvas(args)
    warming = (threading.Thread(target=drawn.warm_here, daemon=True, args=(
        (art_width(wide), tall), ZOOM * TOON_ZOOM, held))
               if drawing is not None and card.held() is not None else None)
    if warming is not None:
        warming.start()
    started = _launched(args, drawing, (warming, held))
    if started is None and drawing is not None:
        drawing.close()
    return started


def _launched(args, drawing, warmed):
    """Behind one boot strip that rides the milestones (link, six
    decimations, shadow casters, the part, its power, the rotation vector,
    the pool or the drawing's process), so the strip ends where the view
    begins.
    """
    # power_afe by name: the default is False and the part this view shows
    # is AFE-powered; the rail goes back as found.
    rig = open_rig('LINKING BNO085', port=args.port, power_afe=True,
                   execution_mode=mode_of(args))
    if rig is None:
        return None
    with boot('BRINGING THE PART UP') as step:
        origin, board = rig.origin, rig.board
        step(0.12, 'BNO085 LINKED')
        pool = drawn.bands(args, step) if drawing is None else None
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
            # One retry, after the loop has settled: the first launch after the
            # rail rises can catch the part mid-advertisement.
            board.imu.settled()
            pid = start_reporting(board, args.interval_us)
        if pid is None:
            rig.close()
            return None
        step(0.95, 'DRAWING')
        drawing, pool = drawn.where(args, step, drawing, pool, warmed)
        shop = drawn.workshop(args)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, standing(origin)))
    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')
    return rig, origin, board, part, pid, pool, shop, drawing


def _taken(view, state, first, new):
    """A fresh rotation vector into the view."""
    moved = orientation.angle_between(new, view['quaternion']) >= DEADBAND_DEG
    if first or moved:
        view['quaternion'] = new
    if first:
        state['tare'] = view['quaternion']


def main(argv=None):
    args = parse_args(argv)

    started = launch(args)
    if started is None:
        return 1
    rig, origin, board, part, pid, pool, shop, drawing = started

    period = period_of(args.hz)

    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None

    view = {'zoom': ZOOM,
            'quaternion': (0.0, 0.0, 0.0, 1.0), 'frame': 0}
    # `persist` holds the two frames before this one, so three can vote per
    # cell - steady._steady, one frame of latency for no blinks.
    state = {'tare': None, 'flip': [False, False, False],
             'frame_on': True, 'persist': {}, 't0': time.monotonic(),
             'drawing': None if drawing is None else {'ahead': drawing, 'asked': None,
                                                      'handed': False}}
    tally = Freshness()
    # The board read on its own thread: a frame draws at the screen's pace, not the link's - an
    # emulated board's link is several times slower than a real one's. Asked as each frame takes
    # its reading (`Feed.kick`), at the screen's rate, the pose being the picture; FEED_HZ when
    # no frame asks. At FEED_HZ alone a third of the frames drew the pose before (19 new poses a
    # second of 29.5 frames on native://).
    feed = Feed(lambda: latest(board), period=1.0 / FEED_HZ).start()

    def draw():
        wide, tall = canvas(args)
        record: Any = feed.latest
        feed.kick()
        if record is not None:
            record = dict(record, clock=getattr(board.transport, 'time_scale', 1.0))
        fresh = record['quaternion'] if record else None
        if record is not None and fresh is not None \
                and record['updates'] != tally.seen:
            _taken(view, state, tally.seen < 0,
                   (fresh['i'], fresh['j'], fresh['k'], fresh['real']))
        tally.take(record['updates'] if record else None)
        view['frame'] += 1
        shown = dict(state, part=part, pid=pid, record=record,
                     quaternion=view['quaternion'], rate=tally.rate,
                     stale=tally.stale, note=tally.note, frame=view['frame'],
                     zoom=view['zoom'], shop=shop, crew=pool,
                     wide=wide, tall=tall,
                     scroll=time.monotonic() - state['t0'])
        return compose(origin, args, shown,
                       colour=terminal and not args.photo,
                       console=board_view)

    def on_input(typed, moved):
        # The keys zoom too.
        for key in typed:
            if key in '+=-_':
                moved += WHEEL_STEP if key in '+=' else -WHEEL_STEP
        if moved:
            view['zoom'] = max(0.25, min(6.0, view['zoom'] * (1.0 + moved)))
        bindings(typed, state, view['quaternion'])

    try:
        leaving = run_view(board_view, terminal, period, args.frames, draw,
                           on_input, mouse=True)
    finally:
        feed.stop()
        sys.stdout.write('\n')
        if shop:
            shop.close()
        if pool:
            pool.close()
        if state['drawing'] is not None and state['drawing']['ahead'] is not None:
            state['drawing']['ahead'].close()
        done = put_back(board, part)
        rig.close()
        done.append((part['power'] or 'supply',
                     'back the way it was found'))
        closing(done, terminal, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
