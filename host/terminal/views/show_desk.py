#!/usr/bin/env python3
"""Every analog channel as a meter bridge, redrawn until you close it.

    python terminal/views/show_desk.py
    python terminal/views/show_desk.py --port COM4 --hz 10 --samples 32

The drawing itself is `coaxial.draw.desk`, which is pure and tested; this file is
the loop, the screen and the cable. Nothing here judges a reading: the face
is the converter's own scale (invariant 10).

AFE_ON is on for the run (invariant 9) and put back the way it was found on
the way out.
"""
import argparse
import collections
import os
import sys
import time

from coaxial.comm.session import standing
from coaxial.draw import desk
from coaxial.errors import RigError
from terminal.loader import TO_MENU
from terminal.ui import screen as _screen
from terminal.ui.demo import stop_motor, turn_motor
from terminal.ui.screen import Feed, closing, mode_of, open_rig, run_view, say
from terminal.ui.stage import frame_of, stage
from terminal.views.desk.boxes import (buffer_box, chain_box, digital_box, legend,
                                       scale)
from terminal.views.desk.load import (HOST_RING, _clocked, drain, duties_from, plan,
                                      rows_from, take_link)

_screen.CHATTER = False     # the boot bar replaced the scroll


VALUE = '[38;5;214m'       # the theme's `value`, the light source


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=8.0,
                        help='screen refreshes per second')
    parser.add_argument('--rate', type=float, default=0.0,
                        help='records a second the board produces. Default 0 '
                             'follows --hz, which is what a meter wants: one '
                             'reading a frame, averaged over the whole frame. '
                             'The converter is not slowed to it - it runs flat '
                             'out underneath and the board sums into each '
                             'record')
    parser.add_argument('--order', type=int, default=4,
                        help='Bessel order of the low-pass the board runs')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed')
    args = parser.parse_args(argv)

    # power_afe by name (invariant 9): with the rail down the board refuses
    # to start the task.
    rig = open_rig('LINKING CONVERTERS', port=args.port, power_afe=True,
                   execution_mode=mode_of(args))
    if rig is None:
        return 1
    origin = rig.origin
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, standing(origin)))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    # Every channel the board reports, summed and shaped on the board rather
    # than read one at a time: a meter face wants a low-pass, and one record a
    # frame is what it draws.
    try:
        layout, chain = plan(rig, args)
        params = rig.board.analog.scaling()
        rig.start()
    except RigError as exc:
        # The board's own sentence, said - not a traceback.
        say('fail', 'task', str(exc))
        rig.close()
        return 1
    if chain is None:
        say('warn', 'low pass', 'none - the loop could not be measured, '
                                'so the window averages instead')
    else:
        say('ok', 'low pass', '%.2f Hz, order %d, %.1f dB of what would '
            'fold' % (chain['cutoff'], chain['order'],
                      chain['worst_alias_db']))

    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')
    return watch(rig, args, layout, chain, params)


def watch(rig, args, layout, chain, params):
    """Draw it until Q, ESC or the frame count runs out."""
    origin = rig.origin
    demo = turn_motor(rig, origin)
    # The bar fills the window: at 38 columns the face floated in the frame.
    try:
        columns = os.get_terminal_size().columns
    except OSError:
        columns = 100
    bridge = desk.Desk(bar=max(20, min(80, columns - 72)))
    period = 1.0 / max(args.hz, 0.5)


    held = {}
    last = {'rows': [], 'pins': {}}
    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    # The link is this thread's alone.
    clock = {'at': 0.0, 'state': None, 'triggers': None, 'sweeps': 0.0}
    # The library's own reader, sampled before the drain below empties it.
    link = {'host': 0, 'peak': 0, 'dropped': 0, 'backlog': None,
            'rate': 0.0, 'reads': 0, 'records': 0, 'seen': 0, 'bits': 0.0,
            'at': 0.0, 'stride': layout.get('stride') or 0,
            'baud': rig.baud}
    # The host's own ring.
    inbox = collections.deque()
    host = {'held': 0, 'peak': 0, 'dropped': 0}

    def read():
        now = time.time()
        if demo is not None:
            demo(now)
        if clock['state'] is None or now - clock['at'] > 0.5:
            # The buffer gauge moves slowly by construction, and this is a
            # whole round trip spent on it.
            _clocked(clock, rig, now)
        # The level before the drain: after it every queue is empty by
        # construction and the gauge would read nothing on a task that is only
        # just keeping up.
        take_link(link, rig.buffered, now)
        for record in drain(rig):
            if len(inbox) >= HOST_RING:
                # The host's own drop, counted.
                inbox.popleft()
                host['dropped'] += 1
            inbox.append(record)
        host['held'] = len(inbox)
        host['peak'] = max(host['peak'], host['held'])
        return clock['state']

    # Not tied to the frame rate.
    feed = Feed(read, period=0.01).start()

    def draw():
        # Re-fitted every frame: the bars shrink with the tty instead of
        # overflowing the frame and sliding.
        bridge.bar = max(20, min(80, (board_view.size.width or 100) - 72))
        # Everything queued since the last frame, taken in one go.
        records = [inbox.popleft() for _ in range(len(inbox))]
        host['held'] = len(inbox)
        if records:
            last['rows'] = scale(rows_from(records, layout), params)
            last['pins'] = duties_from(records)
        rows = last['rows']
        if not rows:
            said = ('no reading: %s' % feed.error if feed.error
                    else 'waiting for the first block')
            return frame_of(board_view, origin, 'METER BRIDGE', said,
                            [buffer_box(feed.latest, host, link)],
                            (('Q', 'EXIT'), ('ESC', 'MENU')),
                            art_title='ANALOGUE')
        face = bridge.update(rows, colour=console)
        for row in rows:
            row['params'] = params
        return frame_of(board_view, origin, 'METER BRIDGE', face,
                        [legend(rows, held),
                         chain_box(chain, clock['sweeps'], len(rows)),
                         buffer_box(feed.latest, host, link)],
                        (('Q', 'EXIT'), ('ESC', 'MENU')),
                        art_title='ANALOGUE',
                        under=digital_box(last['pins'], bridge.bar + 30))

    try:
        leaving = run_view(board_view, console, period, args.frames, draw)
    finally:
        # Stopped before anything puts the board back: never two threads on
        # one serial transport.
        feed.stop()
        done = [('acquisition', 'task stopped')]
        if demo is not None:
            done += stop_motor(rig)
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
