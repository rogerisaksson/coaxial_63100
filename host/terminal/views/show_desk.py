#!/usr/bin/env python3
"""Every analog channel as a meter bridge, redrawn until you close it.

    python terminal/views/show_desk.py
    python terminal/views/show_desk.py --port COM4 --hz 10 --samples 32

The drawing itself is `coaxial.draw.desk`, which is pure and tested; this file is
the loop, the screen and the cable. Nothing here judges a reading - the face
is the converter's own scale and invariant 10 applies to a meter exactly as
it applies to a table.

The AFE has to be on for any of it to mean anything (invariant 9), and it is
put back the way it was found on the way out.
"""
import argparse
import collections
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.draw import desk  # noqa: E402
from coaxial.errors import RigError  # noqa: E402
from terminal.loader import TO_MENU  # noqa: E402
from terminal.ui import screen as _screen  # noqa: E402
from terminal.ui.screen import Feed, closing, open_rig, run_view, say  # noqa: E402
from terminal.ui.stage import frame_of, stage  # noqa: E402
from terminal.views.desk.boxes import (buffer_box, chain_box, digital_box, legend,  # noqa: E402
                                       scale)
from terminal.views.desk.load import (HOST_RING, _clocked, drain, duties_from, plan,  # noqa: E402
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

    # power_afe SAID: invariant 9 - with the rail down the board refuses to
    # start the task at all, and that refusal used to escape as a traceback
    # rather than a said line.
    rig = open_rig('LINKING CONVERTERS', port=args.port, power_afe=True,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return 1
    origin = rig.origin
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
    say('ok', 'AFE_ON', 'on for this run, and put back the way it was found')

    # Every channel the board reports, summed AND SHAPED on the board rather
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


#: The stand-in's machine, for the meters to show. Simulated only: a
#: bench with the stage down reads three offsets and their noise, and so
#: does the stand-in now that its phases carry the machine's current and
#: nothing invented (values.py) - so the drive holds a current vector
#: turning at DEMO_HZ electrical, "one electrical revolution every seven
#: seconds or so, slow enough to watch", and the current runs up and down
#: DEMO_AMPS over DEMO_S: a machine at crawl. On a board nothing here
#: touches the stage; the view opens onto whatever the drive is doing.
DEMO_HZ = 0.14
DEMO_AMPS = 30.0
DEMO_S = 45.0


def demo_machine(rig, origin):
    """Turn the stand-in's machine; the per-frame step that runs it up
    and down, or None on a board."""

    if origin.real:
        return None
    rig.board.gate_drivers.bypass_break(True)
    rig.board.gate_drivers.enable()
    drive = rig.drive
    drive.source('model')
    # The stand-in's record clamps the current at 5 A; the meters are 100 A
    # wide.
    drive.set_params(drv_i_max_ma=DEMO_AMPS)
    drive.setpoint(id_ref=0.0, iq_ref=0.0, theta=0.0,
                   omega_target=2.0 * math.pi * DEMO_HZ)
    drive.mode('hold')
    began = time.time()

    def step(now):
        phase = (now - began) / DEMO_S
        drive.setpoint(id_ref=DEMO_AMPS * 0.5
                       * (1.0 - math.cos(2.0 * math.pi * phase)))
    return step


def watch(rig, args, layout, chain, params):
    """Draw it until Q, ESC or the frame count runs out."""
    origin = rig.origin
    demo = demo_machine(rig, origin)
    # THE BAR FILLS THE WINDOW: at 38 columns the face floated in a sea of
    # frame.
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

    # THE LINK IS THIS THREAD'S AND NOTHING ELSE'S.
    clock = {'at': 0.0, 'state': None, 'triggers': None, 'sweeps': 0.0}
    # The library's own reader, sampled BEFORE the drain below empties it.
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
        # The level BEFORE the drain: after it every queue is empty by
        # construction and the gauge would read nothing on a task that is only
        # just keeping up.
        take_link(link, rig.buffered, now)
        for record in drain(rig):
            if len(inbox) >= HOST_RING:
                # THE HOST'S OWN DROP, counted rather than hidden.
                inbox.popleft()
                host['dropped'] += 1
            inbox.append(record)
        host['held'] = len(inbox)
        host['peak'] = max(host['peak'], host['held'])
        return clock['state']

    # NOT TIED TO THE FRAME RATE.
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
        # Stopped BEFORE anything puts the board back: two threads on one
        # serial transport is the one thing this arrangement must not do.
        feed.stop()
        done = [('acquisition', 'task stopped')]
        rig.close()
        done.append(('AFE_ON', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
