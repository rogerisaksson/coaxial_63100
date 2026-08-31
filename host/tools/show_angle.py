#!/usr/bin/env python3
"""The shaft angle, redrawn in this terminal until you close it.

    python tools/show_angle.py
    python tools/show_angle.py --port COM4 --hz 20

Reads the board's parts list for an A1335, powers what powers it, and then
reads the poll loop's shared record - the board reads the sensor, this reads
the board. The drawing itself is `coaxial.dial`, which is pure and tested.

Nothing here judges an angle. It shows the counts the part reported and the
degrees that follow from them - invariant 10 applies to a shaft angle exactly
as it applies to a voltage.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coaxial import angle                                  # noqa: E402
from coaxial import dial                                   # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from screen import (closing, Freshness, say, stamp_crosses,  # noqa: E402
                    steady, TO_MENU)

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

REG_ANG = 0x20
REG_TSEN = 0x28
REG_FIELD = 0x2A


def capability(board):
    """The board's own entry for its angle sensor, or None if it has none.

    Read from the board, not decided here: `channels` kind 4 is the parts
    list the firmware carries, so a board without the part says so itself.
    """
    try:
        parts = board.system.channel_map()['parts']
    except (RigError, KeyError):
        return None

    for part in parts:
        if part['name'].startswith('A13') or 'angle' in part['what'].lower():
            return part
    return None


def preflight(board, part):
    """Power what the part needs, read what does not change, and start the
    loop on the angle register.

    Returns (field, kelvin), read once: the loop reads one register at a
    time, so polling them would cost the angle its sample rate, and neither
    moves at the rate the angle does. The supply is Coaxial63100's - it
    brings AFE_ON up on the way in and puts it back on the way out.
    """
    say('ok', 'capability', '%s - %s, on %s'
        % (part['name'], part['what'], part['where']))
    say('ok', part['power'] or 'supply',
        'on for this run, and put back the way it was found')

    field = kelvin = None
    with board.angle.configuring():
        field = angle.gauss(board.angle.read(REG_FIELD)['value'])
        kelvin = angle.kelvin(board.angle.read(REG_TSEN)['value'])
        board.angle.poll_register(REG_ANG)

    say('ok', 'die temperature', '%.1f K, %.1f C' % (kelvin, kelvin - 273.15))
    say('ok' if field >= 30 else 'warn', 'field', '%d gauss%s'
        % (field, '' if field >= 30 else ' - no magnet at the sensor'))
    say('ok', 'poll loop', 'ANG, read by the board; this reads the board')

    return field, kelvin


def compose(origin, console, part, state, field, kelvin, rate, note):
    """One frame on the stage: the dial left, the target's numbers right."""
    from screen import frame_of, hud

    if state is None:
        art, side = 'no reading', []
    else:
        counts = angle.counts(state['value'])
        degrees = state.get('degrees', counts * 360.0 / 4096.0)
        weak = field is not None and field < dial.WEAK_GAUSS

        face = dial.render(degrees, 52, 21, field)
        art_lines = (dial.colourise(face) if console else face).splitlines()
        margin = min((len(l) - len(l.lstrip(' '))
                      for l in art_lines if l.strip()), default=0)
        art = '\n'.join(
            stamp_crosses([l[max(0, margin - 1):] for l in art_lines], 48))

        side = [hud(part['name'], [
                    ('angle', '--   (no magnet)' if weak
                     else '%8.2f deg' % degrees),
                    ('counts', '%4d of 4096   flags %X'
                     % (counts, state['value'] >> 12)),
                    ('field', '%d gauss' % (field or 0)),
                    ('die', '%.1f C' % ((kelvin or 273.15) - 273.15))]),
                hud('LOOP', [
                    ('state', str(state.get('loop', '?'))),
                    ('rate', '%.0f readings/s' % rate),
                    ('errors', '%d  last %s'
                     % (state.get('errors', 0),
                        state.get('error') or '-'))])]

    return frame_of(console, origin, 'SHAFT ANGLE', art, side,
                    (('Q', 'EXIT'), ('ESC', 'MENU'), ('', note)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='screen refreshes per second')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed')
    args = parser.parse_args(argv)

    # power_afe SAID: the default went quiet-False when every connect
    # stopped flipping the rail, and this view inherited it - the part it
    # exists to show is AFE-powered, so it asks by name and puts it back.
    from screen import open_rig
    rig = open_rig('LINKING A1335', port=args.port, power_afe=True,
                   simulated_device=bool(args.simulated))
    if rig is None:
        return 1
    origin, board = rig.origin, rig.board
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))

    part = capability(board)
    if part is None:
        say('fail', 'capability',
            'this board reports no angle sensor among its parts')
        rig.close()
        return 1

    try:
        field, kelvin = preflight(board, part)
    except RigError as exc:
        say('fail', part['power'] or 'supply',
            'could not set the sensor up: %s' % exc)
        rig.close()
        return 1

    say('wait', 'drawing',
        'Q closes it, ESC goes back to the menu, and both undo the above')

    period = 1.0 / max(args.hz, 0.5)
    tally = Freshness()

    from screen import run_view, stage

    board_view = stage()
    console = board_view.is_terminal
    leaving = None

    def draw():
        state = steady(board.angle.state)
        tally.take(state['updates'] if state is not None else None)
        return compose(origin, console, part, state, field, kelvin,
                       tally.rate, tally.note)

    try:
        leaving = run_view(board_view, console, period, args.frames, draw)
    finally:
        done = [('poll loop', 'running, as the board left it'),
                ('registers', 'untouched - this view only reads')]
        rig.close()
        done.append((part['power'] or 'supply', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
