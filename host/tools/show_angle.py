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
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coaxial import dial                                   # noqa: E402
from coaxial.errors import RigError                        # noqa: E402
from coaxial_mcp.session import open_session               # noqa: E402
from screen import paint, say                              # noqa: E402

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

    Returns (afe_was_on, field, kelvin). The field and the temperature are
    read once: the loop reads one register at a time, so polling them would
    cost the angle its sample rate, and neither moves at the rate the angle
    does.
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
            time.sleep(0.3)
            say('ok', part['power'], 'on')

    field = kelvin = None
    with board.angle.configuring():
        field = board.angle.read(REG_FIELD)['value'] & 0x0FFF
        kelvin = (board.angle.read(REG_TSEN)['value'] & 0x0FFF) / 8.0
        board.angle.poll_register(REG_ANG)

    say('ok', 'die temperature', '%.1f K, %.1f C' % (kelvin, kelvin - 273.15))
    say('ok' if field >= 30 else 'warn', 'field', '%d gauss%s'
        % (field, '' if field >= 30 else ' - no magnet at the sensor'))
    say('ok', 'poll loop', 'ANG, read by the board; this reads the board')

    return was_on, field, kelvin


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

    session, origin = open_session(args.port,
                                   simulated=True if args.simulated else None)
    say('ok' if origin.real else 'warn', 'link',
        '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))

    board = session.board

    part = capability(board)
    if part is None:
        say('fail', 'capability',
            'this board reports no angle sensor among its parts')
        session.close()
        return 1

    try:
        afe_was_on, field, _ = preflight(board, part)
    except RigError as exc:
        say('fail', part['power'] or 'supply',
            'could not set the sensor up: %s' % exc)
        session.close()
        return 1

    say('wait', 'drawing', 'Ctrl+C stops it and undoes all of the above')

    period = 1.0 / max(args.hz, 0.5)
    frame = 0
    seen = -1
    stale = 0

    console = sys.stdout.isatty()
    if console:
        if os.name == 'nt':
            os.system('')    # enables ANSI on a Windows console
        sys.stdout.write(chr(27) + '[2J')
    shown = []

    try:
        while True:
            try:
                state = board.angle.state()
            except RigError:
                state = None

            if state is None or state['updates'] == seen:
                stale += 1
            else:
                seen, stale = state['updates'], 0

            frame += 1
            if state is not None:
                state = dict(state, field=field)

            note = '' if not stale else '   no new reading for %d frame%s' % (
                stale, '' if stale == 1 else 's')
            lines = (['coaxial_63100 - shaft angle   (Ctrl+C to leave)' + note,
                      ''] +
                     (dial.picture(state) if state else 'no reading').split('\n'))
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
            sys.stdout.write('\n')
            if not afe_was_on and part['power']:
                board.afe.disable()
                say('ok', part['power'], 'off again - it was off before this')
            else:
                say('ok', part['power'] or 'supply', 'left on, as it was found')
        except RigError as exc:
            sys.stdout.write('\n')
            say('fail', 'putting it back', str(exc))
        session.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
