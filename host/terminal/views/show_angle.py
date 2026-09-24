#!/usr/bin/env python3
"""The shaft angle, redrawn in this terminal until you close it.

    python terminal/views/show_angle.py
    python terminal/views/show_angle.py --port COM4 --hz 20

Reads the board's parts list for an A1335, powers what powers it, and then
reads the poll loop's shared record - the board reads the sensor, this reads
the board. The drawing itself is `coaxial.draw.dial`, which is pure and tested.

Nothing here judges an angle. It shows the counts the part reported and the
degrees that follow from them - invariant 10 applies to a shaft angle exactly
as it applies to a voltage.
"""
import argparse
import os
import sys
import time
from contextlib import suppress

from coaxial.devices import angle
from coaxial.draw import dial
from coaxial.errors import RigError
from machine import ansi
from terminal.loader import TO_MENU
from terminal.ui import aspect as _aspect, screen as _screen
from terminal.ui.screen import Freshness, closing, open_rig, run_view, say, steady
from terminal.ui.stage import frame_of, hud, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

REG_ANG = 0x20
REG_TSEN = 0x28
REG_FIELD = 0x2A

#: The face, in character cells, bounded by the height: a dot is square
#: and the circle round. The dot matrix has four times the rows and twice
#: the columns of the 52 by 21 character face, whose rim was thirteen
#: columns of full stops. Cut from 64 by 23 on the bench's word once the
#: face was drawn round on its terminal (`aspect.aspect_of`, the rotor
#: observer's probe): like the motor page, an instrument that fills its
#: box - sized to the terminal every frame (2026-09-24); these where it does
#: not say, ART_LEAST the fewest rows it is drawn in.
ART_WIDTH, ART_HEIGHT = 58, 21
ART_LEAST = 12

#: The scales either side of the face - the die's temperature left, the
#: field right - and the face gives way to them: the viewport is what
#: the terminal leaves after the instrument column and the frames, and
#: the face is drawn as wide as the room left after two scales and their
#: air, down to FACE_MIN. A fixed 124-column gate showed SHAFT ANGLE
#: without them on the bench's narrower terminal (2026-09-07). The
#: face is bounded by its height anyway: at 21 rows the rim is 33 dots
#: however wide the box, and at FACE_MIN it is 29 - an eighth smaller,
#: with the scales beside it. Under that the face stands alone;
#: `--scales` and `--no-scales` force either way.
FACE_MIN = 36
#: What the stage takes off a terminal's columns before the art: the
#: instrument column and the viewport's frame and padding; off its rows:
#: the band, the key bar, the viewport's frame and the caption under the face.
STAGE_COLS = 40 + 4
STAGE_ROWS = 5
#: How often TSEN and FIELD are read again while the page runs. Each read
#: is one register through `configuring()`, which stops the poll loop on
#: ANG and starts it again, so the angle loses a reading or two every
#: SIDE_EVERY - the price of a die temperature and a field that are live
#: rather than the ones the page opened with. Neither moves fast.
SIDE_EVERY = 5.0


def capability(board):
    """The board's own entry for its angle sensor, or None if it has none."""
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
    """
    say('ok', 'capability', '%s - %s, on %s'
        % (part['name'], part['what'], part['where']))
    say('ok', part['power'] or 'supply',
        'on for this run, and put back the way it was found')

    field = kelvin = None
    with board.angle.configuring():
        field = angle.gauss(board.angle.peek(REG_FIELD)['value'])
        kelvin = angle.kelvin(board.angle.peek(REG_TSEN)['value'])
        board.angle.configure(REG_ANG)

    say('ok', 'die temperature', '%.1f K, %.1f C' % (kelvin, kelvin - 273.15))
    say('ok' if field >= 30 else 'warn', 'field', '%d gauss%s'
        % (field, '' if field >= 30 else ' - no magnet at the sensor'))
    say('ok', 'poll loop', 'ANG, read by the board; this reads the board')

    return field, kelvin


def fit(columns, lines=0, forced=None):
    """(scales, face width, face height) for a terminal `columns` wide and
    `lines` tall - 0 for one that would not say - and `forced` True or
    False from the command line, None to decide from the width."""
    room = columns - STAGE_COLS if columns else 0
    tall = max(ART_LEAST, lines - STAGE_ROWS) if lines else ART_HEIGHT
    with_scales = room - 2 * (dial.SCALE_W + 1)
    scales = (with_scales >= FACE_MIN) if forced is None else forced
    if scales and room:
        return True, max(FACE_MIN, with_scales), tall
    if room:
        return scales, max(FACE_MIN, room), tall
    return scales, ART_WIDTH, tall


def reread(board, field, kelvin):
    """TSEN and FIELD again, the poll loop put back on ANG after; what
    was known before, if the board refuses."""
    with suppress(RigError):
        with board.angle.configuring():
            field = angle.gauss(board.angle.peek(REG_FIELD)['value'])
            kelvin = angle.kelvin(board.angle.peek(REG_TSEN)['value'])
            board.angle.configure(REG_ANG)
    return field, kelvin


def _foot(colour, degrees, field, width=ART_WIDTH):
    """The reading under the face, in the needle's own colour."""

    text = dial.caption(degrees, field)
    line = ' ' * max(0, (width - len(text)) // 2) + text
    return ansi.paint(line, dial.INK[dial.NEEDLE]) if colour else line


def _face(degrees, field, kelvin, width, height, aspect, colour, scales):
    """The dial between its two scales, or alone with its caption
    under it."""
    if scales:
        return dial.instrument(degrees, field, kelvin, width, height,
                               aspect, colour=colour)
    return '\n'.join([dial.render(degrees, width, height, field,
                                  aspect=aspect, colour=colour),
                      _foot(colour, degrees, field, width)])


def compose(origin, console, part, state, field, kelvin, rate, note,
            aspect=(dial.CELL_ASPECT, 'assumed'), scales=False,
            width=ART_WIDTH, height=ART_HEIGHT):
    """One frame on the stage: the dial left, the target's numbers right."""

    if state is None:
        art, side = 'no reading', []
    else:
        counts = angle.counts(state['value'])
        degrees = state.get('degrees', counts * 360.0 / 4096.0)
        weak = field is not None and field < dial.WEAK_GAUSS

        # Coloured at the render, not after it: a braille cell carries dots
        # from up to eight places and its glyph does not say which, so there is
        # nothing for a `colourise(text)` to key on.
        art = _face(degrees, field, kelvin, width, height, aspect[0],
                    console.is_terminal, scales)

        side = [hud(part['name'], [
                    ('angle', '--   (no magnet)' if weak
                     else '%8.2f deg' % degrees),
                    ('counts', '%4d of 4096   flags %X'
                     % (counts, state['value'] >> 12)),
                    ('field', '%d gauss' % (field or 0)),
                    ('die', '%.1f C' % ((kelvin or 273.15) - 273.15)),
                    ('cell', '%.2f tall %s' % aspect)]),
                hud('LOOP', [
                    ('state', str(state.get('loop', '?'))),
                    ('rate', '%.0f readings/s' % rate),
                    ('errors', '%d  last %s'
                     % (state.get('errors', 0),
                        state.get('error') or '-'))])]

    return frame_of(console, origin, 'SHAFT ANGLE', art, side,
                    (('Q', 'EXIT'), ('ESC', 'MENU'), ('', note)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--hz', type=float, default=20.0,
                        help='screen refreshes per second')
    parser.add_argument('--simulated', action='store_true',
                        help='the stand-in, without probing for a board')
    parser.add_argument('--frames', type=int, default=0,
                        help='stop after this many, instead of running until '
                             'closed')
    parser.add_argument('--cell-aspect', type=float, default=None,
                        help='how tall this terminal\'s cell is against its '
                             'width; asked of the terminal when not given')
    parser.add_argument('--scales', dest='scales', action='store_true',
                        default=None,
                        help='the die temperature and the field as scales '
                             'either side of the face, the face giving way '
                             'to them; the default is yes wherever the '
                             'terminal leaves the face %d columns or more'
                             % FACE_MIN)
    parser.add_argument('--no-scales', dest='scales', action='store_false')
    args = parser.parse_args(argv)

    # power_afe given: the default is False, as no connect flips the rail,
    # and the part this view shows is AFE-powered; rig.close() releases the
    # rail (`Coaxial63100._release_afe`).
    rig = open_rig('LINKING A1335', port=args.port, power_afe=True,
                   simulated=bool(args.simulated))
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

    board_view = stage()
    terminal = board_view.is_terminal
    leaving = None
    # The cell's shape, asked once: the face is drawn round for this terminal
    # the way the rotor observer's can is, and the box says whether it was
    # measured.
    aspect = _aspect.aspect_of(args.cell_aspect)
    say('ok', 'cell', '%.2f tall, %s' % aspect)
    try:
        columns = os.get_terminal_size().columns
    except OSError:
        columns = 0
    forced = args.scales
    scales, width, _tall = fit(columns, 0, forced)
    say('ok', 'scales', 'die and field beside a face %d wide, read again '
        'every %.0f s' % (width, SIDE_EVERY) if scales
        else 'off - the face alone at %d, %d columns leave no room'
        % (width, columns))
    side = {'at': time.time(), 'field': field, 'kelvin': kelvin}

    def draw():
        state = steady(board.angle.state)
        tally.take(state['updates'] if state is not None else None)
        size = board_view.size if terminal else None
        scales, width, tall = fit(size.width if size else 0, size.height if size else 0,
                                  forced)
        if scales and time.time() - side['at'] >= SIDE_EVERY:
            side['field'], side['kelvin'] = reread(board, side['field'],
                                                   side['kelvin'])
            side['at'] = time.time()
        return compose(origin, board_view, part, state, side['field'],
                       side['kelvin'], tally.rate, tally.note, aspect,
                       scales=scales, width=width, height=tall)

    try:
        leaving = run_view(board_view, terminal, period, args.frames, draw)
    finally:
        done = [('poll loop', 'running, as the board left it'),
                ('registers', 'untouched - this view only reads')]
        rig.close()
        done.append((part['power'] or 'supply', 'back the way it was found'))
        sys.stdout.write('\n')
        closing(done, terminal, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
