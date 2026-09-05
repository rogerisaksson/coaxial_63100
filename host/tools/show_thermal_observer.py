"""The board's thermal observer, live: one measurement and five estimates.

The estimates come off the wire, not from this file. `0x6E` device 8 is the
thermal observer running in the firmware at 10 Hz, and drawing anything the host
recomputed would be a second answer to a question the board already settles.

**The AFE stays as it was found.** The gate is inverted, so switching it on
takes the gate drivers' supply away - a thermal view that powered the AFE
would stop the load it is there to watch. With it off there is no NTC either,
and the thermal observer runs open on power and time; `open for` says how long.

`--switch` drives the load from inside this view. Not a convenience: the port
is exclusive, so `switch.py` running beside this would keep it, and there was
otherwise no way to watch the zones move while anything switched. The gates go
down through the same `finally` that puts the screen back.
"""
import argparse
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from screen import (closing, say, stamp_crosses, TO_MENU,  # noqa: E402
                    visible)

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

from coaxial import Coaxial63100                          # noqa: E402
from coaxial.errors import NoReplyError, RigError         # noqa: E402
from coaxial import gauges                                 # noqa: E402
from coaxial.thermal import ALL_NODES, pretty              # noqa: E402
from coaxial.thermalmap import (CELL_ASPECT, SCALE_LINES,  # noqa: E402
                                render)

#: Above the picture: a blank, the banner, a blank, the state line,
#: the budget line, a blank.
#:
#: The FIRST blank is not decoration. `paint` addresses absolute rows from 1,
#: so without it the banner lands on the terminal's top row - underneath the
#: shell's own decoration, where the LIVE/SIMULATED tag cannot be read. That
#: tag is the one thing in the frame that must never be hidden.
# The stage's frame around the map: the title band (1), the viewport's
# top and bottom edge (2), and the gutter row the crosses sit in. It was 6
# from the banner era, and the map drew two rows smaller than the screen
# allowed.
HEAD_LINES = 4

#: Below the scale: the keys and the blank under them - TRAILING already
#: covers the blank above. The keys sit at the bottom and the state at the
#: top because they answer different questions - one is what to press, the
#: other is what the colours mean - and reading them as one crowded line was
#: what made the frame feel closed in.
FOOT_LINES = 1

#: Blank lines between the scale and the keys. Zero since 2026-08-30:
#: the row went to the board, which was asked a size up.
TRAILING = 0

#: What ESC and Q do. ESC returns TO_MENU so coaxial_tty.ps1 draws its menu again.


#: The status panel's field width, map margin included. Fixed, so the map
#: never breathes when a number changes length.
PANEL_W = 42

#: The soak bar: cells wide, and three braille rows tall on the bench's
#: word - the spend against the worst node's ceiling, the one level on
#: the page that is not a temperature.
SOAK_CELLS = 16
SOAK_ROWS = 3


def soak(budget):
    """The HEADROOM box's rows: the spend as a bar SOAK_ROWS tall in the
    margin's colour, the throttle point marked through it, the figure on
    its middle row beside the label. It was `[⣿⣿⠒⠒] 42 %` on one row;
    the bench asked for the brackets gone and three rows of braille."""
    from rich.text import Text

    from coaxial.thermal_device import THROTTLE_AT

    used = budget['worst']
    lines = gauges.bar(used, SOAK_CELLS, SOAK_ROWS,
                       cls=gauges.margin_class(used, budget['tripped']),
                       marks=[(THROTTLE_AT, gauges.MARK)])
    rows = []
    for index, line in enumerate(lines):
        if index == SOAK_ROWS // 2:
            rows.append(('soak', Text.from_ansi('%s  %3.0f %%'
                                                % (line, 100.0 * used))))
        else:
            rows.append(('', Text.from_ansi(line)))
    return rows


def status_boxes(state, budget, aspect=None):
    """The thermal observer's numbers as instrument boxes, every one the
    board's - and, given `(aspect, how)`, the one number that is the
    terminal's: how tall its cell was measured, or assumed, to be."""
    from rich.text import Text

    from screen import hud

    age = state.get('seen_s_ago')
    every = state.get('sample_every_s') or 0.0
    fresh = state['ntc'] is not None and (
        age is None or every <= 0.0 or age <= 2.0 * every)

    if state['ntc'] is None:
        sense = [Text('AFE off - open loop', style='value')]
    elif not fresh:
        sense = [('NTC', '%.1f C' % state['ntc']),
                 Text('%.0f s old - open loop' % age, style='value')]
    else:
        sense = [('NTC', '%.1f C   err %+.2f K'
                  % (state['ntc'], state['error']))]
    sense += [('open', '%d s   %s' % (state['seconds'],
                                      'settled' if state['settled']
                                      else 'settling')),
              ('sample', 'every %.0f s - last %s'
               % (every, '%.0f s ago' % age if age is not None else '-'))]
    if aspect is not None:
        sense.append(('cell', '%.2f tall %s' % aspect))

    boxes = [hud('SENSE', sense)]
    if budget is not None:
        left = budget['seconds_to_limit']
        state_text = ('TRIPPED' if budget['tripped']
                      else 'THROTTLING' if budget['throttling'] else 'ok')
        boxes.append(hud('HEADROOM', soak(budget) + [
            ('worst', Text.assemble(
                (pretty(budget['worst_node']), 'name'), '   ',
                (state_text, 'value' if state_text != 'ok' else 'label'))),
            ('to limit', ('%.0f s' % left) if left is not None
             else 'not heating')]))
    # Every node the thermal observer estimates, by name, plus what is MEASURED
    # (the dies, the ambient it infers). The map shows where; this shows
    # how much, to the decimal.
    nodes = state.get('nodes') or {}
    rows = [(pretty(name), '%.1f C' % nodes[name])
            for name in ALL_NODES if name in nodes]
    rows += [('ambient', '%.1f C' % (state.get('ambient') or 0.0)),
             ('mcu die', '%.1f C' % state['mcu']
              if state.get('mcu') is not None else '-'),
             ('a1335 die', '%.1f C' % state['afe']
              if state.get('afe') is not None else '-')]
    boxes.append(hud('LEVELS', rows))
    boxes.append(hud('TUBES  %.0f to %.0f C' % (gauges.TEMP_FLOOR_C,
                                                gauges.TEMP_SCALE_C),
                     [Text.from_ansi(line) for line in tubes(state, budget)]))
    return boxes


#: What each tube is called under itself, two letters at the tubes'
#: pitch of three: the leg for the drivers and the phases, the part for
#: the rest.
SHORT_NODE = {'driver_u': 'dU', 'driver_v': 'dV', 'driver_w': 'dW',
              'phase_u': 'pU', 'phase_v': 'pV', 'phase_w': 'pW',
              'mcu': 'MC', 'regulators': 'RG', 'afe': 'AF', 'board': 'PB'}
TUBE_ROWS = 8


def tubes(state, budget):
    """The ten nodes and the thermistor as thermometers - the motor
    page's own, on this page too.

    HEIGHT IS DEGREES ON THE ONE SCALE every page shares, colour is the
    node's margin against its own ceiling (the board's bands, from the
    record), and the NTC wears the thermometer ramp because it has no
    ceiling to be a margin against. The map beside them says WHERE the
    heat sits; these say how much, against the same rulers the motor
    page uses, so a reader moving between the two pages reads one
    instrument.
    """
    nodes = state.get('nodes') or {}
    used = (budget or {}).get('used') or {}
    tripped = bool((budget or {}).get('tripped'))
    entries, labels = [], []
    for name in ALL_NODES:
        if name in nodes:
            entries.append((gauges.temp_share(nodes[name]),
                            gauges.margin_class(used.get(name, 0.0),
                                                tripped)))
            labels.append(SHORT_NODE.get(name, name[:2]))
    if state.get('ntc') is not None:
        entries += [None, (gauges.temp_share(state['ntc']),
                           gauges.thermometer_class(state['ntc']))]
        labels += ['', 'NTC']
    return gauges.tubes(entries, TUBE_ROWS, labels, pitch=3)


def picture(state, console, reserve, aspect=CELL_ASPECT):
    """The board and its scale. Nothing else.

    `aspect` is the FIELD row's height against a cell's width - half the
    character aspect `screen.aspect_of` measures, since the halftone puts
    two field rows in a character row - so the board is round on the
    terminal it is drawn on rather than on an assumed one.
    """
    nodes = state['nodes']
    board_c = nodes.get('board')
    if board_c is None:
        return ['  the board sent no board node - device 8 is out of step']

    zones = {k: v for k, v in nodes.items() if k != 'board'}
    # The leading blank moves the board one row down the frame - asked
    # 2026-08-30, and counted in the caller's reserve.
    return [''] + render(zones, board_c=board_c, colour=console,
                         margin=PANEL_W, reserve=reserve,
                         trailing=TRAILING, aspect=aspect).split('\n')


def put_back(rig, load):
    """Undo what the run armed, step by step, and say what each did.

    One failed step must not skip the next, and the way out is the only
    place that says whether it took."""
    if load is None:
        return [('AFE_ON', 'untouched - this run only watched'),
                ('gate stage', 'untouched, nothing was armed')]
    done = []
    for name, what, undo in (
            ('duty', 'three legs to zero',
             lambda: rig.write(analog=dict.fromkeys(load, 0.0))),
            ('gate stage', 'disarmed, MOE clear', rig.gates.disarm)):
        try:
            undo()
            done.append((name, what))
        except (NoReplyError, RigError) as exc:
            done.append((name, 'FAILED: %s' % exc))
    done.append(('AFE_ON', 'back the way it was found'))
    return done


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=2.0)
    p.add_argument('--frames', type=int, default=0,
                   help='stop after this many; 0 = until Q, ESC or Ctrl+C')
    p.add_argument('--switch', type=float, metavar='DUTY',
                   help='arm the gate drivers at this duty (0-1) and hold it '
                        'while drawing, so the zones have something to move')
    p.add_argument('-P', '--phases', default='U,V,W')
    p.add_argument('--cell-aspect', type=float, default=None,
                   help='how tall a character cell is against its width; '
                        'measured off the terminal when not given')
    a = p.parse_args()

    # power_afe=False, and it is not a preference. AFE_ON high unpowers the
    # gate drivers, so opening the rig the usual way would stop the switching
    # this view exists to watch.
    from screen import boot
    with boot('LINKING OBSERVER') as ready,          Coaxial63100(port=a.port, simulated_device=a.simulated,
                      power_afe=False) as rig:
        ready()
        origin = rig.origin
        say('ok' if origin.real else 'warn', 'link',
            '%s - %s' % (origin.label, 'live' if origin.real else 'simulated'))
        say('ok', 'AFE_ON', 'left exactly as found - it gates the drivers')
        say('wait', 'drawing', 'Q closes it, ESC goes back to the menu')

        load = None
        if a.switch is not None:
            legs = [x.strip().upper() for x in a.phases.split(',')]
            # AFE off FIRST, then arm. The gate is inverted: arming with the
            # AFE on gives six switching inputs and no supply behind them.
            rig.board.afe.disable()
            rig.gates.arm(bypass_sto=True, ignore_interlock=True)
            load = {'Phase ' + leg: a.switch for leg in legs}
            rig.write(analog=load)
            say('warn', 'switching', '%s at %.0f %% - AFE off, STO bypassed'
                % ('+'.join(legs), a.switch * 100))

        from screen import frame_of, run_view, stage

        board_view = stage()
        console = board_view.is_terminal
        # ROUND ON THIS TERMINAL: the field's row aspect is half the
        # character's, and the character's is asked, not assumed.
        aspect = _screen.aspect_of(a.cell_aspect)

        period = 1.0 / max(a.hz, 0.2)
        # Everything in the frame that is not picture, so `render` can size
        # the board to what is left. Counted, not guessed - a guess is what
        # clipped the bottom edge off.
        reserve = HEAD_LINES + 1 + SCALE_LINES + TRAILING + FOOT_LINES
        last = {'body': ['  waiting for device 8'], 'boxes': []}
        leaving = None

        def draw():
            try:
                got = rig.board.thermal.state()
                last['boxes'] = status_boxes(got, rig.board.thermal.budget(),
                                             aspect)
                last['body'] = picture(got, console, reserve,
                                       aspect[0] / 2.0)
            except (NoReplyError, RigError):
                pass    # keep the last good picture: the link goes quiet
                        # now and then (FINDINGS); a blank board each time
                        # made the view unreadable
            # Three cells of pad and eight of field: six and twelve read as
            # dead air around the board. The whole body is picture, all of
            # it stamped - the scale rides beside the board.
            body = last['body']
            field = max((visible(l) for l in body), default=0) + 8
            art = stamp_crosses(['   ' + l for l in body], field)
            return frame_of(board_view, origin, 'THERMAL OBSERVER',
                            '\n'.join(art), last['boxes'],
                            (('Q', 'EXIT'), ('ESC', 'MENU')))

        try:
            leaving = run_view(board_view, console, period, a.frames, draw)
        finally:
            done = put_back(rig, load)
            sys.stdout.write('\n')
            closing(done, console, 0)

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
