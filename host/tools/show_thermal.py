"""The board's thermal observer, live: one measurement and five estimates.

The estimates come off the wire, not from this file. `0x6E` device 8 is the
observer running in the firmware at 10 Hz, and drawing anything the host
recomputed would be a second answer to a question the board already settles.

**The AFE stays as it was found.** The gate is inverted, so switching it on
takes the gate drivers' supply away - a thermal view that powered the AFE
would stop the load it is there to watch. With it off there is no NTC either,
and the observer runs open on power and time; `open for` says how long.

`--switch` drives the load from inside this view. Not a convenience: the port
is exclusive, so `switch.py` running beside this would keep it, and there was
otherwise no way to watch the zones move while anything switched. The gates go
down through the same `finally` that puts the screen back.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from screen import (TO_MENU, Keys, banner, clear, paint,  # noqa: E402
                    say)

from coaxial import Coaxial63100                          # noqa: E402
from coaxial.errors import NoReplyError, RigError         # noqa: E402
from coaxial.thermal import tau_minutes                   # noqa: E402
from coaxial.thermalmap import SCALE_LINES, render        # noqa: E402

#: Above the picture: a blank, the banner, a blank, the state line,
#: the budget line, a blank.
#:
#: The FIRST blank is not decoration. `paint` addresses absolute rows from 1,
#: so without it the banner lands on the terminal's top row - underneath the
#: shell's own decoration, where the LIVE/SIMULATED tag cannot be read. That
#: tag is the one thing in the frame that must never be hidden.
HEAD_LINES = 6

#: Below the scale: the keys and the blank under them - TRAILING already
#: covers the blank above. The keys sit at the bottom and the state at the
#: top because they answer different questions - one is what to press, the
#: other is what the colours mean - and reading them as one crowded line was
#: what made the frame feel closed in.
FOOT_LINES = 2

#: Blank lines between the scale and the keys.
TRAILING = 1

#: What ESC and Q do. ESC returns TO_MENU so demo.ps1 draws its menu again.
KEYS = 'Q closes     ESC back to the menu'


def summary(state):
    """What the colours cannot say: whether a measurement is behind them.

    Not a table of the nodes - the picture already carries those, and a list
    beside it says the same numbers twice. With AFE_ON low there is no NTC at
    all and the nodes run open on power and time, which is the one thing that
    changes how the picture should be read.
    """
    # A STALE SAMPLE IS NOT A MEASUREMENT. The board reports the last one it
    # took together with its age, and judging the age is the host's job
    # (invariant 10). Measured 2026-08-28: during a switching run this line
    # printed `NTC 36.0 C` with a model error that grew 7.75 -> 12.46 K,
    # because the reading was frozen from before the rail went down and only
    # the model was moving. Two samples' grace, so a late one does not blink.
    age = state.get('seen_s_ago')
    every = state.get('sample_every_s') or 0.0
    fresh = state['ntc'] is not None and (
        age is None or every <= 0.0 or age <= 2.0 * every)

    if state['ntc'] is None:
        anchor = 'AFE off, open loop'
    elif not fresh:
        anchor = 'NTC %.1f C, %.0f s old - open loop since' % (state['ntc'],
                                                              age)
    else:
        anchor = 'NTC %.1f C, error %+.2f K' % (state['ntc'], state['error'])
    return '%s     open %d s     %s' % (
        anchor, state['seconds'],
        'settled' if state['settled'] else 'settling')


def budget_line(got):
    """The thermal budget as one line: how close, to what, and how long.

    A bar rather than degrees. The question a burst asks is how much of the
    ceiling is spent, and a temperature does not answer that without the
    ceiling beside it - so the board sends the fraction and this draws it.
    """
    used, worst = got['worst'], got['worst_node']
    width = 24
    full = int(round(max(0.0, min(1.0, used)) * width))
    bar = ('#' * full) + ('.' * (width - full))
    left = got['seconds_to_limit']

    where = ('tripped' if got['tripped']
             else 'THROTTLING' if got['throttling'] else 'ok')
    return '  [%s] %3.0f %% %-11s %-10s %s' % (
        bar, 100.0 * used, worst, where,
        ('%.1f s to limit' % left) if left is not None else 'not heating')


def picture(state, console, reserve):
    """The board and its scale. Nothing else."""
    nodes = state['nodes']
    board_c = nodes.get('board')
    if board_c is None:
        return ['  the board sent no board node - device 8 is out of step']

    zones = {k: v for k, v in nodes.items() if k != 'board'}
    return render(zones, board_c=board_c, colour=console,
                  reserve=reserve, trailing=TRAILING).split('\n')


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
    a = p.parse_args()

    # power_afe=False, and it is not a preference. AFE_ON high unpowers the
    # gate drivers, so opening the rig the usual way would stop the switching
    # this view exists to watch.
    with Coaxial63100(port=a.port, simulated_device=a.simulated,
                      power_afe=False) as rig:
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

        console = sys.stdout.isatty()
        if console:
            if os.name == 'nt':
                os.system('')           # enables ANSI on a Windows console
            sys.stdout.write(chr(27) + '[2J')

        period = 1.0 / max(a.hz, 0.2)
        # Everything in the frame that is not picture, so `render` can size
        # the board to what is left. Counted, not guessed - a guess is what
        # clipped the bottom edge off.
        reserve = HEAD_LINES + 1 + SCALE_LINES + TRAILING + FOOT_LINES
        frame, shown, leaving = 0, [], None
        body, note = ['  waiting for device 8'], 'starting'
        spend = '  budget: waiting'

        try:
            with Keys(console) as keys:
                while True:
                    try:
                        got = rig.board.thermal.state()
                        note = summary(got)
                        spend = budget_line(rig.board.thermal.budget())
                        body = picture(got, console, reserve)
                    except (NoReplyError, RigError) as exc:
                        # Keep the last good picture: the link goes quiet now
                        # and then (FINDINGS), and blanking the board every
                        # time it does makes the view unreadable.
                        note = 'link quiet: %s' % exc

                    frame += 1
                    lines = (['', banner(origin, 'thermal - observer',
                                         console, origin.label),
                              '', '  ' + note, spend, '']
                             + body + ['  ' + KEYS, ''])
                    sys.stdout.write(paint(shown, lines, console))
                    sys.stdout.flush()
                    shown = lines

                    if a.frames and frame >= a.frames:
                        break
                    leaving, _moved = keys.poll()
                    if leaving:
                        break
                    time.sleep(period)
        except KeyboardInterrupt:
            pass
        finally:
            clear(console)
            if load is not None:
                for undo in (lambda: rig.write(analog=dict.fromkeys(load, 0.0)),
                             rig.gates.disarm):
                    try:
                        undo()
                    except (NoReplyError, RigError):
                        pass       # one failed step must not skip the next
                say('ok', 'board', 'duty zeroed and the gate drivers disarmed')
            else:
                say('ok', 'board', 'AFE_ON untouched, nothing disarmed')

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
