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
from coaxial.thermal import NODES, tau_minutes            # noqa: E402
from coaxial.thermalmap import render                     # noqa: E402


def lines_for(state, console):
    """The whole frame under the banner, as a list of lines."""
    nodes = state['nodes']
    board_c = nodes.get('board')
    if board_c is None:
        return ['  the board sent no board node - device 8 is out of step']

    out = ['  measured                 sampling every %.1f s, settle %.0f ms'
           % (state['sample_every_s'], state['sample_settle_s'] * 1000)]
    if state['ntc'] is None:
        out.append('    NTC          -        AFE off, so the gate drivers '
                   'have supply and the model runs open')
    else:
        out.append('    NTC       %6.2f C    beside the middle gate driver'
                   % state['ntc'])
        out.append('    model     %6.2f C    error %+.2f K'
                   % (state['expected_ntc'], state['error']))

    out.append('')
    out.append('  estimated                open for %4d s   %s'
               % (state['seconds'],
                  'settled' if state['settled']
                  else 'settling, tau %.1f min' % tau_minutes()))
    for name in NODES:
        if name in nodes:
            out.append('    %-11s %6.2f C' % (name, nodes[name]))
    out.append('    %-11s %6.2f C' % ('board', board_c))
    out.append('    %-11s %6.2f C' % ('ambient', state['ambient']))
    out.append('')

    zones = {k: v for k, v in nodes.items() if k != 'board'}
    out.extend(render(zones, board_c=board_c, colour=console).split('\n'))
    return out


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
        frame, shown, leaving, body = 0, [], None, ['  waiting for device 8']

        try:
            with Keys(console) as keys:
                while True:
                    try:
                        body = lines_for(rig.board.thermal.state(), console)
                    except (NoReplyError, RigError) as exc:
                        # Keep the last good frame: the link goes quiet now
                        # and then (FINDINGS), and blanking the picture every
                        # time it does makes the view unreadable.
                        body = body[:1] + ['  link quiet: %s' % exc]

                    frame += 1
                    lines = [banner(origin, 'thermal - observer', console,
                                    'Q closes, ESC for the menu   frame %d'
                                    % frame), ''] + body
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
