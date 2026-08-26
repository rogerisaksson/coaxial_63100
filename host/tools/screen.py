"""Two things every live view on this board needs: a preflight line and a
redraw that does not flicker.

Shared because both views want them identically, and a second copy of either
is the thing that drifts - the structure suite fails a definition that lives
in two files.
"""
import sys


#: Preflight colours, matching board_prompt/Say.ps1 so the two read the
#: same. Green ok, cyan waiting, yellow worth knowing, red stop.
STATES = {'ok': '32', 'wait': '36', 'warn': '33', 'fail': '31'}


def say(state, text, detail=''):
    """One preflight line, the shape board_prompt.ps1 prints.

    Written here rather than shelling out to Say.ps1 because the preflight
    and the view share one session: splitting them would open the port
    twice, and the second open is the one that finds it busy.
    """
    esc = chr(27)
    colour = STATES.get(state, '37')
    sys.stdout.write('  %s[%sm%-6s%s[0m%-22s %s[90m%s%s[0m\n'
                     % (esc, colour, state, esc, text, esc, detail, esc))
    sys.stdout.flush()


def paint(shown, lines, console):
    """What to write to move the screen from `shown` to `lines`.

    Only the rows that differ, each addressed directly. Rewriting all of it
    every frame is what made the prompt flicker: at 20 Hz the terminal
    repaints two dozen unchanged rows, so the header and the caption blink
    along with the drawing they are not part of.
    """
    if not console:
        return '\n'.join(lines) + '\n'

    out = []
    for row in range(max(len(shown), len(lines))):
        was = shown[row] if row < len(shown) else None
        now = lines[row] if row < len(lines) else ''
        if now != was:
            out.append('%s[%d;1H%s%s[K' % (chr(27), row + 1, now, chr(27)))

    return ''.join(out)
