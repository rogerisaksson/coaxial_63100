"""The terminal's cell aspect: asked of the terminal, else the flag, else 2.0."""
import re
import select
import sys
import time

from coaxial.draw.ascii3d import CELL_ASPECT


#: XTWINOPS: `CSI 14 t` answers the size in pixels, `CSI 18 t` in cells -
#: between them the cell's aspect, which every round drawing depends on.
ASPECT_QUERY = '\033[14t\033[18t'

#: How long to wait for the answer. A terminal that supports it replies
#: at once; one that does not never will, and a view must not hang on a
#: cosmetic question.
ASPECT_WAIT = 0.15

#: What a cell's height-over-width may plausibly be. Outside this the
#: reply is not a cell - a terminal reporting its size in some other unit
#: would otherwise stretch the drawing into a smear.
ASPECT_RANGE = (1.2, 3.2)


def cell_aspect_of(pixels, cells):
    """Height over width for one cell, from `(h, w)` pixels and `(rows,
    cols)` cells.
    """
    try:
        ph, pw = float(pixels[0]), float(pixels[1])
        rows, cols = float(cells[0]), float(cells[1])
    except (TypeError, ValueError, IndexError):
        return None
    if min(ph, pw, rows, cols) <= 0:
        return None
    aspect = (ph / rows) / (pw / cols)
    lo, hi = ASPECT_RANGE
    return aspect if lo <= aspect <= hi else None


def probe_aspect(console=True, wait=ASPECT_WAIT):
    """Ask the terminal how tall its cell is against its width."""
    # Both ends a tty, or the query itself is the damage: written to a pipe
    # the escape lands in whatever is reading the render.
    if not console or not sys.stdin.isatty() or not sys.stdout.isatty():
        return None
    saved = posix = None
    try:
        if sys.platform != 'win32':
            import termios
            import tty
            try:
                posix, saved = termios, termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            except (termios.error, OSError, ValueError):
                # stdin not a tty, or without a descriptor
                saved = None
        out = sys.__stdout__ or sys.stdout
        out.write(ASPECT_QUERY)
        out.flush()
        got, until = '', time.monotonic() + wait
        while time.monotonic() < until and got.count('t') < 2:
            got += _read_now()
        found = re.findall(r'\033\[(4|8);(\d+);(\d+)t', got)
        seen = {kind: (int(a), int(b)) for kind, a, b in found}
        if '4' not in seen or '8' not in seen:
            return None
        return cell_aspect_of(seen['4'], seen['8'])
    except (OSError, ValueError, TypeError):    # the terminal's answer, or none
        return None
    finally:
        if saved is not None and posix is not None:
            posix.tcsetattr(sys.stdin, posix.TCSADRAIN, saved)


def _read_now():
    """Whatever is waiting on stdin this instant, or ''."""
    try:
        import msvcrt
    except ImportError:

        if select.select([sys.stdin], [], [], 0.01)[0]:
            return sys.stdin.read(1)
        return ''
    got = ''
    while msvcrt.kbhit():
        got += msvcrt.getwch()
    if not got:
        time.sleep(0.01)
    return got


def aspect_of(cell_aspect=None):
    """`(aspect, how)`: what makes a circle round on this terminal."""
    if cell_aspect is not None:
        return cell_aspect, 'given'
    seen = probe_aspect()
    if seen:
        return seen, 'measured'
    return CELL_ASPECT, 'assumed'
