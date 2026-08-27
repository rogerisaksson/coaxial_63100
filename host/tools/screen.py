"""Three things every live view on this board needs: a preflight line, a
redraw that does not flicker, and a way out that runs the cleanup.

Shared because every view wants them identically, and a second copy of any of
them is the thing that drifts - the structure suite fails a definition that
lives in two files.
"""
import re
import sys

#: What leaves a view, and where it leaves you. Q closes the lot; ESC goes
#: back to the menu, because picking the wrong view is the common mistake and
#: retyping the command to fix it is the annoying part. Ctrl+C still works and
#: still lands in the same `finally`.
QUIT_KEYS = frozenset({'q', 'Q'})
MENU_KEYS = frozenset({chr(27)})

#: What a view exits with when ESC sent it back. Distinct from 0 so the menu
#: can tell "show me again" from "that is all", and from 130 so neither is
#: mistaken for Ctrl+C.
TO_MENU = 64

#: Windows console input flags. A console hands mouse movement to the
#: program as MOUSE_EVENT records, which `msvcrt.getwch` cannot see - it
#: returns key events and nothing else. Setting VIRTUAL_TERMINAL_INPUT makes
#: the console translate them into the same SGR sequences an xterm sends, so
#: one parser serves both. QUICK_EDIT has to go: it keeps the mouse for
#: selecting text and the program never hears about it. EXTENDED_FLAGS is
#: what makes clearing QUICK_EDIT stick.
#:
#: Without this the wheel does nothing at all on Windows, whatever the
#: program prints to turn reporting on.
VT_INPUT = 0x0200
MOUSE_INPUT = 0x0010
EXTENDED_FLAGS = 0x0080
QUICK_EDIT = 0x0040
LINE_INPUT = 0x0002
ECHO_INPUT = 0x0004


def console_mode(was):
    """The input mode to run a mouse-driven view in, from the current one.

    Pure, so it can be checked without a console: the view that needs it
    cannot be run by a test at all.
    """
    return ((was | VT_INPUT | MOUSE_INPUT | EXTENDED_FLAGS)
            & ~QUICK_EDIT & ~LINE_INPUT & ~ECHO_INPUT)


#: Mouse reporting, xterm's SGR encoding. 1002 is button-and-drag; 1006 is
#: the encoding that survives a terminal wider than 223 columns - the older
#: one packs each coordinate into one byte and stops reporting past that,
#: which on a full-screen window is most of it.
MOUSE_ON = '\033[?1002h\033[?1006h'
MOUSE_OFF = '\033[?1006l\033[?1002l'

#: A wheel notch, as a fraction of the distance to the model. 12 % a notch is
#: about eight notches between filling the window and half of it, which is
#: roughly what one flick of a hand expects to do.
WHEEL_STEP = 0.12

#: A right-drag, per row of movement. Smaller than a notch: a drag is one
#: continuous gesture and lands twenty reports where a wheel lands one.
DRAG_STEP = 0.02


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


def clear(console):
    """Wipe the screen and put the cursor home.

    Called on the way out as well as on the way in: a view leaves a drawing
    behind that is no longer a reading of anything, and the lines saying what
    was put back belong on a clean screen rather than under a picture of the
    board as it was two seconds ago.
    """
    if console:
        sys.stdout.write(chr(27) + '[2J' + chr(27) + '[H')
        sys.stdout.flush()


def _set_console_mode(restore=None):
    """Put the console into mouse-reporting mode, or back as it was.

    Returns what the mode was, or None where there is no Windows console to
    set - every other platform's terminal reports the mouse once asked in
    band, and needs nothing here.
    """
    try:
        import ctypes
    except ImportError:
        return None

    try:
        kernel = ctypes.windll.kernel32
    except AttributeError:
        return None                     # not Windows

    handle = kernel.GetStdHandle(-10)
    was = ctypes.c_uint()
    if not kernel.GetConsoleMode(handle, ctypes.byref(was)):
        return None                     # a pipe, not a console

    wanted = restore if restore is not None else console_mode(was.value)
    kernel.SetConsoleMode(handle, wanted)
    return was.value


class Keys:

    """Non-blocking key reads, for a view that has to keep drawing.

    A view cannot block on input - it is redrawing at 8 to 20 Hz - so this
    only ever asks whether a key is already waiting. On Windows that is
    msvcrt and needs no terminal mode at all; elsewhere the terminal has to
    be taken out of line mode first, and put back afterwards whatever
    happens, which is what the context manager is for.

    Off when there is no terminal: a view piped to a file has no keyboard,
    and reading stdin there would eat the pipe.
    """

    #: One SGR mouse report: ESC [ < button ; column ; row (M press, m release)
    MOUSE_RE = re.compile(r'\033\[<(\d+);(\d+);(\d+)([Mm])')

    def __init__(self, console, mouse=False):
        self.console = console
        self.mouse = mouse and console
        self._saved = None
        self._was_mode = None
        self._posix = None
        self._buffer = ''
        self._dragging = False
        self._last_row = None
        self._typed = []

    def __enter__(self):
        if not self.console:
            return self

        try:
            import termios
            import tty
            self._posix = termios
            self._saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:               # noqa: BLE001 - Windows, or no tty
            self._saved = None

        if self.mouse:
            self._was_mode = _set_console_mode()
            sys.stdout.write(MOUSE_ON)
            sys.stdout.flush()
        return self

    def __exit__(self, *exc_info):
        if self.mouse:
            sys.stdout.write(MOUSE_OFF)
            sys.stdout.flush()
            _set_console_mode(self._was_mode)
        if self._saved is not None:
            self._posix.tcsetattr(sys.stdin, self._posix.TCSADRAIN,
                                  self._saved)
        return False

    def poll(self):
        """(leave, zoom) for everything that arrived since the last frame.

        `leave` is 'quit', 'menu' or None. `zoom` is a signed fraction: a
        wheel notch each way, plus whatever a right-drag moved, added up so
        one frame's worth of reports becomes one change. Twenty drag reports
        applied one at a time would move twenty frames behind the hand.
        """
        if not self.console:
            return None, 0.0

        self._buffer += ''.join(self._drain())
        zoom = 0.0

        while True:
            found = self.MOUSE_RE.search(self._buffer)
            if not found:
                break
            zoom += self._mouse(int(found.group(1)), int(found.group(3)),
                                found.group(4))
            self._buffer = (self._buffer[:found.start()]
                            + self._buffer[found.end():])

        leave, keys = None, self._buffer
        self._buffer = ''

        for key in keys:
            if leave is None and key in QUIT_KEYS:
                leave = 'quit'
            elif leave is None and key in MENU_KEYS:
                leave = 'menu'
            else:
                # Kept for a view that binds keys of its own. Every other
                # view ignores this and is unaffected; poll() still answers
                # the same two things it always did.
                self._typed.append(key)
        return leave, zoom

    def taken(self):
        """Characters typed since the last call, for a view with bindings.

        Drained rather than read: a key held down repeats, and a view that
        acted on the whole buffer every frame would keep acting on presses
        it had already handled.
        """
        out, self._typed = self._typed, []
        return out

    def _mouse(self, button, row, kind):
        """What one mouse report is worth, as a zoom fraction.

        POSITIVE IS NEARER, the way the caller uses it: it scales zoom by
        1 + this, and a bigger zoom stands closer. Wheel up returned a
        negative and the view backed away from the reader - the sign was
        wrong the whole way from here, and the test that should have caught
        it asserted the sign of this number instead of what the picture did.

        Right-drag is the same gesture with a hand instead of a finger: pull
        down to come back, push up to go in.
        """
        if button == 64:
            return WHEEL_STEP
        if button == 65:
            return -WHEEL_STEP

        # 2 is the right button; 32 is the drag bit the terminal sets while
        # it is held. Anything else is a click this view has no use for.
        if button == 2 and kind == 'M':
            self._dragging, self._last_row = True, row
            return 0.0
        if button == 2 and kind == 'm':
            self._dragging = False
            return 0.0
        if button == 34 and self._dragging:
            moved = row - (self._last_row if self._last_row is not None
                           else row)
            self._last_row = row
            return -moved * DRAG_STEP
        return 0.0

    def _drain(self):
        """Every key waiting right now, and none of the ones that are not."""
        try:
            import msvcrt
        except ImportError:
            return self._drain_posix()

        keys = []
        while msvcrt.kbhit():
            got = msvcrt.getwch()
            # A function or arrow key arrives as a prefix and then a code.
            # Read the second half so it is not mistaken for a letter.
            if got in ('\x00', '\xe0'):
                msvcrt.getwch()
                continue
            keys.append(got)
        return keys

    def _drain_posix(self):
        import select

        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            got = sys.stdin.read(1)
            if not got:
                break
            keys.append(got)
        return keys


def banner(origin, title, console, detail=''):
    """The view's top line, saying what is behind it.

    Drawn into the frame rather than printed once at the top, because a
    preflight line scrolls away and a view left running for an hour has to
    keep answering "is this the board". Green for a board, yellow for the
    stand-in - the same two colours board_prompt tags its prompt with, so
    both read the same across the session.
    """
    esc = chr(27)
    tag = ' LIVE ' if origin.real else ' SIMULATED - every value invented '
    colour = '30;42' if origin.real else '30;43'

    if not console:
        return '%s  %s   %s' % (tag.strip(), title, detail)
    return ('%s[%sm%s%s[0m  %s[97m%s%s[0m   %s[90m%s%s[0m'
            % (esc, colour, tag, esc, esc, title, esc, esc, detail, esc))


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
