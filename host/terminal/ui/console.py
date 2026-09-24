"""The console's input: keys and the mouse, read raw off the Windows console or a tty."""
import ctypes
import importlib
import re
import select
import sys


#: Q closes, ESC goes back to the menu - picking the wrong view is the common
#: mistake. Ctrl+C lands in the same `finally`.
QUIT_KEYS = frozenset({'q', 'Q'})

MENU_KEYS = frozenset({chr(27)})

#: The console's mouse record: the flag that says the pointer moved,
#: the button bits, the SGR codes a held button reports a drag as -
#: left before right, one report a record - and the press codes.
MOUSE_MOVED = 0x1

LEFT_BUTTON, RIGHT_BUTTON = 0x1, 0x2

DRAG_REPORTS = ((LEFT_BUTTON, 32), (RIGHT_BUTTON, 34))

BUTTON_REPORTS = ((LEFT_BUTTON, 0), (RIGHT_BUTTON, 2))


def _ignore(*_):
    """The callback a view did not pass."""


#: The key that lends the mouse to the view and gives it back. The mouse is
#: the terminal's by default: a view reporting it (SGR, QUICK_EDIT cleared)
#: cannot be selected from, and taking it on entry was reported three times
#: from the bench. Here, not per view, so every page takes the same key - F,
#: since C is the attitude view's frame and a menu entry.
SELECT_KEYS = frozenset({'f', 'F'})


#: Whether a `Keys` holds the mouse: module state, as the mouse is one; set
#: on entry, cleared on exit.
def holding():
    """Whether a view has the mouse right now, for a key legend."""
    return Keys.holder is not None and Keys.holder.holding()


#: Windows console input flags: VIRTUAL_TERMINAL_INPUT sends the mouse as
#: xterm's SGR sequences (one parser); QUICK_EDIT keeps the mouse for text and
#: must go, EXTENDED_FLAGS makes that stick. Without them the wheel is dead.
VT_INPUT = 0x0200

MOUSE_INPUT = 0x0010

EXTENDED_FLAGS = 0x0080

QUICK_EDIT = 0x0040

LINE_INPUT = 0x0002

ECHO_INPUT = 0x0004


def console_mode(was):
    """The input mode to run a mouse-driven view in, from the current one."""
    return ((was | VT_INPUT | MOUSE_INPUT | EXTENDED_FLAGS)
            & ~QUICK_EDIT & ~LINE_INPUT & ~ECHO_INPUT)


#: xterm SGR mouse reporting. 1002 is button-and-drag; 1006 survives a
#: terminal wider than 223 columns, where the older encoding stops.
MOUSE_ON = '\033[?1002h\033[?1006h'

MOUSE_OFF = '\033[?1006l\033[?1002l'

#: A wheel notch, as a fraction of the distance to the model. 12 % a notch is
#: about eight notches between filling the window and half of it, which is
#: roughly what one flick of a hand expects to do.
WHEEL_STEP = 0.12

#: A right-drag, per row of movement. Smaller than a notch: a drag is one
#: continuous gesture and lands twenty reports where a wheel lands one.
DRAG_STEP = 0.02


def _set_console_mode(restore=None):
    """Put the console into mouse-reporting mode, or back as it was."""
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


def _console_records():
    """The Windows INPUT_RECORD layout, built once ctypes exists."""

    class Coord(ctypes.Structure):
        _fields_ = (('X', ctypes.c_short), ('Y', ctypes.c_short))

    class Key(ctypes.Structure):
        _fields_ = (('down', ctypes.c_int), ('repeat', ctypes.c_ushort),
                    ('vk', ctypes.c_ushort), ('scan', ctypes.c_ushort),
                    ('ch', ctypes.c_wchar), ('state', ctypes.c_uint))

    class Mouse(ctypes.Structure):
        _fields_ = (('pos', Coord), ('buttons', ctypes.c_uint),
                    ('state', ctypes.c_uint), ('flags', ctypes.c_uint))

    class Event(ctypes.Union):
        _fields_ = (('key', Key), ('mouse', Mouse))

    class Record(ctypes.Structure):
        _fields_ = (('type', ctypes.c_ushort), ('event', Event))

    return Record


try:
    _RECORD = _console_records()
except ImportError:                      # pragma: no cover - no ctypes
    _RECORD = None


class Keys:

    """Non-blocking key reads, for a view redrawing at 8 to 20 Hz."""

    #: The Keys holding the mouse right now, if any: set on entry and
    #: cleared on exit, so a view outside a run reads False rather than
    #: a stale True.
    holder: 'Keys | None' = None

    #: One SGR mouse report: ESC [ < button ; column ; row (M press, m release)
    MOUSE_RE = re.compile(r'\033\[<(\d+);(\d+);(\d+)([Mm])')

    #: Arrows start with ESC, and a lone ESC leaves the view - so pressing
    #: one closed whatever was being adjusted. Taken out before that test.
    ARROW_RE = re.compile(r'\033\[([ABCD])')

    #: What a view binds against, so no view has to know the escape codes.
    ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}

    #: A mouse or arrow sequence's prefix still in flight: split across two
    #: drains, a report was eaten as typed keys.
    PARTIAL_RE = re.compile(r'\033(\[(<[\d;]*)?)?$')

    def __init__(self, console, mouse=False, quits=QUIT_KEYS):
        # `quits` is which letters leave.
        self._quits = frozenset(quits)
        self.console = console
        self.mouse = mouse and console
        self.reports = 0            # SGR mouse reports parsed, for a view's HUD
        self._clicked = []
        self._saved = None
        self._was_mode = None
        self._posix = None
        self._buffer = ''
        self._dragging = False
        self._last_row = None
        self._holding = False
        self._grip = None
        self._spun = (0.0, 0.0)
        self._pending = ''
        self._buttons = 0
        self._typed = []
        self._grabbed = False

    def __enter__(self):
        if not self.console:
            return self

        try:
            termios = importlib.import_module('termios')      # POSIX only
            tty = importlib.import_module('tty')
            self._posix = termios
            self._saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:       # noqa: BLE001 - Windows, or no tty; and
            # termios has its own error class, absent where it is absent
            self._saved = None
        # THE TERMINAL KEEPS THE MOUSE until a view is asked to take it.
        if self.mouse:
            Keys.holder = self
        return self

    def __exit__(self, *exc_info):
        self.grab(False)
        if Keys.holder is self:
            Keys.holder = None
        if self._saved is not None and self._posix is not None:
            self._posix.tcsetattr(sys.stdin, self._posix.TCSADRAIN,
                                  self._saved)
        return False

    def poll(self):
        """(leave, zoom) for everything that arrived since the last frame."""
        if not self.console:
            return None, 0.0

        self._buffer += ''.join(self._drain())
        zoom = 0.0

        while True:
            found = self.MOUSE_RE.search(self._buffer)
            if not found:
                break
            zoom += self._mouse(int(found.group(1)), int(found.group(2)),
                                int(found.group(3)), found.group(4))
            self._buffer = (self._buffer[:found.start()]
                            + self._buffer[found.end():])

        # Arrows before the lone-ESC test, or every one of them leaves the
        # view.
        while True:
            found = self.ARROW_RE.search(self._buffer)
            if not found:
                break
            self._typed.append(self.ARROWS[found.group(1)])
            self._buffer = (self._buffer[:found.start()]
                            + self._buffer[found.end():])

        # A trailing partial sequence waits ONE poll for its other half; the
        # same partial twice in a row is a real lone keypress (ESC) and goes
        # through 20 ms late instead of never.
        held = ''
        head, esc, rest = self._buffer.rpartition('\033')
        tail = esc + rest
        if self.PARTIAL_RE.match(tail) and tail != self._pending:
            held, self._buffer = tail, head
        self._pending = held

        leave, keys = None, self._buffer
        self._buffer = held

        for key in keys:
            if leave is None and key in self._quits:
                leave = 'quit'
            elif leave is None and key in MENU_KEYS:
                leave = 'menu'
            elif self.mouse and key in SELECT_KEYS:
                # SWALLOWED, not passed on: no view binds it, and one that did
                # would fight the terminal for the same gesture.
                self.grab(not self._grabbed)
            else:
                # Kept for a view that binds keys of its own.
                self._typed.append(key)
        return leave, zoom

    def holding(self):
        """Whether the VIEW has the mouse at the moment."""
        return self.mouse and self._grabbed

    def grab(self, on):
        """Take the mouse for the view, or hand it to the terminal."""
        if not self.mouse or on == self._grabbed:
            return self._grabbed
        out = sys.__stdout__ or sys.stdout
        if on:
            self._was_mode = _set_console_mode()
            out.write(MOUSE_ON)
        else:
            out.write(MOUSE_OFF)
            _set_console_mode(self._was_mode)
        out.flush()
        sys.stdout.flush()
        self._grabbed = on
        return self._grabbed

    def taken(self):
        """Characters typed since the last call, for a view with bindings."""
        out, self._typed = self._typed, []
        return out

    def _mouse(self, button, col, row, kind):
        """What one mouse report is worth, as a zoom fraction."""
        # Shift, meta and ctrl ride as +4/+8/+16 on the button code; the
        # gesture is the same gesture.
        self.reports += 1
        button &= ~28
        if button == 64:
            return WHEEL_STEP
        if button == 65:
            return -WHEEL_STEP

        # 2 is the right button; 32 is the drag bit the terminal sets while it
        # is held.
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

        # 0 is the left button, 32 its drag bit.
        if button == 0 and kind == 'M':
            self._holding, self._grip = True, (col, row)
            # The press is kept as well as the grip: a drag starts the same way
            # a click does, and a view that draws something to click on needs
            # the cell, not the delta.
            self._clicked.append((col, row))
            return 0.0
        if button == 0 and kind == 'm':
            self._holding = False
            return 0.0
        if button == 32 and self._holding:
            last = self._grip if self._grip is not None else (col, row)
            dx, dy = self._spun
            self._spun = (dx + (col - last[0]), dy + (row - last[1]))
            self._grip = (col, row)
        return 0.0

    def dragged(self):
        """Left-drag cell deltas (dx, dy) since the last call, drained."""
        out, self._spun = self._spun, (0.0, 0.0)
        return out

    def clicked(self):
        """Left-button presses as (column, row) since the last call."""
        out, self._clicked = self._clicked, []
        return out

    def _drain(self):
        """Every key waiting right now, and none of the ones that are not."""
        try:
            import msvcrt                                    # noqa: F401
        except ImportError:
            return self._drain_posix()
        got = self._drain_records()
        if got is not None:
            return got
        return self._drain_msvcrt()

    @staticmethod
    def _key_of(key):
        """A key-down record as the characters a terminal would send."""
        if key.ch and key.ch != '\x00':
            return key.ch * max(1, key.repeat)
        return {37: '\x1b[D', 38: '\x1b[A', 39: '\x1b[C',
                40: '\x1b[B'}.get(key.vk, '')

    def _mouse_of(self, mouse):
        """A mouse record as SGR reports: presses, releases, and drags with
        a button held.
        """
        x, y = mouse.pos.X + 1, mouse.pos.Y + 1
        if mouse.flags & MOUSE_MOVED:
            return ['\x1b[<%d;%d;%dM' % (code, x, y)
                    for bit, code in DRAG_REPORTS if self._buttons & bit][:1]
        if mouse.flags != 0:                        # wheel, double click
            return []
        keys = []
        for bit, name in BUTTON_REPORTS:
            had, has = self._buttons & bit, mouse.buttons & bit
            if has and not had:
                keys.append('\x1b[<%d;%d;%dM' % (name, x, y))
            elif had and not has:
                keys.append('\x1b[<%d;%d;%dm' % (name, x, y))
        self._buttons = mouse.buttons
        return keys

    def _drain_records(self):

        try:
            kernel = ctypes.windll.kernel32
        except AttributeError:
            return None
        handle = kernel.GetStdHandle(-10)
        count = ctypes.c_uint()
        if not kernel.GetNumberOfConsoleInputEvents(
                handle, ctypes.byref(count)):
            return None                  # a pipe, not a console
        if not count.value or _RECORD is None:
            return []
        buf = (_RECORD * count.value)()
        read = ctypes.c_uint()
        if not kernel.ReadConsoleInputW(handle, buf, count.value,
                                        ctypes.byref(read)):
            return None

        keys = []
        for i in range(read.value):
            record = buf[i]
            if record.type == 1 and record.event.key.down:
                keys.append(self._key_of(record.event.key))
            elif record.type == 2:
                keys.extend(self._mouse_of(record.event.mouse))
        return keys

    def _drain_msvcrt(self):
        import msvcrt

        keys = []
        while msvcrt.kbhit():
            got = msvcrt.getwch()
            # A function or arrow key arrives as a prefix and then a code.
            if got in ('\x00', '\xe0'):
                code = msvcrt.getwch()
                keys.append({'H': '\x1b[A', 'P': '\x1b[B',
                             'M': '\x1b[C', 'K': '\x1b[D'}.get(code, ''))
                continue
            keys.append(got)
        return keys

    def _drain_posix(self):

        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            got = sys.stdin.read(1)
            if not got:
                break
            keys.append(got)
        return keys
