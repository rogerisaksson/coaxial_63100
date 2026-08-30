"""Three things every live view on this board needs: a preflight line, a
redraw that does not flicker, and a way out that runs the cleanup.

Shared because every view wants them identically, and a second copy of any of
them is the thing that drifts - the structure suite fails a definition that
lives in two files.
"""
import re
import sys

from coaxial import ansi
from coaxial.errors import (DeviceStateError, NoReplyError,  # noqa: E402
                            RigError)

ansi.utf8_stdout()          # every view draws outside ASCII

#: Q closes, ESC goes back to the menu - picking the wrong view is the common
#: mistake. Ctrl+C lands in the same `finally`.
QUIT_KEYS = frozenset({'q', 'Q'})
MENU_KEYS = frozenset({chr(27)})

#: What a view exits with when ESC sent it back. Distinct from 0 so the menu
#: can tell "show me again" from "that is all", and from 130 so neither is
#: mistaken for Ctrl+C.
TO_MENU = 64

#: Windows console input flags. VIRTUAL_TERMINAL_INPUT makes the console
#: send mouse movement as the SGR sequences an xterm does, so one parser
#: serves both; QUICK_EDIT keeps the mouse for selecting text and has to go,
#: and EXTENDED_FLAGS is what makes clearing it stick. Without this the wheel
#: does nothing on Windows whatever the program prints.
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


#: Preflight colours, matching board_chat/Say.ps1 so the two read the
#: same. Green ok, cyan waiting, yellow worth knowing, red stop.
STATES = {'ok': '32', 'wait': '36', 'warn': '33', 'fail': '31'}

#: Whether ok/wait preflight lines print at all. The views turn this off:
#: their boot bar replaced the scroll of green lines on the way in, and
#: the teardown listing on the way out - warn and fail always print.
CHATTER = True


# -- the motif -----------------------------------------------------------
#
# Blade Runner, not a christmas tree: a dark street with two light sources
# and wet asphalt between them. Three roles, and nothing else gets a colour:
#
#   NEON    teal 44     the chrome that NAMES things - titles, rules
#   SODIUM  amber 214   the value that matters right now - armed, hot, held
#   ASH     grey 242    frame lines, key hints, the street itself
#
# Meaning colours stay what they always were - green LIVE, yellow SIMULATED,
# red fault. The motif dresses the frame; it never carries the verdict.
#
# Colour is applied at assembly and STRIPPED AT THE DOOR: paint() and say()
# drop the escapes when stdout is not a console, so the blocks can tint
# freely without threading a console flag through every builder.

NEON = 44
SODIUM = 214
#: Inline-label ash - THE SAME 66 the theme's `label` style uses, so a
#: tinted string beside a hud row reads as one voice. 242 stays for the
#: gauge rails and key hints, where receding further is the point.
ASH = 242
LABEL = 66

_SGR = re.compile(chr(27) + r'\[[0-9;]*m')


def plain(text):
    """`text` with every SGR escape removed - what the eye would count."""
    return _SGR.sub('', text)


def visible(text):
    return len(plain(text))


def tint(text, colour):
    """`text` in one of the motif's colours. Stripped at the door if piped."""
    return '%s[38;5;%dm%s%s[0m' % (chr(27), colour, text, chr(27))


#: The corner registration cross, three rows tall - the mark the Nostromo
#: screens put in every dead corner of a viewport. Stamped, not drawn: it
#: lands only where the field is empty, so a drawing that reaches a corner
#: simply keeps it.
CROSS = ('  │  ', ' ─┼─ ', '  │  ')


def stamp_crosses(lines, width, inset=2):
    """Registration crosses in the four corners of a field of `lines`.

    `width` is the field's visible width - lines are padded to it, so the
    right-hand crosses land in margin that exists whether or not the line
    was rstripped. A cell is only written where it holds a plain space,
    which keeps the mark off the drawing and off its colours.
    """
    if len(lines) < 8 or width < 24:
        return lines

    out = [line + ' ' * max(0, width - visible(line)) for line in lines]

    tall = len(CROSS)
    for top in (inset - 1, len(out) - inset - tall + 1):
        for left in (inset, width - inset - len(CROSS[0])):
            rows = range(max(0, top), min(len(out), top + tall))
            # ALL OR NOTHING: a cross that loses a row to the drawing is
            # not a mark, it is debris - so the whole corner yields if any
            # of its cells are taken.
            clear = all(
                out[r][left:left + len(CROSS[0])] == ' ' * len(CROSS[0])
                for r in rows) and len(rows) == tall
            if not clear:
                continue
            for i, bit in enumerate(CROSS):
                line = out[top + i]
                out[top + i] = line[:left] + bit + line[left + len(bit):]
    return [tint(line, NEON) if line.strip() and set(line) <= set(' |-+│─┼')
            else line for line in out]


# -- the console renderer -------------------------------------------------
# Defined in tools/stage.py: the THEME with every named style, and the
# renderer built around it. Re-exported here because every view imports
# its screen machinery from this module:
# THEME band band_of boot chip curtain footer frame_of header hud live
# panels_of stage viewport
from stage import (THEME, band, band_of, boot, chip, curtain, footer,  # noqa: E402,F401
                   frame_of, header, live, hud, panels_of, stage, viewport)


def paced(keys, period, step=0.02):
    """Sleep `period` while polling the keys every `step`.

    (leave, zoom, typed), returning the moment a leave key arrives. The
    draw rate and the INPUT rate used to be the same number: at the
    thermal view's 2 Hz a Q took half a second to bite, and the attitude
    view's zoom moved in board-round-trip-sized steps. Input is 50 Hz now
    whatever the view draws at.
    """
    import time as _time

    leave, zoom, typed = None, 0.0, []
    deadline = _time.monotonic() + period
    while True:
        got, moved = keys.poll()
        zoom += moved
        typed.extend(keys.taken())
        if got:
            return got, zoom, typed
        now = _time.monotonic()
        if now >= deadline:
            return None, zoom, typed
        _time.sleep(min(step, deadline - now))


def gauge(fraction, width, hot=0.85):
    """A meter in the motif: filled cells, ash rest, sodium past `hot`.

    The glyphs are ASCII on purpose - the session may run on a console
    that was never put in UTF-8, and a meter that half-renders is worse
    than a plain one.
    """
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * width))
    bar = '=' * filled + tint('-' * (width - filled), ASH)
    return tint(bar, SODIUM) if fraction >= hot else bar


def say(state, text, detail=''):
    """One preflight line, the shape board_chat.ps1 prints.

    Written here rather than shelling out to Say.ps1 because the preflight
    and the view share one session: splitting them would open the port
    twice, and the second open is the one that finds it busy.
    """
    if not CHATTER and state in ('ok', 'wait'):
        return
    if not sys.stdout.isatty():
        # Same rule the views follow: colour at the edge, and a pipe is
        # not one. Without this a redirected preflight carried raw
        # escapes into the log while the drawing beside it came out clean.
        sys.stdout.write('  %-6s%-22s %s\n' % (state, text, detail))
        sys.stdout.flush()
        return

    esc = chr(27)
    colour = STATES.get(state, '37')
    sys.stdout.write('  %s[%sm%-6s%s[0m%-22s %s[90m%s%s[0m\n'
                     % (esc, colour, state, esc, text, esc, detail, esc))
    sys.stdout.flush()


#: What a board hiccup looks like from a view: a refusal, a lost frame, a
#: state the board will not answer for. Named here because every view and
#: every bench tool holds the same list, and a copy of it in two files is
#: what the structure suite refuses.
QUIET = (NoReplyError, RigError, DeviceStateError)


def steady(fn, *args, **kwargs):
    """Call it, retrying the link's occasional silence. None if it stayed.

    NOT the library's contract - `coaxial` raises and never returns None for
    failure (invariant 8). This is the edge: a view that stopped drawing
    every time a frame was lost would be a view nobody could leave running,
    and one missing reading is a gap in a picture rather than an error.

    Four tries, because a lost frame is lost once. A board that is actually
    gone costs four timeouts to find out, which is the price of not redrawing
    a dashboard as empty every time the link hiccups.
    """
    import time

    for _ in range(4):
        try:
            return fn(*args, **kwargs)
        except QUIET:
            time.sleep(0.15)
    return None


def park(rows, console):
    """Put the cursor on the first line BELOW a painted frame.

    `paint` addresses every row absolutely and never scrolls, so when a view
    stops, the cursor is wherever the last changed row left it - somewhere in
    the middle of the drawing. Anything printed then lands on top of the
    picture and is read as part of it, and the shell prompt lands there too.

    Measured the hard way: a teardown list was reported missing five times
    and was being written into the middle of the dashboard every time. It was
    invisible to every check because a redirected stdout is not a console,
    where `paint` writes plain lines and the cursor is already at the end.
    """
    if console:
        sys.stdout.write('%s[%d;1H%s[J' % (chr(27), rows + 1, chr(27)))
        sys.stdout.flush()


def closing(done, console, drawn):
    """List what a view is putting back, under the frame it drew.

    One line per thing, the shape the preflight uses on the way in. A single
    'put back the way it was' was what every view used to print, and it says
    nothing about WHICH things - a stage that had been switching and a report
    that had been streaming both came out as one line, or as none at all.

    A step that failed is listed too, marked, rather than dropped: the way
    out is the only place that says whether it took.
    """
    park(drawn, console)
    say('wait', 'closing', 'putting back what this view changed')
    for name, what in done:
        failed = str(what).startswith('FAILED')
        say('fail' if failed else 'ok', name, what)


def clear(console):
    """Wipe the screen and put the cursor home.

    On the way out too: a view leaves a drawing that is no longer a reading
    of anything, and what was put back belongs on a clean screen.
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


def _console_records():
    """The Windows INPUT_RECORD layout, built once ctypes exists."""
    import ctypes

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

    """Non-blocking key reads, for a view redrawing at 8 to 20 Hz.

    Only ever asks whether a key is already waiting. Windows needs no
    terminal mode; elsewhere it comes out of line mode and goes back
    afterwards, which is what the context manager is for. Off with no
    terminal - reading stdin from a pipe would eat it.
    """

    #: One SGR mouse report: ESC [ < button ; column ; row (M press, m release)
    MOUSE_RE = re.compile(r'\033\[<(\d+);(\d+);(\d+)([Mm])')

    #: Arrows start with ESC, and a lone ESC leaves the view - so pressing
    #: one closed whatever was being adjusted. Taken out before that test.
    ARROW_RE = re.compile(r'\033\[([ABCD])')

    #: What a view binds against, so no view has to know the escape codes.
    ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}

    #: The prefix of a mouse or arrow sequence still in flight. A report
    #: SPLIT across two drains matched nothing, fell through to the key
    #: loop and was eaten as typed characters - drags turned sporadic,
    #: and the halves' letters hit view bindings.
    PARTIAL_RE = re.compile(r'\033(\[(<[\d;]*)?)?$')

    def __init__(self, console, mouse=False):
        self.console = console
        self.mouse = mouse and console
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
            (sys.__stdout__ or sys.stdout).write(MOUSE_ON)
            (sys.__stdout__ or sys.stdout).flush()
            sys.stdout.flush()
        return self

    def __exit__(self, *exc_info):
        if self.mouse:
            (sys.__stdout__ or sys.stdout).write(MOUSE_OFF)
            (sys.__stdout__ or sys.stdout).flush()
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
            zoom += self._mouse(int(found.group(1)), int(found.group(2)),
                                int(found.group(3)), found.group(4))
            self._buffer = (self._buffer[:found.start()]
                            + self._buffer[found.end():])

        # Arrows before the lone-ESC test, or every one of them leaves the
        # view. They join the typed buffer under their names, so a binding
        # reads `'up'` rather than three bytes.
        while True:
            found = self.ARROW_RE.search(self._buffer)
            if not found:
                break
            self._typed.append(self.ARROWS[found.group(1)])
            self._buffer = (self._buffer[:found.start()]
                            + self._buffer[found.end():])

        # A trailing partial sequence waits ONE poll for its other half;
        # the same partial twice in a row is a real lone keypress (ESC)
        # and goes through 20 ms late instead of never.
        held = ''
        cut = self._buffer.rfind('\033')
        if cut != -1:
            tail = self._buffer[cut:]
            if self.PARTIAL_RE.match(tail) and tail != self._pending:
                held, self._buffer = tail, self._buffer[:cut]
        self._pending = held

        leave, keys = None, self._buffer
        self._buffer = held

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

    def _mouse(self, button, col, row, kind):
        """What one mouse report is worth, as a zoom fraction.

        POSITIVE IS NEARER, the way the caller uses it: it scales zoom by
        1 + this, and a bigger zoom stands closer. Wheel up returned a
        negative and the view backed away from the reader - the sign was
        wrong the whole way from here, and the test that should have caught
        it asserted the sign of this number instead of what the picture did.

        Right-drag is the same gesture with a hand instead of a finger: pull
        down to come back, push up to go in. LEFT-drag is the other channel:
        its cell deltas pile up for dragged(), the trackball a view can
        bind to rotation.
        """
        # Shift, meta and ctrl ride as +4/+8/+16 on the button code; the
        # gesture is the same gesture.
        self.reports = getattr(self, 'reports', 0) + 1
        button &= ~28
        if button == 64:
            return WHEEL_STEP
        if button == 65:
            return -WHEEL_STEP

        # 2 is the right button; 32 is the drag bit the terminal sets while
        # it is held.
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

    def _drain(self):
        """Every key waiting right now, and none of the ones that are not.

        On Windows the CONSOLE RECORDS are read directly. The msvcrt
        path only ever saw what conhost chose to translate to VT: the
        wheel came through, LEFT-BUTTON MOTION never did, and the mouse
        counter froze the moment a drag began - measured on the bench.
        Mouse records synthesize into the same SGR strings the in-band
        protocol sends, so everything downstream stays one parser; the
        wheel record is deliberately skipped because its VT translation
        already arrives, and two sources would double every notch.
        """
        try:
            import msvcrt                                    # noqa: F401
        except ImportError:
            return self._drain_posix()
        got = self._drain_records()
        if got is not None:
            return got
        return self._drain_msvcrt()

    def _drain_records(self):
        import ctypes

        try:
            kernel = ctypes.windll.kernel32
        except AttributeError:
            return None
        handle = kernel.GetStdHandle(-10)
        count = ctypes.c_uint()
        if not kernel.GetNumberOfConsoleInputEvents(
                handle, ctypes.byref(count)):
            return None                  # a pipe, not a console
        if not count.value:
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
                ch = record.event.key.ch
                if ch and ch != '\x00':
                    keys.append(ch * max(1, record.event.key.repeat))
                else:
                    keys.append({37: '\x1b[D', 38: '\x1b[A',
                                 39: '\x1b[C', 40: '\x1b[B'}.get(
                                     record.event.key.vk, ''))
            elif record.type == 2:
                mouse = record.event.mouse
                x, y = mouse.pos.X + 1, mouse.pos.Y + 1
                if mouse.flags == 0:
                    for bit, name in ((0x1, 0), (0x2, 2)):
                        had = self._buttons & bit
                        has = mouse.buttons & bit
                        if has and not had:
                            keys.append('\x1b[<%d;%d;%dM' % (name, x, y))
                        elif had and not has:
                            keys.append('\x1b[<%d;%d;%dm' % (name, x, y))
                    self._buttons = mouse.buttons
                elif mouse.flags & 0x1:
                    if self._buttons & 0x1:
                        keys.append('\x1b[<32;%d;%dM' % (x, y))
                    elif self._buttons & 0x2:
                        keys.append('\x1b[<34;%d;%dM' % (x, y))
        return keys

    def _drain_msvcrt(self):
        import msvcrt

        keys = []
        while msvcrt.kbhit():
            got = msvcrt.getwch()
            # A function or arrow key arrives as a prefix and then a code.
            # The code was read and DISCARDED once - so the menu's UP/DOWN
            # did nothing on the one platform this bench runs on. Translated
            # to the VT sequences instead, so the arrow path is ONE path.
            if got in ('\x00', '\xe0'):
                code = msvcrt.getwch()
                keys.append({'H': '\x1b[A', 'P': '\x1b[B',
                             'M': '\x1b[C', 'K': '\x1b[D'}.get(code, ''))
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


