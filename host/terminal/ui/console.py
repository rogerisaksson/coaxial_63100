"""The console's input: keys and the mouse, read raw off the Windows console or a tty."""
import ctypes
import os
import re
import select
import sys
import time


#: Q closes, ESC goes back to the menu; Ctrl+C lands in the same `finally`.
QUIT_KEYS = frozenset({'q', 'Q'})

MENU_KEYS = frozenset({chr(27)})

#: Console mouse record flags and SGR drag/press button codes.
MOUSE_MOVED = 0x1

LEFT_BUTTON, RIGHT_BUTTON = 0x1, 0x2

DRAG_REPORTS = ((LEFT_BUTTON, 32), (RIGHT_BUTTON, 34))

BUTTON_REPORTS = ((LEFT_BUTTON, 0), (RIGHT_BUTTON, 2))


def _ignore(*_):
    """The callback a view did not pass."""


#: Key lending the mouse to the view and giving it back (F, shared across views).
SELECT_KEYS = frozenset({'f', 'F'})


def holding():
    """Whether a view has the mouse right now, for a key legend."""
    return Keys.holder is not None and Keys.holder.holding()


def held():
    """Currently held movement keys from hardware state when available, else None."""
    return Keys.holder.held() if Keys.holder is not None else None


def left_held():
    """Whether the left mouse button is held in the active view right now."""
    return Keys.holder is not None and Keys.holder.left_held()


#: Windows console input flags: VT_INPUT sends SGR mouse; QUICK_EDIT must be cleared with EXTENDED_FLAGS.
VT_INPUT, MOUSE_INPUT, EXTENDED_FLAGS = 0x0200, 0x0010, 0x0080
QUICK_EDIT, LINE_INPUT, ECHO_INPUT = 0x0040, 0x0002, 0x0004


def console_mode(was, mouse=True):
    """The input mode to run a mouse-driven view in, from the current one."""
    return ((was | VT_INPUT | (MOUSE_INPUT if mouse else 0) | EXTENDED_FLAGS)
            & ~QUICK_EDIT & ~LINE_INPUT & ~ECHO_INPUT)


#: xterm SGR mouse reporting (1002 button-drag, 1006 wide terminals) and bracketed paste.
MOUSE_ON, MOUSE_OFF = '\033[?1002h\033[?1006h', '\033[?1006l\033[?1002l'
PASTE_ON, PASTE_OFF = '\033[?2004h', '\033[?2004l'
HOLD_S, WHEEL_STEP, DRAG_STEP = 1.0, 0.12, 0.02


def _set_console_mode(restore=None, mouse=True):
    """Put the console into mouse-reporting mode, or back as it was."""
    try:
        kernel = ctypes.windll.kernel32
    except AttributeError:
        return None
    handle, was = kernel.GetStdHandle(-10), ctypes.c_uint()
    if not kernel.GetConsoleMode(handle, ctypes.byref(was)):
        return None
    kernel.SetConsoleMode(handle, restore if restore is not None else console_mode(was.value, mouse))
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
except ImportError:                      # no ctypes
    _RECORD = None


class Keys:

    """Non-blocking key reads, for a view redrawing at 8 to 20 Hz."""

    holder: 'Keys | None' = None

    MOUSE_RE = re.compile(r'\033\[<(\d+);(\d+);(\d+)([Mm])')
    PASTE_START = '\033[200~'
    PASTE_RE = re.compile(r'\033\[200~.*?\033\[201~', re.S)
    ARROW_RE = re.compile(r'\033\[([ABCD])')
    ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}
    FUNCTION_RE = re.compile(r'\033(?:O([P-S])|\[\[([A-E])|\[(\d+)~)')

    FUNCTION_KEYS = {
        **{c: 'f%d' % (i + 1) for seq in ('PQRS', 'ABCDE') for i, c in enumerate(seq)},
        **{str(k): 'f%d' % (i + 1) for i, k in enumerate((11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 23, 24))},
        '1': 'home', '4': 'end', '5': 'pageup', '6': 'pagedown',
    }

    PARTIAL_RE = re.compile(r'\033(?:\[(?:<[\d;]*|\d*)?|O[P-S]?)?$')
    CSI_RE = re.compile(r'\033\[[\x30-\x3f]*[\x20-\x2f]*[\x40-\x7e]')
    VK_MAP = {87: 'w', 65: 'a', 83: 's', 68: 'd', 81: 'q', 69: 'e', 32: ' ',
              16: 'shift', 160: 'shift', 161: 'shift'}

    def __init__(self, console, mouse=False, quits=QUIT_KEYS, select=SELECT_KEYS, pan=False):
        self._quits, self._select, self._pan = frozenset(quits), frozenset(select), pan
        self.console, self.mouse, self.reports = console, mouse and console, 0
        self._clicked, self._typed, self._keys_down = [], [], set()
        self._saved = self._was_mode = self._posix = self._last_row = self._grip = self._term_hwnd = None
        self._dragging = self._holding = self._grabbed = self._async_keys = self._latched_wasd = self._hw_seen = False
        self._buffer = self._pending = ''
        self._pan_at, self._panned, self._spun = (0, 0), (0.0, 0.0), (0.0, 0.0)
        self._buttons, self._held_at, self._last_key_at, self._paste_at = 0, 0.0, 0.0, 0.0

    def __enter__(self):
        if not self.console:
            return self
        if sys.platform != 'win32':
            import termios
            import tty
            self._posix = termios
            try:
                self._saved = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
            except (termios.error, OSError, ValueError, TypeError):
                self._saved = None
        else:
            self._was_mode = _set_console_mode(mouse=False)
        if self.mouse:
            Keys.holder = self
            if not self._select:
                self.grab(True)
        return self

    def __exit__(self, *exc_info):
        self.grab(False)
        if self._was_mode is not None:
            _set_console_mode(self._was_mode)
        if Keys.holder is self:
            Keys.holder = None
        if self._saved is not None and self._posix is not None:
            self._posix.tcsetattr(sys.stdin, self._posix.TCSADRAIN, self._saved)
        return False

    def _strip_sequences(self):
        """Drain raw input and strip paste, mouse, arrow, function, and CSI sequences."""
        now = time.monotonic()
        self._buffer = self.PASTE_RE.sub('', self._buffer + ''.join(self._drain()))
        start = self._buffer.find(self.PASTE_START)
        if start >= 0 and not self._paste_at:
            self._paste_at = now
        elif start < 0:
            self._paste_at = 0.0
        pasting = self._buffer[start:] if start >= 0 and now - self._paste_at <= 0.2 else ''
        self._buffer = self._buffer[:start] if start >= 0 else self._buffer
        zoom = 0.0
        for found in self.MOUSE_RE.finditer(self._buffer):
            self._held_at = now
            zoom += self._mouse(int(found.group(1)), int(found.group(2)),
                                int(found.group(3)), found.group(4))
        self._buffer = self.MOUSE_RE.sub('', self._buffer)
        seq = [(m.start(), self.ARROWS[m.group(1)]) for m in self.ARROW_RE.finditer(self._buffer)]
        seq += [(m.start(), self.FUNCTION_KEYS.get(m.group(1) or m.group(2) or m.group(3), 'f1'))
                for m in self.FUNCTION_RE.finditer(self._buffer)]
        self._typed.extend(k for _, k in sorted(seq))
        self._buffer = self.CSI_RE.sub('', self.FUNCTION_RE.sub('', self.ARROW_RE.sub('', self._buffer)))

        held = ''
        head, esc, rest = self._buffer.rpartition('\033')
        tail = esc + rest
        if self.PARTIAL_RE.match(tail) and not (self._async_keys and len(tail) == 1):
            if tail != self._pending:
                held, self._buffer, self._pending_at = tail, head, now
            elif len(tail) > 1 and now - getattr(self, '_pending_at', 0.0) < 0.05:
                held, self._buffer = tail, head
            elif len(tail) > 1:
                self._buffer = head
        self._pending = held
        keys, self._buffer = self._buffer, held + pasting
        return keys, zoom

    def poll(self):
        """(leave, zoom) for everything that arrived since the last frame."""
        if not self.console:
            return None, 0.0
        if self._grabbed and time.monotonic() - self._held_at >= HOLD_S:
            if self._async_keys and self._hw_buttons():
                self._held_at = time.monotonic()
            else:
                self._holding = self._dragging = False
                self._buttons = 0
                self._hold()
        keys, zoom = self._strip_sequences()
        leave = None
        for key in keys:
            if leave is None and key in self._quits:
                leave = 'quit'
            elif leave is None and key in MENU_KEYS:
                leave = 'menu'
            elif self.mouse and key in self._select:
                self.grab(not self._grabbed)
            else:
                self._typed.append(key)
        return leave, zoom

    def holding(self):
        """Whether the view has the mouse at the moment."""
        return self.mouse and self._grabbed

    def grab(self, on):
        """Take the mouse for the view, or hand it to the terminal."""
        if not self.mouse or on == self._grabbed:
            return self._grabbed
        out = sys.__stdout__ or sys.stdout
        if on:
            was = _set_console_mode(mouse=True)
            if self._was_mode is None:
                self._was_mode = was
            out.write(MOUSE_ON + PASTE_ON)
            self._held_at = time.monotonic()
        else:
            out.write(MOUSE_OFF + PASTE_OFF)
            _set_console_mode(mouse=False)
        out.flush()
        sys.stdout.flush()
        self._grabbed = on
        return self._grabbed

    def _hold(self):
        """Re-assert mouse reporting and console mode."""
        _set_console_mode()
        out = sys.__stdout__ or sys.stdout
        out.write(MOUSE_ON + PASTE_ON)
        out.flush()
        self._held_at = time.monotonic()

    def taken(self):
        """Characters typed since the last call, for a view with bindings."""
        out, self._typed = self._typed, []
        return out

    def _mouse(self, button, col, row, kind):
        """What one mouse report is worth, as a zoom fraction."""
        self.reports += 1
        button &= ~28
        if button in (64, 65):
            return WHEEL_STEP if button == 64 else -WHEEL_STEP
        if button in (2, 34) and kind == 'm':
            self._dragging = False
            return 0.0
        if button in (0, 32) and kind == 'm':
            self._holding = False
            return 0.0
        if button == 2 and kind == 'M':
            self._dragging, self._last_row, self._pan_at = True, row, (col, row)
            return 0.0
        if button == 34 and kind == 'M':
            if not self._dragging or self._pan_at is None:
                self._dragging, self._last_row, self._pan_at = True, row, (col, row)
            if self._pan:
                dx, dy = self._panned
                self._panned = (dx + col - self._pan_at[0], dy + row - self._pan_at[1])
                self._pan_at = (col, row)
                return 0.0
            moved = row - (self._last_row if self._last_row is not None else row)
            self._last_row = row
            return -moved * DRAG_STEP
        if button == 0 and kind == 'M':
            self._holding, self._grip = True, (col, row)
            self._clicked.append((col, row))
            return 0.0
        if button == 32 and kind == 'M':
            if not self._holding or self._grip is None:
                self._holding, self._grip = True, (col, row)
            last = self._grip
            dx, dy = self._spun
            self._spun = (dx + (col - last[0]), dy + (row - last[1]))
            self._grip = (col, row)
            return 0.0
        if kind == 'm':
            self._holding = self._dragging = False
        return 0.0

    def dragged(self):
        """Left-drag cell deltas (dx, dy) since the last call, drained."""
        out, self._spun = self._spun, (0.0, 0.0)
        return out

    def panned(self):
        """Right-drag cell deltas (dx, dy) since the last call, drained, `pan` set."""
        out, self._panned = self._panned, (0.0, 0.0)
        return out

    def clicked(self):
        """Left-button presses as (column, row) since the last call."""
        out, self._clicked = self._clicked, []
        return out

    def _hw_buttons(self):
        try:
            u = ctypes.windll.user32
            return bool(sys.platform == 'win32' and (u.GetAsyncKeyState(1) & 0x8000 or u.GetAsyncKeyState(2) & 0x8000))
        except AttributeError:
            return False

    def left_held(self):
        """Whether the left mouse button is currently held down."""
        if self._holding:
            return True
        if self._async_keys and self._is_console_focused() and sys.platform == 'win32':
            try:
                return bool(ctypes.windll.user32.GetAsyncKeyState(1) & 0x8000)
            except AttributeError:
                return False
        return False

    def _is_console_focused(self):
        """True when the console/terminal window is focused."""
        if sys.platform != 'win32':
            return True
        try:
            kernel, user32 = ctypes.windll.kernel32, ctypes.windll.user32
            fg = user32.GetForegroundWindow()
            if not fg:
                return False
            if self._term_hwnd is not None:
                return fg == self._term_hwnd
            cw = kernel.GetConsoleWindow()
            return fg == cw if cw and user32.IsWindowVisible(cw) else True
        except AttributeError:
            return True

    def held(self):
        """Currently held movement keys (w, a, s, d, q, e, shift) that entered this terminal."""
        now = time.monotonic()
        if not self._is_console_focused():
            self._latched_wasd = self._hw_seen = False
            self._keys_down.clear()
            return set() if self._async_keys else None
        if sys.platform == 'win32' and self.console and (self._latched_wasd or self._keys_down or self._async_keys):
            try:
                user32 = ctypes.windll.user32
                self._async_keys = True
                if self._latched_wasd or self._keys_down:
                    hw = {c for vk, c in self.VK_MAP.items() if user32.GetAsyncKeyState(vk) & 0x8000}
                    if hw & {'w', 'a', 's', 'd', 'q', 'e', ' '}:
                        self._hw_seen = self._latched_wasd = True
                        self._keys_down, self._last_key_at = set(hw), now
                        return set(hw)
                    if self._hw_seen:
                        self._latched_wasd = False
                        self._keys_down.clear()
                        return set()
                    if now - self._last_key_at > 0.65:
                        self._latched_wasd = False
                        self._keys_down.clear()
                return set() if self._hw_seen else None
            except AttributeError:
                pass
        return None

    def _drain(self):
        """Every key waiting right now, and none of the ones that are not."""
        if sys.platform != 'win32':
            return self._drain_posix()
        got = self._drain_records()
        return got if got is not None else self._drain_msvcrt()

    @staticmethod
    def _key_of(key):
        """A key-down record as the characters a terminal would send."""
        if key.ch and key.ch != '\x00':
            return key.ch * max(1, key.repeat)
        if key.vk == 27:
            return '\x1b'
        if 112 <= key.vk <= 123:
            return 'f%d' % (key.vk - 111)
        return {37: '\x1b[D', 38: '\x1b[A', 39: '\x1b[C', 40: '\x1b[B'}.get(key.vk, '')

    def _mouse_of(self, mouse):
        """A mouse record as SGR reports: presses, releases, and drags."""
        self._held_at = time.monotonic()
        if mouse.flags & 0x0C:
            return []
        x, y = max(1, mouse.pos.X + 1), max(1, mouse.pos.Y + 1)
        keys = []
        for bit, name in BUTTON_REPORTS:
            had, has = self._buttons & bit, mouse.buttons & bit
            if has and not had:
                keys.append('\x1b[<%d;%d;%dM' % (name, x, y))
            elif had and not has:
                keys.append('\x1b[<%d;%d;%dm' % (name, x, y))
        self._buttons = mouse.buttons
        if (mouse.flags & MOUSE_MOVED) and not keys:
            keys.extend(['\x1b[<%d;%d;%dM' % (code, x, y)
                         for bit, code in DRAG_REPORTS if self._buttons & bit][:1])
        return keys

    def _drain_records(self):
        try:
            kernel = ctypes.windll.kernel32
            user32 = getattr(ctypes.windll, 'user32', None)
        except AttributeError:
            return None
        handle, count = kernel.GetStdHandle(-10), ctypes.c_uint()
        if not kernel.GetNumberOfConsoleInputEvents(handle, ctypes.byref(count)):
            return None
        self._async_keys = True
        if not count.value or _RECORD is None:
            return []
        buf, read = (_RECORD * count.value)(), ctypes.c_uint()
        if not kernel.ReadConsoleInputW(handle, buf, count.value, ctypes.byref(read)):
            return None
        fg = user32.GetForegroundWindow() if user32 is not None else 0
        keys, now = [], time.monotonic()
        for i in range(read.value):
            record = buf[i]
            if record.type == 1:
                key = record.event.key
                ch_low = (key.ch or '').lower()
                tok = self.VK_MAP.get(key.vk) or (ch_low if ch_low in ('w', 'a', 's', 'd', 'q', 'e', ' ') else None)
                if key.down:
                    if fg:
                        self._term_hwnd = fg
                    self._last_key_at = now
                    if tok:
                        self._latched_wasd = True
                        self._keys_down.add(tok)
                        if user32 is not None and (user32.GetAsyncKeyState(key.vk) & 0x8000):
                            self._hw_seen = True
                    if key.ch in ('W', 'A', 'S', 'D', 'Q', 'E') or (key.state & 0x0010):
                        self._latched_wasd = True
                        self._keys_down.add('shift')
                    keys.append(self._key_of(key))
                elif tok and (user32 is None or not (user32.GetAsyncKeyState(key.vk) & 0x8000)):
                    self._hw_seen = True
                    self._keys_down.discard(tok)
            elif record.type == 2:
                keys.extend(self._mouse_of(record.event.mouse))
        return keys

    def _drain_msvcrt(self):
        import msvcrt

        user32 = getattr(ctypes.windll, 'user32', None)
        fg = user32.GetForegroundWindow() if user32 is not None else 0
        keys, now = [], time.monotonic()
        while msvcrt.kbhit():
            got = msvcrt.getwch()
            if got in ('\x00', '\xe0'):
                code = msvcrt.getwch()
                keys.append(('f%d' % (ord(code) - 58)) if ';' <= code <= 'D' else
                            {'H': '\x1b[A', 'P': '\x1b[B', 'M': '\x1b[C', 'K': '\x1b[D'}.get(code, ''))
                continue
            if fg:
                self._term_hwnd = fg
            if got.lower() in ('w', 'a', 's', 'd', 'q', 'e', ' '):
                self._last_key_at, self._latched_wasd = now, True
                self._keys_down.add(got.lower())
                if got in ('W', 'A', 'S', 'D', 'Q', 'E'):
                    self._keys_down.add('shift')
            keys.append(got)
        return keys

    def _drain_posix(self):

        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            got = os.read(sys.stdin.fileno(), 1024).decode('utf-8', 'replace')
            if not got:
                break
            keys.extend(got)
        return keys
