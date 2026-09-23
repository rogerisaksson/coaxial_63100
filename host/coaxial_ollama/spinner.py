"""The prompt line: a bookend group, the board name, and what it is on."""
import threading

TICK = 0.12

ROBOT = '\U0001F916'                     # "🤖"
ROBOT_FALLBACK = 'o'

# All three need to be full-colour by default, not text glyphs forced into
# colour with a variation selector - a forced one can sit at a slightly
# different advance width than a native one in some fonts, which reads as
# uneven spacing next to the others even though there is no actual space
# character anywhere in this string.
ICON_WAIT = '\U0001F4A4'                 # "💤"
ICON_ERROR = '❌'                    # "❌"
ICON_WAIT_FALLBACK = 'z'
ICON_ERROR_FALLBACK = 'X'

# The busy icon turns in place, in the bookend group.
SPIN = ('\U000023F3', '\U000023F3', '\U0000231B')   # flowing, flowing, done
SPIN_FALLBACK = ('|', '/', '-', '\\')

# Guillemets rather than plain pipes framing the bookend group - narrower
# footprint than an emoji bracket would be, and already inside cp1252 (0x AB /
# 0xBB), so a legacy console gets them too even though it cannot hold the robot
# it sits around.
OPEN = '«'
CLOSE = '»'
OPEN_FALLBACK = '|'
CLOSE_FALLBACK = '|'

SAVE = '\x1b7'
RESTORE = '\x1b8'
UP = '\x1b[%dA'
GREEN = '\x1b[32m'
YELLOW = '\x1b[33m'
RED = '\x1b[31m'
RESET = '\x1b[0m'


def _encodable(out, text):
    encoding = getattr(out, 'encoding', None) or 'ascii'
    try:
        text.encode(encoding)
        return True
    except (UnicodeEncodeError, LookupError, TypeError):
        return False


def _capable(out):
    """Whether this stream can hold the robot and the state icons."""
    return _encodable(out, ROBOT + ICON_WAIT + SPIN[0] + ICON_ERROR)


def _brackets_capable(out):
    """Whether this stream can hold the guillemets framing the group."""
    return _encodable(out, OPEN + CLOSE)


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class _Tracked:
    """A stream that counts the newlines passed through it, otherwise
    behaving exactly like the one it wraps.
    """

    def __init__(self, real, lock):
        self.real = real
        self.lock = lock
        self.lines = 0

    def write(self, text):
        with self.lock:
            self.lines += text.count('\n')
            return self.real.write(text)

    def flush(self):
        with self.lock:
            return self.real.flush()

    def isatty(self):
        try:
            return self.real.isatty()
        except (AttributeError, ValueError):
            return False

    @property
    def encoding(self):
        return getattr(self.real, 'encoding', None)


class Prompt:
    """One prompt line: a robot/icon group up front, and a bar that ticks on
    its own thread until stop()ped - inside the text's own '1' if it has
    one, appended after it otherwise.
    """

    def __init__(self, text, out, lock=None, ok=True, tick=TICK,
                 tag=None, tag_ok: 'bool | str' = True):
        # RLock, not Lock: _trace() in debug.py already holds this lock for its
        # whole loop of print()s, each of which re-enters it again inside
        # _Tracked.write() - a plain Lock would deadlock the second of those
        # against itself, on the same thread, over nothing.
        self.lock = lock or threading.RLock()
        self.out = _Tracked(out, self.lock)
        self.vt = _vt(out)
        self.tick = tick
        real = _capable(out)
        self.robot = ROBOT if real else ROBOT_FALLBACK
        self.icon_wait = ICON_WAIT if real else ICON_WAIT_FALLBACK
        self.icon_error = ICON_ERROR if real else ICON_ERROR_FALLBACK
        self.open = OPEN if _brackets_capable(out) else OPEN_FALLBACK
        self.close = CLOSE if _brackets_capable(out) else CLOSE_FALLBACK

        self.spin = SPIN if real else SPIN_FALLBACK
        self.name = text
        self.busy_now = False

        self.icon = self.icon_wait if ok else self.icon_error
        self.color = GREEN if ok else RED
        # What the session is actually talking to, in parentheses after the
        # name: "COM4, 115200" green for a board, "Simulated" yellow for a
        # stand-in.
        self.tag = tag
        # Green a board, yellow a stand-in, and RED every node at once.
        self.tag_color = (RED if tag_ok == 'all'
                          else GREEN if tag_ok else YELLOW)
        self.rows_up = 0                 # 0 while still on this row, 1 after
        self.frame = 0
        self.done = threading.Event()
        self.thread = None

        with self.lock:
            self.out.write(self._prefix() + self._tag() + '>')
            self.out.flush()
        if self.vt:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _tag(self):
        if not self.tag:
            return ''
        if not self.vt:
            return '(%s)' % self.tag
        return '(%s%s%s)' % (self.tag_color, self.tag, RESET)

    def _prefix(self):
        """The bookend group and the name - everything before the tag."""
        icon = (self.spin[self.frame % len(self.spin)]
                if self.busy_now else self.icon)
        return '%s%s%s%s%s%s%s' % (self.open, self.robot, icon, self.close,
                                   self.color, self.name, RESET)

    def _paint(self):
        # rows_up is the fixed "Enter moved to a fresh row" step; out.lines is
        # however many more rows whatever else printed since busy() - both
        # together are the true climb back to the prompt line, not a number
        # decided once and trusted for as long as the question takes.
        rows = self.rows_up + self.out.lines if self.rows_up else 0
        up = (UP % rows) if rows else ''
        try:
            with self.lock:
                self.out.write(SAVE + up + '\r' + self._prefix() + RESTORE)
                self.out.flush()
        except (OSError, ValueError):
            # The stream closed under us - the prompt is over, and a spinner is
            # not worth an exception on the way out.
            self.done.set()

    def _run(self):
        while not self.done.wait(self.tick):
            self.frame += 1
            self._paint()

    def busy(self):
        """Call once input() has returned: the icon starts turning, yellow."""
        self.out.lines = 0
        self.color = YELLOW
        self.busy_now = True
        self.rows_up = 1
        if self.vt:
            self._paint()

    def stop(self, ok):
        """Stop ticking and freeze the whole group - the last thing a
        scrolled-past line says about the question it carried.
        """
        self.done.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.frame = 0
        self.busy_now = False
        self.color = GREEN if ok else RED
        self.icon = self.icon_wait if ok else self.icon_error
        if self.vt:
            self._paint()


def prompt(text, out, lock=None, ok=True, tick=TICK, tag=None, tag_ok=True):
    return Prompt(text, out, lock=lock, ok=ok, tick=tick,
                  tag=tag, tag_ok=tag_ok)
