"""A prompt with a robot on one side, the board on the other, and a spinner
between them that actually turns.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; something turning says which is waiting for a question, and its
colour says what kind of waiting: green for "nothing submitted yet", yellow
for "working on it", red for "that just failed".

    |<robot><bar><pager>| Coaxial_63100>
           ^^^ this is the only thing that ever moves or changes colour

The robot and the pager either side of it are static bookends, printed once
and never touched again - only the bar between them is repainted, on a timer,
in place:

    ESC 7            save the cursor, wherever input() has got to
    ESC [ <n> A      up n rows, onto the prompt line (0 while still typing,
                     1 once Enter has moved the cursor to a fresh row)
    ESC [ <col> G    to the bar's column on that row
    <bar>            one frame, in the current colour
    ESC 8            back to where the cursor was

Typed text is never written over, because it is never written to; only the
bar's own column is. Every bar frame is the same width for exactly this
reason - a repaint only overwrites the bar's own cell, so an uneven frame
would leave a stray character behind. The one thing this cannot get right is
a robot or pager wide enough to render as two terminal columns on a stream
that also renders them at all: the bar's column is computed from Python
string length, not terminal cell width, same limitation the very first
version of this had for a prompt long enough to wrap.

`_trace()` in debug.py can print mid-question, while the bar is still
ticking for the busy phase - the caller passes in the same lock both sides
write through, so a tick and a trace line never interleave into garbage.

No console, no VT, or output redirected: the prompt is printed once, static,
and start()/busy()/stop() all do nothing further. A script piping commands in
gets exactly what it would have got before any of this existed.
"""
import threading

TICK = 0.12

ROBOT = '\U0001F916'                     # "🤖"
ROBOT_FALLBACK = 'o'
PAGER = '\U0001F4DF'                     # "📟"
PAGER_FALLBACK = '#'

BARS = ('|', '/', '–', '\\')
BARS_FALLBACK = ('|', '/', '-', '\\')

SAVE = '\x1b7'
RESTORE = '\x1b8'
UP = '\x1b[%dA'
COLUMN = '\x1b[%dG'
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


def _bookend(out, real, fallback):
    return real if _encodable(out, real) else fallback


def _bars(out):
    return BARS if _encodable(out, ''.join(BARS)) else BARS_FALLBACK


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class Prompt:
    """One prompt line: static robot and pager, a bar between them that
    ticks on its own thread until stop()ped."""

    def __init__(self, text, out, lock=None, ok=True, tick=TICK):
        self.out = out
        self.lock = lock or threading.Lock()
        self.vt = _vt(out)
        self.tick = tick
        self.bars = _bars(out)
        robot = _bookend(out, ROBOT, ROBOT_FALLBACK)
        pager = _bookend(out, PAGER, PAGER_FALLBACK)
        self.prefix = '|' + robot
        self.suffix = pager + '| '
        self.column = len(self.prefix) + 1
        self.rows_up = 0                 # 0 while still on this row, 1 after
        self.color = GREEN if ok else RED
        self.frame = 0
        self.done = threading.Event()
        self.thread = None

        with self.lock:
            self.out.write('%s%s%s%s%s%s> '
                            % (self.prefix, self.color, self.bars[0], RESET,
                               self.suffix, text))
            self.out.flush()
        if self.vt:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _paint(self):
        glyph = self.bars[self.frame % len(self.bars)]
        up = (UP % self.rows_up) if self.rows_up else ''
        try:
            with self.lock:
                self.out.write(SAVE + up + (COLUMN % self.column)
                                + self.color + glyph + RESET + RESTORE)
                self.out.flush()
        except (OSError, ValueError):
            # The stream closed under us - the prompt is over, and a spinner
            # is not worth an exception on the way out.
            self.done.set()

    def _run(self):
        while not self.done.wait(self.tick):
            self.frame += 1
            self._paint()

    def busy(self):
        """Call once input() has returned: yellow, one row up from here on."""
        self.color = YELLOW
        self.rows_up = 1

    def stop(self, ok):
        """Stop ticking and freeze the bar green or red - the last thing a
        scrolled-past line says about the question it carried."""
        self.done.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.color = GREEN if ok else RED
        if self.vt:
            self._paint()


def prompt(text, out, lock=None, ok=True, tick=TICK):
    return Prompt(text, out, lock=lock, ok=ok, tick=tick)
