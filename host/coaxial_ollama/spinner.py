"""A prompt with a robot, a state icon and the board's own pager up front -
and the spinner is the "1" in "Coaxial_63100" itself.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; something turning in the name itself says which is waiting for a
question, and its colour says what kind of waiting: green for "nothing
submitted yet", yellow for "working on it", red for "that just failed". The
icon in the bookend group up front says the same thing a second way, in case
the colour alone does not survive whatever the terminal does to it.

    |<robot><icon><pager>| Coaxial_63<bar>00>
           ^^^^^                     ^^^^ these are the only two things that
            state                       ever move or change colour - one
            icon                        discrete, on state change; one
                                         ticking, on a timer, resting on
                                         '1' when idle so the name reads
                                         normally between ticks

If the text has no '1' to spin - not this board's, but Prompt takes whatever
it is given - the bar is appended after it instead, exactly where the very
first version of this put it.

Every repaint rewrites the whole bookend-and-bar group from column 1, using
carriage return rather than an absolute-column escape:

    ESC 7            save the cursor, wherever input() has got to
    ESC [ <n> A      up n rows, onto the prompt line (0 while still typing,
                     1 once Enter has moved the cursor to a fresh row)
    CR               to column 1 of that row - always exactly column 1,
                     never a number this module has to get right
    <the whole       robot, icon, pager, the text up to where the bar
     prefix>         sits, and the bar itself, one frame, in colour
    ESC 8            back to where the cursor was

This is not the cosmetic choice it looks like. The first two versions of
this computed an absolute column with Python's len() and jumped straight to
it - which is exactly wrong the moment 🤖 or 📟 render as two terminal
columns instead of the one len() counts, and most terminals render pictured
emoji exactly that wide. Landing two columns short of where '1' actually
sits is not a rounding error, it is a different character - measured on
this bench as the bar spinning "6" instead of "1", once real emoji reached
a real terminal instead of a StringIO fixture. A rewrite from column 1 does
not have this problem because it never asks how wide anything is: the
terminal advances the cursor by however many columns each glyph actually
costs, and the write always starts and ends at the same place as long as
the icon options themselves render at one consistent width - which is the
one thing this file still has to get right on purpose, not compute. The
bar's own frames are plain ASCII for exactly this reason: no width
ambiguity to inherit in the first place.

`_trace()` in debug.py can print mid-question, while the bar is still
ticking for the busy phase - the caller passes in the same lock both sides
write through, so a tick and a trace line never interleave into garbage.

No console, no VT, or output redirected: the prompt is printed once, static,
and busy()/stop() only change state, painting nothing further. A script
piping commands in gets exactly what it would have got before any of this
existed.
"""
import threading

TICK = 0.12

ROBOT = '\U0001F916'                     # "🤖"
ROBOT_FALLBACK = 'o'
PAGER = '\U0001F4DF'                     # "📟"
PAGER_FALLBACK = '#'

# U+FE0F (the variation selector after the pause mark) forces emoji
# presentation on it, which - unlike the other two - is text-presentation by
# default in most fonts. Matching their rendered width no longer matters for
# the column math (there is none any more), but it still matters for not
# looking like a plain punctuation mark sitting between two full-colour
# pictures.
ICON_WAIT = '⏸️'               # "⏸️"
ICON_BUSY = '\U0001F504'                 # "🔄"
ICON_ERROR = '❌'                    # "❌"
ICON_WAIT_FALLBACK = '.'
ICON_BUSY_FALLBACK = '~'
ICON_ERROR_FALLBACK = 'X'

# Rests on '1' - the digit it replaces - so the name reads normally between
# ticks; the other three frames are what makes that digit's position turn.
# Plain ASCII throughout: this sits inside the board's own name, not next to
# an emoji, so there is no width question to inherit in the first place.
BARS = ('1', '/', '-', '\\')

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
    """Whether this stream can hold the robot, pager and state icons.

    Decided once, for the whole set together: switching some bookends to the
    real glyph and others to ASCII mid-line would look like a bug, not a
    feature, so the fallback is all-or-nothing. The bar is not part of this
    check - it is plain ASCII regardless.
    """
    return _encodable(out, ROBOT + PAGER + ICON_WAIT + ICON_BUSY + ICON_ERROR)


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class Prompt:
    """One prompt line: a robot/icon/pager group up front, and a bar that
    ticks on its own thread until stop()ped - inside the text's own '1' if
    it has one, appended after it otherwise. Every repaint rewrites the
    whole group from column 1; see the module docstring for why that,
    rather than an absolute column, is what makes this correct."""

    def __init__(self, text, out, lock=None, ok=True, tick=TICK):
        self.out = out
        self.lock = lock or threading.Lock()
        self.vt = _vt(out)
        self.tick = tick
        real = _capable(out)
        self.robot = ROBOT if real else ROBOT_FALLBACK
        self.pager = PAGER if real else PAGER_FALLBACK
        self.icon_wait = ICON_WAIT if real else ICON_WAIT_FALLBACK
        self.icon_busy = ICON_BUSY if real else ICON_BUSY_FALLBACK
        self.icon_error = ICON_ERROR if real else ICON_ERROR_FALLBACK

        spin_at = text.find('1')
        if spin_at == -1:
            self.head, self.tail = text, ''
        else:
            self.head, self.tail = text[:spin_at], text[spin_at + 1:]

        self.icon = self.icon_wait if ok else self.icon_error
        self.color = GREEN if ok else RED
        self.rows_up = 0                 # 0 while still on this row, 1 after
        self.frame = 0
        self.done = threading.Event()
        self.thread = None

        with self.lock:
            self.out.write(self._prefix() + self.tail + '> ')
            self.out.flush()
        if self.vt:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _prefix(self):
        glyph = BARS[self.frame % len(BARS)]
        return '|%s%s%s| %s%s%s%s' % (self.robot, self.icon, self.pager,
                                       self.head, self.color, glyph, RESET)

    def _paint(self):
        up = (UP % self.rows_up) if self.rows_up else ''
        try:
            with self.lock:
                self.out.write(SAVE + up + '\r' + self._prefix() + RESTORE)
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
        """Call once input() has returned: yellow, icon and bar both."""
        self.color = YELLOW
        self.icon = self.icon_busy
        self.rows_up = 1
        if self.vt:
            self._paint()

    def stop(self, ok):
        """Stop ticking and freeze the whole group - the last thing a
        scrolled-past line says about the question it carried."""
        self.done.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.color = GREEN if ok else RED
        self.icon = self.icon_wait if ok else self.icon_error
        if self.vt:
            self._paint()


def prompt(text, out, lock=None, ok=True, tick=TICK):
    return Prompt(text, out, lock=lock, ok=ok, tick=tick)
