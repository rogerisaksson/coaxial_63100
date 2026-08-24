"""A prompt with a robot and a state icon up front - and the spinner is the
"1" in "Coaxial_63100" itself.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; something turning in the name itself says which is waiting for a
question, and its colour says what kind of waiting: green for "nothing
submitted yet", yellow for "working on it", red for "that just failed". The
icon in the bookend group up front says the same thing a second way, in case
the colour alone does not survive whatever the terminal does to it.

    «<robot><icon>»Coaxial_63<bar>00>
           ^^^^^                    ^^^^ these are the only two things that
            state                      ever move or change colour - one
            icon                       discrete, on state change; one
                                        ticking, on a timer, resting on
                                        '1' when idle so the name reads
                                        normally between ticks

No space anywhere in this - not after the closing guillemet, not after the
final ">" - on purpose: asked for, and it also means every column after the
bookend group is exactly where the text's own characters put it, nothing
this file added shifting it over.

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
    <the whole       robot, icon, the text up to where the bar
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

How many rows "up" actually means is not fixed at 1 the moment Enter is
pressed, either - trace output between busy() and stop() can be one tool
result or five, a single line or a whole table, and the answer itself can
run to several lines before stop() ever runs. Measured live: a repaint that
still thought it was one row up landed inside a channel table mid-print,
overwriting the start of a data row with the prompt group instead of
climbing back to the actual prompt line above it. Prompt wraps whatever
stream it is given in a small counter that everything else - _trace(),
the final answer, an error line - writes through as well, so "up" is
recomputed from how many newlines have actually gone by since busy(), not
guessed at once and trusted for however long the question takes.

No console, no VT, or output redirected: the prompt is printed once, static,
and busy()/stop() only change state, painting nothing further. A script
piping commands in gets exactly what it would have got before any of this
existed.
"""
import threading

TICK = 0.12

ROBOT = '\U0001F916'                     # "🤖"
ROBOT_FALLBACK = 'o'

# All three need to be full-colour by default, not text glyphs forced into
# colour with a variation selector - a forced one can sit at a slightly
# different advance width than a native one in some fonts, which reads as
# uneven spacing next to the others even though there is no actual space
# character anywhere in this string. Confirmed twice now, not just
# suspected: the pause mark did this, and so did WARNING SIGN when it
# stood in for error - both are text-presentation by default and need
# U+FE0F to become colour; ❌ CROSS MARK does not, and was the original
# choice for exactly this reason.
ICON_WAIT = '\U0001F4A4'                 # "💤"
ICON_BUSY = '⌛'                    # "⌛"
ICON_ERROR = '❌'                    # "❌"
ICON_WAIT_FALLBACK = 'z'
ICON_BUSY_FALLBACK = '~'
ICON_ERROR_FALLBACK = 'X'

# Rests on '1' - the digit it replaces - so the name reads normally between
# ticks; the other three frames are what makes that digit's position turn.
# Plain ASCII throughout: this sits inside the board's own name, not next to
# an emoji, so there is no width question to inherit in the first place.
BARS = ('1', '/', '-', '\\')

# Guillemets rather than plain pipes framing the bookend group - narrower
# footprint than an emoji bracket would be, and already inside cp1252 (0x AB
# / 0xBB), so this bench's own console gets them too even though it cannot
# hold the robot it sits around. A stream that cannot even manage that -
# plain ASCII - gets the pipes back.
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
    """Whether this stream can hold the robot and the state icons.

    Decided once, for the whole set together: switching some bookends to the
    real glyph and others to ASCII mid-line would look like a bug, not a
    feature, so the fallback is all-or-nothing. The bar is not part of this
    check - it is plain ASCII regardless. Neither are the guillemets - see
    _brackets_capable(): they ask less of the stream than an emoji does, so
    they get to succeed on their own where the emoji group cannot.
    """
    return _encodable(out, ROBOT + ICON_WAIT + ICON_BUSY + ICON_ERROR)


def _brackets_capable(out):
    """Whether this stream can hold the guillemets framing the group.

    A separate, easier question from _capable(): « and » are cp1252, so
    this bench's own console answers yes to this and no to that - real
    brackets around ASCII bookends rather than falling all the way back
    to plain pipes just because the robot cannot render.
    """
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

    Anything can print between busy() and stop() - the caller does not have
    to be the one telling this how much. Wrapping the stream is the only way
    to know the true row count without every future caller remembering to
    report it themselves, and a caller that forgets is exactly how the bar
    ends up climbing to a row that no longer holds the prompt.

    The lock lives here too, not just around _paint()'s own writes - every
    write through this object holds it, including debug.py's own
    print(answer, file=face.out). Measured live: a tick landed mid-print of
    the actual answer, splicing the prompt group into the middle of a
    sentence, because that print() call was never part of the same critical
    section a tick's write already was. A caller-side lock only protects the
    calls someone remembered to wrap; a lock the stream itself holds
    protects all of them, including the one this bug was.
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
    """One prompt line: a robot/icon group up front, and a bar that
    ticks on its own thread until stop()ped - inside the text's own '1' if
    it has one, appended after it otherwise. Every repaint rewrites the
    whole group from column 1; see the module docstring for why that,
    rather than an absolute column, is what makes this correct. `out` is
    wrapped so a caller (debug.py's repl()) can point Chat's own output at
    the same tracked stream and have "how far up" stay correct regardless
    of what prints while the bar is busy."""

    def __init__(self, text, out, lock=None, ok=True, tick=TICK):
        # RLock, not Lock: _trace() in debug.py already holds this lock for
        # its whole loop of print()s, each of which re-enters it again
        # inside _Tracked.write() - a plain Lock would deadlock the second
        # of those against itself, on the same thread, over nothing.
        self.lock = lock or threading.RLock()
        self.out = _Tracked(out, self.lock)
        self.vt = _vt(out)
        self.tick = tick
        real = _capable(out)
        self.robot = ROBOT if real else ROBOT_FALLBACK
        self.icon_wait = ICON_WAIT if real else ICON_WAIT_FALLBACK
        self.icon_busy = ICON_BUSY if real else ICON_BUSY_FALLBACK
        self.icon_error = ICON_ERROR if real else ICON_ERROR_FALLBACK
        self.open = OPEN if _brackets_capable(out) else OPEN_FALLBACK
        self.close = CLOSE if _brackets_capable(out) else CLOSE_FALLBACK

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
            self.out.write(self._prefix() + self.tail + '>')
            self.out.flush()
        if self.vt:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _prefix(self):
        glyph = BARS[self.frame % len(BARS)]
        return '%s%s%s%s%s%s%s%s' % (self.open, self.robot, self.icon,
                                     self.close, self.head,
                                     self.color, glyph, RESET)

    def _paint(self):
        # rows_up is the fixed "Enter moved to a fresh row" step; out.lines
        # is however many more rows whatever else printed since busy() -
        # both together are the true climb back to the prompt line, not a
        # number decided once and trusted for as long as the question takes.
        rows = self.rows_up + self.out.lines if self.rows_up else 0
        up = (UP % rows) if rows else ''
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
        """Call once input() has returned: yellow, icon and bar both.

        Resets the line count first: this is the moment "one row up" starts
        being true, and it stays true only until the first thing prints.
        """
        self.out.lines = 0
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
