"""The prompt line: a bookend group, the board name, and what it is on.

    «<robot><icon>»Coaxial 63100(JTAG and COM4, node 1)>
           ^^^^^^                ^^^^^^^^^^^^^^^^^^^^^
           the only thing that   the link and the node,
           moves - state, then   green board / yellow
           turning while busy    stand-in / red broadcast

Two terminals in one docked panel look identical at a glance; something
turning says which one is waiting for a question, and the colour says
green idle, yellow working, red just failed.

Three defects found in use, and what each one needs:

* **Nothing turns inside the name.** The first version span the "1" in
  "Coaxial 63100" and wrote "Coaxial 63-00" into the operator's own
  transcript. A name is not a widget.
* **Every repaint rewrites the group from column 1** - ESC 7, UP, CR,
  prefix, ESC 8 - never a computed column. len() counts 🤖 as one and
  terminals draw it as two, so computed columns landed two short.
* **"Up" is recounted, not remembered.** Trace output between busy() and
  stop() can be one line or a whole table, so `_Tracked` counts the
  newlines that actually went by. A repaint that still thought it was one
  row up landed inside a channel table mid-print.

The caller passes the lock `_trace()` writes through, so a tick cannot
splice itself into the middle of an answer. No console, no VT, or output
redirected: printed once, static, and busy()/stop() paint nothing.
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
ICON_ERROR = '❌'                    # "❌"
ICON_WAIT_FALLBACK = 'z'
ICON_ERROR_FALLBACK = 'X'

# The busy icon turns in place, in the bookend group. Emoji frames, for the
# same reason the icons are: one glyph swaps for another at the same advance
# width, so nothing after the group moves while it ticks. Both hourglasses are
# emoji-presentation by default, so neither needs a variation selector and the
# forced-colour case measured as uneven spacing does not arise.
#
# Three frames, two of them the same: the sand falls for two ticks and the
# glass is done for one, which reads as a flip rather than a strobe.
#
# The first version span the '1' in "Coaxial 63100" itself, splitting the name
# around it. It was measured writing "Coaxial 63-00" and "Coaxial 63\00" into
# the operator's own transcript, twice, and read as a corrupted board name
# rather than as a spinner. A name is not a widget.
SPIN = ('\U000023F3', '\U000023F3', '\U0000231B')   # flowing, flowing, done
SPIN_FALLBACK = ('|', '/', '-', '\\')

# Guillemets rather than plain pipes framing the bookend group - narrower
# footprint than an emoji bracket would be, and already inside cp1252 (0x AB
# / 0xBB), so a legacy console gets them too even though it cannot
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
    return _encodable(out, ROBOT + ICON_WAIT + SPIN[0] + ICON_ERROR)


def _brackets_capable(out):
    """Whether this stream can hold the guillemets framing the group.

    A separate, easier question from _capable(): « and » are cp1252, so
    a legacy console answers yes to this and no to that - real
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

    def __init__(self, text, out, lock=None, ok=True, tick=TICK,
                 tag=None, tag_ok=True):
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
        # stand-in. Written once, after `tail`, and never repainted - _paint()
        # rewrites only the prefix, which ends before this. The colour is the
        # whole point: a session that quietly fell back to invented values is
        # the one thing on this screen a reader must not have to ask about.
        self.tag = tag
        # Green a board, yellow a stand-in, and RED every node at once.
        # Broadcast is the one mode where a command reaches every inverter
        # on the bus and nothing answers to say it landed - on a bus of
        # 63 V / 100 A drives that is worth a colour of its own, and red is
        # the one already spent on "something is wrong here".
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
        """The bookend group and the name - everything before the tag.

        One glyph moves: the icon, which becomes a turning frame while the
        model is working. The name is written whole, in the state colour.
        """
        icon = (self.spin[self.frame % len(self.spin)]
                if self.busy_now else self.icon)
        return '%s%s%s%s%s%s%s' % (self.open, self.robot, icon, self.close,
                                   self.color, self.name, RESET)

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
        """Call once input() has returned: the icon starts turning, yellow.

        Resets the line count first: this is the moment "one row up" starts
        being true, and it stays true only until the first thing prints.
        """
        self.out.lines = 0
        self.color = YELLOW
        self.busy_now = True
        self.rows_up = 1
        if self.vt:
            self._paint()

    def stop(self, ok):
        """Stop ticking and freeze the whole group - the last thing a
        scrolled-past line says about the question it carried.

        Ticking stops and the icon goes back to a state glyph: a row
        scrolled past says how its question ended, not whatever frame
        the ticker happened to land on.
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
