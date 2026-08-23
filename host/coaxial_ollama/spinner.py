"""A prompt that keeps blinking while you type.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; something moving says which is waiting for a question - and a small
face says what kind of thing is waiting, rather than a bare spinner bar that
could just as well be a progress meter.

The first version stopped at the first keypress, because a spinner that redraws
the whole line repaints under the characters being typed - which is how a
prompt eats an argument. Stopping made it useless exactly when you are looking
at it, so this one repaints *only its own cell* and puts the cursor back:

    ESC 7            save the cursor, wherever the typing has got to
    ESC [ <col> G    go to the spinner's column, on this row
    <face>           the whole glyph, one frame
    ESC 8            back to where the cursor was

The typed text is never written over, because it is never written to. Every
frame within a state is the same width for exactly this reason: the repaint
only writes over the glyph's own columns, so a shorter frame following a
longer one would leave a stray character behind, and a longer one would eat
into the prompt text that follows it on the same line. What can still go
wrong is a line long enough to wrap: the column is on the current row, so a
wrapped line puts the face on the wrong one. A bench question is not that
long, and the alternative - tracking the wrap - is a terminal emulator.

No console, no VT, or output redirected: the prompt is printed once, static,
and `stop()` does nothing. A script piping commands in gets exactly what it
would have got before any of this existed.

Colour and the face itself both carry the same one bit: whether the board link
came up. Green and blinking reads as "normal, thinking"; red and glitching
between an error face and a crying one reads as "something is off" without a
word of text - useful exactly at the moment you are not looking at the last
error line any more.
"""
import threading
import time

TICK = 0.12                      # seconds per frame

# Two frames per state, and every frame the same width - see the module
# docstring for why. The bullet is plain cp1252 (0x95), not a wide or
# combining character, so it costs one column same as an ASCII 'o'.
OK_GLYPHS = ('[•_•]', '[•o•]')      # [.-.]  [.o.] - blinking
OK_FALLBACK = ('[o_o]', '[o_O]')
BAD_GLYPHS = ('[x_x]', '[T_T]')                          # dead / crying
BAD_FALLBACK = BAD_GLYPHS                                # already plain ASCII

SAVE = '\x1b7'
RESTORE = '\x1b8'
COLUMN = '\x1b[%dG'
GREEN = '\x1b[32m'
RED = '\x1b[31m'
RESET = '\x1b[0m'


def _frames(out, ok=True):
    """The nicest face this stream can actually encode, for this state."""
    glyphs, fallback = (OK_GLYPHS, OK_FALLBACK) if ok else (BAD_GLYPHS, BAD_FALLBACK)
    encoding = getattr(out, 'encoding', None) or 'ascii'
    try:
        ''.join(glyphs).encode(encoding)
    except (UnicodeEncodeError, LookupError, TypeError):
        return fallback
    return glyphs


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class Spinner:
    """Paints one face over the same cell on a timer. Stop it before printing."""

    def __init__(self, out, column, glyphs, tick=TICK, ok=True):
        self.out = out
        self.column = column
        self.glyphs = glyphs
        self.tick = tick
        self.ok = ok
        self.color = GREEN if ok else RED
        self.done = threading.Event()
        self.thread = None

    def _glyph(self, frame):
        return self.glyphs[frame % len(self.glyphs)]

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        frame = 0
        while not self.done.wait(self.tick):
            frame += 1
            glyph = self._glyph(frame)
            try:
                self.out.write(SAVE + (COLUMN % self.column) +
                               self.color + glyph + RESET + RESTORE)
                self.out.flush()
            except (OSError, ValueError):
                # The stream closed under us - the prompt is over, and a
                # spinner is not worth an exception on the way out.
                return

    def stop(self):
        """Stop painting and wait for the last frame to land.

        Joined rather than left to finish on its own: the next thing to happen
        is the answer being printed, and a frame arriving in the middle of it
        would put a glyph inside the text.
        """
        self.done.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None


def spinning_prompt(prompt, out, tick=TICK, ok=True):
    """Write the prompt, start the face blinking in it, return something to stop.

    The prompt is written whole and once, so `input()` reads a line that is
    already on screen; only the face's own columns, at the very start of the
    line, are touched afterwards.

    `ok` is whether the board link is up: a green, blinking face when it is,
    a red one glitching between an error and a crying face when it is not -
    so the state is visible without a line of text competing with whatever
    the last answer already said about it.
    """
    glyphs = _frames(out, ok)
    color = GREEN if ok else RED
    line = '%s%s%s %s> ' % (color, glyphs[0], RESET, prompt)
    out.write(line)
    out.flush()

    if not _vt(out):
        return lambda: None

    # 1-based column of the face: the very first thing on the line.
    return Spinner(out, 1, glyphs, tick, ok=ok).start().stop
