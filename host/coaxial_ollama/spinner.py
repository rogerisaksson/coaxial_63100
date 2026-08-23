"""A prompt that keeps turning while you type.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; something turning says which is waiting for a question.

The first version stopped at the first keypress, because a spinner that redraws
the whole line repaints under the characters being typed - which is how a
prompt eats an argument. Stopping made it useless exactly when you are looking
at it, so this one repaints *only its own cell* and puts the cursor back:

    ESC 7            save the cursor, wherever the typing has got to
    ESC [ <col> G    go to the spinner's column, on this row
    <glyph>          one character
    ESC 8            back to where the cursor was

The typed text is never written over, because it is never written to. What can
still go wrong is a line long enough to wrap: the column is on the current row,
so a wrapped line puts the spinner on the wrong one. A bench question is not
that long, and the alternative - tracking the wrap - is a terminal emulator.

No console, no VT, or output redirected: the prompt is printed once, static,
and `stop()` does nothing. A script piping commands in gets exactly what it
would have got before any of this existed.

Colour and direction both carry the same one bit: whether the board link came
up. Green and turning the way the glyphs are listed reads as "normal"; red and
turning backward reads as "something is off" without a word of text - useful
exactly at the moment you are not looking at the last error line any more.
"""
import threading
import time

TICK = 0.12                      # seconds per frame

# An en dash for the horizontal bar, not a hyphen: at the size a terminal draws
# them, `-` is a third the width of `|` and the spinner visibly limps. cp1252
# has the en dash at 0x96 so this console renders it, but a console that cannot
# gets the ASCII set instead rather than a question mark in the corner of the
# eye - see FALLBACK and _frames.
GLYPHS = ('|', '/', '–', '\\')
FALLBACK = ('|', '/', '-', '\\')

SAVE = '\x1b7'
RESTORE = '\x1b8'
COLUMN = '\x1b[%dG'
GREEN = '\x1b[32m'
RED = '\x1b[31m'
RESET = '\x1b[0m'


def _frames(out):
    """The nicest set of glyphs this stream can actually encode."""
    encoding = getattr(out, 'encoding', None) or 'ascii'
    try:
        ''.join(GLYPHS).encode(encoding)
    except (UnicodeEncodeError, LookupError, TypeError):
        return FALLBACK
    return GLYPHS


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class Spinner:
    """Paints one cell of the prompt on a timer. Stop it before printing."""

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
        # Forward through the tuple when the link is up, backward when it is
        # not - the same four glyphs, read the other way round, is a spinner
        # turning the other way with no extra frames to draw.
        step = frame if self.ok else -frame
        return self.glyphs[step % len(self.glyphs)]

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
    """Write the prompt, start the spinner in it, return something to stop.

    The prompt is written whole and once, so `input()` reads a line that is
    already on screen; only the spinner's own column is touched afterwards.

    `ok` is whether the board link is up: green and forward when it is, red
    and backward when it is not, so the state is visible without a line of
    text competing with whatever the last answer already said about it.
    """
    glyphs = _frames(out)
    color = GREEN if ok else RED
    line = '%s%s%s%s> ' % (prompt, color, glyphs[0], RESET)
    out.write(line)
    out.flush()

    if not _vt(out):
        return lambda: None

    # 1-based column of the glyph: right after the prompt, no space between.
    return Spinner(out, len(prompt) + 1, glyphs, tick, ok=ok).start().stop
