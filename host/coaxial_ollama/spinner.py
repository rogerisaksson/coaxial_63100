"""A prompt whose face changes colour with what is happening.

The point is telling two terminals apart. This prompt shares a docked panel
with a PowerShell one, and two of those with a `>` in them look identical at a
glance; a coloured face at the start of the line says which is which, and
which state it is in, without a word of text competing with whatever the
last answer already said.

The face itself never changes shape - a static robot, or the ASCII fallback a
console that cannot encode it gets instead. Only its colour does, across
three states:

    green   waiting    - blocked on input(), nothing submitted yet
    cyan    busy       - a question was submitted and is being worked on
    red     error      - that call raised; the coloured line is the record
                         of which question failed, once scrolled past
    green   done       - it answered cleanly; back to normal

There is no timer and no background thread. The first version animated on a
tick, which was solving a problem this one does not have: nothing here
changes unless an event drives it, so nothing needs repainting between
events. What is worth keeping from that version is the repaint trick itself
- the face is recoloured *in place*, never by reprinting the line, because
the line may already carry the question that was typed on it:

    ESC 7            save the cursor, wherever the answer is about to print
    ESC [ 1 A        up one row, onto the prompt line just submitted
    ESC [ 1 G        to its first column, where the face sits
    <face>           the same glyph, a different colour
    ESC 8            back down to where the answer belongs

No console, no VT, or output redirected: the prompt is printed once, static,
and recolouring does nothing. A script piping commands in gets exactly what
it would have got before any of this existed.
"""
ROBOT = '\U0001F916'                     # "🤖"
ROBOT_FALLBACK = '[o_o]'

SAVE = '\x1b7'
RESTORE = '\x1b8'
UP = '\x1b[1A'
COLUMN = '\x1b[1G'
GREEN = '\x1b[32m'
CYAN = '\x1b[36m'
RED = '\x1b[31m'
RESET = '\x1b[0m'


def _glyph(out):
    """The nicest face this stream can actually encode."""
    encoding = getattr(out, 'encoding', None) or 'ascii'
    try:
        ROBOT.encode(encoding)
    except (UnicodeEncodeError, LookupError, TypeError):
        return ROBOT_FALLBACK
    return ROBOT


def _vt(out):
    """Whether it is worth sending escape sequences to this stream at all."""
    try:
        return bool(out.isatty())
    except (AttributeError, ValueError):
        return False


class Prompt:
    """One prompt line, written once, whose leading face can be recoloured."""

    def __init__(self, text, out, ok=True):
        self.out = out
        self.glyph = _glyph(out)
        self.vt = _vt(out)
        color = GREEN if ok else RED
        out.write('%s%s%s %s> ' % (color, self.glyph, RESET, text))
        out.flush()

    def recolor(self, color):
        """Repaint the face on the line just submitted, without touching it.

        Only meaningful on a real terminal, and only right after input() has
        returned: the cursor then sits at column 1 of the fresh row Enter
        made, one row below the face - so up-one, repaint, back down is the
        whole trick, and it never overwrites the question that line carries.
        """
        if not self.vt:
            return
        try:
            self.out.write(SAVE + UP + COLUMN + color + self.glyph + RESET
                            + RESTORE)
            self.out.flush()
        except (OSError, ValueError):
            # The stream closed under us - not worth an exception on the way
            # out over a face nobody will see anyway.
            pass


def prompt(text, out, ok=True):
    """Write the prompt line, return the Prompt object that can recolor it."""
    return Prompt(text, out, ok=ok)
