"""ANSI text into rich: SGR runs to styles, cropped to a width, the sliding marquee."""
import re
import time

from rich.cells import cell_len
from rich.color import Color
from rich.measure import Measurement
from rich.segment import Segment
from rich.style import Style
from rich.text import Text


#: The marquee's pace: cells per second of travel, and the hold at each
#: end so the reader gets a full look before it turns back.
SLIDE_CPS = 5.0

SLIDE_HOLD = 1.2


def _slide(extra):
    """Where the window sits over art `extra` cells too wide: a ping-pong on
    the wall clock, held at each end.
    """

    travel = extra / SLIDE_CPS
    cycle = 2.0 * (travel + SLIDE_HOLD)
    t = time.monotonic() % cycle
    if t < SLIDE_HOLD:
        return 0
    t -= SLIDE_HOLD
    if t < travel:
        return int(extra * t / travel)
    t -= travel
    if t < SLIDE_HOLD:
        return extra
    return int(extra * (1.0 - (t - SLIDE_HOLD) / travel))


#: One SGR sequence, its parameters captured; and any escape at all, for
#: the line that carries something else.
_SGR = re.compile(r'\x1b\[([0-9;]*)m')

#: rich Styles by (foreground, background) Color, made once: a moving
#: attitude frame carries ~700 runs of ~230 colours, and building a
#: Style per run was a third of what decoding the art cost.
_STYLES = {}


def _sgr_colour(codes, j):
    """The colour at `codes[j]` as (rich Color or None for the default,
    background?, parameters taken); None when the parameter is not a
    colour or a reset - which is the line's cue to take the slow road."""
    c = codes[j]
    if c in (38, 48) and j + 4 < len(codes) and codes[j + 1] == 2:
        return Color.from_rgb(*codes[j + 2:j + 5]), c == 48, 5
    if c in (38, 48) and j + 2 < len(codes) and codes[j + 1] == 5:
        return Color.from_ansi(codes[j + 2]), c == 48, 3
    if 30 <= c <= 37 or 40 <= c <= 47:
        return Color.from_ansi(c % 10), c >= 40, 1
    if 90 <= c <= 97 or 100 <= c <= 107:
        return Color.from_ansi(c % 10 + 8), c >= 100, 1
    if c in (39, 49):
        return Color.default(), c == 49, 1
    return None


def _style_of(fg, bg):
    if fg is None and bg is None:
        return None
    got = _STYLES.get((fg, bg))
    if got is None:
        got = _STYLES[(fg, bg)] = Style(color=fg, bgcolor=bg)
    return got


#: What an SGR parameter string does to (foreground, background), decoded
#: once per distinct string: the art repeats a few hundred of them
#: thousands of times a frame, and parsing each into ints and a Color per
#: run was three times the cost of the split itself (measured, the first
#: version of this decoder against the prototype: 7.3 ms against 2.5).
#: KEEP leaves a channel as it was; None sets it to the default.
KEEP = object()

_SGR_DONE = {}

#: A list of these that would stop being a cache is a session's worth of
#: art gone wrong; the cap only bounds it.
SGR_KEPT = 4096


def _sgr_effect(params):
    """(foreground, background) after `params` - each a Color, None for the
    default, or KEEP - or False when the string holds a parameter this
    does not read."""
    fg = bg = KEEP
    codes = [int(c) for c in params.split(';') if c] or [0]
    j = 0
    while j < len(codes):
        if codes[j] == 0:
            fg, bg, took = None, None, 1
        else:
            got = _sgr_colour(codes, j)
            if got is None:
                return False
            colour, background, took = got
            if background:
                bg = colour
            else:
                fg = colour
        j += took
    return fg, bg


def _runs_of(line):
    """The line as [(text, Style or None)] runs, decoded here; None for a
    line carrying an escape this does not read (a bold, a cursor move),
    which Text.from_ansi then decodes whole."""
    pieces = _SGR.split(line)
    runs, fg, bg = [], None, None
    style = None
    for i in range(0, len(pieces), 2):
        text = pieces[i]
        if text:
            if '\x1b' in text:
                return None
            runs.append((text, style))
        if i + 1 == len(pieces):
            break
        params = pieces[i + 1]
        effect = _SGR_DONE.get(params)
        if effect is None:
            if len(_SGR_DONE) >= SGR_KEPT:
                _SGR_DONE.clear()
            effect = _SGR_DONE[params] = _sgr_effect(params)
        if effect is False:
            return None
        new_fg, new_bg = effect
        if new_fg is not KEEP:
            fg = new_fg
        if new_bg is not KEEP:
            bg = new_bg
        style = _style_of(fg, bg)
    return runs


def _cropped(segments, at, width):
    """`segments` sliced to cells [at, at + width), runs re-joined: the
    crop the Text slice made, on Segments."""
    cells = [(ch, s.style) for s in segments for ch in s.text][at:at + width]
    out, style, buf = [], None, []
    for ch, st in cells:
        if buf and st is not style:
            out.append(Segment(''.join(buf), style))
            buf = []
        style = st
        buf.append(ch)
    if buf:
        out.append(Segment(''.join(buf), style))
    return out


class Marquee:

    """Art cropped to its frame instead of wrapped, slid back and forth when
    it is wider: the bars shorten, and the hidden end comes past on the
    slide.
    """

    def __init__(self, art):
        self.lines, self.widths = [], []
        for raw in art.split('\n'):
            runs = _runs_of(raw)
            if runs is None:
                line = Text.from_ansi(raw)
                line.no_wrap, line.overflow = True, 'crop'
                self.lines.append(line)
                self.widths.append(line.cell_len)
            else:
                self.lines.append([Segment(text, style) for text, style in runs])
                self.widths.append(cell_len(''.join(t for t, _ in runs)))
        self.wide = max(self.widths, default=0)

    def __rich_measure__(self, console, options):
        return Measurement(min(self.wide, options.max_width), self.wide)

    def __rich_console__(self, console, options):
        width = options.max_width
        extra = self.wide - width
        at = _slide(extra) if extra > 0 else 0
        newline = Segment.line()
        for line, wide in zip(self.lines, self.widths):
            if isinstance(line, Text):
                if at or wide > width:
                    line = line[at:at + width]
                    line.no_wrap, line.overflow = True, 'crop'
                yield line
                continue
            if at or wide > width:
                line = _cropped(line, at, width)
            yield from line
            yield newline
