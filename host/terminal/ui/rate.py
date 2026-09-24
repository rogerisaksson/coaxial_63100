"""The frame rate and one frame's cost, eased, and the corner that prints them."""
import time
import weakref

from rich.cells import cell_len
from rich.measure import Measurement
from rich.segment import Segment


#: The newest frame's weight in the drawing's rate: about five frames
#: smoothed, so the figure reads rather than flickers.
RATE_WEIGHT = 0.2

_RATES = weakref.WeakKeyDictionary()


def _eased(was, now):
    return now if was is None else was + RATE_WEIGHT * (now - was)


class Rate:
    """Frames a second, start to start, and one frame's cost from the
    draw's start to the terminal's write: the drawing's top-left corner."""

    def __init__(self):
        self.began = self.fps = self.ms = None

    def tick(self, started):
        self.ms = _eased(self.ms, 1000.0 * (time.monotonic() - started))
        if self.began is not None and started > self.began:
            self.fps = _eased(self.fps, 1.0 / (started - self.began))
        self.began = started

    def label(self):
        if self.ms is None:
            return ''
        fps = '%.1f fps  ' % self.fps if self.fps is not None else ''
        return '%s%.0f ms' % (fps, self.ms)


def rate_of(console):
    """The frame rate drawn on this console, made on first ask."""
    if console not in _RATES:
        _RATES[console] = Rate()
    return _RATES[console]


class Corner:
    """`inner` with `label` painted over its top-left cells: the frame rate
    in the drawing's own corner, no box of its own."""

    def __init__(self, inner, label):
        self.inner, self.label = inner, label

    def __rich_measure__(self, console, options):
        return Measurement.get(console, options, self.inner)

    def __rich_console__(self, console, options):
        lines = console.render_lines(self.inner, options, pad=True)
        wide = cell_len(self.label)
        if lines and 0 < wide < options.max_width:
            cut = list(Segment.divide(lines[0], [wide, options.max_width]))
            # Text.render drops a base style with no spans: the label came out white.
            lines[0] = ([Segment(self.label, console.get_style('rate'))]
                        + (cut[1] if len(cut) > 1 else []))
        newline = Segment.line()
        for line in lines:
            yield from line
            yield newline
