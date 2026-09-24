"""The house HUD over a page's drawing: the attitude view's look on every page.

An old CRT's snow in the drawing's blank cells, barely there - NOISE of them lit a frame,
a dot each in SNOW's near-black inks, redrawn NOISE_HZ times a second; its refresh sweep,
a beam going down over SWEEP_S seconds, the rows behind it glowing BEAM at the beam and
decaying over DECAY_ROWS as a phosphor does, under everything; lock brackets round
what the page draws, `ﾛｯｸ ｵﾝ` blinking on them; the page's clock bottom left; its status tag in red
kana, blinking, over a teal subtag bottom right. Chrome only fills blanks: nothing the
page draws is covered, and a piece with no room is left out whole. KANA names each page.
"""
import random
import time

from rich.cells import cell_len
from rich.color import Color
from rich.segment import Segment
from rich.style import Style

from coaxial.graphics.approach import AMBER, DATA, RED

#: Page title: (its name in kana, the status tag, the subtag).
KANA = {
    'COAXIAL 63100': ('ｺｱｷｼｬﾙ', '>> ｼｽﾃﾑ ﾀｲｷ', 'ｼｭﾀｰﾐﾅﾙ 01'),
    'SESSION': ('ｾｯｼｮﾝ', '>> ｼｽﾃﾑ ｶﾝｼ', 'ﾘｸﾞ ｲﾁﾗﾝ 6'),
    'BOARD ATTITUDE': ('ｷﾊﾞﾝ ｼｾｲ', '>> ﾁｬｸﾘｸﾁ ﾀﾝｻｸﾁｭｳ', 'ﾁﾊﾞ ｽﾌﾟﾗｳﾙ ﾅﾋﾞ 7G'),
    'SHAFT ANGLE': ('ｼﾞｸ ｶｸﾄﾞ', '>> ｶｸﾄﾞ ﾂｲｾｷ', 'A1335 ｼﾞｷ ｾﾝｻ'),
    'METER BRIDGE': ('ｹｲｷ ﾌﾞﾘｯｼﾞ', '>> ｹｲｿｸﾁｭｳ', 'ﾁｬﾝﾈﾙ 10 ｶﾝｼ'),
    'CAPTURE': ('ｷｬﾌﾟﾁｬ', '>> ｼｭｳﾛｸﾁｭｳ', 'ﾘﾝｸﾞ ﾊﾞｯﾌｧ 2'),
    'GATE DRIVERS': ('ｹﾞｰﾄ ﾄﾞﾗｲﾊﾞ', '>> ｹﾞｰﾄ ｸﾄﾞｳ', 'TIM1 50kHz ﾁｭｳｼﾝ'),
    'ROTOR OBSERVER': ('ﾛｰﾀ ｶﾝｿｸ', '>> ﾛｰﾀ ﾂｲｾｷ', 'ﾄﾞﾗｲﾌﾞ 10 ｽｲﾃｲ'),
    'THERMAL OBSERVER': ('ﾈﾂ ｶﾝｿｸ', '>> ﾈﾂ ｽｲﾃｲﾁｭｳ', 'ｵﾌﾞｻﾞｰﾊﾞ 8 ｺﾞｼﾞｹﾞﾝ'),
    'MOTOR CONTROLLER': ('ﾓｰﾀ ｾｲｷﾞｮ', '>> ｸﾄﾞｳ ｼﾞｭﾝﾋﾞ', 'ﾄﾞﾗｲﾌﾞ 10'),
    'BOARD CHAT': ('ﾂｳｼﾝ', '>> ﾂｳｼﾝﾁｭｳ', 'ﾛｰｶﾙ ﾓﾃﾞﾙ ｾﾂｿﾞｸ'),
    'RENDER': ('ｴﾝｼﾞﾝ', '>> ｴﾝｼﾞﾝ ｼｹﾝ', 'ｷｭｰﾌﾞ ﾃｽﾄﾍﾞｯﾄﾞ'),
}

NOISE = 0.035
NOISE_HZ = 15.0
SNOW = tuple(Style(color=Color.from_rgb(*rgb))
             for rgb in ((16, 22, 24), (22, 30, 33), (28, 38, 41)))
SWEEP_S = 5.0
DECAY_ROWS = 6.0
BEAM = (9, 24, 26)
#: The glow's steps, faintest first: a row takes the one its decay rounds to.
GLOW = tuple(Style(bgcolor=Color.from_rgb(*(int(ch * (i / 8.0) ** 2) for ch in BEAM)))
             for i in range(1, 9))
INK = {
    'lock': Style(color=Color.from_rgb(*AMBER)),
    'tag': Style(color=Color.from_rgb(*RED)),
    'sub': Style(color=Color.from_rgb(*DATA)),
}
LOCKED = 'ﾛｯｸ ｵﾝ'

_BORN = time.monotonic()


def clock(now=None):
    """'T+mm:ss.s': the page's time since it opened."""
    minutes, seconds = divmod((time.monotonic() if now is None else now) - _BORN, 60.0)
    return 'T+%02d:%04.1f' % (minutes % 100, seconds)


def _blank(cell):
    ch, style = cell
    return ch in (' ', '⠀') and (style is None or style.bgcolor is None)


class Chrome:
    """`inner` with the house HUD in its blank cells: the snow, lock brackets round
    what it draws (`lock`), the clock and `title`'s tags (KANA, `tags`)."""

    def __init__(self, inner, title, lock=True, tags=True):
        self.inner, self.title, self.lock, self.tags = inner, title, lock, tags

    def __rich_measure__(self, console, options):
        from rich.measure import Measurement
        return Measurement.get(console, options, self.inner)

    def __rich_console__(self, console, options):
        if options.height is None:                     # piped: no region to dress
            yield from console.render(self.inner, options)
            return
        t = time.monotonic()
        rows = [_cells(line) for line in console.render_lines(self.inner, options, pad=True)]
        if rows:
            _dress(rows, self.title if self.tags else None, self.lock, t)
        newline = Segment.line()
        for row in rows:
            yield from _segments(row)
            yield newline


def _cells(line):
    """[[char, style]] a cell each; a wide char's second cell ['', style]."""
    out = []
    for seg in line:
        if seg.control:
            continue
        for ch in seg.text:
            out.append([ch, seg.style])
            if cell_len(ch) == 2:
                out.append(['', seg.style])
    return out


def _segments(row):
    run, style = [], None
    for ch, st in row:
        if st != style and run:
            yield Segment(''.join(run), style)
            run = []
        style = st
        run.append(ch)
    if run:
        yield Segment(''.join(run), style)


def _put(rows, r, c, text, style):
    """`text` at (r, c) if every cell it takes is in the region and blank."""
    if not (0 <= r < len(rows) and c >= 0 and c + len(text) <= len(rows[r])):
        return False
    if not all(_blank(rows[r][c + i]) for i in range(len(text))):
        return False
    for i, ch in enumerate(text):
        rows[r][c + i] = [ch, style]
    return True


def _dress(rows, title, lock, t):
    height, width = len(rows), len(rows[0])
    drawn = [(r, c) for r in range(height) for c in range(width) if not _blank(rows[r][c])]
    kana = KANA.get(title)
    # The tags and the clock first: the snow goes round them.
    if kana and height >= 6 and width >= 40:
        _put(rows, height - 1, 1, clock(t), INK['lock'])
        if int(t * 1.5) % 2 == 0:
            _put(rows, height - 1, width - len(kana[1]) - 1, kana[1], INK['tag'])
        _put(rows, height - 2, width - len(kana[2]) - 1, kana[2], INK['sub'])
    if lock and drawn and height >= 8:
        top = min(r for r, _c in drawn) - 1
        bottom = max(r for r, _c in drawn) + 1
        left = min(c for _r, c in drawn) - 2
        right = max(c for _r, c in drawn) + 1
        if top >= 0 and bottom < height and left >= 0 and right < width - 1:
            for r, c, text in ((top, left, '┌─'), (top, right, '─┐'),
                               (bottom, left, '└─'), (bottom, right, '─┘')):
                _put(rows, r, c, text, INK['lock'])
            if int(t * 2.0) % 2 == 0:
                _put(rows, top, left + 3, LOCKED, INK['lock'])
    rng = random.Random(int(t * NOISE_HZ))
    for _ in range(int(NOISE * width * height)):
        r, c = rng.randrange(height), rng.randrange(width)
        # Off any text or line by a cell: a dot beside one reads as part of it.
        if all(_blank(rows[m][n]) for m in (r - 1, r, r + 1) for n in (c - 1, c, c + 1)
               if 0 <= m < height and 0 <= n < width):
            rows[r][c] = [chr(0x2800 + (1 << rng.randrange(8))), rng.choice(SNOW)]
    # The sweep last, under everything: a background, so what is drawn keeps its ink.
    beam = (t / SWEEP_S) % 1.0 * (height + DECAY_ROWS)
    for r in range(max(0, int(beam - DECAY_ROWS)), min(height, int(beam) + 1)):
        step = int((1.0 - (beam - r) / DECAY_ROWS) * len(GLOW))
        if step > 0:
            glow = GLOW[min(len(GLOW), step) - 1]
            for cell in rows[r]:
                cell[1] = glow if cell[1] is None else cell[1] + glow
