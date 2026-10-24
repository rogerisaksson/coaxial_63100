"""The house HUD: the attitude view's look on every page.

Over the whole screen (`Crt`, every frame `stage.curtain` shows): an old CRT's snow in
its blank cells, barely there - NOISE of them lit a frame, a dot each in SNOW's
near-black inks, redrawn NOISE_HZ times a second; its refresh sweep, a beam going down
over SWEEP_S seconds and decaying behind it over DECAY_ROWS as a phosphor does, in faint
braille only: BAND_DENSITY of a row's cells a dot at the beam, thinning as the square of
the decay, its ink stepping down BAND_INK to the snow's. No background: a glow a row at a
time banded.
Over a page's drawing (`Chrome`): lock brackets round what the page draws, `ﾛｯｸ ｵﾝ` blinking on them; the page's clock bottom left; its status tag in red
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
    'HUMANOID': ('ﾋｭｰﾏﾉｲﾄﾞ', '>> ﾎｺｳﾁｭｳ', 'ｼﾞｭｳﾘｮｸ 9.81'),
    'QUAD ACRO FLYING': ('ｸｱｯﾄﾞ ｱｸﾛ', '>> ﾋｺｳﾁｭｳ', 'ﾛｰﾀ 4 ｾﾞﾝｶｲ'),
    'HELICOPTER 3D FLYING': ('ﾍﾘｺﾌﾟﾀ 3D', '>> ﾋｺｳﾁｭｳ', 'ﾛｰﾀ 2 3D'),
    'ORNITHOPTER DUNE FLYING': ('ｵｰﾆｿﾌﾟﾀ', '>> ﾊﾊﾞﾀｷ ﾋｺｳ', 'ﾖｸ 4 ｻﾊﾞｸ'),
    'AIRCRAFT XE AEROBATICS': ('ｴｱﾛﾊﾞﾃｨｸｽ', '>> ｷｮｸｷﾞ ﾋｺｳ', '3D ﾄﾙｸﾛｰﾙ'),
    'QUIT': ('ｼｭｳﾘｮｳ', '>> ｼｭｳﾘｮｳ', 'ﾀｰﾐﾅﾙ ﾍｲｻ'),
}

NOISE = 0.035
NOISE_HZ = 15.0
SNOW = tuple(Style(color=Color.from_rgb(*rgb))
             for rgb in ((16, 22, 24), (22, 30, 33), (28, 38, 41)))
SWEEP_S = 5.0
DECAY_ROWS = 6.0
BAND_DENSITY = 0.08
#: The band's inks, faintest first: from the snow's brightest up a shade at a time.
BAND_INK = tuple(Style(color=Color.from_rgb(28 + 2 * i, 38 + 3 * i, 41 + 3 * i))
                 for i in range(8))
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


#: A blank cell's characters: a space, an empty braille cell.
BLANK = ' \u2800'

#: A wide character's second cell in a row's cells.
WIDE = '\0'


class _Row:

    """A rendered line as rich gave it - its segments, control codes left out -, its cells' chars
    (`text`, a wide one's second WIDE) and backgrounds (`bg`) for the blank tests, and the cells
    `put` over it, {cell: (char, style)}."""

    __slots__ = ('segs', 'text', 'bg', 'put')

    def __init__(self, line):
        self.segs, self.put = [seg for seg in line if not seg.control], {}
        text = ''.join(seg.text for seg in self.segs)
        if cell_len(text) == len(text):             # a cell a char: the row measured once
            self.text, self.bg, at = text, bytearray(len(text)), 0
            for seg in self.segs:
                end = at + len(seg.text)
                if seg.style is not None and seg.style.bgcolor is not None:
                    self.bg[at:end] = b'\x01' * (end - at)
                at = end
            return
        chars, self.bg = [], bytearray()
        for seg in self.segs:
            text, style = seg.text, seg.style
            back = b'\x01' if style is not None and style.bgcolor is not None else b'\x00'
            if cell_len(text) == len(text):
                chars.append(text)
                self.bg += back * len(text)
                continue
            for ch in text:
                wide = cell_len(ch) == 2
                chars.append(ch + WIDE if wide else ch)
                self.bg += back * (2 if wide else 1)
        self.text = ''.join(chars)

    def blank(self, c):
        """Whether cell `c` shows nothing: a blank char and no background of its own."""
        cell = self.put.get(c)
        if cell is None:
            return self.text[c] in BLANK and not self.bg[c]
        return cell[0] in BLANK and (cell[1] is None or cell[1].bgcolor is None)

    def drawn(self):
        """(its first, its last cell drawn), or None: a blank under a background drawn too."""
        text, spans = self.text, []
        left, right = len(text) - len(text.lstrip(BLANK)), len(text.rstrip(BLANK)) - 1
        if left <= right:
            spans.append((left, right))
        if self.bg.find(1) >= 0:
            spans.append((self.bg.find(1), self.bg.rfind(1)))
        put = [c for c in self.put if not self.blank(c)]
        if put:
            spans.append((min(put), max(put)))
        return (min(a for a, _b in spans), max(b for _a, b in spans)) if spans else None

    def segments(self):
        """Its segments, the cells put over it in their places."""
        if not self.put:
            yield from self.segs
            return
        puts, k, at = sorted(self.put), 0, 0
        for seg in self.segs:
            text, style = seg.text, seg.style
            width = cell_len(text)
            here = []
            while k < len(puts) and puts[k] < at + width:
                here.append(puts[k])
                k += 1
            if not here:
                yield seg
                at += width
                continue
            cells = [ch for ch in text] if width == len(text) else [
                piece for ch in text for piece in ((ch, '') if cell_len(ch) == 2 else (ch,))]
            for c in here:
                cells[c - at] = None
            run = []
            for i, ch in enumerate(cells):
                if ch is None:
                    if run:
                        yield Segment(''.join(run), style)
                        run = []
                    put_ch, put_style = self.put[at + i]
                    yield Segment(put_ch, put_style)
                else:
                    run.append(ch)
            if run:
                yield Segment(''.join(run), style)
            at += width


class Chrome:
    """`inner` with the house HUD in its blank cells: lock brackets round
    what it draws (`lock`), the clock and `title`'s tags (KANA, `tags`); `corner` (the frame
    rate, `rate.Corner`'s) over its first cells, over all of it."""

    def __init__(self, inner, title, lock=True, tags=True, corner=''):
        self.inner, self.title, self.lock, self.tags, self.corner = inner, title, lock, tags, corner

    def __rich_console__(self, console, options):
        newline = Segment.line()
        if options.height is None:                     # piped: no region to dress
            lines = console.render_lines(self.inner, options, pad=True)
        else:
            t = time.monotonic()
            rows = [_Row(line) for line in console.render_lines(self.inner, options, pad=True)]
            if rows:
                _dress(rows, self.title if self.tags else None, self.lock, t)
            lines = [list(row.segments()) for row in rows]
        wide = cell_len(self.corner)
        if lines and 0 < wide < options.max_width:
            cut = list(Segment.divide(lines[0], [wide, options.max_width]))
            lines[0] = [Segment(self.corner, console.get_style('rate'))] + (cut[1] if len(cut) > 1 else [])
        for line in lines:
            yield from line
            yield newline


def _put(rows, r, c, text, style):
    """`text` at (r, c) if every cell it takes is in the region and blank."""
    if not (0 <= r < len(rows) and c >= 0 and c + len(text) <= len(rows[r].text)):
        return False
    row = rows[r]
    if not all(row.blank(c + i) for i in range(len(text))):
        return False
    for i, ch in enumerate(text):
        row.put[c + i] = (ch, style)
    return True


def _dress(rows, title, lock, t):
    height, width = len(rows), len(rows[0].text)
    spans = [(r, span) for r, span in ((r, row.drawn()) for r, row in enumerate(rows)) if span]
    kana = KANA.get(title)
    # The tags and the clock first: the lock goes round them.
    if kana and height >= 6 and width >= 40:
        _put(rows, height - 1, 1, clock(t), INK['lock'])
        if int(t * 1.5) % 2 == 0:
            _put(rows, height - 1, width - len(kana[1]) - 1, kana[1], INK['tag'])
        _put(rows, height - 2, width - len(kana[2]) - 1, kana[2], INK['sub'])
    if lock and spans and height >= 8:
        top, bottom = spans[0][0] - 1, spans[-1][0] + 1
        left = min(span[0] for _r, span in spans) - 2
        right = max(span[1] for _r, span in spans) + 1
        corners = ((top, left, '┌─'), (top, right, '─┐'), (bottom, left, '└─'),
                   (bottom, right, '─┘'))
        # All four or none: half a lock read as a broken one.
        if (top >= 0 and bottom < height and left >= 0 and right < width - 1
                and all(rows[r].blank(c + i) for r, c, text in corners
                        for i in range(len(text)))):
            for r, c, text in corners:
                _put(rows, r, c, text, INK['lock'])
            if int(t * 2.0) % 2 == 0:
                _put(rows, top, left + 3, LOCKED, INK['lock'])


class Crt:
    """`inner` on an old CRT: its snow in the blank cells, its refresh sweep under all
    that has no background of its own."""

    def __init__(self, inner):
        self.inner = inner

    def __rich_console__(self, console, options):
        # A screen-mode Live gives no height: the screen's, as a Layout takes it. Passed
        # through on None, there was no CRT at all (42f9432).
        options = options.update(height=options.height or console.height)
        rows = [_Row(line) for line in console.render_lines(self.inner, options, pad=True)]
        if rows and rows[0].text:
            _crt(rows, time.monotonic())
        newline = Segment.line()
        for row in rows:
            yield from row.segments()
            yield newline


def _crt(rows, t):
    height, width = len(rows), len(rows[0].text)
    rng = random.Random(int(t * NOISE_HZ))

    def clear(r, c):
        """Off any text or line by a cell: a dot beside one reads as part of it."""
        return all(rows[m].blank(n) for m in (r - 1, r, r + 1) for n in (c - 1, c, c + 1)
                   if 0 <= m < height and 0 <= n < width)
    for _ in range(int(NOISE * width * height)):
        r, c = rng.randrange(height), rng.randrange(width)
        if clear(r, c):
            rows[r].put[c] = (chr(0x2800 + (1 << rng.randrange(8))), rng.choice(SNOW))
    # The sweep: faint braille at the beam, thinning and dimming behind it.
    beam = (t / SWEEP_S) % 1.0 * (height + DECAY_ROWS)
    for r in range(max(0, int(beam - DECAY_ROWS)), min(height, int(beam) + 1)):
        level = 1.0 - (beam - r) / DECAY_ROWS
        if level <= 0.0:
            continue
        ink = BAND_INK[min(len(BAND_INK) - 1, int(level * len(BAND_INK)))]
        for _ in range(int(BAND_DENSITY * level * level * width)):
            c = rng.randrange(width)
            if clear(r, c):
                rows[r].put[c] = (chr(0x2800 + (1 << rng.randrange(8))), ink)
