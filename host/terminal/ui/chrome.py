"""The house HUD: the attitude view's look on every page.

Over the whole screen (`Crt`, every frame `stage.curtain` shows): an old CRT's snow in
its blank cells, barely there - NOISE of them lit a frame, a dot each in SNOW's
near-black inks, redrawn NOISE_HZ times a second; its refresh sweep, a beam going down
over SWEEP_S seconds and decaying behind it over DECAY_ROWS as a phosphor does, in faint
braille only: BAND_DENSITY of a row's cells a dot at the beam, thinning as the square of
the decay, its ink stepping down BAND_INK to the snow's. No background: a glow a row at a
time banded.
Over a page's drawing (`Chrome`): lock brackets round what the page draws, `\uff9b\uff6f\uff78 \uff75\uff9d` blinking on them; the page's clock bottom left; its status tag in red
kana, blinking, over a teal subtag bottom right. Chrome only fills blanks: nothing the
page draws is covered, and a piece with no room is left out whole. KANA names each page.
"""
import functools
import random
import time

from rich.cells import cell_len
from rich.color import Color
from rich.segment import Segment
from rich.style import Style

from coaxial.graphics.approach import AMBER, DATA, RED

#: Page title: (its name in kana, the status tag, the subtag).
KANA = {
    'COAXIAL 63100': ('\uff7a\uff71\uff77\uff7c\uff6c\uff99', '>> \uff7c\uff7d\uff83\uff91 \uff80\uff72\uff77', '\uff7c\uff6d\uff80\uff70\uff90\uff85\uff99 01'),
    'SESSION': ('\uff7e\uff6f\uff7c\uff6e\uff9d', '>> \uff7c\uff7d\uff83\uff91 \uff76\uff9d\uff7c', '\uff98\uff78\uff9e \uff72\uff81\uff97\uff9d 6'),
    'BOARD ATTITUDE': ('\uff77\uff8a\uff9e\uff9d \uff7c\uff7e\uff72', '>> \uff81\uff6c\uff78\uff98\uff78\uff81 \uff80\uff9d\uff7b\uff78\uff81\uff6d\uff73', '\uff81\uff8a\uff9e \uff7d\uff8c\uff9f\uff97\uff73\uff99 \uff85\uff8b\uff9e 7G'),
    'SHAFT ANGLE': ('\uff7c\uff9e\uff78 \uff76\uff78\uff84\uff9e', '>> \uff76\uff78\uff84\uff9e \uff82\uff72\uff7e\uff77', 'A1335 \uff7c\uff9e\uff77 \uff7e\uff9d\uff7b'),
    'METER BRIDGE': ('\uff79\uff72\uff77 \uff8c\uff9e\uff98\uff6f\uff7c\uff9e', '>> \uff79\uff72\uff7f\uff78\uff81\uff6d\uff73', '\uff81\uff6c\uff9d\uff88\uff99 10 \uff76\uff9d\uff7c'),
    'CAPTURE': ('\uff77\uff6c\uff8c\uff9f\uff81\uff6c', '>> \uff7c\uff6d\uff73\uff9b\uff78\uff81\uff6d\uff73', '\uff98\uff9d\uff78\uff9e \uff8a\uff9e\uff6f\uff8c\uff67 2'),
    'GATE DRIVERS': ('\uff79\uff9e\uff70\uff84 \uff84\uff9e\uff97\uff72\uff8a\uff9e', '>> \uff79\uff9e\uff70\uff84 \uff78\uff84\uff9e\uff73', 'TIM1 50kHz \uff81\uff6d\uff73\uff7c\uff9d'),
    'ROTOR OBSERVER': ('\uff9b\uff70\uff80 \uff76\uff9d\uff7f\uff78', '>> \uff9b\uff70\uff80 \uff82\uff72\uff7e\uff77', '\uff84\uff9e\uff97\uff72\uff8c\uff9e 10 \uff7d\uff72\uff83\uff72'),
    'THERMAL OBSERVER': ('\uff88\uff82 \uff76\uff9d\uff7f\uff78', '>> \uff88\uff82 \uff7d\uff72\uff83\uff72\uff81\uff6d\uff73', '\uff75\uff8c\uff9e\uff7b\uff9e\uff70\uff8a\uff9e 8 \uff7a\uff9e\uff7c\uff9e\uff79\uff9e\uff9d'),
    'MOTOR CONTROLLER': ('\uff93\uff70\uff80 \uff7e\uff72\uff77\uff9e\uff6e', '>> \uff78\uff84\uff9e\uff73 \uff7c\uff9e\uff6d\uff9d\uff8b\uff9e', '\uff84\uff9e\uff97\uff72\uff8c\uff9e 10'),
    'BOARD CHAT': ('\uff82\uff73\uff7c\uff9d', '>> \uff82\uff73\uff7c\uff9d\uff81\uff6d\uff73', '\uff9b\uff70\uff76\uff99 \uff93\uff83\uff9e\uff99 \uff7e\uff82\uff7f\uff9e\uff78'),
    'RENDER': ('\uff74\uff9d\uff7c\uff9e\uff9d', '>> \uff74\uff9d\uff7c\uff9e\uff9d \uff7c\uff79\uff9d', '\uff77\uff6d\uff70\uff8c\uff9e \uff83\uff7d\uff84\uff8d\uff9e\uff6f\uff84\uff9e'),
    'HUMANOID': ('\uff8b\uff6d\uff70\uff8f\uff89\uff72\uff84\uff9e', '>> \uff8e\uff7a\uff73\uff81\uff6d\uff73', '\uff7c\uff9e\uff6d\uff73\uff98\uff6e\uff78 9.81'),
    'QUAD ACRO FLYING': ('\uff78\uff71\uff6f\uff84\uff9e \uff71\uff78\uff9b', '>> \uff8b\uff7a\uff73\uff81\uff6d\uff73', '\uff9b\uff70\uff80 4 \uff7e\uff9e\uff9d\uff76\uff72'),
    'HELICOPTER 3D FLYING': ('\uff8d\uff98\uff7a\uff8c\uff9f\uff80 3D', '>> \uff8b\uff7a\uff73\uff81\uff6d\uff73', '\uff9b\uff70\uff80 2 3D'),
    'ORNITHOPTER DUNE FLYING': ('\uff75\uff70\uff86\uff7f\uff8c\uff9f\uff80', '>> \uff8a\uff8a\uff9e\uff80\uff77 \uff8b\uff7a\uff73', '\uff96\uff78 4 \uff7b\uff8a\uff9e\uff78'),
    'AIRCRAFT XE AEROBATICS': ('\uff74\uff71\uff9b\uff8a\uff9e\uff83\uff68\uff78\uff7d', '>> \uff77\uff6e\uff78\uff77\uff9e \uff8b\uff7a\uff73', '3D \uff84\uff99\uff78\uff9b\uff70\uff99'),
    'SYSTEM ATLAS': ('\uff7c\uff7d\uff83\uff91 \uff71\uff84\uff97\uff7d', '>> \uff81\uff7d\uff9e \uff83\uff9d\uff76\uff72', '\uff80\uff70\uff90\uff85\uff99 \uff76\uff97 \uff7c\uff6c\uff9d\uff84'),
    'QUIT': ('\uff7c\uff6d\uff73\uff98\uff6e\uff73', '>> \uff7c\uff6d\uff73\uff98\uff6e\uff73', '\uff80\uff70\uff90\uff85\uff99 \uff8d\uff72\uff7b'),
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
LOCKED = '\uff9b\uff6f\uff78 \uff75\uff9d'

_BORN = time.monotonic()


def clock(now=None):
    """'T+mm:ss.s': the page's time since it opened."""
    minutes, seconds = divmod((time.monotonic() if now is None else now) - _BORN, 60.0)
    return 'T+%02d:%04.1f' % (minutes % 100, seconds)


#: A blank cell's characters: a space, an empty braille cell.
BLANK = ' \u2800'

#: A wide character's second cell in a row's cells.
WIDE = '\0'
_FREE3 = b'\x01\x01\x01'


@functools.lru_cache(maxsize=4096)
def _seg_width(text):
    return len(text) if text.isascii() else cell_len(text)


class _Row:

    """A rendered line as rich gave it - its segments, control codes left out -, its cells' chars
    (`text`, a wide one's second WIDE) and backgrounds (`bg`) for the blank tests, and the cells
    `put` over it, {cell: (char, style)}."""

    __slots__ = ('segs', 'text', 'bg', 'put')

    def __init__(self, line):
        self.segs, self.put = [seg for seg in line if not seg.control], {}
        widths = [_seg_width(seg.text) for seg in self.segs]
        if all(w == len(seg.text) for w, seg in zip(widths, self.segs)):
            self.text = ''.join(seg.text for seg in self.segs)
            self.bg, at = bytearray(len(self.text)), 0
            for seg in self.segs:
                end = at + len(seg.text)
                if seg.style is not None and seg.style.bgcolor is not None:
                    self.bg[at:end] = b'\x01' * (end - at)
                at = end
            return
        chars, self.bg = [], bytearray()
        for w, seg in zip(widths, self.segs):
            text, style = seg.text, seg.style
            back = b'\x01' if style is not None and style.bgcolor is not None else b'\x00'
            if w == len(text):
                chars.append(text)
                self.bg += back * len(text)
                continue
            for ch in text:
                wide = _seg_width(ch) == 2
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
        """The row's segments, the cells put over it in their places."""
        if not self.put:
            yield from self.segs
            return
        puts, k, at = sorted(self.put), 0, 0
        for seg in self.segs:
            text, style = seg.text, seg.style
            width = _seg_width(text)
            here = []
            while k < len(puts) and puts[k] < at + width:
                here.append(puts[k])
                k += 1
            if not here:
                yield seg
                at += width
                continue
            cells = list(text) if width == len(text) else [
                piece for ch in text for piece in ((ch, '') if _seg_width(ch) == 2 else (ch,))]
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
        corners = ((top, left, '\u250c\u2500'), (top, right, '\u2500\u2510'), (bottom, left, '\u2514\u2500'),
                   (bottom, right, '\u2500\u2518'))
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

    def lines(self, console, options):
        """The CRT's rows' Segments, each the region's width and as many as its height (`render_lines` padded them):
        what a screen shows from its top as they are (`stage._Tube`), nothing to split or measure again."""
        # A screen-mode Live gives no height: the screen's, as a Layout takes it. Passed
        # through on None, there was no CRT at all (42f9432).
        options = options.update(height=options.height or console.height)
        rows = [_Row(line) for line in console.render_lines(self.inner, options, pad=True)]
        if rows and rows[0].text:
            _crt(rows, time.monotonic())
        return [list(row.segments()) for row in rows]

    def __rich_console__(self, console, options):
        newline = Segment.line()
        for line in self.lines(console, options):
            yield from line
            yield newline


def _crt(rows, t):
    height, width = len(rows), len(rows[0].text)
    stride = width + 2
    free = bytearray(b'\x01' * (stride * (height + 2)))
    for r, row in enumerate(rows):
        base = (r + 1) * stride + 1
        txt, bg, put = row.text, row.bg, row.put
        for c in range(width):
            cell = put.get(c)
            is_blank = (txt[c] in BLANK and not bg[c]) if cell is None else (
                cell[0] in BLANK and (cell[1] is None or cell[1].bgcolor is None))
            if not is_blank:
                free[base + c] = 0
    rng = random.Random(int(t * NOISE_HZ))

    def clear(r, c):
        """Off any text or line by a cell: a dot beside one reads as part of it."""
        o = (r + 1) * stride + c
        return (free[o - stride:o - stride + 3] == _FREE3
                and free[o:o + 3] == _FREE3
                and free[o + stride:o + stride + 3] == _FREE3)
    for _ in range(int(NOISE * width * height)):
        r, c = rng.randrange(height), rng.randrange(width)
        if clear(r, c):
            rows[r].put[c] = (chr(0x2800 + (1 << rng.randrange(8))), rng.choice(SNOW))
            free[(r + 1) * stride + c + 1] = 0
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
                free[(r + 1) * stride + c + 1] = 0
