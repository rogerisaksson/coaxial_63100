"""Colour, kept in one place so the renderers stay drawings."""
import re
import sys
import warnings
from contextlib import suppress


def utf8_stdout():
    """Make stdout carry the glyphs in this module."""
    reconfigure = getattr(sys.stdout, 'reconfigure', None)
    if reconfigure is None:                # not a reconfigurable stream
        return
    with suppress(ValueError):
        reconfigure(encoding='utf-8', errors='replace')
RESET = '\033[0m'

#: xterm-256 greyscale, black to white. 24 steps, and the ramp characters ride
#: on top of them: a cell is a glyph AND a grey, which is twice the levels
#: either could carry alone.
GREY_FIRST = 232
GREY_STEPS = 24

#: The meter's zones, after the lamp bridge they are drawn from: green below
#: -10 dBFS, amber above it, red at the rail. Numbers are xterm-256.
GREEN = 46
AMBER = 214
RED = 196
#: The cold end of a three-band scale - under normal, where green is
#: normal and red is past it. Brighter than the thermal ramp's blues,
#: which are a field's floor and not a band to read at a glance.
BLUE = 33
DIM = 236
WHITE = 231

# The views' motif - terminal/ui/screen.py names the roles (NEON names things, SODIUM
# is what matters now, ASH is the street).
TEAL = 44
ASH = 242


def code(number):
    """One foreground escape: a palette number, or an (r, g, b) tuple as
    24-bit colour - the glow ramp interpolates between palette steps,
    and a 256-colour cyan has only five rungs between black and full."""
    if isinstance(number, tuple):
        return '\033[38;2;%d;%d;%dm' % number
    return '\033[38;5;%dm' % number


def back(number):
    """One background escape: a palette number, or an (r, g, b) tuple as
    24-bit colour, as `code` takes them."""
    if isinstance(number, tuple):
        return '\033[48;2;%d;%d;%dm' % number
    return '\033[48;5;%dm' % number


#: xterm-256: the sixteen named colours, then a 6 x 6 x 6 cube whose
#: channel levels are 0 and 55 + 40 n, then a grey ramp from 8 in tens.
CUBE_START = 16
CUBE = 6
CUBE_LEVEL_BASE, CUBE_LEVEL_STEP = 55, 40
GREY_START = 232
GREY_BASE, GREY_STEP = 8, 10
#: SGR: the colour parameters this codec reads.
SGR_FG, SGR_FG_BRIGHT, SGR_BG = range(30, 38), range(90, 98), range(40, 48)
SGR_FG_EXTENDED, SGR_BG_EXTENDED = 38, 48
SGR_FG_DEFAULT, SGR_BG_DEFAULT = 39, 49
BRIGHT = 8       #: the bright eight follow the plain eight in the palette


def rgb(number):
    """An xterm-256 cube or grey-ramp colour as (r, g, b)."""
    if number >= GREY_START:
        grey = GREY_BASE + GREY_STEP * (number - GREY_START)
        return (grey, grey, grey)
    c = number - CUBE_START
    return tuple(0 if v == 0 else CUBE_LEVEL_BASE + CUBE_LEVEL_STEP * v
                 for v in (c // (CUBE * CUBE), (c // CUBE) % CUBE, c % CUBE))


#: Lower half block. A cell drawn with it carries two colours - foreground
#: below, background above - so one character row holds two picture rows.
#: Taken from esimov/ascii-fluid, which uses the same glyph for the same
#: reason: a terminal cell is twice as tall as it is wide, and this is how
#: you get the other half back.
HALF = '▄'

#: The other half, for a cell whose LOWER row is off the picture. Without
#: it that case was drawn as a background-coloured space - a whole cell
#: painted where half was meant - so the bottom edge of a round board
#: resolved a cell coarser than the top and read as blocky.
HALF_UP = '▀'


def paint(text, number):
    """`text` in one colour, closed again."""
    return code(number) + text + RESET


def grey(fraction):
    """The grey for a brightness in 0..1."""
    step = int(max(0.0, min(1.0, fraction)) * (GREY_STEPS - 1))
    return GREY_FIRST + step


#: Dark solder mask to lit copper, in the xterm-256 cube. A board drawn in
#: greys is a grey disc; a board drawn in its own colour is recognisably a
#: board, and the components read as the bright end because they are what
#: catches the light. Not decoration - it is the one cue that survives at a
#: cell per two millimetres.
BOARD_RAMP = (22, 22, 28, 34, 40, 46, 47, 83, 119, 155, 191, 227, 231)


#: Colour stops by DEGREES, not by fraction of a span: auto-ranging made a
#: cool board look exactly like a hot one. A thermal camera's ironbow
#: (2026-09-25; the rainbow before it read as mossy): blue at the floor - a
#: colour, not black, "bottom-frozen" - violet where the board idles, magenta,
#: red from 50, orange, yellow at 80, white-hot at 100. 5 K apart through
#: 25-60, where this board works: coarser stops hid the hot swap.
THERMAL_STOPS = (
    (-20.0, (0, 0, 160)),
    (0.0, (30, 0, 175)),
    (15.0, (70, 0, 180)),
    (25.0, (110, 0, 176)),
    (30.0, (145, 0, 162)),
    (35.0, (175, 10, 140)),
    (40.0, (200, 22, 112)),
    (45.0, (220, 36, 82)),
    (50.0, (235, 56, 52)),
    (55.0, (245, 80, 30)),
    (60.0, (250, 106, 14)),
    (65.0, (255, 130, 4)),
    (70.0, (255, 155, 0)),
    (75.0, (255, 180, 0)),
    (80.0, (255, 205, 20)),
    (85.0, (255, 225, 62)),
    (90.0, (255, 240, 122)),
    (95.0, (255, 250, 186)),
    (100.0, (255, 255, 240)),
)

#: The ends of the scale. What a picture is drawn against, whatever is in it.
THERMAL_MIN = THERMAL_STOPS[0][0]
THERMAL_MAX = THERMAL_STOPS[-1][0]


def thermal(celsius):
    """The colour for an absolute temperature, from THERMAL_STOPS."""
    if celsius <= THERMAL_STOPS[0][0]:
        return THERMAL_STOPS[0][1]
    for (lo_c, lo_n), (hi_c, hi_n) in zip(THERMAL_STOPS, THERMAL_STOPS[1:]):
        if celsius <= hi_c:
            # Nearest stop rather than a blend: a band readable as a band.
            half = (lo_c + hi_c) / 2.0
            return lo_n if celsius < half else hi_n
    return THERMAL_STOPS[-1][1]


def thermal_rgb(celsius):
    """The ramp's colour BETWEEN its stops, as (r, g, b): linear from one
    stop's colour to the next, so a field that varies by a kelvin a cell
    is a gradient and not a staircase of bands.
    """
    if celsius <= THERMAL_STOPS[0][0]:
        return THERMAL_STOPS[0][1]
    for (lo_c, a), (hi_c, b) in zip(THERMAL_STOPS, THERMAL_STOPS[1:]):
        if celsius <= hi_c:
            f = (celsius - lo_c) / (hi_c - lo_c)
            return (int(a[0] + (b[0] - a[0]) * f + 0.5),
                    int(a[1] + (b[1] - a[1]) * f + 0.5),
                    int(a[2] + (b[2] - a[2]) * f + 0.5))
    return THERMAL_STOPS[-1][1]


def board(fraction):
    """The board's colour for a brightness in 0..1."""
    step = int(max(0.0, min(1.0, fraction)) * (len(BOARD_RAMP) - 1))
    return BOARD_RAMP[step]


def run(cells):
    """Cells of (text, colour) as one line, changing colour only where it
    changes.
    """
    out = []
    current = None

    for text, number in cells:
        if number is not None and number != current:
            out.append(code(number))
            current = number
        out.append(text)

    if current is not None:
        out.append(RESET)
    return ''.join(out)


# ------------------------------------------------------------- pictures A
# frame drawn for the terminal, as the terminal would show it: the colour
# braille the pages draw, in a notebook or a file.

#: The sixteen system colours as most terminals draw them; the cube and
#: the grey ramp are `rgb`.
SYSTEM = ((0, 0, 0), (205, 0, 0), (0, 205, 0), (205, 205, 0),
          (0, 0, 238), (205, 0, 205), (0, 205, 205), (229, 229, 229),
          (127, 127, 127), (255, 0, 0), (0, 255, 0), (255, 255, 0),
          (92, 92, 255), (255, 0, 255), (0, 255, 255), (255, 255, 255))
#: The default foreground: a terminal's light grey.
PLAIN = (204, 204, 204)
#: One cell in pixels, near the bench's terminal.
CELL = (10, 20)
#: The faces, where the bench's Windows keeps them.
TEXT_FONT = 'C:/Windows/Fonts/consola.ttf'
BRAILLE_FONT = 'C:/Windows/Fonts/seguisym.ttf'
#: Half-width kana (U+FF61..FF9F): the terminal falls back to MS Gothic for them.
KANA_FONT = 'C:/Windows/Fonts/msgothic.ttc'
KANA = (0xFF61, 0xFF9F)
#: Characters from here up are drawn with the braille face: the box
#: drawing, blocks and braille ranges, none of which Consolas carries
#: in the widths the pages assume.
SYMBOL_FROM = 0x2500

_SGR = re.compile(r'\x1b\[([0-9;]*)m')
_OTHER = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\x1b\][^\x07]*\x07'
                    r'|\x1b[()][A-Za-z0-9]')


def _colour(n):
    """An xterm-256 index as (r, g, b)."""
    return SYSTEM[n] if n < 16 else rgb(n)


def _step(codes, j, fg, bg):
    """One SGR parameter at `codes[j]` applied: the pair after it, and
    how many parameters it took - three for `38;5;n`, five for
    `38;2;r;g;b`, one for the rest."""
    c = codes[j]
    if c == 0:
        return PLAIN, None, 1
    extended = (SGR_FG_EXTENDED, SGR_BG_EXTENDED)
    if c in extended and j + 2 < len(codes) and codes[j + 1] == 5:
        colour = _colour(codes[j + 2])
        return (colour, bg, 3) if c == SGR_FG_EXTENDED else (fg, colour, 3)
    if c in extended and j + 4 < len(codes) and codes[j + 1] == 2:
        colour = tuple(codes[j + 2:j + 5])
        return (colour, bg, 5) if c == SGR_FG_EXTENDED else (fg, colour, 5)
    if c in SGR_FG:
        return _colour(c - SGR_FG.start), bg, 1
    if c in SGR_FG_BRIGHT:
        return _colour(c - SGR_FG_BRIGHT.start + BRIGHT), bg, 1
    if c in SGR_BG:
        return fg, _colour(c - SGR_BG.start), 1
    if c == SGR_FG_DEFAULT:
        return PLAIN, bg, 1
    if c == SGR_BG_DEFAULT:
        return fg, None, 1
    return fg, bg, 1


def parse(text):
    """Rows of (char, fg, bg) from ANSI text: what each cell shows and in
    which colours.
    """
    text = _OTHER.sub(lambda m: m.group(0) if _SGR.fullmatch(m.group(0))
                      else '', text)
    rows, row = [], []
    fg, bg = PLAIN, None
    i = 0
    while i < len(text):
        m = _SGR.match(text, i)
        if m:
            codes = [int(c) for c in m.group(1).split(';') if c != ''] or [0]
            j = 0
            while j < len(codes):
                fg, bg, took = _step(codes, j, fg, bg)
                j += took
            i = m.end()
            continue
        ch = text[i]
        if ch == '\n':
            rows.append(row)
            row = []
        elif ch != '\r':
            row.append((ch, fg, bg))
        i += 1
    if row:
        rows.append(row)
    return rows


_SAID = set()


def _font(path, size):
    from PIL import ImageFont
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        if path not in _SAID:
            _SAID.add(path)
            warnings.warn('no %s - Pillow\'s own face stands in, and the '
                          'braille seam it shows is not the bench\'s' % path)
        return ImageFont.load_default()


def image(text, cell=CELL, fonts=(TEXT_FONT, BRAILLE_FONT)):
    """`text` drawn cell by cell in its colours on black, as a Pillow image
    - a notebook shows it inline as the value of a cell, or under
    `IPython.display.display`.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        raise ImportError('machine.ansi.image needs Pillow, which '
                          'matplotlib brings: pip install matplotlib')
    rows = parse(text)
    cell_w, cell_h = cell
    width = max((len(r) for r in rows), default=1)
    img = Image.new('RGB', (width * cell_w, max(1, len(rows)) * cell_h),
                    (0, 0, 0))
    draw = ImageDraw.Draw(img)
    text_font = _font(fonts[0], int(cell_h * 0.8))
    symbol_font = _font(fonts[1], int(cell_h * 0.8))
    kana_font = _font(KANA_FONT, int(cell_h * 0.8))
    for y, row in enumerate(rows):
        for x, (c, fg, bg) in enumerate(row):
            if bg:
                draw.rectangle([x * cell_w, y * cell_h,
                                (x + 1) * cell_w - 1, (y + 1) * cell_h - 1],
                               fill=bg)
            if c == ' ':
                continue
            font = (kana_font if KANA[0] <= ord(c) <= KANA[1] else
                    symbol_font if ord(c) >= SYMBOL_FROM else text_font)
            draw.text((x * cell_w, y * cell_h), c, fill=fg, font=font)
    return img


def png(text, path, cell=CELL):
    """`text` drawn as `image` draws it, saved as the PNG at `path`; the
    image size in pixels."""
    img = image(text, cell)
    img.save(path)
    return img.size
