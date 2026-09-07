"""Colour, kept in one place so the renderers stay drawings.

Every renderer here takes a `colour` flag and defaults it to off. That is not
timidity: a picture with escape sequences in it cannot be compared to an
expected string, and the suites compare these pictures. Colour is added at
the edge, by the tool that knows whether it is writing to a terminal, and the
tests see the same characters either way.

256-colour rather than the 16 basic ones. The greys are what make a shaded
surface read as shaded - eight levels of grey against a ramp of ten
characters is most of the picture's information, and the basic set has one.
"""
import re


def utf8_stdout():
    """Make stdout carry the glyphs in this module. Call it at the edge.

    A pipe takes its encoding from the locale, cp1252 here, and every
    ramp below is outside ASCII. Measured: `show_desk.py | anything`
    died with UnicodeEncodeError before the first frame, and
    thermal_model.py died the same way when redirected. `replace`, not
    `strict` - a log is worth reading with glyphs substituted and
    worthless as a traceback.

    Not done on import: a library that reconfigures the caller's
    stdout is a side effect nobody asked for, and this module already
    says colour is added at the edge.
    """
    import sys
    reconfigure = getattr(sys.stdout, 'reconfigure', None)
    if reconfigure is None:                # not a reconfigurable stream
        return
    try:
        reconfigure(encoding='utf-8', errors='replace')
    except ValueError:
        pass
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

# The views' motif - tools/screen.py names the roles (NEON names things,
# SODIUM is what matters now, ASH is the street). The numbers live here so
# renderers under coaxial/ can wear them without importing a tool.
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


def rgb(number):
    """An xterm-256 cube or grey-ramp colour as (r, g, b)."""
    if number >= 232:
        grey = 8 + 10 * (number - 232)
        return (grey, grey, grey)
    c = number - 16
    return tuple(0 if v == 0 else 55 + 40 * v
                 for v in (c // 36, (c // 6) % 6, c % 6))


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
#: cool board look exactly like a hot one. Spacing is deliberately uneven -
#: the board idles near 30 C, so 20-60 gets the resolution and red starts at
#: 90, where a laminate is in trouble rather than merely working.
THERMAL_STOPS = (
    (-20.0, 19),    # blue - 19, not 17: at four dots in ten the
                    # halftone's cold end was black on the bench, and a
                    # scale's floor should be a colour, "bottom-frozen"
    (0.0, 20),
    (15.0, 25),
    (25.0, 31),     # ambient, and where the resolution has to be fine:
    (30.0, 37),     # this board idles near 30 and works between 30 and 60,
    (35.0, 43),     # so the steps are 5 K apart through there. Coarser stops
    (40.0, 44),     # put a 6 K difference inside one colour and hid the hot
    (45.0, 49),     # swap entirely.
    (50.0, 50),
    (55.0, 79),
    (60.0, 83),     # green reaches here
    (65.0, 118),
    (70.0, 154),
    (75.0, 190),
    (80.0, 220),    # yellow
    (85.0, 214),
    (90.0, 208),    # orange - the warm end starts late on purpose
    (95.0, 202),
    (100.0, 196),   # red
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
            # Nearest stop rather than a blend: the 256-colour cube has no
            # useful intermediates between these, and rounding to one of the
            # two keeps a band readable as a band.
            half = (lo_c + hi_c) / 2.0
            return lo_n if celsius < half else hi_n
    return THERMAL_STOPS[-1][1]


def thermal_rgb(celsius):
    """The ramp's colour BETWEEN its stops, as (r, g, b): linear from one
    stop's colour to the next, so a field that varies by a kelvin a cell
    is a gradient and not a staircase of bands. `thermal` above stays
    the banded one for anything that wants a band to read as a band.
    """
    if celsius <= THERMAL_STOPS[0][0]:
        return rgb(THERMAL_STOPS[0][1])
    for (lo_c, lo_n), (hi_c, hi_n) in zip(THERMAL_STOPS, THERMAL_STOPS[1:]):
        if celsius <= hi_c:
            f = (celsius - lo_c) / (hi_c - lo_c)
            a, b = rgb(lo_n), rgb(hi_n)
            return (int(a[0] + (b[0] - a[0]) * f + 0.5),
                    int(a[1] + (b[1] - a[1]) * f + 0.5),
                    int(a[2] + (b[2] - a[2]) * f + 0.5))
    return rgb(THERMAL_STOPS[-1][1])


def board(fraction):
    """The board's colour for a brightness in 0..1."""
    step = int(max(0.0, min(1.0, fraction)) * (len(BOARD_RAMP) - 1))
    return BOARD_RAMP[step]


def run(cells):
    """Cells of (text, colour) as one line, changing colour only where it
    changes.

    One escape per run rather than per character: a 44-column picture is 44
    escapes a frame either way, but a board drawn in flat regions is three or
    four, and the difference is what a terminal repainting at 20 Hz notices.
    `colour` None means leave whatever is current alone.
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


# ------------------------------------------------------------- pictures
#
# A frame drawn for the terminal, as the terminal would show it: the
# colour braille the pages draw, in a notebook or a file. `image` is a
# Pillow image, which Jupyter displays inline on its own; `png` writes
# one. `tools/ansi2png.py` is the command line over the same drawing.
#
# A PICTURE IS JUDGED IN A RASTER, NOT IN GLYPH COUNTS (CLAUDE.md): the
# faces are the bench's terminal's, Consolas for text and Segoe UI
# Symbol for braille - Consolas has none - and the seam between them,
# which makes dense braille read as bricks, is reproduced on purpose.
# Where those faces are not installed Pillow's own stands in and the
# image says so once, since the seam it shows is not the bench's.

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
    if c in (38, 48) and j + 2 < len(codes) and codes[j + 1] == 5:
        colour = _colour(codes[j + 2])
        return (colour, bg, 3) if c == 38 else (fg, colour, 3)
    if c in (38, 48) and j + 4 < len(codes) and codes[j + 1] == 2:
        colour = tuple(codes[j + 2:j + 5])
        return (colour, bg, 5) if c == 38 else (fg, colour, 5)
    if 30 <= c <= 37 or 90 <= c <= 97:
        return _colour(c - 30 if c < 90 else c - 82), bg, 1
    if 40 <= c <= 47:
        return fg, _colour(c - 40), 1
    if c == 39:
        return PLAIN, bg, 1
    if c == 49:
        return fg, None, 1
    return fg, bg, 1


def parse(text):
    """Rows of (char, fg, bg) from ANSI text: what each cell shows and in
    which colours. Cursor movement, OSC titles and charset selections
    are dropped; only colour survives."""
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
            import warnings
            warnings.warn('no %s - Pillow\'s own face stands in, and the '
                          'braille seam it shows is not the bench\'s' % path)
        return ImageFont.load_default()


def image(text, cell=CELL, fonts=(TEXT_FONT, BRAILLE_FONT)):
    """`text` drawn cell by cell in its colours on black, as a Pillow
    image - a notebook shows it inline as the value of a cell, or under
    `IPython.display.display`.

        from coaxial import ansi, thermalmap
        ansi.image(thermalmap.render(nodes, board_c, cells=60, colour=True))

    Pillow arrives with matplotlib (requirements.txt); nothing else in
    the library needs it, so it is imported here and not above.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        raise ImportError('coaxial.ansi.image needs Pillow, which '
                          'matplotlib brings: pip install matplotlib')
    rows = parse(text)
    cell_w, cell_h = cell
    width = max((len(r) for r in rows), default=1)
    img = Image.new('RGB', (width * cell_w, max(1, len(rows)) * cell_h),
                    (0, 0, 0))
    draw = ImageDraw.Draw(img)
    text_font = _font(fonts[0], int(cell_h * 0.8))
    symbol_font = _font(fonts[1], int(cell_h * 0.8))
    for y, row in enumerate(rows):
        for x, (c, fg, bg) in enumerate(row):
            if bg:
                draw.rectangle([x * cell_w, y * cell_h,
                                (x + 1) * cell_w - 1, (y + 1) * cell_h - 1],
                               fill=bg)
            if c == ' ':
                continue
            font = symbol_font if ord(c) >= SYMBOL_FROM else text_font
            draw.text((x * cell_w, y * cell_h), c, fill=fg, font=font)
    return img


def png(text, path, cell=CELL):
    """`text` drawn as `image` draws it, saved as the PNG at `path`; the
    image size in pixels."""
    img = image(text, cell)
    img.save(path)
    return img.size
