"""Rasterise an ANSI-coloured text frame to a PNG, the way a terminal
would draw it - so a braille picture can be JUDGED before it lands.

    python tools/ansi2png.py frame.txt frame.png [cell_w cell_h]

A PICTURE IS JUDGED IN A RASTER, NOT IN GLYPH COUNTS. Three attitude
renders that passed every check in `test_render` were "blocky as hell"
on the bench, and the cause each time was only visible with the cells
drawn: a carpet of one rung, Bayer's two-by-two clusters, a font whose
braille box is narrower than its cell. Two choices in the thermal map -
a two-dot edge, a half-density floor - were reversed on the raster
before the bench saw them. The routine in CLAUDE.md names this tool.

Consolas for text and Segoe UI Symbol for braille (Consolas has none),
one cell 10 x 20 px on black by default - which is near the bench's
terminal, and the fallback-font seam that makes dense braille read as
bricks is reproduced, which is the point. Pillow draws it; it is not in
`requirements.txt`, because it is a compiled wheel and this is a
judging tool, not the host - `pip install pillow` when asked.
"""
import re
import sys

#: The xterm-256 cube's six levels.
CUBE = [0, 95, 135, 175, 215, 255]

#: The sixteen system colours, as most terminals draw them.
SYSTEM = [(0, 0, 0), (205, 0, 0), (0, 205, 0), (205, 205, 0),
          (0, 0, 238), (205, 0, 205), (0, 205, 205), (229, 229, 229),
          (127, 127, 127), (255, 0, 0), (0, 255, 0), (255, 255, 0),
          (92, 92, 255), (255, 0, 255), (0, 255, 255), (255, 255, 255)]

#: The default foreground: a terminal's light grey.
PLAIN = (204, 204, 204)

#: Where the fonts are on the bench's Windows; the default PIL face
#: stands in elsewhere, and says so.
TEXT_FONT = 'C:/Windows/Fonts/consola.ttf'
BRAILLE_FONT = 'C:/Windows/Fonts/seguisym.ttf'


def xterm(n):
    """An xterm-256 index as (r, g, b)."""
    if n < 16:
        return SYSTEM[n]
    if n >= 232:
        g = 8 + 10 * (n - 232)
        return (g, g, g)
    c = n - 16
    return (CUBE[c // 36], CUBE[(c // 6) % 6], CUBE[c % 6])


SGR = re.compile(r'\x1b\[([0-9;]*)m')
OTHER = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]|\x1b\][^\x07]*\x07'
                   r'|\x1b[()][A-Za-z0-9]')


def _step(codes, j, fg, bg):
    """One SGR parameter at `codes[j]` applied: the pair after it, and
    how many parameters it took - three for `38;5;n`, five for
    `38;2;r;g;b`, one for the rest."""
    c = codes[j]
    if c == 0:
        return PLAIN, None, 1
    if c in (38, 48) and j + 2 < len(codes) and codes[j + 1] == 5:
        colour = xterm(codes[j + 2])
        return (colour, bg, 3) if c == 38 else (fg, colour, 3)
    if c in (38, 48) and j + 4 < len(codes) and codes[j + 1] == 2:
        colour = tuple(codes[j + 2:j + 5])
        return (colour, bg, 5) if c == 38 else (fg, colour, 5)
    if 30 <= c <= 37 or 90 <= c <= 97:
        return xterm(c - 30 if c < 90 else c - 82), bg, 1
    if 40 <= c <= 47:
        return fg, xterm(c - 40), 1
    if c == 39:
        return PLAIN, bg, 1
    if c == 49:
        return fg, None, 1
    return fg, bg, 1


def _sgr(codes, fg, bg):
    """One SGR sequence's colours applied to the current pair."""
    j = 0
    while j < len(codes):
        fg, bg, took = _step(codes, j, fg, bg)
        j += took
    return fg, bg


def parse(text):
    """Rows of (char, fg, bg) from ANSI text. Cursor movement, OSC
    titles and charset selections are dropped; only colour survives."""
    text = OTHER.sub(lambda m: m.group(0) if SGR.fullmatch(m.group(0))
                     else '', text)
    rows, row = [], []
    fg, bg = PLAIN, None
    i = 0
    while i < len(text):
        m = SGR.match(text, i)
        if m:
            codes = [int(c) for c in m.group(1).split(';') if c != ''] or [0]
            fg, bg = _sgr(codes, fg, bg)
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


def _font(path, size):
    from PIL import ImageFont
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        print('  no %s - the default face stands in, and the braille '
              'seam it shows is not the bench\'s' % path)
        return ImageFont.load_default()


def raster(text, out, cell_w=10, cell_h=20):
    """`text` drawn cell by cell to the PNG at `out`; the image size."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        raise SystemExit('ansi2png needs Pillow: pip install pillow')
    rows = parse(text)
    width = max((len(r) for r in rows), default=1)
    img = Image.new('RGB', (width * cell_w, max(1, len(rows)) * cell_h),
                    (0, 0, 0))
    draw = ImageDraw.Draw(img)
    text_font = _font(TEXT_FONT, int(cell_h * 0.8))
    braille_font = _font(BRAILLE_FONT, int(cell_h * 0.8))
    for y, row in enumerate(rows):
        for x, (c, fg, bg) in enumerate(row):
            if bg:
                draw.rectangle([x * cell_w, y * cell_h,
                                (x + 1) * cell_w - 1, (y + 1) * cell_h - 1],
                               fill=bg)
            if c == ' ':
                continue
            font = braille_font if ord(c) > 0x2500 else text_font
            draw.text((x * cell_w, y * cell_h), c, fill=fg, font=font)
    img.save(out)
    return img.size


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__.split('\n\n')[1].strip())
        return 2
    src, out = argv[0], argv[1]
    cell_w = int(argv[2]) if len(argv) > 2 else 10
    cell_h = int(argv[3]) if len(argv) > 3 else 20
    with open(src, encoding='utf-8') as f:
        size = raster(f.read(), out, cell_w, cell_h)
    print('wrote %s %dx%d' % (out, size[0], size[1]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
