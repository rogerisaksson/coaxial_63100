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

The drawing is `coaxial.ansi.image` - the same one a notebook shows
inline - and this is the command line over it: Consolas for text and
Segoe UI Symbol for braille (Consolas has none), one cell 10 x 20 px on
black by default, which is near the bench's terminal. Pillow draws it,
and arrives with matplotlib.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..'))

from coaxial import ansi  # noqa: E402


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print((__doc__ or '').split('\n\n')[1].strip())
        return 2
    src, out = argv[0], argv[1]
    cell = (int(argv[2]) if len(argv) > 2 else ansi.CELL[0],
            int(argv[3]) if len(argv) > 3 else ansi.CELL[1])
    with open(src, encoding='utf-8') as f:
        size = ansi.png(f.read(), out, cell)
    print('wrote %s %dx%d' % (out, size[0], size[1]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
