#!/usr/bin/env python3
"""The exporter's LIT screenshots against our colour render, in numbers.

tests/renders/ascii-<pose>.png is a screenshot of the exporter's
render of ascii-<pose>.txt with its lighting on. Each fixture cell
maps onto the PNG (bounding boxes aligned) and yields the Rec.709
luma of its ink; our render's cells yield the luma of their GLOW
colour. Luma, not the peak channel: the exporter's cyan-blue and
the console's cyan differ in hue, and peak-channel matching landed
two tones bright. Per class and pose: mean, spread within the class,
and the spread between thirds of the picture - a gradient the
exporter does not have.

    python tools/tonecheck.py            # measure both, side by side
    python tools/tonecheck.py --fit      # tune the glow constants
                                         # toward the targets until
                                         # the error stops falling;
                                         # prints what to bake

The fit's objective: class mean and within-class spread at the two
photographed poses, a cap on the thirds spread (a soft lamp), and
the class means held still across five more rotations - a turn must
not swing the light.

Pure Python: the PNG reader is zlib and the five filters, the statistics
are means, deviations and percentiles over a few hundred cells.
"""
import argparse
import os
import re
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coaxial import engine, wireframe                      # noqa: E402
import facecheck                                           # noqa: E402

PHOTOS = ('x0y45z0', 'x45y45z45')
TURNS = ('x0y0z0', 'x0y15z0', 'x0y30z0', 'x0y60z0', 'x30y0z0')

#: Per class off the screenshots 2026-08-30: (mean, sd, mean |luma
#: step| to the right-hand neighbour of the same class). '.' 93-99 /
#: 24 / 21 - texture, its neighbour correlation only 0.22; ':' 128-130
#: / 18-23 / 8-10 - smooth blocks. Flat across the picture. The means
#: sit a shade under his: the room was asked darker.
TARGET = {'.': (90.0, 24.0, 21.0), ':': (125.0, 20.0, 9.0)}
SPREAD_CAP = 20.0


TOK = re.compile(r'\x1b\[([0-9;]*)m|(.)')


def xterm_rgb(code):
    if code < 16:
        return (0, 0, 0)
    if code < 232:
        c = code - 16
        return tuple(0 if v == 0 else 55 + 40 * v
                     for v in (c // 36, (c // 6) % 6, c % 6))
    grey = 8 + 10 * (code - 232)
    return (grey, grey, grey)


def luma(rgb):
    return 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]


def rotation(pose):
    got = re.match(r'x(-?\d+)y(-?\d+)z(-?\d+)$', pose)
    return tuple(int(v) for v in got.groups())


def png(path):
    """(width, height, rows) of an 8-bit RGB/RGBA PNG, each row a list
    of (r, g, b). Non-interlaced only - what a screenshot tool writes."""
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('%s is not a PNG' % path)
    at, width = 8, 0
    height = depth = kind = 0
    idat = []
    while at < len(data):
        size, tag = struct.unpack('>I4s', data[at:at + 8])
        body = data[at + 8:at + 8 + size]
        if tag == b'IHDR':
            width, height, depth, kind, _c, _f, lace = struct.unpack(
                '>IIBBBBB', body)
            if depth != 8 or kind not in (2, 6) or lace:
                raise ValueError('%s: only 8-bit RGB/RGBA, not interlaced'
                                 % path)
        elif tag == b'IDAT':
            idat.append(body)
        elif tag == b'IEND':
            break
        at += 12 + size
    raw = zlib.decompress(b''.join(idat))
    step = 3 if kind == 2 else 4
    stride = width * step
    rows, prior = [], bytearray(stride)
    for r in range(height):
        start = r * (stride + 1)
        kind_of, line = raw[start], bytearray(raw[start + 1:start + 1 + stride])
        for i in range(stride):
            a = line[i - step] if i >= step else 0
            b = prior[i]
            c = prior[i - step] if i >= step else 0
            if kind_of == 1:
                line[i] = (line[i] + a) & 255
            elif kind_of == 2:
                line[i] = (line[i] + b) & 255
            elif kind_of == 3:
                line[i] = (line[i] + (a + b) // 2) & 255
            elif kind_of == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[i] = (line[i] + (a if pa <= pb and pa <= pc
                                      else (b if pb <= pc else c))) & 255
        rows.append([(line[i], line[i + 1], line[i + 2])
                     for i in range(0, stride, step)])
        prior = line
    return width, height, rows


def exporter_cells(pose):
    """[(glyph, x01, y01, luma, row, col)] off the screenshot, one per
    ink cell - the fixture's grid mapped onto the image by their
    bounding boxes, each cell's ink the pixels near its own peak."""
    with open(os.path.join(facecheck.RENDERS, 'ascii-%s.txt' % pose),
              encoding='ascii') as f:
        rows = f.read().splitlines()
    wide = max(len(r) for r in rows)
    rows = [r.ljust(wide) for r in rows]
    width, height, img = png(os.path.join(facecheck.RENDERS,
                                          'ascii-%s.png' % pose))
    lit = [(x, y) for y in range(height) for x in range(width)
           if max(img[y][x]) > 24]
    px0, px1 = min(x for x, _ in lit), max(x for x, _ in lit) + 1
    py0, py1 = min(y for _, y in lit), max(y for _, y in lit) + 1
    filled = [(r, c) for r, line in enumerate(rows)
              for c, ch in enumerate(line) if ch != ' ']
    r0, r1 = min(r for r, _ in filled), max(r for r, _ in filled) + 1
    c0, c1 = min(c for _, c in filled), max(c for _, c in filled) + 1
    cw, chh = (px1 - px0) / (c1 - c0), (py1 - py0) / (r1 - r0)
    out = []
    for r, c in filled:
        x, y = px0 + (c - c0) * cw, py0 + (r - r0) * chh
        box = [img[yy][xx]
               for yy in range(int(y), min(height, int(y + chh) + 1))
               for xx in range(int(x), min(width, int(x + cw) + 1))]
        peak = max((max(p) for p in box), default=0)
        if peak <= 24:
            continue                    # the grid map missed the dot
        ink = [p for p in box if max(p) >= max(24, peak * 0.6)]
        mean = tuple(sum(p[k] for p in ink) / len(ink) for k in range(3))
        out.append((rows[r][c], (c - c0) / (c1 - c0), (r - r0) / (r1 - r0),
                    luma(mean), r, c))
    return out


def our_cells(pose, width=52, height=34):
    """The same tuple off our colour render."""
    wireframe._SHADOWS.clear()
    art = wireframe.render(facecheck.euler(*rotation(pose), order='xyz'),
                           width, height, colour=True, 
                           horizon=False, tip=0.0)
    got = []
    for py, line in enumerate(art.splitlines()):
        lum, col = None, 0
        for m in TOK.finditer(line):
            if m.group(2) is None:
                true = re.match(r'38;2;(\d+);(\d+);(\d+)$', m.group(1))
                pal = re.match(r'38;5;(\d+)$', m.group(1))
                if true:
                    lum = luma(tuple(int(v) for v in true.groups()))
                elif pal:
                    lum = luma(xterm_rgb(int(pal.group(1))))
                elif m.group(1) in ('0', ''):
                    lum = None
                continue
            if m.group(2) in '.:' and lum is not None:
                got.append((py, col, m.group(2), lum))
            col += 1
    xs, ys = [g[1] for g in got], [g[0] for g in got]
    x0, x1, y0, y1 = min(xs), max(xs) + 1, min(ys), max(ys) + 1
    return [(ch, (col - x0) / (x1 - x0), (py - y0) / (y1 - y0), lum, py, col)
            for py, col, ch, lum in got]


def mean(values):
    return sum(values) / len(values)


def deviation(values):
    m = mean(values)
    return (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5


def stats(cells):
    """{glyph: (mean, sd, thirds spread)} over the ink cells."""
    out = {}
    for glyph in '.:':
        sel = [c for c in cells if c[0] == glyph]
        if len(sel) < 8:
            continue
        lum = [c[3] for c in sel]
        thirds = []
        for ry in range(3):
            for rx in range(3):
                pick = [c[3] for c in sel
                        if ry / 3 <= c[2] < (ry + 1) / 3
                        and rx / 3 <= c[1] < (rx + 1) / 3]
                if len(pick) >= 4:
                    thirds.append(mean(pick))
        out[glyph] = (mean(lum), deviation(lum),
                      max(thirds) - min(thirds) if thirds else 0.0)
    return out


def steps(cells):
    """Share of horizontally adjacent same-class cells more than 25 luma
    apart - the hard iso-lines a reader sees as banding - and the mean
    step between such neighbours."""
    grid = {(c[4], c[5]): (c[0], c[3]) for c in cells}
    jumps = [abs(nb[1] - lum) for (r, col), (ch, lum) in grid.items()
             for nb in (grid.get((r, col + 1)),) if nb and nb[0] == ch]
    if not jumps:
        return 0.0, 0.0
    return (100.0 * sum(1 for j in jumps if j > 25) / len(jumps),
            mean(jumps))


def neighbours(cells):
    """{glyph: mean |luma step| to the right-hand same-class neighbour}
    - the per-dot texture the exporter has and a flat wash lacks."""
    grid = {(c[4], c[5]): (c[0], c[3]) for c in cells}
    out = {}
    for glyph in '.:':
        d = [abs(nb[1] - lum) for (r, col), (ch, lum) in grid.items()
             for nb in (grid.get((r, col + 1)),)
             if ch == glyph and nb and nb[0] == glyph]
        if len(d) >= 8:
            out[glyph] = mean(d)
    return out


def objective():
    err = 0.0
    for pose in PHOTOS:
        cells = our_cells(pose)
        near = neighbours(cells)
        for glyph, (mean, sd, spread) in stats(cells).items():
            want, want_sd, want_step = TARGET[glyph]
            err += ((mean - want) / 8.0) ** 2
            err += ((sd - want_sd) / 6.0) ** 2
            err += (max(0.0, spread - SPREAD_CAP) / 8.0) ** 2
            if glyph in near:
                err += ((near[glyph] - want_step) / 4.0) ** 2
    for pose in TURNS:
        for glyph, (mean, _sd, spread) in stats(our_cells(pose)).items():
            err += ((mean - TARGET[glyph][0]) / 10.0) ** 2
            err += (max(0.0, spread - SPREAD_CAP) / 8.0) ** 2
    return err


KNOBS = (
    (wireframe, 'DUSK', 0.03, -0.3, 0.6),
    (wireframe, 'SPOT', 0.03, 0.0, 0.8),
    (wireframe, 'SPOT_R', 0.2, 1.2, 3.5),
    (wireframe, 'TONE_LO', 0.05, -1.5, 1.5),
    (wireframe, 'TONE_SPAN', 0.2, 1.0, 8.0),
    (wireframe, 'EDGE', 0.1, 0.8, 3.0),
    (wireframe, 'RELIEF_CAP', 0.02, 0.1, 0.26),
    (wireframe, 'GRAIN_DOT', 0.2, 0.0, 4.0),
    (wireframe, 'GRAIN_COLON', 0.1, 0.0, 2.0),
    (engine, 'TONE_DEPTH', 0.1, 0.0, 1.0),
)


def fit(rounds=16):
    best = objective()
    print('start: fel %.2f' % best)
    scale = 1.0
    for _ in range(rounds):
        moved = False
        for mod, name, step, lo, hi in KNOBS:
            base = getattr(mod, name)
            for cand in (base + step * scale, base - step * scale):
                if cand < lo or cand > hi:
                    continue
                setattr(mod, name, cand)
                got = objective()
                if got < best - 1e-6:
                    best, base, moved = got, cand, True
                    print('  %s = %.2f  fel %.2f' % (name, cand, got))
                    break
                setattr(mod, name, base)
        if not moved:
            if scale < 0.3:
                break
            scale *= 0.5                # nothing moved: finer steps
    print('slut: fel %.2f - baka in:' % best)
    for mod, name, *_ in KNOBS:
        print('  %s.%s = %.2f' % (mod.__name__.split('.')[-1], name,
                                  getattr(mod, name)))


def show():
    print('%-10s     %-38s %-38s' % ('pose', 'exportören medel sd spann granne',
                                    'vår medel sd spann granne'))
    for pose in PHOTOS:
        his, mine = exporter_cells(pose), our_cells(pose)
        theirs, ours = stats(his), stats(mine)
        his_near, my_near = neighbours(his), neighbours(mine)
        for glyph in '.:':
            print("%-10s '%s'  %5.1f %4.1f %5.1f  granne %4.1f   %5.1f %4.1f %5.1f  granne %4.1f"
                  % (pose, glyph, *theirs[glyph], his_near.get(glyph, 0.0),
                     *ours[glyph], my_near.get(glyph, 0.0)))
    for pose in TURNS:
        print("%-10s      %-26s " % (pose, '') + '  '.join(
            "'%s' %5.1f sd %4.1f spann %4.1f" % (g, *v)
            for g, v in stats(our_cells(pose)).items()))


def staged_cells(pose, color_system, width=94, height=36):
    """Our render as the TERMINAL receives it: through a rich Console
    of the given colour depth into a buffer, then parsed back. What
    show() measures is what we send; this is what arrives."""
    import io
    from rich.console import Console
    from rich.text import Text
    wireframe._SHADOWS.clear()
    art = wireframe.render(facecheck.euler(*rotation(pose), order='xyz'),
                           width, height, colour=True, 
                           horizon=False, tip=0.0)
    sink = io.StringIO()
    court = Console(file=sink, force_terminal=True, width=width + 4,
                    color_system=color_system, highlight=False)
    court.print(Text.from_ansi(art), end='')
    got = []
    for py, line in enumerate(sink.getvalue().splitlines()):
        lum, col = None, 0
        for m in TOK.finditer(line):
            if m.group(2) is None:
                true = re.match(r'38;2;(\d+);(\d+);(\d+)$', m.group(1))
                pal = re.match(r'38;5;(\d+)$', m.group(1))
                if true:
                    lum = luma(tuple(int(v) for v in true.groups()))
                elif pal:
                    lum = luma(xterm_rgb(int(pal.group(1))))
                elif m.group(1) in ('0', '39') or m.group(1).startswith('3') \
                        and len(m.group(1)) == 2:
                    lum = None
                continue
            if m.group(2) in '.:' and lum is not None:
                got.append((m.group(2), 0.0, 0.0, lum, py, col))
            col += 1
    return got


def lines(cells):
    """The longest iso-line: hard steps (>25 luma) between same-class
    horizontal neighbours, chained where one sits above another. A
    smooth gradient has none; a requantised one draws a contour."""
    grid = {(c[4], c[5]): (c[0], c[3]) for c in cells}
    edges = set()
    for (r, col), (ch, lum) in grid.items():
        nb = grid.get((r, col + 1))
        if nb and nb[0] == ch and abs(nb[1] - lum) > 25:
            edges.add((r, col))
    longest, seen = 0, set()
    for start in edges:
        if start in seen:
            continue
        run, stack = 0, [start]
        seen.add(start)
        while stack:
            r, col = stack.pop()
            run += 1
            for step in ((r + 1, col - 1), (r + 1, col), (r + 1, col + 1),
                         (r - 1, col - 1), (r - 1, col), (r - 1, col + 1)):
                if step in edges and step not in seen:
                    seen.add(step)
                    stack.append(step)
        longest = max(longest, run)
    return len(edges), longest


def stage_report():
    """Every pose through both colour depths: the LOD line, measured.
    With the surface texture off - its random steps chain by chance -
    so what remains is structure: a rim, a terminator, a band."""
    print('%-10s %-11s %-12s %s' % ('pose', 'färgdjup', 'hårda steg',
                                   'längsta linje'))
    grain = wireframe.GRAIN_DOT, wireframe.GRAIN_COLON
    wireframe.GRAIN_DOT = wireframe.GRAIN_COLON = 0.0
    try:
        for pose in PHOTOS + TURNS[1:]:
            for depth in ('256', 'truecolor'):
                hard, longest = lines(staged_cells(pose, depth))
                print('%-10s %-11s %-12d %d' % (pose, depth, hard, longest))
    finally:
        wireframe.GRAIN_DOT, wireframe.GRAIN_COLON = grain


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--fit', action='store_true')
    parser.add_argument('--stage', action='store_true',
                        help='the render through rich at 256 colours and '
                             'truecolor: hard steps and the longest '
                             'iso-line each way')
    args = parser.parse_args(argv)
    if args.stage:
        stage_report()
        return 0
    show()
    if args.fit:
        fit()
        show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
