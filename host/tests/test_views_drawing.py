"""The motor's drawing in braille: its cells, its teeth, its lines, nothing sheared."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_level_is_drawn_at_the_dot(report):
    """The top of a bar's mercury is `⣀`, `⣤`, `⣶` - one dot a step, every LED_PITCH-th
    dark: LED segments."""
    from coaxial.draw import cross_section

    track = chr(0x28D2)
    tops, dots = [], []
    for k in range(0, 8):
        share = (k + 0.5) / 40.0          # a ten-row tube is forty dots
        art = cross_section.render(0.0, 24, 28, 30, 12,
                             left=[(share, cross_section.SOA_OK)]).split(chr(10))
        column = [row[0] for row in art]
        mercury = [c for c in column if c not in (track, chr(0x2800))]
        tops.append(mercury[0] if mercury else '?')
        dots.append(sum(bin(ord(c) - 0x2800).count('1') for c in mercury))
    report.check('the top cell climbs a dot at a time, its fourth dot a segment gap',
                 tops[:4] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                              chr(0x28F6)], ''.join(tops))
    report.check('and keeps climbing into the next cell the same way',
                 tops[4:8] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                               chr(0x28F6)], ''.join(tops))
    gap = cross_section.LED_PITCH - 1
    report.check('two dots a step - both lanes - none at a gap, never a whole cell',
                 all(b - a == (0 if (i + 1) % cross_section.LED_PITCH == gap else 2)
                     for i, (a, b) in enumerate(zip(dots, dots[1:]))), str(dots))

    # Along the foot, one lane at a time.
    ends = []
    for k in range(1, 5):
        share = (k + 0.5) / 60.0
        frame, _lit = cross_section._raster(
            0.0, 24, 28, 30, 12, None, None, None, None, None, None,
            [(share, cross_section.WATTS)], 2.0)
        row = frame.height - 1
        level = [col for col in range(frame.width)
                 if frame.owner[row][col] == cross_section.WATTS]
        ends.append(chr(0x2800 + frame.dots[row][max(level)]) if level
                    else '?')
    report.check('the foot gauge ends on a lane, not a cell, its fourth dot a gap',
                 ends == [chr(0x2807), chr(0x283F), chr(0x2807), chr(0x2807)],
                 ''.join(ends))


def test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most(
        report):
    """The air gap is less than a cell tall, and that is a trade the drawing
    makes on purpose.
    """
    from coaxial.draw import cross_section

    magnet = {cross_section.NORTH, cross_section.SOUTH}
    teeth = {cross_section.TOOTH_U, cross_section.TOOTH_V, cross_section.TOOTH_W}
    seat = cross_section.Seat(46, 18, None, None, None, None, None, None, 2.0)
    report.check('the teeth reach their full fraction of the radius',
                 abs(seat.radii.tooth_out
                     - seat.radii.can * cross_section.F_TOOTH_OUT) < 1e-9,
                 '%.2f of %.2f' % (seat.radii.tooth_out,
                                   seat.radii.can * cross_section.F_TOOTH_OUT))
    mixed = elsewhere = 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = cross_section._raster(
                6.0, 24, 28, 46, 18, None,
                cross_section._drive((30.0, -15.0, -15.0)), None,
                None, None, None, None, aspect)
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if tally and set(tally) & magnet and set(tally) & teeth:
                        mixed += 1
                        most = max(tally, key=lambda c: (tally[c], c))
                        elsewhere += frame.owner[row][col] != most
    report.check('cells holding both exist - the gap is under a cell',
                 mixed > 0, '%d cells' % mixed)
    report.check('and every one goes to whichever has more of it',
                 elsewhere == 0, '%d did not' % elsewhere)


def test_a_line_keeps_the_cell_it_shares_with_an_area(report):
    """A ring through a cell full of tooth or magnet keeps its colour."""
    import math

    from coaxial.draw import cross_section

    magnet = {cross_section.NORTH, cross_section.SOUTH}
    seat = cross_section.Seat(46, 18, None, None, None, None, None, None, 2.0)
    r = seat.radii
    yoke_worst, lost = 1.0, 0
    for aspect in (2.0, 2.3):
        for deg in (0.0, 6.0, 12.0, 18.0):
            frame, _lit = cross_section._raster(
                deg, 24, 28, 46, 18, None,
                cross_section._drive((30.0, -15.0, -15.0)), None,
                None, None, None, None, aspect)
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if (tally and cross_section.CAN in tally and set(tally) & magnet
                            and frame.owner[row][col] != cross_section.CAN):
                        lost += 1
            ring = set()
            for k in range(720):
                phi = math.radians(k / 2.0)
                x = seat.cx + r.tooth_in * math.cos(phi)
                y = seat.cy - r.tooth_in * math.sin(phi) / (aspect / 2.0)
                ring.add((int(y) // 4, int(x) // 2))
            own = sum(1 for row, col in ring
                      if frame.owner[row][col] in (cross_section.YOKE, cross_section.BORE))
            yoke_worst = min(yoke_worst, own / len(ring))
    report.check('the yoke ring is wholly its own colour where the teeth '
                 'root', yoke_worst >= 0.99, 'worst %.2f' % yoke_worst)
    report.check('and no can-ring cell shared with a magnet is lost to it',
                 lost == 0, '%d cells' % lost)

    # The rule itself, on one cell: a line with one dot beats an area with
    # seven; two lines settle by dots; two areas settle by dots.
    frame = cross_section.Frame(1, 1)
    for k in range(7):
        frame.put(k % 2, k // 2, cross_section.NORTH)
    frame.put(1, 3, cross_section.CAN)
    report.check('one dot of ring outweighs seven of magnet',
                 frame.owner[0][0] == cross_section.CAN)
    frame = cross_section.Frame(1, 1)
    for k in range(6):
        frame.put(k % 2, k // 2, cross_section.TOOTH_U)
    frame.put(0, 3, cross_section.TOOTH_W)
    report.check('and between two areas the most dots win, not the rank',
                 frame.owner[0][0] == cross_section.TOOTH_U)
    # Not the truth stroke, which wins outright: `Frame.put` has why.
    report.check('the lines are the rings - not the arc, not the stroke',
                 cross_section.LINES == frozenset((cross_section.BORE, cross_section.YOKE,
                                             cross_section.CAN)))

    # The shaft sensor's stroke is drawn through the magnet band.
    teeth = {cross_section.TOOTH_U, cross_section.TOOTH_V, cross_section.TOOTH_W}
    gutter = set(range(0, 8)) | set(range(38, 46))
    took = in_gutter = 0
    own_min, ring_max = 999, 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = cross_section._raster(
                6.0, 24, 28, 46, 18, float(deg),
                cross_section._drive((30.0, -15.0, -15.0)), 41.0,
                [(0.3, cross_section.SOA_OK)] * 8, [(0.3, cross_section.SOA_OK)] * 8,
                None, [(0.3, cross_section.SOA_WARN), (0.3, cross_section.WATTS)],
                aspect)
            own = rings = 0
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if not tally or cross_section.TRUTH not in tally:
                        continue
                    if frame.owner[row][col] == cross_section.TRUTH:
                        took += bool(set(tally) & teeth)
                        own += 1
                        in_gutter += col in gutter
                        rings += (cross_section.CAN in tally
                                  or cross_section.YOKE in tally)
            own_min = min(own_min, own)
            ring_max = max(ring_max, rings)
    # A cell's diagonal still bridges the band's inner end and a tooth's tip at
    # some angles, so the stroke may share a cell with a tooth; in that cell it
    # is not a candidate, because a white cell on a tooth is a mark on the
    # stator.
    report.check('the truth stroke takes no cell a tooth is in',
                 took == 0, '%d cells' % took)
    report.check('and never lands in a gutter', in_gutter == 0,
                 '%d cells' % in_gutter)
    report.check('and is seen in every pose', own_min >= 1,
                 'fewest own cells %d' % own_min)
    report.check('and takes at most one cell of the rim, at its own angle',
                 ring_max <= 1, 'most in one pose %d' % ring_max)


def test_nothing_in_the_drawing_can_be_sheared(report):
    """No character in the art has East Asian ambiguous width.

    Unicode does not decide for those. A terminal set for East Asian text
    draws them two columns wide and every other one draws them narrow,
    and it is a setting rather than a font - so a page carrying one is a
    page that renders correctly on one bench and shears on the next.
    Sheared, the mark doubles, everything after it on the row slides a
    column, and the colour runs slide with it: the drawing bleeds inside
    its own box.

    Braille is narrow by definition, so what caught this out was the
    furniture: `\u25c0` and `\u25b6` as arrowheads, `\u25b2` and
    `\u25bc` at the foot, and the degree sign. All four triangles have
    unambiguous small twins and the degree has U+1D52.

    The bead is not among them - U+29BF is narrow, and the fallback built
    for it picked U+25CF, which is ambiguous. The safe substitute was the
    only unsafe character in the pair, and both are gone.
    """
    import unicodedata

    from coaxial.draw import cross_section
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import legend
    from terminal.ui import scroll

    drawn = cross_section.render(6.0, 24, 28, 46, 18, pointer_deg=41.0)
    # The scroll arrows are the stage's, every page's furniture.
    said = ''.join(str(x) for x in
                   (legend.AIM_LEFT, legend.AIM_RIGHT, scroll.UP, scroll.DOWN,
                    legend.DEGREE, legend.LEADER, cross_section.POINTER_GLYPH)
                   ) + ''.join(legend.TURN) + ''.join(legend.DROP)
    for name, text in (('the drawing', drawn), ("the view's furniture", said)):
        bad = sorted({c for c in text
                      if unicodedata.east_asian_width(c) == 'A'})
        report.check('%s carries no ambiguous-width character' % name,
                     not bad,
                     ' '.join('%s U+%04X' % (c, ord(c)) for c in bad))

    # The substitutes are the same marks, not near misses: a small triangle
    # points the same way as its big twin.
    report.check('the arrowheads are the small triangles',
                 (legend.AIM_LEFT, legend.AIM_RIGHT) == (chr(0x25C2), chr(0x25B8)),
                 legend.AIM_LEFT + legend.AIM_RIGHT)
    from terminal.ui import scroll
    report.check('and the foot uses their up and down - the stage\'s, which '
                 'every page\'s scroll markers wear too',
                 (scroll.UP, scroll.DOWN) == (chr(0x25B4), chr(0x25BE)),
                 scroll.UP + scroll.DOWN)


def test_the_flat_drawings_spend_the_block(report):
    """The 2D drawings place their edges by coverage, not by "any corner"."""
    from coaxial.draw import cross_section, dial
    from coaxial.graphics import raster

    of = len(raster.SUBDOT)
    report.check('a dot the shape covers lights',
                 raster.covered(of, of))
    report.check('a dot it misses never does',
                 not raster.covered(0, of))
    report.check('half a dot lights - a one-dot rim is a line the '
                 'drawing means', raster.covered(2, of))
    report.check('and a quarter of one does not, whatever the position',
                 not any(raster.covered(1, of, x, y)
                         for x in range(4) for y in range(4)))

    # The rotor and the protractor both raster through the same rule, so both
    # wear patterns a fringe rounded up to solid could never produce.
    art = cross_section.render(0.0, 24, 28, 46, 18)
    face = dial.render(137.0, 60, 20)
    for name, drawn in (('the rotor', art), ('the protractor', face)):
        seen = {c for c in drawn if 0x2800 < ord(c) < 0x2900}
        report.check('%s draws more than a handful of patterns' % name,
                     len(seen) >= 40, '%d distinct' % len(seen))
        report.check('%s draws partial cells, not only solid ones' % name,
                     any(0 < bin(ord(c) - 0x2800).count('1') < 8
                         for c in seen))


ROSTER = (test_the_level_is_drawn_at_the_dot,
          test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most,
          test_a_line_keeps_the_cell_it_shares_with_an_area,
          test_nothing_in_the_drawing_can_be_sheared, test_the_flat_drawings_spend_the_block)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
