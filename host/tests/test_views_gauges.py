"""The dials and gauges: the SOA legend, each on its own scale, round on this terminal, the sweep behind the needle."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report, ansi_plain, rows_of


def test_the_soa_legend_reads_the_whole_soa(report):
    """SWITCH SOA and MOTOR SOA say how much of the record's SOA is spent,
    and flash red where the ceiling in force is.
    """
    from coaxial.draw import cross_section
    from coaxial.model import thermal
    from terminal.views.rotor import thermal as rotor
    IDENT_MARGIN = {'UNCERTAIN': 0.80, 'CONVERGING': 0.90, 'STABLE': 1.0}

    def a_view(state, worst, tripped=False, winding_used=None):
        budget = {'worst': worst, 'tripped': tripped,
                  'throttling': worst >= 0.9}
        if winding_used is not None:
            budget['winding_used'] = winding_used
            budget['winding_derate'] = 1.0 if winding_used < 0.9 else 0.5
        return {'thermal': {'nodes': {'driver_u': 60.0}}, 'budget': budget,
                'ident': {'state': state, 'margin': IDENT_MARGIN[state]},
                'params': {}, 'winding_at': None,
                'state': {'id': 0.0, 'iq': 0.0, 'vd': 0.0, 'vq': 0.0}}

    reads = {}
    for state in ('UNCERTAIN', 'CONVERGING', 'STABLE'):
        spent, cls = rotor.headrooms(a_view(state, 1.0, tripped=True))[0]
        reads[state] = (spent, cls)
    report.check('a board at the ceiling its policy leaves reads 80 % of '
                 'the SOA UNCERTAIN, 90 CONVERGING, 100 STABLE',
                 abs(reads['UNCERTAIN'][0] - 0.80) < 1e-9
                 and abs(reads['CONVERGING'][0] - 0.90) < 1e-9
                 and abs(reads['STABLE'][0] - 1.00) < 1e-9,
                 ' '.join('%s %.2f' % (s, v[0]) for s, v in reads.items()))
    report.check('and in the trip\'s red, pulsing, since the board is '
                 'acting there',
                 all(cls in (cross_section.SOA_TRIP, cross_section.SOA_FLASH)
                     for _spent, cls in reads.values()),
                 [cls for _s, cls in reads.values()])
    spent, cls = rotor.headrooms(a_view('UNCERTAIN', 0.5))[0]
    report.check('half way to that ceiling reads 40 % of the SOA and is '
                 'neither red nor pulsing - the amber there is the '
                 'gauge\'s own band, not the policy\'s',
                 abs(spent - 0.40) < 1e-9
                 and cls not in (cross_section.SOA_TRIP, cross_section.SOA_FLASH),
                 '%.2f cls %d' % (spent, cls))
    motor = rotor.headrooms(a_view('UNCERTAIN', 0.2, winding_used=1.0))[1]
    report.check('the winding\'s own legend the same way, off the board\'s '
                 'winding under the same policy, and pulsing when the '
                 'board holds the stage back for it',
                 abs(motor[0] - 0.80) < 1e-9
                 and motor[1] in (cross_section.SOA_TRIP, cross_section.SOA_FLASH),
                 '%.2f cls %d' % motor)
    report.check('the ceiling in force is the record\'s span trimmed: the '
                 'laminate\'s 105 is 89 C at the 0.8 floor',
                 abs(thermal.ceiling_of('board', 0.8) - 89.0) < 1e-9
                 and abs(thermal.ceiling_of('board', 1.0) - 105.0) < 1e-9,
                 '%.1f' % thermal.ceiling_of('board', 0.8))
    # A board at the ceiling a margin of 0.86 leaves reads 86 % of the SOA: the
    # legend follows the number, not the word.
    view = a_view('CONVERGING', 1.0, tripped=True)
    view['ident']['margin'] = 0.86
    spent, _cls = rotor.headrooms(view)[0]
    report.check('and a margin between the steps reads to the percent: 0.86 '
                 'at the ceiling in force is 86 % of the SOA',
                 abs(spent - 0.86) < 1e-9, '%.2f' % spent)


def test_every_gauge_shows_its_own_scale(report):
    """The dimmed track runs the whole of every bar, at its own width."""
    from coaxial.draw import cross_section

    n = 4
    art = cross_section.render(0.0, 24, 28, 46, 18,
                         left=[(0.0, cross_section.SOA_OK)] * n,
                         right=[(0.0, cross_section.SOA_OK)] * n,
                         bottom=[(0.0, cross_section.SOA_WARN), (0.0, cross_section.WATTS)])
    rows = art.split(chr(10))
    left, right = cross_section.gutters(46, 18, n, n)

    # Every tube, every row of it: down to the row of air over the floors.
    seen = set()
    for row in rows[1:-(2 + cross_section.FLOOR_AIR)]:
        for col in list(left) + list(right):
            seen.add(row[col])
    report.check('an empty tube is drawn in every one of its rows',
                 ' ' not in seen and chr(0x2800) not in seen,
                 ''.join(sorted(seen)))
    report.check('and every tube is drawn the same way',
                 len(seen) == 1, ''.join(sorted(seen)))
    report.check('at the tube\'s own width, both lanes',
                 all(ord(c) - 0x2800 & 0x08 or ord(c) - 0x2800 & 0x10
                     or ord(c) - 0x2800 & 0x20 or ord(c) - 0x2800 & 0x80
                     for c in seen), ''.join(sorted(seen)))

    # The flat gauges along the foot, one dot a cell rather than one every
    # other cell.
    first, last = cross_section.span(46, 18, n, n)
    floor = rows[-1]
    drawn = [floor[col] for col in range(first, last + 1)]
    report.check('the foot gauge draws a scale in every cell it spans',
                 all(c != ' ' and c != chr(0x2800) for c in drawn),
                 '%d of %d blank'
                 % (sum(1 for c in drawn if c in (' ', chr(0x2800))),
                    len(drawn)))


def test_the_dial_is_round_on_this_terminal(report):
    """The shaft angle's face takes the measured cell aspect, a notch under
    64 by 23.
    """
    from terminal.ui import aspect
    from coaxial.draw import dial
    from terminal.views import show_angle as view

    report.check('a given aspect wins, said as given',
                 aspect.aspect_of(2.3) == (2.3, 'given'))
    report.check('and the face is a notch smaller than 64 by 23',
                 view.ART_WIDTH < 64 and view.ART_HEIGHT < 23
                 and view.ART_HEIGHT >= 19,
                 '%d by %d' % (view.ART_WIDTH, view.ART_HEIGHT))

    def rows_of(aspect):
        lines = dial.render(0.0, view.ART_WIDTH, view.ART_HEIGHT, 100,
                            aspect=aspect).split('\n')
        return sum(1 for line in lines
                   if any(0x2800 < ord(c) <= 0x28FF for c in line))

    report.check('a taller cell draws the face over fewer rows - the '
                 'circle stays a circle on the screen',
                 rows_of(2.3) < rows_of(2.0),
                 '%d rows at 2.3 against %d at 2.0'
                 % (rows_of(2.3), rows_of(2.0)))


def test_the_sweep_decays_behind_the_needle(report):
    """The sweep is where the needle has been, not an arc hung off the reading:
    a phosphor lit by the needle passing and decaying everywhere else, so it
    trails whichever way the dial turns. Drawn off the reading alone it sat
    ahead of a needle running counter-clockwise (2026-09-28).
    """
    import math
    from coaxial.draw import dial

    def sides(way):
        """Lit bins behind the needle against ahead of it, sweeping `way`."""
        trail = dial.trail()
        clock, real = [0.0], dial.time.monotonic
        dial.time.monotonic = lambda: clock[0]
        try:
            behind = ahead = 0
            for i in range(40):
                clock[0] = i * 0.05
                span = math.radians((180.0 + way * 3.0 * i) % 360.0)
                glow = dial._glow(trail, span, clock[0])
                at = int(span / (2.0 * math.pi) * dial.TRAIL_BINS) % dial.TRAIL_BINS
                if i < 5:
                    continue
                for b in range(dial.TRAIL_BINS):
                    if glow[b] <= 0.02:
                        continue
                    off = ((b - at + dial.TRAIL_BINS // 2) % dial.TRAIL_BINS
                           - dial.TRAIL_BINS // 2)
                    if off:
                        behind, ahead = ((behind + 1, ahead) if off * way < 0
                                         else (behind, ahead + 1))
        finally:
            dial.time.monotonic = real
        return behind, ahead

    for way, name in ((1, 'clockwise'), (-1, 'counter-clockwise')):
        behind, ahead = sides(way)
        report.check('the sweep trails a needle running %s' % name,
                     behind > 20 * ahead and behind > 100,
                     '%d behind, %d ahead' % (behind, ahead))

    # And it fades: the oldest lit bin is dimmer than the newest.
    trail = dial.trail()
    clock, real = [0.0], dial.time.monotonic
    dial.time.monotonic = lambda: clock[0]
    try:
        for i in range(10):
            clock[0] = i * 0.05
            dial._glow(trail, math.radians(180.0 + 3.0 * i), clock[0])
        glow = trail['glow']
    finally:
        dial.time.monotonic = real
    first = int(math.radians(180.0) / (2.0 * math.pi) * dial.TRAIL_BINS)
    last = int(math.radians(180.0 + 27.0) / (2.0 * math.pi) * dial.TRAIL_BINS)
    report.check('and the bin it left first is the dimmer',
                 glow[first] < glow[last] and glow[last] > 0.9,
                 '%.3f then %.3f' % (glow[first], glow[last]))

    # A still has no history: it still shows a sweep, so a notebook's frame reads.
    art = dial.render(137.0, 60, 20)
    report.check('a still with no phosphor still draws a face',
                 any(0x2800 < ord(c) <= 0x28FF for c in art), art[:40])


def test_the_face_wears_its_two_scales(report):
    """SHAFT ANGLE's die temperature and field stand either side of the face
    as tubes on their own ranges - the scales beside it the bench asked
    for, die temperature and field strength in gauss, 2026-09-07.
    """
    from coaxial.draw import dial
    from machine import ansi

    def dots(lines):
        return sum(bin(ord(c) - 0x2800).count('1')
                   for line in lines for c in line
                   if 0x2800 <= ord(c) <= 0x28FF)

    cold = dial.scale(-40.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                      '-40.0 C', dial.die_ink, 'left')
    warm = dial.scale(61.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                      '61.0 C', dial.die_ink, 'left')
    hot = dial.scale(150.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                     '150.0 C', dial.die_ink, 'left')
    report.check('a scale is the face\'s rows and a caption, SCALE_W wide',
                 len(warm) == 22 and all(len(l) == dial.SCALE_W for l in warm)
                 and warm[0].strip() == 'DIE' and warm[-1].strip() == '61.0 C',
                 (len(warm), sorted({len(l) for l in warm}), warm[0], warm[-1]))
    report.check('its graduations are numbered, -40 at the foot and 150 at '
                 'the top',
                 '-40' in warm[-3] and '150' in warm[1]
                 and all(str(t) in ''.join(warm) for t in dial.DIE_TICKS),
                 [l[:5] for l in warm])
    report.check('the tube is four dots wide the whole way, glass and fill',
                 all(bin(ord(c) - 0x2800).count('1') == 8
                     for line in warm[1:-2] for c in line[5:7]),
                 [line[5:7] for line in warm[1:-2]])

    def inked(celsius):
        lines = dial.scale(celsius, dial.DIE_RANGE, 21, dial.DIE_TICKS,
                           'DIE', '', dial.die_ink, 'left', colour=True)
        return sum(ansi.code(dial.die_ink(celsius)) in line
                   for line in lines[1:-2])

    report.check('and the fill rises with the reading, the glass above it '
                 'in ash',
                 0 == inked(-40.0) < inked(61.0) < inked(150.0) == 19
                 and ansi.code(dial.LABEL_INK) in ''.join(
                     dial.scale(61.0, dial.DIE_RANGE, 21, dial.DIE_TICKS,
                                'DIE', '', dial.die_ink, 'left',
                                colour=True)[1:5]),
                 '%d < %d < %d rows inked' % (inked(-40.0), inked(61.0),
                                              inked(150.0)))
    inks = [dial.scale(g, dial.FIELD_RANGE, 21, dial.FIELD_TICKS, 'FIELD',
                       '%d G' % g, dial.field_ink, 'right', colour=True)
            for g in (12, 380, 1100)]
    report.check('the field tube is blue under the recommended band - a '
                 'weak magnet or none - green in it, red past it',
                 ansi.code(dial.BAND_INK[0]) in ''.join(inks[0])
                 and ansi.code(dial.BAND_INK[1]) in ''.join(inks[1])
                 and ansi.code(dial.BAND_INK[2]) in ''.join(inks[2])
                 and ansi.code(dial.BAND_INK[1]) not in ''.join(inks[0])
                 and dial.field_ink(200) == dial.BAND_INK[0],
                 [dial.field_ink(g) for g in (12, 200, 380, 1100)])
    report.check('and the die tube is blue under the board\'s working '
                 'range, green through it, red past it',
                 dial.die_ink(5.0) == dial.BAND_INK[0]
                 and dial.die_ink(25.0) == dial.BAND_INK[1]
                 and dial.die_ink(61.0) == dial.BAND_INK[1]
                 and dial.die_ink(90.0) == dial.BAND_INK[2],
                 [dial.die_ink(c) for c in (5.0, 25.0, 61.0, 90.0)])
    art = dial.instrument(137.0, 380, 273.15 + 61.0, colour=True).split('\n')
    report.check('the instrument is the face and two scales with their air, '
                 'line for line, and the die\'s reading wears its band',
                 len(art) == 22 and all(
                     len(ansi_plain(l)) == 58 + 2 * (dial.SCALE_W + 1)
                     for l in art)
                 and ansi.code(dial.die_ink(61.0)) in art[-1]
                 and '61.0 C' in ansi_plain(art[-1]),
                 (len(art), sorted({len(ansi_plain(l)) for l in art})))
    from terminal.views import show_angle as page
    report.check('and the face gives way to the scales and fills the rest: 68 wide at 130 '
                 'columns, FACE_MIN at 98, alone under that, as tall as the terminal leaves, '
                 'the whole face where the terminal would not say',
                 page.fit(130) == (True, 68, page.ART_HEIGHT)
                 and page.fit(100) == (True, 38, page.ART_HEIGHT)
                 and page.fit(98) == (True, page.FACE_MIN, page.ART_HEIGHT)
                 and page.fit(90) == (False, 46, page.ART_HEIGHT)
                 and page.fit(150, 44) == (True, 88, 44 - page.STAGE_ROWS)
                 and page.fit(0) == (False, page.ART_WIDTH, page.ART_HEIGHT)
                 and page.fit(0, 0, True) == (True, page.ART_WIDTH, page.ART_HEIGHT),
                 [page.fit(c) for c in (130, 100, 98, 90, 0)])
    report.check('and the caption leaves the gauss to the scale that shows it',
                 'gauss' not in dial.caption(137.0, 380, gauss=False)
                 and 'gauss' in dial.caption(137.0, 380)
                 and 'no magnet' in dial.caption(0.0, 12, gauss=False),
                 dial.caption(137.0, 380, gauss=False))


def test_the_power_face_has_its_middle_at_half_a_kilowatt(report):
    """The kW bar is a power law pinned at 500 W, full at 2 kW, red past."""
    from coaxial.draw import cross_section
    from terminal.views.rotor import thermal

    at = {w: thermal.watts_share(w) for w in (0, 20, 100, 500, 2000, 2500)}
    report.check('nothing draws nothing', at[0][0] == 0.0)
    report.check('twenty watts is a tenth of the bar - a small draw is seen',
                 abs(at[20][0] - 0.1) < 0.01, '%.3f' % at[20][0])
    report.check('and a hundred is not yet a quarter',
                 0.2 < at[100][0] < 0.25, '%.3f' % at[100][0])
    report.check('half a kilowatt is half the bar',
                 abs(at[500][0] - 0.5) < 1e-9, '%.3f' % at[500][0])
    report.check('two kilowatts is the whole of it, still in its own ink',
                 at[2000] == (1.0, cross_section.WATTS), str(at[2000]))
    report.check('and past it the bar is full and deep red',
                 at[2500] == (1.0, cross_section.SOA_TRIP), str(at[2500]))
    report.check('the middle is a named constant, not a magic exponent',
                 thermal.WATTS_MID == 500.0 and thermal.WATTS_SCALE == 2000.0)


def test_every_frame_corner_on_the_map_is_a_right_angle(report):
    """A frame's top and bottom lines start at the side's lane."""
    from coaxial.draw import thermalmap as tm

    # side, edge, the lane the side runs down -> the corner cell's glyph.
    right_angle = {('left', 'top', 0): '\u2856', ('left', 'top', 1): '\u28b0',
                   ('right', 'top', 1): '\u28b2', ('right', 'top', 0): '\u2846',
                   ('left', 'bottom', 0): '\u2827', ('left', 'bottom', 1): '\u2838',
                   ('right', 'bottom', 1): '\u283c', ('right', 'bottom', 0): '\u2807'}

    def glyph(rows, r, c):
        bits = 0
        for lane in (0, 1):
            for y in range(4):
                if rows[4 * r + y][2 * c + lane] == tm.MARK:
                    bits |= tm.BRAILLE_BITS[lane][y]
        return chr(tm.BRAILLE + bits)

    judged, wrong = 0, []
    for cells in (40, 48, 60, 72, 88):
        dx = dy = tm.OUTER_MM / cells
        rim, _ = tm._mask(cells, cells, ())
        for mark in tm.MARKS:
            label, refs, _where, margin = mark
            (c0, c1, r0, r1), lanes = tm._cell_rect(
                tm.frame(refs, margin), cells, cells, dx, dy)
            rows, _ = tm._mask(cells, cells, (mark,))
            for side, c, lane in (('left', c0, lanes[0]),
                                  ('right', c1, lanes[1])):
                for edge, r in (('top', r0), ('bottom', r1)):
                    if any(rim[4 * r + y][2 * c + lane_] != tm.FIELD
                           for lane_ in (0, 1) for y in range(4)):
                        continue
                    judged += 1
                    got = glyph(rows, r, c)
                    if got != right_angle[(side, edge, lane)]:
                        wrong.append('%d %s %s-%s %s' % (cells, label, edge,
                                                         side, got))
    report.check('every judged corner of every frame at every size is its '
                 'right angle: %d judged' % judged,
                 judged >= 120 and not wrong, '; '.join(wrong[:6]))

    # And the eight glyphs themselves, off a blank field, both lane
    # combinations: the lines meet the side and go no further.
    def drawn(lanes):
        rows = [[tm.FIELD] * 24 for _ in range(24)]
        tm._draw_frame(rows, [2, 8, 1, 4], lanes, 12, 12)
        return [glyph(rows, r, c) for r, c in ((1, 2), (1, 8), (4, 2), (4, 8))]
    report.check('a side in the outer lanes: ' + ' '.join(drawn([0, 1])),
                 drawn([0, 1]) == ['\u2856', '\u28b2', '\u2827', '\u283c'])
    report.check('a side in the inner lanes: ' + ' '.join(drawn([1, 0])),
                 drawn([1, 0]) == ['\u28b0', '\u2846', '\u2838', '\u2807'])


def test_a_held_peak_falls_ever_faster(report):
    """gauges.peak: up with its level at once, held PEAK_HOLD_S, then back to the level ever
    faster and no further; a floor gauge draws it, a tick in the mark's ink past its level."""
    from coaxial.draw import cross_section, gauges
    state, dt, trace = None, 0.05, []
    for k in range(80):
        state = gauges.peak(state, 0.9 if k < 4 else 0.2, dt)
        trace.append(state[0])
    held = trace[3:4 + int(gauges.PEAK_HOLD_S / dt)]
    falls = [a - b for a, b in zip(trace, trace[1:]) if a - b > 1e-12][:-1]
    report.check('up at once, held %.1f s, then falling ever faster to its level and no further'
                 % gauges.PEAK_HOLD_S,
                 all(abs(x - 0.9) < 1e-12 for x in held) and len(falls) >= 3
                 and all(b > a for a, b in zip(falls, falls[1:])) and min(trace) >= 0.2 - 1e-12
                 and abs(trace[-1] - 0.2) < 1e-12,
                 'held %.2f for %d frames, falls %s' % (
                     held[-1], len(held), ' '.join('%.3f' % f for f in falls[:5])))
    bare = cross_section.render(0.0, bottom=[(0.3, cross_section.WATTS)]).splitlines()[-1]
    peaked = cross_section.render(0.0, bottom=[(0.3, cross_section.WATTS, 0.8)]).splitlines()[-1]
    moved = [k for k, (a, b) in enumerate(zip(bare, peaked)) if a != b]
    report.check('a floor gauge draws its held peak: one tick past its level',
                 len(moved) == 1 and moved[0] > 0.5 * len(bare),
                 'cells %s of %d changed' % (moved, len(bare)))


ROSTER = (test_the_soa_legend_reads_the_whole_soa, test_every_gauge_shows_its_own_scale,
          test_the_dial_is_round_on_this_terminal, test_the_sweep_decays_behind_the_needle,
          test_the_face_wears_its_two_scales,
          test_the_power_face_has_its_middle_at_half_a_kilowatt,
          test_every_frame_corner_on_the_map_is_a_right_angle, test_a_held_peak_falls_ever_faster)

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
