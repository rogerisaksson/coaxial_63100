"""The motor's drawing in braille: its cells, its teeth, its lines, nothing sheared."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_level_is_drawn_at_the_dot(report):
    """The top of a bar's mercury is `\u28c0`, `\u28e4`, `\u28f6` - one dot a step, every LED_PITCH-th
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

def test_lines_draw_what_line_draws(report):
    """`callouts.lines`, a ring's chords at once, sets the dots `line` sets one at a time: lines
    in, out of and across the grid, flat, upright and points, a ring of 96."""
    import random
    import numpy as np
    from coaxial.graphics.callouts import line, lines
    rng, bad = random.Random(5), 0
    for trial in range(300):
        h, w = rng.choice(((88, 300), (40, 120), (5, 7), (1, 1)))
        if trial % 10 == 0:
            k = np.arange(96) * (2.0 * math.pi / 96)
            x, y = w / 2 + 0.4 * w * np.cos(k), h / 2 + 0.6 * h * np.sin(k)
            segs = list(zip(zip(x, y), zip(np.roll(x, 1), np.roll(y, 1))))
        else:
            ends = ([(rng.uniform(-w, 2 * w), rng.uniform(-h, 2 * h)) for _ in range(2)]
                    for _ in range(rng.choice((1, 7, 96))))
            segs = [(a, rng.choice((b, a, (a[0], b[1]), (b[0], a[1])))) for a, b in ends]
        one, many = np.zeros((h, w), bool), np.zeros((h, w), bool)
        for a, b in segs:
            line(one, a, b)
        lines(many, [a[0] for a, _b in segs], [a[1] for a, _b in segs], [b[0] for _a, b in segs],
              [b[1] for _a, b in segs])
        bad += not np.array_equal(one, many)
    report.check('lines sets the dots line sets, dot for dot', bad == 0, '%d of 300 differ' % bad)


def test_c11_raster_cells_parity(report):
    """C11 `raster.c` and `raster_cells.c` match `engine.raster_py`, `engine.fold_py`,
    `drape.bent_py`, and `lit._braille_cells_py`."""
    import numpy as np
    from coaxial.graphics import drape, engine, gynoid, lit, shading, shapes
    from machine.body import figure
    from tools.cores import raster as c_raster
    from tools.render.oracle import DISTANCE, cube
    if c_raster.ensure() is None:
        return
    solid, half = cube()
    reach = math.sqrt(sum(h * h for h in half))
    fine_c = engine.fine(engine.camera(48, 22, reach, distance=DISTANCE))
    m_cube = engine.pose(35, 25, 110)
    d_c, t_c, s_c = engine.raster(solid, m_cube, fine_c, beam=shading.LIGHT, sun_min=shading.SUN_MIN)
    d_p, t_p, s_p = engine.raster_py(solid, m_cube, fine_c, beam=shading.LIGHT, sun_min=shading.SUN_MIN)
    db_c, tb_c, sb_c = engine.raster(solid, m_cube, fine_c, beam=shading.LIGHT, sun_min=shading.SUN_MIN, band=(12, 52))
    db_p, tb_p, sb_p = engine.raster_py(solid, m_cube, fine_c, beam=shading.LIGHT, sun_min=shading.SUN_MIN, band=(12, 52))
    f_c = engine.fold(d_c, t_c, s_c, 48, 22)
    f_p = engine.fold_py(d_p, t_p, s_p, 48, 22)
    report.check('C11 triangle raster and 2x4 fold bit-exact with Python oracles',
                 d_c == d_p and t_c == t_p and s_c == s_p
                 and db_c == db_p and tb_c == tb_p and sb_c == sb_p and f_c == f_p)

    body = gynoid.body()
    stride = dict(figure.stand(), left_hip=-28.0, right_hip=20.0, left_knee=42.0, waist_yaw=14.0)
    frames = body._frames(stride, ((0.0, figure.standing(), 0.0), figure.IDENT))
    p_dc, n_dc = shapes.moved(body.dense[0], body.dense[1], body.dense_spans, frames)
    p_dp, n_dp = p_dc.copy(), n_dc.copy()
    drape.bent(p_dc, n_dc, frames, stride, body.bends, dense=True)
    drape.bent_py(p_dp, n_dp, frames, stride, body.bends, dense=True)
    drape_err = max(float(np.max(np.abs(p_dc - p_dp))), float(np.max(np.abs(n_dc - n_dp))))
    report.check('C11 drape cloth skinning matches Python bent_py within 1e-14',
                 drape_err <= 1e-14, 'max %.2e' % drape_err)

    view_m = shapes.view(30.0, 14.0)
    cam_b = engine.fine(engine.camera(48, 22, 1.0, distance=3.5))
    depth_b, rgb_b = lit.splat((p_dc, n_dc, body.dense[3], body.dense[2]), view_m, cam_b, (0.0, 0.85, 0.0))
    fl_b = lit.grid(view_m, cam_b, (0.0, 0.85, 0.0))
    _np, _k, _f, _h, _pl, _bt, fl_ink, ld_ink = lit._const()
    over = np.zeros_like(depth_b, dtype=bool)
    over[16:24, 20:28] = True
    alone = np.zeros_like(depth_b, dtype=bool)
    alone[28:32, 30:34] = True
    leaders = np.zeros_like(depth_b, dtype=bool)
    leaders[4:12, 4:12] = True
    bc_c = c_raster.braille_cells(depth_b, rgb_b, fl_b, lit._noise64(), over, alone, leaders,
                                  fl_ink, ld_ink, 48, 22)
    bc_p = lit._braille_cells_py(depth_b, rgb_b, fl_b, over, alone, leaders, fl_ink, ld_ink, 48, 22)
    ink_err = float(np.max(np.abs(bc_c[2] - bc_p[2]))) if bc_c is not None else math.inf
    report.check('C11 Braille cell fold matches _braille_cells_py',
                 bc_c is not None and np.array_equal(bc_c[0], bc_p[0])
                 and np.array_equal(bc_c[1], bc_p[1]) and ink_err <= 1e-12,
                 'ink max %.2e' % ink_err)

    from coaxial.graphics import callouts, inset, onboard, quadcopter, scenery
    rng = np.random.default_rng(42)
    dc2, dp2 = np.zeros((144, 208), bool), np.zeros((144, 208), bool)
    ax, ay = rng.uniform(-40.0, 240.0, 160), rng.uniform(-30.0, 180.0, 160)
    bx, by = rng.uniform(-40.0, 240.0, 160), rng.uniform(-30.0, 180.0, 160)
    callouts.lines(dc2, ax, ay, bx, by)
    callouts.lines_twin(dp2, ax, ay, bx, by)
    lines_ok = np.array_equal(dc2, dp2)

    cam_s = engine.fine({'scale': 62.0, 'cx': 52.0, 'cy': 18.0, 'distance': 11.0, 'width': 104, 'height': 36, 'reach': 11.0})
    m_s, ctr_s = shapes.view(25.0, -18.0), (0.2, 1.8, -0.4)
    g_c, g_py = scenery.ground(m_s, cam_s, ctr_s), scenery.ground_twin(m_s, cam_s, ctr_s)
    ground_ok = np.array_equal(g_c.view(np.uint64), g_py.view(np.uint64))
    ends, _inks = scenery.edges(0, True)
    ys_c, xs_c, w_c, wh_c = scenery.strokes(ends, m_s, cam_s, ctr_s)
    ys_p, xs_p, w_p, wh_p = scenery.strokes_twin(ends, m_s, cam_s, ctr_s)
    strokes_ok = (np.array_equal(ys_c, ys_p) and np.array_equal(xs_c, xs_p)
                  and np.array_equal(wh_c, wh_p) and np.array_equal(w_c.view(np.uint64), w_p.view(np.uint64)))
    turn_q = shapes.turn_about('z', 28.0) @ shapes.turn_about('x', -19.0) @ shapes.turn_about('y', 43.0)
    at_q = np.array([0.2, 2.4, -5.0])
    pose = {'turn': turn_q, 'at': at_q, 'controls': (0.4, 0.25, -0.3, 0.15)}
    rotors = [(0.5, 420.0, 48.0, 42.0, 36.0), (1.2, 10.0, 49.0, 43.0, 36.5),
              (2.1, 420.0, 47.0, 41.0, 35.5), (0.8, 8.0, 48.5, 42.5, 36.0)]
    pos_c, nrm_c, uv_c, mat_c, idx_c = quadcopter._posed(pose, rotors, True, 'shell')
    from machine.aircraft import quad as quad_phys
    q_frame, q_prop = quadcopter.mesh('frame', 'shell'), quadcopter.mesh('prop')
    q_mov = [(q_prop, shapes.turn_about('y', math.degrees(sp * r[0])), (rx, quad_phys.DISC_M + 0.006, rz))
             for (rx, rz), sp, r in zip(quad_phys.ROTOR_AT, quad_phys.SPIN, rotors)
             if abs(r[1]) < quadcopter.BLADES_RAD_S]
    ref_p = np.concatenate([q_frame.dense[0] @ turn_q.T + at_q]
                           + [pt.dense[0] @ (turn_q @ rl).T + at_q + turn_q @ np.asarray(sp3, float)
                              for pt, rl, sp3 in q_mov])
    ref_n = np.concatenate([q_frame.dense[1] @ turn_q.T]
                           + [pt.dense[1] @ (turn_q @ rl).T for pt, rl, _sp3 in q_mov])
    pose_err = max(float(np.max(np.abs(pos_c - ref_p))), float(np.max(np.abs(nrm_c - ref_n))))
    pic = onboard.picture((0.0, 2.5, -6.0), (0.15, -0.12, 0.08), gate=0, line_=True)
    thumb = rng.integers(0, 256, (32, 48, 3), dtype=np.uint8)
    eyes_box = inset.eyes((thumb, [('chair', (210, 150, 80), [(80, 120), (220, 110), (240, 290), (70, 280)])]), 104, 36)
    report.check('C11 raster_strokes.c lines, strokes, ground, pose_parts and insets hold parity',
                 lines_ok and ground_ok and strokes_ok and pose_err <= 1e-14
                 and len(pos_c) == len(nrm_c) == len(uv_c) == len(mat_c) and len(idx_c) > 0
                 and len(pic) == onboard.HIGH and len(eyes_box) > 0,
                 'lines=%s ground=%s strokes=%s pose_err=%.2e' % (lines_ok, ground_ok, strokes_ok, pose_err))


ROSTER = (test_lines_draw_what_line_draws, test_c11_raster_cells_parity, test_the_level_is_drawn_at_the_dot,
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
