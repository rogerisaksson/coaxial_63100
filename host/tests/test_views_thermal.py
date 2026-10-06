"""The thermal observer's page: the board's halftone map, its evidence, its headroom, the legs heating together."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_thermal_map_is_a_halftone_with_its_parts_marked(report):
    """The thermal observer's board is braille: a blue-noise stipple denser
    where it is hotter, in the ramp blended to 24 bits, the rim a dot
    wide, and the parts that make the heat drawn on it as blocks with
    edges and a label - placed from the pick and place.
    """
    import re
    from coaxial.draw import thermalmap
    from machine import ansi
    from coaxial.model.thermal import ALL_NODES

    warm = {n: 40.0 for n in ALL_NODES}
    cells = 88

    def picture(nodes):
        return thermalmap.render(nodes, board_c=30.0, cells=cells,
                                 colour=True, reserve=0, trailing=0)

    def strip(text):
        return re.sub('\x1b\\[[0-9;]*m', '', text)

    def dots(text):
        return sum(bin(ord(ch) - 0x2800).count('1') for ch in text
                   if 0x2800 <= ord(ch) < 0x2900)

    said = picture(warm)
    rows = strip(said).split('\n')
    art = [row[:cells] for row in rows]
    letters = set(''.join(mark[0] for mark in thermalmap.MARKS))
    stray = set(ch for row in art for ch in row
                if ch != ' ' and not 0x2800 <= ord(ch) < 0x2900
                and ch not in letters)
    report.check('the board is braille, and the only letters on it are '
                 'the labels', not stray, repr(''.join(sorted(stray))))
    for label, _refs, _where, _margin in thermalmap.MARKS:
        report.check('%s is written on it' % label,
                     any(label in row for row in art))

    # Round, and nothing past the rim: every lit cell's centre inside the
    # radius plus a cell, the top row narrow, the middle row the width.
    per_cell = 2.0 * thermalmap.OUTER_MM / cells
    per_line = 4.0 * thermalmap.OUTER_MM / (2 * len(art))
    worst = 0.0
    spans = []
    for r, row in enumerate(art):
        lit = [c for c, ch in enumerate(row) if ch != ' ']
        spans.append((lit[-1] - lit[0] + 1) if lit else 0)
        for c in lit:
            x = (c - (cells - 1) / 2.0) * per_cell
            y = ((len(art) - 1) / 2.0 - r) * per_line
            worst = max(worst, math.hypot(x, y))
    report.check('nothing is lit past the rim',
                 worst <= thermalmap.OUTER_MM + per_line, '%.1f mm' % worst)
    report.check('and it is round: narrow at the top, the full width in '
                 'the middle',
                 spans[0] < cells // 2 and max(spans) >= cells - 2,
                 'spans %d .. %d of %d' % (spans[0], max(spans), cells))

    hot = picture(dict(warm, mcu=95.0))
    report.check('a hotter MCU is a denser halftone',
                 dots(strip(hot)) > dots(strip(said)) + 40,
                 '%d dots against %d' % (dots(strip(hot)),
                                         dots(strip(said))))
    from terminal.views import show_thermal_observer as page
    middle = picture(dict(warm, board=95.0))
    drawn = [page.picture({'nodes': dict(warm, board=c)}, False, 20) for c in (40.0, 95.0)]
    report.check('a hotter centre patch is a denser halftone, on the page\'s own map too',
                 dots(strip(middle)) > dots(strip(said)) + 40 and drawn[0] != drawn[1],
                 '%d dots against %d' % (dots(strip(middle)), dots(strip(said))))
    report.check('the ramp is blended to 24 bits on the field',
                 '38;2;' in said)
    report.check('the frames and the labels wear the mark ink',
                 ansi.code(thermalmap.MARK_INK) in said)

    # Frames, not areas (the bench's word): a marked cell draws the line's
    # dots alone, so no marked cell is solid and the marks are a thin share of
    # the board.
    lit = [(ch, fg) for row in ansi.parse(said) for ch, fg, _bg in row
           if 0x2800 <= ord(ch) < 0x2900]
    marked = [ch for ch, fg in lit if fg == thermalmap.MARK_INK]
    # A solid marked cell is two frames' sides sharing a cell column - REG's
    # right and the MCU's left are a millimetre apart - and nothing else: a
    # handful, never an area.
    solid = sum(dots(ch) == 8 for ch in marked)
    report.check('a marked cell is a line, never a solid block',
                 marked and solid <= 0.02 * len(marked),
                 '%d solid of %d' % (solid, len(marked)))
    # A quarter: the rim alone is two fifths of the marks, and eight frames'
    # perimeters the rest - lines, however many of them.
    report.check('and the frames are a thin share of the board',
                 0 < len(marked) < 0.25 * len(lit),
                 '%d marked of %d lit' % (len(marked), len(lit)))
    # Right angles: the frames are box-drawing in braille, the bench's own
    # glyphs - corners, and straight runs between them.
    corners = {pair: sum(marked.count(ch) for ch in pair)
               for pair in ('⡖⢰', '⢲⡆', '⠧⠸', '⠼⠇')}
    runs = {ch: marked.count(ch) for ch in '⠒⠤⡇⢸'}
    report.check('the frames have right-angled corners - '
                 '⡖⠒⠒⢲ over ⠧⠤⠤⠼, or ⢰ ⡆ ⠸ ⠇ where the side is in the '
                 'inner lane',
                 all(n >= 5 for n in corners.values()), str(corners))
    report.check('and straight sides between them',
                 all(n >= 10 for n in runs.values()), str(runs))

    rail = [row[cells + 2:cells + 4] for row in rows if len(row) > cells + 3]
    report.check('the scale beside it is braille too, denser at the hot '
                 'end than the cold',
                 rail and all(0x2800 <= ord(ch) < 0x2900
                              for cell in rail for ch in cell)
                 and dots(rail[0]) > dots(rail[-1]),
                 '%s .. %s' % (rail[:1], rail[-1:]))
    cold = ansi.thermal_rgb(ansi.THERMAL_MIN)
    report.check('and the cold end of the scale is blue, not black',
                 cold[2] >= 128 and cold[0] == 0 and cold[1] == 0, str(cold))

    plain = thermalmap.render(warm, board_c=30.0, cells=40, colour=False,
                              reserve=0, trailing=0).split('\n')
    report.check('a pipe still gets the character ramp',
                 all(ch in thermalmap.RAMP + ' ' for ch in plain[0][:80])
                 and any(ch in thermalmap.RAMP for row in plain
                         for ch in row[:80]),
                 plain[len(plain) // 2][:80])


def test_the_thermal_page_shows_its_evidence(report):
    """Under the board a bar of the span the model has earned - red to
    yellow to green as it fills - with the innovation and the margin; and
    SENSE one fact a row.
    """
    import re
    from rich.console import Console

    from coaxial.draw import cross_section
    from coaxial.simulated.thermal.observer import SimulatedThermal
    from terminal.ui.screen import plain as visible
    from terminal.views import show_thermal_observer as page
    from terminal.views.thermal import boxes
    from terminal.ui import scroll, stage

    def ident(margin, state='CONVERGING'):
        return {'state': state, 'margin': margin, 'margin_floor': 0.8,
                'innovation_k': 0.17,
                'scales': {'air': 1.64, 'capacity': 0.95, 'spread': 1.0,
                           'ntc': 1.0},
                'sigma': {'air': 0.28, 'capacity': 0.10, 'spread': 0.5,
                          'ntc': 0.3},
                'online': ['air', 'capacity'], 'ambient': 24.5,
                'ambient_sigma': 4.2, 'updates': 3, 'saves': 0,
                'since_save_s': None,
                'truth': {'situation': 'box', 'air': 2.0, 'capacity': 1.0,
                          'ambient': 25.0, 'since_s': 240.0, 'load_a': 18.0,
                          'asked_a': 30.0}}

    rows = page.evidence_rows(ident(0.8))
    said = visible(rows[1])
    # The bar alone (the bench: "remove the text to the right of the scale,
    # move it to HEADROOM"); its figures are `envelope_rows`.
    report.check('two rows under the board: a blank, then TH OBS and the '
                 'bar alone - its figures are HEADROOM\'s',
                 len(rows) == 2 and rows[0] == ''
                 and said.startswith('   TH OBS ') and len(said.split()) == 3
                 and boxes.envelope_rows(ident(0.8)) == [
                     ('margin', '0.80'), ('floor', '0.80'),
                     ('innovation', '0.17 K'), ('doubt', 'air 0.37')],
                 '%s | %s' % (said, boxes.envelope_rows(ident(0.8))))
    capped = [boxes.envelope_rows(dict(ident(margin), trip_cap=cap))[0][1]
              for margin, cap in ((0.7004, 0.7012), (0.70, 0.75), (0.91, 1.0))]
    report.check('the margin is called the trip\'s cap within a trim step and a half of it',
                 capped == ['0.70  the trip cap', '0.70', '0.91'], str(capped))

    def bar_of(margin):
        return visible(page.evidence_rows(ident(margin))[1]).split()[2]

    inks, greys = {}, {}
    for margin, cls in ((0.8, None), (0.9, cross_section.SOA_WARN),
                        (1.0, cross_section.SOA_OK)):
        row = page.evidence_rows(ident(margin))[1]
        head, _bar = row.split('TH OBS')[0], row.split('TH OBS')[1]
        # The label constant in the leaders' grey (the bench: "only the
        # thermometer changes colour"), the bar's ink after it.
        greys[margin] = ('38;5;%dm' % cross_section.LEADER_GREY) in head
        inks[margin] = (cls is None or ('38;5;%dm' % cross_section.INK[cls])
                        in row.split('TH OBS')[1])
    report.check('empty at the floor - the tip and the track alone - half '
                 'full at 0.90 in yellow, full at the whole span in green: '
                 'red to yellow to green as it fills, and TH OBS in the '
                 'leaders\' grey whatever the bar wears',
                 all(inks.values()) and all(greys.values())
                 and bar_of(0.8).count('⣿') == 0
                 and 9 <= bar_of(0.9).count('⣿') <= 10
                 and bar_of(1.0).count('⣿') == page.GAUGE_CELLS - 1
                 and page.PAGE_CYCLE_ON_S < SimulatedThermal.CYCLE_ON_S
                 and page.PAGE_CYCLE_OFF_S < SimulatedThermal.CYCLE_OFF_S,
                 '%s %s | %s %s %s' % (inks, greys, bar_of(0.8),
                                       bar_of(0.9), bar_of(1.0)))
    report.check('and a dash before the board has answered op 10',
                 'TH OBS -' in visible(page.evidence_rows(None)[1]),
                 visible(page.evidence_rows(None)[1]))
    # The room's hint, on the estimated room: the bench's emoji pairs, and the
    # thermometer thinking while the innovation is large.
    import unicodedata

    def at(room, innovation=0.1):
        return boxes.room_hint({'ambient': room, 'innovation_k': innovation})

    report.check('the hint above the board shivers under 5 C, is mild to '
                 '35 and sweats from there - on the ESTIMATED room - thinks '
                 'while the innovation is three floors or more, and is '
                 'nothing before the board has said a room',
                 at(-25.0) == 'cold' and at(20.0) == 'mild'
                 and at(34.9) == 'mild' and at(45.0) == 'hot'
                 and at(20.0, 0.3) == 'unsure' and at(-25.0, 2.0) == 'unsure'
                 and boxes.room_hint(None) == '' and boxes.room_hint({}) == ''
                 and boxes.ROOM_HINTS['unsure'] == '🤒 🤔'
                 and all(' ' in pair for pair in boxes.ROOM_HINTS.values())
                 # Every glyph wide on its own, no variation selector: a narrow
                 # character made emoji by one ran the row a cell long and
                 # broke the frame beside it.
                 and all(len(pair) == 3 and all(
                     unicodedata.east_asian_width(ch) == 'W'
                     for ch in pair.replace(' ', ''))
                     for pair in boxes.ROOM_HINTS.values()),
                 ' '.join(boxes.ROOM_HINTS[at(c)] for c in (-25.0, 20.0, 45.0)))
    # Hysteresis (the bench: "so the emoji do not flutter near the limits"): a
    # held word stands two kelvin past its threshold, and the thermometer
    # stands until the innovation is under 0.2 K.
    def held(room, word, innovation=0.1):
        return boxes.room_hint({'ambient': room, 'innovation_k': innovation},
                               held=word)

    report.check('a held word stands two kelvin past its threshold - cold '
                 'at 6.5, hot at 33.5, mild at 3.5 and 36.5 - and lets go '
                 'beyond that, and the thermometer stands until the '
                 'innovation is under 0.2 K',
                 held(6.5, 'cold') == 'cold' and held(7.5, 'cold') == 'mild'
                 and held(33.5, 'hot') == 'hot' and held(32.5, 'hot') == 'mild'
                 and held(3.5, 'mild') == 'mild' and held(36.5, 'mild') == 'mild'
                 and held(2.5, 'mild') == 'cold' and held(37.5, 'mild') == 'hot'
                 and held(20.0, 'unsure', 0.25) == 'unsure'
                 and held(20.0, 'unsure', 0.15) == 'mild'
                 and held(20.0, 'mild', 0.25) == 'mild',
                 ' '.join(held(c, 'cold') for c in (6.5, 7.5)))
    # In SENSE, beside the room (the bench: "maybe move the emojis to the
    # SENSE block on the right, a bit more uniform").
    rows = boxes.ident_rows(ident(0.91))
    room = [value for label, value in rows if label == 'room'][0]
    report.check('and the hint sits in SENSE beside the room, the pair '
                 'after the figure',
                 room.plain.startswith('24.5 ±4.2 C')
                 and room.plain.endswith(boxes.ROOM_HINTS['mild']),
                 room.plain)

    # SENSE: one fact a row.
    rows = boxes.ident_rows(ident(0.91))
    texts = [(str(label), value if isinstance(value, str) else value.plain)
             for label, value in rows]
    # `sim`, not `truth` - the bench: only in simulated mode is the thermal
    # situation known.
    off = dict(ident(0.91)['truth'], load_a=0.0, asked_a=0.0)
    report.check('SENSE carries the identification one fact a row - model, '
                 'air, cap, room, the simulation in two and the load, let '
                 'through of asked, idle where none is',
                 [l for l, _v in texts] == ['model', 'air', 'cap', 'room',
                                            'sim', '', 'load']
                 and texts[-1][1] == '18 of 30 A rms'
                 and boxes.ident_rows(dict(ident(0.91), truth=off))[-1] == ('load', 'idle'),
                 texts)
    state = {'nodes': {}, 'ntc': 59.8, 'error': 0.54, 'seconds': 240,
             'settled': True, 'seen_s_ago': 4.0, 'sample_every_s': 30.0,
             'mcu': 72.0, 'afe': 40.0, 'ambient': 25.0}
    budget = {'worst': 0.42, 'worst_node': 'phase_v',
              'seconds_to_limit': 12.0, 'throttling': False,
              'tripped': False, 'used': {}}
    console = Console(record=True, width=scroll.HUD_WIDTH,
                      force_terminal=True, color_system='truecolor',
                      theme=stage.THEME)
    # The map's letters explained: a box of its own under SENSE, each row the
    # mark's references off the pick and place and what they are.
    from coaxial.draw.thermalmap import MARKS
    rows = dict(boxes.map_rows())
    report.check('MAP says what every mark is - U, V, W, REG, MCU, HS, AFE '
                 'and NTC - with the references its frame is drawn round',
                 [label for label, _r, _w, _m in MARKS]
                 == ['MCU', 'REG', 'U', 'V', 'W', 'AFE', 'HS', 'NTC']
                 and all(label in rows for label in ('U', 'V', 'W', 'REG',
                                                     'MCU', 'HS', 'AFE'))
                 and rows['U'].startswith('Q1U Q2U RU1 RU2 - FETs')
                 and 'hot swap' in rows['HS'] and 'STM32' in rows['MCU']
                 and rows['AFE'].startswith('OP1U..OP2W (6)')
                 # three cells of label, a space, the value, inside the panel's
                 # frame and padding
                 and all(len(value) <= page.PANEL_W - 8
                         for value in rows.values()),
                 rows)
    # At the column's own width, in its states: a reading, none yet, a cold room, a limit.
    cold = dict(ident(0.91), ambient=-25.0, truth=dict(ident(0.91)['truth'], ambient=-25.0))
    waiting = dict(state, ntc=None, mcu=None, afe=None, seconds=2, seen_s_ago=0.0)
    spent = dict(budget, worst=1.0, seconds_to_limit=None)
    sense, headroom = boxes.status_boxes(state, budget, ident=ident(0.91))[:3:2]
    for box in (sense, headroom, boxes.status_boxes(waiting, spent, ident=cold)[0],
                boxes.status_boxes(waiting, spent, ident=cold)[2],
                boxes.status_boxes(dict(state, ntc=None), dict(budget, seconds_to_limit=None),
                                   afe=False)[2]):
        console.print(box)
    said = re.sub('\x1b\\[[0-9;]*m', '', console.export_text(styles=True))
    lines = [l for l in said.splitlines()]
    report.check('drawn at the column\'s width nothing is cropped: the NTC, the observer\'s '
                 'run, the sample interval and the last sample rows of their own, no error '
                 'of a reading a sample old; HEADROOM\'s margin, floor and innovation',
                 '…' not in said
                 and any('NTC 59.8 C' in l for l in lines)
                 and not any(' err ' in l for l in lines)
                 and any('run 240 s' in l for l in lines)
                 and any('sample 30 s' in l for l in lines)
                 and any('last 4 s ago' in l for l in lines)
                 and any('margin 0.91' in l for l in lines)
                 and any('floor 0.80' in l for l in lines)
                 and any('innovation 0.17 K' in l for l in lines)
                 and said.find('soak') < said.find('margin 0.91'),
                 said)
    report.check('no thermometer yet: `last` a dash, the first sample\'s seconds said; the '
                 'worst node at its ceiling at the limit, under it not heating',
                 any('open loop, sample in 28 s' in l for l in lines)
                 and any(l.split()[1:3] == ['last', '-'] for l in lines if 'last' in l)
                 and any('to limit at the limit' in l for l in lines)
                 and any('to limit not heating' in l for l in lines),
                 [l for l in lines if 'last' in l or 'limit' in l or 'loop' in l])


def test_the_headroom_box_carries_a_solid_bar_with_a_tip(report):
    """The thermal observer's spend is HEADROOM, its level one row of `⣿`
    ending in an orange `⡇` or `⢸`, labelled `soak` (the bench's word): not
    BUDGET's `[⣿⣿⠒⠒] 42 %`, nor three rows of braille.
    """
    import re
    from rich.console import Console

    from coaxial.draw import cross_section, gauges
    from machine import ansi
    from terminal.views.thermal import boxes
    from terminal.ui import scroll, stage

    half = gauges.bar(0.5, 16)
    line = re.sub('\x1b\\[[0-9;]*m', '', half)
    report.check('a bar is one row of sixteen cells',
                 len(line) == 16, str(len(line)))
    report.check('solid ⣿ to half way, then the tip in the lane the '
                 'level ends in - ⡇ - then the track, a grey column the '
                 'cell\'s full height in every cell',
                 line[:8] == '⣿' * 8 and line[8] == '⡇'
                 and line[9:] == '⡇' * 7, line)
    report.check('the tip is orange and the track is the track\'s grey',
                 '38;5;%dm' % ansi.AMBER in half
                 and '38;5;%dm' % cross_section.INK[cross_section.TRACK] in half,
                 half.replace(chr(27), '^'))
    odd = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(17.0 / 32.0, 16))
    report.check('a level ending in the other lane tips with ⢸, the '
                 'tip\'s cell holding the tip alone',
                 odd[:8] == '⣿' * 8 and odd[8] == '⢸', odd)
    empty = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(0.0, 16))
    full = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(1.0, 16))
    report.check('nothing spent is a tip at the start of a full-height '
                 'track; everything, a solid row to a tip at the end',
                 empty == '⡇' * 16
                 and full[:15] == '⣿' * 15 and full[15] == '⢸',
                 '%s | %s' % (empty, full))

    state = {'nodes': {}, 'ntc': None, 'seconds': 3, 'settled': False,
             'seen_s_ago': None, 'sample_every_s': 5.0, 'mcu': 40.0,
             'afe': None, 'ambient': 25.0}
    budget = {'worst': 0.42, 'worst_node': 'phase_v',
              'seconds_to_limit': 12.0, 'throttling': False,
              'tripped': False, 'used': {}}
    console = Console(record=True, width=scroll.HUD_WIDTH, force_terminal=True,
                      color_system='truecolor', theme=stage.THEME)
    console.print(boxes.status_boxes(state, budget)[2])   # SENSE, MAP, then HEADROOM
    said = re.sub('\x1b\\[[0-9;]*m', '', console.export_text(styles=True))
    report.check('the box is HEADROOM, and the level is labelled soak',
                 'HEADROOM' in said and 'soak' in said and 'BUDGET' not in said,
                 said)
    braille_rows = [l for l in said.splitlines()
                    if any(0x2800 <= ord(ch) < 0x2900 for ch in l)]
    report.check('one braille row, no brackets, the figure beside it',
                 len(braille_rows) == 1 and '[' not in said
                 and '42 %' in said and '⣿' in said, said)


def test_the_thermal_load_heats_the_legs_together(report):
    """THERMAL OBSERVER on the stand-in: the page's load is 30 A a phase on all three legs, as a
    turning motor's is - the legs rise and cool together. The demo motor's held vector put in
    its place (569ae47) carried the current a leg at a time, and the hottest leg changed 9
    times in 16 s (2026-09-28).
    """
    from terminal.views import show_thermal_observer as view
    from tools.render import page

    legs = ('driver_u', 'driver_v', 'driver_w')
    seen, real = [], view.status_boxes

    def boxes(state, *a, **k):
        nodes = state.get('nodes') or {}
        if all(nodes.get(n) is not None for n in legs):
            seen.append([nodes[n] for n in legs])
        return real(state, *a, **k)

    view.status_boxes = boxes
    try:
        page.frame('thermal_observer', 150, 44, frames=300)
    finally:
        view.status_boxes = real
    lead, changes, spread = None, 0, 0.0
    for temps in seen:
        hot = temps.index(max(temps))
        if lead is not None and hot != lead and max(temps) - sorted(temps)[1] > 0.5:
            changes += 1
        lead = hot
        rise = [t - t0 for t, t0 in zip(temps, seen[0])]
        if max(rise) > 5.0:
            spread = max(spread, (max(rise) - min(rise)) / max(rise))
    report.check('the legs heat together: the hottest never changes', seen and changes == 0,
                 '%d changes in %d frames' % (changes, len(seen)))
    report.check('and stay within a fifth of their rise of each other', spread <= 0.20,
                 '%.0f %%' % (100.0 * spread))


ROSTER = (test_the_thermal_map_is_a_halftone_with_its_parts_marked,
          test_the_thermal_page_shows_its_evidence,
          test_the_headroom_box_carries_a_solid_bar_with_a_tip,
          test_the_thermal_load_heats_the_legs_together)

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
