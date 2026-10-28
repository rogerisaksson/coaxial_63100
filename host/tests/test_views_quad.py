"""The QUAD page on four stand-in boards: its words, its traces and its wind, its flights."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report, typed

from page_kit import paged

from test_quad import spans
from test_quad_course import CLEAR_M, passes, reach, worst


def test_the_page_words_its_boards(report):
    """The TH OBS row: who is stable, who converges."""
    from terminal.views.quad import flight as view

    def rotors(*states):
        return [{'ident': {'state': s}, 'budget': {}} for s in states]
    cases = ((('STABLE',) * 4, '4 of 4 stable'),
             (('STABLE', 'CONVERGING', 'CONVERGING', 'CONVERGING'), '1 stable, 3 converging'),
             (('CONVERGING',) * 4, '0 stable, 4 converging'),
             (('CONVERGING', 'CONVERGING', 'UNCERTAIN', 'UNCERTAIN'), '0 stable, 2 converging of 4'))
    got = [view.observers(rotors(*states)) for states, _want in cases]
    report.check('four stable say so, a mix its counts',
                 got == [want for _s, want in cases], str(got))


def test_its_traces_are_drawn(report):
    """The side column's two plots: the height with the power on its right, the bus with the
    hottest board on its right - each curve's scale on its own side, in its own ink; a pass at
    1.9 kW among the power's hundreds a spike to the plot's top."""
    from terminal.views.quad import traces
    trace = [(0.1 * k, 1.5 + 0.2 * k, 63.0 - 0.02 * k, 1900.0 if k == 75 else 150.0 + k,
              60.0 + 0.3 * k) for k in range(150)]
    high, low = traces.heights(trace, 15.0, 36), traces.buses(trace, 15.0, 36)
    plain = [''.join(c for c in line if 0x2800 <= ord(c) <= 0x28FF) for line in high + low]
    report.check('seven rows of height and power, four of bus and temperature, 36 cells wide',
                 len(high) == traces.HEIGHT_ROWS and len(low) == traces.BUS_ROWS
                 and len({len(p) for p in plain}) == 1 and len(plain[0]) == 36 - 15,
                 '%d and %d rows, %s cells of plot' % (len(high), len(low),
                                                      sorted({len(p) for p in plain})))
    text = '\n'.join(high + low)
    report.check('a plot\'s left curve and its right in their inks, both plots alike, each '
                 'scale on its own side in its curve\'s',
                 all(ink in text for ink in (traces.LEFT, traces.RIGHT))
                 and high[0].startswith(traces.LEFT) and traces.RIGHT + '├ 2kW' in high[0]
                 and low[0].startswith(traces.LEFT + '   63 ┤') and '   57 ┤' in low[-1]
                 and traces.RIGHT + '├ 105C' in low[0],
                 '%r | %r' % (high[0][-12:], low[0][-12:]))
    spike = [sum(c != chr(0x2800) for c in p) for p in plain[:traces.HEIGHT_ROWS]]
    flat = traces.heights([row[:3] + (150.0 + k,) + row[4:] for k, row in enumerate(trace)],
                          15.0, 36)
    report.check('the pass at 1.9 kW a spike up the plot, none without it',
                 spike[1] >= 1 and traces.RIGHT + '├ 2kW' in high[0]
                 and traces.RIGHT + '├ 1kW' in flat[0],
                 'dots a row, top down: %s' % spike)


def test_its_wind_is_pointed(report):
    """terminal.views.quad.wind: the dial's up is into the screen - a wind along the heading of
    a frame seen from behind points up, against it down; its needle the longer the stronger;
    its rows the dial and the air's kind, kept, in words."""
    from machine.aircraft import weather
    from terminal.views.quad import wind
    ways = [math.degrees(wind.seen((0.0, 0.0, z), 180.0)[0]) for z in (3.0, -3.0)]
    dots = [sum(bin(cell).count('1') for row in wind.dial(0.7, share)[1] for cell in row)
            for share in (0.1, 0.5, 1.0)]
    report.check('a tail wind seen from behind points up, a head wind down; a needle the '
                 'longer the stronger',
                 abs(ways[0]) < 1e-6 and abs(abs(ways[1]) - 180.0) < 1e-6
                 and dots[0] < dots[1] < dots[2],
                 '%.0f and %.0f degrees from up; %s dots' % (ways[0], ways[1], dots))
    air = weather.air(3, 'gusty')
    for _ in range(500):
        weather.blown(air, 0.02)
    lines = wind.pointer(air, 5.0, 30.0)
    report.check('%d rows of %d cells of dial, the kind and its wind beside it' % (
        wind.ROWS, 2 * wind.ROWS),
        [sum(0x2800 <= ord(c) <= 0x28FF for c in line) for line in lines]
        == [2 * wind.ROWS] * wind.ROWS
        and 'GUSTY' in lines[0] and 'kept' in lines[1] and 'm/s' in lines[2],
        lines[2].split('  ')[-1])


#: A flight is judged where its clock - its steps' sum - kept within this of the wall's: a
#: page starved past it has not landed in LANDED_S (page_kit).
BEHIND = 0.2

#: The page's pack for its test, A h: a first flight whole, spent early in the second - on the
#: flight's own seconds (clock=1) the routine takes 0.152 on the 16x14s; 0.19 was spent in its descend.
TEST_PACK_AH = 0.21


def test_the_page_flies_four_boards(report):
    """The page on its four stand-in boards and a small pack, into its third flight: the quad
    drawn in the viewport; flown from its first hover, no observer STABLE yet, inside the
    boards' envelopes - none tripped, full tilt into their throttle; full tilt past 20 m, the
    fall burned to a stop over the floor and held at 10 cm; the bus drooping under it; the pack
    spent in the second flight, the way down, a charged one on the floor and its routine again
    - the observers kept through it all."""
    from machine.aircraft import aerobatics, flightpack, quad
    from terminal.views.quad import flight as flown
    first = aerobatics.CARD[[name for name, *_row in aerobatics.CARD].index('hover') + 1][0]
    # The routine alone, flight after flight, into the one after the pack's change.
    rows, frames, rate = paged(aerobatics.CARD, lambda flew, now: now == first and 'swap' in flew,
                               TEST_PACK_AH)
    behind = (1.0 - sum(r['dt'] for r in rows[1:]) / max(1e-9, rows[-1]['t'] - rows[0]['t'])
              if len(rows) > 1 else 0.0)
    if rate < 4.0 or behind > BEHIND:
        report.skip('the flight', "the page drew %.1f frames a second, its flight's clock "
                    "%.0f %% behind the wall's" % (rate, 100.0 * behind))
        return
    # Its spectator's frame of the sky holds the quad alone, a quarter of its height: 66 braille
    # cells in its flips, rolls and corkscrew, where the ground made 1 700-2 200.
    report.check('the quad drawn in the viewport, every frame',
                 min(f['cells'] for f in frames) >= 50, '%d braille cells at the least'
                 % min(f['cells'] for f in frames))
    early = [r for r in rows if r['name'] == first][:1]
    hard = [r for r in rows if r['name'] in ('full tilt', 'burn')]
    report.check('flown from its first hover, no observer STABLE yet; no board tripped and '
                 'nothing struck, full tilt and the burn into the boards\' throttle at most',
                 bool(early) and 'STABLE' not in early[0]['states']
                 and not any(r['tripped'] or r['name'] == 'crashed' for r in rows)
                 and bool(hard) and max(r['soa'] for r in hard) <= 1.0,
                 'the %s at %.1f s on %s; SOA %.2f at most, %.0f %% of their pull at the least' % (
                     first, early[0]['t'] - rows[0]['t'], early[0]['states'],
                     max(r['soa'] for r in rows), 100.0 * min((r['share'] for r in hard),
                                                              default=math.nan))
                 if early else 'no %s' % first)
    # Out of UNCERTAIN it stays out: the landing and the pack's change leave the observers be.
    known = [sum(s in flown.GO for s in r['states']) for r in rows]
    report.check('the observers kept through the landings and the pack\'s change',
                 max(known) == 4 and all(b >= a for a, b in zip(known, known[1:])),
                 '%d known at the end, fewer after more %d times' % (
                     known[-1], sum(b < a for a, b in zip(known, known[1:]))))
    tilt = [r for r in rows if r['name'] == 'full tilt']
    held = [r['y'] for r in rows if r['name'] == 'hold']
    low = min((r['y'] for r in rows if r['name'] in ('burn', 'hold')), default=math.nan)
    report.check('full tilt past 15 m',
                 bool(tilt) and max(r['y'] for r in rows) >= 15.0,
                 '%.1f A at most, %.1f m' % (max((r['amps'] for r in tilt), default=0.0),
                                            max(r['y'] for r in rows)))
    report.check('the fall burned to a stop over the floor, held at 10 cm',
                 low >= 0.05 and bool(held) and abs(held[-1] - quad.FLOOR_M) <= 0.05,
                 'lowest %.3f m, held at last %.3f m' % (low, held[-1] if held else math.nan))
    sag = min((r['volts'] for r in tilt), default=math.nan)
    # Its start: the first two seconds' middle, the rotors' spool to their idle in it. Its
    # kilowatt a step's: 0.15 s of full tilt's two, a frame's read 814 W on CI.
    early = sorted(r['volts'] for r in rows if r['t'] - rows[0]['t'] <= 2.0)
    full = early[len(early) // 2]
    report.check('the bus droops a volt and more under full tilt\'s kilowatt, 63 V at its start',
                 -1.5 < full - flightpack.open_volts(1.0) < 0.2
                 and bool(tilt) and max(r['watts'] for r in tilt) >= 1000.0
                 and sag <= flightpack.open_volts(tilt[0]['left']) - 1.0,
                 '%.1f V at its start; %.1f V under %.0f W, %.1f open' % (
                     full, sag, max((r['watts'] for r in tilt), default=math.nan),
                     flightpack.open_volts(tilt[0]['left']) if tilt else math.nan))
    names = [name for name, _rows in spans(rows)]
    swap = names.index('swap') if 'swap' in names else len(names)
    spent = next((r for r in rows if r['left'] <= flightpack.RESERVE), None)
    report.check('its pack spent in the air, the way down, a charged one on the floor, and its '
                 'routine again',
                 spent is not None and spent['y'] > quad.FLOOR_M
                 and names[swap - 2:swap] == ['descend', 'land'] and rows[-1]['name'] == first
                 and rows[-1]['left'] > 0.9,
                 'spent at %.1f m in the %s; %s; %.0f %% in it at the end, in its %s' % (
                     spent['y'], spent['name'], ' '.join(names[max(0, swap - 3):swap + 2]),
                     100.0 * rows[-1]['left'], rows[-1]['name'])
                 if spent else 'never spent: %.0f %% left' % (100.0 * rows[-1]['left']))
    report.check('and the boards\' thermal observers spend their SOA',
                 max(r['soa'] for r in rows) >= 0.15, '%.2f at most' % max(r['soa'] for r in rows))


def test_the_page_flies_its_course(report):
    """The page's course on its four stand-in boards, from the floor and back: flown from
    behind, its gates stood in the view, the one flown to next lit; every gate passed inside
    its opening on each lap it flew; the boards to their ceiling at the most, none tripped;
    landed where it rose."""
    from machine.aircraft import course, grounds
    rows, frames, rate = paged(course.CARD, lambda flew, now: now in ('idle', 'cool', 'swap')
                               and 'land' in flew)
    laps = [r for r in rows if r['name'] == 'lap']
    behind = 1.0 - sum(r['dt'] for r in laps[1:]) / max(1e-9, laps[-1]['t'] - laps[0]['t']) \
        if len(laps) > 1 else 0.0
    if rate < 4.0 or behind > BEHIND:
        report.skip('the course', 'the page drew %.1f frames a second, its flight\'s clock '
                    '%.0f %% behind the wall\'s' % (rate, 100.0 * behind))
        return
    floor = [f['cells'] for f in frames if f['name'] in ('idle', 'cool')][:5]
    aloft = [f['cells'] for f in frames if f['name'] == 'lap']
    report.check('its world drawn about it on its laps: the gates, the trees, the houses',
                 bool(aloft) and bool(floor) and min(aloft) >= 300 and
                 sorted(aloft)[len(aloft) // 2] >= 600,
                 '%d braille cells at the least and %d at the middle on its laps' % (
                     min(aloft or [0]), sorted(aloft or [0])[len(aloft) // 2]))
    flew = laps[-1]['of'] if laps else 0
    gates = passes(rows)
    counted = [len(gates[g]) for g in range(1, len(grounds.GATES))]
    report.check('every gate passed on each lap it flew inside its opening, a propeller\'s tip '
                 '%.2f m clear of its frame, nothing struck' % CLEAR_M,
                 flew >= 1 and counted == [flew] * len(counted) and worst(gates) <= 0.0
                 and not any(r['name'] == 'crashed' for r in rows),
                 '%d of %d laps, passes %s, %+.2f m past a gate\'s room; its clock %.0f %% behind '
                 'the wall\'s' % (flew, course.LAPS, counted, worst(gates), 100.0 * behind))
    ahead = {f['gate'] for f in frames if f['name'] == 'lap'}
    after = {f['gate'] for f in frames if f['name'] in ('descend', 'land')}
    report.check('the gate lit: each in its turn on its laps, the first again on its way down',
                 ahead == set(range(len(grounds.GATES))) and after == {1},
                 '%d of them on its laps, %s after' % (len(ahead), sorted(after)))
    report.check('the boards to their ceiling at the most through its laps, raw; '
                 'none tripped',
                 bool(laps) and max(r['soa'] for r in laps) <= 1.0
                 and not any(r['tripped'] for r in rows),
                 'SOA %.2f at the most, %.0f %% of the pull at the least' % (
                     max((r['soa'] for r in laps), default=math.nan),
                     100.0 * min((r['share'] for r in laps), default=math.nan)))
    down = [r for r in rows if r['name'] == 'land']
    report.check('landed where it rose',
                 bool(down) and abs(down[-1]['y']) <= 0.01
                 and math.hypot(down[-1]['x'], down[-1]['z']) <= 0.3,
                 '%.3f m up, %.2f m off' % (down[-1]['y'], math.hypot(down[-1]['x'], down[-1]['z']))
                 if down else 'never landed')


def test_its_thrust_and_its_line_are_drawn(report):
    """The page's frame: an arrow up each rotor's axis in its own ink, the longer the more it
    pulls, none for none; the line the course asks dashed through it in its own, only where
    the page has it on (L)."""
    from coaxial.graphics import engine, quadcopter, scenery, shapes
    from coaxial.model.blocks import numpy as np
    m, reach = shapes.view(200.0, 18.0), 2.0
    cam = engine.fine(engine.camera(100, 36, reach, distance=quadcopter.SIGHT * reach))
    pose = {'at': (0.0, 2.0, 0.0), 'turn': ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))}

    def dots(newtons):
        (drawn, ink, _over), = quadcopter._arrows(pose, [newtons] * 4, m, cam, pose['at'])
        return int(drawn.sum()), ink
    none, (small, ink), (big, _ink) = dots(0.0)[0], dots(10.0), dots(40.0)
    report.check('an arrow a rotor in its own ink, the longer the more it pulls, none for none',
                 none == 0 < small < big and ink == quadcopter.ARROW_INK
                 and ink not in scenery.INKS.values(),
                 '%d dots at 10 N, %d at 40, %d at none' % (small, big, none))
    want = np.asarray(scenery.INKS['line'], float)

    def lined(on):
        (got, inks), = scenery.props(m, cam, (2.0, 2.4, 22.0), 3, on)
        size = np.linalg.norm(inks, axis=2)
        like = (inks @ want) / np.maximum(1e-9, size * np.linalg.norm(want))
        return int(((size > 0.0) & (like > 0.9995)).sum())
    report.check('the line asked through the course in its own ink where it is on, none off',
                 lined(True) >= 20 and lined(False) == 0,
                 '%d cells on, %d off' % (lined(True), lined(False)))


def test_r_records_its_flight(report):
    """R on a flying page records its flight a row a pass, the keys typed beside; R again writes
    it, A resets the flight, and while it records the drawing says REC."""
    import csv
    import tempfile
    from machine.aircraft import grounds, quad
    from terminal.ui import recorder
    from terminal.views import show_quad
    from terminal.views.quad import keys
    held: dict = {}
    rotors, sent = [{'w': 100.0}, {'w': 110.0}], []
    held.update(zoomed=lambda k: None, record=lambda: show_quad.recorded(held, 'quad', rotors),
                send=lambda **command: sent.append(command))
    frame, cells = quad.Sky(grounds.solids()).state(), {'volts': 50.0, 'watts': 10.0}
    was, recorder.RECORDINGS = recorder.RECORDINGS, tempfile.mkdtemp()
    try:
        typed(keys.PAD, 'rw', held)
        recorder.added(held['recording'], [show_quad.flown_row(0.01, frame, cells, rotors, held.pop('keyed', ''))])
        corner = ''.join(chr(c) for c, _ink in recorder.corner(held['recording'], 80).values())
        typed(keys.PAD, 'Ra', held)
        with open(held['recorded'], newline='', encoding='utf-8') as f:
            rows = list(csv.reader(f))
    finally:
        recorder.RECORDINGS = was
    report.check('R, a pass, R: the header and its row, the keys typed between in it; W keeps the '
                 'air and A resets the flight',
                 rows[0] == list(show_quad.FLIGHT) + ['w_0', 'w_1'] and len(rows) == 2
                 and rows[1][len(show_quad.FLIGHT) - 1] == 'rw' and held['recording'] is None
                 and sent == [{'wind': True}, {'reset': True}],
                 (rows[1][len(show_quad.FLIGHT) - 1:], sent))
    report.check('while it records the drawing carries REC and its rows', corner.strip() == '● REC ⠙ 1', corner)
    from terminal.ui.console import WHEEL_STEP
    zooms = []
    show_quad.wheeled(zooms.append, 2.0 * WHEEL_STEP)
    show_quad.wheeled(zooms.append, 0.0)
    report.check('two wheel notches zoom two steps in, none nothing',
                 zooms == [show_quad.ZOOM_STEP ** 2], zooms)


def test_s_undresses_the_crafts(report):
    """S steps the layers round; each craft's frame thins to its shafts, its pack shows between,
    the motors and their 63100 boards stay till bare, and every layer draws."""
    from coaxial.graphics import aerobat, helicopter, ornithopter, quadcopter
    from coaxial.graphics.lit import paint
    from terminal.views.quad import keys
    held = {'see': 0}
    typed(keys.PAD, 's', held)
    once = held['see']
    typed(keys.PAD, 'Sss', held)
    report.check('S steps the layer, four steps round', (once, held['see']) == (1, 0), (once, held['see']))
    crafts = ((quadcopter, quadcopter.mesh, 4, 'shaft0'), (helicopter, helicopter.heli_mesh, 2, 'mast'),
              (ornithopter, ornithopter.orni_mesh, 2, 'crank0'), (aerobat, aerobat.aero_mesh, 2, 'shaft0'))
    pose = {'at': (0.0, 1.5, 0.0), 'turn': ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            'controls': (0.4, 0.1, 0.05, 0.1)}
    for gfx, mesh, n, moving in crafts:
        parts = {see: set(mesh('frame', see).parts) for see in quadcopter.LAYERS}
        packed = mesh('frame', 'pack')
        cells = 'pack' in parts['pack'] or (
            packed.materials[packed.span('tray')[0]] == paint(quadcopter.CELLS))
        drives = all({'can0', 'board0', 'sink0'} <= parts[see] for see in ('shell', 'frame', 'pack'))
        bare = moving in parts['bare'] and 'can0' not in parts['bare'] and len(parts['bare']) < len(parts['frame'])
        rotors = [(0.3 * k, 420.0, 45.0, 40.0, 35.0) for k in range(n)]
        drawn = all(len(gfx.render(pose, rotors, 60, 24, world=False, lit=None, see=see)) == 24
                    for see in quadcopter.LAYERS)
        report.check('%s: the pack shows, the drives stay till bare, bare keeps %s, every layer draws'
                     % (gfx.__name__.rpartition('.')[2], moving), cells and drives and bare and drawn,
                     (cells, drives, bare, drawn, sorted(parts['bare'])))


ROSTER = (test_the_page_words_its_boards, test_its_traces_are_drawn, test_its_wind_is_pointed,
          test_its_thrust_and_its_line_are_drawn, test_the_page_flies_four_boards,
          test_the_page_flies_its_course, test_r_records_its_flight,
          test_s_undresses_the_crafts)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
