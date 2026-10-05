"""The QUAD page on four stand-in boards: their air, its words, its traces, its flights."""
import math
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report

from test_quad import spans


def test_the_boards_air_is_the_rotors(report):
    """A stand-in board armed as the page arms it, its 63100 at a hover's speed: the rpm its
    thermal network's air sees is the rotor's own, by the record's pole pairs."""
    from coaxial import Coaxial63100
    from machine import quad
    from machine.modes import SIMULATED
    from terminal.views import show_quad as view
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        rotor = view.arm(rig)
        drive, w = rig.board.drive, 0.0
        hover = quad.speed_for(quad.MASS_KG * quad.GRAVITY / 4.0)
        began = time.monotonic()
        while time.monotonic() - began < 3.0:
            now = drive.state()
            w_hat = (now.get('omega_hat') or 0.0) / rotor['pairs']
            w = drive.model.read()['omega'] / rotor['pairs']
            drive.write(iq_ref=rotor['pi'].step(0.01, setpoint=hover, measured=w_hat)['command'])
            drive.model.configure(load=quad.K_DRAG * w * abs(w))
            time.sleep(0.01)
        air = rig.board.thermal.state().get('speed_rpm') or 0.0
        link = drive.state().get('vdc')
    finally:
        rig.board.drive.off()
        rig.gates.off()
        rig.close()
    rpm = w * 60.0 / math.tau
    report.check('the air sees the rotor\'s own rpm at a hover, within 3 %',
                 rpm > 1000.0 and abs(air - rpm) <= 0.03 * rpm,
                 'the rotor %.0f rpm, the air %.0f' % (rpm, air))
    report.check('its link is the pack\'s, 63 V full, and its drive says so',
                 link is not None and abs(link - quad.open_volts(1.0)) < 0.01, '%s V' % link)


def test_the_page_words_its_boards(report):
    """The TH OBS row - who is stable, who converges - and the envelopes' share: the least room
    under a throttle's point of a flight's spend, a throttling board's derate, taken and given
    back over their seconds."""
    from coaxial.devices.thermal import THROTTLE_AT
    from terminal.views import show_quad as view

    def rotors(*states):
        return [{'ident': {'state': s}, 'budget': {}} for s in states]
    cases = ((('STABLE',) * 4, '4 of 4 stable'),
             (('STABLE', 'CONVERGING', 'CONVERGING', 'CONVERGING'), '1 stable, 3 converging'),
             (('CONVERGING',) * 4, '0 stable, 4 converging'),
             (('CONVERGING', 'CONVERGING', 'UNCERTAIN', 'UNCERTAIN'), '0 stable, 2 converging of 4'))
    got = [view.observers(rotors(*states)) for states, _want in cases]
    report.check('four stable say so, a mix its counts',
                 got == [want for _s, want in cases], str(got))

    def boards(*budgets):
        return [{'budget': b} for b in budgets]
    under = THROTTLE_AT - view.UNDER
    cool, warm = {'worst': under - view.SPEND}, {'worst': under - 0.5 * view.SPEND}
    hot, cut = {'worst': under}, {'worst': 0.2, 'derate': 0.5}
    held = [view.envelope(boards(cool, cool, cool, b), was, 10.0) for b, was in (
        (cool, 1.0), (warm, 1.0), (hot, 1.0), (cut, 1.0), ({}, 1.0))]
    report.check('the whole of their pull with a flight\'s spend of room, half of it with half, '
                 'none %.2f under the throttle; a board\'s own derate' % view.UNDER,
                 all(abs(a - b) < 1e-9 for a, b in zip(held, (1.0, 0.5, 0.0, 0.5, 1.0))),
                 str(['%.2f' % s for s in held]))
    step = 0.1
    report.check('taken over %.1f s and given back over %.1f' % (view.TAKEN_S, view.RECOVER_S),
                 abs(view.envelope(boards(hot), 1.0, step) - (1.0 - step / view.TAKEN_S)) < 1e-9
                 and abs(view.envelope(boards(cool), 0.0, step) - step / view.RECOVER_S) < 1e-9,
                 '%.2f after %.1f s at the throttle, %.2f after as long cool' % (
                     view.envelope(boards(hot), 1.0, step), step,
                     view.envelope(boards(cool), 0.0, step)))


def test_its_traces_are_drawn(report):
    """The side column's two plots: the height with the power on its right, the bus with the
    hottest board on its right - each curve's scale on its own side, in its own ink; a pass at
    1.9 kW among the power's hundreds a spike to the plot's top."""
    from terminal.views.quad import traces
    trace = [(0.1 * k, 1.5 + 0.2 * k, 63.0 - 0.02 * k, 1900.0 if k == 75 else 150.0 + k,
              60.0 + 0.3 * k) for k in range(150)]
    high, low = traces.heights(trace, 15.0, 36), traces.buses(trace, 15.0, 36)
    plain = [''.join(c for c in line if 0x2800 <= ord(c) <= 0x28FF) for line in high + low]
    report.check('eight rows of height and power, four of bus and temperature, 36 cells wide',
                 len(high) == traces.HEIGHT_ROWS and len(low) == traces.BUS_ROWS
                 and len({len(p) for p in plain}) == 1 and len(plain[0]) == 36 - 15,
                 '%d and %d rows, %s cells of plot' % (len(high), len(low),
                                                      sorted({len(p) for p in plain})))
    text = '\n'.join(high + low)
    report.check('each curve in its own ink, its scale on its own side in that ink',
                 all(ink in text for ink in (traces.HEIGHT, traces.POWER, traces.BUS, traces.TEMP))
                 and high[0].startswith(traces.HEIGHT) and traces.POWER + '├ 2kW' in high[0]
                 and low[0].startswith(traces.BUS) and '63 V' in low[0]
                 and traces.TEMP + '├ 105 C' in low[0],
                 '%r | %r' % (high[0][-12:], low[0][-12:]))
    spike = [sum(c != chr(0x2800) for c in p) for p in plain[:traces.HEIGHT_ROWS]]
    flat = traces.heights([row[:3] + (150.0 + k,) + row[4:] for k, row in enumerate(trace)],
                          15.0, 36)
    report.check('the pass at 1.9 kW a spike up the plot, none without it',
                 spike[1] >= 1 and traces.POWER + '├ 2kW' in high[0]
                 and traces.POWER + '├ 1kW' in flat[0],
                 'dots a row, top down: %s' % spike)


#: The page's flights drawn this long at most, s.
LANDED_S = 200.0

#: The page's pack for its test, A h: a first flight whole, spent early in the second.
TEST_PACK_AH = 0.075


def test_the_page_flies_four_boards(report):
    """The page on its four stand-in boards and a small pack, into its third flight: the quad
    drawn in the viewport; flown from its first hover, no observer STABLE yet, inside the
    boards' envelopes - none tripped, the pull cut under full tilt; full tilt past 20 m, the
    fall burned to a
    stop over the floor and held at 10 cm; the bus drooping under it; the pack spent in the
    second flight, the way down, a charged one on the floor and its routine again - the
    observers kept through it all."""
    from coaxial.devices.thermal import THROTTLE_AT
    from machine import aerobatics, quad
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_quad as view
    from tools.render import page

    rows, real, pack_ah = [], view.compose, quad.PACK_AH
    first = aerobatics.CARD[[name for name, *_row in aerobatics.CARD].index('hover') + 1][0]

    class Again(Exception):
        """The flight after the pack's change in its routine."""

    def compose(console, origin, rotors, frame, flight, trace, now, art):
        out = real(console, origin, rotors, frame, flight, trace, now, art)
        cells = flight['cells']
        rows.append({'t': time.monotonic(), 'name': flight['stage'], 'h': frame['h'],
                     'amps': max(r['amps'] for r in rotors),
                     'soa': max((r['budget'] or {}).get('worst') or 0.0 for r in rotors),
                     'states': [(r['ident'] or {}).get('state') for r in rotors],
                     'cells': sum(0x2800 < ord(c) <= 0x28FF for c in art),
                     'tripped': any((r['budget'] or {}).get('tripped') for r in rotors),
                     'share': flight['share'], 'volts': cells['volts'], 'watts': cells['watts'],
                     'left': cells['left']})
        if flight['stage'] == first and any(r['name'] == 'swap' for r in rows):
            raise Again
        return out
    view.compose, quad.PACK_AH = compose, TEST_PACK_AH
    # Drawn into the flight after the pack's change, LANDED_S at most: drawn 48 s, on CI's
    # host, its boards' observers slower, the first hold was still coming down as the frames
    # ran out (2026-09-28).
    try:
        page.frame('quad', 150, 44, frames=int(LANDED_S * FPS_CAP))
    except Again:
        pass
    finally:
        view.compose, quad.PACK_AH = real, pack_ah
    rate = (len(rows) - 1) / max(1e-9, rows[-1]['t'] - rows[0]['t']) if len(rows) > 1 else 0.0
    if rate < 4.0:
        report.skip('the flight', 'the page drew %.1f frames a second' % rate)
        return
    report.check('the quad drawn in the viewport, every frame',
                 min(r['cells'] for r in rows) >= 150, '%d braille cells at the least'
                 % min(r['cells'] for r in rows))
    early = [r for r in rows if r['name'] == first][:1]
    # On a host as loaded as the gate's the boards' clocks outrun the flight's and a share
    # stood at 0.92 for a read (2026-10-05): what must hold is that none trips.
    hard = [r for r in rows if r['name'] in ('full tilt', 'burn')]
    report.check('flown from its first hover, no observer STABLE yet; no board tripped through '
                 'it all, the envelopes cutting the rotors\' pull under full tilt and the burn',
                 bool(early) and 'STABLE' not in early[0]['states']
                 and not any(r['tripped'] for r in rows)
                 and bool(hard) and min(r['share'] for r in hard) < 1.0
                 and max(r['soa'] for r in hard) > THROTTLE_AT - view.UNDER - view.SPEND,
                 'the %s at %.1f s on %s; SOA %.2f at most, %.0f %% of their pull at the least' % (
                     first, early[0]['t'] - rows[0]['t'], early[0]['states'],
                     max(r['soa'] for r in rows), 100.0 * min((r['share'] for r in hard),
                                                              default=math.nan))
                 if early else 'no %s' % first)
    # Out of UNCERTAIN it stays out: the landing and the pack's change leave the observers be.
    known = [sum(s in view.GO for s in r['states']) for r in rows]
    report.check('the observers kept through the landings and the pack\'s change',
                 max(known) == 4 and all(b >= a for a, b in zip(known, known[1:])),
                 '%d known at the end, fewer after more %d times' % (
                     known[-1], sum(b < a for a, b in zip(known, known[1:]))))
    tilt = [r for r in rows if r['name'] == 'full tilt']
    held = [r['h'] for r in rows if r['name'] == 'hold']
    low = min((r['h'] for r in rows if r['name'] in ('burn', 'hold')), default=math.nan)
    report.check('full tilt past 20 m',
                 bool(tilt) and max(r['h'] for r in rows) >= 20.0,
                 '%.1f A at most, %.1f m' % (max((r['amps'] for r in tilt), default=0.0),
                                            max(r['h'] for r in rows)))
    report.check('the fall burned to a stop over the floor, held at 10 cm',
                 low >= 0.05 and bool(held) and abs(held[-1] - quad.FLOOR_M) <= 0.05,
                 'lowest %.3f m, held at last %.3f m' % (low, held[-1] if held else math.nan))
    sag = min((r['volts'] for r in tilt), default=math.nan)
    # Its reads' greatest: CI's runner caught 880-999 W of the kilowatt in three gates of four
    # (2026-10-06), this host 1 kW and over; and its first frame there comes with the rotors
    # already drawing, the bus more than 0.2 V under its 63 - within 1.5 V of them.
    report.check('the bus droops a volt and more under full tilt\'s kilowatt, 63 V at its start',
                 -1.5 < rows[0]['volts'] - quad.open_volts(1.0) < 0.2
                 and bool(tilt) and max(r['watts'] for r in tilt) >= 850.0
                 and sag <= quad.open_volts(tilt[0]['left']) - 1.0,
                 '%.1f V under %.0f W, %.1f open; %.1f V at the first frame' % (
                     sag, max((r['watts'] for r in tilt), default=math.nan),
                     quad.open_volts(tilt[0]['left']) if tilt else math.nan, rows[0]['volts']))
    names = [name for name, _rows in spans(rows)]
    swap = names.index('swap') if 'swap' in names else len(names)
    spent = next((r for r in rows if r['left'] <= quad.RESERVE), None)
    report.check('its pack spent in the air, the way down, a charged one on the floor, and its '
                 'routine again',
                 spent is not None and spent['h'] > 1.0
                 and names[swap - 2:swap] == ['descend', 'land'] and rows[-1]['name'] == first
                 and rows[-1]['left'] > 0.9,
                 'spent at %.1f m in the %s; %s; %.0f %% in it at the end' % (
                     spent['h'], spent['name'], ' '.join(names[max(0, swap - 3):swap + 2]),
                     100.0 * rows[-1]['left']) if spent else 'never spent: %.0f %% left' % (
                         100.0 * rows[-1]['left']))
    report.check('and the boards\' thermal observers spend their SOA',
                 max(r['soa'] for r in rows) >= 0.15, '%.2f at most' % max(r['soa'] for r in rows))


ROSTER = (test_the_boards_air_is_the_rotors, test_the_page_words_its_boards,
          test_its_traces_are_drawn, test_the_page_flies_four_boards)


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
