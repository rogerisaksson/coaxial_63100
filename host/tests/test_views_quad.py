"""The QUAD page: its flight (machine.quad) on ideal rotors, and the page on four stand-in boards -
the quad drawn in the viewport, full tilt into the sky once the boards' thermal observers have left
UNCERTAIN, the fall burned to a stop over the floor, the boards warm."""
import math
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


def test_the_flight_stops_at_its_mark(report):
    """machine.quad's frame in MuJoCo on rotors lagging 0.05 s to their speed, capped at the
    63100's ceiling: full tilt past 30 m, the burn and the hold never at the floor, the hold at
    10 cm, level."""
    from machine import quad
    sky, route, t, top_rad_s = quad.Sky(), quad.flight(), 0.0, 310.0
    top = 4.0 * quad.K_THRUST * top_rad_s ** 2
    w = [0.0] * 4
    apex, low, hold, tilt = 0.0, math.inf, [], 0.0
    while t < 28.0:
        state = sky.state()
        name, thrust = quad.plan(route, state, top, t)
        for k, share in enumerate(quad.mix(state, thrust)):
            w[k] += (min(top_rad_s, quad.speed_for(share)) - w[k]) * min(1.0, 0.01 / 0.05)
        sky.step(w, 0.01)
        state = sky.state()
        apex = max(apex, state['h'])
        tilt = max(tilt, math.degrees(math.acos(min(1.0, state['turn'][1][1]))))
        if name in ('burn', 'hold'):
            low = min(low, state['h'])
        if name == 'hold':
            hold.append(state['h'])
        t += 0.01
    settled = hold[-len(hold) // 3:] if hold else []
    report.check('full tilt past 30 m; the burn and the hold never lower than 3 cm, the hold '
                 'settled within 5 cm of %.0f cm, level' % (100 * quad.FLOOR_M),
                 apex >= 30.0 and low >= 0.03 and bool(settled) and tilt < 1.0
                 and all(abs(h - quad.FLOOR_M) <= 0.05 for h in settled),
                 'apex %.1f m, lowest %.3f m, settled %.3f-%.3f m, %.2f degrees at most' % (
                     apex, low, min(settled or [math.nan]), max(settled or [math.nan]), tilt))


def test_the_page_flies_four_boards(report):
    """The page on its four stand-in boards for a flight: the quad drawn in the viewport, full
    tilt only once every board's thermal observer has left UNCERTAIN and then past 20 m, the fall
    burned to a stop over the floor and held at 10 cm, the boards' SOA spent."""
    from machine import quad
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_quad as view
    from tools.render import page

    rows, real = [], view.compose

    def compose(console, origin, rotors, frame, route, trace, now, apex, ready, art):
        out = real(console, origin, rotors, frame, route, trace, now, apex, ready, art)
        rows.append((time.monotonic(), quad.STAGES[route['stage']][0], frame['h'],
                     max(r['amps'] for r in rotors),
                     max((r['budget'] or {}).get('worst') or 0.0 for r in rotors),
                     [(r['ident'] or {}).get('state') for r in rotors],
                     sum(0x2800 < ord(c) <= 0x28FF for c in art)))
        return out
    view.compose = compose
    try:
        page.frame('quad', 150, 44, frames=int(48.0 * FPS_CAP))
    finally:
        view.compose = real
    rate = (len(rows) - 1) / max(1e-9, rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0
    if rate < 4.0:
        report.skip('the flight', 'the page drew %.1f frames a second' % rate)
        return
    report.check('the quad drawn in the viewport, every frame',
                 min(r[6] for r in rows) >= 200, '%d braille cells at the least'
                 % min(r[6] for r in rows))
    tilt = [r for r in rows if r[1] == 'full tilt']
    report.check('full tilt only once the four observers have left UNCERTAIN',
                 bool(tilt) and all(all(s in view.GO for s in r[5]) for r in tilt)
                 and any(r[1] == 'hover' and not all(s in view.GO for s in r[5])
                         for r in rows),
                 'full tilt at %.1f s, the observers %s' % (
                     tilt[0][0] - rows[0][0], tilt[0][5]) if tilt else 'no full tilt')
    held = [r[2] for r in rows if r[1] == 'hold']
    low = min((r[2] for r in rows if r[1] in ('burn', 'hold')), default=math.nan)
    report.check('full tilt past 20 m',
                 bool(tilt) and max(r[2] for r in rows) >= 20.0,
                 '%.1f A at most, %.1f m' % (max((r[3] for r in tilt), default=0.0),
                                            max(r[2] for r in rows)))
    report.check('the fall burned to a stop over the floor, held at 10 cm',
                 low >= 0.03 and bool(held) and abs(held[-1] - quad.FLOOR_M) <= 0.1,
                 'lowest %.3f m, held at last %.3f m' % (low, held[-1] if held else math.nan))
    report.check('and the boards\' thermal observers spend their SOA',
                 max(r[4] for r in rows) >= 0.15, '%.2f at most' % max(r[4] for r in rows))


ROSTER = (test_the_flight_stops_at_its_mark, test_the_page_flies_four_boards)


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
