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
    63100's ceiling: the lift on its ramp, full tilt past 30 m, the burn and the hold never
    under their mark, the hold on it a second in, level, the landing still on the floor."""
    from machine import quad
    sky, route, t, top_rad_s = quad.Sky(), quad.flight(), 0.0, 310.0
    top = 4.0 * quad.K_THRUST * top_rad_s ** 2
    w = [0.0] * 4
    apex, low, tilt, seen = 0.0, math.inf, 0.0, {}
    while t < 30.0:
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
        if 'land' in seen and name == 'spool':
            break
        seen.setdefault(name, []).append((state['h'], state['v']))
        t += 0.01
    lift, hover = seen.get('lift') or [(math.nan, 0.0)], seen.get('hover') or [(math.nan, 0.0)]
    report.check('the lift ends within 5 cm of its hover and the hover never 5 cm over it',
                 abs(lift[-1][0] - quad.HOVER_M) <= 0.05
                 and max(h for h, _v in hover) <= quad.HOVER_M + 0.05,
                 'the lift ends at %.2f m, the hover peaks at %.2f' % (
                     lift[-1][0], max(h for h, _v in hover)))
    settled = [h for h, _v in (seen.get('hold') or [])[100:]]
    report.check('full tilt past 30 m; the burn and the hold never 2 cm under their mark, the '
                 'hold within 2 cm of %.0f cm from a second in, level' % (100 * quad.FLOOR_M),
                 apex >= 30.0 and low >= quad.FLOOR_M - 0.02 and bool(settled) and tilt < 1.0
                 and all(abs(h - quad.FLOOR_M) <= 0.02 for h in settled),
                 'apex %.1f m, lowest %.3f m, settled %.3f-%.3f m, %.2f degrees at most' % (
                     apex, low, min(settled or [math.nan]), max(settled or [math.nan]), tilt))
    down = (seen.get('land') or [(math.nan, math.nan)])[-1]
    report.check('the landing ends on the floor, still',
                 abs(down[0]) <= 0.005 and abs(down[1]) <= 0.05,
                 '%.3f m at %+.2f m/s' % down)


#: The page's flights drawn this long at most, s.
LANDED_S = 120.0


def test_the_page_flies_four_boards(report):
    """The page on its four stand-in boards into its second flight: the quad drawn in the
    viewport, full tilt only once every board's thermal observer has left UNCERTAIN and then
    past 20 m, the fall burned to a stop over the floor and held at 10 cm, the boards' SOA
    spent, and full tilt again."""
    from machine import quad
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_quad as view
    from tools.render import page

    rows, real = [], view.compose

    class Landed(Exception):
        """The second flight at full tilt."""

    def compose(console, origin, rotors, frame, route, trace, now, apex, ready, art):
        out = real(console, origin, rotors, frame, route, trace, now, apex, ready, art)
        rows.append((time.monotonic(), quad.STAGES[route['stage']][0], frame['h'],
                     max(r['amps'] for r in rotors),
                     max((r['budget'] or {}).get('worst') or 0.0 for r in rotors),
                     [(r['ident'] or {}).get('state') for r in rotors],
                     sum(0x2800 < ord(c) <= 0x28FF for c in art)))
        if rows[-1][1] == 'full tilt' and any(r[1] == 'land' for r in rows):
            raise Landed
        return out
    view.compose = compose
    # Drawn into the second flight's full tilt, LANDED_S at most: drawn 48 s, on CI's host,
    # its boards' observers slower, the first hold was still coming down as the frames ran
    # out (2026-09-28).
    try:
        page.frame('quad', 150, 44, frames=int(LANDED_S * FPS_CAP))
    except Landed:
        pass
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
                 low >= 0.05 and bool(held) and abs(held[-1] - quad.FLOOR_M) <= 0.05,
                 'lowest %.3f m, held at last %.3f m' % (low, held[-1] if held else math.nan))
    # At a room of 0.4 the third flight never left its hover, the hover's own share over it
    # since the dry loss's refit (2026-10-05).
    again = [r for i, r in enumerate(rows) if r[1] == 'full tilt'
             and any(x[1] == 'land' for x in rows[:i])]
    report.check('landed, it lifts and goes full tilt again',
                 bool(again), '%.1f s after the first' % (again[0][0] - tilt[0][0])
                 if again and tilt else 'the frames ran out in %s' % rows[-1][1])
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
