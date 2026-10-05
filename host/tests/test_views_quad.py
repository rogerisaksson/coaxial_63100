"""The QUAD page: its routine (machine.aerobatics on machine.flying) on ideal rotors, the page on four stand-ins."""
import functools
import math
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


@functools.lru_cache(maxsize=None)
def flown():
    """The routine once round, a row a pass: machine.quad's frame in MuJoCo on rotors lagging
    0.05 s to their speed, capped at the 63100's ceiling, every wait held."""
    from machine import aerobatics, quad
    from machine.flying import Flying
    top_rad_s = 310.0
    sky, route = quad.Sky(), aerobatics.routine()
    flying = Flying(4.0 * quad.K_THRUST * top_rad_s ** 2, aerobatics.DOWN)
    w, t, rows = [0.0] * 4, 0.0, []
    while t < 120.0:
        name, flying.ask = aerobatics.fly(route, t, ('ready', 'held') if flying.held else ('ready',))
        if rows and rows[-1]['name'] == aerobatics.CARD[-1][0] and name == aerobatics.CARD[0][0]:
            break
        for k, share in enumerate(flying.step(sky.state(), 0.01)):
            w[k] += (min(top_rad_s, quad.speed_for(share)) - w[k]) * min(1.0, 0.01 / 0.05)
        sky.step(w, 0.01)
        state = sky.state()
        rows.append({'name': name, 'h': state['h'], 'v': state['v'], 'a': state['a'],
                     'x': float(state['at'][0]), 'z': float(state['at'][2]),
                     'tilt': math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1]))))),
                     'heading': math.degrees(flying.heading), 'thrust': flying.thrust,
                     'spin': tuple(abs(float(s)) for s in state['spin'])})
        t += 0.01
    return tuple(rows)


def spans(rows):
    """[(name, its rows)] as the routine flew them, in order."""
    out = []
    for row in rows:
        if not out or out[-1][0] != row['name']:
            out.append((row['name'], []))
        out[-1][1].append(row)
    return out


def test_the_flight_stops_at_its_mark(report):
    """The lift from rest onto its hover, full tilt past 30 m, the burn and the hold never under
    their mark, the hold on it a second in, level, the landing still on the floor."""
    from machine import quad
    seen = {}
    for name, rows in spans(flown()):
        seen.setdefault(name, rows)
    lift, hover = seen.get('lift') or [{'h': math.nan}], seen.get('hover') or [{'h': math.nan}]
    report.check('the lift ends within 5 cm of its hover and the hover never 5 cm over it',
                 abs(lift[-1]['h'] - quad.HOVER_M) <= 0.05
                 and max(r['h'] for r in hover) <= quad.HOVER_M + 0.05,
                 'the lift ends at %.2f m, the hover peaks at %.2f' % (
                     lift[-1]['h'], max(r['h'] for r in hover)))
    burn, hold = seen.get('burn') or [], seen.get('hold') or []
    settled = [r['h'] for r in hold[100:]]
    low = min((r['h'] for r in burn + hold), default=math.nan)
    tilt = max((r['tilt'] for r in hover + hold), default=math.nan)
    apex = max(r['h'] for r in flown())
    report.check('full tilt past 30 m; the burn and the hold never 2 cm under their mark, the '
                 'hold within 2 cm of %.0f cm from a second in, level' % (100 * quad.FLOOR_M),
                 apex >= 30.0 and low >= quad.FLOOR_M - 0.02 and bool(settled) and tilt < 1.0
                 and all(abs(h - quad.FLOOR_M) <= 0.02 for h in settled),
                 'apex %.1f m, lowest %.3f m, settled %.3f-%.3f m, %.2f degrees at most' % (
                     apex, low, min(settled or [math.nan]), max(settled or [math.nan]), tilt))
    down = (seen.get('land') or [{'h': math.nan, 'v': math.nan}])[-1]
    report.check('the landing ends on the floor, still',
                 abs(down['h']) <= 0.005 and abs(down['v']) <= 0.05,
                 '%.3f m at %+.2f m/s' % (down['h'], down['v']))


def test_its_figures_are_flown(report):
    """The routine's figures as its card names them: the pirouette a turn on its spot, the orbit
    a banked lap about the point ahead of it, the corkscrew that lap climbed, the roll and the
    flip each once over and caught where they were tossed from, the way back down - gently, and
    clear of the pole."""
    from coaxial.graphics import quadcopter
    from machine import aerobatics, quad
    flight = spans(flown())
    seen = {}
    for name, rows in flight:
        seen.setdefault(name, rows)
    spin = seen.get('pirouette', []) + seen.get('poise', [])
    turned = spin[-1]['heading'] - spin[0]['heading'] if spin else math.nan
    off = max((math.hypot(r['x'], r['z']) for r in spin), default=math.nan)
    lifted = max((abs(r['h'] - quad.HOVER_M) for r in spin), default=math.nan)
    report.check('the pirouette a whole turn on its spot, within 5 cm and 3 cm of its height',
                 abs(turned - 360.0) <= 3.0 and off <= 0.05 and lifted <= 0.03,
                 '%.1f degrees, %.3f m off, %.3f m off its height' % (turned, off, lifted))
    orbit, screw = seen.get('orbit') or [], seen.get('corkscrew') or []
    late = orbit[len(orbit) // 2:]
    # About the point ORBIT_M ahead of where it stood, its nose on it.
    out = [math.hypot(r['x'], r['z'] - aerobatics.ORBIT_M) for r in late + screw]
    bank = [r['tilt'] for r in late]
    lap = orbit[-1]['heading'] - orbit[0]['heading'] if orbit else math.nan
    report.check('the orbit a lap about the point %.0f m ahead, within half a metre of its '
                 'circle, banked 20-30 degrees' % aerobatics.ORBIT_M,
                 bool(out) and max(abs(d - aerobatics.ORBIT_M) for d in out) <= 0.5
                 and 20.0 <= min(bank) and max(bank) <= 30.0 and abs(abs(lap) - 310.0) <= 60.0,
                 '%.2f-%.2f m out, banked %.1f-%.1f degrees, %.0f degrees round' % (
                     min(out or [math.nan]), max(out or [math.nan]), min(bank or [math.nan]),
                     max(bank or [math.nan]), lap))
    report.check('the corkscrew climbs that lap to %.0f m' % aerobatics.TOP_M,
                 bool(screw) and abs(screw[-1]['h'] - aerobatics.TOP_M) <= 0.1
                 and abs(screw[-1]['heading'] - screw[0]['heading']) >= 300.0,
                 'ends at %.2f m, %.0f degrees round' % (
                     screw[-1]['h'] if screw else math.nan,
                     screw[-1]['heading'] - screw[0]['heading'] if screw else math.nan))
    # A turn over and its catch: the rows named for it and the catch after them.
    for name, axis, about in (('roll', 2, 'its nose'), ('flip', 0, 'its wing')):
        at = next((i for i, (n, _rows) in enumerate(flight) if n == name), None)
        over = flight[at][1] + flight[at + 1][1] if at is not None else []
        caught = over[-1] if over else {'h': math.nan, 'tilt': math.nan, 'v': math.nan}
        fast = max(over, key=lambda r: max(r['spin']), default={'spin': (0.0, 0.0, 0.0)})['spin']
        report.check('the %s once over about %s, never under where it was tossed from, caught '
                     'there level' % (name, about),
                     bool(over) and max(r['tilt'] for r in over) >= 170.0
                     and fast[axis] == max(fast) > 0.0
                     and min(r['h'] for r in over) >= aerobatics.TOP_M - 0.1
                     and abs(caught['h'] - aerobatics.TOP_M) <= 0.1 and caught['tilt'] <= 5.0
                     and abs(caught['v']) <= 0.2,
                     '%.0f degrees over, lowest %.2f m, caught at %.2f m %+.2f m/s, %.1f degrees'
                     % (max((r['tilt'] for r in over), default=math.nan),
                        min((r['h'] for r in over), default=math.nan), caught['h'], caught['v'],
                        caught['tilt']))
    level = (seen.get('level') or [{'h': math.nan, 'x': math.nan, 'z': math.nan}])[-1]
    report.check('unwound, it is level over its spot at its hover again',
                 abs(level['h'] - quad.HOVER_M) <= 0.05
                 and math.hypot(level['x'], level['z']) <= 0.2,
                 '%.2f m up, %.2f m off' % (level['h'], math.hypot(level['x'], level['z'])))
    # Gently: outside the turns, full tilt and the burn, the pull within 0.7 g of none.
    hard = ('roll', 'flip', 'catch', 'full tilt', 'burn')
    pull = max(abs(r['a']) / quad.GRAVITY for r in flown() if r['name'] not in hard)
    still = [r['thrust'] for r in spin]
    px, pz = quadcopter.POLE_AT
    pole = min(math.hypot(r['x'] - px, r['z'] - pz) for r in flown())
    report.check('gently: 0.7 g at most outside its turns, full tilt and the burn, the '
                 'pirouette on its hover\'s thrust within 1 N; the pole 1.5 m off at the least',
                 pull <= 0.7 and bool(still) and max(still) - min(still) <= 1.0 and pole >= 1.5,
                 '%.2f g, the pirouette %.1f-%.1f N, the pole %.2f m off' % (
                     pull, min(still or [math.nan]), max(still or [math.nan]), pole))


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
            w_hat = (drive.state().get('omega_hat') or 0.0) / rotor['pairs']
            w = drive.model.read()['omega'] / rotor['pairs']
            drive.write(iq_ref=rotor['pi'].step(0.01, setpoint=hover, measured=w_hat)['command'])
            drive.model.configure(load=quad.K_DRAG * w * abs(w))
            time.sleep(0.01)
        air = rig.board.thermal.state().get('speed_rpm') or 0.0
    finally:
        rig.board.drive.off()
        rig.gates.off()
        rig.close()
    rpm = w * 60.0 / math.tau
    report.check('the air sees the rotor\'s own rpm at a hover, within 3 %',
                 rpm > 1000.0 and abs(air - rpm) <= 0.03 * rpm,
                 'the rotor %.0f rpm, the air %.0f' % (rpm, air))


def test_the_observers_are_worded(report):
    """The TH OBS row: who is stable, who converges, what the routine waits for."""
    from terminal.views import show_quad as view

    def rotors(*states):
        return [{'ident': {'state': s}} for s in states]
    cases = ((('STABLE',) * 4, False, '4 of 4 stable'),
             (('STABLE', 'CONVERGING', 'CONVERGING', 'CONVERGING'), False, '1 stable, 3 converging'),
             (('CONVERGING',) * 4, False, '0 stable, 4 converging'),
             (('CONVERGING', 'CONVERGING', 'UNCERTAIN', 'UNCERTAIN'), True, '2 of 4, the routine waits'),
             (('STABLE',) * 4, True, '4 of 4, cooling to %.0f %%' % (100.0 * view.ROOM)))
    got = [view.observers(rotors(*states), waiting) for states, waiting, _want in cases]
    report.check('four stable say so, a mix its counts, a wait what for',
                 got == [want for _s, _w, want in cases], str(got))


#: The page's flights drawn this long at most, s.
LANDED_S = 150.0


def test_the_page_flies_four_boards(report):
    """The page on its four stand-in boards into its second flight: the quad drawn in the
    viewport, its routine only once every board's thermal observer has left UNCERTAIN, flown
    under the envelopes' throttle, full tilt past 20 m, the fall burned to a stop over the floor
    and held at 10 cm, the boards' SOA spent, and its routine again."""
    from coaxial.devices.thermal import THROTTLE_AT
    from machine import aerobatics, quad
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_quad as view
    from tools.render import page

    rows, real = [], view.compose
    first = aerobatics.CARD[[name for name, *_row in aerobatics.CARD].index('hover') + 1][0]

    class Again(Exception):
        """The second flight in its routine."""

    def compose(console, origin, rotors, frame, name, waiting, trace, now, apex, art):
        out = real(console, origin, rotors, frame, name, waiting, trace, now, apex, art)
        rows.append((time.monotonic(), name, frame['h'], max(r['amps'] for r in rotors),
                     max((r['budget'] or {}).get('worst') or 0.0 for r in rotors),
                     [(r['ident'] or {}).get('state') for r in rotors],
                     sum(0x2800 < ord(c) <= 0x28FF for c in art),
                     any((r['budget'] or {}).get('throttling') for r in rotors)))
        if name == first and any(r[1] == 'land' for r in rows):
            raise Again
        return out
    view.compose = compose
    # Drawn into the second flight's routine, LANDED_S at most: drawn 48 s, on CI's host, its
    # boards' observers slower, the first hold was still coming down as the frames ran out
    # (2026-09-28).
    try:
        page.frame('quad', 150, 44, frames=int(LANDED_S * FPS_CAP))
    except Again:
        pass
    finally:
        view.compose = real
    rate = (len(rows) - 1) / max(1e-9, rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0
    if rate < 4.0:
        report.skip('the flight', 'the page drew %.1f frames a second' % rate)
        return
    report.check('the quad drawn in the viewport, every frame',
                 min(r[6] for r in rows) >= 150, '%d braille cells at the least'
                 % min(r[6] for r in rows))
    routine = [r for r in rows if r[1] == first]
    report.check('its routine only once the four observers have left UNCERTAIN',
                 bool(routine) and all(all(s in view.GO for s in r[5]) for r in routine)
                 and any(r[1] == 'hover' and not all(s in view.GO for s in r[5])
                         for r in rows),
                 'the %s at %.1f s, the observers %s' % (
                     first, routine[0][0] - rows[0][0], routine[0][5]) if routine
                 else 'no %s' % first)
    figures = [r for r in rows if r[1] not in ('idle', 'spool', 'lift', 'hover', 'full tilt',
                                               'fall', 'burn', 'hold', 'land')]
    report.check('its figures under the envelopes\' throttle, no board throttling',
                 bool(figures) and max(r[4] for r in figures) < THROTTLE_AT
                 and not any(r[7] for r in figures),
                 'SOA %.2f at most over %d frames of them' % (
                     max((r[4] for r in figures), default=math.nan), len(figures)))
    tilt = [r for r in rows if r[1] == 'full tilt']
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
    report.check('landed, it lifts and flies its routine again',
                 rows[-1][1] == first and any(r[1] == 'land' for r in rows),
                 '%.1f s after the first' % (rows[-1][0] - routine[0][0]) if routine
                 else 'the frames ran out in %s' % rows[-1][1])
    report.check('and the boards\' thermal observers spend their SOA',
                 max(r[4] for r in rows) >= 0.15, '%.2f at most' % max(r[4] for r in rows))


ROSTER = (test_the_flight_stops_at_its_mark, test_its_figures_are_flown,
          test_the_boards_air_is_the_rotors, test_the_observers_are_worded,
          test_the_page_flies_four_boards)


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
