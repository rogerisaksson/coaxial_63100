"""The quad's flight: its routine (machine.aerobatics on machine.flying) on ideal rotors, its pack, its air."""
import functools
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


#: The flight's rows without its figures: the floor, the hover, full tilt, the burn, the way down.
BASE = ('idle', 'spool', 'lift', 'hover', 'full tilt', 'burn', 'hold', 'descend', 'land')

#: A spent pack's stand on the floor before it is fit again, s, in the flights flown here.
FLOOR_S = 2.0


@functools.lru_cache(maxsize=None)
def flown(share=1.0, base=False, spent_at=None):
    """The routine once round, a row a pass: machine.quad's frame in MuJoCo on rotors lagging
    0.05 s to their speed, capped at the page's top, on `share` of their pull - only its
    BASE rows, `base`; its pack spent from `spent_at` s until it has stood FLOOR_S on the floor,
    and the flight ended as it lifts again."""
    from machine import aerobatics, quad
    from machine.flying import Flying
    from terminal.views.quad.flight import TOP_RAD_S as top_rad_s
    card = tuple(row for row in aerobatics.CARD if row[0] in BASE) if base else aerobatics.CARD
    sky, route = quad.Sky(), aerobatics.routine(card)
    flying = Flying(4.0 * quad.K_THRUST * top_rad_s ** 2, aerobatics.DOWN)
    w, t, rows, flat, stood = [0.0] * 4, 0.0, [], False, 0.0
    while t < 120.0:
        flat = flat or (spent_at is not None and t >= spent_at and stood < FLOOR_S)
        holds = (['held'] if flying.held else []) + ['spent' if flat else 'fit']
        name, flying.ask = aerobatics.fly(route, t, holds)
        if flat and name == card[0][0]:
            stood += 0.01
            flat = stood < FLOOR_S
        if rows and name == card[2][0] and stood >= FLOOR_S:
            break
        if rows and rows[-1]['name'] == card[-1][0] and name == card[0][0] and spent_at is None:
            break
        for k, thrust in enumerate(flying.step(sky.state(), 0.01, share)):
            w[k] += ((min(top_rad_s, quad.speed_for(thrust, flying.along)) - w[k])
                     * min(1.0, 0.01 / 0.05))
        sky.step(w, 0.01)
        state = sky.state()
        rows.append({'t': t, 'name': name, 'h': state['h'], 'v': state['v'], 'a': state['a'],
                     'x': float(state['at'][0]), 'z': float(state['at'][2]),
                     'tilt': math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1]))))),
                     'heading': math.degrees(flying.heading), 'thrust': flying.thrust,
                     'spin': tuple(abs(float(s)) for s in state['spin']), 'flat': flat})
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
    their mark, the hold on it a second in, level, the landing still on the floor - and the
    same fall stopped on a third of the rotors' pull, the boards' envelopes all but spent."""
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
    burn, hold = seen.get('burn') or [], (seen.get('hold') or []) + (seen.get('descend') or [])
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
    # The envelopes' share: what it is sped up on and a stop is planned on, not what a stop
    # may take.
    weak = flown(share=0.3, base=True)
    stop = [r for r in weak if r['name'] in ('burn', 'hold', 'descend')]
    report.check('on a third of their pull: a lower apex, the burn still never 2 cm under its '
                 'mark and held on it',
                 10.0 <= max(r['h'] for r in weak) < apex - 10.0 and bool(stop)
                 and min(r['h'] for r in stop) >= quad.FLOOR_M - 0.02
                 and abs(stop[-1]['h'] - quad.FLOOR_M) <= 0.02,
                 'apex %.1f m, lowest %.3f m, held at %.3f m' % (
                     max(r['h'] for r in weak), min((r['h'] for r in stop), default=math.nan),
                     stop[-1]['h'] if stop else math.nan))


def test_its_figures_are_flown(report):
    """The routine's figures as its card names them: the pirouette a turn on its spot, the orbit
    a banked lap about the point ahead of it, the corkscrew that lap climbed, the roll and the
    flip each once over and caught where they were tossed from, the way back down - gently, and
    clear of the pole."""
    from coaxial.graphics import scenery
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
    px, pz = scenery.POLE_AT
    pole = min(math.hypot(r['x'] - px, r['z'] - pz) for r in flown())
    report.check('gently: 0.7 g at most outside its turns, full tilt and the burn, the '
                 'pirouette on its hover\'s thrust within 1 N; the pole 1.5 m off at the least',
                 pull <= 0.7 and bool(still) and max(still) - min(still) <= 1.0 and pole >= 1.5,
                 '%.2f g, the pirouette %.1f-%.1f N, the pole %.2f m off' % (
                     pull, min(still or [math.nan]), max(still or [math.nan]), pole))


def test_spent_it_comes_down(report):
    """Its pack spent in the corkscrew, metres up and going round: the routine leaves for its
    way down, at its pace and never under the stop's mark, lands still and waits on the floor
    until it is fit; fit, it lifts again."""
    from machine import aerobatics, quad
    flight = flown(spent_at=22.5)
    names = [name for name, _rows in spans(flight)]
    at = next((i for i, r in enumerate(flight) if r['flat']), len(flight))
    down = [r for r in flight[at:] if r['name'] in ('descend', 'land')]
    floor = [r for r in flight[at:] if r['name'] == aerobatics.CARD[0][0]]
    report.check('spent at %.1f m in the %s: the way down, the landing, the floor, the lift' % (
        flight[at]['h'] if at < len(flight) else math.nan,
        flight[at - 1]['name'] if 0 < at < len(flight) else '?'),
        names[names.index('descend') - 1] == 'corkscrew'
        and names[names.index('descend'):] == ['descend', 'land', aerobatics.CARD[0][0], 'spool'],
        ' '.join(names[-6:]))
    report.check('down at its pace, never 2 cm under the mark, on the floor still',
                 bool(down) and min(r['v'] for r in down) >= -1.25 * aerobatics.OVER['pace']
                 and min(r['h'] for r in down if r['name'] == 'descend') >= quad.FLOOR_M - 0.02
                 and abs(down[-1]['h']) <= 0.005 and abs(down[-1]['v']) <= 0.05,
                 '%.2f m/s at its fastest, lowest %.3f m, ends %.3f m at %+.2f m/s' % (
                     -min((r['v'] for r in down), default=math.nan),
                     min((r['h'] for r in down if r['name'] == 'descend'), default=math.nan),
                     down[-1]['h'] if down else math.nan, down[-1]['v'] if down else math.nan))
    report.check('it waits on the floor while it is not fit, its rotors run down to their idle',
                 bool(floor) and sum(r['flat'] for r in floor) >= 100.0 * FLOOR_S - 5
                 and floor[-1]['thrust'] <= 1.05 * 4.0 * quad.K_THRUST * quad.IDLE_RAD_S ** 2,
                 '%d passes spent on the floor, %.1f N at their end' % (
                     sum(r['flat'] for r in floor), floor[-1]['thrust'] if floor else math.nan))


def test_its_pack(report):
    """machine.quad's pack: 63 V full, its bus drooping its resistance a link amp, its charge
    counted down by those amps and up by what is braked into it, spent at its reserve."""
    from machine import quad
    cells = quad.pack()
    full = cells['volts']
    loaded = quad.drawn(cells, 1500.0, 1.0)
    amps, left = cells['amps'], cells['left']
    report.check('63 V full; 1.5 kW droops it its ohms a link amp and takes those amps\' charge',
                 abs(full - 63.0) < 1e-6 and abs(full - loaded - quad.PACK_OHM * amps) < 0.05
                 and abs(loaded * amps - 1500.0) < 1.0
                 and abs((1.0 - left) - amps / (3600.0 * quad.PACK_AH)) < 1e-9,
                 '%.1f V full, %.2f V at %.1f A, %.5f of it left' % (full, loaded, amps, left))
    raised = quad.drawn(cells, -300.0, 1.0)
    report.check('braked into, its bus rises over its open volts and its charge with it',
                 raised > quad.open_volts(left) and cells['left'] > left and cells['amps'] < 0.0,
                 '%.2f V over %.2f open, %.5f left' % (raised, quad.open_volts(left), cells['left']))
    seconds = 0
    while cells['left'] > quad.RESERVE and seconds < 3600:
        quad.drawn(cells, 250.0, 1.0)
        seconds += 1
    report.check('a hover and its figures\' 250 W spend it to its reserve in minutes, its volts '
                 'down the cells\' curve',
                 120 <= seconds <= 900 and quad.open_volts(quad.RESERVE) < cells['volts'] + 1.0 < 60.0,
                 '%d s, %.1f V under it' % (seconds, cells['volts']))


def test_its_air(report):
    """machine.quad's air, a kind kept a minute: the wind its kind's, gusts over it where the
    kind has them, its way swung in a changing one and kept in a constant, its eddies their
    size; the same from the same seed; sheared along the floor and rising nowhere at it; its
    kinds toured in turn, and stepped by hand."""
    from machine import quad

    def minute(kind, seed=3):
        air, rows = quad.air(seed, kind), []
        for _ in range(3000):
            wind = quad.blown(air, 0.02)
            rows.append((math.hypot(wind[0], wind[2]), math.atan2(wind[0], wind[2]),
                         air['blows'], air['eddy'][0], air['eddy'][1]))
        return rows[500:]
    seen = {kind: minute(kind) for kind in quad.AIR}
    mean = {kind: sum(r[0] for r in rows) / len(rows) for kind, rows in seen.items()}
    gust = {kind: max(r[2] for r in rows) for kind, rows in seen.items()}
    swing = {kind: math.degrees(max(r[1] for r in rows) - min(r[1] for r in rows))
             for kind, rows in seen.items()}
    report.check('a constant wind its 4 m/s on one way, a calm under 1; gusts in the gusty one '
                 'alone of the two, 2 m/s and more; a changing one\'s way swung 100 degrees',
                 abs(mean['constant'] - quad.AIR['constant'][0]) <= 0.3 and mean['calm'] < 1.0
                 and gust['constant'] == 0.0 and gust['gusty'] >= 2.0
                 and swing['constant'] <= 20.0 and swing['changing'] >= 100.0,
                 'means %s m/s; gusts %.1f and %.1f; ways swung %.0f and %.0f degrees' % (
                     ' '.join('%.1f' % mean[k] for k in quad.AIR), gust['constant'], gust['gusty'],
                     swing['constant'], swing['changing']))
    rough = [math.sqrt(sum(r[k] ** 2 for r in seen['turbulent']) / len(seen['turbulent']))
             for k in (3, 4)]
    size = quad.AIR['turbulent'][5]
    report.check('a turbulent one\'s eddies their size along the floor, %.1f of it up' % quad.EDDY_UP,
                 0.6 * size <= rough[0] <= 1.4 * size
                 and 0.6 * quad.EDDY_UP * size <= rough[1] <= 1.4 * quad.EDDY_UP * size,
                 '%.2f and %.2f m/s rms of %.1f' % (rough[0], rough[1], size))
    report.check('the same air from the same seed, another from another',
                 minute('gusty') == seen['gusty'] and minute('gusty', 4) != seen['gusty'],
                 '%d passes alike' % len(seen['gusty']))
    low, ref, high = (quad.wind_at((4.0, 1.0, 0.0), h) for h in (0.0, quad.REF_M, 100.0))
    report.check('sheared along the floor - under half of it at the floor, %.1f of it at the most '
                 '- and rising nowhere at the floor' % quad.SHEAR_MOST,
                 low[0] < 2.0 and low[1] == 0.0 and ref == (4.0, 1.0, 0.0)
                 and abs(high[0] - quad.SHEAR_MOST * 4.0) < 1e-9,
                 '%.2f m/s at the floor, %.2f up there; %.1f at %g m' % (
                     low[0], low[1], high[0], 100.0))
    air, kinds = quad.air(), []
    for _ in range(len(quad.TOUR)):
        kinds.append(air['kind'])
        quad.blown(air, quad.KIND_S)
    by_hand = []
    for _ in range(len(quad.TOUR) + 1):
        quad.kept(air)
        by_hand.append((air['kind'], air['kept']))
    report.check('its kinds toured in turn, %.0f s each; by hand each kept in turn, then the tour '
                 'again' % quad.KIND_S,
                 tuple(kinds) == quad.TOUR
                 and by_hand == [(kind, True) for kind in quad.TOUR] + [(quad.TOUR[0], False)],
                 '%s; %s' % (' '.join(kinds), ' '.join(k for k, _kept in by_hand)))


def test_it_holds_in_wind(report):
    """The law in the air, a hover at 5 m on the boards' rotors (quad_race.LAG_S past the
    page's spool): in a constant wind it is held on its spot, the wind as it has learnt it the
    wind on it - unlearnt, ten times as far off and never held; in gusts within half a metre."""
    from machine import aerobatics, flying as law, quad
    from terminal.views.quad.flight import SPOOL_RAD_S2, TOP_RAD_S
    from tools.sim.quad_race import LAG_S

    def hover(kind, learn=True):
        """(the furthest off its spot, m, the share of the passes held, how far the law's wind
        was off the wind on it, m/s rms) over 20 s of the air, 12 s into it."""
        was = law.AIR_S
        law.AIR_S = was if learn else math.inf
        try:
            sky, air = quad.Sky(), quad.air(3, kind)
            flying = law.Flying(4.0 * quad.K_THRUST * TOP_RAD_S ** 2, dict(aerobatics.HOVER, height=5.0))
            w, t, rows = [0.0] * 4, 0.0, []
            while t < 40.0:
                for k, thrust in enumerate(flying.step(sky.state(), 0.02, 1.0)):
                    more = ((min(TOP_RAD_S, quad.speed_for(thrust, flying.along)) - w[k])
                            * min(1.0, 0.02 / LAG_S))
                    w[k] += max(-SPOOL_RAD_S2 * 0.02, min(SPOOL_RAD_S2 * 0.02, more))
                wind = quad.blown(air, 0.02) if t >= 8.0 else (0.0, 0.0, 0.0)
                sky.step(w, 0.02, air if t >= 8.0 else None)
                t += 0.02
                if t >= 20.0:
                    s = sky.state()
                    here = quad.wind_at(wind, s['h'])
                    rows.append((math.hypot(float(s['at'][0]) - flying.spot[0],
                                            float(s['at'][2]) - flying.spot[1]), flying.held,
                                 math.hypot(flying.wind[0] - here[0], flying.wind[1] - here[2])))
        finally:
            law.AIR_S = was
        return (max(r[0] for r in rows), sum(r[1] for r in rows) / len(rows),
                math.sqrt(sum(r[2] ** 2 for r in rows) / len(rows)))
    steady, blind, gusty = hover('constant'), hover('constant', False), hover('gusty')
    report.check('a wind of 4 m/s: on its spot within 0.2 m and held nine passes of ten, the '
                 'wind learnt within 0.3 m/s of it; unlearnt, ten times as far off and never held',
                 steady[0] <= 0.2 and steady[1] >= 0.9 and steady[2] <= 0.3
                 and blind[0] >= 10.0 * steady[0] and blind[1] <= 0.1,
                 '%.2f m off, held %.0f %%, its wind %.2f m/s off; unlearnt %.2f m off, held '
                 '%.0f %%' % (steady[0], 100.0 * steady[1], steady[2], blind[0], 100.0 * blind[1]))
    report.check('in gusts to 7 m/s within half a metre of its spot',
                 gusty[0] <= 0.5, '%.2f m off at the most, held %.0f %% of it' % (
                     gusty[0], 100.0 * gusty[1]))


def test_a_line_ends_itself(report):
    """A routine's card with a row that gives its own (aerobatics.fly): spent, a figure's row
    leaves for its flight's way down - not the card's first, nor on it the next flight's -; the
    line's row stays its own until it holds what it waits for."""
    from machine import aerobatics

    def line(route, _now):
        if route.get('done'):
            route['holds'] = ('lapped',)
        return dict(aerobatics.HOVER, speed=2.0)
    card = (('idle', aerobatics.DOWN, 0.0, 0.0, 'fit'), ('hover', aerobatics.HOVER, 9.0, 0.0, ''),
            ('descend', aerobatics.OVER, 0.0, 0.0, 'held'),
            ('idle', aerobatics.DOWN, 0.0, 0.0, 'fit'), ('lift', aerobatics.HOVER, 9.0, 0.0, ''),
            ('lap', line, 0.0, 0.0, 'lapped'), ('descend', aerobatics.OVER, 0.0, 0.0, 'held'))
    went = []
    for row in (1, 2, 4, 5):
        route = dict(aerobatics.routine(card), row=row)
        went.append(aerobatics.fly(route, 1.0, ('spent',))[0])
        went.append(route['row'])
    report.check('spent: the first flight\'s hover to its own way down and no further from '
                 'there, the second\'s lift to the second\'s, the line\'s row its own',
                 went == ['descend', 2, 'descend', 2, 'descend', 6, 'lap', 5], str(went))
    route = dict(aerobatics.routine(card), row=5)
    first = aerobatics.fly(route, 1.0, ('fit',))
    route['done'] = True
    aerobatics.fly(route, 2.0, ('fit',))
    after = aerobatics.fly(route, 3.0, ('fit',))
    report.check('the line gives its row, and holding what it waits for it is left',
                 first[0] == 'lap' and first[1]['speed'] == 2.0 and after[0] == 'descend'
                 and route['row'] == 6,
                 '%s at %.1f m/s, then %s' % (first[0], first[1]['speed'], after[0]))


ROSTER = (test_the_flight_stops_at_its_mark, test_its_figures_are_flown, test_spent_it_comes_down,
          test_a_line_ends_itself, test_its_pack, test_its_air, test_it_holds_in_wind)


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
