"""The QUAD page's flight on four stand-in boards, no page: its late passes, a wreck, its WEP."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def flown(seconds, until, things=(), dt=0.05):
    """The page's card on four stand-in boards among `things`, a pass of `dt` s at a time
    (flight.passed), `seconds` at the most or until `until(the flight's own)`: ([(the stage,
    the clock, the envelopes' share, the rotors' most amps, the frame's height)] a step, the
    flight's own)."""
    from coaxial import Coaxial63100
    from machine import aerobatics, aerokin, quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rotors = [view.arm(Coaxial63100(execution_mode=SIMULATED).open()) for _ in range(4)]
    rows, clock = [], 0.0
    try:
        sky, route = quad.Sky(things), aerobatics.routine(view.CARD)
        law, flight = aerokin.fresh(), view.fresh()
        while clock < seconds and not until(flight):
            for clock, frame in view.passed(rotors, sky, route, law, flight, clock, dt):
                rows.append((flight['stage'], clock, flight['share'],
                             max(r['amps'] for r in rotors), frame['h']))
    finally:
        for rotor in rotors:
            rotor['rig'].board.drive.off()
            rotor['rig'].gates.off()
            rotor['rig'].close()
    return rows, flight


def test_the_boards_air_is_the_rotors(report):
    """A stand-in board armed and stepped as the page arms and steps it, its 63100 at a hover's
    speed: the rpm its thermal network's air sees is the rotor's own, by the record's pole
    pairs."""
    from coaxial import Coaxial63100
    from machine import quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        rotor = view.arm(rig)
        drive, w = rig.board.drive, 0.0
        hover = quad.speed_for(quad.MASS_KG * quad.GRAVITY / 4.0)
        rig.board.ctrl.write(hover)
        for _ in range(300):
            drive.paced(0.01)
            w = drive.model.read()['omega'] / rotor['pairs']
            drive.model.configure(load=quad.K_DRAG * w * abs(w))
            rig.board.thermal.fast_forward(0.01 * rig.board.thermal.HASTE, live=True)
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


def test_a_rotor_turns_by_its_pass(report):
    """A stand-in drive paced by its page (`SimulatedDrive.paced`): spun up over the same
    passes it comes to the same speed however long the wall's clock stood between them."""
    import time

    from coaxial import Coaxial63100
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    came = []
    for stall in (0.0, 0.12):
        rig = Coaxial63100(execution_mode=SIMULATED).open()
        try:
            view.arm(rig)
            drive = rig.board.drive
            rig.board.ctrl.off()
            drive.write(iq_ref=8.0)
            for k in range(20):
                drive.paced(0.01)
                speed = drive.model.read()['omega']
                if k in (5, 12):
                    time.sleep(stall)
            came.append(speed)
        finally:
            rig.board.drive.off()
            rig.gates.off()
            rig.close()
    report.check('twenty passes of 10 ms at 8 A, two of them 0.12 s late: the same speed',
                 came[0] > 50.0 and abs(came[1] - came[0]) <= 1e-6 * came[0],
                 '%.3f and %.3f rad/s electrical' % tuple(came))


def test_a_late_pass_is_its_steps(report):
    """Every pass the page's longest (flight.passed): taken in steps of STEP_S, the rest owed,
    its corkscrew is flown on most of the rotors' pull and `home` held in its seconds. Stepped
    whole, the frame rang on its rotors: 30 A, 22 % of their pull, `home` never held."""
    from terminal.views.quad import flight as view
    count, owed = view.steps(0.05)
    report.check('a pass of 50 ms is two steps of 20 and 10 ms owed, one of 19 none',
                 count == 2 and abs(owed - 0.01) < 1e-12 and view.steps(0.019)[0] == 0,
                 '%d and %.3f s owed, %d' % (count, owed, view.steps(0.019)[0]))
    rows, flight = flown(34.0, lambda flight: flight['stage'] == 'toss')
    hard = [r for r in rows if r[0] in ('corkscrew', 'home')]
    report.check('passes of 50 ms: the corkscrew on three fifths of the rotors\' pull and more, '
                 'home held in its seconds, nothing struck',
                 flight['stage'] == 'toss' and rows[-1][1] <= 28.5 and not flight['crashes']
                 and min(r[2] for r in hard) >= 0.6,
                 '%s at %.1f s; %.0f %% of their pull at the least, %.1f A at the most' % (
                     flight['stage'], rows[-1][1],
                     100.0 * min((r[2] for r in hard), default=math.nan),
                     max((r[3] for r in hard), default=math.nan)))


def test_struck_it_is_begun_again(report):
    """A slab a metre over the spot, in the lift's way: struck, the flight is CRASHED and says
    what it struck, its rotors stopped and the wreck on the floor; CRASH_S on the frame is on
    its spot again and the flight at its card's first row - and struck again."""
    from terminal.views.quad import flight as view
    slab = [('box', 'house', (0.0, 1.2, 0.0), (0.6, 0.05, 0.6), 0.0)]
    words = []

    def twice(flight):
        if flight['wreck']:
            words.append(view.struck(flight))
        return flight['crashes'] >= 2
    rows, flight = flown(40.0, twice, slab)
    stages = [row[0] for row in rows]
    first = stages.index(view.CRASHED) if view.CRASHED in stages else len(stages)
    wreck = [row for row in rows[first:] if row[0] == view.CRASHED]
    again = next((k for k in range(first, len(rows)) if rows[k][0] != view.CRASHED), len(rows))
    report.check('the lift struck the slab under it: crashed into a house, %.0f s a wreck, its '
                 'rotors stopped' % view.CRASH_S,
                 first < len(rows) and rows[first - 1][0] == 'lift' and 0.5 < rows[first][4] < 1.2
                 and set(words) == {'into a house'} and again < len(rows)
                 and abs(rows[again][1] - rows[first][1] - view.CRASH_S) <= 0.1
                 and wreck[-1][3] < 1.0,
                 'struck at %.2f m in the %s, %s; a wreck %.1f s, %.1f A at its end' % (
                     rows[first][4], rows[first - 1][0], sorted(set(words)),
                     rows[again][1] - rows[first][1] if again < len(rows) else math.nan,
                     wreck[-1][3]) if first < len(rows) else 'never struck')
    report.check('on its spot again, its flight begun over from its card\'s first row - and '
                 'struck a second time',
                 again < len(rows) and abs(rows[again][4]) <= 0.02
                 and rows[again][0] in (view.CARD[0][0], 'cool', 'swap')
                 and 'lift' in stages[again:] and flight['crashes'] == 2,
                 '%s at %.3f m; %d crashes' % (
                     rows[again][0], rows[again][4], flight['crashes'])
                 if again < len(rows) else 'never put back')


def test_a_board_holds_its_clamp(report):
    """A board asked its rotor's top from a hover: it carries what its envelope leaves
    (`hold_amps`) and no more, its legs come to their throttle's point and none past it, the
    three of them alike - their mean squares counted, not sampled -; it says its clamp is
    held, and trips nothing."""
    from coaxial import Coaxial63100
    from coaxial.devices.thermal import THROTTLE_AT
    from machine import pull, quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        rotor = view.arm(rig)
        drive, loop, heat = rig.board.drive, rig.board.ctrl, rig.board.thermal
        rows, t = [], 0.0
        while t < 13.5:
            loop.write(211.0 if t < 12.0 else pull.held(pull.I_MAX))
            drive.paced(view.STEP_S)
            w = drive.model.read()['omega'] / rotor['pairs']
            drive.model.configure(load=quad.K_DRAG * w * abs(w))
            heat.fast_forward(view.STEP_S * heat.HASTE, live=True)
            t += view.STEP_S
            if t >= 12.0:
                rows.append((drive.state()['iq'], heat.budget()))
        squares, legs = drive.sample()['squares'], heat.state()['nodes']
    finally:
        rig.board.drive.off()
        rig.gates.off()
        rig.close()
    first, last = rows[0][1]['hold_amps'], rows[-1][1]['hold_amps']
    # against the clamp it was given, the step before
    over = max(iq - was['hold_amps'] for (iq, _b), (_iq, was) in zip(rows[1:], rows))
    report.check('from a hover it may carry its clamp; at its top what its legs balance on',
                 first >= pull.I_MAX > last > 0.5 * pull.I_MAX,
                 '%.1f A at the hover, %.1f A 1.5 s into its top' % (first, last))
    report.check('it carries what it may and no more', over < 0.05 * pull.I_MAX,
                 '%.1f A over at the most, %.1f A at its end' % (over, rows[-1][0]))
    worst = max(b['worst'] for _iq, b in rows)
    report.check('its legs come to their throttle\'s point and none past it; its clamp held, '
                 'it says so, and trips nothing',
                 THROTTLE_AT - 0.05 < worst <= THROTTLE_AT + 1.0 / 255.0
                 and rows[-1][1]['throttling'] and not rows[0][1]['throttling']
                 and not any(b['tripped'] for _iq, b in rows),
                 'envelope %.3f at the most, of %.2f' % (worst, THROTTLE_AT))
    warm = [legs[n] for n in ('driver_u', 'driver_v', 'driver_w')]
    amps = rows[-1][0]
    report.check('the three legs alike: each phase\'s mean square half its amplitude\'s',
                 max(warm) - min(warm) < 0.1 * (min(warm) - 25.0)
                 and all(abs(sq - 0.5 * amps * amps) < 0.05 * amps * amps for sq in squares),
                 'legs %s C; mean squares %s of %.0f A^2' % (
                     ' '.join('%.0f' % c for c in warm), ' '.join('%.0f' % sq for sq in squares),
                     0.5 * amps * amps))


def test_full_tilt_cuts_no_rotor_alone(report):
    """The page's card to its first full tilt and through it, four boards: none past its
    throttle's point or tripped, the four on the same current - each its own board's hold, and
    what leans the frame into the wind up there -, the frame on its axis."""
    from coaxial import Coaxial63100
    from coaxial.devices.thermal import THROTTLE_AT
    from machine import aerobatics, aerokin, pull, quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rotors = [view.arm(Coaxial63100(execution_mode=SIMULATED).open()) for _ in range(4)]
    rows, clock = [], 0.0
    try:
        sky, route = quad.Sky(()), aerobatics.routine(view.CARD)
        law, flight = aerokin.fresh(), view.fresh()
        while clock < 60.0 and flight['stage'] != 'fall':
            for clock, frame in view.passed(rotors, sky, route, law, flight, clock, 0.05):
                if flight['stage'] == 'full tilt':
                    rows.append((clock, [r['amps'] for r in rotors],
                                 [dict(r['budget'] or {}) for r in rotors],
                                 math.degrees(math.acos(max(-1.0, min(
                                     1.0, float(frame['turn'][1][1]))))), frame['h']))
    finally:
        for rotor in rotors:
            rotor['rig'].board.drive.off()
            rotor['rig'].gates.off()
            rotor['rig'].close()
    cut = [(t, k) for t, _a, budgets, _tilt, _h in rows for k, b in enumerate(budgets)
           if b.get('tripped') or (b.get('worst') or 0.0) > THROTTLE_AT + 1.0 / 255.0]
    report.check('a full tilt flown, no board of the four past its throttle\'s point or '
                 'tripped in it', len(rows) > 50 and not cut,
                 '%d steps, %s' % (len(rows), cut[:3] or 'none past it'))
    began = rows[0][0] + 0.5
    apart = max(max(a) - min(a) for t, a, _b, _tilt, _h in rows if t >= began)
    most = max(max(a) for _t, a, _b, _tilt, _h in rows)
    report.check('the four on the same current, 0.15 of their clamp apart at the most',
                 apart < 0.15 * pull.I_MAX and most > 0.6 * pull.I_MAX,
                 '%.1f A apart at the most, %.1f A the most' % (apart, most))
    tilt = max(tilt for _t, _a, _b, tilt, _h in rows)
    report.check('the frame on its axis up it', tilt < 20.0,
                 '%.1f deg at the most, %.1f m up at its end' % (tilt, rows[-1][4]))


def test_war_emergency_power(report):
    """flight.emergency: all of the rotors' pull, whatever the envelopes leave, where the law
    asks more than it has and a thing is in the frame's way - not for either alone -, kept
    WEP_HOLD_S past it, WEP_S of it a flight, whole again on the floor; and the law's lean in
    one all that pull gives, whatever its row's."""
    from machine import aerobatics, aerokin, quad
    from terminal.views.quad import flight as view

    class Sky:
        """What is ahead, as told."""
        thing: str | None = None
        asked = 0

        def ahead(self, _seconds):
            self.asked += 1
            return self.thing and (self.thing, 0.3)

    sky, law, flight = Sky(), {'short': False}, view.fresh()
    route = {'card': view.CARD, 'row': 4}

    def step(share=0.4, dt=0.1):
        return view.emergency(flight, sky, law, route, share, dt)
    calm = step()
    law['short'] = True
    clear = step()
    sky.thing, law['short'] = 'tree', False
    unasked = (step(), sky.asked)
    law['short'] = True
    whole, taken = step(1.0), step()
    report.check('the envelopes\' share while nothing is in its way or it asks no more than it '
                 'has; all of the pull where both, a tree named, on all of it already as well',
                 calm == clear == unasked[0] == (0.4, None) and unasked[1] == 1
                 and whole == taken == (1.0, 'tree')
                 and flight['wep']['from'] == 'tree' and flight['wep']['taken'] == 1,
                 '%s, %s, %s; %s; %s from a %s' % (calm, clear, unasked[0], whole, taken,
                                                   flight['wep']['from']))
    sky.thing = None
    kept = [step()[1] for _ in range(8)]
    sky.thing = 'tree'
    spent = [step()[1] for _ in range(80)]
    route['row'] = 0
    floor = (step(), flight['wep']['left'])
    report.check('kept %.1f s past it; %.0f s of it a flight and no more; whole again where the '
                 'card waits to be fit' % (view.WEP_HOLD_S, view.WEP_S),
                 kept == ['tree'] * 5 + [None] * 3 and spent.count('tree') == 43
                 and not spent[-1] and floor == ((0.4, None), view.WEP_S),
                 '%d steps past it, %d more of 80 on it, %.1f s left on the floor' % (
                     kept.count('tree'), spent.count('tree'), floor[1]))
    leans = []
    for emergency in (None, 'tree'):
        law = dict(aerokin.fresh(), spot=[30.0, 0.0])
        level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        payload = aerokin.step(
            law, dict(aerobatics.HOVER, x=30.0, lean=2.0),
            {'turn': level, 'spin': (0.0, 0.0, 0.0), 'vel': (0.0, 0.0, 0.0),
             'at': (0.0, quad.HOVER_M, 0.0), 'h': quad.HOVER_M, 'v': 0.0},
            0.01, 0.4 * aerokin.HEADROOM * view.fresh()['top'], None, emergency)
        leans.append((math.degrees(math.hypot(payload.latent_targets[aerokin.PITCH],
                                              payload.latent_targets[aerokin.ROLL])),
                      law['short']))
    report.check('its spot 30 m off, the law short of its row\'s lean of 2 m/s^2; in an '
                 'emergency its rotors\' axis asked further over, for all the pull they have',
                 leans[0][1] and leans[1][0] > 1.5 * leans[0][0] > 0.0,
                 '%.1f deg off up asked, %.1f in an emergency' % (leans[0][0], leans[1][0]))


def test_wep_past_the_soa(report):
    """Thermal op 14 on the stand-in: the clamp's thermal derate held at one for the seconds
    asked, given back after them, the trip on the record's ceiling standing; 2 s at most."""
    from coaxial.errors import RigError
    from coaxial.simulated.thermal.observer import SimulatedThermal
    th = SimulatedThermal(situation='bench')
    th.fast_forward(0.0)
    clamp, trips = [], []
    th._derate_to, th._gate = clamp.append, lambda: trips.append(1) or True
    th._sample = lambda: {'amps': (0.0, 0.0, 0.0), 'switching': True}
    top = th.LIMIT['driver_u']
    hot = th.state()['ambient'] + 0.97 * (top - th.state()['ambient'])
    th._place('driver_u', hot)
    th._judge()
    held = clamp[-1]
    th.wep(0.5)
    th._judge()
    on = clamp[-1]
    th.fast_forward(0.6 * th.HASTE)
    th._place('driver_u', hot)
    th._judge()
    report.check('a leg at 97 % of its span: its clamp shut, WEP opens it for 0.5 s, then shut',
                 held == 0.0 and on == 1.0 and clamp[-1] == 0.0,
                 '%.2f, %.2f, %.2f' % (held, on, clamp[-1]))
    th._place('driver_u', top + 1.0)
    th.wep(0.5)
    th._judge()
    try:
        th.wep(3.0)
        refused = False
    except RigError:
        refused = True
    report.check('on WEP the trip stands at the record\'s ceiling, and 3 s is refused',
                 len(trips) == 1 and refused, '%d trips, refused %s' % (len(trips), refused))


ROSTER = (test_the_boards_air_is_the_rotors, test_a_rotor_turns_by_its_pass,
          test_a_board_holds_its_clamp, test_full_tilt_cuts_no_rotor_alone,
          test_a_late_pass_is_its_steps, test_struck_it_is_begun_again,
          test_war_emergency_power, test_wep_past_the_soa)


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
