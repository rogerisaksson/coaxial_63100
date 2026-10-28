"""The QUAD page's flight on four stand-in boards, no page: its late passes, a wreck, its shell."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def flown(seconds, until, things=(), dt=0.05):
    """The page's card on four stand-in boards among `things`, a pass of `dt` s at a time
    (flight.passed), `seconds` at the most or until `until(the flight's own)`: ([(the stage,
    the clock, the share of the clamp's pull its rows have, the rotors' most amps, the frame's
    height)] a step, the flight's own)."""
    from coaxial import Coaxial63100
    from machine.aircraft import aerobatics, aerokin, quad
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
    from machine.aircraft import flightpack, quad
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
                 link is not None and abs(link - flightpack.open_volts(1.0)) < 0.01, '%s V' % link)


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


def test_an_ask_is_rows_a_tick_of_its_loop(report):
    """A rotor on its board asked down 300 rad/s^2, an ask a pass of the core: the current its
    loop asks, a tick to the next - as the flight asks it (`flight.asked`), and written whole,
    where each ask is a spool to the loop's feed-forward and the tick after its end."""
    from coaxial import Coaxial63100
    from machine.aircraft import pull, quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    apart = {}
    for rows in (True, False):
        rig = Coaxial63100(execution_mode=SIMULATED).open()
        try:
            rotor = view.arm(rig)
            drive, loop, asks = rig.board.drive, rig.board.ctrl, []
            for k in range(700):
                drive.model.configure(b=pull.ROTOR.b + 2.0 * quad.K_DRAG * abs(rotor['w']),
                                      load=-quad.K_DRAG * rotor['w'] * abs(rotor['w']))
                ask = 450.0 - 300.0 * max(0.0, k * view.TICK_S - 1.2)
                view.asked(rotor, ask) if rows else loop.write(ask)
                for _ in range(view.ROWS):
                    view.turned(rotor, 1.0 / view.LOOP_HZ)
                    asks.append(loop.state()['out'])
            apart[rows] = max(abs(b - a) for a, b in zip(asks[-100:], asks[-99:]))
        finally:
            rig.board.drive.off()
            rig.gates.off()
            rig.close()
    report.check('an ask as rows a tick of its loop long: the current asked a tick to the next '
                 'a tenth of what an ask written whole leaves it',
                 view.ROWS > 1 and apart[True] < 0.1 * apart[False],
                 '%.2f A apart at the most, %.2f written whole' % (apart[True], apart[False]))


def test_a_late_pass_is_its_steps(report):
    """Every pass the page's longest (flight.passed): taken in steps of STEP_S, the rest owed,
    its corkscrew is flown on most of the rotors' pull and `home` held in its seconds. Stepped
    whole, the frame rang on its rotors: 30 A, 22 % of their pull, `home` never held."""
    from terminal.views.quad import flight as view
    count, owed = view.steps(0.05)
    report.check('a pass of 50 ms is two steps of 20 and 10 ms owed, one of 19 none',
                 count == 2 and abs(owed - 0.01) < 1e-12 and view.steps(0.019)[0] == 0,
                 '%d and %.3f s owed, %d' % (count, owed, view.steps(0.019)[0]))
    # The page's card sweeps past the spectator first, 14 s.
    rows, flight = flown(48.0, lambda flight: flight['stage'] == 'toss')
    hard = [r for r in rows if r[0] in ('corkscrew', 'home')]
    report.check('passes of 50 ms: the corkscrew on three fifths of the rotors\' pull and more, '
                 'home held in its seconds, nothing struck',
                 flight['stage'] == 'toss' and rows[-1][1] <= 43.0 and not flight['crashes']
                 and min(r[2] for r in hard) >= 0.6,
                 '%s at %.1f s; %.0f %% of their pull at the least, %.1f A at the most' % (
                     flight['stage'], rows[-1][1],
                     100.0 * min((r[2] for r in hard), default=math.nan),
                     max((r[3] for r in hard), default=math.nan)))


def test_struck_it_is_begun_again(report):
    """A slab over the spot under its hover, in the lift's way: struck, the flight is CRASHED and says
    what it struck, its rotors stopped and the wreck on the floor; CRASH_S on the frame is on
    its spot again and the flight at its card's first row - and struck again."""
    from machine.aircraft import quad
    from terminal.views.quad import flight as view
    slab = [('box', 'house', (0.0, 0.4, 0.0), (0.6, 0.05, 0.6), 0.0)]
    words = []

    def twice(flight):
        if flight['wreck']:
            words.append(view.struck(flight))
        return flight['crashes'] >= 2
    rows, flight = flown(40.0, twice, slab)
    stages = [row[0] for row in rows]
    first = stages.index(view.CRASHED) if view.CRASHED in stages else len(stages)
    again = next((k for k in range(first, len(rows)) if rows[k][0] != view.CRASHED), len(rows))
    wreck = rows[first:max(first + 1, again - 1)]
    report.check('the lift struck the slab under it: crashed into a house, %.0f s a wreck, its '
                 'rotors stopped' % view.CRASH_S,
                 first < len(rows) and rows[first - 1][0] == 'lift' and 0.0 < rows[first][4] < quad.HOVER_M
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
    from machine.aircraft import pull, quad
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    try:
        rotor = view.arm(rig, clock=10, warm_s=0.0)
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
    from machine.aircraft import aerobatics, aerokin, pull, quad
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
    # Its full tilt 0.45 s, a row 0.02 s: the four 4.5 A apart at its first, 1.8 at most from 0.08 s.
    report.check('a full tilt flown, no board of the four past its throttle\'s point or '
                 'tripped in it', len(rows) > 15 and not cut,
                 '%d steps, %s' % (len(rows), cut[:3] or 'none past it'))
    began = rows[0][0] + 0.1
    apart = max(max(a) - min(a) for t, a, _b, _tilt, _h in rows if t >= began)
    most = max(max(a) for _t, a, _b, _tilt, _h in rows)
    report.check('the four on the same current, 0.15 of their clamp apart at the most',
                 apart < 0.15 * pull.I_MAX and most > 0.6 * pull.I_MAX,
                 '%.1f A apart at the most, %.1f A the most' % (apart, most))
    tilt = max(tilt for _t, _a, _b, tilt, _h in rows)
    report.check('the frame on its axis up it', tilt < 20.0,
                 '%.1f deg at the most, %.1f m up at its end' % (tilt, rows[-1][4]))


def test_its_rows_are_inside_the_shell(report):
    """flight.shell, the action space as the flight is in it: a row's pull the rotors' on the
    clamp the board with the least in it holds, the flight's at the most, and on the pack's
    charge; spent where that is under what the frame is sped up on or the pack at its
    reserve; fit with a charged pack and every board holding the flight's clamp."""
    from machine.aircraft import aerokin, flightpack, pull, quad
    from terminal.views.quad import flight as view

    def shell(left, *amps):
        flight = {'cells': {'left': left}}
        return view.shell(flight, [{'budget': {} if a is None else {'hold_amps': a}}
                                   for a in amps])

    def have(left, amps):
        return aerokin.HEADROOM * pull.thrust(pull.top(left, amps)) / quad.MASS_KG
    whole, wep = pull.I_MAX, pull.I_WEP
    full, unsaid, warm = shell(1.0, wep, whole, wep, wep), shell(1.0, *[None] * 4), shell(
        1.0, wep, 0.5 * whole, wep, whole)
    report.check('every board holding the flight\'s clamp or more, or none having said: the '
                 'four\'s top on the pack, fit; one holding half of it: the pull that leaves, '
                 'not spent, not fit',
                 full == unsaid == (have(1.0, whole), False, True)
                 and warm == (have(1.0, 0.5 * whole), False, False) and warm[0] < full[0],
                 '%.1f m/s^2 a kg on all of it, %.1f on half' % (full[0], warm[0]))
    low, shut = shell(0.5, wep, wep, wep, wep), shell(1.0, wep, wep, 8.0, wep)
    flat = shell(flightpack.RESERVE, wep, wep, wep, wep)
    report.check('on half its charge less; a board holding 8 A: under what the frame is sped '
                 'up on, spent; its pack at its reserve: spent with all its pull, not fit',
                 low[0] < full[0] and low[1:] == (False, True)
                 and shut[0] < aerokin.LEAST * quad.GRAVITY and shut[1:] == (True, False)
                 and flat[0] > aerokin.LEAST * quad.GRAVITY and flat[1:] == (True, False),
                 '%.1f on half its charge; %.1f on 8 A, %.1f the least; %.1f at its reserve' % (
                     low[0], shut[0], aerokin.LEAST * quad.GRAVITY, flat[0]))


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
          test_an_ask_is_rows_a_tick_of_its_loop,
          test_a_board_holds_its_clamp, test_full_tilt_cuts_no_rotor_alone,
          test_a_late_pass_is_its_steps, test_struck_it_is_begun_again,
          test_its_rows_are_inside_the_shell, test_wep_past_the_soa)


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
