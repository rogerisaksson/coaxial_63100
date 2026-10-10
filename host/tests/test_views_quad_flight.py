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
    from machine import aerobatics, quad
    from machine.flying import Flying
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rotors = [view.arm(Coaxial63100(execution_mode=SIMULATED).open()) for _ in range(4)]
    rows, clock = [], 0.0
    try:
        sky, route = quad.Sky(things), aerobatics.routine(view.CARD)
        flying, flight = Flying(view.TOP_N, aerobatics.DOWN), view.fresh()
        while clock < seconds and not until(flight):
            for clock, frame in view.passed(rotors, sky, route, flying, flight, clock, dt):
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
        for _ in range(300):
            drive.paced(0.01)
            now = drive.state()
            w_hat = (now.get('omega_hat') or 0.0) / rotor['pairs']
            w = drive.model.read()['omega'] / rotor['pairs']
            drive.write(iq_ref=rotor['pi'].step(0.01, setpoint=hover, measured=w_hat)['command'])
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


def test_the_envelopes_cut(report):
    """flight.envelope: all of the rotors' pull while every board has SPEND of its span in hand
    under its throttle's point, less UNDER; none at that point; a throttling board's own derate
    on it - taken over TAKEN_S, given back over RECOVER_S."""
    from coaxial.devices.thermal import THROTTLE_AT
    from terminal.views.quad import flight as view
    top = THROTTLE_AT - view.UNDER

    def cut(worst, derate=1.0, was=1.0, dt=10.0):
        return view.envelope([{'budget': {'worst': 0.1}},
                              {'budget': {'worst': worst, 'derate': derate}}], was, dt)
    halves = (cut(top - view.SPEND), cut(top - 0.5 * view.SPEND), cut(top), cut(0.95, 0.5))
    report.check('all of it SPEND under the point, half at half of it, none at it or past it',
                 [round(h, 6) for h in halves] == [1.0, 0.5, 0.0, 0.0], str(halves))
    report.check('a cut taken over TAKEN_S and given back over RECOVER_S',
                 abs(cut(top, dt=0.1) - (1.0 - 0.1 / view.TAKEN_S)) < 1e-9
                 and abs(cut(top - view.SPEND, was=0.0, dt=0.1) - 0.1 / view.RECOVER_S) < 1e-9,
                 '%.3f and %.3f' % (cut(top, dt=0.1), cut(top - view.SPEND, was=0.0, dt=0.1)))


def test_war_emergency_power(report):
    """flight.emergency: all of the rotors' pull, whatever the envelopes leave, where the law
    asks more than it has and a thing is in the frame's way - not for either alone -, kept
    WEP_HOLD_S past it, WEP_S of it a flight, whole again on the floor; and the law's lean in
    one all that pull gives, whatever its row's."""
    from machine import aerobatics, quad
    from machine.flying import Flying
    from terminal.views.quad import flight as view

    class Sky:
        """What is ahead, as told."""
        thing: str | None = None
        asked = 0

        def ahead(self, _seconds):
            self.asked += 1
            return self.thing and (self.thing, 0.3)

    class Law:
        short = False
    sky, law, flight = Sky(), Law(), view.fresh()
    route = {'card': view.CARD, 'row': 4}

    def step(share=0.4, dt=0.1):
        return view.emergency(flight, sky, law, route, share, dt)
    calm = step()
    law.short = True
    clear = step()
    sky.thing, law.short = 'tree', False
    unasked = (step(), sky.asked)
    law.short = True
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
        flying = Flying(view.TOP_N, dict(aerobatics.HOVER, x=30.0, lean=2.0))
        flying.spot = [30.0, 0.0]
        level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        thrusts = flying.step({'turn': level, 'spin': (0.0, 0.0, 0.0), 'vel': (0.0, 0.0, 0.0),
                               'at': (0.0, quad.HOVER_M, 0.0), 'h': quad.HOVER_M, 'v': 0.0,
                               'acc': (0.0, 0.0, 0.0), 'lift': quad.MASS_KG * quad.GRAVITY},
                              0.01, 0.4, emergency)
        leans.append((abs(thrusts[1] + thrusts[2] - thrusts[0] - thrusts[3]), flying.short))
    report.check('its spot 30 m off, the law short of its row\'s lean of 2 m/s^2; in an '
                 'emergency its discs turned harder, for all the pull the rotors have',
                 leans[0][1] and leans[1][0] > 1.5 * leans[0][0] > 0.0,
                 '%.1f N across its discs, %.1f in an emergency' % (leans[0][0], leans[1][0]))


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
          test_a_late_pass_is_its_steps, test_struck_it_is_begun_again,
          test_the_envelopes_cut, test_war_emergency_power, test_wep_past_the_soa)


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
