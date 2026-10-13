"""The quad's action space (machine.pull) against the board's law: a rotor's top, its reach, its
stage's watts on a stand-in; the four's on their pack."""
import sys

from tools.dev.focus import chosen
from views_kit import Report

#: A link's volts and a clamp's amps a rotor's top is taken at: the clamp's end, the link's
#: under each clamp, a link between two of the table's rows, a spent pack's.
CASES = ((63.0, 60.0), (63.0, 90.0), (57.3, 60.0), (48.0, 90.0))


def armed():
    """(a stand-in board armed as the page's flight arms it, its rotor)."""
    from coaxial import Coaxial63100
    from machine.modes import SIMULATED
    from terminal.views.quad import flight as view
    rig = Coaxial63100(execution_mode=SIMULATED).open()
    return rig, view.arm(rig)


def off(rig):
    """A board's stage and its session closed."""
    rig.board.drive.off()
    rig.gates.off()
    rig.close()


def turned(rig, rotor, dt):
    """A rotor `dt` s on under its propeller: its speed, rad/s."""
    from machine import quad
    drive = rig.board.drive
    drive.paced(dt)
    w = drive.model.read()['omega'] / rotor['pairs']
    drive.model.configure(load=quad.K_DRAG * w * abs(w))
    return w


def test_the_space_closes(report):
    """The space on its own: a clamp's top takes all its amps and a link's all the link
    leaves, the reach none at either; the tables read between two rows are the top sought
    there; the four on their pack are under one on its open volts, and under less charge
    lower."""
    from machine import pull, quad
    rotor = pull.ROTOR
    clamp = [abs(pull.torque(pull.held(a)) / rotor.kt - a) for a in (10.0, pull.I_MAX, pull.I_WEP)]
    link = [abs(pull.room(pull.linked(v), v) - pull.torque(pull.linked(v)) / rotor.kt)
            for v in (40.0, 55.0, 63.0)]
    still = [abs(pull.reach(pull.ceiling(v, a), v, a)) for v, a in CASES]
    report.check('a clamp\'s top takes its amps, a link\'s what the link leaves; no reach at '
                 'either',
                 max(clamp) < 1e-9 and max(link) < 1e-6 and max(still) < 1.0,
                 '%.1e A and %.1e A off, %.2f rad/s^2 left at the most' % (
                     max(clamp), max(link), max(still)))
    volts = [abs(pull.ceiling(v, 1e3) - pull.linked(v)) for v in (41.3, 52.77, 57.3, 62.9)]
    lefts = (0.0, 0.03, 0.2, 0.33, 0.62, 0.9, 0.97, 1.0)
    packs = [abs(pull.top(left, 1e3) - pull.linked(quad.open_volts(left), 4)) for left in lefts]
    report.check('read between two rows a table is the top sought there, within 0.05 rad/s',
                 max(volts) < 0.05 and max(packs) < 0.05,
                 '%.4f rad/s off on a link, %.4f on the pack' % (max(volts), max(packs)))
    tops = [pull.top(left, pull.I_MAX) for left in lefts]
    full = pull.top(1.0, pull.I_MAX)
    need = pull.torque(full) / rotor.kt
    bus = quad.sagged(quad.open_volts(1.0), 4.0 * pull.watts(full, need, need))
    report.check('the four on a full pack: under one on its open volts, their bus what their '
                 'draw leaves of it; lower on less charge, WEP\'s clamp no higher',
                 full < pull.ceiling(quad.open_volts(1.0), pull.I_MAX)
                 and abs(pull.ceiling(bus, 1e3) - full) < 0.05 and tops == sorted(tops)
                 and tops[0] < tops[-1] and pull.top(1.0, pull.I_WEP) == full,
                 '%.1f rad/s on %.1f A each, the bus %.1f V of %.1f; %.1f at the reserve' % (
                     full, need, bus, quad.open_volts(1.0), pull.top(quad.RESERVE, pull.I_MAX)))


def test_a_rotors_top_is_the_laws(report):
    """A stand-in board armed as the page arms it: the space's motor is its record's; its
    rotor's steady top, the q current asked up to its clamp on a held link, is the space's -
    where the clamp ends it and where the link does."""
    from machine import pull
    rig, rotor = armed()
    try:
        drive = rig.board.drive
        params = drive.params()
        record = [params[name] for name in ('motor_r', 'motor_lq', 'motor_lambda',
                                            'motor_pole_pairs')]
        rig.board.ctrl.off()
        tops = []
        for volts, clamp in CASES:
            drive.model.configure(vdc=volts)
            w = 0.0
            for k in range(250):
                drive.write(iq_ref=clamp * min(1.0, (k + 1) / 50.0))
                w = turned(rig, rotor, 0.01)
            tops.append((volts, clamp, w, drive.state()['iq'], drive.state()['fault']))
    finally:
        off(rig)
    report.check('the space is computed on the motor its board\'s record holds',
                 all(abs(a - b) <= 1e-6 * abs(b) for a, b in zip(record, pull.ROTOR[:4])),
                 '%s and %s' % (record, list(pull.ROTOR[:4])))
    worst = max(abs(w - pull.ceiling(v, a)) / w for v, a, w, _iq, _fault in tops)
    report.check('its top on a held link is the space\'s within 0.1 %: the clamp\'s end at 63 V '
                 'and 60 A, the link\'s elsewhere, no trip',
                 worst < 1e-3 and not any(fault for *_x, fault in tops)
                 and abs(tops[0][3] - CASES[0][1]) < 0.1 and tops[1][3] < 0.8 * CASES[1][1],
                 '; '.join('%.1f V %.0f A: %.1f rad/s on %.1f A, the space %.1f' % (
                     v, a, w, iq, pull.ceiling(v, a)) for v, a, w, iq, _fault in tops))


def test_a_rotors_reach_and_watts(report):
    """A stand-in rotor stepped from a hover's current to its clamp: its speed gains what the
    space says, on a full link and where a spent one leaves less than the clamp; steady under
    its loop its stage and the law's own v . i take its shaft's and its copper's watts."""
    from machine import pull
    from terminal.views.quad import flight as view
    rig, rotor = armed()
    gains, taken = [], []
    try:
        drive, loop = rig.board.drive, rig.board.ctrl
        loop.off()
        for volts, at in ((63.0, 400.0), (44.0, 520.0)):
            drive.model.configure(vdc=volts)
            drive.write(iq_ref=pull.torque(211.0) / pull.ROTOR.kt)
            for _ in range(200):
                turned(rig, rotor, 0.01)
            drive.write(iq_ref=pull.I_MAX)
            trail = [turned(rig, rotor, view.TICK_S) for _ in range(250)]
            k = next(k for k, w in enumerate(trail) if w >= at)
            gains.append((volts, trail[k], (trail[k + 5] - trail[k - 5]) / (10.0 * view.TICK_S),
                          pull.reach(trail[k], volts, pull.I_MAX), pull.room(trail[k], volts)))
        drive.model.configure(vdc=63.0)
        drive.write(iq_ref=pull.torque(211.0) / pull.ROTOR.kt)
        for _ in range(200):
            turned(rig, rotor, 0.01)
        loop.write(211.0)
        loop.on()
        for ask in (211.0, 500.0, 680.0):
            loop.write(ask)
            w = 0.0
            for _ in range(250):
                w = turned(rig, rotor, 0.01)
            rotor['w'] = w
            watts, law = view.stated(rotor), drive.state()
            taken.append((w, watts, pull.torque(w) * w + 1.5 * pull.ROTOR.r * rotor['amps'] ** 2,
                          1.5 * (law['vd'] * law['id'] + law['vq'] * law['iq'])))
    finally:
        off(rig)
    report.check('its speed gains what the space says within 3 %, on the clamp at 63 V and on '
                 'what a link at 44 V leaves of it',
                 all(abs(got - want) < 0.03 * want for _v, _at, got, want, _room in gains)
                 and gains[0][4] > pull.I_MAX > gains[1][4],
                 '; '.join('%.0f V at %.0f rad/s: %.0f rad/s^2, the space %.0f, the link '
                           'leaving %.0f A' % row for row in gains))
    report.check('steady, its stage and the law\'s own v . i take its shaft\'s and its copper\'s '
                 'watts within 1 %',
                 all(abs(watts - true) < 0.005 * true and abs(vi - true) < 0.01 * true
                     for _w, watts, true, vi in taken),
                 '; '.join('%.0f rad/s: %.0f W of %.0f, v . i %.0f' % row for row in taken))


def test_four_on_their_pack(report):
    """Four stand-in boards asked past their top on one pack, their thermal derate held off:
    they come to the top the space has for the charge left in it, the bus where their draw
    leaves it."""
    from machine import pull, quad
    from terminal.views.quad import flight as view
    rigs = [armed() for _ in range(4)]
    cells, rows = quad.pack(), []
    try:
        for rig, _rotor in rigs:
            rig.board.thermal.wep(2.0)
            rig.board.ctrl.write(pull.held(pull.I_MAX))
        for _ in range(180):
            watts = 0.0
            for rig, rotor in rigs:
                rig.board.drive.model.configure(vdc=cells['volts'])
                rotor['w'] = turned(rig, rotor, 0.01)
                watts += view.stated(rotor)
            quad.drawn(cells, watts, 0.01)
            rows.append(([rotor['w'] for _rig, rotor in rigs], cells['volts'], cells['amps'],
                         cells['left'], [rotor['amps'] for _rig, rotor in rigs]))
    finally:
        for rig, _rotor in rigs:
            off(rig)
    speeds, volts, amps, left, each = rows[-1]
    want = pull.top(left, pull.I_MAX)
    need = pull.torque(want) / pull.ROTOR.kt
    report.check('the four come to the space\'s top for the charge left, within 0.3 %, under '
                 'their clamp: the link\'s end',
                 max(abs(w - want) for w in speeds) < 3e-3 * want and max(each) < pull.I_MAX
                 and want < pull.held(pull.I_MAX),
                 '%s rad/s, the space %.1f at %.3f of the charge; %.1f A each, the clamp %.0f' % (
                     ' '.join('%.1f' % w for w in speeds), want, left, max(each), pull.I_MAX))
    bus = quad.sagged(quad.open_volts(left), 4.0 * pull.watts(want, need, need))
    report.check('the bus where their draw leaves it, within 0.3 V; a second and a half of '
                 'it a twentieth of the charge',
                 abs(volts - bus) < 0.3 and 0.03 < 1.0 - left < 0.07,
                 '%.1f V on %.0f A, the space\'s %.1f; %.3f of the charge left' % (
                     volts, amps, bus, left))


ROSTER = (test_the_space_closes, test_a_rotors_top_is_the_laws, test_a_rotors_reach_and_watts,
          test_four_on_their_pack)


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
