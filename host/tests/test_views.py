"""Every live view runs two frames against the stand-in, as a subprocess.

The suite that was missing while the views were restyled: four separate
breaks in one afternoon - a NameError in a compose, a view inheriting
power_afe=False and refusing at open, the toon path silently falling back
to the photographic renderer, dead --watch plumbing - and every one was
found by running the view by hand, because nothing else runs them at all.

Subprocesses, not imports: a view's crash on a real console involves its
own argument parsing, its preflight and its teardown, and importing main()
skips the first of those. --simulated so no board and no ollama; --frames 2
so the loop, the painter and the teardown all execute once.
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)

#: Every view, with the flags its two-frame run needs. Read off tools/
#: rather than hardcoded where possible - a new show_*.py joins by existing.
EXTRA = {
    'show_session.py': [],
    'menu.py': [],
    'show_capture.py': [],
    'show_desk.py': [],
    'show_gate_drivers.py': [],
    'show_orientation.py': ['--width', '72', '--height', '14'],
    'show_angle.py': [],
    'show_thermal_observer.py': [],
    'show_rotor_observer.py': [],
}


class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


def views():
    """The scripts under tools/ that are views, plus the session."""
    got = sorted(name for name in os.listdir(os.path.join(HOST, 'tools'))
                 if name.startswith('show_') and name.endswith('.py'))
    return ['show_session.py', 'menu.py'] + got


def run_view(name):
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    return subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join('tools', name),
         '--simulated', '--frames', '2'] + EXTRA.get(name, []),
        cwd=HOST, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=120)


def test_each_view_draws_two_frames(report):
    for name in views():
        done = run_view(name)
        tail = (done.stdout + done.stderr).strip().splitlines()
        last = tail[-1][:70] if tail else 'no output at all'
        report.check('%s exits 0 simulated' % name,
                     done.returncode == 0,
                     'exit %d: %s' % (done.returncode, last))
        report.check('%s raised nothing' % name,
                     'Traceback' not in done.stdout + done.stderr, last)


def rows_of(owner, width, height, kind):
    """Which rows of a rendered drawing carry `kind`."""
    return [row for row in range(height)
            if any(owner[row][col] == kind for col in range(width))]


def test_the_instruments_stand_clear_of_the_machine(report):
    """The gutters equidistant, and the foot gauges off the can.

    BOTH FOUND BY EYE, which is the whole reason for the test. The left
    group is six thermometers and the right four, and a machine centred
    in the box sat one column off the left group and two off the right -
    it reads as a drawing that is not quite straight. Then the winding
    gauge was drawn along row 16 of nineteen with the can's own bottom
    edge on the same row, and the level ran through the machine.

    Rendered at the view's own size, because both faults are the size's:
    at another width the machine misses the gutters by luck.
    """
    sys.path.insert(0, HOST)
    from coaxial import machine
    from tools import show_rotor_observer as view

    width, height = view.ART_WIDTH, view.ART_ROWS
    n_left, n_right = len(view.SOA_NODES), len(view.BOARD_NODES)
    _, owner, _text, _lit = machine._raster(
        6.0, 24, 28, width, height, None, None, None,
        [(0.4, machine.SOA_OK)] * n_left, [(0.3, machine.SOA_OK)] * n_right,
        [(0.5, machine.SOA_OK)],
        [(0.4, machine.SOA_WARN), (0.3, machine.WATTS)], 2.0)
    can = [col for row in range(height) for col in range(width)
           if owner[row][col] == machine.CAN]
    left, right = machine.gutters(width, height, n_left, n_right)
    gaps = (min(can) - max(left) - 1, min(right) - max(can) - 1)
    report.check('the gutters stand the same distance off the machine',
                 gaps[0] == gaps[1], 'left %d, right %d columns' % gaps)
    report.check('both groups fit inside the frame',
                 len(left) == n_left and len(right) == n_right,
                 '%d of %d left, %d of %d right'
                 % (len(left), n_left, len(right), n_right))

    can_rows = rows_of(owner, width, height, machine.CAN)
    for name, kind in (('winding', machine.SOA_WARN),
                       ('power', machine.WATTS)):
        on = rows_of(owner, width, height, kind)
        report.check('the %s gauge clears the can' % name,
                     bool(on) and not set(on) & set(can_rows),
                     'gauge rows %s, can rows %d..%d'
                     % (on, min(can_rows), max(can_rows)))


def test_each_gutter_says_its_hottest_node(report):
    """The caption's third row, in degrees, and WHICH node each one is.

    A share of a ceiling is what the board acts on and it is not a
    temperature: the tubes cannot say 118 C and a bench asks for exactly
    that. The left figure is the hottest of the six power nodes, which is
    what SWITCH TEMPS means.

    THE RIGHT ONE IS THE COPPER, not the hottest of its four. It was the
    hottest, and the bench read SWITCH TEMPS below BOARD TEMPS and took it
    for a broken model - the right gutter's hottest is almost always the
    MCU, 15 K over the copper on an idle board, so the caption said BOARD
    and reported something else. The other three are still tubes, and the
    SOA HEADROOM gauge is what says which node of all ten is worst.
    """
    sys.path.insert(0, HOST)
    from tools import show_rotor_observer as view

    nodes = dict.fromkeys(view.SOA_NODES + view.BOARD_NODES, 30.0)
    nodes['phase_v'], nodes['regulators'], nodes['board'] = 118.4, 71.2, 44.5
    # `used` for every node, because `soa_bars` draws only what the board
    # reported a spend for - a node with no ceiling in the record is a node
    # it says nothing about.
    said = {'thermal': {'nodes': nodes, 'ambient': 20.0},
            'budget': {'used': {name: (nodes[name] - 20.0) / 105.0
                                for name in nodes},
                       'tripped': False}}
    said['budget']['used']['phase_v'] = 0.95
    peak, cls = view.hottest(said, view.SOA_NODES)
    report.check('the switch caption takes the hottest leg',
                 peak == 118.4, 'said %s' % (peak,))
    report.check('and its colour comes from that same node margin',
                 cls == view.machine.SOA_WARN, 'class %s' % (cls,))

    peak, _ = view.hottest(said, view.BOARD_NODES)
    report.check('the board caption takes the hottest of its four, which '
                 'is the tube standing tallest beside it',
                 peak == 71.2, 'said %s' % (peak,))
    # REPORTING THE COPPER INSTEAD WAS TRIED AND WITHDRAWN. It bought the
    # ordering a reader expects and broke something worse: the figure then
    # disagreed with its own gutter, 20.9 C under a stack whose tallest
    # tube was the regulators at 33.7. What fixed the confusion was the
    # shared scale below, not the choice of node.
    bars = view.soa_bars(said, view.BOARD_NODES)
    tallest = max(zip(view.BOARD_NODES, bars), key=lambda p: p[1][0])[0]
    report.check('and it names the tallest tube, not some other node',
                 nodes[tallest] == peak,
                 '%s at %.1f C' % (tallest, nodes[tallest]))
    report.check('a group with nothing measured says nothing',
                 view.hottest({}, view.SOA_NODES)[0] is None)


def test_both_gutters_run_on_one_scale(report):
    """Height is degrees, colour is margin, and they are two questions.

    THE TUBES HAD TWO RULERS. Each was its node's share of its OWN
    ceiling, and the ceilings differ - the copper's is 105 where the
    silicon's is 125 - so two tubes at the same height were two different
    temperatures, standing under captions in degrees that disagreed with
    them. The check is a copper and a FET at the SAME temperature: the
    heights have to match and the colours must not.
    """
    sys.path.insert(0, HOST)
    from coaxial import machine
    from tools import show_rotor_observer as view

    nodes = dict.fromkeys(view.SOA_NODES + view.BOARD_NODES, 20.0)
    nodes['phase_u'] = nodes['board'] = 100.0
    said = {'thermal': {'nodes': nodes, 'ambient': 20.0},
            'budget': {'used': {'phase_u': 80.0 / 105.0,
                                'board': 80.0 / 85.0},
                       'tripped': False}}
    left = view.soa_bars(said, ('phase_u',))[0]
    right = view.soa_bars(said, ('board',))[0]
    report.check('two nodes at one temperature draw one height',
                 abs(left[0] - right[0]) < 1e-9,
                 '%.4f against %.4f' % (left[0], right[0]))
    report.check('and the copper still colours hotter, because its ceiling '
                 'is lower - the margin is what the colour carries',
                 left[1] == machine.SOA_OK and right[1] == machine.SOA_WARN,
                 'phase %s, board %s' % (left[1], right[1]))
    report.check('the scale is stated, not taken from a limit the board '
                 'acts on',
                 view.TEMP_SCALE_C == 125.0, '%.0f C' % view.TEMP_SCALE_C)


def test_a_power_node_never_reads_below_the_copper(report):
    """It sheds INTO the board, so it cannot be colder than the board.

    The relation the bench expected and the page was hiding. It holds in
    the model by construction - `thermal_step` sheds `(t - board) /
    to_board`, so a node below the copper takes a negative shed and is
    pulled back up - and this is the check that the two captions can
    actually be compared now that the right one is the copper.

    Against the stand-in rather than by inspection: it is the model, the
    limits and the labels together that have to come out right.
    """
    sys.path.insert(0, HOST)
    from coaxial import Coaxial63100
    from tools import show_rotor_observer as view

    rig = Coaxial63100(simulated_device=True)
    rig.open()
    try:
        rig.board.gate_drivers.bypass_break(True)
        rig.board.gate_drivers.enable()
        rig.drive.mode('hold')
        rig.drive.setpoint(iq_ref=30.0)
        worst = None
        for _ in range(8):
            time.sleep(0.15)
            said = {'thermal': rig.thermal.state(),
                    'budget': rig.thermal.budget()}
            switch = view.hottest(said, view.SOA_NODES)[0]
            copper = said['thermal']['nodes']['board']
            gap = switch - copper
            worst = gap if worst is None else min(worst, gap)
        report.check('no power node ever reads below the copper it sheds '
                     'into - the thing the gutters looked like they denied',
                     worst >= -1e-6, 'closest %.4f K' % worst)
    finally:
        rig.close()


def test_the_ntc_is_shown_as_the_one_measurement(report):
    """The reference above the headroom scale, and what it says unread.

    Every other figure on the page is an estimate - ten nodes of a lumped
    network, a winding relaxed into a placeholder pair - and the NTC is
    the only one with a sensor behind it. It stands over the scale the
    model's own verdict is drawn on, in TRUTH's ink, which is what this
    page gives what is known rather than modelled.
    """
    sys.path.insert(0, HOST)
    from coaxial import machine
    from tools import show_rotor_observer as view

    report.check('a reading is shown in degrees',
                 view.reference({'thermal': {'ntc': 38.04}})
                 == 'NTC 38.0 %sC' % view.DEGREE,
                 view.reference({'thermal': {'ntc': 38.04}}))
    # AFE_ON LOW IS NOT A COLD BOARD. The board answers None because the
    # AFE powers the ADC reference and there is no reading at all then
    # (invariant 9), and a dash cannot be mistaken for a temperature.
    for empty in ({'thermal': {'ntc': None}}, {}):
        report.check('and no reading says so rather than drawing a number',
                     'unread' in view.reference(empty),
                     view.reference(empty))

    # `simulated` and `spin` are view keys the real page always carries;
    # the caption reaches the winding estimate through the margin rows
    # now, and that asks how fast the stand-in's clock is running.
    rows = view.gutter_caption({
        'simulated': True, 'spin': 0.0,
        'thermal': {'nodes': dict.fromkeys(view.SOA_NODES + view.BOARD_NODES,
                                           40.0),
                    'ambient': 20.0, 'ntc': 38.0},
        'budget': {'used': {}, 'tripped': False},
        'state': {'id': 0.0, 'iq': 0.0, 'vd': 0.0, 'vq': 0.0},
        'params': {}, 'winding_at': None})
    # THE FIRST CAPTION ROW, and it has a tube of its own now. Everything
    # under it is an estimate, and a page that opens with a model teaches
    # a bench to trust one.
    said = rows[0]
    report.check('it opens the stack, above every estimate',
                 'NTC' in said and not any('NTC' in row
                                           for row in rows[1:]),
                 said.replace(chr(27), '^'))
    # ITS OWN TUBE'S COLOUR, which is the thermometer ramp - blue at the
    # cold end and red at the hot - because the thermistor has no ceiling
    # to be a margin against. Every legend here shares an ink with the
    # level it names.
    report.check('and takes its own tube colour, off the thermometer ramp',
                 any('38;5;%d' % machine.INK[step] in said
                     for step in machine.NTC_RAMP),
                 said.replace(chr(27), '^'))


def test_two_headrooms_named_apart(report):
    """The board's margin and the motor's are different facts.

    TWO WAYS TO COOK A BENCH. The board's headroom is the worst of ten
    nodes against ceilings the calibration record gave it, and the board
    acts on that itself - it throttles, and at a ceiling it drops MOE.
    The winding has no sensor and no ceiling the board was given: it is
    `3 i^2 R` relaxed into a placeholder pair, drawn against this page's
    own scale. One is a margin the board acts on and the other only the
    operator can, which is why they are named apart rather than averaged
    into one bar.
    """
    sys.path.insert(0, HOST)
    from tools import show_rotor_observer as view

    report.check('both scales are named, and named differently',
                 len(view.HEADROOM_TITLES) == 2
                 and len(set(view.HEADROOM_TITLES)) == 2,
                 str(view.HEADROOM_TITLES))

    # A COLD MOTOR HAS ALL OF ITS MARGIN, a cooking one has none, and
    # neither ever leaves the scale - a headroom below zero would draw a
    # bar longer than its own track.
    for celsius, want in ((20.0, 1.0), (view.WINDING_SCALE_C, 0.0),
                          (view.WINDING_SCALE_C + 80.0, 0.0)):
        got = view.motor_headroom_of(celsius)
        report.check('a winding at %.0f C leaves %.0f %% of the scale'
                     % (celsius, 100.0 * want),
                     abs(got - want) < 0.02, '%.3f' % got)

    half = (20.0 + view.WINDING_SCALE_C) / 2.0
    report.check('and half way up the scale is half the margin',
                 abs(view.motor_headroom_of(half) - 0.5) < 0.02,
                 '%.3f at %.0f C' % (view.motor_headroom_of(half), half))


def test_the_flat_drawings_spend_the_block(report):
    """The 2D drawings antialias their fringe instead of rounding it up.

    A DOT IS ONE BIT AND `SUBDOT` SAMPLES FOUR CORNERS. Read as "any", a
    shape covering a quarter of a dot lit it whole - so every arc came
    out a dot fatter than it is and the can's rim stepped against the
    magnets inside it. Read as COVERAGE, half a dot or more still lights
    outright and the fringe beyond that is dithered, which is what puts
    the patterns between solid and blank on the page.
    """
    from coaxial import dial, machine, raster

    of = len(raster.SUBDOT)
    report.check('a dot the shape covers lights wherever it is',
                 all(raster.dithered(of, of, x, y)
                     for x in range(4) for y in range(4)))
    report.check('a dot it misses never does',
                 not any(raster.dithered(0, of, x, y)
                         for x in range(4) for y in range(4)))
    report.check('half a dot always lights - a one-dot rim is a line',
                 all(raster.dithered(2, of, x, y)
                     for x in range(4) for y in range(4)))
    quarter = [raster.dithered(1, of, x, y)
               for x in range(4) for y in range(4)]
    report.check('a quarter of one lights at a quarter of the positions',
                 sum(quarter) == 4, '%d of 16' % sum(quarter))

    # AND THE DRAWINGS ARE RICHER FOR IT: the rotor and the protractor
    # both raster through the same rule, so both wear patterns a fringe
    # rounded up to solid could never produce.
    art = machine.render(0.0, 24, 28, 46, 18)
    face = dial.render(137.0, 60, 20)
    for name, drawn in (('the rotor', art), ('the protractor', face)):
        seen = {c for c in drawn if 0x2800 < ord(c) < 0x2900}
        report.check('%s draws more than a handful of patterns' % name,
                     len(seen) >= 40, '%d distinct' % len(seen))
        report.check('%s draws partial cells, not only solid ones' % name,
                     any(0 < bin(ord(c) - 0x2800).count('1') < 8
                         for c in seen))


def test_the_soa_gauge_pulses_only_when_the_board_acts(report):
    """The alarm is the envelope acting, not a level this page picked.

    A red bar cannot get redder, so a stage being HELD BACK by its own
    envelope looked exactly like one sitting near a limit. The pulse is
    the difference. What it keys on is `throttling` and `tripped`, which
    the board reports out of the ceilings its record gave it - the page
    inventing a threshold to flash at would be the page judging a
    reading (invariant 10).
    """
    sys.path.insert(0, HOST)
    from tools import show_rotor_observer as view

    report.check('an idle board does not pulse', not view.flashing({}))
    report.check('nor does one merely close to a limit - near is not an '
                 'event',
                 not view.flashing({'budget': {'worst': 0.99,
                                               'throttling': False}}))

    for flag in ('throttling', 'tripped'):
        seen = set()
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            seen.add(view.flashing({'budget': {flag: True}}))
            time.sleep(0.01)
        report.check('while %s it pulses - both halves inside a second at '
                     '%.0f Hz' % (flag, view.FLASH_HZ),
                     seen == {True, False}, str(sorted(seen)))


def main():
    report = Report()
    print('\n-- every view, two frames, no board --')
    test_each_view_draws_two_frames(report)
    print('\n-- the rotor observer\'s geometry --')
    test_the_instruments_stand_clear_of_the_machine(report)
    test_each_gutter_says_its_hottest_node(report)
    test_both_gutters_run_on_one_scale(report)
    test_a_power_node_never_reads_below_the_copper(report)
    test_the_ntc_is_shown_as_the_one_measurement(report)
    test_two_headrooms_named_apart(report)
    test_the_soa_gauge_pulses_only_when_the_board_acts(report)
    test_the_flat_drawings_spend_the_block(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
