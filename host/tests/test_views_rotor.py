"""The rotor observer's instruments: its gutters, the SOA, the foot's policy and headrooms."""
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report, rows_of


def test_the_instruments_stand_clear_of_the_motor(report):
    """The gutters equidistant, and the foot gauges off the can."""
    from coaxial.draw import cross_section
    from terminal.views.rotor import layout

    width, height = layout.BOX.width, layout.BOX.rows
    n_left, n_right = len(layout.SOA_NODES), len(layout.BOARD_NODES)
    frame, _lit = cross_section._raster(
        6.0, 24, 28, width, height, None, None, None,
        [(0.4, cross_section.SOA_OK)] * n_left, [(0.3, cross_section.SOA_OK)] * n_right,
        [(0.5, cross_section.SOA_OK)],
        [(0.4, cross_section.SOA_WARN), (0.3, cross_section.WATTS)], 2.0)
    can = [col for row in range(height) for col in range(width)
           if frame.owner[row][col] == cross_section.CAN]
    left, right = cross_section.gutters(width, height, n_left, n_right)
    gaps = (min(can) - max(left) - 1, min(right) - max(can) - 1)
    report.check('the gutters stand the same distance off the motor',
                 gaps[0] == gaps[1], 'left %d, right %d columns' % gaps)
    report.check('both groups fit inside the frame',
                 len(left) == n_left and len(right) == n_right,
                 '%d of %d left, %d of %d right'
                 % (len(left), n_left, len(right), n_right))

    can_rows = rows_of(frame.owner, width, height, cross_section.CAN)
    for name, kind in (('winding', cross_section.SOA_WARN),
                       ('power', cross_section.WATTS)):
        on = rows_of(frame.owner, width, height, kind)
        report.check('the %s gauge clears the can' % name,
                     bool(on) and not set(on) & set(can_rows),
                     'gauge rows %s, can rows %d..%d'
                     % (on, min(can_rows), max(can_rows)))


def test_each_gutter_says_its_hottest_node(report):
    """The caption's third row, in degrees, and which node each one is."""
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import layout, legend, thermal

    nodes = dict.fromkeys(layout.SOA_NODES + layout.BOARD_NODES, 30.0)
    nodes['phase_v'], nodes['regulators'], nodes['board'] = 118.4, 71.2, 44.5
    # `used` for every node, because `soa_bars` draws only what the board
    # reported a spend for - a node with no ceiling in the record is a node it
    # says nothing about.
    said = {'thermal': {'nodes': nodes, 'ambient': 20.0},
            'budget': {'used': {name: (nodes[name] - 20.0) / 105.0
                                for name in nodes},
                       'tripped': False}}
    said['budget']['used']['phase_v'] = 0.95
    peak, cls = legend.hottest(said, layout.SOA_NODES)
    report.check('the switch caption takes the hottest leg',
                 peak == 118.4, 'said %s' % (peak,))
    report.check('and its colour comes from that same node margin',
                 cls == view.cross_section.SOA_WARN, 'class %s' % (cls,))

    peak, _ = legend.hottest(said, layout.BOARD_NODES)
    report.check('the board caption takes the hottest of its four, which '
                 'is the tube standing tallest beside it',
                 peak == 71.2, 'said %s' % (peak,))
    # The caption names the tallest tube, not the copper.
    bars = thermal.soa_bars(said, layout.BOARD_NODES)
    tallest = max(zip(layout.BOARD_NODES, bars), key=lambda p: p[1][0])[0]
    report.check('and it names the tallest tube, not some other node',
                 nodes[tallest] == peak,
                 '%s at %.1f C' % (tallest, nodes[tallest]))
    report.check('a group with nothing measured says nothing',
                 legend.hottest({}, layout.SOA_NODES)[0] is None)


def test_both_gutters_run_on_one_scale(report):
    """Height is degrees, colour is margin, and they are two questions."""
    from coaxial.draw import cross_section
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import layout, thermal

    nodes = dict.fromkeys(layout.SOA_NODES + layout.BOARD_NODES, 20.0)
    nodes['phase_u'] = nodes['board'] = 100.0
    said = {'thermal': {'nodes': nodes, 'ambient': 20.0},
            'budget': {'used': {'phase_u': 80.0 / 105.0,
                                'board': 80.0 / 85.0},
                       'tripped': False}}
    left = thermal.soa_bars(said, ('phase_u',))[0]
    right = thermal.soa_bars(said, ('board',))[0]
    report.check('two nodes at one temperature draw one height',
                 abs(left[0] - right[0]) < 1e-9,
                 '%.4f against %.4f' % (left[0], right[0]))
    report.check('and the copper still colours hotter, because its ceiling '
                 'is lower - the margin is what the colour carries',
                 left[1] == cross_section.SOA_OK and right[1] == cross_section.SOA_WARN,
                 'phase %s, board %s' % (left[1], right[1]))
    report.check('the scale is stated, not taken from a limit the board '
                 'acts on: -35 to 130 on the bench\'s word, one ruler for '
                 'every tube',
                 (view.TEMP_FLOOR_C, view.TEMP_SCALE_C) == (-35.0, 130.0)
                 and view.NTC_COLD_C == view.TEMP_FLOOR_C
                 and view.NTC_HOT_C == view.TEMP_SCALE_C,
                 '%.0f to %.0f C' % (view.TEMP_FLOOR_C, view.TEMP_SCALE_C))
    report.check('and the room sits on the tube, not under it',
                 abs(view.temp_share(25.0) - 60.0 / 165.0) < 1e-9,
                 '%.3f' % view.temp_share(25.0))


def test_a_power_node_never_reads_below_the_copper(report):
    """It sheds into the board, so it cannot be colder than the board - switching: AFE_ON up,
    the STO chain's pilot detector on +5, the drivers on the chain's supply."""
    from coaxial import SIMULATED, Coaxial63100
    from terminal.views.rotor import layout, legend

    rig = Coaxial63100(execution_mode=SIMULATED)
    rig.open()
    try:
        rig.board.afe.on()
        rig.board.gate_drivers.configure(bypass_break=True)
        rig.board.gate_drivers.on()
        rig.drive.hold()
        rig.drive.write(iq_ref=30.0)
        worst = None
        for _ in range(8):
            time.sleep(0.15)
            said = {'thermal': rig.thermal.state(),
                    'budget': rig.thermal.budget()}
            switch = legend.hottest(said, layout.SOA_NODES)[0]
            copper = said['thermal']['nodes']['board']
            gap = switch - copper
            worst = gap if worst is None else min(worst, gap)
        report.check('no power node ever reads below the copper it sheds '
                     'into - the thing the gutters looked like they denied',
                     worst is not None and worst >= -1e-6,
                     'closest %.4f K' % (worst if worst is not None else float('nan')))
    finally:
        rig.close()


def test_the_ntc_is_shown_as_the_one_measurement(report):
    """The reference above the headroom scale, and what it says unread."""
    from coaxial.draw import cross_section
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import layout, legend

    report.check('a reading is shown in degrees',
                 legend.reference({'thermal': {'ntc': 38.04}})
                 == 'NTC 38.0 %sC' % legend.DEGREE,
                 legend.reference({'thermal': {'ntc': 38.04}}))
    # AFE_ON low is not a cold board.
    for empty in ({'thermal': {'ntc': None}}, {}):
        report.check('and no reading says so rather than drawing a number',
                     'unread' in legend.reference(empty),
                     legend.reference(empty))

    # `simulated` and `spin` are view keys the real page always carries; the
    # caption reaches the winding estimate through the margin rows, and
    # that asks how fast the stand-in's clock is running.
    rows = legend.gutter_caption({
        'simulated': True, 'spin': 0.0,
        'thermal': {'nodes': dict.fromkeys(layout.SOA_NODES + layout.BOARD_NODES,
                                           40.0),
                    'ambient': 20.0, 'ntc': 38.0},
        'budget': {'used': {}, 'tripped': False},
        'state': {'id': 0.0, 'iq': 0.0, 'vd': 0.0, 'vq': 0.0},
        'params': {}, 'winding_at': None})
    # The first caption row, with a tube of its own.
    said = rows[0]
    report.check('it opens the stack, above every estimate',
                 'NTC' in said and not any('NTC' in row
                                           for row in rows[1:]),
                 said.replace(chr(27), '^'))
    # Its own tube's colour, the thermometer ramp - blue at the cold
    # end and red at the hot - because the thermistor has no ceiling to be a
    # margin against.
    report.check('and takes its own tube colour, off the thermometer ramp',
                 any('38;5;%d' % cross_section.INK[step] in said
                     for step in cross_section.NTC_RAMP),
                 said.replace(chr(27), '^'))


def test_two_headrooms_named_apart(report):
    """The board's margin and the motor's are different facts."""
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import layout, thermal

    report.check('both scales are named, and named differently',
                 len(layout.HEADROOM_TITLES) == 2
                 and len(set(layout.HEADROOM_TITLES)) == 2,
                 str(layout.HEADROOM_TITLES))

    # A motor at the scale's floor has all of its margin, a cooking one has
    # none, and neither ever leaves the scale - a headroom below zero would
    # draw a bar longer than its own track.
    for celsius, want in ((view.TEMP_FLOOR_C, 1.0), (view.TEMP_SCALE_C, 0.0),
                          (view.TEMP_SCALE_C + 80.0, 0.0)):
        got = thermal.motor_headroom_of(celsius)
        report.check('a winding at %.0f C leaves %.0f %% of the scale'
                     % (celsius, 100.0 * want),
                     abs(got - want) < 0.02, '%.3f' % got)

    half = (view.TEMP_FLOOR_C + view.TEMP_SCALE_C) / 2.0
    report.check('and half way up the scale is half the margin',
                 abs(thermal.motor_headroom_of(half) - 0.5) < 0.02,
                 '%.3f at %.0f C' % (thermal.motor_headroom_of(half), half))


def test_the_foot_carries_the_policy(report):
    """TH OBS and the policy between WINDING and POWER, in the margin's
    colours, and nothing moves when the power goes negative.
    """
    from coaxial.draw import cross_section
    from terminal.ui.screen import plain as visible      # the row without its inks
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import layout, legend, thermal

    def a_view(watts, ident):
        # `watts(view)` is 1.5 (vd id + vq iq) off the loop's means; a volt of
        # vq makes the current the power, and its sign.
        return {'simulated': True, 'spin': 0.0,
                'thermal': {'nodes': dict.fromkeys(
                    layout.SOA_NODES + layout.BOARD_NODES, 40.0),
                    'ambient': 20.0, 'ntc': 38.0},
                'budget': {'used': {}, 'tripped': False},
                'state': {'id': 0.0, 'iq': watts / 1.5, 'vd': 0.0,
                          'vq': 1.0, 'vdc': 24.0},
                'params': {}, 'winding_at': None, 'ident': ident}

    # The margin is a number the board sends, continuous since 2026-09-06;
    # these three are what the states meant while it was three steps.
    IDENT_MARGIN = {'UNCERTAIN': 0.80, 'CONVERGING': 0.90, 'STABLE': 1.0}
    stable = {'state': 'STABLE', 'margin': 1.0}
    foot = legend.gutter_caption(a_view(20.0, stable))[-1]
    plain = visible(foot)
    report.check('the foot row names WINDING, TH OBS with its state, and '
                 'POWER, in that order, and is the art\'s width',
                 plain.find('WINDING') < plain.find('TH OBS STABLE')
                 < plain.find('POWER') and len(plain) == layout.BOX.width,
                 '%d: %s' % (len(plain), plain))
    at = plain.find('TH OBS')
    inks, trims = {}, {}
    for state in ('STABLE', 'CONVERGING', 'UNCERTAIN'):
        row = legend.gutter_caption(a_view(20.0, {
            'state': state, 'margin': IDENT_MARGIN[state]}))[-1]
        inks[state] = ('38;5;%dm%s' % (cross_section.INK[thermal.POLICY_INK[state]],
                                       thermal.POLICY_WORD[state])) in row
        trims[state] = visible(row)
    # The trim is said while there is one (the bench: "make it visible that it
    # throttles at 80 % of the SOA already, then 90, then 100 as the model's
    # uncertainty goes to zero"), and the row stays its width.
    report.check('the ceiling it leaves is on the row: UNCR 80%, CONV 90%, '
                 'and STABLE alone at the whole span',
                 'TH OBS UNCR 80%' in trims['UNCERTAIN']
                 and 'TH OBS CONV 90%' in trims['CONVERGING']
                 and 'TH OBS STABLE ' in trims['STABLE']
                 and all(len(t) == layout.BOX.width for t in trims.values()),
                 ' | '.join(trims.values()))
    report.check('the word wears the margin\'s ink: STABLE green, CONV '
                 'yellow, UNCR red - and TH OBS the leaders\' grey',
                 all(inks.values())
                 and ('38;5;%dmTH OBS' % cross_section.LEADER_GREY) in foot,
                 '%s %s' % (inks, foot.replace(chr(27), '^')))
    # The percent is the margin rounded, whatever the word: a
    # CONVERGING board at 0.93 says so, a STABLE one at 0.97 as STBL, since
    # `STABLE 97%` is a cell wider than the row has between the gauges' names -
    # and the row keeps its width and its columns.
    between = {}
    for state, margin in (('CONVERGING', 0.93), ('STABLE', 0.97),
                          ('UNCERTAIN', 0.812)):
        between[state] = visible(legend.gutter_caption(a_view(20.0, {
            'state': state, 'margin': margin}))[-1])
    report.check('a margin between the steps is said to the percent: CONV '
                 '93%, STBL 97%, UNCR 81% - and the row keeps its width',
                 'TH OBS CONV 93%' in between['CONVERGING']
                 and 'TH OBS STBL 97%' in between['STABLE']
                 and 'TH OBS UNCR 81%' in between['UNCERTAIN']
                 and all(len(t) == layout.BOX.width
                         and t.find('POWER') == plain.find('POWER')
                         for t in between.values()),
                 ' | '.join(between.values()))
    negative = visible(legend.gutter_caption(a_view(-20.0, stable))[-1])
    report.check('and a negative kilowatt shifts nothing: POWER, TH OBS '
                 'and the arrows stay where they are',
                 negative.find('POWER') == plain.find('POWER')
                 and negative.find('TH OBS') == at
                 and len(negative) == len(plain) and '-0.02 kW' in negative,
                 negative)
    absent = visible(legend.gutter_caption(a_view(20.0, None))[-1])
    report.check('and a dash before the board has answered op 10',
                 'TH OBS -' in absent and absent.find('POWER')
                 == plain.find('POWER'), absent)
    # Three digits on the winding.
    hot = a_view(20.0, stable)
    hot['budget']['winding_c'] = 123.4
    three = visible(legend.gutter_caption(hot)[-1])
    report.check('and a three-digit winding pushes nothing: TH OBS and '
                 'POWER keep their columns',
                 'WINDING 123.4' in three and three.find('TH OBS') == at
                 and three.find('POWER') == plain.find('POWER')
                 and len(three) == layout.BOX.width, three)


def test_the_foot_says_trip_while_the_cap_holds(report):
    """`TRIP 72%` in the trip's red while the trip cap holds the margin
    under the floor, whatever the model's state - the bench seeing STBL
    at 70 % of SOA (2026-09-08), a number no state can give; the state's
    own word with the percent in force once the cap is over the floor -
    the bench's point that it must let go of TRIP once over 80 %, its
    line being `WINDING 97.7 C TH OBS TRIP 89%` (the same day); and the
    state's word and the model's number once the cap has recovered past
    the model.
    """
    from coaxial.draw import cross_section
    from terminal.views.rotor import thermal

    def foot(state, margin, cap, floor=None):
        ident = {'state': state, 'margin': margin, 'trip_cap': cap}
        if floor is not None:
            ident['margin_floor'] = floor
        return thermal._policy({'ident': ident})

    label, word, ink = foot('STABLE', 0.72, 0.72)
    report.check('the trip cap in hand under the floor says TRIP with the '
                 'capped percent, in the trip\'s red',
                 word == 'TRIP 72%' and ink == cross_section.INK[cross_section.SOA_TRIP],
                 (word, ink))
    label, word, ink = foot('STABLE', 0.89, 0.89)
    report.check('over the floor the cap still in hand leaves the word to '
                 'the state, its percent the one in force: STBL 89%, not '
                 'TRIP 89%',
                 word == 'STBL 89%' and ink == cross_section.INK[cross_section.SOA_OK],
                 (word, ink))
    report.check('and the floor is the wire\'s: under a floor of 0.90 the '
                 'same 0.89 is still the trip\'s',
                 foot('STABLE', 0.89, 0.89, floor=0.90)[1] == 'TRIP 89%',
                 foot('STABLE', 0.89, 0.89, floor=0.90)[1])
    label, word, ink = foot('STABLE', 0.90, 1.0)
    report.check('no trip: the state\'s word and the margin',
                 word == 'STBL 90%' and ink == cross_section.INK[cross_section.SOA_OK],
                 (word, ink))
    label, word, ink = foot('CONVERGING', 0.90, 0.95)
    report.check('a cap that has recovered past the identification leaves '
                 'the word to the model', word == 'CONV 90%', word)
    label, word, ink = foot('UNCERTAIN', 0.80, 1.0)
    report.check('and a board before MINOR 17 answers no cap and reads as '
                 'before', foot('UNCERTAIN', 0.80, 1.0)[1] == 'UNCR 80%'
                 and thermal._policy({'ident': {'state': 'UNCERTAIN',
                                             'margin': 0.8}})[1] == 'UNCR 80%',
                 word)


def test_the_soa_gauge_pulses_only_when_the_board_acts(report):
    """The alarm is the envelope acting, not a level this page picked."""
    from terminal.views.rotor import thermal

    report.check('an idle board does not pulse', not thermal.flashing({}))
    report.check('nor does one merely close to a limit - near is not an '
                 'event',
                 not thermal.flashing({'budget': {'worst': 0.99,
                                               'throttling': False}}))

    for flag in ('throttling', 'tripped'):
        seen = set()
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            seen.add(thermal.flashing({'budget': {flag: True}}))
            time.sleep(0.01)
        report.check('while %s it pulses - both halves inside a second at '
                     '%.0f Hz' % (flag, thermal.FLASH_HZ),
                     seen == {True, False}, str(sorted(seen)))


def test_the_mode_says_whether_the_board_holds_it_back(report):
    """`HOLD (NORM)`, `SENSORLESS (THR)`: the envelope's state beside the
    mode.
    """
    from rich.text import Text
    from coaxial.draw import cross_section
    from terminal.views.rotor import rows, thermal

    def said(mode, budget=None):
        raw = rows.mode_text({'state': {'mode': mode}, 'budget': budget})
        return Text.from_ansi(raw).plain, raw

    plain, raw = said('off')
    report.check('off, the drive is STOPPED', plain == 'STOPPED', plain)
    plain, raw = said('hold')
    report.check('holding with no budget answered, HOLD (NORM)',
                 plain == 'HOLD (NORM)', plain)
    report.check('and nothing in it wears the throttle red',
                 ('38;5;%d' % rows.THROTTLE_RED) not in raw,
                 raw.replace(chr(27), '^'))
    plain, raw = said('sensorless', {'throttling': True})
    report.check('throttling, SENSORLESS (THR)',
                 plain == 'SENSORLESS (THR)', plain)
    report.check('with THR in the throttle red',
                 ('38;5;%dm(THR)' % rows.THROTTLE_RED) in raw,
                 raw.replace(chr(27), '^'))
    report.check('tripped is held back too',
                 said('hold', {'tripped': True})[0] == 'HOLD (THR)')
    report.check('and a clamp merely open is NORM',
                 said('hold', {'throttling': False, 'derate': 1.0})[0]
                 == 'HOLD (NORM)')

    # The red is a darker one: in the 6x6x6 cube, less red and no green or blue
    # - not the trip's 196, not the pulse's 210.
    def cube(index):
        i = index - 16
        return i // 36, i // 6 % 6, i % 6

    ours, trip = cube(rows.THROTTLE_RED), cube(cross_section.INK[cross_section.SOA_TRIP])
    report.check('THR is a red darker than the trip',
                 ours[1] == 0 and ours[2] == 0 and ours[0] < trip[0],
                 '%s against %s' % (ours, trip))
    report.check('and the word and the pulse take the same verdict',
                 thermal.envelope_acting({'budget': {'throttling': True}})
                 and not thermal.envelope_acting({'budget': {'derate': 0.5}})
                 and not thermal.flashing({'budget': None}))


def test_switch_soa_is_the_switches_and_motor_soa_the_winding(report):
    """The two gutter tubes read two different things: the worst of the six
    switch nodes, and the winding.
    """
    from terminal.views.rotor import layout, thermal

    used = {n: 0.3 for n in layout.SOA_NODES}
    used.update({'patch_u': 0.78, 'winding': 0.6, 'board': 0.2})
    seen = {'budget': {'worst': 0.78, 'worst_node': 'patch_u', 'used': used,
                       'winding_used': 0.6, 'throttling': False,
                       'tripped': False},
            'ident': {'margin': 1.0}, 'thermal': {'nodes': {}}}
    (switch, _), (motor, _) = thermal.headrooms(seen)
    report.check("SWITCH SOA is the worst of the six switch nodes, not the "
                 "board's worst",
                 abs(switch - 0.3) < 1e-9
                 and abs((1.0 - thermal.headroom(seen)) - 0.78) < 1e-9,
                 (switch, 1.0 - thermal.headroom(seen)))
    report.check("and MOTOR SOA is the winding's, so the tubes differ when "
                 "the winding is the worst node",
                 abs(motor - 0.6) < 1e-9 and switch != motor, (switch, motor))
    bare = {'budget': {'worst': 0.5, 'throttling': False, 'tripped': False},
            'ident': {}}
    report.check('a board that reports no per-node spend falls back to its '
                 'worst',
                 abs(thermal.switch_headroom(bare) - 0.5) < 1e-9,
                 thermal.switch_headroom(bare))


ROSTER = (test_the_instruments_stand_clear_of_the_motor, test_each_gutter_says_its_hottest_node,
          test_both_gutters_run_on_one_scale, test_a_power_node_never_reads_below_the_copper,
          test_the_ntc_is_shown_as_the_one_measurement, test_two_headrooms_named_apart,
          test_the_foot_carries_the_policy, test_the_foot_says_trip_while_the_cap_holds,
          test_the_soa_gauge_pulses_only_when_the_board_acts,
          test_the_mode_says_whether_the_board_holds_it_back,
          test_switch_soa_is_the_switches_and_motor_soa_the_winding)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
