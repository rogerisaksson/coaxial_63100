"""Every live view runs two frames against the stand-in, as a subprocess."""
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)

#: Every view, with the flags its two-frame run needs. Read off terminal/views/
#: rather than hardcoded where possible - a new show_*.py joins by existing.
EXTRA = {
    'show_session.py': [],
    'menu.py': [],
    'show_capture.py': [],
    'show_desk.py': [],
    'show_gate_drivers.py': [],
    'show_orientation.py': ['--width', '72', '--height', '14'],
    'show_angle.py': ['--scales'],
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
    """The views under terminal/views/, plus the session and the front page."""
    got = sorted(name for name in os.listdir(os.path.join(HOST, 'terminal', 'views'))
                 if name.startswith('show_') and name.endswith('.py'))
    return ['show_session.py', 'menu.py'] + got


#: A view gets this long to draw its two frames and exit.
VIEW_TIMEOUT = 120


def run_view(name):
    """One view run against the stand-in for two frames; its whole process
    tree killed at the timeout - a view's crew workers held the captured
    pipe otherwise, and the suite sat on it (2026-09-13)."""
    from tools.dev.runner import run_captured
    where = 'terminal' if name == 'menu.py' else os.path.join('terminal', 'views')
    done = run_captured(
        [sys.executable, '-X', 'utf8', os.path.join(where, name),
         '--simulated', '--frames', '2'] + EXTRA.get(name, []),
        VIEW_TIMEOUT, cwd=HOST)
    if done is None:
        raise subprocess.TimeoutExpired(name, VIEW_TIMEOUT)
    return done


def test_the_crt_draws_on_the_terminal(report):
    """Through a screen-mode Live - the terminal's path, which gives its renderable no
    height - the CRT draws: snow over the screen, the braille band at the beam."""
    import io
    from rich.console import Console
    from rich.live import Live
    from rich.text import Text
    from terminal.ui import chrome

    def dots(t):
        out = io.StringIO()
        court = Console(file=out, force_terminal=True, width=100, height=30,
                        color_system='truecolor', legacy_windows=False)
        was = chrome.time.monotonic
        chrome.time.monotonic = lambda: t
        try:
            with Live(console=court, screen=True, auto_refresh=False, transient=True) as live:
                live.update(chrome.Crt(Text('hello')), refresh=True)
        finally:
            chrome.time.monotonic = was
        return sum(1 for ch in out.getvalue() if 0x2801 <= ord(ch) <= 0x28FF)
    quiet, beam = dots(0.0), dots(chrome.SWEEP_S * 0.5)
    report.check('the CRT draws through a screen-mode Live: snow, and more at the beam',
                 quiet > 0 and beam > quiet, '%d dots, %d with the beam mid-screen'
                 % (quiet, beam))


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


def test_the_loader_reads_the_pages(report):
    """The terminal is what lies under terminal/pages/: the loader lists the
    pages in ORDER with unique keys, the front page draws that very list,
    every name a page or an item answers to is found, every page runs in
    this process for two frames on the stand-in and answers 0, and
    `python -m terminal --frames 2` - the front page with the preload
    underneath - exits 0.
    """
    import argparse
    from terminal import loader
    from terminal import menu
    entries, sub, opens, picks = loader.listing()
    keys = [key for key, _h, _w in entries]
    report.check('loader: seven pages, keys unique, SESSION first',
                 len(entries) == 7 and len(set(keys)) == 7
                 and entries[0][1] == 'SESSION', str(entries))
    report.check('loader: the front page draws the loader\'s list',
                 menu.ENTRIES == entries and menu.SUB == sub and menu.OPEN == opens,
                 str(menu.ENTRIES[:2]))
    names = sorted(loader.names())
    report.check('loader: every name answers - the pages and the items',
                 names == sorted(['session', 'imu', 'angle', 'adc', 'gate_drivers',
                                  'rotor_observer', 'thermal_observer', 'chat',
                                  'claude'])
                 and all(loader.by_name(n)[1] == n for n in names), str(names))
    args = argparse.Namespace(port='COM4', simulated=True, frames=2)
    for code, (page, name) in sorted(picks.items()):
        got = loader.run_page(page, name, args)
        report.check('loader: %s runs here for two frames and answers 0' % name,
                     got == 0, 'answered %r' % (got,))
    from tools.dev.runner import run_captured
    done = run_captured([sys.executable, '-X', 'utf8', '-m', 'terminal',
                         '--simulated', '--frames', '2'], VIEW_TIMEOUT, cwd=HOST)
    tail = ((done.stdout + done.stderr).strip().splitlines() or ['no output'])[-1][:70] \
        if done is not None else 'timed out'
    report.check('loader: python -m terminal --frames 2 exits 0',
                 done is not None and done.returncode == 0
                 and 'Traceback' not in done.stdout + done.stderr, tail)


def rows_of(owner, width, height, kind):
    """Which rows of a rendered drawing carry `kind`."""
    return [row for row in range(height)
            if any(owner[row][col] == kind for col in range(width))]


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
    """It sheds into the board, so it cannot be colder than the board."""
    from coaxial import Coaxial63100
    from terminal.views.rotor import layout, legend

    rig = Coaxial63100(simulated=True)
    rig.open()
    try:
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


def test_the_soa_legend_reads_the_whole_soa(report):
    """SWITCH SOA and MOTOR SOA say how much of the record's SOA is spent,
    and flash red where the ceiling in force is.
    """
    from coaxial.draw import cross_section
    from coaxial.model import thermal
    from terminal.views.rotor import thermal as rotor
    IDENT_MARGIN = {'UNCERTAIN': 0.80, 'CONVERGING': 0.90, 'STABLE': 1.0}

    def a_view(state, worst, tripped=False, winding_used=None):
        budget = {'worst': worst, 'tripped': tripped,
                  'throttling': worst >= 0.9}
        if winding_used is not None:
            budget['winding_used'] = winding_used
            budget['winding_derate'] = 1.0 if winding_used < 0.9 else 0.5
        return {'thermal': {'nodes': {'driver_u': 60.0}}, 'budget': budget,
                'ident': {'state': state, 'margin': IDENT_MARGIN[state]},
                'params': {}, 'winding_at': None,
                'state': {'id': 0.0, 'iq': 0.0, 'vd': 0.0, 'vq': 0.0}}

    reads = {}
    for state in ('UNCERTAIN', 'CONVERGING', 'STABLE'):
        spent, cls = rotor.headrooms(a_view(state, 1.0, tripped=True))[0]
        reads[state] = (spent, cls)
    report.check('a board at the ceiling its policy leaves reads 80 % of '
                 'the SOA UNCERTAIN, 90 CONVERGING, 100 STABLE',
                 abs(reads['UNCERTAIN'][0] - 0.80) < 1e-9
                 and abs(reads['CONVERGING'][0] - 0.90) < 1e-9
                 and abs(reads['STABLE'][0] - 1.00) < 1e-9,
                 ' '.join('%s %.2f' % (s, v[0]) for s, v in reads.items()))
    report.check('and in the trip\'s red, pulsing, since the board is '
                 'acting there',
                 all(cls in (cross_section.SOA_TRIP, cross_section.SOA_FLASH)
                     for _spent, cls in reads.values()),
                 [cls for _s, cls in reads.values()])
    spent, cls = rotor.headrooms(a_view('UNCERTAIN', 0.5))[0]
    report.check('half way to that ceiling reads 40 % of the SOA and is '
                 'neither red nor pulsing - the amber there is the '
                 'gauge\'s own band, not the policy\'s',
                 abs(spent - 0.40) < 1e-9
                 and cls not in (cross_section.SOA_TRIP, cross_section.SOA_FLASH),
                 '%.2f cls %d' % (spent, cls))
    motor = rotor.headrooms(a_view('UNCERTAIN', 0.2, winding_used=1.0))[1]
    report.check('the winding\'s own legend the same way, off the board\'s '
                 'winding under the same policy, and pulsing when the '
                 'board holds the stage back for it',
                 abs(motor[0] - 0.80) < 1e-9
                 and motor[1] in (cross_section.SOA_TRIP, cross_section.SOA_FLASH),
                 '%.2f cls %d' % motor)
    report.check('the ceiling in force is the record\'s span trimmed: the '
                 'laminate\'s 105 is 89 C at the 0.8 floor',
                 abs(thermal.ceiling_of('board', 0.8) - 89.0) < 1e-9
                 and abs(thermal.ceiling_of('board', 1.0) - 105.0) < 1e-9,
                 '%.1f' % thermal.ceiling_of('board', 0.8))
    # A board at the ceiling a margin of 0.86 leaves reads 86 % of the SOA: the
    # legend follows the number, not the word.
    view = a_view('CONVERGING', 1.0, tripped=True)
    view['ident']['margin'] = 0.86
    spent, _cls = rotor.headrooms(view)[0]
    report.check('and a margin between the steps reads to the percent: 0.86 '
                 'at the ceiling in force is 86 % of the SOA',
                 abs(spent - 0.86) < 1e-9, '%.2f' % spent)


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


def test_the_headroom_box_carries_a_solid_bar_with_a_tip(report):
    """The thermal observer's spend is HEADROOM, its level one row of `⣿`
    ending in an orange `⡇` or `⢸`, labelled `soak` (the bench's word): not
    BUDGET's `[⣿⣿⠒⠒] 42 %`, nor three rows of braille.
    """
    import re
    from rich.console import Console

    from coaxial.draw import cross_section, gauges
    from machine import ansi
    from terminal.views import show_thermal_observer as page
    from terminal.ui import stage

    half = gauges.bar(0.5, 16)
    line = re.sub('\x1b\\[[0-9;]*m', '', half)
    report.check('a bar is one row of sixteen cells',
                 len(line) == 16, str(len(line)))
    report.check('solid ⣿ to half way, then the tip in the lane the '
                 'level ends in - ⡇ - then the track, a grey column the '
                 'cell\'s full height in every cell',
                 line[:8] == '⣿' * 8 and line[8] == '⡇'
                 and line[9:] == '⡇' * 7, line)
    report.check('the tip is orange and the track is the track\'s grey',
                 '38;5;%dm' % ansi.AMBER in half
                 and '38;5;%dm' % cross_section.INK[cross_section.TRACK] in half,
                 half.replace(chr(27), '^'))
    odd = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(17.0 / 32.0, 16))
    report.check('a level ending in the other lane tips with ⢸, the '
                 'tip\'s cell holding the tip alone',
                 odd[:8] == '⣿' * 8 and odd[8] == '⢸', odd)
    empty = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(0.0, 16))
    full = re.sub('\x1b\\[[0-9;]*m', '', gauges.bar(1.0, 16))
    report.check('nothing spent is a tip at the start of a full-height '
                 'track; everything, a solid row to a tip at the end',
                 empty == '⡇' * 16
                 and full[:15] == '⣿' * 15 and full[15] == '⢸',
                 '%s | %s' % (empty, full))

    state = {'nodes': {}, 'ntc': None, 'seconds': 3, 'settled': False,
             'seen_s_ago': None, 'sample_every_s': 5.0, 'mcu': 40.0,
             'afe': None, 'ambient': 25.0}
    budget = {'worst': 0.42, 'worst_node': 'phase_v',
              'seconds_to_limit': 12.0, 'throttling': False,
              'tripped': False, 'used': {}}
    console = Console(record=True, width=44, force_terminal=True,
                      color_system='truecolor', theme=stage.THEME)
    console.print(page.status_boxes(state, budget)[2])   # SENSE, MAP, then HEADROOM
    said = re.sub('\x1b\\[[0-9;]*m', '', console.export_text(styles=True))
    report.check('the box is HEADROOM, and the level is labelled soak',
                 'HEADROOM' in said and 'soak' in said and 'BUDGET' not in said,
                 said)
    braille_rows = [l for l in said.splitlines()
                    if any(0x2800 <= ord(ch) < 0x2900 for ch in l)]
    report.check('one braille row, no brackets, the figure beside it',
                 len(braille_rows) == 1 and '[' not in said
                 and '42 %' in said and '⣿' in said, said)


def test_the_thermal_page_shows_its_evidence(report):
    """Under the board a bar of the span the model has earned - red to
    yellow to green as it fills - with the innovation and the margin; and
    SENSE one fact a row.
    """
    import re
    from rich.console import Console

    from coaxial.draw import cross_section
    from coaxial.simulated.thermal.observer import SimulatedThermal
    from terminal.ui.screen import plain as visible
    from terminal.views import show_thermal_observer as page
    from terminal.ui import stage

    def ident(margin, state='CONVERGING'):
        return {'state': state, 'margin': margin, 'margin_floor': 0.8,
                'innovation_k': 0.17,
                'scales': {'air': 1.64, 'capacity': 0.95, 'spread': 1.0,
                           'ntc': 1.0},
                'sigma': {'air': 0.28, 'capacity': 0.10, 'spread': 0.5,
                          'ntc': 0.3},
                'online': ['air', 'capacity'], 'ambient': 24.5,
                'ambient_sigma': 4.2, 'updates': 3, 'saves': 0,
                'since_save_s': None,
                'truth': {'situation': 'box', 'air': 2.0, 'capacity': 1.0,
                          'ambient': 25.0, 'since_s': 240.0, 'load_a': 30.0}}

    rows = page.evidence_rows(ident(0.8))
    said = visible(rows[1])
    # The bar alone (the bench: "remove the text to the right of the scale,
    # move it to HEADROOM"); its figures are `envelope_rows`.
    report.check('two rows under the board: a blank, then TH OBS and the '
                 'bar alone - its figures are HEADROOM\'s',
                 len(rows) == 2 and rows[0] == ''
                 and said.startswith('   TH OBS ') and len(said.split()) == 3
                 and page.envelope_rows(ident(0.8)) == [
                     ('margin', '0.80'), ('floor', '0.80'),
                     ('innovation', '0.17 K'), ('doubt', 'air 0.37')],
                 '%s | %s' % (said, page.envelope_rows(ident(0.8))))

    def bar_of(margin):
        return visible(page.evidence_rows(ident(margin))[1]).split()[2]

    inks, greys = {}, {}
    for margin, cls in ((0.8, None), (0.9, cross_section.SOA_WARN),
                        (1.0, cross_section.SOA_OK)):
        row = page.evidence_rows(ident(margin))[1]
        head, _bar = row.split('TH OBS')[0], row.split('TH OBS')[1]
        # The label constant in the leaders' grey (the bench: "only the
        # thermometer changes colour"), the bar's ink after it.
        greys[margin] = ('38;5;%dm' % cross_section.LEADER_GREY) in head
        inks[margin] = (cls is None or ('38;5;%dm' % cross_section.INK[cls])
                        in row.split('TH OBS')[1])
    report.check('empty at the floor - the tip and the track alone - half '
                 'full at 0.90 in yellow, full at the whole span in green: '
                 'red to yellow to green as it fills, and TH OBS in the '
                 'leaders\' grey whatever the bar wears',
                 all(inks.values()) and all(greys.values())
                 and bar_of(0.8).count('⣿') == 0
                 and 9 <= bar_of(0.9).count('⣿') <= 10
                 and bar_of(1.0).count('⣿') == page.GAUGE_CELLS - 1
                 and page.PAGE_CYCLE_ON_S < SimulatedThermal.CYCLE_ON_S
                 and page.PAGE_CYCLE_OFF_S < SimulatedThermal.CYCLE_OFF_S,
                 '%s %s | %s %s %s' % (inks, greys, bar_of(0.8),
                                       bar_of(0.9), bar_of(1.0)))
    report.check('and a dash before the board has answered op 10',
                 'TH OBS -' in visible(page.evidence_rows(None)[1]),
                 visible(page.evidence_rows(None)[1]))
    # The room's hint, on the estimated room: the bench's emoji pairs, and the
    # thermometer thinking while the innovation is large.
    import unicodedata

    def at(room, innovation=0.1):
        return page.room_hint({'ambient': room, 'innovation_k': innovation})

    report.check('the hint above the board shivers under 5 C, is mild to '
                 '35 and sweats from there - on the ESTIMATED room - thinks '
                 'while the innovation is three floors or more, and is '
                 'nothing before the board has said a room',
                 at(-25.0) == 'cold' and at(20.0) == 'mild'
                 and at(34.9) == 'mild' and at(45.0) == 'hot'
                 and at(20.0, 0.3) == 'unsure' and at(-25.0, 2.0) == 'unsure'
                 and page.room_hint(None) == '' and page.room_hint({}) == ''
                 and page.ROOM_HINTS['unsure'] == '🤒 🤔'
                 and all(' ' in pair for pair in page.ROOM_HINTS.values())
                 # Every glyph wide on its own, no variation selector: a narrow
                 # character made emoji by one ran the row a cell long and
                 # broke the frame beside it.
                 and all(len(pair) == 3 and all(
                     unicodedata.east_asian_width(ch) == 'W'
                     for ch in pair.replace(' ', ''))
                     for pair in page.ROOM_HINTS.values()),
                 ' '.join(page.ROOM_HINTS[at(c)] for c in (-25.0, 20.0, 45.0)))
    # Hysteresis (the bench: "so the emoji do not flutter near the limits"): a
    # held word stands two kelvin past its threshold, and the thermometer
    # stands until the innovation is under 0.2 K.
    def held(room, word, innovation=0.1):
        return page.room_hint({'ambient': room, 'innovation_k': innovation},
                              held=word)

    report.check('a held word stands two kelvin past its threshold - cold '
                 'at 6.5, hot at 33.5, mild at 3.5 and 36.5 - and lets go '
                 'beyond that, and the thermometer stands until the '
                 'innovation is under 0.2 K',
                 held(6.5, 'cold') == 'cold' and held(7.5, 'cold') == 'mild'
                 and held(33.5, 'hot') == 'hot' and held(32.5, 'hot') == 'mild'
                 and held(3.5, 'mild') == 'mild' and held(36.5, 'mild') == 'mild'
                 and held(2.5, 'mild') == 'cold' and held(37.5, 'mild') == 'hot'
                 and held(20.0, 'unsure', 0.25) == 'unsure'
                 and held(20.0, 'unsure', 0.15) == 'mild'
                 and held(20.0, 'mild', 0.25) == 'mild',
                 ' '.join(held(c, 'cold') for c in (6.5, 7.5)))
    # In SENSE, beside the room (the bench: "maybe move the emojis to the
    # SENSE block on the right, a bit more uniform").
    rows = page.ident_rows(ident(0.91))
    room = [value for label, value in rows if label == 'room'][0]
    report.check('and the hint sits in SENSE beside the room, the pair '
                 'after the figure',
                 room.plain.startswith('24.5 ±4.2 C')
                 and room.plain.endswith(page.ROOM_HINTS['mild']),
                 room.plain)

    # SENSE: one fact a row, none wider than the panel.
    rows = page.ident_rows(ident(0.91))
    texts = [(str(label), value if isinstance(value, str) else value.plain)
             for label, value in rows]
    # `sim`, not `truth` - the bench: only in simulated mode is the thermal
    # situation known.
    report.check('SENSE carries the identification one fact a row - model, '
                 'air, cap, room, the simulation in two and the load - none '
                 'wider than the panel',
                 [l for l, _v in texts] == ['model', 'air', 'cap', 'room',
                                            'sim', '', 'load']
                 and all(len(l) + 1 + len(v) <= page.PANEL_W - 4
                         for l, v in texts),
                 texts)
    state = {'nodes': {}, 'ntc': 59.8, 'error': 0.54, 'seconds': 240,
             'settled': True, 'seen_s_ago': 4.0, 'sample_every_s': 30.0,
             'mcu': 72.0, 'afe': 40.0, 'ambient': 25.0}
    budget = {'worst': 0.42, 'worst_node': 'phase_v',
              'seconds_to_limit': 12.0, 'throttling': False,
              'tripped': False, 'used': {}}
    console = Console(record=True, width=page.PANEL_W + 2,
                      force_terminal=True, color_system='truecolor',
                      theme=stage.THEME)
    # The map's letters explained: a box of its own under SENSE, each row the
    # mark's references off the pick and place and what they are.
    from coaxial.draw.thermalmap import MARKS
    rows = dict(page.map_rows())
    report.check('MAP says what every mark is - U, V, W, REG, MCU, HS, AFE '
                 'and NTC - with the references its frame is drawn round',
                 [label for label, _r, _w, _m in MARKS]
                 == ['MCU', 'REG', 'U', 'V', 'W', 'AFE', 'HS', 'NTC']
                 and all(label in rows for label in ('U', 'V', 'W', 'REG',
                                                     'MCU', 'HS', 'AFE'))
                 and rows['U'].startswith('Q1U Q2U RU1 RU2 - FETs')
                 and 'hot swap' in rows['HS'] and 'STM32' in rows['MCU']
                 and rows['AFE'].startswith('OP1U..OP2W (6)')
                 # three cells of label, a space, the value, inside the panel's
                 # frame and padding
                 and all(len(value) <= page.PANEL_W - 8
                         for value in rows.values()),
                 rows)
    sense, headroom = page.status_boxes(state, budget, ident=ident(0.91))[:3:2]
    console.print(sense)
    console.print(headroom)
    said = re.sub('\x1b\\[[0-9;]*m', '', console.export_text(styles=True))
    lines = [l for l in said.splitlines()]
    report.check('drawn at the panel\'s width nothing is cropped: the NTC '
                 'and its error, the sample interval and the last sample '
                 'are rows of their own, and HEADROOM carries the margin, '
                 'the floor and the innovation under the soak',
                 '…' not in said
                 and any('NTC 59.8 C' in l for l in lines)
                 and any('err +0.54 K' in l for l in lines)
                 and any('sample 30 s' in l for l in lines)
                 and any('last 4 s ago' in l for l in lines)
                 and any('margin 0.91' in l for l in lines)
                 and any('floor 0.80' in l for l in lines)
                 and any('innovation 0.17 K' in l for l in lines)
                 and said.find('soak') < said.find('margin 0.91'),
                 said)


def test_the_attitude_caps_its_frame_rate(report):
    """BOARD ATTITUDE draws at most HZ_CAP frames a second whatever
    `--hz` asks - the bench's word, so the laptop's fans stay down -
    and never slower than one every two seconds."""
    from terminal.views import show_orientation as view

    report.check('the cap is thirty', view.HZ_CAP == 30.0, str(view.HZ_CAP))
    report.check('--hz 60 draws at thirty',
                 abs(view.period_of(60.0) - 1.0 / 30.0) < 1e-9,
                 '%.4f s' % view.period_of(60.0))
    report.check('--hz 20, the default, is honoured',
                 abs(view.period_of(20.0) - 0.05) < 1e-9
                 and view.parse_args(['--simulated']).hz == 20.0,
                 '%.4f s' % view.period_of(20.0))
    report.check('and nothing slower than one frame every two seconds',
                 view.period_of(0.0) == 2.0, '%.4f s' % view.period_of(0.0))


def test_the_thermal_map_is_a_halftone_with_its_parts_marked(report):
    """The thermal observer's board is braille: a blue-noise stipple denser
    where it is hotter, in the ramp blended to 24 bits, the rim a dot
    wide, and the parts that make the heat drawn on it as blocks with
    edges and a label - placed from the pick and place.
    """
    import re
    from coaxial.draw import thermalmap
    from machine import ansi
    from coaxial.model.thermal import ALL_NODES

    warm = {n: 40.0 for n in ALL_NODES if n != 'board'}
    cells = 88

    def picture(nodes):
        return thermalmap.render(nodes, board_c=30.0, cells=cells,
                                 colour=True, reserve=0, trailing=0)

    def strip(text):
        return re.sub('\x1b\\[[0-9;]*m', '', text)

    def dots(text):
        return sum(bin(ord(ch) - 0x2800).count('1') for ch in text
                   if 0x2800 <= ord(ch) < 0x2900)

    said = picture(warm)
    rows = strip(said).split('\n')
    art = [row[:cells] for row in rows]
    letters = set(''.join(mark[0] for mark in thermalmap.MARKS))
    stray = set(ch for row in art for ch in row
                if ch != ' ' and not 0x2800 <= ord(ch) < 0x2900
                and ch not in letters)
    report.check('the board is braille, and the only letters on it are '
                 'the labels', not stray, repr(''.join(sorted(stray))))
    for label, _refs, _where, _margin in thermalmap.MARKS:
        report.check('%s is written on it' % label,
                     any(label in row for row in art))

    # Round, and nothing past the rim: every lit cell's centre inside the
    # radius plus a cell, the top row narrow, the middle row the width.
    per_cell = 2.0 * thermalmap.OUTER_MM / cells
    per_line = 4.0 * thermalmap.OUTER_MM / (2 * len(art))
    worst = 0.0
    spans = []
    for r, row in enumerate(art):
        lit = [c for c, ch in enumerate(row) if ch != ' ']
        spans.append((lit[-1] - lit[0] + 1) if lit else 0)
        for c in lit:
            x = (c - (cells - 1) / 2.0) * per_cell
            y = ((len(art) - 1) / 2.0 - r) * per_line
            worst = max(worst, math.hypot(x, y))
    report.check('nothing is lit past the rim',
                 worst <= thermalmap.OUTER_MM + per_line, '%.1f mm' % worst)
    report.check('and it is round: narrow at the top, the full width in '
                 'the middle',
                 spans[0] < cells // 2 and max(spans) >= cells - 2,
                 'spans %d .. %d of %d' % (spans[0], max(spans), cells))

    hot = picture(dict(warm, mcu=95.0))
    report.check('a hotter MCU is a denser halftone',
                 dots(strip(hot)) > dots(strip(said)) + 40,
                 '%d dots against %d' % (dots(strip(hot)),
                                         dots(strip(said))))
    report.check('the ramp is blended to 24 bits on the field',
                 '38;2;' in said)
    report.check('the frames and the labels wear the mark ink',
                 '38;5;%dm' % ansi.WHITE in said)

    # Frames, not areas (the bench's word): a marked cell draws the line's
    # dots alone, so no marked cell is solid and the marks are a thin share of
    # the board.
    white = ansi.rgb(ansi.WHITE)
    lit = [(ch, fg) for row in ansi.parse(said) for ch, fg, _bg in row
           if 0x2800 <= ord(ch) < 0x2900]
    marked = [ch for ch, fg in lit if fg == white]
    # A solid marked cell is two frames' sides sharing a cell column - REG's
    # right and the MCU's left are a millimetre apart - and nothing else: a
    # handful, never an area.
    solid = sum(dots(ch) == 8 for ch in marked)
    report.check('a marked cell is a line, never a solid block',
                 marked and solid <= 0.02 * len(marked),
                 '%d solid of %d' % (solid, len(marked)))
    # A quarter: the rim alone is two fifths of the marks, and eight frames'
    # perimeters the rest - lines, however many of them.
    report.check('and the frames are a thin share of the board',
                 0 < len(marked) < 0.25 * len(lit),
                 '%d marked of %d lit' % (len(marked), len(lit)))
    # Right angles: the frames are box-drawing in braille, the bench's own
    # glyphs - corners, and straight runs between them.
    corners = {pair: sum(marked.count(ch) for ch in pair)
               for pair in ('⡖⢰', '⢲⡆', '⠧⠸', '⠼⠇')}
    runs = {ch: marked.count(ch) for ch in '⠒⠤⡇⢸'}
    report.check('the frames have right-angled corners - '
                 '⡖⠒⠒⢲ over ⠧⠤⠤⠼, or ⢰ ⡆ ⠸ ⠇ where the side is in the '
                 'inner lane',
                 all(n >= 5 for n in corners.values()), str(corners))
    report.check('and straight sides between them',
                 all(n >= 10 for n in runs.values()), str(runs))

    rail = [row[cells + 2:cells + 4] for row in rows if len(row) > cells + 3]
    report.check('the scale beside it is braille too, denser at the hot '
                 'end than the cold',
                 rail and all(0x2800 <= ord(ch) < 0x2900
                              for cell in rail for ch in cell)
                 and dots(rail[0]) > dots(rail[-1]),
                 '%s .. %s' % (rail[:1], rail[-1:]))
    cold = ansi.thermal_rgb(ansi.THERMAL_MIN)
    report.check('and the cold end of the scale is blue, not black',
                 cold[2] >= 128 and cold[0] == 0 and cold[1] == 0, str(cold))

    plain = thermalmap.render(warm, board_c=30.0, cells=40, colour=False,
                              reserve=0, trailing=0).split('\n')
    report.check('a pipe still gets the character ramp',
                 all(ch in thermalmap.RAMP + ' ' for ch in plain[0][:80])
                 and any(ch in thermalmap.RAMP for row in plain
                         for ch in row[:80]),
                 plain[len(plain) // 2][:80])


def test_the_demo_actually_loads_the_motor(report):
    """Two hundred frames of the stand-in warm the winding and spend the
    switches' margin - the demo puts a load on, and it stays on.
    """
    import re

    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    # No `-P`: it arrived in Python 3.11, and on the 3.10 runner CI declares as
    # its floor it is "unknown option", exit 2.
    done = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('terminal', 'views', 'show_rotor_observer.py'),
         '--simulated', '--frames', '200'],
        cwd=HOST, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=300)
    out = done.stdout + done.stderr
    winding = re.search(r'WINDING +([0-9.]+)', out)   # %5.1f: a space at two digits
    soa = re.search(r'SWITCH SOA ([0-9.]+) %', out)
    report.check('the view ran two hundred frames simulated',
                 done.returncode == 0 and winding and soa,
                 'exit %d' % done.returncode)
    if winding and soa:
        report.check('the winding is warm - the demo\'s load is on',
                     float(winding.group(1)) >= 45.0,
                     '%s C, floor 45' % winding.group(1))
        report.check('and the switches have spent a fifth of their margin',
                     float(soa.group(1)) >= 20.0,
                     '%s %%, floor 20' % soa.group(1))


def test_the_power_face_has_its_middle_at_half_a_kilowatt(report):
    """The kW bar is a power law pinned at 500 W, full at 2 kW, red past."""
    from coaxial.draw import cross_section
    from terminal.views.rotor import thermal

    at = {w: thermal.watts_share(w) for w in (0, 20, 100, 500, 2000, 2500)}
    report.check('nothing draws nothing', at[0][0] == 0.0)
    report.check('twenty watts is a tenth of the bar - a small draw is seen',
                 abs(at[20][0] - 0.1) < 0.01, '%.3f' % at[20][0])
    report.check('and a hundred is not yet a quarter',
                 0.2 < at[100][0] < 0.25, '%.3f' % at[100][0])
    report.check('half a kilowatt is half the bar',
                 abs(at[500][0] - 0.5) < 1e-9, '%.3f' % at[500][0])
    report.check('two kilowatts is the whole of it, still in its own ink',
                 at[2000] == (1.0, cross_section.WATTS), str(at[2000]))
    report.check('and past it the bar is full and deep red',
                 at[2500] == (1.0, cross_section.SOA_TRIP), str(at[2500]))
    report.check('the middle is a named constant, not a magic exponent',
                 thermal.WATTS_MID == 500.0 and thermal.WATTS_SCALE == 2000.0)


def test_the_level_is_drawn_at_the_dot(report):
    """The top of a bar's mercury is `⣀`, `⣤`, `⣶` - one dot a step, every LED_PITCH-th
    dark: LED segments."""
    from coaxial.draw import cross_section

    track = chr(0x28D2)
    tops, dots = [], []
    for k in range(0, 8):
        share = (k + 0.5) / 40.0          # a ten-row tube is forty dots
        art = cross_section.render(0.0, 24, 28, 30, 12,
                             left=[(share, cross_section.SOA_OK)]).split(chr(10))
        column = [row[0] for row in art]
        mercury = [c for c in column if c not in (track, chr(0x2800))]
        tops.append(mercury[0] if mercury else '?')
        dots.append(sum(bin(ord(c) - 0x2800).count('1') for c in mercury))
    report.check('the top cell climbs a dot at a time, its fourth dot a segment gap',
                 tops[:4] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                              chr(0x28F6)], ''.join(tops))
    report.check('and keeps climbing into the next cell the same way',
                 tops[4:8] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                               chr(0x28F6)], ''.join(tops))
    gap = cross_section.LED_PITCH - 1
    report.check('two dots a step - both lanes - none at a gap, never a whole cell',
                 all(b - a == (0 if (i + 1) % cross_section.LED_PITCH == gap else 2)
                     for i, (a, b) in enumerate(zip(dots, dots[1:]))), str(dots))

    # Along the foot, one lane at a time.
    ends = []
    for k in range(1, 5):
        share = (k + 0.5) / 60.0
        frame, _lit = cross_section._raster(
            0.0, 24, 28, 30, 12, None, None, None, None, None, None,
            [(share, cross_section.WATTS)], 2.0)
        row = frame.height - 1
        level = [col for col in range(frame.width)
                 if frame.owner[row][col] == cross_section.WATTS]
        ends.append(chr(0x2800 + frame.dots[row][max(level)]) if level
                    else '?')
    report.check('the foot gauge ends on a lane, not a cell, its fourth dot a gap',
                 ends == [chr(0x2807), chr(0x283F), chr(0x2807), chr(0x2807)],
                 ''.join(ends))


def test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most(
        report):
    """The air gap is less than a cell tall, and that is a trade the drawing
    makes on purpose.
    """
    from coaxial.draw import cross_section

    magnet = {cross_section.NORTH, cross_section.SOUTH}
    teeth = {cross_section.TOOTH_U, cross_section.TOOTH_V, cross_section.TOOTH_W}
    seat = cross_section.Seat(46, 18, None, None, None, None, None, None, 2.0)
    report.check('the teeth reach their full fraction of the radius',
                 abs(seat.radii.tooth_out
                     - seat.radii.can * cross_section.F_TOOTH_OUT) < 1e-9,
                 '%.2f of %.2f' % (seat.radii.tooth_out,
                                   seat.radii.can * cross_section.F_TOOTH_OUT))
    mixed = elsewhere = 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = cross_section._raster(
                6.0, 24, 28, 46, 18, None,
                cross_section._drive((30.0, -15.0, -15.0)), None,
                None, None, None, None, aspect)
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if tally and set(tally) & magnet and set(tally) & teeth:
                        mixed += 1
                        most = max(tally, key=lambda c: (tally[c], c))
                        elsewhere += frame.owner[row][col] != most
    report.check('cells holding both exist - the gap is under a cell',
                 mixed > 0, '%d cells' % mixed)
    report.check('and every one goes to whichever has more of it',
                 elsewhere == 0, '%d did not' % elsewhere)


def test_a_line_keeps_the_cell_it_shares_with_an_area(report):
    """A ring through a cell full of tooth or magnet keeps its colour."""
    import math

    from coaxial.draw import cross_section

    magnet = {cross_section.NORTH, cross_section.SOUTH}
    seat = cross_section.Seat(46, 18, None, None, None, None, None, None, 2.0)
    r = seat.radii
    yoke_worst, lost = 1.0, 0
    for aspect in (2.0, 2.3):
        for deg in (0.0, 6.0, 12.0, 18.0):
            frame, _lit = cross_section._raster(
                deg, 24, 28, 46, 18, None,
                cross_section._drive((30.0, -15.0, -15.0)), None,
                None, None, None, None, aspect)
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if (tally and cross_section.CAN in tally and set(tally) & magnet
                            and frame.owner[row][col] != cross_section.CAN):
                        lost += 1
            ring = set()
            for k in range(720):
                phi = math.radians(k / 2.0)
                x = seat.cx + r.tooth_in * math.cos(phi)
                y = seat.cy - r.tooth_in * math.sin(phi) / (aspect / 2.0)
                ring.add((int(y) // 4, int(x) // 2))
            own = sum(1 for row, col in ring
                      if frame.owner[row][col] in (cross_section.YOKE, cross_section.BORE))
            yoke_worst = min(yoke_worst, own / len(ring))
    report.check('the yoke ring is wholly its own colour where the teeth '
                 'root', yoke_worst >= 0.99, 'worst %.2f' % yoke_worst)
    report.check('and no can-ring cell shared with a magnet is lost to it',
                 lost == 0, '%d cells' % lost)

    # The rule itself, on one cell: a line with one dot beats an area with
    # seven; two lines settle by dots; two areas settle by dots.
    frame = cross_section.Frame(1, 1)
    for k in range(7):
        frame.put(k % 2, k // 2, cross_section.NORTH)
    frame.put(1, 3, cross_section.CAN)
    report.check('one dot of ring outweighs seven of magnet',
                 frame.owner[0][0] == cross_section.CAN)
    frame = cross_section.Frame(1, 1)
    for k in range(6):
        frame.put(k % 2, k // 2, cross_section.TOOTH_U)
    frame.put(0, 3, cross_section.TOOTH_W)
    report.check('and between two areas the most dots win, not the rank',
                 frame.owner[0][0] == cross_section.TOOTH_U)
    # Not the truth stroke, which wins outright: `Frame.put` has why.
    report.check('the lines are the rings - not the arc, not the stroke',
                 cross_section.LINES == frozenset((cross_section.BORE, cross_section.YOKE,
                                             cross_section.CAN)))

    # The shaft sensor's stroke is drawn through the magnet band.
    teeth = {cross_section.TOOTH_U, cross_section.TOOTH_V, cross_section.TOOTH_W}
    gutter = set(range(0, 8)) | set(range(38, 46))
    took = in_gutter = 0
    own_min, ring_max = 999, 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = cross_section._raster(
                6.0, 24, 28, 46, 18, float(deg),
                cross_section._drive((30.0, -15.0, -15.0)), 41.0,
                [(0.3, cross_section.SOA_OK)] * 8, [(0.3, cross_section.SOA_OK)] * 8,
                None, [(0.3, cross_section.SOA_WARN), (0.3, cross_section.WATTS)],
                aspect)
            own = rings = 0
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if not tally or cross_section.TRUTH not in tally:
                        continue
                    if frame.owner[row][col] == cross_section.TRUTH:
                        took += bool(set(tally) & teeth)
                        own += 1
                        in_gutter += col in gutter
                        rings += (cross_section.CAN in tally
                                  or cross_section.YOKE in tally)
            own_min = min(own_min, own)
            ring_max = max(ring_max, rings)
    # A cell's diagonal still bridges the band's inner end and a tooth's tip at
    # some angles, so the stroke may share a cell with a tooth; in that cell it
    # is not a candidate, because a white cell on a tooth is a mark on the
    # stator.
    report.check('the truth stroke takes no cell a tooth is in',
                 took == 0, '%d cells' % took)
    report.check('and never lands in a gutter', in_gutter == 0,
                 '%d cells' % in_gutter)
    report.check('and is seen in every pose', own_min >= 1,
                 'fewest own cells %d' % own_min)
    report.check('and takes at most one cell of the rim, at its own angle',
                 ring_max <= 1, 'most in one pose %d' % ring_max)


def test_nothing_in_the_drawing_can_be_sheared(report):
    """No character in the art has East Asian ambiguous width.

    Unicode does not decide for those. A terminal set for East Asian text
    draws them two columns wide and every other one draws them narrow,
    and it is a setting rather than a font - so a page carrying one is a
    page that renders correctly on one bench and shears on the next.
    Sheared, the mark doubles, everything after it on the row slides a
    column, and the colour runs slide with it: the drawing bleeds inside
    its own box.

    Braille is narrow by definition, so what caught this out was the
    furniture: `\u25c0` and `\u25b6` as arrowheads, `\u25b2` and
    `\u25bc` at the foot, and the degree sign. All four triangles have
    unambiguous small twins and the degree has U+1D52.

    The bead is not among them - U+29BF is narrow, and the fallback built
    for it picked U+25CF, which is ambiguous. The safe substitute was the
    only unsafe character in the pair, and both are gone.
    """
    import unicodedata

    from coaxial.draw import cross_section
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import legend
    from terminal.ui import scroll

    drawn = cross_section.render(6.0, 24, 28, 46, 18, pointer_deg=41.0)
    # The scroll arrows are the stage's, every page's furniture.
    said = ''.join(str(x) for x in
                   (legend.AIM_LEFT, legend.AIM_RIGHT, scroll.UP, scroll.DOWN,
                    legend.DEGREE, legend.LEADER, cross_section.POINTER_GLYPH)
                   ) + ''.join(legend.TURN) + ''.join(legend.DROP)
    for name, text in (('the drawing', drawn), ("the view's furniture", said)):
        bad = sorted({c for c in text
                      if unicodedata.east_asian_width(c) == 'A'})
        report.check('%s carries no ambiguous-width character' % name,
                     not bad,
                     ' '.join('%s U+%04X' % (c, ord(c)) for c in bad))

    # The substitutes are the same marks, not near misses: a small triangle
    # points the same way as its big twin.
    report.check('the arrowheads are the small triangles',
                 (legend.AIM_LEFT, legend.AIM_RIGHT) == (chr(0x25C2), chr(0x25B8)),
                 legend.AIM_LEFT + legend.AIM_RIGHT)
    from terminal.ui import scroll
    report.check('and the foot uses their up and down - the stage\'s, which '
                 'every page\'s scroll markers wear too',
                 (scroll.UP, scroll.DOWN) == (chr(0x25B4), chr(0x25BE)),
                 scroll.UP + scroll.DOWN)


def test_the_flat_drawings_spend_the_block(report):
    """The 2D drawings place their edges by coverage, not by "any corner"."""
    from coaxial.draw import cross_section, dial
    from coaxial.graphics import raster

    of = len(raster.SUBDOT)
    report.check('a dot the shape covers lights',
                 raster.covered(of, of))
    report.check('a dot it misses never does',
                 not raster.covered(0, of))
    report.check('half a dot lights - a one-dot rim is a line the '
                 'drawing means', raster.covered(2, of))
    report.check('and a quarter of one does not, whatever the position',
                 not any(raster.covered(1, of, x, y)
                         for x in range(4) for y in range(4)))

    # The rotor and the protractor both raster through the same rule, so both
    # wear patterns a fringe rounded up to solid could never produce.
    art = cross_section.render(0.0, 24, 28, 46, 18)
    face = dial.render(137.0, 60, 20)
    for name, drawn in (('the rotor', art), ('the protractor', face)):
        seen = {c for c in drawn if 0x2800 < ord(c) < 0x2900}
        report.check('%s draws more than a handful of patterns' % name,
                     len(seen) >= 40, '%d distinct' % len(seen))
        report.check('%s draws partial cells, not only solid ones' % name,
                     any(0 < bin(ord(c) - 0x2800).count('1') < 8
                         for c in seen))


def test_every_gauge_shows_its_own_scale(report):
    """The dimmed track runs the whole of every bar, at its own width."""
    from coaxial.draw import cross_section

    n = 4
    art = cross_section.render(0.0, 24, 28, 46, 18,
                         left=[(0.0, cross_section.SOA_OK)] * n,
                         right=[(0.0, cross_section.SOA_OK)] * n,
                         bottom=[(0.0, cross_section.SOA_WARN), (0.0, cross_section.WATTS)])
    rows = art.split(chr(10))
    left, right = cross_section.gutters(46, 18, n, n)

    # Every tube, every row of it: down to the row of air over the floors.
    seen = set()
    for row in rows[1:-(2 + cross_section.FLOOR_AIR)]:
        for col in list(left) + list(right):
            seen.add(row[col])
    report.check('an empty tube is drawn in every one of its rows',
                 ' ' not in seen and chr(0x2800) not in seen,
                 ''.join(sorted(seen)))
    report.check('and every tube is drawn the same way',
                 len(seen) == 1, ''.join(sorted(seen)))
    report.check('at the tube\'s own width, both lanes',
                 all(ord(c) - 0x2800 & 0x08 or ord(c) - 0x2800 & 0x10
                     or ord(c) - 0x2800 & 0x20 or ord(c) - 0x2800 & 0x80
                     for c in seen), ''.join(sorted(seen)))

    # The flat gauges along the foot, one dot a cell rather than one every
    # other cell.
    first, last = cross_section.span(46, 18, n, n)
    floor = rows[-1]
    drawn = [floor[col] for col in range(first, last + 1)]
    report.check('the foot gauge draws a scale in every cell it spans',
                 all(c != ' ' and c != chr(0x2800) for c in drawn),
                 '%d of %d blank'
                 % (sum(1 for c in drawn if c in (' ', chr(0x2800))),
                    len(drawn)))


def test_the_bead_is_round_at_every_angle(report):
    """The pointer is `POINTER_GLYPH`, and it rides the rim."""
    from coaxial.draw import cross_section

    for aspect in (2.0, 2.4):
        was, seats = None, set()
        for deg in range(0, 360, 5):
            art = cross_section.render(0.0, 24, 28, 46, 18, aspect=aspect,
                                 pointer_deg=float(deg)).split(chr(10))
            at = [(r, line.index(cross_section.POINTER_GLYPH))
                  for r, line in enumerate(art)
                  if cross_section.POINTER_GLYPH in line]
            if len(at) != 1:
                was = '%d degrees: %d marks' % (deg, len(at))
                break
            if len(art[at[0][0]]) != len(art[0]):
                was = '%d degrees: its row came out a different length' % deg
                break
            seats.add(at[0])
        report.check('at aspect %.1f, one mark at every angle round the can'
                     % aspect, was is None, was or '72 angles')
        report.check('and it travels rather than sitting in a few seats',
                     len(seats) > 60, '%d distinct cells' % len(seats))

    # It rides the rim in the drawing's own space.
    for aspect in (2.0, 2.4):
        stretch = aspect / 4.0 * 2.0
        cx, r, _, _ = cross_section.layout(46, 18, 0, 0, rows=18)
        cy = 18 * 4 / 2.0 - 0.5
        seat = r.can + cross_section.POINTER_SEAT
        out = []
        for deg in range(0, 360, 5):
            phi = math.radians(deg)
            ax = cx + seat * math.cos(phi)
            ay = cy - seat * math.sin(phi) / stretch
            out.append(math.hypot(ax - cx, (cy - ay) * stretch))
        report.check('at aspect %.1f it sits one radius out at every angle'
                     % aspect, max(out) - min(out) < 1e-9,
                     '%.3f to %.3f against a rim at %.3f'
                     % (min(out), max(out), r.can))

    # The nearest cell centre, not the one the point fell inside.
    cx, r, _, _ = cross_section.layout(46, 18, 0, 0, rows=18)
    cy = 18 * 4 / 2.0 - 0.5
    seat = r.can + cross_section.POINTER_SEAT

    def worst(pick):
        out = 0.0
        for deg in range(360):
            phi = math.radians(deg)
            ax, ay = cx + seat * math.cos(phi), cy - seat * math.sin(phi)
            col, row = pick(ax, ay)
            out = max(out, math.hypot(col * 2 + 0.5 - ax, row * 4 + 1.5 - ay))
        return out

    near = worst(lambda x, y: (int(math.floor((x - 0.5) / 2 + 0.5)),
                               int(math.floor((y - 1.5) / 4 + 0.5))))
    cut = worst(lambda x, y: (int(x) // 2, int(y) // 4))
    report.check('the nearest cell centre beats the one it fell inside',
                 near < cut - 0.5, '%.2f dots against %.2f' % (near, cut))


def test_the_terminal_is_asked_how_tall_a_cell_is(report):
    """The cell's shape is measured, not assumed."""
    from terminal.ui import aspect

    report.check('a terminal 1200 by 800 pixels over 100 by 40 cells has a '
                 'cell 1.67 times as tall as it is wide',
                 abs((aspect.cell_aspect_of((800, 1200), (40, 100)) or 0)
                     - 5.0 / 3.0) < 1e-9)
    report.check('nothing divisible by zero comes back as a number',
                 aspect.cell_aspect_of((0, 0), (1, 1)) is None
                 and aspect.cell_aspect_of((800, 1200), (0, 100)) is None)
    report.check('and a reply that cannot be a cell is refused',
                 aspect.cell_aspect_of((8000, 100), (40, 100)) is None
                 and aspect.cell_aspect_of((10, 1200), (40, 100)) is None,
                 str(aspect.ASPECT_RANGE))
    report.check('a pipe is never asked, so the query cannot land in a '
                 'render', aspect.probe_aspect(console=False) is None)


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


def test_every_page_scrolls_its_boxes(report):
    """The instrument column pages on every view: the arrows move it a box,
    a click on its markers and a drag over it too, and the key bar says
    SCROLL only while there is somewhere to go.
    """
    from terminal.ui import stage
    from terminal.ui import scroll

    class Size:
        width, height = 100, 14

    class Console:
        is_terminal = True
        size = Size()

    console = Console()
    boxes = [stage.hud('BOX %d' % i, [('a', 1), ('b', 2), ('c', 3)])
             for i in range(6)]
    shown = scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('a short terminal shows what fits and says the rest are '
                 'below', at == 0 and 0 < seen < total == 6
                 and scroll.DOWN in shown[-1].plain,
                 '%d of %d shown' % (seen, total))
    scroll.scroll_by(console, 1)
    scroll.scroll_by(console, 1)
    shown = scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('two arrows down start two boxes in, with a row saying so',
                 at == 2 and scroll.UP in shown[0].plain, '%d above' % at)
    for _ in range(10):
        scroll.scroll_by(console, 1)
    scroll.paged(console, boxes)
    at, seen, total = scroll.scroll_state(console)['pages']
    report.check('and the bottom is a full column, not one box and air',
                 seen == total and at < total, '%d..%d of %d' % (at, seen, total))
    scroll.scroll_click(console, Size.width - 2, 2)
    scroll.paged(console, boxes)
    report.check('a click on the top marker goes up one',
                 scroll.scroll_state(console)['pages'][0] == at - 1)
    scroll.scroll_click(console, 10, 2)
    report.check('a click over the drawing takes no grip, so a drag there '
                 'does not scroll', not scroll.scroll_state(console)['grip'])
    before = scroll.scroll_state(console)['pages'][0]
    scroll.scroll_drag(console, 20.0)
    report.check('...and moves nothing', scroll.scroll_state(console)['at'] == before)

    class Pipe:
        is_terminal = False

    report.check('piped, nothing is windowed',
                 len(scroll.paged(Pipe(), boxes)) == 6)
    try:
        scroll.paged(True, boxes)
        refused = False
    except TypeError:
        refused = True
    report.check('and a view handing over its is_terminal flag instead of '
                 'the console is refused - where a piped test run sees it',
                 refused)


def test_the_dial_is_round_on_this_terminal(report):
    """The shaft angle's face takes the measured cell aspect, a notch under
    64 by 23.
    """
    from terminal.ui import aspect
    from coaxial.draw import dial
    from terminal.views import show_angle as view

    report.check('a given aspect wins, said as given',
                 aspect.aspect_of(2.3) == (2.3, 'given'))
    report.check('and the face is a notch smaller than 64 by 23',
                 view.ART_WIDTH < 64 and view.ART_HEIGHT < 23
                 and view.ART_HEIGHT >= 19,
                 '%d by %d' % (view.ART_WIDTH, view.ART_HEIGHT))

    def rows_of(aspect):
        lines = dial.render(0.0, view.ART_WIDTH, view.ART_HEIGHT, 100,
                            aspect=aspect).split('\n')
        return sum(1 for line in lines
                   if any(0x2800 < ord(c) <= 0x28FF for c in line))

    report.check('a taller cell draws the face over fewer rows - the '
                 'circle stays a circle on the screen',
                 rows_of(2.3) < rows_of(2.0),
                 '%d rows at 2.3 against %d at 2.0'
                 % (rows_of(2.3), rows_of(2.0)))


def test_the_face_wears_its_two_scales(report):
    """SHAFT ANGLE's die temperature and field stand either side of the face
    as tubes on their own ranges - the scales beside it the bench asked
    for, die temperature and field strength in gauss, 2026-09-07.
    """
    from coaxial.draw import dial
    from machine import ansi

    def dots(lines):
        return sum(bin(ord(c) - 0x2800).count('1')
                   for line in lines for c in line
                   if 0x2800 <= ord(c) <= 0x28FF)

    cold = dial.scale(-40.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                      '-40.0 C', dial.die_ink, 'left')
    warm = dial.scale(61.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                      '61.0 C', dial.die_ink, 'left')
    hot = dial.scale(150.0, dial.DIE_RANGE, 21, dial.DIE_TICKS, 'DIE',
                     '150.0 C', dial.die_ink, 'left')
    report.check('a scale is the face\'s rows and a caption, SCALE_W wide',
                 len(warm) == 22 and all(len(l) == dial.SCALE_W for l in warm)
                 and warm[0].strip() == 'DIE' and warm[-1].strip() == '61.0 C',
                 (len(warm), sorted({len(l) for l in warm}), warm[0], warm[-1]))
    report.check('its graduations are numbered, -40 at the foot and 150 at '
                 'the top',
                 '-40' in warm[-3] and '150' in warm[1]
                 and all(str(t) in ''.join(warm) for t in dial.DIE_TICKS),
                 [l[:5] for l in warm])
    report.check('the tube is four dots wide the whole way, glass and fill',
                 all(bin(ord(c) - 0x2800).count('1') == 8
                     for line in warm[1:-2] for c in line[5:7]),
                 [line[5:7] for line in warm[1:-2]])

    def inked(celsius):
        lines = dial.scale(celsius, dial.DIE_RANGE, 21, dial.DIE_TICKS,
                           'DIE', '', dial.die_ink, 'left', colour=True)
        return sum(ansi.code(dial.die_ink(celsius)) in line
                   for line in lines[1:-2])

    report.check('and the fill rises with the reading, the glass above it '
                 'in ash',
                 0 == inked(-40.0) < inked(61.0) < inked(150.0) == 19
                 and ansi.code(dial.LABEL_INK) in ''.join(
                     dial.scale(61.0, dial.DIE_RANGE, 21, dial.DIE_TICKS,
                                'DIE', '', dial.die_ink, 'left',
                                colour=True)[1:5]),
                 '%d < %d < %d rows inked' % (inked(-40.0), inked(61.0),
                                              inked(150.0)))
    inks = [dial.scale(g, dial.FIELD_RANGE, 21, dial.FIELD_TICKS, 'FIELD',
                       '%d G' % g, dial.field_ink, 'right', colour=True)
            for g in (12, 380, 1100)]
    report.check('the field tube is blue under the recommended band - a '
                 'weak magnet or none - green in it, red past it',
                 ansi.code(dial.BAND_INK[0]) in ''.join(inks[0])
                 and ansi.code(dial.BAND_INK[1]) in ''.join(inks[1])
                 and ansi.code(dial.BAND_INK[2]) in ''.join(inks[2])
                 and ansi.code(dial.BAND_INK[1]) not in ''.join(inks[0])
                 and dial.field_ink(200) == dial.BAND_INK[0],
                 [dial.field_ink(g) for g in (12, 200, 380, 1100)])
    report.check('and the die tube is blue under the board\'s working '
                 'range, green through it, red past it',
                 dial.die_ink(5.0) == dial.BAND_INK[0]
                 and dial.die_ink(25.0) == dial.BAND_INK[1]
                 and dial.die_ink(61.0) == dial.BAND_INK[1]
                 and dial.die_ink(90.0) == dial.BAND_INK[2],
                 [dial.die_ink(c) for c in (5.0, 25.0, 61.0, 90.0)])
    art = dial.instrument(137.0, 380, 273.15 + 61.0, colour=True).split('\n')
    report.check('the instrument is the face and two scales with their air, '
                 'line for line, and the die\'s reading wears its band',
                 len(art) == 22 and all(
                     len(ansi_plain(l)) == 58 + 2 * (dial.SCALE_W + 1)
                     for l in art)
                 and ansi.code(dial.die_ink(61.0)) in art[-1]
                 and '61.0 C' in ansi_plain(art[-1]),
                 (len(art), sorted({len(ansi_plain(l)) for l in art})))
    from terminal.views import show_angle as page
    report.check('and the face gives way to the scales and fills the rest: 68 wide at 130 '
                 'columns, FACE_MIN at 98, alone under that, as tall as the terminal leaves, '
                 'the whole face where the terminal would not say',
                 page.fit(130) == (True, 68, page.ART_HEIGHT)
                 and page.fit(100) == (True, 38, page.ART_HEIGHT)
                 and page.fit(98) == (True, page.FACE_MIN, page.ART_HEIGHT)
                 and page.fit(90) == (False, 46, page.ART_HEIGHT)
                 and page.fit(150, 44) == (True, 88, 44 - page.STAGE_ROWS)
                 and page.fit(0) == (False, page.ART_WIDTH, page.ART_HEIGHT)
                 and page.fit(0, 0, True) == (True, page.ART_WIDTH, page.ART_HEIGHT),
                 [page.fit(c) for c in (130, 100, 98, 90, 0)])
    report.check('and the caption leaves the gauss to the scale that shows it',
                 'gauss' not in dial.caption(137.0, 380, gauss=False)
                 and 'gauss' in dial.caption(137.0, 380)
                 and 'no magnet' in dial.caption(0.0, 12, gauss=False),
                 dial.caption(137.0, 380, gauss=False))


def ansi_plain(text):
    """`text` without its colour escapes."""
    import re
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


def test_the_bead_trails_its_speed(report):
    """The wake behind the bead: its length is the speed, its side the
    direction, and it fades from the bead's orange into the can's teal.
    """
    import re

    from coaxial.draw import cross_section
    from machine import ansi

    inks = {ansi.code(cross_section.INK[c]) for c in cross_section.TRAIL}

    def wake(rate):
        lines = cross_section.motor(0.0, width=60, height=30, pointer_deg=0.0,
                              pointer_rate=rate, colour=True)
        rows = []
        for row, line in enumerate(lines):
            for hit in re.finditer('(' + chr(27) + r'\[38;[25];[\d;]+m)([^' + chr(27)
                                   + ']*)', line):
                if hit.group(1) in inks:
                    rows += [row] * len(hit.group(2))
        bead = next(row for row, line in enumerate(lines)
                    if cross_section.POINTER_GLYPH in line)
        return rows, bead

    still, _ = wake(0.0)
    slow, bead = wake(360.0)
    fast, _ = wake(1200.0)
    back, _ = wake(-360.0)
    report.check('no wake at rest', not still, '%d cells' % len(still))
    report.check('and a longer one the faster the can turns',
                 0 < len(slow) < len(fast),
                 '%d cells at 360, %d at 1200' % (len(slow), len(fast)))
    report.check('behind the bead: at three o\'clock, below it turning '
                 'counter-clockwise and above it turning clockwise',
                 bool(slow and back)
                 and sum(slow) / len(slow) > bead > sum(back) / len(back),
                 'rows %.1f and %.1f about the bead\'s %d'
                 % (sum(slow) / max(1, len(slow)),
                    sum(back) / max(1, len(back)), bead))
    report.check('the bead wears the palette\'s orange, the north pole\'s',
                 cross_section.INK[cross_section.POINTER] == ansi.AMBER
                 == cross_section.INK[cross_section.NORTH])


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


def test_every_frame_corner_on_the_map_is_a_right_angle(report):
    """A frame's top and bottom lines start at the side's lane."""
    from coaxial.draw import thermalmap as tm

    # side, edge, the lane the side runs down -> the corner cell's glyph.
    right_angle = {('left', 'top', 0): '\u2856', ('left', 'top', 1): '\u28b0',
                   ('right', 'top', 1): '\u28b2', ('right', 'top', 0): '\u2846',
                   ('left', 'bottom', 0): '\u2827', ('left', 'bottom', 1): '\u2838',
                   ('right', 'bottom', 1): '\u283c', ('right', 'bottom', 0): '\u2807'}

    def glyph(rows, r, c):
        bits = 0
        for lane in (0, 1):
            for y in range(4):
                if rows[4 * r + y][2 * c + lane] == tm.MARK:
                    bits |= tm.BRAILLE_BITS[lane][y]
        return chr(tm.BRAILLE + bits)

    judged, wrong = 0, []
    for cells in (40, 48, 60, 72, 88):
        dx = dy = tm.OUTER_MM / cells
        rim, _ = tm._mask(cells, cells, ())
        for mark in tm.MARKS:
            label, refs, _where, margin = mark
            (c0, c1, r0, r1), lanes = tm._cell_rect(
                tm.frame(refs, margin), cells, cells, dx, dy)
            rows, _ = tm._mask(cells, cells, (mark,))
            for side, c, lane in (('left', c0, lanes[0]),
                                  ('right', c1, lanes[1])):
                for edge, r in (('top', r0), ('bottom', r1)):
                    if any(rim[4 * r + y][2 * c + lane_] != tm.FIELD
                           for lane_ in (0, 1) for y in range(4)):
                        continue
                    judged += 1
                    got = glyph(rows, r, c)
                    if got != right_angle[(side, edge, lane)]:
                        wrong.append('%d %s %s-%s %s' % (cells, label, edge,
                                                         side, got))
    report.check('every judged corner of every frame at every size is its '
                 'right angle: %d judged' % judged,
                 judged >= 120 and not wrong, '; '.join(wrong[:6]))

    # And the eight glyphs themselves, off a blank field, both lane
    # combinations: the lines meet the side and go no further.
    def drawn(lanes):
        rows = [[tm.FIELD] * 24 for _ in range(24)]
        tm._draw_frame(rows, [2, 8, 1, 4], lanes, 12, 12)
        return [glyph(rows, r, c) for r, c in ((1, 2), (1, 8), (4, 2), (4, 8))]
    report.check('a side in the outer lanes: ' + ' '.join(drawn([0, 1])),
                 drawn([0, 1]) == ['\u2856', '\u28b2', '\u2827', '\u283c'])
    report.check('a side in the inner lanes: ' + ' '.join(drawn([1, 0])),
                 drawn([1, 0]) == ['\u28b0', '\u2846', '\u2838', '\u2807'])


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


def test_a_frame_rasterises_as_the_terminal_draws_it(report):
    """`ansi.image` draws a coloured frame cell by cell, the way the bench's
    terminal shows it: the notebooks' pictures, and `tools/render/ansi2png.py`.
    """
    from machine import ansi

    frame = (ansi.paint('ab', ansi.RED) + 'c\n'
             + ansi.paint('\u28ff', ansi.GREEN) + ' \u2801')
    rows = ansi.parse(frame)
    report.check('parse keeps every cell and its colour',
                 [len(r) for r in rows] == [3, 3]
                 and rows[0][0][1] == ansi.rgb(ansi.RED)
                 and rows[0][2][1] == ansi.PLAIN
                 and rows[1][0][1] == ansi.rgb(ansi.GREEN),
                 [[(c, fg) for c, fg, _bg in r] for r in rows])
    cell = (8, 16)
    img = ansi.image(frame, cell=cell)
    report.check('and the image is one cell per character',
                 img.size == (3 * cell[0], 2 * cell[1]), img.size)
    px = img.load()
    if px is None:
        raise AssertionError('no pixels')

    def ink(x0, y0, wants):
        seen = [px[x, y] for x in range(x0, x0 + cell[0])
                for y in range(y0, y0 + cell[1])]
        return any(wants(p) for p in seen), seen

    red, _ = ink(0, 0, lambda p: p[0] > 150 and p[1] < 60 and p[2] < 60)
    green, _ = ink(0, cell[1], lambda p: p[1] > 150 and p[0] < 60 and p[2] < 60)
    lit, blank = ink(cell[0], cell[1], lambda p: p != (0, 0, 0))
    report.check('the red letters, the green braille and the blank cell '
                 'are drawn in their own inks',
                 red and green and not lit, (red, green, not lit))


def test_the_marquee_decodes_the_art_itself(report):
    """The viewport's art reaches rich as ready Segments, and the bytes on
    the console are the ones Text.from_ansi produced.
    """
    import io
    from rich.console import Console
    from rich.measure import Measurement
    from rich.text import Text
    from machine import ansi
    from terminal.ui import marquee, stage

    class ByText:
        """Text.from_ansi per line: the reference the Marquee is held to."""

        def __init__(self, art):
            self.lines = [Text.from_ansi(line) for line in art.split('\n')]
            for line in self.lines:
                line.no_wrap, line.overflow = True, 'crop'
            self.wide = max((l.cell_len for l in self.lines), default=0)

        def __rich_measure__(self, console, options):
            return Measurement(min(self.wide, options.max_width), self.wide)

        def __rich_console__(self, console, options):
            width = options.max_width
            extra = self.wide - width
            at = marquee._slide(extra) if extra > 0 else 0
            for line in self.lines:
                if at or line.cell_len > width:
                    line = line[at:at + width]
                    line.no_wrap, line.overflow = True, 'crop'
                yield line

    def drawn(marquee, width):
        sink = io.StringIO()
        court = Console(file=sink, force_terminal=True, legacy_windows=False,
                        color_system='truecolor', width=width, height=12,
                        theme=stage.THEME)
        court.print(stage.Panel(stage.Align(marquee, align='center',
                                            vertical='middle'),
                                box=stage.box.HEAVY, padding=(0, 1)))
        return sink.getvalue()

    art = '\n'.join((
        ansi.code((10, 20, 30)) + '⣿⣿' + ansi.code((10, 20, 31))
        + '⣿' + ansi.RESET + ' ab',
        ansi.code(214) + 'x' + ansi.back(17) + 'y' + '\x1b[39mz\x1b[49mw'
        + ansi.RESET,
        '\x1b[31mred\x1b[92mbright\x1b[44m on blue\x1b[0m plain',
        'no colour at all',
        '\x1b[1mbold\x1b[0m falls back to Text'))
    mine, theirs = marquee.Marquee(art), ByText(art)
    report.check('every colour form renders to the bytes the Text path '
                 'produced', drawn(mine, 40) == drawn(theirs, 40),
                 repr(drawn(mine, 40))[:200])
    report.check('the lines are Segments, and the bold line alone took '
                 'the Text road',
                 all(isinstance(l, list) for l in mine.lines[:4])
                 and isinstance(mine.lines[4], Text),
                 [type(l).__name__ for l in mine.lines])
    report.check('a run is one Segment per colour change - a reset before '
                 'text is a change, one at the end is not',
                 [len(l) for l in mine.lines[:4]] == [3, 4, 4, 1],
                 [len(l) for l in mine.lines[:4]])
    wide = ansi.code((1, 2, 3)) + 'x' * 20 + ansi.code(40) + 'y' * 20 + ansi.RESET
    slide = marquee._slide
    marquee._slide = lambda extra: 7
    try:
        report.check('cropped and slid, the same cells in the same bytes',
                     drawn(marquee.Marquee(wide), 24) == drawn(ByText(wide), 24),
                     repr(drawn(marquee.Marquee(wide), 24))[:200])
    finally:
        marquee._slide = slide


def test_the_preload_is_the_first_inquiry(report):
    """The front page fetches the model's decimates behind itself into one
    pickle under the user's local application data, and the readout's
    first inquiry says what is being loaded and the room it has.
    """
    import tempfile
    from terminal import readout
    from coaxial.draw import orientation
    from coaxial.graphics import preload
    identity = {'origin': 'simulated', 'real': False,
                'info': {'device': 'stand-in'}, 'parts': []}
    state = {'model': 'board.stl  5.8 mb', 'memory': '41.7 gb free',
             'disk': '120.0 gb free', 'steps': ['decimate grid 12: 765 triangles'],
             'status': 'building in the background'}
    pages = readout.pages(identity, 54, preload=state)
    flat = ' '.join(t for _title, rows in pages[:1] for row in rows for _s, t in row)
    report.check('preload: with a preload afoot the first inquiry is '
                 'PRELOAD, carrying the status',
                 pages[0][0] == 'PRELOAD' and 'BUILDING IN THE BACKGROUND' in flat.upper()
                 and pages[1][0] == 'IDENTITY', str([p[0] for p in pages]))
    with tempfile.TemporaryDirectory() as where:
        bundle = {'stamp': preload.stamp(orientation.MODEL), 'lods': {},
                  'exact': (None, []), 'prims': []}
        size = preload.save(bundle, where=where)
        back = preload.load(orientation.MODEL, where=where)
        bundle['stamp'] = ('elsewhere',) + bundle['stamp'][1:]
        preload.save(bundle, where=where)
        stale = preload.load(orientation.MODEL, where=where)
        why = preload.refusal(where)
    report.check('preload: a saved bundle loads back on the model\'s stamp '
                 'and not on another', size > 0 and back is not None
                 and back['stamp'] == preload.stamp(orientation.MODEL)
                 and stale is None, '%d bytes, %s, %s' % (size, back is not None, stale))
    report.check('preload: the room is measured - no refusal, or one in '
                 'words', why is None or why.startswith('skipped'), str(why))
    # Shown once: on a scripted clock the inquiries run PRELOAD,
    # IDENTITY, FITMENT, PROVENANCE, and round again without the preload.
    scripted = readout.fresh(0.0)
    seen = []
    for tick in range(2400):
        text = readout.draw(scripted, identity, 54, 12, now=tick * 0.1,
                            preload=state).plain
        head = text.split('\n', 1)[0]
        title = head.split('  ', 1)[1] if '  ' in head else ''
        if not seen or seen[-1] != title:
            seen.append(title)
    report.check('preload: shown once - the second time round the '
                 'readout goes on without it',
                 seen[:5] == ['PRELOAD', 'IDENTITY', 'FITMENT', 'PROVENANCE',
                              'IDENTITY'] and seen.count('PRELOAD') == 1,
                 str(seen[:8]))


def test_the_readout_prints_what_the_bus_said(report):
    """The front page's lower box: the board's identity and fitment off the
    bus, the host's provenance as host text, typed in, held, decayed and
    cycled - a late-seventies console's register in the tree's palette,
    without its lines: the bench struck those as silly.
    """
    from terminal import readout

    identity = {
        'info': {'device': 'coaxial_63100', 'type': 'bldc_inverter',
                 'firmware': '1.6.0', 'proto_major': 2, 'proto_minor': 8,
                 'mcu': 'STM32H753VIT6', 'commands': 21,
                 'description': 'Three-phase BLDC inverter, 63 V / 100 A, '
                                'PCB mounted coaxially behind an outrunner'},
        'parts': [{'name': 'IAUCN10S7N021', 'what': 'bridge FETs, 63 V 100 A'},
                  {'name': '2EDL8034 x3', 'what': 'half bridge gate drivers'},
                  {'name': 'DC link divider', 'what': '49.9k/2.2k, 78.15 V FS'}],
        'origin': 'COM4 (debug probe)', 'real': True}
    pages = readout.pages(identity, 54)
    plain = {title: [''.join(t for _s, t in row) for row in rows]
             for title, rows in pages}
    report.check('three pages, every line within the box',
                 [t for t, _r in pages] == ['IDENTITY', 'FITMENT', 'PROVENANCE']
                 and all(len(l) <= 54 for ls in plain.values() for l in ls),
                 str({t: max(len(l) for l in ls) for t, ls in plain.items()}))
    first = '\n'.join(plain['IDENTITY'])
    report.check('the identity page is 0x41: unit, type, firmware and '
                 'protocol, the MCU, the link, the board\'s own words',
                 'COAXIAL_63100' in first and 'BLDC INVERTER' in first
                 and '1.6.0' in first and 'PROTOCOL 2.8' in first
                 and 'STM32H753VIT6' in first and 'COM4 (DEBUG PROBE) LIVE' in first
                 and 'BEHIND AN OUTRUNNER' in first, first)
    fit = '\n'.join(plain['FITMENT'])
    bare = '\n'.join(''.join(t for _s, t in row) for _t, rows
                     in readout.pages(dict(identity, parts=[]), 54)
                     for row in rows)
    report.check('the fitment page is the parts list, and with no parts '
                 'on the bus no part is named anywhere',
                 all(p['name'].upper() in fit and p['what'].upper() in fit
                     for p in identity['parts'])
                 and 'IAUCN10S7N021' not in bare and '2EDL8034' not in bare
                 and 'NO FITMENT REPORTED' in bare, fit)
    third = '\n'.join(plain['PROVENANCE'])
    report.check('the provenance page is the host\'s: the dialogue with '
                 'Claude, the local LLM over MCP, the record',
                 'CLAUDE / ANTHROPIC' in third and 'MCP' in third
                 and 'FINDINGS' in third, third)
    narrow = readout.pages(identity, 22)
    report.check('at the 26-column floor every line still fits',
                 all(len(''.join(t for _s, t in row)) <= 22
                     for _t, rows in narrow for row in rows))

    state = readout.fresh(0.0)
    seen, frames = [], {}
    for tick in range(0, 400):
        now = tick * 0.08
        text = readout.draw(state, identity, 54, 12, now=now)
        seen.append((state['phase'], state['page']))
        frames.setdefault((state['phase'], state['page']), text.plain)
    phases = [p for p, _pg in seen]
    # 400 frames of 80 ms: three inquiries of ~9 s each - typing at CPS,
    # HOLD_S, the rows' decay - and the cycle back to the first.
    report.check('the motion types, holds, decays and moves to the next '
                 'inquiry, then round again',
                 phases[0] == 'type' and 'hold' in phases
                 and 'decay' in phases and ('type', 1) in seen
                 and ('type', 2) in seen and ('type', 0) in seen[150:],
                 str(sorted(set(seen))))
    typing = frames[('type', 0)]
    report.check('a frame mid-typing carries the status row and ends on '
                 'the cursor',
                 typing.startswith('INQUIRY 1/') and typing.endswith(readout.CURSOR),
                 repr(typing[-40:]))


def main():
    report = Report()
    print('\n-- every view, two frames, no board --')
    test_each_view_draws_two_frames(report)
    test_the_loader_reads_the_pages(report)
    print('\n-- the rotor observer\'s geometry --')
    test_the_instruments_stand_clear_of_the_motor(report)
    test_each_gutter_says_its_hottest_node(report)
    test_both_gutters_run_on_one_scale(report)
    test_a_power_node_never_reads_below_the_copper(report)
    test_the_ntc_is_shown_as_the_one_measurement(report)
    test_two_headrooms_named_apart(report)
    test_the_foot_carries_the_policy(report)
    test_the_foot_says_trip_while_the_cap_holds(report)
    test_every_frame_corner_on_the_map_is_a_right_angle(report)
    test_the_soa_legend_reads_the_whole_soa(report)
    test_the_soa_gauge_pulses_only_when_the_board_acts(report)
    test_the_mode_says_whether_the_board_holds_it_back(report)
    test_switch_soa_is_the_switches_and_motor_soa_the_winding(report)
    test_the_flat_drawings_spend_the_block(report)
    test_every_gauge_shows_its_own_scale(report)
    test_the_demo_actually_loads_the_motor(report)
    test_the_power_face_has_its_middle_at_half_a_kilowatt(report)
    test_the_level_is_drawn_at_the_dot(report)
    test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most(
        report)
    test_a_line_keeps_the_cell_it_shares_with_an_area(report)
    test_nothing_in_the_drawing_can_be_sheared(report)
    test_the_bead_is_round_at_every_angle(report)
    test_the_bead_trails_its_speed(report)
    test_the_dial_is_round_on_this_terminal(report)
    test_the_face_wears_its_two_scales(report)
    test_every_page_scrolls_its_boxes(report)
    test_the_terminal_is_asked_how_tall_a_cell_is(report)
    print('\n-- the thermal observer\'s board --')
    test_the_thermal_map_is_a_halftone_with_its_parts_marked(report)
    print('\n-- the attitude\'s frame rate --')
    test_the_attitude_caps_its_frame_rate(report)
    test_the_marquee_decodes_the_art_itself(report)
    print('\n-- the front page\'s readout --')
    test_the_readout_prints_what_the_bus_said(report)
    test_the_preload_is_the_first_inquiry(report)
    print('\n-- the thermal observer\'s headroom --')
    test_the_headroom_box_carries_a_solid_bar_with_a_tip(report)
    test_the_thermal_page_shows_its_evidence(report)
    test_a_frame_rasterises_as_the_terminal_draws_it(report)
    test_the_crt_draws_on_the_terminal(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
