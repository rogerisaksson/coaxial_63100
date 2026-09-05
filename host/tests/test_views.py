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
import math
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
    frame, _lit = machine._raster(
        6.0, 24, 28, width, height, None, None, None,
        [(0.4, machine.SOA_OK)] * n_left, [(0.3, machine.SOA_OK)] * n_right,
        [(0.5, machine.SOA_OK)],
        [(0.4, machine.SOA_WARN), (0.3, machine.WATTS)], 2.0)
    can = [col for row in range(height) for col in range(width)
           if frame.owner[row][col] == machine.CAN]
    left, right = machine.gutters(width, height, n_left, n_right)
    gaps = (min(can) - max(left) - 1, min(right) - max(can) - 1)
    report.check('the gutters stand the same distance off the machine',
                 gaps[0] == gaps[1], 'left %d, right %d columns' % gaps)
    report.check('both groups fit inside the frame',
                 len(left) == n_left and len(right) == n_right,
                 '%d of %d left, %d of %d right'
                 % (len(left), n_left, len(right), n_right))

    can_rows = rows_of(frame.owner, width, height, machine.CAN)
    for name, kind in (('winding', machine.SOA_WARN),
                       ('power', machine.WATTS)):
        on = rows_of(frame.owner, width, height, kind)
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


def test_every_gauge_shows_its_own_scale(report):
    """The dimmed track runs the whole of every bar, at its own width.

    A BAR WITH NOTHING OVER IT SAYS HOW HOT A NODE IS; a bar in a tube
    says how hot it is OF WHAT IT MAY BE, which is the only version of
    the question a ceiling makes sense of. Two ways it was not saying it:
    the tubes drew their track in ONE lane, so the empty half of a
    thermometer was narrower than the mercury under it, and the flat
    gauges put a dot every FOURTH one, which is a dash in every other
    cell. Both read as some bars having a scale and some not.
    """
    from coaxial import machine

    n = 4
    art = machine.render(0.0, 24, 28, 46, 18,
                         left=[(0.0, machine.SOA_OK)] * n,
                         right=[(0.0, machine.SOA_OK)] * n,
                         bottom=[(0.0, machine.SOA_WARN), (0.0, machine.WATTS)])
    rows = art.split(chr(10))
    left, right = machine.gutters(46, 18, n, n)

    # EVERY TUBE, EVERY ROW OF IT. Empty bars, so what is drawn is track
    # and nothing else.
    seen = set()
    for row in rows[1:-2]:
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

    # AND THE FLAT GAUGES ALONG THE FOOT, one dot a cell rather than one
    # every other cell.
    first, last = machine.span(46, 18, n, n)
    floor = rows[-1]
    drawn = [floor[col] for col in range(first, last + 1)]
    report.check('the foot gauge draws a scale in every cell it spans',
                 all(c != ' ' and c != chr(0x2800) for c in drawn),
                 '%d of %d blank'
                 % (sum(1 for c in drawn if c in (' ', chr(0x2800))),
                    len(drawn)))


def test_the_demo_actually_loads_the_machine(report):
    """Two hundred frames of the stand-in warm the winding and spend the
    switches' margin - the demo puts a load on, and it stays on.

    THE ONE THAT WOULD HAVE CAUGHT IT. `main` was split and the demo's
    defaults - `args.b`, the friction the model turns against - landed
    after the preflight that hands the model its parameters. The model
    ran unloaded, drew no current and warmed nothing: every thermometer
    sat at its floor and the bench reported them dead, twice, while the
    thermal model was measured identical at three revisions. Measured
    broken at 600 frames: winding 22.9 C, SWITCH SOA 20 %; working at
    200: 64.2 C and 32.5 %; at 600: 99.2 C and 50.8 %.

    Two hundred frames rather than six hundred, because this is the
    slowest check in the suite and the broken case is flat from the
    first frame.
    """
    import re

    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    done = subprocess.run(
        [sys.executable, '-P', '-X', 'utf8',
         os.path.join('tools', 'show_rotor_observer.py'),
         '--simulated', '--frames', '200'],
        cwd=HOST, env=env, capture_output=True, text=True,
        encoding='utf-8', errors='replace', timeout=300)
    out = done.stdout + done.stderr
    winding = re.search(r'WINDING ([0-9.]+)', out)
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


def test_the_level_is_drawn_at_the_dot(report):
    """The top of a bar's mercury is `⣀`, `⣤`, `⣶`, `⣿` - one dot a step.

    IT WAS A WHOLE CELL WHATEVER THE LEVEL. The cell the mercury ended
    in took track dots above the level, and a cell is one colour with
    the mercury's class winning it, so those dots were coloured mercury
    and the top of every bar read `⣿`. A tube that fills in cell steps
    barely moves - which is what a bench watching a board warm up saw.
    The end cell holds mercury and nothing else now, so a level that
    moves one dot is seen to move.
    """
    from coaxial import machine

    track = chr(0x28D2)
    tops, dots = [], []
    for k in range(0, 8):
        share = (k + 0.5) / 40.0          # a ten-row tube is forty dots
        art = machine.render(0.0, 24, 28, 30, 12,
                             left=[(share, machine.SOA_OK)]).split(chr(10))
        column = [row[0] for row in art]
        mercury = [c for c in column if c not in (track, chr(0x2800))]
        tops.append(mercury[0] if mercury else '?')
        dots.append(sum(bin(ord(c) - 0x2800).count('1') for c in mercury))
    report.check('the top cell climbs a dot at a time',
                 tops[:4] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                              chr(0x28FF)], ''.join(tops))
    report.check('and keeps climbing into the next cell the same way',
                 tops[4:8] == [chr(0x28C0), chr(0x28E4), chr(0x28F6),
                               chr(0x28FF)], ''.join(tops))
    report.check('two dots a step - both lanes - and never a whole cell',
                 all(b - a == 2 for a, b in zip(dots, dots[1:])), str(dots))

    # AND ALONG THE FOOT, one lane at a time.
    ends = []
    for k in range(1, 5):
        share = (k + 0.5) / 60.0
        art = machine.render(0.0, 24, 28, 30, 12,
                             bottom=[(share, machine.WATTS)]).split(chr(10))
        foot = [c for c in art[-1] if c not in (chr(0x2802), chr(0x2800))]
        ends.append(foot[-1] if foot else '?')
    report.check('the foot gauge ends on a lane, not a cell',
                 ends == [chr(0x2807), chr(0x283F), chr(0x2807), chr(0x283F)],
                 ''.join(ends))


def test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most(
        report):
    """The air gap is less than a cell tall, and that is a trade the
    drawing makes on purpose.

    A CELL IS EIGHT DOTS AND ONE COLOUR. The gap between magnet band and
    tooth tip is 0.08 of the radius - 2.6 dots against a cell four tall -
    so at twelve and six o'clock one cell holds a magnet's inner edge and
    a tooth's tip. Three answers were built and each was seen on the
    bench: the gap held open to a cell's diagonal (no shared cell, and
    the teeth 1.9 dots short - "the slots are too small"); the tooth
    given the cell (green on the band); the magnet given the cell (amber
    on a tooth tip). The teeth keep their full fraction and the cell goes
    to whichever has more of it, which is the magnet in every one of the
    240 such cells over 48 poses. A colour a dot wide on a tooth's tip at
    two angles is the fault the bench can live with; short slots were
    not.
    """
    from coaxial import machine

    magnet = {machine.NORTH, machine.SOUTH}
    teeth = {machine.TOOTH_U, machine.TOOTH_V, machine.TOOTH_W}
    seat = machine.Seat(46, 18, None, None, None, None, None, None, 2.0)
    report.check('the teeth reach their full fraction of the radius',
                 abs(seat.radii.tooth_out
                     - seat.radii.can * machine.F_TOOTH_OUT) < 1e-9,
                 '%.2f of %.2f' % (seat.radii.tooth_out,
                                   seat.radii.can * machine.F_TOOTH_OUT))
    mixed = elsewhere = 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = machine._raster(
                6.0, 24, 28, 46, 18, None,
                machine._drive((30.0, -15.0, -15.0)), None,
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
    """A ring through a cell full of tooth or magnet keeps its colour.

    A CELL IS EIGHT DOTS AND ONE COLOUR, and two rules were tried before
    this one, each wrong in one place. The highest RANK that had lit any
    dot: a tooth outranks the yoke, so the yoke came out chopped into
    phase-coloured segments that changed with the drive. The MOST DOTS:
    mended the yoke and broke the can - the magnet band's outer edge and
    the can's inner ring are 0.10 of the radius apart, 3.3 dots against a
    cell four tall, so at twelve o'clock the cell they share is mostly
    magnet and the ring went amber in three places, ringed in red on the
    bench.

    A line that loses its cell is a broken line; an area that loses one
    is a dot short at its edge. So a line takes the cell. Measured: the
    yoke ring wholly its own colour at every angle and drive (0.80 of it
    under the vote), and no can-ring cell shared with a magnet lost to
    it (three under the vote).
    """
    import math

    from coaxial import machine

    magnet = {machine.NORTH, machine.SOUTH}
    seat = machine.Seat(46, 18, None, None, None, None, None, None, 2.0)
    r = seat.radii
    yoke_worst, lost = 1.0, 0
    for aspect in (2.0, 2.3):
        for deg in (0.0, 6.0, 12.0, 18.0):
            frame, _lit = machine._raster(
                deg, 24, 28, 46, 18, None,
                machine._drive((30.0, -15.0, -15.0)), None,
                None, None, None, None, aspect)
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if (tally and machine.CAN in tally and set(tally) & magnet
                            and frame.owner[row][col] != machine.CAN):
                        lost += 1
            ring = set()
            for k in range(720):
                phi = math.radians(k / 2.0)
                x = seat.cx + r.tooth_in * math.cos(phi)
                y = seat.cy - r.tooth_in * math.sin(phi) / (aspect / 2.0)
                ring.add((int(y) // 4, int(x) // 2))
            own = sum(1 for row, col in ring
                      if frame.owner[row][col] in (machine.YOKE, machine.BORE))
            yoke_worst = min(yoke_worst, own / len(ring))
    report.check('the yoke ring is wholly its own colour where the teeth '
                 'root', yoke_worst >= 0.99, 'worst %.2f' % yoke_worst)
    report.check('and no can-ring cell shared with a magnet is lost to it',
                 lost == 0, '%d cells' % lost)

    # THE RULE ITSELF, on one cell: a line with one dot beats an area
    # with seven; two lines settle by dots; two areas settle by dots.
    frame = machine.Frame(1, 1)
    for k in range(7):
        frame.put(k % 2, k // 2, machine.NORTH)
    frame.put(1, 3, machine.CAN)
    report.check('one dot of ring outweighs seven of magnet',
                 frame.owner[0][0] == machine.CAN)
    frame = machine.Frame(1, 1)
    for k in range(6):
        frame.put(k % 2, k // 2, machine.TOOTH_U)
    frame.put(0, 3, machine.TOOTH_W)
    report.check('and between two areas the most dots win, not the rank',
                 frame.owner[0][0] == machine.TOOTH_U)
    # NOT THE TRUTH STROKE, which wins outright - `Frame.put` has why.
    # NOT THE SOUTH ARC: drawn thin, but a magnet. Counted as a line its
    # fringe took 46 of 240 cells it shared with tooth tips - the rotor's
    # colour on the stator by another door.
    report.check('the lines are the rings - not the arc, not the stroke',
                 machine.LINES == frozenset((machine.BORE, machine.YOKE,
                                             machine.CAN)))

    # THE SHAFT SENSOR'S STROKE IS DRAWN THROUGH THE MAGNET BAND. It was
    # a tick in the air gap, and the air gap is the stator's side of the
    # picture: a white mark at the slot mouths where the teeth show their
    # current, reported as a second indicator drawn across the
    # magnetisation - twice trimmed, twice still there. Outside the rim
    # it reached the gutter at three and nine o'clock and found no empty
    # cell at some angles. The band has room, is the rotor, and is what
    # the sensor's angle is compared with; the air gap keeps every tooth
    # a cell's diagonal away, and the stroke owns every cell it is in or
    # it is magnet-coloured and gone. Measured over 48 poses: no cell
    # shared with a tooth, none in a gutter, at least one cell its own
    # in every pose, and at most one cell of the can's ring taken - the
    # angle at which it reads as reaching the rim.
    teeth = {machine.TOOTH_U, machine.TOOTH_V, machine.TOOTH_W}
    gutter = set(range(0, 8)) | set(range(38, 46))
    took = in_gutter = 0
    own_min, ring_max = 999, 0
    for aspect in (2.0, 2.3):
        for deg in range(0, 360, 30):
            frame, _lit = machine._raster(
                6.0, 24, 28, 46, 18, float(deg),
                machine._drive((30.0, -15.0, -15.0)), 41.0,
                [(0.3, machine.SOA_OK)] * 8, [(0.3, machine.SOA_OK)] * 8,
                None, [(0.3, machine.SOA_WARN), (0.3, machine.WATTS)],
                aspect)
            own = rings = 0
            for row, cells in enumerate(frame.tally):
                for col, tally in enumerate(cells):
                    if not tally or machine.TRUTH not in tally:
                        continue
                    if frame.owner[row][col] == machine.TRUTH:
                        took += bool(set(tally) & teeth)
                        own += 1
                        in_gutter += col in gutter
                        rings += (machine.CAN in tally
                                  or machine.YOKE in tally)
            own_min = min(own_min, own)
            ring_max = max(ring_max, rings)
    # A cell's diagonal still bridges the band's inner end and a tooth's
    # tip at some angles, so the stroke may SHARE a cell with a tooth; in
    # that cell it is not a candidate, because a white cell on a tooth is
    # a mark on the stator.
    report.check('the truth stroke takes no cell a tooth is in',
                 took == 0, '%d cells' % took)
    report.check('and never lands in a gutter', in_gutter == 0,
                 '%d cells' % in_gutter)
    report.check('and is seen in every pose', own_min >= 1,
                 'fewest own cells %d' % own_min)
    report.check('and takes at most one cell of the rim, at its own angle',
                 ring_max <= 1, 'most in one pose %d' % ring_max)


def test_nothing_in_the_drawing_can_be_sheared(report):
    """No character in the art is EAST ASIAN AMBIGUOUS WIDTH.

    UNICODE DOES NOT DECIDE FOR THOSE. A terminal set for East Asian text
    draws them two columns wide and every other one draws them narrow,
    and it is a SETTING rather than a font - so a page carrying one is a
    page that renders correctly on one bench and shears on the next.
    Sheared, the mark doubles, everything after it on the row slides a
    column, and the colour runs slide with it: the drawing bleeds inside
    its own box, which is exactly what was reported.

    Braille is narrow by definition, so what caught this out was the
    furniture: `\u25c0` and `\u25b6` as arrowheads, `\u25b2` and
    `\u25bc` at the foot, and the degree sign. All four triangles have
    unambiguous small twins and the degree has U+1D52.

    The bead is not among them - U+29BF is narrow, and the fallback built
    for it picked U+25CF, which is ambiguous. The safe substitute was the
    only unsafe character in the pair, and both are gone.
    """
    import unicodedata

    from coaxial import machine
    import show_rotor_observer as view

    drawn = machine.render(6.0, 24, 28, 46, 18, pointer_deg=41.0)
    said = ''.join(str(x) for x in
                   (view.AIM_LEFT, view.AIM_RIGHT, view.UP, view.DOWN,
                    view.DEGREE, view.LEADER, machine.POINTER_GLYPH)
                   ) + ''.join(view.TURN) + ''.join(view.DROP)
    for name, text in (('the drawing', drawn), ("the view's furniture", said)):
        bad = sorted({c for c in text
                      if unicodedata.east_asian_width(c) == 'A'})
        report.check('%s carries no ambiguous-width character' % name,
                     not bad,
                     ' '.join('%s U+%04X' % (c, ord(c)) for c in bad))

    # AND THE SUBSTITUTES ARE THE SAME MARKS, not near misses: a small
    # triangle points the same way as its big twin.
    report.check('the arrowheads are the small triangles',
                 (view.AIM_LEFT, view.AIM_RIGHT) == (chr(0x25C2), chr(0x25B8)),
                 view.AIM_LEFT + view.AIM_RIGHT)
    report.check('and the foot uses their up and down',
                 (view.UP, view.DOWN) == (chr(0x25B4), chr(0x25BE)),
                 view.UP + view.DOWN)


def test_the_bead_is_round_at_every_angle(report):
    """The pointer is `POINTER_GLYPH`, and it rides the rim.

    THE MARK IS SETTLED - `machine._bead` has why, what a glyph costs,
    and the four dot answers that were built and not kept. What this
    holds is the two things that were actually broken: it must be ONE
    mark drawn at every angle, and it must be PLACED in the same space
    the machine is drawn in.
    """
    from coaxial import machine

    for aspect in (2.0, 2.4):
        was, seats = None, set()
        for deg in range(0, 360, 5):
            art = machine.render(0.0, 24, 28, 46, 18, aspect=aspect,
                                 pointer_deg=float(deg)).split(chr(10))
            at = [(r, line.index(machine.POINTER_GLYPH))
                  for r, line in enumerate(art)
                  if machine.POINTER_GLYPH in line]
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

    # IT RIDES THE RIM IN THE DRAWING'S OWN SPACE. The radii are in
    # x-dots and `_body` scales y by `stretch`, so a bead placed with
    # plain trigonometry rode the rim only where a dot happened to be
    # square - on a terminal whose cell is not two-by-one it sat outside
    # the periphery, which is where the bench found it.
    for aspect in (2.0, 2.4):
        stretch = aspect / 4.0 * 2.0
        cx, r, _, _ = machine.layout(46, 18, 0, 0, rows=18)
        cy = 18 * 4 / 2.0 - 0.5
        seat = r.can + machine.POINTER_SEAT
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

    # THE NEAREST CELL CENTRE, not the one the point fell inside. A cell
    # is two dots across by four down, so truncating quantises the path
    # twice as coarsely down as across - an egg, not a circle.
    cx, r, _, _ = machine.layout(46, 18, 0, 0, rows=18)
    cy = 18 * 4 / 2.0 - 0.5
    seat = r.can + machine.POINTER_SEAT

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
    """The cell's shape is measured, not assumed.

    THE ONE NUMBER A ROUND DRAWING NEEDS AND NOBODY CAN LOOK UP. The
    renderers work in square pixels and fold the cell in at the end, so
    getting it wrong does not blur the picture - it stretches it, and a
    can drawn wide of round reads as a rotor that is turned when it is
    not. Measured here: at 2.0 the can comes out 47.0 cell-widths across
    and 46.5 down, so the GEOMETRY is right and an oval on screen is the
    font, which is why it is worth asking.
    """
    import screen

    report.check('a terminal 1200 by 800 pixels over 100 by 40 cells has a '
                 'cell 1.67 times as tall as it is wide',
                 abs((screen.cell_aspect_of((800, 1200), (40, 100)) or 0)
                     - 5.0 / 3.0) < 1e-9)
    report.check('nothing divisible by zero comes back as a number',
                 screen.cell_aspect_of((0, 0), (1, 1)) is None
                 and screen.cell_aspect_of((800, 1200), (0, 100)) is None)
    report.check('and a reply that cannot be a cell is refused',
                 screen.cell_aspect_of((8000, 100), (40, 100)) is None
                 and screen.cell_aspect_of((10, 1200), (40, 100)) is None,
                 str(screen.ASPECT_RANGE))
    report.check('a pipe is never asked, so the query cannot land in a '
                 'render', screen.probe_aspect(console=False) is None)


def test_the_flat_drawings_spend_the_block(report):
    """The 2D drawings place their edges by coverage, not by "any corner".

    A DOT IS ONE BIT AND `SUBDOT` SAMPLES FOUR CORNERS. Read as "any", a
    shape covering a quarter of a dot lit it whole - so every arc came
    out a dot fatter than it is and the can's rim stepped against the
    magnets inside it. Read as COVERAGE the arc lands where it is, and
    the grading a braille cell can show falls out of it for free: an arc
    crossing the bottom of a cell draws the bottom row, the lower half
    two rows, and so on up.

    AN ORDERED DITHER ON THE FRINGE WAS BUILT AND TAKEN OUT - it is fixed
    in screen space, so a shape moving across it has its fringe pop on
    and off in a standing pattern, and on a still picture it only made
    the lines a dot fatter here and there.
    """
    from coaxial import dial, machine, raster

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


def test_every_gauge_shows_its_own_scale(report):
    """The dimmed track runs the whole of every bar, at its own width.

    A BAR WITH NOTHING OVER IT SAYS HOW HOT A NODE IS; a bar in a tube
    says how hot it is OF WHAT IT MAY BE, which is the only version of
    the question a ceiling makes sense of. Two ways it was not saying it:
    the tubes drew their track in ONE lane, so the empty half of a
    thermometer was narrower than the mercury under it, and the flat
    gauges put a dot every FOURTH one, which is a dash in every other
    cell. Both read as some bars having a scale and some not.
    """
    from coaxial import machine

    n = 4
    art = machine.render(0.0, 24, 28, 46, 18,
                         left=[(0.0, machine.SOA_OK)] * n,
                         right=[(0.0, machine.SOA_OK)] * n,
                         bottom=[(0.0, machine.SOA_WARN), (0.0, machine.WATTS)])
    rows = art.split(chr(10))
    left, right = machine.gutters(46, 18, n, n)

    # EVERY TUBE, EVERY ROW OF IT. Empty bars, so what is drawn is track
    # and nothing else.
    seen = set()
    for row in rows[1:-2]:
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

    # AND THE FLAT GAUGES ALONG THE FOOT, one dot a cell rather than one
    # every other cell.
    first, last = machine.span(46, 18, n, n)
    floor = rows[-1]
    drawn = [floor[col] for col in range(first, last + 1)]
    report.check('the foot gauge draws a scale in every cell it spans',
                 all(c != ' ' and c != chr(0x2800) for c in drawn),
                 '%d of %d blank'
                 % (sum(1 for c in drawn if c in (' ', chr(0x2800))),
                    len(drawn)))


def test_the_bead_is_round_at_every_angle(report):
    """The pointer is `POINTER_GLYPH`, and it rides the rim.

    THE MARK IS SETTLED - `machine._bead` has why, what a glyph costs,
    and the four dot answers that were built and not kept. What this
    holds is the two things that were actually broken: it must be ONE
    mark drawn at every angle, and it must be PLACED in the same space
    the machine is drawn in.
    """
    from coaxial import machine

    for aspect in (2.0, 2.4):
        was, seats = None, set()
        for deg in range(0, 360, 5):
            art = machine.render(0.0, 24, 28, 46, 18, aspect=aspect,
                                 pointer_deg=float(deg)).split(chr(10))
            at = [(r, line.index(machine.POINTER_GLYPH))
                  for r, line in enumerate(art)
                  if machine.POINTER_GLYPH in line]
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

    # IT RIDES THE RIM IN THE DRAWING'S OWN SPACE. The radii are in
    # x-dots and `_body` scales y by `stretch`, so a bead placed with
    # plain trigonometry rode the rim only where a dot happened to be
    # square - on a terminal whose cell is not two-by-one it sat outside
    # the periphery, which is where the bench found it.
    for aspect in (2.0, 2.4):
        stretch = aspect / 4.0 * 2.0
        cx, r, _, _ = machine.layout(46, 18, 0, 0, rows=18)
        cy = 18 * 4 / 2.0 - 0.5
        seat = r.can + machine.POINTER_SEAT
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

    # THE NEAREST CELL CENTRE, not the one the point fell inside. A cell
    # is two dots across by four down, so truncating quantises the path
    # twice as coarsely down as across - an egg, not a circle.
    cx, r, _, _ = machine.layout(46, 18, 0, 0, rows=18)
    cy = 18 * 4 / 2.0 - 0.5
    seat = r.can + machine.POINTER_SEAT

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
    """The cell's shape is measured, not assumed.

    THE ONE NUMBER A ROUND DRAWING NEEDS AND NOBODY CAN LOOK UP. The
    renderers work in square pixels and fold the cell in at the end, so
    getting it wrong does not blur the picture - it stretches it, and a
    can drawn wide of round reads as a rotor that is turned when it is
    not. Measured here: at 2.0 the can comes out 47.0 cell-widths across
    and 46.5 down, so the GEOMETRY is right and an oval on screen is the
    font, which is why it is worth asking.
    """
    import screen

    report.check('a terminal 1200 by 800 pixels over 100 by 40 cells has a '
                 'cell 1.67 times as tall as it is wide',
                 abs((screen.cell_aspect_of((800, 1200), (40, 100)) or 0)
                     - 5.0 / 3.0) < 1e-9)
    report.check('nothing divisible by zero comes back as a number',
                 screen.cell_aspect_of((0, 0), (1, 1)) is None
                 and screen.cell_aspect_of((800, 1200), (0, 100)) is None)
    report.check('and a reply that cannot be a cell is refused',
                 screen.cell_aspect_of((8000, 100), (40, 100)) is None
                 and screen.cell_aspect_of((10, 1200), (40, 100)) is None,
                 str(screen.ASPECT_RANGE))
    report.check('a pipe is never asked, so the query cannot land in a '
                 'render', screen.probe_aspect(console=False) is None)


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
    test_every_gauge_shows_its_own_scale(report)
    test_the_demo_actually_loads_the_machine(report)
    test_the_level_is_drawn_at_the_dot(report)
    test_the_teeth_keep_their_length_and_a_shared_cell_goes_to_the_most(
        report)
    test_a_line_keeps_the_cell_it_shares_with_an_area(report)
    test_nothing_in_the_drawing_can_be_sheared(report)
    test_the_bead_is_round_at_every_angle(report)
    test_the_terminal_is_asked_how_tall_a_cell_is(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
