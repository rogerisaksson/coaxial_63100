"""The front page and the loader: every view two frames, the view loop, the marquee, the preload, the readout."""
import os
import sys
import time

from tools.dev.focus import chosen
from views_kit import HOST, Report, VIEW_TIMEOUT, run_view, views


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
    report.check('loader: eight pages, keys unique, SESSION first',
                 len(entries) == 8 and len(set(keys)) == 8
                 and entries[0][1] == 'SESSION', str(entries))
    report.check('loader: the front page draws the loader\'s list',
                 menu.ENTRIES == entries and menu.SUB == sub and menu.OPEN == opens,
                 str(menu.ENTRIES[:2]))
    names = sorted(loader.names())
    report.check('loader: every name answers - the pages and the items',
                 names == sorted(['session', 'imu', 'angle', 'adc', 'gate_drivers',
                                  'rotor_observer', 'thermal_observer', 'humanoid',
                                  'chat', 'claude'])
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


def test_the_view_loop_and_its_helpers(report):
    """run_view against scripted keys - a scroll, a click, a drag, then leaving; Ctrl+C;
    a view's own tick - and what a view draws with around it."""
    import contextlib
    import io
    import sys as _sys
    import types
    from coaxial.errors import NoReplyError, RigError
    from rich.console import Console
    from rich.text import Text
    from coaxial.comm.session import Origin
    from terminal.ui import screen, stage
    from tools.render import page

    class Scripted:
        """Keys that say what the test says, a frame at a time."""
        script, typed = [], []

        def __init__(self, _console, mouse=False):
            self.mouse = mouse

        def __enter__(self):
            return self

        def __exit__(self, *_exc):
            return False

        def poll(self):
            return Scripted.script.pop(0) if Scripted.script else (None, 0.0)

        def taken(self):
            typed, Scripted.typed = Scripted.typed, []
            return typed

        def clicked(self):
            return [(3, 4)]

        def dragged(self):
            return (1, 2)

    seen = {'scroll': [], 'click': [], 'drag': [], 'input': []}
    real = screen.Keys, screen.scroll_by, screen.scroll_click, screen.scroll_drag
    screen.Keys = Scripted
    screen.scroll_by = lambda _view, step: seen['scroll'].append(step)
    screen.scroll_click = lambda _view, col, row: seen['click'].append((col, row))
    screen.scroll_drag = lambda _view, dy: seen['drag'].append(dy)
    court = Console(file=io.StringIO())
    try:
        Scripted.script, Scripted.typed = [(None, 0.5)], ['down', 'x']
        screen.run_view(court, False, 0.01, 2, lambda: Text('frame'), mouse=True,
                        on_input=lambda typed, moved: seen['input'].append((typed, moved)),
                        on_click=lambda col, row: None, on_drag=lambda dx, dy: None)
        Scripted.script = [('menu', 0.0)]
        left = screen.run_view(court, False, 0.01, 0, lambda: Text('frame'))

        def interrupted():
            raise KeyboardInterrupt
        stopped = screen.run_view(court, False, 0.01, 0, interrupted)
        ticked = screen.run_view(court, False, 0.01, 0, lambda: Text('frame'),
                                 tick=lambda: True)
    finally:
        screen.Keys, screen.scroll_by, screen.scroll_click, screen.scroll_drag = real
    report.check('the loop scrolls on its keys, hands the rest on, takes clicks and drags '
                 'and leaves on the key that says so',
                 left == 'menu' and seen['scroll'] == [1]
                 and seen['input'][:1] == [(['x'], 0.5)] and (3, 4) in seen['click']
                 and 2 in seen['drag'], '%s %s' % (left, seen))
    report.check('Ctrl+C and a view that says it is done both end it quietly',
                 stopped is None and ticked is None)

    class Leaving:
        def poll(self):
            return 'quit', 0.0

        def taken(self):
            return []
    report.check('a key cuts the wait short', screen.paced(Leaving(), 5.0)[0] == 'quit')

    feed = screen.Feed(lambda: 1 / 0).start()
    for _ in range(50):
        if feed.error is not None:
            break
        time.sleep(0.01)
    feed.stop()
    report.check('a feed keeps what its read raised, for the view to show',
                 isinstance(feed.error, ZeroDivisionError))

    class Silent:
        def __init__(self, **_kwargs):
            pass

        def open(self):
            raise RigError('no board on COM9')
    was = screen.Coaxial63100
    screen.Coaxial63100 = Silent
    try:
        with contextlib.redirect_stdout(io.StringIO()) as said:
            rig = screen.open_rig('LINKING', port='COM9')
    finally:
        screen.Coaxial63100 = was
    report.check('a board that does not answer is no rig, with its words said',
                 rig is None and 'no board on COM9' in said.getvalue())

    clock = [100.0]
    real_time = screen.time.time
    screen.time.time = lambda: clock[0]
    try:
        fresh = screen.Freshness()
        fresh.take(5)
        fresh.take(5)
        repeated = (fresh.stale, fresh.note)
        clock[0] += 2.0
        fresh.take(25)
        clock[0] += 2.0
        fresh.take(45)
        fresh.take(None)
        held = fresh.note
        clock[0] += screen.Freshness.STILL_S
        gone = fresh.note
    finally:
        screen.time.time = real_time
    report.check('freshness: a draw that outruns the readings repeats one and stays live, '
                 'and the rate is read',
                 repeated == (1, 'live') and fresh.rate == 10.0 and held == 'live',
                 '%s %.1f %s' % (repeated, fresh.rate, held))
    report.check('freshness: a counter still for STILL_S is stale, in seconds',
                 gone.startswith('stale') and gone.endswith('s') and fresh.stale == 1, gone)

    class Tty(io.StringIO):
        def isatty(self):
            return True
    tty = Tty()
    real_out, real_sleep = _sys.stdout, screen.time.sleep
    _sys.stdout, screen.time.sleep = tty, lambda _s: None
    try:
        screen.say('warn', 'link', 'slow')
        tries = []

        def quiet():
            tries.append(1)
            raise NoReplyError('silent')
        gave = screen.steady(quiet)
        screen.closing([('gates', 'off'), ('rail', 'FAILED: held')], True, 10)
        screen.clear(True)
    finally:
        _sys.stdout, screen.time.sleep = real_out, real_sleep
    wrote = tty.getvalue()
    report.check('on a terminal the preflight line is coloured, a closing parks under the '
                 'frame and a clear wipes it',
                 chr(27) + '[33m' in wrote and chr(27) + '[11;1H' in wrote
                 and 'FAILED' in wrote and chr(27) + '[2J' in wrote)
    report.check('a quiet link is retried four times, then given up on',
                 gave is None and len(tries) == 4)
    report.check('a field too small for the corner crosses keeps its lines',
                 screen.stamp_crosses(['ab'], 10) == ['ab'])

    live = stage._Tube(types.SimpleNamespace(console='the console'))
    real_clients = stage.broker.clients
    stage.broker.clients = lambda: 2
    try:
        chips = [stage.live(1).plain, stage.live(0).plain,
                 stage.chip(Origin(True, 'COM4', 115200, 'serial', 'RS485 at COM4', 'RS485',
                                   1)).plain]

        def lit(port, kind, real):
            # The band's modes, and the one in its chip's colours.
            tag = stage.chip(Origin(real, port, 115200, kind, 'x', 'x', 1))
            return tag.plain, [tag.plain[s.start:s.end].strip() for s in tag.spans
                               if s.style != 'bar.dim']
        modes = [lit('emulator://', 'emulator', True), lit('native://', 'emulator', True),
                 lit('Simulated', 'simulated', False)]
    finally:
        stage.broker.clients = real_clients
    report.check('the band: LIVE with the sessions on the port, a Live passes the rest on',
                 chips == [' LIVE 1 SESSION ', ' LIVE ', ' LIVE 2 SESSIONS ']
                 and live.console == 'the console', str(chips))
    report.check('and EMU SIM NAT, the page\'s own lit: EMU on Renode, NAT native, SIM the '
                 'stand-in', modes == [(' EMU  SIM  NAT ', ['EMU']), (' EMU  SIM  NAT ', ['NAT']),
                                       (' EMU  SIM  NAT ', ['SIM'])], str(modes))

    blank = types.ModuleType('blank_page')
    setattr(blank, 'main', lambda argv: 0)
    _sys.modules['blank_page'] = blank
    page.PAGES['blank'] = 'blank_page'
    try:
        try:
            page.frame('blank', 60, 20, frames=1)
            drew = True
        except RuntimeError:
            drew = False
    finally:
        del page.PAGES['blank'], _sys.modules['blank_page']
    shot = os.path.join(os.environ.get('TEMP', '.'), 'page_test.png')
    with contextlib.redirect_stdout(io.StringIO()) as out, \
            contextlib.redirect_stderr(io.StringIO()) as err:
        code = page.main(['desk', '--size', '100', '30', '--frames', '2', '--png', shot])
    report.check('the page tool: a page that draws nothing is said to, a page draws to '
                 'text and a PNG', not drew and code == 0 and 'METER BRIDGE' in out.getvalue()
                 and 'page_test.png' in err.getvalue() and os.path.exists(shot))


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


ROSTER = (test_each_view_draws_two_frames, test_the_loader_reads_the_pages,
          test_the_view_loop_and_its_helpers, test_the_marquee_decodes_the_art_itself,
          test_the_preload_is_the_first_inquiry, test_the_readout_prints_what_the_bus_said)

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
