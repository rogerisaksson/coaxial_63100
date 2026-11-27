"""The terminal's one UX (`terminal.ui.bindings`): every page's keys off its rows, the terminal's
after them - Esc the front page -; a key acts by the mode it finds - the page's own mode, its row -;
the instruments' column folded to its rail at first."""
import sys

from tools.dev.focus import chosen
from views_kit import Report, ansi_plain, looked_round, typed


def test_a_page_its_keys_the_terminals_after(report):
    """A key the page binds is its own - the terminal's row it has a key of gone from its bar and
    help -; a typing page keeps ? and Q; a key two of its rows bind refused by name."""
    from terminal import menu
    from terminal.ui import bindings
    from terminal.views import humanoid_keys, show_chat
    from terminal.views.quad import keys
    robot, chat, shared = humanoid_keys.PAD, show_chat.PAD, (('ESC', 'BACK'), ('TAB', 'COLUMN'), ('?', 'KEYS'))
    report.check("HUMANOID's Q sidesteps it, the terminal's Q off its bar; Esc, Tab and ? the terminal's",
                 'q' in robot['held'] and ('Q', 'EXIT') not in robot['bar'] and set(shared) <= set(robot['bar'])
                 and [label for label, _what in dict(robot['groups'])['TERMINAL']] == ['ESC', 'TAB', '?'], robot['bar'])
    folded = bindings.UX['column']
    rest = bindings.act_on(menu.FRONT, ['\t'], {'picked': 0, 'who': None})[1]
    tabbed, bindings.UX['column'] = bindings.UX['column'], folded
    report.check("the chat's typing keeps ? and Q; a flying page's bar its rows' then the terminal's; the front "
                 "page's Esc its own, Tab the terminal's: its board folds as every page's column",
                 not {'?', 'q', 'Q'} & set(chat['keys']) and keys.PAD['bar'][-4:] == shared + (('Q', 'EXIT'),)
                 and ('ESC', 'BACK') not in menu.FRONT['bar'] and ('TAB', 'COLUMN') in menu.FRONT['bar']
                 and rest == [] and tabbed != folded)
    try:
        bindings.page((('A', 'X', 'one', (), {'x': bindings.did(print)}), ('A', 'X', 'two', (), {'x': bindings.did(print)})))
        refused = ''
    except ValueError as e:
        refused = str(e)
    report.check('a key two rows bind refused by name', "'x'" in refused, refused)


def test_esc_the_front_page(report):
    """Esc leaves any page for the front page, Q the terminal - nothing drawn over the page between."""
    from terminal import loader
    from terminal.views.quad import keys
    answers = [typed(keys.PAD, key, {'see': 0}) for key in ('\x1b', 'q')]
    report.check('Esc the front page, Q out of the terminal, the loader reading them so',
                 answers == ['menu', 'quit'] and (loader.answer('menu'), loader.answer('quit')) == (loader.TO_MENU, 0),
                 answers)


def test_a_compiling_machines_entry_waits(report):
    """The front page while a machine's control system compiles (`terminal.compiling`): its entry greyed, a braille
    spinner turning before it, how far its compile has come said on it, chosen by neither the cursor's Enter nor its
    letter - the cursor left on it -; chosen at once its compile done."""
    import time
    from rich.text import Text
    from terminal import compiling, loader, menu
    at = menu.MACHINES.index('robot')
    key, name, _what = menu.ENTRIES[at]
    nav = {'picked': 0, 'who': None}
    with compiling._LOCK:
        compiling.STATE['robot'] = [0.5, 'STANDING: 3 OF 34 BUILT ON 6 CORES', time.monotonic() - 61.0]
    try:
        row = next(t for t in menu.roster(at).renderables if isinstance(t, Text) and name in t.plain)
        entered = menu._chosen(nav, at)
        typed_in = menu.bindings.act_on(menu.FRONT, [key.lower()], nav)[0]
    finally:
        with compiling._LOCK:
            compiling.STATE.pop('robot', None)
    done = menu._chosen({'picked': 0, 'who': None}, at)
    imu_at = next(i for i, p in enumerate(loader.pages()) if p.NAME == 'imu')
    compiling.note('raster', '3D ENGINE & BRAILLE C11')
    try:
        imu_row = next(t for t in menu.roster(imu_at).renderables
                       if isinstance(t, Text) and menu.ENTRIES[imu_at][1] in t.plain)
        imu_blocked = menu._chosen({'picked': 0, 'who': None}, imu_at)
    finally:
        compiling.done('raster')
    imu_ready = menu._chosen({'picked': 0, 'who': None}, imu_at)
    turns = {menu.spun(at, k / menu.SPIN_HZ) for k in range(len(menu.SPIN))}
    report.check('greyed, a spinner turning before it, its compile said on it, chosen by no key, the cursor on it; '
                 'chosen once done',
                 'COMPILING ROBOT: STANDING: 3 OF 34 BUILT ON 6 CORES  1:01' in row.plain
                 and row.plain[0] == '>' and row.plain[1] in menu.SPIN and turns == set(menu.SPIN)
                 and any(span.style == 'greyed' for span in row.spans) and entered is None and not typed_in
                 and nav['picked'] == at and done == loader.FIRST + at
                 and 'COMPILING RASTER: 3D ENGINE & BRAILLE C11' in imu_row.plain
                 and imu_blocked is None and imu_ready == loader.FIRST + imu_at,
                 '%r; %r %r -> %r' % (row.plain.strip(), entered, typed_in, done))


def test_leaving_sweeps_no_board_never_opened(report):
    """Q's way out (`session.run.leave`) on a native board this process never opened: nothing on it was left
    running, none laid nor built to be swept - a clean tree's way out stood 33 s on its libraries -; one opened is."""
    import time
    from tools.cores import native
    from terminal.views.session import run
    url = 'native://?leaving=never-opened'
    began = time.perf_counter()
    said = run.leave(url, False)
    took = time.perf_counter() - began
    native._LIMBS[url] = None
    try:
        laid = native.opened(url)
    finally:
        native._LIMBS.pop(url, None)
    report.check('none opened: left at once, none laid; one laid is opened',
                 said == 0 and took < 0.5 and not native.opened(url) and laid, '%.3f s' % took)


def test_tab_and_help_in_any_mode(report):
    """Tab shows the instruments' column - folded to its rail at first - and folds it again, the
    page's own mode open too, the front page's board as every page's column; ? the page's keys up and
    down; the arrows no row binds the column's."""
    from rich.layout import Layout
    from terminal import menu
    from terminal.ui import bindings, taskbar
    from terminal.ui.scroll import HUD_WIDTH
    from terminal.views import humanoid_keys
    from terminal.views.quad import keys
    st = {'llm_prompt': {'open': True, 'text': ''}}
    try:
        folded = bindings.UX['column'], taskbar.column()
        typed(humanoid_keys.PAD, '\t', st)
        out = bindings.UX['column'], taskbar.column()
        typed(humanoid_keys.PAD, 'a?\t', st)
        helped = [typed(keys.PAD, '?', {}) or bindings.UX['help'] for _ in range(2)]
        rest = bindings.act_on(keys.PAD, ['up', 'down', 'w'], {'send': lambda **_c: None})[1]
        body = menu.compose(menu.PORT, 0, {})['body'].renderable
        board = body['board'] if isinstance(body, Layout) else None
        typed(menu.FRONT, '\t', {'who': None, 'picked': 0})
        front = bindings.UX['column']
    finally:
        bindings.UX.update(column=False, help=False)
    report.check("folded to its rail at first, out on Tab, folded on Tab with the robot's prompt open, the prompt's the rest",
                 folded == (False, taskbar.RAIL) and out == (True, HUD_WIDTH) and not bindings.UX['column']
                 and st['llm_prompt']['text'] == 'a?', (folded, out, st))
    report.check("the front page's board its column: the rail folded, out on its Tab",
                 board is not None and board.size == taskbar.RAIL and type(board.renderable) is type(taskbar.rail())
                 and front, (board and board.size, board and type(board.renderable).__name__, front))
    report.check('? the keys up, ? again down; up and down handed back to page the column',
                 helped == [True, False] and rest == ['up', 'down'], (helped, rest))


def test_the_robots_modes_before_its_rows(report):
    """HUMANOID: its prompt open takes every key, Esc closing it - the page kept -; F's choice open
    keeps the view's keys, a key that acts on the robot's shuts it before it acts, Esc shuts it alone; a
    key's repeat within REPEAT_S acts once, P's repeats charge its push."""
    from types import SimpleNamespace as SN
    from terminal.ui import bindings
    from terminal.views import humanoid_keys, scanner, viewpoint
    up = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    room = [('wood', (1.5, 0.45, 0.8), (0.22, 0.02, 0.22), up), ('bed', (-1.8, 0.35, 1.8), (0.5, 0.12, 1.0), up),
            ('wood', (1.5, 0.72, 1.02), (0.22, 0.25, 0.02), up)]
    sent, pad = [], humanoid_keys.PAD
    st = {'body': SN(send=lambda **kw: sent.append(kw)), 'scanner': scanner.VisionScanner(), 'tour': None,
          **viewpoint.HOME, 'now': {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': room}}
    looked_round(st['scanner'], st['now'])
    typed(pad, '/x\x1b', st)
    prompt = st['llm_prompt']
    typed(pad, ['f', 'left'], st)
    kept = st['choosing'] is not None and st['yaw'] < viewpoint.YAW
    typed(pad, 'c', st)
    acted = st['choosing'], sent[-1]
    shut = typed(pad, 'f\x1b', st), st['choosing']
    back = typed(pad, '\x1b', st)
    report.check("Esc closes the robot's prompt, the page kept; the view's keys keep F's choice; C shuts it and crouches the robot",
                 prompt == {'open': False, 'text': 'x'} and kept and acted == (None, {'say': 'crouch'}),
                 (prompt, kept, acted))
    report.check("Esc shuts F's choice alone; the next Esc the front page", shut == (None, None) and back == 'menu',
                 (shut, back))
    for at in (100.0, 100.0 + bindings.REPEAT_S / 2, 100.0 + 2 * bindings.REPEAT_S):
        bindings.act_on(pad, ['x'], st, at)
        bindings.act_on(pad, ['p'], st, at)
    report.check('X and its repeat once, X again twice; each P a repeat of its charge',
                 st['shoulder'] == viewpoint.HOME['shoulder'] and st['charge']['repeats'] == 3, (st['shoulder'], st['charge']))


def test_folded_into_the_page(report):
    """A page drawn (`stage.frame_of`): its column folded to its edge, a tab in the edge's middle -
    its arrow breathing with the screen's sweep, a key's ink BREATH of the way to an edge's and back
    - at first, its band its name alone; its boxes out on Tab."""
    import io
    from rich.console import Console
    from rich.panel import Panel
    from coaxial.comm.session import Origin
    from terminal.ui import bindings, stage, taskbar
    from terminal.ui.chrome import SWEEP_S
    con = Console(width=120, height=36, theme=stage.THEME, force_terminal=True, file=io.StringIO())
    origin = Origin(False, 'simulated', 0, 'simulated', 'Simulated', 'simulated', 1)

    def drawn(**ux):
        bindings.UX.update(ux)
        with con.capture() as cap:
            con.print(stage.frame_of(con, origin, 'TESTBED', 'QZXJ', [Panel('BOXED')], bindings.BARE))
        return ansi_plain(cap.get())
    try:
        folded, out = drawn(), drawn(column=True)
    finally:
        bindings.UX.update(column=False)
    side = [line[-1 - taskbar.RAIL // 2] for line in folded.splitlines()[1:-1]]
    mid = side.index('\u25c2') if '\u25c2' in side else -1
    key, edge = (taskbar._rgb(con.get_style(name)) for name in ('keys.key', 'frame.hud'))
    lit, dim = (taskbar._rgb(taskbar.breath(con, t)) for t in (0.0, SWEEP_S / 2.0))
    report.check("at first: the band its name alone, the column's edge a line with a tab in its middle - its "
                 "arrow over Tab's keycap, no word -, no boxes; Tab the boxes, no rail",
                 '\u2261' not in folded and 'TESTBED' in folded and abs(mid - len(side) // 2) <= 2 and side[0] == '\u2502'
                 and side[mid + 1] == '\u21b9' and not set('TAB') & set(side) and 'BOXED' not in folded
                 and 'BOXED' in out and '\u25c2' not in out, (mid, len(side)))
    report.check("the arrow a key's ink as the sweep starts, BREATH of the way to an edge's half a sweep on",
                 lit == key and all(abs(d - k - taskbar.BREATH * (e - k)) <= 0.5 for d, k, e in zip(dim, key, edge)),
                 (lit, dim))


def test_the_drawing_takes_the_columns_room(report):
    """A page's drawing the window less the column as it stands and the viewport's frame
    (`taskbar.room`): BOARD ATTITUDE drew 106 of 150 folded, open and folded again - the open
    column's room, centred in the folded one's."""
    from terminal.ui import bindings, taskbar
    from terminal.views import show_desk, show_orientation

    def at(column):
        bindings.UX.update(column=column)
        return taskbar.room(150), show_orientation.art_width(150), show_desk.bar_in(120)
    try:
        out, folded = at(True), at(False)
    finally:
        bindings.UX.update(column=False)
    report.check("out: 106 of 150 the drawing's, the bridge's bar 48 of 120; folded, the rail's room: 141, "
                 "83", out == (106, 106, 48) and folded == (141, 141, 83), (out, folded))


def test_a_paste_is_never_typed(report):
    """terminal.ui.console: a bracketed paste is dropped whole however long it takes to come - VS Code pastes
    the clipboard on a right press, 50 characters every 5 ms, and the rest of one cut at 0.2 s was typed as
    keys: a chord of both buttons froze the atlas's tree in thousands of them -, its end marker found where a
    read splits it, the keys before and after it typed; a paste whose bytes stop before its end let go; a
    function key's code the table has not is no key (Insert, a stray paste marker were F1)."""
    import time
    from terminal.ui import console

    class Fed(console.Keys):
        """Keys handed `reads` as a console hands them, one a poll."""

        def __init__(self, reads):
            super().__init__(console=True)
            self.reads = list(reads)

        def _drain(self):
            return [self.reads.pop(0)] if self.reads else []

    body = 'Kolla igenom 4 och resten / o w q ' * 100
    end = console.PASTE_END
    reads = ['a', console.PASTE_START + body[:50]] + [body[k:k + 50] for k in range(50, len(body), 50)] + [end[:3], end[3:] + 'b']
    keys, typed = Fed(reads), ''
    for _ in range(len(reads) + 2):
        typed += keys._strip_sequences()[0]
        time.sleep(0.01)
    report.check('a paste of %d characters come 50 a read over %.1f s: none of it typed, the keys round it typed'
                 % (len(body), 0.01 * len(reads)), typed == 'ab' and not keys.taken(), repr(typed[:40]))
    keys = Fed([console.PASTE_START + 'cut off'])
    keys._strip_sequences()
    keys._paste_at -= console.PASTE_GAP_S + 0.1
    keys._strip_sequences()
    keys.reads = ['c']
    report.check('a paste whose bytes stop before its end let go: the next key typed', keys._strip_sequences()[0] == 'c', '')
    keys = Fed(['\x1b[2~\x1b[11~'])
    keys._strip_sequences()
    report.check("a function key's code the table has not is no key: Insert none, F1 F1", keys.taken() == ['f1'], '')


def test_a_chord_rearms_the_terminal(report):
    """terminal.ui.console: a press told while another is not let go - a chord of both buttons - turns the
    terminal's reporting off and on once (VS Code's xterm.js lost its drag and release listeners at the
    chord's second press and told presses alone after it: the atlas's tree froze, the wheel zoomed), no
    press left owed a release; a drag after it turns by its moves; presses after releases ask nothing."""
    from terminal.ui import console

    class Fed(console.Keys):
        """Keys handed `reads` one a poll, holding the mouse, the re-arms they ask of the terminal counted."""

        def __init__(self, reads):
            super().__init__(console=True, mouse=True, select=frozenset(), pan=True)
            self.reads, self.rearmed = list(reads), 0

        def _drain(self):
            return [self.reads.pop(0)] if self.reads else []

        def _rearm(self):
            self.rearmed += 1

    esc = chr(27)
    chord = [esc + '[<0;30;23M', esc + '[<2;30;23M']
    drag = [esc + '[<0;30;23M', esc + '[<32;35;23M', esc + '[<32;40;24M', esc + '[<0;40;24m']
    keys = Fed(chord + drag)
    for _ in range(len(chord)):
        keys._strip_sequences()
    report.check('the chord: one re-arm at its second press, no press owed a release, no click',
                 keys.rearmed == 1 and not keys._unreleased and not keys.clicked(), '%d re-arms' % keys.rearmed)
    for _ in range(len(drag)):
        keys._strip_sequences()
    spun = keys.dragged()
    report.check('the drag after it: turned (10, 1) cells, let go', spun == (10, 1) and keys.rearmed == 1
                 and not keys._unreleased and not keys._holding, '%s, %d re-arms' % (spun, keys.rearmed))
    keys = Fed([esc + '[<0;30;23M', esc + '[<0;30;23m', esc + '[<2;31;23M', esc + '[<34;33;24M', esc + '[<2;33;24m'])
    for _ in range(5):
        keys._strip_sequences()
    report.check('a click, then a right drag: no re-arm, the click told, the drag panned',
                 keys.rearmed == 0 and keys.clicked() == [(30, 23)] and keys.panned() == (2, 1), str(keys.rearmed))


ROSTER = (test_a_page_its_keys_the_terminals_after, test_esc_the_front_page, test_a_compiling_machines_entry_waits,
          test_leaving_sweeps_no_board_never_opened, test_tab_and_help_in_any_mode,
          test_the_robots_modes_before_its_rows, test_folded_into_the_page, test_the_drawing_takes_the_columns_room,
          test_a_paste_is_never_typed, test_a_chord_rearms_the_terminal)


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
