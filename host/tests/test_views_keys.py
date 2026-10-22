"""The terminal's one UX (`terminal.ui.bindings`): every page's keys off its rows, the terminal's
after them - Esc the front page -; a key acts by the mode it finds - the page's own mode, its row -;
the instruments' column folded to its rail at first."""
import sys

from tools.dev.focus import chosen
from views_kit import Report, ansi_plain, typed


def test_a_page_its_keys_the_terminals_after(report):
    """A key the page binds is its own - the terminal's row it has a key of gone from its bar and
    help -; a typing page keeps ? and Q; a key two of its rows bind refused by name."""
    from terminal import menu
    from terminal.ui import bindings
    from terminal.views import humanoid_keys, show_chat
    from terminal.views.quad import keys
    her, chat, shared = humanoid_keys.PAD, show_chat.PAD, (('ESC', 'BACK'), ('TAB', 'COLUMN'), ('?', 'KEYS'))
    report.check("HUMANOID's Q turns her, the terminal's Q off its bar; Esc, Tab and ? the terminal's",
                 'q' in her['held'] and ('Q', 'EXIT') not in her['bar'] and set(shared) <= set(her['bar'])
                 and [label for label, _what in dict(her['groups'])['TERMINAL']] == ['ESC', 'TAB', '?'], her['bar'])
    folded = bindings.UX['column']
    report.check("the chat's typing keeps ? and Q; a flying page's bar its rows' then the terminal's; the front "
                 "page's Esc its own, no Tab: its panels stay up",
                 not {'?', 'q', 'Q'} & set(chat['keys']) and keys.PAD['bar'][-4:] == shared + (('Q', 'EXIT'),)
                 and ('ESC', 'BACK') not in menu.FRONT['bar'] and ('TAB', 'COLUMN') not in menu.FRONT['bar']
                 and bindings.act_on(menu.FRONT, ['\t'], {'picked': 0, 'who': None})[1] == ['\t']
                 and bindings.UX['column'] == folded)
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


def test_tab_and_help_in_any_mode(report):
    """Tab shows the instruments' column - folded to its rail at first - and folds it again, the
    page's own mode open too; ? the page's keys up and down; the arrows no row binds the column's."""
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
    finally:
        bindings.UX.update(column=False, help=False)
    report.check("folded to its rail at first, out on Tab, folded on Tab with her prompt open, the prompt's the rest",
                 folded == (False, taskbar.RAIL) and out == (True, HUD_WIDTH) and not bindings.UX['column']
                 and st['llm_prompt']['text'] == 'a?', (folded, out, st))
    report.check('? the keys up, ? again down; up and down handed back to page the column',
                 helped == [True, False] and rest == ['up', 'down'], (helped, rest))


def test_her_modes_before_her_rows(report):
    """HUMANOID: her prompt open takes every key, Esc closing it - the page kept -; F's choice open
    keeps the view's keys, a key that acts on her shuts it before it acts, Esc shuts it alone; a
    key's repeat within REPEAT_S acts once, P's repeats charge its push."""
    from types import SimpleNamespace as SN
    from terminal.ui import bindings
    from terminal.views import humanoid_keys, scanner, viewpoint
    up = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    room = [('wood', (1.5, 0.45, 0.8), (0.22, 0.02, 0.22), up), ('bed', (-1.8, 0.35, 1.8), (0.5, 0.12, 1.0), up)]
    sent, pad = [], humanoid_keys.PAD
    st = {'body': SN(send=lambda **kw: sent.append(kw)), 'scanner': scanner.VisionScanner(), 'tour': None,
          **viewpoint.HOME, 'now': {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': room}}
    typed(pad, '/x\x1b', st)
    prompt = st['llm_prompt']
    typed(pad, ['f', 'left'], st)
    kept = st['choosing'] is not None and st['yaw'] < viewpoint.YAW
    typed(pad, 'c', st)
    acted = st['choosing'], sent[-1]
    shut = typed(pad, 'f\x1b', st), st['choosing']
    back = typed(pad, '\x1b', st)
    report.check("Esc closes her prompt, the page kept; the view's keys keep F's choice; C shuts it and crouches her",
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
    mid = side.index('◂') if '◂' in side else -1
    key, edge = (taskbar._rgb(con.get_style(name)) for name in ('keys.key', 'frame.hud'))
    lit, dim = (taskbar._rgb(taskbar.breath(con, t)) for t in (0.0, SWEEP_S / 2.0))
    report.check("at first: the band its name alone, the column's edge a line with a tab in its middle - its "
                 "arrow over Tab's keycap, no word -, no boxes; Tab the boxes, no rail",
                 '≡' not in folded and 'TESTBED' in folded and abs(mid - len(side) // 2) <= 2 and side[0] == '│'
                 and side[mid + 1] == '↹' and not set('TAB') & set(side) and 'BOXED' not in folded
                 and 'BOXED' in out and '◂' not in out, (mid, len(side)))
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


ROSTER = (test_a_page_its_keys_the_terminals_after, test_esc_the_front_page, test_tab_and_help_in_any_mode,
          test_her_modes_before_her_rows, test_folded_into_the_page, test_the_drawing_takes_the_columns_room)


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
