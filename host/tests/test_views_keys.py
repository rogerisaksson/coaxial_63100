"""The terminal's one UX (`terminal.ui.bindings`): every page's keys off its rows, the terminal's
after them; a key acts by the mode it finds - the start menu over the page, the page's own mode,
its row -; the instruments' column folded to its rail at first, the start menu drawn over the
viewport."""
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
    her, chat, shared = humanoid_keys.PAD, show_chat.PAD, (('ESC', 'MENU'), ('TAB', 'COLUMN'), ('?', 'KEYS'))
    report.check("HUMANOID's Q turns her, the terminal's Q off its bar; Esc, Tab and ? the terminal's",
                 'q' in her['held'] and ('Q', 'EXIT') not in her['bar'] and set(shared) <= set(her['bar'])
                 and [label for label, _what in dict(her['groups'])['TERMINAL']] == ['ESC', 'TAB', '?'], her['bar'])
    folded = bindings.UX['column']
    report.check("the chat's typing keeps ? and Q; a flying page's bar its rows' then the terminal's; the front "
                 "page's Esc its own, no Tab: its panels stay up",
                 not {'?', 'q', 'Q'} & set(chat['keys']) and keys.PAD['bar'][-4:] == shared + (('Q', 'EXIT'),)
                 and ('ESC', 'MENU') not in menu.FRONT['bar'] and ('TAB', 'COLUMN') not in menu.FRONT['bar']
                 and bindings.act_on(menu.FRONT, ['\t'], {'picked': 0, 'who': None})[1] == ['\t']
                 and bindings.UX['column'] == folded)
    try:
        bindings.page((('A', 'X', 'one', (), {'x': bindings.did(print)}), ('A', 'X', 'two', (), {'x': bindings.did(print)})))
        refused = ''
    except ValueError as e:
        refused = str(e)
    report.check('a key two rows bind refused by name', "'x'" in refused, refused)


def test_the_start_menu_over_any_page(report):
    """Esc opens the start menu over the page, its cursor on the front page's line; the arrows move
    it round, Enter or a page's letter opens that page, 0 the front page, Esc shuts it - the page
    kept -, Q leaves the terminal; while it is open every key is its."""
    from terminal import loader
    from terminal.ui import bindings
    from terminal.views.quad import keys
    entries, held = loader.listing()[0], {'see': 0}
    try:
        opened = typed(keys.PAD, '\x1b', held), bindings.UX['menu']
        walked = typed(keys.PAD, ['down', 'down', 'w', 'S'], held), bindings.UX['menu'], held['see']
        shut = typed(keys.PAD, '\x1b\x1b', held), bindings.UX['menu']
        answers = [typed(keys.PAD, ['\x1b'] + list(k), held) for k in (
            ('down', '\r'), ('up', '\r'), '0', entries[2][0].lower(), 'q')]
    finally:
        bindings.UX['menu'] = None
    report.check('Esc opens it on the front page; the arrows, W and S its - S a page -; Esc shuts it',
                 opened == (None, 0) and walked == (loader.FIRST + [e[0] for e in entries].index('S'), None, 0)
                 and shut == (None, None), (opened, walked, shut))
    report.check('Enter the first page, round up the last, 0 the front page, a letter its page, Q out',
                 answers == [loader.FIRST, loader.FIRST + len(entries) - 1, 'menu', loader.FIRST + 2, 'quit'], answers)
    report.check('the loader opens a page so answered, the front page on 0, nothing on Q',
                 (loader.answer(loader.FIRST + 2), loader.answer('menu'), loader.answer('quit'))
                 == (loader.FIRST + 2, loader.TO_MENU, 0))


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
    try:
        typed(pad, '/x\x1b', st)
        prompt = st['llm_prompt'], bindings.UX['menu']
        typed(pad, ['f', 'left'], st)
        kept = st['choosing'] is not None and st['yaw'] < viewpoint.YAW
        typed(pad, 'c', st)
        acted = st['choosing'], sent[-1]
        typed(pad, 'f\x1b', st)
        shut = st['choosing'], bindings.UX['menu']
        typed(pad, '\x1b', st)
        menu = bindings.UX['menu']
    finally:
        bindings.UX['menu'] = None
    report.check("Esc closes her prompt, the page kept; the view's keys keep F's choice; C shuts it and crouches her",
                 prompt == ({'open': False, 'text': 'x'}, None) and kept and acted == (None, {'say': 'crouch'}),
                 (prompt, kept, acted))
    report.check("Esc shuts F's choice alone; the next Esc the start menu", shut == (None, None) and menu == 0, (shut, menu))
    for at in (100.0, 100.0 + bindings.REPEAT_S / 2, 100.0 + 2 * bindings.REPEAT_S):
        bindings.act_on(pad, ['x'], st, at)
        bindings.act_on(pad, ['p'], st, at)
    report.check('X and its repeat once, X again twice; each P a repeat of its charge',
                 st['shoulder'] == viewpoint.HOME['shoulder'] and st['charge']['repeats'] == 3, (st['shoulder'], st['charge']))


def test_folded_into_the_page(report):
    """A page drawn (`stage.frame_of`): its column folded to a rail - its arrow breathing with the
    screen's sweep, a key's ink BREATH of the way to an edge's and back - and the start menu to the
    band's ≡ at first; its boxes out on Tab; Esc's start menu over the viewport, the front page's
    line and every page's, its own bar under it."""
    import io
    from rich.console import Console
    from rich.panel import Panel
    from coaxial.comm.session import Origin
    from terminal import loader
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
        folded, out, menu = drawn(), drawn(column=True), drawn(column=False, menu=0)
    finally:
        bindings.UX.update(column=False, menu=None)
    rail = [line.rstrip()[-2:-1] for line in folded.splitlines()[2:4]]
    key, edge = (taskbar._rgb(con.get_style(name)) for name in ('keys.key', 'frame.hud'))
    lit, dim = (taskbar._rgb(taskbar.breath(con, t)) for t in (0.0, SWEEP_S / 2.0))
    report.check("at first: the band's ≡, the rail's arrow alone, no boxes; Tab the boxes, no rail",
                 '≡ TESTBED' in folded and rail == ['◂', ' '] and 'BOXED' not in folded
                 and 'BOXED' in out and '◂' not in out, rail)
    report.check("the arrow a key's ink as the sweep starts, BREATH of the way to an edge's half a sweep on",
                 lit == key and all(abs(d - k - taskbar.BREATH * (e - k)) <= 0.5 for d, k, e in zip(dim, key, edge)),
                 (lit, dim))
    report.check("Esc's menu over the viewport: the front page and every page, its bar under it",
                 'QZXJ' not in menu and 'QZXJ' in folded and 'FRONT PAGE' in menu and all(head in menu for _k, head, _w in loader.listing()[0])
                 and 'ENTER: OPEN' in menu and 'ESC: STAY' in menu)


ROSTER = (test_a_page_its_keys_the_terminals_after, test_the_start_menu_over_any_page, test_tab_and_help_in_any_mode,
          test_her_modes_before_her_rows, test_folded_into_the_page)


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
