"""Every page's keys from one table, the ones every page shares after its own.

    pad = bindings.page(ROWS, take=None, held=None)    # its acts, bar and help read off its rows
    leaving, rest = bindings.act_on(pad, typed, state)  # a frame's keys; where the page leaves
    bindings.UX                                        # the terminal's: its column, its help

A row is a line of the page's help: (its group, its keys as the help says them, what they do, its
words on the bar, {key: act}), an act (state, key, now) -> where the page leaves or None: 'menu',
'quit' or a page's code (`terminal.loader`). HELD binds a key the page holds - its pad's (`held`,
the keys typed together). SHARED, every page's after its own: Esc the front page, Tab the
instruments' column hidden or shown, ? the page's keys, Q the terminal left. A key acts by the
mode it finds: Tab in any; a page's own mode next (`take`, PASS to hand the key on);
then its row; a key no row binds goes back (`rest`), what a page's own loop still reads.
"""
import time
from collections import Counter

#: The terminal's: the instruments' column out - folded to a rail at first -; the page's keys
#: asked up (?), how far up they are, when a key was last typed, when they were last drawn
#: (`taskbar.helped`).
UX = {'column': False, 'help': False, 'up': 0.0, 'keyed': 0.0, 'framed': 0.0}
#: A key bound to HELD the page holds; PASS a mode hands a key on.
HELD, PASS = None, 'pass'
#: A key typed again within REPEAT_S of its act is its repeat (`once`).
REPEAT_S = 0.12


def once(act):
    """`act` once a press: a key typed again within REPEAT_S of its last typing is its repeat - timed from its last
    act, a held key acted every REPEAT_S, a held Space a new hop every 0.12 s in the air, and she fell."""
    def pressed(state, key, now):
        last, at = state.get('_last_action_key', (None, float('-inf')))
        state['_last_action_key'] = (key, now)
        if key == last and now - at < REPEAT_S:
            return None
        return act(state, key, now)
    return pressed


def did(verb):
    """The act of `verb` (state) alone."""
    return lambda state, _key, _now: verb(state)


def cased(letter, act):
    """{key: act} of `letter` and its upper case."""
    return {letter: act, letter.upper(): act}


def _column(_state, _key, _now):
    """Tab: the instruments' column hidden, or shown."""
    UX['column'] = not UX['column']


def _helped(_state, _key, _now):
    """?: the page's keys up, or down."""
    UX['help'] = not UX['help']


#: Every page's, after its own: a key the page binds stays the page's.
SHARED = (
    ('TERMINAL', 'ESC', 'the front page', (('ESC', 'BACK'),), {'\x1b': did(lambda _s: 'menu')}),
    ('TERMINAL', 'TAB', "the instruments' column hidden, or shown", (('TAB', 'COLUMN'),), {'\t': _column}),
    ('TERMINAL', '?', "these keys", (('?', 'KEYS'),), {'?': _helped}),
    ('TERMINAL', 'Q', 'the terminal left', (('Q', 'EXIT'),), {'q': did(lambda _s: 'quit'), 'Q': did(lambda _s: 'quit')}),
)


def page(rows, take=None, held=None, column=True, typing=False):
    """A page's keys: {'keys': {key: act}, 'held': its HELD keys, 'bar': ((key, word), ..), 'groups':
    ((group, ((keys, what), ..)), ..), 'take', 'hold'} off its `rows` and SHARED - a shared row
    the page has a key of lost to it, its line and bar words too; Tab's where it has no `column`, ?'s
    and Q's where its mode takes what is `typing` -; `take` (state, key, now) its mode, PASS handing
    the key on; `held` (state, keys, now) its HELD keys typed together. A bar's word may be a function
    of the page's state (`shown`). A key two of its rows bind is refused."""
    mine = Counter(key for *_head, binds in rows for key in binds)
    twice = sorted(key for key, n in mine.items() if n > 1)
    if twice:
        raise ValueError('keys two rows bind: %r' % twice)
    own = set(mine) | (set() if column else {'\t'}) | ({'?', 'q', 'Q'} if typing else set())
    shared = [row for row in SHARED if not own & set(row[4])]
    lines = tuple(rows) + tuple(shared)
    bound = [(key, act) for *_head, binds in lines for key, act in binds.items()]
    groups = tuple(dict.fromkeys(row[0] for row in lines))
    return {'keys': {key: act for key, act in bound if act is not HELD},
            'held': frozenset(key for key, act in bound if act is HELD),
            'bar': tuple(pair for _g, _l, _w, bar, _b in lines for pair in bar),
            'groups': tuple((g, tuple((label, what) for gr, label, what, _bar, _b in lines if gr == g)) for g in groups),
            'take': take, 'hold': held}


#: An empty page: SHARED alone - a page whose own keys its loop still reads -, and one with no
#: instruments' column.
BARE, FLAT = page(()), page((), column=False)


def shown(pad, state=None, note=''):
    """`pad` as this frame shows it: each bar key or word that is a function of the page's `state`
    told by it - a word None drops its pair -, `note` after them."""
    told = tuple((key(state) if callable(key) else key, word(state) if callable(word) else word)
                 for key, word in pad['bar'])
    return dict(pad, bar=tuple(pair for pair in told if pair[1] is not None) + ((('', note),) if note else ()))


def act_on(pad, typed, state, now=None):
    """A frame's `typed` keys on the page's `state` by its `pad` (`page`), each by the mode it
    finds (the module's head), its HELD keys typed together held together: (where the page
    leaves, or None; the keys no row binds, in order)."""
    now = time.monotonic() if now is None else now
    UX['keyed'] = now if typed else UX['keyed']
    leaving, held, rest = None, [], []
    for key in typed:
        if key == '\t' and key in pad['keys']:
            _column(state, key, now)
            continue
        took = PASS if pad['take'] is None else pad['take'](state, key, now)
        if took != PASS:
            leaving = leaving or took
            continue
        if key in pad['held']:
            held.append(key)
            continue
        if held and pad['hold'] is not None:
            pad['hold'](state, held, now)
        held, act = [], pad['keys'].get(key)
        if act is None:
            rest.append(key)
            continue
        leaving = leaving or act(state, key, now)
    if held and pad['hold'] is not None:
        pad['hold'](state, held, now)
    return leaving, rest
