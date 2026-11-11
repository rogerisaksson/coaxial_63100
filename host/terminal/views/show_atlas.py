#!/usr/bin/env python3
"""The atlas on the terminal: the trunk from the terminal up to the shunts, opened a node at a time.

    python terminal/views/show_atlas.py
    python terminal/views/show_atlas.py --frames 2

The tree is tools/dev/atlas_tree.py's, read from the files as they stand when the page opens. The drawing
is the trunk as a stack of slabs, its crown up - a slab's tiles its folders, a tile's cells its files -:
the node open lit blue, the folders it reaches green, those reaching it amber, a word's hits rose; beside
it the node open, its children and the cursor on one. W lays the same atlas as a page
(tools/dev/atlas.py --html) and opens it in the browser.
"""
import argparse
import colorsys
import sys
import textwrap
import webbrowser

from coaxial.comm.session import Origin
from coaxial.draw import braille
from terminal.loader import answer
from terminal.ui import bindings, screen as _screen
from terminal.ui.screen import ENTER_KEYS, FPS_CAP, PORT, run_view, say
from terminal.ui.stage import boot, frame_of, hud, stage
from terminal.ui.taskbar import room
from tools.dev import atlas
from tools.dev.atlas_tree import FILE, FOLDER, KINDS, LAYER, LAYERS, SYMBOL, build

TITLE = 'SYSTEM ATLAS'
#: A slab's cells across; the trunk's columns, its labels with it; the least a lens beside it takes.
SLAB, TRUNK, LENS = 26, 56, 34
#: The page's legend: the node open, what it reaches, what reaches it, a word's hits; dim, bright.
OPEN, OUT, IN, HIT = (90, 162, 255), (70, 192, 106), (242, 154, 74), (242, 125, 187)
DIM, BRIGHT = (128, 138, 150), (226, 231, 238)
#: A slab in braille dots: its plate's length along the top, the top's depth and slant (the side's width),
#: a folder's least plate - a dot of gap and one lit; the cursor; a cut line's mark; a full cell.
PLATE, SLANT, LEAST = 2 * SLAB - 4, 3, 2
CURSOR, CUT, FULL = '▶', '…', braille.ALL[255]


def ink(rgb, text, bold=False):
    """`text` in the colour `rgb`, bold where asked: an SGR run the viewport decodes."""
    return '\x1b[%s38;2;%d;%d;%dm%s\x1b[0m' % ('1;' if bold else '', rgb[0], rgb[1], rgb[2], text)


def tone(hue, light, saturation=0.55):
    """The colour of `hue` (degrees) at `light`, the page's saturation unless told."""
    return tuple(int(255 * c) for c in colorsys.hls_to_rgb(hue / 360.0, light, saturation))


def fit(text, width, tail=False):
    """`text` cut to `width` cells, the cut marked - its head kept, or its `tail` (a path's file)."""
    if len(text) <= width:
        return text
    return CUT + text[len(text) - width + 1:] if tail else text[:max(0, width - 1)] + CUT


def cells(across, counts, least=1):
    """Each count's share of `across`, `least` at the least, the largest remainders first; the counts past
    what `across` holds of `least` get none."""
    counts = counts[:across // least]
    total = float(sum(counts)) or 1.0
    want = [across * c / total for c in counts]
    got = [max(least, int(w)) for w in want]
    while sum(got) > across:
        k = max((i for i, g in enumerate(got) if g > least), key=lambda i: got[i] - want[i])
        got[k] -= 1
    while got and sum(got) < across:
        got[max(range(len(got)), key=lambda i: want[i] - got[i])] += 1
    return got


def _face(x, y):
    """The face of a slab its dot (x, y) lies on, and the dot's place along the plate: ('top', p) on the
    parallelogram SLANT deep, ('front', 0) the rows under it, ('side', 0) the slanted end between; None off it."""
    if 0 <= y <= SLANT and 0 <= x - (SLANT - y) < PLATE:
        return 'top', x - (SLANT - y)
    if SLANT < y <= 2 * SLANT and 0 <= x < PLATE:
        return 'front', 0
    d = x - PLATE
    return ('side', 0) if 0 <= d <= SLANT and SLANT - d <= y <= 2 * SLANT - d else None


def _laid():
    """A slab's two rows of cells, each cell's dots on it as (bit, face, place along the plate)."""
    rows = []
    for r in range(2):
        row = []
        for c in range(SLAB):
            dots = []
            for lane in range(2):
                for dy in range(4):
                    at = _face(2 * c + lane, 4 * r + dy)
                    if at is not None:
                        dots.append((braille.BIT[lane][dy], at[0], at[1]))
            row.append(tuple(dots))
        rows.append(tuple(row))
    return tuple(rows)


#: A slab's dots, laid once.
DOTS = _laid()


def slab(plates, front, side):
    """A slab's two rows in braille: `plates` [(first dot, past its last, colour)] along its top, each
    plate's first dot a gap but the first's; its front and its side in their colours. A cell takes the
    colour most of its dots have."""
    rows = []
    for row in DOTS:
        text = []
        for dots in row:
            bits, votes = 0, {}
            for bit, kind, p in dots:
                colour = front if kind == 'front' else side if kind == 'side' else next(
                    (c for a, b, c in plates if a < p < b or p == a == 0), None)
                if colour is not None:
                    bits |= bit
                    votes[colour] = votes.get(colour, 0) + 1
            text.append(ink(max(votes, key=lambda c: votes[c]), braille.ALL[bits]) if bits else ' ')
        rows.append(''.join(text))
    return rows


def opened(tree):
    """The page's state over `tree` (atlas_tree.build): its columns, the node open (-1 the trunk), each
    list's cursor, the word found and whether it is being typed, the band's origin."""
    kids, below = atlas.tree_of(tree)
    sums = [('files', str(tree.kind.count(FILE))), ('symbols', str(tree.kind.count(SYMBOL))),
            ('ties', str(len(tree.src)))]
    return {'tree': tree, 'kids': kids, 'below': below, 'own': atlas.owners(tree), 'hue': atlas.hues(),
            'number': {layer: n for n, layer in atlas.order()}, 'cur': -1, 'at': {}, 'find': None,
            'typing': False, 'said': '', 'room': 10, 'memo': {}, 'sums': sums,
            'origin': Origin(False, 'tree', 0, 'tree', 'THE TREE AT %s' % atlas.commit(), 'tree', 0)}


def _memo(state, key, make):
    memo = state['memo']
    if key not in memo:
        if len(memo) > 64:
            memo.clear()
        memo[key] = make()
    return memo[key]


def hits(state):
    """{layer: [(node, named)]} of the word found (atlas.found); none under two letters."""
    word = state['find'] or ''
    return _memo(state, ('find', word),
                 lambda: atlas.found(state['tree'], word, state['own']) if len(word) > 1 else {})


def items(state):
    """What the lens lists: a word's hits while one is found, the layers at the trunk, else the node's
    children."""
    if state['find'] is not None:
        found = hits(state)
        return [i for _n, layer in atlas.order() for i, _named in found.get(layer, ())]
    return [layer for _n, layer in atlas.order()] if state['cur'] < 0 else state['kids'][state['cur']]


def cursor(state):
    """The cursor's row in the lens's list, kept within it."""
    key = 'find' if state['find'] is not None else state['cur']
    state['at'][key] = min(max(0, state['at'].get(key, 0)), max(0, len(items(state)) - 1))
    return state['at'][key]


def lit(state):
    """([ties out, ties in, hits] a layer, {folder: ink}): the node open's ties into each layer, the folders
    it reaches (green) and those reaching it (amber), itself blue - or the word's hits, rose."""
    return _memo(state, ('lit', state['cur'], state['find']), lambda: _lit(state))


def _lit(state):
    tree, own, cur = state['tree'], state['own'], state['cur']
    layers, folders = [[0, 0, 0] for _ in LAYERS], {}
    if state['find'] is not None:
        for layer, found in hits(state).items():
            layers[layer][2] = len(found)
            folders.update((own[i][FOLDER], HIT) for i, _named in found if own[i][FOLDER] >= 0)
        return layers, folders
    if cur < 0:
        return layers, folders
    cur = tree.up[cur] if tree.kind[cur] == SYMBOL else cur
    level = tree.kind[cur]
    for s, d in zip(tree.src, tree.dst):
        a, b = own[s][level] == cur, own[d][level] == cur
        if a != b:
            other = d if a else s
            layers[own[other][LAYER]][0 if a else 1] += 1
            folders.setdefault(own[other][FOLDER], OUT if a else IN)
    if level > LAYER:
        folders[own[cur][FOLDER]] = OPEN
    return layers, folders


def trunk_lines(state):
    """The trunk in braille, its crown first, then those beside it: a slab's plates its folders - a plate's
    length its files -, its front and side under and beside them, its name and files to the right and the
    node open's ties into it under them; the crown and the foot said while nothing is open; a legend."""
    return _memo(state, ('trunk', state['cur'], state['find']), lambda: _trunk(state))


def _trunk(state):
    kids, below, hue = state['kids'], state['below'], state['hue']
    layers, folders = lit(state)
    here = state['own'][state['cur']][LAYER] if state['cur'] >= 0 else -1
    stack = atlas.order()
    said = {stack[0][1]: 'crown: up to the shunts', next(i for n, i in stack if n == '1'): 'foot: this terminal'}
    lines, apart = [], False
    for n, layer in stack:
        if not n and not apart:
            lines += ['', ink(DIM, '   beside the trunk')]
            apart = True
        out, into, found = layers[layer]
        faded = state['find'] is not None and not found
        light, plates, at = (0.30, 0.24) if faded else (0.64, 0.52), [], 0
        for k, (f, w) in enumerate(zip(kids[layer], cells(PLATE, [max(1, below[f][FILE]) for f in kids[layer]],
                                                           LEAST))):
            plates.append((at, at + w, folders.get(f, tone(hue[layer], light[k % 2], 0.68))))
            at += w
        front, side = ((52, 104, 190), (36, 72, 136)) if layer == here else \
            (tone(hue[layer], 0.18 if faded else 0.34), tone(hue[layer], 0.12 if faded else 0.23))
        top, edge = slab(plates, front, side)
        name = ink(OPEN if layer == here else DIM if faded else BRIGHT, '%2s %-9s' % (n, LAYERS[layer][0]),
                   bold=layer == here)
        lines.append('%s  %s %s' % (top, name, ink(DIM, '%4d files' % below[layer][FILE])))
        note = ink(HIT, '%d found' % found) if found else ' '.join(
            ink(rgb, '%s %d' % (arrow, count)) for rgb, arrow, count in ((OUT, '->', out), (IN, '<-', into)) if count)
        if not note and state['cur'] < 0 and state['find'] is None and layer in said:
            note = ink(DIM, said[layer])
        lines.append('%s  %s' % (edge, note))
    legend = '  '.join(ink(rgb, FULL * 2) + ' ' + ink(DIM, word)
                       for rgb, word in ((OPEN, 'open'), (OUT, 'reaches'), (IN, 'reached by'), (HIT, 'found')))
    return lines + ['', ' ' + legend]


def strip(state):
    """The trunk in a line, for a window its slabs do not fit: two cells a layer from the foot up to the
    crown, those beside it after a gap - the open one blue, a word's hits rose, the rest in their hue."""
    layers, _folders = lit(state)
    here = state['own'][state['cur']][LAYER] if state['cur'] >= 0 else -1
    stack = atlas.order()
    line = []
    for layer in [i for n, i in stack if n][::-1] + [-1] + [i for n, i in stack if not n]:
        colour = (OPEN if layer == here else HIT if layers[layer][2] else
                  tone(state['hue'][layer], 0.24 if state['find'] is not None else 0.58, 0.68)) if layer >= 0 else None
        line.append(ink(colour, FULL * 2) if colour else ' ')
    where = ('%s %s' % (state['number'][here], LAYERS[here][0])).strip() if here >= 0 else 'the trunk, foot to crown'
    return ' ' + ''.join(line) + '  ' + ink(BRIGHT, where)


def _name(state, i):
    tree, kind = state['tree'], state['tree'].kind[i]
    if state['find'] is not None:
        return tree.id[i]
    if kind == LAYER:
        return '%2s %s' % (state['number'][i], tree.id[i])
    return tree.label[i] if kind >= FILE else tree.id[i]


def _row(state, i, picked, col, width):
    tree, below, kind = state['tree'], state['below'], state['tree'].kind[i]
    count = ':%d' % tree.line[i] if kind == SYMBOL else str(below[i][FILE] if kind <= FOLDER else below[i][SYMBOL])
    left = '%s %s %-*s %5s' % (CURSOR if picked else ' ', '+' if state['kids'][i] else '.', col,
                               fit(_name(state, i), col, tail=state['find'] is not None), count)
    colour = OPEN if picked else tone(state['hue'][state['own'][i][LAYER]], 0.72)
    return ink(colour, left, bold=picked) + '  ' + ink(DIM, fit(tree.brief[i], max(0, width - len(left) - 2)))


def _head(state, width):
    tree, cur, below = state['tree'], state['cur'], state['below']
    if state['find'] is not None:
        word = state['find'] + ('_' if state['typing'] else '')
        return [ink(HIT, 'FIND ', bold=True) + ink(BRIGHT, fit(word, width - 5), bold=True),
                ink(DIM, fit('every name, signature, brief and text holding it, crown to foot', width))]
    if cur < 0:
        return [ink(OPEN, 'TRUNK ', bold=True) + ink(BRIGHT, fit('from the terminal at its foot up to the shunts',
                                                                 width - 6), bold=True),
                ink(DIM, fit('%d files, %d symbols, %d ties' % (tree.kind.count(FILE), tree.kind.count(SYMBOL),
                                                               len(tree.src)), width))]
    kind = KINDS[tree.kind[cur]].upper()
    held = ', '.join(atlas.counted(below[cur][k], KINDS[k]) for k in range(tree.kind[cur] + 1, 4) if below[cur][k])
    name = tree.label[cur] if tree.kind[cur] == SYMBOL else tree.id[cur]
    lines = [ink(OPEN, kind + ' ', bold=True) + ink(BRIGHT, fit(name, width - len(kind) - 1), bold=True)]
    lines += [ink(DIM, line) for line in textwrap.wrap(tree.brief[cur], width)[:3]]
    where = held or 'line %d of %s' % (tree.line[cur], tree.id[tree.up[cur]])
    return lines + [ink(DIM, fit(where, width))]


def lens_lines(state, width, height):
    """The node open - what it is, what for, what it holds - then its children, a word's hits while one
    is found, the cursor on one: as many as `height` leaves."""
    head, rows, at = _head(state, width), items(state), cursor(state)
    state['room'] = shown = max(3, height - len(head) - 2)
    top = min(max(0, at - shown // 2), max(0, len(rows) - shown))
    window = rows[top:top + shown]
    told = any(state['tree'].brief[i] for i in window)
    col = min(max([len(_name(state, i)) for i in window] or [8]), max(8, width * 2 // 3 if told else width - 12))
    body = [_row(state, i, k == at, col, width) for k, i in enumerate(window, top)]
    return head + [''] + body + [ink(DIM, '  %d of %d' % (at + 1, len(rows)) if rows else '  nothing beneath')]


def _ranked(tree, ties):
    total = {}
    for (node, _tie), n in ties.items():
        total[node] = total.get(node, 0) + n
    return sorted(total.items(), key=lambda kv: (-kv[1], tree.id[kv[0]]))


def boxes(state):
    """The instruments: where the node open lies, what it reaches and what reaches it - a word's hits a
    layer while one is found - and the atlas's sums."""
    tree, cur, own, out = state['tree'], state['cur'], state['own'], []
    if state['find'] is not None:
        found = hits(state)
        out.append(hud('FOUND', [(str(len(found[layer])), '%2s %s' % (n, LAYERS[layer][0]))
                                 for n, layer in atlas.order() if layer in found] or ['none yet']))
    elif cur >= 0:
        where = [('layer', '%s %s' % (state['number'][own[cur][LAYER]], tree.id[own[cur][LAYER]]))]
        where += [(KINDS[k], fit(tree.id[own[cur][k]], 30, tail=True)) for k in (FOLDER, FILE) if own[cur][k] >= 0]
        out.append(hud('HERE', where + ([('line', str(tree.line[cur]))] if tree.kind[cur] == SYMBOL else [])))
        reach, reached = _memo(state, ('ties', cur), lambda: atlas.ties_of(tree, cur, own))
        for title, ties in (('REACHES', reach), ('REACHED BY', reached)):
            ranked = _ranked(tree, ties)
            out.append(hud(title, [(str(n), fit(tree.id[j], 30, tail=True)) for j, n in ranked[:7]]
                           + (['%d more' % (len(ranked) - 7)] if len(ranked) > 7 else []) or ['none']))
    return out + [hud('ATLAS', state['sums'] + [state['said'] or 'W lays it as a page'])]


def beside(left, right, at):
    """Two columns of lines side by side, the left padded to `at` cells."""
    rows = max(len(left), len(right))
    left, right = left + [''] * (rows - len(left)), right + [''] * (rows - len(right))
    return [a + ' ' * max(0, at - _screen.visible(a)) + '  ' + b for a, b in zip(left, right)]


def _moved(state, key, _now):
    """UP DOWN PGUP PGDN HOME END: the cursor along the lens's list."""
    n, step = len(items(state)), max(1, state['room'] - 1)
    k = 'find' if state['find'] is not None else state['cur']
    by = {'up': -1, 'down': 1, 'pageup': -step, 'pagedown': step, 'home': -n, 'end': n}[key]
    state['at'][k] = min(max(0, state['at'].get(k, 0) + by), max(0, n - 1))


def _opened(state, _key, _now):
    """ENTER RIGHT: the node under the cursor opened - a hit's own node, the word let go."""
    rows = items(state)
    if rows:
        state['cur'] = rows[cursor(state)]
        state['find'], state['typing'] = None, False


def _upped(state, _key, _now):
    """BKSP LEFT: up a level, the cursor on the node left."""
    cur = state['cur']
    if cur >= 0:
        state['cur'] = up = state['tree'].up[cur]
        siblings = items(state)
        state['at'][up] = siblings.index(cur) if cur in siblings else 0


def _finding(state, _key, _now):
    """/: a word to find, typed."""
    state['find'], state['typing'] = '', True


def _typed(state, key, _now):
    """The page's mode while a word is typed: a character into it, BKSP one back, ENTER it kept, ESC it let
    go - and ESC lets a kept word go before it leaves the page. The rest to their rows."""
    if not state['typing']:
        if key == '\x1b' and state['find'] is not None:
            state['find'] = None
            return None
        return bindings.PASS
    if key in ENTER_KEYS:
        state['typing'] = False
    elif key == '\x1b':
        state['find'], state['typing'] = None, False
    elif key in ('\x08', '\x7f'):
        state['find'] = state['find'][:-1]
    elif len(key) == 1 and key.isprintable():
        state['find'] += key
    else:
        return bindings.PASS
    return None


def _web(state, _key, _now):
    """W: the same atlas laid as a page (tools/dev/atlas.py --html) and opened in the browser."""
    path = atlas.page(state['tree'])
    webbrowser.open('file:///' + path.replace('\\', '/'))
    state['said'] = 'page: ' + fit(path, 30, tail=True)


#: The page's keys, rows of the terminal's one table (`terminal.ui.bindings`).
ROWS = (
    ('ATLAS', 'UP DOWN', 'the cursor to the next node or the last, PGUP PGDN HOME END a page or all',
     (('UP DOWN', 'PICK'),), dict.fromkeys(('up', 'down', 'pageup', 'pagedown', 'home', 'end'), _moved)),
    ('ATLAS', 'ENTER RIGHT', 'the node under the cursor opened', (('ENTER', 'OPEN'),),
     dict.fromkeys(ENTER_KEYS + ('right',), _opened)),
    ('ATLAS', 'BKSP LEFT', 'up a level', (('BKSP', 'UP'),), dict.fromkeys(('\x08', '\x7f', 'left'), _upped)),
    ('ATLAS', '/', 'a word found in every name, signature, brief and text', (('/', 'FIND'),), {'/': _finding}),
    ('ATLAS', 'W', 'the same atlas as a page, in the browser', (('W', 'WEB'),), bindings.cased('w', _web)),
)
PAD = bindings.page(ROWS, take=_typed)


def draw_of(view, state, keys=''):
    """The frame: the trunk and the lens in the viewport, the instruments beside it, the keys under; `keys`
    typed a frame each first ('.' none), as the console would type them."""
    script = list(keys)

    def draw():
        typed = script.pop(0) if script else '.'
        if typed != '.':
            bindings.act_on(PAD, [typed], state)
        width = max(40, room(view.size.width or 120) - 4)
        height = max(12, (view.size.height or 40) - 6)
        trunk = trunk_lines(state)
        if width - TRUNK - 2 >= LENS and height > len(trunk):
            art = beside(trunk, lens_lines(state, width - TRUNK - 2, height - 1), TRUNK)
        else:
            art = [strip(state), ''] + lens_lines(state, width, height - 3)
        art = [''] + [line + ' ' * max(0, width - _screen.visible(line)) for line in art]
        return frame_of(view, state['origin'], TITLE, '\n'.join(art), boxes(state), bindings.shown(PAD, state),
                        art_title='TRUNK')
    return draw


def main(argv=None):
    """The page until ESC or Q: its answer to the loader."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT, help="the loader's; the atlas reads no board")
    parser.add_argument('--simulated', action='store_true', help="the loader's; nothing here is simulated")
    parser.add_argument('--frames', type=int, default=0, help='stop after this many - the smoke')
    parser.add_argument('--keys', default='', help="a key a frame from the first, '.' none (tools/render/page.py)")
    args = parser.parse_args(argv)
    view = stage()
    with boot('READING THE TREE', view):
        tree = build()
    state = opened(tree)
    say('ok', 'atlas', '%d files, %d symbols, %d ties at %s' % (
        tree.kind.count(FILE), tree.kind.count(SYMBOL), len(tree.src), atlas.commit()))
    leaving = run_view(view, view.is_terminal, 1.0 / FPS_CAP, args.frames, draw_of(view, state, args.keys),
                       pad=PAD, state=state)
    return answer(leaving)


if __name__ == '__main__':
    sys.exit(main())
