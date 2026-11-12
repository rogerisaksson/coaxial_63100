"""The SYSTEM ATLAS page's panel: where the node open lies, what the nearest key holds, keys, ties.

The instruments beside the scene: the node open's layer, folder, file and brief; what the key nearest the
viewport holds at the zoom's depth; the keys that open nodes; the open node's ties; the atlas. Zoomed out,
the panel lists the layers; wheeled in as near the key nearest the viewport as its own frame - a layer's,
then a folder's, a file's -, its children: what the scene shows as dots the panel names, the deeper the
nearer, whether the node is open or not.
"""
import textwrap

from terminal.ui.stage import hud
from terminal.views.atlas import scene
from tools.dev import atlas
from tools.dev.atlas_tree import FILE, KINDS, LAYERS, SYMBOL

#: The most rows a list shows; the cells a name takes; the ties shown each way; a brief's lines.
ROWS, WIDE, TIES, BRIEF = 14, 32, 5, 3
#: The column's cells inside its frame and padding (terminal.ui.scroll.HUD_WIDTH's 40 less 4).
INNER = 36
CUT, MIDDLE, DOT = chr(0x2026), chr(0x25b8) + ' ', chr(0x25cf)
#: The legend: the scene's inks and what they tell, two rows.
LEGEND = ['  '.join('\x1b[38;2;%d;%d;%dm%s\x1b[0m %s' % (scene.INK[ink] + (DOT, said)) for ink, said in row)
          for row in (((scene.OPEN, 'open'), (scene.HIT, 'found')),
                      ((scene.OUT, 'reaches'), (scene.IN, 'reached by')))]


def fit(text, width, tail=False):
    """`text` cut to `width` cells, the cut marked - its head kept, or its `tail` (a path's file)."""
    if len(text) <= width:
        return text
    return CUT + text[len(text) - width + 1:] if tail else text[:max(0, width - 1)] + CUT


def middle(fell, v, among=None):
    """The shown node nearest the viewport ({node: (x, y, depth)} in dots, `v` the view): the least depth
    among `among` - the keys - drawn within the view, else among all drawn there, the nearest the view's
    middle between equals; None for none."""
    w, h = v['w'], v['h']
    inside = [n for n, (x, y, _depth) in fell.items() if 0 <= x < w and 0 <= y < h]
    pool = [n for n in inside if n in (among or ())] or inside
    return min(pool, key=lambda n: (fell[n][2], (fell[n][0] - w / 2) ** 2 + (fell[n][1] - h / 2) ** 2), default=None)


def under(state, base, kind):
    """The nodes of `kind` beneath `base`: a file's symbols by line, the rest the heaviest first."""
    tree, kids, below = state['tree'], state['kids'], state['below']
    level = [base]
    while level and tree.kind[level[0]] < kind:
        level = [c for n in level for c in kids[n]]
    if kind == SYMBOL:
        return sorted(level, key=lambda c: tree.line[c])
    return sorted(level, key=lambda c: (-scene.weight(tree, below, c), tree.id[c]))


def in_view(state, base):
    """(title, nodes) the panel lists: `base`'s children - the layers at none (-1)."""
    if base < 0:
        return 'LAYERS', list(range(len(LAYERS)))
    tree, kind = state['tree'], state['tree'].kind[base] + 1
    nodes = state['memo'].setdefault(('under', base), under(state, base, kind))
    return '%sS IN %s' % (KINDS[kind].upper(), fit(scene.short(tree, base), 20)), nodes


def held(state, node):
    """What a row's number tells: files under a layer or a folder, symbols in a file, a symbol's line."""
    tree = state['tree']
    return str(tree.line[node]) if tree.kind[node] == SYMBOL else str(scene.weight(tree, state['below'], node))


def here(state):
    """HERE: where the node open lies - its layer, folder, file and symbol - and its brief."""
    tree, focus = state['tree'], state['focus']
    if focus < 0:
        return hud('HERE', [('trunk', 'the terminal at its foot'), ('crown', 'the shunts, sensing'),
                            ('beside', ', '.join(row[0] for row in LAYERS if row[1] != 'trunk'))])
    own = state['own'][focus] + ([focus] if tree.kind[focus] == SYMBOL else [])
    rows = [(KINDS[k], fit(scene.short(tree, node), WIDE, tail=k == FILE)) for k, node in enumerate(own) if node >= 0]
    # A row of its own sits past the labels' column, as hud lays it: what is left is the brief's.
    brief = textwrap.wrap(tree.brief[focus] or '', INNER - 1 - max(len(label) for label, _ in rows))
    return hud('HERE', rows + brief[:BRIEF] + ([CUT] if len(brief) > BRIEF else []))


def listed(state, base, mid, keyed):
    """`base`'s children (`in_view`), a row a node - its key where one opens it, else what it holds, the
    key nearest the viewport marked -, and the keys whose nodes it leaves out."""
    tree = state['tree']
    title, nodes = in_view(state, base)
    shown = nodes[:ROWS]
    rows = [(keyed.get(n, held(state, n)), fit((MIDDLE if n == mid else '') + scene.short(tree, n), WIDE))
            for n in shown] + (['%d more' % (len(nodes) - ROWS)] if len(nodes) > ROWS else [])
    out = [hud(title, rows or ['nothing beneath'])]
    rest = [n for n in keyed if n not in shown]
    if rest:
        out.append(hud('KEYS', [(keyed[n], fit(scene.short(tree, n), WIDE)) for n in rest]))
    return out


def ties(state):
    """REACHES, REACHED BY: the open node's ties, the other end at its level (atlas.ties_of); in the mirror
    FLOWS OUT, FLOWS IN: its flows, the other end the node shown (`state['flowing']`, mirror.mirrored's)."""
    tree, focus = state['tree'], state['focus']
    if focus < 0:
        return []
    flowing = state.get('flowing')
    if flowing:
        boxes = (('FLOWS OUT', {(n, 0): k for n, k in flowing[0].items()}),
                 ('FLOWS IN', {(n, 0): k for n, k in flowing[1].items()}))
    else:
        reach, reached = state['memo'].setdefault(('ties', focus), atlas.ties_of(tree, focus, state['own']))
        boxes = (('REACHES', reach), ('REACHED BY', reached))
    out = []
    for title, said in boxes:
        total = {}
        for (node, _tie), n in said.items():
            total[node] = total.get(node, 0) + n
        ranked = sorted(total.items(), key=lambda kv: (-kv[1], tree.id[kv[0]]))
        rows = [(str(n), fit(tree.id[j], WIDE, tail=True)) for j, n in ranked[:TIES]]
        out.append(hud(title, rows + (['%d more' % (len(ranked) - TIES)] if len(ranked) > TIES else []) or ['none']))
    return out


def boxes(state, base, mid, keyed, hits):
    """The column: HERE, `base`'s children (`listed`), the keys, the ties - a word's `hits` ({layer: [(node,
    named)]}) a layer while one is found - and the atlas's sums under the legend."""
    out = [here(state)]
    if state['find'] is not None:
        out.append(hud('FOUND %s' % fit(state['find'], 16), [(str(len(hits[layer])), LAYERS[layer][0])
                                                            for layer in sorted(hits)] or ['none yet']))
    out += listed(state, base, mid, keyed) + ties(state)
    return out + [hud('ATLAS', state['sums'] + LEGEND + [state['said'] or 'W lays it as a page'])]
