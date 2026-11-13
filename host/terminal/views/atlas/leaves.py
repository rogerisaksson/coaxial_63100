"""The atlas's leaves in panels on its viewport, framed as her eyes' picture is.

A panel a key's branch: the nodes F1 to F12 offer hold leaves laid out ahead - at the trunk a layer's folders, under a layer a
folder's files, under a folder a file's symbols -, named in no word of the tree. Each branch with leaves is
a panel at the drawing's edge on the side its node stands, the branch nearest the viewport first
(`ranked`, the key the panel beside marks): it active - wide, bright, a leaf a line with its brief beside, the heaviest first -, the next
idle - narrow, dim, two names a line, the rest counted -, as many as MOST and the edges hold, stacked down
from the top. Turned, the tree hands the panels on.
"""
import math

from coaxial.graphics.callouts import FRAME_CORNERS
from terminal.views.atlas.scene import BRIEF, CUT, FRAME, FRAME_DIM, KEY, NAME, NAME_DIM, NOTE, TITLE, short
from tools.dev.atlas_tree import KINDS

#: The most panels; an active panel's width - a share of the drawing's, its least and its most, cells -, an
#: idle one's and its rows; the rows kept free at the drawing's foot, its clock and its tag standing there.
MOST, ACTIVE, IDLE, IDLE_ROWS, FOOT = 4, (1 / 3, 26, 44), (1 / 4, 18, 30), 2, 2


def ranked(fell, keyed, rings, cols, rows):
    """The keys' nodes with leaves drawn within the drawing, nearest the viewport first - the least depth,
    the nearest the drawing's middle between equals -: [((depth, dots off the middle), node)]; braille's two
    dots a cell across and four down, so the middle of `cols` by `rows` cells is (cols, 2 * rows)."""
    return sorted(((fell[n][2], math.hypot(fell[n][0] - cols, fell[n][1] - 2 * rows)), n) for n in keyed
                  if rings.get(n) and n in fell and 0 <= fell[n][0] < 2 * cols and 0 <= fell[n][1] < 4 * rows)


def _side(fell, taken, cols, rows, wide, node):
    """Whether a panel `wide` cells in its frame stands at the right: where it covers less of what is drawn - the
    nodes fallen under it, the names taken there -, away from its node between equals."""
    def covers(right):
        left = cols - wide - 2 if right else 0
        return sum(1 for x, y, _z in fell.values() if left <= x / 2 <= left + wide + 1 and 0 <= y < 4 * rows) \
            + sum(1 for c, _r in taken if left <= c <= left + wide + 1)
    return (covers(True), fell[node][0] >= cols) < (covers(False), fell[node][0] < cols)


def width(cols, active=True):
    """A panel's width on a drawing `cols` cells across, its frame's cells in: active or idle (ACTIVE, IDLE)."""
    share, least, most = ACTIVE if active else IDLE
    return max(least, min(most, int(cols * share)))


def _fit(text, wide):
    """`text` in `wide` cells, cut with an ellipsis."""
    return text if len(text) <= wide else text[:max(0, wide - 1)] + CUT


def lines_of(tree, leaves, wide, tall, active, names=None):
    """A panel's lines, [(text, ink)] runs each, `wide` cells and `tall` lines at most: active a leaf a line
    with its brief beside it, idle two names a line - a scene's `names` where it has them -; what does not
    fit counted on the last."""
    room, said = tall if active else 2 * tall, names or {}
    shown = len(leaves) if len(leaves) <= room else room - 1
    rest = ['+%d more' % (len(leaves) - shown)] if shown < len(leaves) else []
    if active:
        names = [said.get(n) or short(tree, n) for n in leaves[:shown]]
        named = min(max(map(len, names), default=0), wide // 2)
        briefed = wide - named - 2
        out = [[(_fit(name, named).ljust(named + 2), NAME)]
               + ([(_fit(tree.brief[n], briefed), BRIEF)] if briefed > 1 else []) for name, n in zip(names, leaves)]
        return out + [[(text, BRIEF)] for text in rest]
    half = (wide - 1) // 2
    words = [_fit(said.get(n) or short(tree, n), half) for n in leaves[:shown]] + rest
    return [[(' '.join(word.ljust(half) for word in words[k:k + 2]), NAME_DIM)] for k in range(0, len(words), 2)]


def _runs(busy, start, stop):
    """The runs of rows from `start` to `stop` none of them `busy`: [(its first row, its rows)]."""
    out, first = [], None
    for r in range(start, stop + 1):
        if r < stop and r not in busy:
            first = r if first is None else first
        elif first is not None:
            out.append((first, r - first))
            first = None
    return out


def _put(out, col, row, runs, stop):
    """`runs` [(text, ink)] from (col, row) on in `out`, none at or past column `stop`."""
    for text, ink in runs:
        for ch in text:
            if col < stop:
                out[(col, row)] = (ch, ink)
            col += 1


def _laid(out, left, top, wide, lines, title, note, active):
    """`lines` framed from (left, top) in `out` {(col, row): (char, ink)}, `wide` cells inside: her eyes'
    frame - the instruments' rounded corners -, `title` runs in its top edge and `note` in its bottom one."""
    frame, tall = FRAME if active else FRAME_DIM, len(lines)
    for row, (a, b) in ((top, FRAME_CORNERS[:2]), (top + tall + 1, FRAME_CORNERS[2:])):
        _put(out, left, row, [(a + '─' * wide + b, frame)], left + wide + 2)
    for r, runs in enumerate(lines):
        _put(out, left, top + 1 + r, [('│' + ' ' * wide + '│', frame)], left + wide + 2)
        _put(out, left + 1, top + 1 + r, runs, left + wide + 1)
    _put(out, left + 1, top, title, left + wide + 1)
    _put(out, left + wide + 1 - len(note), top + tall + 1, [(note, NOTE if active else FRAME_DIM)], left + wide + 1)


def panels(tree, rings, fell, keyed, cols, rows, taken=frozenset(), say=None, noted=None, most=MOST, docked=False,
           names=None, side=None):
    """{(col, row): (char, ink)}: the branches with leaves as panels on a drawing `cols` by `rows` cells
    (`ranked`; `rings` {node: its children as laid}, its lines by `say` and its note by `noted` - `lines_of`
    and its leaves' count where None, the mirror's flows its own; a scene's `names` where it has them, a
    page's headline (atlas/apps.py)) - the first laid active, the rest idle,
    `most` at most -, each at the edge where it covers less of what is drawn (`_side`), the active one at `side`
    where told (right or not: the side its frame left it, atlas/apps.py), on the other where that has no room -,
    or `docked` all at the right,
    stacked down from the top in rows free of a cell `taken` (the keys' names): the active one in the
    longest such run, as many of its leaves as it holds, an idle one in the first; none where its edge has no
    room left above the foot, none wider than half the drawing."""
    out, tops, laid = {}, {False: 0, True: 0}, 0
    for _off, node in ranked(fell, keyed, rings, cols, rows):
        active, leaves = not laid, rings[node]
        wide = width(cols, active) - 2
        own = docked or (side if side is not None and active else _side(fell, taken, cols, rows, wide, node))
        if laid == most or wide + 2 > cols // 2:
            continue
        for right in (own, not own) if not docked else (own,):
            left = cols - wide - 2 if right else 0
            busy = {r for c, r in taken if left <= c <= left + wide + 1}
            runs = [run for run in _runs(busy, tops[right], rows - FOOT) if run[1] >= (4 if active else 3)]
            if runs:   # the active one in the longest run free, its lines as many as it holds; an idle the first
                top, room = max(runs, key=lambda run: (run[1], -run[0])) if active else runs[0]
                tall = min(room - 2, len(leaves) if active else IDLE_ROWS)
                lines = say(tree, leaves, wide, tall, active) if say else lines_of(tree, leaves, wide, tall, active, names)
                break
        else:
            continue
        kinds = set() if noted else {tree.kind[n] for n in leaves}
        note = noted(leaves) if noted else ' %d %ss ' % (len(leaves), KINDS[kinds.pop()]) if len(kinds) == 1 \
            else ' %d leaves ' % len(leaves)
        key = keyed.get(node)   # none for a child past the keys' page
        title = ([(' %s ' % key, KEY)] if key else [(' ', KEY)]) \
            + [(((names or {}).get(node) or short(tree, node)) + ' ', TITLE if active else NAME_DIM)]
        _laid(out, left, top, wide, lines, title, note, active)
        tops[right], laid = top + len(lines) + 2, laid + 1
    return out
