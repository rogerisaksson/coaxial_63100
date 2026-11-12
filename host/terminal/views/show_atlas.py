#!/usr/bin/env python3
"""The atlas on the terminal: the system as a tree in 3D, turned and opened by the mouse and F-keys.

    python terminal/views/show_atlas.py
    python terminal/views/show_atlas.py --frames 30 --keys f4,.,f1

The terminal stands at the trunk's foot and the shunts at its crown. The tree is tools/dev/atlas_tree.py's,
read from the files as they stand when the page opens; the scene is terminal/views/atlas/scene.py's, the
panel beside it terminal/views/atlas/panel.py's. The page holds the mouse from its start: a left drag turns
the tree, a right drag moves it, the wheel brings it near. The nodes F1 to F12 offer (or 1 to 9 and 0,
where a terminal keeps a function key) are open already, their own children laid out ahead, and dim with
the rest of the tree; the key choosing one lights it, those above it and all laid out below it - at the
trunk the twelve layers, the terminal F1 -, PGUP PGDN the next twelve, BKSP chooses the node above, a
click the node under it and ENTER the child nearest the viewport. The farther a thing from the point the
view turns about, the more out of focus - its dots fewer, scattered, darker. The tree names no leaf: a
key's leaves are a panel on the viewport, the branch nearest the viewport the active one, the panel beside
marking its key (terminal/views/atlas/leaves.py). Python's package plumbing - a package's __init__.py, its
__main__.py - is left out, its ties landing on its folder. Along the viewport's top the child nearest it
says its name and its brief; as the wheel brings it near, the panel beside names what it holds, the
deeper the nearer: layers, then folders, files, symbols - Tab folds it, as every page's column. M turns
the page to its mirror world, faced whole: the files a flow joins as a schematic of the data's way in
box-drawing lines - her pass a loop closed by what comes back through the world, the wire columns from the
host's end to the board's, a dot running each flow -, a panel the flows of the child nearest the viewport;
M again, the tree as it was seen (terminal/views/atlas/mirror.py). W lays the same atlas as a page
(tools/dev/atlas.py --html) and opens it.
"""
import argparse
import functools
import math
import sys
import threading
import time
import webbrowser

from rich.segment import Segment

from coaxial.comm.session import Origin
from terminal.loader import answer
from terminal.ui import bindings
from terminal.ui.marquee import Marquee, styled
from terminal.ui.screen import ENTER_KEYS, FPS_CAP, PORT, run_view, say
from terminal.ui.stage import boot, frame_of, stage
from terminal.ui.taskbar import room
from terminal.views.atlas import leaves, mirror, panel, scene
from tools.dev import atlas, atlas_flow
from tools.dev.atlas_tree import FILE, FOLDER, KINDS, LAYER, LAYERS, SYMBOL, build

TITLE = 'SYSTEM ATLAS'
#: The keys that open the context's nodes in its order: F1 to F12, and the digits 1 to 9 and 0 for a
#: terminal that keeps a function key to itself (F11 its full screen); a page of them.
FKEYS, DIGITS, PAGE = tuple('f%d' % k for k in range(1, 13)), tuple('1234567890'), 12
#: A left drag's degrees a cell across and a row down, the tip's bounds; an arrow's turn and tip; a key's
#: zoom, the distance's bounds; the ease's time constant, s; O's orbit, degrees a second.
DRAG_DEG, TIP_DEG, KEY_DEG = (2.0, 3.0), (-80.0, 85.0), (10.0, 5.0)
ZOOM_KEY, DISTANCE, EASE_S, ORBIT_DEG_S = 1.25, (0.25, 40.0), 0.2, 9.0
#: How near a node's frame the camera is when the panel lists its children; a click's reach, dots.
LISTS_AT, CLICK_DOTS = 1.3, 10.0


def plumbing(tree):
    """Whether each node is Python's package plumbing or lies in it: a file named __x__.py - a package's
    __init__.py, its __main__.py - and its symbols. A parent comes before its children in the tree."""
    out = [False] * len(tree.kind)
    for i, (up, kind, name) in enumerate(zip(tree.up, tree.kind, tree.label)):
        out[i] = (up >= 0 and out[up]) or (kind == FILE and name.startswith('__') and name.endswith('__.py'))
    return out


def opened(tree):
    """The page's state over `tree` (atlas_tree.build): its columns - Python's package plumbing left out,
    the ties into it landing on its folder -, the ties between layers, the node open (-1 the trunk), the
    camera and the frame it eases to, the word found, the band's origin."""
    hidden = plumbing(tree)
    kids, below = atlas.tree_of(tree, [not h for h in hidden])
    own = atlas.owners(tree)
    between = [[0] * len(LAYERS) for _ in LAYERS]
    for s, d in zip(tree.src, tree.dst):
        if own[s][LAYER] != own[d][LAYER]:
            between[own[s][LAYER]][own[d][LAYER]] += 1
    sums = [('files', str(tree.kind.count(FILE))), ('symbols', str(tree.kind.count(SYMBOL))),
            ('ties', str(len(tree.src)))]
    return {'tree': tree, 'kids': kids, 'below': below, 'hidden': hidden, 'own': own, 'between': between,
            'focus': -1, 'page': 0, 'cam': scene.camera(), 'goal': scene.camera(), 'then': None, 'orbit': False,
            'mirror': False, 'upright': None, 'flow': None, 'flowing': None,
            'find': None, 'typing': False, 'said': '', 'memo': {}, 'sums': sums, 'fell': {}, 'view': None,
            'size': (120, 40), 'origin': Origin(False, 'tree', 0, 'tree',
                                                 'THE TREE AT %s' % atlas.told(atlas.commit()), 'tree', 0)}


def _memo(state, key, make):
    memo = state['memo']
    if key not in memo:
        if len(memo) > 64:
            memo.clear()
        memo[key] = make()
    return memo[key]


def flowing(state):
    """What the code says flows between the tree's files (atlas_flow.flows), read once a page: beside its
    frames from its start (`main`, waited for where M comes first), else on first ask."""
    reading = state.get('reading')
    if reading is not None and reading is not threading.current_thread():
        reading.join()
    if state['flow'] is None:
        state['flow'] = atlas_flow.flows(state['tree'])
    return state['flow']


@functools.lru_cache(maxsize=None)
def painted(key):
    """The style of a drawn run by its key (scene.cells, scene.shaded), as the marquee reads it."""
    return styled(scene.shaded(key))


def drawing(rows):
    """The scene's rows (scene.cells) as the viewport's lines, a blank one above: styled Segments, widths known."""
    lines = [[Segment(line[a:b], painted(k)) for a, b, k in zip(go, go[1:] + [len(line)], keys)]
             for line, go, keys in rows]
    return Marquee([[]] + lines, centred=True, widths=[0] + [len(line) for line, _go, _keys in rows])


def placed(state):
    """The scene at the node open (scene.layout, scene.tied): where each node shown stands, the stubs, the
    reaches, its ties to the nodes shown; at the trunk the faint ties between layers, the most first - its
    mirror world (`mirror.mirrored`) while M holds."""
    focus, tree, between = state['focus'], state['tree'], state['between']

    def make():
        at, stubs, reach, rings = scene.layout(tree, state['kids'], state['below'], between, focus)
        out, into = scene.tied(tree, state['own'], at, focus) if focus >= 0 else ({}, {})
        pairs = sorted(((between[a][b] + between[b][a], a, b) for a in range(len(LAYERS))
                        for b in range(a + 1, len(LAYERS))), reverse=True)
        return {'tree': tree, 'focus': focus, 'at': at, 'stubs': stubs, 'reach': reach, 'rings': rings,
                'out': out, 'into': into, 'faint': [(a, b) for n, a, b in pairs[:scene.FAINT_TIES] if n]}
    upright = _memo(state, ('placed', focus), make)
    if not state['mirror']:
        return upright
    width, height = sized(state)
    room_ = (width, height, leaves.width(width))   # laid for the drawing, beside the panel where it fits
    return _memo(state, ('mirror', focus, room_), lambda: mirror.mirrored(upright, flowing(state), room_))


def hits(state):
    """{layer: [(node, named)]} of the word found (atlas.found), the plumbing the page leaves out not among
    them; none under two letters."""
    word, hidden = state['find'] or '', state['hidden']

    def make():
        found = atlas.found(state['tree'], word, state['own']) if len(word) > 1 else {}
        shown = {layer: [(node, named) for node, named in rows if not hidden[node]] for layer, rows in found.items()}
        return {layer: rows for layer, rows in shown.items() if rows}
    return _memo(state, ('find', word), make)


def context(state):
    """The nodes the keys open, in order: a word's hits while one is found, the twelve layers at the
    trunk (the terminal F1), else the open node's children as its ring or cap lays them."""
    if state['find'] is not None:
        return [node for layer in sorted(hits(state)) for node, _named in hits(state)[layer]]
    if state['focus'] < 0:
        return list(range(len(LAYERS)))
    return scene.children(state['tree'], state['kids'], state['below'], state['focus'])


def keyed(state):
    """{node: its key} on the context's page: F1 to F12."""
    nodes = context(state)
    state['page'] = min(state['page'], max(0, (len(nodes) - 1) // PAGE))
    start = state['page'] * PAGE
    return {node: 'F%d' % (k + 1) for k, node in enumerate(nodes[start:start + PAGE])}


def sized(state):
    """(cells across, rows) of the drawing on the terminal's size, the column's room left beside it."""
    return max(40, room(state['size'][0])), max(12, state['size'][1] - 6)


def frame(state, node):
    """(target, distance) that frames `node` and what it opens - the trunk whole at none -; in the mirror
    its flow world whole (mirror.framed)."""
    if state['mirror']:
        return mirror.framed(placed(state), *sized(state))
    if node < 0:
        home = scene.camera()
        return home['target'], home['distance']
    tree, p = state['tree'], placed(state)['at'][node]
    n, kind = len(state['kids'][node]), tree.kind[node]
    if kind == LAYER:
        return p, 2.8 * (scene.RING[0] + scene.RING[1] * n) + 1.0
    if kind in (FOLDER, FILE) and n:
        r = scene.CAP[kind][0] + scene.CAP[kind][1] * math.sqrt(n)
        return (p[0], p[1] - 0.55 * r, p[2]), 2.8 * r + 0.5
    return p, 0.8


def _mirrored(state, _key, _now):
    """M: the mirror world (`mirror`) faced whole, the tree's view kept; M again, the tree as it was seen -
    framed anew where another node opened meanwhile."""
    cam, goal = state['cam'], state['goal']
    state['mirror'] = not state['mirror']
    if state['mirror']:
        state['upright'] = (state['focus'], dict(goal), cam['yaw'], cam['pitch'])
        cam['yaw'] = cam['pitch'] = 0.0
        goal['target'], goal['distance'] = frame(state, state['focus'])
        return
    focus, seen, cam['yaw'], cam['pitch'] = state['upright']
    goal['target'], goal['distance'] = (seen['target'], seen['distance']) if focus == state['focus'] \
        else frame(state, state['focus'])


def focused(state, node):
    """`node` open - those above it with it, the trunk alone at none -, the keys on their first page, the
    word let go; the camera eased to frame it."""
    state['focus'], state['page'], state['find'], state['typing'] = node, 0, None, False
    state['goal']['target'], state['goal']['distance'] = frame(state, node)


def listing(state, mid):
    """The node whose children the panel lists: the deepest of the middle node's chain - shown, each, as
    only an open node's children are - the camera is as near as LISTS_AT its frame; -1 (the layers) where
    it is near none."""
    tree, distance = state['tree'], state['cam']['distance']
    for node in reversed(scene.chain(tree, mid) if mid is not None else []):
        if tree.kind[node] < SYMBOL and state['kids'][node] and distance <= LISTS_AT * frame(state, node)[1]:
            return node
    return -1


def eased(state, now):
    """The camera a frame on: its target and distance eased to the frame asked (EASE_S), O's orbit."""
    then, state['then'] = state['then'], now
    dt = 0.0 if then is None else min(0.25, max(0.0, now - then))
    cam, goal, k = state['cam'], state['goal'], 1.0 - math.exp(-dt / EASE_S)
    cam['target'] = tuple(c + (g - c) * k for c, g in zip(cam['target'], goal['target']))
    cam['distance'] *= (goal['distance'] / cam['distance']) ** k
    if state['orbit'] and not state['mirror']:
        cam['yaw'] += ORBIT_DEG_S * dt


def turned(state, dx, dy):
    """A left drag of (dx, dy) cells, or an arrow's: the tree turned with it, tipped within TIP_DEG - the
    mirror's flat world moved with it (`moved`)."""
    cam = state['cam']
    if state['mirror']:
        return moved(state, dx, dy)
    cam['yaw'] -= dx * DRAG_DEG[0]
    cam['pitch'] = max(TIP_DEG[0], min(TIP_DEG[1], cam['pitch'] + dy * DRAG_DEG[1]))


def moved(state, dx, dy):
    """A right drag of (dx, dy) cells: what was under it stays under it - the target moved in the view's
    plane by the drag's dots at its distance."""
    v, cam = state['view'], state['cam']
    if v is None:
        return
    m = cam['distance'] / v['focal']
    cam['target'] = tuple(t - v['r'][k] * 2 * dx * m + v['u'][k] * 4 * dy * m for k, t in enumerate(cam['target']))
    state['goal']['target'] = cam['target']


def zoomed(state, k):
    """The camera `k` times nearer, within DISTANCE - in the mirror no farther than its framing -; held there."""
    cam, far = state['cam'], frame(state, state['focus'])[1] if state['mirror'] else DISTANCE[1]
    cam['distance'] = state['goal']['distance'] = max(DISTANCE[0], min(far, cam['distance'] / k))


def wheeled(state, _keys, moved_by):
    """The wheel's notches in a frame (screen.WHEEL_STEP each, up nearer): the zoom e to their sum, so each
    notch is the same step in and out - 1 + their sum went negative past eight notches down and the
    camera leapt to its nearest."""
    if moved_by:
        zoomed(state, math.exp(moved_by))


def mouse(state):
    """`run_view`'s mouse for the page: held from the page's start - the terminal's own selection and
    paste took the clicks of a page that waited for F, a chord of both buttons froze the tree -; a left
    drag turns the tree, a right drag moves it, the wheel brings it near, a left click on the drawing
    opens the node under it."""
    return {'mouse': True, 'select': frozenset(),
            'on_input': lambda keys, moved_by: wheeled(state, keys, moved_by),
            'on_click': lambda col, row: clicked(state, col, row) if col < room(state['size'][0]) + 2 else None,
            'on_drag': lambda dx, dy: turned(state, dx, dy), 'on_pan': lambda dx, dy: moved(state, dx, dy)}


def clicked(state, col, row):
    """A left click at the screen's (col, row): the node drawn nearest it opened, within CLICK_DOTS."""
    width, height = state['size']
    across, down = state['art']
    x = 2 * (col - 2 - (room(width) - across) // 2) + 1
    y = 4 * (row - 2 - (height - 4 - down) // 2 - 1) + 2
    fell = state['fell']
    near = min(fell, key=lambda n: (fell[n][0] - x) ** 2 + (fell[n][1] - y) ** 2, default=None)
    if near is not None and math.hypot(fell[near][0] - x, fell[near][1] - y) <= CLICK_DOTS:
        focused(state, near)


def _keyed(state, key, _now):
    """F1 to F12, 1 to 9 and 0: the node whose name carries the key opened."""
    k = (FKEYS.index(key) if key in FKEYS else DIGITS.index(key))
    nodes = list(keyed(state))
    if k < len(nodes):
        focused(state, nodes[k])


def _paged(state, key, _now):
    """PGUP PGDN: the keys on the context's twelve before or after."""
    state['page'] = max(0, state['page'] + (1 if key == 'pagedown' else -1))


def _upped(state, _key, _now):
    """BKSP: the node open closed, the one above it open."""
    if state['find'] is None and state['focus'] >= 0:
        focused(state, state['tree'].up[state['focus']])


def _middled(state, _key, _now):
    """ENTER: the child nearest the viewport opened (`panel.middle`), whichever page of keys holds it - in
    the mirror the node lit (mirror.middle)."""
    v, fell = state['view'], state['fell']
    mid = None if v is None else mirror.middle(placed(state), fell, v, state['art'][0]) if state['mirror'] \
        else panel.middle(fell, v, set(context(state)))
    if mid is not None:
        focused(state, mid)


def _turned(state, key, _now):
    """The arrows: the tree turned and tipped a step."""
    turned(state, {'left': -1, 'right': 1}.get(key, 0) * KEY_DEG[0] / DRAG_DEG[0],
           {'up': -1, 'down': 1}.get(key, 0) * KEY_DEG[1] / DRAG_DEG[1])


def _homed(state, _key, _now):
    """HOME: the view turned back - in the mirror to face its world -, the node open framed."""
    home = scene.camera(0.0, 0.0) if state['mirror'] else scene.camera()
    state['cam']['yaw'], state['cam']['pitch'] = home['yaw'], home['pitch']
    state['goal']['target'], state['goal']['distance'] = frame(state, state['focus'])


def _finding(state, _key, _now):
    """/: a word to find, typed."""
    state['find'], state['typing'], state['page'] = '', True, 0


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
    state['said'] = 'page: ' + panel.fit(path, 30, tail=True)


#: The page's keys, rows of the terminal's one table (`terminal.ui.bindings`).
ROWS = (
    ('ATLAS', 'F1-F12', 'the node whose name carries the key opened, 1 to 9 and 0 the same',
     (('F1-F12', 'OPEN'),), dict.fromkeys(FKEYS + DIGITS, _keyed)),
    ('ATLAS', 'PGUP PGDN', 'the keys on the next twelve or the last', (('PGDN', 'MORE'),),
     dict.fromkeys(('pageup', 'pagedown'), _paged)),
    ('ATLAS', 'BKSP', 'the node open closed, the one above it open', (('BKSP', 'UP'),),
     dict.fromkeys(('\x08', '\x7f'), _upped)),
    ('ATLAS', 'ENTER', 'the child nearest the viewport opened; a click the node under it',
     (('ENTER', 'NEAREST'),), dict.fromkeys(ENTER_KEYS, _middled)),
    ('ATLAS', 'ARROWS', 'the tree turned and tipped; a left drag the same, a right drag moves it, the '
     'wheel brings it near', (), dict.fromkeys(('left', 'right', 'up', 'down'), _turned)),
    ('ATLAS', '+ -', 'nearer, farther', (), {'+': lambda s, _k, _n: zoomed(s, ZOOM_KEY),
                                           '=': lambda s, _k, _n: zoomed(s, ZOOM_KEY),
                                           '-': lambda s, _k, _n: zoomed(s, 1.0 / ZOOM_KEY)}),
    ('ATLAS', 'HOME', 'the view turned back, the node open framed', (), {'home': _homed}),
    ('ATLAS', 'O', 'the tree orbits, or stops', (('O', 'ORBIT'),),
     bindings.cased('o', lambda s, _k, _n: s.update(orbit=not s['orbit']))),
    ('ATLAS', 'M', 'the mirror world: what flows between the files, laid out the way it goes; and back',
     (('M', 'MIRROR'),), bindings.cased('m', _mirrored)),
    ('ATLAS', '/', 'a word found in every name, signature, brief and text; its hits on the keys',
     (('/', 'FIND'),), {'/': _finding}),
    ('ATLAS', 'W', 'the same atlas as a page, in the browser', (('W', 'WEB'),), bindings.cased('w', _web)),
)
PAD = bindings.page(ROWS, take=_typed)


def titled(state, mid, width):
    """The viewport's title along its top: TREE, then the child nearest the viewport (`panel.middle`) by its
    name and its brief (its kind where it has none), as wide as the frame lets it."""
    head = 'FLOW' if state['mirror'] else 'TREE'
    if mid is None:
        return head
    tree = state['tree']
    return panel.fit('%s  %s  %s' % (head, scene.short(tree, mid), tree.brief[mid] or KINDS[tree.kind[mid]]), width - 2)


def draw_of(view, state, keys=''):
    """The frame: the scene in the viewport, the panel beside it, the keys under; `keys` comma-separated
    key names typed a frame each first ('.' none: f3, pagedown, a, ...), as the console would type them."""
    script = [k for k in keys.split(',') if k]

    def draw():
        typed = script.pop(0) if script else '.'
        if typed != '.':
            bindings.act_on(PAD, ['\r' if typed == 'enter' else '\x08' if typed == 'bksp' else typed], state)
        state['size'] = (view.size.width or 120, view.size.height or 40)
        width, height = sized(state)
        turned_over = state['mirror']
        if turned_over and state.get('art') != (width, height):   # the mirror world framed for this drawing
            state['goal']['target'], state['goal']['distance'] = frame(state, state['focus'])
        eased(state, time.monotonic())
        placing, marks = placed(state), keyed(state)
        # The child nearest the viewport, whichever page of keys holds it: its panel, its mark, its brief.
        named = {n: marks.get(n, '') for n in context(state)}
        found = {scene.shown_of(state['tree'], placing['at'], node)
                 for nodes in hits(state).values() for node, _named in nodes}
        state['flowing'] = (placing['flows_out'], placing['flows_in']) if turned_over else None
        sight, now = scene.view(state['cam'], width, height), time.monotonic()
        rows, state['fell'], state['view'] = scene.picture(
            placing, state['cam'], width, height, marks, found,
            lambda fell, cols, rows, taken: mirror.over(placing, sight, marks, fell, cols, rows, taken, now)
            if turned_over else leaves.panels(state['tree'], placing['rings'], fell, named, cols, rows, taken))
        state['art'] = (width, height)
        fell = state['fell']
        mid = mirror.middle(placing, fell, sight, width) if turned_over else panel.middle(fell, sight, named)
        boxes = panel.boxes(state, listing(state, mid), mid, marks, hits(state))
        return frame_of(view, state['origin'], TITLE, drawing(rows), boxes, bindings.shown(PAD, state),
                        art_title=titled(state, mid, width))
    return draw


def main(argv=None):
    """The page until ESC or Q: its answer to the loader. The panel beside the viewport is the instruments'
    column, out or folded as Tab left it, as on every page."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT, help="the loader's; the atlas reads no board")
    parser.add_argument('--simulated', action='store_true', help="the loader's; nothing here is simulated")
    parser.add_argument('--frames', type=int, default=0, help='stop after this many - the smoke')
    parser.add_argument('--keys', default='', help="key names a frame each, comma-separated, '.' none")
    args = parser.parse_args(argv)
    view = stage()
    with boot('READING THE TREE', view):
        tree = build()
    state = opened(tree)
    # What flows between the files, read beside the first frames: M finds it read (`flowing`).
    state['reading'] = threading.Thread(target=flowing, args=(state,), daemon=True)
    state['reading'].start()
    say('ok', 'atlas', '%d files, %d symbols, %d ties at %s' % (
        tree.kind.count(FILE), tree.kind.count(SYMBOL), len(tree.src), atlas.told(atlas.commit())))
    leaving = run_view(view, view.is_terminal, 1.0 / FPS_CAP, args.frames, draw_of(view, state, args.keys),
                       pad=PAD, state=state, **mouse(state))
    return answer(leaving)


if __name__ == '__main__':
    sys.exit(main())
