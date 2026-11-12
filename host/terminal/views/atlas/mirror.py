"""The atlas's mirror world: its tree turned over, what flows between its files drawn.

What holds what gives way to what flows where. M turns the page's world over: every node where its
reflection in the trunk's foot falls, the tree's edges faint, and the flows the code says pass between its
files (tools/dev/atlas_flow.py: her pass, the wire, what comes back) as bowed curves between the nodes shown
- a flow's ends taken to the nearest node shown, so a layer carries what its files pass -, those out of the
chosen node and into it lit, the rest in their kind's ink: her pass the title's, the wire the note's, what
comes back - her loop through the world, the wire's readings - the hit's. A bright dot runs each curve from
its maker to its taker (`pulsed`): the way the data goes, a loop going round. Its panels list each key's
flows, out, in and back (↺), by the file at the other end and what passes.
"""
from terminal.views.atlas import scene
from terminal.views.atlas.panel import fit
from tools.dev.atlas_tree import LAYER, LAYERS

#: Each kind's ink (atlas_flow.KINDS: her pass, the wire, what comes back); how far a flow's curve bows; a
#: curve's segments (scene.bow's points, less one); the seconds a dot takes along one, and how far apart the
#: curves' dots start, a share of the way (the golden one, so no two run together).
INKS, BOW, SPAN, TRAVEL_S, PHASE = (scene.TITLE, scene.NOTE, scene.HIT), 0.8, 12, 2.0, 0.618


def turned(p):
    """`p` (x, y, z) reflected in the trunk's foot."""
    return (p[0], -p[1], p[2])


def mirrored(placed, flow, keyed):
    """The mirror of the scene `placed` (show_atlas.placed): its nodes reflected; its flat arrays laid as
    `scene.picture` reads them - the trunk and the tree's edges faint, the flows between the nodes shown
    bowed, `arcs` where each curve's segments start -; `out` and `into` {node: flows} the chosen node's flows
    reach and come from; `rings` {key's node: its flows [(arrow, what passes, the node at the other end)]}."""
    from coaxial.model.blocks import numpy as np
    tree, focus, back = placed['tree'], placed['focus'], len(INKS) - 1
    at = {n: turned(p) for n, p in placed['at'].items()}
    lit = set(scene.chain(tree, focus) + [n for n in at if focus in scene.chain(tree, n)]) if focus >= 0 else set()
    key_of = {n: next((k for k in scene.chain(tree, n) if k in keyed), -1) for n in at}
    passed = {}
    for a, b, kind, label in flow:
        a, b = scene.shown_of(tree, at, a), scene.shown_of(tree, at, b)
        if a >= 0 and b >= 0 and a != b:
            passed.setdefault((a, b, kind), []).append(label)
    trunk = [i for i, row in enumerate(LAYERS) if row[1] == 'trunk']
    parts = [(at[a], at[b], scene.FAINT, True) for a, b in zip(trunk, trunk[1:])]
    parts += [(at[tree.up[n]], at[n], scene.FAINT, False) for n in at if tree.kind[n] != LAYER]
    out, into, rings, arcs = {}, {}, {k: [] for k in keyed}, []
    for (a, b, kind), labels in sorted(passed.items(), key=lambda kv: -len(kv[1])):
        ink = INKS[kind]
        if b in lit and a not in lit:
            ink, into[a] = scene.TIE_IN if kind != back else ink, into.get(a, 0) + len(labels)
        elif a in lit and b not in lit:
            ink, out[b] = scene.TIE_OUT if kind != back else ink, out.get(b, 0) + len(labels)
        points = scene.bow(at[a], at[b], BOW)
        arcs.append(len(parts))
        parts += [(p, q, ink, False) for p, q in zip(points, points[1:])]
        said, arrows = ', '.join(sorted(set(labels))), ('↺', '↺') if kind == back else ('→', '←')
        if key_of[a] in rings:
            rings[key_of[a]].append((arrows[0], said, b))
        if key_of[b] in rings and key_of[b] != key_of[a]:
            rings[key_of[b]].append((arrows[1], said, a))
    nodes = list(at)
    ink = [scene.OPEN if n == focus else (scene.OUT if out.get(n, 0) >= into.get(n, 0) else scene.IN)
           if n in out or n in into else scene.NODE if n in lit else scene.DIM_NODE for n in nodes]
    ends = np.array([s[0] for s in parts] + [s[1] for s in parts], float).reshape(2, -1, 3)
    flat = {'nodes': np.array(nodes, np.intp), 'pos': np.array([at[n] for n in nodes], float).reshape(-1, 3),
            'disc': np.array([scene.DISC[tree.kind[n]] for n in nodes], float), 'ink': np.array(ink, np.uint8),
            'lit': lit, 'from': ends[0], 'to': ends[1], 'segink': np.array([s[2] for s in parts], np.uint8),
            'thick': np.array([s[3] for s in parts], bool)}
    return {**placed, 'at': at, 'out': out, 'into': into, 'rings': {k: v for k, v in rings.items() if v},
            'faint': [], 'flat': flat, 'arcs': np.array(arcs, np.intp)}


def pulsed(mirror, now):
    """The `mirror` (`mirrored`) at `now`: a bright dot on each flow's curve, run from its maker to its taker
    every TRAVEL_S, the curves' dots PHASE of the way apart - the way what flows goes, a loop going round."""
    from coaxial.model.blocks import numpy as np
    arcs, flat = mirror['arcs'], mirror['flat']
    if not len(arcs):
        return mirror
    ink = flat['segink'].copy()
    ink[arcs + ((now / TRAVEL_S + np.arange(len(arcs)) * PHASE) % 1.0 * SPAN).astype(np.intp)] = scene.TOLD
    return {**mirror, 'flat': {**flat, 'segink': ink}}


def lines(tree, flows, wide, tall, active):
    """A mirror panel's lines, `wide` cells and `tall` lines at most: a flow a line - its arrow and the file
    at its other end, what passes beside -, what does not fit counted on the last."""
    shown = flows if len(flows) <= tall else flows[:tall - 1]
    named = min(max((len(scene.short(tree, n)) + 2 for _a, _s, n in shown), default=0), wide // 2)
    out = [[(fit('%s %s' % (arrow, scene.short(tree, n)), named).ljust(named + 1), scene.NAME if active
             else scene.NAME_DIM), (fit(said, wide - named - 1), scene.BRIEF)] for arrow, said, n in shown]
    return out + ([[('+%d more' % (len(flows) - len(shown)), scene.BRIEF)]] if len(shown) < len(flows) else [])


def noted(flows):
    """A mirror panel's note: how many flows."""
    return ' %d flow%s ' % (len(flows), '' if len(flows) == 1 else 's')
