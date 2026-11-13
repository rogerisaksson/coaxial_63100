"""How the atlas's trees grow: boughs out of a stem, round it by the golden angle, at Murray's angles.

A bough is an arc (`along`) out of its stem along the stem's way there, turning back to it by its angle with
its chord at that angle - sagging out of the stem, its tip rising to the light -, straight at none. Out of a
stem laid in steps its kids grow a step each (`sprout`), the last at its end: each round the stem by the
golden angle from the last (scene.GOLDEN), as leaves stand round a stem, none over another; its chord at the
angle Murray's law gives what it holds of what the stem carries where it leaves (`murray`); as long as its
own steps need, APART a step, or as far as the crown is wide there where that is further - a paraboloid's,
RING times the root of the share of what the stem carries from there up -; a leader its steps alone, the
stem going on. Which kids a stem has is what its leaves share (`nest`): each set of them a thing is shared
by a branch, kept where it nests in those kept; the kid holding most leads (`grow`), down to the leaves. The
applications' tree grows so (atlas/apps.py), and so does a subtree out of a node opened in it
(atlas/subtree.py): the same tree, inherited down.
"""
import math

from terminal.views.atlas import scene

#: A step along a bough - a folder carried, a kid out of it -, the crown's widest reach, world units; a
#: bough's straight pieces drawn.
APART, RING, STEPS = 0.22, 3.0, 12


def lean(way, turn):
    """The unit way across the unit `way`, turned `turn` round it from a side of its own."""
    side = (0.0, way[2], -way[1]) if abs(way[0]) < 0.9 else (-way[2], 0.0, way[0])
    norm = math.sqrt(sum(v * v for v in side))
    side = tuple(v / norm for v in side)
    other = (way[1] * side[2] - way[2] * side[1], way[2] * side[0] - way[0] * side[2], way[0] * side[1] - way[1] * side[0])
    return tuple(math.cos(turn) * side[k] + math.sin(turn) * other[k] for k in range(3))


def along(bough, t):
    """(where, which way) a bough (start, way, across, angle, length, whether it leads) stands `t` of its length
    out: an arc turning back to its stem's way by its angle, its chord at its angle - out of the stem half as far
    again, at its tip half -, as a bough sags out of its stem and its tip rises to the light; straight at none. A
    circle's arc: its chord as long as the arc by sinc of half the turn, along the way it heads halfway."""
    from coaxial.model.blocks import numpy as np
    start, way, across, angle, length, _leads = bough
    halfway, heads = angle * (1.5 - t / 2), angle * (1.5 - t)
    chord = float(length * t * np.sinc(angle * t / 2 / math.pi))
    return (tuple(start[k] + chord * (math.cos(halfway) * way[k] + math.sin(halfway) * across[k]) for k in range(3)),
            tuple(math.cos(heads) * way[k] + math.sin(heads) * across[k] for k in range(3)))


def murray(above, held):
    """The angle off its stem of a bough holding `held` of the `above` the stem carries where it leaves, by
    Murray's law - each bough's radius cubed as what it carries, the stem going on with the rest -: cos =
    (r0^4 + r1^4 - r2^4) / (2 r0^2 r1^2); 0 where it carries all, the stem going on as it."""
    r0, r1, r2 = (math.pow(n, 2.0 / 3.0) for n in (above, held, above - held))   # radii squared
    return math.acos((r0 * r0 + r1 * r1 - r2 * r2) / (2 * r0 * r1))


def sprout(stem, steps, kids, turn, every):
    """{kid: its bough} out of the bough `stem` laid in `steps`: `kids` [(kid, what it holds, its own steps,
    whether it leads)] in their order, the last at the stem's end, a step each before it; each turned round the
    stem by the golden angle from the last (`turn` [the count so far], a leader turning none), its chord at
    Murray's angle; as long as its own steps need, a lateral as far as the crown is wide where it leaves if
    further - RING times the root of the share of `every` the stem carries from there up."""
    out, above = {}, sum(held for _k, held, _own, _leads in kids)
    for j, (kid, held, own, leads) in enumerate(kids, steps - len(kids) + 1):
        at, way = along(stem, j / steps)
        turn[0] += not leads   # the leader no organ round the stem, the stem going on
        out[kid] = (at, way, lean(way, turn[0] * scene.GOLDEN), murray(above, held),
                    max(RING * math.sqrt(above / every) * (not leads), APART * own), leads)
        above -= held
    return out


def nest(items, uses, weigh, first=-2):
    """The branches `items` part along, as a tree's do: each set of them a thing is shared by (`uses` {thing: the
    items sharing it}) - two or more, not all -, the most shared first (`weigh` of its things), kept where it
    nests in those kept. {'up': {branch or item: the branch it hangs from, None the root}, 'kids': {None or a
    branch: its items, then its branches, the most first}, 'held': {branch: its items}, 'by': {branch: its
    things, the heaviest first}}; a branch an id from `first` down, the one holding most first."""
    items, sets = frozenset(items), {}
    for thing, users in uses.items():
        shared = frozenset(users) & items
        if 1 < len(shared) < len(items):
            sets.setdefault(shared, []).append(thing)
    kept = []
    for s in sorted(sets, key=lambda s: (-sum(weigh(x) for x in sets[s]), -len(s), sorted(s))):
        if all(s <= k or k <= s or not s & k for k in kept):
            kept.append(s)
    kept.sort(key=lambda s: (-len(s), sorted(s)))
    ids = {s: first - k for k, s in enumerate(kept)}
    up = {ids[s]: next((ids[k] for k in reversed(kept) if s < k), None) for s in kept}
    up.update((i, next((ids[k] for k in reversed(kept) if i in k), None)) for i in items)
    kids = {None: [], **{b: [] for b in ids.values()}}
    for node in sorted(up, key=lambda n: (n not in items, -len(kept[first - n]) if n not in items else n)):
        kids[up[node]].append(node)
    return {'up': up, 'kids': kids, 'held': {ids[s]: s for s in kept},
            'by': {ids[s]: sorted(sets[s], key=lambda x: (-weigh(x), x)) for s in kept}}


def grow(stem, node, nested, held, steps, every, turn, out):
    """Into `out` {kid: its bough}: `node`'s kids (`nested` {node: its kids}) out of its bough `stem` laid in
    steps(node), each holding held(kid) - the kid with kids of its own holding the most last, leading on, the
    rest round the stem (`sprout`, `every` all the root holds) -, each kid's own out of its bough so, down to the
    leaves."""
    kids = nested.get(node, ())
    boughs = [k for k in kids if nested.get(k)]
    leader = max(boughs, key=lambda k: (held(k), -k)) if boughs else None
    order = sorted((k for k in kids if k != leader), key=lambda k: (-held(k), k)) + [leader] * bool(boughs)
    out.update(sprout(stem, steps(node), [(k, held(k), steps(k), k == leader) for k in order], turn, every))
    for k in order:
        if nested.get(k):
            grow(out[k], k, nested, held, steps, every, turn, out)


def pieces(grown):
    """[(from, to, bough, whether it leads)]: the boughs `grown` {bough: (start, way, across, angle, length,
    leads)} each in STEPS straight pieces (`along`)."""
    return [(along(b, i / STEPS)[0], along(b, (i + 1) / STEPS)[0], n, b[5]) for n, b in grown.items()
            for i in range(STEPS)]
