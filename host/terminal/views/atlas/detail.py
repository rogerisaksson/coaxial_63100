"""What a mirror lane carries, written as it finds room: more of it nearer, out of the background.

The mirror (atlas/mirror.py) lays each lane in its cells; here what it carries is written into them where
it fits (`detailed`): in the lane's own run - cells it alone passes straight through, no head nor text on
one (schematic.laid) -, a hung stage's beside the head each way goes into. Each lane's writing has levels
(`_written`): the pass's words, then where each is made, then where it is taken, each on its side as it
flows; the wire's asks and readings counted, then the commands asked; long lists cut to what fits, '+n'
after (`_cut`). The richest level that fits is written; what it adds to the one before comes out of the
background as its room grows by a zoom key's step (`faded`). Each node's brief beside its name likewise.
"""
import itertools
import math
import re
import textwrap

from terminal.views.atlas import scene
from tools.dev import atlas_flow

#: A detail's inks as it comes out of the background (`faded`): from just fitting to a zoom key's step roomier.
FADE = (scene.FAINT, scene.DIM_NODE, scene.NAME_DIM, scene.BRIEF)


def took(tree, nodes):
    """Where a word is taken, as the tree writes it: each file once, its functions after it."""
    files = {}
    for t in nodes:
        files.setdefault(scene.short(tree, tree.up[t]), []).append(tree.id[t].rpartition('::')[2])
    return ', '.join('%s › %s' % (f, ', '.join(fns)) for f, fns in files.items())


def _chain(mark, word, maker, taken, level, right):
    """An item of a lane's writing, [(text, the level it shows from)]: `word`, `mark` before it, from level 1
    where it is made (`maker`, 'file › function', the file alone before a function of its name), from 2
    where it is taken (`taken`, `took`), each on its side as it flows (`right`)."""
    file, _, fn = maker.partition(' › ')
    units = [(0, [(mark + word, 0)])]
    if level and maker:
        units = [(0, [(file + ' › ', 1), (mark + fn, 0)])] if fn == word else [(0, [(maker, 1)]), (1, units[0][1])]
    units += [(2, [(taken, 2)])] if level > 1 and taken else []
    if not right:   # the arrows between the same units, read the other way
        units = [(0, units[-1][1])] + [(units[k + 1][0], units[k][1]) for k in range(len(units) - 2, -1, -1)]
    return [s for k, (lv, unit) in enumerate(units) for s in [(' → ' if right else ' ← ', lv)] * bool(k) + unit]


def _written(flows, level, right):
    """A lane's `flows` one way at `level` as items [[(text, the level it shows from)]]: the pass's words, from
    level 1 each with where it is made, from 2 where it is taken (`_chain`); the wire's counted from 1, the
    commands it asks from 2; what comes back marked ↺."""
    items = []
    for kind, words in flows:
        mark, n = '↺ ' * (kind == atlas_flow.KINDS.index('feedback')), len(words)
        if words[0][1] is not None:
            items += [_chain(mark * (k == 0), w, maker, taken, level, right) for k, (w, maker, taken) in enumerate(words)]
        elif level == 1 or level and mark:
            items.append([(mark + '%d' % n if mark else '%d ask%s' % (n, 's' * (n > 1)), 1)])
        elif level:
            items += [[(w, 2)] for w, _m, _t in words]
    return items


def _cut(parts, room):
    """(segments [(text, level)] `parts` [[item]] (`_written`) are written as in `room` cells - its parts
    ' · ' apart, a part's items ', ', the longest part cut to as many as fit, '+n' after; none where none
    fits -, the fewest cells they are written in: that part cut to its first)."""
    if not parts:
        return [], 0
    size = [[sum(len(t) for t, _l in item) for item in part] for part in parts]
    cut = max(range(len(parts)), key=lambda i: len(parts[i]))
    rest = 3 * len(parts) - 3 + sum(sum(z) + 2 * len(z) - 2 for i, z in enumerate(size) if i != cut)
    pre, n = list(itertools.accumulate(size[cut], initial=0)), len(size[cut])

    def sized(keep):
        return rest + pre[keep] + 2 * keep - 2 + (len(' +%d' % (n - keep)) if keep < n else 0)
    keep, out = next((k for k in range(n, 0, -1) if sized(k) <= room), 0), []
    if not keep:
        return out, sized(1)
    for i, part in enumerate(parts):
        shown, low = part[:keep] if i == cut else part, min(lv for item in part for _t, lv in item)
        out += [(' · ', low)] * bool(out)
        for j, item in enumerate(shown):
            out += [(', ', low)] * bool(j) + item
        out += [(' +%d' % (len(part) - len(shown)), low)] * (len(shown) < len(part))
    return out, sized(1)


def _levels(mirror, key, said, d, right):
    """[(parts, the fewest cells they are written in)] a level of the writing of lane `key`, `said` (what goes
    a to b, what comes back) (`_written`, `_cut`): 0 its words, 1 where they are made, 2 where they are taken -
    its way `d` alone (0 a to b), both where None, its first way flowing `right`; kept."""
    kept = mirror.setdefault('written', {})
    if (key, d, right) not in kept:
        ways = [(e, way) for e, way in enumerate(said) if d in (None, e)]
        levels = [[p for p in (_written(way, j, right == (e == 0)) for e, way in ways) if p] for j in range(3)]
        kept[(key, d, right)] = [(parts, _cut(parts, 0)[1]) for parts in levels]
    return kept[(key, d, right)]


def faded(ratio):
    """A detail's ink `ratio` times the room it needs at least (FADE): out of the background as that room grows
    by a zoom key's step (scene.ZOOM)."""
    return FADE[max(0, min(len(FADE) - 1, int((len(FADE) - 1) * math.log(ratio) / math.log(scene.ZOOM))))]


def detailed(mirror, cells, lanes, runs, slots, spots, wide, rows):
    """{(lane, way): (the level written, its cells)} - way None for both, else 0 a to b, 1 back -: into `cells`
    (mirror.laid's) on a drawing `wide` columns by `rows`, each lane's - `lanes` [(lane, what it carries)] -
    richest writing that fits (`_levels`), centred in its run (`runs`, schematic.laid) a blank either side - a
    hung stage's each way beside the head it goes into (`slots` [(col, row, the side it grows to, lane, what it
    carries, way)], mirror.mirrored's) -; then each node's brief beside its name away from its sign (`spots`),
    its first clause at least, cut at a word to what fits; each level's ink by its room (`faded`)."""
    briefs = {nd: (name[0][0], name[0][1] + (-1 if name[0][1] < sign[1] else 1)) for nd, (sign, name) in spots.items()
              if name}
    stops, said = set(briefs.values()), {}

    def free(x, y, way):   # the cells clear from (x, y) on `way`, before another brief's, the last kept clear
        n = 0
        while 0 <= x + n * way < wide and 0 <= y < rows and (x + n * way, y) not in cells \
                and not (n and (x + n * way, y) in stops):
            n += 1
        return n - 1

    def write(levels, room, x, y, way, lane):   # from x on `way`, centred in `room` cells where 0
        best = max((j for j, (_p, least) in enumerate(levels) if 0 < least <= room), default=-1)
        if best >= 0:
            segments = _cut(levels[best][0], room)[0]
            inks, size = [scene.BRIEF] + [faded(room / max(least, 1)) for _p, least in levels[1:]], \
                sum(len(t) for t, _l in segments)
            x = x - size + 1 if way < 0 else x + (room - size) // 2 * (not way)
            put = {} if way else {(x - 1, y): (' ', scene.BRIEF), (x + size, y): (' ', scene.BRIEF)}
            for text, level in segments:
                put.update({(x + c, y): (ch, inks[level]) for c, ch in enumerate(text)})
                x += len(text)
            cells.update(put)
            said[lane] = (best, sorted(put))
    slotted = {key for _x, _y, _way, key, _said, _d in slots}
    for (key, carried), run in zip(lanes, runs):
        run = [c for c in run if c[0] < wide]
        if key not in slotted and len(run) > 4:
            write(_levels(mirror, key, carried, None, run[-1][0] > run[0][0]), len(run) - 4,
                  min(c for c, _r in run) + 2, run[0][1], 0, (key, None))
    for x, y, way, key, carried, d in slots:
        write(_levels(mirror, key, carried, d, d == 0), free(x, y, way), x, y, way, (key, d))
    for nd, (x, y) in briefs.items():   # its first clause at least
        brief, room = ' '.join(mirror['tree'].brief[nd].split()), free(x, y, 1)
        clause = re.search('[:;,]', brief)
        least = clause.end() + len(scene.CUT) if clause else len(brief)
        if brief and least <= room:
            text = textwrap.shorten(brief, room, placeholder=scene.CUT)
            text = text if text == brief else text[:-len(scene.CUT)].rstrip(':;,') + scene.CUT
            cells.update({(x + c, y): (ch, faded(room / least)) for c, ch in enumerate(text)})
    return said
