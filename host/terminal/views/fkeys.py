"""F's keys held: each thing she has identified at its own function key while her eyes keep it.

    got = fkeys.slotted(state, now, offers, turn)   # [(slot, offer)], each thing at its key's place

A thing keeps its key while she turns; a new one takes the first free; one past the keys' count takes
the key of the one she faces least only HELD_RAD nearer facing, so two in a tie do not trade it.
"""
import math

#: The keys' count: F1 to F10.
KEYS = 10
#: A thing past the keys' count takes the key of the one she faces least only turned this much nearer
#: to it than to that one, rad: an eighth of a turn, her slant arrival's (`dock.ARRIVALS`).
HELD_RAD = 0.25 * math.pi


def _same(held, offer):
    """Whether `held` (verb, lo, hi) is `offer`'s thing: its verb, their footprints met over the floor."""
    verb, lo, hi = held
    a, b = offer[4]
    return verb == offer[0] and all(lo[i] <= b[i] and a[i] <= hi[i] for i in (0, 2))


def slotted(state, now, got, turn):
    """[(slot, offer)] each of `got` [(verb, word, at, thing, (lo, hi))], ranked, at its key's place,
    by place: a thing again at its own (`state['slots']`, laid anew with her scanner or its epoch), a
    new one at the first free, one past KEYS at the key of the one she faces least (`turn(now, at)`)
    only HELD_RAD nearer facing."""
    sc, mem = state.get('scanner'), state.get('slots')
    if mem is None or mem['of'][0] is not sc or mem['of'][1] != now.get('epoch'):
        mem = state['slots'] = {'of': (sc, now.get('epoch')), 'held': [None] * KEYS}
    held, at = mem['held'], {}
    for i, offer in enumerate(got):
        at.update({i: s for s in [s for s, h in enumerate(held) if h and s not in at.values() and _same(h, offer)][:1]})
    for i in [i for i in range(len(got)) if i not in at]:
        free = [s for s in range(KEYS) if s not in at.values()]
        least = None if free else max(at, key=lambda j: turn(now, got[j][2]))
        if free:
            at[i] = free[0]
        elif turn(now, got[i][2]) + HELD_RAD < turn(now, got[least][2]):
            at[i] = at.pop(least)
    for i, s in at.items():
        held[s] = (got[i][0],) + tuple(got[i][4])
    return sorted(((s, got[i]) for i, s in at.items()), key=lambda p: p[0])


def shown(state, offer, propped=None):
    """The offers of `offer` (`gym.offered`'s) at a thing in her sight (`scanner.scan`), or bearing her (`propped`)."""
    now, sc = state.get('now'), state.get('scanner')
    view = sc.scan(now) if sc is not None and now and not propped else None
    return [o for o in offer if o[3] is not None and (view is None or any(
        abs(o[3][0] - d['pos'][0]) <= d['extent'][0] + 0.1 and abs(o[3][2] - d['pos'][2]) <= d['extent'][2] + 0.1
        for d in view))]


def paired(on, docks, reach_m, onto_m):
    """`docks` paired in the room's frame with their identified offers `on` [(key, verb, word, at, ..)]:
    each dock bound to the offer of its verb nearest its faces (`'for'`), within `reach_m` (`onto_m` for
    a seat) so an unidentified seat's dock never steals another's balloon."""
    out, taken = [], set()
    for d in docks:
        if not d.get('at'):
            continue
        mx = sum(float(p[0]) for p in d['at']) / len(d['at'])
        mz = sum(float(p[2]) for p in d['at']) / len(d['at'])
        span = onto_m if d.get('kind') == 'sit_chair' else reach_m
        pts = [(mx, 0.0, mz)] + list(d['at'])
        near = [(min(math.hypot(float(p[0]) - float(o[3][0]), float(p[2]) - float(o[3][2])) for p in pts), i)
                for i, o in enumerate(on) if i not in taken and o[1] == d.get('kind')]
        if near and min(near)[0] <= span:
            i = min(near)[1]
            taken.add(i)
            out.append(dict(d, **{'for': i}))
    return out
