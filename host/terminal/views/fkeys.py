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
