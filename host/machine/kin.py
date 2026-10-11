"""Her kinematic language. A word is lines of asks, each held till it is met, compiled to rows
the one law's steering reads: a word is text, never code.

    out:
        hand latch at door
        go door-1.8
        go door
        go door+1.0
        pace walk

    rows = kin.compiled(TEXT)['out']        # ((verb, place, metres, amount, name), ..)
    bound = kin.bound(rows, frames)         # ((verb, x, z, yaw, amount, side, name), ..)

A line is a verb and what it asks:

    go PLACE            her way walked there (`machine.pursuit`), round what she knows
    face PLACE          standing, her heading turned to the place's own
    stand               the stand's row, till her stand wants no step
    pace ROW            the row of her way asked - stand, walk, jog, run or its number
    hand LIMB at PLACE  a hand raised before her while she is near the place
    reach LIMB          an arm raised till what is before it is touched
    play NAME           the arrival's keyframes of that name

A PLACE is a frame's name - where a thing is used from: x, z, the heading held there, the side
its latch is on - and, signed, metres along that heading from it: `door-1.8` is 1.8 m before
the door, `door+1.0` a metre beyond. A LIMB is `left` or `right`, or a frame's own `latch` and
`hinge`. Lines of `go` one after another are one way, walked through; what follows them says
whether she stands at its end. What a verb asks of her balance is the law's (`machine.going`):
a word names no leg and no step.
"""
import math
import re

GO, FACE, STAND, PACE, HAND, REACH, PLAY = range(7)
VERBS = {'go': GO, 'face': FACE, 'stand': STAND, 'pace': PACE, 'hand': HAND, 'reach': REACH,
         'play': PLAY}
#: The rows of her way by name (`machine.gaits.between`).
ROWS = {'stand': -1.0, 'walk': 0.0, 'jog': 0.5, 'run': 1.0}
_PLACE = re.compile(r'^([a-z_.]+?)([+-]\d+(?:\.\d+)?)?$')


def _place(token, n):
    found = _PLACE.match(token)
    if not found:
        raise SyntaxError('line %d: %r is no place - a frame, and metres along it' % (n, token))
    return found.group(1), float(found.group(2) or 0.0)


def _row(tokens, n):
    """One line's row (verb, place, metres, amount, name)."""
    verb, rest = tokens[0], tokens[1:]
    shapes = {'go': 1, 'face': 1, 'stand': 0, 'pace': 1, 'hand': 3, 'reach': 1, 'play': 1}
    if verb not in VERBS or len(rest) != shapes[verb] or (verb == 'hand' and rest[1] != 'at'):
        raise SyntaxError('line %d: %r - the verbs are %s' % (n, ' '.join(tokens), ', '.join(VERBS)))
    if verb in ('go', 'face'):
        return (VERBS[verb],) + _place(rest[0], n) + (0.0, '')
    if verb == 'pace':
        if rest[0] not in ROWS and not re.match(r'^-?\d+(\.\d+)?$', rest[0]):
            raise SyntaxError('line %d: no row %r - %s, or its number' % (n, rest[0], ', '.join(ROWS)))
        return (PACE, '', 0.0, ROWS[rest[0]] if rest[0] in ROWS else float(rest[0]), '')
    if verb == 'hand':
        return (HAND,) + _place(rest[2], n) + (0.0, rest[0])
    return (VERBS[verb], '', 0.0, 0.0, rest[0] if rest else '')


def compiled(text):
    """{word: rows} of `text`: a word a line ending ':' at the margin, its lines indented under
    it, '#' to a line's end a remark."""
    words, rows = {}, None
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split('#', 1)[0].rstrip()
        if not line.strip():
            continue
        if not line[0].isspace():
            if not line.endswith(':'):
                raise SyntaxError('line %d: a word ends with a colon: %r' % (n, line))
            rows = words[line[:-1].strip()] = []
        elif rows is None:
            raise SyntaxError('line %d: a line under no word: %r' % (n, line))
        else:
            rows.append(_row(line.split(), n))
    return {word: tuple(lines) for word, lines in words.items()}


def bound(rows, frames):
    """`rows` with each place a point and a heading and each limb a side: ((verb, x, z, yaw,
    amount, side, name), ..), side 1 her left, -1 her right. `frames` {name: (x, z, yaw,
    latch's side)}."""
    out = []
    for verb, place, metres, amount, name in rows:
        x = z = yaw = side = 0.0
        if place:
            if place not in frames:
                raise KeyError('no frame %r: %s' % (place, ', '.join(sorted(frames))))
            fx, fz, yaw, latch = frames[place]
            x, z = fx + metres * math.sin(yaw), fz + metres * math.cos(yaw)
            side = {'left': 1.0, 'right': -1.0, 'latch': latch, 'hinge': -latch}.get(name, 0.0)
        elif verb == REACH:
            side = {'left': 1.0, 'right': -1.0}[name]
        out.append((verb, x, z, yaw, amount, side, name))
    return tuple(out)

