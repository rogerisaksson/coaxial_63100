#!/usr/bin/env python3
"""Her axes by their drive's type, as docs/DIMENSIONS.md has them.

    python tools/sim/dimensions.py            # the page, printed
    python tools/sim/dimensions.py --write    # docs/DIMENSIONS.md written

A type is a stack (`drives.STACKS`): its motor, its gearbox, its inverter; under it every axis it
turns. The page is this tool's output: a candidate laid in `machine/drives.py` and written again
is the page edited (tests/test_mirrors.py holds the two together).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

PAGE = os.path.join(REPO, 'docs', 'DIMENSIONS.md')


def _joint(kind):
    """A joint of `kind`: its left one where it has sides."""
    from machine.figure import JOINTS
    return kind if kind in JOINTS else 'left_' + kind


def types():
    """[(name, stack, [kind, ..])] of the driven kinds, the type turning the most first: a type
    its frame's letter, its box's and its inverter's after it where another type shares it."""
    from machine import drives
    by = {}
    for kind, stack in drives.STACKS.items():
        if not drives.passive(_joint(kind)):
            by.setdefault(stack, []).append(kind)
    frames = [stack[0] for stack in by]
    return sorted(((stack[0] if frames.count(stack[0]) == 1 else '%s/%s/%d' % stack, stack, kinds)
                   for stack, kinds in by.items()),
                  key=lambda row: -max(drives.peak(_joint(k)) for k in row[2]))


def _through(kind):
    from machine import linkage
    other = linkage.PAIRS.get(kind) or next((k for k, v in linkage.PAIRS.items() if v == kind), None)
    if other:
        return 'a rod, a pair with the %s' % other.replace('_', ' ')
    if kind in linkage.PLANAR:
        return 'a four-bar'
    if kind in linkage.GEARS:
        return 'a spur pair 1:%.2f' % linkage.GEARS[kind]
    return '-'


def _sits(kind):
    from machine import drives
    where = drives.JOINTS[kind]
    board = drives.BOARDS.get(kind)
    return ('on its axis' if where is None else 'on the %s' % where[0].replace('_', ' ')) + (
        ', its inverter on the %s' % board[0].replace('_', ' ') if board else '')


def text():
    """The page."""
    from machine import drives, physics
    from machine.figure import JOINTS
    from tools.sim import drive_sizes
    rows = types()
    driven = [j for j in JOINTS if not drives.passive(j)]
    out = ['# Dimensions', '',
           'Her axes by their drive\'s type: %d driven on %d type%s, %d without a drive. The'
           % (len(driven), len(rows), '' if len(rows) == 1 else 's', len(JOINTS) - len(driven)),
           'candidate of record, laid in `machine/drives.py` and written by',
           '`tools/sim/dimensions.py --write`; why it is this one, docs/findings/stacks.md.', '',
           '## Types', '',
           '| type | drives | motor | gearbox | inverter | stack mm | kg | peak N m | holds N m '
           '| deg/s | rotor kg m2 |',
           '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name, (frame, box, inverter), kinds in rows:
        size = drives.size(kinds[0])
        rotor, stack = drives.FRAMES[frame]
        count = sum(1 for j in driven if drives.kind(j) in kinds)
        # one on its own axis, alone on its joint: the type's numbers at its gearbox's output
        plain = next((k for k in kinds if drives.ratio(_joint(k)) == drives.RATIO
                      and drives.motors(_joint(k)) == 1), kinds[0])
        lever = drives.ratio(_joint(plain)) / drives.RATIO * drives.motors(_joint(plain))
        long = sum(length for _part, _r, length in size.parts) + (
            0.010 if plain in drives.BOARDS else 0.0)
        out.append('| %s | %d | %.0f x %.0f mm, KV %.0f | %.0f mm, 1:%.0f | %d mm, %.0f A | '
                   '%.0f x %.0f | %.2f | %.0f | %.0f | %.0f | %.3f |' % (
                       name, count, rotor * 1e3, stack * 1e3, size.kv, drives.BOXES[box] * 1e3,
                       drives.RATIO, inverter, size.amps,
                       max(size.diameter, 2e3 * size.board[1] / 1e3) * 1e3, long * 1e3,
                       size.mass + size.board[0], drives.peak(_joint(plain)) / lever,
                       drive_sizes.held(_joint(plain))[0] / lever,
                       size.kv * drives.PACK_V * 6.0 / drives.RATIO,
                       (1.0 + drives.GEAR_J) * size.rotor * drives.RATIO ** 2))
    out += ['',
            "A stack its inverter's disc, its outrunner and its gearbox on one axis; kg",
            "with the inverter. Peak at the gearbox's output at the inverter's amps;",
            'holds, for ever in the oil (`drives.COOLING` %.1f, assumed); deg/s unloaded'
            % drives.COOLING,
            "at the pack's lowest, %.0f V; the rotor as its joint feels it through the"
            % drives.PACK_V, 'box.', '', '## Axes']
    for name, _stack, kinds in rows:
        out += ['', '### Type %s' % name, '',
                '| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |',
                '| --- | --- | --- | --- | --- | --- | --- | --- |']
        for kind in sorted(kinds, key=lambda k: -drives.peak(_joint(k))):
            j = _joint(kind)
            clamp = min(physics.SERVO[kind][0], physics.CLAMPED * drives.peak(j))
            out.append('| %s | %d | %s | %s | %.1f | %.0f | %.0f | %.0f |' % (
                kind.replace('_', ' '), sum(1 for d in driven if drives.kind(d) == kind),
                _sits(kind), _through(kind), drives.ratio(j), drives.peak(j), clamp,
                drives.speed(j)))
    bare = sorted({drives.kind(j) for j in JOINTS if drives.passive(j)})
    out += ['', '### No drive', '', '| axis | joints | held by |', '| --- | --- | --- |']
    for kind in bare:
        stiff, damp, rest = drives.passive(_joint(kind)) or (None, 0.0, 0.0)
        out.append('| %s | %d | %s |' % (
            kind.replace('_', ' ').replace('foot', 'toes'),
            sum(1 for j in JOINTS if drives.kind(j) == kind),
            'a spring, %.0f N m/rad about %.0f deg' % (stiff, rest) if stiff is not None
            else 'a stop, at %.0f deg' % rest))
    return '\n'.join(out) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--write', action='store_true', help='docs/DIMENSIONS.md written')
    args = parser.parse_args(argv)
    page = text()
    if args.write:
        with open(PAGE, 'w', encoding='utf-8', newline='\n') as f:
            f.write(page)
    else:
        sys.stdout.write(page)
    return 0


if __name__ == '__main__':
    sys.exit(main())
