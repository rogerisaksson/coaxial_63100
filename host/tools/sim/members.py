#!/usr/bin/env python3
"""Her members under their drives' peaks: each tube's, plate's and rod's stress and flex.

    python tools/sim/members.py             # every member, the ones past a limit starred
    python tools/sim/members.py --family 3  # her tubes cut from three stock sizes

A member is a run of her skeleton (`machine.body.skeleton`: the bones, the trunk's frame, the boom,
the hip's gimbal, the holders' struts) or a linkage's part (`machine.body.linkage`: a crank, a horn, a
rod). Its load is the peak its drive delivers (`drives.peak`, through the lever) as a bending
moment, a twist, or a rod's pull on a crank's arm; its section a carbon tube (WALL of its radius),
a printed PAHT-CF plate loaded in its plane, or a solid carbon rod. Each is judged on stress
against its material's allowable and on flex over its length at FLEX_AT of the load against
FLEX_DEG (docs/findings/body.md: 0.25 deg a member at 0.6). No safety factor beyond ALLOW_MPA.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine.body import structural
from machine.body.structural import (CARBON, CF_KG_M3, FLEX_AT, FLEX_DEG, HM_CARBON, MARGIN, ROOM,
                                     TUBES, judge, tube_section, wind)


def members():
    """[(name, kind, length m, load, (I, J, A), material, c)] loaded at each drive's bounded peak."""
    from machine.body import drives
    from machine.body.servos import SERVO
    peak = {k: min(drives.peak(k if k in ('spine', 'spine_roll', 'waist', 'neck', 'head')
                               else 'left_' + k), MARGIN * SERVO[k][0]) for k in drives.STACKS}
    return structural.members(peak)


def tubes(rows):
    """The members that are carbon tubes (a round section, carbon), each (row, judged)."""
    return [(m, j) for m, j in rows if m[5] in (CARBON, HM_CARBON)
            and math.isclose(m[4][1], 2.0 * m[4][0])]


def _passes(row, r, t):
    name, kind, length, load, _s, material, _c = row
    stress, flex = judge(name, kind, length, load, tube_section(r, t), material, r)
    return stress <= 1.0 and flex <= FLEX_DEG


def spans(rows):
    """[(name, length, (first, last) of TUBES passing it, or None)] a tube member: the stock sizes
    passing `judge` no more than ROOM over its radius now - a bigger one passes too."""
    out = []
    for row, _j in tubes(rows):
        ok = [k for k, (r, t) in enumerate(TUBES) if r <= row[6] + ROOM and _passes(row, r, t)]
        out.append((row[0], row[2], (min(ok), max(ok)) if ok else None))
    return out


def families(served, most=6):
    """[(TUBES' indices, kg)] the lightest family of 1..`most` stock sizes cutting every member
    of `served` [(name, length, (first, last))] - the smallest of the family within its span -,
    None where none can."""
    import itertools
    out = []
    for n in range(1, most + 1):
        best = None
        for family in itertools.combinations(range(len(TUBES)), n):
            kg = 0.0
            for _name, length, (first, last) in served:
                k = next((k for k in family if first <= k <= last), None)
                if k is None:
                    break
                kg += CF_KG_M3 * tube_section(*TUBES[k])[2] * length
            else:
                best = (family, kg) if best is None or kg < best[1] else best
        out.append(best)
    return out


def main(argv=None):
    import argparse
    from machine.body import drives
    from machine.body.servos import SERVO
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--family', type=int, default=0, metavar='N',
                        help='the tubes cut from the lightest family of N stock sizes')
    args = parser.parse_args(argv)
    rows = [(m, judge(*m)) for m in members()]
    if args.family:
        served = [s for s in spans(rows) if s[2] is not None]
        for name, _length, span in spans(rows):
            if span is None:
                print('%-20s no stock size within %.0f mm of its radius' % (name, 1e3 * ROOM))
        now = sum(CF_KG_M3 * m[4][2] * m[2] for m, _j in tubes(rows))
        print('%d tube members, %.2f kg as they are; the lightest family a count of sizes:'
              % (len(served), now))
        found = families(served, args.family)
        for n, best in enumerate(found, 1):
            print('  %d  %-32s %s' % (n, ' '.join('%gx%g' % (2e3 * r, 1e3 * t) for r, t in (
                TUBES[k] for k in best[0])) if best else '-', '%.2f kg' % best[1] if best else ''))
        if found[-1]:
            for name, _length, (first, last) in served:
                k = next(k for k in found[-1][0] if first <= k <= last)
                print('  %-20s %gx%g' % (name, 2e3 * TUBES[k][0], 1e3 * TUBES[k][1]))
        return 0
    print('%-22s %-6s %5s %7s %9s | %6s %6s' % ('member', 'load', 'mm', 'N m / N', 'section',
                                                 'stress', 'flex'))
    for (name, kind, length, load, section, material, c), (stress, flex) in rows:
        bad = stress > 1.0 or flex > FLEX_DEG
        print('%s %-20s %-6s %5.0f %7.0f %9s | %6.2f %6.2f%s' % (
            '*' if bad else ' ', name, kind, 1e3 * length, load,
            'r %.0f' % (1e3 * c) if kind != 'pull' else 'rod', stress, flex,
            ' deg' if kind != 'pull' else ''))
    print('%d of %d past a limit (stress 1, flex %.2f deg at %.1f of the peak)' % (
        sum(s > 1.0 or f > FLEX_DEG for _m, (s, f) in rows), len(rows), FLEX_DEG, FLEX_AT))
    print('wind-up a joint, its structure then its gearbox, mrad a N m, and deg at its clamp:')
    for kind, w in wind(rows).items():
        box = 1.0 / drives.BOX_K[drives.STACKS[kind][1]]
        clamp = min(drives.peak(kind if kind in ('spine', 'spine_roll', 'waist', 'neck', 'head')
                                else 'left_' + kind), SERVO[kind][0])
        print('  %-11s %6.3f + %5.3f mrad/N m  %5.2f deg at %3.0f N m' % (
            kind, 1e3 * w, 1e3 * box, math.degrees((w + box) * clamp), clamp))
    return 0


if __name__ == '__main__':
    sys.exit(main())
