#!/usr/bin/env python3
"""How many values the specs state: the static solver's inputs, a file and a table at a time.

    python tools/dev/stated.py           # each spec file's stated values, its tables by size
    python tools/dev/stated.py --json    # ... as JSON

A spec states construction alone - what a machine is built of - and the static solver derives the rest
(`machine.body.mechatronics`, `machine.body.compile`, `machine.aircraft.compile`): every value a spec states is
an input someone must know to configure a robot, a quad or an aircraft. CEILING holds each file's count, held by
test_metacompiler: a value more is an input the solver was to derive; a value fewer lowers its ceiling with it,
so the count only falls.
"""
import argparse
import glob
import json
import os
import sys
import tomllib

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
#: Where the specs lie, from the host folder.
SPECS = ('machine/library/*.toml', 'machine/humanoids/*.toml', 'machine/aircraft/*.toml')
#: The values each spec file states, at most: its count as the solver stands.
CEILING = {
    'machine/aircraft/crafts.toml': 91,
    'machine/humanoids/gynoid_mesomorph_medium.toml': 339,
    'machine/humanoids/gynoid_mesomorph_medium_packs.toml': 2,
    'machine/humanoids/gynoid_mesomorph_medium_performance.toml': 5,
    'machine/humanoids/gynoid_mesomorph_medium_r80.toml': 3,
    'machine/humanoids/gynoid_mesomorph_medium_s105.toml': 2,
    'machine/humanoids/gynoid_mesomorph_medium_s95.toml': 2,
    'machine/library/gearboxes.toml': 19,
    'machine/library/inverters.toml': 37,
    'machine/library/materials.toml': 17,
    'machine/library/motors.toml': 74,
    'machine/library/world.toml': 43,
}


def leaves(value):
    """The values a table states: its own and its tables'; a value that is no table, one."""
    return sum(leaves(v) for v in value.values()) if isinstance(value, dict) else 1


def counted():
    """{spec file: (its stated values, {table: its values})}, each file by its path from the host folder."""
    out = {}
    for pattern in SPECS:
        for path in sorted(glob.glob(os.path.join(HOST, *pattern.split('/')))):
            with open(path, 'rb') as f:
                spec = tomllib.load(f)
            out[os.path.relpath(path, HOST).replace(os.sep, '/')] = (leaves(spec), {k: leaves(v) for k, v in spec.items()})
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--json', action='store_true', help='the counts as JSON')
    args = p.parse_args(argv)
    got = counted()
    if args.json:
        print(json.dumps(got))
        return 0
    for path, (total, tables) in got.items():
        print('%4d of %4d  %-58s %s' % (total, CEILING.get(path, 0), path, ', '.join(
            '%s %d' % item for item in sorted(tables.items(), key=lambda item: -item[1])[:8])))
    print('%4d of %4d  values stated in %d files' % (
        sum(total for total, _tables in got.values()), sum(CEILING.values()), len(got)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
