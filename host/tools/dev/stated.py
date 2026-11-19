#!/usr/bin/env python3
"""The specs' stated values, counted, and what the static solver derives from them, fingerprinted.

    python tools/dev/stated.py            # each spec file's stated values, its tables by size
    python tools/dev/stated.py --json     # ... as JSON
    python tools/dev/stated.py --solved   # every machine's solved parts, a SHA-256 each

A spec states construction alone - what a machine is built of - and the static solver derives the rest
(`machine.body.mechatronics`, `machine.body.compile`, `machine.aircraft.compile`): every value a spec states -
a number, a name, each member of a list - is an input someone must know to configure a robot, a quad or an
aircraft. CEILING holds each file's count, held by test_metacompiler: a value more is an input the solver was
to derive; a value fewer lowers its ceiling with it, so the count only falls.

An input that goes leaves what is solved as it was: `--solved` at the tree before and at the tree after says
the same lines - each humanoid build's subtrees, servos, gait, stacks, DC link, model and peripherals (a
process a build: ROBOT is a process's) and the crafts' blocks and parts, equal texts for equal bits.
"""
import argparse
import glob
import hashlib
import json
import os
import subprocess
import sys
import tomllib

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: Where the specs lie, from the host folder.
SPECS = ('machine/library/*.toml', 'machine/humanoids/*.toml', 'machine/aircraft/*.toml')
#: The values each spec file states, at most: its count as the solver stands. The robot's rose by 71 with its
#: guards (`[guards]`): new construction - what its limbs and its trunk are built of in place of its shells -,
#: no value the solver could derive.
CEILING = {
    'machine/aircraft/crafts.toml': 102,
    'machine/humanoids/gynoid_mesomorph_medium.toml': 1064,
    # 124: the coaxial calf - new construction (its drives' places, parts, clamped levers, heel bar and its strut,
    # struts, the foot's keel, the thigh's leant run, the knee's stop, its guards: the arm's on its back, the
    # shin's alone on the calf; every inverter of the leg on its motor's back); its rods' arms level at rest - each
    # ball's arm stated beside its crank's, the heel's bar a straight shaft by its two ends; its two rods alike,
    # each a printed cartridge of leaves: their material, their count, a leaf's width and thickness, their bow and
    # their length
    'machine/humanoids/gynoid_mesomorph_medium_coax.toml': 124,
    'machine/humanoids/gynoid_mesomorph_medium_packs.toml': 2,
    'machine/humanoids/gynoid_mesomorph_medium_performance.toml': 9,
    'machine/humanoids/gynoid_mesomorph_medium_r80.toml': 3,
    'machine/humanoids/gynoid_mesomorph_medium_s105.toml': 2,
    'machine/humanoids/gynoid_mesomorph_medium_s95.toml': 2,
    'machine/library/control.toml': 7,
    # 20: B45, the 45-lobe ring the coaxial calf's small drives turn through - a part bought, not derived
    'machine/library/gearboxes.toml': 20,
    'machine/library/inverters.toml': 40,
    # 64: the bus's robot hose ([hoses]: its radius, its weight, its two stiffnesses, its least bend, its damping)
    # and a board's leads ([leads]: a phase's, a bus's, a signal's - each its copper and its outside) - parts
    # bought, no other value tells
    'machine/library/materials.toml': 64,
    'machine/library/motors.toml': 64,
    # 88: the friction's Stribeck speed (stribeck_m_s), a property of a sole on a floor no other value tells
    'machine/library/world.toml': 88,
}
#: The peripherals the solver derives of a humanoid build (`machine.body.peripherals`), each by its name.
PERIPHERALS = ('bus', 'thermal', 'cooling', 'harness', 'eyes', 'world', 'service', 'identify')
#: The crafts' compiled parts (`machine.aircraft.compile.part`), each by its name.
PARTS = ('pack', 'pull', 'aerokin', 'wingborne', 'aerobat', 'mpc', 'gimbal', 'instruments')
#: What a process of a build's own is asked: its solved parts' fingerprints as one line of JSON.
ASKED = 'import json; from tools.dev import stated; print(json.dumps(stated.build()))'


def leaves(value):
    """The values a table states: a table's and a list's are their members', anything else one."""
    if isinstance(value, dict):
        return sum(leaves(v) for v in value.values())
    if isinstance(value, list):
        return sum(leaves(v) for v in value)
    return 1


def counted():
    """{spec file: (its stated values, {table: its values})}, each file by its path from the host folder."""
    out = {}
    for pattern in SPECS:
        for path in sorted(glob.glob(os.path.join(HOST, *pattern.split('/')))):
            with open(path, 'rb') as f:
                spec = tomllib.load(f)
            out[os.path.relpath(path, HOST).replace(os.sep, '/')] = (leaves(spec), {k: leaves(v) for k, v in spec.items()})
    return out


def _sha(value):
    """A solved value's fingerprint: its JSON's SHA-256, its keys in order, an array as its list."""
    text = json.dumps(value, sort_keys=True, default=lambda o: o.tolist() if hasattr(o, 'tolist') else repr(o))
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def build():
    """{part: fingerprint} of what the solver derives of this process's build (`mechatronics.ROBOT`)."""
    from machine.body import compile as body, dclink, drives, mjcf, peripherals
    whole = {'subtrees': body.subtrees(), 'servos': body.servos(), 'gait': body.gait(), 'stacks': drives.STACKS,
             'link': [dclink.PACK_OHM, dclink.CHARGE_A, dclink.LINK_F, dclink.CLAMP_V, dclink.ONSET_V],
             'model': mjcf.mjcf()}
    whole.update({name: getattr(peripherals, name)() for name in PERIPHERALS})
    return {part: _sha(value) for part, value in whole.items()}


def crafts():
    """{part: fingerprint} of the crafts' compiled blocks, parts, weather and defaults."""
    from machine.aircraft import compile as craft
    whole = {'blocks': craft.blocks(), 'weather': craft.weather(), 'defaults': craft.defaults()}
    whole.update({name: craft.part(name) for name in PARTS})
    return {part: _sha(value) for part, value in whole.items()}


def solved():
    """{machine: {part: fingerprint}}: every humanoid build's, a process each, and the crafts'."""
    from machine.body import mechatronics
    out = {}
    for name in mechatronics.robots():
        got = subprocess.run([sys.executable, '-X', 'utf8', '-c', ASKED], cwd=HOST, check=True, capture_output=True,
                             text=True, env=dict(os.environ, COAXIAL_ROBOT=name, PYTHONPATH=HOST))
        out[name] = json.loads(got.stdout.splitlines()[-1])
    out['crafts'] = crafts()
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--json', action='store_true', help='the counts, or with --solved the fingerprints, as JSON')
    p.add_argument('--solved', action='store_true', help="every machine's solved parts, a SHA-256 each")
    args = p.parse_args(argv)
    got = solved() if args.solved else counted()
    if args.json:
        print(json.dumps(got))
        return 0
    if args.solved:
        for machine, parts in got.items():
            for part, sha in parts.items():
                print('%s  %-11s %s' % (sha[:16], part, machine))
        return 0
    for path, (total, tables) in got.items():
        print('%4d of %4d  %-58s %s' % (total, CEILING.get(path, 0), path, ', '.join(
            '%s %d' % item for item in sorted(tables.items(), key=lambda item: -item[1])[:8])))
    print('%4d of %4d  values stated in %d files' % (
        sum(total for total, _tables in got.values()), sum(CEILING.values()), len(got)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
