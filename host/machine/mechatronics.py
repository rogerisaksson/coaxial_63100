"""Mechatronics as data: the parts library (motors, gearboxes, inverters) and a robot's build of it.

    lib = mechatronics.library()            # {'motors': .., 'gearboxes': .., 'inverters': ..}
    build = mechatronics.robot(ROBOT)       # machine/robots/<name>.toml, its parts the library's

A robot's file names, for each joint kind, its stack's motor, gearbox and inverter from the
library, the servo its board runs, the structure between its gearbox and its limb, and what has
no drive; `machine.drives` and `machine.servos` read it, `tools/cores/manifest.py` writes its
orthokinematic manifest of it.
"""
import os
import tomllib

from machine.errors import MachineError

HERE = os.path.dirname(os.path.abspath(__file__))
LIBRARY, ROBOTS = os.path.join(HERE, 'library'), os.path.join(HERE, 'robots')
#: The robot this tree builds.
ROBOT = 'gynoid_mesomorph_medium'
#: Each part's file in the library, and what a stack names of it in turn.
PARTS = ('motors', 'gearboxes', 'inverters')

_READ = {}


def _toml(path):
    if path not in _READ:
        with open(path, 'rb') as f:
            _READ[path] = tomllib.load(f)
    return _READ[path]


def library():
    """{part: {name: its data}} of every part in the library."""
    return {part: _toml(os.path.join(LIBRARY, part + '.toml')) for part in PARTS}


def robot(name=ROBOT):
    """A robot's build (machine/robots/`name`.toml), each stack's parts found in the library."""
    path = os.path.join(ROBOTS, name + '.toml')
    if not os.path.isfile(path):
        raise MachineError('no robot %s: %s' % (name, ', '.join(sorted(
            f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml')))))
    build, lib = _toml(path), library()
    for kind, stack in build['stacks'].items():
        for part, named in zip(PARTS, stack):
            if named not in lib[part]:
                raise MachineError('%s: %s %s is no %s in the library (%s)' % (
                    name, kind, named, part[:-1], ', '.join(lib[part])))
    return build
