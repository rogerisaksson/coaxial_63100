"""The machine's constants as knobs: set where they live, read where they live.

    knobs.set_({'SOFT_KNEE': 6.0, 'arrival.LIFT_UP_M': 0.04, 'drives.WAYS.head': 2.0})
    knobs.now('arrival.FIRST')

A name is a constant's, in the first of MODULES holding it, or its module's and its own
(walkplan.TRACK_M: gait has its own), or a table's entry's as module.TABLE.key. Unqualified, a
name two modules hold sets the first: TURN_DEG meant for the fall turned the walk's pelvis
(2026-10-01). The gait's fits and the plan's tables are cleared on every set.
"""
import importlib

MODULES = ('walker', 'gait', 'walkplan', 'landing', 'stance', 'arrival', 'director', 'falls', 'parry', 'dcm', 'bearing',
           'getup', 'observer',
           'capture', 'physics', 'mjcf', 'build', 'buses', 'events', 'drives', 'stand', 'floor',
           'figure')


def _modules():
    return [importlib.import_module('machine.' + m) for m in MODULES]


def set_(values):
    """{name: value} set where each lives (the module's brief)."""
    mods = _modules()
    for name, value in values.items():
        module, _dot, name = name.partition('.') if '.' in name else ('', '', name)
        table, _dot, key = name.partition('.')
        owner = next((m for m in mods if hasattr(m, table)
                      and (not module or m.__name__ == 'machine.' + module)), None)
        if owner is None:
            raise KeyError('no %s in machine.%s' % (table, ', machine.'.join(MODULES)))
        if key:
            getattr(owner, table)[key] = value
        else:
            setattr(owner, table, value)
    gait, walkplan = mods[1], mods[2]
    gait._FITS.clear()
    walkplan._TABLES.clear()


def now(name):
    """A constant's value where it lives, as a float."""
    mods = _modules()
    parts = name.split('.')
    key = parts.pop() if len(parts) > 2 else None
    module, _dot, bare = '.'.join(parts).rpartition('.')
    owner = next(m for m in mods if hasattr(m, bare)
                 and (not module or m.__name__ == 'machine.' + module))
    value = getattr(owner, bare)
    return float(value[key] if key else value)
