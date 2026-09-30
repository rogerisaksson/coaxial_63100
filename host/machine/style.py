"""Her walk's style as knobs: the constants its plan and walker read, trimmed while she walks.

    style.set('turn', 4.0)        # gait.TURN_DEG; the plan's tables built again from it
    style.state()                 # {name: value} of every knob
    style.trim('turn', +1)        # a step up, within its bounds

The HUMANOID page trims them (`running`'s 'style' command), a controller the same way; each is a
constant of the module that owns it, so `look.py NAME=V` and the Monte Carlo move them too.
"""
from machine import gait, walker, walkplan

#: (name, module, constant, step, low, high, unit): what the page shows and trims. The pelvis's
#: turn and drop and its planned shift over the standing foot (`gait`), the torso's share of the
#: turn taken back (`gait.COUNTER`), the shoulders held over the line against the hips' sway
#: (`walker.SHOULDERS_BACK`).
KNOBS = (('turn', gait, 'TURN_DEG', 0.5, 0.0, 12.0, 'deg'),
         ('drop', gait, 'ROLL_DEG', 0.5, 0.0, 12.0, 'deg'),
         ('shift', gait, 'SHIFT_M', 0.002, 0.0, 0.03, 'm'),
         ('counter', gait, 'COUNTER', 0.05, 0.0, 1.2, ''),
         ('shoulders', walker, 'SHOULDERS_BACK', 0.025, 0.0, 0.35, ''))
NAMES = tuple(k[0] for k in KNOBS)
_BY = {k[0]: k for k in KNOBS}


def value(name):
    _n, module, constant = _BY[name][:3]
    return float(getattr(module, constant))


def unit(name):
    return _BY[name][6]


def set(name, v):
    """Knob `name` at `v` within its bounds; the walk's plan eased over to it (`walkplan.retable`)."""
    _n, module, constant, _step, low, high, _unit = _BY[name]
    setattr(module, constant, max(low, min(high, float(v))))
    walkplan.retable()
    return value(name)


def trim(name, steps):
    """Knob `name` moved `steps` of its step."""
    return set(name, value(name) + steps * _BY[name][3])


def state():
    """{name: value} of every knob."""
    return {name: value(name) for name in NAMES}
