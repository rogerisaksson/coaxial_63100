"""Her walk's style as knobs: the constants its plan and walker read, trimmed while she walks.

    style.set('turn', 4.0)        # gait.TURN_DEG; the plan's tables built again from it
    style.state()                 # {name: value} of every knob
    style.trim('turn', +1)        # a step up, within its bounds
    style.sway(-0.5)              # every knob half way to the catwalk (`SWAY`)

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

#: The walk along one axis, -1 to 1 (`sway`): a catwalk's pelvis turning and dropping more at -1,
#: the walk as tuned at 0, a swagger's torso counter-turning and shoulders loose at 1 - each knob
#: on a line through its three. Both ends walk at 0.65 and 0.85 strides/s, their drives at their
#: peaks 0.34-0.49 % of the time where the walk as tuned is 0.47; the catwalk's feet on a 15 mm
#: track fell (2026-09-30).
SWAY = {'turn': (8.0, 4.0, 5.0), 'drop': (7.0, 6.0, 4.0), 'shift': (0.014, 0.010, 0.008),
        'counter': (1.0, 1.0, 1.2), 'shoulders': (0.30, 0.25, 0.15)}
#: Where on it she walks.
AXIS = 0.0


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


def sway(s):
    """Every knob at `s` on the axis (`SWAY`), within bounds; the plan eased over to them once."""
    global AXIS
    AXIS = max(-1.0, min(1.0, float(s)))
    for name, (catwalk, tuned, swagger) in SWAY.items():
        _n, module, constant, _step, low, high, _unit = _BY[name]
        v = tuned + ((tuned - catwalk) * AXIS if AXIS < 0.0 else (swagger - tuned) * AXIS)
        setattr(module, constant, max(low, min(high, v)))
    walkplan.retable()
    return AXIS
