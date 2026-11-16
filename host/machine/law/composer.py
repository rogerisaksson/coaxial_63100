"""The robot's law's pass composed of its parts at work: each run or rested by its activity, no switch.

    @composer.capability(works, rest)          # on a part in its owner's module: its activity, its rest
    def step(g, ...): ...                      # its run, the part as it always was

The kernel-on-demand gating of `working_envelope_scoper.lisp` on the robot's law: a part's activity reads its drivers
and its own state off their rest; at rest the part is its rest - what the part gives there, the same bits, at a
fraction of its cost, its owner's proof beside it -, so the robot's pass's solver is built of the parts at work, each
picked by index (`parts[works]`), handed over the pass its activity leaves rest and back the pass it reaches it.
"""
import functools

#: Every part composed, by its owner's name and its own.
LAID: dict = {}


def capability(works, rest, g=0):
    """A part composed by its activity `works` (its arguments -> whether it works) over its rest `rest` (its arguments
    -> what the part gives at rest, the same bits); the robot's session `g` its argument at that place, the robot's
    working set kept on it (`working`)."""
    def laid(run):
        parts, name = (rest, run), '%s.%s' % (run.__module__.rsplit('.', 1)[-1], run.__name__)

        @functools.wraps(run)
        def composed(*a):
            at = bool(works(*a))
            a[g].setdefault('working', {})[name] = int(at)
            return parts[at](*a)
        LAID[name] = composed
        return composed
    return laid
