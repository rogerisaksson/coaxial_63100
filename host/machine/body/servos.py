"""Her drives' servos by their joints' kind: each one's peak, stiffness, damping and armature.

    from machine.body.servos import CLAMPED, SERVO      # SERVO[kind] = (N m, N m/rad, N m s/rad, kg m^2)

Derived by `machine.body.compile` from her subtree inertias, gravity levers and actuator stacks.
"""
from machine.body import compile, mechatronics

_BUILD = mechatronics.robot()
#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2).
SERVO = compile.servos(_BUILD.get('servo'))
#: Whether a joint's clamp is its drive's peak where that is less (`drives.peak`), 0 or 1.
CLAMPED = float(_BUILD.get('clamped', 1.0))
