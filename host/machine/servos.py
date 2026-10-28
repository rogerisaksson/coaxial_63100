"""Her drives' servos by their joints' kind: each one's peak, stiffness, damping and armature.

    from machine.servos import CLAMPED, SERVO      # SERVO[kind] = (N m, N m/rad, N m s/rad, kg m^2)

Her build's file holds them (machine/humanoids, `machine.mechatronics`).
"""
import math

from machine import drives, mechatronics
from machine.figure import JOINTS

_BUILD = mechatronics.robot()
#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2); a peak said nan its
#: stack's (`drives.peak`), its strength said once.
SERVO = {kind: (drives.peak(next(j for j in JOINTS if drives.kind(j) == kind)) if math.isnan(servo[0]) else servo[0],
                *servo[1:]) for kind, servo in _BUILD['servo'].items()}
#: Whether a joint's clamp is its drive's peak where that is less (`drives.peak`), 0 or 1.
CLAMPED = _BUILD['clamped']
