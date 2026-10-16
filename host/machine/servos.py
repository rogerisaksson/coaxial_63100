"""Her drives' servos by their joints' kind: each one's peak, stiffness, damping and armature.

    from machine.servos import CLAMPED, SERVO      # SERVO[kind] = (N m, N m/rad, N m s/rad, kg m^2)

Her build's file holds them (machine/robots, `machine.mechatronics`).
"""
from machine import mechatronics

_BUILD = mechatronics.robot()
#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2).
SERVO = {kind: tuple(servo) for kind, servo in _BUILD['servo'].items()}
#: Whether a joint's clamp is its drive's peak where that is less (`drives.peak`), 0 or 1.
CLAMPED = _BUILD['clamped']
