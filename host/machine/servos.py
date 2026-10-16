"""Her drives' servos by their joints' kind: each one's peak, stiffness, damping and armature.

    from machine.servos import CLAMPED, SERVO      # SERVO[kind] = (N m, N m/rad, N m s/rad, kg m^2)
"""

#: A drive by its joint's kind: (peak N m, kp N m/rad, kd N m s/rad, armature kg m^2). The ankles
#: stiffer than the body leaning on them (m g h, 490 N m/rad). The neck under a 3.3 kg head
#: 0.18 m up (0.13 kg m^2), damped at half critical: at 60 N m/rad it nodded 6.1 degrees a step
#: to her surge; 150, 300, 600: 3.0, 2.2, 1.7. The peaks her controller's own, at the
#: drives' clamps before the stacks: her stacks grew to 1.5x them (`drives.STACKS`).
SERVO = {'spine': (142.5, 800.0, 30.0, 0.05), 'spine_roll': (77.4, 800.0, 30.0, 0.05),
         'waist': (77.4, 400.0, 20.0, 0.02),
         'neck': (15.0, 300.0, 6.0, 0.02), 'head': (8.0, 30.0, 1.0, 0.02),
         'shoulder': (40.0, 150.0, 6.0, 0.02), 'elbow': (25.0, 80.0, 3.0, 0.02),
         'wrist': (11.3, 150.0, 5.0, 0.02), 'gripper': (10.0, 40.0, 1.0, 0.02),
         'hip_yaw': (51.6, 300.0, 10.0, 0.05), 'hip_roll': (128.9, 900.0, 40.0, 0.05),
         'hip': (171.0, 900.0, 40.0, 0.05), 'knee': (171.0, 900.0, 35.0, 0.05),
         'ankle': (140.0, 1500.0, 40.0, 0.05), 'ankle_roll': (100.0, 1500.0, 40.0, 0.05),
         'foot': (11.6, 40.0, 1.0, 0.02)}

#: Whether a joint's clamp is its drive's peak where that is less (`drives.peak`), 0 or 1.
CLAMPED = 1.0
