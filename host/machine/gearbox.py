"""Each drive's gearbox: its rotor free across the play, the box wound up past it, the link turned.

The rotor runs free across the play, winds the box and the structure up past it against the
mesh's friction, and the link is turned by that wind-up alone (`physics.BOXED`).

    boxes = Boxes(np)                                   # the numbers a joint (`machine.drives`)
    passed = boxes.step(np, ctrl, qd, brake, dt)        # the links' torques; the rotors stepped on
    boxes.delta, boxes.w                                # each rotor's angle past its link, its speed
"""
import math

from machine import drives
from machine.figure import JOINTS

#: A board's torque counts as taking up the play from this, N m (`ahead`).
PLAY_NM = 1.0

#: The mesh's friction rounded over this speed, rad/s; the rotors stepped this many times within
#: the world's step: at one on the knee's 6.2 kN m/rad and 0.177 kg m^2 (187 rad/s, 0.19 of a
#: step) the rotor's mode grew faster than its damping took it and she fell rising (2026-10-11).
MESH_RAD_S, SUBSTEPS = 0.02, 5


class Boxes:

    """The gearboxes' numbers a joint and their state: half each play, rad (`drives.play`); the
    wind-up past it, N m/rad (`drives.wind`), and its damping, N m s/rad (`drives.BOX_ZETA`); the
    mesh's friction, N m (`drives.mesh`); the rotor's inertia, kg m^2 (`drives.armature`, off the
    joint's armature while BOXED); the rotor's angle past the link's and its speed."""

    def __init__(self, np):
        self.driven = np.array([drives.passive(j) is None for j in JOINTS])
        self.gap = np.array([drives.play(j) for j in JOINTS])
        self.k = np.array([drives.wind(j) for j in JOINTS])
        self.j = np.array([drives.armature(j) for j in JOINTS])
        self.c = 2.0 * drives.BOX_ZETA * np.sqrt(self.k * self.j)
        self.mesh = np.array([drives.mesh(j) for j in JOINTS])
        self.delta, self.w = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        self.held, self.q_was = math.radians(drives.RIGID_PLAY_DEG) / 2.0, np.zeros(len(JOINTS))

    def reset(self, q):
        """Every rotor on its joint at `q`, still: a board's encoder reads its hold there."""
        self.delta[:] = self.w[:] = 0.0
        self.q_was[:] = q

    def rigid(self, np, q):
        """The boards' encoders on rigid boxes: each held within `held` of its joint at `q`, rad,
        every board's - as the rigid world was modelled before the boxes (`drives.RIGID_PLAY_DEG`)."""
        self.delta = np.clip(self.delta + self.q_was - q, -self.held, self.held)
        self.q_was[:] = q
        return self.delta

    def ahead(self, np, torque):
        """How far past its setpoint a board on the motor's encoder asks its rotor, rad, so the
        link lands on it: the box wound up by `torque` and the play that torque has taken up.
        On the play alone her toes dragged 20 mm at lift where 1 rigid (2026-10-11)."""
        k = np.where(self.driven, self.k, 1.0)
        return np.where(self.driven, drives.AHEAD_WIND * torque / k
                        + drives.AHEAD_PLAY * self.gap * np.tanh(torque / PLAY_NM), 0.0)

    def step(self, np, ctrl, qd, brake, dt):
        """The links' torques from the motors' `ctrl` with the links at `qd` rad/s: what each box
        passes on - the wind-up past the play, its damping, the mesh's friction -, the rotors
        stepped `dt` on by what is left of their motors' torques against it, `brake` each rotor's
        braking, N m s/rad (a short's), implicit."""
        on = self.driven
        j = np.where(on, self.j, 1.0)
        h = dt / SUBSTEPS
        mean = np.zeros(len(ctrl))
        for _ in range(SUBSTEPS):
            rel = self.w - qd
            past = np.sign(self.delta) * np.maximum(np.abs(self.delta) - self.gap, 0.0)
            passed = np.where(past != 0.0, self.k * past + self.c * rel
                              + self.mesh * np.tanh(rel / MESH_RAD_S), 0.0)
            self.w = np.where(on, (self.w + h * (ctrl - passed) / j) / (1.0 + h * brake / j), qd)
            self.delta += h * (self.w - qd) * on
            mean += passed / SUBSTEPS
        return np.where(on, mean, ctrl)
