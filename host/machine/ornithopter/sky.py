"""The ornithopter in MuJoCo: its four wings flapping, folding, in the air."""
import math

from machine.helicopter import sky as hsky
from machine.ornithopter.airframe import (AFT_Z, BEAT_PULSE, FOOT_M, FORE_Z, GEAR_RATIO, K_PITCH,
                                          K_ROLL, K_THRUST, K_YAW, RHO, SKID_M, _flap_inflow,
                                          _wing_cda, _wing_drag, mjcf)
from machine.quad import STEP_S


class Sky(hsky.Sky):

    """The 4-wing Dune ornithopter in MuJoCo on the floor among `things`."""

    _force_key = 'yaw_force'

    def __init__(self, things=()):
        self.yaw_force, self.phase = 0.0, 0.0
        hsky._init_sky(self, mjcf(things), SKID_M + FOOT_M)

    def reset(self):
        """Return the ornithopter to its claws at the origin, still, nothing struck."""
        super().reset()
        self.phase = 0.0

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_fore, w_aft, stroke, pitch_tilt, roll_diff, yaw_twist)`."""
        (w_f, w_a), (stroke, pitch_t, roll_d, yaw_t), (wx, wy, wz), along, vh = self._unpack_and_wind(command, air)
        w_sq = 0.5 * (w_f * w_f + w_a * w_a)
        w_eq = math.sqrt(w_sq)
        self.lift = K_THRUST * w_sq * stroke
        lift = self.lift * _flap_inflow(along, vh, w_eq)
        self.yaw_force = f_yaw = K_YAW * w_a * w_a * yaw_t
        ab0 = K_PITCH * w_sq * pitch_t
        ab1 = f_yaw - _wing_drag(stroke, pitch_t, roll_d, w_eq)
        ab2 = K_ROLL * w_sq * roll_d
        dk = 0.5 * RHO * _wing_cda(stroke)
        beat, rock = w_eq / GEAR_RATIO, 0.5 * lift * BEAT_PULSE * (FORE_Z - AFT_Z)
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self.phase = (self.phase + beat * STEP_S) % math.tau
            self._substep(0.0, lift, wx, wy, wz, dk, ab0 + rock * math.cos(2.0 * self.phase), ab1, ab2)

