"""The ornithopter in MuJoCo: its four wings flapping, folding, in the air."""
import math

from machine import quad
from machine.helicopter.sky import _init_sky
from machine.ornithopter.airframe import (AFT_Z, BEAT_PULSE, FOOT_M, FORE_Z, GEAR_RATIO, K_PITCH,
                                          K_ROLL, K_THRUST, K_YAW, RHO, SKID_M, _flap_inflow,
                                          _wing_cda, _wing_drag, mjcf)
from machine.quad import STEP_S
from machine.weather import wind_at


class Sky(quad.Sky):

    """The 4-wing Dune ornithopter in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        self.yaw_force, self.phase = 0.0, 0.0
        _init_sky(self, mjcf(things), SKID_M + FOOT_M)

    def reset(self):
        """Return the ornithopter to its claws at the origin, still, nothing struck."""
        super().reset()
        self.yaw_force, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
        self.phase = 0.0

    def state(self):
        """The ornithopter's state: `quad.Sky.state` plus `yaw_force`, `speeds`, and `controls`."""
        out = super().state()
        out.update(yaw_force=self.yaw_force, speeds=self.speeds, controls=self.controls)
        return out

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_fore, w_aft, stroke, pitch_tilt, roll_diff, yaw_twist)`."""
        d, f = self.data, self.frame
        cmd = list(command) + [0.0] * 6
        w_f, w_a, stroke, pitch_t, roll_d, yaw_t = (float(x) for x in cmd[:6])
        self.speeds, self.controls = (w_f, w_a), (stroke, pitch_t, roll_d, yaw_t)
        w_sq = 0.5 * (w_f * w_f + w_a * w_a)
        w_eq = math.sqrt(w_sq)
        self.lift = K_THRUST * w_sq * stroke
        turn, h = d.xmat[f].reshape(3, 3), float(d.qpos[1]) - (SKID_M + FOOT_M)
        wx, wy, wz = (tuple(a + b for a, b in zip(wind_at(air['wind'], h),
                                                  wind_at(air['eddies'][0], h)))
                      if air is not None else (0.0, 0.0, 0.0))
        along = sum((float(d.qvel[a]) - w) * float(turn[a, 1]) for a, w in enumerate((wx, wy, wz)))
        vh = math.hypot(float(d.qvel[0]) - wx, float(d.qvel[2]) - wz)
        lift = self.lift * _flap_inflow(along if stroke >= 0.0 else -along, vh, w_eq)
        self.yaw_force = f_yaw = K_YAW * w_a * w_a * yaw_t
        ab = (K_PITCH * w_sq * pitch_t, f_yaw - _wing_drag(stroke, pitch_t, roll_d, w_eq),
              K_ROLL * w_sq * roll_d)
        dk, qv, xf = 0.5 * RHO * _wing_cda(stroke), d.qvel, d.xfrc_applied
        beat, rock = w_eq / GEAR_RATIO, 0.5 * lift * BEAT_PULSE * (FORE_Z - AFT_Z)
        for _ in range(max(1, int(round(dt / STEP_S)))):
            t = d.xmat[f].reshape(3, 3)
            self.phase = (self.phase + beat * STEP_S) % math.tau
            vx, vy, vz = float(qv[0]) - wx, float(qv[1]) - wy, float(qv[2]) - wz
            dr = dk * math.sqrt(vx * vx + vy * vy + vz * vz)
            xf[f, :3] = (lift * float(t[0, 1]) - dr * vx,
                         lift * float(t[1, 1]) - dr * vy,
                         lift * float(t[2, 1]) - dr * vz)
            ab0 = ab[0] + rock * math.cos(2.0 * self.phase)
            xf[f, 3:] = tuple(float(t[r, 0]) * ab0 + float(t[r, 1]) * ab[1]
                              + float(t[r, 2]) * ab[2] for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)
