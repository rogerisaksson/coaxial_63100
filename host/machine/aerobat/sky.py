"""The 3D aerobatic monoplane in MuJoCo: its propeller, its surfaces in the propwash, its wing."""
import math

from machine import quad
from machine.aerobat.airframe import (FOOT_M, K_PITCH, K_ROLL, K_THRUST, K_YAW, RHO, RUNWAY_RAD,
                                      SKID_M, WING, _aero_cda, _aero_inflow, _prop_drag, mjcf)
from machine.helicopter.sky import _init_sky
from machine.quad import STEP_S
from machine.weather import wind_at
from machine.wingborne import PITCH_REST


class Sky(quad.Sky):

    """The 3D aerobatic monoplane in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        self.rudder_force = 0.0
        _init_sky(self, mjcf(things), SKID_M + FOOT_M, ('hstab', 'vfin'))
        self._rest_pose()

    def _rest_pose(self):
        """On its wheels at the runway's end, its nose up the runway."""
        cy, sy = math.cos(0.5 * RUNWAY_RAD), math.sin(0.5 * RUNWAY_RAD)
        cx, sx = math.cos(0.5 * PITCH_REST), math.sin(0.5 * PITCH_REST)
        self.data.qpos[3:7] = (cy * cx, cy * sx, sy * cx, -sy * sx)
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """On its wheels at the spot, still, nothing struck."""
        super().reset()
        self.rudder_force, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
        self._rest_pose()

    def state(self):
        """`quad.Sky.state` with `rudder_force`, `speeds` and `controls`."""
        out = super().state()
        out.update(rudder_force=self.rudder_force, speeds=self.speeds, controls=self.controls)
        return out

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_prop, w_vect, throttle_pitch, elev, ail, rud)`."""
        d, f = self.data, self.frame
        cmd = list(command) + [0.0] * 6
        w_p, w_v, throt, elev, ail, rud = (float(x) for x in cmd[:6])
        self.speeds, self.controls = (w_p, w_v), (throt, elev, ail, rud)
        self.lift = K_THRUST * w_p * w_p * throt
        turn, h = d.xmat[f].reshape(3, 3), float(d.qpos[1]) - (SKID_M + FOOT_M)
        wx, wy, wz = (tuple(a + b for a, b in zip(wind_at(air['wind'], h),
                                                  wind_at(air['eddies'][0], h)))
                      if air is not None else (0.0, 0.0, 0.0))
        along = sum((float(d.qvel[a]) - w) * float(turn[a, 1]) for a, w in enumerate((wx, wy, wz)))
        vh = math.hypot(float(d.qvel[0]) - wx, float(d.qvel[2]) - wz)
        lift = self.lift * _aero_inflow(along if throt >= 0.0 else -along, vh, w_p)
        self.rudder_force = tau_rud = K_YAW * w_v * w_v * rud
        ab = (K_PITCH * w_p * w_p * elev, tau_rud - _prop_drag(throt, elev, ail, w_p),
              K_ROLL * w_p * w_p * ail)
        dk, qv, xf = 0.5 * RHO * _aero_cda(throt, elev, ail), d.qvel, d.xfrc_applied
        for _ in range(max(1, int(round(dt / STEP_S)))):
            t = d.xmat[f].reshape(3, 3)
            vx, vy, vz = float(qv[0]) - wx, float(qv[1]) - wy, float(qv[2]) - wz
            lx, ly, lz = WING.lift((vx, vy, vz), t)
            dr = dk * math.hypot(vx, vy, vz)
            xf[f, :3] = (lift * float(t[0, 1]) + lx - dr * vx,
                         lift * float(t[1, 1]) + ly - dr * vy,
                         lift * float(t[2, 1]) + lz - dr * vz)
            a0, a1, a2 = WING.moments((vx, vy, vz), t, qv)
            xf[f, 3:] = tuple(float(t[r, 0]) * (ab[0] + a0) + float(t[r, 1]) * (ab[1] + a1)
                              + float(t[r, 2]) * (ab[2] + a2) for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)
