"""The 3D helicopter in MuJoCo (`Sky`), and the frame every two-drive craft lays in it (`_init_sky`)."""
import math

from machine import quad
from machine.helicopter.airframe import (BODY_CDA, FOOT_M, K_CYC, K_TAIL, K_THRUST, RHO, SKID_M,
                                         TAIL_M, _rotor_drag, _rotor_inflow, mjcf)
from machine.quad import STEP_S
from machine.weather import wind_at


def _init_sky(sky, xml, rest, touch=()):
    quad.Sky.__init__(sky, xml=xml, rest=rest, touch=touch)
    sky.speeds, sky.controls = (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)


class Sky(quad.Sky):

    """The 3D helicopter in MuJoCo on the floor among `things`."""

    _force_key = 'tail'

    def __init__(self, things=()):
        setattr(self, self._force_key, 0.0)
        _init_sky(self, mjcf(things), SKID_M + FOOT_M)

    def reset(self):
        super().reset()
        setattr(self, self._force_key, 0.0)
        self.speeds, self.controls = (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)

    def state(self, copy=True):
        out = super().state(copy)
        out[self._force_key] = getattr(self, self._force_key)
        out['speeds'] = self.speeds
        out['controls'] = self.controls
        return out

    def _unpack_and_wind(self, command, air):
        """Unpack 6-element `command` into `speeds` and `controls`, and return
        `((w0, w1), controls, (wx, wy, wz), signed_along, v_horiz)` at the current pose."""
        c = tuple(float(x) for x in command[:6])
        if len(c) < 6:
            c = c + (0.0,) * (6 - len(c))
        self.speeds, self.controls = c[:2], c[2:]
        d, xm = self.data, self._xmat
        h = float(d.qpos[1]) - self.rest
        if air is not None:
            w0, e0 = wind_at(air['wind'], h), wind_at(air['eddies'][0], h)
            wx, wy, wz = w0[0] + e0[0], w0[1] + e0[1], w0[2] + e0[2]
        else:
            wx = wy = wz = 0.0
        vx0, vy0, vz0 = self._vel.tolist()
        vx, vy, vz = vx0 - wx, vy0 - wy, vz0 - wz
        along = vx * float(xm[1]) + vy * float(xm[4]) + vz * float(xm[7])
        return self.speeds, self.controls, (wx, wy, wz), (along if c[2] >= 0.0 else -along), math.hypot(vx, vz)

    def step(self, command, dt, air=None):
        (w_m, w_t), (coll, cyc_p, cyc_r, tail_p), (wx, wy, wz), along, vh = self._unpack_and_wind(command, air)
        self.lift = K_THRUST * w_m * w_m * coll
        lift = self.lift * _rotor_inflow(along, vh, w_m)
        self.tail = f_t = K_TAIL * w_t * w_t * tail_p
        ab0 = K_CYC * w_m * w_m * cyc_p
        ab1 = TAIL_M * f_t - _rotor_drag(coll, cyc_p, cyc_r, w_m)
        ab2 = K_CYC * w_m * w_m * cyc_r
        dk = 0.5 * RHO * BODY_CDA
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self._substep(f_t, lift, wx, wy, wz, dk, ab0, ab1, ab2)

