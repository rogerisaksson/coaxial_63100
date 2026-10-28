"""The 3D helicopter in MuJoCo (`Sky`), and the frame every two-drive craft lays in it (`_init_sky`)."""
import math

from machine.aircraft import quad
from machine.aircraft.helicopter.airframe import (BODY_CDA, DISC_M, FOOT_M, K_CYC, K_TAIL, K_THRUST, RHO,
                                         ROTOR_R, SKID_M, TAIL_M, _rotor_drag, _rotor_inflow, mjcf)
from machine.aircraft.quad import STEP_S
from machine.aircraft.weather import wind_at


def _init_sky(sky, xml, rest, touch=()):
    quad.Sky.__init__(sky, xml=xml, rest=rest, touch=touch)
    sky.speeds, sky.controls = (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)


class Sky(quad.Sky):

    """The 3D helicopter in MuJoCo on the floor among `things`."""

    _force_key, _disc = 'tail', (DISC_M, ROTOR_R)

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
        qv, xm = self._qv, self._xm
        h = self._qp[1] - self.rest
        if air is not None:
            w0, e0 = wind_at(air['wind'], h, air['steady']), wind_at(air['eddies'][0], h)
            wx, wy, wz = w0[0] + e0[0], w0[1] + e0[1], w0[2] + e0[2]
        else:
            wx = wy = wz = 0.0
        vx, vy, vz = qv[0] - wx, qv[1] - wy, qv[2] - wz
        along = vx * xm[1] + vy * xm[4] + vz * xm[7]
        return self.speeds, self.controls, (wx, wy, wz), (along if c[2] >= 0.0 else -along), math.hypot(vx, vz)

    def ground(self, ask):
        """Its disc's thrust's share over the floor (`quad.ground`): the disc `_disc` (its middle
        up the frame's axis, its radius, m), its thrust along that axis or against it as `ask`."""
        middle, radius = self._disc
        up = (self._xm[4], -self._xm[4])[int(ask < 0.0)]
        return quad.ground((self._qp[1] + self._xm[4] * middle) / up if up > 0.0 else 0.0, radius)

    def step(self, command, dt, air=None):
        (w_m, w_t), (coll, cyc_p, cyc_r, tail_p), (wx, wy, wz), along, vh = self._unpack_and_wind(command, air)
        self.lift = K_THRUST * w_m * w_m * coll
        lift = self.lift * _rotor_inflow(along, vh, w_m) * self.ground(coll)
        self.tail = f_t = K_TAIL * w_t * w_t * tail_p
        ab0 = K_CYC * w_m * w_m * cyc_p
        ab1 = TAIL_M * f_t - _rotor_drag(coll, cyc_p, cyc_r, w_m)
        ab2 = K_CYC * w_m * w_m * cyc_r
        dk = 0.5 * RHO * BODY_CDA
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self._substep(f_t, lift, wx, wy, wz, dk, ab0, ab1, ab2)

