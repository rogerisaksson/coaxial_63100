"""The 3D aerobatic monoplane in MuJoCo: its propeller, its surfaces in the propwash, its wing."""
import math

from machine.aircraft.aerobat.airframe import (FOOT_M, K_PITCH_Q, K_ROLL_Q, K_THRUST, K_YAW, PROP_R, PROP_Z,
                                      RHO, RUNWAY_RAD, SKID_M, WING, _aero_cda, _aero_inflow, _prop_drag,
                                      mjcf, slipstream)
from machine.aircraft.helicopter import sky as hsky
from machine.aircraft.quad import STEP_S
from machine.aircraft.wingborne import PITCH_REST, wing_substep
from tools.cores import bodied


class Sky(hsky.Sky):

    """The 3D aerobatic monoplane in MuJoCo on the floor among `things`."""

    _force_key, _disc = 'rudder_force', (PROP_Z, PROP_R)

    def __init__(self, things=()):
        self.rudder_force, self.wing = 0.0, WING
        hsky._init_sky(self, mjcf(things), SKID_M + FOOT_M, ('hstab', 'vfin'))
        self._wing_step = bodied.on_body(wing_substep, 'wing_substep', tuple(range(1, 9)))
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
        self._rest_pose()

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_prop, w_vect, _, elev, ail, rud)`: its propeller's pitch
        fixed, its thrust its speed's through its inflow, its surfaces in its slipstream."""
        (w_p, w_v), (_pitch, elev, ail, rud), (wx, wy, wz), along, vh = self._unpack_and_wind(command, air)
        self.lift, self.share = K_THRUST * w_p * w_p, _aero_inflow(along, vh, w_p)
        lift = self.lift * self.share * self.ground(1.0)
        q_s = slipstream(along, self.lift * self.share)
        self.rudder_force = tau_rud = K_YAW * w_v * w_v * rud
        ab0 = K_PITCH_Q * q_s * elev
        ab1 = tau_rud - _prop_drag(self.share, elev, ail, w_p)
        ab2 = K_ROLL_Q * q_s * ail
        dk, d = 0.5 * RHO * _aero_cda(elev, ail), self.data
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self._wing_step(self, lift, wx, wy, wz, dk, ab0, ab1, ab2)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)

