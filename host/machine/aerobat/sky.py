"""The 3D aerobatic monoplane in MuJoCo: its propeller, its surfaces in the propwash, its wing."""
import math

from machine.aerobat.airframe import (FOOT_M, K_PITCH, K_ROLL, K_THRUST, K_YAW, RHO, RUNWAY_RAD,
                                      SKID_M, WING, _aero_cda, _aero_inflow, _prop_drag, mjcf)
from machine.helicopter import sky as hsky
from machine.quad import STEP_S
from machine.wingborne import PITCH_REST, wing_substep
from tools.cores import bodied


class Sky(hsky.Sky):

    """The 3D aerobatic monoplane in MuJoCo on the floor among `things`."""

    _force_key = 'rudder_force'

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
        """Step `dt` s under `command` `(w_prop, w_vect, throttle_pitch, elev, ail, rud)`."""
        (w_p, w_v), (throt, elev, ail, rud), (wx, wy, wz), along, vh = self._unpack_and_wind(command, air)
        self.lift = K_THRUST * w_p * w_p * throt
        lift = self.lift * _aero_inflow(along, vh, w_p)
        self.rudder_force = tau_rud = K_YAW * w_v * w_v * rud
        ab0 = K_PITCH * w_p * w_p * elev
        ab1 = tau_rud - _prop_drag(throt, elev, ail, w_p)
        ab2 = K_ROLL * w_p * w_p * ail
        dk, d = 0.5 * RHO * _aero_cda(throt, elev, ail), self.data
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self._wing_step(self, lift, wx, wy, wz, dk, ab0, ab1, ab2)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)

