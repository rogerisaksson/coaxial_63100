"""The 3D aerobatic monoplane in MuJoCo: its propeller, its surfaces in the propwash, its wing."""
import math

from machine.aerobat.airframe import (FOOT_M, K_PITCH, K_ROLL, K_THRUST, K_YAW, RHO, RUNWAY_RAD,
                                      SKID_M, WING, _aero_cda, _aero_inflow, _prop_drag, mjcf)
from machine.helicopter import sky as hsky
from machine.quad import STEP_S
from machine.wingborne import PITCH_REST


class Sky(hsky.Sky):

    """The 3D aerobatic monoplane in MuJoCo on the floor among `things`."""

    _force_key = 'rudder_force'

    def __init__(self, things=()):
        self.rudder_force = 0.0
        hsky._init_sky(self, mjcf(things), SKID_M + FOOT_M, ('hstab', 'vfin'))
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
        dk, qv, turn = 0.5 * RHO * _aero_cda(throt, elev, ail), self.data.qvel, self._turn
        for _ in range(max(1, int(round(dt / STEP_S)))):
            vx0, vy0, vz0 = self._vel.tolist()
            u = (vx0 - wx, vy0 - wy, vz0 - wz)
            lx, ly, lz = WING.lift(u, turn)
            a0, a1, a2 = WING.moments(u, turn, qv)
            self._substep(0.0, lift, wx, wy, wz, dk, ab0 + a0, ab1 + a1, ab2 + a2, lx, ly, lz)

