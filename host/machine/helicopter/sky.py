"""The 3D helicopter in MuJoCo (`Sky`), and the frame every two-drive craft lays in it (`_init_sky`)."""
import importlib
import math
import os

from machine import quad
from machine.helicopter.airframe import (BODY_CDA, FOOT_M, K_CYC, K_TAIL, K_THRUST, RHO, SKID_M,
                                         TAIL_M, _rotor_drag, _rotor_inflow, mjcf)
from machine.quad import STEP_S, wind_at


def _init_sky(sky, xml, rest, touch=()):
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    sky._mj, sky._np = importlib.import_module('mujoco'), importlib.import_module('numpy')
    sky.model = sky._mj.MjModel.from_xml_string(xml)
    sky.data = sky._mj.MjData(sky.model)
    sky.frame, sky.rest = sky.model.body('frame').id, rest
    sky.lift, sky.speeds, sky.controls, sky.hit = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0), None
    names = [sky.model.geom(g).name for g in range(sky.model.ngeom)]
    sky._names = [n.split('.')[0] for n in names]
    sky._mine = {g for g in range(sky.model.ngeom) if sky.model.geom_bodyid[g] == sky.frame}
    sky._feet = {(min(g, names.index('floor')), max(g, names.index('floor')))
                 for g, n in enumerate(names) if n.startswith('skid') or n in touch}
    sky._gates = [g for g, n in enumerate(names) if n.startswith('gate')]
    sky.gated = True
    sky._mj.mj_forward(sky.model, sky.data)


class Sky(quad.Sky):

    """The 3D helicopter in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        self.tail = 0.0
        _init_sky(self, mjcf(things), SKID_M + FOOT_M)

    def reset(self):
        super().reset()
        self.tail, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)

    def state(self):
        out = super().state()
        out.update(tail=self.tail, speeds=self.speeds, controls=self.controls)
        return out

    def step(self, command, dt, air=None):
        d, f = self.data, self.frame
        w_m, w_t, coll, cyc_p, cyc_r, tail_p = (float(x) for x in (list(command) + [0.0] * 6)[:6])
        self.speeds, self.controls, self.lift = (w_m, w_t), (coll, cyc_p, cyc_r, tail_p), K_THRUST * w_m * w_m * coll
        turn, h = d.xmat[f].reshape(3, 3), float(d.qpos[1]) - (SKID_M + FOOT_M)
        wx, wy, wz = (tuple(a + b for a, b in zip(wind_at(air['wind'], h), wind_at(air['eddies'][0], h))) if air is not None else (0.0, 0.0, 0.0))
        along = sum((float(d.qvel[a]) - w) * float(turn[a, 1]) for a, w in enumerate((wx, wy, wz)))
        lift = self.lift * _rotor_inflow(along if coll >= 0.0 else -along, math.hypot(float(d.qvel[0]) - wx, float(d.qvel[2]) - wz), w_m)
        self.tail = f_t = K_TAIL * w_t * w_t * tail_p
        ab = (K_CYC * w_m * w_m * cyc_p, TAIL_M * f_t - _rotor_drag(coll, cyc_p, cyc_r, w_m), K_CYC * w_m * w_m * cyc_r)
        dk, qv, xf = 0.5 * RHO * BODY_CDA, d.qvel, d.xfrc_applied
        for _ in range(max(1, int(round(dt / STEP_S)))):
            t = d.xmat[f].reshape(3, 3)
            vx, vy, vz = float(qv[0]) - wx, float(qv[1]) - wy, float(qv[2]) - wz
            dr = dk * math.hypot(vx, vy, vz)
            xf[f, :3] = (lift * float(t[0, 1]) + f_t * float(t[0, 0]) - dr * vx,
                         lift * float(t[1, 1]) + f_t * float(t[1, 0]) - dr * vy,
                         lift * float(t[2, 1]) + f_t * float(t[2, 0]) - dr * vz)
            xf[f, 3:] = tuple(sum(float(t[r, k]) * ab[k] for k in range(3)) for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)
