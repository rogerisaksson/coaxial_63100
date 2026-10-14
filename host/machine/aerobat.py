"""The unlimited 3D RC aerobatic monoplane in MuJoCo: prop-hang, high-deflection surfaces, stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

An Extra 330SC / Edge 540 style 3D RC aerobatic monoplane (`aerobat.lisp`) powered by two Coaxial
63100 drives (`PROP` driving the 3D tractor propeller and `VECT` driving the coaxial contra/pitch
assist stage on finned aluminium heatsinks with temperature-controlled cooling fans): reversible
3D pitch/throttle pulls upright in vertical prop-hang (`hover` / `torque-roll`) or inverted in
flat spins and harriers, while propwash over the oversized 3D elevator, full-span ailerons, and
rudder provides crisp attitude authority from zero airspeed through the 16-gate race course.
"""
import importlib
import math
import os

from machine import aerokin, aerolisp, pull, quad
from machine.aerobatics import HOVER
from machine.helicopter import (Answer, _align_axis, _mat_mul, _rot_x, _rot_y, _rot_z,
                                _so3_error, _turn_heading)
from machine.quad import STEP_S, inflow, wind_at

LISP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aerobat.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
CARD, COOLING = MANIFEST['routines']['aerobat_3d'], DYNAMICS['cooling']
_AS, _AF, _DL = AERO['aero_surfaces'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
WING_SPAN, WING_CHORD, WING_AREA = _AS['wing_span'], _AS['wing_chord'], _AS['wing_area']
PROP_R, PROP_Z = _AS['prop_radius'], _AS['prop_z']
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), 0.5 * WING_SPAN
IDLE_RAD_S, GOV_RAD_S, VECT_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, VECT_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
ELEV_DEG, AIL_DEG, RUD_DEG = (round(math.degrees(x), 6) for x in (
    _AS['elevator_limit'], _AS['aileron_limit'], _AS['rudder_limit']))
K_THRUST, K_DRAG = _AS['k_thrust'], _AS['k_drag']
CL_ALPHA, CD0, CD_INDUCED = _AS['cl_alpha'], _AS['cd0'], _AS['cd_induced']
K_PITCH, K_ROLL, K_YAW = _AS['k_pitch'], _AS['k_roll'], _AS['k_yaw']
PROPWASH_SHARE, INFLOW_FLOOR = _AS['propwash_share'], _AS['inflow_floor']
RHO, BODY_CDA = _AF['air_density'], _AF['parasite_cda']
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (KINEMATICS['envelope'][k] for k in (
    'floor_height', 'hover_height', 'top_height'))
WATERFALL_HZ, WATERFALL_RAD = (KINEMATICS['oscillator'][k] for k in (
    'tictoc_frequency', 'tictoc_amplitude'))
TORQUE_ROLL, KNIFE_CIRCLE, CLIMBING_HARRIER, SNAP_ENTRY, INVERTED, FLAT_SPIN = (STUNTS[k] for k in (
    'torque_roll', 'knife_circle', 'climbing_harrier', 'snap_entry', 'inverted', 'flat_spin'))
WATERFALL, ROLLING_HARRIER, LOMCEVAK, GYRO_TUMBLE, CUBAN_UNWIND, UNWIND = (STUNTS[k] for k in (
    'waterfall', 'rolling_harrier', 'lomcevak', 'gyro_tumble', 'cuban_unwind', 'unwind'))


def drive_loads(controls):
    """External load scales `(scale_prop, scale_vect)` on the PROP and VECT 63100 drives."""
    throt, elev, ail, rud = controls
    return (min(_DL['main_ceiling'], _DL['main_base'] + _DL['main_collective'] * (
                throt * throt + _DL['main_cyclic'] * (elev * elev + ail * ail))),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['tail_pitch'] * (
                0.5 * throt * throt + rud * rud)))


def surface_angles(controls):
    """`(elev_deg, ail_deg, rud_deg)` of the 3D elevator, ailerons, and rudder for `controls`."""
    _throt, elev, ail, rud = controls
    return elev * ELEV_DEG, ail * AIL_DEG, rud * RUD_DEG


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into an aerobat stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


def mjcf(things=()):
    """The 3D aerobatic monoplane and `things` (`grounds.solids`) as MuJoCo XML."""
    m = ('<geom name="fuselage" type="box" pos="0 0.09 0" size="0.045 0.22 0.045" contype="1" '
         'conaffinity="1"/><geom name="wing" type="box" pos="0 0.06 0" size="%g %g 0.010" '
         'contype="1" conaffinity="1"/><geom name="prop" type="cylinder" pos="0 %g 0" '
         'size="%g 0.008" euler="90 0 0" contype="1" conaffinity="1"/>'
         '<geom name="hstab" type="box" pos="0 -0.08 0" size="0.16 0.040 0.008" contype="1" '
         'conaffinity="1"/><geom name="vfin" type="box" pos="0 -0.08 0" '
         'size="0.008 0.040 0.085" contype="1" conaffinity="1"/>'
         % (0.5 * WING_SPAN, 0.5 * WING_CHORD, PROP_Z, PROP_R))
    for k, (sx, sz) in enumerate((sx, sz) for sx in (-SKID_X, SKID_X) for sz in STRUT_Z):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -SKID_M, sz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((head, SKID_M + FOOT_M, MASS_KG) + INERTIA + (m,)))


def _prop_drag(throt, elev, ail, w):
    return (0.34 + 1.60 * (throt * throt + 0.45 * (elev * elev + ail * ail))) * K_DRAG * w * w


def _aero_cda(throt=0.0, elev=0.0, ail=0.0):
    """Total airframe + wing profile (`CD0`) and induced (`CD_INDUCED`) drag area $C_D A$."""
    cl_eff = 0.12 * CL_ALPHA * (abs(throt) + PROPWASH_SHARE * math.hypot(elev, ail))
    return BODY_CDA + WING_AREA * (CD0 + CD_INDUCED * cl_eff * cl_eff)


def _aero_inflow(along, v_horiz, w_p):
    """Combined tractor-propeller axial inflow and translational wing-lift factor."""
    tip = max(w_p * PROP_R, 8.0)
    return inflow(along, w_p) * (1.0 + 0.04 * CL_ALPHA * min(0.35, v_horiz / tip))


class Sky(quad.Sky):

    """The 3D aerobatic monoplane in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        self._mj, self._np = importlib.import_module('mujoco'), importlib.import_module('numpy')
        self.model = self._mj.MjModel.from_xml_string(mjcf(things))
        self.data = self._mj.MjData(self.model)
        self.frame = self.model.body('frame').id
        self.lift, self.rudder_force, self.speeds = 0.0, 0.0, (0.0, 0.0)
        self.controls, self.hit = (0.0, 0.0, 0.0, 0.0), None
        names = [self.model.geom(g).name for g in range(self.model.ngeom)]
        self._names = [n.split('.')[0] for n in names]
        self._mine = {g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == self.frame}
        self._feet = {(min(g, names.index('floor')), max(g, names.index('floor')))
                      for g, n in enumerate(names) if n.startswith('skid')}
        self._gates = [g for g, n in enumerate(names) if n.startswith('gate')]
        self.gated = True
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """Return the aerobat to its landing gear at the spot, still, nothing struck."""
        super().reset()
        self.rudder_force, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)

    def state(self):
        """The aerobat's state: `quad.Sky.state` plus `rudder_force`, `speeds`, and `controls`."""
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
            dr = dk * math.sqrt(vx * vx + vy * vy + vz * vz)
            xf[f, :3] = (lift * float(t[0, 1]) - dr * vx,
                         lift * float(t[1, 1]) - dr * vy,
                         lift * float(t[2, 1]) - dr * vz)
            xf[f, 3:] = tuple(float(t[r, 0]) * ab[0] + float(t[r, 1]) * ab[1]
                              + float(t[r, 2]) * ab[2] for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)


class Core:

    """The 3D aerobat's SO(3) prop-hang, propwash surface deflection, and wind-observer core."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset observer and trajectory state."""
        self.vel_hat, self.push_hat, self.wind = [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.spin_hat, self.turn_hat = [0.0] * 3, [0.0] * 3
        self.begun, self.flip_ang, self.roll_ang = False, 0.0, 0.0
        self.piro_ang, self.harrier_phase = 0.0, 0.0
        self.Controls = (IDLE_RAD_S, IDLE_RAD_S, 0.0, 0.0, 0.0, 0.0)
        self.out = Answer((IDLE_RAD_S, IDLE_RAD_S), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0),
                          (1.0, 0.0), (0.0, 0.0, 0.0))

    def _observe(self, state, dt):
        turn = [[float(state['turn'][r][c]) for c in range(3)] for r in range(3)]
        vel, spin, h = [float(x) for x in state['vel']], [float(x) for x in state['spin']], float(state['h'])
        if not (h > 0.03 and self.begun):
            self.vel_hat, self.spin_hat, self.begun = list(vel), list(spin), True
            self.push_hat = [p * max(0.0, 1.0 - WO_PULL * dt) for p in self.push_hat]
            self.turn_hat = [t * max(0.0, 1.0 - WO_TURN * dt) for t in self.turn_hat]
            self.wind = [w * max(0.0, 1.0 - WO_PULL * dt) for w in self.wind]
            return
        u = [vel[a] - self.wind[a] for a in range(3)]
        fast = math.hypot(*u)
        sp = state.get('speeds', (0.0, 0.0))
        w_p = float(sp[0]) if sp[0] > 1.0 else self.Controls[0]
        w_v = float(sp[1]) if sp[1] > 1.0 else self.Controls[1]
        throt, elev, ail, rud = state.get('controls', self.Controls[2:])
        alpha = 0.5 * RHO * _aero_cda(throt, elev, ail) / MASS_KG * fast
        up = (turn[0][1], turn[1][1], turn[2][1])
        s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if throt >= 0.0 else -1.0)
        lift = float(state.get('lift', 0.0)) * _aero_inflow(s_along, math.hypot(u[0], u[2]), w_p)
        for a in range(3):
            off = vel[a] - self.vel_hat[a]
            acc = (lift * up[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
                + self.push_hat[a] - alpha * u[a]
            self.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
            self.push_hat[a] += dt * WO_PULL * WO_PULL * off
        inv_fast = (1.0 / fast) if fast > MIN_AIR_M_S else 0.0
        uh = tuple(u[a] * inv_fast for a in range(3))
        gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_p * PROP_R)
                 if w_p > 1.0 and abs(s_along) < 4.0 * w_p * PROP_R else 0.0)
        m = [[alpha * ((1.0 if a == b else 0.0) + uh[a] * uh[b]) + gamma * up[a] * up[b]
              for b in range(3)] for a in range(3)]
        for a in (0, 2):
            mp = sum(m[a][b] * self.push_hat[b] for b in range(3))
            dw = dt * WO_WIND * mp / (m[0][a] ** 2 + m[1][a] ** 2 + m[2][a] ** 2 + WIND_REG)
            self.wind[a] += dw
            for b in range(3):
                self.push_hat[b] -= m[b][a] * dw
        q_p = _prop_drag(throt, elev, ail, w_p)
        tau = (K_PITCH * w_p * w_p * elev - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
               K_YAW * w_v * w_v * rud - q_p - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
               K_ROLL * w_p * w_p * ail - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
        for a in range(3):
            e = spin[a] - self.spin_hat[a]
            self.spin_hat[a] += dt * (tau[a] / INERTIA[a] + self.turn_hat[a] + 2.0 * WO_TURN * e)
            self.turn_hat[a] += dt * WO_TURN * WO_TURN * e

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_stop, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        wfall, inv = (WATERFALL_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(wfall) > 1e-6
        if stunt:
            d_flip, d_roll = -math.tau * flips * dt, -math.tau * rolls * dt
            if abs(wfall) > 1e-6:
                if self.harrier_phase == 0.0:
                    self.harrier_phase = math.pi if turn[1][1] > 0.0 else 0.0
                self.flip_ang = self.roll_ang = self.piro_ang = 0.0
                self.harrier_phase += math.tau * WATERFALL_HZ * dt
                rock = _rot_z if nose_val <= -2.5 else _rot_x
                r_des = _mat_mul(_rot_y(yaw), rock(0.5 * math.pi + wfall * math.cos(self.harrier_phase)))
                r_next = _mat_mul(_rot_y(yaw + rate * dt), rock(
                    0.5 * math.pi + wfall * math.cos(self.harrier_phase + math.tau * WATERFALL_HZ * dt)))
            else:
                self.harrier_phase = 0.0
                self.flip_ang += d_flip
                self.roll_ang += d_roll
                self.piro_ang += rate * dt
                base = _rot_y(yaw - self.piro_ang)
                r_des = _mat_mul(_mat_mul(base, _rot_x(self.flip_ang)),
                                 _mat_mul(_rot_z(self.roll_ang), _rot_y(self.piro_ang)))
                r_next = _mat_mul(_mat_mul(base, _rot_x(self.flip_ang + d_flip)),
                                  _mat_mul(_rot_z(self.roll_ang + d_roll), _rot_y(self.piro_ang + rate * dt)))
            w_ff = tuple(x / max(dt, 1e-6) for x in _so3_error(r_des, r_next))
            r11 = turn[1][1]
            ay_stunt = GRAVITY * 2.0 * r11 if abs(wfall) <= 1e-6 else ay_des * r11 / max(r11 * r11, 0.34)
            f_des = MASS_KG * (ay_stunt + 0.35 * (ax_des * turn[0][1] + az_des * turn[2][1]))
            return r_des, w_ff, f_des, stunt, inv

        self.harrier_phase = self.roll_ang = self.piro_ang = 0.0
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        want_up = (sign * ax_des / a_norm, sign * ay_des / a_norm, sign * az_des / a_norm)
        r_des = _align_axis((0.0, sign, 0.0), want_up, _mat_mul(_rot_y(yaw), _rot_x(self.flip_ang)))
        f_des = sign * MASS_KG * a_norm * sum(turn[k][1] * want_up[k] for k in range(3))
        if row['height'] <= FLOOR_M + 0.05 and state['h'] > 1.0:
            f_des = MASS_KG * (ax_des * turn[0][1] + ay_stop * turn[1][1] + az_des * turn[2][1])
        return r_des, (0.0, sign * rate, 0.0), f_des, stunt, inv

    def plan(self, law, row, state, dt, pull_max):
        """Advance outer horizontal kinematics `aerokin.lean` across `dt` s."""
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), math.radians(row.get('turn', 0.0)), dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        beside = math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))
        along = aerokin.lean(law, row, state['at'], state['vel'], dt, min(row['lean'], beside))
        law['heading'] = (c0, s0)
        return along

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        """One pass of `dt` s on `row` and `state`: returns `Answer`."""
        self._observe(state, dt)
        turn = [[float(state['turn'][r][c]) for c in range(3)] for r in range(3)]
        spin, vel = [float(x) for x in state['spin']], [float(x) for x in state['vel']]
        u = [vel[a] - self.wind[a] for a in range(3)]
        prev_ctrl = state.get('controls', self.Controls[2:])
        drag = 0.5 * RHO * _aero_cda(*prev_ctrl[:3]) / MASS_KG * math.sqrt(
            u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
        air_acc = [self.push_hat[a] - drag * u[a] for a in range(3)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        rate = math.radians(row.get('turn', 0.0))
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        up_pull, law['doing'] = aerokin.climb(row, state['h'], state['v'], reach - GRAVITY)

        if row['height'] < 0.0:
            self.flip_ang = self.roll_ang = self.piro_ang = self.harrier_phase = 0.0
            w_p_ask = GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            w_v_ask = VECT_GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            if limits:
                w_p_ask, w_v_ask = min(limits[0], w_p_ask), min(limits[1], w_v_ask)
            controls = (0.0, 0.0, 0.0, 0.0)
            self.Controls = (w_p_ask, w_v_ask) + controls
            law['held'] = False
            self.out = Answer((w_p_ask, w_v_ask), controls, (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out

        light_val = float(row.get('light', 0.5))
        acc_des = (along[0] - air_acc[0],
                   max(GRAVITY + up_pull, light_val * GRAVITY) - air_acc[1],
                   along[1] - air_acc[2])
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        r_des, w_ff, f_des, stunt, inv = self._target_attitude(
            row, state, turn, yaw, rate, acc_des, ay_stop, dt)

        f_limit = MASS_KG * reach
        f_des = max(-f_limit, min(f_limit, f_des))
        er = _so3_error(turn, r_des)
        ew = (w_ff[0] - spin[0], w_ff[1] - spin[1], w_ff[2] - spin[2])
        tau_x = INERTIA[0] * (ATT_KP * er[0] + ATT_KD * ew[0] - self.turn_hat[0]) \
            + (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2]
        tau_y = INERTIA[1] * (YAW_KP * er[1] + YAW_KD * ew[1] - self.turn_hat[1]) \
            + (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0]
        tau_z = INERTIA[2] * (ROLL_KP * er[2] + ROLL_KD * ew[2] - self.turn_hat[2]) \
            + (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1]

        w_top = limits[0] if limits else pull.held(pull.I_MAX)
        w_vect_top = limits[1] if limits else w_top
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _aero_inflow(
            along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_p_ask = min(w_top, max(w_gov, math.sqrt(abs(f_comp) / K_THRUST)))
        w_p_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_p_ask)
        throt = max(-1.0, min(1.0, f_comp / (K_THRUST * w_p_cur * w_p_cur)))
        elev = max(-1.0, min(1.0, tau_x / (K_PITCH * w_p_cur * w_p_cur)))
        ail = max(-1.0, min(1.0, tau_z / (K_ROLL * w_p_cur * w_p_cur)))

        tau_rud_des = tau_y + _prop_drag(throt, elev, ail, w_p_cur)
        w_v_ask = min(w_vect_top, max(VECT_GOV_RAD_S, math.sqrt(abs(tau_rud_des) / K_YAW),
                                      VECT_RATIO * w_p_ask))
        w_v_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_v_ask)
        rud = max(-1.0, min(1.0, tau_rud_des / (K_YAW * w_v_cur * w_v_cur)))

        controls = (throt, elev, ail, rud)
        self.Controls = (w_p_ask, w_v_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        law['held'] = bool(
            not stunt and abs(row['height'] - state['h']) <= aerokin.HELD_M
            and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(float(state['at'][0]) - law['spot'][0],
                           float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and sign_ok)
        law['sequence'] += 1
        bound = int(abs(throt) >= 0.999) | (int(abs(rud) >= 0.999) << 1)
        self.out = Answer((w_p_ask, w_v_ask), controls,
                          (K_THRUST * w_p_cur * w_p_cur * throt, K_YAW * w_v_cur * w_v_cur * rud),
                          (c, s), tuple(self.wind), bound)
        return self.out
