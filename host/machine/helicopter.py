"""The 3D aerobatic helicopter in MuJoCo: collective/cyclic swashplate, tail rotor, and stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

A single-main-rotor 3D helicopter on two 63100 drives (`MAIN` on the mast, `TAIL` on the boom):
collective pitch pulls either way along the mast, cyclic tilts the hub through zero thrust, and
the tail rotor counters main-rotor torque and drives pirouettes.
"""
import importlib
import math
import os

from machine import aerokin, aerolisp, pull, quad
from machine.aerobatics import HOVER
from machine.figure import cross
from machine.quad import STEP_S, inflow, wind_at

LISP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'helicopter.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())
QUAD_LISP_PATH, QUAD_MANIFEST = quad.QUAD_LISP_PATH, quad.MANIFEST

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
CARD, COOLING = MANIFEST['routines']['helicopter_3d'], DYNAMICS['cooling']
_MR, _TR, _AF, _DL = AERO['main_rotor'], AERO['tail_rotor'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY = _RB['mass'], _RB['gravity']
INERTIA = (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz'])
DISC_M, ROTOR_R, TAIL_M, TAIL_R = _MR['disc_height'], _MR['radius'], _TR['boom_arm'], _TR['radius']
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), ROTOR_R
IDLE_RAD_S, GOV_RAD_S, TAIL_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, TAIL_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
COLL_DEG, CYC_DEG, TAIL_DEG = (round(math.degrees(x), 6) for x in (
    _MR['collective_limit'], _MR['cyclic_limit'], _TR['pitch_limit']))
K_THRUST, K_CYC, K_DRAG = _MR['k_thrust'], _MR['k_cyclic'], _MR['k_drag']
PROFILE_DRAG, INDUCED_DRAG, CYCLIC_DRAG = (
    _MR['profile_drag_share'], _MR['induced_drag_share'], _MR['cyclic_drag_share'])
INFLOW_FLOOR, K_TAIL, RHO, BODY_CDA = (
    _MR['inflow_floor'], _TR['k_thrust'], _AF['air_density'], _AF['parasite_cda'])
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (KINEMATICS['envelope'][k] for k in (
    'floor_height', 'hover_height', 'top_height'))
TICTOC_HZ, TICTOC_RAD = (KINEMATICS['oscillator'][k] for k in (
    'tictoc_frequency', 'tictoc_amplitude'))
FUNNEL, SPIRAL, FLIP_IN, INVERTED, INV_PIRO = (STUNTS[k] for k in (
    'funnel', 'spiral', 'flip_in', 'inverted', 'inv_piro'))
TICTOC, AILERON_TICTOC, PIRO_FLIP, CHAOS, HURRICANE, UNWIND = (STUNTS[k] for k in (
    'tictoc', 'aileron_tictoc', 'piro_flip', 'chaos', 'hurricane', 'unwind'))


def drive_loads(controls):
    """External load scales `(scale_main, scale_tail)` on the two 63100 drives for `controls`."""
    c, p, r, t = controls
    return (min(_DL['main_ceiling'], _DL['main_base'] + _DL['main_collective'] * (
                c * c + _DL['main_cyclic'] * (p * p + r * r))),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['tail_pitch'] * t * t))


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into a stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


def mjcf(things=()):
    """The helicopter and `things` (`grounds.solids`) as MuJoCo XML."""
    m = ('<geom name="canopy" type="box" pos="0 0 0.08" size="0.06 0.05 0.18" contype="1" '
         'conaffinity="1"/><geom name="disc" type="cylinder" pos="0 %g 0" size="%g 0.008" '
         'euler="90 0 0" contype="1" conaffinity="1"/><geom name="boom" type="capsule" '
         'fromto="0 0.012 -0.05 0 0.012 %g" size="0.014" contype="1" conaffinity="1"/>'
         '<geom name="fin" type="box" pos="0 0.02 %g" size="0.01 0.08 0.03" contype="1" '
         'conaffinity="1"/>' % (DISC_M, ROTOR_R, -TAIL_M, -TAIL_M))
    for k, (sx, sz) in enumerate((sx, sz) for sx in (-SKID_X, SKID_X) for sz in STRUT_Z):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -SKID_M, sz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((head, SKID_M + FOOT_M, MASS_KG) + INERTIA + (m,)))


def _rotor_drag(coll, cyc_p, cyc_r, w):
    return (PROFILE_DRAG + INDUCED_DRAG * (coll * coll + CYCLIC_DRAG * (
        cyc_p * cyc_p + cyc_r * cyc_r))) * K_DRAG * w * w


class Sky(quad.Sky):

    """The 3D helicopter in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        self._mj, self._np = importlib.import_module('mujoco'), importlib.import_module('numpy')
        self.model = self._mj.MjModel.from_xml_string(mjcf(things))
        self.data = self._mj.MjData(self.model)
        self.frame = self.model.body('frame').id
        self.lift, self.tail, self.speeds = 0.0, 0.0, (0.0, 0.0)
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
        """The helicopter back on its skids at its spot, still, nothing struck."""
        super().reset()
        self.tail, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)

    def state(self):
        """The helicopter's state: `quad.Sky.state` plus `tail`, `speeds`, and `controls`."""
        out = super().state()
        out.update(tail=self.tail, speeds=self.speeds, controls=self.controls)
        return out

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_main, w_tail, coll, cyc_pitch, cyc_roll, tail_pitch)`."""
        d, f = self.data, self.frame
        cmd = list(command) + [0.0] * 6
        w_m, w_t, coll, cyc_p, cyc_r, tail_p = (float(x) for x in cmd[:6])
        self.speeds, self.controls = (w_m, w_t), (coll, cyc_p, cyc_r, tail_p)
        self.lift = K_THRUST * w_m * w_m * coll
        turn, h = d.xmat[f].reshape(3, 3), float(d.qpos[1]) - (SKID_M + FOOT_M)
        wx, wy, wz = (tuple(a + b for a, b in zip(wind_at(air['wind'], h),
                                                  wind_at(air['eddies'][0], h)))
                      if air is not None else (0.0, 0.0, 0.0))
        along = sum((float(d.qvel[a]) - w) * float(turn[a, 1]) for a, w in enumerate((wx, wy, wz)))
        lift = self.lift * inflow(along if coll >= 0.0 else -along, w_m)
        self.tail = f_t = K_TAIL * w_t * w_t * tail_p
        ab = (K_CYC * w_m * w_m * cyc_p, TAIL_M * f_t - _rotor_drag(coll, cyc_p, cyc_r, w_m),
              K_CYC * w_m * w_m * cyc_r)
        dk, qv, xf = 0.5 * RHO * BODY_CDA, d.qvel, d.xfrc_applied
        for _ in range(max(1, int(round(dt / STEP_S)))):
            t = d.xmat[f].reshape(3, 3)
            vx, vy, vz = float(qv[0]) - wx, float(qv[1]) - wy, float(qv[2]) - wz
            dr = dk * math.sqrt(vx * vx + vy * vy + vz * vz)
            xf[f, :3] = (lift * float(t[0, 1]) + f_t * float(t[0, 0]) - dr * vx,
                         lift * float(t[1, 1]) + f_t * float(t[1, 0]) - dr * vy,
                         lift * float(t[2, 1]) + f_t * float(t[2, 0]) - dr * vz)
            xf[f, 3:] = tuple(float(t[r, 0]) * ab[0] + float(t[r, 1]) * ab[1]
                              + float(t[r, 2]) * ab[2] for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)


def _mat_mul(a, b):
    return tuple(tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
                       for j in range(3)) for i in range(3))


def _mat_t(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(rad):
    c, s = math.cos(rad), math.sin(rad)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _align_axis(from_u, to_u, mat):
    w = cross(from_u, to_u)
    d = 1.0 + from_u[0] * to_u[0] + from_u[1] * to_u[1] + from_u[2] * to_u[2]
    if d <= 1e-4:
        return _mat_mul(_rot_x(math.pi), mat)
    cols = []
    for col in range(3):
        v = (mat[0][col], mat[1][col], mat[2][col])
        u = cross(w, v)
        wu = cross(w, u)
        cols.append(tuple(v[k] + u[k] + wu[k] / d for k in range(3)))
    return _mat_t(cols)


def _so3_error(r_cur, r_des):
    e = _mat_mul(_mat_t(r_cur), r_des)
    vx, vy, vz = 0.5 * (e[2][1] - e[1][2]), 0.5 * (e[0][2] - e[2][0]), 0.5 * (e[1][0] - e[0][1])
    sin_a = math.sqrt(vx * vx + vy * vy + vz * vz)
    cos_a = max(-1.0, min(1.0, 0.5 * (e[0][0] + e[1][1] + e[2][2] - 1.0)))
    if sin_a <= 1e-4 and cos_a < 0.0:
        return (-math.pi * (math.sqrt(max(0.0, 0.5 * (e[0][0] + 1.0))) or 1.0),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[1][1] + 1.0))),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[2][2] + 1.0))))
    scale = math.atan2(sin_a, cos_a) / sin_a if sin_a > 1e-5 else 1.0
    return vx * scale, vy * scale, vz * scale


def _turn_heading(heading, rate, dt):
    half = 0.5 * rate * dt
    c0, s0 = heading
    c = ((1.0 - half * half) * c0 - 2.0 * half * s0) / (1.0 + half * half)
    s = (2.0 * half * c0 + (1.0 - half * half) * s0) / (1.0 + half * half)
    n_h = math.hypot(c, s) or 1.0
    return c / n_h, s / n_h


class Answer:

    """The 3D helicopter controller's answer for a pass."""

    def __init__(self, speeds, controls, thrust, head, wind, bound=0):
        self.speeds, self.controls, self.thrust = speeds, controls, thrust
        self.head, self.wind, self.bound = head, wind, bound


class Core:

    """The 3D helicopter's SO(3) attitude, signed collective and wind-observer core."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset observer and trajectory state."""
        self.vel_hat, self.push_hat, self.wind = [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.spin_hat, self.turn_hat = [0.0] * 3, [0.0] * 3
        self.begun, self.flip_ang, self.roll_ang = False, 0.0, 0.0
        self.piro_ang, self.tic_phase = 0.0, 0.0
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
        alpha = 0.5 * RHO * BODY_CDA / MASS_KG * fast
        sp = state.get('speeds', (0.0, 0.0))
        w_m = float(sp[0]) if sp[0] > 1.0 else self.Controls[0]
        w_t = float(sp[1]) if sp[1] > 1.0 else self.Controls[1]
        coll, cyc_p, cyc_r, tail_p = state.get('controls', self.Controls[2:])
        up, side = (turn[0][1], turn[1][1], turn[2][1]), (turn[0][0], turn[1][0], turn[2][0])
        s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if coll >= 0.0 else -1.0)
        lift, tail = float(state.get('lift', 0.0)) * inflow(s_along, w_m), float(state.get('tail', 0.0))
        for a in range(3):
            off = vel[a] - self.vel_hat[a]
            acc = (lift * up[a] + tail * side[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
                + self.push_hat[a] - alpha * u[a]
            self.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
            self.push_hat[a] += dt * WO_PULL * WO_PULL * off
        inv_fast = (1.0 / fast) if fast > MIN_AIR_M_S else 0.0
        uh = tuple(u[a] * inv_fast for a in range(3))
        gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_m * ROTOR_R)
                 if w_m > 1.0 and abs(s_along) < 4.0 * w_m * ROTOR_R else 0.0)
        m = [[alpha * ((1.0 if a == b else 0.0) + uh[a] * uh[b]) + gamma * up[a] * up[b]
              for b in range(3)] for a in range(3)]
        for a in (0, 2):
            mp = sum(m[a][b] * self.push_hat[b] for b in range(3))
            dw = dt * WO_WIND * mp / (m[0][a] ** 2 + m[1][a] ** 2 + m[2][a] ** 2 + WIND_REG)
            self.wind[a] += dw
            for b in range(3):
                self.push_hat[b] -= m[b][a] * dw
        q_m = _rotor_drag(coll, cyc_p, cyc_r, w_m)
        tau = (K_CYC * w_m * w_m * cyc_p - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
               TAIL_M * K_TAIL * w_t * w_t * tail_p - q_m - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
               K_CYC * w_m * w_m * cyc_r - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
        for a in range(3):
            e = spin[a] - self.spin_hat[a]
            self.spin_hat[a] += dt * (tau[a] / INERTIA[a] + self.turn_hat[a] + 2.0 * WO_TURN * e)
            self.turn_hat[a] += dt * WO_TURN * WO_TURN * e

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_stop, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        tictoc, inv = (TICTOC_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(tictoc) > 1e-6
        if stunt:
            d_flip, d_roll = -math.tau * flips * dt, -math.tau * rolls * dt
            if abs(tictoc) > 1e-6:
                if self.tic_phase == 0.0:
                    self.tic_phase = math.pi if turn[1][1] > 0.0 else 0.0
                self.flip_ang = self.roll_ang = self.piro_ang = 0.0
                self.tic_phase += math.tau * TICTOC_HZ * dt
                rock = _rot_z if nose_val <= -2.5 else _rot_x
                r_des = _mat_mul(_rot_y(yaw), rock(0.5 * math.pi + tictoc * math.cos(self.tic_phase)))
                r_next = _mat_mul(_rot_y(yaw + rate * dt), rock(
                    0.5 * math.pi + tictoc * math.cos(self.tic_phase + math.tau * TICTOC_HZ * dt)))
            else:
                self.tic_phase = 0.0
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
            ay_stunt = GRAVITY * 2.0 * r11 if abs(tictoc) <= 1e-6 else ay_des * r11 / max(r11 * r11, 0.34)
            f_des = MASS_KG * (ay_stunt + 0.35 * (ax_des * turn[0][1] + az_des * turn[2][1]))
            return r_des, w_ff, f_des, stunt, inv

        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
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
        drag = 0.5 * RHO * BODY_CDA / MASS_KG * math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
        air_acc = [self.push_hat[a] - drag * u[a] for a in range(3)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        rate = math.radians(row.get('turn', 0.0))
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        up_pull, law['doing'] = aerokin.climb(row, state['h'], state['v'], reach - GRAVITY)

        if row['height'] < 0.0:
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            w_ask = GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            w_t_ask = TAIL_GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            if limits:
                w_ask, w_t_ask = min(limits[0], w_ask), min(limits[1], w_t_ask)
            controls = (0.0, 0.0, 0.0, 0.0)
            self.Controls = (w_ask, w_t_ask) + controls
            law['held'] = False
            self.out = Answer((w_ask, w_t_ask), controls, (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out

        light_val, tail_prev = float(row.get('light', 0.5)), float(state.get('tail', 0.0))
        acc_des = (along[0] - air_acc[0] - tail_prev * turn[0][0] / MASS_KG,
                   max(GRAVITY + up_pull, light_val * GRAVITY) - air_acc[1],
                   along[1] - air_acc[2] - tail_prev * turn[2][0] / MASS_KG)
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
        w_tail_top = limits[1] if limits else w_top
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, inflow(along_u if f_des >= 0.0 else -along_u, w_ref))
        w_ask = min(w_top, max(w_gov, math.sqrt(abs(f_comp) / K_THRUST)))
        w_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_ask)
        coll = max(-1.0, min(1.0, f_comp / (K_THRUST * w_cur * w_cur)))
        cyc_p = max(-1.0, min(1.0, tau_x / (K_CYC * w_cur * w_cur)))
        cyc_r = max(-1.0, min(1.0, tau_z / (K_CYC * w_cur * w_cur)))

        f_tail_des = (tau_y + _rotor_drag(coll, cyc_p, cyc_r, w_cur)) / TAIL_M
        w_t_ask = min(w_tail_top, max(TAIL_GOV_RAD_S, math.sqrt(abs(f_tail_des) / K_TAIL),
                                      TAIL_RATIO * w_ask))
        w_t_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_t_ask)
        tail_p = max(-1.0, min(1.0, f_tail_des / (K_TAIL * w_t_cur * w_t_cur)))

        controls = (coll, cyc_p, cyc_r, tail_p)
        self.Controls = (w_ask, w_t_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        law['held'] = bool(
            not stunt and abs(row['height'] - state['h']) <= aerokin.HELD_M
            and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(float(state['at'][0]) - law['spot'][0],
                           float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and sign_ok)
        law['sequence'] += 1
        bound = int(abs(coll) >= 0.999) | (int(abs(tail_p) >= 0.999) << 1)
        self.out = Answer((w_ask, w_t_ask), controls,
                          (K_THRUST * w_cur * w_cur * coll, K_TAIL * w_t_cur * w_t_cur * tail_p),
                          (c, s), tuple(self.wind), bound)
        return self.out
