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

from machine import aerokin, aerolisp, grounds, pull, quad
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

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
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
FLOOR_M, HOVER_M, TOP_M = (
    KINEMATICS['envelope'][k] for k in ('floor_height', 'hover_height', 'top_height'))
PULL_MAX = K_THRUST * (GOV_RAD_S * STUNT_BOOST) ** 2 / MASS_KG
COURSE_SPHERE = 2.0 * GRAVITY
TICTOC_HZ, TICTOC_RAD = (
    KINEMATICS['oscillator'][k] for k in ('tictoc_frequency', 'tictoc_amplitude'))
FUNNEL, SPIRAL, FLIP_IN, INVERTED, INV_PIRO, TICTOC, AILERON_TICTOC, PIRO_FLIP, CHAOS, HURRICANE, UNWIND = (
    STUNTS[k] for k in ('funnel', 'spiral', 'flip_in', 'inverted', 'inv_piro', 'tictoc',
                        'aileron_tictoc', 'piro_flip', 'chaos', 'hurricane', 'unwind'))


def drive_loads(controls):
    """The two drives' load scales `(main, tail)` for `controls`."""
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


def _rotor_inflow(along, v_horiz, w_m):
    return inflow(along, w_m) * (1.0 + 0.65 * min(0.35, v_horiz / max(w_m * ROTOR_R, 8.0)))


def _init_sky(sky, xml, rest):
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
                 for g, n in enumerate(names) if n.startswith('skid')}
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


def _mat_mul(a, b):
    return tuple(tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3)) for i in range(3))


def _mat_t(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _rot_x(r):
    c, s = math.cos(r), math.sin(r)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(r):
    c, s = math.cos(r), math.sin(r)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(r):
    c, s = math.cos(r), math.sin(r)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _align_axis(from_u, to_u, mat):
    w, d = cross(from_u, to_u), 1.0 + sum(a * b for a, b in zip(from_u, to_u))
    if d <= 1e-4:
        return _mat_mul(_rot_x(math.pi), mat)
    cols = []
    for col in range(3):
        v = (mat[0][col], mat[1][col], mat[2][col])
        u = cross(w, v)
        cols.append(tuple(v[k] + u[k] + cross(w, u)[k] / d for k in range(3)))
    return _mat_t(cols)


def _so3_error(r_cur, r_des):
    e = _mat_mul(_mat_t(r_cur), r_des)
    vx, vy, vz = 0.5 * (e[2][1] - e[1][2]), 0.5 * (e[0][2] - e[2][0]), 0.5 * (e[1][0] - e[0][1])
    sa, ca = math.hypot(vx, vy, vz), max(-1.0, min(1.0, 0.5 * (e[0][0] + e[1][1] + e[2][2] - 1.0)))
    if sa <= 1e-4 and ca < 0.0:
        return (-math.pi * (math.sqrt(max(0.0, 0.5 * (e[0][0] + 1.0))) or 1.0),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[1][1] + 1.0))),
                math.pi * math.sqrt(max(0.0, 0.5 * (e[2][2] + 1.0))))
    sc = math.atan2(sa, ca) / sa if sa > 1e-5 else 1.0
    return vx * sc, vy * sc, vz * sc


def _turn_heading(heading, rate, dt):
    h, (c0, s0) = 0.5 * rate * dt, heading
    c, s = (1.0 - h * h) * c0 - 2.0 * h * s0, 2.0 * h * c0 + (1.0 - h * h) * s0
    n = math.hypot(c, s) or 1.0
    return c / n, s / n


def _course_rate(heading, row):
    rate = math.radians(row.get('turn', 0.0))
    if row.get('height', 9.0) < 2.0 and abs(rate) <= 1e-6 and abs(row.get('slide', 0.0)) <= 1e-6 and row.get('nose', 1.0) > 0.0:
        err = (math.radians(grounds.GATES[0][3]) - math.atan2(heading[1], heading[0]) + math.pi) % math.tau - math.pi
        return max(-2.4, min(2.4, 3.0 * err))
    return rate


def _observe_wind_spin(core, u, fast, alpha, gamma, up, spin, tau, inertia, dt):
    uh = tuple(u[a] / fast if fast > MIN_AIR_M_S else 0.0 for a in range(3))
    m = [[alpha * ((1.0 if a == b else 0.0) + uh[a] * uh[b]) + gamma * up[a] * up[b] for b in range(3)] for a in range(3)]
    for a in (0, 2):
        dw = dt * WO_WIND * sum(m[a][b] * core.push_hat[b] for b in range(3)) / (m[0][a] ** 2 + m[1][a] ** 2 + m[2][a] ** 2 + WIND_REG)
        core.wind[a] += dw
        for b in range(3):
            core.push_hat[b] -= m[b][a] * dw
    for a in range(3):
        e = spin[a] - core.spin_hat[a]
        core.spin_hat[a] += dt * (tau[a] / inertia[a] + core.turn_hat[a] + 2.0 * WO_TURN * e)
        core.turn_hat[a] = max(-12.0, min(12.0, (1.0 - 2.5 * dt) * core.turn_hat[a] + dt * WO_TURN * WO_TURN * e))


def _stunt_step(core, flips, rolls, nose_val, osc_amp, osc_hz, turn, yaw, rate, acc_des, dt):
    core.r_prev, core.up_filt, core.w_ff = None, None, (0.0, 0.0, 0.0)
    df, dr = -math.tau * flips * dt, -math.tau * rolls * dt
    if abs(osc_amp) > 1e-6:
        if core.tic_phase == 0.0:
            core.tic_phase = math.pi if turn[1][1] > 0.0 else 0.0
        core.flip_ang = core.roll_ang = core.piro_ang = 0.0
        core.tic_phase += math.tau * osc_hz * dt
        rk = _rot_z if nose_val <= -2.5 else _rot_x
        r_des = _mat_mul(_rot_y(yaw), rk(0.5 * math.pi + osc_amp * math.cos(core.tic_phase)))
        r_next = _mat_mul(_rot_y(yaw + rate * dt), rk(0.5 * math.pi + osc_amp * math.cos(core.tic_phase + math.tau * osc_hz * dt)))
    else:
        core.tic_phase = 0.0
        core.flip_ang, core.roll_ang, core.piro_ang = core.flip_ang + df, core.roll_ang + dr, core.piro_ang + rate * dt
        b = _rot_y(yaw - core.piro_ang)
        r_des = _mat_mul(_mat_mul(b, _rot_x(core.flip_ang)), _mat_mul(_rot_z(core.roll_ang), _rot_y(core.piro_ang)))
        r_next = _mat_mul(_mat_mul(b, _rot_x(core.flip_ang + df)), _mat_mul(_rot_z(core.roll_ang + dr), _rot_y(core.piro_ang + rate * dt)))
    w_ff = tuple(x / max(dt, 1e-6) for x in _so3_error(r_des, r_next))
    r11, (ax, ay, az) = turn[1][1], acc_des
    ay_s = GRAVITY * 2.0 * r11 if abs(osc_amp) <= 1e-6 else ay * r11 / max(r11 * r11, 0.34)
    return r_des, w_ff, MASS_KG * (ay_s + 0.35 * (ax * turn[0][1] + az * turn[2][1]))


def _smooth_target(core, want_up, sign, yaw, rate, pace, dt):
    w_ff, a, aw = (0.0, sign * rate, 0.0), min(1.0, 24.0 * dt), min(1.0, 12.0 * dt)
    if core.up_filt is not None and dt > 1e-6:
        ub = [core.up_filt[k] + a * (want_up[k] - core.up_filt[k]) for k in range(3)]
        un = math.hypot(*ub) or 1.0
        want_up = (ub[0] / un, ub[1] / un, ub[2] / un)
    r0 = r_des = _align_axis((0.0, sign, 0.0), want_up, _mat_mul(_rot_y(yaw), _rot_x(core.flip_ang)))
    if pace > 20.0 and core.r_prev is not None:
        r_tr = _mat_mul(_align_axis((core.r_prev[0][1], core.r_prev[1][1], core.r_prev[2][1]), want_up, core.r_prev), _rot_y(sign * rate * dt))
        r_des = _mat_mul(r_tr, _rot_y(aw * _so3_error(r_tr, r0)[1])) if sign * want_up[1] > 0.25 else r_tr
        raw = tuple(max(-5.5, min(5.5, 0.50 * x / max(dt, 1e-6))) for x in _so3_error(core.r_prev, r_des))
        w_ff = tuple(core.w_ff[k] + aw * (raw[k] - core.w_ff[k]) for k in range(3))
    core.up_filt, core.r_prev, core.w_ff = want_up, r_des, w_ff
    return r_des, w_ff


def _so3_torques(core, turn, r_des, w_ff, spin, inertia, kp, kd):
    er, ew = _so3_error(turn, r_des), (w_ff[0] - spin[0], w_ff[1] - spin[1], w_ff[2] - spin[2])
    return (inertia[0] * (kp[0] * er[0] + kd[0] * ew[0] - core.turn_hat[0]) + (inertia[2] - inertia[1]) * spin[1] * spin[2],
            inertia[1] * (kp[1] * er[1] + kd[1] * ew[1] - core.turn_hat[1]) + (inertia[0] - inertia[2]) * spin[2] * spin[0],
            inertia[2] * (kp[2] * er[2] + kd[2] * ew[2] - core.turn_hat[2]) + (inertia[1] - inertia[0]) * spin[0] * spin[1])


class Answer:

    """Controller answer for a pass."""

    def __init__(self, speeds, controls, thrust, head, wind, bound=0):
        self.speeds, self.controls, self.thrust, self.head, self.wind, self.bound = speeds, controls, thrust, head, wind, bound


class Core:

    """SO(3) attitude, signed collective and wind-observer core."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.vel_hat, self.push_hat, self.wind, self.spin_hat, self.turn_hat = ([0.0] * 3 for _ in range(5))
        self.begun, self.flip_ang, self.roll_ang, self.piro_ang, self.tic_phase = False, 0.0, 0.0, 0.0, 0.0
        self.r_prev, self.up_filt, self.w_ff, self.up_pull = None, None, (0.0, 0.0, 0.0), 0.0
        self.Controls = (IDLE_RAD_S, IDLE_RAD_S, 0.0, 0.0, 0.0, 0.0)
        self.out = Answer((IDLE_RAD_S, IDLE_RAD_S), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0, 0.0))

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
        fast, sp = math.hypot(*u), state.get('speeds', (0.0, 0.0))
        alpha = 0.5 * RHO * BODY_CDA / MASS_KG * fast
        w_m, w_t = (float(sp[k]) if sp[k] > 1.0 else self.Controls[k] for k in (0, 1))
        coll, cyc_p, cyc_r, tail_p = state.get('controls', self.Controls[2:])
        up, side = (turn[0][1], turn[1][1], turn[2][1]), (turn[0][0], turn[1][0], turn[2][0])
        s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if coll >= 0.0 else -1.0)
        lift = float(state.get('lift', 0.0)) * _rotor_inflow(s_along, math.hypot(u[0], u[2]), w_m)
        tail = float(state.get('tail', 0.0))
        for a in range(3):
            off = vel[a] - self.vel_hat[a]
            acc = (lift * up[a] + tail * side[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) + self.push_hat[a] - alpha * u[a]
            self.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
            self.push_hat[a] += dt * WO_PULL * WO_PULL * off
        gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_m * ROTOR_R)
                 if w_m > 1.0 and abs(s_along) < 4.0 * w_m * ROTOR_R else 0.0)
        q_m = _rotor_drag(coll, cyc_p, cyc_r, w_m)
        tau = (K_CYC * w_m * w_m * cyc_p - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
               TAIL_M * K_TAIL * w_t * w_t * tail_p - q_m - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
               K_CYC * w_m * w_m * cyc_r - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
        _observe_wind_spin(self, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt)

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_thrust, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        tictoc, inv = (TICTOC_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(tictoc) > 1e-6
        if stunt:
            return _stunt_step(self, flips, rolls, nose_val, tictoc, TICTOC_HZ, turn, yaw, rate, acc_des, dt) + (stunt, inv)
        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        s_f = 1.0 if (ay_des if inv >= 0.5 else ay_thrust) >= 0.0 else -1.0
        want_up = (sign * s_f * ax_des / a_norm, sign * ay_des / a_norm, sign * s_f * az_des / a_norm)
        r_des, w_ff = _smooth_target(self, want_up, sign, yaw, rate, float(row.get('pace', 0.0)), dt)
        ay_use = ay_des if inv >= 0.5 else ay_thrust
        return r_des, w_ff, MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1]), stunt, inv

    def plan(self, law, row, state, dt, pull_max):
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), _course_rate((c0, s0), row), dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.up_pull, law['doing'] = aerokin.climb(row, state['h'], state['v'], reach - GRAVITY)
        along = aerokin.lean(law, row, state['at'], state['vel'], dt, min(row['lean'], math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))))
        law['heading'] = (c0, s0)
        return along

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        self._observe(state, dt)
        turn = [[float(state['turn'][r][c]) for c in range(3)] for r in range(3)]
        spin, vel = [float(x) for x in state['spin']], [float(x) for x in state['vel']]
        u = [vel[a] - self.wind[a] for a in range(3)]
        drag = 0.5 * RHO * BODY_CDA / MASS_KG * math.hypot(*u)
        air_acc = [self.push_hat[a] - drag * u[a] for a in range(3)]
        along = self.plan(law, row, state, dt, pull_max) if along is None else along
        rate = _course_rate(law['heading'], row)
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw, reach, up_pull = math.atan2(s, c), max(aerokin.LEAST * GRAVITY, pull_max), self.up_pull
        if row['height'] < 0.0:
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff = None, None, (0.0, 0.0, 0.0)
            w_ask, w_t_ask = (GOV_RAD_S, TAIL_GOV_RAD_S) if row['height'] > -0.1 else (IDLE_RAD_S, IDLE_RAD_S)
            if limits:
                w_ask, w_t_ask = min(limits[0], w_ask), min(limits[1], w_t_ask)
            self.Controls, law['held'] = (w_ask, w_t_ask, 0.0, 0.0, 0.0, 0.0), False
            self.out = Answer((w_ask, w_t_ask), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out
        light_val, tail_prev = float(row.get('light', 0.5)), float(state.get('tail', 0.0))
        ay_des = max(GRAVITY + up_pull - air_acc[1], light_val * GRAVITY)
        acc_des = (along[0] - air_acc[0] - tail_prev * turn[0][0] / MASS_KG, ay_des,
                   along[1] - air_acc[2] - tail_prev * turn[2][0] / MASS_KG)
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        in_burn = row['height'] <= FLOOR_M + 0.05 and (state['h'] < 22.0 or state['v'] <= -0.5 * row['pace'])
        ay_thrust = ay_stop if in_burn else max(GRAVITY + up_pull, -1.5 * GRAVITY) - air_acc[1]
        r_des, w_ff, f_des, stunt, inv = self._target_attitude(
            row, state, turn, yaw, rate, acc_des, ay_thrust, dt)

        f_limit = MASS_KG * reach
        f_des = max(-f_limit, min(f_limit, f_des))
        tau_x, tau_y, tau_z = _so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD))

        w_top, w_tail_top = (limits[0] if limits else pull.held(pull.I_MAX)), (limits[1] if limits else pull.held(pull.I_MAX))
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S
                             or math.hypot(*along) > 0.5 * GRAVITY else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _rotor_inflow(along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_ask = min(w_top, w_gov)
        w_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_ask)
        coll = max(-1.0, min(1.0, f_comp / (K_THRUST * w_cur * w_cur)))
        cyc_p, cyc_r = (max(-1.0, min(1.0, t / (K_CYC * w_cur * w_cur))) for t in (tau_x, tau_z))
        f_tail_des = (tau_y + _rotor_drag(coll, cyc_p, cyc_r, w_cur)) / TAIL_M
        w_t_ask = min(w_tail_top, max(TAIL_GOV_RAD_S, TAIL_RATIO * w_ask))
        w_t_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_t_ask)
        tail_p = max(-1.0, min(1.0, f_tail_des / (K_TAIL * w_t_cur * w_t_cur)))
        controls = (coll, cyc_p, cyc_r, tail_p)
        self.Controls = (w_ask, w_t_ask) + controls
        law['held'] = bool(
            not stunt and abs(row['height'] - state['h']) <= aerokin.HELD_M and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(float(state['at'][0]) - law['spot'][0], float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD))
        law['sequence'] += 1
        self.out = Answer((w_ask, w_t_ask), controls, (K_THRUST * w_cur * w_cur * coll, K_TAIL * w_t_cur * w_t_cur * tail_p),
                          (c, s), tuple(self.wind), int(abs(coll) >= 0.999) | (int(abs(tail_p) >= 0.999) << 1))
        return self.out
