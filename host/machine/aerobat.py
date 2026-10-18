"""The unlimited 3D RC aerobatic monoplane in MuJoCo: prop-hang, high-deflection surfaces, stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

An Edge 540 style 3D monoplane (`aerobat.lisp`) on two 63100 drives, `PROP` the tractor
propeller and `VECT` the coaxial contra stage: propwash over oversized surfaces gives attitude
authority from zero airspeed, a prop-hang to the race course.
"""
import math
import os

from machine import aerokin, aerolisp, grounds, pull, quad
from machine.aerobatics import HOVER
from machine.helicopter import (Answer, _course_rate, _init_sky, _mat_mul, _observe_wind_spin,
                                _rot_x, _rot_y, _smooth_target, _so3_torques, _stunt_step,
                                _turn_heading)
from machine.wingborne import AIRBORNE_M, PITCH_REST, Wing, plane, run
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
TAIL_ARM, STAB_AREA, FIN_AREA = _AS['tail_arm'], _AS['stab_area'], _AS['fin_area']
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), 0.5 * WING_SPAN
#: The stabiliser's span, m.
STAB_SPAN = math.sqrt(STAB_AREA * WING_SPAN / WING_CHORD)
GEAR_MAIN_Z = (STRUT_Z[1] * math.cos(PITCH_REST) + SKID_M) / math.sin(PITCH_REST)
GEAR_TAIL_Z = (STRUT_Z[0] * math.cos(PITCH_REST) + SKID_M) / math.sin(PITCH_REST)
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
TAKEOFF, CLIMBOUT, FINAL = STUNTS['takeoff'], STUNTS['climbout'], STUNTS['final']
DIVE_M_S = 4.0
#: The flare's nose, the height its tail wheel touches at.
HARRIER_RAD = math.radians(30.0)
TOUCH_M = TAIL_ARM * math.sin(HARRIER_RAD) + 0.145 * math.cos(HARRIER_RAD) - SKID_M - FOOT_M + 0.02
#: The pull the law plans on, m/s^2.
PULL_MAX = K_THRUST / MASS_KG * (GOV_RAD_S * STUNT_BOOST) ** 2
WING = Wing(MASS_KG, GRAVITY, RHO, WING_AREA, CL_ALPHA, WING_SPAN, STAB_AREA, FIN_AREA, TAIL_ARM)
ROTATE_M_S, RUNWAY_M = WING.rotate, WING.runway(HOVER['lean'])
RUNWAY_RAD = math.radians(grounds.GATES[0][3] + 50.0)


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
    m = ('<geom name="fuselage" type="box" pos="0 0.04 0" size="0.045 %g 0.04" contype="1" '
         'conaffinity="1"/><geom name="wing" type="box" pos="0 0.06 0" size="%g %g 0.010" '
         'contype="1" conaffinity="1"/><geom name="prop" type="cylinder" pos="0 %g 0" '
         'size="%g 0.008" euler="90 0 0" contype="1" conaffinity="1"/>'
         '<geom name="hstab" type="box" pos="0 %g 0" size="%g 0.075 0.008" contype="1" '
         'conaffinity="1"/><geom name="vfin" type="box" pos="0 %g -0.08" '
         'size="0.008 0.07 0.08" contype="1" conaffinity="1"/>'
         % (PROP_Z - 0.04, 0.5 * WING_SPAN, 0.5 * WING_CHORD, PROP_Z, PROP_R, -TAIL_ARM,
            STAB_SPAN / 2.0, -TAIL_ARM))
    wheels = ((STRUT_Z[0], GEAR_TAIL_Z, 0.45), (STRUT_Z[1], GEAR_MAIN_Z, 1.0))
    for k, (sx, (by, bz, x_sc)) in enumerate((sx, w) for sx in (-SKID_X, SKID_X) for w in wheels):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'priority="1" solref="0.03 1" friction="0.3 0.002 0.0001"/>'
              % (k, x_sc * sx, by, bz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="%g %g %g"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((head, -RUNWAY_M * math.sin(RUNWAY_RAD), SKID_M + FOOT_M,
                                        -RUNWAY_M * math.cos(RUNWAY_RAD), MASS_KG) + INERTIA + (m,)))


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
        self.rudder_force = 0.0
        _init_sky(self, mjcf(things), SKID_M + FOOT_M)
        self._rest_pose()

    def _rest_pose(self):
        """On its wheels at the runway's end, its nose up the runway: R_y(RUNWAY_RAD) R_x(PITCH_REST)."""
        cy, sy = math.cos(0.5 * RUNWAY_RAD), math.sin(0.5 * RUNWAY_RAD)
        cx, sx = math.cos(0.5 * PITCH_REST), math.sin(0.5 * PITCH_REST)
        self.data.qpos[3:7] = (cy * cx, cy * sx, sy * cx, -sy * sx)
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """Return the aerobat to its landing gear at the spot, still, nothing struck."""
        super().reset()
        self.rudder_force, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
        self._rest_pose()

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
            lx, ly, lz = WING.lift((vx, vy, vz), t)
            dr = dk * math.sqrt(vx * vx + vy * vy + vz * vz)
            xf[f, :3] = (lift * float(t[0, 1]) + lx - dr * vx,
                         lift * float(t[1, 1]) + ly - dr * vy,
                         lift * float(t[2, 1]) + lz - dr * vz)
            a0, a1, a2 = WING.moments((vx, vy, vz), t, qv)
            xf[f, 3:] = tuple(float(t[r, 0]) * (ab[0] + a0) + float(t[r, 1]) * (ab[1] + a1)
                              + float(t[r, 2]) * (ab[2] + a2) for r in range(3))
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
        self.piro_ang, self.tic_phase = 0.0, 0.0
        self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
        self.rotated, self.wingborne = False, 0.0
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
        along = sum(u[a] * up[a] for a in range(3))
        s_along = along * (1.0 if throt >= 0.0 else -1.0)
        lift = float(state.get('lift', 0.0)) * _aero_inflow(s_along, math.hypot(u[0], u[2]), w_p)
        w_lift = WING.lift(u, turn)
        # Wing-borne, a push learnt is the wing's own.
        self.wingborne = w = (min(1.0, max(0.0, along / ROTATE_M_S)) ** 2
                              * min(1.0, max(0.0, 1.0 + vel[1] / DIVE_M_S)))
        for a in range(3):
            off = vel[a] - self.vel_hat[a]
            acc = (lift * up[a] + w_lift[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
                + self.push_hat[a] - alpha * u[a]
            self.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
            self.push_hat[a] += dt * WO_PULL * WO_PULL * off * (1.0 - w)
        gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_p * PROP_R)
                 if w_p > 1.0 and abs(s_along) < 4.0 * w_p * PROP_R else 0.0)
        q_p = _prop_drag(throt, elev, ail, w_p)
        tau = (K_PITCH * w_p * w_p * elev - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
               K_YAW * w_v * w_v * rud - q_p - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
               K_ROLL * w_p * w_p * ail - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
        _observe_wind_spin(self, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt)

    def _target_attitude(self, law, row, state, turn, yaw, rate, acc_des, ay_thrust, ay_stop, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        wfall, inv = (WATERFALL_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(wfall) > 1e-6
        if stunt:
            self.airborne = True
            r_des, w_ff, f_des = _stunt_step(
                self, flips, rolls, nose_val, wfall, WATERFALL_HZ, turn, yaw, rate, acc_des, dt)
            return r_des, w_ff, f_des, stunt, inv

        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        rh, pace, h = float(row['height']), float(row.get('pace', 0.0)), float(state['h'])
        c, s = math.cos(yaw), math.sin(yaw)
        v_fwd = s * float(state['vel'][0]) + c * float(state['vel'][2])
        if not self.airborne and rh >= 0.0 and h >= AIRBORNE_M:
            self.airborne, brake = True, max(1.0, float(row.get('lean', GRAVITY)))
            ahead = -(float(state['at'][0]) * s + float(state['at'][2]) * c)
            law['spot'] = ([0.0, 0.0] if 0.0 < ahead < 2.5 * RUNWAY_M else
                           [float(state['at'][a]) + float(state['vel'][a]) * abs(v_fwd) / (2.0 * brake)
                            for a in (0, 2)])
        if not self.airborne and rh >= 0.0 and pace < 20.0:
            return run(WING, self, v_fwd, h, yaw, rate) + (stunt, inv)

        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        s_f = 1.0 if (ay_des if inv >= 0.5 else ay_thrust) >= 0.0 else -1.0
        want_up = [sign * s_f * ax_des / a_norm, sign * ay_des / a_norm, sign * s_f * az_des / a_norm]
        ay_use = ay_des if inv >= 0.5 else ay_thrust
        f_des = MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1])
        u = [float(state['vel'][a]) - self.wind[a] for a in range(3)]
        r_des, want_up = plane(WING, turn, (ax_des, ay_des, az_des), u, self.wingborne, want_up)
        if r_des is not None:
            self.r_prev, self.up_filt, self.w_ff = r_des, tuple(want_up), (0.0, 0.0, 0.0)
            return r_des, (0.0, 0.0, 0.0), f_des, stunt, inv
        r_des, w_ff = _smooth_target(self, tuple(want_up), sign, yaw, rate, pace, dt)
        return r_des, w_ff, f_des, stunt, inv

    def plan(self, law, row, state, dt, pull_max):
        """Advance outer horizontal kinematics `aerokin.lean` across `dt` s."""
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), _course_rate((c0, s0), row), dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.up_pull, law['doing'] = aerokin.climb(
            row, state['h'], state['v'], reach - GRAVITY,
            aerokin.spooled(self.out.speeds[0], math.sqrt(reach * MASS_KG / K_THRUST), IDLE_RAD_S), dt)
        beside = math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))
        along = aerokin.lean(law, row, state['at'], state['vel'], dt, min(row['lean'], beside))
        if not self.airborne and row['height'] >= 0.0:
            law['spot'] = [float(state['at'][0]), float(state['at'][2])]
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
        air_acc = [(1.0 - self.wingborne) * self.push_hat[a] - drag * u[a] for a in range(3)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        # Asked no turn under 2 m in the air, the nose comes round to the runway's heading.
        rate = math.radians(float(row.get('turn', 0.0))) if self.airborne else 0.0
        if self.airborne and abs(rate) <= 1e-6 and row['height'] < 2.0 and abs(row.get('slide', 0.0)) <= 1e-6:
            err = (RUNWAY_RAD - math.atan2(law['heading'][1], law['heading'][0]) + math.pi) % math.tau - math.pi
            rate = max(-2.4, min(2.4, 3.0 * err))
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach, up_pull = max(aerokin.LEAST * GRAVITY, pull_max), self.up_pull

        if row['height'] < 0.0 and (row['height'] <= -0.1 or not self.airborne):
            if state['h'] < 0.03 and max(abs(x) for x in vel) < 0.05:
                law['heading'] = c, s = math.cos(RUNWAY_RAD), math.sin(RUNWAY_RAD)
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
            self.rotated = False
            law['spot'] = [float(state['at'][0]), float(state['at'][2])]
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
        ay_des = max(GRAVITY + up_pull - air_acc[1], light_val * GRAVITY)
        acc_des = (along[0] - air_acc[0], ay_des, along[1] - air_acc[2])
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        in_burn = row['height'] <= FLOOR_M + 0.05 and (state['h'] < 22.0 or state['v'] <= -0.5 * row['pace'])
        ay_thrust = ay_stop if in_burn else max(GRAVITY + up_pull, -1.5 * GRAVITY) - air_acc[1]
        if row['height'] < 0.0:
            v_fwd = s * vel[0] + c * vel[2]
            flaring = state['h'] > TOUCH_M
            r_des = _mat_mul(_rot_y(yaw), _rot_x(0.5 * math.pi - HARRIER_RAD if flaring else PITCH_REST))
            w_ff, stunt, inv, f_des = (0.0, rate, 0.0), False, 0.0, 0.0
            if state['h'] <= 0.025 and abs(v_fwd) <= 0.03 and row['height'] <= -0.012:
                self.airborne = False
                law['spot'] = [float(state['at'][0]), float(state['at'][2])]
        else:
            r_des, w_ff, f_des, stunt, inv = self._target_attitude(
                law, row, state, turn, yaw, rate, acc_des, ay_thrust, ay_stop, dt)

        f_limit = MASS_KG * reach
        f_des = max(0.0, min(f_limit, f_des))
        tau_x, tau_y, tau_z = _so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD))

        w_top = limits[0] if limits else pull.held(pull.I_MAX)
        w_vect_top = limits[1] if limits else w_top
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _aero_inflow(
            along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_p_ask = min(w_top, w_gov)
        w_p_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_p_ask)
        throt = max(-1.0, min(1.0, f_comp / (K_THRUST * w_p_cur * w_p_cur)))
        elev = max(-1.0, min(1.0, tau_x / (K_PITCH * w_p_cur * w_p_cur)))
        ail = max(-1.0, min(1.0, tau_z / (K_ROLL * w_p_cur * w_p_cur)))

        tau_rud_des = tau_y + _prop_drag(throt, elev, ail, w_p_cur)
        w_v_ask = min(w_vect_top, max(VECT_GOV_RAD_S, VECT_RATIO * w_p_ask))
        w_v_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_v_ask)
        rud = max(-1.0, min(1.0, tau_rud_des / (K_YAW * w_v_cur * w_v_cur)))

        controls = (throt, elev, ail, rud)
        self.Controls = (w_p_ask, w_v_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        vel_ok = max(abs(x) for x in vel) <= aerokin.HELD_M_S
        law['held'] = bool(
            not stunt and row['height'] >= 0.0 and abs(row['height'] - state['h']) <= aerokin.HELD_M
            and vel_ok
            and math.hypot(float(state['at'][0]) - law['spot'][0],
                           float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and sign_ok)
        law['sequence'] += 1
        bound = int(abs(throt) >= 0.999) | (int(abs(rud) >= 0.999) << 1)
        self.out = Answer((w_p_ask, w_v_ask), controls,
                          (K_THRUST * w_p_cur * w_p_cur * throt, K_YAW * w_v_cur * w_v_cur * rud),
                          (c, s), tuple(self.wind), bound)
        return self.out
