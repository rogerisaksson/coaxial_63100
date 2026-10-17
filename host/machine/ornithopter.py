"""The Dune-style 4-wing flapping ornithopter in MuJoCo: articulated wings, dive fold, and stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

An agile 2 kg 4-wing flapping ornithopter (`ornithopter.lisp`) powered by two Coaxial 63100 drives
(`FORE` and `AFT` driving the fore and aft articulated dragonfly wing pairs on finned aluminium
heatsinks with temperature-controlled cooling fans): collective stroke amplitude and wing pitch
pull upright or inverted, stroke-plane tilt pitches the carapace, differential left/right stroke
rolls it, and fore/aft wing-twist skew yaws it through pirouettes, sandstorm dives, and gates.
"""
import math
import os

from machine import aerokin, aerolisp, pull, quad
from machine.aerobatics import HOVER
from machine.helicopter import (Answer, _course_rate, _init_sky, _observe_wind_spin,
                                _smooth_target, _so3_torques, _stunt_step, _turn_heading)
from machine.quad import STEP_S, inflow, wind_at

LISP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ornithopter.lisp')
with open(LISP_PATH, encoding='utf-8') as _f:
    MANIFEST = aerolisp.compile_manifest(_f.read())

AERO, DYNAMICS, KINEMATICS = MANIFEST['aerodynamics'], MANIFEST['dynamics'], MANIFEST['kinematics']
STREAMS, STUNTS, BEHAVIORS = MANIFEST['streams'], MANIFEST['stunts'], MANIFEST['behaviors']
CARD, COOLING = MANIFEST['routines']['ornithopter_dune'], DYNAMICS['cooling']
_FW, _AF, _DL = AERO['flapping_wings'], AERO['airframe'], AERO['drive_load']
_RB, _GOV, _IMP, _OBS = (
    DYNAMICS['rigid_body'], DYNAMICS['governor'], DYNAMICS['impedance'], DYNAMICS['observers'])

MASS_KG, GRAVITY, INERTIA = (
    _RB['mass'], _RB['gravity'], (_RB['inertia_xx'], _RB['inertia_yy'], _RB['inertia_zz']))
WING_SPAN, WING_CHORD = _FW['wing_span'], _FW['wing_chord']
SHOULDER_X, FORE_Z, AFT_Z, SHOULDER_Y = (
    _FW['shoulder_x'], _FW['fore_z'], _FW['aft_z'], _FW['shoulder_y'])
SKID_M, FOOT_M, SKID_X = _RB['skid_height'], _RB['foot_radius'], _RB['skid_span']
STRUT_Z, HALF_M = (_RB['strut_aft'], _RB['strut_fore']), WING_SPAN
IDLE_RAD_S, GOV_RAD_S, AFT_GOV_RAD_S = _GOV['idle_speed'], _GOV['main_speed'], _GOV['tail_speed']
STUNT_BOOST, STUNT_YAW_RAD_S, AFT_RATIO = (
    _GOV['stunt_boost'], _GOV['stunt_yaw_threshold'], _GOV['tail_ratio'])
STROKE_DEG, PITCH_DEG, FOLD_DEG = (round(math.degrees(x), 6) for x in (
    _FW['stroke_limit'], _FW['pitch_limit'], _FW['sweep_fold_limit']))
K_THRUST, K_LIFT, K_DRAG = _FW['k_thrust'], _FW['k_lift'], _FW['k_drag']
K_PITCH, K_ROLL, K_YAW = _FW['k_pitch'], _FW['k_roll'], _FW['k_yaw']
INFLOW_FLOOR, RHO, BODY_CDA = _FW['inflow_floor'], _AF['air_density'], _AF['parasite_cda']
ATT_KP, ATT_KD = _IMP['stiffness']['attitude_pitch'], _IMP['damping']['attitude_pitch']
ROLL_KP, ROLL_KD = _IMP['stiffness']['attitude_roll'], _IMP['damping']['attitude_roll']
YAW_KP, YAW_KD = _IMP['stiffness']['attitude_yaw'], _IMP['damping']['attitude_yaw']
WO_TURN, WO_PULL, WO_WIND = _OBS['turn_bandwidth'], _OBS['pull_bandwidth'], _OBS['wind_bandwidth']
WIND_REG, MIN_AIR_M_S = _OBS['wind_regularization'], _OBS['min_air_speed']
FLOOR_M, HOVER_M, TOP_M = (KINEMATICS['envelope'][k] for k in (
    'floor_height', 'hover_height', 'top_height'))
FLAP_ROCK_HZ, FLAP_ROCK_RAD = (KINEMATICS['oscillator'][k] for k in (
    'tictoc_frequency', 'tictoc_amplitude'))
RIDGE_SOAR, THERMAL_SPIRAL, FOLD_ENTRY, SANDSTORM_DIVE, DIVE_PIRO = (STUNTS[k] for k in (
    'ridge_soar', 'thermal_spiral', 'fold_entry', 'sandstorm_dive', 'dive_piro'))
WING_ROCK, BANK_ROCK, DUNE_SLASH, CORIOLIS_STORM, DESERT_UNWIND, UNWIND = (STUNTS[k] for k in (
    'wing_rock', 'bank_rock', 'dune_slash', 'coriolis_storm', 'desert_unwind', 'unwind'))
#: The crank's gear from a drive's turn to a beat: 9.7 Hz at the governor, a 2 kg flapper's.
#: A pair's lift pulses BEAT_PULSE of its mean twice a beat, the pairs in antiphase: their
#: sum steady, a pitching moment at the beat.
GEAR_RATIO, BEAT_PULSE = _FW['gear_ratio'], _FW['beat_pulse']
PULL_MAX = (GOV_RAD_S * STUNT_BOOST) ** 2 * K_THRUST / MASS_KG
#: Its laps' sphere: 2.0 and 1.7 struck gate 7, 1.5 lapped in 24 s whole.
COURSE_SPHERE = 1.5 * GRAVITY


def drive_loads(controls):
    """External load scales `(scale_fore, scale_aft)` on the FORE and AFT 63100 wing drives."""
    stroke, pitch_t, roll_d, yaw_t = controls
    return (min(_DL['main_ceiling'], _DL['main_base'] + _DL['main_collective'] * (
                stroke * stroke + _DL['main_cyclic'] * (pitch_t * pitch_t + roll_d * roll_d))),
            min(_DL['tail_ceiling'], _DL['tail_base'] + _DL['main_collective'] * (
                0.85 * stroke * stroke + 0.5 * _DL['tail_pitch'] * yaw_t * yaw_t)))


def wing_kinematics(controls, speeds):
    """`(beat_hz, stroke_deg, fold_deg)` of the 4 articulated wings for `controls` and `speeds`."""
    stroke, pitch_t, _roll_d, _yaw_t = controls
    w_mean = 0.5 * (float(speeds[0]) + float(speeds[1]))
    beat_hz = w_mean / (math.tau * GEAR_RATIO)
    stroke_deg = stroke * STROKE_DEG
    folded = max(0.0, min(1.0, -stroke - 0.10)) if stroke < -0.10 else (
        0.35 if w_mean < 120.0 else 0.12 * abs(pitch_t))
    return beat_hz, stroke_deg, folded * FOLD_DEG


def row_from_payload(payload, base=None):
    """Translate a `CompiledAeroPayload` into an ornithopter stunt row dict."""
    return aerolisp.payload_to_row(
        payload, base or HOVER, MASS_KG * GRAVITY, KINEMATICS['horizontal']['orbit_speed'])


def mjcf(things=()):
    """The 4-wing Dune ornithopter and `things` (`grounds.solids`) as MuJoCo XML."""
    m = ('<geom name="carapace" type="box" pos="0 0.01 0.08" size="0.065 0.048 0.19" contype="1" '
         'conaffinity="1"/><geom name="fore_wings" type="box" pos="0 %g %g" size="%g 0.008 %g" '
         'contype="1" conaffinity="1"/><geom name="aft_wings" type="box" pos="0 %g %g" '
         'size="%g 0.008 %g" contype="1" conaffinity="1"/><geom name="boom" type="capsule" '
         'fromto="0 0.018 -0.08 0 0.018 -0.64" size="0.013" contype="1" conaffinity="1"/>'
         '<geom name="vtail" type="box" pos="0 0.04 -0.62" size="0.11 0.045 0.03" contype="1" '
         'conaffinity="1"/>' % (SHOULDER_Y, FORE_Z, 0.85 * WING_SPAN, 0.5 * WING_CHORD,
                                SHOULDER_Y + 0.01, AFT_Z, 0.75 * WING_SPAN, 0.45 * WING_CHORD))
    for k, (sx, sz) in enumerate((sx, sz) for sx in (-SKID_X, SKID_X) for sz in STRUT_Z):
        m += ('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
              'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -SKID_M, sz, FOOT_M))
    head = quad.mjcf(things).split('<body name="frame"')[0]
    return ('%s<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody>' '</mujoco>' % ((head, SKID_M + FOOT_M, MASS_KG) + INERTIA + (m,)))


def _wing_drag(stroke, pitch_t, roll_d, w_eq):
    return (0.32 + 1.55 * (stroke * stroke + 0.48 * (pitch_t * pitch_t + roll_d * roll_d))) * (
        K_DRAG * w_eq * w_eq)


def _wing_cda(stroke=0.0):
    """Total carapace and articulated 4-wing drag area $C_D A$."""
    return BODY_CDA + 0.08 * WING_SPAN * WING_CHORD * (1.0 + 0.5 * abs(float(stroke)))


def _flap_inflow(along, v_horiz, w_eq):
    """Combined axial inflow and translational flapping lift factor (`K_LIFT`)."""
    tip = max(w_eq * WING_SPAN, 8.0)
    return inflow(along, w_eq) * (1.0 + K_LIFT * min(0.35, v_horiz / tip))


class Sky(quad.Sky):

    """The 4-wing Dune ornithopter in MuJoCo on the floor among `things`."""

    def __init__(self, things=()):
        self.yaw_force, self.phase = 0.0, 0.0
        _init_sky(self, mjcf(things), SKID_M + FOOT_M)

    def reset(self):
        """Return the ornithopter to its claws at the origin, still, nothing struck."""
        super().reset()
        self.yaw_force, self.speeds, self.controls = 0.0, (0.0, 0.0), (0.0, 0.0, 0.0, 0.0)
        self.phase = 0.0

    def state(self):
        """The ornithopter's state: `quad.Sky.state` plus `yaw_force`, `speeds`, and `controls`."""
        out = super().state()
        out.update(yaw_force=self.yaw_force, speeds=self.speeds, controls=self.controls)
        return out

    def step(self, command, dt, air=None):
        """Step `dt` s under `command` `(w_fore, w_aft, stroke, pitch_tilt, roll_diff, yaw_twist)`."""
        d, f = self.data, self.frame
        cmd = list(command) + [0.0] * 6
        w_f, w_a, stroke, pitch_t, roll_d, yaw_t = (float(x) for x in cmd[:6])
        self.speeds, self.controls = (w_f, w_a), (stroke, pitch_t, roll_d, yaw_t)
        w_sq = 0.5 * (w_f * w_f + w_a * w_a)
        w_eq = math.sqrt(w_sq)
        self.lift = K_THRUST * w_sq * stroke
        turn, h = d.xmat[f].reshape(3, 3), float(d.qpos[1]) - (SKID_M + FOOT_M)
        wx, wy, wz = (tuple(a + b for a, b in zip(wind_at(air['wind'], h),
                                                  wind_at(air['eddies'][0], h)))
                      if air is not None else (0.0, 0.0, 0.0))
        along = sum((float(d.qvel[a]) - w) * float(turn[a, 1]) for a, w in enumerate((wx, wy, wz)))
        vh = math.hypot(float(d.qvel[0]) - wx, float(d.qvel[2]) - wz)
        lift = self.lift * _flap_inflow(along if stroke >= 0.0 else -along, vh, w_eq)
        self.yaw_force = f_yaw = K_YAW * w_a * w_a * yaw_t
        ab = (K_PITCH * w_sq * pitch_t, f_yaw - _wing_drag(stroke, pitch_t, roll_d, w_eq),
              K_ROLL * w_sq * roll_d)
        dk, qv, xf = 0.5 * RHO * _wing_cda(stroke), d.qvel, d.xfrc_applied
        beat, rock = w_eq / GEAR_RATIO, 0.5 * lift * BEAT_PULSE * (FORE_Z - AFT_Z)
        for _ in range(max(1, int(round(dt / STEP_S)))):
            t = d.xmat[f].reshape(3, 3)
            self.phase = (self.phase + beat * STEP_S) % math.tau
            vx, vy, vz = float(qv[0]) - wx, float(qv[1]) - wy, float(qv[2]) - wz
            dr = dk * math.sqrt(vx * vx + vy * vy + vz * vz)
            xf[f, :3] = (lift * float(t[0, 1]) - dr * vx,
                         lift * float(t[1, 1]) - dr * vy,
                         lift * float(t[2, 1]) - dr * vz)
            ab0 = ab[0] + rock * math.cos(2.0 * self.phase)
            xf[f, 3:] = tuple(float(t[r, 0]) * ab0 + float(t[r, 1]) * ab[1]
                              + float(t[r, 2]) * ab[2] for r in range(3))
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)


class Core:

    """The 4-wing Dune ornithopter's SO(3) stroke-plane, wing-beat, and wind-observer core."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset observer and trajectory state."""
        self.vel_hat, self.push_hat, self.wind = [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.spin_hat, self.turn_hat = [0.0] * 3, [0.0] * 3
        self.begun, self.flip_ang, self.roll_ang = False, 0.0, 0.0
        self.piro_ang, self.tic_phase = 0.0, 0.0
        self.r_prev, self.up_filt, self.w_ff = None, None, (0.0, 0.0, 0.0)
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
        w_f = float(sp[0]) if sp[0] > 1.0 else self.Controls[0]
        w_a = float(sp[1]) if sp[1] > 1.0 else self.Controls[1]
        w_sq = 0.5 * (w_f * w_f + w_a * w_a)
        w_eq = math.sqrt(w_sq)
        stroke, pitch_t, roll_d, yaw_t = state.get('controls', self.Controls[2:])
        alpha = 0.5 * RHO * _wing_cda(stroke) / MASS_KG * fast
        up = (turn[0][1], turn[1][1], turn[2][1])
        s_along = sum(u[a] * up[a] for a in range(3)) * (1.0 if stroke >= 0.0 else -1.0)
        lift = float(state.get('lift', 0.0)) * _flap_inflow(s_along, math.hypot(u[0], u[2]), w_eq)
        for a in range(3):
            off = vel[a] - self.vel_hat[a]
            acc = (lift * up[a]) / MASS_KG - (GRAVITY if a == 1 else 0.0) \
                + self.push_hat[a] - alpha * u[a]
            self.vel_hat[a] += dt * (acc + 2.0 * WO_PULL * off)
            self.push_hat[a] += dt * WO_PULL * WO_PULL * off
        gamma = (abs(float(state.get('lift', 0.0))) / (2.0 * MASS_KG * w_eq * WING_SPAN)
                 if w_eq > 1.0 and abs(s_along) < 4.0 * w_eq * WING_SPAN else 0.0)
        q_w = _wing_drag(stroke, pitch_t, roll_d, w_eq)
        tau = (K_PITCH * w_sq * pitch_t - (INERTIA[2] - INERTIA[1]) * spin[1] * spin[2],
               K_YAW * w_a * w_a * yaw_t - q_w - (INERTIA[0] - INERTIA[2]) * spin[2] * spin[0],
               K_ROLL * w_sq * roll_d - (INERTIA[1] - INERTIA[0]) * spin[0] * spin[1])
        _observe_wind_spin(self, u, fast, alpha, gamma, up, spin, tau, INERTIA, dt)

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_thrust, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        rock_amp, inv = (FLAP_ROCK_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(rock_amp) > 1e-6
        if stunt:
            r_des, w_ff, f_des = _stunt_step(
                self, flips, rolls, nose_val, rock_amp, FLAP_ROCK_HZ, turn, yaw, rate, acc_des, dt)
            return r_des, w_ff, f_des, stunt, inv
        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        # Pulled the other way along its axis - a burn's brake - a lean leans it the other way.
        s_f = 1.0 if (ay_des if inv >= 0.5 else ay_thrust) >= 0.0 else -1.0
        want_up = (sign * s_f * ax_des / a_norm, sign * ay_des / a_norm, sign * s_f * az_des / a_norm)
        r_des, w_ff = _smooth_target(self, want_up, sign, yaw, rate, float(row.get('pace', 0.0)), dt)
        ay_use = ay_des if inv >= 0.5 else ay_thrust
        f_des = MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1])
        return r_des, w_ff, f_des, stunt, inv

    def plan(self, law, row, state, dt, pull_max):
        """Advance outer horizontal kinematics `aerokin.lean` across `dt` s."""
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), _course_rate((c0, s0), row), dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.up_pull, law['doing'] = aerokin.climb(row, state['h'], state['v'], reach - GRAVITY)
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
        prev_stroke = float(state.get('controls', self.Controls[2:])[0])
        drag = 0.5 * RHO * _wing_cda(prev_stroke) / MASS_KG * math.sqrt(
            u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
        air_acc = [self.push_hat[a] - drag * u[a] for a in range(3)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        rate = _course_rate(law['heading'], row)
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach, up_pull = max(aerokin.LEAST * GRAVITY, pull_max), self.up_pull

        if row['height'] < 0.0:
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff = None, None, (0.0, 0.0, 0.0)
            w_f_ask = GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            w_a_ask = AFT_GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            if limits:
                w_f_ask, w_a_ask = min(limits[0], w_f_ask), min(limits[1], w_a_ask)
            controls = (0.0, 0.0, 0.0, 0.0)
            self.Controls = (w_f_ask, w_a_ask) + controls
            law['held'] = False
            self.out = Answer((w_f_ask, w_a_ask), controls, (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out

        light_val = float(row.get('light', 0.5))
        ay_des = max(GRAVITY + up_pull - air_acc[1], light_val * GRAVITY)
        acc_des = (along[0] - air_acc[0], ay_des, along[1] - air_acc[2])
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        in_burn = row['height'] <= FLOOR_M + 0.05 and (state['h'] < 22.0 or state['v'] <= -0.5 * row['pace'])
        ay_thrust = ay_stop if in_burn else max(GRAVITY + up_pull, -1.5 * GRAVITY) - air_acc[1]
        r_des, w_ff, f_des, stunt, inv = self._target_attitude(
            row, state, turn, yaw, rate, acc_des, ay_thrust, dt)

        f_limit = MASS_KG * reach
        f_des = max(-f_limit, min(f_limit, f_des))
        tau_x, tau_y, tau_z = _so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD))

        w_top = limits[0] if limits else pull.held(pull.I_MAX)
        w_aft_top = limits[1] if limits else w_top
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S
                             or math.hypot(*along) > 0.5 * GRAVITY else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _flap_inflow(
            along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_f_ask = min(w_top, w_gov)
        w_f_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_f_ask)

        stroke = max(-1.0, min(1.0, f_comp / (K_THRUST * w_f_cur * w_f_cur)))
        pitch_t = max(-1.0, min(1.0, tau_x / (K_PITCH * w_f_cur * w_f_cur)))
        roll_d = max(-1.0, min(1.0, tau_z / (K_ROLL * w_f_cur * w_f_cur)))

        tau_yaw_des = tau_y + _wing_drag(stroke, pitch_t, roll_d, w_f_cur)
        w_a_ask = min(w_aft_top, max(AFT_GOV_RAD_S, AFT_RATIO * w_f_ask))
        w_a_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_a_ask)
        w_sq_cur = 0.5 * (w_f_cur * w_f_cur + w_a_cur * w_a_cur)
        stroke = max(-1.0, min(1.0, f_comp / (K_THRUST * w_sq_cur)))
        pitch_t = max(-1.0, min(1.0, tau_x / (K_PITCH * w_sq_cur)))
        roll_d = max(-1.0, min(1.0, tau_z / (K_ROLL * w_sq_cur)))
        yaw_t = max(-1.0, min(1.0, (tau_y + _wing_drag(stroke, pitch_t, roll_d, math.sqrt(w_sq_cur)))
                              / (K_YAW * w_a_cur * w_a_cur)))

        controls = (stroke, pitch_t, roll_d, yaw_t)
        self.Controls = (w_f_ask, w_a_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        law['held'] = bool(
            not stunt and abs(row['height'] - state['h']) <= aerokin.HELD_M
            and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(float(state['at'][0]) - law['spot'][0],
                           float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and sign_ok)
        law['sequence'] += 1
        bound = int(abs(stroke) >= 0.999) | (int(abs(yaw_t) >= 0.999) << 1)
        self.out = Answer((w_f_ask, w_a_ask), controls,
                          (0.5 * K_THRUST * w_f_cur * w_f_cur * stroke,
                           0.5 * K_THRUST * w_a_cur * w_a_cur * stroke),
                          (c, s), tuple(self.wind), bound)
        return self.out
