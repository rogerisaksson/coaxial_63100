"""The ornithopter's SO(3) law: its attitude, its strokes' thrust, its wings' twist; its observer's."""
import math

from machine import aerokin, pull
from machine.helicopter.kit import (Answer, _course_rate, _mat_mul, _plan_hold, _rot_x,
                                    _smooth_target, _so3_torques, _step_hold, _stunt_step,
                                    _turn_heading)
from machine.ornithopter.airframe import (AFT_GOV_RAD_S, AFT_RATIO, AFT_Z, ATT_KD, ATT_KP,
                                          BEAT_PULSE, FLAP_ROCK_HZ, FLAP_ROCK_RAD, FORE_Z,
                                          GEAR_RATIO, GOV_RAD_S, GRAVITY, IDLE_RAD_S, INERTIA,
                                          INFLOW_FLOOR, K_PITCH, K_ROLL, K_THRUST, K_YAW, MASS_KG,
                                          RHO, ROLL_KD, ROLL_KP, STUNT_BOOST, STUNT_YAW_RAD_S,
                                          YAW_KD, YAW_KP, _flap_inflow, _wing_cda, _wing_drag)
from machine.ornithopter.observer import observe
from machine.quad import STEP_S


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

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_use, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        rock_amp, inv = (FLAP_ROCK_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(rock_amp) > 1e-6
        if stunt:
            r_des, w_ff, f_des = _stunt_step(
                self, flips, rolls, nose_val, rock_amp, FLAP_ROCK_HZ, turn, yaw, rate, acc_des, dt)
            return r_des, w_ff, f_des, stunt, inv
        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
        prev_flip = self.flip_ang
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        if self.r_prev is not None:
            self.r_prev = _mat_mul(self.r_prev, _rot_x(self.flip_ang - prev_flip))
        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        want_up = (sign * ax_des / a_norm, sign * ay_des / a_norm, sign * az_des / a_norm)
        r_des, w_ff = _smooth_target(self, want_up, sign, yaw, rate, float(row.get('pace', 0.0)), dt)
        f_des = MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1])
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
        law['heading'] = (c0, s0)
        return _plan_hold(self, along, dt)

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        """One pass of `dt` s on `row` and `state`: returns `Answer`."""
        observe(self, state, dt)
        t_in, s_in, v_in = state['turn'], state['spin'], state['vel']
        turn = t_in if (type(t_in) is tuple and type(t_in[0]) is tuple and type(t_in[0][0]) is float) \
            else (t_in.tolist() if hasattr(t_in, 'tolist') else [[float(t_in[r][c]) for c in range(3)] for r in range(3)])
        spin = s_in if (type(s_in) is tuple and type(s_in[0]) is float) else [float(x) for x in s_in]
        vel = v_in if (type(v_in) is tuple and type(v_in[0]) is float) else [float(x) for x in v_in]
        u = (vel[0] - self.wind[0], vel[1] - self.wind[1], vel[2] - self.wind[2])
        prev_stroke = float(state.get('controls', self.Controls[2:])[0])
        sp = state.get('speeds', (0.0, 0.0))
        w_f_prev = float(sp[0]) if sp[0] > 1.0 else self.Controls[0]
        w_a_prev = float(sp[1]) if sp[1] > 1.0 else self.Controls[1]
        w_eq_prev = math.sqrt(0.5 * (w_f_prev * w_f_prev + w_a_prev * w_a_prev))
        beat = max(1.0, w_eq_prev / GEAR_RATIO)
        s_along = sum(u[a] * turn[a][1] for a in range(3)) * (1.0 if prev_stroke >= 0.0 else -1.0)
        lift_prev = float(state.get('lift', 0.0)) * _flap_inflow(s_along, math.hypot(u[0], u[2]), w_eq_prev)
        rock = 0.5 * lift_prev * BEAT_PULSE * (FORE_Z - AFT_Z)
        w_rock = rock * math.sin(2.0 * float(state.get('phase', 0.0)) + beat * STEP_S) / (2.0 * beat * INERTIA[0])
        spin = (spin[0] - w_rock, spin[1], spin[2])
        drag = 0.5 * RHO * _wing_cda(prev_stroke) / MASS_KG * math.sqrt(
            u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
        air_acc = (self.push_hat[0] - drag * u[0], self.push_hat[1] - drag * u[1], self.push_hat[2] - drag * u[2])
        along = self.plan(law, row, state, dt, pull_max) if along is None else _step_hold(self, along, dt)
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
        ay_use = (1.0 - float(law.get('doing') == aerokin.FALL)) * ay_des
        r_des, w_ff, f_des, stunt, inv = self._target_attitude(
            row, state, turn, yaw, rate, acc_des, ay_use, dt)

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
