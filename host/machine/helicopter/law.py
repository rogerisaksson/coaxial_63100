"""The helicopter's SO(3) law: its attitude, its signed collective, its tail; its observer's."""
import math

from machine import aerokin, pull
from machine.helicopter.airframe import (ATT_KD, ATT_KP, BODY_CDA, GOV_RAD_S, GRAVITY, IDLE_RAD_S,
                                         INERTIA, INFLOW_FLOOR, K_CYC, K_TAIL, K_THRUST, MASS_KG,
                                         RHO, ROLL_KD, ROLL_KP, STUNT_BOOST, STUNT_YAW_RAD_S,
                                         TAIL_GOV_RAD_S, TAIL_M, TAIL_RATIO, TICTOC_HZ, TICTOC_RAD,
                                         YAW_KD, YAW_KP, _rotor_drag, _rotor_inflow)
from machine.helicopter.kit import (Answer, _course_rate, _mat_mul, _plan_hold, _rot_x,
                                    _smooth_target, _so3_torques, _step_hold, _stunt_step,
                                    _turn_heading)
from machine.helicopter.observer import observe


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

    def _target_attitude(self, row, state, turn, yaw, rate, acc_des, ay_use, dt):
        flips, rolls, nose_val = float(row.get('flip', 0.0)), float(row.get('roll', 0.0)), float(row.get('nose', 1.0))
        tictoc, inv = (TICTOC_RAD if nose_val <= -1.5 else 0.0), (1.0 if -1.5 < nose_val <= -0.5 else 0.0)
        ax_des, ay_des, az_des = acc_des
        stunt = abs(flips) > 1e-6 or abs(rolls) > 1e-6 or abs(tictoc) > 1e-6
        if stunt:
            return _stunt_step(self, flips, rolls, nose_val, tictoc, TICTOC_HZ, turn, yaw, rate, acc_des, dt) + (stunt, inv)
        self.tic_phase = self.roll_ang = self.piro_ang = 0.0
        prev_flip = self.flip_ang
        self.flip_ang, sign = (math.pi, -1.0) if inv >= 0.5 else (0.0, 1.0)
        if self.r_prev is not None:
            self.r_prev = _mat_mul(self.r_prev, _rot_x(self.flip_ang - prev_flip))
        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        want_up = (sign * ax_des / a_norm, sign * ay_des / a_norm, sign * az_des / a_norm)
        r_des, w_ff = _smooth_target(self, want_up, sign, yaw, rate, float(row.get('pace', 0.0)), dt)
        return r_des, w_ff, MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1]), stunt, inv

    def plan(self, law, row, state, dt, pull_max):
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), _course_rate((c0, s0), row), dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.up_pull, law['doing'] = aerokin.climb(
            row, state['h'], state['v'], reach - GRAVITY,
            aerokin.spooled(self.out.speeds[0], math.sqrt(reach * MASS_KG / K_THRUST), IDLE_RAD_S), dt)
        along = aerokin.lean(law, row, state['at'], state['vel'], dt, min(row['lean'], math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))))
        law['heading'] = (c0, s0)
        return _plan_hold(self, along, dt)

    def setpoint(self, law, row, state, turn, u, dt, pull_max, along=None):
        """Outer guidance and setpoint regulation on SE(3): `(r_des, w_ff, f_des, stunt, inv,
        c, s, rate, along)` for the inner dynamic SE(3) loop (`se3`)."""
        drag = 0.5 * RHO * BODY_CDA / MASS_KG * math.hypot(*u)
        air_acc = (self.push_hat[0] - drag * u[0], self.push_hat[1] - drag * u[1], self.push_hat[2] - drag * u[2])
        along = self.plan(law, row, state, dt, pull_max) if along is None else _step_hold(self, along, dt)
        rate = _course_rate(law['heading'], row)
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        if row['height'] < 0.0:
            return None, None, 0.0, False, 0.0, c, s, rate, along
        light_val, tail_prev = float(row.get('light', 0.5)), float(state.get('tail', 0.0))
        ay_des = max(GRAVITY + self.up_pull - air_acc[1], light_val * GRAVITY)
        acc_des = (along[0] - air_acc[0] - tail_prev * turn[0][0] / MASS_KG, ay_des,
                   along[1] - air_acc[2] - tail_prev * turn[2][0] / MASS_KG)
        ay_use = (1.0 - float(law.get('doing') == aerokin.FALL)) * ay_des
        r_des, w_ff, f_des, stunt, inv = self._target_attitude(
            row, state, turn, yaw, rate, acc_des, ay_use, dt)
        return r_des, w_ff, f_des, stunt, inv, c, s, rate, along

    def se3(self, state, turn, spin, u, r_des, w_ff, f_des, stunt, inv, rate, along, reach, limits=None):
        """Inner dynamic SE(3) control and rotor/swash allocation for `(r_des, w_ff, f_des)`."""
        f_limit = MASS_KG * reach
        f_des = max(-f_limit, min(f_limit, f_des))
        tau_x, tau_y, tau_z = _so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD))
        w_top = limits[0] if limits else pull.held(pull.I_MAX)
        w_tail_top = limits[1] if limits else pull.held(pull.I_MAX)
        sp = state.get('speeds', (0.0, 0.0))
        w_gov = GOV_RAD_S * (STUNT_BOOST if stunt or inv >= 0.5 or abs(rate) > STUNT_YAW_RAD_S
                             or math.hypot(*along) > 0.5 * GRAVITY else 1.0)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _rotor_inflow(
            along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_ask = min(w_top, w_gov)
        w_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_ask)
        coll = max(-1.0, min(1.0, f_comp / (K_THRUST * w_cur * w_cur)))
        cyc_p, cyc_r = (max(-1.0, min(1.0, t / (K_CYC * w_cur * w_cur))) for t in (tau_x, tau_z))
        f_tail_des = (tau_y + _rotor_drag(coll, cyc_p, cyc_r, w_cur)) / TAIL_M
        w_t_ask = min(w_tail_top, max(TAIL_GOV_RAD_S, TAIL_RATIO * w_ask))
        w_t_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_t_ask)
        tail_p = max(-1.0, min(1.0, f_tail_des / (K_TAIL * w_t_cur * w_t_cur)))
        controls = (coll, cyc_p, cyc_r, tail_p)
        bound = int(abs(coll) >= 0.999) | (int(abs(tail_p) >= 0.999) << 1)
        return ((w_ask, w_t_ask), controls,
                (K_THRUST * w_cur * w_cur * coll, K_TAIL * w_t_cur * w_t_cur * tail_p), bound)

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        observe(self, state, dt)
        t_in, s_in, v_in = state['turn'], state['spin'], state['vel']
        turn = t_in if (type(t_in) is tuple and type(t_in[0]) is tuple and type(t_in[0][0]) is float) \
            else (t_in.tolist() if hasattr(t_in, 'tolist') else [[float(t_in[r][c]) for c in range(3)] for r in range(3)])
        spin = s_in if (type(s_in) is tuple and type(s_in[0]) is float) else [float(x) for x in s_in]
        vel = v_in if (type(v_in) is tuple and type(v_in[0]) is float) else [float(x) for x in v_in]
        u = (vel[0] - self.wind[0], vel[1] - self.wind[1], vel[2] - self.wind[2])
        r_des, w_ff, f_des, stunt, inv, c, s, rate, along = self.setpoint(
            law, row, state, turn, u, dt, pull_max, along)
        if r_des is None:
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff = None, None, (0.0, 0.0, 0.0)
            w_ask, w_t_ask = (GOV_RAD_S, TAIL_GOV_RAD_S) if row['height'] > -0.1 else (IDLE_RAD_S, IDLE_RAD_S)
            if limits:
                w_ask, w_t_ask = min(limits[0], w_ask), min(limits[1], w_t_ask)
            self.Controls, law['held'] = (w_ask, w_t_ask, 0.0, 0.0, 0.0, 0.0), False
            self.out = Answer((w_ask, w_t_ask), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        speeds, controls, forces, bound = self.se3(
            state, turn, spin, u, r_des, w_ff, f_des, stunt, inv, rate, along, reach, limits)
        self.Controls = speeds + controls
        law['held'] = bool(
            not stunt and abs(row['height'] - state['h']) <= aerokin.HELD_M and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(float(state['at'][0]) - law['spot'][0], float(state['at'][2]) - law['spot'][1]) <= aerokin.HELD_SPOT_M
            and (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD))
        law['sequence'] += 1
        self.out = Answer(speeds, controls, forces, (c, s), tuple(self.wind), bound)
        return self.out
