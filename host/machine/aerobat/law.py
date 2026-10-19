"""The aerobat's SO(3) law: hover and prop-hang, its stunts, on its wing nose first; its observer's."""
import math

from machine import aerokin, pull, se3mpc
from machine.aerobatics import ease
from machine.aerobat.airframe import (ATT_KD, ATT_KP, FLOOR_M, GOV_RAD_S, GRAVITY, IDLE_RAD_S,
                                      INERTIA, INFLOW_FLOOR, K_PITCH, K_ROLL, K_THRUST, K_YAW,
                                      MASS_KG, RHO, ROLL_KD, ROLL_KP, ROTATE_M_S, RUNWAY_M,
                                      RUNWAY_RAD, STUNT_BOOST, STUNT_YAW_RAD_S, VECT_GOV_RAD_S,
                                      VECT_RATIO, WATERFALL_HZ, WATERFALL_RAD, WING, YAW_KD,
                                      YAW_KP, _aero_cda, _aero_inflow, _prop_drag)
from machine.aerobat.observer import observe
from machine.helicopter.kit import (Answer, _smooth_target, _so3_torques,
                                    _stunt_step, _turn_heading)
from machine.wingborne import AIRBORNE_M, flare, lined, plane, run, toward, whole

#: A row asked faster than FAST_M_S, m/s, flies on its wing whole once it is past half of it
#: (`wingborne.whole`), eased in over the span: none slower, all faster.
FAST_M_S = (3.0, 7.0)
#: Low it comes round to the runway's line where its row is `lined` (1 unsaid; a line's own
#: none): asked under LINED_M, m (all under its first), its turn and its slide eased out by
#: LINED_RAD_S and LINED_M_S - switched on as they came to none, the run-in's yaw stepped to
#: 1 rad/s and its rudder 0.56 in a tick.
LINED_M, LINED_RAD_S, LINED_M_S = (1.5, 2.0), math.radians(10.0), 1.0
#: The head idles up to STUNT_BOOST over BOOST of its turn's and its lean's asks - stepped at
#: theirs whole, a yaw lined up crossed it with the rudder.
BOOST = (0.8, 1.0)


def flies(row, w):
    """How much a row is flown on the wing, wing-borne `w`: as fast as it is asked and as whole."""
    return whole(w) * ease((row.get('speed', 0.0) - FAST_M_S[0]) / (FAST_M_S[1] - FAST_M_S[0]))


class Core:

    """The aerobat's SO(3) law, its surfaces in the propwash, its wind observer."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset observer and trajectory state."""
        self.vel_hat, self.push_hat, self.wind = [0.0] * 3, [0.0] * 3, [0.0] * 3
        self.spin_hat, self.turn_hat = [0.0] * 3, [0.0] * 3
        self.begun, self.flip_ang, self.roll_ang = False, 0.0, 0.0
        self.piro_ang, self.tic_phase = 0.0, 0.0
        self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
        self.rotated, self.wingborne, self.wing_air, self.floor = 0.0, 0.0, 0.0, FLOOR_M
        self.Controls = (IDLE_RAD_S, IDLE_RAD_S, 0.0, 0.0, 0.0, 0.0)
        self.out = Answer((IDLE_RAD_S, IDLE_RAD_S), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0),
                          (1.0, 0.0), (0.0, 0.0, 0.0))

    def _target_attitude(self, law, row, state, turn, yaw, rate, acc_des, ay_thrust, dt):
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
            law['spot'] = ([0.0, 0.0] if -0.5 * RUNWAY_M < ahead < 2.5 * RUNWAY_M else
                           [float(state['at'][a]) + float(state['vel'][a]) * abs(v_fwd) / (2.0 * brake)
                            for a in (0, 2)])
        if not self.airborne and rh >= 0.0 and pace < 20.0:
            return run(WING, self, v_fwd, h, yaw, rate) + (stunt, inv)

        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        s_f = -1.0 if inv >= 0.5 and ay_des < 0.0 else 1.0
        want_up = [sign * s_f * ax_des / a_norm, sign * ay_des / a_norm, sign * s_f * az_des / a_norm]
        ay_use = ay_des if inv >= 0.5 else ay_thrust
        u = [float(state['vel'][a]) - self.wind[a] for a in range(3)]
        # The thrust gives along the nose what the wing does not; the hover's share is in acc_des.
        w_nose = sum(x * turn[a][1] for a, x in enumerate(WING.lift(u, turn)))
        f_des = MASS_KG * (ax_des * turn[0][1] + ay_use * turn[1][1] + az_des * turn[2][1]) - self.wing_air * w_nose
        r_wing, want_up = plane(WING, turn, (ax_des, ay_des, az_des), u, self.wingborne, want_up,
                                row.get('knife', 0.0))
        r_des, w_ff = _smooth_target(self, tuple(want_up), sign, yaw, rate, pace, dt)
        k = whole(self.wingborne) * (r_wing is not None)
        return toward(r_des, r_wing or r_des, k), tuple((1.0 - k) * x for x in w_ff), f_des, stunt, inv

    def yawed(self, law, row, state):
        """(the heading's rate asked, rad/s; the runway's line low flight comes round to): the
        row's turn, and low, straight and unslid the line's, eased in (LINED_M). The plan turns
        the heading by it too: on the quad course's gate's, switched on, a pass's lean asked 13
        m/s^2 across the final."""
        vel, fast = [float(x) for x in state['vel']], row.get('speed', 0.0) > 5.0
        rate = math.radians(float(row.get('turn', 0.0))) if self.airborne else 0.0
        target = lined(RUNWAY_RAD, fast and law['heading'], vel, [float(x) for x in state['at']], state['h'],
                       ROTATE_M_S)
        low = (row.get('lined', 1.0) * max(float(self.airborne), ease((state['h'] - 0.05) / 0.1))
               * ease((LINED_M[1] - row['height']) / (LINED_M[1] - LINED_M[0]))
               * (1.0 - ease(abs(rate) / LINED_RAD_S)) * (1.0 - ease(abs(row.get('slide', 0.0)) / LINED_M_S)))
        err = (target - math.atan2(law['heading'][1], law['heading'][0]) + math.pi) % math.tau - math.pi
        # No faster than its lean carries it round at its speed: on a wing it stalled.
        top = min(2.4, row['lean'] / max(1.0, math.hypot(vel[0], vel[2])))
        return rate + low * (max(-top, min(top, 3.0 * err)) - rate), target

    def plan(self, law, row, state, dt, pull_max):
        """Advance outer horizontal kinematics `aerokin.lean` across `dt` s."""
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), self.yawed(law, row, state)[0], dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.floor = FLOOR_M * max(0.0, 1.0 - math.hypot(row.get('speed', 0.0), row.get('slide', 0.0)) / 5.0)
        r_climb = dict(row, height=max(self.floor, row['height'])) if row['height'] >= 0.0 else row
        self.up_pull, law['doing'] = aerokin.climb(
            r_climb, state['h'], state['v'], reach - GRAVITY,
            aerokin.spooled(self.out.speeds[0], math.sqrt(reach * MASS_KG / K_THRUST), IDLE_RAD_S), dt)
        beside = math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))
        # On its wing it flies the air's speed: the ground's 14 m/s stalled it in a 7 m/s tail.
        c, s = law['heading']
        on = flies(row, self.wingborne)
        wx, wz = on * self.wind[0], on * self.wind[2]
        aired = dict(row, speed=row['speed'] + wx * s + wz * c, slide=row['slide'] + wx * c - wz * s)
        along = aerokin.lean(law, aired, state['at'], state['vel'], dt, min(row['lean'], beside))
        # Nose first it flies a speed: chasing a place it lagged a turn by 12 m.
        held = 1.0 - self.airborne * (1.0 - on)
        if row['height'] >= 0.0:
            law['spot'] = [p + held * (float(state['at'][a]) - p) for p, a in zip(law['spot'], (0, 2))]
        law['heading'] = (c0, s0)
        return along

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        """One pass of `dt` s on `row` and `state`: returns `Answer`."""
        observe(self, state, dt)
        turn = [[float(x) for x in r] for r in state['turn']]
        spin, vel, at = ([float(x) for x in state[k]] for k in ('spin', 'vel', 'at'))
        u, knife, fast = [vel[a] - self.wind[a] for a in range(3)], row.get('knife', 0.0), row.get('speed', 0.0) > 5.0
        # On its side the nose is off the air by the slip; fast, the share carried whole.
        k = flies(row, max(self.wingborne, abs(knife) * sum(x * x for x in u) / ROTATE_M_S ** 2))
        self.wingborne, self.wing_air = (row.get('wing', 1.0) * (x + k * (1.0 - x))
                                         for x in (self.wingborne, self.wing_air))
        prev_ctrl = state.get('controls', self.Controls[2:])
        drag = 0.5 * RHO * _aero_cda(*prev_ctrl[:3]) / MASS_KG * math.hypot(*u)
        w_lift = WING.lift(u, turn)
        air_acc = [(1.0 - self.wing_air) * (self.push_hat[a] + w_lift[a] / MASS_KG) - drag * u[a]
                   for a in range(3)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        rate, target_yaw = self.yawed(law, row, state)
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach, up_pull = max(aerokin.LEAST * GRAVITY, pull_max), self.up_pull

        if row['height'] < 0.0 and (row['height'] <= -0.1 or not self.airborne):
            if state['h'] < 0.03 and max(abs(x) for x in vel) < 0.05:
                law['heading'] = c, s = math.cos(target_yaw), math.sin(target_yaw)
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
            self.rotated = 0.0
            law['spot'] = [at[0], at[2]]
            w_p_ask = GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            w_v_ask = VECT_GOV_RAD_S if row['height'] > -0.1 else IDLE_RAD_S
            if limits:
                w_p_ask, w_v_ask = min(limits[0], w_p_ask), min(limits[1], w_v_ask)
            controls = (0.0, 0.0, 0.0, 0.0)
            self.Controls = (w_p_ask, w_v_ask) + controls
            law['held'] = False
            self.out = Answer((w_p_ask, w_v_ask), controls, (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out

        rh = max(self.floor, float(row['height'])) if row['height'] >= 0.0 else float(row['height'])
        ay_des = max(GRAVITY + up_pull - air_acc[1], float(row.get('light', 0.5)) * GRAVITY)
        acc_des = (along[0] - air_acc[0], ay_des, along[1] - air_acc[2])
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        in_burn = rh <= FLOOR_M + 0.05 and (state['h'] < 22.0 or state['v'] <= -0.5 * row['pace'])
        ay_thrust = ay_stop if in_burn else max(GRAVITY + up_pull, -1.5 * GRAVITY) - air_acc[1]
        if row['height'] < 0.0:
            v_fwd = s * vel[0] + c * vel[2]
            r_des, w_ff, f_des = flare(WING, v_fwd, state['h'], state['v'], yaw, rate, turn[1][1])
            stunt, inv = False, 0.0
            if state['h'] <= 0.025 and abs(v_fwd) <= 0.03 and row['height'] <= -0.012:
                self.airborne, law['spot'] = False, [at[0], at[2]]
        else:
            r_des, w_ff, f_des, stunt, inv = self._target_attitude(
                law, row, state, turn, yaw, rate, acc_des, ay_thrust, dt)

        f_des = max(0.0, min(MASS_KG * reach, f_des))
        tau_x, tau_y, tau_z = WING.trimmed(_so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD)),
            u, turn, spin)

        w_top = limits[0] if limits else pull.held(pull.I_MAX)
        w_vect_top = limits[1] if limits else w_top
        sp = state.get('speeds', (0.0, 0.0))
        ask = max(abs(rate) / STUNT_YAW_RAD_S, math.hypot(*along) / (0.5 * GRAVITY))
        boost = max(float(stunt or inv >= 0.5), abs(knife), ease((ask - BOOST[0]) / (BOOST[1] - BOOST[0])))
        w_gov = GOV_RAD_S * (1.0 + (STUNT_BOOST - 1.0) * boost)
        w_ref = max(w_gov, float(sp[0] if sp[0] > 1.0 else self.Controls[0]))
        along_u = sum(u[a] * turn[a][1] for a in range(3))
        f_comp = f_des / max(INFLOW_FLOOR, _aero_inflow(
            along_u if f_des >= 0.0 else -along_u, math.hypot(u[0], u[2]), w_ref))
        w_p_ask = min(w_top, w_gov)
        w_p_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_p_ask)
        q_p = w_p_cur * w_p_cur
        throt = max(-1.0, min(1.0, f_comp / (K_THRUST * q_p)))
        elev = max(-1.0, min(1.0, tau_x / (K_PITCH * q_p)))
        ail = max(-1.0, min(1.0, tau_z / (K_ROLL * q_p)))

        w_v_ask = min(w_vect_top, max(VECT_GOV_RAD_S, VECT_RATIO * w_p_ask))
        w_v_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_v_ask)
        rud = max(-1.0, min(1.0, (tau_y + _prop_drag(throt, elev, ail, w_p_cur)) / (K_YAW * w_v_cur * w_v_cur)))

        throt, elev, ail, rud = controls = se3mpc.blended(
            self, law, row, state, r_des, (w_p_cur, w_v_cur), dt, (throt, elev, ail, rud))
        self.Controls = (w_p_ask, w_v_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        law['held'] = bool(
            not stunt and rh >= 0.0 and abs(rh - state['h']) <= aerokin.HELD_M
            and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(at[0] - law['spot'][0], at[2] - law['spot'][1]) <= aerokin.HELD_SPOT_M and sign_ok)
        law['sequence'] += 1
        bound = int(abs(throt) >= 0.999) | (int(abs(rud) >= 0.999) << 1)
        self.out = Answer((w_p_ask, w_v_ask), controls,
                          (K_THRUST * q_p * throt, K_YAW * w_v_cur * w_v_cur * rud),
                          (c, s), tuple(self.wind), bound)
        return self.out
