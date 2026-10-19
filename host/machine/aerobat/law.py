"""The aerobat's SO(3) law: hover and prop-hang, its stunts, on its wing nose first; its observer's."""
import math

from machine import aerokin, airfield, se3mpc
from machine.aerobatics import ease
from machine.aerobat.airframe import (A_DISC, ATT_KD, ATT_KP, FLOOR_M, GRAVITY, IDLE_RAD_S, INERTIA,
                                      K_PITCH_Q, K_ROLL_Q, K_THRUST, K_YAW, MASS_KG, RHO, ROLL_KD,
                                      ROLL_KP, ROTATE_M_S, RUNWAY_M, RUNWAY_RAD, VECT_GOV_RAD_S,
                                      VECT_RATIO, W_TOP, WATERFALL_HZ, WATERFALL_RAD, WING, YAW_KD,
                                      YAW_KP, _aero_cda, _aero_inflow, _prop_drag, prop_speed,
                                      slipstream)
from machine.aerobat.observer import observe
from machine.helicopter.kit import (Answer, _smooth_target, _so3_torques,
                                    _stunt_step, _turn_heading)
from machine.wingborne import AIRBORNE_M, STALL_RAD, WHEELS_M, flare, lined, plane, run, toward, whole

#: A row asked faster than FAST_M_S, m/s, flies on its wing whole once it is past half of it
#: (`wingborne.whole`), eased in over the span: none slower, all faster.
FAST_M_S = (3.0, 7.0)
#: Low it comes round to the runway's line where its row is `lined` (1 unsaid; a line's own
#: none): asked under LINED_M, m (all under its first), its turn and its slide eased out by
#: LINED_RAD_S and LINED_M_S - switched on as they came to none, the run-in's yaw stepped to
#: 1 rad/s and its rudder 0.56 in a tick.
LINED_M, LINED_RAD_S, LINED_M_S = (1.5, 2.0), math.radians(10.0), 1.0
#: The share of its surfaces' throw its propeller's wash is kept for.
AUTHORITY = 0.7
#: Gusty, its figures higher: its wind's swing about its mean over GUST_SLOW_S, rms over GUST_RMS_S,
#: past GUST_FLOOR, times GUST_S, m - a figure's row (`lined` 1), not a laid line's (0), which says
#: them to its giver (`aerokin.seen`). In still air its own estimate swung 0.10-0.17 m/s rms.
GUST_SLOW_S, GUST_RMS_S, GUST_FLOOR, GUST_S = 5.0, 3.0, 0.3, airfield.GUST_S
#: On its wing its climb follows its pitch, by its path's lag, m / (q S v), and its attitude
#: loop's: its height's loop no quicker than PATH_POLE over them (`path_pole`). On the hover's
#: 6.4 rad/s it swung its pull 6.7 m/s^2 at 1.8 Hz in a turn, past the stall, over 10 cm; the
#: hover's past 0.5-1 m of its height, it swung so off the hoop. On its side its thrust and its
#: side bear it, not its wing: on the wing's loop there it put its tip on the floor at 0.45 m.
PATH_POLE = 0.35
#: Its wheels on the runway, m over their rest: cut at 0.05, it dropped onto them at 1 m/s.
TOUCH_M = 0.03
#: On its wing the push its observer reads off its model, over PUSH_S: dropped there, a turn's
#: lift 14 % short of its model on a wind read 1 m/s off sank it 1.2 m on that loop.
PUSH_S = 1.0


def path_pole(v):
    """Its height's loop on its wing at `v` m/s through the air, 1/s (PATH_POLE)."""
    return PATH_POLE / (MASS_KG / (WING.q * WING.area * max(ROTATE_M_S, v)) + ATT_KD / ATT_KP)


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
        self.wind_slow, self.gust2, self.push_slow, self.landed = [0.0] * 3, 0.0, [0.0] * 3, False
        self.touched = 0.0
        self.spin_hat, self.turn_hat = [0.0] * 3, [0.0] * 3
        self.begun, self.flip_ang, self.roll_ang = False, 0.0, 0.0
        self.piro_ang, self.tic_phase = 0.0, 0.0
        self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
        self.rotated, self.wingborne, self.wing_air, self.floor = 0.0, 0.0, 0.0, FLOOR_M
        self.Controls = (IDLE_RAD_S, IDLE_RAD_S, 0.0, 0.0, 0.0, 0.0)
        self.out = Answer((IDLE_RAD_S, IDLE_RAD_S), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0),
                          (1.0, 0.0), (0.0, 0.0, 0.0))

    def _target_attitude(self, law, row, state, turn, yaw, rate, acc_des, ay_thrust, dt,
                         u=None, w_lift=None):
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
            # It climbs out no faster than its wing's loop stops at its height: at 12 deg it
            # rose 1.3 m over the lap's line into the limbo's bar 30 m on.
            return run(WING, self, v_fwd, h, yaw, rate, path_pole(v_fwd) * max(0.0, rh - h)) + (stunt, inv)

        a_norm = math.hypot(ax_des, ay_des, az_des) or GRAVITY
        s_f = -1.0 if inv >= 0.5 and ay_des < 0.0 else 1.0
        want_up = [sign * s_f * ax_des / a_norm, sign * ay_des / a_norm, sign * s_f * az_des / a_norm]
        ay_use = ay_des if inv >= 0.5 else ay_thrust
        if u is None:
            u = (float(state['vel'][0]) - self.wind[0],
                 float(state['vel'][1]) - self.wind[1],
                 float(state['vel'][2]) - self.wind[2])
        if w_lift is None:
            w_lift = WING.lift(u, turn)
        # The thrust gives along the nose what the wing does not; the hover's share is in acc_des.
        w_nose = sum(x * turn[a][1] for a, x in enumerate(w_lift))
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
        # No faster than its lean carries it round at its speed: on a wing it stalled - its
        # heading turned at the line's 115 deg/s, its nose at 64, 100 deg behind it, the pull
        # ahead asked against its nose: no thrust, into the house at 5 m/s.
        top = min(2.4, row['lean'] / max(1.0, math.hypot(vel[0], vel[2])))
        return max(-top, min(top, rate + low * (max(-top, min(top, 3.0 * err)) - rate))), target

    def margin(self, row):
        """How much higher `row` is flown in its air's gusts, m: a figure's, not a laid line's."""
        return float(row.get('lined', 1.0) >= 0.5) * GUST_S * max(0.0, math.sqrt(self.gust2) - GUST_FLOOR)

    def plan(self, law, row, state, dt, pull_max):
        """Advance outer horizontal kinematics `aerokin.lean` across `dt` s."""
        c0, s0 = law['heading']
        law['heading'] = _turn_heading((c0, s0), self.yawed(law, row, state)[0], dt)
        reach = max(aerokin.LEAST * GRAVITY, pull_max)
        self.floor = FLOOR_M * max(0.0, 1.0 - math.hypot(row.get('speed', 0.0), row.get('slide', 0.0)) / 5.0)
        r_climb = dict(row, height=max(self.floor, row['height'] + self.margin(row))) if row['height'] >= 0.0 else row
        v2 = sum((float(state['vel'][a]) - self.wind[a]) ** 2 for a in range(3))
        kd = aerokin.KD + self.wingborne * (1.0 - abs(row.get('knife', 0.0))) * (
            min(aerokin.KD, 2.0 * path_pole(math.sqrt(v2))) - aerokin.KD)
        self.up_pull, law['doing'] = aerokin.climb(
            r_climb, state['h'], state['v'], reach - GRAVITY,
            aerokin.spooled(self.out.speeds[0], math.sqrt(reach * MASS_KG / K_THRUST), IDLE_RAD_S), dt, kd)
        beside = math.sqrt(max(reach * reach - GRAVITY * GRAVITY, 0.0))
        # On its wing it turns on the wing, its lift short of the stall, not the propeller's
        # sphere: held to its 1.1 g, a 12 m turn at 20 m/s ran 9.5 m wide of the hoop. It
        # speeds up on its propeller: held to its wing's under the stall's speed, it hung
        # nose-high at 7 m/s from a hover into its circuit.
        lift = WING.q * WING.area * v2 * STALL_RAD / MASS_KG
        across = beside + self.wingborne * (math.sqrt(max(lift * lift - GRAVITY * GRAVITY, 0.0)) - beside)
        # On its wing it flies the air's speed: the ground's 14 m/s stalled it in a 7 m/s tail.
        c, s = law['heading']
        on = flies(row, self.wingborne)
        wx, wz = on * self.wind[0], on * self.wind[2]
        aired = dict(row, speed=row['speed'] + wx * s + wz * c, slide=row['slide'] + wx * c - wz * s)
        most = min(row['lean'], max(beside, across))
        ax, az = aerokin.lean(law, aired, state['at'], state['vel'], dt, most)
        ahead, side = ax * s + az * c, ax * c - az * s
        ahead = max(-min(most, beside), min(min(most, beside), ahead))
        side = max(-min(most, across), min(min(most, across), side))
        along = (ahead * s + side * c, ahead * c - side * s)
        # Nose first it flies a speed: chasing a place it lagged a turn by 12 m.
        held = 1.0 - self.airborne * (1.0 - on)
        if row['height'] >= 0.0:
            law['spot'] = [p + held * (float(state['at'][a]) - p) for p, a in zip(law['spot'], (0, 2))]
        law['heading'] = (c0, s0)
        return along

    def step(self, law, row, state, dt, pull_max, limits=None, along=None):
        """One pass of `dt` s on `row` and `state`: returns `Answer`."""
        observe(self, state, dt)
        a_slow, a_rms = min(1.0, dt / GUST_SLOW_S), min(1.0, dt / GUST_RMS_S)
        self.wind_slow = [s + a_slow * (w - s) for s, w in zip(self.wind_slow, self.wind)]
        self.gust2 += a_rms * (sum((w - s) ** 2 for w, s in zip(self.wind, self.wind_slow)) - self.gust2)
        law['gust'] = max(0.0, math.sqrt(self.gust2) - GUST_FLOOR)
        a_push = min(1.0, dt / PUSH_S)
        self.push_slow = [s + a_push * (p - s) for s, p in zip(self.push_slow, self.push_hat)]
        t_in, s_in, v_in, a_in = state['turn'], state['spin'], state['vel'], state['at']
        turn = t_in if (type(t_in) is tuple and type(t_in[0]) is tuple and type(t_in[0][0]) is float) \
            else getattr(t_in, 'tolist', lambda: [[float(x) for x in r] for r in t_in])()
        spin = s_in if (type(s_in) is tuple and type(s_in[0]) is float) else [float(x) for x in s_in]
        vel = v_in if (type(v_in) is tuple and type(v_in[0]) is float) else [float(x) for x in v_in]
        at = a_in if (type(a_in) is tuple and type(a_in[0]) is float) else [float(x) for x in a_in]
        u, knife, fast = (vel[0] - self.wind[0], vel[1] - self.wind[1], vel[2] - self.wind[2]), row.get('knife', 0.0), row.get('speed', 0.0) > 5.0
        # On its side the nose is off the air by the slip; fast, the share carried whole.
        k = flies(row, max(self.wingborne, abs(knife) * sum(x * x for x in u) / ROTATE_M_S ** 2))
        self.wingborne, self.wing_air = (row.get('wing', 1.0) * (x + k * (1.0 - x))
                                         for x in (self.wingborne, self.wing_air))
        prev_ctrl = state.get('controls', self.Controls[2:])
        drag = 0.5 * RHO * _aero_cda(*prev_ctrl[1:3]) / MASS_KG * math.hypot(*u)
        w_lift = WING.lift(u, turn)
        air_acc = [(1.0 - self.wing_air) * (self.push_hat[a] + w_lift[a] / MASS_KG)
                   + self.wing_air * self.push_slow[a] - drag * u[a] for a in (0, 1, 2)]
        if along is None:
            along = self.plan(law, row, state, dt, pull_max)
        rate, target_yaw = self.yawed(law, row, state)
        law['heading'] = c, s = _turn_heading(law['heading'], rate, dt)
        yaw = math.atan2(s, c)
        reach, up_pull = max(aerokin.LEAST * GRAVITY, pull_max), self.up_pull
        # Landed, its motors cut till a row flies it again: idled, its propeller's thrust rolled
        # it on at 5 m/s; its land's row and its run's spool alike, restarted it idled at a stand.
        landed = row['height'] < 0.0 and (self.landed or (self.airborne and state['h'] <= TOUCH_M))
        self.touched = self.touched if self.landed or not landed else self.Controls[3]
        self.landed = landed

        if row['height'] < 0.0 and (row['height'] <= -0.1 or not self.airborne):
            if state['h'] < 0.03 and max(abs(x) for x in vel) < 0.05:
                law['heading'] = c, s = math.cos(target_yaw), math.sin(target_yaw)
            self.flip_ang = self.roll_ang = self.piro_ang = self.tic_phase = 0.0
            self.r_prev, self.up_filt, self.w_ff, self.airborne = None, None, (0.0, 0.0, 0.0), False
            self.rotated = 0.0
            law['spot'] = [at[0], at[2]]
            # Its pitch fixed, the propeller idles on its wheels for its run, its motors cut where
            # its row is down: spooled to the head's 365 rad/s it pulled 36 N, six times what they
            # hold.
            ready = float(row['height'] > -0.1 and not self.landed)
            w_p_ask, w_v_ask = ready * IDLE_RAD_S, ready * VECT_GOV_RAD_S
            if limits:
                w_p_ask, w_v_ask = min(limits[0], w_p_ask), min(limits[1], w_v_ask)
            controls = (0.0, 0.0, 0.0, 0.0)
            self.Controls = (w_p_ask, w_v_ask) + controls
            law['held'] = False
            self.out = Answer((w_p_ask, w_v_ask), controls, (0.0, 0.0), (c, s), tuple(self.wind))
            return self.out

        rh = max(self.floor, float(row['height']) + self.margin(row)) if row['height'] >= 0.0 else float(row['height'])
        ay_des = max(GRAVITY + up_pull - air_acc[1], float(row.get('light', 0.5)) * GRAVITY)
        acc_des = (along[0] - air_acc[0], ay_des, along[1] - air_acc[2])
        ay_stop = max(0.0, GRAVITY + up_pull - air_acc[1])
        in_burn = rh <= FLOOR_M + 0.05 and (state['h'] < 22.0 or state['v'] <= -0.5 * row['pace'])
        ay_thrust = ay_stop if in_burn else max(GRAVITY + up_pull, -1.5 * GRAVITY) - air_acc[1]
        if row['height'] < 0.0:
            v_fwd = s * vel[0] + c * vel[2]
            # Its lift on the air's speed: on the ground's a 1.1 m/s tail wind left it 16 % short
            # of its weight, and it touched at 0.34-0.50 m/s where in still air 0.05.
            r_des, w_ff, f_des = flare(WING, v_fwd - s * self.wind[0] - c * self.wind[2], state['h'], state['v'],
                                       yaw, rate, turn[1][1])
            stunt, inv = False, 0.0
            if state['h'] <= 0.025 and abs(v_fwd) <= 0.03 and row['height'] <= -0.012:
                self.airborne, law['spot'] = False, [at[0], at[2]]
        else:
            r_des, w_ff, f_des, stunt, inv = self._target_attitude(
                law, row, state, turn, yaw, rate, acc_des, ay_thrust, dt, u, w_lift)

        f_des = max(0.0, min(MASS_KG * reach, f_des))
        tau_x, tau_y, tau_z = WING.trimmed(_so3_torques(
            self, turn, r_des, w_ff, spin, INERTIA, (ATT_KP, YAW_KP, ROLL_KP), (ATT_KD, YAW_KD, ROLL_KD)),
            u, turn, spin)

        w_top = limits[0] if limits else W_TOP
        w_vect_top = limits[1] if limits else W_TOP
        sp = state.get('speeds', (0.0, 0.0))
        # Its pitch fixed, its thrust is its speed: the propeller's for what is asked through its
        # inflow, its surfaces' moments the slipstream's at the speed it is at.
        along_u, across_u = sum(u[a] * turn[a][1] for a in range(3)), math.hypot(u[0], u[2])
        # No slower than its surfaces' wash gives what they are asked at AUTHORITY of their throw,
        # thrust or none: idled through a waterfall's top its tail read 5 Pa where a hover's 97,
        # its elevator at its stop, and it fell 4.5 m onto its back. Not on its wheels: the
        # wash's thrust rolled it out at 1.9 m/s^2 where its wheels stop it at 2.35.
        q_need = max(abs(tau_x) / K_PITCH_Q, abs(tau_z) / K_ROLL_Q) / AUTHORITY
        held = A_DISC * max(0.0, q_need - 0.5 * RHO * max(0.0, along_u) ** 2) * float(state['h'] > WHEELS_M)
        w_p_ask = max(IDLE_RAD_S, min(w_top, prop_speed(max(f_des, held), along_u, across_u)))
        w_p_cur = max(IDLE_RAD_S, float(sp[0]) if limits and sp[0] > 1.0 else w_p_ask)
        share = _aero_inflow(along_u, across_u, w_p_cur)
        thrust = K_THRUST * w_p_cur * w_p_cur * share
        q_s = max(1.0, slipstream(along_u, thrust))
        elev = max(-1.0, min(1.0, tau_x / (K_PITCH_Q * q_s)))
        ail = max(-1.0, min(1.0, tau_z / (K_ROLL_Q * q_s)))

        w_v_ask = min(w_vect_top, max(VECT_GOV_RAD_S, VECT_RATIO * w_p_ask))
        if self.landed:
            w_p_ask = w_v_ask = 0.0
        w_v_cur = max(IDLE_RAD_S, float(sp[1]) if limits and sp[1] > 1.0 else w_v_ask)
        rud = max(-1.0, min(1.0, (tau_y + _prop_drag(share, elev, ail, w_p_cur)) / (K_YAW * w_v_cur * w_v_cur)))

        _pitch, elev, ail, rud = controls = se3mpc.blended(
            self, law, row, state, r_des, (w_p_cur, w_v_cur), dt, (1.0, elev, ail, rud))
        if self.landed:
            # Its wheels hold its attitude, its rudder on the runway's line: on its law, with no
            # wash, its elevator flapped 0.24 rms from stop to stop. Its stick where its flare
            # left it - neutral, its nose dipped 2.7 deg -, back once its three points' 12 deg
            # cannot lift it off - from its rotate speed, all of it at three quarters -, its tail
            # held down.
            back = ease((ROTATE_M_S - math.hypot(u[0], u[2])) / (0.25 * ROTATE_M_S))
            _pitch, elev, ail, rud = controls = (1.0, min(self.touched, -back), 0.0, rud)
        self.Controls = (w_p_ask, w_v_ask) + controls
        sign_ok = (-turn[1][1] if inv >= 0.5 else turn[1][1]) >= math.cos(2.0 * aerokin.HELD_RAD)
        law['held'] = bool(
            not stunt and rh >= 0.0 and abs(rh - state['h']) <= aerokin.HELD_M
            and max(abs(x) for x in vel) <= aerokin.HELD_M_S
            and math.hypot(at[0] - law['spot'][0], at[2] - law['spot'][1]) <= aerokin.HELD_SPOT_M and sign_ok)
        law['sequence'] += 1
        bound = int(w_p_ask >= w_top and f_des > thrust) | (int(abs(rud) >= 0.999) << 1)
        self.out = Answer((w_p_ask, w_v_ask), controls, (thrust, K_YAW * w_v_cur * w_v_cur * rud),
                          (c, s), tuple(self.wind), bound)
        return self.out
