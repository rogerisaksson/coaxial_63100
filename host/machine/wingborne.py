"""A plane's wing and tail in the air, its run on its wheels and its law wing-borne.

    wing = Wing(MASS_KG, GRAVITY, RHO, WING_AREA, CL_ALPHA, WING_SPAN, STAB_AREA, FIN_AREA, TAIL_ARM, SIDE_AREA)
    lift = wing.lift(u, turn)                      # N, the wing's and the side's, the air `u` through it
    pitch, roll, yaw = wing.moments(u, turn, qv)   # N m about its axes
    tau = wing.trimmed(tau, u, turn, spin)         # less its own moments, as it turns
    r_des, w_ff, f_des = run(wing, core, v_fwd, h, yaw, rate)    # the takeoff run
    r_des, want_up = plane(wing, turn, acc_des, u, w, want_up, knife)   # nose on the air
    r = toward(r_hover, r_des, whole(w))                         # the wing's share of it
    yaw = lined(runway, heading, vel, at, h, wing.rotate)        # low, onto the runway's line

The run, at RUN_G of gravity: the wing bears the weight at ALPHA_RAD, so fast (`Wing.rotate`);
the tail comes up at TAIL_UP of it, the nose to CLIMB_RAD over ALPHA_RAD once the wheels are
off, the climb on the wing until it is airborne at AIRBORNE_M, the hover's brake from there
(`Wing.runway`). Wing-borne - nose first through the air at the wing's speed - the nose goes
on the air and over it by the angle the wing needs for what is asked across the path, no more
than ALPHA_MAX, its back rolled to that; leant against the air as a hover, the wing's 1.5 g
turned a vertical line into a 45 deg tumble.
"""
import ctypes
import math

from machine.aerobatics import ease
from machine.figure import cross
from machine.helicopter.kit import _aero_lib, _mat_mul, _rot_x, _rot_y, _so3_error

_d3, _d4, _d6, _d7, _d9 = (
    ctypes.c_double * 3, ctypes.c_double * 4, ctypes.c_double * 6, ctypes.c_double * 7, ctypes.c_double * 9)
_R_BUF, _TO_BUF, _OUT_R = _d9(), _d9(), _d9()

ALPHA_RAD, TAIL_UP, CLIMB_RAD, AIRBORNE_M, RUN_G, CLIMB_V = (
    math.radians(10.0), 0.4, math.radians(12.0), 0.85, 1.2, 1.3)
ALPHA_MAX = math.radians(15.0)
#: The airfoil stalls at STALL_RAD, a plate over STALL_BLEND past it; its drag CD_LIFT of its
#: lift squared (an aspect ratio of 5).
STALL_RAD, STALL_BLEND, CD_LIFT = math.radians(15.0), math.radians(8.0), 0.07
#: On its wheels: the nose this far from the vertical, 12 deg up; the flare's limit, 16 deg.
PITCH_REST, HARRIER_RAD = math.radians(78.0), math.radians(16.0)
#: On its wheels the tail comes up from TAIL_UP of the wing's speed and the nose rotates up to
#: the wing's, each over RUN_EASE of it: stepped, the rotation asked 22 deg in a tick.
RUN_EASE = 0.15
#: Wing-borne the whole attitude the wing's, eased in over WHOLE of the share: handed over at
#: half, the share's crossing stepped the target 1.8 deg in a tick, 127 rad/s^2, the rudder 0.5.
WHOLE = (0.4, 0.6)


def whole(w):
    """How much of the attitude is the wing's, wing-borne `w`: none to WHOLE[0], all from WHOLE[1]."""
    return ease((w - WHOLE[0]) / (WHOLE[1] - WHOLE[0]))


def toward(r, to, k):
    """The attitude `k` of the way from `r` to `to` along SO(3)'s geodesic."""
    lib = _aero_lib()
    if lib is not None:
        r0, r1, r2 = r
        t0, t1, t2 = to
        _R_BUF[:] = (r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2])
        _TO_BUF[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
        lib.aeh_toward(_R_BUF, _TO_BUF, k, _OUT_R)
        return ((_OUT_R[0], _OUT_R[1], _OUT_R[2]),
                (_OUT_R[3], _OUT_R[4], _OUT_R[5]),
                (_OUT_R[6], _OUT_R[7], _OUT_R[8]))
    e0, e1, e2 = _so3_error(r, to)
    x, y, z = k * e0, k * e1, k * e2
    a = math.sqrt((x * x + y * y) + z * z)
    s, c = (math.sin(a) / a, (1.0 - math.cos(a)) / (a * a)) if a > 1e-9 else (1.0, 0.5)
    k_ = ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0))
    k2 = _mat_mul(k_, k_)
    turn = tuple(tuple(float(i == j) + s * k_[i][j] + c * k2[i][j] for j in (0, 1, 2)) for i in (0, 1, 2))
    return _mat_mul(r, turn)


class Wing:

    """The wing, the stabiliser and the fin of a plane of `mass` kg: the air's `rho`, the wing's
    `area` m^2, `cl_alpha` a radian, its `span`; the stabiliser's and the fin's areas at
    `tail_arm` behind the middle; `side_area`, what the fuselage and the fin show the air
    across the span."""

    def __init__(self, mass, gravity, rho, area, cl_alpha, span, stab_area, fin_area, tail_arm, side_area):
        self.mass, self.gravity, self.q = mass, gravity, 0.5 * rho * cl_alpha
        self.rho, self.cl_alpha = rho, cl_alpha
        self.area, self.span, self.stab, self.fin, self.arm = area, span, stab_area, fin_area, tail_arm
        self.side = side_area
        #: The speed the wing bears the weight at ALPHA_RAD, m/s.
        self.rotate = math.sqrt(2.0 * mass * gravity / (rho * area * cl_alpha * ALPHA_RAD))
        self._w_lift = _d4(rho, side_area, area, cl_alpha)
        self._w_mom = _d6(self.q, tail_arm, stab_area, area, span, fin_area)
        self._w_plane = _d7(self.q, area, mass, cl_alpha, rho, side_area, gravity)
        self._u, self._turn, self._spin, self._out3, self._out9 = _d3(), _d9(), _d3(), _d3(), _d9()
        self._acc, self._want = _d3(), _d3()

    def runway(self, lean):
        """The run, the climb and a hover's brake at `lean` m/s^2 laid end to end, m."""
        v2 = self.rotate ** 2
        return v2 / (2.0 * RUN_G * self.gravity) + AIRBORNE_M / math.tan(CLIMB_RAD) + v2 / (2.0 * lean)

    def lift(self, u, turn):
        """The airframe's force, N, in the world, through the air `u`. The wing's, nose first (none
        tail first): its lift across the air in the plane of the nose and the back, rising with
        the angle to the stall at STALL_RAD and falling to a flat plate's past it, its drag along
        the air. Along the back the lift was a drag of L sin a: at 12 m/s, 9 deg, the thrust
        bore 39 % of the weight. The side's, a flat plate across the span, `rho S |u| u_x`
        against the slip: with the wings vertical, what holds a knife-edge up beside the thrust."""
        lib = _aero_lib()
        if lib is not None:
            t0, t1, t2 = turn
            self._u[:] = u
            self._turn[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
            lib.aeh_wing_lift(self._w_lift, self._u, self._turn, self._out3)
            return (self._out3[0], self._out3[1], self._out3[2])
        span = [float(turn[a][0]) for a in range(3)]
        slip = -self.rho * self.side * math.sqrt(sum(x * x for x in u)) * sum(u[a] * span[a] for a in range(3))
        uy = max(0.0, sum(u[a] * float(turn[a][1]) for a in range(3)))
        uz = sum(u[a] * float(turn[a][2]) for a in range(3))
        v2 = uy * uy + uz * uz
        if uy <= 0.0 or v2 < 1e-6:
            return tuple(slip * span[a] for a in range(3))
        alpha = math.atan2(uz, uy)
        a_abs = abs(alpha)
        # The plate's normal force past the stall, the airfoil's lift before it, blended over
        # STALL_BLEND; the airfoil's drag a share of its lift squared.
        flat = min(1.0, max(0.0, (a_abs - STALL_RAD) / STALL_BLEND))
        sa, ca = math.sin(alpha), math.cos(alpha)
        c_lin = self.cl_alpha * alpha
        c_l = (1.0 - flat) * c_lin + flat * 2.0 * sa * ca
        c_d = (1.0 - flat) * CD_LIFT * c_lin * c_lin + flat * 2.0 * sa * sa
        q = 0.5 * self.rho * self.area * v2
        f_l, f_d = q * c_l, q * c_d
        speed = math.sqrt(v2)
        nose, back = [float(turn[a][1]) for a in range(3)], [-float(turn[a][2]) for a in range(3)]
        air = [(uy * nose[a] - uz * back[a]) / speed for a in range(3)]
        up = [(uz * nose[a] + uy * back[a]) / speed for a in range(3)]
        return tuple(slip * span[a] + f_l * up[a] - f_d * air[a] for a in range(3))

    def jacobian(self, u, turn):
        """Finite-difference Jacobian of `lift` over `u` divided by `mass`, as a flat 9-double
        buffer for `_observe_wind_spin` (or 3x3 list when no C compiler is present)."""
        lib = _aero_lib()
        if lib is not None:
            t0, t1, t2 = turn
            self._u[:] = u
            self._turn[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
            lib.aeh_wing_jacobian(self._w_lift, self._u, self._turn, self.mass, self._out9)
            return self._out9
        ends = [[self.lift([u[k] + d * (k == b) for k in range(3)], turn) for d in (0.1, -0.1)]
                for b in range(3)]
        return [[(ends[b][1][a] - ends[b][0][a]) / (0.2 * self.mass) for b in range(3)]
                for a in range(3)]

    def trimmed(self, tau, u, turn, spin):
        """The law's torques `tau` less the airframe's own moments in the air `u` turning at `spin`:
        on its side the fin's, 4.5 N m at 12 m/s, held the slip 8 deg short and the plane sank;
        on its wing the stabiliser's, 1.9 N m at 9 deg, held the angle 2.6 deg short; the 1.6 m
        wing's damping, 3.5 N m a rad/s at 17 m/s, rolled it onto its side at 15 deg/s."""
        m0, m1, m2 = self.moments(u, turn, (0.0, 0.0, 0.0, spin[0], spin[1], spin[2]))
        return (tau[0] - m0, tau[1] - m1, tau[2] - m2)

    def moments(self, u, turn, qv):
        """The airframe's own moments about its axes, N m: the stabiliser's and the fin's lift
        at the arm turning the nose into the air it flies through and against its rates, the
        wing's against its roll - a tail slide's turn it over, the propwash's surfaces hold it."""
        lib = _aero_lib()
        if lib is not None:
            t0, t1, t2 = turn
            self._u[:] = u
            self._turn[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
            self._spin[:] = (qv[3], qv[4], qv[5])
            lib.aeh_wing_moments(self._w_mom, self._u, self._turn, self._spin, self._out3)
            return (self._out3[0], self._out3[1], self._out3[2])
        ux, uy, uz = (sum(u[a] * float(turn[a][k]) for a in range(3)) for k in range(3))
        wx, wy, wz = float(qv[3]), float(qv[4]), float(qv[5])
        q, qd = self.q * uy, self.q * abs(uy)
        return (self.arm * self.stab * (q * uz - qd * self.arm * wx),
                -qd * self.area * (0.25 * self.span) ** 2 * wy,
                -self.arm * self.fin * (q * ux + qd * self.arm * wz))


#: On its wheels, WHEELS_M of its rest, it rolls along them and grips across them: GRIP of its
#: load at the most, its slip across taken back over GRIP_S. On skids alone, sliding, a 1.1 m/s
#: crosswind carried it 2.3-3.3 m off its line on the roll-out.
GRIP, GRIP_S, WHEELS_M = 0.8, 0.05, 0.05


def wing_substep(sky, lift, wx, wy, wz, dk, ab0, ab1, ab2):
    """One STEP_S MuJoCo substep on `sky` with `sky.wing`'s lift, side force and moments, and
    its wheels' grip across them."""
    from machine.quad import substep
    np, xm, qv, sp, w = sky._np, sky._xmat, sky._vel, sky._spin, sky.wing
    vx, vy, vz = qv[0] - wx, qv[1] - wy, qv[2] - wz
    ux = vx * xm[0] + vy * xm[3] + vz * xm[6]
    uy0 = vx * xm[1] + vy * xm[4] + vz * xm[7]
    uz = vx * xm[2] + vy * xm[5] + vz * xm[8]
    slip = -w.rho * w.side * np.sqrt(vx * vx + vy * vy + vz * vz) * ux
    uy = np.maximum(0.0, uy0)
    v2 = uy * uy + uz * uz
    on = (uy > 0.0) & (v2 >= 1e-6)
    alpha = np.arctan2(uz, uy)
    flat = np.minimum(1.0, np.maximum(0.0, (np.abs(alpha) - STALL_RAD) / STALL_BLEND))
    sa, ca, c_lin = np.sin(alpha), np.cos(alpha), w.cl_alpha * alpha
    q_w = np.where(on, 0.5 * w.rho * w.area * v2, 0.0)
    f_l = q_w * ((1.0 - flat) * c_lin + flat * 2.0 * sa * ca)
    f_d = q_w * ((1.0 - flat) * CD_LIFT * c_lin * c_lin + flat * 2.0 * sa * sa)
    speed = np.where(on, np.sqrt(v2), 1.0)
    n0, n1, n2, b0, b1, b2 = xm[1], xm[4], xm[7], -xm[2], -xm[5], -xm[8]
    lx = slip * xm[0] + np.where(on, f_l * ((uz * n0 + uy * b0) / speed) - f_d * ((uy * n0 - uz * b0) / speed), 0.0)
    ly = slip * xm[3] + np.where(on, f_l * ((uz * n1 + uy * b1) / speed) - f_d * ((uy * n1 - uz * b1) / speed), 0.0)
    lz = slip * xm[6] + np.where(on, f_l * ((uz * n2 + uy * b2) / speed) - f_d * ((uy * n2 - uz * b2) / speed), 0.0)
    flat = np.sqrt(xm[0] * xm[0] + xm[6] * xm[6]) + 1e-9
    load = np.where(sky._at[1] - sky.rest < WHEELS_M, np.maximum(0.0, w.mass * w.gravity - ly), 0.0)
    grip = -np.minimum(GRIP * load, np.maximum(-GRIP * load, w.mass * (qv[0] * xm[0] + qv[2] * xm[6]) / (flat * GRIP_S)))
    lx, lz = lx + grip * xm[0] / flat, lz + grip * xm[6] / flat
    q, qd = w.q * uy0, w.q * np.abs(uy0)
    a0 = w.arm * w.stab * (q * uz - qd * w.arm * sp[0])
    a1 = -qd * w.area * (0.25 * w.span) ** 2 * sp[1]
    a2 = -w.arm * w.fin * (q * ux + qd * w.arm * sp[2])
    substep(sky, 0.0, lift, wx, wy, wz, dk, ab0 + a0, ab1 + a1, ab2 + a2, lx, ly, lz, 0.0, 0.0, 0.0)


def run(wing, core, v_fwd, h, yaw, rate):
    """The run on its wheels: `(r_des, w_ff, f_des)` for `core` going `v_fwd` m/s along `yaw`,
    `h` m over its rest; rotated by the wing's speed it asks the climb and stays rotated, its
    tail wheel holding it at its rest till it lifts; the tail clear of the floor before."""
    v = v_fwd / wing.rotate
    core.rotated = max(core.rotated, ease((v - 1.0 + RUN_EASE) / RUN_EASE))
    pitch = (PITCH_REST + (0.5 * math.pi - PITCH_REST) * ease((v - TAIL_UP) / RUN_EASE)
             - (CLIMB_RAD + ALPHA_RAD) * core.rotated)
    r_des = _mat_mul(_rot_y(yaw), _rot_x(pitch))
    g = wing.gravity
    f_run = wing.mass * max(0.20 * g, min(RUN_G * g, 2.5 * (CLIMB_V * wing.rotate - v_fwd)
                                           + g * math.sin(0.5 * math.pi - pitch)))
    return r_des, (0.0, rate, 0.0), f_run


#: The flare's sink asked: its height over FLARE_S, TOUCH_M_S at the least - onto its wheels,
#: the wheels' 2.35 m/s^2 stopping it where the float's drag slows it 1. Held off at 5 cm till
#: its wing could hold it no longer it floated 12-38 m in gusts, every landing of 13 past the
#: runway's end. On 0.3 m/s at any height it eased off from 8.8 deg at 0.29 m and touched at
#: 0.64 m/s; on 0.2 at the least, its wing lifting it on its wheels, it skimmed 34-55 m.
FLARE_S, TOUCH_M_S = 0.5, 0.15


def flare(wing, v_fwd, h, v, yaw, rate, r11):
    """The flare and roll-out at `v_fwd` through the air along its heading: `(r_des, w_ff, f_des)`
    pitching up as `1 / v_fwd^2` to HARRIER_RAD, its sink as its height asks (FLARE_S, TOUCH_M_S);
    on its wheels its tail up as the run has it at its speed, its wing off them: three-point at
    10.5 m/s the wing bore it 34-55 m."""
    g, m = wing.gravity, wing.mass
    # The lift a radian, `q` its lift slope's already: counted twice, and the path's angle added
    # to the nose where it is the wing's own, it touched at 0.64 m/s at 7-10 deg.
    q_fl = wing.q * wing.area * max(1.0, v_fwd * v_fwd)
    sink = max(TOUCH_M_S, h / FLARE_S)
    a_fl = min(HARRIER_RAD, max(0.0, m * max(0.0, g - 6.0 * (sink + v)) / q_fl + math.atan2(v, max(1.0, v_fwd))))
    rolled = PITCH_REST + (0.5 * math.pi - PITCH_REST) * ease((v_fwd / wing.rotate - TAIL_UP) / RUN_EASE)
    r_des = _mat_mul(_rot_y(yaw), _rot_x(0.5 * math.pi - a_fl if h > 0.03 and v_fwd > 2.0 else rolled))
    f_des = m * max(0.0, g - 3.0 * (0.4 + v)) * r11 if v_fwd <= 2.0 and r11 > 0.4 and h > 0.04 else 0.0
    return r_des, (0.0, rate, 0.0), f_des


def plane(wing, turn, acc, u, w, want_up, knife=0.0):
    """Wing-borne `w` (0 a hover, 1 at the wing's speed) through the air `u` asked `acc`: the
    nose `want_up` blended onto the air and over it by the wing's angle, and the wing's whole
    attitude `r_des`, its back to what is asked across the path - little asked, where it is.
    `knife` 1 (-1) rolls it a quarter turn about the nose, its left (right) wing where its back
    was: a knife-edge, its span up, the side's plate at the slip `rho S v^2 sin b = need cos b -
    along sin b` bearing what is asked up, the wing what is asked across - the span rolled to
    all of it, a gusty course's asks across rocked its belly 0.07-0.86 in a second."""
    lib = _aero_lib()
    if lib is not None:
        if w <= 0.0:
            return None, want_up
        t0, t1, t2 = turn
        wing._turn[:] = (t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
        wing._acc[:] = acc
        wing._u[:] = u
        wing._want[:] = want_up
        ok = lib.aeh_plane(wing._w_plane, wing._turn, wing._acc, wing._u, w, wing._want, knife, wing._out9)
        if not ok:
            return None, want_up
        r = wing._out9
        return (((r[0], r[1], r[2]), (r[3], r[4], r[5]), (r[6], r[7], r[8])),
                [wing._want[0], wing._want[1], wing._want[2]])
    fast = math.sqrt((u[0] * u[0] + u[1] * u[1]) + u[2] * u[2])
    if w <= 0.0 or fast <= 1e-3:
        return None, want_up
    v_hat = [u[a] / fast for a in range(3)]
    along = sum(acc[a] * v_hat[a] for a in range(3))
    perp = [acc[a] - along * v_hat[a] for a in range(3)]
    size = math.sqrt((perp[0] * perp[0] + perp[1] * perp[1]) + perp[2] * perp[2])
    # Its back across the path as it is; asked under it, it pushes - a plane does not roll
    # onto its back to go down - else its back comes round to what is asked.
    cr, sr = math.cos(0.5 * math.pi * knife), math.sin(0.5 * math.pi * knife)
    back = [sr * turn[a][0] - cr * turn[a][2] for a in range(3)]
    bv = sum(back[a] * v_hat[a] for a in range(3))
    back = [back[a] - bv * v_hat[a] for a in range(3)]
    n = math.sqrt((back[0] * back[0] + back[1] * back[1]) + back[2] * back[2]) or 1.0
    back = [x / n for x in back]
    under = sum(perp[a] * back[a] for a in range(3))
    if under < 0.0:
        d, sign, need = back, -1.0, -under
    else:
        d = [perp[a] + max(0.0, 3.0 - size) * back[a] for a in range(3)]
        n = math.sqrt((d[0] * d[0] + d[1] * d[1]) + d[2] * d[2]) or 1.0
        d, sign, need = [x / n for x in d], 1.0, size
    ka = abs(knife)
    upp = [float(a == 1) - v_hat[1] * v_hat[a] for a in range(3)]
    n = math.sqrt((upp[0] * upp[0] + upp[1] * upp[1]) + upp[2] * upp[2]) or 1.0
    upp = [x / n for x in upp]
    d = [(1.0 - ka) * d[a] + ka * upp[a] for a in range(3)]
    n = math.sqrt((d[0] * d[0] + d[1] * d[1]) + d[2] * d[2]) or 1.0
    d = [x / n for x in d]
    up_ask = sum(perp[a] * upp[a] for a in range(3))
    across = [perp[a] - up_ask * upp[a] for a in range(3)]
    k_w = wing.q * wing.area * fast * fast / wing.mass
    al_pos = max(0.0, along)
    hi = (ALPHA_MAX if sign < 0.0
          else math.atan2(need, al_pos) if need > k_w * ALPHA_MAX else ALPHA_MAX)
    lo = 0.0
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        flat = min(1.0, max(0.0, (mid - STALL_RAD) / STALL_BLEND))
        c_lin = wing.cl_alpha * mid
        a_w = (k_w / wing.cl_alpha) * (
            (1.0 - flat) * c_lin + flat * 2.0 * math.sin(mid) * math.cos(mid))
        d_w = (k_w / wing.cl_alpha) * (
            (1.0 - flat) * CD_LIFT * c_lin * c_lin + flat * 2.0 * math.sin(mid) ** 2)
        if need - (al_pos + d_w) * math.tan(mid) > a_w:
            lo = mid
        else:
            hi = mid
    alpha = sign * 0.5 * (lo + hi)
    alpha += ka * (math.atan2(up_ask, wing.rho * wing.side * fast * fast / wing.mass + al_pos) - alpha)
    across_n = math.sqrt((across[0] * across[0] + across[1] * across[1]) + across[2] * across[2])
    a_cross = min(ALPHA_MAX, across_n / max(k_w, 1e-9)) / (across_n or 1.0)
    # Asked to slow, a propeller pulling one way, it pulls up: its speed goes into height until
    # the hover's law has it; coasting on it ran 45 m into a tree.
    if along < 0.0 and under >= 0.0:
        alpha = max(alpha, ALPHA_MAX * min(1.0, -along / wing.gravity))
    nose = [math.cos(alpha) * v_hat[a] + math.sin(alpha) * d[a] + ka * a_cross * across[a] for a in range(3)]
    want_up = [(1.0 - w) * want_up[a] + w * nose[a] for a in range(3)]
    n = math.sqrt((want_up[0] * want_up[0] + want_up[1] * want_up[1]) + want_up[2] * want_up[2]) or 1.0
    y = [x / n for x in want_up]
    dy = sum(d[a] * y[a] for a in range(3))
    back = [d[a] - dy * y[a] for a in range(3)]
    n = math.sqrt((back[0] * back[0] + back[1] * back[1]) + back[2] * back[2]) or 1.0
    z = [-b / n for b in back]
    x = cross(y, z)
    x, z = [cr * x[a] - sr * z[a] for a in range(3)], [sr * x[a] + cr * z[a] for a in range(3)]
    return tuple((x[a], y[a], z[a]) for a in range(3)), y


def lined(runway, heading, vel, at, h, rotate):
    """The heading low flight comes round to: along `runway` (rad) the way nearer `heading`
    (cos, sin), up it without one, crabbed onto its line over a second of flight from 0.1 m up."""
    way = math.copysign(1.0, heading[0] * math.cos(runway) + heading[1] * math.sin(runway)) if heading else 1.0
    rc, rs = way * math.cos(runway), way * math.sin(runway)
    crab = math.atan2(rc * (vel[0] + at[0]) - rs * (vel[2] + at[2]), max(rotate, rs * vel[0] + rc * vel[2]))
    return runway + 0.5 * math.pi * (1.0 - way) - crab * min(1.0, max(0.0, 10.0 * (h - 0.1)))
