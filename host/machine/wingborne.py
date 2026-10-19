"""A plane's wing and tail in the air, its run on its wheels and its law wing-borne.

    wing = Wing(MASS_KG, GRAVITY, RHO, WING_AREA, CL_ALPHA, WING_SPAN, STAB_AREA, FIN_AREA, TAIL_ARM, SIDE_AREA)
    lift = wing.lift(u, turn)                      # N, the wing's and the side's, the air `u` through it
    pitch, roll, yaw = wing.moments(u, turn, qv)   # N m about its axes
    tau = wing.trimmed(tau, u, turn, spin)         # less its own moments, as it turns
    r_des, w_ff, f_des = run(wing, core, v_fwd, h, yaw, rate)    # the takeoff run
    r_des, want_up = plane(wing, turn, acc_des, u, w, want_up, knife)   # nose on the air
    yaw = lined(runway, heading, vel, at, h, wing.rotate)        # low, onto the runway's line

The run, at RUN_G of gravity: the wing bears the weight at ALPHA_RAD, so fast (`Wing.rotate`);
the tail comes up at TAIL_UP of it, the nose to CLIMB_RAD over ALPHA_RAD once the wheels are
off, the climb on the wing until it is airborne at AIRBORNE_M, the hover's brake from there
(`Wing.runway`). Wing-borne - nose first through the air at the wing's speed - the nose goes
on the air and over it by the angle the wing needs for what is asked across the path, no more
than ALPHA_MAX, its back rolled to that; leant against the air as a hover, the wing's 1.5 g
turned a vertical line into a 45 deg tumble.
"""
import math

from machine.figure import cross
from machine.helicopter.kit import _mat_mul, _rot_x, _rot_y

ALPHA_RAD, TAIL_UP, CLIMB_RAD, AIRBORNE_M, RUN_G, CLIMB_V = (
    math.radians(10.0), 0.4, math.radians(12.0), 0.85, 1.2, 1.3)
ALPHA_MAX = math.radians(15.0)
#: The airfoil stalls at STALL_RAD, a plate over STALL_BLEND past it; its drag CD_LIFT of its
#: lift squared (an aspect ratio of 5).
STALL_RAD, STALL_BLEND, CD_LIFT = math.radians(15.0), math.radians(8.0), 0.07
#: On its wheels: the nose this far from the vertical, 12 deg up; the flare's limit, 16 deg.
PITCH_REST, HARRIER_RAD = math.radians(78.0), math.radians(16.0)


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

    def trimmed(self, tau, u, turn, spin):
        """The law's torques `tau` less the airframe's own moments in the air `u` turning at `spin`:
        on its side the fin's, 4.5 N m at 12 m/s, held the slip 8 deg short and the plane sank;
        on its wing the stabiliser's, 1.9 N m at 9 deg, held the angle 2.6 deg short; the 1.6 m
        wing's damping, 3.5 N m a rad/s at 17 m/s, rolled it onto its side at 15 deg/s."""
        return tuple(t - m for t, m in zip(tau, self.moments(u, turn, (0.0,) * 3 + tuple(spin))))

    def moments(self, u, turn, qv):
        """The airframe's own moments about its axes, N m: the stabiliser's and the fin's lift
        at the arm turning the nose into the air it flies through and against its rates, the
        wing's against its roll - a tail slide's turn it over, the propwash's surfaces hold it."""
        ux, uy, uz = (sum(u[a] * float(turn[a][k]) for a in range(3)) for k in range(3))
        wx, wy, wz = float(qv[3]), float(qv[4]), float(qv[5])
        q, qd = self.q * uy, self.q * abs(uy)
        return (self.arm * self.stab * (q * uz - qd * self.arm * wx),
                -qd * self.area * (0.25 * self.span) ** 2 * wy,
                -self.arm * self.fin * (q * ux + qd * self.arm * wz))


def run(wing, core, v_fwd, h, yaw, rate):
    """The run on its wheels: `(r_des, w_ff, f_des)` for `core` going `v_fwd` m/s along `yaw`,
    `h` m over its rest; rotated at the wing's speed it asks the climb and stays rotated, its
    tail wheel holding it at its rest till it lifts; the tail clear of the floor before."""
    core.rotated = core.rotated or v_fwd >= wing.rotate
    pitch = (0.5 * math.pi - CLIMB_RAD - ALPHA_RAD if core.rotated
             else PITCH_REST if v_fwd < TAIL_UP * wing.rotate else 0.5 * math.pi)
    r_des = _mat_mul(_rot_y(yaw), _rot_x(pitch))
    g = wing.gravity
    f_run = wing.mass * max(0.20 * g, min(RUN_G * g, 2.5 * (CLIMB_V * wing.rotate - v_fwd)
                                           + g * math.sin(0.5 * math.pi - pitch)))
    return r_des, (0.0, rate, 0.0), f_run


def flare(wing, v_fwd, h, v, yaw, rate, r11):
    """The flare and roll-out: `(r_des, w_ff, f_des)` pitching up as `1 / v_fwd^2` to HARRIER_RAD."""
    g, m = wing.gravity, wing.mass
    q_fl = wing.q * wing.area * wing.cl_alpha * max(1.0, v_fwd * v_fwd)
    a_fl = min(HARRIER_RAD, max(0.0, m * max(0.0, g - 6.0 * (0.3 + v)) / q_fl - math.atan2(v, max(1.0, v_fwd))))
    r_des = _mat_mul(_rot_y(yaw), _rot_x(0.5 * math.pi - a_fl if h > 0.03 and v_fwd > 2.0 else PITCH_REST))
    f_des = m * max(0.0, g - 3.0 * (0.4 + v)) * r11 if v_fwd <= 2.0 and r11 > 0.4 and h > 0.04 else 0.0
    return r_des, (0.0, rate, 0.0), f_des


def plane(wing, turn, acc, u, w, want_up, knife=0.0):
    """Wing-borne `w` (0 a hover, 1 at the wing's speed) through the air `u` asked `acc`: the
    nose `want_up` blended onto the air and over it by the wing's angle, and past half wing-borne
    the whole attitude `r_des`, its back to what is asked across the path - little asked, where
    it is. `knife` 1 (-1) rolls it a quarter turn about the nose, its left (right) wing where its
    back was: a knife-edge, the side's plate at the slip `rho S v^2 sin b = need cos b - along
    sin b` bearing what the wing did."""
    fast = math.hypot(*u)
    if w <= 0.0 or fast <= 1e-3:
        return None, want_up
    v_hat = [u[a] / fast for a in range(3)]
    along = sum(acc[a] * v_hat[a] for a in range(3))
    perp = [acc[a] - along * v_hat[a] for a in range(3)]
    size = math.hypot(*perp)
    # Its back across the path as it is; asked under it, it pushes - a plane does not roll
    # onto its back to go down - else its back comes round to what is asked.
    cr, sr = math.cos(0.5 * math.pi * knife), math.sin(0.5 * math.pi * knife)
    back = [sr * turn[a][0] - cr * turn[a][2] for a in range(3)]
    bv = sum(back[a] * v_hat[a] for a in range(3))
    back = [back[a] - bv * v_hat[a] for a in range(3)]
    n = math.hypot(*back) or 1.0
    back = [x / n for x in back]
    under = sum(perp[a] * back[a] for a in range(3))
    if under < 0.0:
        d, sign, need = back, -1.0, -under
    else:
        d = [perp[a] + max(0.0, 3.0 - size) * back[a] for a in range(3)]
        n = math.hypot(*d) or 1.0
        d, sign, need = [x / n for x in d], 1.0, size
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
    alpha += abs(knife) * (sign * math.atan2(need, wing.rho * wing.side * fast * fast / wing.mass + al_pos) - alpha)
    # Asked to slow, a propeller pulling one way, it pulls up: its speed goes into height until
    # the hover's law has it; coasting on it ran 45 m into a tree.
    if along < 0.0 and under >= 0.0:
        alpha = max(alpha, ALPHA_MAX * min(1.0, -along / wing.gravity))
    nose = [math.cos(alpha) * v_hat[a] + math.sin(alpha) * d[a] for a in range(3)]
    want_up = [(1.0 - w) * want_up[a] + w * nose[a] for a in range(3)]
    n = math.hypot(*want_up) or 1.0
    y = [x / n for x in want_up]
    if w <= 0.5:
        return None, y
    dy = sum(d[a] * y[a] for a in range(3))
    back = [d[a] - dy * y[a] for a in range(3)]
    n = math.hypot(*back) or 1.0
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
