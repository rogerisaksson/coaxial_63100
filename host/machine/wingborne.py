"""A plane's wing and tail in the air, its run on its wheels and its law wing-borne.

    wing = Wing(MASS_KG, GRAVITY, RHO, WING_AREA, CL_ALPHA, WING_SPAN, STAB_AREA, FIN_AREA, TAIL_ARM)
    lift = wing.lift(u, turn)                      # N along its back, the air `u` through it
    pitch, roll, yaw = wing.moments(u, turn, qv)   # N m about its axes
    r_des, w_ff, f_des = run(wing, core, v_fwd, h, yaw, rate)    # the takeoff run
    r_des, want_up = plane(wing, turn, acc_des, u, w, want_up)   # nose on the air

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
from machine.helicopter import _mat_mul, _rot_x, _rot_y

ALPHA_RAD, TAIL_UP, CLIMB_RAD, AIRBORNE_M, RUN_G, CLIMB_V = (
    math.radians(10.0), 0.4, math.radians(12.0), 0.85, 1.2, 1.3)
ALPHA_MAX = math.radians(15.0)
#: On its wheels: the nose this far from the vertical, 12 deg up.
PITCH_REST = math.radians(78.0)


class Wing:

    """The wing, the stabiliser and the fin of a plane of `mass` kg: the air's `rho`, the wing's
    `area` m^2, `cl_alpha` a radian, its `span`; the stabiliser's and the fin's areas at
    `tail_arm` behind the middle."""

    def __init__(self, mass, gravity, rho, area, cl_alpha, span, stab_area, fin_area, tail_arm):
        self.mass, self.gravity, self.q = mass, gravity, 0.5 * rho * cl_alpha
        self.area, self.span, self.stab, self.fin, self.arm = area, span, stab_area, fin_area, tail_arm
        #: The speed the wing bears the weight at ALPHA_RAD, m/s.
        self.rotate = math.sqrt(2.0 * mass * gravity / (rho * area * cl_alpha * ALPHA_RAD))

    def runway(self, lean):
        """The run, the climb and a hover's brake at `lean` m/s^2 laid end to end, m."""
        v2 = self.rotate ** 2
        return v2 / (2.0 * RUN_G * self.gravity) + AIRBORNE_M / math.tan(CLIMB_RAD) + v2 / (2.0 * lean)

    def lift(self, u, turn):
        """The wing's lift, N, in the world along the back (`-turn[:, 2]`), nose first through
        the air `u`; none tail first; 1.5 of the weight at most."""
        uy = max(0.0, sum(u[a] * float(turn[a][1]) for a in range(3)))
        uz = sum(u[a] * float(turn[a][2]) for a in range(3))
        lim = 1.5 * self.mass * self.gravity
        f_l = max(-lim, min(lim, self.q * self.area * uy * uz))
        return (-f_l * float(turn[0][2]), -f_l * float(turn[1][2]), -f_l * float(turn[2][2]))

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
    `h` m over its rest; rotated at the wing's speed it stays rotated, on its wheels no further
    than it rests, the tail clear of the floor."""
    core.rotated = core.rotated or v_fwd >= wing.rotate
    pitch = ((0.5 * math.pi - CLIMB_RAD - ALPHA_RAD if h > 0.1 else PITCH_REST) if core.rotated
             else PITCH_REST if v_fwd < TAIL_UP * wing.rotate else 0.5 * math.pi)
    r_des = _mat_mul(_rot_y(yaw), _rot_x(pitch))
    g = wing.gravity
    f_run = wing.mass * max(0.20 * g, min(RUN_G * g, 2.5 * (CLIMB_V * wing.rotate - v_fwd)
                                           + g * math.sin(0.5 * math.pi - pitch)))
    return r_des, (0.0, rate, 0.0), f_run


def plane(wing, turn, acc, u, w, want_up):
    """Wing-borne `w` (0 a hover, 1 at the wing's speed) through the air `u` asked `acc`: the
    nose `want_up` blended onto the air and over it by the wing's angle, and past half wing-borne
    the whole attitude `r_des`, its back to what is asked across the path - little asked, where
    it is."""
    fast = math.hypot(*u)
    if w <= 0.0 or fast <= 1e-3:
        return None, want_up
    v_hat = [u[a] / fast for a in range(3)]
    along = sum(acc[a] * v_hat[a] for a in range(3))
    perp = [acc[a] - along * v_hat[a] for a in range(3)]
    size = math.hypot(*perp)
    # Its back across the path as it is; asked under it, it pushes - a plane does not roll
    # onto its back to go down - else its back comes round to what is asked.
    back = [-turn[a][2] for a in range(3)]
    bv = sum(back[a] * v_hat[a] for a in range(3))
    back = [back[a] - bv * v_hat[a] for a in range(3)]
    n = math.hypot(*back) or 1.0
    back = [x / n for x in back]
    under = sum(perp[a] * back[a] for a in range(3))
    if under < 0.0:
        d, sign = back, -1.0
    else:
        d = [perp[a] + max(0.0, 3.0 - size) * back[a] for a in range(3)]
        n = math.hypot(*d) or 1.0
        d, sign = [x / n for x in d], 1.0
    alpha = sign * min(ALPHA_MAX, wing.mass * abs(under if under < 0.0 else size)
                       / (wing.q * wing.area * fast * fast))
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
    return tuple((x[a], y[a], z[a]) for a in range(3)), y
