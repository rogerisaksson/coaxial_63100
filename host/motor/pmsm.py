"""A PMSM in its own dq frame, an inverter's average voltage in front of it, and its constants."""
import math

TWO_PI = 2.0 * math.pi
#: One revolution a minute, in radians a second.
RAD_S_PER_RPM = TWO_PI / 60.0
#: The dq frame's torque factor - three halves, the power-invariant form.
TORQUE_FACTOR = 1.5
#: sin 60, as the firmware's drive_math.c rounds it.
HALF_SQRT3 = 0.8660254


def flux_from_kv(kv_rpm_per_volt, pole_pairs):
    """Peak per-phase flux linkage, Wb, from a hobby motor's KV."""
    return 60.0 / (math.sqrt(3.0) * TWO_PI * pole_pairs * kv_rpm_per_volt)


#: The winding against still air, and its heat capacity: placeholders,
#: the order of magnitude an outrunner of this size has, measured
#: against nothing - a motor profile carries its own pair, and a bench
#: with a thermocouple writes real ones over them. They exist so an
#: estimate is made from a number that travels with the machine rather
#: than one written at whatever call site wanted a temperature.
WINDING_K_PER_W = 2.2
WINDING_J_PER_K = 180.0


class Parameters:

    """One machine's constants, and where they came from."""

    __slots__ = ('name', 'r', 'ld', 'lq', 'lam', 'poles', 'j', 'b',
                 'sat', 'i_sat', 'measured', 'source',
                 'winding_k_per_w', 'winding_j_per_k')

    def __init__(self, name, r, ld, lq, lam, poles, j=2e-5, b=1e-5,
                 sat=0.0, i_sat=5.0, measured=False, source='',
                 winding_k_per_w=WINDING_K_PER_W,
                 winding_j_per_k=WINDING_J_PER_K):
        self.name = name
        self.r, self.ld, self.lq, self.lam = r, ld, lq, lam
        self.poles = poles              # pole pairs
        self.j, self.b = j, b
        #: How far Ld bends at `i_sat` of d current. The polarity
        #: pulse has nothing to find without it: injection locks the
        #: d axis, and only saturation says which end is the magnet.
        self.sat, self.i_sat = sat, i_sat
        self.measured = measured
        self.source = source
        #: How the winding sheds what it makes, and how much it holds.
        #: Not measured and not measurable from the terminals - `measured`
        #: says nothing about these two, and a page that estimates a
        #: winding temperature from them says estimate.
        self.winding_k_per_w = winding_k_per_w
        self.winding_j_per_k = winding_j_per_k

    @property
    def kv(self):
        """The KV this flux linkage implies, by `flux_from_kv` inverted."""
        got = math.sqrt(3.0) * TWO_PI * self.poles * self.lam
        return 60.0 / got if got else float('inf')

    @property
    def saliency(self):
        """Lq / Ld."""
        return self.lq / self.ld if self.ld else float('inf')

    def __repr__(self):
        return ('<%s %s: R %.4f ohm, Ld %.1f uH, Lq %.1f uH, '
                'lambda %.5f Wb, %d pole pairs, KV %.0f>'
                % (self.name, 'measured' if self.measured else 'ESTIMATED',
                   self.r, self.ld * 1e6, self.lq * 1e6, self.lam,
                   self.poles, self.kv))


class Motor:

    """A PMSM in its own dq frame, an inverter in front of it."""

    def __init__(self, r=0.05, ld=20e-6, lq=30e-6, lam=0.005, p=7,
                 j=2e-5, b=1e-5, load=0.0, sat=0.0, i_sat=5.0,
                 v_dt=0.0, i_knee=0.3, theta=0.0, locked=False, sub=10,
                 k_load=0.0):
        self.r, self.ld0, self.lq, self.lam, self.p = r, ld, lq, lam, p
        self.j, self.b, self.load = j, b, load
        #: Quadratic drag, N.m/(rad/s)^2 - a propeller. `b` cannot stand in
        #: for one: linear drag reaches the same speed at a torque that is
        #: wrong everywhere except the point it was fitted at.
        self.k_load = k_load
        self.sat, self.i_sat, self.v_dt, self.i_knee = sat, i_sat, v_dt, i_knee
        self.theta = theta        # electrical
        self.omega = 0.0          # electrical
        self.id = self.iq = 0.0
        self.locked = locked
        self.sub = sub

    @classmethod
    def of(cls, params, **kw):
        """A motor from a `Parameters`, so a caller names a machine once."""
        got = {'sat': params.sat, 'i_sat': params.i_sat}
        got.update(kw)
        return cls(r=params.r, ld=params.ld, lq=params.lq, lam=params.lam,
                   p=params.poles, j=params.j, b=params.b, **got)

    def ld(self):
        return self.ld0 * (1.0 - self.sat * math.tanh(self.id / self.i_sat))

    def currents(self):
        """Phase currents now, amplitude-invariant."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        ia = self.id * c - self.iq * s
        ib_ = self.id * s + self.iq * c
        return (ia, -0.5 * ia + HALF_SQRT3 * ib_, -0.5 * ia - HALF_SQRT3 * ib_)

    def advance(self, duty, vdc, ts):
        """One PWM period at these duties - the average-voltage model."""
        mean = sum(duty) / 3.0
        v = [vdc * (d - mean) for d in duty]
        if self.v_dt:
            for k, i in enumerate(self.currents()):
                v[k] -= self.v_dt * math.tanh(i / self.i_knee)
        va = (2.0 * v[0] - v[1] - v[2]) / 3.0
        vb = (v[1] - v[2]) / math.sqrt(3.0)
        dt = ts / self.sub
        for _ in range(self.sub):
            c, s = math.cos(self.theta), math.sin(self.theta)
            vd = va * c + vb * s
            vq = vb * c - va * s
            ld = self.ld()
            did = (vd - self.r * self.id + self.omega * self.lq * self.iq) / ld
            diq = (vq - self.r * self.iq - self.omega * ld * self.id
                   - self.omega * self.lam) / self.lq
            self.id += did * dt
            self.iq += diq * dt
            if not self.locked:
                torque = 1.5 * self.p * (self.lam * self.iq
                                         + (ld - self.lq) * self.id * self.iq)
                wm = self.omega / self.p
                drag = self.b * wm + self.k_load * wm * abs(wm)
                wm += (torque - drag - self.load) / self.j * dt
                self.omega = wm * self.p
            self.theta = (self.theta + self.omega * dt) % TWO_PI
