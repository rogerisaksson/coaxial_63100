"""A PMSM and an inverter, in one place, for everything that needs one.

The model was written inside `tests/test_drive_core.py`, where it closed
the loop around the firmware's control law. It is here now because three
other things want the same motor - the DAQ stand-in, the system
identification, and a notebook - and four copies of a machine is four
places for an inductance to drift.

WHAT IT IS AND IS NOT. It integrates the dq equations with saliency,
saturation, back-EMF, a dead-time voltage error and a mechanical load. It
is a MODEL: every number it produces is arithmetic, and nothing here has
been near a motor. The board's own rule applies to it (invariant 10) -
a number from nowhere and one from hardware must never look alike, which
is why the parameter sets below carry `measured=False` and say where they
came from.
"""
import math

from .sensorless import TWO_PI


def flux_from_kv(kv_rpm_per_volt, pole_pairs):
    """Peak per-phase flux linkage, Wb, from a hobby motor's KV.

    THE CONVENTION, WRITTEN DOWN, because getting it wrong poisons every
    number downstream and looks right the whole way:

      * KV is no-load mechanical rpm per volt applied LINE TO LINE, which
        is what a hobby motor's label means.
      * This model uses the AMPLITUDE-INVARIANT Park transform, so the q
        axis back-EMF is `omega_e * lambda` and the line-to-line peak is
        `sqrt(3)` times it.

    At n rpm, `omega_e = n / 60 * 2*pi * P` and `V_ll_peak = n / KV`, so

        sqrt(3) * (n / 60 * 2*pi * P) * lambda = n / KV
        lambda = 60 / (sqrt(3) * 2*pi * P * KV)

    A DERIVATION AND NOT A MEASUREMENT. A real KV is quoted at a
    temperature nobody states, from a no-load test that includes iron
    loss, and the pole count is often wrong on the box. What this returns
    is a starting point for a model and a prior for an identification -
    `coaxial.sysid` is what replaces it with a number off a real machine.
    """
    return 60.0 / (math.sqrt(3.0) * TWO_PI * pole_pairs * kv_rpm_per_volt)


#: The winding against still air, and its heat capacity. PLACEHOLDERS,
#: the order of magnitude an outrunner of this size has and measured
#: against nothing - a motor profile carries its own pair, and a bench
#: with a thermocouple writes real ones over them. They exist so an
#: estimate is made from a number that travels with the machine rather
#: than one written at whatever call site wanted a temperature.
WINDING_K_PER_W = 2.2
WINDING_J_PER_K = 180.0


class Parameters:

    """One machine's constants, and where they came from.

    `measured` is the whole point of the class: a parameter set recovered
    from a real motor and one estimated from a label must not be
    interchangeable without anybody noticing which is which.
    """

    __slots__ = ('name', 'r', 'ld', 'lq', 'lam', 'poles', 'j', 'b',
                 'sat', 'i_sat', 'measured', 'source',
                 'winding_k_per_w', 'winding_j_per_k')

    def __init__(self, name, r, ld, lq, lam, poles, j=2e-5, b=1e-5,
                 sat=0.0, i_sat=5.0, measured=False, source='',
                 winding_k_per_w=WINDING_K_PER_W,
                 winding_j_per_k=WINDING_J_PER_K):
        self.name = name
        self.r, self.ld, self.lq, self.lam = r, ld, lq, lam
        self.poles = poles              # pole PAIRS
        self.j, self.b = j, b
        #: How far Ld bends at `i_sat` of d current. The polarity
        #: pulse has nothing to find without it: injection locks the
        #: d AXIS, and only saturation says which end is the magnet.
        self.sat, self.i_sat = sat, i_sat
        self.measured = measured
        self.source = source
        #: How the winding sheds what it makes, and how much it holds.
        #: NOT MEASURED and not measurable from the terminals - `measured`
        #: says nothing about these two, and a page that estimates a
        #: winding temperature from them says estimate.
        self.winding_k_per_w = winding_k_per_w
        self.winding_j_per_k = winding_j_per_k

    @property
    def kv(self):
        """The KV this flux linkage implies, by `flux_from_kv` inverted.
        Infinite at zero flux rather than a crash: a failed identification
        hands back lam 0.0, and its repr still has to print."""
        got = math.sqrt(3.0) * TWO_PI * self.poles * self.lam
        return 60.0 / got if got else float('inf')

    @property
    def saliency(self):
        """Lq / Ld. Below about 1.1 an injection observer has little to
        find, which is what `coaxial.sensorless` judges."""
        return self.lq / self.ld if self.ld else float('inf')

    def __repr__(self):
        return ('<%s %s: R %.4f ohm, Ld %.1f uH, Lq %.1f uH, '
                'lambda %.5f Wb, %d pole pairs, KV %.0f>'
                % (self.name, 'measured' if self.measured else 'ESTIMATED',
                   self.r, self.ld * 1e6, self.lq * 1e6, self.lam,
                   self.poles, self.kv))


#: The nameplate, off Hobbywing's own specification page. These are the
#: manufacturer's numbers, not this bench's - nothing here has been near an
#: instrument. Kept because two of them bound what the drive may ask for:
#: the motor's 112.5 A burst sits just above the board's 100 A rating, so
#: the INVERTER is the limit, and a 12S pack at full charge is 50.4 V
#: against a 63 V rating and 78.15 V of divider scale.
RATINGS = {
    'slots_poles': '24N28P',        # 24 stator slots, 28 magnet poles
    'kv': 190.0,
    'cells': '10-12S LiPo',
    'i_noload': 3.0, 'v_noload': 44.4,
    'i_max': 112.5, 't_i_max': 42.0,
    'p_max': 4985.0,
    'mass_kg': 0.582, 'shaft_mm': 8.0,
    'stator': '51.7 x 30 mm',
    'source': 'hobbywing.com Platinum 5220/5230 specification page',
}

#: The motor this bench is aimed at. NOT MEASURED HERE - every electrical
#: constant is an estimate, and the notebook's first job is to replace
#: them. What comes from where:
#:
#:   poles   the SHEET: 24N28P is 28 poles, 14 pairs. It is not derivable
#:           from any torque measurement - see KT_NM_PER_AMP - and it sets
#:           the whole electrical frequency, so it had to come from the
#:           winding.
#:   b       the SHEET, arithmetic: 3.0 A at 44.4 V spins it at 190*44.4 =
#:           8436 rpm, so 133 W of iron and friction against 883 rad/s is
#:           0.151 N.m of drag, b = 1.71e-4. Copper at 3 A is 0.4 W, lost
#:           in the rounding. The guess this replaced was 4.3x too low.
#:   lambda  the label's 190 KV through `flux_from_kv` at 14 pole pairs.
#:   R       ESTIMATE, the size class. The sheet states no winding
#:           resistance. R is what an identification finds first, because
#:           it is the easiest thing to see.
#:   Ld, Lq  ESTIMATE, the size class again, with the ~1.3 saliency an
#:           outrunner of this construction tends to show. This is the
#:           number the sensorless observer lives or dies on, and it is the
#:           LEAST trustworthy one here.
#:   J       ESTIMATE. 582 g total, of which the bell is the part that
#:           turns; a thin ring at 30 mm would be 3e-4 and a hub pulls it
#:           down, so this is a middling guess and not arithmetic.
PLATINUM_5230SL = Parameters(
    name='Hobbywing Platinum 5230SL 190KV',
    r=0.030,
    ld=22e-6,
    lq=29e-6,
    lam=flux_from_kv(190.0, 14),
    poles=14,                       # 24N28P: 28 poles, 14 pairs
    j=1.2e-4,
    b=1.71e-4,
    # ESTIMATE. 30 % of Ld gone by 40 A of d current - the size class
    # again, scaled off the 112.5 A this machine is rated for.
    sat=0.3, i_sat=40.0,
    measured=False,
    source='poles and friction from the manufacturer sheet (see RATINGS); '
           'R, Ld, Lq and J estimated from the size class - replace with '
           'an identification against the real machine')


#: The machine behind the stand-in's drive device. DELIBERATELY NOT the
#: 5230SL: its constants were picked so every commissioning step recovers a
#: number it can be checked against, which is a different job from
#: resembling the motor on the bench. It lives here rather than as literals
#: on `SimulatedDrive` so that a third copy of a machine cannot drift away
#: from the two above.
BENCH_MOTOR = Parameters(
    name='stand-in bench motor',
    r=0.05, ld=20e-6, lq=30e-6, lam=0.005, poles=7, j=2e-5, b=1e-5,
    sat=0.3, i_sat=4.0,        # SimulatedDrive.SAT / I_SAT
    measured=False,
    source='invented for the stand-in, chosen to be recoverable')


class Propeller:

    """A load that grows with the square of speed, off a measured curve.

    `T = k * omega_m^2` is the propeller law. `k` here is a least squares
    over the manufacturer's own thrust stand, so the load a simulation
    pulls against is the one the motor was actually tested with rather
    than a number chosen to make a plot look right.
    """

    def __init__(self, k, name='', source=''):
        self.k, self.name, self.source = k, name, source

    def torque(self, omega_electrical, pole_pairs):
        wm = omega_electrical / pole_pairs
        return self.k * wm * wm

    def __repr__(self):
        return '<%s: %.3e N.m/(rad/s)^2>' % (self.name or 'propeller', self.k)


#: The thrust stand `APC20x10E.k` was fitted over: Hobbywing's own 190KV
#: table at 37 V and 25 C, 22 rows of (rpm, shaft torque N.m, input W).
#: Kept whole so the fit can be re-done or argued with rather than taken
#: on trust - and so a model has something to be checked against that it
#: was not built from.
APC20X10E_CURVE = (
    (2534, 0.32, 125.2), (2684, 0.37, 143.2), (2827, 0.41, 162.7),
    (2964, 0.45, 183.9), (3163, 0.51, 219.1), (3359, 0.58, 258.8),
    (3554, 0.65, 303.6), (3750, 0.73, 354.1), (3946, 0.81, 410.7),
    (4144, 0.90, 473.7), (4342, 1.00, 543.5), (4541, 1.10, 620.0),
    (4740, 1.20, 703.7), (4938, 1.32, 794.8), (5134, 1.43, 893.7),
    (5328, 1.56, 1001.1), (5518, 1.69, 1117.7), (5703, 1.82, 1244.1),
    (5883, 1.96, 1380.8), (6058, 2.11, 1527.6), (6226, 2.27, 1683.8),
    (6717, 2.78, 2214.0),
)

#: The APC20x10E the 190KV was tested on, fitted over its 22 points.
#: Worst point 13.2 % off a pure square, which is the propeller and not
#: the fit - efficiency moves along the curve.
APC20x10E = Propeller(
    k=5.143e-6, name='APC20x10E',
    source='least squares over the 190KV sheet, 37 V, 25 C, sea level')

#: WHAT THE SHEET ACTUALLY PINS. Torque per ampere of q current, from
#: 2.78 N.m at the top of the 190KV curve:
#:
#:     Kt = 1.5 * P * lambda = T / iq
#:
#: and it is the PRODUCT that the sheet determines. iq comes out 63.9 A at
#: full throttle whether the machine has 5 pole pairs or 10, because
#: lambda from a KV goes as 1/P and the P cancels - so this document
#: cannot tell you the pole count, and neither can any torque measurement.
#: The observer needs P on its own, because electrical speed is P times
#: mechanical. It came from the WINDING and not from any measurement of
#: torque: 24N28P is 28 poles, 14 pairs. Kt is unchanged by that - 1.5 *
#: 14 * 0.00207 is the same 0.0435 as 1.5 * 7 * 0.00415 - which is the
#: cancellation stated above, seen from the other side.
KT_NM_PER_AMP = 0.0435


class Motor:

    """A PMSM in its own dq frame, an inverter in front of it.

    `sat` bends Ld with the d current - the saturation saliency an SPM
    shows - and `v_dt` is the inverter's dead-time voltage error, odd in
    the phase current and saturating over `i_knee`. Both default off.
    """

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
        return (ia, -0.5 * ia + 0.8660254 * ib_, -0.5 * ia - 0.8660254 * ib_)

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
