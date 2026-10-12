"""The frame's core (aero/) on a rigid body in a vacuum, or in a wind: a suite's rig."""
import math

#: A vacuum's pass, s: fine enough that the pass itself is no part of what is measured.
FINE = 0.0002


def hover(k=5000.0, d=80.0, extra='', blend=0, sequence=1, pitch='0deg', roll='0deg',
          thrust='9.81'):
    """A payload: level - or `pitch`, `roll` - on the collective `thrust`, `k` and `d` on the
    tilt's modes, `extra` blocks after."""
    from machine import aerolisp
    from tools.cores import aero
    return aero.payload(aerolisp.compiled(
        '(stream_inject (sequence_id %d) (blend_duration %dms) (latent_targets '
        '(mode_collective_thrust %s) (mode_attitude_pitch %s) (mode_attitude_roll %s) '
        '(mode_yaw_rate 0rad_s)) (impedance (stiffness mode_attitude_pitch %g) '
        '(stiffness mode_attitude_roll %g) (damping global %g)) %s)'
        % (sequence, blend, thrust, pitch, roll, k, k, d, extra))[0])


def asks():
    """(payloads, readings) a core is stepped through, each over each, to show a pass costs
    the same whatever it is asked: the power loop's four streams and its first again; the
    frame level at rest, tilted and turning, on its back, its rotors' reach shut, and nothing
    given over a pass of 20 ms."""
    from machine import aerolisp, intents, pull, quad
    flown = aerolisp.compiled(intents.POWER_LOOP, {
        'MAX_SYS_THRUST': pull.thrust(pull.top(1.0, pull.I_MAX)) / quad.MASS_KG})
    hover_n, c, s = quad.MASS_KG * quad.GRAVITY / 4.0, math.cos(0.7), math.sin(0.7)
    level = {'r': (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), 'spin': (0.0, 0.0, 0.0),
             'vel': (0.0, 0.0, 0.0), 'height': 1.5, 'given': [hover_n] * 4, 'lo': [0.8] * 4,
             'hi': [50.0] * 4, 'dt': 0.002}
    return flown + flown[:1], (
        level,
        dict(level, r=(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c), spin=(2.0, -1.5, 3.0),
             vel=(5.0, -2.0, 8.0), height=6.0, given=(3.0, 9.0, 12.0, 1.0)),
        dict(level, r=(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0), spin=(-9.0, 0.3, 0.1),
             vel=(0.0, -12.0, 3.0), height=20.0),
        dict(level, lo=[hover_n] * 4, hi=[hover_n] * 4),
        dict(level, given=[0.0] * 4, lo=[0.0] * 4, hi=[0.0] * 4, dt=0.02))


def params():
    """`machine.quad`'s frame as a board is laid it (`coaxial.devices.aero.Aero.lay`): its
    params by name."""
    from tools.cores import aero
    frame = aero.frame()
    return {field[0]: getattr(frame, field[0]) if isinstance(getattr(frame, field[0]), float)
            else list(getattr(frame, field[0])) for field in aero.Params._fields_}


def _ctypes(values):
    """{name: its values} as a struct's keywords: one a number, more an array of floats."""
    import ctypes
    return {name: v[0] if len(v) == 1 else (ctypes.c_float * len(v))(*v)
            for name, v in values.items()}


def beside(laid):
    """A host core on the frame as a board has it laid (`Aero.lay`'s answer)."""
    from tools.cores import aero
    return aero.Core(aero.Params(**_ctypes(laid)))


def read(had):
    """A reading as a board had it (`Aero.step`'s `read`), the host core's."""
    from tools.cores import aero
    return aero.In(**_ctypes({name: v if isinstance(v, list) else [v]
                              for name, v in had.items()}))


class Vacuum:

    """The frame as a rigid body with nothing on it but its rotors' thrusts and gravity -
    and a push, m/s^2 in the world, and a twist, N m about its axes, where a test lays one;
    in a `wind`, m/s in the world, the air's drag and its rotors' loss to it with them, as
    the frame's own sizes have them - in none, its core is told of no air. The thrusts asked
    are given a pass on; `slew`: no faster than a rotor's speed changes, rad/s^2."""

    def __init__(self, dt=FINE, slew=None, height=5.0, wind=None):
        import numpy as np
        from machine import quad
        from tools.cores import aero
        self.np, self.quad, self.aero = np, quad, aero
        params = aero.frame()
        self.air = (params.drag, params.inflow) if wind is not None else (0.0, 0.0)
        params.drag, params.inflow = self.air
        self.wind = np.array(wind if wind is not None else (0.0, 0.0, 0.0))
        self.core, self.dt, self.slew = aero.Core(params), dt, slew
        self.turn, self.spin, self.vel = np.eye(3), np.zeros(3), np.zeros(3)
        self.height, self.clock = height, 0.0
        self.given = [quad.MASS_KG * quad.GRAVITY / 4.0] * 4
        self.push, self.twist = np.zeros(3), np.zeros(3)

    def tilted(self, about, rad):
        """The frame turned `rad` about its own axis `about` (0 its wing, 2 its nose)."""
        self.turn = self.turn @ self._turned(about, rad)

    def _turned(self, about, rad):
        np = self.np
        c, s = math.cos(rad), math.sin(rad)
        a, b = (about + 1) % 3, (about + 2) % 3
        out = np.eye(3)
        out[a, a], out[a, b], out[b, a], out[b, b] = c, -s, s, c
        return out

    def state(self):
        return {'turn': self.turn, 'spin': self.spin, 'vel': self.vel, 'h': self.height}

    def step(self, payload):
        """One pass on `payload`: the core's answer."""
        np, quad, dt = self.np, self.quad, self.dt
        if self.slew is None:
            lo, hi = [0.0] * 4, [200.0] * 4
        else:
            w = [math.sqrt(f / quad.K_THRUST) for f in self.given]
            lo = [quad.K_THRUST * max(0.0, x - self.slew * dt) ** 2 for x in w]
            hi = [quad.K_THRUST * (x + self.slew * dt) ** 2 for x in w]
        out = self.core.step(payload, self.aero.reading(self.state(), self.given, lo, hi, dt))
        self.given = [float(f) for f in out.thrust]
        still = self.vel - self.wind
        through = float(still @ self.turn[:, 1])
        thrusts = [f - self.air[1] * math.sqrt(f) * through for f in self.given]
        lift = sum(thrusts)
        torque = np.array([
            -sum(f * z for f, (_x, z) in zip(thrusts, quad.ROTOR_AT)),
            sum(f * s for f, s in zip(self.given, quad.SPIN)) * quad.K_DRAG / quad.K_THRUST,
            sum(f * x for f, (x, _z) in zip(thrusts, quad.ROTOR_AT))]) + self.twist
        inertia = np.array(quad.INERTIA)
        self.spin = self.spin + dt * (torque - np.cross(self.spin, inertia * self.spin)) / inertia
        for about in range(3):
            self.turn = self.turn @ self._turned(about, self.spin[about] * dt)
        acc = (lift / quad.MASS_KG * self.turn[:, 1] - np.array([0.0, quad.GRAVITY, 0.0])
               + self.push - self.air[0] * float(np.linalg.norm(still)) * still)
        self.vel = self.vel + dt * acc
        self.height += dt * self.vel[1]
        self.clock += dt
        return out

    def nose(self):
        """The nose's rise, rad."""
        return math.asin(max(-1.0, min(1.0, self.turn[1][2])))
