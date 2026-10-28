"""A plane's flight on SE(3) and its sampled predictive control, on the airframe's own forces.

    mpc = se3mpc.Mppi(aerobat)                    # on a CUDA card where one answers, else the CPU
    u = mpc.solve(se3mpc.observed(state), ref, w_p, w_v, wind)   # ref (h, REF): mppi.h's MR_*
    controls = se3mpc.blended(core, law, row, state, r_des, speeds, dt, law_u)   # a row's plan

The forces are the plant's (`aerobat.Sky.step`, `wingborne.Wing`): the propeller's thrust along
the nose through its inflow, the wing's lift and the side's, the surfaces' moments in the
propwash, the airframe's drag; the floor bears on the tail and the wheels. The rollouts are one
C source, `mppi/src/mppi_plane.c` (`tools.cores.mppi`); `Batch` is its numpy twin.
"""
import ctypes
import importlib
import math
import os

from tools.cores import mppi

#: The floor under a contact point, a spring and a damper: at 4 000 N/m the tail's 0.18 kg swung
#: at 149 rad/s, 3.7 rad a 0.025 s step, and the rollouts blew up.
K_FLOOR, C_FLOOR = 400.0, 20.0
#: The model's constants in mppi.h's order.
CONSTS = ('m', 'g', 'kt', 'kp', 'kr', 'ky', 'kd', 'rho', 'area', 'cla', 'span', 'stab', 'fin', 'arm', 'wq',
          'stall', 'blend', 'cdl', 'j0', 'pitch', 'r', 'cla_a', 'cda', 'area_a', 'cd0', 'cdi', 'share',
          'ix', 'iy', 'iz', 'kf', 'cf', 'side', 'adisc', 'pb', 'pl', 'ps')
#: The tail's lowest point a row's `skim` asks, m over the floor: a few centimetres.
SKIM_M = 0.03
#: A step's reference, mppi.h's MR_*: where each part starts in a row of REF.
PLACE, SPEED, BELLY, NOSE, W_BELLY, W_NOSE, SKIM, W_SKIM, W_HEIGHT, REF = 0, 3, 6, 9, 12, 13, 14, 15, 16, 17


def observed(state):
    """(p, v, R, w) of a sky's `state`: world place and speed, body to world, body rates."""
    import numpy as np
    return (np.array(state['at'], float), np.array(state['vel'], float),
            np.array(state['turn'], float).reshape(3, 3), np.array(state['spin'], float))


def forces(craft, x, u, w_p, w_v, wind=(0.0, 0.0, 0.0)):
    """(force, torque): the world force on the frame, N, and the body torque, N m, of `craft`
    (`aerobat`) at `x` under `u`, its propeller at `w_p` and its vector stage at `w_v` rad/s,
    off the floor - through the craft's own functions, the twin's reference."""
    import numpy as np
    p, v, r, w = x
    _pitch, elev, ail, rud = (float(k) for k in u)
    air = v - np.asarray(wind, float)
    along = float(air @ r[:, 1])
    share = craft._aero_inflow(along, math.hypot(air[0], air[2]), w_p)
    lift = craft.K_THRUST * w_p * w_p * share
    drag = 0.5 * craft.RHO * craft._aero_cda(elev, ail) * float(np.linalg.norm(air))
    force = lift * r[:, 1] + np.array(craft.WING.lift(tuple(air), r), float) - drag * air + np.array(
        [0.0, -craft.MASS_KG * craft.GRAVITY, 0.0])
    q_s = craft.slipstream(along, lift)
    ab = np.array((craft.K_PITCH_Q * q_s * elev,
                   craft.K_YAW * w_v * w_v * rud - craft._prop_drag(share, elev, ail, w_p),
                   craft.K_ROLL_Q * q_s * ail))
    return force, ab + np.array(craft.WING.moments(tuple(air), r, (0.0, 0.0, 0.0) + tuple(w)), float)


def constants(craft):
    """`craft`'s model constants by name."""
    from machine.aircraft import quad, wingborne
    w = craft.WING
    ix, iy, iz = craft.INERTIA
    return dict(m=craft.MASS_KG, g=craft.GRAVITY, kt=craft.K_THRUST, kp=craft.K_PITCH_Q, kr=craft.K_ROLL_Q,
                ky=craft.K_YAW, kd=craft.K_DRAG, rho=w.rho, area=w.area, cla=w.cl_alpha, span=w.span,
                stab=w.stab, fin=w.fin, arm=w.arm, wq=w.q, stall=wingborne.STALL_RAD,
                blend=wingborne.STALL_BLEND, cdl=wingborne.CD_LIFT, j0=craft.INFLOW_J0, pitch=craft.BLADE_PITCH,
                r=craft.PROP_R, cla_a=craft.CL_ALPHA, cda=craft.BODY_CDA, area_a=craft.WING_AREA,
                cd0=craft.CD0, cdi=craft.CD_INDUCED, share=craft.PROPWASH_SHARE, ix=ix, iy=iy, iz=iz,
                kf=K_FLOOR, cf=C_FLOOR, side=w.side, adisc=craft.A_DISC, pb=craft.PROP_BASE,
                pl=craft.PROP_LOAD, ps=craft.PROP_SURF)


def points(craft):
    """(contacts (N, 5), tail, rest (M, 3)): body points that may bear on the floor with their
    radius and friction - the tail's `tail` corners first, then the wheels, as `aerobat.mjcf`
    lays them - and those that must not: the wingtips and the propeller's rim."""
    import numpy as np
    r, z = craft.PROP_R, craft.PROP_Z
    tail = [tuple(p) + (0.0, 0.4) for p in craft.FORM['tail']]
    wheels = [(x_sc * sx, by, bz, craft.FOOT_M, 0.3) for sx in (-craft.SKID_X, craft.SKID_X)
              for by, bz, x_sc in ((craft.STRUT_Z[0], craft.GEAR_TAIL_Z, 0.45), (craft.STRUT_Z[1], craft.GEAR_MAIN_Z, 1.0))]
    rest = [tuple(p) for p in craft.FORM['tips']] + [(r, z, 0.0), (-r, z, 0.0), (0.0, z, r), (0.0, z, -r)]
    return np.array(tail + wheels, float), len(tail), np.array(rest, float)


def backend():
    """cupy on a CUDA card, numpy without one - CI, a bench PC - or under COAXIAL_GPU=0."""
    if os.environ.get('COAXIAL_GPU', '1') != '0':
        try:
            cupy = importlib.import_module('cupy')
            if cupy.cuda.runtime.getDeviceCount() > 0:
                return cupy
        except Exception:    # no cupy, no card, no driver: the CPU
            pass
    return importlib.import_module('numpy')


class Batch:

    """`craft`'s rollouts over a batch of states on `xp`: mppi_plane.c's numpy twin."""

    def __init__(self, craft, xp):
        self.xp, self.k = xp, constants(craft)
        self.inertia = xp.asarray(craft.INERTIA, dtype=xp.float64)

    def pos(self, x):
        """mppi_plane.c's POS: 1 over 0, 0 at or under it, a ramp below 1e-12."""
        return self.xp.minimum(1.0, self.xp.maximum(0.0, x * 1e12))

    def forces(self, v, r, w, u, w_p, w_v, wind):
        """(force (K, 3) world, torque (K, 3) body) of states (v, r, w) under controls `u` (K, 4)."""
        xp, k = self.xp, self.k
        elev, ail, rud = u[:, 1], u[:, 2], u[:, 3]
        air = v - wind
        span, nose, belly = r[:, :, 0], r[:, :, 1], r[:, :, 2]
        along, speed = (air * nose).sum(1), xp.sqrt((air * air).sum(1))
        vh = xp.hypot(air[:, 0], air[:, 2])
        j = xp.maximum(0.0, 1.0 - along / (k['j0'] * max(1.0, k['pitch'] * abs(w_p) / math.tau)))
        share = j * (1.0 + 0.04 * k['cla_a'] * xp.minimum(0.35, vh / max(w_p * k['r'], 8.0)))
        lift = k['kt'] * w_p * w_p * share
        uy, uz = xp.maximum(0.0, along), (air * belly).sum(1)
        v2 = uy * uy + uz * uz
        on = self.pos(uy) * self.pos(v2 - 1e-6)
        alpha = xp.arctan2(uz, on * uy + (1.0 - on))
        flat = xp.clip((xp.abs(alpha) - k['stall']) / k['blend'], 0.0, 1.0)
        sa, ca, c_lin = xp.sin(alpha), xp.cos(alpha), k['cla'] * alpha
        c_l = (1.0 - flat) * c_lin + flat * 2.0 * sa * ca
        c_d = (1.0 - flat) * k['cdl'] * c_lin * c_lin + flat * 2.0 * sa * sa
        q = on * 0.5 * k['rho'] * k['area'] * v2
        sp = xp.sqrt(xp.maximum(v2, 1e-12))[:, None]
        dirn, up = (uy[:, None] * nose + uz[:, None] * belly) / sp, (uz[:, None] * nose - uy[:, None] * belly) / sp
        ux = (air * span).sum(1)
        slip = -k['rho'] * k['side'] * speed * ux
        cl_eff = 0.12 * k['cla_a'] * k['share'] * xp.hypot(elev, ail)
        cda = k['cda'] + k['area_a'] * (k['cd0'] + k['cdi'] * cl_eff * cl_eff)
        drag = (0.5 * k['rho'] * cda * speed)[:, None] * air
        force = (lift[:, None] * nose + (q * c_l)[:, None] * up - (q * c_d)[:, None] * dirn
                 + slip[:, None] * span - drag)
        force[:, 1] -= k['m'] * k['g']
        qw, qd = k['wq'] * along, k['wq'] * xp.abs(along)
        prop = (k['pb'] + k['pl'] * share + k['ps'] * (elev * elev + ail * ail)) * k['kd'] * w_p * w_p
        q_s = 0.5 * k['rho'] * uy * uy + xp.maximum(0.0, lift) / k['adisc']
        torque = xp.stack((k['kp'] * q_s * elev + k['arm'] * k['stab'] * (qw * uz - qd * k['arm'] * w[:, 0]),
                           k['ky'] * w_v * w_v * rud - prop - qd * k['area'] * (0.25 * k['span']) ** 2 * w[:, 1],
                           k['kr'] * q_s * ail - k['arm'] * k['fin'] * (qw * ux + qd * k['arm'] * w[:, 2])), 1)
        return force, torque

    def floor(self, p, v, r, w, contacts):
        """(force (K, 3), torque (K, 3) body) of the floor on `contacts` (`points`) under it: a spring
        and a damper up, friction against the slip."""
        xp, k = self.xp, self.k
        arm = xp.einsum('kij,nj->kni', r, contacts[:, :3])
        pen = xp.maximum(0.0, contacts[:, 3] - p[:, 1, None] - arm[:, :, 1])
        vp = v[:, None, :] + xp.cross(xp.einsum('kij,kj->ki', r, w)[:, None, :], arm)
        n = self.pos(pen) * xp.maximum(0.0, k['kf'] * pen - k['cf'] * vp[:, :, 1])
        slip = vp.copy()
        slip[:, :, 1] = 0.0
        f = -(contacts[:, 4] * n)[:, :, None] * slip / xp.sqrt((slip * slip).sum(2) + 0.01)[:, :, None]
        f[:, :, 1] += n
        return f.sum(1), xp.einsum('kji,kj->ki', r, xp.cross(arm, f).sum(1))

    def stepped(self, p, v, r, w, u, w_p, w_v, dt, wind, contacts):
        """The batch `dt` s on, on the floor at `contacts`."""
        xp = self.xp
        force, torque = self.forces(v, r, w, u, w_p, w_v, wind)
        ff, ft = self.floor(p, v, r, w, contacts)
        v1 = v + dt * (force + ff) / self.k['m']
        w1 = w + dt * (torque + ft - xp.cross(w, self.inertia * w)) / self.inertia
        th = xp.sqrt((w1 * w1).sum(1)) * dt
        e = w1 * dt / xp.maximum(th, 1e-12)[:, None]
        kx = xp.zeros(r.shape)
        kx[:, 0, 1], kx[:, 0, 2], kx[:, 1, 2] = -e[:, 2], e[:, 1], -e[:, 0]
        kx[:, 1, 0], kx[:, 2, 0], kx[:, 2, 1] = e[:, 2], -e[:, 1], e[:, 0]
        inc = xp.sin(th)[:, None, None] * kx + (1.0 - xp.cos(th))[:, None, None] * (kx @ kx) + xp.eye(3)[None]
        return p + dt * v1, v1, r @ inc, w1


class Mppi:

    """Sampled predictive control of `craft`: `k` rollouts of `h` steps of `dt`, the controls
    (throttle 0-1, the three surfaces -1-1) noised by `sigma` and correlated `ar` along the
    horizon, each weighed by exp(-cost / lam), lam the cost a `share` of them come within. Asked a
    place, a speed, the belly's and the nose's directions and the tail's lowest point a step; held
    off the floor but at the tail and the wheels, the surfaces within half where it can, the least
    thrust that does it."""

    #: The cost's weights in mppi.h's order.
    W = dict(x=4.0, y=400.0, z=4.0, speed=1.0, belly=8.0, nose=8.0, spin=0.05, reserve=60.0, touch=4e4,
             crash=4e5, ease=2.0, thrust=1.0, skim=4000.0)

    def __init__(self, craft, k=None, h=32, dt=0.025, sigma=(0.05, 0.15, 0.15, 0.15), share=0.1, ar=0.85,
                 xp=None, seed=0):
        self.xp = xp = backend() if xp is None else xp
        card = xp.__name__ != 'numpy'
        self.k, self.h, self.dt, self.share, self.ar = k or (4096 if card else 512), h, dt, share, ar
        self.sigma = xp.asarray(sigma, dtype=xp.float64)
        self.lo, self.hi = xp.asarray([0.0, -1.0, -1.0, -1.0]), xp.asarray([1.0, 1.0, 1.0, 1.0])
        self.nominal = xp.zeros((h, 4))
        self.rng = xp.random.default_rng(seed)
        contacts, self.ns, rest = points(craft)
        self.contacts, self.rest = xp.asarray(contacts), xp.asarray(rest)
        named = constants(craft)
        self.consts = xp.asarray([named[n] for n in CONSTS])
        self.weights = xp.asarray(list(self.W.values()))
        self.model = Batch(craft, xp)
        self.run = mppi.kernel(xp) if card else mppi.library().mppi_plane_costs
        self.cost = 0.0

    def costs(self, x, u, ref, w_p, w_v, wind=(0.0, 0.0, 0.0)):
        """Each rollout's cost (k,) from `x` under the controls `u` (k, h, 4), asked `ref` (h, REF):
        mppi_plane.c on the card or the CPU."""
        import numpy as np
        xp, k, h = self.xp, len(u), self.h
        args = [np.concatenate([np.ravel(a) for a in x]), u, np.asarray(ref, float), self.consts, self.contacts,
                len(self.contacts), self.ns, self.rest, len(self.rest), self.weights, self.nominal[0], k, h,
                self.dt, w_p, w_v, np.asarray(wind, float)]
        if xp is not np:
            cost = xp.zeros(k)
            self.run(((k + 127) // 128,), (128,), tuple(
                np.int32(a) if isinstance(a, int) else np.float64(a) if isinstance(a, float)
                else xp.ascontiguousarray(xp.asarray(a, dtype=xp.float64)) for a in args) + (cost,))
            return cost
        cost, d = np.zeros(k), ctypes.POINTER(ctypes.c_double)
        laid = [a if isinstance(a, (int, float)) else np.ascontiguousarray(a, dtype=np.float64) for a in args]
        self.run(*(a if isinstance(a, (int, float)) else a.ctypes.data_as(d) for a in laid), cost.ctypes.data_as(d))
        return cost

    def twin(self, x, u, ref, w_p, w_v, wind=(0.0, 0.0, 0.0)):
        """`costs` on the numpy twin: its parity reference."""
        import numpy as np
        xp, k, W, c, ns = self.xp, len(u), self.W, self.contacts, self.ns
        p, v, r, w = (xp.broadcast_to(xp.asarray(a), (k,) + np.shape(a)).copy() for a in x)
        rf, wind = xp.asarray(ref, dtype=xp.float64), xp.asarray(wind, dtype=xp.float64)
        cost, last = xp.zeros(k), xp.broadcast_to(self.nominal[0], (k, 4))
        for t in range(self.h):
            p, v, r, w = self.model.stepped(p, v, r, w, u[:, t], w_p, w_v, self.dt, wind, c)
            s = rf[t]
            cost += (W['x'] * (p[:, 0] - s[PLACE]) ** 2 + W['z'] * (p[:, 2] - s[PLACE + 2]) ** 2
                     + W['y'] * s[W_HEIGHT] * (p[:, 1] - s[PLACE + 1]) ** 2)
            cost += W['speed'] * ((v - s[SPEED:SPEED + 3]) ** 2).sum(1)
            cost += W['belly'] * s[W_BELLY] * (1.0 - (r[:, :, 2] * s[BELLY:BELLY + 3]).sum(1))
            cost += W['nose'] * s[W_NOSE] * (1.0 - (r[:, :, 1] * s[NOSE:NOSE + 3]).sum(1))
            cost += W['spin'] * (w * w).sum(1)
            cost += W['reserve'] * (xp.maximum(0.0, xp.abs(u[:, t, 1:]) - 0.5) ** 2).sum(1)
            ys = p[:, 1, None] + xp.einsum('kj,nj->kn', r[:, 1, :], c[:, :3]) - c[:, 3]
            cost += W['touch'] * (xp.maximum(0.0, -0.02 - ys) ** 2).sum(1)
            cost += W['skim'] * s[W_SKIM] * (xp.minimum(1e3, ys[:, :ns].min(1)) - s[SKIM]) ** 2
            ys = p[:, 1, None] + xp.einsum('kj,nj->kn', r[:, 1, :], self.rest)
            cost += W['crash'] * (xp.maximum(0.0, 0.03 - ys) ** 2).sum(1)
            cost += W['ease'] * ((u[:, t] - last) ** 2).sum(1) + W['thrust'] * u[:, t, 0] ** 2
            last = u[:, t]
        return xp.fmin(cost, 1e18)

    def solve(self, x, ref, w_p, w_v, wind=(0.0, 0.0, 0.0), prior=None):
        """The controls for now at `x` (`observed`), asked `ref` (h, REF) at its steps; `prior`,
        a law's controls, weighed beside the samples, held over the horizon."""
        xp, k, h = self.xp, self.k, self.h
        white = self.rng.standard_normal((k, h, 4))
        for n in range(1, h):     # a manoeuvre, not a buzz: noise correlated along the horizon
            white[:, n] = self.ar * white[:, n - 1] + math.sqrt(1.0 - self.ar ** 2) * white[:, n]
        u = xp.clip(self.nominal[None] + white * self.sigma, self.lo, self.hi)
        if prior is not None:
            u[0] = xp.clip(xp.asarray(prior, dtype=xp.float64), self.lo, self.hi)
        noise = u - self.nominal[None]
        cost = self.costs(x, u, ref, w_p, w_v, wind)
        gap = cost - cost.min()
        lam = max(1e-6, float(xp.sort(gap)[max(1, int(self.share * k))]))    # a share of them weighs
        weight = xp.exp(-gap / lam)
        self.nominal = self.nominal + (weight[:, None, None] * noise).sum(0) / weight.sum()
        out = self.nominal[0].copy()
        self.nominal = xp.concatenate((self.nominal[1:], self.nominal[-1:]))
        self.cost = float(cost.min())
        return [float(a) for a in (out.get() if hasattr(out, 'get') else out)]


def reference(mpc, law, row, r_des, rest):
    """(h, REF): the law's spot carried on at its speed, the row's height over `rest`, the belly and
    the nose of the law's attitude `r_des`; the tail's lowest point at SKIM_M by the row's `skim`
    (0-1), the height and the nose let go as much: a skim's nose its own, its brake's the law's."""
    import numpy as np
    tau = mpc.dt * np.arange(1, mpc.h + 1)
    (x, z), (vx, vz) = law['spot'], law['speed']
    ref = np.zeros((mpc.h, REF))
    ref[:, PLACE], ref[:, PLACE + 1], ref[:, PLACE + 2] = x + vx * tau, rest + max(0.0, row['height']), z + vz * tau
    ref[:, SPEED], ref[:, SPEED + 2] = vx, vz
    ref[:, BELLY:BELLY + 3] = [r_des[a][2] for a in range(3)]
    ref[:, NOSE:NOSE + 3] = [r_des[a][1] for a in range(3)]
    skims = min(1.0, max(0.0, row.get('skim', 0.0)))
    ref[:, W_BELLY], ref[:, W_NOSE], ref[:, SKIM], ref[:, W_SKIM] = 12.0, 2.2 - 2.0 * skims, SKIM_M, skims
    ref[:, W_HEIGHT] = 1.0 - 0.95 * skims
    return ref


def blended(core, law, row, state, r_des, speeds, dt, law_u):
    """The controls `core` flies `row` with: the law's `law_u` and the planner's mixed by the row's
    `plan`, 0 the law's alone."""
    plan = row.get('plan', 0.0)
    if plan <= 0.0:     # the law's alone: no solve
        core.mpc_on = False
        return law_u
    u = flown(core, law, row, state, r_des, speeds, dt, law_u)
    core.mpc_on = True
    return tuple((1.0 - plan) * a + plan * b for a, b in zip(law_u, u))


def flown(core, law, row, state, r_des, speeds, dt, law_u):
    """The controls `core` flies `row` with now on its planner: solved each of the planner's
    steps, held between; taken over from the law's controls `law_u` as its first nominal."""
    mpc = getattr(core, 'mpc', None)
    if mpc is None:
        from machine.aircraft import aerobat
        mpc = core.mpc = Mppi(aerobat)
    if not getattr(core, 'mpc_on', False):
        mpc.nominal[:] = mpc.xp.asarray(law_u)
        core.mpc_wait, core.mpc_u = 0.0, list(law_u)
    core.mpc_wait -= dt
    if core.mpc_wait <= 0.0:
        from machine.aircraft import aerobat
        core.mpc_wait += mpc.dt
        core.mpc_u = mpc.solve(observed(state), reference(mpc, law, row, r_des, aerobat.SKID_M + aerobat.FOOT_M),
                               speeds[0], speeds[1], core.wind, law_u)
    return core.mpc_u
