"""The robot's hoses across its joints: cables in a MuJoCo world of their own, hung from fittings.

    world = hoses.World(hoses.spans(), poses)        # laid at `poses` {segment: (turn 3x3, at 3)}, let fall
    world.follow(poses, seconds)                     # its fittings carried on to where the segments are now
    lines = world.lines()                            # [(k, 3)]: each hose's centre line, the world's frame

A hose across a joint (`spans`, the build's as `machine/world/hoses/<build>.toml` lays them, the right side the left's
mirrored): MuJoCo's cable (`mujoco.elasticity.cable`) of links `link` m long at most - bending and twisting as stiff as
the hose (`materials.toml` [hoses]), straight unloaded, each link weighing what the hose does over it - a link more at
each end inside its fitting, the first held by the body riding that fitting's segment, the last welded into the
other's - as heavy as what holds it (HELD_KG), a weld being as stiff as what it holds is heavy -, so the hose bends
from each fitting's mouth on. Each segment is a fixed body the world moves itself (`follow`), no mocap body: the
cable's plugin reads its bodies' places as the world's data is made, before MuJoCo has set a mocap body's - whatever
the memory held, a hose now and then stiff as nothing is. What a span tells it to keep off ('solids': a sketch's cans
and guards) are cylinders on the segments they ride - a build's file tells none: the figure turns its segments more
loosely than a sketch's joint (a rest turn, a longer travel), where they would stand in a fitting's way. The hose's
joints damp its first bending mode, its ends held, at the hose's ratio (a Kelvin-Voigt beam's), and at that mode's
critical while it is let fall (`settle`). The world's step is 2 over the angular frequency of the shortest wave a
hose's links carry, bending or twisting (two links long, the springs explicit) - a longer one falls over where a hose
is drawn taut -, so where a step is asked (`step`) its links are the fewest that hold it. Laid along a curve of its
length leaving each fitting along it (`_curve`). A world that falls over all the same raises, MuJoCo's reset off.
What a hose pulls on the robot is left out: its ~10 g against the robot's ~29 kg.
"""
import math
import os

from machine import libm_bound

#: Where each build's hoses are laid (`tools/render/hosed.py --write`), a file a build.
LAID = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hoses')
#: Settled: no joint turning faster than this, rad/s; the longest it is let fall, s; the longest a frame is stepped
#: through, s - past it (a pause, a seek) its fittings are carried there in that time.
STILL_RAD_S, FALL_S, CARRIED_S = 1e-2, 5.0, 0.1
#: A beam's first bending mode, its ends held: (beta L)^2, its angular frequency that over L^2 times sqrt(EI / kg/m);
#: what a hose's link in its far fitting weighs, kg - a limb's order, the fitting's and what carries it - and its
#: radius of gyration, m.
FIRST, HELD_KG, HELD_M = 22.37, 1.0, 0.01


def table(robot=None):
    """{name: {'kind', 'length', 'from', 'to'}}: the build's hoses as laid, the left side's; {} for a build without."""
    import tomllib
    from machine.body import mechatronics
    path = os.path.join(LAID, (robot or mechatronics.ROBOT) + '.toml')
    if not os.path.exists(path):
        return {}
    with open(path, 'rb') as f:
        return tomllib.load(f)


def spans(robot=None):
    """[{'name', 'kind', 'length', 'from', 'to', 'solids'}]: the build's hoses, each side's - 'from'/'to' (segment, at,
    way) a fitting's mouth and the way the hose leaves it, m in the segments' frames, 'solids' none; [] for a build
    without."""
    out = []
    for name, span in sorted(table(robot).items()):
        for side, sign in (('left', 1.0), ('right', -1.0)):
            def mirrored(seg, *vectors):
                return (seg.replace('left_', side + '_', 1),) + tuple((sign * v[0], v[1], v[2]) for v in vectors)
            out.append({'name': side + '_' + name, 'kind': span['kind'], 'length': float(span['length']),
                        'from': mirrored(*span['from']), 'to': mirrored(*span['to']), 'solids': []})
    return out


def spline(points, n=24):
    """A Catmull-Rom curve through `points` (k, 3), `n` a span."""
    import numpy as np
    points = np.asarray(points, float)
    p = np.vstack([points[0], points, points[-1]])
    t = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]
    t2 = t * t                                           # a whole power as its products: no platform's pow
    out = [0.5 * ((2 * p[i]) + (-p[i - 1] + p[i + 1]) * t + (2 * p[i - 1] - 5 * p[i] + 4 * p[i + 1] - p[i + 2]) * t2
                  + (-p[i - 1] + 3 * p[i] - 3 * p[i + 1] + p[i + 2]) * t2 * t) for i in range(1, len(p) - 2)]
    return np.vstack(out + [points[-1:]])


def _curve(a, u, b, v, length, n):
    """(n + 1, 3): `n` links along a cubic from `a` leaving along `u` to `b` left along `v`, their chords `length`
    in all - its handles as long as make them so."""
    import numpy as np
    from machine.errors import MachineError
    a, u, b, v = (np.asarray(q, float) for q in (a, u, b, v))
    s = np.linspace(0.0, 1.0, 400)[:, None]
    c = 1 - s

    def cut(k):
        pts = c * c * c * a + 3 * c * c * s * (a + k * u) + 3 * c * s * s * (b + k * v) + s * s * s * b
        along = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        at = np.linspace(0.0, along[-1], n + 1)
        return np.c_[[np.interp(at, along, pts[:, j]) for j in range(3)]].T

    def long(k):
        return float(np.linalg.norm(np.diff(cut(k), axis=0), axis=1).sum())
    if long(0.0) >= length:
        raise MachineError('a hose %.4f m long cannot reach %.4f m' % (length, long(0.0)))
    lo, hi = 0.0, length
    while long(hi) < length:
        hi *= 2.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if long(mid) < length else (lo, mid)
    return cut(0.5 * (lo + hi))


def _quat(turn):
    mujoco = libm_bound.mujoco()
    import numpy as np
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(turn, float).ravel())
    return q


class World:
    """The hoses of `spans` in a MuJoCo world of their own, laid at `poses` and let fall: links `link` m at most, each
    hose's least bending radius unless told - or, with `step` s, each hose's fewest links that hold that step."""

    def __init__(self, spans, poses, link=None, step=None):
        mujoco = libm_bound.mujoco()
        import numpy as np
        from machine.body import mechatronics
        made, g = mechatronics.materials()['hoses'], float(mechatronics.environment()['g_m_s2'])
        self.segments = sorted({end[0] for s in spans for end in (s['from'], s['to'])}
                               | {solid[0] for s in spans for solid in s['solids']})
        inside, welds, apart, steps, self._ends, lie = {seg: [] for seg in self.segments}, [], [], [], [], []
        for k, span in enumerate(spans):
            r, kgm, ei, gj, bend, ratio = (float(q) for q in made[span['kind']])
            (seg_a, at_a, way_a), (seg_b, at_b, way_b) = span['from'], span['to']
            turn_a, put_a = (np.asarray(q, float) for q in poses[seg_a])
            turn_b, put_b = (np.asarray(q, float) for q in poses[seg_b])
            b = turn_a.T @ (turn_b @ np.asarray(at_b, float) + put_b - put_a)    # the far fitting in the near's frame
            into = turn_a.T @ turn_b @ np.asarray(way_b, float)
            wave = math.pi * math.pi * math.sqrt(ei / kgm) / 2.0            # a link's step: its length squared / this,
            twist = math.pi * math.sqrt(2.0 * gj / kgm) / (2.0 * r)         # and, twisting, its length / this
            links = (max(2, int(span['length'] / max(math.sqrt(step * wave), step * twist))) if step
                     else max(2, int(math.ceil(span['length'] / (link or bend)))))
            free = _curve(at_a, way_a, b, into, span['length'], links)      # mouth to mouth
            ell = float(np.linalg.norm(np.diff(free, axis=0), axis=1).mean())
            vertex = np.vstack([free[0] - ell * np.asarray(way_a, float), free, free[-1] - ell * into])
            chords = np.linalg.norm(np.diff(vertex, axis=0), axis=1)
            first = FIRST / (span['length'] * span['length']) * math.sqrt(ei / kgm)   # its first mode, rad/s
            least = float(chords.min())
            steps.append(min(least * least / wave, least / twist))           # 2 / its shortest
            # wave's frequency: a link's own turn between its neighbours', its weight about its axis r^2 / 2 a kg
            self._ends.append(float(chords[-1]))
            area, inertia = math.pi * r * r, 0.25 * math.pi * r * r * r * r
            critical = 2.0 * ei / (ell * first)                              # a joint's, critical on that mode
            lie.append(critical)
            density = kgm * ell / (area * ell + 4.0 / 3.0 * math.pi * r * r * r)   # a capsule's ends counted
            inside[seg_a].append(
                '<composite prefix="h%d" type="cable" vertex="%s" initial="none"><plugin plugin="mujoco.'
                'elasticity.cable"><config key="bend" value="%r"/><config key="twist" value="%r"/><config '
                'key="flat" value="true"/><config key="vmax" value="0"/></plugin><joint kind="main" '
                'damping="%r"/><geom type="capsule" size="%r" density="%r" contype="1" conaffinity="2" '
                '{held}/></composite>' % (k, ' '.join('%r' % float(q) for q in vertex.ravel()), ei / inertia,
                                         gj / (2.0 * inertia), ratio * critical, r, density))
            welds.append('<weld body1="h%dB_last" body2="%s" class="held"/>' % (k, seg_b))   # held, not struck
            apart.append('<exclude body1="h%dB_last" body2="%s"/>' % (k, seg_b))
            for seg, centre, axis, radius, half in span['solids']:
                inside[seg].append('<geom type="cylinder" size="%r %r" pos="%r %r %r" zaxis="%r %r %r" contype="2" '
                                   'conaffinity="1" class="held"/>'
                                   % tuple(float(q) for q in (radius, half, *centre, *axis)))
        self.dt = min(steps) if steps else 0.001
        held = 'solref="%r 1" solimp="0.99 0.99 0.001"' % (2.0 * self.dt)     # as stiff as its step holds: MuJoCo's
        xml = ['<mujoco><option timestep="%r" gravity="0 %r 0" integrator="implicitfast"><flag autoreset='
               '"disable"/></option><extension><plugin plugin='
               '"mujoco.elasticity.cable"/></extension><default><default class="held"><equality %s/><geom %s/>'
               '</default></default><worldbody>' % (self.dt, -g, held, held)]   # soft rows scale with a link's mass
        for seg in self.segments:
            turn, put = poses[seg]
            xml += ['<body name="%s" pos="%r %r %r" quat="%r %r %r %r">' % (
                (seg,) + tuple(float(q) for q in (*put, *_quat(turn))))] + inside[seg] + ['</body>']
        xml += ['</worldbody><contact>'] + apart + ['</contact><equality>'] + welds + ['</equality></mujoco>']
        self.model = mujoco.MjModel.from_xml_string(''.join(xml).replace('{held}', held))   # a cable's own geom
        self.data = mujoco.MjData(self.model)
        names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(self.model.nbody)]
        self._chains = [np.array([i for i, n in enumerate(names) if n and n.startswith('h%dB_' % k)])
                        for k in range(len(spans))]
        for ids in self._chains:                         # the link in the far fitting: the fitting's own weight
            self.model.body_mass[ids[-1]] = HELD_KG
            self.model.body_inertia[ids[-1]] = HELD_KG * HELD_M * HELD_M
        mujoco.mj_setConst(self.model, self.data)
        self._own, self._lie = self.model.dof_damping.copy(), self.model.dof_damping.copy()
        for ids, critical in zip(self._chains, lie):
            for i in ids:
                first = self.model.body_dofadr[i]
                self._lie[first:first + self.model.body_dofnum[i]] = critical
        self._rides = np.array([self.model.body(seg).id for seg in self.segments], int)
        self._held = self._placed(poses)
        mujoco.mj_forward(self.model, self.data)
        self.settle()

    def _placed(self, poses):
        import numpy as np
        return (np.array([np.asarray(poses[seg][1], float) for seg in self.segments]).reshape(-1, 3),
                np.array([_quat(poses[seg][0]) for seg in self.segments]).reshape(-1, 4))

    def kept(self):
        """The world's state - its hoses' lie, its fittings' places - to come back to (`restore`)."""
        d = self.data
        return d.time, d.qpos.copy(), d.qvel.copy(), self._held

    def restore(self, kept):
        """Back to a state `kept` had."""
        mujoco = libm_bound.mujoco()
        d = self.data
        d.time, d.qpos[:], d.qvel[:], self._held = kept
        self.model.body_pos[self._rides], self.model.body_quat[self._rides] = self._held
        mujoco.mj_forward(self.model, d)

    def _stepped(self):
        """One step; a MachineError where the world fell over (MuJoCo's warnings of a bad state counted)."""
        from machine.errors import MachineError
        mujoco = libm_bound.mujoco()
        mujoco.mj_step(self.model, self.data)
        if int(self.data.warning.number.sum()):
            raise MachineError("the hoses' world fell over at %.4f s, its step %.3g s" % (self.data.time, self.dt))

    def settle(self):
        """Steps with the fittings held, each hose's first mode damped critically, until no joint turns faster than
        STILL_RAD_S - FALL_S at most."""
        mujoco = libm_bound.mujoco()
        import numpy as np
        self.model.dof_damping[:] = self._lie
        for _ in range(int(FALL_S / self.dt)):
            self._stepped()
            if float(np.abs(self.data.qvel).max(initial=0.0)) < STILL_RAD_S:
                break
        self.model.dof_damping[:] = self._own

    def follow(self, poses, seconds):
        """Steps `seconds` - CARRIED_S at most - each fitting's segment carried from where it was to `poses`."""
        mujoco = libm_bound.mujoco()
        import numpy as np
        (p0, q0), (p1, q1) = self._held, self._placed(poses)
        q1 = np.where((q0 * q1).sum(axis=1, keepdims=True) < 0.0, -q1, q1)
        n = int(math.ceil(min(max(float(seconds), 0.0), CARRIED_S) / self.dt))
        for i in range(1, n + 1):
            f = i / n
            q = q0 + f * (q1 - q0)
            self.model.body_pos[self._rides] = p0 + f * (p1 - p0)
            self.model.body_quat[self._rides] = q / np.linalg.norm(q, axis=1, keepdims=True)
            self._stepped()
        if not n:                                        # no time: the fittings put there, the hose as it lay
            self.model.body_pos[self._rides], self.model.body_quat[self._rides] = p1, q1
        self._held = (p1, q1)

    def lines(self, inside=False):
        """[(k, 3)]: each hose's centre line from the mouth of the fitting it leaves to the mouth of the one it goes
        into, the world's frame; with `inside` its link in each fitting too."""
        import numpy as np
        out = []
        for ids, ell in zip(self._chains, self._ends):
            pts = self.data.xpos[ids]
            out.append(np.vstack([pts, pts[-1] + self.data.xmat[ids[-1]].reshape(3, 3)[:, 0] * ell]) if inside
                       else pts[1:].copy())
        return out
