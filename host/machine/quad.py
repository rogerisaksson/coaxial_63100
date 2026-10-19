"""A quad on four coaxial boards in MuJoCo: its frame and on what its rotors turn.

    sky = Sky()                                   # the frame on the floor, y up
    sky.step(rotor_speeds, dt, air)               # on what the rotors turn, in `weather`'s air

A 2 kg frame on four direct-drive 63100 rotors under 16x14 propellers (board/emu/worlds/
quad.json) on skids: gravity, the air on it (`machine.weather`) and the floor; its boards on one
pack of 15 cells (`machine.flightpack`). Its flying is `machine.aerobatics`' rows made payloads
(`machine.aerokin`) for its core (aero/); the rotors are the caller's.
"""
import importlib
import math
import os

from machine import aerolisp
from machine.weather import wind_at
from tools.cores import bodied

QUAD_LISP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'quadcopter.lisp')
with open(QUAD_LISP_PATH, encoding='utf-8') as _qf:
    MANIFEST = aerolisp.compile_manifest(_qf.read())
AERO = MANIFEST['aerodynamics']
DYNAMICS = MANIFEST['dynamics']
KINEMATICS = MANIFEST['kinematics']
STREAMS = MANIFEST['streams']
STUNTS = MANIFEST['stunts']
BEHAVIORS = MANIFEST['behaviors']
_ROT, _AF = AERO['rotors'], AERO['airframe']
_RB, _GOV = DYNAMICS['rigid_body'], DYNAMICS['governor']
_ENV = KINEMATICS['envelope']

#: The frame, kg, and gravity, m/s^2.
MASS_KG, GRAVITY = float(_RB['mass']), float(_RB['gravity'])

#: A rotor's thrust and drag against its speed squared, mechanical rad/s: a 16x14's, CT 0.133
#: and CP 0.077 - the APC20x10E's (quad.json's CT 0.10, motor.loads' fit, CP 0.032) scaled to
#: its pitch as APC's thin electric series go. On the pack's 63 V the 20x10 held the 63100 to
#: 34 % of its no-load speed at its 30 A and to a pitch's speed of 12.5 m/s at the 2 960 rpm
#: it was flown on, the frame level at 13 m/s; a 16x14 at 60 A turns 7 350 rpm, the link's
#: ceiling, 13 times the frame's weight, 43 m/s its pitch's speed.
K_THRUST, K_DRAG = _ROT['k_thrust'], _ROT['k_drag']

#: The air on the frame, m^2 of drag area - an assumption, a 2 kg frame and four 0.5 m discs
#: edge on - and its density, kg/m^3.
BODY_CDA, RHO = _AF['parasite_cda'], _AF['air_density']

#: Where the rotors stand round the frame's middle, m, x right and z ahead in its own frame,
#: the discs 0.6 m apart for 0.406 m props, and the way each turns seen from above: the
#: diagonals alike, so their drags' torques cancel. Front left, front right, rear right, rear
#: left.
ARM_M = _ROT['arm_length']
ROTOR_AT = ((-ARM_M, ARM_M), (ARM_M, ARM_M), (ARM_M, -ARM_M), (-ARM_M, -ARM_M))
SPIN = (1.0, -1.0, 1.0, -1.0)

#: The discs' height over the frame's middle, the skids' under it and a skid's foot's radius,
#: m; what of the frame strikes a thing: a disc, a 16 in propeller's radius and half its
#: depth, and the hub's half sizes, m.
DISC_M, SKID_M, FOOT_M = _ROT['disc_height'], _RB['skid_height'], _RB['foot_radius']
DISC_R, DISC_HALF_M, HUB_M = (
    _ROT['disc_radius'], _ROT['disc_half_height'], (_RB['hub_x'], _RB['hub_y'], _RB['hub_z']))

#: The hover's height, the stop over the floor, m, and the rotors' idle, rad/s: a sixth of
#: the frame's weight, the quad sitting on the floor.
HOVER_M, FLOOR_M, IDLE_RAD_S = _ENV['hover_height'], _ENV['floor_height'], _GOV['idle_speed']

#: Its rotors come to a speed asked of them in about this long, s: the frame's clock. The
#: law's loops (`aerokin`) and the lap's plan (`course`) are tuned by it and go by it at
#: another size - the tilt's loop at twice its gain rang on these rotors.
SPIN_S = _GOV['spin_time']

#: The world's step, s.
STEP_S = _GOV['step_time']

#: A disc's thrust falls with the air through it along its axis - the frame's own way, the
#: wind's and its eddy's against it -, to none at INFLOW_J0 of its pitch's speed, a 16x14's
#: 0.356 m a turn: APC's zero thrust stands a little past the geometric pitch.
INFLOW_J0, PITCH_M = _ROT['inflow_j0'], _ROT['blade_pitch']

#: The frame's inertia, kg m^2 about its axes: a rotor's share of its mass at each corner.
_CORNER = MASS_KG / 8.0 * 2.0 * ARM_M * ARM_M
INERTIA = (_CORNER, 2.0 * _CORNER, _CORNER)

#: The frame's numbers as they go with its size, each by the size to this power (`sized`):
#: its lengths, its mass, its propellers' thrust and their drag's torque at a speed, its
#: drag's area, its rotors' idle - a sixth of its weight at any size.
_POWERS = {'MASS_KG': 3.0, 'ARM_M': 1.0, 'K_THRUST': 4.0, 'K_DRAG': 5.0, 'BODY_CDA': 2.0,
           'DISC_M': 1.0, 'SKID_M': 1.0, 'FOOT_M': 1.0, 'DISC_R': 1.0, 'DISC_HALF_M': 1.0,
           'PITCH_M': 1.0, 'IDLE_RAD_S': -0.5, 'HOVER_M': 1.0, 'FLOOR_M': 1.0,
           'SPIN_S': 1.0}
_BUILT = dict({name: globals()[name] for name in _POWERS}, HUB_M=HUB_M)


def sized(size=1.0):
    """The frame `size` times as large, its shape and its propellers' tips' speed kept: its
    lengths by `size`, its mass by the cube and its inertia by the fifth, its propellers'
    thrust by the fourth and their drag's torque by the fifth at a speed, its drag's area by
    the square, its rotors as much slower to a speed - set where they live, for every
    reader. Its rotors' top is 1/size of theirs, the caller's as they are. The law, the
    routine and the course go by it once each is `sized` after it."""
    scope = globals()
    for name, power in _POWERS.items():
        scope[name] = _BUILT[name] * size ** power
    arm, corner = scope['ARM_M'], scope['MASS_KG'] / 8.0 * 2.0 * scope['ARM_M'] ** 2
    scope.update(HUB_M=tuple(size * x for x in _BUILT['HUB_M']),
                 ROTOR_AT=((-arm, arm), (arm, arm), (arm, -arm), (-arm, -arm)),
                 INERTIA=(corner, 2.0 * corner, corner))


def reach():
    """The frame from its middle to a propeller's tip, m."""
    return math.hypot(*ROTOR_AT[0]) + DISC_R


def scales():
    """(the frame's size, its clock) of the frame's as built: what its lengths and its times
    go by."""
    return ARM_M / _BUILT['ARM_M'], SPIN_S / _BUILT['SPIN_S']


def rescaled(scope, units, built):
    """`scope`'s constants named in `units` - {name: (m, s)}, the powers of its metres and its
    seconds - set for the frame as it is sized, from `built`: their values for the frame as
    built, taken the first time."""
    size, clock = scales()
    if not built:
        built.update({name: scope[name] for name in units})
    for name, (metres, seconds) in units.items():
        scope[name] = built[name] * size ** metres * clock ** seconds


def skids(skid_x, skid_m, strut_z, foot_m):
    """Four skid feet at `(-skid_x, skid_x)` by `strut_z`, `skid_m` under the frame, as MuJoCo
    `<geom>` tags."""
    return ''.join('<geom name="skid%d" type="sphere" pos="%g %g %g" size="%g" '
                   'solref="0.01 1" friction="0.8 0.005 0.0001"/>' % (k, sx, -skid_m, sz, foot_m)
                   for k, (sx, sz) in enumerate((sx, sz) for sx in (-skid_x, skid_x) for sz in strut_z))


def mjcf(things=(), geoms=None, pos=None, mass=None, inertia=None):
    """The quad (or a craft with `geoms` at `pos`, `mass` and `inertia`) as MuJoCo's XML, y up:
    the frame free over the floor, its mass a plate's with a rotor at each corner, on four skid
    feet, its discs and its hub what it strikes with; `things` what stands about it
    (`grounds.solids`), each ('box', its name, its middle, its half sizes, its heading, degrees),
    ('rod', name, an end, the other, its radius) or ('hull', name, the points it is the hull
    of), m."""
    if geoms is None:
        geoms = ''.join('<geom name="skid%d" type="sphere" size="%g" pos="%g %g %g"/>'
                        '<geom name="disc%d" type="cylinder" size="%g %g" pos="%g %g %g" '
                        'euler="90 0 0"/>' % (k, FOOT_M, 0.7 * x, -SKID_M, 0.7 * z, k, DISC_R,
                                              DISC_HALF_M, x, DISC_M, z)
                        for k, (x, z) in enumerate(ROTOR_AT))
        geoms += '<geom name="hub" type="box" size="%g %g %g"/>' % HUB_M
    pos = (0.0, SKID_M + FOOT_M, 0.0) if pos is None else pos
    mass = MASS_KG if mass is None else mass
    inertia = INERTIA if inertia is None else inertia
    hulls, stood = '', ''
    for n, (shape, name, *size) in enumerate(things):
        if shape == 'box':
            stood += '<geom name="%s.%d" type="box" pos="%g %g %g" size="%g %g %g" ' \
                     'euler="0 %g 0"/>' % ((name, n) + tuple(size[0]) + tuple(size[1])
                                           + (size[2],))
        elif shape == 'rod':
            stood += '<geom name="%s.%d" type="capsule" fromto="%g %g %g %g %g %g" ' \
                     'size="%g"/>' % ((name, n) + tuple(size[0]) + tuple(size[1]) + (size[2],))
        else:
            hulls += '<mesh name="hull%d" vertex="%s"/>' % (
                n, ' '.join('%g' % c for point in size[0] for c in point))
            stood += '<geom name="%s.%d" type="mesh" mesh="hull%d"/>' % (name, n, n)
    return ('<mujoco model="quad"><option timestep="%g" gravity="0 %g 0"/><asset>%s</asset>'
            '<worldbody><geom name="floor" type="plane" size="50 50 0.1" zaxis="0 1 0"/>%s'
            '<body name="frame" pos="%g %g %g"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((STEP_S, -GRAVITY, hulls, stood) + tuple(pos) + (mass,)
                                       + tuple(inertia) + (geoms,)))


def substep(sky, fx_b, fy_b, wx, wy, wz, dk, ab0, ab1, ab2, px, py, pz, tx, ty, tz):
    """One STEP_S MuJoCo substep on `sky`: body forces `(fx_b, fy_b, 0)`, body torques
    `(ab0, ab1, ab2)`, drag factor `dk` in wind `(wx, wy, wz)` and world wrench offset
    `(px, py, pz, tx, ty, tz)` into `xfrc_applied`, then `mj_step`."""
    np, xm, qv, xf = sky._np, sky._xmat, sky._vel, sky._xfrc
    vx, vy, vz = qv[0] - wx, qv[1] - wy, qv[2] - wz
    dr = dk * np.sqrt(vx * vx + vy * vy + vz * vz)
    xf[0] = fy_b * xm[1] + fx_b * xm[0] - dr * vx + px
    xf[1] = fy_b * xm[4] + fx_b * xm[3] - dr * vy + py
    xf[2] = fy_b * xm[7] + fx_b * xm[6] - dr * vz + pz
    xf[3] = xm[0] * ab0 + xm[1] * ab1 + xm[2] * ab2 + tx
    xf[4] = xm[3] * ab0 + xm[4] * ab1 + xm[5] * ab2 + ty
    xf[5] = xm[6] * ab0 + xm[7] * ab1 + xm[8] * ab2 + tz
    sky._mj.mj_step(sky.model, sky.data)


class Sky:

    """The frame in MuJoCo on the floor among `things` (`mjcf`), stepped on its rotors'
    speeds."""

    def __init__(self, things=(), xml=None, rest=None, touch=()):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        self._mj = importlib.import_module('mujoco')
        self._np = importlib.import_module('numpy')
        self.model = self._mj.MjModel.from_xml_string(mjcf(things) if xml is None else xml)
        self.data = self._mj.MjData(self.model)
        self.frame = self.model.body('frame').id
        self.rest = SKID_M + FOOT_M if rest is None else rest
        #: What the rotors gave at the last step in still air, N: their speeds' own.
        self.lift = 0.0
        #: What the frame struck first since it was put on its spot - a thing's name, the
        #: 'floor' with more of it than its skids - or None.
        self.hit = None
        names = [self.model.geom(g).name for g in range(self.model.ngeom)]
        self._names = [name.split('.')[0].rpartition(':')[2] for name in names]
        self._mine = {g for g in range(self.model.ngeom)
                      if self.model.geom_bodyid[g] == self.frame}
        floor = names.index('floor')
        self._feet = {(min(g, floor), max(g, floor))
                      for g, name in enumerate(names)
                      if name.startswith('skid') or name in touch}
        self._gates = [g for g, name in enumerate(names) if name.startswith('gate')]
        self._course = [g for g, name in enumerate(names) if name.startswith('course:')]
        self._field = [g for g, name in enumerate(names) if name.startswith('field:')]
        self.gated = True
        for g in self._field:
            self.model.geom_contype[g] = self.model.geom_conaffinity[g] = 0
        self._xmat = self.data.xmat[self.frame]
        self._turn = self._xmat.reshape(3, 3)
        self._spin = self.data.qvel[3:6]
        self._at = self.data.qpos[0:3]
        self._vel = self.data.qvel[0:3]
        self._acc = self.data.qacc[0:3]
        self._xfrc = self.data.xfrc_applied[self.frame]
        self._qp = memoryview(self.data.qpos)
        self._qv = memoryview(self.data.qvel)
        self._qa = memoryview(self.data.qacc)
        self._xm = memoryview(self._xmat)
        self._step = bodied.on_body(substep, 'substep', tuple(range(1, 16)))
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """The frame back on its skids at its spot, still, nothing struck."""
        self._mj.mj_resetData(self.model, self.data)
        self.lift, self.hit = 0.0, None
        self._mj.mj_forward(self.model, self.data)

    def stand(self, gates):
        """The gates and the course's grounds among the things stood in the frame's way and the
        airfield's not, or `gates` False the other way round: each stands for its own flights,
        as it is drawn (`grounds.world`)."""
        if gates != self.gated:
            self.gated = gates
            for g in self._gates + self._course:
                self.model.geom_contype[g] = self.model.geom_conaffinity[g] = int(gates)
            for g in self._field:
                self.model.geom_contype[g] = self.model.geom_conaffinity[g] = int(not gates)

    def _struck(self, data):
        """What the frame of `data` touches with more than a skid on the floor: its name, or
        None."""
        con = data.contact
        feet, mine, names = self._feet, self._mine, self._names
        for d_val, (a, b) in zip(con.dist.tolist(), con.geom.tolist()):
            if d_val <= 0.0:
                pair = (a, b) if a < b else (b, a)
                if pair not in feet and (a in mine or b in mine):
                    return names[b if a in mine else a]
        return None

    def state(self, copy=True):
        """{'h', 'v', 'a'}: the frame's height over its rest on the skids, m, its climb, m/s,
        and acceleration, m/s^2, up; 'turn' its 3x3 in the world, 'spin' its rates about its
        own axes, rad/s, 'at' its middle, m, 'vel' and 'acc' its speed and what that changes
        by in the world; 'lift' what its rotors' speeds give in still air, N; 'hit' what it
        struck first, or None."""
        qp, qv, qa = self._qp, self._qv, self._qa
        if not copy:
            return {'h': qp[1] - self.rest, 'v': qv[1],
                    'a': qa[1], 'turn': self._turn, 'spin': self._spin,
                    'at': self._at, 'vel': self._vel, 'acc': self._acc,
                    'lift': self.lift, 'hit': self.hit}
        return {'h': qp[1] - self.rest, 'v': qv[1],
                'a': qa[1], 'turn': self._turn.copy(),
                'spin': self._spin.copy(), 'at': self._at.copy(),
                'vel': self._vel.copy(), 'acc': self._acc.copy(), 'lift': self.lift,
                'hit': self.hit}

    def _substep(self, fx_b, fy_b, wx, wy, wz, dk, ab0, ab1, ab2,
                 px=0.0, py=0.0, pz=0.0, tx=0.0, ty=0.0, tz=0.0):
        """One STEP_S MuJoCo substep under body forces `(fx_b, fy_b, 0)`, body torques
        `(ab0, ab1, ab2)`, drag factor `dk` in wind `(wx, wy, wz)`, and world wrench offset."""
        self._step(self, fx_b, fy_b, wx, wy, wz, dk, ab0, ab1, ab2, px, py, pz, tx, ty, tz)
        if self.data.ncon and self.hit is None:
            self.hit = self._struck(self.data)

    def step(self, speeds, dt, air=None):
        """The frame `dt` s on under the rotors at `speeds`, mechanical rad/s: each one's thrust
        up its axis at its place, their drags' torques about it, the air on the frame - still,
        or `air` (`blown`): its drag against the wind where it is, a quarter of it at each
        disc in that disc's own eddy, a disc's thrust the more for the air rising through it."""
        qv, xm = self._qv, self._xm
        thrusts = [K_THRUST * w * w for w in speeds]
        self.lift = sum(thrusts)
        wx = wy = wz = px = py = pz = tx = ty = tz = 0.0
        if air is None:
            along = qv[0] * xm[1] + qv[1] * xm[4] + qv[2] * xm[7]
            thrusts = [t * inflow(along, w) for t, w in zip(thrusts, speeds)]
        else:
            h = self._qp[1] - (SKID_M + FOOT_M)
            wx, wy, wz = wind_at(air['wind'], h)
            r00, r01, r02, r10, r11, r12, r20, r21, r22 = (
                xm[0], xm[1], xm[2], xm[3], xm[4], xm[5], xm[6], xm[7], xm[8])
            s0, s1, s2 = qv[0] - wx, qv[1] - wy, qv[2] - wz
            fast = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2)
            k_eddy = -0.125 * RHO * BODY_CDA
            for k, ((x, z), eddy) in enumerate(zip(ROTOR_AT, air['eddies'])):
                ex, ey, ez = wind_at(eddy, h)
                o0, o1, o2 = s0 - ex, s1 - ey, s2 - ez
                thrusts[k] *= inflow(o0 * r01 + o1 * r11 + o2 * r21, speeds[k])
                size = math.sqrt(o0 * o0 + o1 * o1 + o2 * o2)
                m0 = k_eddy * (size * o0 - fast * s0)
                m1 = k_eddy * (size * o1 - fast * s1)
                m2 = k_eddy * (size * o2 - fast * s2)
                a0, a1, a2 = r00 * x + r02 * z, r10 * x + r12 * z, r20 * x + r22 * z
                px += m0
                py += m1
                pz += m2
                tx += a1 * m2 - a2 * m1
                ty += a2 * m0 - a0 * m2
                tz += a0 * m1 - a1 * m0
        lift = sum(thrusts)
        ab0 = -sum(t * z for t, (_x, z) in zip(thrusts, ROTOR_AT))
        ab1 = sum(spin * K_DRAG * w * w for spin, w in zip(SPIN, speeds))
        ab2 = sum(t * x for t, (x, _z) in zip(thrusts, ROTOR_AT))
        dk = 0.5 * RHO * BODY_CDA
        for _ in range(max(1, int(round(dt / STEP_S)))):
            self._substep(0.0, lift, wx, wy, wz, dk, ab0, ab1, ab2, px, py, pz, tx, ty, tz)



def inflow(along, w):
    """A disc's thrust's share with the air through it `along` its axis, m/s - the disc moving
    up it -, at `w` rad/s: falling to none at INFLOW_J0 of its pitch's speed."""
    return max(0.0, 1.0 - along / (INFLOW_J0 * max(1.0, PITCH_M * abs(w) / math.tau)))


def speed_for(thrust, along=0.0):
    """A rotor's speed for `thrust` of its own, mechanical rad/s, the air `along` its axis
    (`inflow`): the root of K w^2 (1 - along / (INFLOW_J0 PITCH_M w / tau)) = thrust."""
    lose = K_THRUST * max(0.0, along) * math.tau / (INFLOW_J0 * PITCH_M)
    return (lose + math.sqrt(lose * lose + 4.0 * K_THRUST * max(0.0, thrust))) / (2.0 * K_THRUST)


COOLING = DYNAMICS['cooling']


