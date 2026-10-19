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


def mjcf(things=()):
    """The quad as MuJoCo's XML, y up: the frame free over the floor, its mass a plate's with a
    rotor at each corner, on four skid feet, its discs and its hub what it strikes with;
    `things` what stands about it (`grounds.solids`), each ('box', its name, its middle, its
    half sizes, its heading, degrees), ('rod', name, an end, the other, its radius) or
    ('hull', name, the points it is the hull of), m."""
    mine = ''.join('<geom name="skid%d" type="sphere" size="%g" pos="%g %g %g"/>'
                   '<geom name="disc%d" type="cylinder" size="%g %g" pos="%g %g %g" '
                   'euler="90 0 0"/>' % (k, FOOT_M, 0.7 * x, -SKID_M, 0.7 * z, k, DISC_R,
                                         DISC_HALF_M, x, DISC_M, z)
                   for k, (x, z) in enumerate(ROTOR_AT))
    mine += '<geom name="hub" type="box" size="%g %g %g"/>' % HUB_M
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
            '<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((STEP_S, -GRAVITY, hulls, stood, SKID_M + FOOT_M, MASS_KG)
                                       + INERTIA + (mine,)))


class Sky:

    """The frame in MuJoCo on the floor among `things` (`mjcf`), stepped on its rotors'
    speeds."""

    def __init__(self, things=()):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        self._mj = importlib.import_module('mujoco')
        self._np = importlib.import_module('numpy')
        self.model = self._mj.MjModel.from_xml_string(mjcf(things))
        self.data = self._mj.MjData(self.model)
        self.frame = self.model.body('frame').id
        self.rest = SKID_M + FOOT_M
        #: What the rotors gave at the last step in still air, N: their speeds' own.
        self.lift = 0.0
        #: What the frame struck first since it was put on its spot - a thing's name, the
        #: 'floor' with more of it than its skids - or None.
        self.hit = None
        names = [self.model.geom(g).name for g in range(self.model.ngeom)]
        self._names = [name.split('.')[0] for name in names]
        self._mine = {g for g in range(self.model.ngeom)
                      if self.model.geom_bodyid[g] == self.frame}
        self._feet = {(min(g, names.index('floor')), max(g, names.index('floor')))
                      for g, name in enumerate(names) if name.startswith('skid')}
        self._gates = [g for g, name in enumerate(names) if name.startswith('gate')]
        self.gated = True
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """The frame back on its skids at its spot, still, nothing struck."""
        self._mj.mj_resetData(self.model, self.data)
        self.lift, self.hit = 0.0, None
        self._mj.mj_forward(self.model, self.data)

    def stand(self, gates):
        """The gates among the things stood in the frame's way, or `gates` False - not: they
        stand for the course's flights alone, as they are drawn."""
        if gates != self.gated:
            self.gated = gates
            for g in self._gates:
                self.model.geom_contype[g] = self.model.geom_conaffinity[g] = int(gates)

    def _struck(self, data):
        """What the frame of `data` touches with more than a skid on the floor: its name, or
        None."""
        geom, dist, ncon = data.contact.geom, data.contact.dist, data.ncon
        feet, mine, names = self._feet, self._mine, self._names
        for k in range(ncon):
            if dist[k] <= 0.0:
                a, b = int(geom[k, 0]), int(geom[k, 1])
                pair = (a, b) if a < b else (b, a)
                if pair not in feet and (a in mine or b in mine):
                    return names[b if a in mine else a]
        return None

    def state(self):
        """{'h', 'v', 'a'}: the frame's height over its rest on the skids, m, its climb, m/s,
        and acceleration, m/s^2, up; 'turn' its 3x3 in the world, 'spin' its rates about its
        own axes, rad/s, 'at' its middle, m, 'vel' and 'acc' its speed and what that changes
        by in the world; 'lift' what its rotors' speeds give in still air, N; 'hit' what it
        struck first, or None."""
        d = self.data
        return {'h': float(d.qpos[1]) - self.rest, 'v': float(d.qvel[1]),
                'a': float(d.qacc[1]), 'turn': d.xmat[self.frame].reshape(3, 3).copy(),
                'spin': d.qvel[3:6].copy(), 'at': d.qpos[0:3].copy(),
                'vel': d.qvel[0:3].copy(), 'acc': d.qacc[0:3].copy(), 'lift': self.lift,
                'hit': self.hit}

    def step(self, speeds, dt, air=None):
        """The frame `dt` s on under the rotors at `speeds`, mechanical rad/s: each one's thrust
        up its axis at its place, their drags' torques about it, the air on the frame - still,
        or `air` (`blown`): its drag against the wind where it is, a quarter of it at each
        disc in that disc's own eddy, a disc's thrust the more for the air rising through it."""
        np, d = self._np, self.data
        # The four's lift and their torque about the frame's own axes, the pass's: a thrust up
        # the frame's axis at (x, z) turns it (-z, 0, x) of itself, the drags about that axis.
        thrusts = [K_THRUST * w * w for w in speeds]
        self.lift = sum(thrusts)
        wind, push, twist = np.zeros(3), np.zeros(3), np.zeros(3)
        turn = d.xmat[self.frame].reshape(3, 3)
        if air is None:
            along = float(np.dot(d.qvel[0:3], turn[:, 1]))
            thrusts = [t * inflow(along, w) for t, w in zip(thrusts, speeds)]
        else:
            # A disc's arithmetic in floats: as arrays of three 51 us of a call's 86, 19.
            h = float(d.qpos[1]) - (SKID_M + FOOT_M)
            blows, rows = wind_at(air['wind'], h), turn.tolist()
            still = [float(v) - u for v, u in zip(d.qvel[0:3], blows)]
            fast = math.sqrt(still[0] * still[0] + still[1] * still[1] + still[2] * still[2])
            axis, shove, turned = (rows[0][1], rows[1][1], rows[2][1]), [0.0] * 3, [0.0] * 3
            for k, ((x, z), eddy) in enumerate(zip(ROTOR_AT, air['eddies'])):
                own = [s - e for s, e in zip(still, wind_at(eddy, h))]
                thrusts[k] *= inflow(own[0] * axis[0] + own[1] * axis[1] + own[2] * axis[2],
                                     speeds[k])
                size = math.sqrt(own[0] * own[0] + own[1] * own[1] + own[2] * own[2])
                more = [-0.125 * RHO * BODY_CDA * (size * o - fast * s)
                        for o, s in zip(own, still)]
                arm = [row[0] * x + row[2] * z for row in rows]
                shove = [a + b for a, b in zip(shove, more)]
                turned = [turned[0] + arm[1] * more[2] - arm[2] * more[1],
                          turned[1] + arm[2] * more[0] - arm[0] * more[2],
                          turned[2] + arm[0] * more[1] - arm[1] * more[0]]
            wind, push, twist = np.array(blows), np.array(shove), np.array(turned)
        lift = sum(thrusts)
        about = np.array([-sum(t * z for t, (_x, z) in zip(thrusts, ROTOR_AT)),
                          sum(spin * K_DRAG * w * w for spin, w in zip(SPIN, speeds)),
                          sum(t * x for t, (x, _z) in zip(thrusts, ROTOR_AT))])
        drag_const = 0.5 * RHO * BODY_CDA
        wx, wy, wz = float(wind[0]), float(wind[1]), float(wind[2])
        px, py, pz = float(push[0]), float(push[1]), float(push[2])
        tx, ty, tz = float(twist[0]), float(twist[1]), float(twist[2])
        ab0, ab1, ab2 = float(about[0]), float(about[1]), float(about[2])
        frame = self.frame
        qvel = d.qvel
        xfrc = d.xfrc_applied
        for _ in range(max(1, int(round(dt / STEP_S)))):
            turn = d.xmat[frame].reshape(3, 3)
            vx = float(qvel[0]) - wx
            vy = float(qvel[1]) - wy
            vz = float(qvel[2]) - wz
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            drag = drag_const * speed
            t00, t01, t02 = float(turn[0, 0]), float(turn[0, 1]), float(turn[0, 2])
            t10, t11, t12 = float(turn[1, 0]), float(turn[1, 1]), float(turn[1, 2])
            t20, t21, t22 = float(turn[2, 0]), float(turn[2, 1]), float(turn[2, 2])
            xfrc[frame, 0] = lift * t01 - drag * vx + px
            xfrc[frame, 1] = lift * t11 - drag * vy + py
            xfrc[frame, 2] = lift * t21 - drag * vz + pz
            xfrc[frame, 3] = t00 * ab0 + t01 * ab1 + t02 * ab2 + tx
            xfrc[frame, 4] = t10 * ab0 + t11 * ab1 + t12 * ab2 + ty
            xfrc[frame, 5] = t20 * ab0 + t21 * ab1 + t22 * ab2 + tz
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)


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


