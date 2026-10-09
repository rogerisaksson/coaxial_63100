"""A quad on four coaxial boards in MuJoCo: its frame, on what its rotors turn, its air, its pack.

    sky = Sky()                                   # the frame on the floor, y up
    wind = blown(air, dt)                         # the air on: its wind, its gusts, its eddies
    sky.step(rotor_speeds, dt, air)               # the frame on what the rotors turn, in that air
    volts = drawn(cells, watts, dt)               # its pack's bus under what they take

A 2 kg frame on four direct-drive 63100 rotors under APC 20x10E propellers (board/emu/worlds/
quad.json) on skids: gravity, the air on it and the floor; its boards on one pack of 15 cells,
63 V full. The air is a kind of weather at a time (`AIR`): a wind, its way swinging, gusts over
it and eddies in it - what the frame is flown through, none of it told to its law. Its flying
is `machine.flying`'s law on `machine.aerobatics`' rows; the rotors are the caller's.
"""
import importlib
import math
import os
import random

#: The frame, kg, and gravity, m/s^2.
MASS_KG, GRAVITY = 2.0, 9.81

#: A rotor's thrust and drag against its speed squared, mechanical rad/s: a 16x14's, CT 0.133
#: and CP 0.077 - the APC20x10E's (quad.json's CT 0.10, motor.loads' fit, CP 0.032) scaled to
#: its pitch as APC's thin electric series go. On the pack's 63 V the 20x10 held the 63100 to
#: 34 % of its no-load speed at its 30 A and to a pitch's speed of 12.5 m/s at the 2 960 rpm
#: it was flown on, the frame level at 13 m/s; a 16x14 at 60 A turns 7 350 rpm, the link's
#: ceiling, 13 times the frame's weight, 43 m/s its pitch's speed (2026-10-09).
K_THRUST, K_DRAG = 1.0995e-4, 4.136e-6

#: The air on the frame, m^2 of drag area - an assumption, a 2 kg frame and four 0.5 m discs
#: edge on - and its density, kg/m^3.
BODY_CDA, RHO = 0.15, 1.2

#: Where the rotors stand round the frame's middle, m, x right and z ahead in its own frame,
#: the discs 0.6 m apart for 0.406 m props, and the way each turns seen from above: the
#: diagonals alike, so their drags' torques cancel. Front left, front right, rear right, rear
#: left.
ARM_M = 0.3
ROTOR_AT = ((-ARM_M, ARM_M), (ARM_M, ARM_M), (ARM_M, -ARM_M), (-ARM_M, -ARM_M))
SPIN = (1.0, -1.0, 1.0, -1.0)

#: The discs' height over the frame's middle, the skids' under it and a skid's foot's radius,
#: m; what of the frame strikes a thing: a disc, a 16 in propeller's radius and half its
#: depth, and the hub's half sizes, m.
DISC_M, SKID_M, FOOT_M = 0.06, 0.12, 0.015
DISC_R, DISC_HALF_M, HUB_M = 0.2032, 0.008, (0.06, 0.03, 0.06)

#: Looked ahead (`Sky.ahead`), a thing this near the frame's discs or hub is in its way, m.
NEAR_M = 0.15

#: The hover's height, the stop over the floor, m, and the rotors' idle, rad/s: a sixth of
#: the frame's weight, the quad sitting on the floor.
HOVER_M, FLOOR_M, IDLE_RAD_S = 1.5, 0.1, 86.0

#: Its rotors come to a speed asked of them in about this long, s: the frame's clock. The
#: law's loops (`flying`) and the lap's plan (`course`) are tuned by it and go by it at
#: another size - the tilt's loop at twice its gain rang on these rotors (2026-10-06).
SPIN_S = 0.08

#: The world's step, s.
STEP_S = 0.002

#: The air's kinds, a row each - a day's weather over a field at REF_M, assumptions: the wind's
#: mean, m/s; how far its way swings either side, degrees, and a swing's seconds; its gusts'
#: size over the mean, m/s, and the seconds between two; its eddies', m/s rms.
AIR = {
    'calm':      (0.5,  0.0,  0.0, 0.0, 0.0, 0.1),
    'constant':  (4.0,  0.0,  0.0, 0.0, 0.0, 0.2),
    'gusty':     (3.0, 15.0,  8.0, 4.0, 5.0, 0.5),
    'changing':  (3.5, 80.0, 14.0, 1.0, 9.0, 0.3),
    'turbulent': (2.5, 25.0,  6.0, 2.0, 6.0, 1.2),
}

#: The kinds in turn, KIND_S of each, the air come to a kind's sizes over EASE_S and round to
#: its own way, within TURN_DEG of the last.
TOUR, KIND_S, EASE_S, TURN_DEG = tuple(AIR), 20.0, 4.0, 120.0

#: A gust: up and down again as 1 - cos over so long, s, so much of its kind's size, within
#: GUST_DEG of the wind's way.
GUST_S, GUST_SHARE, GUST_DEG = (1.5, 3.0), (0.6, 1.0), 30.0

#: The eddies: the frame's turn over in EDDY_S; each disc has one of its own besides,
#: EDDY_DISC of the size, over in EDDY_DISC_S; up and down EDDY_UP of along the floor.
EDDY_S, EDDY_DISC_S, EDDY_DISC, EDDY_UP = 1.5, 0.3, 0.5, 0.5

#: The wind over the ground: (h / REF_M) ** SHEAR of what it is at REF_M, counted from SHEAR_M
#: up and SHEAR_MOST at the most; the air rises and falls not at all at the floor, wholly from
#: RISE_M up.
REF_M, SHEAR, SHEAR_M, SHEAR_MOST, RISE_M = 5.0, 0.2, 0.1, 1.5, 3.0

#: A disc's thrust falls with the air through it along its axis - the frame's own way, the
#: wind's and its eddy's against it -, to none at INFLOW_J0 of its pitch's speed, a 16x14's
#: 0.356 m a turn: APC's zero thrust stands a little past the geometric pitch.
INFLOW_J0, PITCH_M = 1.05, 0.3556

#: The frame's inertia, kg m^2 about its axes: a rotor's share of its mass at each corner.
_CORNER = MASS_KG / 8.0 * 2.0 * ARM_M * ARM_M
INERTIA = (_CORNER, 2.0 * _CORNER, _CORNER)

#: The frame's numbers as they go with its size, each by the size to this power (`sized`):
#: its lengths, its mass, its propellers' thrust and their drag's torque at a speed, its
#: drag's area, its rotors' idle - a sixth of its weight at any size.
_POWERS = {'MASS_KG': 3.0, 'ARM_M': 1.0, 'K_THRUST': 4.0, 'K_DRAG': 5.0, 'BODY_CDA': 2.0,
           'DISC_M': 1.0, 'SKID_M': 1.0, 'FOOT_M': 1.0, 'DISC_R': 1.0, 'DISC_HALF_M': 1.0,
           'PITCH_M': 1.0, 'NEAR_M': 1.0, 'IDLE_RAD_S': -0.5, 'HOVER_M': 1.0, 'FLOOR_M': 1.0,
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


def mjcf(things=(), near=0.0):
    """The quad as MuJoCo's XML, y up: the frame free over the floor, its mass a plate's with a
    rotor at each corner, on four skid feet, its discs and its hub what it strikes with -
    their contacts found `near` m off, for a frame that is only looked at; `things` what
    stands about it (`grounds.solids`), each ('box', its name, its middle, its half sizes, its
    heading, degrees), ('rod', name, an end, the other, its radius) or ('hull', name, the
    points it is the hull of), m."""
    near = 'margin="%g"' % near
    mine = ''.join('<geom name="skid%d" type="sphere" size="%g" pos="%g %g %g"/>'
                   '<geom name="disc%d" type="cylinder" size="%g %g" pos="%g %g %g" '
                   'euler="90 0 0" %s/>' % (k, FOOT_M, 0.7 * x, -SKID_M, 0.7 * z, k, DISC_R,
                                            DISC_HALF_M, x, DISC_M, z, near)
                   for k, (x, z) in enumerate(ROTOR_AT))
    mine += '<geom name="hub" type="box" size="%g %g %g" %s/>' % (HUB_M + (near,))
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
        #: The frame's ghost: where it will be, looked at for what is in its way (`ahead`),
        #: in a world of its own - a margin on the flown one's discs and hub bore the frame
        #: 4.5 cm off the floor.
        self._seen = self._mj.MjModel.from_xml_string(mjcf(things, NEAR_M))
        self._ghost = self._mj.MjData(self._seen)
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
            for model in (self.model, self._seen):
                for g in self._gates:
                    model.geom_contype[g] = model.geom_conaffinity[g] = int(gates)

    def _struck(self, data, near=0.0):
        """What the frame of `data` touches - or has within `near` m - with more than a skid
        on the floor: its name, or None."""
        pairs, gaps = data.contact.geom[:data.ncon].tolist(), data.contact.dist[:data.ncon]
        for (a, b), gap in zip(pairs, gaps.tolist()):
            if gap <= near and (min(a, b), max(a, b)) not in self._feet \
                    and (a in self._mine or b in self._mine):
                return self._names[b if a in self._mine else a]
        return None

    def ahead(self, seconds, looks=3):
        """(what the frame would strike, the seconds to it) as it goes - its speed and what
        that changes by kept, its discs and hub with NEAR_M to spare - within `seconds`, or
        None: its ghost put where it will be, `looks` times on the way. What it is that near
        already is not ahead: on the floor it lifts from and lands on, WEP was 5 s a flight
        (2026-10-09)."""
        d, ghost = self.data, self._ghost
        ghost.qpos[:] = d.qpos
        self._mj.mj_fwdPosition(self._seen, ghost)
        near = self._struck(ghost, NEAR_M)
        for k in range(1, looks + 1):
            t = seconds * k / looks
            ghost.qpos[:] = d.qpos
            ghost.qpos[0:3] += d.qvel[0:3] * t + 0.5 * d.qacc[0:3] * t * t
            self._mj.mj_fwdPosition(self._seen, ghost)
            thing = self._struck(ghost, NEAR_M)
            if thing and thing != near:
                return thing, t
        return None

    def state(self):
        """{'h', 'v', 'a'}: the frame's height over its rest on the skids, m, its climb, m/s,
        and acceleration, m/s^2, up; 'turn' its 3x3 in the world, 'spin' its rates about its
        own axes, rad/s, 'at' its middle, m, 'vel' and 'acc' its speed and what that changes
        by in the world; 'lift' what its rotors' speeds give in still air, N; 'hit' what it
        struck first, or None."""
        d = self.data
        return {'h': float(d.qpos[1]) - (SKID_M + FOOT_M), 'v': float(d.qvel[1]),
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
        # Crossed a rotor a step in the world's frame, 40 crosses were 1.0 ms of a 20 ms pass.
        thrusts = [K_THRUST * w * w for w in speeds]
        self.lift = sum(thrusts)
        wind, push, twist = np.zeros(3), np.zeros(3), np.zeros(3)
        turn = d.xmat[self.frame].reshape(3, 3)
        if air is None:
            along = float(np.dot(d.qvel[0:3], turn[:, 1]))
            thrusts = [t * inflow(along, w) for t, w in zip(thrusts, speeds)]
        else:
            h = float(d.qpos[1]) - (SKID_M + FOOT_M)
            wind = np.array(wind_at(air['wind'], h))
            still = d.qvel[0:3] - wind
            drag = np.linalg.norm(still) * still
            for k, ((x, z), eddy) in enumerate(zip(ROTOR_AT, air['eddies'])):
                own = np.array(wind_at(eddy, h))
                thrusts[k] *= inflow(float(np.dot(still - own, turn[:, 1])), speeds[k])
                more = -0.125 * RHO * BODY_CDA * (np.linalg.norm(still - own) * (still - own)
                                                  - drag)
                arm = turn @ (x, 0.0, z)
                push += more
                twist += (arm[1] * more[2] - arm[2] * more[1], arm[2] * more[0] - arm[0] * more[2],
                          arm[0] * more[1] - arm[1] * more[0])
        lift = sum(thrusts)
        about = np.array([-sum(t * z for t, (_x, z) in zip(thrusts, ROTOR_AT)),
                          sum(spin * K_DRAG * w * w for spin, w in zip(SPIN, speeds)),
                          sum(t * x for t, (x, _z) in zip(thrusts, ROTOR_AT))])
        for _ in range(max(1, int(round(dt / STEP_S)))):
            turn = d.xmat[self.frame].reshape(3, 3)
            v = d.qvel[0:3] - wind
            d.xfrc_applied[self.frame, 0:3] = (
                lift * turn[:, 1] - 0.5 * RHO * BODY_CDA * np.linalg.norm(v) * v + push)
            d.xfrc_applied[self.frame, 3:6] = turn @ about + twist
            self._mj.mj_step(self.model, d)
            if d.ncon and self.hit is None:
                self.hit = self._struck(d)


def air(seed=3, kind=None):
    """Air before its first second, still: the 'kind' it is in - toured from TOUR's first, or
    `kind` 'kept' -, how long 'for', its sizes 'come' so far (the wind's mean, its way's swing,
    its gusts', its eddies'); the 'way' its wind blows, rad from z toward x, the way it comes
    round 'to' and its 'swing'; its 'gust' (s into it, of, its share, rad off the way) and the
    seconds to the 'next'; the frame's 'eddy' and each disc's, m/s; what they make: its 'wind'
    at REF_M, m/s in the world, the gust in it 'blows', m/s."""
    return {'kind': kind or TOUR[0], 'kept': kind is not None, 'for': 0.0, 'come': [0.0] * 4,
            'way': 0.0, 'to': 0.0, 'swing': 0.0, 'gust': None, 'next': 0.0, 'blows': 0.0,
            'eddy': [0.0] * 3, 'eddies': [[0.0] * 3 for _ in ROTOR_AT],
            'dice': random.Random(seed), 'wind': (0.0, 0.0, 0.0)}


def blown(air, dt):
    """The air `dt` s on, its wind at REF_M: its kind's turn over, TOUR's next and a new way;
    its sizes eased to its kind's and its way round, swung; a gust begun where one is due,
    risen and fallen; its eddies turned over, each as far as its seconds let it."""
    dice = air['dice']
    air['for'] += dt
    if not air['kept'] and air['for'] >= KIND_S:
        air['kind'], air['for'] = TOUR[(TOUR.index(air['kind']) + 1) % len(TOUR)], 0.0
        air['to'] += math.radians(dice.uniform(-TURN_DEG, TURN_DEG))
    mean, veer, veer_s, gust, every, rough = AIR[air['kind']]
    ease = min(1.0, dt / EASE_S)
    mean, veer, gust, rough = air['come'] = [was + (now - was) * ease for was, now in zip(
        air['come'], (mean, veer, gust, rough))]
    air['way'] += (air['to'] - air['way']) * ease
    air['swing'] += math.tau * dt / veer_s if veer_s else 0.0
    way = air['way'] + math.radians(veer) * math.sin(air['swing'])
    if air['gust'] is None:
        air['next'] -= dt
        if every and air['next'] <= 0.0:
            air['gust'] = (0.0, dice.uniform(*GUST_S), dice.uniform(*GUST_SHARE),
                           math.radians(dice.uniform(-GUST_DEG, GUST_DEG)))
            air['next'] = every * dice.uniform(0.5, 1.5)
    blows, off = 0.0, 0.0
    if air['gust'] is not None:
        into, of, share, off = air['gust']
        blows = 0.5 * share * gust * (1.0 - math.cos(math.tau * into / of))
        air['gust'] = (into + dt, of, share, off) if into + dt < of else None
    for eddy, size, over in [(air['eddy'], rough, EDDY_S)] + [
            (own, EDDY_DISC * rough, EDDY_DISC_S) for own in air['eddies']]:
        keep = math.exp(-dt / over)
        kick = size * math.sqrt(1.0 - keep * keep)
        for k in range(3):
            eddy[k] = eddy[k] * keep + (EDDY_UP if k == 1 else 1.0) * kick * dice.gauss(0.0, 1.0)
    air['blows'] = blows
    air['wind'] = (mean * math.sin(way) + blows * math.sin(way + off) + air['eddy'][0],
                   air['eddy'][1],
                   mean * math.cos(way) + blows * math.cos(way + off) + air['eddy'][2])
    return air['wind']


def kept(air):
    """The air's kind stepped by hand: TOUR's first, kept, then the next; after the last, the
    tour again."""
    k = TOUR.index(air['kind']) + 1 if air['kept'] else 0
    air['kind'], air['kept'], air['for'] = TOUR[k % len(TOUR)], k < len(TOUR), 0.0


def wind_at(wind, h):
    """The air's speed `wind`, m/s at REF_M, where the frame is `h` m up: sheared along the
    floor, its rise and fall none at the floor."""
    low = min(SHEAR_MOST, (max(h, SHEAR_M) / REF_M) ** SHEAR)
    return (low * wind[0], low * min(1.0, max(0.0, h) / RISE_M) * wind[1], low * wind[2])


def inflow(along, w):
    """A disc's thrust's share with the air through it `along` its axis, m/s - the disc moving
    up it -, at `w` rad/s: falling to none at INFLOW_J0 of its pitch's speed."""
    return max(0.0, 1.0 - along / (INFLOW_J0 * max(1.0, PITCH_M * abs(w) / math.tau)))


def speed_for(thrust):
    """A rotor's speed for `thrust` of its own, mechanical rad/s."""
    return math.sqrt(max(0.0, thrust) / K_THRUST)


#: The pack: PACK_CELLS in series, a cell's open volts by the share of its charge left - a LiPo's
#: curve, 63 V full -, its charge, A h - a racer's, a few flights -, and its and its leads'
#: resistance, ohm, the bus drooping that much a link amp: assumptions with a name each. Spent
#: at RESERVE of its charge. At 0.12 ohm the four's 8 kW at their clamps took the bus to half.
PACK_CELLS, PACK_AH, PACK_OHM, RESERVE = 15, 1.0, 0.04, 0.2
CELL_V = ((0.0, 3.30), (0.05, 3.50), (0.2, 3.70), (0.5, 3.85), (0.9, 4.05), (1.0, 4.20))


def open_volts(left):
    """The pack's volts unloaded with `left` of its charge in it."""
    left = max(0.0, min(1.0, left))
    for (a, low), (b, high) in zip(CELL_V, CELL_V[1:]):
        if left <= b:
            return PACK_CELLS * (low + (high - low) * (left - a) / (b - a))
    return PACK_CELLS * CELL_V[-1][1]


def pack():
    """A fresh pack: the share of its charge left, its bus's volts, the amps and watts it gave
    last."""
    return {'left': 1.0, 'volts': open_volts(1.0), 'amps': 0.0, 'watts': 0.0}


def drawn(cells, watts, dt):
    """The pack `cells` `dt` s on with `watts` taken at its bus - given, braked into: the bus's
    volts, its open volts less PACK_OHM a link amp, its charge less what those amps took."""
    volts = open_volts(cells['left'])
    amps = (volts - math.sqrt(max(0.0, volts * volts - 4.0 * PACK_OHM * watts))) / (2.0 * PACK_OHM)
    cells.update(left=max(0.0, min(1.0, cells['left'] - amps * dt / (3600.0 * PACK_AH))),
                 volts=volts - PACK_OHM * amps, amps=amps, watts=watts)
    return cells['volts']
