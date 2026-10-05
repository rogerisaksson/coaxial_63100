"""A quad on four coaxial boards in MuJoCo: its frame, on what its rotors turn, and its pack.

    sky = Sky()                                   # the frame on the floor, y up
    sky.step(rotor_speeds, dt)                    # the frame on what the rotors turn
    volts = drawn(cells, watts, dt)               # its pack's bus under what they take

A 2 kg frame on four direct-drive 63100 rotors under APC 20x10E propellers (board/emu/worlds/
quad.json) on skids: gravity, the air on it and the floor; its boards on one pack of 15 cells,
63 V full. Its flying is `machine.flying`'s law on `machine.aerobatics`' rows; the rotors are
the caller's.
"""
import importlib
import math
import os

#: The frame, kg, and gravity, m/s^2.
MASS_KG, GRAVITY = 2.0, 9.81

#: A rotor's thrust and drag against its speed squared, mechanical rad/s: quad.json's CT 0.10
#: on the 0.508 m disc, and motor.loads' APC20x10E fit.
K_THRUST, K_DRAG = 2.07e-4, 5.143e-6

#: The air on the frame, m^2 of drag area - an assumption, a 2 kg frame and four 0.5 m discs
#: edge on - and its density, kg/m^3.
BODY_CDA, RHO = 0.15, 1.2

#: Where the rotors stand round the frame's middle, m, x right and z ahead in its own frame,
#: the discs 0.6 m apart for 0.508 m props, and the way each turns seen from above: the
#: diagonals alike, so their drags' torques cancel. Front left, front right, rear right, rear
#: left.
ARM_M = 0.3
ROTOR_AT = ((-ARM_M, ARM_M), (ARM_M, ARM_M), (ARM_M, -ARM_M), (-ARM_M, -ARM_M))
SPIN = (1.0, -1.0, 1.0, -1.0)

#: The discs' height over the frame's middle, and the skids' under it, m.
DISC_M, SKID_M = 0.06, 0.12

#: The hover's height, the stop over the floor, m, and the rotors' idle, rad/s: a sixth of
#: the frame's weight, the quad sitting on the floor.
HOVER_M, FLOOR_M, IDLE_RAD_S = 1.5, 0.1, 60.0

#: The world's step, s.
STEP_S = 0.002

#: The frame's inertia, kg m^2 about its axes: a rotor's share of its mass at each corner.
_CORNER = MASS_KG / 8.0 * 2.0 * ARM_M * ARM_M
INERTIA = (_CORNER, 2.0 * _CORNER, _CORNER)


def mjcf():
    """The quad as MuJoCo's XML, y up: the frame free over the floor, its mass a plate's with a
    rotor at each corner, on four skid feet."""
    feet = ''.join('<geom type="sphere" size="0.015" pos="%g %g %g" contype="1" '
                   'conaffinity="1"/>' % (0.7 * x, -SKID_M, 0.7 * z) for x, z in ROTOR_AT)
    return ('<mujoco model="quad"><option timestep="%g" gravity="0 %g 0"/><worldbody>'
            '<geom name="floor" type="plane" size="50 50 0.1" zaxis="0 1 0" contype="1" '
            'conaffinity="1"/>'
            '<body name="frame" pos="0 %g 0"><freejoint name="root"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>%s</body>'
            '</worldbody></mujoco>' % ((STEP_S, -GRAVITY, SKID_M + 0.015, MASS_KG)
                                       + INERTIA + (feet,)))


class Sky:

    """The frame in MuJoCo on the floor, stepped on its rotors' speeds."""

    def __init__(self):
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        self._mj = importlib.import_module('mujoco')
        self._np = importlib.import_module('numpy')
        self.model = self._mj.MjModel.from_xml_string(mjcf())
        self.data = self._mj.MjData(self.model)
        self.frame = self.model.body('frame').id
        self._mj.mj_forward(self.model, self.data)

    def reset(self):
        """The frame back on its skids at its spot, still."""
        self._mj.mj_resetData(self.model, self.data)
        self._mj.mj_forward(self.model, self.data)

    def state(self):
        """{'h', 'v', 'a'}: the frame's height over its rest on the skids, m, its climb, m/s,
        and acceleration, m/s^2, up; 'turn' its 3x3 in the world, 'spin' its rates about its
        own axes, rad/s, 'at' its middle, m."""
        d = self.data
        return {'h': float(d.qpos[1]) - (SKID_M + 0.015), 'v': float(d.qvel[1]),
                'a': float(d.qacc[1]), 'turn': d.xmat[self.frame].reshape(3, 3).copy(),
                'spin': d.qvel[3:6].copy(), 'at': d.qpos[0:3].copy(),
                'vel': d.qvel[0:3].copy()}

    def step(self, speeds, dt):
        """The frame `dt` s on under the rotors at `speeds`, mechanical rad/s: each one's thrust
        up its axis at its place, their drags' torques about it, the air on the frame."""
        np, d = self._np, self.data
        for _ in range(max(1, int(round(dt / STEP_S)))):
            turn = d.xmat[self.frame].reshape(3, 3)
            up = turn[:, 1]
            force, torque = np.zeros(3), np.zeros(3)
            for (x, z), spin, w in zip(ROTOR_AT, SPIN, speeds):
                thrust = K_THRUST * w * w
                force += thrust * up
                torque += (np.cross(turn @ np.array([x, DISC_M, z]), thrust * up)
                           + spin * K_DRAG * w * w * up)
            v = d.qvel[0:3]
            d.xfrc_applied[self.frame, 0:3] = force - 0.5 * RHO * BODY_CDA * np.linalg.norm(v) * v
            d.xfrc_applied[self.frame, 3:6] = torque
            self._mj.mj_step(self.model, d)


def speed_for(thrust):
    """A rotor's speed for `thrust` of its own, mechanical rad/s."""
    return math.sqrt(max(0.0, thrust) / K_THRUST)


#: The pack: PACK_CELLS in series, a cell's open volts by the share of its charge left - a LiPo's
#: curve, 63 V full -, its charge, A h - a demo's, a few flights -, and its and its leads'
#: resistance, ohm, the bus drooping that much a link amp: assumptions with a name each. Spent
#: at RESERVE of its charge.
PACK_CELLS, PACK_AH, PACK_OHM, RESERVE = 15, 0.22, 0.12, 0.2
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
