"""A quad on four coaxial boards in MuJoCo: its frame, its flight and the thrust that flies it.

    sky = Sky()                                   # the frame on the floor, y up
    name, thrust = plan(route, sky.state(), top, now, ready)
    thrusts = mix(sky.state(), thrust)            # each rotor's share, level held
    sky.step(rotor_speeds, dt)                    # the frame on what the rotors turn

A 2 kg frame on four direct-drive 63100 rotors under APC 20x10E propellers (board/emu/worlds/
quad.json) on skids: gravity, the air on it and the floor. The flight spools, lifts to a hover
and holds it until the boards' thermal observers have converged - blasted at the floor's
margin they throttle and trip - goes full tilt into the sky, falls with the rotors idling and
burns to stop dead 10 cm over the floor, holds there and lands. The rotors are the caller's.
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

#: The altitude loop, 1/s^2 and 1/s: a couple of rad/s, damped; the attitude's, 1/s^2 and 1/s
#: on the tilt and its rate, and the yaw's damping, 1/s; the spot's, 1/s^2 and 1/s, the frame
#: tilted back over where it rose, its lean at most LEAN_M_S2. A rotor 5 % weak held the frame
#: 5 degrees over on the tilt's loop alone and walked it 8 m in a hover (2026-09-28).
KP, KD = 4.0, 3.0
TILT_KP, TILT_KD, YAW_KD = 60.0, 12.0, 4.0
SPOT_KP, SPOT_KD, LEAN_M_S2 = 1.0, 1.6, 3.0

#: The rotors' collective at full tilt and in the burn, of their top: at every rotor's cap
#: the tilt's loop had nothing left to turn with, and a rotor 5 % weak flipped the frame.
HEADROOM = 0.9

#: The flight: each stage's name and seconds, None where the body ends it - the fall when the
#: burn must start, the burn when the frame has stopped; the hover also waits for `ready`.
STAGES = (('spool', 2.0), ('lift', 3.0), ('hover', 3.0), ('full tilt', 2.0), ('fall', None),
          ('burn', None), ('hold', 3.0), ('land', 3.0))

#: The burn, flown to its mark: begun when the deceleration that stops the fall at FLOOR_M is
#: this share of the rotors' and the air's, the rotors' spool from idle to their top run at
#: the fall's speed first, s, at the whole clamp. Scheduled once at full thrust it stopped
#: 0.86 m up; begun at 85 %, the rotors spooled from idle on the 50 A clamp, the switches
#: at 0.95 of the envelope throttled, and a flight burned into the floor at 9.7 m/s; at 55 %,
#: 0.32 and 0.73, stopped 0.3 m up (2026-09-28).
BURN_SHARE, SPOOL_S = 0.55, 0.3

#: The burn hands the frame to the hold's damped loop at this speed down, m/s: ended at a
#: standstill it crept 7 s on rotors lagging 0.08 s, and bounced 1.8 m on 0.15.
HANDOVER_M_S = 0.5

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


def mix(state, thrust):
    """Each rotor's thrust for `thrust` in all, N, the frame held over its spot and unturned:
    the lean that takes it back there, the tilt from that lean and its rate made torques about
    the frame's axes and shared over the rotors - an X, its diagonals spun alike."""
    turn, spin, at, vel = state['turn'], state['spin'], state['at'], state['vel']
    up = turn[:, 1]
    lean = [max(-LEAN_M_S2, min(LEAN_M_S2, -SPOT_KP * at[k] - SPOT_KD * vel[k])) for k in (0, 2)]
    norm = math.sqrt(lean[0] ** 2 + GRAVITY ** 2 + lean[1] ** 2)
    want = (lean[0] / norm, GRAVITY / norm, lean[1] / norm)
    # The turn that stands the frame on the lean wanted, in its own axes: up x wanted.
    upright = (up[1] * want[2] - up[2] * want[1], up[2] * want[0] - up[0] * want[2],
               up[0] * want[1] - up[1] * want[0])
    tilt = [sum(turn[r][c] * upright[r] for r in range(3)) for c in range(3)]
    tx, tz = ((TILT_KP * tilt[k] - TILT_KD * spin[k]) * INERTIA[k] for k in (0, 2))
    ty = -YAW_KD * spin[1] * INERTIA[1]
    ratio, reach = K_DRAG / K_THRUST, 4.0 * ARM_M * ARM_M
    return [max(0.0, thrust / 4.0 - tx * z / reach + tz * x / reach + ty * s / (4.0 * ratio))
            for (x, z), s in zip(ROTOR_AT, SPIN)]


def flight():
    """The flight at its first stage."""
    return {'stage': 0, 'at': 0.0}


def plan(route, frame, top, now, ready=True, spool_s=SPOOL_S):
    """(the stage's name, the thrust it asks, N): `route` moved on where its stage is done -
    by its clock, the hover's once `ready` too, or by the body for the fall and the burn - `top`
    the four rotors' thrust at their fastest, N, `spool_s` their spool to it, s."""
    name, seconds = STAGES[route['stage']]
    into = now - route['at']
    air = 0.5 * RHO * BODY_CDA * frame['v'] * frame['v']
    need = frame['v'] * frame['v'] / (
        2.0 * max(0.02, frame['h'] + frame['v'] * spool_s - FLOOR_M))
    done = ((into >= seconds and (ready or name != 'hover')) if seconds is not None else
            (name == 'fall' and frame['v'] < 0.0
             and need >= BURN_SHARE * ((HEADROOM * top + air) / MASS_KG - GRAVITY))
            or (name == 'burn' and frame['v'] >= -HANDOVER_M_S))
    if done:
        route['stage'] = (route['stage'] + 1) % len(STAGES)
        route['at'] = now
        name, into = STAGES[route['stage']][0], 0.0
    idle = 4.0 * K_THRUST * IDLE_RAD_S ** 2
    if name in ('spool', 'fall'):
        return name, idle
    if name == 'full tilt':
        return name, HEADROOM * top
    if name == 'burn':
        return name, min(HEADROOM * top, max(idle, MASS_KG * (GRAVITY + need) - air))
    if name == 'lift':
        height = HOVER_M * min(1.0, into / STAGES[1][1])
    elif name == 'land':
        height = FLOOR_M * max(0.0, 1.0 - into / STAGES[-1][1])
    else:
        height = HOVER_M if name == 'hover' else FLOOR_M
    return name, max(idle, MASS_KG * (GRAVITY + KP * (height - frame['h']) - KD * frame['v']))


def speed_for(thrust):
    """A rotor's speed for `thrust` of its own, mechanical rad/s."""
    return math.sqrt(max(0.0, thrust) / K_THRUST)
