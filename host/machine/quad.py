"""A quad on four coaxial boards in MuJoCo: its frame, its flight and the thrust that flies it.

    sky = Sky()                                   # the frame on the floor, y up
    name, thrust = plan(route, sky.state(), top, now, ready)
    thrusts = mix(sky.state(), thrust)            # each rotor's share, level held
    sky.step(rotor_speeds, dt)                    # the frame on what the rotors turn

A 2 kg frame on four direct-drive 63100 rotors under APC 20x10E propellers (board/emu/worlds/
quad.json) on skids: gravity, the air on it and the floor. The flight spools, lifts to a hover
and holds it until the boards' thermal observers have converged - blasted at the floor's
margin they throttle and trip - goes full tilt into the sky, falls on rotors run down to
their idle and burns to a stop 10 cm over the floor, holds there and lands. The rotors are the
caller's.
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

#: The altitude loop about a height and its rate, 1/s^2 and 1/s: both poles at KD / 2, 5
#: rad/s. At 2 rad/s on the height alone the lift trailed its ramp 0.4 m and the hold crept 3 s
#: down to its mark (2026-10-05). The attitude's, 1/s^2 and 1/s on the tilt and its rate, and
#: the yaw's damping, 1/s; the spot's, 1/s^2 and 1/s, the frame tilted back over where it rose,
#: its lean at most LEAN_M_S2. A rotor 5 % weak held the frame 5 degrees over on the tilt's
#: loop alone and walked it 8 m in a hover (2026-09-28).
KP, KD = 25.0, 10.0
TILT_KP, TILT_KD, YAW_KD = 60.0, 12.0, 4.0
SPOT_KP, SPOT_KD, LEAN_M_S2 = 1.0, 1.6, 3.0

#: The rotors' collective at full tilt and in the burn, of their top: at every rotor's cap
#: the tilt's loop had nothing left to turn with, and a rotor 5 % weak flipped the frame.
HEADROOM = 0.9

#: The flight: each stage's name and seconds, None where the body ends it - the fall when the
#: burn must start, the burn when the frame has stopped; the hover also waits for `ready`.
STAGES = (('spool', 2.0), ('lift', 3.0), ('hover', 3.0), ('full tilt', 2.0), ('fall', None),
          ('burn', None), ('hold', 3.0), ('land', 3.0))

#: The burn, flown down a speed for each height over its mark (`descent`): braked at this share
#: of what the rotors' headroom gives, then eased in on the altitude loop's own pole, the
#: rotors' spool from idle run at the fall's speed first, s, at the whole clamp. Scheduled once
#: at full thrust it stopped 0.86 m up; begun at 85 %, the rotors spooled from idle on the 50
#: A clamp, the switches at 0.95 of the envelope throttled, and a flight burned into the floor
#: at 9.7 m/s (2026-09-28). Asked the pull that stops it, the spool's allowance kept through
#: the burn, it stopped 0.44 m up and crept 3 s to its mark (2026-10-05).
BURN_SHARE, SPOOL_S = 0.55, 0.3

#: The burn is the hold's from this speed down, m/s.
HANDOVER_M_S = 0.5

#: The landing's reference ends this far under the floor, m: the skids down before the spool.
PRESS_M = 0.02

#: A rotor's run down from its top on its propeller's drag alone, s: the can's and the
#: propeller's 9.2e-4 kg m^2 over K_DRAG at 310 rad/s. The fall and the landed spool ask their
#: idle no faster: stepped to it, the loops braked at the clamp and spent 0.16-0.25 of the
#: envelope, on the floor and ahead of the burn (2026-10-05).
RUNDOWN_S = 0.58

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
    """The flight at its first stage: the stage, when it began, the thrust it began on and the
    last asked, N."""
    return {'stage': 0, 'at': 0.0, 'begun': 0.0, 'asked': 0.0}


def descent(over, brake):
    """The speed down the burn flies `over` m above its mark, m/s: the altitude loop's own
    approach near it, `brake`'s constant pull above where the two meet in speed and pull."""
    pole = 0.5 * KD
    near = brake / (pole * pole)
    return pole * over if over <= near else math.sqrt(2.0 * brake * (over - 0.5 * near))


def ease(x):
    """A move's share made `x` of the way through its time, its rate and its rate's, at rest
    at both ends."""
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x), 6.0 * x * (1.0 - x), 6.0 - 12.0 * x


def plan(route, frame, top, now, ready=True, spool_s=SPOOL_S):
    """(the stage's name, the thrust it asks, N): `route` moved on where its stage is done -
    by its clock, the hover's once `ready` too, or by the body for the fall and the burn - `top`
    the four rotors' thrust at their fastest, N, `spool_s` their spool to it, s."""
    name, seconds = STAGES[route['stage']]
    into = now - route['at']
    h, v = frame['h'], frame['v']
    brake = BURN_SHARE * (HEADROOM * top / MASS_KG - GRAVITY)
    done = ((into >= seconds and (ready or name != 'hover')) if seconds is not None else
            (name == 'fall' and v < 0.0 and -v >= descent(h + v * spool_s - FLOOR_M, brake))
            or (name == 'burn' and v >= -HANDOVER_M_S))
    if done:
        route.update(stage=(route['stage'] + 1) % len(STAGES), at=now, begun=route['asked'])
        name, seconds = STAGES[route['stage']]
        into = 0.0
    route['asked'] = thrust = _thrust(name, seconds, into, route['begun'], h, v, top, brake)
    return name, thrust


def _thrust(name, seconds, into, begun, h, v, top, brake):
    """The thrust a stage asks `into` s of it, N, begun on `begun`."""
    idle = 4.0 * K_THRUST * IDLE_RAD_S ** 2
    if name in ('spool', 'fall'):
        # Down to the idle as the propellers run down, from the thrust the stage began on.
        down = RUNDOWN_S * math.sqrt(top / begun) if begun > idle else 0.0
        return max(idle, begun / (1.0 + into / down) ** 2) if down else idle
    if name == 'full tilt':
        return HEADROOM * top
    # A height, its rate and its rate's to fly: the lift and the landing eased, the burn's
    # the speed its height over the mark allows, the air's drag its own.
    height, rate, pull, air = (HOVER_M if name == 'hover' else FLOOR_M), 0.0, 0.0, 0.0
    if name in ('lift', 'land'):
        span = HOVER_M if name == 'lift' else -(FLOOR_M + PRESS_M)
        share, slope, bend = ease(into / seconds)
        height = (0.0 if name == 'lift' else FLOOR_M) + span * share
        rate, pull = span * slope / seconds, span * bend / (seconds * seconds)
    elif name == 'burn' and h - FLOOR_M > brake / (0.25 * KD * KD):
        height, rate, pull = h, -descent(h - FLOOR_M, brake), brake
        air = 0.5 * RHO * BODY_CDA * v * v
    wanted = MASS_KG * (GRAVITY + pull + KP * (height - h) + KD * (rate - v)) - air
    return min(HEADROOM * top, max(idle, wanted))


def speed_for(thrust):
    """A rotor's speed for `thrust` of its own, mechanical rad/s."""
    return math.sqrt(max(0.0, thrust) / K_THRUST)
