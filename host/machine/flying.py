"""The quad's flying as one law: a hover, a climb, an orbit and a flip rows of the same setpoints.

    flying = Flying(top, aerobatics.DOWN)          # the four rotors' thrust at their fastest, N
    flying.ask = aerobatics.HOVER                  # any pass: the setpoints, nothing else
    thrusts = flying.step(sky.state(), dt)      # each rotor's thrust, every pass

The frame is asked a height and the pace it may go there at; a speed along its heading and
across it, its heading's turn, and how much its spot goes home; turns about its nose and about
its wing. Up and down it flies a speed for each distance from its height (`descent`): the
altitude loop's own approach near it, the rotors' pull - rising, gravity's - above where the two
meet, its pace at most and come to as gently. Asked a height far under it at any pace it is let
fall, the rotors run down as their propellers do, and burns as that speed comes to be its own.
Along the floor its spot goes at
the speed asked - home, as far as that is asked - and it leans to keep on it and into what that
speed changes by. Its discs keep that lean and its nose its heading, turned about its own axes
as far as its roll and its flip have come - asked no longer, a turn comes round to the whole
one - and the thrust is what of it all its discs give where they point; turned over, they
bear its weight. A figure is a row of `machine.aerobatics`; nothing here knows one from another.
"""
import math

from machine.figure import mul, rx, rz, t
from machine.quad import (ARM_M, BODY_CDA, GRAVITY, IDLE_RAD_S, INERTIA, K_DRAG, K_THRUST,
                          MASS_KG, RHO, ROTOR_AT, SPIN)

#: The altitude loop's gain on its rate, 1/s: both its poles at KD / 2, 5 rad/s. At 2 rad/s on
#: the height alone the lift trailed its ramp 0.4 m and the hold crept 3 s down to its mark
#: (2026-10-05).
KD = 10.0

#: The attitude's, 1/s^2 and 1/s, on its tilt and its heading and their rates. A rotor 5 % weak
#: held the frame 5 degrees over on the tilt's loop alone and walked it 8 m in a hover
#: (2026-09-28). The heading's torque is the discs' drag, 0.5 N m in a hover: at 16 and 8 a
#: pirouette of 180 deg/s swung the rotors 1 470-2 080 rpm at the clamp, both ways
#: (2026-10-05).
TILT_KP, TILT_KD, YAW_KP, YAW_KD = 60.0, 12.0, 4.0, 4.0

#: Along the floor: the spot's loop, 1/s^2 and 1/s, and the lean at most, m/s^2 - in a hover
#: 39 degrees, its angle at most whatever is asked up; its spot goes home at this gain, 1/s,
#: this fast at most, m/s.
SPOT_KP, SPOT_KD, LEAN_M_S2 = 1.0, 1.6, 8.0
HOME_K, HOME_M_S = 1.5, 3.0

#: The rotors' collective at most, of their top: at every rotor's cap the tilt's loop had
#: nothing left to turn with, and a rotor 5 % weak flipped the frame (2026-09-28).
HEADROOM = 0.9

#: A height is flown to braked at this share of what the rotors' headroom gives - rising, of
#: gravity: the rest is the rotors' spool's, begun KD's band ahead of the speed. Scheduled once
#: at full thrust a burn stopped 0.86 m up; begun at 85 %, the rotors spooled from idle on a 50
#: A clamp, the switches at 0.95 of the envelope throttled, and a flight burned into the floor
#: at 9.7 m/s (2026-09-28). Asked the pull that stops it, the spool's allowance kept through
#: the burn, it stopped 0.44 m up and crept 3 s to its mark; the allowance as much of the spool
#: as was yet to make, it braked at 0.9 g of its 1.5 and crept as long (2026-10-05).
BRAKE_SHARE = 0.55

#: A pace is come to and stopped from in this long, s, stopped sooner only as its height needs:
#: at 3 m/s down on the rotors' share it braked 1.2 g in a pass, and came to 2 m/s up at 1.3 g
#: (2026-10-05). The thrust itself slewed to that pull, 15 N/s in a hover, the altitude loop
#: swung 0.4 m about its height and never held (2026-10-05).
PACE_S = 1.0

#: A rotor's run down from its top on its propeller's drag alone, s: the can's and the
#: propeller's 9.2e-4 kg m^2 over K_DRAG at 310 rad/s. Asked less than their idle the rotors
#: come to it no faster: stepped to it, the loops braked at the clamp and spent 0.16-0.25 of
#: the envelope, on the floor and ahead of a burn (2026-10-05).
RUNDOWN_S = 0.58

#: A turn leaves the whole one and comes to it at this gain, 1/s, on how far from it it is: at
#: both ends alike, what the discs push as it begins they push back as it ends.
ROUND_K = 6.0

#: Held: within this of its height, m, and of its spot, m, this slow, m/s, its turns whole
#: within this, rad.
HELD_M, HELD_SPOT_M, HELD_M_S, HELD_RAD = 0.03, 0.15, 0.15, 0.05

#: A height asked this much faster than the last, m/s, is a row's jump: no rate of its own.
JUMP_M_S = 20.0

#: What it is doing where it is asked a height far under it: let fall, burning.
FALL, BURN = 'fall', 'burn'


def descent(over, brake):
    """The speed the frame flies `over` m from its height, m/s: the altitude loop's own approach
    near it, `brake`'s constant pull beyond where the two meet in speed and pull."""
    pole = 0.5 * KD
    near = brake / (pole * pole)
    return pole * over if over <= near else math.sqrt(2.0 * brake * (over - 0.5 * near))


def _unit(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Flying:

    """Each rotor's thrust for the quad's flying, every pass, from what the frame reads."""

    def __init__(self, top, ask):
        self.top, self.ask = top, dict(ask)
        self.idle = 4.0 * K_THRUST * IDLE_RAD_S ** 2
        #: Its heading, rad about up from z, and its turn, rad/s; how far it has turned about
        #: its nose and its wing, rad, and how fast, rad/s.
        self.heading, self.swing = 0.0, 0.0
        self.turns, self.turning = [0.0, 0.0], [0.0, 0.0]
        #: The collective asked last, N.
        self.thrust = self.idle
        #: The height asked last and its rate, m/s: the reference's own.
        self.height, self.rate = self.ask['height'], 0.0
        #: Its spot, world (x, z), m, and the speed that went at last, m/s.
        self.spot, self.speed = [0.0, 0.0], (0.0, 0.0)
        self.held, self.doing = False, None

    # -- the setpoints' own ------------------------------------------------------------------

    def climb(self, h, v, dt, share):
        """(the pull asked up, m/s^2, what it is doing) for the frame at `h` climbing `v`, the
        rotors' pull `share` of their own."""
        a = self.ask
        over = a['height'] - h
        # an eased height's own rate, its pace at most, and pull, as gentle - come to its pace
        # in PACE_S; a row's jump has no rate
        gentle = a['pace'] / PACE_S
        moved = a['height'] - self.height
        rate = max(-a['pace'], min(a['pace'], moved / dt)) if abs(moved) < JUMP_M_S * dt else 0.0
        push = max(-gentle, min(gentle, (rate - self.rate) / dt))
        self.height, self.rate = a['height'], rate
        way = 1.0 if over >= 0.0 else -1.0
        left, toward, going, pole = abs(over), way * (v - rate), way * v, 0.5 * KD
        # braked as its pace asks or as its height needs, the rotors' share - rising, gravity's
        # - at most
        hard = BRAKE_SHARE * (share * (HEADROOM * self.top / MASS_KG - GRAVITY) if way < 0.0
                              else GRAVITY)
        soft = min(gentle, hard)
        need = pole * pole * (left - math.sqrt(max(0.0, left * left - (toward / pole) ** 2))) \
            if toward > 0.0 else 0.0
        brake = min(hard, max(soft, need))
        # the speed its distance allows on its height's own, its pace at most, and that
        # speed's own fall, begun where its pace's stop turns: sped toward its height as
        # gently, the height's own pull in it, and stopped outright going from it
        cruise = a['pace'] - way * rate
        speed = min(descent(left, brake), cruise)
        slows = min(brake, pole * pole * left, KD * max(0.0, cruise - descent(left, soft)))
        pull = way * (min(KD * (way * rate + speed - max(going, 0.0)) - slows + way * push,
                          gentle) - KD * min(going, 0.0))
        falls = way < 0.0 and left > brake / (pole * pole)
        return pull, (FALL if MASS_KG * (GRAVITY + pull) <= self.idle else BURN) if falls else None

    def lean(self, at, vel, dt):
        """The pull asked along the floor, world (x, z), m/s^2: its spot gone on at the speed
        asked and home, the frame kept on it and leant into what that speed changed by and
        the air takes of it."""
        a = self.ask
        c, s = math.cos(self.heading), math.sin(self.heading)
        far = math.hypot(self.spot[0], self.spot[1])
        home = -a['home'] * min(HOME_K, HOME_M_S / far if far > 0.0 else HOME_K)
        speed = (a['speed'] * s + a['slide'] * c + home * self.spot[0],
                 a['speed'] * c - a['slide'] * s + home * self.spot[1])
        into = [(now - was) / dt for now, was in zip(speed, self.speed)]
        air = 0.5 * RHO * BODY_CDA * math.hypot(speed[0], speed[1]) / MASS_KG
        self.speed = speed
        self.spot = [p + v * dt for p, v in zip(self.spot, speed)]
        out = [into[k] + air * speed[k] + SPOT_KD * (speed[k] - float(vel[axis]))
               + SPOT_KP * (self.spot[k] - float(at[axis])) for k, axis in enumerate((0, 2))]
        size = math.hypot(out[0], out[1])
        return [x * min(1.0, LEAN_M_S2 / size) if size > 0.0 else 0.0 for x in out]

    def turned(self, dt):
        """Its turns' rates about its nose and its wing, rad/s, the turns moved on `dt`: as
        asked - or, asked no longer, on at the rate it had, a turn a quarter made made whole -
        begun and brought round to the whole one alike, ROUND_K of how far it is from whole at
        most."""
        for k, key in enumerate(('roll', 'flip')):
            rate = math.tau * self.ask[key]
            if rate != 0.0:
                rate = math.copysign(min(abs(rate), ROUND_K * (abs(self.turns[k]) + HELD_RAD)),
                                     rate)
            elif self.turns[k] != 0.0:
                on = math.copysign(0.25, self.turning[k]) if self.turning[k] else 0.0
                left = math.tau * round(self.turns[k] / math.tau + on) - self.turns[k]
                rate = math.copysign(min(abs(self.turning[k]) or math.tau, ROUND_K * abs(left)),
                                     left)
                if abs(left) < HELD_RAD:
                    self.turns[k], rate = 0.0, 0.0
            self.turning[k] = rate
            self.turns[k] += rate * dt
        return self.turning

    # -- every pass --------------------------------------------------------------------------

    def step(self, frame, dt, share=1.0):
        """Each rotor's thrust, N, for the frame as `frame` (`quad.Sky.state`) has it, `dt` s
        on; `share` what the rotors have of their pull, a board derated."""
        a = self.ask
        turn = [[float(frame['turn'][r][c]) for c in range(3)] for r in range(3)]
        spin, at, vel = frame['spin'], frame['at'], frame['vel']
        rate = math.radians(a['turn'])
        self.heading += rate * dt
        up_pull, self.doing = self.climb(frame['h'], frame['v'], dt, share)
        along = self.lean(at, vel, dt)
        # the discs' lean - against gravity at least, LEAN_M_S2's angle at most - the nose's
        # heading, its turns on them
        up = _unit((along[0], GRAVITY + max(0.0, up_pull), along[1]))
        nose = (math.sin(self.heading), 0.0, math.cos(self.heading))
        dot = sum(n * u for n, u in zip(nose, up))
        ahead = _unit([n - dot * u for n, u in zip(nose, up)])
        across = _cross(up, ahead)
        rates = self.turned(dt)
        want = mul(tuple(zip(across, up, ahead)), mul(rz(self.turns[0]), rx(self.turns[1])))
        miss = mul(t(want), turn)
        tilt = (0.5 * (miss[2][1] - miss[1][2]), 0.5 * (miss[0][2] - miss[2][0]),
                0.5 * (miss[1][0] - miss[0][1]))
        asked = (rates[1] + rate * want[1][0], rate * want[1][1], rates[0] + rate * want[1][2])
        torque = [INERTIA[k] * (-kp * tilt[k] - kd * (float(spin[k]) - asked[k]))
                  for k, (kp, kd) in enumerate(((TILT_KP, TILT_KD), (YAW_KP, YAW_KD),
                                                (TILT_KP, TILT_KD)))]
        torque[1] += INERTIA[1] * (rate - self.swing) / dt * want[1][1]
        self.swing = rate
        # what its discs give of the pull where they point; turned over, its weight - a whole
        # turn's push is none; under the idle, run down to it
        wanted = MASS_KG * (along[0] * turn[0][1] + max(0.0, GRAVITY + up_pull) * turn[1][1]
                            + along[1] * turn[2][1])
        if any(self.turns):
            wanted = MASS_KG * GRAVITY
        elif wanted <= self.idle:
            down = RUNDOWN_S * math.sqrt(self.top / self.thrust) if self.thrust > self.idle else 0.0
            wanted = max(self.idle, self.thrust / (1.0 + dt / down) ** 2) if down else self.idle
        self.thrust = min(HEADROOM * self.top, wanted)
        self.held = (abs(a['height'] - frame['h']) <= HELD_M
                     and max(abs(float(x)) for x in vel) <= HELD_M_S
                     and not any(self.turns)
                     and math.hypot(float(at[0]) - self.spot[0],
                                    float(at[2]) - self.spot[1]) <= HELD_SPOT_M)
        return self.shared(torque)

    def shared(self, torque):
        """The collective and `torque`, N m about the frame's axes, shared over the rotors - an
        X, its diagonals spun alike - none under nothing nor over its top: the tilt's torque
        kept, the collective next, the heading's what they leave."""
        reach, cap = 4.0 * ARM_M * ARM_M, self.top / 4.0
        parts = [-torque[0] * z / reach + torque[2] * x / reach for x, z in ROTOR_AT]
        each = max(-min(parts), min(self.thrust / 4.0, cap - max(parts)))
        room = max(0.0, min(min(each + part, cap - each - part) for part in parts))
        yaw = max(-room, min(room, torque[1] * K_THRUST / (4.0 * K_DRAG)))
        return [max(0.0, min(cap, each + part + yaw * s)) for part, s in zip(parts, SPIN)]
