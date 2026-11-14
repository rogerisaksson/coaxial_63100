"""A camera on a stabilised gimbal: its sight the world's, leant a little into a turn.

    aim = gimbal.step(aim, turn, vel, dt)        # (pan, tilt, roll), rad, a step on
    eye = gimbal.lens(at, turn)                  # where the lens is, m

Three axes, ideal - none lags what it is asked: the pan and the tilt after the way the frame
goes, its speed over the ground and its climb, FOLLOW a second - at rest after its nose, as far
as it stands upright: on its side or its back it has no nose over the ground, and the sight
keeps its own. The tilt looks TILT under that way, within its axis' ends. Under way the sight
takes a share of the frame's lean across its way: looked into the turn, rolled with it - at
rest none, the horizon level.
"""
import math

from machine.aircraft import compile as aircraft_compile

_GM = aircraft_compile.gimbal()
MOUNT = _GM['mount']
FOLLOW, NOSE_M_S, TILT = _GM['follow'], _GM['nose_m_s'], _GM['tilt']
TILTS = _GM['tilts']
LOOK, LEAN, MOST, LEAN_M_S = _GM['look'], _GM['lean'], _GM['most'], _GM['lean_m_s']
REST = (0.0, -TILT, 0.0)


def step(aim, turn, vel, dt):
    """The sight `dt` s on, (pan, rad about up from z toward x; tilt, rad over the horizon;
    roll, rad, leant to the side a growing pan looks to): `aim` come round to the way the
    frame goes - `vel`, m/s, and its nose, `turn`'s z -, and to its shares of the frame's
    lean across that way, the rotors' axis off up to that side of it."""
    pan, tilt, roll = aim
    speed = math.hypot(float(vel[0]), float(vel[2]))
    upright, fast = NOSE_M_S * max(0.0, float(turn[1][1])), speed / NOSE_M_S
    x = fast * float(vel[0]) + upright * float(turn[0][2])
    z = fast * float(vel[2]) + upright * float(turn[2][2])
    firm, share = math.hypot(x, z), -math.expm1(-FOLLOW * dt)
    want = max(TILTS[0], min(TILTS[1], math.atan2(float(vel[1]), speed + NOSE_M_S) - TILT))
    across = (float(turn[0][1]) * z - float(turn[2][1]) * x) / (firm + 1e-9)
    lean = math.asin(max(-1.0, min(1.0, across))) * speed / (speed + LEAN_M_S)
    look, bank = (max(-MOST, min(MOST, k * lean)) for k in (LOOK, LEAN))
    return (pan + math.remainder(math.atan2(x, z) + look - pan, math.tau) * share * firm
            / (firm + NOSE_M_S), tilt + (want - tilt) * share, roll + (bank - roll) * share)


def lens(at, turn):
    """Where the lens is, m: MOUNT in the frame at `at`, turned `turn`."""
    return tuple(float(at[k]) + sum(float(turn[k][j]) * MOUNT[j] for j in range(3))
                 for k in range(3))
