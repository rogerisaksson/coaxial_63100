#!/usr/bin/env python3
"""A text's streams flown: `machine.quad`'s frame in MuJoCo on the core (aero/), ideal rotors.

    python tools/sim/streams.py                                  # the power loop, a row a stream
    python tools/sim/streams.py my.aero --seconds 3,2 --air 3    # a file's, in the tour's air
    python tools/sim/streams.py --upsets                         # a hover asked from upsets

A row a stream: where it ends - height, speed, the nose's and the wing's rise, the rotors' axis
off up, the rates -, and over it the axis's tilt, the most any rate was, the heights, the share
of its passes a rotor stood at an end of its reach, what was struck. The rotors come to a
thrust asked inside their reach a pass on (`reach`), a rotor's own lag taken out of the ask.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import aerolisp, intents, quad  # noqa: E402
from tools.cores import aero  # noqa: E402

#: The ideal rotors: their top and idle, rad/s, what their speed changes by at the most,
#: rad/s^2, and their lag to a speed inside that, s - the race's (`tools/sim/quad_race.py`).
TOP_RAD_S, SPOOL_RAD_S2, LAG_S = 700.0, 2900.0, 0.02

#: A hover asked at a height, m: the upsets'.
HOVER = """(stream_inject (sequence_id 1) (blend_duration 0ms)
  (latent_targets (mode_collective_thrust 9.81) (mode_attitude_pitch 0deg)
                  (mode_attitude_roll 0deg) (mode_yaw_rate 0rad_s))
  (impedance (stiffness mode_attitude_pitch 5000) (stiffness mode_attitude_roll 5000)
             (damping global 80))
  (task_constraints (maintain_altitude %gm)))"""

#: The upsets: the rotors' axis this far off up, degrees, the frame turning about it this fast,
#: rad/s. It stands with its axis within STOOD_DEG of where it settles and every rate under
#: STOOD_RAD_S.
TILTS, TURNS, UPSET_M, STOOD_DEG, STOOD_RAD_S = (30.0, 90.0, 150.0), (0.0, 6.0, 9.0), 5.0, 3.0, 0.5


def symbols():
    """The frame's symbols a text may name: all its rotors give at their top, m/s^2."""
    return {'MAX_SYS_THRUST': 4.0 * quad.K_THRUST * TOP_RAD_S ** 2 / quad.MASS_KG}


def reach(speeds, dt, slew=SPOOL_RAD_S2, least=None, top=TOP_RAD_S):
    """(each rotor's thrust as its speed has it, at the least and the most it reaches `dt` on),
    N: its speed `slew` either way, inside its idle and its top."""
    k, least = quad.K_THRUST, quad.IDLE_RAD_S if least is None else least
    return ([k * w * w for w in speeds], [k * max(least, w - slew * dt) ** 2 for w in speeds],
            [k * min(top, w + slew * dt) ** 2 for w in speeds])


def tilt(turn):
    """(the nose's rise, the wing's, the rotors' axis off up), degrees, of the frame's axes."""
    return tuple(math.degrees(f(max(-1.0, min(1.0, float(x))))) for f, x in (
        (math.asin, turn[1][2]), (math.asin, turn[1][0]), (math.acos, turn[1][1])))


class Flight:

    """The frame on the floor - or `at` (x, m up, z), hovering -, its core and its four ideal
    rotors; `air` the flight's (`quad.air`), or still."""

    def __init__(self, air=None, at=None, things=()):
        self.sky, self.core, self.air = quad.Sky(things), aero.Core(), air
        self.w, self.clock, self.dt = [0.0] * 4, 0.0, quad.STEP_S
        if at is not None:
            self.sky.data.qpos[0:3] = (at[0], at[1] + quad.SKID_M + quad.FOOT_M, at[2])
            self.w = [quad.speed_for(quad.MASS_KG * quad.GRAVITY / 4.0)] * 4
            self.forward()

    def forward(self):
        """The world as its state was set by hand."""
        self.sky._mj.mj_forward(self.sky.model, self.sky.data)

    def step(self, payload):
        """One pass on `payload`: (the frame as read, the core's answer)."""
        dt, state = self.dt, self.sky.state()
        out = self.core.step(payload, aero.reading(state, *reach(self.w, dt), dt))
        gain = min(1.0, dt / LAG_S)
        for k in range(4):
            want = math.sqrt(max(0.0, out.thrust[k]) / quad.K_THRUST)
            more = (min(TOP_RAD_S, self.w[k] + (want - self.w[k]) / gain) - self.w[k]) * gain
            self.w[k] += max(-SPOOL_RAD_S2 * dt, min(SPOOL_RAD_S2 * dt, more))
        if self.air:
            quad.blown(self.air, dt)
        self.sky.step(self.w, dt, self.air)
        self.clock += dt
        return state, out


def flown(flight, payloads, seconds):
    """The payloads flown one after the other, each its seconds: a row each - {'id', 'ends':
    the frame after it, 'nose', 'wing', 'off': its tilt then, 'offs', 'heights': the least and
    the most over it, 'spin': the most any rate was, 'bound': the share of its passes a rotor
    stood at an end, 'passes': [(clock, frame, thrusts, bound)]}."""
    rows = []
    for payload, span in zip(payloads, seconds):
        passes = []
        for _ in range(int(round(span / flight.dt))):
            state, out = flight.step(payload)
            passes.append((flight.clock, state, list(out.thrust), out.bound))
        ends = flight.sky.state()
        nose, wing, off = tilt(ends['turn'])
        offs = [tilt(state['turn'])[2] for _t, state, _f, _b in passes]
        rows.append({'id': payload.sequence_id, 'ends': ends, 'nose': nose, 'wing': wing,
                     'off': off, 'offs': (min(offs), max(offs)),
                     'heights': (min(s['h'] for _t, s, _f, _b in passes),
                                 max(s['h'] for _t, s, _f, _b in passes)),
                     'spin': max(abs(float(x)) for _t, s, _f, _b in passes for x in s['spin']),
                     'bound': sum(1 for *_x, b in passes if b) / len(passes), 'passes': passes})
    return rows


def upset(off_deg, turning, seconds=4.0):
    """A hover asked of the frame UPSET_M up with its axis `off_deg` off up - nose down - and
    turning `turning` rad/s about it: (the seconds it stood from or None, its axis off up at
    the end, its lowest, what it struck)."""
    payload = aero.payload(aerolisp.compiled(HOVER % UPSET_M)[0])
    flight = Flight(at=(0.0, UPSET_M, 0.0))
    half = math.radians(off_deg) / 2.0
    flight.sky.data.qpos[3:7] = (math.cos(half), math.sin(half), 0.0, 0.0)
    flight.sky.data.qvel[3:6] = (0.0, turning, 0.0)
    flight.forward()
    passes = flown(flight, [payload], [seconds])[0]['passes']
    rest = tilt(passes[-1][1]['turn'])[2]
    stood = None
    for clock, state, _f, _b in passes:
        still = (abs(tilt(state['turn'])[2] - rest) < STOOD_DEG
                 and max(abs(float(x)) for x in state['spin']) < STOOD_RAD_S)
        stood = (stood if stood is not None else clock) if still else None
    return stood, rest, min(s['h'] for _t, s, _f, _b in passes), passes[-1][1]['hit']


def _show(row):
    ends = row['ends']
    print('%5d  h %5.2f m  v %6.2f %6.2f %6.2f m/s  nose %6.1f wing %6.1f off %5.1f deg  rates '
          '%6.2f %6.2f %6.2f | off %5.1f..%5.1f, a rate to %5.1f rad/s, h %5.2f..%5.2f, at an '
          'end %3.0f %%%s' % ((row['id'], ends['h']) + tuple(float(x) for x in ends['vel'])
                             + (row['nose'], row['wing'], row['off'])
                             + tuple(float(x) for x in ends['spin']) + row['offs']
                             + (row['spin'],) + row['heights']
                             + (100.0 * row['bound'],
                                '  STRUCK %s' % ends['hit'] if ends['hit'] else '')))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('text', nargs='?', help="a file of streams; none: the power loop's")
    parser.add_argument('--seconds', help='each stream\'s seconds, a comma between')
    parser.add_argument('--air', type=int, help="the tour's air of this seed; none: still")
    parser.add_argument('--hover', type=float, default=4.0,
                        help='the first stream flown again after the last, s; 0: not')
    parser.add_argument('--upsets', action='store_true', help='a hover asked from upsets')
    args = parser.parse_args(argv)
    if args.upsets:
        for off in TILTS:
            for turning in TURNS:
                stood, rest, low, hit = upset(off, turning)
                print('%5.0f deg off, %4.1f rad/s about its axis: %s, %4.1f deg off at the end, '
                      '%.2f m at the lowest%s' % (
                          off, turning, 'stands from %.2f s' % stood if stood else 'never stands',
                          rest, low, '  STRUCK %s' % hit if hit else ''))
        return 0
    text, seconds = intents.POWER_LOOP, list(intents.POWER_LOOP_S)
    if args.text:
        with open(args.text, encoding='utf-8') as f:
            text = f.read()
    payloads = [aero.payload(p) for p in aerolisp.compiled(text, symbols())]
    if args.seconds:
        seconds = [float(x) for x in args.seconds.split(',')]
    if args.hover:
        again = aero.payload(bytes(payloads[0]))
        again.sequence_id = max(p.sequence_id for p in payloads) + 1
        payloads, seconds = payloads + [again], seconds + [args.hover]
    for row in flown(Flight(air=None if args.air is None else quad.air(args.air)), payloads,
                     seconds):
        _show(row)
    return 0


if __name__ == '__main__':
    sys.exit(main())
