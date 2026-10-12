#!/usr/bin/env python3
"""The frame's core (aero/) through ctypes: its form, its share, its impedance in a vacuum.

No branch in its sources; the active set against every set of rotors held at an end; the
payload's stiffness and damping as the frame answers them where nothing else acts; a mode
dropped, a blend, the oscillator, the observers, a height's constraints, the rotors' reach."""
import glob
import io
import itertools
import math
import os
import platform
import random
import re
import shutil
import struct
import subprocess
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: What a pass may not hold: a jump the data decides.
BANNED = r'\b(if|else|switch|case|while|do|goto|break|continue)\b|\?|&&|\|\|'

#: In the machine's code, a flag set from the data: a float compare, a move or a set on a
#: condition - by the machine this host is. With none, a conditional jump's flags are a loop's
#: counter's.
FLAGGED = {
    'x86': r'\s(v?u?comis[sd]|v?cmp[sp][sd]|cmov[a-z]+|set[a-z]{1,3}|fu?comi?p?)\s',
    'arm': r'\s(vcmpe?\.\w+|fcmpe?|fccmpe?|fcsel|csel|cset[m]?|csinc|csinv|csneg)\s',
}
JUMPS = {'x86': r'\s(j(?!mp)[a-z]+)\s', 'arm': r'\s(b\.?(?:eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|hs|lo)|cbn?z|tbn?z)\s'}

#: A vacuum's pass, s: fine enough that the pass itself is no part of what is measured.
FINE = 0.0002


def _hover(k=5000.0, d=80.0, extra='', blend=0, sequence=1, pitch='0deg', roll='0deg',
           thrust='9.81'):
    """A payload: level - or `pitch`, `roll` - on the collective `thrust`, `k` and `d` on the
    tilt's modes, `extra` blocks after."""
    from machine import aerolisp
    from tools.cores import aero
    return aero.payload(aerolisp.compiled(
        '(stream_inject (sequence_id %d) (blend_duration %dms) (latent_targets '
        '(mode_collective_thrust %s) (mode_attitude_pitch %s) (mode_attitude_roll %s) '
        '(mode_yaw_rate 0rad_s)) (impedance (stiffness mode_attitude_pitch %g) '
        '(stiffness mode_attitude_roll %g) (damping global %g)) %s)'
        % (sequence, blend, thrust, pitch, roll, k, k, d, extra))[0])


class Vacuum:

    """The frame as a rigid body with nothing on it but its rotors' thrusts and gravity -
    and a push, m/s^2 in the world, and a twist, N m about its axes, where a test lays one.
    The thrusts asked are given a pass on; `slew`: no faster than a rotor's speed changes,
    rad/s^2."""

    def __init__(self, dt=FINE, slew=None, height=5.0):
        import numpy as np
        from machine import quad
        from tools.cores import aero
        self.np, self.quad, self.aero = np, quad, aero
        self.core, self.dt, self.slew = aero.Core(), dt, slew
        self.turn, self.spin, self.vel = np.eye(3), np.zeros(3), np.zeros(3)
        self.height, self.clock = height, 0.0
        self.given = [quad.MASS_KG * quad.GRAVITY / 4.0] * 4
        self.push, self.twist = np.zeros(3), np.zeros(3)

    def tilted(self, about, rad):
        """The frame turned `rad` about its own axis `about` (0 its wing, 2 its nose)."""
        self.turn = self.turn @ self._turned(about, rad)

    def _turned(self, about, rad):
        np = self.np
        c, s = math.cos(rad), math.sin(rad)
        a, b = (about + 1) % 3, (about + 2) % 3
        out = np.eye(3)
        out[a, a], out[a, b], out[b, a], out[b, b] = c, -s, s, c
        return out

    def state(self):
        return {'turn': self.turn, 'spin': self.spin, 'vel': self.vel, 'h': self.height}

    def step(self, payload):
        """One pass on `payload`: the core's answer."""
        np, quad, dt = self.np, self.quad, self.dt
        if self.slew is None:
            lo, hi = [0.0] * 4, [200.0] * 4
        else:
            w = [math.sqrt(f / quad.K_THRUST) for f in self.given]
            lo = [quad.K_THRUST * max(0.0, x - self.slew * dt) ** 2 for x in w]
            hi = [quad.K_THRUST * (x + self.slew * dt) ** 2 for x in w]
        out = self.core.step(payload, self.aero.reading(self.state(), self.given, lo, hi, dt))
        self.given = [float(f) for f in out.thrust]
        lift = sum(self.given)
        torque = np.array([
            -sum(f * z for f, (_x, z) in zip(self.given, quad.ROTOR_AT)),
            sum(f * s for f, s in zip(self.given, quad.SPIN)) * quad.K_DRAG / quad.K_THRUST,
            sum(f * x for f, (x, _z) in zip(self.given, quad.ROTOR_AT))]) + self.twist
        inertia = np.array(quad.INERTIA)
        self.spin = self.spin + dt * (torque - np.cross(self.spin, inertia * self.spin)) / inertia
        for about in range(3):
            self.turn = self.turn @ self._turned(about, self.spin[about] * dt)
        acc = lift / quad.MASS_KG * self.turn[:, 1] - np.array([0.0, quad.GRAVITY, 0.0]) + self.push
        self.vel = self.vel + dt * acc
        self.height += dt * self.vel[1]
        self.clock += dt
        return out

    def nose(self):
        """The nose's rise, rad."""
        return math.asin(max(-1.0, min(1.0, self.turn[1][2])))


def test_a_pass_has_no_branch(report):
    """aero/src holds no `if`, `else`, `switch`, `while`, `?`, `&&`, `||`, `break` or `goto`,
    its comments and strings aside, and builds under the firmware's warnings with none; and
    as this host's compiler makes it at the firmware's -O2, no flag is set from its data - no
    float compare, no move or set on a condition: what conditional jumps it has are its loops'
    ends."""
    from tools.cores import aero
    from tools.cores.build import FLAGS, OUT, build, find_cc
    here = os.path.join(os.path.dirname(aero.INCLUDES[0]), 'src')
    sources = sorted(glob.glob(os.path.join(here, '*.c')) + glob.glob(os.path.join(here, '*.h')))
    found = []
    for path in sources:
        with io.open(path, encoding='utf-8') as f:
            code = re.sub(r'"[^"\n]*"', '""', re.sub(r'/\*.*?\*/', ' ', f.read(), flags=re.S))
        found += ['%s: %s' % (os.path.basename(path), hit.group(0))
                  for hit in re.finditer(BANNED, code)]
    report.check('no jump the data decides in %d sources' % len(sources),
                 len(sources) == 4 and not found
                 and sorted(os.path.basename(p) for p in aero.SOURCES if 'harness' not in p)
                 == [os.path.basename(p) for p in sources if p.endswith('.c')],
                 ', '.join(found[:6]) or ' '.join(os.path.basename(p) for p in sources))
    cc = find_cc()
    _lib, warnings = build(cc, aero.SOURCES, aero.INCLUDES, 'aero_warned', list(aero.BUILT))
    report.check('built under the firmware\'s warnings with none', not warnings,
                 '; '.join(warnings[:2]) or '0 warnings')
    machine = 'arm' if platform.machine().lower() in ('arm64', 'aarch64') else 'x86'
    objdump = shutil.which('objdump') or os.path.join(os.path.dirname(cc or ''), 'objdump')
    flagged, jumps, size = [], 0, 0
    for path in (p for p in sources if p.endswith('.c')):
        obj = os.path.join(OUT, 'aero_%d_%s.o' % (os.getpid(), os.path.basename(path)[:-2]))
        subprocess.run([cc, '-c'] + FLAGS + ['-O2', '-fno-math-errno', '-I', aero.INCLUDES[0],
                                              path, '-o', obj], check=True)
        text = subprocess.run([objdump, '-d', obj], capture_output=True, text=True,
                              check=True).stdout
        os.remove(obj)
        flagged += ['%s: %s' % (os.path.basename(path), hit) for hit in
                    re.findall(FLAGGED[machine], text)]
        jumps += len(re.findall(JUMPS[machine], text))
        size += len(re.findall(r'^\s*[0-9a-f]+:\t', text, flags=re.M))
    report.check('in this host\'s code no flag is set from its data', not flagged and size > 1000,
                 ', '.join(sorted(set(flagged))[:6])
                 or '%d instructions, %d conditional jumps: its loops\' ends' % (size, jumps))


def _best(a, wanted, lo, hi):
    """(the least cost, its thrusts) over every set of rotors held at an end: each rotor free,
    at its least or at its most, the free ones' least squares kept where it lies inside."""
    import numpy as np
    least = (math.inf, None)
    for held in itertools.product((None, 0, 1), repeat=4):
        f = np.array([0.0 if h is None else (lo[i], hi[i])[h] for i, h in enumerate(held)])
        free = [i for i, h in enumerate(held) if h is None]
        if free:
            f[free] = np.linalg.lstsq(a[:, free], wanted - a @ f, rcond=None)[0]
            if any(f[i] < lo[i] - 1e-9 or f[i] > hi[i] + 1e-9 for i in free):
                continue
        cost = float(np.sum((a @ f - wanted) ** 2))
        if cost < least[0]:
            least = (cost, f)
    return least


def test_the_share_is_the_best_of_every_set(report):
    """`aero_share`: for wrenches and reaches drawn at random, its thrusts lie inside each
    rotor's reach and cost no more than the best of the 81 sets of rotors held at an end,
    weighed as it weighs; a wrench inside the reach is given whole; the rotors it says stand
    at an end do."""
    import numpy as np
    from machine import quad
    from tools.cores import aero
    core, dice = aero.Core(), random.Random(7)
    weigh = np.array(list(core.params.weigh))
    rows = np.array([[1.0] * 4, [-z for _x, z in quad.ROTOR_AT],
                     [s * quad.K_DRAG / quad.K_THRUST for s in quad.SPIN],
                     [x for x, _z in quad.ROTOR_AT]])
    a = weigh[:, None] * rows
    worst, outside, ends, whole, bound_any = 0.0, 0, 0, 0.0, 0
    for n in range(400):
        lo = [dice.uniform(0.0, 20.0) for _ in range(4)]
        hi = [low + dice.choice((0.0, dice.uniform(0.01, 2.0), dice.uniform(1.0, 40.0)))
              for low in lo]
        if n % 4 == 0:
            inside = np.array([dice.uniform(low, high) for low, high in zip(lo, hi)])
            want = rows @ inside
        else:
            want = np.array([dice.uniform(-20.0, 200.0), dice.uniform(-30.0, 30.0),
                             dice.uniform(-3.0, 3.0), dice.uniform(-30.0, 30.0)])
        f, bound = core.share(list(want), lo, hi)
        lo32 = [struct.unpack('f', struct.pack('f', x))[0] for x in lo]
        hi32 = [struct.unpack('f', struct.pack('f', x))[0] for x in hi]
        cost = float(np.sum((a @ np.array(f) - weigh * want) ** 2))
        best, _f = _best(a, weigh * want, lo32, hi32)
        worst = max(worst, (cost - best) / (1.0 + best))
        outside += any(x < low - 1e-9 or x > high + 1e-9 for x, low, high in zip(f, lo32, hi32))
        ends += any(bool(bound >> i & 1) != (f[i] <= lo32[i]) or
                    bool(bound >> (4 + i) & 1) != (f[i] >= hi32[i]) for i in range(4))
        bound_any += bool(bound)
        if n % 4 == 0:
            whole = max(whole, float(np.max(np.abs(rows @ np.array(f) - want))))
    report.check('400 wrenches: never past the best of the 81 sets held at an end',
                 worst < 1e-6, 'the most past it %.1e of its cost' % worst)
    report.check('every thrust inside its rotor\'s reach, the ends it names the ends it is at',
                 outside == 0 and ends == 0 and bound_any > 100,
                 '%d outside, %d misnamed, %d with a rotor at an end' % (outside, ends, bound_any))
    report.check('a wrench inside the reach is given whole', whole < 1e-4,
                 '%.1e N or N m off at the most' % whole)


def test_the_payloads_impedance_in_a_vacuum(report):
    """A frame let go a little off its attitude with nothing on it but its rotors: it comes
    back as its payload's stiffness and damping say - theta'' = -k theta - d theta', its
    period and its overshoot's the closed form's."""
    for k, d in ((5000.0, 80.0), (1500.0, 25.0), (400.0, 30.0), (8000.0, 180.0)):
        frame, payload = Vacuum(), _hover(k, d)
        off = math.radians(0.5)
        frame.tilted(0, -off)                       # nose up
        zeta, wn = d / (2.0 * math.sqrt(k)), math.sqrt(k)
        worst, span = 0.0, 4.0 * math.tau / wn
        for _ in range(int(span / frame.dt)):
            frame.step(payload)
            t = frame.clock
            if zeta < 1.0:
                wd = wn * math.sqrt(1.0 - zeta * zeta)
                ideal = off * math.exp(-zeta * wn * t) * (
                    math.cos(wd * t) + zeta * wn / wd * math.sin(wd * t))
            else:
                a, b = (-zeta * wn + wn * math.sqrt(zeta * zeta - 1.0),
                        -zeta * wn - wn * math.sqrt(zeta * zeta - 1.0))
                ideal = off * (a * math.exp(b * t) - b * math.exp(a * t)) / (a - b)
            worst = max(worst, abs(frame.nose() - ideal))
        report.check('k %g, d %g: the closed form\'s return, damped %.2f at %.0f rad/s'
                     % (k, d, zeta, wn), worst < 0.02 * off,
                     'the most off it %.2f %% of the miss' % (100.0 * worst / off))


def test_a_dropped_mode_is_masked(report):
    """A target of NaN drops its mode: no NaN reaches a thrust, and the torque about that
    mode's axis is none whatever the frame's attitude about it - the mode beside it answered
    the way that closes its miss from every such attitude, on its back too."""
    from machine import aerolisp
    from tools.cores import aero
    payload = aero.payload(aerolisp.compiled(
        '(stream_inject (sequence_id 1) (latent_targets (mode_collective_thrust 9.81) '
        '(mode_attitude_pitch ignore) (mode_attitude_roll 0deg) (mode_yaw_rate 0rad_s)) '
        '(impedance (stiffness global 5000) (damping global 80)))')[0])
    asked = []
    for pitch in (0.0, 0.4, 1.3, 2.9, -2.0):
        frame = Vacuum()
        frame.tilted(0, pitch)
        frame.tilted(2, 0.05)
        out = frame.step(payload)
        asked.append((list(out.asked), list(out.thrust), list(out.miss)))
    finite = all(math.isfinite(x) for row in asked for part in row for x in part)
    report.check('a NaN target reaches no thrust', finite and math.isnan(payload.target[1]),
                 '%s N' % ' '.join('%.2f' % x for x in asked[1][1]))
    report.check('about the dropped mode\'s axis no torque, wherever the frame stands about it',
                 max(abs(row[0][1]) for row in asked) < 1e-6,
                 '%s N m' % ' '.join('%.1e' % row[0][1] for row in asked))
    report.check('the mode beside it answered the closing way from every such attitude',
                 all(row[0][3] < -0.1 for row in asked),
                 '%s N m about the nose' % ' '.join('%.3f' % row[0][3] for row in asked))
    frame, away = Vacuum(), _hover(extra='', sequence=2)
    frame.tilted(0, 0.3)
    report.check('tracked, the same miss is answered',
                 abs(frame.step(away).asked[1]) > 0.1)


def test_a_payload_is_blended_in_and_swung(report):
    """A payload's rows come to its values over its blend, a line - its attitude along the
    sphere, the axis asked half way between the two half its blend in; a mode tracked from a
    dropped one takes its target whole; the oscillator swings a target its amplitude either
    way at its frequency, a sine from nothing where it began."""
    from machine import aerolisp
    from tools.cores import aero
    frame = Vacuum(dt=0.002)
    first = _hover(1000.0, 60.0)
    for _ in range(5):
        frame.step(first)
    second = _hover(3000.0, 20.0, blend=200, sequence=2, pitch='-20deg', thrust='12')
    seen = []
    for n in range(150):
        out = frame.step(second)
        targets, _wound = frame.core.row(aero.TARGET)
        stiff, _w = frame.core.row(aero.STIFF)
        axis = frame.turn @ list(out.axis)
        seen.append((math.atan2(-axis[2], axis[1]), stiff[1], targets[0]))
    half, full = seen[49], seen[-1]
    report.check('half its blend in, half way: the axis asked, its stiffness, the thrust',
                 abs(half[0] - math.radians(-10.0)) < 5e-3 and abs(half[1] - 2000.0) < 1.0
                 and abs(half[2] - 10.905) < 1e-2, '%.2f deg, %.0f, %.2f' % (
                     math.degrees(half[0]), half[1], half[2]))
    report.check('its blend over, its own', abs(full[0] - math.radians(-20.0)) < 2e-3
                 and full[1] == 3000.0, '%.2f deg, %.0f' % (math.degrees(full[0]), full[1]))
    dropped = aero.payload(aerolisp.compiled(
        '(stream_inject (sequence_id 3) (blend_duration 100ms) (latent_targets '
        '(mode_collective_thrust 9.81) (mode_attitude_pitch ignore)))')[0])
    for _ in range(100):
        frame.step(dropped)
    back = _hover(1000.0, 60.0, blend=100, sequence=4, pitch='30deg')
    frame.step(back)
    targets, _w = frame.core.row(aero.TARGET)
    weights, _w = frame.core.row(aero.WEIGHT)
    report.check('tracked from a dropped one: its target whole at once, its weight blended',
                 abs(targets[1] - math.radians(30.0)) < 1e-6 and 0.0 < weights[1] < 0.05,
                 '%.1f deg at %.2f' % (math.degrees(targets[1]), weights[1]))
    swung = aero.payload(aerolisp.compiled(
        '(stream_inject (sequence_id 5) (latent_targets (mode_collective_thrust 9.81) '
        '(mode_attitude_pitch 0deg) (mode_attitude_roll 0deg) (mode_yaw_rate 0rad_s)) '
        '(impedance (stiffness global 1000) (damping global 60)) (cpg_config (enable true) '
        '(frequency 2.0Hz) (amplitude mode_attitude_roll 35deg)))')[0])
    still, asked = Vacuum(dt=0.002), []
    for _ in range(800):
        still.step(swung)
        asked.append(math.degrees(still.core.row(aero.ASK)[0][2]))
    crossings = [n for n in range(1, len(asked)) if asked[n - 1] < 0.0 <= asked[n]]
    period = (crossings[-1] - crossings[0]) * 0.002 / (len(crossings) - 1)
    report.check('the oscillator swings the roll its amplitude either way, from nothing',
                 abs(max(asked) - 35.0) < 0.1 and abs(min(asked) + 35.0) < 0.1
                 and abs(asked[0]) < 1.0,
                 '%.2f to %.2f deg, %.2f at its first pass' % (min(asked), max(asked), asked[0]))
    report.check('at its frequency', abs(period - 0.5) < 1e-3, 'a turn in %.4f s' % period)


def test_the_observers_take_what_pushes(report):
    """A push across the rotors' axis and a twist about the wing, neither told: the observers
    come to hold them, the frame leans against the push and keeps along the floor the speed it
    gathered while they learnt it - the push over half the observer's bandwidth -, its height's
    own with it, and the twist leaves its attitude where it was asked."""
    from tools.cores import aero
    frame, payload = Vacuum(dt=0.002), _hover()
    frame.push[0], frame.twist[0] = 1.5, 0.4
    for _ in range(int(3.0 / frame.dt)):
        out = frame.step(payload)
    report.check('the push held by the speed\'s observer, the twist by the rates\'',
                 abs(out.pushed[0] - 1.5) < 0.02 and abs(out.turned[0] - 0.4 / 0.045) < 0.1,
                 '%.3f m/s^2 of 1.5, %.2f rad/s^2 of %.2f' % (out.pushed[0], out.turned[0],
                                                             0.4 / 0.045))
    speed = math.hypot(frame.vel[0], frame.vel[2])
    lean = math.degrees(math.acos(max(-1.0, min(1.0, frame.turn[1][1]))))
    report.check('the frame leans against the push and keeps the speed its learning cost',
                 abs(lean - math.degrees(math.atan2(1.5, 9.81))) < 0.05
                 and speed < 1.2 * 1.5 / (0.5 * aero.WO_PULL),
                 '%.2f deg of %.2f, %.3f m/s' % (lean, math.degrees(math.atan2(1.5, 9.81)), speed))
    report.check('its height kept under the lean', abs(frame.vel[1]) < 0.01,
                 '%.4f m/s up' % frame.vel[1])


def test_a_height_is_held_and_capped(report):
    """`maintain_altitude`: the frame comes to the height from under it and from over it and
    stands; `limit_vertical_ascent`: asked twice its weight it rises that far from where the
    payload came and no further."""
    for began in (0.5, 4.0):
        frame = Vacuum(dt=0.002, height=began)
        payload = _hover(extra='(task_constraints (maintain_altitude 2.0m))')
        heights = []
        for _ in range(int(4.0 / frame.dt)):
            frame.step(payload)
            heights.append(frame.height)
        report.check('held from %.1f m: at 2.0 m and still' % began,
                     abs(frame.height - 2.0) < 0.005 and abs(frame.vel[1]) < 0.005
                     and (max(heights) < 2.02 if began < 2.0 else min(heights) > 1.98),
                     '%.3f m, %.4f m/s, %.2f..%.2f m on the way' % (
                         frame.height, frame.vel[1], min(heights), max(heights)))
    frame = Vacuum(dt=0.002, height=3.0)
    payload = _hover(thrust='19.62', extra='(task_constraints (limit_vertical_ascent 2.5m))')
    most = 0.0
    for _ in range(int(5.0 / frame.dt)):
        frame.step(payload)
        most = max(most, frame.height)
    report.check('asked twice its weight under a ceiling 2.5 m up: there, and no higher',
                 abs(frame.height - 5.5) < 0.01 and most < 5.52,
                 '%.3f m, %.3f at the most' % (frame.height, most))


def test_a_miss_past_the_reach_is_closed_as_fast_as_it_stops(report):
    """On rotors whose speed changes 2 900 rad/s^2 at the most a tilt of 45 degrees is asked
    at a stiffness the reach renders half a degree of: the frame is there in under twice the
    time a bounded jerk takes for it, and passes it by under a degree; at a stiffness the
    reach renders whole it is the payload's own return."""
    from machine import quad
    step = math.radians(45.0)
    frame = Vacuum(dt=0.002, slew=2900.0)
    hover = _hover(5000.0, 80.0)
    for _ in range(50):
        frame.step(hover)
    asked = _hover(5000.0, 80.0, sequence=2, pitch='45deg')
    arrived, most = None, 0.0
    for _ in range(int(1.5 / frame.dt)):
        frame.step(asked)
        most = max(most, frame.nose())
        if arrived is None and abs(frame.nose() - step) < math.radians(1.0):
            arrived = frame.clock - 50 * frame.dt
    w = math.sqrt(quad.MASS_KG * quad.GRAVITY / 4.0 / quad.K_THRUST)
    jerk = 0.5 * 4.0 * quad.ARM_M * 2.0 * quad.K_THRUST * w * 2900.0 / quad.INERTIA[0]
    best = (32.0 * step / jerk) ** (1.0 / 3.0)
    report.check('45 degrees at k 5000: there in under twice a bounded jerk\'s time',
                 arrived is not None and arrived < 2.0 * best,
                 '%.3f s, the jerk\'s %.3f at %.0f rad/s^3' % (arrived or math.nan, best, jerk))
    report.check('passed by under a degree, and held', most - step < math.radians(1.0)
                 and abs(frame.nose() - step) < math.radians(0.1),
                 '%.2f deg past, %.3f off at the end' % (math.degrees(most - step),
                                                        math.degrees(frame.nose() - step)))


ROSTER = (test_a_pass_has_no_branch, test_the_share_is_the_best_of_every_set,
          test_the_payloads_impedance_in_a_vacuum, test_a_dropped_mode_is_masked,
          test_a_payload_is_blended_in_and_swung, test_the_observers_take_what_pushes,
          test_a_height_is_held_and_capped,
          test_a_miss_past_the_reach_is_closed_as_fast_as_it_stops)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
