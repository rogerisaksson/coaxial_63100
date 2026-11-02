"""Her moves found as puppets: sensed phases on the servos played in a scene in memory and measured.

The method that found her way up the bed (the heel strokes open loop, the lazy crab regulated):

    python tools/sim/puppet.py crab --scene bed_foot_behind --say lie_bed --trunk 55 --gain 30
    python tools/sim/puppet.py heel --plant 0.18,0.22 --bridge 0.06,0.10 --reach 0.45      # a grid: the gain a stroke
    python tools/sim/puppet.py crab --gcode crab_bed            # what she was told, as gcode/crab_bed.gcode (open loop)
    python tools/sim/puppet.py take lying_down_from_sitting --trunk 55 --slow 1.5 --gcode danced_lie   # a take danced

A puppet is a session placed in `--scene` and told `--say`; once her text's program ends (`--from` s without one) a
Regime drives every joint on the boards' servos as `joints.played` does. A regime is phases, each a method
`p_<name>(el, sense)` returning the next phase's name or None, with `set` (a pose), `ramp` (a joint at a rate),
`hold` (joints ramped by an angle's error: the trunk by the anchored hips) and `sense`: loads by body, places along
the furniture's axis, the trunk's elevation. `reach` solves a two-link limb (thigh/shank, upper arm/forearm) for an
end point in the sagittal plane, so poses come from her lengths; `Beats` plays a (name, s, pose) program as a
regime; `search` plays a stroke family over a grid and tabulates the gain a stroke, the slip across and the fall;
`count` is her text's rule (`maneuver._program`). `Danced` plays a take's program (mocap -> `tools/sim/gcode.py` ->
gcode/<take>.gcode) in a situation it was not recorded in, the trunk held by hip offsets over the take's own targets
and its pace scaled: lying_down_from_sitting (recorded on the bed's side) at its foot end rolled her off the side raw,
off the end held at full pace, and at 2/3 pace held 55 deg lay her flat on the bed at -0.49 (the shipped pull and
lie's landing); the danced result written back is a program her text binds. Found so far: the heel stroke (plant 0.22 m, bridge 0.10, reach
0.45: 0.14-0.16 m a stroke supine) and the crab (the trunk held 55-60 deg by the hips while the hands bear behind,
one leg at a time over the edge, the heels set by their felt load, a push sized to the mark, the lie with the heels
lifted: no fall, flat within 8 deg, +-0.15 m of the mark); every open-loop crab fell back or pitched off the end.
"""
import argparse
import math
import os
import sys
from typing import cast

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap
from machine.body import figure
from machine.process import joints
from machine.senses import edge
from machine.texts import gcode

J = tuple(figure.JOINTS)
JI = {j: i for i, j in enumerate(J)}
KP, KD = np.asarray(joints.KP), np.asarray(joints.KD)
SIDES = ('left', 'right')
ARMS = ('arm', 'hand', 'finger')
FEET = ('foot', 'toes')


def both(**deg):
    """A pose's joints, both sides."""
    return joints._both(**deg)


def side(s, **deg):
    """A pose's joints of side `s`."""
    return {s + '_' + k: v for k, v in deg.items()}


def reach(x, y, a=figure.THIGH, b=figure.SHANK):
    """(elevation, bend) deg of a two-link limb whose end sits `x` ahead and `y` up of its root: the first link
    `a` long raised `elevation` above the x axis, the second `b` bent `bend` back (0 straight, her knee's sign);
    the bend-up branch (the knee over the heel). None out of reach."""
    r = math.hypot(x, y)
    if not (abs(a - b) < r < a + b):
        return None
    inner = math.acos(max(-1.0, min(1.0, (a * a + b * b - r * r) / (2.0 * a * b))))
    elevation = math.atan2(y, x) + math.acos(max(-1.0, min(1.0, (a * a + r * r - b * b) / (2.0 * a * r))))
    return math.degrees(elevation), 180.0 - math.degrees(inner)


def end(elevation, bend, a=figure.THIGH, b=figure.SHANK):
    """(x ahead, y up) of the limb's end at (`elevation`, `bend`) deg, `reach`'s inverse."""
    e, k = math.radians(elevation), math.radians(bend)
    return a * math.cos(e) + b * math.cos(e - k), a * math.sin(e) + b * math.sin(e - k)


def count(deep, landed, short, stroke):
    """How many strokes bring her middle `short` m short of the furniture's middle from `landed` m in, `stroke` m
    each, by the depth her eyes saw (`maneuver._program`)."""
    return max(0, math.floor((0.5 * deep - landed - short) / stroke + 0.5))


class Scene:
    """What the puppet senses of the world: the furniture `prop` (its biggest geom's middle, axis and half length),
    her bodies' places along/across/up it, the loads she bears by body."""

    def __init__(self, world, prop):
        self.world, self.m, self.d = world, world.model, world.data
        m, d = self.m, self.d
        self.bed = m.body(prop).id
        frame = d.xmat[self.bed].reshape(3, 3)
        self.axis, self.across = frame[:, 2].copy(), frame[:, 0].copy()
        big = max((k for k in range(m.ngeom) if m.geom_bodyid[k] == self.bed), key=lambda k: m.geom_size[k].max())
        self.centre, self.half = d.geom_xpos[big].copy(), float(m.geom_size[big].max())
        self.ids = {m.body(i).name: i for i in range(m.nbody)}
        self.hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)], int)
        self.force = np.zeros(6)

    def along(self, body):
        return float((self.d.xpos[self.ids[body]] - self.centre) @ self.axis)

    def aside(self, body):
        return float((self.d.xpos[self.ids[body]] - self.centre) @ self.across)

    def up(self, body):
        return float(self.d.xpos[self.ids[body]][1])

    def trunk(self):
        """The trunk's elevation deg off flat, pelvis to torso."""
        v = self.d.xpos[self.ids['torso']] - self.d.xpos[self.ids['pelvis']]
        return math.degrees(math.asin(abs(float(v[1] / np.linalg.norm(v)))))

    def loads(self):
        """{her body: N} of what she bears on the furniture now."""
        m, d, out = self.m, self.d, {}
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (m.body_rootid[a] == self.bed and self.hers[b]) or (m.body_rootid[b] == self.bed and self.hers[a]):
                mine = b if self.hers[b] else a
                self.world._mj.mj_contactForce(m, d, i, self.force)
                out[m.body(mine).name] = out.get(m.body(mine).name, 0.0) + abs(float(self.force[0]))
        return out

    @staticmethod
    def load(con, *keys):
        return sum(v for k, v in con.items() if any(w in k for w in keys))

    def sense(self):
        con = self.loads()
        return {'t': None, 'loads': con, 'trunk': self.trunk(), 'pelvis': self.along('pelvis'),
                'hands': self.load(con, *ARMS), 'left_foot': self.load(con, 'left_foot', 'left_toes'),
                'right_foot': self.load(con, 'right_foot', 'right_toes')}


class Regime:
    """Phases on the servos: `cmd` deg every joint, `phase` the one running, `p_<phase>(el, sense)` its step."""
    first = 'start'

    def __init__(self, **params):
        self.params = params
        self.cmd, self.t_last, self.on = np.zeros(len(J)), 0.0, False
        self.phase, self.since, self.ended = self.first, 0.0, False
        self.record = []

    def begin(self, g):
        self.cmd = np.degrees(g['sense'][edge.Q]).copy()
        self.t_last, self.since, self.on = g['t'], g['t'], True

    def go(self, phase, t):
        self.phase, self.since = phase, t

    def label(self):
        """What the rows call this pass: the phase."""
        return self.phase

    def set(self, **deg):
        for j, v in deg.items():
            self.cmd[JI[j]] = v

    def ramp(self, j, goal, rate, dt):
        k, step = JI[j], rate * dt
        self.cmd[k] += max(-step, min(step, goal - self.cmd[k]))

    def hold(self, names, error, gain, dt, low=-115.0, high=-35.0):
        """`names` ramped by `error` deg at `gain` deg/s a deg: an angle too low flexes them (more negative)."""
        for j in names:
            k = JI[j]
            self.cmd[k] = max(low, min(high, self.cmd[k] - gain * error * dt))

    def tick(self, g, scene):
        dt = max(1e-4, min(0.05, float(g['t'] - self.t_last)))
        self.t_last = g['t']
        sense = scene.sense()
        sense['t'], sense['dt'] = g['t'], dt
        nxt = getattr(self, 'p_' + self.phase)(g['t'] - self.since, sense)
        if nxt == 'end':
            self.ended = True
        elif nxt:
            self.go(nxt, g['t'])
        self.record.append((g['t'], self.cmd.copy()))


class Beats(Regime):
    """A program of (name, s, {joint: deg}) beats as a regime (`joints.pose`), the phase its beat's name."""
    first = 'play'

    def __init__(self, beats, **params):
        super().__init__(**params)
        self.beats, self.length = tuple(beats), sum(b[1] for b in beats)

    def p_play(self, el, sense):
        if el >= self.length:
            return 'end'
        self.cmd = joints.pose(self.beats, self.pose0, el)
        t = 0.0
        for name, seconds, _goals in self.beats:
            t += seconds
            if el < t:
                self.beat = name
                break
        return None

    def label(self):
        return self.beat

    def begin(self, g):
        super().begin(g)
        self.pose0, self.beat = self.cmd.copy(), self.beats[0][0]


class Danced(Beats):
    """A take's beats danced in their situation: their pace `slow` times longer, the hips offset to hold the trunk at
    `trunk` deg (None: as the take has it; the offset clamped +-40 deg, so a lie the take asks still comes), the last
    pose held `tail` s."""

    def __init__(self, beats, trunk=None, slow=1.0, gain=30.0, tail=1.0):
        super().__init__([(n, s * slow, g) for n, s, g in beats], trunk=trunk, slow=slow, gain=gain, tail=tail)
        self.offset = np.zeros(len(J))

    def p_play(self, el, sense):
        if el >= self.length + self.params['tail']:
            return 'end'
        if self.params['trunk'] is not None:
            err = self.params['trunk'] - sense['trunk']
            for s in SIDES:
                k = JI[s + '_hip']
                self.offset[k] = max(-40.0, min(40.0, self.offset[k] - self.params['gain'] * err * sense['dt']))
        self.cmd = joints.pose(self.beats, self.pose0, min(el, self.length)) + self.offset
        t, self.beat = 0.0, 'held'
        for name, seconds, _goals in self.beats:
            t += seconds
            if el < t:
                self.beat = name
                break
        return None


class Crab(Regime):
    """The lazy crab from her sit at the furniture's end: the hands behind, the trunk held `trunk` deg by the
    anchored hips (`gain`), a leg at a time up and over the edge, the heels set by their felt load, the push sized
    to `mark` m along, the lie with the heels lifted."""
    first = 'sit'

    def __init__(self, trunk=55.0, gain=30.0, mark=-0.65, up_hip=-103.0, fold_knee=160.0, hands=-45.0, push_knee=100.0,
                 **params):
        super().__init__(trunk=trunk, gain=gain, mark=mark, up_hip=up_hip, fold_knee=fold_knee, hands=hands,
                         push_knee=push_knee, **params)
        self.strokes, self.retry = 0, {s: 0 for s in SIDES}
        self.end_at = None

    def trunk(self, legs, sense):
        self.hold([s + '_hip' for s in legs], self.params['trunk'] - sense['trunk'], self.params['gain'], sense['dt'])

    def p_sit(self, el, sense):
        self.set(**both(hip=-80.0, shoulder=self.params['hands'], elbow=60.0))
        return 'hands' if el > 0.35 else None

    def p_hands(self, el, sense):
        self.set(**both(elbow=0.0, hip=-70.0))
        return 'lean' if el > 0.3 else None

    def p_lean(self, el, sense):
        self.trunk(SIDES, sense)
        return 'lift_left' if el > 0.4 and abs(self.params['trunk'] - sense['trunk']) < 6.0 else None

    def _leg(self, phase, el, sense):
        step, s = phase.split('_')
        other = 'right' if s == 'left' else 'left'
        p = self.params
        if step == 'lift':
            self.set(**side(s, hip=p['up_hip'], knee=30.0))
            self.trunk((other,), sense)
            return 'fold_' + s if el > 0.5 else None
        if step == 'fold':
            self.set(**side(s, knee=p['fold_knee'], hip_roll=0.0, hip_yaw=0.0))
            self.trunk((other,), sense)
            return 'set_' + s if el > 0.35 else None
        self.ramp(s + '_hip', -80.0, 40.0, sense['dt'])
        self.trunk((other,), sense)
        if sense[s + '_foot'] >= 40.0 or el > 1.0:
            if self.end_at is not None and self.scene.along(s + '_foot') < self.end_at + 0.05 and self.retry[s] < 2:
                self.retry[s] += 1
                return 'lift_' + s
            return 'lift_right' if s == 'left' else 'push'
        return None

    def __getattr__(self, name):
        if name.startswith('p_') and name[2:].split('_')[0] in ('lift', 'fold', 'set') and name.count('_') == 2:
            return lambda el, sense: self._leg(name[2:], el, sense)
        raise AttributeError(name)

    def p_push(self, el, sense):
        for s in SIDES:
            self.ramp(s + '_knee', self.params['push_knee'], 100.0, sense['dt'])
        self.trunk(SIDES, sense)
        done = sense['pelvis'] >= self.params['mark'] or self.cmd[JI['left_knee']] <= self.params['push_knee']
        return 'setdown' if done or el > 0.8 else None

    def p_setdown(self, el, sense):
        self.set(**both(knee=min(135.0, float(self.cmd[JI['left_knee']]) + 15.0)))
        self.trunk(SIDES, sense)
        if el > 0.2:
            self.strokes += 1
            if sense['pelvis'] >= self.params['mark'] or self.strokes >= 6:
                return 'lie'
            return 'rehand' if sense['hands'] < 25.0 else 'draw'
        return None

    def p_rehand(self, el, sense):
        self.set(**both(shoulder=self.params['hands'] - 5.0, elbow=60.0))
        self.trunk(SIDES, sense)
        return 'replant' if el > 0.25 else None

    def p_replant(self, el, sense):
        self.set(**both(elbow=0.0))
        self.trunk(SIDES, sense)
        return 'draw' if el > 0.25 else None

    def p_draw(self, el, sense):
        self.set(**both(knee=self.params['fold_knee']))
        for s in SIDES:
            self.ramp(s + '_hip', -112.0, 60.0, sense['dt'])
        return 'plant' if el > 0.3 else None

    def p_plant(self, el, sense):
        for s in SIDES:
            self.ramp(s + '_hip', -80.0, 40.0, sense['dt'])
        return 'push' if (sense['left_foot'] >= 40.0 and sense['right_foot'] >= 40.0) or el > 0.8 else None

    def p_lie(self, el, sense):
        self.set(**both(knee=self.params['fold_knee'], hip=-115.0))
        return 'recline' if el > 0.4 else None

    def p_recline(self, el, sense):
        for s in SIDES:
            self.ramp(s + '_hip', -40.0, 75.0, sense['dt'])
            self.ramp(s + '_shoulder', 0.0, 45.0, sense['dt'])
            self.ramp(s + '_elbow', 5.0, 60.0, sense['dt'])
        return 'straight' if el > 1.0 else None

    def p_straight(self, el, sense):
        for s in SIDES:
            self.ramp(s + '_hip', -3.0, 40.0, sense['dt'])
            self.ramp(s + '_knee', 6.0, 150.0, sense['dt'])
        self.set(spine=0.0, waist=0.0, neck=0.0)
        return 'end' if el > 1.2 else None


def heel(plant=0.22, bridge=0.10, reach_m=0.45, hip=0.055):
    """The supine heel stroke's beats from her figure: the heels planted `plant` m ahead of the hips `hip` m
    under them, the pelvis bridged `bridge` m, the heels pushed to `reach_m`, the pelvis set down; None where a
    pose is out of her legs' reach."""
    asked = [reach(plant, -hip), reach(plant + 0.04, -hip - bridge), reach(reach_m, -hip - bridge), reach(reach_m, -hip)]
    poses = [p for p in asked if p is not None]
    if len(poses) < len(asked):
        return None
    names, seconds = ('plant', 'bridge', 'push', 'set'), (0.4, 0.25, 0.3, 0.2)
    extra = {'plant': {'hip_roll': 0.0, 'hip_yaw': 0.0}}
    return tuple((n, s, both(hip=-e, knee=k, **extra.get(n, {}))) for n, s, (e, k) in zip(names, seconds, poses))


class only_pull:
    """Her `bed_pull` program cut to its first beat (the text's pull) while a regime takes over after it."""

    def __enter__(self):
        programs = cast(dict, joints.PROGRAMS)
        self.keep, self.strokes = programs['bed_pull'], joints.STROKES.pop('bed_pull', None)
        programs['bed_pull'] = (self.keep[0],) if self.keep and self.keep[0][0] == 'pull' else ()
        return self

    def __exit__(self, *exc):
        cast(dict, joints.PROGRAMS)['bed_pull'] = self.keep
        if self.strokes is not None:
            joints.STROKES['bed_pull'] = self.strokes


def play(scene, say, regime, prop='prop_bed', start=None, settle=0.1, until=30.0, rows=None, texts=None):
    """`regime` played on her in `scene` told `say`: from `settle` s after her text's program ends (or from
    `start` s; taken over at the end itself she rolled off the crab), every joint hers on the servos; rows (t, phase, pelvis along/across/up, trunk, hands N, feet N) a phase change and
    each 0.25 s, until `until` s or the regime ends or she falls."""
    from machine.process import worker
    from machine.world import physics
    from tools.sim import scenarios
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session(texts=texts) if texts else worker.session()
    scenarios.placed(s, scene)
    g, world = s['g'], s['g']['world']
    sc = Scene(world, prop)
    regime.scene, regime.end_at = sc, -sc.half
    rows = [] if rows is None else rows
    played = joints.played

    def puppet(gg, w):
        if gg.get('played') is not None:
            regime.armed = gg['t'] + settle  # her text's program plays: the regime follows its end, settled
        if gg.get('played') is not None or regime.ended:
            return played(gg, w)
        if not regime.on:
            due = gg['t'] >= (start if start is not None else getattr(regime, 'armed', math.inf))
            if not due:
                return played(gg, w)
            regime.begin(gg)
            w.gains[:, 1] = KD
        regime.tick(gg, sc)
        idx = np.asarray(gg['joints'], int)
        edge.drive(w, idx, target=np.radians(regime.cmd)[idx], feed=np.zeros(len(idx)), kp=KP[idx],
                   limit=gg['clamp'][idx])

    joints.played = puppet
    try:
        while g['t'] < 1.0:
            worker.stepped(s)
        worker.apply(s, {'say': say})
        last, nxt = None, 0.0
        while g['t'] < until and s['epoch'] == 0 and not regime.ended:
            worker.stepped(s)
            tag = regime.label() if regime.on else 'law'
            if tag != last or g['t'] >= nxt:
                con = sc.loads()
                rows.append({'t': round(g['t'], 3), 'phase': tag, 'along': sc.along('pelvis'), 'across': sc.aside('pelvis'),
                             'up': sc.up('pelvis'), 'trunk': sc.trunk(), 'hands': sc.load(con, *ARMS),
                             'feet': sc.load(con, *FEET), 'fallen': s['fallen_at']})
                last, nxt = tag, g['t'] + 0.25
    finally:
        joints.played = played
    return rows, sc, s


def gains(rows, beat='plant'):
    """The pelvis's along at each `beat` row's start: the strokes' gains."""
    at = [r['along'] for i, r in enumerate(rows) if r['phase'] == beat and (i == 0 or rows[i - 1]['phase'] != beat)]
    return [round(b - a, 3) for a, b in zip(at, at[1:])]


def search(scene, say, family, grid, strokes=3, prefix=None):
    """`family(**params)` beats played `strokes` times after `prefix` beats (her shipped program up to its strokes
    by default) in `scene` told `say`, over `grid` {param: [values]}: a table of the gain a stroke, the slip across,
    the end and the fall for each combination."""
    import itertools
    shipped = joints.PROGRAMS['bed_pull']
    prefix = tuple(prefix if prefix is not None else [b for b in shipped if b[0] in ('pull', 'lie', 'up', 'fold', 'lower', 'shove', 'stretch', 'still')])
    tail = tuple(b for b in shipped if b[0] == 'straight')
    out = []
    for combo in itertools.product(*grid.values()):
        params = dict(zip(grid, combo))
        beats = family(**params)
        if beats is None:
            out.append(dict(params, gains=None, note='out of reach'))
            continue
        with only_pull():
            regime = Beats(prefix[1:] + beats * strokes + tail)
            rows, sc, s = play(scene, say, regime)
        out.append(dict(params, gains=gains(rows), across=round(rows[-1]['across'] - rows[0]['across'], 3),
                        end=round(rows[-1]['along'], 3), fallen=rows[-1]['fallen']))
    return out


def written(name, regime, hz=10.0, out=gcode.DIR):
    """What `regime` told her, as her G-code `name` under `out` (a move a 1/`hz` s), the file's path."""
    times, values, last = [], [], -1.0
    for t, cmd in regime.record:
        if t - last >= 1.0 / hz:
            times.append(t - regime.record[0][0])
            values.append(cmd)
            last = t
    header = {'humanoid': 'gcode 1', 'rig': 'hers', 'take': 'puppet/' + name, 'hz': '%.3f' % hz}
    path = os.path.join(out, name + '.gcode')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(gcode.text(header, times, J, values))
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    ap.add_argument('regime', choices=('crab', 'heel', 'take'))
    ap.add_argument('take', nargs='?', default='lying_down_from_sitting', help="the take's program (gcode/<take>.gcode)")
    ap.add_argument('--scene', default='bed_foot_behind')
    ap.add_argument('--say', default='lie_bed')
    ap.add_argument('--trunk', type=float, default=55.0, help='the trunk held, deg off flat; 0 or less: not held')
    ap.add_argument('--slow', type=float, default=1.0, help="a take's pace, times longer")
    ap.add_argument('--gain', type=float, default=30.0)
    ap.add_argument('--mark', type=float, default=-0.65)
    ap.add_argument('--plant', default='0.22')
    ap.add_argument('--bridge', default='0.10')
    ap.add_argument('--reach', default='0.45')
    ap.add_argument('--strokes', type=int, default=3)
    ap.add_argument('--gcode', default=None)
    ap.add_argument('--until', type=float, default=30.0)
    ap.add_argument('--start', type=float, default=None, help="the s the regime takes over; the end of her program otherwise")
    a = ap.parse_args(argv)
    if a.regime == 'heel':
        grid = {'plant': [float(v) for v in a.plant.split(',')], 'bridge': [float(v) for v in a.bridge.split(',')],
                'reach_m': [float(v) for v in a.reach.split(',')]}
        for row in search(a.scene, a.say, heel, grid, a.strokes):
            print(row)
        return 0
    with only_pull():
        if a.regime == 'take':
            regime = Danced(joints.program(a.take), trunk=a.trunk if a.trunk > 0.0 else None, slow=a.slow, gain=a.gain)
        else:
            regime = Crab(trunk=a.trunk, gain=a.gain, mark=a.mark)
        rows, sc, s = play(a.scene, a.say, regime, start=a.start, until=a.until)
    for k, r in enumerate(rows):
        if k + 1 == len(rows) or (rows[k + 1]['phase'] != r['phase'] if a.regime != 'take' else r['t'] % 0.5 < 0.01):
            print('%6.2f %-10s along %+.3f across %+.3f up %.3f trunk %5.1f hands %4.0f feet %4.0f' % (
                r['t'], r['phase'], r['along'], r['across'], r['up'], r['trunk'], r['hands'], r['feet']))
    print('strokes %d end %+.3f fallen %s' % (getattr(regime, 'strokes', 0), rows[-1]['along'], rows[-1]['fallen']))
    if a.gcode:
        print('written', written(a.gcode, regime))
    return 0


if __name__ == '__main__':
    sys.exit(main())
