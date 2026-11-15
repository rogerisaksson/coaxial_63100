"""The robot's session watched over a lodge errand: its joints' swings, its phases, bumps and brakes.

    w = house_watch.Watch(x, z, yaw)        # the robot's session laid there in the lodge, stepped half a second
    w.go(seconds, until)                    # stepped on, to its fall or `until(w)`
    w.spasm(), w.phases, w.bumps, w.stops   # what a run's line tells (`tools/sim/house.py`)

Each joint group's fastest swing (deg/s), widest sweep (deg) and the seconds the robot's master bus held it;
the robot's errand's phase and its doing as they change, with its place; what of the robot's
own bumped the furniture in which phase; a rise's rows (`_row`) where traced; and each brake as it begins -
the robot's speed along and across its heading, what steered it the half second before
(`Watch._stopped`).
"""
import collections
import math

#: The furniture the robot's bumps are counted on, and the lodge's own boxes that are not.
FURNITURE = ('seat', 'back', 'chair_leg', 'bed', 'table', 'lamp', 'loft', 'stair')
SCENERY = ('floor', 'wall', 'roof', 'deck', 'door', 'pane', 'mullion', 'rail', 'post', 'stone', 'switch', 'shade')
#: The robot's joint groups a spasm is read on.
GROUPS = {'arms': ('left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow'), 'neck': ('neck', 'head'),
          'trunk': ('spine', 'spine_roll', 'waist')}
#: The robot's joints read this often, s.
EVERY_S = 0.01
#: A rise's doings traced (those they begin), a row this often, s (`_row`).
TRACED, TRACE_S = ('sit_edge', 'bed_rest', 'sit_up', 'plant', 'lean', 'thrust', 'stand', 'rest'), 0.1
#: A brake's row reads this long before it, s: the robot's phases begun, what steered it
#: (`Watch._stopped`).
FLIPS_S = 0.5


def named(m, i, heads):
    """Whether the model's geom `i` is named as one of `heads` begins."""
    return any(m.geom(i).name.startswith(h) for h in heads)


class Watch:

    """The robot's session stepped: each joint group's fastest swing (deg/s) and widest sweep (deg), the
    robot's phases, what of its own bumped the furniture in which, each brake as it begins
    (`_stopped`)."""

    def __init__(self, x, z, yaw):
        from machine.world import physics
        from machine.process import sprung, worker
        from machine.body.figure import JOINTS
        physics.ROOM, physics.SIGHT = 1.0, 0.0
        self.s = s = worker.session()
        sprung.relaid(s['g'], (x, z), yaw)
        self.g, self.world = s['g'], s['g']['world']
        m = self.world.model
        self.furn = {i for i in range(m.ngeom) if named(m, i, FURNITURE) and float(self.world.data.geom_xpos[i][1]) > -0.5}
        self.robot_own = {i for i in range(m.ngeom) if int(m.geom_bodyid[i]) > 0 and i not in self.furn and not named(m, i, SCENERY)}
        self.last, self.read, self.fast, self.lo, self.hi, self.when, self.held = None, -1.0, {}, {}, {}, {}, {}
        self.phases, self.bumps, self.stops = [], {}, []
        self.steered = collections.deque(maxlen=int(FLIPS_S / EVERY_S))
        self.trace: list | None = None
        self.driven = [JOINTS[j] for j in self.g['joints']]
        self.go(0.5)
        self.fast, self.lo, self.hi, self.when, self.held = {}, {}, {}, {}, {}

    def go(self, seconds, until=None):
        """`seconds` of passes, or to the robot's fall or `until(self)`: whether `until` came."""
        from machine.process import sprung, worker
        g, t0 = self.g, self.g['t']
        while g['t'] - t0 < seconds and self.s['fallen_at'] is None:
            worker.stepped(self.s)
            self._read()
            er = g.get('_errand') or {}
            tag = '%s/%s' % (er.get('phase'), sprung.doing(g))
            q = self.world.data.qpos
            fresh = not self.phases or self.phases[-1][1] != tag
            self.stops += [self._stopped(tag)] if fresh and tag.endswith('/brake') else []
            self.phases += [(round(g['t'], 2), tag, round(float(q[0]), 2), round(float(q[2]), 2))] if fresh else []
            if not self.steered or g['t'] - self.steered[-1][0] >= EVERY_S:
                self.steered.append((g['t'], float(g.get('twist', (0.0, 0.0, 0.0))[2]), float(g.get('has_heading', 0.0)),
                                     float(g['heading'])))
            if until is not None and until(self):
                return True
        return False

    def _stopped(self, tag):
        """A brake begun: (t, its phase/doing, the robot's speed along and across its heading, m/s - across to
        the robot's left -, its pelvis's yaw off its heading, rad, its phases begun in the
        FLIPS_S before it, and over them the fastest turn told the robot, rad/s, the share of them its heading
        was held to a target, and how fast its heading turned, rad/s): what its stop finds the robot
        in and what steered it into it."""
        from machine.senses import edge
        g = self.g
        h, v = float(g['heading']), g['est']['vel']
        was = list(self.steered) or [(g['t'] - EVERY_S, 0.0, 0.0, h)]
        return (round(g['t'], 2), tag, round(float(v[0]) * math.sin(h) + float(v[2]) * math.cos(h), 2),
                round(float(v[0]) * math.cos(h) - float(v[2]) * math.sin(h), 2),
                round(math.remainder(float(edge.yaw(g['sense'])) - h, math.tau), 2),
                sum(1 for p in self.phases if p[0] >= g['t'] - FLIPS_S),
                round(max(abs(r[1]) for r in was), 2), round(sum(r[2] for r in was) / len(was), 2),
                round(math.remainder(h - was[0][3], math.tau) / max(g['t'] - was[0][0], EVERY_S), 2))

    def _read(self):
        from machine.process import sprung, worker
        g, d, m = self.g, self.world.data, self.world.model
        if g['t'] - self.read < EVERY_S:
            return
        angles, dt = worker.told(self.s)['angles'], g['t'] - self.read
        for name, joints in GROUPS.items():
            for j in joints:
                v = float(angles.get(j, 0.0))
                if self.last is not None and abs(v - self.last.get(j, v)) / dt > self.fast.get(name, 0.0):
                    self.fast[name] = abs(v - self.last.get(j, v)) / dt
                    self.when[name] = (round(g['t'], 2), j, sprung.doing(g), (g.get('_errand') or {}).get('phase'))
                self.lo[name], self.hi[name] = min(self.lo.get(name, v), v), max(self.hi.get(name, v), v)
        held = dict(zip(self.driven, g['bus'][2]['alpha']))
        for name, joints in GROUPS.items():
            self.held[name] = self.held.get(name, 0.0) + dt * float(max((held.get(j, 0.0) for j in joints), default=0.0) > 0.0)
        self.last, self.read = {j: float(angles.get(j, 0.0)) for js in GROUPS.values() for j in js}, g['t']
        if self.trace is not None and str(sprung.doing(g)).startswith(TRACED) and (
                not self.trace or g['t'] - self.trace[-1][0] >= TRACE_S):
            self.trace.append(_row(g, d, m))
        phase = (g.get('_errand') or {}).get('phase')
        for k in range(d.ncon):
            a, b = int(d.contact[k].geom1), int(d.contact[k].geom2)
            for f, o in ((a, b), (b, a)):
                if f in self.furn and o in self.robot_own:
                    self.bumps.setdefault('%s@%s' % (m.geom(f).name, phase), round(g['t'], 2))

    def spasm(self):
        return {name: (round(self.fast.get(name, 0.0)), round(self.hi.get(name, 0.0) - self.lo.get(name, 0.0)),
                       self.when.get(name), round(self.held.get(name, 0.0), 2)) for name in GROUPS}


def _row(g, d, m):
    """The robot's rise traced (`TRACED`): t, its doing, its centre of mass (x, z, height) and its speed
    (x, z), each sole's place (x, z, height) and load (N), the line of the robot's hips off square (deg) - the robot's
    law's state, its thighs the world's -, the move it plays and its phase, what the robot's moves sense
    (the robot's pitch, deg, its fists' and feet's shares, its pelvis off the bed's middle, m), the
    robot's pelvis's, fists' and left foot's heights (m) and its left knee, hip and shoulder (deg)."""
    from machine.law import modal
    from machine.process import joints, sprung
    y = g['y']
    c, v, so = y[modal.COM], y[modal.VCOM], y[modal.SOLE].reshape(2, 3)
    lt, rt = d.xpos[m.body('left_thigh').id], d.xpos[m.body('right_thigh').id]
    p, felt = g.get('played') or {}, joints._felt(g)
    played = None if p.get('state') is None else [p['name'], int(p['state'][0])]
    return (round(g['t'], 2), sprung.doing(g), [round(float(c[i]), 2) for i in (0, 2, 1)],
            [round(float(v[i]), 2) for i in (0, 2)], [[round(float(so[k, i]), 3) for i in (0, 2, 1)] for k in (0, 1)],
            [round(float(n)) for n in y[modal.LOAD]], round(math.degrees(math.atan2(float(lt[2] - rt[2]), float(lt[0] - rt[0]))), 1),
            played, [round(felt[k], 3) for k in ('pitch', 'hands', 'left_foot', 'right_foot', 'along')],
            [round(float(d.xpos[m.body(b).id][1]), 3) for b in ('pelvis', 'left_hand', 'right_hand', 'left_foot')],
            [round(math.degrees(float(d.qpos[m.joint(k).qposadr[0]]))) for k in ('left_knee', 'left_hip', 'left_shoulder')])
