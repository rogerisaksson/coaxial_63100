"""Her joints driven as said: a program of poses on the drives' own servos, her law's torques aside.

    worker.apply(s, {'joints': 'bed_pull', 'when': 'bed_rest_ischial_bilateral', 'strokes': 5})
    worker.apply(s, {'joints': 'standing_yawn', 'keep': 'legs'})    # her law keeps her legs: she stands through it

A program (PROGRAMS, or `<name>.gcode` in her programs' home, `texts.gcode`) is beats of (name, seconds,
{joint: degrees}): each beat blends every joint it names from
where the beat before left it to its degrees over its seconds, the rest held where the program found them; its
last beat over, her law drives again (`ended`, the boards' gains as the session laid them). Said with `when` it
waits for her text's phase of that name (`se2.doing`) and plays
from the pose she has then; said without, from the next pass; bound to a phase by her text (a stage's
`:joint-program`, laid on her run by `maneuver.said` as `g['programs']` {phase: program}) it plays from that
phase's first tick, so `lie_bed` at the bed's end pulls her up on its own. Played, her boards are told the
setpoints, no feed,
the servo table's stiffness and her drives' clamps (`edge.drive`), the SIL boards' damping set back to the
table's (the session zeroed both for her law's own PD, `sprung.SERVO_SHARE`); a say, entries or scenario heard
ends it and her law drives again. Said with `keep`, the groups named (KEPT: legs, arms, trunk) stay her law's and
the program drives the rest, so a take's upper body plays on her own balance. Said with `strokes`, the beats STROKES
names (first to last) play that many times over: `bed_pull`'s stroke up the bed, 0.12 m each (her text counts them
from the bed's depth her eyes saw, `maneuver`). `bed_pull`, up the bed from its end, is what a puppet of her found
(her joints
driven from her text's sit at the bed's edge, fists planted): the shoulders flexed against the fists as the hips
lift the thighs and the knees open, 0.20 m in; the trunk down over 1.3 s, 0.14 m more; the legs raised straight,
the knees folded and the thighs lowered so the heels set down on the foam 0.22 m in at 65 N; one stroke of the
heels 0.23 m, the stretch 0.16 more with her trunk straightened: 0.86 m in, supine, the legs along the bed. The
knees opened while the soles still bore stood her up (301 N), the heels dropped from the air hit the edge (725 N)
and pushed her off. Held after its last beat on the peak clamps she lay still but rigid (90 N m, the trunk 18 deg
off flat): over, her text's flat rest limps her on the foam (`se2.limp`). The stroke (poses solved from her figure's
thigh 0.39 and shank 0.38 m, her hip 0.11 m over the foam): the heels planted 0.22 m from the hips, the pelvis
bridged 0.10 m, the heels pushed to a 0.45 m reach, the pelvis set down: 0.14-0.16 m up the bed a stroke on the
heels' grip, none lost, lying square (bridged 0.06 m she gained 0.07 a stroke square, 0.13 yawed 17 deg on her
side; the push to a 0.50 m reach 0.08); the legs planted with their hips' roll and yaw at zero push even: splayed
as the lie left them she slid 0.04 m aside a stroke at no yaw. The elbows pressed beside her dragged the pelvis back
0.10 m at each bridge, and the heels planted 0.43 m out landed on the foot edge and slipped off it. She lies still 1.2 s before the strokes, as the
take ends: begun inside its window they drew her feet in (hips back 0.66 of its 1.07) and the bench refused the lie.
"""
import functools
import os

from machine.body import drives, figure
from machine.body.servos import SERVO
from machine.errors import MachineError
from machine.law.se2 import BACK_SPEED, WALK_SPEED, YAW_RATE, doing
from machine.senses import edge
from machine.texts import gcode


def _both(**deg):
    """A pose's joints, both sides."""
    return {side + j: v for j, v in deg.items() for side in ('left_', 'right_')}


#: What the loop hears of a program (`worker.heard`): its name, the phase it waits for, what her law keeps, how many
#: strokes.
HEARD = ('joints', 'when', 'keep', 'strokes')
#: Her joints by group, what `keep` names.
KEPT = {'legs': tuple(j for j in figure.JOINTS if any(k in j for k in ('hip', 'knee', 'ankle', 'foot'))),
        'arms': tuple(j for j in figure.JOINTS if any(k in j for k in ('shoulder', 'elbow', 'wrist', 'gripper'))),
        'trunk': ('spine_roll', 'spine', 'waist', 'neck', 'head')}
#: Her programs: beats of (name, s, {joint: deg}).
PROGRAMS = {
    'bed_pull': (('pull', 0.65, _both(shoulder=30.0, elbow=0.0, hip=-75.0, knee=30.0, ankle=0.0)),
                 ('lie', 1.3, _both(shoulder=0.0, elbow=5.0, hip=-10.0, knee=15.0)),
                 ('up', 0.6, _both(hip=-75.0, knee=6.0)),
                 ('fold', 0.5, _both(knee=150.0)),
                 ('lower', 0.3, _both(hip=-60.0)),
                 ('shove', 0.6, _both(hip=-35.0, knee=100.0)),
                 ('stretch', 0.3, {'spine': 0.0, 'waist': 0.0, 'neck': 0.0, **_both(knee=6.0, hip=-3.0)}),
                 ('still', 1.2, {}),
                 ('plant', 0.4, _both(hip=-56.0, knee=146.0, hip_roll=0.0, hip_yaw=0.0)),
                 ('bridge', 0.25, _both(hip=-34.0, knee=134.0)),
                 ('push', 0.3, _both(hip=-32.0, knee=104.0)),
                 ('set', 0.2, _both(hip=-46.0, knee=108.0)),
                 ('straight', 0.4, {'spine': 0.0, 'waist': 0.0, 'neck': 0.0, **_both(knee=6.0, hip=-3.0)})),
}
#: A program's stroke: the beats from the first named to the last, played over as many times as said (`strokes`).
STROKES = {'bed_pull': ('plant', 'set')}
#: Her moves: programs whose phases her senses tell (a module's `laid` and `step`), not beats, her text's rows not
#: yet due waiting while one plays (`waits.played_back`): `bed_exit`, out of a bed over its end onto its edge as her
#: law sits her (`machine.moves.bedexit`, played in C: `tools.cores.bedexit`); `scoot`, along the floor as her twist
#: tells her once she sits on it (`scooted`, `machine.moves.scoot`, in C: `tools.cores.scoot`); `bed_entry`, into a
#: bed over its end to its middle as where she lies tells her (`machine.moves.bedentry`, in C: `tools.cores.bedentry`),
#: her text going on under it (its `HOLDS`). A move's `SENSED` names what a pass gives it beyond her pose (`_felt`),
#: her bed exit's soles, shanks and pitch where it names none.
MOVES = {'bed_exit': 'tools.cores.bedexit', 'scoot': 'tools.cores.scoot', 'bed_entry': 'tools.cores.bedentry'}
SENSED = ('soles', 'shins', 'pitch')
#: The senses `bodily` tells at once: her soles', shanks', fists' and each foot's share of her weight, her pitch.
BODILY = ('soles', 'shins', 'hands', 'left_foot', 'right_foot', 'pitch')
#: Each joint's index (`figure.JOINTS`); the servo table's stiffness and damping a joint, N m/rad and N m s/rad.
INDEX = {j: i for i, j in enumerate(figure.JOINTS)}
KP = tuple(float(SERVO[drives.kind(j)][1]) for j in figure.JOINTS)
#: Her shanks in her skin (`edge.SKIN`): a move feels the edge it scoots over by them; her fists (hands and fingers)
#: and each foot (foot and toes) the floor she scoots on.
SHANKS = [i for i, segment in enumerate(edge.SKIN) if segment.endswith('_shank')]
FISTS = [i for i, segment in enumerate(edge.SKIN) if segment.endswith(('_hand', '_fingers'))]
FEET = tuple([edge.SKIN.index(side + part) for part in ('_foot', '_toes')] for side in ('left', 'right'))
KD = tuple(float(SERVO[drives.kind(j)][2]) for j in figure.JOINTS)


def program(name):
    """The beats of program `name`: PROGRAMS' or read from `<name>.gcode` in her programs' home (`gcode.program`)."""
    if name in PROGRAMS:
        return PROGRAMS[name]
    path = os.path.join(gcode.DIR, name + '.gcode')
    if not os.path.isfile(path):
        raise MachineError('her programs are %s and %s, not %s' % (', '.join(sorted(PROGRAMS)), gcode.DIR, name))
    return gcode.program(path)


def said(g, command):
    """`command`'s program ({'joints': name, 'when': phase or None, 'strokes': n}) kept on `g` to play till it ends,
    its stroke (STROKES) `strokes` times over where said."""
    kept = {j for group in str(command.get('keep', '')).split() for j in KEPT[group]}
    if command['joints'] in MOVES:
        g['played'] = {'name': command['joints'], 'move': MOVES[command['joints']], 'when': command.get('when'),
                       'at': None, 'pose': None, 'gains': None,
                       'mine': [i for i, j in enumerate(figure.JOINTS) if j not in kept]}
        return
    beats = program(command['joints'])
    first, last = STROKES.get(command['joints'], (None, None))
    if first is not None and 'strokes' in command:
        i = next(k for k, b in enumerate(beats) if b[0] == first)
        j = next(k for k, b in enumerate(beats) if b[0] == last) + 1
        beats = tuple(beats[:i]) + tuple(beats[i:j]) * int(command['strokes']) + tuple(beats[j:])
    g['played'] = {'name': command['joints'], 'beats': beats, 'when': command.get('when'), 'at': None, 'pose': None,
                   'length': sum(seconds for _beat, seconds, _goals in beats), 'gains': None,
                   'mine': [i for i, j in enumerate(figure.JOINTS) if j not in kept]}


def ended(g):
    """The program on `g` over, the gains it set on her boards hers again (cut by a say and kept by the move after it as
    hers, her law drove at their stiffness: her spine 943 deg/s), her text's bindings dropped (`g['programs']`): her
    law drives again."""
    kept = (g.get('played') or {}).get('gains')
    g['world'].gains[:] = g['world'].gains if kept is None else kept
    g['played'], g['programs'] = None, {}


def scooted(g, twist):
    """Whether her `twist` is her scoot's (`worker.heard`): sat on the floor (`g['sat']`, her seat on it) or scooting,
    it drives her scoot (MOVES), begun as it asks one, her gaits untold (told one there, she stood off her seat into a
    walk and fell at 9.9 s); playing another program, or not sat on the floor, it is her gaits'."""
    from machine.senses import touch
    p = g.get('played') or {}
    if p.get('name', 'scoot') != 'scoot' or (not p and g.get('sat', 0.0) <= 0.5):
        return False
    g['asked'] = (0.0, 0.0, 0.0) if twist is None else tuple(float(v) for v in twist)
    vx, _vy, wz = g['twist'] = touch.against(g, g['asked'])
    if not p and (vx != 0.0 or wz != 0.0):
        said(g, {'joints': 'scoot'})
    return True


def _felt(g):
    """{sense: value} a move's pass may name (its `SENSED`), each told as the pass first reads it (`_Felt`)."""
    return _Felt(g)


class _Felt(dict):

    """A move's senses: her soles', shanks', fists' and each foot's share of her weight, her trunk's pitch (deg), her
    twist back and turning left as shares of her keys' (`se2`) as asked - not as `touch.against` cuts it: her fists
    behind her were felt as a wall, and S held 8 s scooted her 0.00 m -, her pelvis's heading (deg, left +) as her
    estimate turns it, and her pelvis from the middle of the bed she lies on along its length, that half length (m)
    and her pelvis's speed (m/s) as a look takes her (`furniture.lain`). Each told at its first read, those read
    together at once - the same expressions, the same bits: a bed's entry reads five of the thirteen, its exit three.
    Her soles', her skin's shares and her pitch told by one compiled part at their first read (`bodily`)."""

    def __init__(self, g):
        super().__init__()
        self.g = g

    def __missing__(self, name):
        g, modal = self.g, _modal()
        if name in ('along', 'half', 'speed'):
            self['along'], self['half'], self['speed'] = _furniture().lain(g)
        elif name in BODILY:
            if g.get('bodily') is None:
                from tools.cores import bodied
                g['bodily'] = bodied.on_args(bodily, 'bodily', (0, 1, 2, 3, 4, 5))
            fr = g['fr']
            told = g['bodily'](g['loads'][0], g['loads'][1], fr[modal.WEIGHT], g['sense'][edge.TOUCH], g['y'], fr[modal.ROT])
            self.update(zip(BODILY, (float(v) for v in told)))
        elif name in ('back', 'turn'):
            vx, _vy, wz = (float(v) for v in g.get('asked', (0.0, 0.0, 0.0)))
            self['back'] = min(max(-vx / (BACK_SPEED if vx < 0.0 else WALK_SPEED), -1.0), 1.0)
            self['turn'] = min(max(wz / YAW_RATE, -1.0), 1.0)
        elif name == 'heading':
            import math
            w, x, j, k = (float(v) for v in g['y'][modal.QUAT][:4])
            self['heading'] = math.degrees(math.atan2(2.0 * (w * j + x * k), 1.0 - 2.0 * (j * j + x * x)))
        else:
            raise KeyError(name)
        return self[name]


@functools.cache
def _modal():
    """`machine.law.modal`, imported at a move's first sense and not walked again each (a package's fromlist)."""
    from machine.law import modal
    return modal


@functools.cache
def _waits():
    from machine.law import waits
    return waits


@functools.cache
def _played():
    from tools.cores import played
    return played


@functools.cache
def _furniture():
    from machine.way import furniture
    return furniture


def bodily(load0, load1, weight, skin, y, rot):
    """(her soles', shanks', fists' and each foot's share of her weight off her skin, her trunk's pitch along her way,
    deg) of her law's arrays (her soles' loads, her `weight`, her edge's skin, modal's `y` and her frame's `rot`) -
    numpy's alone, so compiled (`bodied.on_args`, g['bodily']): told through numpy and byte-keyed memos her bed's
    exit read them in 39 us of its 109 a pass."""
    import numpy as np
    touch = np.linalg.norm(skin.reshape(-1, 3), axis=1)
    return ((load0 + load1) / weight, np.sum(touch[SHANKS]) / weight, np.sum(touch[FISTS]) / weight,
            np.sum(touch[FEET[0]]) / weight, np.sum(touch[FEET[1]]) / weight,
            _modal().pitched(y, rot.reshape(3, 3))[1] * (180.0 / np.pi))


def pose(beats, start, elapsed):
    """A program's pose (deg, every joint) `elapsed` s into its `beats`, from the pose `start` it began at."""
    import numpy as np
    out = np.array(start, float)
    for _beat, seconds, goals in beats:
        a = min(max(elapsed / seconds, 0.0), 1.0)
        for j, v in goals.items():
            out[INDEX[j]] += a * (v - out[INDEX[j]])
        elapsed -= seconds
        if elapsed <= 0.0:
            break
    return out


def scored(beats, start):
    """`beats` from the pose `start` (deg, every joint) as the score `posed` plays: each beat's seconds, its joints'
    indices and degrees end to end (`at` each beat's first), the pose the beats passed leave and how many have."""
    import numpy as np
    seconds = [float(s) for _beat, s, _goals in beats]
    return {'seconds': seconds, 'secs': np.array(seconds), 'k': 0, 'base': np.array(start, float),
            'at': np.cumsum([0] + [len(goals) for _beat, _s, goals in beats]),
            'joint': np.array([INDEX[j] for _beat, _s, goals in beats for j in goals], np.intp),
            'deg': np.array([v for _beat, _s, goals in beats for v in goals.values()], float)}


def posed(score, elapsed):
    """`pose` `elapsed` s into a program by its score (`scored`), `elapsed` never less than at the call before - the
    same bits, a pass a beat's goals: the beats passed kept as the pose they leave (each at a = 1, as `pose` blends a
    beat its `elapsed` is past), `elapsed` brought to the first not passed by `pose`'s own subtractions in its order
    (numpy's subtract.reduce, sequential). Replayed from its first beat each pass a program's pose took 0.14-1.2 ms over
    the bed's lie (173 beats) and 2.6-18 ms over turning in bed (2550), her world 0.05 of the wall."""
    import numpy as np
    k, seconds = score['k'], score['seconds']
    if k:
        elapsed = float(np.subtract.reduce(score['secs'][:k], initial=elapsed))
    out, at, joint, deg = score['base'].copy(), score['at'], score['joint'], score['deg']
    while k < len(seconds):
        a = min(max(elapsed / seconds[k], 0.0), 1.0)
        j = joint[at[k]:at[k + 1]]
        out[j] += a * (deg[at[k]:at[k + 1]] - out[j])
        elapsed -= seconds[k]
        if elapsed <= 0.0:
            break
        k += 1
        score['k'], score['base'] = k, out.copy()
    return out


def paced(beats, start):
    """`beats` with the first over no less time than the program's own pace asks of each joint from the pose `start`
    (deg, every joint) she begins it at: no joint faster than the program moves it from beat to beat. Over its own
    time - a take's first frame its T0, 1 ms - her trunk swung 734 deg/s, her arms 702 and her ankles 784 into the
    take's sit on the bed's side, and threw her 0.30 m in."""
    name, seconds, goals = beats[0]
    held, pace = dict(goals), {}
    for _beat, s, now in beats[1:]:
        for j, v in now.items():
            pace[j] = max(pace.get(j, 0.0), abs(v - held[j]) / s) if j in held else pace.get(j, 0.0)
            held[j] = v
    need = max((abs(v - float(start[INDEX[j]])) / pace[j] for j, v in goals.items() if pace.get(j, 0.0) > 0.0), default=0.0)
    return ((name, max(seconds, need), goals),) + tuple(beats[1:])


def drive(g, world):
    """Her boards told this pass (`sprung.passed`): her law's setpoints, feed and clamps (`g['cmd']`, laid once and
    bound to her edge with them, `edge.bound`), then the program's over them where one plays."""
    told = g.get('_driven')
    if told is None:
        told = g['_driven'] = (edge.bound(world, g['joints'], **g['cmd'])
                               or functools.partial(edge.drive, world, g['joints'], **g['cmd']))
    told()
    played(g, world)


def played(g, world):
    """The program on `g`, if any, this pass: its pose to her boards (`edge.drive`) over the law's; none, the one
    her text binds to the phase she is doing (`g['programs']`) begun from this pass."""
    import numpy as np
    p = g.get('played')
    if p is None and g.get('programs'):
        command = g['programs'].get(doing(g))
        if command is not None:
            said(g, command)
            p = g['played']
    if p is None:
        return
    if p['at'] is None:
        if p['when'] is not None and doing(g) != p['when']:
            return
        p['at'], p['pose'], p['gains'] = g['t'], np.degrees(g['sense'][edge.Q]), world.gains.copy()
        world.gains[:, 1] = KD
        # its joints, their stiffness and no feed laid once, its joints int32 so her edge's index cache keeps them
        p['joints'] = np.array([i for i in g['joints'] if i in p['mine']], np.int32)
        p['kp'], p['feed'] = np.asarray(KP)[p['joints']], np.zeros(len(p['joints']))
        # its setpoints and clamps written into buffers of its own each pass, its drive bound to them once
        p['target'], p['limit'] = np.zeros(len(p['joints'])), np.zeros(len(p['joints']))
        p['drive'] = edge.bound(world, p['joints'], target=p['target'], feed=p['feed'], kp=p['kp'], limit=p['limit'])
        if 'beats' in p:
            p['beats'] = paced(p['beats'], p['pose'])
            p['length'] = sum(seconds for _beat, seconds, _goals in p['beats'])
            p['score'] = scored(p['beats'], p['pose'])
        if 'move' in p:
            import importlib
            p['move'] = importlib.import_module(p['move'])
            p['state'], p['cmd'], p['last'] = p['move'].laid(p['pose'], g['t']), p['pose'].copy(), g['t']
            p['sensed'], p['holds'] = getattr(p['move'], 'SENSED', SENSED), getattr(p['move'], 'HOLDS', True)
            # its step and its joints told in one C call where it is one of her C moves (`tools.cores.played`)
            p['pass'] = _played().laid(g, world, p)
    if 'move' in p:
        dt, felt = g['t'] - p['last'], _felt(g)
        senses, q = [felt[name] for name in p['sensed']], p['pass']
        done = (_played().passed(q, g['t'], dt, senses) if q is not None
                else p['move'].step(p['state'], p['cmd'], np.degrees(g['sense'][edge.Q]), g['t'], dt, *senses))
        p['last'] = g['t']
        if done:
            ended(g)
            return
        if p['holds']:
            _waits().played_back(g, dt)
        if q is None:
            _told(g, world, p, p['cmd'])
        return
    if g['t'] - p['at'] >= p['length']:
        ended(g)
        return
    _told(g, world, p, posed(p['score'], g['t'] - p['at']))


def _told(g, world, p, deg):
    """Her boards told the program's pose `deg` (every joint's) on its joints: in radians, its clamps her law's, into
    its buffers and its bound drive (`edge.bound`) - numpy's arithmetic on each joint as on all of them taken after."""
    import numpy as np
    joints = p['joints']
    np.radians(np.take(deg, joints, out=p['target']), out=p['target'])
    np.take(g['clamp'], joints, out=p['limit'])
    if p['drive'] is not None:
        p['drive']()
    else:
        edge.drive(world, joints, target=p['target'], feed=p['feed'], kp=p['kp'], limit=p['limit'])
