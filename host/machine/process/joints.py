"""Her joints driven as said: a program of poses on the drives' own servos, her law's torques aside.

    worker.apply(s, {'joints': 'bed_pull', 'when': 'bed_rest_ischial_bilateral'})
    worker.apply(s, {'joints': 'standing_yawn', 'keep': 'legs'})    # her law keeps her legs: she stands through it

A program (PROGRAMS, or `<name>.gcode` in her programs' home, `texts.gcode`) is beats of (name, seconds,
{joint: degrees}): each beat blends every joint it names from
where the beat before left it to its degrees over its seconds, the rest held where the program found them, and
its last pose is held after. Said with `when` it waits for her text's phase of that name (`se2.doing`) and plays
from the pose she has then; said without, from the next pass. Played, her boards are told the setpoints, no feed,
the servo table's stiffness and her drives' clamps (`edge.drive`), the SIL boards' damping set back to the
table's (the session zeroed both for her law's own PD, `sprung.SERVO_SHARE`); a say, entries or scenario heard
ends it and her law drives again. Said with `keep`, the groups named (KEPT: legs, arms, trunk) stay her law's and
the program drives the rest, so a take's upper body plays on her own balance. `bed_pull`, up the bed from its end,
is what a puppet of her found (her joints
driven from her text's sit at the bed's edge, fists planted): the shoulders flexed against the fists as the hips
lift the thighs and the knees open, 0.20 m in; the trunk down over 1.3 s, 0.14 m more; the legs raised straight,
the knees folded and the thighs lowered so the heels set down on the foam 0.22 m in at 65 N; one stroke of the
heels 0.23 m, the stretch 0.16 more: 0.86 m in, supine, the legs along the bed. The knees opened while the soles
still bore stood her up (301 N), the heels dropped from the air hit the edge (725 N) and pushed her off.
"""
import os

from machine.body import drives, figure
from machine.body.servos import SERVO
from machine.errors import MachineError
from machine.law.se2 import doing
from machine.senses import edge
from machine.texts import gcode


def _both(**deg):
    """A pose's joints, both sides."""
    return {side + j: v for j, v in deg.items() for side in ('left_', 'right_')}


#: What the loop hears of a program (`worker.heard`): its name, the phase it waits for, what her law keeps.
HEARD = ('joints', 'when', 'keep')
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
                 ('stretch', 0.3, _both(knee=6.0, hip=-3.0))),
}
#: Each joint's index (`figure.JOINTS`); the servo table's stiffness and damping a joint, N m/rad and N m s/rad.
INDEX = {j: i for i, j in enumerate(figure.JOINTS)}
KP = tuple(float(SERVO[drives.kind(j)][1]) for j in figure.JOINTS)
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
    """`command`'s program ({'joints': name, 'when': phase or None}) kept on `g` to play till it ends."""
    kept = {j for group in str(command.get('keep', '')).split() for j in KEPT[group]}
    g['played'] = {'name': command['joints'], 'beats': program(command['joints']), 'when': command.get('when'),
                   'at': None, 'pose': None, 'mine': [i for i, j in enumerate(figure.JOINTS) if j not in kept]}


def ended(g):
    """The program on `g` over: her law drives again."""
    g['played'] = None


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


def drive(g, world):
    """Her boards told this pass (`sprung.passed`): her law's setpoints, feed and clamps (`g['cmd']`), then the
    program's over them where one plays."""
    edge.drive(world, g['joints'], **g['cmd'])
    played(g, world)


def played(g, world):
    """The program on `g`, if any, this pass: its pose to her boards (`edge.drive`) over the law's."""
    import numpy as np
    p = g.get('played')
    if p is None:
        return
    if p['at'] is None:
        if p['when'] is not None and doing(g) != p['when']:
            return
        p['at'], p['pose'] = g['t'], np.degrees(g['sense'][edge.Q])
        world.gains[:, 1] = KD
    joints = np.asarray([i for i in g['joints'] if i in p['mine']], int)
    edge.drive(world, joints, target=np.radians(pose(p['beats'], p['pose'], g['t'] - p['at']))[joints],
               feed=np.zeros(len(joints)), kp=np.asarray(KP)[joints], limit=g['clamp'][joints])
