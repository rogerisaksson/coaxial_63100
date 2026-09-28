#!/usr/bin/env python3
"""Her get-up from the floor, tried in the world alone. She is laid down `how` (side, prone,
supine) in a start pose (curled, the squat's joints; flat, legs and arms straight), the pelvis
`y` up, dropped and settled, then a sequence of joint poses is eased one to the next and printed
at 0.1 s: the pelvis's height, its axes' world-y parts, the loads, what touches the floor.

    python tools/sim/getup_lab.py prone flat 0.2 pushup
    python tools/sim/getup_lab.py side curled 0.3 toprone

Found (2026-09-27): from the side, straightening out rolls her onto her back; from the back, a
leg crossed over (either way) or the right arm and leg swung up roll her onto her right side;
from the front the push-up onto hands and knees and on into the dog (the pelvis 0.48 m up on
hands and toes) work, but the squat's joints from there put the knees down and the torso on the
floor, and a lunge with the right foot tips her onto her left side. Sequences in the joints alone
do not stand her up: the moves with the feet down want the centre of mass placed over them, as
the arrival's keyframes are (`arrival.over`), and the kneel to the squat a step with a hand's
support. From the front (2026-09-28): the push-up leaves the head and the torso on the floor
(`heels`: onto the shins, the toes tucked, back on the heels, the soles never bearing; the pelvis
0.40 m); in the dog the pelvis is 0.48 m up with the head and the forearms down, the feet walked
in one at a time and the hips lowered over them bore 1-110 N (`bearwalk`) - the arms (40 and
25 N m at the shoulder and the elbow) do not lift her torso off the floor. From the back
(2026-09-28): the spine and the hips sit her up (`situp`, the pelvis upright), but folded over
drawn-in feet her centre of mass stays behind the heels, the pelvis rolled back 45 degrees; the
heels drawn to the buttocks (`tuck`) or a rock back and a throw forward (`rock`), the light legs
swing up and she rolls back over her shoulders. From the knees, the hips straightened with the
chest on the floor slid the knees back and laid her flat (`tallkneel`). On 80 and 60 N m arms,
twice hers, the dog stood 0.57 m up and still on its forearms and head, the feet 50-100 N, and
stepped in she tipped onto her side: the joints alone do not balance her on hands and feet.
"""
import math
import sys

from machine import arrival, figure, gait, physics
from machine.figure import JOINTS

_LEGS = {'left_hip_yaw': 0.0, 'right_hip_yaw': 0.0, 'left_hip_roll': 0.0, 'right_hip_roll': 0.0,
         'left_ankle_roll': 0.0, 'right_ankle_roll': 0.0, 'left_foot': 0.0, 'right_foot': 0.0,
         'spine_roll': 0.0, 'waist': 0.0, 'head': 0.0, 'left_wrist': 0.0, 'right_wrist': 0.0,
         'left_gripper': 20.0, 'right_gripper': 20.0}


def pose(hip, knee, ankle, spine, neck, shoulder, elbow):
    """{joint: deg} with both sides alike."""
    return dict(_LEGS, left_hip=hip, right_hip=hip, left_knee=knee, right_knee=knee,
                left_ankle=ankle, right_ankle=ankle, spine=spine, neck=neck,
                left_shoulder=shoulder, right_shoulder=shoulder, left_elbow=elbow,
                right_elbow=elbow)


FLAT = pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0)
#: The right arm and leg swung up and over.
RIGHT_OVER = {'right_shoulder': 110.0, 'right_elbow': 10.0, 'right_hip': -100.0, 'right_knee': 90.0}
#: (stage, seconds, the joints - None the squat's) by name.
SEQS = {
    'toprone': (('flat', 0.8, FLAT), ('turn', 1.0, RIGHT_OVER), ('hold', 1.0, {})),
    'sup2prone': (('flat', 0.8, FLAT),
                  ('cross', 1.0, {'right_hip': -70.0, 'right_knee': 90.0, 'right_hip_roll': 45.0,
                                  'right_hip_yaw': -30.0, 'right_shoulder': 90.0}),
                  ('flat', 0.8, FLAT), ('turn', 1.0, RIGHT_OVER), ('hold', 1.0, {})),
    'pushup': (('hands', 0.8, pose(-30.0, 30.0, 20.0, 0.0, -30.0, 60.0, 130.0)),
               ('push', 1.0, pose(-90.0, 90.0, 20.0, 0.0, -40.0, 90.0, 10.0)),
               ('dog', 1.2, pose(-100.0, 30.0, -30.0, 10.0, -30.0, 130.0, 5.0)),
               ('crouch', 1.0, pose(-125.0, 110.0, -30.0, 45.0, -20.0, 60.0, 30.0)),
               ('squat', 1.0, None)),
    'heels': (('hands', 0.8, pose(-30.0, 30.0, 20.0, 0.0, -30.0, 60.0, 130.0)),
              ('push', 1.0, pose(-90.0, 90.0, 20.0, 0.0, -40.0, 90.0, 10.0)),
              ('tuck', 0.6, pose(-90.0, 100.0, -35.0, 0.0, -40.0, 90.0, 10.0)),
              ('sit', 1.2, pose(-125.0, 140.0, -35.0, 10.0, -10.0, 20.0, 30.0)),
              ('squat', 1.0, None)),
    'bearwalk': (('hands', 0.8, pose(-30.0, 30.0, 20.0, 0.0, -30.0, 60.0, 130.0)),
                 ('push', 1.0, pose(-90.0, 90.0, 20.0, 0.0, -40.0, 90.0, 10.0)),
                 ('dog', 1.2, pose(-100.0, 30.0, -30.0, 10.0, -30.0, 130.0, 5.0)),
                 ('lstep', 0.8, dict(pose(-100.0, 30.0, -30.0, 10.0, -30.0, 130.0, 5.0),
                                     left_hip=-140.0, left_knee=100.0, left_ankle=-35.0)),
                 ('rstep', 0.8, dict(pose(-140.0, 100.0, -35.0, 10.0, -30.0, 130.0, 5.0))),
                 ('frog', 1.0, pose(-125.0, 115.0, -35.0, 45.0, -20.0, 90.0, 10.0)),
                 ('squat', 1.0, None)),
    'situp': (('crunch', 0.8, pose(0.0, 30.0, 0.0, 60.0, 40.0, 60.0, 10.0)),
              ('sit', 1.2, pose(-80.0, 30.0, 0.0, 40.0, 20.0, 60.0, 10.0)),
              ('knees', 1.2, pose(-110.0, 130.0, -30.0, 45.0, 10.0, 90.0, 10.0)),
              ('fold', 1.0, pose(-135.0, 145.0, -38.0, 60.0, 0.0, 100.0, 10.0))),
    'tuck': (('crunch', 0.8, pose(0.0, 30.0, 0.0, 60.0, 40.0, 60.0, 10.0)),
             ('sit', 1.2, pose(-80.0, 30.0, 0.0, 40.0, 20.0, 60.0, 10.0)),
             ('heels', 2.2, pose(-100.0, 165.0, -45.0, 40.0, 10.0, 100.0, 10.0)),
             ('lean', 2.0, pose(-125.0, 165.0, -45.0, 75.0, -10.0, 110.0, 10.0))),
    'rock': (('crunch', 0.8, pose(0.0, 30.0, 0.0, 60.0, 40.0, 60.0, 10.0)),
             ('sit', 1.2, pose(-80.0, 30.0, 0.0, 40.0, 20.0, 60.0, 10.0)),
             ('knees', 1.0, pose(-110.0, 140.0, -30.0, 45.0, 10.0, 90.0, 10.0)),
             ('back', 0.4, pose(-120.0, 150.0, -20.0, 20.0, 30.0, 30.0, 40.0)),
             ('swing', 0.3, pose(-140.0, 150.0, -40.0, 70.0, -10.0, 150.0, 0.0))),
    'tallkneel': (('hands', 0.8, pose(-30.0, 30.0, 20.0, 0.0, -30.0, 60.0, 130.0)),
                  ('push', 1.0, pose(-90.0, 90.0, 20.0, 0.0, -40.0, 90.0, 10.0)),
                  ('kneel', 1.5, pose(0.0, 90.0, 20.0, 0.0, 0.0, 0.0, 10.0))),
    'halfkneel': (('hands', 0.8, pose(-30.0, 30.0, 20.0, 0.0, -30.0, 60.0, 130.0)),
                  ('push', 1.0, pose(-90.0, 90.0, 20.0, 0.0, -40.0, 90.0, 10.0)),
                  ('child', 1.0, pose(-130.0, 140.0, 30.0, 20.0, -20.0, 120.0, 5.0)),
                  ('lunge', 1.0, dict(pose(-60.0, 140.0, 30.0, 30.0, -20.0, 70.0, 40.0),
                                      right_hip=-130.0, right_knee=110.0, right_ankle=-30.0)),
                  ('rise', 1.2, pose(-116.0, 123.0, -32.0, 45.0, 0.0, 60.0, 30.0)),
                  ('squat', 1.0, None)),
}
LAID = {'side': (math.cos(math.pi / 4), 0.0, 0.0, -math.sin(math.pi / 4)),
        'prone': (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0),
        'supine': (math.cos(math.pi / 4), -math.sin(math.pi / 4), 0.0, 0.0)}


def main(argv=None):
    how, start, y0, seq = (argv or sys.argv[1:])[:4]
    world = physics.World()
    m, d = world.model, world.data
    curled = arrival.angles_of(arrival.keyframes(gait.CADENCE)[0][2])
    world.reset(curled if start == 'curled' else FLAT, where=(0.0, float(y0), 0.0), turn=LAID[how])
    now = [0.0]
    world.clock = lambda: now[0]

    def show(tag):
        p = world.pose()
        t = figure.quat(p['qw'], p['qx'], p['qy'], p['qz'])
        on = sorted({m.body(m.geom_bodyid[g]).name for i in range(d.ncon)
                     for g in (d.contact[i].geom1, d.contact[i].geom2)}
                    - {'world', 'rug', 'sill', 'slip'})
        print('%5.2f %-6s y %.3f  up_y %+.2f fwd_y %+.2f left_y %+.2f  L %4.0f R %4.0f | %s' % (
            now[0], tag, p['y'], t[1][1], t[1][2], t[1][0], p['left_load'], p['right_load'],
            ' '.join(x.replace('left_', 'l.').replace('right_', 'r.') for x in on)))

    def play(seconds, to, tag):
        begun = {j: math.degrees(world.target[i]) for i, j in enumerate(JOINTS)}
        t0 = now[0]
        while now[0] < t0 + seconds:
            now[0] += 0.001
            k = gait.eased((now[0] - t0) / seconds)
            for i, j in enumerate(JOINTS):
                world.write(i, begun[j] + (to.get(j, begun[j]) - begun[j]) * k)
            world.advance()
            if round(now[0] * 1000) % 100 == 0:
                show(tag)

    play(1.5, {}, 'lie')
    for stage, seconds, to in SEQS[seq]:
        play(seconds, curled if to is None else to, stage)
    play(1.0, {}, 'hold')


if __name__ == '__main__':
    main()
