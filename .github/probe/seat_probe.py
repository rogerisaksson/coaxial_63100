"""The chair as test_errands plays it, a sit text swapped in: seat_probe.py HOST VARIANT WOBBLE.
Prints one JSON line: the seat's share and her soles' at 7 s, when the seat first bears 0.15 of
her, her pelvis's fall speed then, her hips and centre of mass along her soles then (m, her
frame: + ahead), her trunk's pitch, the rise's outcome."""
import builtins
import io
import json
import math
import sys

sys.path.insert(0, sys.argv[1])

import numpy as np

from machine import leap, modal, ortho, physics, sprung, worker

VARIANTS = {
    'base': '',
    # the design: an eccentric descent over 1.1 s to the knee's 1.52 rad (axial 1.84), the trunk
    # forward 0.68 rad (pitch 1.36) and the arms 0.45 (arm 1.29) as counterweights, the centre of
    # mass over the heels (sagittal -1.0); seated: upright, over the seat, the legs at -24 dB
    'design': '''
(defbehavior sit_chair (extends stance_hold)
  (phase lower
    (duration 1100ms) (blend_duration 1100ms)
    (latent_targets (mode_axial_compression 1.84) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0)))
  (phase seated (use lower)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))
    (impedance (stiffness mode_axial_compression 189) (damping mode_axial_compression 38))))
''',
    'design_deep': '''
(defbehavior sit_chair (extends stance_hold)
  (phase lower
    (duration 1100ms) (blend_duration 1100ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0)))
  (phase seated (use lower)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))
    (impedance (stiffness mode_axial_compression 189) (damping mode_axial_compression 38))))
''',
    'design_hold': '''
(defbehavior sit_chair (extends stance_hold)
  (phase lower
    (duration 1100ms) (blend_duration 1100ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0)))
  (phase seated (use lower)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0))))
''',
    # two stages: quickly to just over the seat with the centre of mass over the heels, then
    # slowly down till the seat meets her; seated as the base
    'soft_base': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 0.8) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 1800ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 0.8) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 800ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_sagittal_shift -1.35))))
''',
    'soft_design': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 1800ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 800ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))))
''',
    'soft_design_soft': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 1800ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 800ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))
    (impedance (stiffness mode_axial_compression 189) (damping mode_axial_compression 38))))
''',
    'w_design_lin': '''
(defbehavior sit_chair (extends stance_hold)
  (phase lower
    (duration 50ms) (blend_duration 1100ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0)))
  (phase seated (use lower)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))))
''',
    'w_soft_design': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 50ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))))
''',
    'w_soft_base': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 0.8) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 50ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 0.8) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_sagittal_shift -1.35))))
''',
    'w_soft_design_legs': '''
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 700ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 1.6) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -0.5)))
  (phase settle
    (duration 50ms) (blend_duration 1600ms)
    (latent_targets (mode_axial_compression 2.3) (mode_pitch_attitude 1.36) (mode_arm_swing 1.29) (mode_sagittal_shift -1.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase seated (use settle)
    (duration 60s) (blend_duration 320ms)
    (latent_targets (mode_axial_compression 2.40) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0) (mode_sagittal_shift -1.35))
    (impedance (stiffness mode_axial_compression 189) (damping mode_axial_compression 38))))
''',
    'r_design': '''
(defbehavior rise_chair (extends stance_hold)
  (phase preload
    (duration 350ms) (blend_duration 350ms)
    (latent_targets (mode_axial_compression 2.2) (mode_pitch_attitude 1.48) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 800)))
  (phase drive (use preload)
    (duration 2000ms) (blend_duration 950ms)
    (latent_targets (mode_axial_compression 0.05) (mode_pitch_attitude 0.08))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase stand (use drive)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0))))
''',
    'r_mid': '''
(defbehavior rise_chair (extends stance_hold)
  (phase preload
    (duration 350ms) (blend_duration 350ms)
    (latent_targets (mode_axial_compression 2.2) (mode_pitch_attitude 1.0) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 800)))
  (phase drive (use preload)
    (duration 2000ms) (blend_duration 950ms)
    (latent_targets (mode_axial_compression 0.05) (mode_pitch_attitude 0.08))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase stand (use drive)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0))))
''',
    'r_soft_design': '''
(defbehavior rise_chair (extends stance_hold)
  (phase preload
    (duration 350ms) (blend_duration 350ms)
    (latent_targets (mode_axial_compression 2.2) (mode_pitch_attitude 1.48) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 800)))
  (phase drive (use preload)
    (duration 2000ms) (blend_duration 950ms)
    (latent_targets (mode_axial_compression 0.05) (mode_pitch_attitude 0.08))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase stand (use drive)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0))))
''',
}

variant, wobble = sys.argv[2], float(sys.argv[3])
VARIANTS['r_soft_design'] = VARIANTS['w_soft_base'] + VARIANTS['r_soft_design']
extra = VARIANTS[variant]


def swapped(text):
    """stunts_base.lisp with each of the variant's behaviors in place of its own."""
    for form in ortho.read(extra):
        name = form[1]
        start = text.index('(defbehavior %s ' % name)
        end = text.index('\n;;', start)
        new = extra[extra.index('(defbehavior %s ' % name):]
        nxt = new.find('\n(defbehavior', 1)
        text = text[:start] + (new if nxt < 0 else new[:nxt]).strip() + '\n' + text[end:]
    return text


def opened(path, *args, **kw):
    if extra and path.replace('\\', '/').endswith('/stunts_base.lisp'):
        with builtins.open(path, encoding='utf-8') as f:
            return io.StringIO(swapped(f.read()))
    return builtins.open(path, *args, **kw)


ortho.open = opened
physics.ROOM, physics.SIGHT = 0.0, 0.0
s = worker.session()
worker.apply(s, {'scenario': 'chair_behind'})
g = s['g']
world = g['world']
q = world.data.qpos[3:7].copy()
c2, s2 = math.cos(wobble / 2.0), math.sin(wobble / 2.0)
world.data.qpos[3:7] = (q[0] * c2 - q[2] * s2, q[1] * c2 + q[3] * s2, q[2] * c2 + q[0] * s2, q[3] * c2 - q[1] * s2)
world._mj.mj_forward(world.model, world.data)
m, d = world.model, world.data
torso, pelvis = m.body('torso').id, m.body('pelvis').id
weight = float(g['fr'][modal.WEIGHT])
rows = []


def record():
    y, fr = g['y'], g['fr']
    rot = fr[modal.ROT].reshape(3, 3)
    c = rot @ y[modal.COM]
    mid = (y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2].mean(axis=0)
    yaw = float(fr[modal.HEADING])
    ahead = np.array([math.sin(yaw), 0.0, math.cos(yaw)])
    tor = d.xmat[torso].reshape(3, 3)[:, 1]
    pitch = math.atan2(float(tor @ ahead), float(tor[1]))
    hips_along = float(rot[0] @ d.xpos[pelvis]) - float(mid[0])
    rows.append((g['t'], float(y[modal.PROP]), sum(g['loads']) / weight, float(d.qpos[1]), float(d.qvel[1]),
                 float(c[0] - mid[0]), hips_along, pitch))


def run(seconds, until=None):
    got = None
    for _ in range(int(round(seconds / worker.DT))):
        worker.stepped(s)
        if g['n'] % 10 == 0:
            record()
        if got is None and until is not None and sprung.doing(g) == until:
            got = g['t']
    return got


run(1.0)
worker.apply(s, {'say': 'sit_chair'})
run(6.0)
prop7, soles7 = float(g['y'][modal.PROP]), sum(g['loads']) / weight
down = tuple(bool(h <= leap.TOUCH_M) for h in g['y'][modal.LOWEST] - g['fr'][modal.FLOOR])
worker.apply(s, {'say': 'rise_chair'})
stand_t = run(6.0, until='stand')
run(1.0)
soles_end = sum(g['loads']) / weight
r = np.array(rows)
t, prop = r[:, 0], r[:, 1]
first = np.flatnonzero((prop >= 0.15) & (t < 7.0))
k = int(first[0]) if len(first) else None
win = (t >= 1.0) & (t <= 7.0)
pick = lambda col: round(float(r[k, col]), 3) if k is not None else None
out = {'variant': variant, 'wobble': wobble, 'prop7': round(prop7, 3), 'soles7': round(soles7, 3), 'down': down,
       'passed': bool(prop7 > soles7 and stand_t is not None and soles_end >= 0.85 and s['fallen_at'] is None),
       'contact_t': pick(0), 'fall_speed_at_contact': pick(4), 'com_along_at_contact': pick(5),
       'hips_along_at_contact': pick(6), 'pitch_at_contact': pick(7),
       'pelvis_min': round(float(r[win, 3].min()), 3), 'com_along_min': round(float(r[win, 5].min()), 3),
       'hips_along_min': round(float(r[win, 6].min()), 3), 'pitch_max': round(float(r[win, 7].max()), 3),
       'prop_max': round(float(prop[win].max()), 3), 'stand_t': stand_t, 'soles_end': round(soles_end, 3),
       'fell': s['fallen_at']}
print(json.dumps(out))
if len(sys.argv) > 4:
    t0, t1, every = (1.0, 11.0, 25) if sys.argv[4] == 'trace' else (float(sys.argv[4]), float(sys.argv[5]), 2)
    for row in r[::every]:
        if t0 <= row[0] <= t1:
            print('t %5.2f prop %5.2f soles %5.2f pelvis %5.3f vz %6.3f com %6.3f hips %6.3f pitch %5.2f' % tuple(row))
