"""Her figure as MuJoCo's XML: segments, a motor a joint, contacts, clothes, what hangs loose.

The world steps it (`machine.physics`).
"""
import math

from machine import build, drives, figure, floor, skeleton
from machine.drives import kind
from machine.figure import BODY, CONTACTS, HAIR_AT, HEM_AT, JOINTS, MASS_KG, SEGMENTS

#: The joints with a mechanical stop, (low, high) deg: the elbow straight at -5, as an arm's is -
#: without it the forearm folded back under her weight, -82 to -161 pushing up (2026-09-30); the
#: back as a woman's bends, the get-up asking -8 to 60 of the spine and the falls 45 of the waist -
#: without, tripped on a sill and down, her spine folded back 98-101 degrees (2026-10-01). The
#: knee stopped at -5: its drive on its axis, swinging, it snapped to -24 in the air, the walk
#: hopped and she fell (the page's recording, 2026-10-02).
STOPS = {'elbow': (-5.0, 160.0), 'spine': (-30.0, 85.0), 'spine_roll': (-35.0, 35.0),
         'waist': (-50.0, 50.0), 'knee': (-5.0, 165.0)}

#: A joint with no drive (`drives.passive`): its armature, kg m^2 - nearly none, a spring's; held,
#: its stops HELD_DEG either side of its rest.
PASSIVE_J, HELD_DEG = 0.0005, 0.5

#: The hips' yaw stopped HIP_YAW_DEG either side, 0 free: the gimbal's roll drum behind the hip
#: swings with it toward the other's (`drives.JOINTS`). At 25 the scoreboard held 81.0 % as free and
#: 3 get-ups of 3 (2026-10-02).
HIP_YAW_DEG = 25.0

#: The contacts' friction cone: elliptic, the same grip every way, at IMPRATIO. On MuJoCo's
#: pyramid she walked along the world's axes and fell 1.2 m on 45 degrees off them; elliptic at 1
#: she fell every way at 1.18 m, at 3 and 10 walked 13 m every way (2026-09-30).
CONE, IMPRATIO = 'elliptic', 10.0

#: What her clothes cover and what the cloth grips with, sliding: jeans over the pelvis, the
#: thighs and the knees, denim; a tee over the torso and the upper arms, cotton - the rest bare,
#: the soles the sneakers'. The cloth gives CLOTH_GIVE_M over its padding before it bears.
CLOTH = {'pelvis': 0.55, 'thigh': 0.55, 'shank': 0.55, 'torso': 0.45, 'upper_arm': 0.45}
CLOTH_GIVE_M = 0.004

#: A jeans' leg hangs `figure.HEM_AT` under the knee, HEM_KG HEM_M further down, on two hinges -
#: fore and aft, and aside - held to the shin by HEM_K N m/rad, damped by HEM_D N m s/rad and
#: stopped within HEM_STOP_S where its cloth meets the leg or the sneaker: its end HEM_FORE_DEG
#: forward (the heel), HEM_BACK_DEG back (the instep), HEM_SIDE_DEG aside (the ankle). It touches
#: nothing else. Free to 46 degrees it swung the leg's end through the cloth; stopped at 14,
#: softly, it swung to 19 and the shin stood 35 mm out of it aside, the sneaker 64 (2026-09-28).
HEM_M, HEM_KG, HEM_K, HEM_D = 0.2, 0.12, 0.6, 0.045
HEM_FORE_DEG, HEM_BACK_DEG, HEM_SIDE_DEG, HEM_STOP_S = 5.0, 8.0, 3.0, 0.005
HEMS = tuple('%s_hem_%s' % (side, axis) for side in ('left', 'right') for axis in 'xz')

#: Her hair's fall hangs from `figure.HAIR_AT`, HAIR_KG HAIR_M under it, on two hinges - fore and
#: aft, and aside - held by HAIR_K N m/rad and damped by HAIR_D N m s/rad: with gravity's 0.041 it
#: swings at 1.8 Hz, a third of critical. Stopped HAIR_DEG back, HAIR_IN_DEG forward and
#: HAIR_SIDE_DEG aside, where it meets her nape and her throat. It touches nothing.
HAIR_M, HAIR_KG, HAIR_K, HAIR_D = 0.07, 0.06, 0.016, 0.0035
HAIR_DEG, HAIR_IN_DEG, HAIR_SIDE_DEG = 20.0, 4.0, 8.0
HAIRS = ('hair_x', 'hair_z')

#: What hangs loose on her: the hinges `World.loose` reads.
LOOSE = HEMS + HAIRS

#: Her geoms' contact bits: 2 meets the floor's 1, 4 her own - a part on a part, MuJoCo leaving
#: out a segment and its parent. Floor alone, her feet passed 22 mm into each other as she
#: walked (2026-09-28). Her thighs' spheres, cruder than her, pressed 2 kN apart at every passing
#: in her catwalk (2026-09-28); drawn, they pass 20 mm apart.
ME, MEETS = 2 | 4, 1 | 4

#: The soles' friction: sliding, and turning in place (m) - a point of contact turns freely, and
#: on its ball's edge the stance foot spun under the swinging leg (2026-09-25).
FRICTION, TORSION_M = 1.0, 0.08

#: The soles' give, MuJoCo's solref and solimp: a contact settles over SOLE_S s at SOLE_DAMP of
#: critical, its impedance SOLE_SOFT at a touch rising to 0.95 over SOLE_WIDTH_M of give - light
#: sneakers. On 27 cm soles, rigid (0.02, 1, 0.9, 0.001) a touchdown peaked at 1.9
#: kN, 3.5 times her weight; the give over 5 mm and the damping 1.5: 1.5 kN and the pendulum's
#: stir 1.7 -> 1.4 mm. Softer felled her first stride from standing every way: settling over
#: 0.035 s the body pitched on twice as fast (the sole a lag in the ankle's hold), damped 1.75
#: the stance foot's load flickered to 70 N as the other swung (2026-09-27).
SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M = 0.02, 1.5, 0.9, 0.005

#: The pads' gel (`figure.PADS`), MuJoCo's solref and solimp over `figure.PAD_M`: a touch's
#: impedance PAD_SOFT, rising to 0.95 as the gel is spent; settling over PAD_S s at PAD_DAMP of
#: critical. Over 12 falls against none, the padded parts' landing peak 5.99 -> 5.32 kN at the
#: median, its spread 3.00 -> 2.53; the pads as stiff as her cloth, the worst 8.4 -> 11.3, at
#: 0.005 s 61 (`tools/sim/landings.py`, 2026-10-01).
PAD_SOFT, PAD_S, PAD_DAMP = 0.5, 0.02, 1.5

#: Her left shoe's lace snagged on the right shoe (`World.lace`): from LACE_AT on the one to LACE_AT
#: on the other (their frames), LACE_M between - a tendon, its limit settling over LACE_S s - until
#: LACE_HOLD_N pulls it off. Under the right sole, pinned as the left foot lifted, it went taut
#: as that foot landed, its step as long, and she walked on; as a tug of 300 N for 0.2 s it
#: felled her on nothing seen (2026-09-28, 2026-10-01).
LACE_AT, LACE_M, LACE_S, LACE_HOLD_N = (0.0, -0.03, 0.07), 0.25, 0.03, 1000.0




def mjcf():
    """The figure as MuJoCo's XML: y up, a drive's motor on every joint, the floor's contacts,
    her skeleton colliding with itself, the floor and her skins but its segment's neighbours' - in
    the squat a hip's yaw stood 45 mm into the thigh's capsule, the boom 56 (`machine.skeleton`) -
    MuJoCo's parent filter off, which takes a welded part for its segment, each segment kept from
    its parent by name, what touches standing or in the squat she lands in too - there her arms
    stood 3-8 mm in her knees' drums and boards, the kick felling her walk."""
    from machine.physics import SKELETON
    xml = _mjcf(SKELETON)
    if not SKELETON:
        return xml
    kin = [(s[0], s[1]) for s in SEGMENTS if s[1]]
    near = {s[0]: [s[1]] + [k[0] for k in SEGMENTS if k[1] == s[0]] for s in SEGMENTS}
    kin += [(part, other) for part, rides, _g in skeleton.bodies() for other in near[rides] if other]
    if xml not in _APART:
        import importlib
        from machine import arrival, gait
        model = importlib.import_module('mujoco').MjModel.from_xml_string(_apart(xml, kin))
        _APART[xml] = sorted(set(skeleton.overlapping(model, gait.stand()))
                             | set(skeleton.overlapping(model, arrival.angles_of(arrival._squat()))))
    return _apart(xml, kin + _APART[xml])


_APART = {}


def _apart(xml, pairs):
    return xml.replace('<contact/>', '<contact>%s</contact>' % ''.join(
        '<exclude body1="%s" body2="%s"/>' % pair for pair in pairs))


def _mjcf(bones):
    from machine.physics import BACKDRIVE, PLACED, REFLECTED, SERVO, STEP_S
    shells = build.segments() if build.SHELLS else {}
    kids, riders = {}, build.riders() if PLACED and not shells else {}
    for seg in SEGMENTS:
        kids.setdefault(seg[1], []).append(seg)
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    give = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M)
    cloth = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, CLOTH_GIVE_M)
    contacts = ' contype="1" conaffinity="2"'
    parts = skeleton.bodies() if bones else []

    def body(seg):
        name, _parent, joints, offset, rest, share, com, gyr = seg
        h = math.radians(rest) / 2.0
        out = ['<body name="%s" pos="%g %g %g" quat="%g 0 0 %g">' % (
            (name,) + tuple(offset) + (math.cos(h), math.sin(h)))]
        if seg[1] is None:
            out.append('<freejoint name="root"/>')
        for joint, axis, sign in joints:
            stop, spring = STOPS.get(kind(joint)), drives.passive(joint)
            if kind(joint) == 'hip_yaw' and HIP_YAW_DEG:
                stop = (-HIP_YAW_DEG, HIP_YAW_DEG)
            if spring:
                stiffness, damp, rest = spring
                stop = (rest - HELD_DEG, rest + HELD_DEG) if stiffness is None else stop
            out.append('<joint name="%s" axis="%g %g %g" armature="%g"%s%s/>' % (
                (joint,) + tuple(sign * v for v in axes[axis]) + (
                    PASSIVE_J if spring else
                    SERVO[kind(joint)][3] + REFLECTED * (drives.armature(joint)
                                                         - SERVO[kind(joint)][3]),
                    ' limited="true" range="%g %g"' % stop if stop else '',
                    ' stiffness="%g" springref="%g" damping="%g"' % (
                        spring[0], spring[2], spring[1]) if spring and spring[0] else
                    ' frictionloss="%g"' % (BACKDRIVE * drives.backdrive(joint))
                    if BACKDRIVE and not spring else '')))
        mass = share * MASS_KG - sum(kg for _j, kg, _at, _i in riders.get(name, ()))
        out.append('<inertial pos="%g %g %g" mass="%g" fullinertia="%g %g %g %g %g %g"/>' % (
            shells[name][1] + (shells[name][0],) + shells[name][2]) if shells else
                   '<inertial pos="%g %g %g" mass="%g" diaginertia="%g %g %g"/>' % (
            tuple(com) + (mass,) + tuple(mass * g * g for g in gyr)))
        out += ['<body name="%s_drive" pos="%g %g %g"><inertial pos="0 0 0" mass="%g" '
                'diaginertia="%g %g %g"/></body>' % ((joint,) + at + (kg,) + inertia)
                for joint, kg, at, inertia in riders.get(name, ())]
        grip = ' contype="%d" conaffinity="%d" condim="4" friction="%%g %g 0.001"' % (
            ME, MEETS, TORSION_M)
        for part, shape, size, at in CONTACTS:
            if name.endswith(part):
                out.append('<geom type="%s" size="%s" pos="%g %g %g"%s%s/>' % (
                    (shape, ' '.join('%g' % v for v in size)) + tuple(at)
                    + (grip % FRICTION, give)))
        if name in ('left_foot', 'right_foot'):
            out.append('<site name="lace_%s" pos="%g %g %g" size="0.005"/>' % (
                (name.split('_')[0],) + LACE_AT))
        for part, radius, top, end in BODY:
            if name == part or name.endswith('_' + part):
                shape = ('type="sphere" size="%g" pos="%g %g %g"' % ((radius,) + top) if top == end
                         else 'type="capsule" size="%g" fromto="%s"' % (
                             radius, ' '.join('%g' % v for v in top + end)))
                out.append('<geom %s%s%s/>' % (shape, grip % CLOTH.get(part, FRICTION),
                                                cloth if part in CLOTH else ''))
        for k, (part, axis, radius, toward, size, wide) in enumerate(figure.PADS):
            for x in ((1.0, -1.0) if name == part == 'pelvis' else
                      ((1.0 if name.startswith('left') else -1.0),) if name.endswith('_' + part)
                      else ()):
                at = figure.pad(axis, radius, toward, size, x)
                shape = ('type="capsule" size="%g" fromto="%g %g %g %g %g %g"' % (
                    (size, at[0] - wide) + at[1:] + (at[0] + wide,) + at[1:]) if wide else
                         'type="sphere" size="%g" pos="%g %g %g"' % ((size,) + at))
                out.append('<geom name="%s_pad%d%s" %s%s priority="1" solref="%g %g" '
                           'solimp="%g 0.95 %g"/>' % (
                               name, k, '' if x > 0 else 'r', shape,
                               grip % CLOTH.get(part, FRICTION), PAD_S, PAD_DAMP, PAD_SOFT,
                               figure.PAD_M))
        if name.endswith('_shank'):
            side = name[:-len('_shank')]
            out += ['<body name="%s_hem" pos="0 %g 0">' % (side, -HEM_AT)]
            out += ['<joint name="%s_hem_%s" axis="%s" stiffness="%g" damping="%g" '
                    'armature="0" limited="true" range="%g %g" solreflimit="%g 1"/>' % (
                        side, axis, direction, HEM_K, HEM_D, low, high, HEM_STOP_S)
                    for axis, direction, low, high in (
                        ('x', '1 0 0', -HEM_FORE_DEG, HEM_BACK_DEG),
                        ('z', '0 0 1', -HEM_SIDE_DEG, HEM_SIDE_DEG))]
            out += ['<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                -HEM_M, HEM_KG, 0.02 * HEM_KG, 0.02 * HEM_KG, 0.02 * HEM_KG), '</body>']
        if name == 'head':
            out += ['<body name="hair" pos="%g %g %g">' % HAIR_AT]
            out += ['<joint name="hair_%s" axis="%s" stiffness="%g" damping="%g" armature="0" '
                    'limited="true" range="%g %g"/>' % (axis, direction, HAIR_K, HAIR_D, low, high)
                    for axis, direction, low, high in (
                        ('x', '1 0 0', -HAIR_IN_DEG, HAIR_DEG),
                        ('z', '0 0 1', -HAIR_SIDE_DEG, HAIR_SIDE_DEG))]
            out += ['<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                -HAIR_M, HAIR_KG, 0.0025 * HAIR_KG, 0.0025 * HAIR_KG, 0.0025 * HAIR_KG),
                '</body>']
        out += ['<body name="%s"><inertial pos="0 0 0" mass="1e-6" diaginertia="1e-10 1e-10 '
                '1e-10"/>%s</body>' % (part, ''.join(geoms)) for part, rides, geoms in parts
                if rides == name]
        for kid in kids.get(name, []):
            out += body(kid)
        return out + ['</body>']

    return '\n'.join(
        ['<mujoco model="gynoid">',
         '<option timestep="%g" gravity="0 -9.81 0" integrator="implicitfast" cone="%s" '
         'impratio="%g"%s' % (STEP_S, CONE, IMPRATIO, '><flag filterparent="disable"/></option>'
                              if bones else '/>'),
         '<default><joint damping="0.3"/><geom contype="0" conaffinity="0"/></default>',
         '<worldbody>',
         ] + floor.ground(contacts, give, TORSION_M)
        + body(SEGMENTS[0])
        + floor.rug(give, FRICTION, TORSION_M)
        + ['</worldbody>', '<contact/>', '<actuator>']
        + ['<motor joint="%s" ctrlrange="%g %g"/>' % (j, -max(SERVO[kind(j)][0], drives.peak(j)),
                                                      max(SERVO[kind(j)][0], drives.peak(j)))
           for j in JOINTS]
        + ['</actuator>', '<tendon>',
           '<spatial name="lace" limited="true" range="0 10" solreflimit="%g 1" width="0.002">'
           '<site site="lace_left"/><site site="lace_right"/></spatial>' % LACE_S,
           '</tendon>', '</mujoco>'])
