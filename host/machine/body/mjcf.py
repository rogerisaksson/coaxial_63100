"""Her figure as MuJoCo's XML: segments, a motor a joint, contacts, clothes, what hangs loose.

The world steps it (`machine.world.physics`).
"""
import math

from machine.body import build, drives, figure, mechatronics, skeleton
from machine.world import floor, room
from machine.body.drives import kind
from machine.body.figure import BODY, HAIR_AT, HEM_AT, JOINTS, MASS_KG, SEGMENTS, contacts as undersides

_BUILD, _WORLD = mechatronics.robot(), mechatronics.environment()
_GARM, _ELAST = _BUILD['garments'], _BUILD['elastomers']
_HEM, _HAIR, _LACE = _GARM['hem'], _GARM['hair'], _GARM['lace']
_SOLE, _PAD = _ELAST['sole'], _ELAST['pad']
_PLIM = _BUILD['passive_limits']

#: Undriven spring armature, kg m^2, and held-joint stop half-width, deg (`drives.passive`, `[passive_limits]`).
PASSIVE_J, HELD_DEG = float(_PLIM['armature_kgm2']), float(_PLIM['held_deg'])

#: Pelvis weld stand height and stiffness time constant for system identification (`world.toml`).
STAND, STAND_M = 0.0, float(_WORLD['stand_m'])
_STAND_SOLREF_S = float(_WORLD['stand_solref_s'])

#: Friction cone, tangential-to-normal impedance ratio, gravity and joint damping (`world.toml`).
CONE, IMPRATIO = str(_WORLD['cone']), float(_WORLD['impratio'])
_GRAVITY, _JOINT_DAMP = float(_WORLD['g_m_s2']), float(_WORLD['joint_damping'])

#: Garment sliding friction, padding compliance, loose hem/hair pendulums and lace tendon (`[garments]`).
CLOTH = {k: float(v) for k, v in _GARM['friction'].items()}
CLOTH_GIVE_M = float(_GARM['give_m'])
HEM_M, HEM_KG = float(_HEM['length_m']), float(_HEM['kg'])
HEM_K, HEM_D = float(_HEM['k_nm_rad']), float(_HEM['d_nms_rad'])
HEM_FORE_DEG, HEM_BACK_DEG = float(_HEM['fore_deg']), float(_HEM['back_deg'])
HEM_SIDE_DEG, HEM_STOP_S = float(_HEM['side_deg']), float(_HEM['stop_s'])
HEMS = tuple('%s_hem_%s' % (side, axis) for side in ('left', 'right') for axis in 'xz')

HAIR_M, HAIR_KG = float(_HAIR['length_m']), float(_HAIR['kg'])
HAIR_K, HAIR_D = float(_HAIR['k_nm_rad']), float(_HAIR['d_nms_rad'])
HAIR_DEG, HAIR_IN_DEG, HAIR_SIDE_DEG = (float(_HAIR['back_deg']), float(_HAIR['in_deg']),
                                        float(_HAIR['side_deg']))
HAIRS = ('hair_x', 'hair_z')

#: What hangs loose on her: the hinges `World.loose` reads.
LOOSE = HEMS + HAIRS

#: Contact bitmasks: 2 meets the floor's 1, 4 her own segments and skeleton.
ME, MEETS = 2 | 4, 1 | 4

#: Sole sliding/torsional friction and sneaker/gel-pad contact compliance (`[elastomers]`).
FRICTION, TORSION_M = float(_SOLE['friction']), figure.sole_torsion_m()
SOLE_S, SOLE_DAMP = float(_SOLE['settle_s']), float(_SOLE['damp'])
SOLE_SOFT, SOLE_WIDTH_M = float(_SOLE['soft']), float(_SOLE['width_m'])
PAD_SOFT, PAD_S, PAD_DAMP = float(_PAD['soft']), float(_PAD['settle_s']), float(_PAD['damp'])
LACE_AT = tuple(float(v) for v in _LACE['at'])
LACE_M, LACE_S, LACE_HOLD_N = float(_LACE['length_m']), float(_LACE['stop_s']), float(_LACE['hold_n'])




def mjcf(free=True):
    """The figure as MuJoCo's XML: y up, a drive's motor on every joint, the floor's contacts,
    her skeleton colliding with itself, the floor and her skins but its segment's neighbours' - in
    the squat a hip's yaw stood 45 mm into the thigh's capsule, the boom 56 (`machine.body.skeleton`) -
    MuJoCo's parent filter off, which takes a welded part for its segment, each segment kept from
    its parent by name, what touches standing or in the squat she lands in too - there her arms
    stood 3-8 mm in her knees' drums and boards, the kick felling her walk. Her room's loose things
    `free` (her world's), else welded where they stand (her law's body: their freedom none of hers)."""
    from machine.world.physics import SKELETON
    xml = _mjcf(SKELETON, free)
    kin = [(s[0], s[1]) for s in SEGMENTS if s[1]]
    if not SKELETON:
        return _apart(xml, kin)
    near = {s[0]: [s[1]] + [k[0] for k in SEGMENTS if k[1] == s[0]] for s in SEGMENTS}
    near.update({j: near[s[0]] + [s[0]] for s in SEGMENTS for j, _a, _s in s[2][:-1]})
    kin += [(part, other) for part, rides, _g in skeleton.bodies() for other in near[rides] if other]
    if xml not in _APART:
        import importlib
        model = importlib.import_module('mujoco').MjModel.from_xml_string(_apart(xml, kin))
        _APART[xml] = sorted(set(skeleton.overlapping(model, figure.stand()))
                             | set(skeleton.overlapping(model, figure.SQUAT))
                             | set(skeleton.overlapping(model, figure.squat((60.0, 30.0, 0.0, 80.0)))))
    return _apart(xml, kin + _APART[xml])


_APART = {}


def stops(model):
    """(lo, hi) of her joints' stops in `model`, rad, `JOINTS`' order: its ranges, unbounded where
    it has none - a pose laid within them (`physics.World.reset`): her fingers, held a fist at 80
    deg, laid at the stand's 18 snapped in at 893.5 deg/s each lay, her wrists at 69.1."""
    import numpy as np
    ids = [model.joint(j).id for j in JOINTS]
    return tuple(np.where(model.jnt_limited[ids, None] > 0, model.jnt_range[ids], [-np.inf, np.inf]).T)


def _apart(xml, pairs):
    return xml.replace('<contact/>', '<contact>%s</contact>' % ''.join(
        '<exclude body1="%s" body2="%s"/>' % pair for pair in pairs))


def _joint(joint, axis, sign):
    """A joint's element: its drive's armature, its stops, its spring or its gearbox's drag."""
    from machine.world.physics import BACKDRIVE, REFLECTED
    from machine.body.servos import SERVO
    stop, spring = drives.TRAVEL.get(kind(joint)), drives.passive(joint)
    if spring:
        stiffness, _damp, rest = spring
        stop = (rest - HELD_DEG, rest + HELD_DEG) if stiffness is None else stop
    axes = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}
    return '<joint name="%s" axis="%g %g %g" armature="%g"%s%s/>' % (
        (joint,) + tuple(sign * v for v in axes[axis]) + (
            PASSIVE_J if spring else
            SERVO[kind(joint)][3] + REFLECTED * (drives.armature(joint) - SERVO[kind(joint)][3]),
            ' limited="true" range="%g %g"' % stop if stop else '',
            ' stiffness="%g" springref="%g" damping="%g"' % (spring[0], spring[2], spring[1])
            if spring and spring[0] else
            ' frictionloss="%g"' % (BACKDRIVE * drives.backdrive(joint))
            if BACKDRIVE and not spring else ''))


def _mjcf(bones, free=True):
    from machine.eyes import gimbal, sight
    from machine.world.physics import CAMERA, PLACED, ROOM, SIGHT, STEP_S
    eyed = bool(CAMERA or SIGHT)
    from machine.body.servos import SERVO
    shells = build.segments() if build.SHELLS else {}
    kids, riders = {}, build.riders() if PLACED and not shells else {}
    for seg in SEGMENTS:
        kids.setdefault(seg[1], []).append(seg)
    give = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, SOLE_WIDTH_M)
    cloth = ' solref="%g %g" solimp="%g 0.95 %g"' % (SOLE_S, SOLE_DAMP, SOLE_SOFT, CLOTH_GIVE_M)
    contacts = ' contype="1" conaffinity="2"'
    parts = skeleton.bodies() if bones else []

    ridden = set(build.riders()) | {rides for _part, rides, _g in parts}
    held = '<inertial pos="%g %g %g" mass="%g" fullinertia="%g %g %g %g %g %g"/>'

    def carried(at):
        """The drives' and her skeleton's bodies riding `at`, welded."""
        out = ['<body name="%s_drive" pos="%g %g %g"><inertial pos="0 0 0" mass="%g" '
               'diaginertia="%g %g %g"/></body>' % ((joint,) + where + (kg,) + inertia)
               for joint, kg, where, inertia in riders.get(at, ())]
        return out + ['<body name="%s"><inertial pos="0 0 0" mass="1e-6" diaginertia="1e-10 '
                      '1e-10 1e-10"/>%s</body>' % (part, ''.join(geoms))
                      for part, rides, geoms in parts if rides == at]

    def body(seg):
        name, _parent, joints, offset, rest, share, com, gyr = seg
        h = math.radians(rest) / 2.0
        # A stage something rides, a body of its own: the joints before it on the one before.
        stages = [j for j, _a, _s in joints[:-1] if j in ridden]
        out = ['<body name="%s" pos="%g %g %g" quat="%g 0 0 %g">' % (
            ((stages or [name])[0],) + tuple(offset) + (math.cos(h), math.sin(h)))]
        if seg[1] is None:
            out.append('<freejoint name="root"/>')
        for joint, axis, sign in joints:
            out.append(_joint(joint, axis, sign))
            if joint in stages:
                k = stages.index(joint) + 1
                out += ([held % (shells[joint][1] + (shells[joint][0],) + shells[joint][2])
                         if joint in shells else
                         '<inertial pos="0 0 0" mass="1e-6" diaginertia="1e-10 1e-10 1e-10"/>']
                        + carried(joint)
                        + ['<body name="%s">' % (stages[k] if k < len(stages) else name)])
        # A kid's stages' drives come out of this segment's share: they rode it unstaged.
        staged = [j for kid in kids.get(name, []) for j, _a, _s in kid[2][:-1]]
        mass = share * MASS_KG - sum(kg for at in [name] + staged
                                     for _j, kg, _at, _i in riders.get(at, ()))
        out.append('<inertial pos="%g %g %g" mass="%g" fullinertia="%g %g %g %g %g %g"/>' % (
            shells[name][1] + (shells[name][0],) + shells[name][2]) if shells else
                   '<inertial pos="%g %g %g" mass="%g" diaginertia="%g %g %g"/>' % (
            tuple(com) + (mass,) + tuple(mass * g * g for g in gyr)))
        if name == 'head' and eyed:
            out += gimbal.cameras()
        out += carried(name)
        grip = ' contype="%d" conaffinity="%d" condim="4" friction="%%g %g 0.001"' % (
            ME, MEETS, TORSION_M)
        for part, shape, size, at in undersides():
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
                      ((1.0 if name.startswith('left') else -1.0),) if name.endswith('_' + part) else ()):
                at = figure.pad(axis, radius, toward, size, x)
                shape = ('type="capsule" size="%g" fromto="%g %g %g %g %g %g"' % (
                    (size, at[0] - wide) + at[1:] + (at[0] + wide,) + at[1:]) if wide else
                         'type="sphere" size="%g" pos="%g %g %g"' % ((size,) + at))
                out.append('<geom name="%s_pad%d%s" %s%s priority="1" solref="%g %g" solimp="%g 0.95 %g"/>' % (
                    name, k, '' if x > 0 else 'r', shape, grip % CLOTH.get(part, FRICTION),
                    PAD_S, PAD_DAMP, PAD_SOFT, figure.PAD_M))
        if name.endswith('_shank'):
            side = name[:-len('_shank')]
            hem_gyr2 = round(0.5 * HEM_M ** 2, 4)
            out += ['<body name="%s_hem" pos="0 %g 0">' % (side, -HEM_AT)] + [
                '<joint name="%s_hem_%s" axis="%s" stiffness="%g" damping="%g" armature="0" limited="true" '
                'range="%g %g" solreflimit="%g 1"/>' % (side, ax, d, HEM_K, HEM_D, lo, hi, HEM_STOP_S)
                for ax, d, lo, hi in (('x', '1 0 0', -HEM_FORE_DEG, HEM_BACK_DEG),
                                      ('z', '0 0 1', -HEM_SIDE_DEG, HEM_SIDE_DEG))] + [
                '<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                    -HEM_M, HEM_KG, hem_gyr2 * HEM_KG, hem_gyr2 * HEM_KG, hem_gyr2 * HEM_KG), '</body>']
        if name == 'head':
            hair_gyr2 = round(0.5 * HAIR_M ** 2, 4)
            out += ['<body name="hair" pos="%g %g %g">' % HAIR_AT] + [
                '<joint name="hair_%s" axis="%s" stiffness="%g" damping="%g" armature="0" limited="true" '
                'range="%g %g"/>' % (ax, d, HAIR_K, HAIR_D, lo, hi)
                for ax, d, lo, hi in (('x', '1 0 0', -HAIR_IN_DEG, HAIR_DEG),
                                      ('z', '0 0 1', -HAIR_SIDE_DEG, HAIR_SIDE_DEG))] + [
                '<inertial pos="0 %g 0" mass="%g" diaginertia="%g %g %g"/>' % (
                    -HAIR_M, HAIR_KG, hair_gyr2 * HAIR_KG, hair_gyr2 * HAIR_KG, hair_gyr2 * HAIR_KG), '</body>']
        for kid in kids.get(name, []):
            out += body(kid)
        return out + ['</body>'] * (1 + len(stages))

    # her constraints' Jacobian dense, as MuJoCo's auto laid it for her alone: her room's loose things' 36 dofs took
    # her past its 60 to sparse, her step's constraint solve 34 us where 28 - asleep, they take no row
    return '\n'.join(
        ['<mujoco model="gynoid">',
         '<option timestep="%g" gravity="0 %g 0" integrator="implicitfast" cone="%s" jacobian="dense" '
         'impratio="%g"%s' % (STEP_S, -_GRAVITY, CONE, IMPRATIO,
                              '><flag filterparent="disable"/></option>' if bones else '/>'),
         '<default><joint damping="%g"/><geom contype="0" conaffinity="0"/></default>' % _JOINT_DAMP,
         ] + (sight.assets() if eyed else []) + [
         '<worldbody>',
         ] + (sight.lights() if eyed else []) + floor.ground(contacts, give, TORSION_M)
        + body(SEGMENTS[0])
        + floor.rug(give, FRICTION, TORSION_M)
        + (room.furnish(contacts, textured=eyed, free=free) if ROOM else [])
        + ['</worldbody>', '<contact/>', '<actuator>']
        + ['<motor joint="%s" ctrlrange="%g %g"/>' % (j, -max(SERVO[kind(j)][0], drives.peak(j)),
                                                      max(SERVO[kind(j)][0], drives.peak(j)))
           for j in JOINTS]
        + ['</actuator>', '<tendon>',
           '<spatial name="lace" limited="true" range="0 10" solreflimit="%g 1" width="0.002">'
           '<site site="lace_left"/><site site="lace_right"/></spatial>' % LACE_S,
           '</tendon>'] + _stand(STAND, STAND_M) + ['</mujoco>'])


def _stand(stand, high):
    """Her pelvis welded to the world `high` m up, where she stands for identification."""
    return (['<equality><weld body1="pelvis" relpose="0 %g 0 1 0 0 0" solref="%g 1"/></equality>'
             % (-high, _STAND_SOLREF_S)] if stand else [])
