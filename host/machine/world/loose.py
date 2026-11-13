"""Her room's loose things: free bodies on the floor, held there by their feet's grip alone.

    xml = loose.body('table', True, (x, 0.0, z), geoms)  # its free body (joint FREE + name) on the floor, else welded
    pairs = loose.things(model)                    # [(body, joint)] of the model's loose things
    things = loose.free(model)                     # each loose thing's joint and its feet, once a model
    loose.home(model, data, things)                # each where her world stands it, asleep (her world laid anew)
    loose.stick(model, data, things)               # a pass: each foot's grip as its thing slides, still ones asleep

A thing's feet are its geoms that stand on the floor where her world stands it; a foot grips as GRIPS says of what
it is - classic stiction and friction against an ordinary wooden floor, at rest and sliding, MuJoCo's one
coefficient switched as its thing slides along the floor past SLIDE_M_S and stops under half that. Its geoms meet
the floor, the room's still things and her (MEETS, as her rug's): with the floor's own bits a chair met none of the
floor and fell through. Stumbled into, a chair, the table, the bed or the lamp moves; sat on carelessly, a chair.
"""

import importlib
import math

#: What a foot is and its grip on a wooden floor, (at rest, sliding): wood on wood; a lamp's felted foot.
GRIPS = {'wood': (0.45, 0.35), 'felt': (0.30, 0.20)}
#: A thing sliding past this along the floor slides, m/s; under half of it it is stuck again.
SLIDE_M_S = 0.01
#: A thing still this long, s - under half SLIDE_M_S every way -, touched by nothing that moves (the world's still
#: geoms alone, things asleep), sleeps: its geoms meet by the world's type alone, none of the world's meeting them -
#: her and things awake still do, as they meet the world -, its weight borne at its centre of mass, MuJoCo solving
#: none of it (24 glides standing were its step's 130 us where 79; a chair shoved over lay on its back on the floor,
#: awake), held where it fell asleep till it moves - pushed: nothing else moves a thing asleep. Laid
#: in her world asleep, at rest as it was built: laid awake, it settled on its glides after her eyes first saw it, and
#: her registry told it anew each look - its estimate the mean of them -, her planner and her peeks with it; its weight
#: borne alone, the two forces' last bits turned it 1e-34 a step, her room's classes told anew each look (1.8 ms).
#: MuJoCo's own sleep (`mjENBL_SLEEP`) poses the world's geoms once: her floor's slabs moved, a chair fell through.
SLEEP_S = 1.0
#: A loose thing's free joint's name begins so; what its geoms meet.
FREE, MEETS = 'free_', ' contype="3" conaffinity="3"'


def foot(what):
    """A foot's grip, of what it is (GRIPS) at rest: its friction over the floor's (`priority`), MuJoCo's torsion
    and roll."""
    return ' priority="2" friction="%g 0.005 0.0001"' % GRIPS[what][0]


def body(name, free, at, geoms):
    """A loose thing's body `name` at `at` (x, y, z): `free` (its joint FREE + `name`, her world's), else welded where
    it stands (her law's body); `geoms` its geoms' XML, posed in it."""
    joint = '<freejoint name="%s%s"/>' % (FREE, name) if free else ''
    return '<body name="%s" pos="%g %g %g">%s%s</body>' % ((name,) + tuple(at) + (joint, ''.join(geoms)))


def things(m):
    """[(body, its joint)] of model `m`'s loose things."""
    return [(b, m.body_jntadr[b]) for b in range(m.nbody)
            if m.body_jntadr[b] >= 0 and m.joint(m.body_jntadr[b]).name.startswith(FREE)]


def free(m):
    """(each loose thing's joint's first dof [n], its first coordinate (n,), its feet (k,), each foot's thing (k,),
    each foot's grip at rest and sliding (k, 2), its body (n,), each thing's (feet, at rest, sliding, its geoms, what
    they meet by type and affinity) [n], whether each slides, sleeps, since when still and where it lies asleep [n],
    the world's type, those asleep as `stick` holds them (`_held`)) of model `m`: a foot a geom of it standing on the
    floor where her world stands it - a glide's sphere its radius over it -, of a grip GRIPS knows."""
    import numpy as np
    mujoco = importlib.import_module('mujoco')
    dofs, at, feet, of, grips, bodies = [], [], [], [], [], []
    rest, sphere = {round(s, 6): (s, k) for s, k in GRIPS.values()}, mujoco.mjtGeom.mjGEOM_SPHERE
    for b, j in things(m):
        for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
            half = m.geom_size[g][0] if m.geom_type[g] == sphere else m.geom_size[g][1]
            low = m.body_pos[b][1] + m.geom_pos[g][1] - half
            grip = rest.get(round(float(m.geom_friction[g][0]), 6))
            if abs(low) < 1e-6 and grip is not None:
                feet.append(g)
                of.append(len(dofs))
                grips.append(grip)
        dofs.append(int(m.jnt_dofadr[j]))
        at.append(int(m.jnt_qposadr[j]))
        bodies.append(b)
    feet, of, grips = np.array(feet, int), np.array(of, int), np.array(grips, float).reshape(-1, 2)
    owned = [np.arange(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]) for b in bodies]
    per = [(feet[of == k], grips[of == k, 0], grips[of == k, 1], owned[k], m.geom_contype[owned[k]].copy(),
            m.geom_conaffinity[owned[k]].copy()) for k in range(len(dofs))]
    world = int(np.bitwise_or.reduce(m.geom_contype[m.geom_bodyid == 0], initial=0))
    return (dofs, np.array(at, int), feet, of, grips, np.array(bodies, int), per,
            [[False, False, 0.0, None] for _ in dofs], world, [np.zeros(0, int)] * 3 + [np.zeros(0), np.zeros(0, int)])


def home(m, d, things):
    """Each loose thing (`free`'s `things`) where her world stands it - its body's pose in the model, as a scene or a
    look round moves it -, its feet stuck, asleep: her world laid anew zeroes every coordinate, and the chairs stood
    in her where she starts; from the model's first coordinates a bed moved in it went back to where it was built."""
    mujoco = importlib.import_module('mujoco')
    _dofs, at, feet, _of, grips, bodies, _per, state, _world, _held = things
    for q, b in zip(at, bodies):
        d.qpos[q:q + 3], d.qpos[q + 3:q + 7] = m.body_pos[b], m.body_quat[b]
    m.geom_friction[feet, 0] = grips[:, 0]
    if state:
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
    for k, s in enumerate(state):
        s[0], s[2] = False, d.time
        _asleep(m, d, things, k)


def stick(m, d, things):
    """A pass of her loose things (`free`'s `things`): each foot's grip sliding while its thing slides along the floor
    (past SLIDE_M_S), at rest once it has stopped (under half that), as it was between - written as a thing goes from
    one to the other alone (its six things' arrays each pass were 24 us of it) -; a thing still SLEEP_S, touched by
    nothing that moves, asleep (`_alone`); one asleep that moves woken, the others held where they lie, still - all
    at once (`_held`): one by one, six asleep were 16 us of each pass."""
    dofs, _at, _feet, _of, _grips, _bodies, per, state, _world, held = things
    qp, qv, t, low = d.qpos, d.qvel, d.time, 0.5 * SLIDE_M_S
    if len(held[4]):
        moved = abs(qv[held[0]]) > low
        if moved.any():
            for k in sorted({int(held[4][i // 3]) for i in moved.nonzero()[0]}):
                _woken(m, d, things, k)
                state[k][2] = t
        qp[held[1]], qv[held[2]] = held[3], 0.0
    for k, v in enumerate(dofs):
        s = state[k]
        if s[1]:
            continue
        speed = math.hypot(qv[v], qv[v + 2])
        now = speed > SLIDE_M_S or (s[0] and speed >= low)
        if now != s[0]:
            s[0] = now
            m.geom_friction[per[k][0], 0] = per[k][2] if now else per[k][1]
        if speed >= low or abs(qv[v + 1]) >= low:
            s[2] = t
        elif t - s[2] >= SLEEP_S:
            if _alone(m, d, things, k):
                _asleep(m, d, things, k)
            s[2] = t


def _alone(m, d, things, k):
    """Whether nothing that moves touches thing `k`: what touches it the world's still geoms alone, or things asleep -
    her, a thing awake or the door touching it, it stays awake."""
    import numpy as np
    b, n, bodies = things[5][k], d.ncon, things[5]
    b1, b2 = m.geom_bodyid[d.contact.geom1[:n]], m.geom_bodyid[d.contact.geom2[:n]]
    other = np.concatenate([b2[b1 == b], b1[b2 == b]])
    asleep = bodies[[s[1] for s in things[7]]]
    return bool(((m.body_weldid[other] == 0) | np.isin(other, asleep)).all())


def _asleep(m, d, things, k):
    """Thing `k` asleep where it lies: its geoms meeting by the world's type alone - none of the world's meeting them,
    her and things awake as they meet the world -, its weight borne at its centre of mass in its own coordinates
    (`mj_applyFT` into `qfrc_applied`, which her world never clears: `xfrc_applied` it clears at a reset and a shove),
    still."""
    import numpy as np
    mujoco = importlib.import_module('mujoco')
    dof, q, b, geoms = things[0][k], things[1][k], things[5][k], things[6][k][3]
    m.geom_contype[geoms], m.geom_conaffinity[geoms] = things[8], 0
    borne = np.zeros(m.nv)
    mujoco.mj_applyFT(m, d, -m.opt.gravity * m.body_subtreemass[b], np.zeros(3), d.xipos[b], int(b), borne)
    d.qfrc_applied[dof:dof + 6] = borne[dof:dof + 6]
    d.qvel[dof:dof + 6] = 0.0
    things[7][k][1], things[7][k][3] = True, d.qpos[q:q + 7].copy()
    _held(things)


def _woken(m, d, things, k):
    """Thing `k` awake: its geoms meeting all they met, its weight its own."""
    dof, (_feet, _rest, _slide, geoms, contype, conaffinity) = things[0][k], things[6][k]
    m.geom_contype[geoms], m.geom_conaffinity[geoms] = contype, conaffinity
    d.qfrc_applied[dof:dof + 6] = 0.0
    things[7][k][1] = False
    _held(things)


def _held(things):
    """The things asleep (`free`'s `things`) laid as `stick` holds them, at once: their velocities along the floor
    and up, their coordinates, their velocities, where each lies, which they are."""
    import numpy as np
    dofs, at, state = things[0], things[1], things[7]
    ks = [k for k, s in enumerate(state) if s[1]]
    things[9][:] = [np.array([dofs[k] + j for k in ks for j in range(3)], int),
                    np.array([at[k] + j for k in ks for j in range(7)], int),
                    np.array([dofs[k] + j for k in ks for j in range(6)], int),
                    np.concatenate([state[k][3] for k in ks]) if ks else np.zeros(0), np.array(ks, int)]
