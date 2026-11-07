"""Her state as her page hears it: her stage, modes, horizon and things; her tilt as the world has it.

    now = page.state(g, torque, power, watts, ratio, fallen_at)     # `sprung.state`: what her page is said
    page.tilt(g), page.staged(g, fallen_at)                          # deg off upright; fallen, air, walk, stand
"""
import math

from machine.eyes import classes
from machine.law.se2 import doing
from machine.moves import leap
from machine.senses import situation
from machine.texts import ortho

#: Her stage (`staged`): walking past WALK_M_S, or with a sole off the floor within STEP_S; fallen, she lies
#: RECOVER_S before a new epoch is laid (`worker`).
WALK_M_S, STEP_S, RECOVER_S = 0.1, 1.0, 3.0


def named(g):
    """Her things by what each is, the one thing each is of and the buildings they make (`classes.read`), told anew only
    as they change: the page names none - 59 things took 12 ms a scan there; keyed by `registry.keyed`."""
    from machine.eyes import registry
    things = g['things']
    if g.get('_named_of') is not things:     # the same list her things are as last keyed: nothing writes into them
        key = registry.keyed(things)
        if g.get('_named', (None,))[0] != key:
            g['_named'] = (key,) + classes.read(things)
        g['_named_of'] = things
    return g['_named'][1:]


def tilt(g):
    """Her pelvis's tilt from upright, deg, as the world has it: the referee's, not her law's."""
    q = g['world'].data.qpos
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[4] ** 2 + q[6] ** 2)))))


def staged(g, fallen_at):
    """Her stage as measured: fallen; in the air, neither sole bearing `leap.TOUCH_N`; walking,
    her pelvis faster than WALK_M_S or a sole off the floor within STEP_S; else standing. Read off
    her gait's amplitude it said stand at 0.9 m/s."""
    if fallen_at is not None:
        return 'fallen'
    if max(g['loads']) < leap.TOUCH_N:
        return 'air'
    d = g['world'].data
    going = math.hypot(float(d.qvel[0]), float(d.qvel[2])) > WALK_M_S
    return 'walk' if going or g['t'] - g.get('lifted', -math.inf) < STEP_S else 'stand'


def state(g, torque, power, watts, ratio, fallen_at, says=(), refused=None, headroom=1.0):
    """Her state as the page hears it: stage, modes, joint torque/power/temp, physical velocity, and
    her law's horizon - her centre of mass, her capture point, her soles, which stand (world),
    her shoulders' range; `ratio` her pace to the wall, `headroom` her process's flat out."""
    import numpy as np
    from machine.law import latent, modal
    from machine.body.figure import JOINTS
    world, out, d, hd = g['world'], g['out'], g['world'].data, float(g['heading'])
    # her modes compiled (`motion.laid`'s g['modes']): numpy's read was 204 us of the 607 a state is said in
    z = g.get('modes', modal.read)(g['y'], g['fr'], g['a'][modal.STANCE])[0]
    # her angles at once: numpy's degrees math.degrees' product by 180/pi, each joint's read and turned were 17 us
    angles = dict(zip(JOINTS, np.degrees(d.qpos[world.qadr[:len(JOINTS)]]).tolist()))
    shoulder = g.get('_shoulder_at')
    if shoulder is None:
        shoulder = g['_shoulder_at'] = [j for j, dv in zip(JOINTS, g['body'].driven) if dv].index('left_shoulder')
    k = len(ortho.MODES)
    vxz = (float(d.qvel[0]), float(d.qvel[2]))
    y, om = g['y'], float(g['fr'][modal.OMEGA])
    horizon = {'com': tuple(y[modal.COM].tolist()), 'xi': (float(y[modal.COM][0] + y[modal.VCOM][0] / om),
                                                           float(y[modal.COM][2] + y[modal.VCOM][2] / om)),
               'soles': tuple(tuple(s) for s in y[modal.SOLE].reshape(2, 3).tolist()),
                'stance': tuple(g['a'][modal.STANCE].tolist()),
                'shoulder': tuple(min(max(float(v), -math.pi), math.pi) for v in g['rng'][shoulder])}
    buses = {b.name: b.stats() for b in (getattr(world, 'limb_buses', None) or ())}
    props = world.props()
    return {'t': g['t'], 'angles': dict(angles, **world.loose()), 'set': angles,
            'torque': dict(zip(JOINTS, torque.tolist())), 'power': dict(zip(JOINTS, power.tolist())),
            'temp': dict(zip(JOINTS, g['temp'].tolist())), 'peak': dict(zip(JOINTS, world.peak.tolist())),
            'watts': float(watts), 'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
            'heading': hd, 'twist': tuple(g.get('twist', (0.0, 0.0, 0.0))),
            'twist_actual': tuple(g.get('twist_actual', (0.0, 0.0, 0.0))),
            'speed': float(vxz[0] * math.sin(hd) + vxz[1] * math.cos(hd)), 'velocity': vxz,
            'phase': float(g['st'][latent.THETA]) / math.tau, 'cadence': float(out[latent.TURNS]) / math.tau,
            'stage': staged(g, fallen_at),
            'fallen': fallen_at is not None, 'loads': tuple(g['loads']), 'props': props, 'rounds': world.rounds(props), 'dents': world.dents(),
            'seen': g['keep'].view() if g['keep'] is not None else None, 'things': g['things'], **dict(zip(('named', 'groups', 'buildings'), named(g))),
            'doing': doing(g), 'entry': g.get('active_entry'),
            'recover': None if fallen_at is None else RECOVER_S - (g['t'] - fallen_at),
            'ratio': min(ratio, 99.0), 'headroom': min(headroom, 99.0), 'says': tuple(says), 'refused': refused,
            'buses': buses,
            'scenario': g.get('scenario', 'flat'), 'situation': situation.told(g['situations'], g['into']), 'horizon': horizon,
            'modes': list(zip(ortho.MODES, out[latent.REF][:k].tolist(), np.asarray(z)[:k].tolist(),
                              out[latent.KS][:k].tolist(), out[latent.DS][:k].tolist()))}
