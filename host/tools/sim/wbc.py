#!/usr/bin/env python3
"""Her stand on the whole-body stack (`machine.stack.wbc`) in a world without buses: still, and shoved.

    python tools/sim/wbc.py                      # 10 s still: drift, pressure, effort, its cost
    python tools/sim/wbc.py --polar 60 80 100    # a shove from each of 8 ways a force, the relay
    python tools/sim/wbc.py --one 80 90 -v       # one: N and way (0 her left, 90 ahead), traced
    ... --no-wep                                 # no war emergency power
    ... --core | --wrench                        # the loop in C | the step without an active set
    ... --core --wrench                          # that step in C (wbc_wrench.c)

A shove is `sprung.SHOVE_S` on the trunk at AT_S; she has stood if to the end her pelvis keeps
within HELD_DEG of upright and nothing but her soles touches the floor. Her ask is the capture
point's law (`law`) or a caller's; her steps are her latent loop's (`machine.process.sprung`).
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Shoved at AT_S, judged TO_S later; fallen past HELD_DEG of the pelvis's tilt; a polar's
#: spread of shoves SPREAD_S apart.
AT_S, TO_S, HELD_DEG, SPREAD_S = 1.0, 3.0, 25.0, 0.035

#: The stand's law: the ZMP asked XI_K/omega past the capture point, away from
#: the soles' middle.
XI_K = 2.0

#: The drives' disturbance observers' bandwidth, rad/s (`--inner eso`, `--eso-wo`).
ESO_WO = 100.0

#: The centre of mass's creep along the floor, mm/s, over a stand's last CREEP_S.
CREEP_S = 2.0

#: The ways a polar shoves from, deg: 0 her left, 90 ahead.
WAYS = tuple(range(0, 360, 45))

#: The drives' PD round the stack's reference and war emergency power, as the loop has them.
from machine.process.sprung import LEAK_S, SERVO_SHARE, STANDING_KP, WEP_HOLD_S, WEP_S

NAMES = ()


def law(s, st, mid):
    """The stack's ask standing: the capture point brought to `mid` (x, z)."""
    import numpy as np
    from machine.stack import wbc
    c, v = s['com'], s['vcom']
    omega = math.sqrt(9.81 / max(0.3, c[1]))
    xi = c[[0, 2]] + v[[0, 2]] / omega
    p = xi + XI_K / omega * (xi - mid)
    acc = omega ** 2 * (c[[0, 2]] - p)
    return {'stance': (True, True), 'posture': st['posture'], 'turns': st['turns'],
            'com_acc': np.array([acc[0], wbc.HEIGHT_KD[0] * (st['height'] - c[1])
                                 - wbc.HEIGHT_KD[1] * v[1], acc[1]])}


def state(s):
    """Standing where she stands: her height, her posture and her turns kept."""
    return {'height': float(s['com'][1]), 'posture': s['q'].copy(),
            'turns': tuple(t['quat'].copy() for t in s['turns'])}


def loads(world, force):
    """Each sole's load now, N: the floor's contacts on its foot and toes, unfiltered."""
    m, d, out = world.model, world.data, [0.0, 0.0]
    if d.ncon:
        on = world.soles[m.geom_bodyid[d.contact.geom]].any(axis=1)
        for side in (0, 1):
            for i in on[:, side].nonzero()[0].tolist():
                world._mj.mj_contactForce(m, d, i, force)
                out[side] += float(force[0])
    return out


def touched(world, ours, hers):
    """The first of her bodies but her soles (`ours`) that the floor or a thing on it touches -
    anything not hers (`hers`) -, or None; her parts on each other are not counted."""
    d, m = world.data, world.model
    for a, b in d.contact.geom[:d.ncon].tolist():
        one, two = int(m.geom_bodyid[a]), int(m.geom_bodyid[b])
        for mine, other in ((one, two), (two, one)):
            if mine in ours and other not in hers:
                return m.body(mine).name
    return None


def _under(m, body, root):
    while body:
        if body == root:
            return True
        body = int(m.body_parentid[body])
    return False


def stand(push=0.0, way=0.0, seconds=None, trace=False, wep=True, at=AT_S,
          core=False, inner='pd', asked=None, wrench=False):
    """{stood, tilt deg, drift m, top share, us a step, wep s, ..} standing `seconds` (TO_S
    past the shove), shoved `push` N from `way` deg at `at`; `wep` war emergency power granted;
    `core` the
    loop in C, `wrench` the step without an active set (`machine.stack.wrench`), in the python
    stack's place; `inner` the drives' own loop: 'pd' round a reference integrated from the
    stack's accelerations, 'eso' each drive's disturbance observed (`parts.Eso` at ESO_WO) and
    taken off its torque, 'none' the torque as asked; `asked` a law of the caller's in the
    capture point's place: (st, s, loads, now, bears, body) -> the stack's ask."""
    import numpy as np
    from machine.body import figure
    from machine.rig import parts
    from machine.world import physics
    from machine.stack import wbc, wrench as bare
    from machine.errors import MachineError
    from machine.body.drives import kind
    from machine.body.figure import JOINTS, SEGMENTS
    from tools.cores.wbc import shared, stack_step, wrench_step
    from tools.sim.look import LEG_KINDS
    global NAMES
    if core:
        shared().reset()
    world, body = physics.World(), wbc.Body()
    NAMES = [j for j, d in zip(JOINTS, body.driven) if d]
    pose = figure.stand()
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    low = min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(pose, where=(0.0, 1.0 - low, 0.0))
    world.gains[:] *= SERVO_SHARE if inner == 'pd' else 0.0
    world.gains[~body.driven] = 0.0
    free_kp = world.gains[:, 0].copy()
    legs = [[i for i, j in enumerate(JOINTS) if j.startswith(side) and kind(j) in LEG_KINDS]
            for side in ('left_', 'right_')]
    clamp, joints = world.limit.copy(), np.flatnonzero(body.driven)
    hold, left = np.zeros(len(body.clamp)), WEP_S if wep else 0.0
    ref_q = world.data.qpos[body.qact].copy()
    ref_v = np.zeros(len(ref_q))
    now = [world.data.time]
    world.clock = lambda: now[0]
    m = world.model
    hers = {i for i in range(m.nbody) if _under(m, i, m.body('pelvis').id)}
    ours = hers - {m.body(seg[0]).id for seg in SEGMENTS if seg[0].endswith(('_foot', '_toes'))}
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    home = s['com'].copy()
    esos = [parts.Eso(float(s['M'][d, d]), ESO_WO) for d in body.act] if inner == 'eso' else []
    mid = (s['feet'][0]['sole'][[0, 2]] + s['feet'][1]['sole'][[0, 2]]) / 2.0
    st = state(s)
    pushed, tilt, drift, top, cost, passes, worst = False, 0.0, 0.0, 0.0, 0.0, 0, 0.0
    failed, down, force, bears = None, None, np.zeros(6), (0.0, 0.0)
    a = math.radians(way)
    seconds = at + TO_S if seconds is None else seconds
    settled = None
    guarded = [0, 0, 0, 0, 0]
    while now[0] < seconds:
        t0 = time.perf_counter()
        s = wbc.sense(body, world.data.qpos, world.data.qvel)
        ask = asked(st, s, loads(world, force), now[0], bears, body) if asked else law(s, st, mid)
        try:
            out = ((wrench_step if core else bare.step) if wrench else stack_step if core else wbc.step)(
                body, s, dict(ask, wep=left > 0.0))
        except MachineError as exc:
            failed = '%s at %.3f s' % (exc, now[0])
            break
        cost += time.perf_counter() - t0
        passes += 1
        world.feed[body.driven] = _torques(esos, world, body, out)
        guarded = [a + b for a, b in zip(guarded, out.get('guarded', guarded))]
        bears = out['bears']
        for k, leg in enumerate(legs):
            world.gains[leg, 0] = free_kp[leg] * (STANDING_KP if ask['stance'][k] else 1.0)
        hold = np.where(out['over'], WEP_HOLD_S, np.maximum(0.0, hold - 0.001))
        left -= 0.001 * bool(hold.any())
        world.limit[joints] = np.where(hold > 0.0, body.wep, clamp[joints])
        leak = 0.001 / LEAK_S
        ref_v += out['qacc'][body.act] * 0.001 + leak * (s['v'][body.act] - ref_v)
        ref_q += ref_v * 0.001 + leak * (s['q'] - ref_q)
        world.target[body.driven] = ref_q
        top = max(top, float(np.abs(out['tau'] / body.top).max()))
        worst = max([worst] + [float(sl.max()) for sl in out['slack'] if len(sl)])
        if push and not pushed and now[0] >= at:
            world.push((push * math.cos(a), 0.0, push * math.sin(a)), 0.12)
            pushed = True
        now[0] += 0.001
        world.advance()
        q = world.data.qpos[3:7]
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q[1] ** 2
                                                                              + q[3] ** 2))))))
        drift = max(drift, float(np.linalg.norm((s['com'] - home)[[0, 2]])))
        if settled is None and now[0] >= seconds - CREEP_S:
            settled = s['com'][[0, 2]].copy()
        down = down or touched(world, ours, hers)
        if trace and passes % 25 == 0:
            _trace(now[0], '-', s, home, tilt, out, body, worst)
        if tilt > HELD_DEG or s['com'][1] < 0.5 * home[1] or down:
            break
    creep = (float(np.linalg.norm(s['com'][[0, 2]] - settled)) / CREEP_S if settled is not None
             else None)
    return {'push': push, 'way': way, 'stood': not failed and not down and tilt <= HELD_DEG,
            'failed': failed, 'tilt': round(tilt, 1), 'drift_mm': round(1e3 * drift, 1),
            'creep_mm_s': None if creep is None else round(1e3 * creep, 2),
            'disturbance_nm': round(float(np.mean([abs(e.j * e.d) for e in esos])), 2) if esos
            else None, 'guarded': guarded,
            'top': round(top, 2), 'us': round(1e6 * cost / max(1, passes)), 'slack': worst,
            't': round(now[0], 3), 'wep_s': round(WEP_S - left, 3) if wep else 0.0, 'down': down}


def _trace(now, phase, s, home, tilt, out, body, worst):
    """A row of the stand's trace: where she is, the soles' centres of pressure, the drive
    nearest its clamp, the stack's worst slack, and the loop's bounds at their edge."""
    import numpy as np
    share = np.abs(out['tau'] / body.top)
    edge = ('  edge %s over %.2g changes %d stuck %d levels %s ids %s' % (
        out['guarded'], out['residual'], out['passes'], out['stuck'],
        ['%.2g' % a for a in out['alpha']], out['ids'][:14])
            if 'guarded' in out else '')
    print('%5.3f %-6s com %+6.1f %+6.1f mm  tilt %4.1f  cop %s  tau %.2f %s  slack %.2g%s'
          % (now, phase, 1e3 * (s['com'][0] - home[0]), 1e3 * (s['com'][2] - home[2]), tilt,
             ' '.join('(%+.0f %+.0f)' % tuple(1e3 * c) if c is not None else '-'
                      for c in out['cop']),
             float(share.max()), NAMES[int(share.argmax())], worst, edge))


def _torques(esos, world, body, out):
    """The drives' torques this pass: the stack's, less what each observer sees of its drive's
    own disturbance; the inertia each observer scales by the loop's own, where it gives it. An
    observer reads its board's encoder (`physics.WOUND`): on the joint it took the gearbox's
    play for 84 N m of disturbance and she fell standing."""
    import numpy as np
    from machine.world import physics
    if not esos:
        return out['tau']
    seen = world.data.qpos[body.qact] + physics.WOUND * world.boxes.delta[body.driven]
    planned, jeff = out['qacc'][body.act], out.get('jeff')
    taken = []
    for k, e in enumerate(esos):
        if jeff is not None:
            e.j = float(jeff[k])
        taken.append(e.step(0.001, measured=float(seen[k]), command=float(planned[k]))['estimate'])
    return out['tau'] - np.array(taken)


def polar(forces, wep=True, spread=1, core=False, inner='pd', wrench=False):
    """A shove from each of WAYS at each of `forces`, `spread` moments apart, a relay job each:
    {N: [stood way ..]}."""
    from tools.dev import focus
    flags = (([] if wep else ['--no-wep'])
             + (['--core'] if core else []) + (['--wrench'] if wrench else []) + ['--inner', inner])
    jobs = [focus.Job('%g@%d.%d' % (n, w, i), [sys.executable, '-X', 'utf8',
                                               os.path.abspath(__file__), '--one', str(n), str(w),
                                               '--at', str(AT_S + SPREAD_S * i)] + flags,
                      0.6, 900.0, None)
            for n in forces for w in WAYS for i in range(spread)]
    rows = []
    for _job, text, _code, _s in focus.relay(jobs):
        line = next((ln for ln in reversed(text.splitlines()) if ln.startswith('{')), None)
        if line:
            rows.append(json.loads(line))
    for n in forces:
        mine = sorted((r for r in rows if r['push'] == n), key=lambda r: r['way'])
        print('%4g N  %d of %d stood, WEP %.1f s   %s' % (
            n, sum(r['stood'] for r in mine), len(mine), sum(r['wep_s'] for r in mine), '  '.join(
                '%d:%s' % (w, ''.join('u' if r['stood'] else 'F' if r['failed'] else '.'
                                      for r in mine if r['way'] == w)) for w in WAYS)))
    return rows


def main(argv=None):
    global ESO_WO
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--polar', nargs='*', type=float, metavar='N')
    parser.add_argument('--one', nargs=2, type=float, metavar=('N', 'WAY'))
    parser.add_argument('--still', type=float, default=10.0, metavar='S')
    parser.add_argument('--at', type=float, default=AT_S, metavar='S', help='when the shove comes')
    parser.add_argument('--spread', type=int, default=1, help="a polar's shoves at each way")
    parser.add_argument('-v', action='store_true', help='a row every 25 ms')
    parser.add_argument('--no-wep', action='store_true', help='no war emergency power')
    parser.add_argument('--core', action='store_true', help='the loop in C, not the python stack')
    parser.add_argument('--wrench', action='store_true',
                        help='the step without an active set (machine.stack.wrench), not the stack')
    parser.add_argument('--rigid', action='store_true', help='the gearboxes rigid (`physics.BOXED`)')
    parser.add_argument('--inner', default='pd', choices=('pd', 'eso', 'none'),
                        help="the drives' own loop")
    parser.add_argument('--eso-wo', type=float, default=ESO_WO, help="the observers' rad/s")
    args = parser.parse_args(argv)
    ESO_WO = args.eso_wo
    from machine.world import physics
    physics.BOXED = 0.0 if args.rigid else 1.0
    flags = {'wep': not args.no_wep, 'core': args.core, 'inner': args.inner, 'wrench': args.wrench}
    if args.polar:
        polar(args.polar, spread=args.spread, **flags)
    elif args.one:
        print(json.dumps(stand(args.one[0], args.one[1], trace=args.v, at=args.at, **flags)))
    else:
        print(json.dumps(stand(seconds=args.still, trace=args.v, **flags)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
