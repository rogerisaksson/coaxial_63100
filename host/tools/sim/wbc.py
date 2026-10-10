#!/usr/bin/env python3
"""Her stand on the whole-body stack (`machine.wbc`) in a world without buses: still, and shoved.

    python tools/sim/wbc.py                      # 10 s still: drift, pressure, effort, its cost
    python tools/sim/wbc.py --polar 60 80 100    # a shove from each of 8 ways a force, the relay
    python tools/sim/wbc.py --one 80 90 -v       # one: N and way (0 her left, 90 ahead), traced
    ... --no-step                                # the capture point's law alone, no MPC, no step
    ... --no-wep                                 # no war emergency power

A shove is `events.SHOVE_S` on the trunk at AT_S; she has stood if to the end her pelvis keeps
within `test_gynoid_stand.HELD_DEG` of upright and nothing but her soles touches the floor.
Stepping, the MPC plans (`machine.balance`, `machine.mpc`).
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

#: The stand's law without steps: the ZMP asked XI_K/omega past the capture point, away from
#: the soles' middle.
XI_K = 2.0

#: A walk measured from TIMED_FROM_S after it is asked, s.
TIMED_FROM_S = 2.0

#: The ways a polar shoves from, deg: 0 her left, 90 ahead.
WAYS = tuple(range(0, 360, 45))

#: The drives' PD (`physics.SERVO`'s gains times SERVO_SHARE: kp, kd) round a reference
#: integrated from the stack's accelerations, leaking to the joints' angles over LEAK_S.
SERVO_SHARE, LEAK_S = (0.5, 1.0), 0.2

#: War emergency power as the quad has it (`terminal/views/quad/flight.py`): a drive the stack
#: asks past its clamp (`wbc.step`'s over) has its board's derate held off and its clamp at its
#: stack's peak for WEP_HOLD_S past its last need, WEP_S of it a run.
WEP_HOLD_S, WEP_S = 0.6, 5.0

NAMES = ()


def law(s, st, mid):
    """The stack's ask standing without steps: the capture point brought to `mid` (x, z)."""
    import numpy as np
    from machine import balance
    c, v = s['com'], s['vcom']
    omega = math.sqrt(9.81 / max(0.3, c[1]))
    xi = c[[0, 2]] + v[[0, 2]] / omega
    p = xi + XI_K / omega * (xi - mid)
    acc = omega ** 2 * (c[[0, 2]] - p)
    return {'stance': (True, True), 'posture': st['posture'], 'turns': st['turns'],
            'com_acc': np.array([acc[0], balance.HEIGHT_KD[0] * (st['height'] - c[1])
                                 - balance.HEIGHT_KD[1] * v[1], acc[1]])}


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


def stand(push=0.0, way=0.0, seconds=None, trace=False, wep=True, steps=True, at=AT_S, walk=None):
    """{stood, tilt deg, drift m, top share, us a step, wep s, steps, ..} standing `seconds` (TO_S
    past the shove), shoved `push` N from `way` deg at `at`; `wep` war emergency power granted;
    `steps` the MPC's law, else the capture point's alone; `walk` (m/s, s a step) asked from
    `at` on, her walk's metres, m/s and J/m measured from TIMED_FROM_S into it."""
    import numpy as np
    from machine import balance, gait, physics, wbc
    from machine.errors import MachineError
    from machine.figure import JOINTS, SEGMENTS
    global NAMES
    world, body = physics.World(), wbc.Body()
    NAMES = [j for j, d in zip(JOINTS, body.driven) if d]
    pose = gait.stand()
    world.reset(pose)
    s = wbc.sense(body, world.data.qpos, world.data.qvel)
    low = min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(pose, where=(0.0, 1.0 - low, 0.0))
    world.gains[:] *= SERVO_SHARE
    world.gains[~body.driven] = 0.0
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
    mid = (s['feet'][0]['sole'][[0, 2]] + s['feet'][1]['sole'][[0, 2]]) / 2.0
    st = balance.state(s, body.knees)
    pushed, tilt, drift, top, cost, passes, worst = False, 0.0, 0.0, 0.0, 0.0, 0, 0.0
    failed, down, force, was, bears = None, None, np.zeros(6), 'stand', (0.0, 0.0)
    a = math.radians(way)
    seconds = at + TO_S if seconds is None else seconds
    walked, drawn, z0 = None, 0.0, None
    while now[0] < seconds:
        t0 = time.perf_counter()
        s = wbc.sense(body, world.data.qpos, world.data.qvel)
        ask = (balance.step(st, s, loads(world, force), now[0], bears=bears) if steps
               else law(s, st, mid))
        try:
            out = wbc.step(body, s, dict(ask, wep=left > 0.0))
        except MachineError as exc:
            failed = '%s at %.3f s' % (exc, now[0])
            break
        cost += time.perf_counter() - t0
        passes += 1
        world.feed[body.driven] = out['tau']
        bears = out['bears']
        hold = np.where(out['over'], WEP_HOLD_S, np.maximum(0.0, hold - 0.001))
        left -= 0.001 * bool(hold.any())
        world.limit[joints] = np.where(hold > 0.0, body.wep, clamp[joints])
        leak = 0.001 / LEAK_S
        ref_v += out['qacc'][body.act] * 0.001 + leak * (s['v'][body.act] - ref_v)
        ref_q += ref_v * 0.001 + leak * (s['q'] - ref_q)
        world.target[body.driven] = ref_q
        top = max(top, float(np.abs(out['tau'] / body.top).max()))
        worst = max([worst] + [float(sl.max()) for sl in out['slack'] if len(sl)])
        if walk and now[0] >= at:
            st['walk'] = walk
            if now[0] >= at + TIMED_FROM_S:
                if z0 is None:
                    z0, t0w = float(s['com'][2]), now[0]
                drawn += world.drawn() * 0.001
                walked = (float(s['com'][2]) - z0, now[0] - t0w)
        if push and not pushed and now[0] >= at:
            world.push((push * math.cos(a), 0.0, push * math.sin(a)), 0.12)
            pushed = True
        now[0] += 0.001
        world.advance()
        q = world.data.qpos[3:7]
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q[1] ** 2
                                                                              + q[3] ** 2))))))
        drift = max(drift, float(np.linalg.norm((s['com'] - home)[[0, 2]])))
        down = down or touched(world, ours, hers)
        if trace and steps and st['phase'] != was:
            omega = math.sqrt(9.81 / s['com'][1])
            xi = s['com'][[0, 2]] + s['vcom'][[0, 2]] / omega
            print('      %5.3f %s -> %s %s  xi (%+.3f %+.3f)  soles %s  land %s  captured %s' % (
                now[0], was, st['phase'], st['side'], xi[0], xi[1],
                ' '.join('(%+.3f %+.3f)' % tuple(f['sole'][[0, 2]]) for f in s['feet']),
                '(%+.3f %+.3f)' % tuple(st['land']) if st.get('land') is not None else '-',
                st.get('captured')))
        was = st['phase']
        if trace and passes % 25 == 0:
            share = np.abs(out['tau'] / body.top)
            print('%5.3f %-6s com %+6.1f %+6.1f mm  tilt %4.1f  cop %s  tau %.2f %s  slack %.2g'
                  % (now[0], st['phase'] if steps else '-', 1e3 * (s['com'][0] - home[0]),
                     1e3 * (s['com'][2] - home[2]), tilt,
                     ' '.join('(%+.0f %+.0f)' % tuple(1e3 * c) if c is not None else '-'
                              for c in out['cop']),
                     float(share.max()), NAMES[int(share.argmax())], worst))
        if tilt > HELD_DEG or s['com'][1] < 0.5 * home[1] or down:
            break
    if walked:
        metres, secs = walked
        return {'walked_m': round(metres, 2), 'm_s': round(metres / max(secs, 1e-6), 2),
                'j_m': round(drawn / max(metres, 1e-6)), 'stood': not failed and not down
                and tilt <= HELD_DEG, 'steps': st['steps'], 'tilt': round(tilt, 1),
                'us': round(1e6 * cost / max(1, passes)), 'failed': failed, 'down': down,
                't': round(now[0], 3)}
    return {'push': push, 'way': way, 'stood': not failed and not down and tilt <= HELD_DEG,
            'failed': failed, 'tilt': round(tilt, 1), 'drift_mm': round(1e3 * drift, 1),
            'top': round(top, 2), 'us': round(1e6 * cost / max(1, passes)), 'slack': worst,
            't': round(now[0], 3), 'wep_s': round(WEP_S - left, 3) if wep else 0.0,
            'steps': st['steps'] if steps else 0, 'down': down}


def polar(forces, wep=True, steps=True, spread=1):
    """A shove from each of WAYS at each of `forces`, `spread` moments apart, a relay job each:
    {N: [stood way ..]}."""
    from tools.dev import focus
    flags = ([] if wep else ['--no-wep']) + ([] if steps else ['--no-step'])
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
        print('%4g N  %d of %d stood, WEP %.1f s, %d steps   %s' % (
            n, sum(r['stood'] for r in mine), len(mine), sum(r['wep_s'] for r in mine),
            sum(r['steps'] for r in mine), '  '.join(
                '%d:%s' % (w, ''.join('u' if r['stood'] else 'F' if r['failed'] else '.'
                                      for r in mine if r['way'] == w)) for w in WAYS)))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--polar', nargs='*', type=float, metavar='N')
    parser.add_argument('--one', nargs=2, type=float, metavar=('N', 'WAY'))
    parser.add_argument('--still', type=float, default=10.0, metavar='S')
    parser.add_argument('--at', type=float, default=AT_S, metavar='S', help='when the shove comes')
    parser.add_argument('--spread', type=int, default=1, help="a polar's shoves at each way")
    parser.add_argument('--walk', nargs=3, type=float, metavar=('M_S', 'STEP_S', 'SECONDS'),
                        help='walk at M_S, a step STEP_S, for SECONDS')
    parser.add_argument('-v', action='store_true', help='a row every 25 ms')
    parser.add_argument('--no-wep', action='store_true', help='no war emergency power')
    parser.add_argument('--no-step', action='store_true', help='the capture point alone')
    args = parser.parse_args(argv)
    flags = {'wep': not args.no_wep, 'steps': not args.no_step}
    if args.walk:
        speed, step_s, secs = args.walk
        print(json.dumps(stand(seconds=AT_S + secs, trace=args.v, walk=(speed, step_s), **flags)))
    elif args.polar:
        polar(args.polar, spread=args.spread, **flags)
    elif args.one:
        print(json.dumps(stand(args.one[0], args.one[1], trace=args.v, at=args.at, **flags)))
    else:
        print(json.dumps(stand(seconds=args.still, trace=args.v, **flags)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
