#!/usr/bin/env python3
"""Her scenarios in MuJoCo: a scene placed, commanded as a player would, judged by the world.

    python tools/sim/scenarios.py                    # every scenario on the relay, a table
    python tools/sim/scenarios.py wall chair         # those named; --shoves the shoves' polar too
    python tools/sim/scenarios.py --one wall         # one, its line as JSON (the relay's job)

A scenario is a scene (`floor.scenario`, or a chair or a bed set at her) and its commands at their
seconds, given through her worker's own session (`worker.apply`, `worker.stepped`) as the page
gives them. The referee reads the world's truth her law never sees: whether she fell
(`worker._is_fallen`), what bore her - the floor, a chair, a mattress, a wall -, where her pelvis
went, and her head's and pelvis's jerk through a 20 ms window, its 99th percentile.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: A player's walk (`se2`'s), m/s; a stop.
WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)
#: What bears her, by the body it is: the props a scene places.
BEARS = {'chair': 'chair', 'prop_bed': 'bed', 'wall': 'wall', 'tree': 'tree'}
#: A goal held this long, s; jerk through this window, s; an obstacle met past this, N.
HELD_S, JERK_S, MET_N = 1.0, 0.02, 20.0


def _walk(seconds, then=6.0):
    """Walk from 1 s for `seconds`, then stop and stand `then` s: (commands, seconds)."""
    return [(1.0, {'twist': WALK}), (1.0 + seconds, {'twist': STOP})], 1.0 + seconds + then


#: name: (scene, commands, seconds, goal, its number). A goal: 'met' (the obstacle touched and
#: her stopped at it), 'past' m (her pelvis that far on), 'rose'/'sank' m (her pelvis that much
#: higher or lower at the end), 'sat'/'lay' (a chair's or a bed's bearing her half or more for
#: HELD_S, standing on her soles at the end), 'stands' (on her soles, a chair's seat or not).
SCENARIOS = {
    'wall': ('wall', *_walk(4.0), 'met', 'wall'),
    'tree': ('tree', *_walk(4.0), 'met', 'tree'),
    'sill': ('sill', *_walk(5.0), 'past', 1.0),
    'rug': ('rug', *_walk(5.0), 'past', 1.5),
    'hole': ('hole', *_walk(5.0), 'past', 1.3),
    'slip': ('slip', *_walk(5.0), 'past', 1.2),
    'ice': ('ice', *_walk(6.0), 'past', 2.0),
    'snow': ('snow', *_walk(6.0), 'past', 2.0),
    'asphalt': ('asphalt', *_walk(6.0), 'past', 2.0),
    'stairs': ('stairs', *_walk(10.0), 'rose', 0.70),
    'stairs_down': ('stairs_down', *_walk(10.0), 'sank', 0.70),
    'slope': ('slope', *_walk(7.0), 'rose', 0.45),
    'slope_down': ('slope_down', *_walk(7.0), 'sank', 0.45),
    'cross_slope': ('cross_slope', *_walk(6.0), 'past', 2.0),
    'chair': ('chair_behind', [(1.0, {'say': 'sit_chair'}), (8.0, {'say': 'rise_chair'})], 14.0, 'sat', 'chair'),
    'bed': ('bed_behind', [(1.0, {'say': 'lie_bed'}), (9.0, {'say': 'rise_bed'})], 16.0, 'lay', 'bed'),
    'mattress': ('mattress_under', [(1.0, {'say': 'lie_floor'}), (9.0, {'say': 'rise_floor'})], 16.0, 'lay', 'bed'),
    'hop_down': ('on_chair', [(1.0, {'say': 'hop_down_chair'})], 6.0, 'sank', 0.40),
    'hop_up': ('chair_ahead', [(1.0, {'say': 'hop_up_chair'})], 6.0, 'rose', 0.40),
}
#: A player's twists, a hold each, s: the page's keys send none outside them
#: (`humanoid_keys.twist`); an agent sends the same.
KEYS = {'w': WALK, 's': (-0.35, 0.0, 0.0), 'a': (0.0, 0.5, 0.0), 'd': (0.0, -0.5, 0.0),
        'wa': (0.69, 0.0, 0.7), 'wd': (0.69, 0.0, -0.7), 'q': (0.0, 0.0, 0.7), 'W': (1.4, 0.0, 0.0), '-': STOP}
PLAYS = {'start_stop': 'w3 -3 w3 -3', 'turns': 'w2 wa2 wd2 -3', 'strafes': 'a2 d2 -3', 'reverse': 'w2 s2 w2 -3',
         'sprint': 'w2 W3 w2 -3', 'spin': 'q3 -3', 'weave': 'w1 wa1 w1 wd1 w1 -3', 'about': 'q2 wa1 w2 -3'}


def _played(keys):
    """(commands, seconds) of a player's keys, 'w3 -2': each a twist held its seconds, from 1 s."""
    at, plan = 1.0, []
    for word in keys.split():
        plan.append((at, {'twist': KEYS[word.rstrip('0123456789.')]}))
        at += float(word[len(word.rstrip('0123456789.')):])
    return plan, at + 1.0


SCENARIOS.update({'play_' + n: ('flat', *_played(k), 'stands', None) for n, k in PLAYS.items()})

#: Shoves: N, the way they push (deg about up from ahead, 90 to her left), standing or walking,
#: at one of MOMENTS - a quarter of her stride apart, the foot that swings decides; 80 N held, 200 N
#: held or fallen with her head slower than FALL_M_S onto the floor.
MOMENTS = (5.0, 5.15, 5.30, 5.45)
SHOVES = {'shove_%d_%03d_%s_%d' % (n, way, gait, k): (n, way, gait, at)
          for n in (80, 200) for way in range(0, 360, 45) for gait in ('stand', 'walk')
          for k, at in enumerate(MOMENTS)}
FALL_M_S = 1.0


def placed(s, scene):
    """`scene` laid round her in her session `s`: a floor event, or a chair or a bed set at her."""
    from machine import floor, worker
    world = s['g']['world']
    set_at = scene.endswith(('_behind', '_ahead', '_under')) or scene == 'on_chair'
    worker.apply(s, {'scenario': 'flat' if set_at else scene})
    if scene == 'chair_behind':
        floor.mocap(world, 'chair', (0.0, 0.0, -0.30), math.pi)
    elif scene == 'chair_ahead':
        floor.mocap(world, 'chair', (0.0, 0.0, 0.40), math.pi)
    elif scene == 'bed_behind':
        floor.mocap(world, 'prop_bed', (0.0, 0.0, -0.40 - floor.BED_MATTRESS[2]), 0.0)
    elif scene == 'mattress_under':
        floor.mocap(world, 'prop_bed', (0.0, 0.004 - floor.BED_MATTRESS_H, -0.20), 0.0)
    elif scene == 'on_chair':
        floor.mocap(world, 'chair', (0.0, 0.0, 0.0), math.pi)
        world.data.qpos[1] += floor.CHAIR_SEAT_H
    world._mj.mj_forward(world.model, world.data)


def borne(world, hers, kinds, force):
    """{kind: N} of what bears her now, by the body her contact is with (`kinds`, else 'floor')."""
    m, d, out = world.model, world.data, {}
    for i in range(d.ncon):
        c = d.contact[i]
        a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if hers[a] == hers[b]:
            continue
        other = b if hers[a] else a
        world._mj.mj_contactForce(m, d, i, force)
        kind = kinds.get(m.body_rootid[other], 'floor')
        out[kind] = out.get(kind, 0.0) + abs(float(force[0]))
    return out


def ran(name, wobble=0.0):
    """{name, stood, fell_at, goal, met, went m, rose m, speed m/s, bore {kind: share held}, jerk}:
    laid turned `wobble` rad about up."""
    import numpy as np
    from machine import edge, modal, physics, worker
    # an open floor: the page's living room walls the longer walks in
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    shove = SHOVES.get(name)
    if shove:
        n, way, gait, at = shove
        yaw = math.radians(way)
        plan = ([(0.5, {'twist': WALK})] if gait == 'walk' else []) + [
            (at, {'push': (n * math.sin(yaw), 0.0, n * math.cos(yaw)), 'seconds': 0.12})]
        scene, seconds, goal, aim = 'flat', 9.0, 'held', n
    else:
        scene, plan, seconds, goal, aim = SCENARIOS[name]
    s = worker.session()
    placed(s, scene)
    world = s['g']['world']
    q = world.data.qpos[3:7].copy()
    world.data.qpos[3:7] = (q[0] * math.cos(wobble / 2.0) - q[2] * math.sin(wobble / 2.0), q[1] * math.cos(wobble / 2.0) + q[3] * math.sin(wobble / 2.0),
                            q[2] * math.cos(wobble / 2.0) + q[0] * math.sin(wobble / 2.0), q[3] * math.cos(wobble / 2.0) - q[1] * math.sin(wobble / 2.0))
    world._mj.mj_forward(world.model, world.data)
    m, d = world.model, world.data
    hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)])
    kinds = {m.body(n).id: k for n, k in BEARS.items()}
    weight = float(s['g']['fr'][modal.WEIGHT])
    head, force = m.body('head').id, np.zeros(6)
    track, fell, hit, k, bore, most = [], None, None, 0, {}, {}
    plan = sorted(plan, key=lambda c: c[0])
    while s['g']['t'] < seconds and s['epoch'] == 0:
        while k < len(plan) and s['g']['t'] >= plan[k][0]:
            worker.apply(s, plan[k][1])
            k += 1
        worker.stepped(s)
        g = s['g']
        if s['fallen_at'] is not None and fell is None:
            fell = round(g['t'], 3)
        speed_head = float(np.linalg.norm(d.cvel[head][3:]))
        if fell is not None and hit is None and float(d.xpos[head][1]) < 0.25:
            hit = speed_head
        track.append((g['t'], *d.qpos[0:3].tolist(), *d.xpos[head].tolist()))
        if g['n'] % 10 == 0:
            now = borne(world, hers, kinds, force)
            for kind, n in now.items():
                most[kind] = max(most.get(kind, 0.0), n)
                if n > 0.5 * weight:
                    bore[kind] = bore.get(kind, 0.0) + 0.01
            g['_borne'] = now
    t = np.array(track)
    w = max(1, int(JERK_S / 0.001))
    smooth = lambda p: np.cumsum(p, axis=0)[w:] / w - np.cumsum(p, axis=0)[:-w] / w
    jerk = lambda p: (float(np.percentile(np.linalg.norm(np.diff(smooth(p), n=3, axis=0), axis=1), 99) / 0.001 ** 3)
                      if len(p) > 4 * w else math.nan)
    end = s['g'].get('_borne', {})
    out = {'name': name, 'stood': fell is None, 'fell_at': fell, 'goal': goal, 'aim': aim,
           'went': round(float(t[-1, 3] - t[0, 3]), 3), 'rose': round(float(t[-1, 2] - t[0, 2]), 3),
           'speed': round(float(np.linalg.norm(t[-1, 1:4] - t[-51, 1:4]) / 0.05), 3),
           'bore': {kd: round(v, 2) for kd, v in bore.items()},
           'touched': {kd: round(v) for kd, v in most.items() if kd != 'floor'},
           'on_soles': round(end.get('floor', 0.0) / weight, 2), 'head_hit': hit,
           'jerk_pelvis': round(jerk(t[:, 1:4]), 1), 'jerk_head': round(jerk(t[:, 4:7]), 1)}
    out['met'] = judged(out)
    return out


def judged(out):
    """Whether a run met its goal, by the world's truth."""
    goal, aim = out['goal'], out['aim']
    if goal == 'held':
        return out['stood'] or (aim > 150 and out['head_hit'] is not None and out['head_hit'] < FALL_M_S)
    if not out['stood']:
        return False
    if goal == 'met':
        return out['touched'].get(aim, 0.0) > MET_N and out['speed'] < 0.1
    if goal == 'past':
        return out['went'] >= aim
    if goal == 'rose':
        return out['rose'] >= aim
    if goal == 'sank':
        return out['rose'] <= -aim
    if goal in ('sat', 'lay'):
        return out['bore'].get(aim, 0.0) >= HELD_S and out['on_soles'] >= 0.8
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('names', nargs='*', help='scenarios (all where none)')
    parser.add_argument('--one', help="one scenario, its line as JSON")
    parser.add_argument('--wobble', type=float, default=0.0, help='with --one: laid turned this rad')
    parser.add_argument('--wobbles', action='store_true', help='each laid turned 0 and 2e-4 rad either way too')
    parser.add_argument('--shoves', help="the shoves' polar too, of these N (80, 200)")
    args = parser.parse_args(argv)
    if args.one:
        print(json.dumps(ran(args.one, args.wobble)))
        return 0
    from tools.dev import focus
    names = args.names or (list(SCENARIOS) + [n for n in SHOVES if args.shoves and n.split('_')[1] == args.shoves])
    wobbles = (0.0, 2e-4, -2e-4) if args.wobbles else (0.0,)
    names = [n if w == 0.0 else '%s@%+g' % (n, w) for n in names for w in wobbles]
    jobs = [focus.Job(n, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', n.split('@')[0],
                          '--wobble', n.split('@')[1] if '@' in n else '0'], timeout=900.0) for n in names]
    rows = {}
    for job, text, _code, _seconds in focus.relay(jobs):
        line = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        rows[job.name] = json.loads(line) if line else {'name': job.name, 'error': (text or '').strip()[-300:]}
    print('%-22s %-5s %-7s %-6s %-6s %6s %6s %6s %-22s %5s %8s %8s' % (
        'scenario', 'stood', 'fell at', 'goal', 'met', 'went', 'rose', 'speed', 'bore (s)', 'soles',
        'jerk pel', 'jerk hd'))
    for n in names:
        r = rows[n]
        if 'error' in r:
            print('%-22s ERROR %s' % (n, r['error'][-120:]))
            continue
        print('%-22s %-5s %-7s %-6s %-6s %6.2f %6.2f %6.2f %-22s %5.2f %8.0f %8.0f' % (
            n, r['stood'], r['fell_at'] or '-', r['goal'], r['met'], r['went'], r['rose'], r['speed'],
            ' '.join('%s %.1f' % kv for kv in r['bore'].items()), r['on_soles'], r['jerk_pelvis'], r['jerk_head']))
    print('%d of %d met' % (sum(bool(r.get('met')) for r in rows.values()), len(rows)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
