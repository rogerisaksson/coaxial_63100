#!/usr/bin/env python3
"""Her floor scoot's lean: pelvis, feet and fists each side at her sit, held, and each back stroke.

    python tools/sim/scoot_lean.py              # four starts side by side on the relay
    python tools/sim/scoot_lean.py 0.7          # one start, its JSON line

Said scoot_floor `at` s after she stands, then S held 8 s. At her sit's end (floor_rest's first pass), held (12 s
after the say) and each back stroke's set (both feet down, the stroke's first pass): her pelvis's tilt and her
trunk's (its left side up, deg), her centre of mass out from her pelvis's middle line (m, left +), each side's leg
joints sensed and commanded (deg), feet's and fists' places in her pelvis's frame (ahead, out from her middle line,
m) and their loads (shares of her weight), the seat's.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her leg's joints, each side.
LEG = ('hip', 'hip_roll', 'hip_yaw', 'knee', 'ankle_roll')
#: The starts compared, s after she stands.
STARTS = ('0.5', '0.7', '0.9', '1.0')
#: Her sit's span after the say, then S held, s.
SIT_S, BACK_S = 12.0, 8.0


def run(at):
    """One start: her snapshots as a dict."""
    from coaxial.model.blocks import numpy as np
    from machine.body.figure import JOINTS
    from machine.law import modal
    from machine.law.se2 import doing
    from machine.moves import scoot
    from machine.process import worker
    from machine.senses import edge
    from machine.world import physics
    from terminal.views import humanoid_keys as hk
    skin = {n: i for i, n in enumerate(edge.SKIN)}
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    g, d, m = s['g'], s['g']['world'].data, s['g']['world'].model
    ids = {n: m.body(n).id for n in ('pelvis', 'torso', 'left_hand', 'right_hand', 'left_foot', 'right_foot')}

    def tilt(body):
        return round(math.degrees(math.asin(max(-1.0, min(1.0, float(d.xmat[ids[body]].reshape(3, 3)[1, 0]))))), 1)

    def snap(tag):
        R = d.xmat[ids['pelvis']].reshape(3, 3)
        ahead = R[:, 2] * np.array([1.0, 0.0, 1.0])
        ahead /= np.linalg.norm(ahead)
        left = np.cross(np.array([0.0, 1.0, 0.0]), ahead)
        rel = lambda p: p - d.xpos[ids['pelvis']]
        q = np.degrees(g['sense'][edge.Q])
        cmd = (g.get('played') or {}).get('cmd')
        touch = np.linalg.norm(g['sense'][edge.TOUCH].reshape(-1, 3), axis=1) / float(g['fr'][modal.WEIGHT])
        out = {'tag': tag, 't': round(g['t'], 2), 'pelvis': tilt('pelvis'), 'torso': tilt('torso'),
               'com': round(float(rel(d.subtree_com[0]) @ left), 3), 'seat': round(float(touch[skin['pelvis']]), 2)}
        for side, sign in (('left', 1.0), ('right', -1.0)):
            at_ = {n: rel(d.xpos[ids[side + n]]) for n in ('_foot', '_hand')}
            out[side] = {j: [round(float(q[JOINTS.index(side + '_' + j)]), 1),
                             None if cmd is None else round(float(cmd[JOINTS.index(side + '_' + j)]), 1)] for j in LEG}
            out[side]['foot'] = [round(float(at_['_foot'] @ ahead), 2), round(sign * float(at_['_foot'] @ left), 2)]
            out[side]['fist'] = [round(float(at_['_hand'] @ ahead), 2), round(sign * float(at_['_hand'] @ left), 2)]
            out[side]['loads'] = [round(float(touch[skin[side + '_foot']] + touch[skin[side + '_toes']]), 2),
                                  round(float(touch[skin[side + '_hand']] + touch[skin[side + '_fingers']]), 2)]
        return out

    worker.apply(s, {'scenario': 'flat'})
    for _ in range(int(round(at / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': 'scoot_floor'})
    snaps, t0 = [], g['t']
    while g['t'] < t0 + SIT_S and s['fallen_at'] is None:
        worker.stepped(s)
        if not snaps and doing(g) == 'floor_rest':
            snaps.append(snap('sit_end'))
    snaps.append(snap('held'))
    keys = dict.fromkeys(('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
    keys['s'] = True
    worker.apply(s, {'twist': hk.twist(keys)})
    last, t1 = None, g['t']
    while g['t'] < t1 + BACK_S and s['fallen_at'] is None:
        worker.stepped(s)
        state = (g.get('played') or {}).get('state')
        phase = scoot.NAMED[int(state[0])] if state is not None else None
        if phase == 'stroke' and last == 'set':
            snaps.append(snap('set_%d' % (len(snaps) - 1)))
        last = phase
    return {'at': at, 'fallen': s['fallen_at'], 'snaps': snaps}


def line(sn):
    """A snapshot a line, left beside right."""
    L, R = sn['left'], sn['right']
    pair = lambda j: '%s %+.0f/%+.0f' % (j, L[j][0], R[j][0])
    return ('  %-7s %5.2f pelvis %+5.1f torso %+5.1f com %+.3f | %s | foot L %s R %s fist L %s R %s | feet %.2f/%.2f '
            'fists %.2f/%.2f seat %.2f' % (sn['tag'], sn['t'], sn['pelvis'], sn['torso'], sn['com'],
                                           ' '.join(pair(j) for j in LEG), L['foot'], R['foot'], L['fist'], R['fist'],
                                           L['loads'][0], R['loads'][0], L['loads'][1], R['loads'][1], sn['seat']))


def relayed(starts):
    """Each start its own process on the relay's batons (`focus.relay`), its snapshots a line each."""
    from tools.dev import focus
    jobs = [focus.Job('at_%s' % a, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), a], timeout=900.0)
            for a in starts]
    for job, text, _code, _s in focus.relay(jobs):
        got = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        if not got:
            print(job.name, (text or '')[-300:], flush=True)
            continue
        r = json.loads(got)
        print('== %s fallen %s' % (job.name, r['fallen']), flush=True)
        for sn in r['snaps']:
            print(line(sn), flush=True)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) == 1:
        print(json.dumps(run(float(argv[0]))), flush=True)
    else:
        relayed(argv or STARTS)
    return 0


if __name__ == '__main__':
    sys.exit(main())
