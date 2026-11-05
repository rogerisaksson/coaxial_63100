#!/usr/bin/env python3
"""Her floor scoot under her keys: how far and which way each key takes her, her tips, her heels' lag.

    python tools/sim/scoot_keys.py                       # every key at two starts, side by side on the relay
    python tools/sim/scoot_keys.py s w a d --at 8 9      # those keys at those starts
    python tools/sim/scoot_keys.py --one sa 8.0          # one session, its JSON line (the relay's)
    python tools/sim/scoot_keys.py --one s,w,a,d 12.5 0.5 --trace   # S, W, A, D in turn, said at 0.5 s, traced
    python tools/sim/scoot_keys.py --python ...          # her scoot's python (`machine.moves.scoot`), not its C

Said scoot_floor 1 s after she stands on the flat floor, then at `at` s the key (or keys: 'sa' is S and A) held
HOLD_S: her pelvis's travel along and across the heading she had (m, ahead and left +), her heading's turn (deg, left
+), when she fell or tipped (her trunk TIPPED off her lean), her pelvis's highest (m), her joints' ranges (deg) and her
knees' lag, left on right (deg of the left's stroke: 0 together, 180 in turn), the crawl take's beside it
(gcode/crawl_backwards.gcode); each side's phases with the turn, travel and seconds spent in them.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The keys compared by default, and their starts, s.
KEYS, STARTS = ('s', 'w', 'a', 'd', 'sa', 'sd', 'wa', 'wd'), ('8.0', '9.0')
#: A key held, s; her rows a pass apart, s; a traced line every TRACE_S; her trunk's pitch this far off her scoot's
#: lean, deg, has tipped her over (on the floor her fall goes unflagged: S at 12.5 s laid her back, -177 deg).
HOLD_S, ROW_S, TRACE_S, TIPPED = 8.0, 0.02, 0.2, 50.0
#: Her joints whose ranges are told.
RANGED = ('hip', 'knee', 'shoulder', 'elbow')


def lag(a, b, dt):
    """Deg: how far `a` leads `b`, of `a`'s main period (its spectrum's peak), None if either barely moves."""
    import numpy as np
    a, b = np.asarray(a, float) - np.mean(a), np.asarray(b, float) - np.mean(b)
    if np.std(a) < 2.0 or np.std(b) < 2.0:
        return None
    f = np.abs(np.fft.rfft(a))
    period = 1.0 / np.fft.rfftfreq(len(a), dt)[1 + int(np.argmax(f[1:]))]
    c = np.correlate(a, b, 'full')
    return round((360.0 * (int(np.argmax(c)) - (len(a) - 1)) * dt / period + 180.0) % 360.0 - 180.0)


def ranged(rows, names):
    """{part: [[lo, hi] left, [lo, hi] right]} of `rows` (n, joints) deg."""
    import numpy as np
    from machine.body.figure import JOINTS
    r = np.asarray(rows, float)
    return {n: [[round(float(r[:, JOINTS.index(s + n)].min())), round(float(r[:, JOINTS.index(s + n)].max()))]
                for s in ('left_', 'right_')] for n in names}


def take():
    """The crawl take's knees' lag and ranges, its program's (gcode/crawl_backwards.gcode)."""
    import numpy as np
    from machine.body.figure import JOINTS
    from machine.texts import gcode
    _h, t, words, v = gcode.read(os.path.join(gcode.DIR, 'crawl_backwards.gcode'))
    tu = np.arange(0.0, float(t[-1]), ROW_S)
    rows = np.zeros((len(tu), len(JOINTS)))
    for i, j in enumerate(JOINTS):
        if j in words:
            rows[:, i] = np.interp(tu, t, v[:, list(words).index(j)])
    knees = [rows[:, JOINTS.index(s + 'knee')] for s in ('left_', 'right_')]
    return {'knee_lag': lag(knees[0], knees[1], ROW_S), **ranged(rows, RANGED)}


def run(keys, at, python=False, trace=False, say=1.0, module='machine.moves.scoot'):
    """One session: said scoot_floor at `say` s, `keys` held from `at` s for HOLD_S (keys 's,w' S then W, HOLD_S each);
    its measures as a dict, each side's phase, her heading's turn, pitch and loads a line every TRACE_S where `trace`."""
    import numpy as np
    from machine.body.figure import JOINTS
    import importlib
    law = importlib.import_module(module)
    from machine.process import joints, worker
    from machine.world import physics
    from terminal.views import humanoid_keys as hk
    if python:
        joints.MOVES['scoot'] = module
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    g = s['g']
    w = g['world']
    d, pelvis = w.data, w.model.body('pelvis').id
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(int(round(say / worker.DT))):
        worker.stepped(s)
    worker.apply(s, {'say': 'scoot_floor'})
    while g['t'] < at and s['fallen_at'] is None:
        worker.stepped(s)

    def seat():
        from machine.law import modal
        from machine.senses import edge
        touch = np.linalg.norm(g['sense'][edge.TOUCH].reshape(-1, 3), axis=1)
        return float(touch[edge.SKIN.index('pelvis')]) / float(g['fr'][modal.WEIGHT])

    def heading():
        R = d.xmat[pelvis].reshape(3, 3)
        return math.degrees(math.atan2(float(R[0, 2]), float(R[2, 2])))

    h0, p0 = heading(), d.xpos[pelvis].copy()
    ahead = np.array([math.sin(math.radians(h0)), 0.0, math.cos(math.radians(h0))])
    rows, t1, due, by, was, legs = [], g['t'], g['t'], {}, (h0, d.xpos[pelvis].copy()), []
    tipped, high = None, float(d.xpos[pelvis][1])
    told = keys.split(',')
    while g['t'] < t1 + HOLD_S * len(told) and s['fallen_at'] is None:
        n = min(int((g['t'] - t1) / HOLD_S), len(told) - 1)
        if n == len(legs):
            if legs:
                legs[-1][1:] = [round(float((d.xpos[pelvis] - legs[-1][1]) @ ahead), 2),
                                round((heading() - legs[-1][2] + 180.0) % 360.0 - 180.0, 1)]
            held = dict.fromkeys(('w', 's', 'a', 'd', 'q', 'e', 'shift'), False)
            for key in told[n]:
                held[key] = True
            worker.apply(s, {'twist': hk.twist(held)})
            legs.append([told[n], d.xpos[pelvis].copy(), heading()])
        worker.stepped(s)
        pitch = joints._felt(g)['pitch']
        tipped = g['t'] if tipped is None and abs(pitch - law.TRUNK) > TIPPED else tipped
        high = max(high, float(d.xpos[pelvis][1]))
        state = (g.get('played') or {}).get('state')
        if state is not None and len(state) > 1:
            named = law.NAMED[int(state[law.PHASE])]
            pair = named + ('%d' % int(state[law.STEP]) if named in ('lift', 'set') and hasattr(law, 'STEP') else '')
            now = heading()
            got = by.setdefault(pair, [0.0, 0.0, 0.0])
            got[0] += (now - was[0] + 180.0) % 360.0 - 180.0
            got[1] += float((d.xpos[pelvis] - was[1]) @ ahead)
            got[2] += worker.DT
            was = (now, d.xpos[pelvis].copy())
        if g['t'] >= due:
            due += ROW_S
            rows.append([math.degrees(float(d.qpos[w.qadr[i]])) for i in range(len(JOINTS))])
            if trace and len(rows) % int(round(TRACE_S / ROW_S)) == 0 and state is not None:
                felt = joints._felt(g)
                print('%6.2f %-6s %d way %+.2f %+.2f turned %+6.1f pitch %+6.1f fists %.2f feet %.2f %.2f seat %.2f pelvis %.3f hips %+4.0f %+4.0f knees %4.0f %4.0f' % (
                    g['t'], law.NAMED[int(state[law.PHASE])], int(state[getattr(law, 'STEP', law.PHASE)]), state[law.WAY], state[law.WAY + 1],
                    (heading() - h0 + 180.0) % 360.0 - 180.0, felt['pitch'], felt['hands'], felt['left_foot'],
                    felt['right_foot'], seat(), float(d.xpos[pelvis][1]),
                    *[rows[-1][JOINTS.index(s_ + p_)] for p_ in ('hip', 'knee') for s_ in ('left_', 'right_')]), flush=True)
    went = d.xpos[pelvis] - p0
    left = np.cross(np.array([0.0, 1.0, 0.0]), ahead)
    legs[-1][1:] = [round(float((d.xpos[pelvis] - legs[-1][1]) @ ahead), 2),
                    round((heading() - legs[-1][2] + 180.0) % 360.0 - 180.0, 1)]
    knees = np.asarray(rows, float)[:, [JOINTS.index(s_ + 'knee') for s_ in ('left_', 'right_')]]
    return {'keys': keys, 'at': at, 'fallen': s['fallen_at'], 'tipped': tipped, 'high': round(high, 3),
            'along': round(float(went @ ahead), 2),
            'across': round(float(went @ left), 2), 'turned': round((heading() - h0 + 180.0) % 360.0 - 180.0, 1),
            'knee_lag': lag(knees[:, 0], knees[:, 1], ROW_S), **ranged(rows, RANGED),
            'legs': legs if len(legs) > 1 else None,
            'phases': {p: [round(v[0], 1), round(v[1], 2), round(v[2], 1)] for p, v in sorted(by.items())}}


def relayed(keys, starts, python=False, say=1.0, module='machine.moves.scoot'):
    """Each key at each start its own process on the relay's batons (`focus.relay`), a line each as it ends."""
    from tools.dev import focus
    flag = ['--python', '--law', module] if python else []
    jobs = [focus.Job('%s@%s' % (k, a), [sys.executable, '-X', 'utf8', os.path.abspath(__file__), *flag, '--one', k, a,
                                         str(say)], timeout=900.0) for a in starts for k in keys]
    print('take', json.dumps(take()), flush=True)
    for job, text, _code, _s in focus.relay(jobs):
        got = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        print(job.name, got if got else (text or '')[-400:], flush=True)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    module = argv[argv.index('--law') + 1] if '--law' in argv else 'machine.moves.scoot'
    argv = argv[:argv.index('--law')] + argv[argv.index('--law') + 2:] if '--law' in argv else argv
    python, trace = '--python' in argv or module != 'machine.moves.scoot', '--trace' in argv
    argv = [a for a in argv if a not in ('--python', '--trace')]
    if argv[:1] == ['--one']:
        print(json.dumps(run(argv[1], float(argv[2]), python, trace, *[float(a) for a in argv[3:4]], module=module)),
              flush=True)
        return 0
    say = float(argv[argv.index('--say') + 1]) if '--say' in argv else 1.0
    argv = argv[:argv.index('--say')] + argv[argv.index('--say') + 2:] if '--say' in argv else argv
    starts = argv[argv.index('--at') + 1:] if '--at' in argv else list(STARTS)
    keys = argv[:argv.index('--at')] if '--at' in argv else argv
    relayed(keys or list(KEYS), starts, python, say, module)
    return 0


if __name__ == '__main__':
    sys.exit(main())
