"""AIL, her AI in the loop: she driven about the lodge on her keys, her faults judged by the truth.

    python tools/sim/ail.py drive --seeds 8 --seconds 60          # sessions of held keys, each one's faults
    python tools/sim/ail.py drive --seeds 8 --stage 5              # and the first 5 faults of their own kinds kept
    python tools/sim/ail.py replay                                 # each staged case again: PASS where it is gone
    python tools/sim/ail.py run CASE.json --trace 4 9              # a key script (a case's), a line a frame 4-9 s
    python tools/sim/ail.py run A.json '[["w", 1.3], ["", 4]]' --held --jobs 8   # scripts, files or inline, side by side

A session: the lodge laid (ROOM), her page's keys held and let go as a hand does - W, S, A, D, Q, E and their pairs,
each held 0.4-3 s, pauses between (`bindings.act_on` each repeat, `humanoid_keys.steered` each frame: her page's
own path, test_wasd's) - and F now and then, her page's first offer docked; a seed's the same session each run. Its
keys typed as a terminal types them (a key typed renews those held beside it), or --held as a keyboard tells them
held (her page on Windows' console, `humanoid_keys.keyboard`).
It ends where she does: fallen, stuck (W held and no way made), crashed, or its seconds run. Her faults: a fall, a
stumble, a spasm (`watch.Watch`'s), her body into a thing on her keys (her place against the lodge's own boxes,
`world.props()`, grown by her half width), stuck, a crash. A fault staged is a case of its own (ail_staged/*.json:
its seed, its keys, its kind) that `replay` runs anew - red till fixed; no suite of CI's runs them.
"""
import argparse
import json
import math
import os
import random
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her page's frame and its key repeat, s (test_wasd's); the words a hand holds, and how long, s.
FRAME_S, REPEAT_S = 0.05, 0.033
WORDS = ('w', 'w', 'w', 'wa', 'wd', 'a', 'd', 's', 'q', 'e', 'wq', 'we')
HELD_S, PAUSE_S = (0.4, 3.0), (0.3, 1.5)
#: Where staged cases are kept.
STAGED = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ail_staged')


def script(seed, seconds):
    """[(word, s)] a seed's session: holds and pauses, an 'f' now and then."""
    rng, out, t = random.Random(seed), [], 0.0
    while t < seconds:
        word = 'f' if rng.random() < 0.08 else rng.choice(WORDS)
        held = rng.uniform(*HELD_S) if word != 'f' else 0.1
        out += [(word, held), ('', rng.uniform(*PAUSE_S) if word != 'f' else 30.0)]
        t += held + out[-1][1]
    return out


def _lodge(g, forward=False):
    """(centres (n, 2), axes (n, 2, 2), halves (n, 2)) of the lodge's things she may not stand in, a door's not: as
    her world's last step posed them, or `forward` posed first (before her first step) - each frame posed anew, its
    solver's warm start went with it and her run was another."""
    import numpy as np
    from machine.eyes import classes, voxels
    w = g['world']
    if forward:
        w._mj.mj_forward(w.model, w.data)
    props = w.props()
    names = classes.of(*classes.stacked(props))[0]
    kept = [p for p, n in zip(props, names) if n not in ('door', 'floor', 'ground') and p[1][1] + p[2][1] > voxels.STAND_M[0]
            and p[1][1] - p[2][1] < voxels.STAND_M[1]]
    c = np.array([(p[1][0], p[1][2]) for p in kept], float).reshape(-1, 2)
    axes = np.array([((p[3][0][0], p[3][2][0]), (p[3][0][2], p[3][2][2])) for p in kept], float).reshape(-1, 2, 2)
    return c, axes, np.array([(p[2][0], p[2][2]) for p in kept], float).reshape(-1, 2), [n for p, n in zip(props, names) if p in kept]


def _near(lodge, at, half):
    """(how far a body `half` m wide at `at` (x, z) stands off the nearest of `lodge`'s things, m - within one less
    than 0, as deep -, its name)."""
    import numpy as np
    c, axes, halves, names = lodge
    if not len(c):
        return math.inf, ''
    over = halves + half - np.abs(np.einsum('nij,nj->ni', axes, np.asarray(at, float) - c))
    gap = np.where((over > 0.0).all(axis=1), -over.min(axis=1), np.linalg.norm(np.maximum(-over, 0.0), axis=1))
    k = int(np.argmin(gap))
    return float(gap[k]), names[k]


def _traced(g, word, fast, gap, name):
    """A frame of her session said short: her key, doing, errand's phase, speed, twist asked (! her cocoon blocked),
    soles' loads, surety, her fastest driven joint since the last, the most her skin feels and where, her nearest
    thing."""
    import numpy as np
    from machine.process import sprung
    from machine.senses import edge
    from machine.way import cocoon
    d, er, ask = g['world'].data, g.get('_errand') or {}, g.get('asked') or (0.0, 0.0, 0.0)
    felt = np.linalg.norm(np.asarray(g['sense'][edge.TOUCH], float).reshape(-1, 3), axis=1)
    k = int(np.argmax(felt))
    return 't %5.2f %-4r %-11s %-6s v %.2f ask %+.1f/%+.1f/%+.1f%s soles %s sure %.2f fast %s %.0f felt %.1f %s near %s %+.2f' % (
        float(g['t']), word, sprung.doing(g), er.get('phase', '-'), math.hypot(float(d.qvel[0]), float(d.qvel[2])), *ask,
        '!' if cocoon.blocked(g) else ' ', '/'.join('%.0f' % v for v in g['loads']), float(g['surety']), fast[1], fast[0],
        felt[k], edge.SKIN[k], name, gap)


def drive(seed, seconds, eyes=False, words=None, trace=None, keyboard=False, room=1.0):
    """A seed's session (`script`, or `words`) in the lodge: {seed, words, faults [{kind, t, detail}], end, t}; a line
    a frame said (`_traced`) between `trace`'s (from, to) s; its keys typed, or `keyboard` held as a keyboard tells;
    `room` 0 on an open floor (a stop's own measure, nothing in her way)."""
    from types import SimpleNamespace as SN
    from machine.body import figure
    from machine.world import physics
    physics.ROOM, physics.SIGHT = float(room), float(eyes)
    from machine.law import se2
    from machine.world import loose
    from machine.process import introspect, sprung, watch, worker
    from machine.way import cocoon
    from terminal.ui import bindings
    from terminal.views import humanoid_keys, scanner
    words = words or script(seed, seconds)
    faults, sent, end = [], [], 'seconds'
    s = worker.session()
    g = s['g']
    lodge = _lodge(g, forward=True)
    # her page's scanner (test_errands'): without it F had no offer and docked nothing
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'scanner': scanner.VisionScanner(), 'tour': None, 'yaw': 180.0,
          'pad': humanoid_keys.pad(), 'now': worker.told(s)}
    word_now = ['']
    if keyboard:
        st['keyboard'] = lambda: set(word_now[0])
    # her driven joints, the spasm judge's (`analyzers.motion`): her toes and wrists no drive turns
    tap, watched, fast, driven = introspect.Tap(), watch.Watch(), (0.0, ''), set(watch.meta()['rated'])
    frame, typed_at, deepest = 0.0, -math.inf, 0.0
    try:
        for word, held in words:
            word_now[0] = word
            t0, x0 = float(g['t']), (float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2]))
            while g['t'] - t0 < held and s['fallen_at'] is None:
                for cmd in sent:
                    worker.apply(s, cmd)
                sent.clear()
                worker.stepped(s)
                t = float(g['t'])
                if word and t - typed_at >= REPEAT_S:
                    typed_at = t
                    bindings.act_on(humanoid_keys.PAD, list(word), st, t)
                if t - frame >= FRAME_S:
                    frame = t
                    st['now'] = worker.told(s)
                    humanoid_keys.steered(st, t, 0.0)
                    lodge = _lodge(g)      # read each frame: her loose things move (`machine.world.loose`)
                    gap, name = _near(lodge, (float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2])),
                                      figure.WIDE_HALF)
                    # on an errand or in its stunt her thing is hers to reach and sit or lie on (permeable)
                    if -gap > deepest + 0.02 and g.get('_errand') is None and g.get('active_entry') not in se2.STUNT_MODES:
                        deepest = -gap
                        faults.append({'kind': 'into', 't': round(t, 2), 'detail': '%.2f m into %s on %r' % (-gap, name, word)})
                    if trace and trace[0] <= t <= trace[1]:
                        print(_traced(g, word, fast, gap, name))
                        fast = (0.0, '')
                snap = tap.passed(s)
                if snap is not None:
                    if trace:
                        peaks = snap['body']['peaks']
                        fast = max(fast, max((v, k) for k, v in peaks.items() if k in driven))
                    for e in watched.step(snap):
                        if e['event'] in ('stumble', 'fall', 'spasm'):
                            faults.append({'kind': e['event'], 't': round(e['t'], 2), 'detail': sprung.doing(g) + (
                                ' %s %.0f deg/s' % (e['joint'], e['deg_s']) if 'joint' in e else '')})
            # W refused short of a thing is her cocoon's (`machine.way.cocoon`), not stuck
            if word == 'w' and held >= 2.0 and s['fallen_at'] is None and cocoon.refused(g, (1.0, 0.0, 0.0))[0] and math.hypot(
                    float(g['world'].data.qpos[0]) - x0[0], float(g['world'].data.qpos[2]) - x0[1]) < 0.05:
                faults.append({'kind': 'stuck', 't': round(float(g['t']), 2), 'detail': 'W held %.1f s, no way made' % held})
                end = 'stuck'
                break
            if s['fallen_at'] is not None:
                faults.append({'kind': 'fall', 't': round(s['fallen_at'], 2), 'detail': sprung.doing(g)})
                end = 'fallen'
                break
    except Exception:
        faults.append({'kind': 'crash', 't': round(float(g['t']), 2), 'detail': traceback.format_exc(limit=3)[-400:]})
        end = 'crashed'
    # her room's loose things (`machine.world.loose`), how far each went from where her world stood it, m
    m, d = g['world'].model, g['world'].data
    moved = {m.body(b).name: round(math.dist(d.xpos[b], m.body_pos[b]), 3) for b, _j in loose.things(m)}
    return {'seed': seed, 'words': words, 'faults': faults, 'end': end, 't': round(float(g['t']), 2), 'eyes': eyes,
            'held': keyboard, 'moved': moved}


def _relayed(calls, jobs):
    """(label, run) of `calls` [(label, `drive`'s arguments)]: here one after another, or `jobs` side by side on the
    relay (`tools.dev.focus.relay`), each a process of its own driving one and saying its run whole."""
    if jobs <= 1:
        for label, args in calls:
            yield label, drive(*args)
        return
    from tools.dev import focus
    here = os.path.dirname(os.path.abspath(__file__))
    work = [focus.Job(label, [sys.executable, '-X', 'utf8', '-c', 'import json, sys; sys.path.insert(0, %r); import ail; '
                              'print("AIL " + json.dumps(ail.drive(*%r)))' % (here, args)], timeout=60.0 + 20.0 * args[1])
            for label, args in calls]
    for job, text, _code, _seconds in focus.relay(work, batons=jobs):
        line = next((ln for ln in (text or '').splitlines() if ln.startswith('AIL ')), None)
        yield job.name, json.loads(line[4:]) if line else {'words': [], 'end': 'crashed', 't': 0.0, 'faults': [
            {'kind': 'crash', 't': 0.0, 'detail': (text or '')[-300:]}]}


def _driven(a):
    """(seed, run) of `a`'s seeds (`_relayed`)."""
    calls = [('ail_%d' % seed, (seed, a.seconds, a.eyes, None, None, a.held)) for seed in range(a.first, a.first + a.seeds)]
    for label, run in _relayed(calls, a.jobs):
        yield int(label[4:]), run


def _literal(text):
    """JSON `text`, else the Python literal it is."""
    import ast
    try:
        return json.loads(text)
    except ValueError:
        return ast.literal_eval(text)


def _scripts(paths):
    """[(label, words, eyes, held)] of key scripts `paths`, each a file's or inline JSON: {"words": [[keys, s], ..],
    "eyes": .., "held": ..} (a staged case's, an explorer's), or its words alone."""
    out = []
    for k, path in enumerate(paths):
        if path.lstrip().startswith(('[', '{')):
            # a shell's quoting may leave it in single quotes: a Python literal then
            case, label = _literal(path), 'inline_%d' % k
        else:
            with open(path, encoding='utf-8-sig') as f:
                case, label = json.load(f), os.path.splitext(os.path.basename(path))[0]
        case = case if isinstance(case, dict) else {'words': case}
        out.append((label, [tuple(w) for w in case['words']], case.get('eyes', False), case.get('held', False)))
    return out


def _staged():
    return sorted(f for f in os.listdir(STAGED) if f.endswith('.json')) if os.path.isdir(STAGED) else []


def main(argv=None):
    p = argparse.ArgumentParser(description='AIL, her AI in the loop: drive her about the lodge, stage her faults')
    p.add_argument('what', choices=('drive', 'replay', 'run'))
    p.add_argument('path', nargs='*', help='run: key scripts, files or inline JSON - {"words": [[keys, s], ...]}')
    p.add_argument('--jobs', type=int, default=1, help='seeds or scripts side by side (`tools.dev.focus.relay`)')
    p.add_argument('--seeds', type=int, default=4)
    p.add_argument('--first', type=int, default=0, help='the first seed')
    p.add_argument('--seconds', type=float, default=60.0)
    p.add_argument('--eyes', action='store_true', help='on her eyes (SIGHT 1), not the world\'s boxes')
    p.add_argument('--stage', type=int, default=0, help='keep the first N faults of kinds not yet kept')
    p.add_argument('--trace', type=float, nargs=2, metavar=('FROM', 'TO'), help='run: a line a frame between, s')
    p.add_argument('--held', action='store_true', help='keys held as a keyboard tells them (Windows\' console), not typed')
    p.add_argument('--kind', nargs='+', help='drive: stage only faults of these kinds (fall, stumble, spasm, into, ..)')
    p.add_argument('--room', type=float, default=1.0, help='run: 0 an open floor, nothing in her way')
    a = p.parse_args(argv)
    if a.what == 'run':
        calls = [(label, (0, sum(s for _w, s in words), a.eyes or eyes, words, a.trace, a.held or held, a.room))
                 for label, words, eyes, held in _scripts(a.path)]
        for label, run in _relayed(calls, a.jobs):
            print(json.dumps({'script': label} | {k: run.get(k) for k in ('end', 't', 'moved', 'faults')}))
        return 0
    if a.what == 'drive':
        kept, kinds = 0, set()
        for seed, run in _driven(a):
            print(json.dumps({k: run[k] for k in ('seed', 'end', 't')} | {'faults': run['faults'][:6]}))
            for f in run['faults']:
                key = (f['kind'], f['detail'].split(' on ')[0].split(' m into ')[-1] if f['kind'] == 'into' else f['detail'])
                if kept < a.stage and key not in kinds and (not a.kind or f['kind'] in a.kind):
                    kinds.add(key)
                    kept += 1
                    os.makedirs(STAGED, exist_ok=True)
                    name = 'ail_%s_%s%d_%s' % (f['kind'], 'held' if a.held else '', seed, str(f['t']).replace('.', '_'))
                    with open(os.path.join(STAGED, name + '.json'), 'w', encoding='utf-8', newline='\n') as out:
                        json.dump({'seed': seed, 'eyes': a.eyes, 'held': a.held, 'until': f['t'] + 1.0, 'fault': f,
                                   'words': run['words']}, out, indent=1)
                    print('staged', name)
        return 0
    failed = 0
    for name in _staged():
        with open(os.path.join(STAGED, name), encoding='utf-8-sig') as f:
            case = json.load(f)
        words, left = [], case['until']
        for word, held in case['words']:
            words.append((word, min(held, left)))
            left -= held
            if left <= 0:
                break
        run = drive(case['seed'], case['until'], case['eyes'], words, keyboard=case.get('held', False))
        # into: into the same thing (a window brushed is her shell's, not the bed she walked into)
        thing = case['fault']['detail'].split(' into ')[-1].split(' ')[0] if case['fault']['kind'] == 'into' else ''
        again = [f for f in run['faults'] if f['kind'] == case['fault']['kind'] and (' into %s' % thing) in ' into ' + f['detail'].split(' into ')[-1]]
        failed += bool(again)
        print('%s %s: %s' % ('FAIL' if again else 'PASS', name, again[0]['detail'] if again else 'gone'))
    print('staged: %d passed, %d failed' % (len(_staged()) - failed, failed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
