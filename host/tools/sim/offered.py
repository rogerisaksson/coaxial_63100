#!/usr/bin/env python3
"""F's errand walked into as test_errands judges it, an ensemble: met or not, her time, her phases.

    python tools/sim/offered.py                          # the chair: headings x walks about the test's own
    python tools/sim/offered.py bed door --jobs 4        # those
    python tools/sim/offered.py --against C:/sources/coaxial_ab     # each run on another tree too, beside

Her session in the lodge walked toward a thing at a heading for some seconds (OFFERED, the test's own), F's first
offer said as the page says it, her passes run to its goal, her fall or PASSES (`walked_up`); judged as test_errands'
viewport test judges it (`reached`). The chair's with her sight on (`physics.SIGHT`, her stereo each frame): its 40 s
took 916 s of the wall here. A run's line: met, her time, where she ends, her phases in order. A run a job on the
relay (`tools.dev.focus.relay`), its commit COMMIT_GB, its wall WALL_X times her bound.
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: F's errands walked into (test_errands' viewport test's): its thing, her heading, her walk's seconds, its verb,
#: the phase its goal is.
OFFERED = (('chair', 1.0, 0.6, 'sit_chair', 'seated_crossfade'), ('bed', -0.65, 1.0, 'lie_bed', 'bed_rest'),
           ('door', 0.22, 1.5, 'exit_door', 'stand'))
#: Her passes to an errand's goal at most (40 s at `worker.DT`).
PASSES = 40000
#: Her walk's twist toward it, and a stop's.
WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)
#: The ensemble about the test's own: headings off it, rad, and walks off it, s.
SPREAD = ((-0.02, 0.0, 0.02), (-0.05, 0.0, 0.05))
#: A run's commit, GB - its tree's private bytes 1.74 at most, measured -, and its wall over her bound: 22.6 times
#: her time measured alone, twice that beside others.
COMMIT_GB, WALL_X = 1.8, 45.0


def walked_up(label, hd0, walk_s, verb, goal_phase, sc, said=None):
    """(her session, the page's state, whether F's first offer was the verb's and shown, the prompt shown, the
    prompt asked, the most a prop bore of her) of her walked toward `label` and F said; `said(doing, t)` told
    each change of what she does."""
    from types import SimpleNamespace as SN
    from machine.law import modal
    from machine.world import physics
    from machine.process import sprung, worker
    from terminal.views import gym, humanoid_keys, viewpoint
    from views_kit import typed
    physics.ROOM, physics.SIGHT = 1.0, float(label == 'chair')
    s = worker.session()
    worker.apply(s, {'heading': hd0, 'twist': WALK})
    for _ in range(int(round(walk_s / worker.DT))):
        worker.stepped(s)
    st = {'body': SN(send=lambda **cmd: worker.apply(s, cmd)), 'scanner': sc,
          'now': worker.told(s), 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
    offer = gym.offered(st)
    key, first = (offer or [('', None)])[0][:2]
    shown_prompt, expected_prompt = gym.prompt(st), '[%s] %s' % (key, gym.NAMED[verb])
    shown = first == verb and shown_prompt.startswith(expected_prompt) and bool(gym.ballooned(st, offer, 120, 40, STOP))
    typed(humanoid_keys.PAD, 'f', st)
    if st.get('choosing') is not None:
        typed(humanoid_keys.PAD, 'f', st)
    max_prop, last = 0.0, None
    for _ in range(PASSES):
        worker.stepped(s)
        g = s['g']
        max_prop = max(max_prop, float(g['y'][modal.PROP]))
        if said is not None and sprung.doing(g) != last:
            last = sprung.doing(g)
            said(last, float(g['t']))
        if s['fallen_at'] is not None:
            break
        if label == 'door' and g.get('_errand') is None and float(g['world'].data.qpos[2]) > 4.1:
            break
        if (label != 'door' and sprung.doing(g) == goal_phase and max_prop > 0.45 and g['t'] > 9.0
                and g.get('played') is None):
            break
    return s, st, shown, shown_prompt, expected_prompt, max_prop


def reached(label, goal_phase, s, shown, max_prop):
    """Whether her errand met its goal as the test judges it: out the door, or in its phase, borne by its prop,
    on it, sure and not fallen."""
    from machine.process import sprung
    g = s['g']
    q = g['world'].data.qpos
    if label == 'door':
        dq = float(q[g['world'].model.jnt_qposadr[g['world'].model.joint('door').id]])
        return shown and float(q[2]) > 4.1 and dq > 1.2 and s['fallen_at'] is None
    return (shown and sprung.doing(g) == goal_phase and max_prop > 0.45 and float(q[1]) < 0.72
            and float(g.get('surety', 1.0)) > 0.65 and s['fallen_at'] is None)


def described(label, s, shown_prompt, max_prop):
    """What the test says of a run: its prompt, where she is, what bore her, fallen."""
    from machine.process import sprung
    g = s['g']
    q = g['world'].data.qpos
    if label == 'door':
        dq = float(q[g['world'].model.jnt_qposadr[g['world'].model.joint('door').id]])
        return 'prompt=%r z=%.2f door=%.2f fallen=%s' % (shown_prompt, float(q[2]), dq, s['fallen_at'])
    return 'prompt=%r doing=%s pos=(%.2f,%.2f,%.2f) prop=%.2f sure=%.2f fallen=%s' % (
        shown_prompt, sprung.doing(g), float(q[0]), float(q[1]), float(q[2]), max_prop, float(g.get('surety', 1.0)),
        s['fallen_at'])


def judged(label, hd0, walk_s):
    """{label, heading, walk, met, t, detail, phases [[doing, t]]} of one run on this process's tree."""
    from terminal.views import scanner
    case = next(c for c in OFFERED if c[0] == label)
    phases = []
    s, _st, shown, prompt, _asked, prop = walked_up(label, hd0, walk_s, case[3], case[4], scanner.VisionScanner(),
                                                    lambda d, t: phases.append([d, round(t, 2)]))
    return {'label': label, 'heading': hd0, 'walk': walk_s, 'met': bool(reached(label, case[4], s, shown, prop)),
            't': round(float(s['g']['t']), 2), 'detail': described(label, s, prompt, prop), 'phases': phases}


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('labels', nargs='*', help='its things: chair, bed, door (the chair where none)')
    p.add_argument('--against', help='another tree (its root), each run there too')
    p.add_argument('--jobs', type=int, default=6, help='runs side by side (`tools.dev.focus.relay`)')
    p.add_argument('--one', nargs=4, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.one:
        root, label, hd0, walk_s = args.one
        sys.path[:0] = [os.path.join(root, 'host'), os.path.join(root, 'host', 'tests')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        print('OFFERED ' + json.dumps(judged(label, float(hd0), float(walk_s))))
        return 0
    from machine.process import worker
    from tools.dev import focus
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    work = [focus.Job('%s %s %+.2f %+.2f' % (os.path.basename(tree), c[0], dh, dw),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', tree, c[0], str(c[1] + dh),
                       str(c[2] + dw)], COMMIT_GB, WALL_X * PASSES * worker.DT)
            for c in OFFERED if c[0] in (args.labels or ['chair']) for dh in SPREAD[0] for dw in SPREAD[1]
            for tree in trees]
    for job, text, _code, seconds in focus.relay(work, batons=args.jobs):
        got = next((ln[8:] for ln in (text or '').splitlines() if ln.startswith('OFFERED ')), None)
        if got is None:
            print('%-30s no run: %s' % (job.name, (text or 'out of time')[-200:].replace('\n', ' | ')), flush=True)
            continue
        r = json.loads(got)
        print('%-30s %-4s %5.1f s  %s | %s  (%.0f s)' % (job.name, 'met' if r['met'] else 'RED', r['t'], r['detail'],
              ' '.join('%s@%.1f' % (d, t) for d, t in r['phases']), seconds), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
