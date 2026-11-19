#!/usr/bin/env python3
"""F's errand walked into as test_errands judges, an ensemble: met or not, the robot's time and phases.

    python tools/sim/offered.py                          # the chair: headings x walks about the test's own
    python tools/sim/offered.py bed door --jobs 4        # those
    python tools/sim/offered.py bed --rise               # the bed's lie and the rise from it, as the test judges
    python tools/sim/offered.py --against C:/sources/coaxial_ab     # each run on another tree too, beside

The robot's session in the lodge walked toward a thing at a heading for some seconds (OFFERED, the test's own), F's
first offer said as the page says it, the robot's passes run to its goal, the robot's fall or PASSES (`walked_up`);
judged as test_errands' viewport test judges it (`reached`). The chair's with the robot's sight on (`physics.SIGHT`,
the robot's stereo each frame): its 40 s took 916 s of the wall here. A run's line: met, the robot's time, where it
ends, its phases in order. A run a job on the relay (`tools.dev.focus.relay`), its commit COMMIT_GB, its wall WALL_X
times the robot's bound.
"""
import argparse
import json
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: F's errands walked into (test_errands' viewport test's): its thing, the robot's heading, its walk's
#: seconds, its verb, the phase its goal is.
OFFERED = (('chair', 1.0, 0.6, 'sit_chair', 'seated_crossfade'), ('bed', -0.65, 1.0, 'lie_bed', 'bed_rest'),
           ('door', 0.22, 1.5, 'exit_door', 'stand'))
#: The robot's passes to an errand's goal at most (40 s at `worker.DT`).
PASSES = 40000
#: The robot's walk's twist toward it, and a stop's.
WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)
#: The ensemble about the test's own: headings off it, rad, and walks off it, s.
SPREAD = ((-0.02, 0.0, 0.02), (-0.05, 0.0, 0.05))
#: A run's commit, GB - its tree's private bytes 1.74 at most, measured -, and its wall over the robot's bound:
#: 22.6 times its time measured alone, twice that beside others.
COMMIT_GB, WALL_X = 1.8, 45.0
#: The bed's rise as test_errands judges it: stood within RISE_S s, its soles bearing more than RISE_N N
#: 0.5 s on, sure past RISE_SURE.
RISE_S, RISE_N, RISE_SURE = 30.0, 250.0, 0.85


def walked_up(label, hd0, walk_s, verb, goal_phase, sc, said=None, each=None):
    """(the robot's session, the page's state, whether F's first offer was the verb's and shown, the prompt
    shown, the prompt asked, the most a prop bore of it) of its walked toward `label` and F said; `said(doing,
    t)` told each change of what it does, `each(s)` called after each pass."""
    from types import SimpleNamespace as SN
    from machine.law import modal
    from machine.world import physics
    from machine.process import sprung, worker
    from terminal.views import gym, humanoid_keys, viewpoint
    from views_kit import typed
    physics.ROOM, physics.SIGHT = 1.0, float(label == 'chair')
    s = worker.session()
    s['each'].extend([each] if each is not None else [])
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
            if said is not None:
                said(ended(s), float(g['t']))
            break
        if label == 'door' and g.get('_errand') is None and float(g['world'].data.qpos[2]) > 4.1:
            break
        if (label != 'door' and sprung.doing(g) == goal_phase and max_prop > 0.45 and g['t'] > 9.0
                and g.get('played') is None):
            break
    return s, st, shown, shown_prompt, expected_prompt, max_prop


def ended(s):
    """How the robot's epoch ended, said in its phases: 'shutdown' by a bus, 'fell' where the world's referee
    finds it down, else 'fault' - its law raised (`worker.stepped`), a new epoch laid RECOVER_S on alike."""
    from machine.world import referee
    return 'shutdown' if s['shutdown'] is not None else 'fell' if referee.fallen(s['g'], s['g']['world']) else 'fault'


def reached(label, goal_phase, s, shown, max_prop):
    """Whether the robot's errand met its goal as the test judges it: out the door, or in its phase, borne by
    its prop, on it, sure and not fallen."""
    from machine.process import sprung
    g = s['g']
    q = g['world'].data.qpos
    if label == 'door':
        dq = float(q[g['world'].model.jnt_qposadr[g['world'].model.joint('door').id]])
        return shown and float(q[2]) > 4.1 and dq > 1.2 and s['fallen_at'] is None
    return (shown and sprung.doing(g) == goal_phase and max_prop > 0.45 and float(q[1]) < 0.72
            and float(g.get('surety', 1.0)) > 0.65 and s['fallen_at'] is None)


def described(label, s, shown_prompt, max_prop):
    """What the test says of a run: its prompt, where the robot is, what bore it, fallen."""
    from machine.process import sprung
    g = s['g']
    q = g['world'].data.qpos
    if label == 'door':
        dq = float(q[g['world'].model.jnt_qposadr[g['world'].model.joint('door').id]])
        return 'prompt=%r z=%.2f door=%.2f fallen=%s' % (shown_prompt, float(q[2]), dq, s['fallen_at'])
    return 'prompt=%r doing=%s pos=(%.2f,%.2f,%.2f) prop=%.2f sure=%.2f fallen=%s' % (
        shown_prompt, sprung.doing(g), float(q[0]), float(q[1]), float(q[2]), max_prop, float(g.get('surety', 1.0)),
        s['fallen_at'])


def rose(s, st, said=None):
    """(whether, what) of the robot's rise from the bed as test_errands judges it after the lie: 0.8 s on, F's
    first offer the bed's rise, said; within RISE_S its doing stands, and 0.5 s on its soles bear more than
    RISE_N, it is sure past RISE_SURE and has not fallen."""
    from machine.process import sprung, worker
    from terminal.views import gym, humanoid_keys
    from views_kit import typed
    for _ in range(int(round(0.8 / worker.DT))):
        worker.stepped(s)
    st['now'] = worker.told(s)
    prompt, on = gym.prompt(st), (gym.offered(st) or [('',) * 5])[0][4]
    typed(humanoid_keys.PAD, 'f', st)
    if st.get('choosing') is not None:
        typed(humanoid_keys.PAD, 'f', st)
    t0, stood, last, fell = s['g']['t'], None, None, None
    for _ in range(int(round((RISE_S + 0.5) / worker.DT))):
        worker.stepped(s)
        if fell is None and s['fallen_at'] is not None:
            fell = ended(s)
            if said is not None:
                said(fell, float(s['g']['t']))
        doing = sprung.doing(s['g'])
        if doing != last:
            last = doing
            stood = stood if stood is not None or doing != 'stand' else s['g']['t'] - t0
            if said is not None:
                said(doing, float(s['g']['t']))
    g = s['g']
    loads, sure = float(sum(g['loads'])), float(g.get('surety', 1.0))
    ok = (prompt == '[F1] rise bed' and on.startswith('bed ') and stood is not None and stood <= RISE_S
          and loads > RISE_N and sure > RISE_SURE and s['fallen_at'] is None)
    return ok, 'rise prompt=%r stood=%s loads=%.0f N sure=%.2f pelvis y=%.2f' % (
        prompt, None if stood is None else round(stood, 2), loads, sure, float(g['world'].data.qpos[1]))


def judged(label, hd0, walk_s, rise=False, each=None):
    """{label, heading, walk, met, t, detail, phases [[doing, t]]} of one run on this process's tree; `rise` the
    bed's rise after its lie too (`rose`)."""
    from terminal.views import scanner
    case = next(c for c in OFFERED if c[0] == label)
    phases = []
    s, st, shown, prompt, _asked, prop = walked_up(label, hd0, walk_s, case[3], case[4], scanner.VisionScanner(),
                                                   lambda d, t: phases.append([d, round(t, 2)]), each)
    met, detail = bool(reached(label, case[4], s, shown, prop)), described(label, s, prompt, prop)
    if rise and label == 'bed' and met:
        met, more = rose(s, st, lambda d, t: phases.append([d, round(t, 2)]))
        detail += '; ' + more
    return {'label': label, 'heading': hd0, 'walk': walk_s, 'met': met,
            't': round(float(s['g']['t']), 2), 'detail': detail, 'phases': phases}


#: The least span between two rows of `rows`, s: a law pass's.
ROW_S = 0.003


def rows(t0, t1):
    """`each(s)` printing the robot's mechanics on a law pass every ROW_S from `t0` to `t1` s of its first epoch:
    its doing, its pelvis's height, its trunk's pitch ahead (deg), its centre of mass and its toes ahead of its
    feet's middle (m), the shares of its weight its soles, its seat and thighs and its hands bear; its law's
    capture point ahead and its standing soles' heel and toe edges (`waits.risen_rows`' own), its prop and palms;
    which of its waits hold its table's rows on its next law pass (`waits`: risen, held, sunk, borne - their python
    on a copy), its capture point's need of a step (`modal.NEED`) and its swings (`modal.SWINGS`); its text's next label and the seconds till it,
    its surety."""
    import math
    from coaxial.model.blocks import numpy as np
    from machine.law import modal, poise, waits
    from machine.process import sprung, worker
    from machine.senses import edge
    skin = {n: i for i, n in enumerate(edge.SKIN)}
    held = {'epoch': None, 'at': -math.inf}
    print('ROW t doing pelvis_y pitch com toes soles seat hands xi heel toe prop palms waits need swings next in sure')

    def each(s):
        g = s['g']
        held['epoch'] = s['epoch'] if held['epoch'] is None else held['epoch']
        if (s['epoch'] != held['epoch'] or not t0 <= g['t'] <= t1 or g['t'] < held['at'] + ROW_S - 1e-9
                or g.get('t_law', math.nan) + worker.DT != g['t']):
            return
        held['at'] = g['t']
        m, d = g['world'].model, g['world'].data
        body = lambda name: m.body(name).id
        R, up = d.xmat[body('pelvis')].reshape(3, 3), np.array([0.0, 1.0, 0.0])
        ahead = R[:, 2] * np.array([1.0, 0.0, 1.0])
        ahead /= np.linalg.norm(ahead)
        trunk = d.xmat[body('torso')].reshape(3, 3)[:, 1]
        feet = 0.5 * (d.xpos[body('left_foot')] + d.xpos[body('right_foot')])
        toes = 0.5 * (d.xpos[body('left_toes')] + d.xpos[body('right_toes')])
        com = d.subtree_com[m.body_rootid[body('pelvis')]]
        share = np.linalg.norm(g['sense'][edge.TOUCH].reshape(-1, 3), axis=1) / float(g['fr'][modal.WEIGHT])
        part = lambda *names: float(sum(share[skin[n]] for n in names))
        sides = lambda *ends: tuple(side + end for side in ('left', 'right') for end in ends)
        y, fr, a, tab = g['y'], g['fr'], g['a'], g['tab']
        rot = fr[modal.ROT].reshape(3, 3)
        c, v = rot @ y[modal.COM], rot @ y[modal.VCOM]
        xi = c[:2] + v[:2] / math.sqrt(modal.G / max(float(c[2] - fr[modal.FLOOR]), 0.3))
        lo, hi = poise.boxes((y[modal.SOLE].reshape(2, 3) @ rot.T)[:, :2], a[modal.EDGE], y[modal.TILT])
        stood = a[modal.STANCE]
        w = stood / max(float(stood[0] + stood[1]), poise.TINY)
        dt = worker.DT * g['every']
        t = g['t_law'] + dt
        held_by = ''.join(k for k, shift in (
            ('R', waits.risen_rows(tab.copy(), y, fr, a, g['st'], t, dt, g.get('_prop_until', -1.0))[0]),
            ('H', waits.held_rows(tab.copy(), a, g['out'], t, dt, g.get('_held_s', 0.0))[0]),
            ('S', waits.sunk_rows(tab.copy(), y, fr, t, dt)[0]),
            ('B', waits.borne_rows(tab.copy(), y, fr, t, dt)[0])) if shift > 0.0) or '-'
        later = [(at, n) for at, n in g['labels'] if at > g['t']]
        nxt = min(later) if later else (math.inf, '-')
        print('ROW %.3f %s %.3f %+.1f %+.3f %+.3f %.2f %.2f %.2f %+.3f %+.3f %+.3f %.2f %.2f %s %.2f %.2f/%.2f %s %.3f %.2f' % (
            g['t'], sprung.doing(g), float(d.qpos[1]), math.degrees(math.atan2(float(trunk @ ahead), float(trunk @ up))),
            float((com - feet) @ ahead), float((toes - feet) @ ahead), part(*sides('_foot', '_toes')),
            part('pelvis', *sides('_thigh')), part(*sides('_hand', '_fingers')), float(xi[0]),
            float(w[0] * lo[0, 0] + w[1] * lo[1, 0]), float(w[0] * hi[0, 0] + w[1] * hi[1, 0]), float(y[modal.PROP]),
            float(y[modal.PALMS].sum()), held_by, float(a[modal.NEED]), *(float(sw) for sw in a[modal.SWINGS]), nxt[1],
            nxt[0] - g['t'],
            float(g.get('surety', 1.0))))
    return each


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('labels', nargs='*', help='its things: chair, bed, door (the chair where none)')
    p.add_argument('--against', help='another tree (its root), each run there too')
    p.add_argument('--jobs', type=int, default=6, help='runs side by side (`tools.dev.focus.relay`)')
    p.add_argument('--rise', action='store_true', help='the bed\'s rise after its lie too, as test_errands judges it')
    p.add_argument('--one', nargs=5, help=argparse.SUPPRESS)
    p.add_argument('--rows', nargs=2, type=float, metavar=('FROM', 'TO'),
                   help="with --one: its first epoch's mechanics from..to s, a row every ROW_S (`rows`)")
    args = p.parse_args(argv)
    if args.one:
        root, label, hd0, walk_s, rise = args.one
        sys.path[:0] = [os.path.join(root, 'host'), os.path.join(root, 'host', 'tests')]
        os.environ['PYTHONPATH'] = os.path.join(root, 'host')
        each = rows(*args.rows) if args.rows else None
        print('OFFERED ' + json.dumps(judged(label, float(hd0), float(walk_s), rise == '1', each)))
        return 0
    from machine.process import worker
    from tools.dev import focus
    trees = [os.path.dirname(HOST)] + ([os.path.abspath(args.against)] if args.against else [])
    work = [focus.Job('%s %s %+.2f %+.2f' % (os.path.basename(tree), c[0], dh, dw),
                      [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', tree, c[0], str(c[1] + dh),
                       str(c[2] + dw), '1' if args.rise else '0'], COMMIT_GB, WALL_X * (PASSES * worker.DT + RISE_S))
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
