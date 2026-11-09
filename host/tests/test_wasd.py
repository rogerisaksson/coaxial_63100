#!/usr/bin/env python3
"""WASD on her: the page's keys drive her, and she neither falls nor stumbles on them.

W, A, D and S typed on the HUMANOID page as a terminal types them (`humanoid_keys.steered` each frame, her
twist the page's), each held its seconds and let go, on her loop as the page lays it; and the player's plays
(`tools/sim/player.py` PLAYS: start-stop, turns, strafes, reverse, sprint, a spin, a weave, a turnabout, her
walk, the moves, the stutter) from each of the ensemble's starts (`tools/sim/handovers.ENSEMBLE`'s diagonal:
her drive and her axial as built and a hair either side), a relay job each (`tools/sim/handover.py --spec
--play`), judged by the world and by her analyzers: `support.stumble` (her surety under her brace),
`support.fall`, and `motion.spasm` counted beside them."""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from tools.sim import player
from gynoid_kit import Report
from views_kit import typed

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: The page's frames, s apart, and a held key's repeats as a terminal types them, s apart (test_leap's page tests).
FRAME_S, REPEAT_S = 0.05, 0.033
#: The keys held in turn, s each: W walks her on, WA and WD arc her, none stops her, S backs her, none again.
HELD = (('w', 3.0), ('wa', 2.0), ('wd', 2.0), ('', 3.0), ('s', 2.0), ('', 3.0))
#: She has gone this far on W, m; turned this far on WA and on WD, deg; backed this far on S, m (a walk back from
#: her stand sets out slowly: 3-16 cm in its first 2.6 s); and stands slower than this, m/s, when the keys are let go.
WENT_M, TURNED_DEG, BACKED_M, STILL_M_S = 1.5, 15.0, 0.02, 0.3
#: The seconds each play is judged over (`handover.py --window`), and each job's budget, s.
WINDOW, JOB_S = '1.0,14.0', 900.0


def _heading(s):
    """Her pelvis's heading, rad, from its quaternion as `leap.step` reads it: positive turns her left."""
    w, x, y, z = (float(v) for v in s['g']['s']['turns'][0]['quat'])
    return math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))


def test_the_keys_that_move_her_are_one_twist(report):
    """W A S D move and turn her relative to her own body (independent of the viewport); Q E
    sidestep; Shift sprints: one twist, sent when it changes, each one the bench's plays hold, the
    build's speeds (`se2`); a back turn said in a brake laid for the other back turn splices into it."""
    import time
    from types import SimpleNamespace as SN
    from machine.law import se2
    from terminal.views import humanoid_keys, viewpoint
    from tools.sim import scenarios
    walk, back, side, run, yaw_s = se2.WALK_SPEED, se2.BACK_SPEED, se2.STRAFE_SPEED, se2.RUN_SPEED, round(se2.YAW_RATE, 2)

    def tw(word, yaw=180.0, heading=0.0):
        keys = dict({k: k in word.lower() for k in 'wsadqe'}, shift=word != word.lower())
        return humanoid_keys.twist(keys, viewpoint.looks({'yaw': yaw}), heading)

    report.check('in her frame: W walks her on, S walks her back, A turns her left in place, Q sidesteps left',
                 (tw('w'), tw('s'), tw('a'), tw('q')) == ((walk, 0.0, 0.0), (-back, 0.0, 0.0), (0.0, 0.0, yaw_s), (0.0, side, 0.0)))
    report.check('independent of view yaw: W walks straight, WA arcs left, Shift+W sprints, E sidesteps right',
                 (tw('w', 240.0), tw('wa', 240.0), tw('W', 90.0), tw('e', 45.0))
                 == ((walk, 0.0, 0.0), (walk, 0.0, yaw_s), (run, 0.0, 0.0), (0.0, -side, 0.0)))
    every = {tw(w, yaw, hd) for w in ('w', 's', 'a', 'd', 'wa', 'wd', 'sa', 'sd', 'q', 'e', 'W', 'WA', 'S')
             for yaw in range(0, 360, 15) for hd in (0.0, 1.0, -2.0)}
    bound = [max(abs(t[i]) for t in scenarios.KEYS.values()) for i in range(3)]
    report.check("every twist the keys send within the bench plays' bounds, a sidestep alone, a sprint all but straight",
                 all(abs(t[i]) <= bound[i] + 1e-9 for t in every for i in range(3))
                 and all(not (t[1] and (t[0] or t[2])) for t in every)
                 and all(abs(t[2]) <= 0.3 for t in every if t[0] > walk), sorted(every)[:4])
    sent = []
    st: dict = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'yaw': 180.0, 'tour': None}
    t = time.monotonic()
    typed(humanoid_keys.PAD, 'w', st, t)
    for dt in (0.0, 0.05, 2.0):
        humanoid_keys.steered(st, t + dt, 0.0)
    report.check('a W typed walks her, sent once while held, the stop once when let go',
                 sent == [{'twist': (walk, 0.0, 0.0)}, {'twist': (0.0, 0.0, 0.0)}]
                 and not humanoid_keys.polled(['s'], st['pad'], t, held=set())['s'], sent)
    from machine.law import latent, modal
    from machine.process import worker
    s = worker.session()
    g, gym = s['g'], s['gym']
    g['active_entry'], g['out'][latent.SWAY][modal.DRIVE], g['est']['vel'][2] = 'turn_right', 0.28, 0.55
    worker.heard(g, gym, {'twist': tw('sd')})
    lbl_sd = [n for w, n in g['labels'] if w < g['t'] + 5.0]
    g['t'] += 0.05
    worker.heard(g, gym, {'twist': tw('sa')})
    lbl_sa = [n for w, n in g['labels'] if w < g['t'] + 5.0]
    report.check('WD -> SD brakes then turns back (never dropping into rest); splicing SA mid-brake keeps the turn queued',
                 lbl_sd == ['brake', 'turn'] and lbl_sa == ['brake', 'turn'] and g['active_entry'] == 'turn_back_left',
                 (lbl_sd, lbl_sa, g.get('active_entry')))


def _driven(texts):
    """Her session of `texts` driven by the page's keys, HELD in turn: {went m on W, turned deg on WA and on WD,
    stood (doing, m/s) at each let-go, backed m on S, twists (t, twist) sent, fell at, stumbles (event, t), spasms}."""
    from types import SimpleNamespace as SN
    from machine.process import introspect, sprung, watch, worker
    from terminal.ui import bindings
    from terminal.views import humanoid_keys
    sent, told, marks, twists = [], [], [], []
    s = worker.session(texts=texts)
    worker.apply(s, {'scenario': 'flat'})
    for _ in range(1000):
        worker.stepped(s)
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'scanner': None, 'tour': None, 'yaw': 180.0,
          'pad': humanoid_keys.pad(), 'now': worker.told(s)}
    tap, watched = introspect.Tap(), watch.Watch()
    frame, typed_at, z0 = 0.0, -math.inf, float(s['g']['est']['pos'][2])
    for word, seconds in HELD:
        t0 = float(s['g']['t'])
        while s['g']['t'] - t0 < seconds and s['fallen_at'] is None:
            for cmd in sent:
                worker.apply(s, cmd)
                twists += [(round(float(s['g']['t']), 2), tuple(cmd['twist']))] if 'twist' in cmd else []
            sent.clear()
            worker.stepped(s)
            t = float(s['g']['t'])
            if word and t - typed_at >= REPEAT_S:
                typed_at = t
                bindings.act_on(humanoid_keys.PAD, list(word), st, t)
            if t - frame >= FRAME_S:
                frame = t
                st['now'] = worker.told(s)
                humanoid_keys.steered(st, t, 0.0)
            snap = tap.passed(s)
            if snap is not None:
                told.extend(watched.step(snap))
        marks.append((float(s['g']['est']['pos'][2]), math.degrees(_heading(s)), sprung.doing(s['g']),
                      math.hypot(float(s['g']['est']['vel'][0]), float(s['g']['est']['vel'][2]))))
    return {'went': marks[0][0] - z0, 'turned': (marks[1][1] - marks[0][1], marks[2][1] - marks[1][1]),
            'stood': [(m[2], round(m[3], 2)) for m in (marks[3], marks[5])], 'backed': marks[3][0] - marks[4][0],
            'twists': twists, 'fell': s['fallen_at'], 'stumbles': [(e['event'], round(e['t'], 2)) for e in told if e['event'] in ('stumble', 'fall')],
            'spasms': sum(e['event'] == 'spasm' for e in told)}


def test_the_pages_keys_drive_her(report):
    """W, WA, WD, nothing, S, nothing, each HELD its seconds on the page, from each of the ensemble's starts: she
    walks WENT_M and more, arcs TURNED_DEG left then right, stops and stands, backs BACKED_M, stands again; the
    twists sent are the keys' as `se2` has them; no fall and no stumble, her spasms counted."""
    from machine.law import se2
    from tools.sim.handover import arranged, parsed
    from tools.sim.handovers import ENSEMBLE
    runs = {'drive=%g,ax=%g' % da: _driven(arranged(parsed('drive=%g,ax=%g' % da))) for da in zip(*ENSEMBLE)}
    yaw = round(se2.YAW_RATE, 2)
    asked = {(se2.WALK_SPEED, 0.0, 0.0), (se2.WALK_SPEED, 0.0, yaw), (se2.WALK_SPEED, 0.0, -yaw), (0.0, 0.0, 0.0),
             (-se2.BACK_SPEED, 0.0, 0.0)}
    report.check('W walks her %.1f m and more in %.0f s, WA then WD arc her %.0f deg and more left then right, each start' % (
        WENT_M, HELD[0][1], TURNED_DEG),
        all(r['went'] >= WENT_M and r['turned'][0] >= TURNED_DEG and r['turned'][1] <= -TURNED_DEG for r in runs.values()),
        {k: 'went %.2f m, turned %+.0f then %+.0f deg' % (r['went'], *r['turned']) for k, r in runs.items()})
    report.check('the keys let go, she stops and stands, after S too, each start',
                 all(d in ('stand', 'rest') and v < STILL_M_S for r in runs.values() for d, v in r['stood']),
                 {k: r['stood'] for k, r in runs.items()})
    report.check('S backs her %.1f m and more in %.0f s, each start' % (BACKED_M, HELD[4][1]),
                 all(r['backed'] >= BACKED_M for r in runs.values()), {k: round(r['backed'], 2) for k, r in runs.items()})
    report.check("the twists sent are the keys' as se2 has them",
                 all({tw for _t, tw in r['twists']} == asked for r in runs.values()), next(iter(runs.values()))['twists'])
    report.check('no fall and no stumble on the keys from any start (%d spasms)' % sum(r['spasms'] for r in runs.values()),
                 all(r['fell'] is None and not r['stumbles'] for r in runs.values()),
                 {k: (r['fell'], r['stumbles'][:4]) for k, r in runs.items()})


_ROWS: dict = {}


def _rows():
    """{play: [row]} of every play from each start, one relay job each (`handover.py --spec --play`), run once."""
    if not _ROWS:
        from tools.dev import focus
        from tools.sim.handovers import ENSEMBLE
        here = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'tools', 'sim', 'handover.py')]
        starts = ['drive=%g,ax=%g' % (d, ax) for d, ax in zip(*ENSEMBLE)]
        jobs = [focus.Job(name + '@' + start, here + ['--spec', start, '--play', keys, '--window', WINDOW], 1.5, JOB_S)
                for name, keys in player.PLAYS.items() for start in starts]
        _ROWS.update({name: [] for name in player.PLAYS})
        for job, text, _code, _seconds in focus.relay(jobs):
            lines = [line for line in text.splitlines() if line.startswith('{')]
            _ROWS[job.name.split('@')[0]].append(json.loads(lines[-1]) if lines else {'stood': False, 'stumbles': ['no row']})
    return _ROWS


def _play(name):
    """A test of one play from each start: stood on every one, no stumble."""
    def test(report):
        rows = _rows()[name]
        stood = sum(bool(r.get('stood')) for r in rows)
        stumbles = sorted(t for r in rows for t in (r.get('stumbles') or ()))
        fell = sorted(round(r['fell'], 1) for r in rows if r.get('fell') is not None)
        report.check("%s ('%s') from %d starts: stood on each, no stumble" % (name, player.PLAYS[name], len(rows)),
                     stood == len(rows) and not stumbles,
                     'stood %d of %d, fell at %s, stumbled at %s, %d spasms' % (
                         stood, len(rows), fell, stumbles[:6], sum(r.get('spasms') or 0 for r in rows)))
    test.__name__ = 'test_she_plays_' + name
    return test


ROSTER = [test_the_keys_that_move_her_are_one_twist, test_the_pages_keys_drive_her] + [_play(name) for name in player.PLAYS]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
