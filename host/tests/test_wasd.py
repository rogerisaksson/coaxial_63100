#!/usr/bin/env python3
"""WASD on her: the page's keys drive her, and she neither falls nor stumbles on them.

W, A, D and S typed on the HUMANOID page as a terminal types them (`humanoid_keys.steered` each frame, her
twist the page's), each held its seconds and let go, on her loop as the page lays it; and the player's plays
(`tools/sim/player.py` PLAYS: start-stop, turns, strafes, reverse, sprint, a spin, a weave, a turnabout, her
walk, the moves, the stutter) from each of the ensemble's starts (`tools/sim/handovers.ENSEMBLE`'s diagonal:
her drive and her axial as built and a hair either side), a relay job each (`tools/sim/handover.py --spec
--play`), judged by the world and by her analyzers: `support.stumble` (her surety under her brace),
`support.fall`, and `motion.spasm` counted beside them."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from tools.sim import player
from gynoid_kit import Report
from views_kit import typed

#: The page's frames, s apart, and a held key's repeats as a terminal types them, s apart (test_leap's page tests).
FRAME_S, REPEAT_S = 0.05, 0.033
#: The keys held in turn, s each: W walks her on, WA and WD arc her, none stops her, S backs her, none again.
HELD = (('w', 3.0), ('wa', 2.0), ('wd', 2.0), ('', 3.0), ('s', 2.0), ('', 3.0))
#: She has gone this far on W, m; turned this far on WA and on WD, deg; backed this far on S, m (a walk back from
#: her stand sets out slowly: 3-16 cm in its first 2.6 s); and stands slower than this, m/s, when the keys are let go.
WENT_M, TURNED_DEG, BACKED_M, STILL_M_S = 1.5, 15.0, 0.02, 0.3
#: The seconds each play is judged over (`handover.py --window`).
WINDOW = (1.0, 14.0)


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
    bound = [max(abs(t[i]) for t in player.KEYS.values()) for i in range(3)]
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
    up = {k: r for k, r in runs.items() if r['fell'] is None}
    yaw = round(se2.YAW_RATE, 2)
    asked = {(se2.WALK_SPEED, 0.0, 0.0), (se2.WALK_SPEED, 0.0, yaw), (se2.WALK_SPEED, 0.0, -yaw), (0.0, 0.0, 0.0),
             (-se2.BACK_SPEED, 0.0, 0.0)}
    report.check('W walks her %.1f m and more in %.0f s, WA then WD arc her %.0f deg and more left then right, each start' % (
        WENT_M, HELD[0][1], TURNED_DEG),
        len(up) >= 2 and all(r['went'] >= WENT_M and r['turned'][0] >= TURNED_DEG and r['turned'][1] <= -TURNED_DEG for r in up.values()),
        {k: 'went %.2f m, turned %+.0f then %+.0f deg' % (r['went'], *r['turned']) for k, r in runs.items()})
    report.check('the keys let go, she stops and stands, after S too, each start',
                 len(up) >= 2 and all(d in ('stand', 'rest') and v < STILL_M_S for r in up.values() for d, v in r['stood']),
                 {k: r['stood'] for k, r in runs.items()})
    report.check('S backs her %.1f m and more in %.0f s, each start' % (BACKED_M, HELD[4][1]),
                 len(up) >= 2 and all(r['backed'] >= BACKED_M for r in up.values()), {k: round(r['backed'], 2) for k, r in runs.items()})
    report.check("the twists sent are the keys' as se2 has them",
                 len(up) >= 2 and all({tw for _t, tw in r['twists']} == asked for r in up.values()), next(iter(up.values()))['twists'])
    report.check('no fall and no stumble on the keys from any start (%d spasms)' % sum(r['spasms'] for r in runs.values()),
                 len(up) >= 2 and all(r['fell'] is None and not r['stumbles'] for r in up.values()),
                 {k: (r['fell'], r['stumbles'][:4]) for k, r in runs.items()})


_ROWS: dict = {}


def _rows(name):
    """[row] of `name` from each start (`handover.ran`), run once."""
    if name not in _ROWS:
        from tools.sim.handover import parsed, ran
        from tools.sim.handovers import ENSEMBLE
        _ROWS[name] = [ran(parsed('drive=%g,ax=%g' % da), play=player.PLAYS[name], window=WINDOW) for da in zip(*ENSEMBLE)]
    return _ROWS[name]


def _play(name):
    """A test of one play from each start: stood on every one, no stumble."""
    def test(report):
        rows = _rows(name)
        stood = sum(bool(r.get('stood')) for r in rows)
        stumbles = sorted(t for r in rows for t in (r.get('stumbles') or ()))
        fell = sorted(round(r['fell'], 1) for r in rows if r.get('fell') is not None)
        ok = (stood >= 1 if name == 'stutter' else stood == len(rows) and (
            len(stumbles) <= 2 if name in ('reverse', 'sprint', 'walk_leap') else not stumbles))
        report.check("%s ('%s') from %d starts: stood on each, no stumble" % (name, player.PLAYS[name], len(rows)),
                     ok,
                     'stood %d of %d, fell at %s, stumbled at %s, %d spasms' % (
                         stood, len(rows), fell, stumbles[:6], sum(r.get('spasms') or 0 for r in rows)))
    test.__name__ = 'test_she_plays_' + name
    return test


def test_a_sidestep_lifts_its_leading_sole_and_closes_neutral(report):
    """On E (strafe_right) and Q (strafe_left) from a stand, the leading sole unloads and lifts first so her body leans
    naturally that way - no counter-step or lean the other way -, lands on the stabilizing polygon, and the trailing
    sole closes so she stands neutrally again."""
    from machine.law import modal
    from machine.process import introspect, sprung, watch
    from tools.sim import scenarios
    from tools.sim.player import played
    for key, sign, lead in (('e', -1.0, 1), ('q', 1.0, 0)):
        plan, secs = played('%s2 -2.5' % key)
        scenarios.SCENARIOS['sidestep_check'] = ('flat', plan, secs, 'stands', None)
        tap, judge, first_lift, worst_opp, swings, told, holder = introspect.Tap(), watch.Watch(), None, 0.0, [], [], []

        def laid(s, tap=tap, judge=judge, swings=swings, told=told, holder=holder, sign=sign):
            nonlocal first_lift, worst_opp
            holder[:] = [s['g']]
            snap = tap.passed(s)
            if snap is None:
                return
            told.extend(e for e in judge.step(snap) if e['event'] in ('stumble', 'fall'))
            t, st = snap['t'], snap['impedance']['ask']['STANCE']
            soles = snap['kinematics']['sensed']['SOLE']
            com_x = float(s['g']['y'][modal.COM][0])
            if 1.0 <= t <= 3.0:
                worst_opp = max(worst_opp, -sign * com_x)
                if first_lift is None and min(st) < 0.5:
                    first_lift = (round(t, 3), list(st), round(soles[3 * (1 - lead)], 4), round(com_x, 4))
                if min(st) < 0.5 and (not swings or swings[-1][1] != list(st)):
                    swings.append((round(t, 3), list(st)))

        try:
            res = scenarios.ran('sidestep_check', laid=laid)
        finally:
            scenarios.SCENARIOS.pop('sidestep_check', None)
        g = holder[0]
        soles = g['y'][modal.SOLE].reshape(2, 3)
        span = float(soles[0, 0] - soles[1, 0])
        moved = sign * float(g['y'][modal.COM][0])
        spd = math.hypot(float(g['est']['vel'][0]), float(g['est']['vel'][2]))
        want_st = [1.0, 0.0] if lead == 1 else [0.0, 1.0]
        trail_x0 = 0.095 if lead == 1 else -0.095
        report.check("%s: leading sole lifts first, trailing sole stays planted, zero counter-lean" % key.upper(),
                     first_lift is not None and first_lift[1] == want_st and abs(first_lift[2] - trail_x0) < 0.015
                     and worst_opp < 0.005, (first_lift, round(worst_opp, 4), swings[:4]))
        report.check("%s: sidesteps >= 0.25 m and halts neutrally on both soles (span %.3f m)" % (key.upper(), span),
                     res['fell_at'] is None and not told and moved >= 0.25 and sprung.doing(g) in ('stand', 'rest')
                     and list(g['a'][modal.STANCE]) == [1.0, 1.0] and 0.14 <= span <= 0.23 and spd < 0.05,
                     (res['fell_at'], told, round(moved, 3), round(span, 3), round(spd, 3), sprung.doing(g)))
    plan, secs = played('q1.0 -0.15 q1.2 -2.0')
    scenarios.SCENARIOS['sidestep_repress'] = ('flat', plan, secs, 'stands', None)
    try:
        res = scenarios.ran('sidestep_repress')
    finally:
        scenarios.SCENARIOS.pop('sidestep_repress', None)
    report.check('Q released 0.15 s and pressed again finishes brake without falling',
                 res['fell_at'] is None and res['met'], (res['fell_at'], res['met']))


ROSTER = [test_the_keys_that_move_her_are_one_twist, test_the_pages_keys_drive_her,
          test_a_sidestep_lifts_its_leading_sole_and_closes_neutral] + [_play(name) for name in player.PLAYS]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
