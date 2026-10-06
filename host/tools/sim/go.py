#!/usr/bin/env python3
"""Her going on the one law, simulated and measured: a track of setpoints, a segment a row.

    python tools/sim/go.py                              # the walk's row, 12 s
    python tools/sim/go.py --track "0:1" --to 10        # the run's
    python tools/sim/go.py --track "0:-1 2:-1 3:0 9:0 10:-1" --to 16    # stood, walked, stood
    python tools/sim/go.py --ask "0:-1 2:1 20:-1" --to 36               # asked the run and back
    python tools/sim/go.py --track "0:-1" --shove "4:120:90"            # shoved toward her left
    python tools/sim/go.py gaits.WALK.knee=16 going.TURN_K=1.4 --form 6,12
    python tools/sim/go.py --to 24 --manner "8:leaning:1,crouched:0.5; 16:" --form "12,16 20,24"
    python tools/sim/go.py --search walk --log build/go_walk.jsonl     # how the rows were found

A track is "t:k ..": at `t` s the row `k` of her way (`gaits.between`: -1 her stand, 0 the
walk's, 0.5 her jog, 1 the run's), linear in time between its points - `--ask`, each the row
wanted from then on, hers following as `gaits.toward` has it; she is placed as its first row
has her (`placed`). A row a segment between them: her speed, J/m drawn and of work,
her steps - their stance, the share landed with the other foot down, the knee as they land, a
sole's load. `--form a,b` samples the look's rows from a to b s and prices them as the
scoreboard prices a walk (`looks.priced`), a window a form. `--manner` is her manners asked
from t on (`gaits.MANNERS`), a segment's end too. `--search` is CMA-ES over SPANS through the
relay (`searched`): a fall ends a trial and is no price of its own.
"""
import argparse
import json
import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: What her rows hold on a flat floor, (measure, least, most), held by test_gynoid_going.py:
#: her speed, m/s, the J/m she draws, the knee as a foot lands, deg, the share of her landings
#: with the other foot down, her steps. The walk's row walks 0.85 m/s at 502 J/m, its knee
#: landing at 11-12 deg - 0.75-0.81 at 380-397, 18-20 deg, the row first found, her quick step
#: now; asked the row -0.3 she strolls 0.52 m/s at 695; the run's ran 1.44 m/s at 522; her jog
#: 0.69-0.76 at 739-801; standing she took no
#: step, and stood again after a walk one or two (2026-10-06).
FORM = {'walk': (('speed', 0.65, 0.95), ('drawn', None, 600.0), ('knee', None, 30.0),
                 ('both', 1.0, None)),
        'run': (('speed', 1.35, 1.6), ('drawn', None, 620.0), ('both', None, 0.0)),
        'jog': (('speed', 0.55, 0.95), ('drawn', None, 900.0), ('both', None, 0.0)),
        'stroll': (('speed', 0.4, 0.65), ('drawn', None, 850.0), ('both', 1.0, None)),
        'stand': (('speed', -0.03, 0.03), ('steps', None, 0.0)),
        'stand again': (('speed', -0.05, 0.05), ('steps', None, 3.0))}

#: Fallen: the pelvis under FELL_M or her trunk tipped past FELL_DEG. The look's rows a second.
#: Placed standing, her feet are STAND_WIDE_M apart.
FELL_M, FELL_DEG, ROWS_HZ, STAND_WIDE_M = 0.55, 55.0, 60.0, 0.17

#: A search's spans, {name: (low, high)}: the walk's row and what of the law it leans on, a foot
#: always down ('both': its stance over its step, s) and the strut the form's; the row between.
#: With the stance free and the strut to 14 deg the search found a walk with no foot down a
#: tenth of a second a step, its knee 14 deg behind plumb: 346 J/m, priced 50.5 (2026-10-05).
SPANS = {
    'walk': {'gaits.WALK.speed': (0.6, 1.0), 'gaits.WALK.step': (0.40, 0.70), 'both': (0.03, 0.25),
             'gaits.WALK.bounce': (0.08, 0.35), 'gaits.WALK.knee': (4.0, 26.0),
             'gaits.WALK.lean': (0.0, 6.0), 'gaits.WALK.fold': (5.0, 60.0),
             'gaits.WALK.folded': (0.5, 1.0), 'gaits.WALK.track': (0.0, 0.05),
             'gaits.WALK.under': (0.0, 0.117), 'gaits.WALK.land': (-25.0, 5.0),
             'gaits.WALK.off': (0.0, 50.0), 'gaits.WALK.list': (0.0, 8.0),
             'gaits.WALK.turn': (0.0, 8.0), 'gaits.WALK.up': (0.0, 0.12),
             'gaits.WALK.reach': (0.12, 0.40), 'going.GAINS_M_S': (0.1, 1.5),
             'going.LATE': (1.0, 2.0), 'free.LEAD': (0.0, 1.0)},
    'between': {'gaits.JOG.speed': (0.6, 1.2), 'gaits.JOG.step': (0.36, 0.58),
                'gaits.JOG.stand': (0.26, 0.64), 'gaits.JOG.up': (0.0, 0.1),
                'gaits.JOG.rise': (0.0, 0.5), 'gaits.JOG.bounce': (0.1, 0.32),
                'gaits.JOG.land': (-15.0, 15.0), 'gaits.JOG.knee': (10.0, 26.0),
                'gaits.JOG.lean': (2.0, 8.0), 'gaits.JOG.fold': (10.0, 60.0),
                'gaits.JOG.track': (0.02, 0.06), 'gaits.JOG.off': (0.0, 50.0),
                'gaits.JOG.under': (0.03, 0.117), 'gaits.JOG.folded': (0.5, 1.0),
                'gaits.JOG.reach': (0.2, 0.36)}}

#: A walk's row is walked WALK_S from a stand - stood each of STARTS s, then asked on over a
#: second - and priced from FROM_S: on one walk placed at its speed the best row's price was
#: chance, 25 and to three digits of it 92, and of 32 rows about it 13 fell in their first
#: steps on any placing. Her jog's row is tried on PASSAGES, rows asked: to her jog and to a
#: stand again, to the run and to a stand again, PASS_S each.
WALK_S, FROM_S, PASS_S, STARTS = 16.0, 8.0, 40.0, (1.0, 1.3, 1.6)
#: A walk's price takes BAND_K a width it is off a woman's band (`normal.off`).
BAND_K = 30.0
#: Its energy, J/m, by ENERGY_J_M in a walk's cost; its stop asked too (STOP), down or still
#: going at its end as a fall.
ENERGY_J_M, STOP = 25.0, ('--ask', '0:-1 1.2:0 9.2:-1 13:-1', [])
#: And a passage, its seconds: to her jog and to a stand again.
PASSAGE, PASSAGE_S = ('--ask', '0:-1 1.3:0.5 12:-1 27:-1', []), 30.0
PASSAGES = ('0:-1 1:0.5 13:-1', '0:-1 1.4:0.5 14.1:-1', '0:-1 1.2:1 18:-1', '0:-1 1.7:1 19.3:-1')


def placed(law):
    """The body placed as the law's row has her. Asked no speed, standing: both feet flat,
    her hips over their soles' points. At a speed: rising in flight as a run does - the left
    leg START_U of its swing from where it left the floor behind her, the right coming down -
    or, no rise asked, mid-stance on her right foot on a strut, the left on its way by from
    where it left the floor a step behind."""
    from machine import figure, free, gait, going, strut, walkplan
    from machine.figure import LEG, SOLE_BALL, apply, rx, ry, sub
    from machine.runner import START_M, START_U
    a = law.ask
    lean = math.radians(a['lean'])
    turn, stood = rx(lean), {}
    law.bias, law.y, law.steps, law.rates, law.com = 0.0, {}, [], {}, None
    law.pace, law.mark = a['speed'], None
    swing = 2.0 * a['step'] - a['stand']
    if a['speed'] <= 0.0:
        # standing: both feet flat STAND_WIDE_M apart, her hips over their soles' points
        v = (0.0, 0.0, 0.0)
        flats = {side: (sign * 0.5 * STAND_WIDE_M, gait.ANKLE_H, 0.0)
                 for side, sign in walkplan.SIDES}
        at = (0.0, a['under'] - apply(turn, (0.0, -gait.HIP_DROP, 0.0))[2])
        pelvis = (0.0, min(strut.reach(sign, at, turn, flats[side], a['strut'])
                           for side, sign in walkplan.SIDES), at[1])
        law.legs = {side: {'stands': True, 't': 1.0, 'flat': flats[side], 'landed': 0.0,
                           'was': None, 'off': (0.0,) * 6} for side in flats}
        law.y = {'t': 1.0, 'span': 1.0, 'landed': pelvis[1], 'rate': 0.0,
                 'from': (pelvis[1], 0.0, 0.0), 'to': (pelvis[1], 0.0, 0.0)}
        stood = {side: figure.leg(sign, pelvis, turn, flats[side], ry(0.0))
                 for side, sign in walkplan.SIDES}
    elif a['rise'] > 0.0:
        v = (0.0, a['rise'], a['speed'])
        off = (0.0, law.land_y() + a['up'], 0.0)
        up = math.radians(a['off'])
        ball = (a['track'], 0.0, -0.5 * a['speed'] * law.support())
        was = figure.leg(1.0, off, turn, sub(ball, apply(rx(up), SOLE_BALL)), rx(up))
        pelvis = (0.0, off[1] + START_M, 0.0)
        fall = law.fall(-1.0, figure.hip(-1.0, pelvis, turn), v)
        law.legs = {'left': {'stands': False, 't': START_U * swing, 'rise': up,
                             'from': was, 'u': START_U, 'rate': (0.0,) * 6,
                             'to_go': fall + a['step']},
                    'right': {'stands': False, 't': 1e3, 'from': None, 'to_go': fall}}
    else:
        v = (0.0, 0.0, a['speed'])
        wide = going.WIDE * a['track']
        mid = 0.5 * a['step'] - a['under'] / a['speed']   # s from a landing to the hip over it
        gone = max(0.0, mid - (a['stand'] - a['step']))   # s the other leg has swung
        hip = apply(turn, (-gait.HIP_HALF, -gait.HIP_DROP, 0.0))
        flat = (-0.5 * wide, gait.ANKLE_H, 0.0)
        at = (-going.START_IN * 0.5 * wide, flat[2] - hip[2])
        pelvis = (at[0], strut.reach(-1.0, at, turn, flat, a['strut']), at[1])
        # the left leg as it left: a step behind, the pelvis `gone` s back, its heel as asked
        back = (pelvis[0], pelvis[1], pelvis[2] - a['speed'] * gone)
        left = (0.5 * wide, gait.ANKLE_H, flat[2] - a['speed'] * a['step'])
        up = strut.need(figure.hip(1.0, back, turn), left, law.heading, a['strut'])
        was = figure.leg(1.0, back, turn, strut.ankle(left, up, law.heading), rx(up))
        law.legs = {'right': {'stands': True, 't': mid, 'flat': flat, 'landed': 0.0,
                              'was': None, 'off': (0.0,) * 6},
                    'left': {'stands': False, 't': gone, 'rise': up, 'from': was,
                             'u': gone / swing, 'rate': (0.0,) * 6, 'to_go': a['step'] - mid,
                             'reach': 0.0}}
        law.y = {'t': 1.0, 'span': 1.0, 'landed': pelvis[1], 'rate': 0.0,
                 'from': (pelvis[1], 0.0, 0.0), 'to': (pelvis[1] + 1.0, 0.0, 0.0)}
        stood = {'right': figure.leg(-1.0, pelvis, turn, flat, ry(0.0))}
    angles = free.upper(law, {'left': -20.0, 'right': 20.0})
    for side, sign in walkplan.SIDES:
        joints = stood.get(side) or free.swung(law, law.legs[side], sign, pelvis, turn, v)
        for k, q in zip(LEG, joints):
            angles[side + k] = math.degrees(q)
    angles['left_foot'] = angles['right_foot'] = 0.0
    law.world.reset(angles, where=pelvis, turn=(math.cos(lean / 2), math.sin(lean / 2), 0.0, 0.0),
                    speed=v)
    law.last, law.v = angles, v
    return angles


def formed(rows, segments, window):
    """A window's form: the look's measures of its `rows`, what of `looks.FORM` they break,
    their price; her walk's by place (`normal.measured`), how far off a woman's band, in
    words and by name."""
    if len(rows) <= 30:
        return {}
    from tools.sim import fbx, looks, normal, strides
    got = strides.measured(rows)
    got['energy'] = next((g['drawn'] for g in segments if g['to'] > window[0]), math.inf)
    walk = normal.measured(*fbx.joints(rows))
    far, out = normal.off(walk)
    return dict(got, broken=[(name, v, bound) for name, v, bound in looks.broken(got)],
                price=looks.priced(got), off=far, out=out, said=normal.said(walk),
                named=normal.named(walk), walk={name: float(v) for name, v in walk.items()})


def went(track, to_s, values=None, form=None, shoves=(), asked=False, manner=()):
    """{'fell': s or None, 'segments': [{from, to, k, speed, drawn, work, steps, stance, both,
    knee, ahead, load}], 'forms': [{measure: value, 'broken': [..], 'price': ..}], 'form': the
    first of them, 'steps': [(landed s, side, left s)]} of `to_s` s on `track` [(s, k)] -
    `asked`, each the row wanted from then on,
    hers following it as `gaits.toward` has it - under `values` {name: value}, in the manners
    `manner` [(s, ((manner, amount), ..))], each asked from then on; the look's rows taken
    over `form`, (from, to) or several, shoved as `shoves` [(s, newtons, degrees from behind
    toward her left)], `events.SHOVE_S` each."""
    from coaxial.model.blocks import numpy as np
    from tools.sim import knobs
    values = dict(values or {})
    both = values.pop('both', None)
    knobs.set_(values)
    from machine import Machine, events, gaits, going
    from machine.modes import DYNAMIC
    if both is not None:
        gaits.WALK['stand'] = gaits.WALK['step'] + both
    gaits.derive()
    track = sorted(track)

    row = [track[0][1], 0.0]             # asked: the row she goes on, her seconds on it

    def mixed(now):
        if asked:
            return row[0]
        if now <= track[0][0]:
            return track[0][1]
        for (t0, k0), (t1, k1) in zip(track, track[1:]):
            if now < t1:
                return k0 + (k1 - k0) * (now - t0) / max(1e-9, t1 - t0)
        return track[-1][1]
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    world = body.nodes['pelvis'].world
    law = going.Going(body, gaits.between(mixed(0.0)))
    placed(law)
    body.loop.step(0.0)
    bus, d, at = body.loop.bus, world.data, np.array(world.vadr)
    marks = sorted({p[0] for p in list(track[1:]) + list(manner) if 0.0 < p[0] < to_s}) + [to_s]
    windows = [form] if form and not isinstance(form[0], tuple) else list(form or ())
    amounts, manner = {}, sorted(manner)
    seg = {'from': 0.0, 'z': bus['pelvis.pose.z'], 'drawn': 0.0, 'work': 0.0, 'k': mixed(0.0),
           'load': 0.0}
    stand_in = types.SimpleNamespace(stage='walk', walker=types.SimpleNamespace(phase=0.0),
                                     touched_at=None)
    segments, rows, said, fell, shoves = [], [[] for _w in windows], -1.0, None, sorted(shoves)

    def close(end):
        on, span = bus['pelvis.pose.z'] - seg['z'], end - seg['from']
        steps = [s for s in law.steps if seg['from'] <= s['t'] < end]
        stood = [s['off'] - s['t'] for s in steps if 'off' in s]
        n = max(1, len(steps))
        segments.append({'from': seg['from'], 'to': end, 'k': (seg['k'], mixed(end)),
                         'speed': on / span if span else 0.0,
                         'drawn': seg['drawn'] / on if on > 0.1 else math.inf,
                         'work': seg['work'] / on if on > 0.1 else math.inf,
                         'steps': len(steps), 'stance': sum(stood) / max(1, len(stood)),
                         'both': sum(1 for s in steps if s['both']) / n,
                         'knee': sum(s['knee'] for s in steps) / n,
                         'ahead': sum(s['ahead'] for s in steps) / n, 'load': seg['load']})
        seg.update({'from': end, 'z': bus['pelvis.pose.z'], 'drawn': 0.0, 'work': 0.0,
                    'k': mixed(end), 'load': 0.0})
    while bus['t'] < to_s:
        accents = next((m for t, m in reversed(manner) if t <= bus['t']), ())
        if asked:
            row[0], row[1], amounts = gaits.passed(row[0], row[1], next(
                k for t, k in reversed(track) if t <= bus['t'] or t == track[0][0]), amounts,
                accents, 0.001)
        else:
            amounts = gaits.manners(amounts, accents, 0.001, mixed(bus['t']) > -1.0)
        law.ask = gaits.mannered(gaits.between(mixed(bus['t'])), amounts)
        if shoves and bus['t'] >= shoves[0][0]:
            _at, newtons, way = shoves.pop(0)
            world.push((newtons * math.sin(math.radians(way)), 0.0,
                        newtons * math.cos(math.radians(way))), events.SHOVE_S)
        body.loop.write(**law.step(0.001))
        body.loop.step(0.001)
        seg['drawn'] += world.drawn() * 0.001
        power = np.array(d.ctrl[:len(at)]) * np.array(d.qvel[at])
        seg['work'] += float(power[power > 0.0].sum()) * 0.001
        seg['load'] = max(seg['load'], bus['pelvis.pose.left_load'], bus['pelvis.pose.right_load'])
        if bus['t'] - said >= 1.0 / ROWS_HZ and any(a <= bus['t'] < b for a, b in windows):
            from tools.sim import look
            said, seen = bus['t'], look.sample(bus, stand_in, world)
            for (a, b), into in zip(windows, rows):
                if a <= bus['t'] < b:
                    into.append(seen)
        if marks and bus['t'] >= marks[0]:
            close(marks.pop(0))
        up = 1.0 - 2.0 * (bus['pelvis.pose.qx'] ** 2 + bus['pelvis.pose.qz'] ** 2)
        if (bus['pelvis.pose.y'] < FELL_M
                or math.degrees(math.acos(max(-1.0, min(1.0, up)))) > FELL_DEG):
            fell = bus['t']
            close(fell)
            break
    forms = [formed(r, segments, w) for w, r in zip(windows, rows)]
    body.disarm()
    body.close()
    return {'fell': fell, 'segments': segments, 'form': forms[0] if forms else {},
            'forms': forms, 'steps': [(s['t'], s['side'], s.get('off')) for s in law.steps]}


def searched(kind, generations, lam, log_path, sigma=0.2):
    """(cost, {name: value}) of CMA-ES (`tools.sim.cmaes`) over SPANS[`kind`] from the rows as
    they are, a candidate a `go.py --json` on the relay, each a line of `log_path`. The walk's
    row costs its price from FROM_S of WALK_S on each of STARTS - down before then, what she
    did not walk of her way, over any price; her jog's, what of PASSAGES she was not up for
    and a little of her last J/m."""
    from machine import free, gaits, going, strut
    from tools.dev import focus
    from tools.sim import cmaes
    walks = kind == 'walk'
    to_s = WALK_S if walks else PASS_S
    # a trial: its track and what else it sets
    trials = ([('--track', '0:-1 %g:-1 %g:0' % (k, k + 1.0), ['--form', '%g,%g' % (FROM_S, to_s)])
               for k in STARTS] + [STOP, PASSAGE] if walks
              else [('--ask', track, []) for track in PASSAGES])

    def now(name):
        if name == 'both':
            return gaits.WALK['stand'] - gaits.WALK['step']
        owner, name, *key = name.split('.')
        value = getattr({'gaits': gaits, 'going': going, 'strut': strut, 'free': free}[owner],
                        name)
        return float(value[key[0]] if key else value)

    def cost(result, values):
        if result is None:
            return 400.0
        up, last = result['fell'] or to_s, result['segments'][-1]
        if not walks:
            return 10.0 * (to_s - up) / to_s + 3e-4 * min(last['drawn'], 4000.0)
        if walks and not result['form'] and not result['fell']:        # her stop
            return 0.0 if abs(last['speed']) < 0.05 and last['steps'] <= 3 else 300.0
        price = result['form'].get('price', math.nan)
        if not result['fell'] and price == price:
            return min(280.0, price + BAND_K * result['form'].get('off', 10.0)
                       + result['form'].get('energy', 2000.0) / ENERGY_J_M)
        way = values.get('gaits.WALK.speed', gaits.WALK['speed']) * to_s
        return 300.0 + 10.0 * (1.0 - max(0.0, min(1.0, last['speed'] * up / way)))

    def runs(_pool, cands):
        jobs = [focus.Job('%d|%d' % (i, k), [sys.executable, '-X', 'utf8', os.path.abspath(__file__),
                                              '--json', '--to', str(
                                                  PASSAGE_S if track == PASSAGE[1] else to_s),
                                              how, track]
                          + more + ['%s=%r' % kv for kv in c.items()], 1.2, 600.0)
                for i, c in enumerate(cands) for k, (how, track, more) in enumerate(trials)]
        out = {}
        for job, text, _code, _s in focus.relay(jobs):
            line = next((ln[7:] for ln in reversed(text.splitlines())
                         if ln.startswith('RESULT ')), None)
            out[job.name] = json.loads(line) if line else None
        got = []
        for i, c in enumerate(cands):
            each = [out['%d|%d' % (i, k)] for k in range(len(trials))]
            got.append((sum(cost(r, c) for r in each) / len(each),
                        min((r['fell'] or to_s) if r else 0.0 for r in each), 0.0, None))
        return got
    with open(log_path, 'a', encoding='utf-8') as log:
        return cmaes.search(None, SPANS[kind], generations, lam, log, runs, now, sigma=sigma)


def shown(result):
    for g in result['segments']:
        print('%5.1f-%5.1f s k %.2f>%.2f | %.2f m/s | %4.0f J/m drawn, %4.0f of work | %2d '
              'steps, stance %.3f s, both down %3.0f %%, knee %4.1f deg at landing, %.3f m '
              'ahead, a sole %4.0f N' % (
                  g['from'], g['to'], g['k'][0], g['k'][1], g['speed'], g['drawn'], g['work'],
                  g['steps'], g['stance'], 100.0 * g['both'], g['knee'], g['ahead'], g['load']))
    for form in filter(None, result['forms']):
        from tools.sim import looks
        print('form: ' + ' | '.join('%s %.1f' % (k, form.get(k, math.nan))
                                    for k, _least, _most in looks.FORM))
        print('off it: %s; priced %.1f' % (', '.join('%s %.1f (%g)' % b for b in form['broken'])
                                           or 'nothing', form['price']))
        print("off a woman's walk %.2f: %s" % (form['off'], ', '.join(
            '%s %.3g (%g)' % tuple(o) for o in form['out'])))
        print('in words: %s%s' % (', '.join('%s %.2f' % tuple(w) for w in form['said'])
                                  or 'none', ''.join(' - ' + n for n in form['named'])))
    print('fell at %.2f s' % result['fell'] if result['fell'] else 'up')


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--track', default='0:0', help='"t:k ..", k -1 her stand, 0 the walk, '
                        '1 the run')
    parser.add_argument('--ask', help='"t:k ..": the row wanted from t on, hers following it '
                        '(`gaits.toward`), in place of --track')
    parser.add_argument('--to', type=float, default=12.0, help='seconds simulated')
    parser.add_argument('--form', help='"from,to ..", s: the look sampled there and priced')
    parser.add_argument('--manner', default='', help='"t:manner:amount,manner:amount; ..": '
                        'her manners asked from t on (`gaits.MANNERS`)')
    parser.add_argument('--shove', default='', help='"t:newtons:deg ..", 0 from behind, 90 '
                        'toward her left')
    parser.add_argument('--json', action='store_true', help='the result, a line')
    parser.add_argument('--search', choices=sorted(SPANS), help='CMA-ES over its spans')
    parser.add_argument('--generations', type=int, default=60)
    parser.add_argument('--lam', type=int, default=32, help='candidates a generation')
    parser.add_argument('--sigma', type=float, default=0.2, help='the first step, of a span')
    parser.add_argument('--log', default='build/go_search.jsonl', help='a candidate a line')
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    if args.search:
        print('BEST ' + json.dumps(searched(args.search, args.generations, args.lam, args.log,
                                            args.sigma)))
        return 0
    track = [tuple(float(x) for x in p.split(':')) for p in (args.ask or args.track).split()]
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    form = [tuple(float(x) for x in w.split(',')) for w in (args.form or '').split()]
    from machine import gaits
    result = went(track, args.to, values, form,
                  [tuple(float(x) for x in p.split(':')) for p in args.shove.split()],
                  bool(args.ask), [(float(m.split(':')[0]), gaits.told(m.split(':', 1)[1]))
                                   for m in args.manner.split(';') if m.strip()])
    if args.json:
        print('RESULT ' + json.dumps(result))
    else:
        shown(result)
    return 0


if __name__ == '__main__':
    sys.exit(main())
