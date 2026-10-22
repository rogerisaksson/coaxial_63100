#!/usr/bin/env python3
"""Her scenarios in MuJoCo: a scene placed, commanded as a player would, judged by the world.

    python tools/sim/scenarios.py                    # every scenario on the relay, a table
    python tools/sim/scenarios.py wall chair         # those named; --shoves the shoves' polar too
    python tools/sim/scenarios.py --one wall         # one, its line as JSON (the relay's job)
    python tools/sim/stunts.py                       # the takes' scenarios (STUNTS) against their takes

A scenario is a scene (`floor.scenario`, or a chair or a bed set at her) and its commands at their
seconds, given through her worker's own session (`worker.apply`, `worker.stepped`) as the page
gives them. The referee reads the world's truth her law never sees: whether she fell
(`worker._is_fallen`), what bore her off her soles - the floor, a chair, a mattress, a wall -,
her soles' loads, where her pelvis went, and each segment's jerk (`JERKED`) through a 20 ms
window, its 99th percentile and its peak; a stunt's rows as the page records her, measured as
its take is (`stunts.measured`).
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine import floor as fl
from machine import sprung, takes
from tools.sim import stunts

#: A player's walk (`se2`'s), m/s; a stop.
WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)
#: What bears her, by the body it is: the props a scene places.
BEARS = {'chair': 'chair', 'prop_bed': 'bed', 'wall': 'wall', 'tree': 'tree'}
#: A goal held this long, s; jerk through this window, s; an obstacle met past this, N; standing
#: on after the last command, s.
HELD_S, JERK_S, MET_N, STILL_S = 1.0, 0.02, 20.0, 6.0
#: The segments whose jerk is judged: her core, her head, her hands and her feet.
JERKED = ('pelvis', 'torso', 'head', 'left_hand', 'right_hand', 'left_foot', 'right_foot')


def _walk(seconds, then=STILL_S):
    """Walk from 1 s for `seconds`, then stop and stand `then` s: (commands, seconds)."""
    return [(1.0, {'twist': WALK}), (1.0 + seconds, {'twist': STOP})], 1.0 + seconds + then


#: name: (scene, commands, seconds, goal, its number). A goal: 'met' (the obstacle touched and
#: her stopped at it), 'past' m (her pelvis that far on), 'rose'/'sank' m (her pelvis that much
#: higher or lower at the end), 'lay' (a bed bearing more of her than her soles do for HELD_S,
#: by the world's contacts), 'sat' (so, and her soles bearing her at the end: risen), 'stands'
#: (on her soles, a chair's seat or not). The bed and the chair by their texts; by the takes'
#: rows she perched on the chair's edge and fell at one wobble of three, and sat at the bed's
#: edge as the human did and collapsed back from there (stunts.md).
#: A stairs take's window opens as her first sole lands on the first tread, its first frame's.
STAIRS_TAKE_S = 1.5


SCENARIOS = {
    'wall': ('wall', *_walk(4.0), 'met', 'wall'),
    'tree': ('tree', *_walk(4.0), 'met', 'tree'),
    'sill': ('sill', *_walk(5.0), 'past', 1.0),
    'rug': ('rug', *_walk(5.0), 'past', 1.5),
    'hole': ('hole', *_walk(5.0), 'past', 1.3),
    'slip': ('slip', *_walk(5.0), 'past', 1.2),
    'ice': ('ice', *_walk(6.0), 'past', 1.7),
    'ice_told': ('ice', [(0.5, {'grip': 0.1})] + _walk(13.0)[0], _walk(13.0)[1], 'past', 2.8),
    'snow': ('snow', *_walk(6.0), 'past', 2.0),
    'asphalt': ('asphalt', *_walk(6.0), 'past', 2.0),
    'stairs': ('stairs', [(0.0, {'floor': fl.told_stairs()})] + _walk(10.0)[0], _walk(10.0)[1], 'rose', 0.70),
    'stairs_down': ('stairs_down', *_walk(10.0), 'sank', 0.70),
    'lodge_down': ('lodge', *_walk(14.0), 'sank', 0.70),
    'slope': ('slope', *_walk(7.0), 'rose', 0.45),
    'slope_down': ('slope_down', *_walk(7.0), 'sank', 0.45),
    'cross_slope': ('cross_slope', *_walk(6.0), 'past', 2.0),
    'chair': ('chair_behind', [(1.0, {'say': 'sit_chair'}), (8.0, {'say': 'rise_chair'})], 14.0, 'sat', 'chair'),
    'bed': ('bed_behind', [(1.0, {'say': 'lie_bed'})], 16.0, 'lay', 'bed'),
    'bed_rise': ('bed_behind', [(1.0, {'say': 'lie_bed'}), (16.0, {'say': 'rise_bed'})], 24.0, 'sat', 'bed'),
    'mattress': ('mattress_under', [(1.0, {'say': 'lie_floor'}), (9.0, {'say': 'rise_floor'})], 16.0, 'lay', 'bed'),
    'hop_down': ('on_chair', [(1.0, {'say': 'hop_down_chair'})], 6.0, 'sank', 0.40),
    'hop_up': ('chair_ahead', [(1.0, {'say': 'hop_up_chair'})], 6.0, 'rose', 0.40),
}
#: A player's twists, a hold each, s: the page's keys send none outside them
#: (`humanoid_keys.twist`); an agent sends the same.
KEYS = {'w': WALK, 's': (-0.35, 0.0, 0.0), 'a': (0.0, 0.5, 0.0), 'd': (0.0, -0.5, 0.0),
        'wa': (0.69, 0.0, 0.7), 'wd': (0.69, 0.0, -0.7), 'q': (0.0, 0.0, 0.7), 'W': (1.4, 0.0, 0.0), '-': STOP}
PLAYS = {'start_stop': 'w3 -3 w3 -3', 'turns': 'w2 wa2 wd2 -3', 'strafes': 'a2 d2 -3', 'reverse': 'w2 s2 w2 -3',
         'sprint': 'w2 W3 w2 -3', 'spin': 'q3 -3', 'weave': 'w1 wa1 w1 wd1 w1 -3', 'about': 'q2 wa1 w2 -3',
         'walk': 'w10 -2', 'moves': 'w2 wa2 a2 s2 W2 -3', 'stutter': 'w3 -0.25 w2 -0.2 w3 -3'}


def _played(keys):
    """(commands, seconds) of a player's keys, 'w3 -2': each a twist held its seconds, from 1 s."""
    at, plan = 1.0, []
    for word in keys.split():
        plan.append((at, {'twist': KEYS[word.rstrip('0123456789.')]}))
        at += float(word[len(word.rstrip('0123456789.')):])
    return plan, at + 1.0


SCENARIOS.update({'play_' + n: ('flat', *_played(k), 'stands', None) for n, k in PLAYS.items()})

#: Shoves: N for SHOVE_S, the way they push (deg about up from ahead, 90 to her left), standing or
#: walking, at one of MOMENTS - a quarter of her stride apart, the foot that swings decides; 80 N
#: held, 200 N held or fallen with her head slower than FALL_M_S onto the floor.
FORCES_N, SHOVE_S, MOMENTS = (80, 200), 0.12, (5.0, 5.15, 5.30, 5.45)
SHOVES = {'shove_%d_%03d_%s_%d' % (n, way, gait, k): (n, way, gait, at)
          for n in FORCES_N for way in range(0, 360, 45) for gait in ('stand', 'walk')
          for k, at in enumerate(MOMENTS)}
FALL_M_S = 1.0

#: The takes as scenarios, each judged against its take (`tools/sim/stunts.py`): key: (the take
#: under stunts/, scene, commands, from when her rows are measured - the telling, or MOMENTS[0]
#: with a gait under way -, the measures of its signature). A posture's take is told as its own
#: rows ({'take': key}, `stunts.wire`), a gait's as a player's twist, a jump's as the hop's text,
#: a fall's as a shove; a verb she has no text for is refused by name: backflip. The falls'
#: signature is SAFE alone: a shove of the polar's most, the slick patch and the ice, a step's
#: edge behind her.
_BACK, _RUN, _SHOVED = KEYS['s'], KEYS['W'], FORCES_N[1]
#: A gait take's pattern laid over her walk said, as the walk's last entry (`sprung.WALK`) has
#: entered: forward over the walk, backward over the walk back (on within 1.5 s of its saying).
PATTERN_S = sprung.WALK[-1][1] + takes.ENTER_S
_PATTERN = lambda key: [(1.0, {'say': 'walk'}), (PATTERN_S, {'pattern': key})]
_BACK_PATTERN = lambda key: [(1.0, {'say': 'walk_back'}), (PATTERN_S, {'pattern': key})]
#: The stairs takes' flight told at 0 s where the scene stands, the walk by twist: a climb's pattern
#: (its tempo, drive and lean over the walk) hopped her onto the first tread and toppled her
#: backward at 4.0 s where the walk climbs the flight (sequencer.md).
_FLIGHT = [(0.0, {'floor': fl.told_stairs(fl.TAKE_RISE_M, fl.TAKE_RUN_M)}), (1.0, {'twist': WALK})]
STUNTS = {
    'hip_hop': ('dance/Hip Hop Dancing', 'flat', [(1.0, {'take': 'hip_hop'})], 1.0, ('travel', 'drop', 'arms')),
    'moonwalk': ('dance/Moonwalk', 'flat', _BACK_PATTERN('moonwalk'), PATTERN_S, ('travel', 'speed', 'flight', 'upright')),
    'falling_on_butt': ('falling/Falling on Butt', 'flat',
                        [(1.0, {'push': (0.0, 0.0, -_SHOVED), 'seconds': SHOVE_S})], 1.0, ()),
    'slipping_on_a_patch': ('falling/Slipping Backwards', 'slip', [(1.0, {'twist': WALK})], 1.0, ()),
    'tripping_forward': ('falling/Tripping Forward', 'flat',
                         [(0.5, {'twist': WALK}), (MOMENTS[0], {'push': (0.0, 0.0, _SHOVED), 'seconds': SHOVE_S})],
                         MOMENTS[0], ()),
    'backflip': ('flips/Backflip', 'flat', [(1.0, {'say': 'backflip'})], 1.0, ('rise', 'flight', 'pitched')),
    'run_to_flip': ('flips/Run To Flip', 'flat', [(0.5, {'twist': _RUN}), (MOMENTS[0], {'say': 'backflip'})],
                    MOMENTS[0], ('rise', 'flight', 'pitched')),
    'jump_down': ('jumping/Jump Down', 'on_chair', [(1.0, {'say': 'hop_down_chair'})], 1.0, ('ended', 'travel')),
    'jumping_down': ('jumping/Jumping Down', 'on_chair', [(1.0, {'say': 'hop_down_chair'})], 1.0, ('ended', 'travel')),
    'jumping_up': ('jumping/Jumping Up', 'chair_ahead', [(1.0, {'say': 'hop_up_chair'})], 1.0, ('ended', 'travel')),
    'running_jump': ('jumping/Running Jump', 'flat', [(0.5, {'twist': _RUN}), (MOMENTS[0], {'charge': 1.0, 'say': 'hop'})],
                     MOMENTS[0], ('flight', 'travel', 'speed')),
    'crouch_up': ('kneel/Crouch Up', 'flat', [(1.0, {'take': 'crouching_down'}), (6.0, {'take': 'crouch_up'})], 6.0,
                  ('ended', 'lasted')),
    'crouching_down': ('kneel/Crouching Down', 'flat', [(1.0, {'take': 'crouching_down'})], 1.0, ('drop', 'ended', 'knee', 'lasted')),
    'kneeling_down': ('kneel/Kneeling Down', 'flat', [(1.0, {'take': 'kneeling_down'})], 1.0, ('drop', 'ended', 'knee', 'lasted')),
    'kneeling_up': ('kneel/Kneeling Up', 'flat', [(1.0, {'take': 'kneeling_down'}), (6.0, {'take': 'kneeling_up'})], 6.0,
                    ('ended', 'lasted')),
    'lying_down_from_sitting': ('lying/Lying Down from Sitting', 'bed_behind',
                                [(1.0, {'take': 'sitting_on_bed'}), (7.0, {'take': 'lying_down_from_sitting'})], 7.0,
                                ('upright', 'hips back', 'ended', 'lasted')),
    'sitting_on_bed': ('lying/Sitting on Bed', 'bed_behind', [(1.0, {'take': 'sitting_on_bed'})], 1.0,
                       ('drop', 'hips back', 'upright', 'knee', 'lifted', 'lasted')),
    'standing_yawn': ('lying/Standing Yawn', 'flat', [(1.0, {'take': 'standing_yawn'})], 1.0, ('arms', 'back', 'travel')),
    'turning_in_bed': ('lying/Turning in Bed', 'mattress_under',
                       [(1.0, {'say': 'lie_floor'}), (9.0, {'take': 'turning_in_bed'})], 9.0, ('rolled', 'turned', 'upright')),
    'jogging_backwards': ('run/Jogging Backwards', 'flat', _BACK_PATTERN('jogging_backwards'), PATTERN_S,
                          ('travel', 'speed', 'upright')),
    'jogging_forward': ('run/Jogging Forward', 'flat', _PATTERN('jogging_forward'), PATTERN_S, ('travel', 'speed', 'flight')),
    'running_backward': ('run/Running Backward', 'flat', _BACK_PATTERN('running_backward'), PATTERN_S, ('travel', 'speed')),
    'running_forward': ('run/Running Forward', 'flat', _PATTERN('running_forward'), PATTERN_S, ('travel', 'speed', 'flight')),
    'sit_to_stand': ('sitting/Sit To Stand', 'chair_behind', [(1.0, {'take': 'stand_to_sit'}), (8.0, {'take': 'sit_to_stand'})], 8.0,
                     ('ended', 'forward', 'lasted')),
    'situp_to_stand': ('sitting/Situp To Stand', 'mattress_under',
                       [(1.0, {'say': 'lie_floor'}), (9.0, {'take': 'situp_to_stand'})], 9.0, ('ended', 'upright', 'lasted')),
    'stand_to_sit': ('sitting/Stand To Sit', 'chair_behind', [(1.0, {'take': 'stand_to_sit'})], 1.0,
                     ('drop', 'hips back', 'forward', 'knee', 'lifted', 'lasted')),
    'edge_tripping_backwards': ('slipping/Edge Tripping Backwards', 'edge_behind', [(1.0, {'twist': _BACK})], 1.0, ('ended',)),
    'slipping_backwards': ('slipping/Slipping Backwards', 'ice', [(1.0, {'twist': WALK})], MOMENTS[0], ()),
    'ascending_stairs': ('stairs/Ascending Stairs', 'stairs_take', _FLIGHT, STAIRS_TAKE_S, ('ended', 'travel')),
    'walking_up_the_stairs': ('stairs/Walking Up The Stairs', 'stairs_take', _FLIGHT, STAIRS_TAKE_S, ('ended', 'travel')),
    'standing_jump': ('standing/Standing Jump', 'flat', [(1.0, {'charge': 1.0, 'say': 'hop'})], 1.0, ('rise', 'flight', 'travel')),
}
SCENARIOS.update({key: (scene, plan, start + ref['seconds'] + STILL_S, 'stunt', key)
                  for key, (_take, scene, plan, start, _signature) in STUNTS.items()
                  for ref in [stunts.reference().get(key, {'seconds': 0.0})]})


def placed(s, scene):
    """`scene` laid round her in her session `s`: a floor event, or a chair or a bed set at her."""
    from machine import worker
    worker.apply(s, {'scenario': scene})


def _row(g, world):
    """Her look's row now, as the page records her (`humanoid_keys.row`)."""
    from machine.figure import JOINTS
    from terminal.views.humanoid_keys import HEADER, row
    d = world.data
    angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
    now = {'t': g['t'], 'stage': '', 'speed': float(d.qvel[2]), 'phase': 0.0,
           'loads': tuple(float(v) for v in g['loads']), 'where': tuple(d.qpos[0:3].tolist()),
           'turn': tuple(d.qpos[3:7].tolist()), 'angles': angles, 'set': angles, 'watts': 0.0}
    return dict(zip(HEADER, row(now, 0.0)))


def stunt_window(key):
    """(from, to) s of a stunt's measured rows: from its telling and the take's entering
    (`takes.ENTER_S`), for its take's seconds at her size (`stunts.similar`)."""
    ref, start = stunts.reference()[key], STUNTS[key][3] + takes.ENTER_S
    return start, start + ref['seconds'] * stunts.similar(ref['leg'], 'time')


def borne(world, hers, kinds, force):
    """({kind: N up}, {kind: N}) of what she touches now (`hers` 2 a sole's body, 1 another of
    hers): her soles ('soles'), and the rest by the body her contact is with (`kinds`, else
    'floor') - what holds her up, the normal forces' upward parts (her shins on a chair's edge
    pushed it 1.8 kN sideways), and the normal forces whole (a wall's push)."""
    m, d, up, whole = world.model, world.data, {}, {}
    for i in range(d.ncon):
        c = d.contact[i]
        a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if (hers[a] > 0) == (hers[b] > 0):
            continue
        mine, other = (a, b) if hers[a] else (b, a)
        world._mj.mj_contactForce(m, d, i, force)
        kind = 'soles' if hers[mine] > 1 else kinds.get(m.body_rootid[other], 'floor')
        up[kind] = up.get(kind, 0.0) + abs(float(force[0]) * float(c.frame[1]))
        whole[kind] = whole.get(kind, 0.0) + abs(float(force[0]))
    return up, whole


def ran(name, wobble=0.0):
    """{name, stood, fell_at, goal, met, went m, rose m, rolled deg (her pelvis about her way at
    the end), speed m/s, bore {kind: share held}, jerk}: laid turned `wobble` rad about up."""
    import numpy as np
    from tools.sim import look
    from machine import edge, modal, physics, sprung, worker
    shove = SHOVES.get(name)
    if shove:
        n, way, gait, at = shove
        yaw = math.radians(way)
        plan = ([(0.5, {'twist': WALK})] if gait == 'walk' else []) + [
            (at, {'push': (n * math.sin(yaw), 0.0, n * math.cos(yaw)), 'seconds': SHOVE_S})]
        scene, seconds, goal, aim = 'flat', 9.0, 'held', n
    else:
        scene, plan, seconds, goal, aim = SCENARIOS[name]
    # the lodge round her for its own scenes alone: its walls would end the longer walks
    physics.ROOM, physics.SIGHT = float(scene == 'lodge'), 0.0
    told = [c['say'] for _at, c in plan if 'say' in c]
    refused = sorted(set(told) - {e[0] for e in sprung.entries()})
    if refused:
        return {'name': name, 'stood': False, 'fell_at': None, 'goal': goal, 'aim': aim, 'refused': refused[0],
                'went': math.nan, 'rose': math.nan, 'speed': math.nan, 'bore': {}, 'on_soles': math.nan,
                'jerk': {seg: math.nan for seg in JERKED}, 'met': False}
    s = worker.session()
    placed(s, scene)
    world = s['g']['world']
    q = world.data.qpos[3:7].copy()
    world.data.qpos[3:7] = (q[0] * math.cos(wobble / 2.0) - q[2] * math.sin(wobble / 2.0), q[1] * math.cos(wobble / 2.0) + q[3] * math.sin(wobble / 2.0),
                            q[2] * math.cos(wobble / 2.0) + q[0] * math.sin(wobble / 2.0), q[3] * math.cos(wobble / 2.0) - q[1] * math.sin(wobble / 2.0))
    world._mj.mj_forward(world.model, world.data)
    m, d = world.model, world.data
    hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)], int)
    hers[[m.body(n).id for n in edge.INSOLES]] = 2
    kinds = {m.body(n).id: k for n, k in BEARS.items()}
    weight = float(s['g']['fr'][modal.WEIGHT])
    head, force = m.body('head').id, np.zeros(6)
    jerked = [m.body(n).id for n in JERKED]
    from machine import wrench
    leg_cols = [s['g']['body'].act[leg] for leg in wrench.LEGS]
    track, fell, hit, k, bore, most = [], None, None, 0, {}, {}
    rows, told_at = [], -math.inf
    sigma_min, scale_min, shrunk_n, qacc_max = math.inf, 1.0, 0, 0.0
    plan = sorted(((at, stunts.told(c)) for at, c in plan), key=lambda c: c[0])
    while s['g']['t'] < seconds and s['epoch'] == 0:
        while k < len(plan) and s['g']['t'] >= plan[k][0]:
            worker.apply(s, plan[k][1])
            k += 1
        worker.stepped(s)
        g = s['g']
        if g['n'] % g['every'] == 1 and 'got' in g and 's' in g:
            got, gs = g['got'], g['s']
            sc = float(got['scale'])
            scale_min = min(scale_min, sc)
            shrunk_n += int(sc < 1.0 - 1e-6)
            qacc_max = max(qacc_max, float(np.abs(got['qacc'][g['body'].act]).max()))
            for side in (0, 1):
                sv = float(np.linalg.svd(gs['feet'][side]['J6'][:, leg_cols[side]], compute_uv=False).min())
                sigma_min = min(sigma_min, sv)
        if s['fallen_at'] is not None and fell is None:
            fell = round(g['t'], 3)
        speed_head = float(np.linalg.norm(d.cvel[head][3:]))
        if fell is not None and hit is None and float(d.xpos[head][1]) < 0.25:
            hit = speed_head
        track.append((g['t'], *d.qpos[0:3].tolist(), *d.xpos[jerked].ravel().tolist()))
        if g['t'] - told_at >= 1.0 / look.RATE_HZ:
            told_at = g['t']
            rows.append(_row(g, world))
        if g['n'] % 10 == 0:
            now, whole = borne(world, hers, kinds, force)
            for kind, n in whole.items():
                most[kind] = max(most.get(kind, 0.0), n)
            for kind, n in now.items():
                if kind != 'soles' and n > now.get('soles', 0.0):
                    bore[kind] = bore.get(kind, 0.0) + 0.01
            g['_borne'] = now
    t = np.array(track)
    w = max(1, int(JERK_S / 0.001))
    smooth = lambda p: np.cumsum(p, axis=0)[w:] / w - np.cumsum(p, axis=0)[:-w] / w
    jerks = lambda p: np.linalg.norm(np.diff(smooth(p), n=3, axis=0), axis=1) / 0.001 ** 3
    cut = lambda f: {seg: round(float(f(jerks(t[:, 4 + 3 * k:7 + 3 * k]))), 1) if len(t) > 4 * w else math.nan
                     for k, seg in enumerate(JERKED)}
    end = s['g'].get('_borne', {})
    qw, qx, qy, qz = (float(v) for v in d.qpos[3:7])
    out = {'name': name, 'stood': fell is None, 'fell_at': fell, 'goal': goal, 'aim': aim,
           'went': round(float(t[-1, 3] - t[0, 3]), 3), 'rose': round(float(t[-1, 2] - t[0, 2]), 3),
           'rolled': round(abs(math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))), 1),
           'speed': round(float(np.linalg.norm(t[-1, 1:4] - t[-51, 1:4]) / 0.05), 3),
           'bore': {kd: round(v, 2) for kd, v in bore.items()},
           'touched': {kd: round(v) for kd, v in most.items() if kd not in ('floor', 'soles')},
           'on_soles': round(sum(s['g']['loads']) / weight, 2), 'head_hit': hit,
           'sigma_min': round(sigma_min, 4), 'scale_min': round(scale_min, 3),
           'shrunk': shrunk_n, 'qacc_max': round(qacc_max, 1),
           'jerk': cut(lambda j: np.percentile(j, 99)), 'jerk_peak': cut(np.max)}
    if goal == 'stunt':
        out['moved'] = stunts.moved(rows, *stunt_window(aim))
    out['met'] = judged(out)
    return out


def judged(out):
    """Whether a run met its goal, by the world's truth."""
    goal, aim = out['goal'], out['aim']
    if goal == 'stunt':
        return ('refused' not in out and stunts.met(out['moved'], stunts.reference()[aim], STUNTS[aim][4])
                and (out['head_hit'] is None or out['head_hit'] < FALL_M_S))
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
    if goal == 'lay':
        from machine import modal
        return out['bore'].get(aim, 0.0) >= HELD_S and out['rolled'] <= math.degrees(modal.LIST_RAD)
    if goal == 'sat':
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
        if 'refused' in r:
            print('%-22s refused %r: no text' % (n, r['refused']))
            continue
        print('%-22s %-5s %-7s %-6s %-6s %6.2f %6.2f %6.2f %-22s %5.2f %8.0f %8.0f' % (
            n, r['stood'], r['fell_at'] or '-', r['goal'], r['met'], r['went'], r['rose'], r['speed'],
            ' '.join('%s %.1f' % kv for kv in r['bore'].items()), r['on_soles'], r['jerk']['pelvis'], r['jerk']['head']))
    print('%d of %d met' % (sum(bool(r.get('met')) for r in rows.values()), len(rows)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
