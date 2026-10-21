#!/usr/bin/env python3
"""The takes as her scenarios: a move's measures from joints' places, a take's and hers alike.

    python tools/sim/stunts.py                      # every take's scenario on the relay, a table
    python tools/sim/stunts.py stand_to_sit backflip
    python tools/sim/stunts.py --save build/stunts.json [--against build/was.json]
    python tools/sim/stunts.py --reference          # the takes measured anew into tests/takes/stunts.json
    measures = stunts.measured(times, joints)       # {name: value} of a take's (`fbx.take`) or her rows'
    got = stunts.evaluate(); stunts.total(got)      # every stunt run, how far off its take, the one number

The takes lie in stunts/<folder>/ beside the repo (dance, falling, flips, jumping, kneel, lying,
run, sitting, slipping, stairs, standing), read once into tests/takes/stunts.json: a take's
seconds, its leg and MOVES' measures, the reference each scenario in `scenarios.STUNTS` is
judged by - the scene placed, her told as a player would, her rows measured here over the
take's seconds from the telling, and each measure of the scenario's signature within WITHIN of
the take's: a sole's length, her list, her arm's cone, one natural time. Whatever she does, her
head goes no lower than the human's (SAFE).

The take's human and she are alike by dynamic similarity (SIMILAR): the measures are in legs,
sqrt(leg / g) and sqrt(g leg), so neither size enters them; what does is a woman of her size,
her weight (`figure.MASS_KG`). Every measure is kinematic, of her body: her drives' demand
(`scenarios.ran`'s scale_min, shrunk) is reported beside, never in them, so a motor or a ratio
changed alters what she reaches, not what a score or a tuned parameter means.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap
from machine import figure, modal
from tools import REPO

G = 9.81
#: The takes beside the repo; their measures kept here.
TAKES = os.path.join(os.path.dirname(REPO), 'stunts')
REFERENCE = os.path.join(REPO, 'host', 'tests', 'takes', 'stunts.json')

#: A move's measures, (name, unit, side): lengths in legs (hip to knee to ankle), times in
#: sqrt(leg / g), speeds in sqrt(g leg), so a human's and hers read alike. A side of 1 is met by
#: more, 0 within the tolerance either way. Heights are over the lower foot's lowest; ahead is
#: where the hips faced at the start.
MOVES = (
    ('lasted', 'sqrt(leg/g)', 0),      # the hips' rise time from their start to their end, RISE of the way
    ('drop', 'legs', 0),               # the hips' lowest under their start
    ('rise', 'legs', 0),               # the hips' highest over their start
    ('ended', 'legs', 0),              # the hips at the end over their start
    ('travel', 'legs', 0),             # the hips along the floor, ahead
    ('speed', 'sqrt(g leg)', 0),       # the hips' peak along the floor, over one natural time
    ('flight', 'legs', 0),             # the lower foot over its lowest within the natural half period before
    ('lifted', 'legs', 0),             # either foot over its lowest within the natural half period before
    ('forward', 'deg', 0),             # the trunk's pitch ahead, most
    ('back', 'deg', 0),                # the trunk's pitch back, most
    ('pitched', 'deg', 0),             # the trunk's pitch, its whole range, unwrapped: a flip's 360
    ('upright', 'deg', 0),             # the trunk off plumb at the end: lying 90
    ('rolled', 'deg', 0),              # the hips' line off level, most
    ('turned', 'deg', 0),              # the hips' line's heading at the end off its start
    ('head', 'legs', 1),               # the head's lowest
    ('knee', 'limb deg', 0),           # a knee's most bent
    ('hips back', 'legs', 0),          # the hips behind the ankles at the end
    ('arms', 'limb deg', 0),           # an upper arm's swing off the trunk, its range
)
#: Within what a measure is the take's: her sole's length and the speed carrying her capture
#: point as far in one natural time (both a sole over a leg), her list, her arm's cone, one
#: natural time.
WITHIN = {'legs': (figure.BALL + figure.HEEL) / modal.LEG_M,
          'sqrt(g leg)': (figure.BALL + figure.HEEL) / modal.LEG_M,
          'sqrt(leg/g)': 1.0, 'deg': math.degrees(modal.LIST_RAD), 'limb deg': math.degrees(modal.ARM_RAD)}
#: Judged whatever the scenario's signature.
SAFE = ('head',)
#: A rise time: from this far along the hips' way from start to end to this far.
RISE = (0.1, 0.9)
#: Dynamic similarity, a quantity's power of L = her leg over the take's: a length goes as L, a
#: time and a speed as its root, a mass and a force as its cube (the take's human weighs her
#: weight over L cubed, a woman of her build), a moment as its fourth power, a power as L^3.5.
SIMILAR = {'length': 1.0, 'time': 0.5, 'speed': 0.5, 'acceleration': 0.0, 'mass': 3.0, 'force': 3.0,
           'moment': 4.0, 'power': 3.5, 'energy': 4.0}
SIDES = ('Left', 'Right')
UP = np.array([0.0, 1.0, 0.0])
FLAT = np.array([1.0, 0.0, 1.0])


def similar(leg, quantity):
    """What a take's `quantity` becomes at her size: (her leg / `leg`) to SIMILAR's power."""
    return (modal.LEG_M / leg) ** SIMILAR[quantity]


def _mean(a, n):
    """`a`'s running mean over n samples along its first axis, the ends held."""
    pad = np.pad(a, [(n // 2, n - 1 - n // 2)] + [(0, 0)] * (a.ndim - 1), mode='edge')
    if a.ndim == 1:
        return np.convolve(pad, np.ones(n) / n, 'valid')
    return np.stack([np.convolve(pad[:, k], np.ones(n) / n, 'valid') for k in range(a.shape[1])], 1)


def angle(u, v):
    """The degrees between `u` and `v`, row by row."""
    cos = (u * v).sum(axis=-1) / np.maximum(np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1), 1e-9)
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def _past_min(x, w):
    """Each sample's least over the `w` samples before it and itself."""
    return np.lib.stride_tricks.sliding_window_view(np.concatenate([np.full(w, x[0]), x]), w + 1).min(axis=1)


def _even(times, joints):
    """(times, joints) on an even grid at the keys' regular spacing: a take keys some curves a
    hair off the frame, and a speed read across that hair is nothing."""
    t = np.asarray(times, float)
    d = np.diff(t)
    step = float(np.median(d[d >= 0.5 * d.max()]))
    grid = t[0] + step * np.arange(int(round((t[-1] - t[0]) / step)) + 1)
    return grid, {k: np.stack([np.interp(grid, t, p[:, a]) for a in range(3)], 1) for k, p in joints.items()}


def measured(times, joints):
    """{measure: value} (MOVES) of a move from its joints' places, with its `leg` m and `seconds`."""
    t, joints = _even(times, joints)
    hips, neck, head = joints['Hips'], joints['Neck'], joints['Head']
    feet = np.stack([joints['LeftFoot'], joints['RightFoot']])
    leg = float(np.mean([np.linalg.norm(joints[s + 'UpLeg'] - joints[s + 'Leg'], axis=1).mean()
                         + np.linalg.norm(joints[s + 'Leg'] - joints[s + 'Foot'], axis=1).mean() for s in SIDES]))
    tick = math.sqrt(leg / G)
    rate = (len(t) - 1) / max(float(t[-1] - t[0]), 1e-9)
    line = joints['LeftUpLeg'] - joints['RightUpLeg']
    left = line[0] * FLAT
    left = left / max(float(np.linalg.norm(left)), 1e-9)
    fwd = np.cross(left, UP)
    low = feet[:, :, 1].min(axis=0)
    floor = float(low.min())
    h, flat = hips[:, 1], hips[:, [0, 2]]
    n = max(1, int(round(tick * rate))) | 1
    v = np.gradient(_mean(flat, n), t, axis=0) if len(t) > 1 else np.zeros_like(flat)
    trunk = neck - hips
    pitch = np.degrees(np.unwrap(np.arctan2(trunk @ fwd, trunk[:, 1])))
    roll = np.degrees(np.arcsin(np.clip(line[:, 1] / np.maximum(np.linalg.norm(line, axis=1), 1e-9), -1.0, 1.0)))
    heading = np.degrees(np.unwrap(np.arctan2(line @ fwd, line @ left)))
    w = max(1, int(round(math.pi * tick * rate)))
    chord = hips[-1] - hips[0]
    way = (hips - hips[0]) @ chord / max(float(chord @ chord), 1e-9)
    knees = [180.0 - angle(joints[s + 'UpLeg'] - joints[s + 'Leg'], joints[s + 'Foot'] - joints[s + 'Leg']) for s in SIDES]
    arms = [angle(joints[s + 'ForeArm'] - joints[s + 'Arm'], hips - neck) for s in SIDES]
    return {
        'lasted': float(t[np.argmax(way >= RISE[1])] - t[np.argmax(way >= RISE[0])]) / tick,
        'drop': float(h[0] - h.min()) / leg,
        'rise': float(h.max() - h[0]) / leg,
        'ended': float(h[-1] - h[0]) / leg,
        'travel': float((flat[-1] - flat[0]) @ fwd[[0, 2]]) / leg,
        'speed': float(np.hypot(v[:, 0], v[:, 1]).max()) / math.sqrt(G * leg),
        'flight': float((low - _past_min(low, w)).max()) / leg,
        'lifted': float(max((feet[k, :, 1] - _past_min(feet[k, :, 1], w)).max() for k in (0, 1))) / leg,
        'forward': float(max(pitch.max(), 0.0)),
        'back': float(max(-pitch.min(), 0.0)),
        'pitched': float(pitch.max() - pitch.min()),
        'upright': float(angle(trunk[-1:], UP[None, :])[0]),
        'rolled': float(np.abs(roll).max()),
        'turned': float(heading[-1] - heading[0]),
        'head': float(head[:, 1].min() - floor) / leg,
        'knee': float(max(k.max() for k in knees)),
        'hips back': -float((hips[-1] - feet[:, -1].mean(axis=0)) @ fwd) / leg,
        'arms': float(max(a.max() - a.min() for a in arms)),
        'leg': leg, 'seconds': float(t[-1] - t[0]),
    }


def moved(rows, start, stop):
    """Her measures over her look's rows between `start` and `stop` s, SAFE's over all of them
    from `start`; {} with fewer than two."""
    from tools.sim import fbx
    after = [r for r in rows if start <= float(r['t'])]
    window = [r for r in after if float(r['t']) <= stop]
    if len(window) < 2:
        return {}
    out, whole = measured(*fbx.joints(window)), measured(*fbx.joints(after))
    return dict(out, **{name: whole[name] for name in SAFE})


def apart(hers, ref, signature):
    """[(measure, hers, the take's, within)] of the signature's measures and SAFE's."""
    out = []
    for name, unit, side in MOVES:
        if name not in signature + SAFE:
            continue
        a, b = hers.get(name, math.nan), ref[name]
        within = (a >= b - WITHIN[unit]) if side > 0 else (abs(a - b) <= WITHIN[unit])
        out.append((name, a, b, bool(within)))
    return out


def met(hers, ref, signature):
    """Whether every measure of the signature and of SAFE is within WITHIN of the take's."""
    return bool(hers) and all(within for _n, _a, _b, within in apart(hers, ref, signature))


def far(hers, ref, signature):
    """How far off the take a run is: each measure of the signature and of SAFE beyond WITHIN,
    in widths of it, summed; one not measured a width."""
    out = 0.0
    for name, unit, side in MOVES:
        if name not in signature + SAFE:
            continue
        a, b = hers.get(name, math.nan), ref[name]
        gap = (max(0.0, b - a) if side > 0 else abs(a - b)) / WITHIN[unit]
        out += 1.0 if a != a else max(0.0, gap - 1.0)
    return out


def reference():
    """{take: its measures} as kept (REFERENCE); {} before the takes were measured."""
    if not os.path.exists(REFERENCE):
        return {}
    with open(REFERENCE, encoding='utf-8') as f:
        return json.load(f)


def measure_takes(takes):
    """{take: its measures} of the takes read from TAKES: `takes` {key: 'folder/Name'}."""
    from tools.sim import fbx
    out = {}
    for key, take in takes.items():
        times, joints = fbx.take(os.path.join(TAKES, take + '.fbx'))
        out[key] = dict(take=take, **{k: round(float(v), 4) for k, v in measured(times, joints).items()})
    return out


def evaluate(names=None, wobble=0.0):
    """{stunt: its run (`scenarios.ran`'s line) with `far`}, every stunt on the relay: the one
    table to iterate on while a parameter of hers or of the loop is tuned."""
    from tools.dev import focus
    from tools.sim import scenarios
    names = list(names or scenarios.STUNTS)
    ref = reference()
    jobs = [focus.Job(n, [sys.executable, '-X', 'utf8', scenarios.__file__, '--one', n, '--wobble', '%g' % wobble],
                      timeout=900.0) for n in names]
    got = {}
    for job, text, _code, _seconds in focus.relay(jobs):
        last = next((ln for ln in reversed((text or '').splitlines()) if ln.startswith('{')), None)
        r = json.loads(last) if last else {'name': job.name, 'error': (text or '').strip()[-300:]}
        r['far'] = math.nan if 'error' in r else far(r.get('moved', {}), ref[job.name], scenarios.STUNTS[job.name][4])
        got[job.name] = r
    return got


def total(got):
    """(met, of, far summed) of an evaluation's runs."""
    return (sum(bool(r.get('met')) for r in got.values()), len(got),
            float(sum(r['far'] for r in got.values() if r['far'] == r['far'])))


def line(name, got, ref, signature):
    """One scenario's line: how far, met, stood or refused, each signature measure take -> hers,
    * out."""
    if 'refused' in got:
        return '%-24s %5.1f refused %r: no text' % (name, got['far'], got['refused'])
    pairs = ' '.join('%s %.2f>%.2f%s' % (n, b, a, '' if ok else '*') for n, a, b, ok in apart(got.get('moved', {}), ref, signature))
    fell = 'stood' if got['stood'] else 'fell' + ('' if got['head_hit'] is None else ' %.1f m/s' % got['head_hit'])
    return '%-24s %5.1f %-4s %-14s %s' % (name, got['far'], 'met' if got['met'] else '-', fell, pairs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('names', nargs='*', help='stunts (all where none)')
    parser.add_argument('--reference', action='store_true', help='the takes measured anew into tests/takes/stunts.json')
    parser.add_argument('--wobble', type=float, default=0.0, help='each laid turned this rad')
    parser.add_argument('--save', help='the runs written here as JSON')
    parser.add_argument('--against', help="a saved run's JSON: what moved")
    args = parser.parse_args(argv)
    from tools.sim import scenarios
    if args.reference:
        got = measure_takes({k: row[0] for k, row in scenarios.STUNTS.items()})
        with open(REFERENCE, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=1, sort_keys=True)
            f.write('\n')
        for key, m in got.items():
            print('%-26s %5.1f s  leg %.2f (%.0f kg)  ' % (key, m['seconds'], m['leg'], figure.MASS_KG / similar(m['leg'], 'mass'))
                  + ' '.join('%s %.2f' % (n, m[n]) for n in scenarios.STUNTS[key][4] + SAFE))
        return 0
    ref = reference()
    got = evaluate(args.names, args.wobble)
    for n, r in got.items():
        print('%-24s ERROR %s' % (n, r['error'][-150:]) if 'error' in r else line(n, r, ref[n], scenarios.STUNTS[n][4]))
    print('%d of %d met, %d refused, %.1f widths off' % (*total(got)[:2], sum('refused' in r for r in got.values()), total(got)[2]))
    if args.save:
        with open(args.save, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=0, sort_keys=True)
    if args.against:
        with open(args.against, encoding='utf-8') as f:
            was = json.load(f)
        print('was %d of %d met, %.1f widths off' % total(was))
        moved_ = ['%s %+.1f' % (n, got[n]['far'] - was[n]['far']) for n in got
                  if n in was and abs(got[n]['far'] - was[n]['far']) > 1e-9]
        print('moved: ' + (', '.join(moved_) or 'nothing'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
