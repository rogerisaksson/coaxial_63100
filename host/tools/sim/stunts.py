#!/usr/bin/env python3
"""The takes as her scenarios: a move's measures from joints' places, a take's and hers alike.

    python tools/sim/stunts.py                      # every take's scenario on the relay, a table
    python tools/sim/stunts.py stand_to_sit backflip
    python tools/sim/stunts.py --save build/stunts.json [--against build/was.json]
    python tools/sim/stunts.py --reference          # the takes measured anew into tests/takes/
    measures = stunts.measured(times, joints)       # {name: value} of a take's (`fbx.take`) or her rows'
    got = stunts.evaluate(); stunts.total(got)      # every stunt run, how far off its take, the one number
The takes lie in stunts/<folder>/ beside the repo (dance, falling, flips, jumping, kneel, lying,
run, sitting, slipping, stairs, standing), read once: a take's seconds, its leg and MOVES'
measures into tests/takes/stunts.json, the reference each scenario in `scenarios.STUNTS` is
judged by - the scene placed, her told as a player would or handed the take's own rows
(`machine.takes`), her rows measured here over the take's seconds from the telling, and each
measure of the scenario's signature within WITHIN of the take's: a sole's length, her list, her
arm's cone, one natural time; whatever she does, her head goes no lower than the human's (SAFE)
- and its modes at its frames into `machine/takes.npz`, read as `modal.reading` reads hers.

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
from machine import figure, modal, stride, takes
from tools import REPO

G = 9.81
#: The takes beside the repo; their measures kept here, their modes in `takes.FILE`.
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
#: The modes a take's rows say (`takes.STREAMED`), her posture's. The rest stay her balance's -
#: where her soles go and how they rock, her pelvis's list and turn over them, her arms' swing:
#: the human's feet widened and shifted through a crouch collapsed her (head 0.02 legs), his
#: pelvis's 20 deg list or his heels' rise in it felled her (head 0.04, 0.24), his arms' swing
#: too (0.05, hands' jerk 29 000 m/s^3); with these three alone the crouch, the sit and the
#: kneel are the takes' (far 0.0, 0.9, 0.0).
SIDES = ('Left', 'Right')
UP = np.array([0.0, 1.0, 0.0])
FLAT = np.array([1.0, 0.0, 1.0])
#: Each of her segments' bone, its share of her weight (`figure.SEGMENTS`, de Leva's for a
#: woman) and where its centre lies: this far along the bone to the next joint's, or at the
#: joint itself with none below it.
CHAIN = {'torso': 'neck', 'neck': 'head', 'upper_arm': 'forearm', 'forearm': 'hand', 'hand': 'fingers',
         'thigh': 'shank', 'shank': 'foot', 'foot': 'toes'}


def _centres():
    from tools.sim import fbx
    by = {seg[0]: seg for seg in figure.SEGMENTS}
    out = []
    for name, _above, _joints, _offset, _z, share, com, _size in figure.SEGMENTS:
        side, _bar, rest = name.partition('_')
        below = CHAIN.get(rest if side in ('left', 'right') else name)
        child = by.get((side + '_' + below) if below and side in ('left', 'right') else below) if below else None
        along = np.array(child[3], float) if child else None
        f = float(np.array(com) @ along / (along @ along)) if along is not None else 0.0
        out.append((fbx.bone(name), fbx.bone(child[0]) if child else None, share, f))
    return tuple(out)


CENTRES = _centres()


def _com(joints):
    """A skeleton's centre of mass, (n, 3), by her segments' shares laid on its bones (CENTRES);
    a bone the take lacks weighs nothing."""
    total, com = 0.0, 0.0
    for bone, child, share, f in CENTRES:
        if bone not in joints:
            continue
        at = joints[bone]
        if child in joints:
            at = at + f * (joints[child] - at)
        total, com = total + share, com + share * at
    return com / total


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


def _leg(joints):
    """A skeleton's leg, m: hip to knee to ankle, both sides meaned."""
    return float(np.mean([np.linalg.norm(joints[s + 'UpLeg'] - joints[s + 'Leg'], axis=1).mean()
                          + np.linalg.norm(joints[s + 'Leg'] - joints[s + 'Foot'], axis=1).mean() for s in SIDES]))


def _reach(joints):
    """(the soles (2, n, 3): the feet's points on the floor under the ankles, the floor the lowest
    an ankle gets; the legs (n,): a hip to its ankle, m, both sides meaned - the knees'
    shortening, the ankle rigid, a heel's rise the feet's rock)."""
    ankle = min(float(joints[s + 'Foot'][:, 1].min()) for s in SIDES)
    feet = np.stack([joints['LeftFoot'], joints['RightFoot']])
    return feet - UP * ankle, np.linalg.norm(np.stack([joints['LeftUpLeg'], joints['RightUpLeg']]) - feet, axis=2).mean(axis=0)


def _stood(hips):
    """The frame of a take's standing end: whichever has the hips higher - a sit begins standing,
    a rise ends so; a lie's are seated and lying, so the legs' zero is the straight leg, not an end's."""
    return int(np.argmax([hips[0, 1], hips[-1, 1]])) * (len(hips) - 1)


def _frame(joints):
    """(left, ahead) of a skeleton on the floor, its hips' line at the first sample."""
    left = (joints['LeftUpLeg'][0] - joints['RightUpLeg'][0]) * FLAT
    left = left / max(float(np.linalg.norm(left)), 1e-9)
    return left, np.cross(left, UP)


def measured(times, joints):
    """{measure: value} (MOVES) of a move from its joints' places, with its `leg` m and `seconds`."""
    t, joints = _even(times, joints)
    hips, neck, head = joints['Hips'], joints['Neck'], joints['Head']
    feet = np.stack([joints['LeftFoot'], joints['RightFoot']])
    leg = _leg(joints)
    tick = math.sqrt(leg / G)
    rate = (len(t) - 1) / max(float(t[-1] - t[0]), 1e-9)
    left, fwd = _frame(joints)
    line = joints['LeftUpLeg'] - joints['RightUpLeg']
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


def modes(times, joints, fr):
    """(times s at her size, (n, N) her modes) of a take, as `modal.reading` reads hers (her
    frame `fr`, `modal.frame`): the legs shorter than standing, the centre of mass over the
    soles' middle to her left and along, the soles apart along, up and across, the trunk's pitch, the
    hips' line's list and turn, the arms' swing and the feet's rock off standing; lengths at her
    size, times by similarity. The legs' zero is the skeleton's straight leg, the arms' and
    the feet's the take's standing end."""
    t, j = _even(times, joints)
    hips, neck = j['Hips'], j['Neck']
    leg = _leg(j)
    scale = modal.LEG_M / leg
    left, along = _frame(j)
    soles, legs = _reach(j)
    stood = np.zeros(len(t), bool)
    stood[_stood(hips)] = True
    place = soles - _com(j)
    mid = 0.5 * (place[0] + place[1])
    trunk = neck - hips
    pitch = np.arctan2(trunk @ along, trunk @ UP)
    line = j['LeftUpLeg'] - j['RightUpLeg']
    arm = [j[s + 'ForeArm'] - j[s + 'Arm'] for s in SIDES]
    swing = np.stack([np.arctan2(u @ along, -(u @ UP)) for u in arm])
    foot = [j[s + 'ToeBase'] - j[s + 'Foot'] for s in SIDES]
    tilt = np.stack([np.arcsin(np.clip(f @ UP / np.maximum(np.linalg.norm(f, axis=1), 1e-9), -1.0, 1.0)) for f in foot])
    out = np.zeros((len(t), modal.N))
    out[:, modal.AXIAL] = (leg - legs) * scale / (modal.AXIAL_M * fr[modal.HIPS_BY])
    out[:, modal.LATERAL] = -(mid @ left) * scale / fr[modal.W0]
    out[:, modal.SAGITTAL] = -(mid @ along) * scale / modal.HALF_M
    out[:, modal.DRIVE] = 0.5 * ((place[1] - place[0]) @ along) * scale / stride.STRIDE_M
    out[:, modal.LIFT] = ((place[0] - place[1]) @ UP) * scale / modal.CLEAR_M
    out[:, modal.WIDTH] = 0.5 * ((place[0] - place[1]) @ left) * scale / fr[modal.W0] - 1.0
    # the trunk's pitch she takes is the human's own, off her stand's: his rig stands with its
    # neck 3-6 deg behind its hips, and zeroed there she leant 5 deg past him and fell
    out[:, modal.PITCH] = (pitch - fr[modal.PITCH0][1]) / modal.PITCH_RAD
    out[:, modal.LIST] = np.arctan2(line @ UP, line @ left) / modal.LIST_RAD
    out[:, modal.TURN] = np.arctan2(line @ along, line @ left) / modal.TURN_RAD
    out[:, modal.ARMS] = 0.5 * (modal.LIMBS @ (swing - np.median(swing[:, stood], axis=1)[:, None])) / modal.ARM_RAD
    out[:, modal.ROCK] = 0.5 * (modal.LIMBS @ (tilt - np.median(tilt[:, stood], axis=1)[:, None])) / modal.ROCK_RAD
    return ((t - t[0]) * similar(leg, 'time')).astype(np.float32), out.astype(np.float32)


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


def told(command):
    """A command as her worker hears it: a take's rows (`{'take': key}`, `takes.wire`) or a gait
    take's pattern (`{'pattern': key}`, `takes.pattern`) as the loop's entries. Measured, not kept
    (stunts.md): the take's legs scaled to the seat's height; the seat told as her support before
    it bore her."""
    take, pattern = command.get('take'), command.get('pattern')
    rows = {'entries': [(take, takes.wire(take)[0], (take,))]} if take else command
    return {'entries': [(pattern, takes.pattern(pattern), (pattern,))]} if pattern else rows


def reference():
    """{take: its measures} as kept (REFERENCE); {} before the takes were measured."""
    if not os.path.exists(REFERENCE):
        return {}
    with open(REFERENCE, encoding='utf-8') as f:
        return json.load(f)


def measure_takes(takes, fr):
    """({take: its measures}, {take_t, take_z: its modes at her size}) of the takes read from
    TAKES: `takes` {key: 'folder/Name'}, `fr` her frame."""
    from tools.sim import fbx
    out, kept = {}, {}
    for key, take in takes.items():
        times, joints = fbx.take(os.path.join(TAKES, take + '.fbx'))
        out[key] = dict(take=take, **{k: round(float(v), 4) for k, v in measured(times, joints).items()})
        kept[key + '_t'], kept[key + '_z'] = modes(times, joints, fr)
    return out, kept


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
    """One scenario's line: how far, met, stood or refused, her hands' jerk, each signature
    measure take -> hers, * out."""
    if 'refused' in got:
        return '%-24s %5.1f refused %r: no text' % (name, got['far'], got['refused'])
    pairs = ' '.join('%s %.2f>%.2f%s' % (n, b, a, '' if ok else '*') for n, a, b, ok in apart(got.get('moved', {}), ref, signature))
    fell = 'stood' if got['stood'] else 'fell' + ('' if got['head_hit'] is None else ' %.1f m/s' % got['head_hit'])
    hands = max(got['jerk']['left_hand'], got['jerk']['right_hand'])
    return '%-24s %5.1f %-4s %-14s hands %6.0f  %s' % (name, got['far'], 'met' if got['met'] else '-', fell, hands, pairs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('names', nargs='*', help='stunts (all where none)')
    parser.add_argument('--reference', action='store_true', help='the takes measured anew into tests/takes/')
    parser.add_argument('--wobble', type=float, default=0.0, help='each laid turned this rad')
    parser.add_argument('--save', help='the runs written here as JSON')
    parser.add_argument('--against', help="a saved run's JSON: what moved")
    args = parser.parse_args(argv)
    from tools.sim import scenarios
    if args.reference:
        from machine import worker
        got, kept = measure_takes({k: row[0] for k, row in scenarios.STUNTS.items()}, worker.session()['g']['fr'])
        with open(REFERENCE, 'w', encoding='utf-8') as f:
            json.dump(got, f, indent=1, sort_keys=True)
            f.write('\n')
        np.savez_compressed(takes.FILE, **kept)
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
