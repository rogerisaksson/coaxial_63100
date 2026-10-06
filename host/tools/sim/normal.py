#!/usr/bin/env python3
"""A woman's normal walk: a walk's measures from its joints' places, and how far off it is.

    measures = normal.measured(times, joints)     # a take's (`fbx.take`), her rows' (`fbx.joints`)
    off, out = normal.off(measures)               # 0.0 on BAND, else what is out of it
    normal.said(measures)                         # in words: [('stiff', 2.4), ('wide', 0.4)]
    far, most = normal.apart(measures, other)     # two walks, a law's and a reference's

Places alone, so her rows, a recording, a mocap take and an animation read alike: a knee the
thigh's line to the shank's, an elbow the upper arm's to the forearm's, an arm the upper arm
ahead of plumb, the pelvis's roll and turn its hips' line off level and off her path. A foot
lands with its ankle furthest ahead of her hips and lifts with its toes furthest behind them
(Zeni 2008): over ground, on a treadmill and in place alike. Lengths in legs (hip to knee to
ankle), times in sqrt(leg / g): a walker's size out of them.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # noqa: E402   behind the OpenBLAS cap

G = 9.81

#: A woman's normal walk, slow to brisk: (measure, unit, least, most). Textbook gait - a knee
#: all but straight as it lands, 15-20 deg bent under her weight, 35-45 as its toes leave at
#: 60 % of the stride, 60-70 swinging; the pelvis turning 6-10 deg - and what tells a woman's
#: from a man's: her pelvis rolls more (8-10 deg where 6-7), her cadence is higher on a shorter
#: step, her feet nearer her line. Held against takes' walks through this module, a mocap suit's
#: or a studio's (2026-10-05): three of four a suit's on it, the fourth a foot's width off; a
#: runway's catwalk 1.3 off - its arms 82 deg, its roll 23; her walk as built 2.0 - its knee
#: lands at 30 deg, both feet bear 25 % of a stride a step, its step is short for its time; the
#: one law's 4.0, on stilts (the user): the swinging knee 26 deg, no heel's rise, the pelvis
#: level, her feet 22 cm apart. Zeni's lift reads 3-6 % of a stride late: `stance` to 70.
#: `walk ratio` is a step, legs, times its seconds, sqrt(leg / g): her step for its time,
#: whatever her pace. `vault` is the pelvis over the standing foot above where it was as that
#: foot landed: a walk's rises over its leg, a run's sinks onto it.
BAND = (
    ('pace', 'sqrt(g leg)', 0.15, 0.55),
    ('stride', 'sqrt(leg/g)', 3.3, 5.2),
    ('step', 'leg', 0.45, 0.90),
    ('walk ratio', '', 1.1, 1.8),
    ('stance', '%', 56.0, 70.0),
    ('knee at landing', 'deg', 0.0, 12.0),
    ('knee loaded', 'deg', 8.0, 33.0),
    ('knee straightest', 'deg', 0.0, 15.0),
    ('knee at lift', 'deg', 30.0, 60.0),
    ('knee swinging', 'deg', 52.0, 78.0),
    ('thigh behind at lift', 'deg', -8.0, 14.0),
    ('thigh ahead at landing', 'deg', 15.0, 32.0),
    ('heel at lift', 'deg', 28.0, 62.0),
    ('toes up at landing', 'deg', 5.0, 35.0),
    ('elbow bent', 'deg', 8.0, 42.0),
    ('elbow', 'deg', 5.0, 45.0),
    ('arm', 'deg', 15.0, 65.0),
    ('hand out', 'leg', 0.08, 0.28),
    ('trunk lean', 'deg', -5.0, 9.0),
    ('pelvis roll', 'deg', 5.0, 15.0),
    ('pelvis turn', 'deg', 4.0, 22.0),
    ('pelvis bob', 'leg', 0.02, 0.07),
    ('vault', 'leg', 0.0, 0.07),
    ('hips wag', 'leg', 0.02, 0.13),
    ('feet apart', 'leg', 0.02, 0.22),
)

#: A walk in words (the user, 2026-10-05: what is a stiff walk without a soft one to hold it
#: to): (word, ((measure, side), ..)) - how far those measures are out of their bands on that
#: side, -1 under and 1 over, by the bands' widths; 0 on a woman's walk, said from SAID. Stiff:
#: the leg goes by unbent, its foot lifted flat - stilts. Crouched: never on a straight leg,
#: Groucho's. Tripping: a step short for its time. Flying: the pelvis lowest over the standing
#: foot where a walk's is highest (`vault`) - a run's bounce.
WORDS = (
    ('stiff', (('knee swinging', -1), ('knee at lift', -1), ('heel at lift', -1))),
    ('crouched', (('knee straightest', 1),)),
    ('landing bent', (('knee at landing', 1),)),
    ('still-hipped', (('pelvis roll', -1), ('pelvis turn', -1), ('pelvis bob', -1))),
    ('still-armed', (('arm', -1), ('elbow', -1))),
    ('arms carried', (('elbow bent', 1),)),
    ('tripping', (('walk ratio', -1),)),
    ('striding', (('walk ratio', 1),)),
    ('wide', (('feet apart', 1),)),
    ('on a line', (('feet apart', -1),)),
    ('swaying', (('pelvis roll', 1), ('pelvis turn', 1), ('hips wag', 1))),
    ('swinging', (('arm', 1), ('elbow', 1), ('knee swinging', 1))),
    ('shuffling', (('stance', 1), ('toes up at landing', -1), ('heel at lift', -1))),
    ('flying', (('vault', -1),)),
    ('leaning', (('trunk lean', 1),)),
    ('leaning back', (('trunk lean', -1),)),
    ('slow', (('pace', -1),)),
    ('brisk', (('pace', 1),)),
)
SAID = 0.1

#: A gait by name: (name, the words it is, the words it is not, how far each of its words at
#: the least) - a run first, a walk's words not its own. Stilts and Groucho's are what her
#: walk is not to be (CLAUDE.md), a catwalk and a run what it may be asked. On stilts from
#: stiff 0.5: the one law's first walk 2.38, its second 0.31 - the heel 20 deg up as its toes
#: leave where 28 -, no stilts; Groucho's from crouched 0.4: her standing knee at 24 deg 0.56.
GAITS = (('a run', ('flying',), (), 0.25), ('on stilts', ('stiff',), ('flying',), 0.5),
         ("Groucho's", ('crouched',), ('flying',), 0.4),
         ('a catwalk', ('swaying',), ('wide', 'flying'), 0.3),
         ('a waddle', ('wide', 'swaying'), ('flying',), 0.3),
         ('a shuffle', ('shuffling', 'tripping'), ('flying',), 0.3))

#: A take no longer than LOOP_S whose last pose is its first within LOOP_LEG legs is a loop:
#: three of it, the seam out.
LOOP_S, LOOP_LEG = 4.0, 0.06

SIDES, LIMB = ('Left', 'Right'), ('UpLeg', 'Leg', 'Foot', 'ToeBase', 'Arm', 'ForeArm', 'Hand')


def _mean(a, n):
    """`a`'s running mean over n samples, the ends held."""
    pad = np.pad(a, [(n // 2, n - 1 - n // 2)] + [(0, 0)] * (a.ndim - 1), mode='edge')
    if a.ndim == 1:
        return np.convolve(pad, np.ones(n) / n, 'valid')
    return np.stack([np.convolve(pad[:, k], np.ones(n) / n, 'valid')
                     for k in range(a.shape[1])], 1)


def _angle(a, b):
    """The degrees between the rows of `a` and of `b`."""
    dot = (a * b).sum(1) / np.maximum(1e-12, np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    return np.degrees(np.arccos(np.clip(dot, -1.0, 1.0)))


def _peaks(x, w):
    """The samples `x` is its greatest within `w` either side, the ends left out."""
    return [i for i in range(w, len(x) - w) if x[i] == x[i - w:i + w + 1].max() and x[i] > x[i - 1]]


def signals(times, joints):
    """{signal: a value a sample} of a walk's joints' places, and its rate, leg and whether it
    is a loop (tripled then)."""
    t = np.asarray(times, float)
    rate = (len(t) - 1) / (t[-1] - t[0])
    j = {side + part: np.asarray(joints[side + part], float) for side in SIDES for part in LIMB}
    up = np.array([0.0, 1.0, 0.0])
    hips, mid = j['LeftUpLeg'] - j['RightUpLeg'], (j['LeftUpLeg'] + j['RightUpLeg']) / 2.0
    flat = hips * [1.0, 0.0, 1.0]
    flat /= np.maximum(1e-9, np.linalg.norm(flat, axis=1))[:, None]
    side = _mean(flat, 2 * max(1, int(0.6 * rate)) + 1)      # to her left, her path's
    side /= np.maximum(1e-9, np.linalg.norm(side, axis=1))[:, None]
    ahead = np.cross(side, up)
    if ((j['LeftToeBase'] - j['LeftFoot']) * ahead).sum() < 0.0:
        ahead = -ahead
    leg = float(np.mean([np.linalg.norm(j[s + 'Leg'] - j[s + 'UpLeg'], axis=1)
                         + np.linalg.norm(j[s + 'Foot'] - j[s + 'Leg'], axis=1) for s in SIDES]))
    head = np.asarray(joints['Head'], float) - mid
    sig = {'mid': mid, 'side': side, 'height': mid[:, 1],
           'lean': np.degrees(np.arctan2((head * ahead).sum(1), head[:, 1])),
           'roll': np.degrees(np.arcsin(np.clip(hips[:, 1] / np.linalg.norm(hips, axis=1), -1, 1))),
           'turn': np.degrees(np.arcsin(np.clip((flat * ahead).sum(1), -1.0, 1.0)))}
    for s, sign in zip(SIDES, (1.0, -1.0)):
        hip, knee, ankle, toe = (j[s + part] for part in LIMB[:4])
        shoulder, elbow, wrist = (j[s + part] for part in LIMB[4:])
        foot = toe - ankle
        sig.update({
            'knee' + s: _angle(knee - hip, ankle - knee),
            'elbow' + s: _angle(elbow - shoulder, wrist - elbow),
            'ankle' + s: ((ankle - mid) * ahead).sum(1), 'toe' + s: ((toe - mid) * ahead).sum(1),
            'behind' + s: ((ankle - hip) * ahead).sum(1),
            'thigh' + s: np.degrees(np.arctan2(((knee - hip) * ahead).sum(1), (hip - knee)[:, 1])),
            'arm' + s: np.degrees(np.arctan2(((elbow - shoulder) * ahead).sum(1),
                                             (shoulder - elbow)[:, 1])),
            'out' + s: sign * ((wrist - hip) * side).sum(1),
            'foot' + s: np.degrees(np.arcsin(np.clip(foot[:, 1] / np.linalg.norm(foot, axis=1),
                                                     -1.0, 1.0))),
            'aside' + s: ((ankle - mid) * side).sum(1)})
    first, last = ({k: p[i] - j['LeftUpLeg'][i] for k, p in j.items()} for i in (0, -1))
    looped = t[-1] - t[0] <= LOOP_S and max(
        np.linalg.norm(first[k] - last[k]) for k in first) < LOOP_LEG * leg
    if looped:                                   # three of it, a travelling one's path on
        sig = {k: np.concatenate([v[:-1] + (i * (v[-1] - v[0]) if k == 'mid' else 0.0)
                                  for i in range(3)]) for k, v in sig.items()}
    return sig, rate, leg, looped


def measured(times, joints):
    """{measure: value} of a walk (BAND's names; `leg` m, `strides` counted, `speed` m/s, `loop`):
    each stride's own, both legs', meaned."""
    sig, rate, leg, looped = signals(times, joints)
    d = sig['ankleLeft'] - sig['ankleLeft'].mean()
    lags = range(int(0.5 * rate), min(len(d) - 2, int(2.5 * rate)))
    period = max(lags, key=lambda k: float(d[:-k] @ d[k:]) / (len(d) - k), default=0)
    out = {name: [] for name, *_rest in BAND}
    out.update(speed=[])
    if not period:
        return {'leg': leg, 'strides': 0, 'loop': looped}
    w, soft = max(2, int(0.3 * period)), 2 * int(rate / 40.0) + 1
    lands = {s: _peaks(_mean(sig['ankle' + s], soft), w) for s in SIDES}
    lifts = {s: _peaks(-_mean(sig['toe' + s], soft), w) for s in SIDES}
    path = _mean(sig['mid'], period | 1)
    sway = ((sig['mid'] - path) * sig['side']).sum(1)
    flat = {}
    for s in SIDES:                               # the foot flat: its pitch midway through stance
        mids = [sig['foot' + s][(a + b) // 2] for a in lands[s]
                for b in [next((x for x in lifts[s] if x > a), None)] if b is not None]
        flat[s] = float(np.mean(mids)) if mids else math.nan
    for s, o in (SIDES, SIDES[::-1]):
        for a, b in zip(lands[s], lands[s][1:]):
            lift = next((x for x in lifts[s] if a < x < b), None)
            o_land = next((x for x in lands[o] if a < x < b), None)
            o_lift = max((x for x in lifts[o] if o_land is not None and x < o_land), default=None)
            if lift is None or o_land is None or o_lift is None:
                continue
            # the foot alone under her: a walk's from the other's lift to its landing, a run's
            # from this one's landing to its lift
            o_lift, last = max(a, o_lift), min(lift, o_land)
            if last <= o_lift:
                continue
            knee, alone = sig['knee' + s], slice(o_lift, last + 1)
            over = next((i for i in range(a, lift) if sig['behind' + s][i] <= 0.0), None)
            back = sig['behind' + s][alone] < 0.0
            # a stride: the other foot's step and this one's, landing to landing
            speed = (sig['ankle' + o][o_land] - sig['ankle' + s][o_land] + sig['ankle' + s][b]
                     - sig['ankle' + o][b]) * rate / (b - a)
            for name, value in (
                    ('speed', speed), ('pace', speed / math.sqrt(G * leg)),
                    ('stride', (b - a) / rate * math.sqrt(G / leg)),
                    ('step', (sig['ankle' + s][a] - sig['ankle' + o][a]) / leg),
                    ('walk ratio', (sig['ankle' + s][a] - sig['ankle' + o][a]) / leg
                     * (b - a) / rate / 2.0 * math.sqrt(G / leg)),
                    ('stance', 100.0 * (lift - a) / (b - a)), ('knee at landing', knee[a]),
                    ('knee loaded', knee[a:(o_lift + last) // 2 + 1].max()),
                    ('knee straightest', knee[alone].min()), ('knee at lift', knee[lift]),
                    ('knee swinging', knee[lift:b + 1].max()),
                    ('thigh behind at lift', -sig['thigh' + s][lift]),
                    ('thigh ahead at landing', sig['thigh' + s][a]),
                    ('heel at lift', flat[s] - sig['foot' + s][lift]),
                    ('toes up at landing', sig['foot' + s][a] - flat[s]),
                    ('elbow bent', sig['elbow' + s][a:b].mean()),
                    ('elbow', np.ptp(sig['elbow' + s][a:b])), ('arm', np.ptp(sig['arm' + s][a:b])),
                    ('hand out', sig['out' + s][a:b].mean() / leg),
                    ('trunk lean', sig['lean'][a:b].mean()),
                    ('pelvis roll', np.ptp(sig['roll'][a:b])),
                    ('pelvis turn', np.ptp(sig['turn'][a:b])),
                    ('pelvis bob', np.ptp(sig['height'][a:o_land + 1]) / leg),
                    ('vault', math.nan if over is None else
                     (sig['height'][over] - sig['height'][a]) / leg),
                    ('hips wag', np.ptp(sway[a:b]) / leg),
                    ('feet apart', abs(sig['aside' + s][a] - sig['aside' + o][a]) / leg)):
                out[name].append(float(value))
            out.setdefault('knee behind plumb', []).append(
                float(knee[alone][back].max()) if back.any() else math.nan)
    strides = len(out['stride'])
    found = {name: float(np.nanmean(v)) if v and not np.all(np.isnan(v)) else math.nan
             for name, v in out.items()}
    return dict(found, leg=leg, strides=strides, loop=looped)


def off(measures):
    """(how far off a woman's normal walk, [(measure, value, bound)]): each of BAND's measures
    out of its band, by the band's width; one unmeasured is a width off."""
    far, out = 0.0, []
    for name, _unit, least, most in BAND:
        v = measures.get(name, math.nan)
        if v != v:
            far, out = far + 1.0, out + [(name, v, math.nan)]
        elif not least <= v <= most:
            bound = least if v < least else most
            far, out = far + abs(v - bound) / (most - least), out + [(name, v, bound)]
    return far, out


def said(measures):
    """[(word, how far)] of WORDS a walk is, the furthest first: each word's measures out of
    their bands on its side, by the bands' widths, SAID or more together."""
    bands = {name: (least, most) for name, _unit, least, most in BAND}
    out = []
    for word, sides in WORDS:
        far = 0.0
        for name, side in sides:
            v, (least, most) = measures.get(name, math.nan), bands[name]
            if v == v:
                far += max(0.0, side * (v - (most if side > 0 else least))) / (most - least)
        if far >= SAID:
            out.append((word, far))
    return sorted(out, key=lambda w: -w[1])


def named(measures):
    """The gaits of GAITS a walk is, by name: 'a woman's walk' with no word said."""
    words = dict(said(measures))
    return [name for name, needs, never, least in GAITS
            if all(words.get(w, 0.0) >= least for w in needs)
            and not any(w in words for w in never)] or ([] if words else ["a woman's walk"])


def apart(a, b):
    """(how far two walks are apart, [(widths, measure, a's, b's)] the furthest first): each
    of BAND's measures both have, by the band's width."""
    each = sorted(((abs(a[name] - b[name]) / (most - least), name, a[name], b[name])
                   for name, _unit, least, most in BAND
                   if a.get(name, math.nan) == a.get(name, math.nan)
                   and b.get(name, math.nan) == b.get(name, math.nan)), reverse=True)
    return sum(e[0] for e in each), each


def table(walks):
    """The lines of `walks` [(label, measures)] beside the band: a measure a line, one out of
    its band marked, how far off it each walk is and how far from the first."""
    wide = max(len('%s, %s' % row[:2]) for row in BAND) + 2
    lines = [' ' * wide + '%13s' % 'a woman' + ''.join('%9s' % label[:8] for label, _m in walks)]
    for name, unit, least, most in BAND:
        cells = ''
        for _label, m in walks:
            v = m.get(name, math.nan)
            cells += '%9s' % ('-' if v != v else ('%.3g' % v) + ('' if least <= v <= most else '*'))
        lines.append(('%s, %s' % (name, unit)).rstrip(', ').ljust(wide)
                     + '%13s' % ('%g..%g' % (least, most)) + cells)
    lines.append('off her band'.ljust(wide + 13) + ''.join('%9.2f' % off(m)[0] for _l, m in walks))
    for label, m in walks:
        lines.append('%s: %s%s' % (label, ', '.join('%s %.2f' % w for w in said(m)) or 'no word',
                                   ''.join(' - ' + n for n in named(m))))
    if len(walks) > 1:
        lines.append(('from %s' % walks[0][0]).ljust(wide + 13)
                     + ''.join('%9.2f' % apart(m, walks[0][1])[0] for _l, m in walks))
    return lines
