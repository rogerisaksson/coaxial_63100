#!/usr/bin/env python3
"""A take identified as an ortho-DSP modifier against a reference walk, for `machine.texts.dsp`.

Its clock locked to its strikes, its coronal mid and side, its pelvis's sidechain, its trunk's
filter bank, emitted as a (defmodifier ..) that `machine.texts.dsp` reads and voices on her.

    python tools/sim/identify.py TAKE.fbx [..] [--against REF.fbx] [--out DIR]
    soa = identify.soa(times, joints)            # flat signals: mm in her path's frame, rad, at her size
    got = identify.identified(soa)               # {'clock', 'coronal', 'sidechain', 'axial', 'path'}
    text = identify.emitted(name, got, ref)      # (defmodifier NAME [gait] ..), `ref`'s values the dry's

numpy alone (the host carries no scipy): a peak's prominence and spacing as scipy's find_peaks
reads them, Welch's averaged periodogram, the release's time constant by a log-spaced search with
its gain in closed form. A stride's events are Zeni's (`tools.sim.normal`): a foot lands with
its ankle furthest ahead of her hips and lifts with its toes furthest behind them - an ankle
joint's lowest is mid-stance, not a strike. Lengths at her size (her leg over the take's), times
by its square root (`stunts.similar`). What is hers to play - tempo, track, roll, sidechain,
postures, arms, stride, clearance, height - is the suite's nominal moved by the take's change off
the reference walk's (`--against`, REFERENCE her walk as approved): her walk against itself emits
the nominal, which plays her walk to the last slot. What `machine.texts.dsp` refuses or names voiceless
- duty, swing gate, the legs' phase, heading and turning, transients - is the take's own, as the
spec's: a duty under 0.5 is a flight wherever her walk's lies. Its way is the nearest of her four
(`se2.GAIT_BOXES`): 0.08 m/s aside of a walk measured 4.8 deg off its facing strafed her.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap
from machine.law import modal
from tools.sim import mocap, normal, stunts
from tools.sim.fbx import take

#: Her walk as approved, the dry every take is told against.
REFERENCE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                         'tests', 'takes', 'walk.fbx')
#: The suite's nominal (lisp/humanoid/suite/00_core_manifold.lisp) the emitted values move from.
NOMINAL = {'duty': 0.62, 'lumbar': -0.14, 'thorax': 0.04, 'atlas': 0.0, 'cutoff': 8.0, 'lag': 0.0,
           'gain-db': -3.0, 'track': 103.0, 'foot-w': 96.5}
#: The spec's numbers: a sidechain's window after a strike, s; its attack's floor, ms (a duck
#: faster unloads her stance under 0.35 g); the release's search, ms; the jerk past which a
#: strike is a hard clip, mm/s^3; Welch's segment; the locomotion band, Hz; the stance and
#: swing windows of a stride's phase, its left strike at 0.
WINDOW_S, ATTACK_MS, RELEASE_MS, CLIP_JERK = 0.45, 105.0, (40.0, 600.0), 2.5e6
NPERSEG, BAND_HZ = 256, (0.5, 12.0)
#: A stride's least and most seconds (`normal.measured`'s lags).
STRIDE_S = (0.5, 2.5)
STANCE_L, STANCE_R, SWING_L = (0.05, 0.25), (0.55, 0.75), (0.65, 0.85)
UP = np.array([0.0, 1.0, 0.0])


def peaks(x, distance, prominence):
    """The samples `x` peaks at - each at least `prominence` over the higher of the lowest points
    between it and a higher peak either side, `distance` samples from a higher one kept -, sorted."""
    x = np.asarray(x, float)
    top = [i for i in range(1, len(x) - 1) if x[i] > x[i - 1] and x[i] >= x[i + 1]]
    prom = {}
    for i in top:
        left = next((k for k in range(i - 1, -1, -1) if x[k] > x[i]), -1)
        right = next((k for k in range(i + 1, len(x)) if x[k] > x[i]), len(x))
        prom[i] = x[i] - max(x[left + 1:i + 1].min(), x[i:right].min())
    kept = []
    for i in sorted((i for i in top if prom[i] >= prominence), key=lambda i: -x[i]):
        if all(abs(i - k) >= distance for k in kept):
            kept.append(i)
    return np.array(sorted(kept), int)


def welch(x, rate, n):
    """(Hz, power) of `x`'s Welch estimate: Hann segments of `n`, half overlapping, averaged."""
    x = np.asarray(x, float) - np.mean(x)
    n = min(n, len(x))
    w = np.hanning(n)
    starts = range(0, len(x) - n + 1, max(1, n // 2))
    power = np.mean([np.abs(np.fft.rfft(w * x[s:s + n])) ** 2 for s in starts], axis=0)
    return np.fft.rfftfreq(n, 1.0 / rate), power / (rate * (w * w).sum())


def _pitch(v, ahead):
    """Rad a vector leans ahead of plumb."""
    return np.arctan2((v * ahead).sum(1), v @ UP)


def soa(times, joints):
    """{signal: array} of a take's straight walk (`mocap.walk`), a loop tripled: the heels'
    (ankles') and the toes' places ahead of her hips' middle (Zeni's events), across her path -
    the hips' middle's stride mean - to her left, and up, mm at her size; the sacrum's (the hips'
    middle's) off her path and its height off its mean; the knees', the pelvis's
    roll, the lumbar and thoracic pitch, the trunk's yaw, the left arm's pitch, the head's on
    the trunk and its rate's share, rad; 'rate' Hz, 'time' her second over the take's, its travel
    at her size, m."""
    t, j = mocap.walk(times, joints)
    t = np.asarray(t, float)
    j = {k: np.asarray(v, float) for k, v in j.items()}
    rate = (len(t) - 1) / (t[-1] - t[0])
    leg = float(np.mean([np.linalg.norm(j[s + 'Leg'] - j[s + 'UpLeg'], axis=1)
                         + np.linalg.norm(j[s + 'Foot'] - j[s + 'Leg'], axis=1) for s in normal.SIDES]))
    hips = j['LeftUpLeg'] - j['RightUpLeg']
    mid = 0.5 * (j['LeftUpLeg'] + j['RightUpLeg'])
    flat = hips * [1.0, 0.0, 1.0]
    side = normal._mean(flat / np.linalg.norm(flat, axis=1)[:, None], 2 * max(1, int(0.6 * rate)) + 1)
    side /= np.linalg.norm(side, axis=1)[:, None]
    ahead = np.cross(side, UP)
    if ((j['LeftToeBase'] - j['LeftFoot']) * ahead).sum() < 0.0:
        ahead = -ahead
    size = stunts.similar(leg, 'length')
    scale = 1e3 * size
    floor = min(j['LeftFoot'][:, 1].min(), j['RightFoot'][:, 1].min())
    path = normal._mean(mid, max(1, int(rate)) | 1)
    out = {}
    for name, p in (('calc_l', j['LeftFoot']), ('calc_r', j['RightFoot']), ('toe_l', j['LeftToeBase']),
                    ('toe_r', j['RightToeBase'])):
        out.update({name + '_x': scale * ((p - mid) * ahead).sum(1), name + '_y': scale * ((p - path) * side).sum(1),
                    name + '_z': scale * (p[:, 1] - floor)})
    sway = mid - path
    out.update(sacro_x=scale * (sway * ahead).sum(1), sacro_y=scale * (sway * side).sum(1),
               sacro_z=scale * (mid[:, 1] - mid[:, 1].mean()))
    for s, k in (('Left', 'l'), ('Right', 'r')):
        thigh, shank = j[s + 'Leg'] - j[s + 'UpLeg'], j[s + 'Foot'] - j[s + 'Leg']
        cos = (thigh * shank).sum(1) / (np.linalg.norm(thigh, axis=1) * np.linalg.norm(shank, axis=1))
        out['tibio_%s_flex' % k] = np.arccos(np.clip(cos, -1.0, 1.0))
    shoulders = j['LeftArm'] - j['RightArm']
    thorax = _pitch(j['Neck'] - j['Spine'], ahead)
    head = _pitch(j['Head'] - j['Neck'], ahead)
    out.update(iliac_roll=np.arcsin(np.clip(hips @ UP / np.linalg.norm(hips, axis=1), -1.0, 1.0)),
               lumbar_pitch=_pitch(j['Spine'] - j['Hips'], ahead), thorax_pitch=thorax,
               thorax_yaw=np.arctan2((shoulders * ahead).sum(1), (shoulders * side).sum(1)),
               gleno_l_pitch=_pitch(j['LeftArm'] - j['LeftForeArm'], ahead), atlas_pitch=head - thorax)
    spin = [np.abs(np.gradient(a, t)) for a in (head, thorax)]
    out['cranio_vel_norm'] = spin[0] / np.maximum(spin[0] + spin[1], 1e-9)
    travel = path[-1] - path[0]
    going = np.gradient(path, t, axis=0)
    heading = np.unwrap(np.arctan2(going[:, 0], going[:, 2]))
    facing = np.arctan2(ahead[:, 0], ahead[:, 2])
    off = np.angle(np.mean(np.exp(1j * (heading - facing))))
    out.update(t=t, travel_ahead=size * float(travel @ ahead.mean(0)), travel_side=size * float(travel @ side.mean(0)),
               heading=heading, off=float(off), height=scale * float(mid[:, 1].mean() - floor))
    looped = t[-1] - t[0] <= normal.LOOP_S and all(
        np.linalg.norm((p[0] - mid[0]) - (p[-1] - mid[-1])) < normal.LOOP_LEG * leg for p in j.values())
    if looped:
        out.update({k: np.concatenate([v[:-1]] * 3) for k, v in out.items() if isinstance(v, np.ndarray) and k != 't'})
        out['t'] = t[0] + np.arange(3 * (len(t) - 1)) / rate
    return dict(out, rate=rate, leg=leg, time=stunts.similar(leg, 'time'), seconds=float(t[-1] - t[0]))


def _clock(soa_):
    """Its strikes and lifts (Zeni), its steps a minute at her size, each foot's stance's share of
    its stride - its period the left ankle's strongest line in STRIDE_S: the autocorrelation's
    highest lag read 22-53 steps a minute off walks' 2 to 4 strides -, the right strike's place in
    the left's stride less a half, its legs' phase apart
    and whether it is off pi - past twice its spread over the strides and a frame -, its strides'
    jitter, ms, the strikes' saturation, a phase per sample; a stride's length, its feet apart
    at both strikes (on a treadmill too), and its vault, the sacrum's rise from a strike till the
    hips pass over that foot, mm: a walk's pendulum rises, a run's spring sinks (`normal`)."""
    rate = soa_['rate']
    x = soa_['calc_l_x'] - soa_['calc_l_x'].mean()
    f = np.fft.rfftfreq(16 * len(x), 1.0 / rate)
    line = np.abs(np.fft.rfft(x * np.hanning(len(x)), 16 * len(x))) * (f >= 1.0 / STRIDE_S[1]) * (f <= 1.0 / STRIDE_S[0])
    period = rate / float(f[int(np.argmax(line))])
    spread = lambda v: np.percentile(v, 95) - np.percentile(v, 5)
    lands = {s: peaks(soa_['calc_%s_x' % s], 0.5 * period, 0.25 * spread(soa_['calc_%s_x' % s])) for s in 'lr'}
    lifts = {s: peaks(-soa_['toe_%s_x' % s], 0.5 * period, 0.25 * spread(soa_['toe_%s_x' % s])) for s in 'lr'}
    if len(lands['l']) < 2 or not len(lands['r']):
        raise ValueError('no stride in the take: %d left and %d right strikes' % (len(lands['l']), len(lands['r'])))
    strides = np.diff(lands['l']) / rate * soa_['time']
    duty, gate, steps, vault = [], [], [], []
    for s, o in (('l', 'r'), ('r', 'l')):
        for a, b in zip(lands[s], lands[s][1:]):
            lift = lifts[s][(lifts[s] > a) & (lifts[s] < b)]
            duty += [(lift[0] - a) / (b - a)] if len(lift) else []
            other = lands[o][(lands[o] > a) & (lands[o] < b)]
            gate += [(other[0] - a) / (b - a) - 0.5] if len(other) and s == 'l' else []
            steps += [soa_['calc_%s_x' % o][other[0]] - soa_['calc_%s_x' % s][other[0]]
                      + soa_['calc_%s_x' % s][b] - soa_['calc_%s_x' % o][b]] if len(other) else []
            over = np.flatnonzero(soa_['calc_%s_x' % s][a:b] <= 0.0)
            vault += [soa_['sacro_z'][a + over[0]] - soa_['sacro_z'][a]] if len(over) else []
    phase = np.zeros(len(x))
    for a, b in zip(lands['l'], lands['l'][1:]):
        phase[a:b] = np.arange(b - a) / (b - a)
    jerk = np.abs(np.gradient(np.gradient(np.gradient(soa_['calc_l_z'], 1.0 / rate), 1.0 / rate), 1.0 / rate))
    off = abs(float(np.median(gate))) > 2.0 * float(np.std(gate)) + 1.0 / float(np.median(np.diff(lands['l'])))
    return {'bpm': 120.0 / float(np.median(strides)), 'duty': float(np.median(duty)), 'gate': float(np.median(gate)),
            'stride-mm': float(np.median(steps)), 'vault-mm': float(np.median(vault)),
            'phase-apart': math.tau * (0.5 + float(np.median(gate))), 'phase-off': off,
            'jitter-ms': 1e3 * float(np.std(strides)),
            'saturation': ':hard-clip' if np.percentile(jerk[lands['l']], 90) > CLIP_JERK else ':tanh-viscoelastic',
            'lands': lands, 'lifts': lifts, 'phase': phase, 'stride-s': float(np.median(strides))}


def _coronal(soa_, phase):
    """The track between the stances, the left swing's bow past half of it, mm; the pelvis's roll's
    amplitude as a sine's, rad."""
    on = lambda w: (phase > w[0]) & (phase < w[1])
    track = abs(float(np.median(soa_['calc_l_y'][on(STANCE_L)]) - np.median(soa_['calc_r_y'][on(STANCE_R)])))
    bow = max(0.0, float(np.percentile(np.abs(soa_['calc_l_y'][on(SWING_L)]), 95)) - 0.5 * track)
    return {'track': track, 'bow': bow, 'lfo': math.sqrt(2.0) * float(np.std(soa_['iliac_roll']))}


def _sidechain(soa_, lands):
    """The sacrum's dip after a strike, both feet's averaged over WINDOW_S: its depth, mm, its
    attack to the nadir and its release's time constant, ms."""
    rate, z = soa_['rate'], soa_['sacro_z']
    n = int(WINDOW_S * rate / soa_['time'])
    dips = [z[s:s + n] - z[s] for s in np.concatenate(list(lands.values())) if s + n < len(z)]
    if not dips:
        raise ValueError('no strike %.2f s before its end' % WINDOW_S)
    env = np.mean(dips, axis=0)
    nadir = int(np.argmin(env))
    ms = 1e3 * np.arange(len(env) - nadir) / rate * soa_['time']
    rise = env[nadir:] - env[nadir]
    fits = []
    for tau in np.geomspace(*RELEASE_MS, 64):
        shape = 1.0 - np.exp(-ms / tau)
        gain = float(rise @ shape / max(shape @ shape, 1e-12))
        fits.append((float(((rise - gain * shape) ** 2).sum()), tau))
    return {'depth': float(env[nadir]), 'attack': 1e3 * nadir / rate * soa_['time'], 'release': min(fits)[1]}


def _axial(soa_, stride_s):
    """The lumbar, thoracic and atlas pitch's means, rad; the trunk's corner over the pelvis's roll
    where its yaw falls 3 dB under the band's first, Hz at her size; the left arm's lag on the
    pelvis's roll at the stride's rate, rad, and its swing's amplitude; the atlas's ceiling; the
    head's share of the head's and trunk's turning."""
    rate = soa_['rate']
    f, roll = welch(soa_['iliac_roll'], rate, NPERSEG)
    _f, yaw = welch(soa_['thorax_yaw'], rate, NPERSEG)
    db = 10.0 * np.log10((yaw + 1e-12) / (roll + 1e-12))
    band = (f >= BAND_HZ[0]) & (f <= BAND_HZ[1])
    under = np.flatnonzero(db[band] < db[band][0] - 3.0) if band.any() else []
    arm, pelvis = soa_['gleno_l_pitch'] - soa_['gleno_l_pitch'].mean(), soa_['iliac_roll'] - soa_['iliac_roll'].mean()
    lag = int(np.argmax(np.correlate(arm, pelvis, 'full'))) - (len(arm) - 1)
    atlas = soa_['atlas_pitch']
    return {'lumbar': float(np.mean(soa_['lumbar_pitch'])), 'thorax': float(np.mean(soa_['thorax_pitch'])),
            'cutoff': float(f[band][under[0]]) / soa_['time'] if len(under) else 4.5,
            'lag': (lag / rate * soa_['time'] * math.tau / stride_s) % math.pi, 'arm': math.sqrt(2.0) * float(np.std(arm)),
            'atlas': float(np.mean(atlas)), 'ceiling': float(np.percentile(np.abs(atlas - atlas.mean()), 98)),
            'cranio': float(np.clip(np.mean(soa_['cranio_vel_norm']), 0.0, 1.0))}


def _path(soa_, clock):
    """Its speed by its strides, m/s, a stride, mm, its way off its facing and its turning - its
    path's heading's slope - where it travels, rad and rad/s, the sacrum's height, mm, the swing's
    clearance - the heel's highest over its strike's height -, mm, all at her size."""
    seconds = soa_['seconds'] * soa_['time']
    lifts = [soa_['calc_l_z'][a:b].max() - soa_['calc_l_z'][a] for a, b in zip(clock['lands']['l'], clock['lands']['l'][1:])]
    travels = math.hypot(soa_['travel_ahead'], soa_['travel_side']) > mocap.IN_PLACE_M
    return {'speed': 1e-3 * clock['stride-mm'] / clock['stride-s'], 'stride': clock['stride-mm'],
            'way': soa_['off'] if travels else 0.0,
            'turning': float(np.polyfit(soa_['t'] * soa_['time'], soa_['heading'], 1)[0]) if travels else 0.0,
            'height': soa_['height'],
            'clearance': float(np.median(lifts)) if lifts else math.nan}


def identified(soa_):
    """{'clock', 'coronal', 'sidechain', 'axial', 'path'} of a take's flat signals (`soa`)."""
    clock = _clock(soa_)
    return {'clock': clock, 'coronal': _coronal(soa_, clock['phase']), 'sidechain': _sidechain(soa_, clock['lands']),
            'axial': _axial(soa_, clock['stride-s']), 'path': _path(soa_, clock)}


def emitted(name, got, ref):
    """A take identified (`got`) as a (defmodifier `name` [gait] ..) of the spec's text: each value
    the suite's nominal moved by the take's change off `ref`'s, so `ref` emits the nominal; a
    duck of 0 mm none; a take striding under two soles steps in place, no walk to emit."""
    c, rc = got['clock'], ref['clock']
    if not c['stride-mm'] >= 4e3 * modal.HALF_M:
        raise ValueError('steps in place: a stride of %.0f mm, under two soles %.0f' % (c['stride-mm'], 4e3 * modal.HALF_M))
    co, rco = got['coronal'], ref['coronal']
    sc, rsc = got['sidechain'], ref['sidechain']
    ax, rax = got['axial'], ref['axial']
    pa, rpa = got['path'], ref['path']
    n = NOMINAL
    phase = '\n          :inter-limb-phase-rad (:set %.4f)' % c['phase-apart'] if c['phase-off'] else ''
    depth = round(sc['depth'] - rsc['depth'], 1)
    duck = f"""(route-sidechain
          :key-input :calcaneus
          :target :sacroiliac-z
          :topology :second-order-damped
          :attack-ms (:clamp-min {max(sc['attack'], ATTACK_MS):.1f})
          :release-ms {sc['release']:.1f}
          :release-curve :logarithmic
          :duck-depth-mm {depth:.1f}
          :min-fz-guard :min-normal-fz-ratio)
        """ if depth else ''
    return f""";; {name}: {c['bpm']:.1f} BPM at her size | raw attack {sc['attack']:.0f} ms (clamped >= {ATTACK_MS:.0f})
(defmodifier {name} [gait]
  (let [foot-w      (get-param gait :foot-collision-w-mm)
        target-tr-w (* {n['track'] * co['track'] / rco['track'] / n['foot-w']:.4f} foot-w)
        req-bow-mm  (max {co['bow']:.1f} (+ (- foot-w target-tr-w) 7.5))]
    (-> gait
        (quantize-grid
          :bpm (:scale {c['bpm'] / rc['bpm']:.5f})
          :stance-duty-pwm (:add {c['duty'] - n['duty']:+.3f})
          :swing-gate {c['gate']:.3f}
          :strip-jitter true{phase}
          :direction-angle-rad (:set {0.5 * math.pi * round(pa['way'] / (0.5 * math.pi)):.5f})
          :yaw-rate-radps (:set {pa['turning']:.4f}))
        (shape-transients :target :calcaneus
          :saturation (:set {c['saturation']}))
        (mid-side-rebalance
          :narrow-stereo (:node :tarsal-coronal
                          :track-width-mm target-tr-w
                          :swing-circumduction-mm req-bow-mm
                          :circumduction-window-phase [0.10 -> 0.40]
                          :circumduction-shape :raised-cosine)
          :stereo-mod    (:node :iliac-coronal
                          :wave :sine-fundamental
                          :lfo-amp-rad (:scale {co['lfo'] / rco['lfo']:.4f})
                          :stereo-width 0.60))
        {duck}(apply-filter-bank
          (dc-bias         :target :l4-l5-lumbar    :offset-rad {n['lumbar'] + ax['lumbar'] - rax['lumbar']:.4f} :q-factor 0.50)
          (dc-bias         :target :c7-t12-thoracic :offset-rad {n['thorax'] + ax['thorax'] - rax['thorax']:.4f} :q-factor 0.80
                           :zmp-counter-bias-x-mm {-120.0 * (ax['thorax'] - rax['thorax']):.1f})
          (butterworth-lpf :target :c7-t12-thoracic :cutoff-hz {n['cutoff'] * ax['cutoff'] / rax['cutoff']:.2f} :slope-db 12)
          (phase-delay     :target :glenohumeral    :phase-lag-rad {n['lag'] + ax['lag'] - rax['lag']:.4f} :gain-db {n['gain-db'] + 20.0 * math.log10(ax['arm'] / rax['arm']):.2f})
          (dc-bias         :target :c1-c2-atlas     :offset-rad {n['atlas'] + ax['atlas'] - rax['atlas']:.4f} :ceiling-rad {ax['ceiling']:.4f})
          (hard-mute       :target :craniofacial    :velocity-sensitivity {ax['cranio']:.2f}))
        (modulate-node :tarsal-sagittal
          :stride-length-mm (:scale {pa['stride'] / rpa['stride']:.4f})
          :swing-clearance-z-mm (:scale {pa['clearance'] / rpa['clearance']:.4f}))
        (modulate-node :com-dynamics
          :topology (:set {':inverted-pendulum' if c['vault-mm'] > 0.0 else ':ballistic-slip'})
          :base-height-z-mm (:add {pa['height'] - rpa['height']:.1f})))))
"""


def main(argv=None):
    """Each take's modifier printed, or written to --out as NAME.lisp."""
    p = argparse.ArgumentParser(description='A take identified as an ortho-DSP modifier.')
    p.add_argument('takes', nargs='+')
    p.add_argument('--against', default=REFERENCE)
    p.add_argument('--out')
    args = p.parse_args(argv)
    ref = identified(soa(*take(args.against)))
    for path in args.takes:
        name = 'take-' + ''.join(ch if ch.isalnum() else '-' for ch in os.path.splitext(os.path.basename(path))[0].lower())
        try:
            text = emitted(name, identified(soa(*take(path))), ref)
        except (ValueError, KeyError) as error:
            print(';; %s: not identified, %s' % (name, error))
            continue
        if args.out:
            os.makedirs(args.out, exist_ok=True)
            with open(os.path.join(args.out, name + '.lisp'), 'w', encoding='utf-8') as f:
                f.write(text)
        print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
