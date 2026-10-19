"""What a craft's law reads of its frame: its IMU, an RTK receiver on the bus, a star camera.

    eyes = senses.eyes(seed)                 # the instruments, none read yet
    seen = senses.read(eyes, state, dt)      # the frame `dt` s on as they give it
    seen = senses.seen(eyes, state)          # as last read, nothing stepped
    seen = senses.told(eyes, state, dt)      # either, or the truth where `eyes` is None

The IMU is the board's, the receiver a Modbus RTU slave on its 10 Mbit RS485 bus, the camera
the receiver's stand-in where the signal is lost. The attitude is the IMU's fused with the
fixes, off by a slow tilt; its rates and specific force noisy. Place and speed are off by what
those errors integrate to - the frame's own kept: integrated afresh, they parted from MuJoCo's
substeps by 1.6 cm through a knife-in -, each fix pulling them in by a Kalman filter's steady
gains against the error it was taken at: the tilt on a pull leaves them 0.35 m off a m/s^2,
a knife-in's 1-2 cm; with the drift a third state, its estimate chased the pulls 0.045 m/s^2
off and a loss drifted 1 m. Lost, they are dead-reckoned on the star camera's attitude: the
drift is the speed's error at the loss. A gust is felt at once in the specific force; the
instruments set how well the frame holds its place.
"""
import collections
import math
import random

#: The attitude, the IMU's fused with the fixes: tilted ATT_RAD rms over ATT_S, an RTK-aided
#: MEMS INS's roll and pitch (Xsens MTi-680G: 0.2 deg) - on the BNO085's own 1 deg the height
#: ran 1.8 cm high through a knife-in and its tip struck. The gyros' noise, rad/s/sqrt(Hz), the
#: specific force's, m/s^2/sqrt(Hz): Bosch's BMI055 class.
ATT_RAD, ATT_S, GYRO, ACC = math.radians(0.2), 10.0, math.radians(0.014), 150e-6 * 9.81
#: The RTK receiver's fix: a second, s behind - its poll on the bus 0.1 ms of it -, its
#: place's noise, m rms, its speed's, m/s.
GNSS = (20.0, 0.06, 0.02, 0.03)
#: The star camera's attitude: a second, s behind, rad rms.
STAR = (5.0, 0.1, math.radians(20.0 / 3600.0))
#: The signal lost about every LOSE_S, for LOST_S.
LOSE_S, LOST_S = 90.0, (8.0, 15.0)
#: The seconds of the estimate kept to meet a fix taken behind it.
KEEP_S = 0.5
GRAVITY = 9.81


def _gains(rate, pos_m, vel_m, acc):
    """The steady gains of a Kalman filter on place and speed fixed `rate` a second with `pos_m`
    and `vel_m` of noise, its speed off by `acc` m/s^2 held a fix: 2x2, (place, speed) of the
    misses (place, speed)."""
    t, q = 1.0 / rate, acc * acc
    q = ((q * t ** 4 / 4.0, q * t ** 3 / 2.0), (q * t ** 3 / 2.0, q * t * t))
    p, k = ((pos_m * pos_m, 0.0), (0.0, vel_m * vel_m)), []
    for _ in range(200):
        m = ((p[0][0] + 2.0 * t * p[0][1] + t * t * p[1][1] + q[0][0], p[0][1] + t * p[1][1] + q[0][1]),
             (p[0][1] + t * p[1][1] + q[1][0], p[1][1] + q[1][1]))
        s = (m[0][0] + pos_m * pos_m, m[0][1], m[1][0], m[1][1] + vel_m * vel_m)
        det = s[0] * s[3] - s[1] * s[2]
        inv = ((s[3] / det, -s[1] / det), (-s[2] / det, s[0] / det))
        k = [[m[r][0] * inv[0][c] + m[r][1] * inv[1][c] for c in range(2)] for r in range(2)]
        p = tuple(tuple(m[r][c] - k[r][0] * m[0][c] - k[r][1] * m[1][c] for c in range(2)) for r in range(2))
    return k


#: The fixes' gains: the speed off by the tilt on gravity.
GAINS = _gains(GNSS[0], GNSS[2], GNSS[3], GRAVITY * ATT_RAD)


def eyes(seed=1, lose=True):
    """A craft's instruments, none read yet - the place's and speed's errors 'p' and 'v', the
    tilt 'err' -; its signal lost now and then where `lose`."""
    dice = random.Random(seed)
    return {'dice': dice, 't': 0.0, 'err': [0.0] * 3, 'p': [0.0] * 3, 'v': [0.0] * 3, 'was': None,
            'hist': collections.deque(), 'fixes': collections.deque(), 'gnss': 0.0, 'star': 0.0,
            'lost': False, 'until': dice.uniform(0.5, 1.5) * LOSE_S if lose else math.inf,
            'seen': None}


def _fixed(eyes, t):
    """The fixes due by `t` laid on the errors, each against what they were when it was taken
    and that carried on to now - the errors kept since with it."""
    hist, fixes, (kpp, kpv), (kvp, kvv) = eyes['hist'], eyes['fixes'], *GAINS
    while fixes and fixes[0][0] <= t + 1e-12:
        _due, took, fp, fv, cam = fixes.popleft()
        then = next((h for h in reversed(hist) if h[0] <= took + 1e-9), hist[0])
        if cam is not None:
            shift = [e0 - c for e0, c in zip(then[3], cam)]
            for h in [h for h in hist if h[0] > took] + [(t, None, None, eyes['err'])]:
                h[3][:] = [e - s for e, s in zip(h[3], shift)]
            continue
        dp = [kpp * (f - p) + kpv * (w - v) for f, w, p, v in zip(fp, fv, then[1], then[2])]
        dv = [kvp * (f - p) + kvv * (w - v) for f, w, p, v in zip(fp, fv, then[1], then[2])]
        for h in [h for h in hist if h[0] > took] + [(t, eyes['p'], eyes['v'], None)]:
            h[1][:] = [x + a + b * (h[0] - took) for x, a, b in zip(h[1], dp, dv)]
            h[2][:] = [x + b for x, b in zip(h[2], dv)]


def read(eyes, state, dt):
    """The frame `state` (`Sky.state`) as its instruments give it `dt` s on: its attitude and
    rates the IMU's, its place, speed and height the estimate's, 'lost' the signal; the rest its
    own."""
    turn = [[float(c) for c in r] for r in state['turn']]
    vel = [float(x) for x in state['vel']]
    dice = eyes['dice']
    g, was = dice.gauss, eyes['was'] or vel
    t = eyes['t'] = eyes['t'] + dt
    if t >= eyes['until']:
        eyes['lost'] = not eyes['lost']
        eyes['until'] = t + (dice.uniform(*LOST_S) if eyes['lost'] else dice.uniform(0.5, 1.5) * LOSE_S)
    if eyes['lost']:
        eyes['err'][:] = [e + GYRO * math.sqrt(dt) * g(0.0, 1.0) for e in eyes['err']]
    else:
        keep = math.exp(-dt / ATT_S)
        eyes['err'][:] = [e * keep + ATT_RAD * math.sqrt(1.0 - keep * keep) * g(0.0, 1.0)
                          for e in eyes['err']]
    # The specific force as the tilt and the IMU's noise have it, m/s^2 off in the world.
    noise = ACC / math.sqrt(2.0 * dt)
    (x, y, z), p, v = eyes['err'], eyes['p'], eyes['v']
    f = [(c - w) / dt + GRAVITY * (a == 1) for a, (c, w) in enumerate(zip(vel, was))]
    off = (y * f[2] - z * f[1], z * f[0] - x * f[2], x * f[1] - y * f[0])
    eyes['was'] = vel
    for a in range(3):
        v[a] += (off[a] + noise * g(0.0, 1.0)) * dt
        p[a] += v[a] * dt
    hist = eyes['hist']
    hist.append((t, list(p), list(v), list(eyes['err'])))
    while hist[0][0] < t - KEEP_S:
        hist.popleft()
    if eyes['lost'] and t >= eyes['star']:
        rate, behind, cam = STAR
        eyes['star'] = t + 1.0 / rate
        eyes['fixes'].append((t + behind, t, (), (), [cam * g(0.0, 1.0) for _ in range(3)]))
    elif not eyes['lost'] and t >= eyes['gnss']:
        rate, behind, pos_m, vel_m = GNSS
        eyes['gnss'] = t + 1.0 / rate
        eyes['fixes'].append((t + behind, t, [pos_m * g(0.0, 1.0) for _ in range(3)],
                              [vel_m * g(0.0, 1.0) for _ in range(3)], None))
    _fixed(eyes, t)
    x, y, z = eyes['err']
    noise = GYRO / math.sqrt(2.0 * dt)
    t0, t1, t2 = turn[0], turn[1], turn[2]
    turn_out = [
        [t0[0] - z * t1[0] + y * t2[0], t0[1] - z * t1[1] + y * t2[1], t0[2] - z * t1[2] + y * t2[2]],
        [z * t0[0] + t1[0] - x * t2[0], z * t0[1] + t1[1] - x * t2[1], z * t0[2] + t1[2] - x * t2[2]],
        [-y * t0[0] + x * t1[0] + t2[0], -y * t0[1] + x * t1[1] + t2[1], -y * t0[2] + x * t1[2] + t2[2]],
    ]
    seen = dict(state, at=[float(c) + d for c, d in zip(state['at'], p)], lost=eyes['lost'],
                vel=[c + d for c, d in zip(vel, v)], h=float(state['h']) + p[1], v=vel[1] + v[1],
                turn=turn_out, spin=[float(s) + noise * g(0.0, 1.0) for s in state['spin']])
    eyes['seen'] = seen
    return seen


def seen(eyes, state):
    """The frame as last read (`read`), nothing stepped; as it is where nothing was read yet."""
    return eyes['seen'] if eyes['seen'] is not None else dict(state, lost=False)


def told(eyes, state, dt=0.0):
    """The frame as a law reads it on `eyes`: `dt` s on (`read`), as last read at none (`seen`);
    its truth where `eyes` is None."""
    if eyes is None:
        return state
    return read(eyes, state, dt) if dt else seen(eyes, state)
