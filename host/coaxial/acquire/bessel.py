"""The anti-alias chain the board runs, designed here."""
import cmath
import math

#: Biquads the board will run. `filter/inc/filter.h`'s FILTER_MAX_SECTIONS,
#: and the one place a host may not simply ask for more.
MAX_SECTIONS = 4

#: A boxcar sums into an int32 against a 65535 code, so it cannot hold more
#: than this - `LIVE_MAX_ADDITIONS` in board_limits.h, the same bound the
#: daq accumulator saturates at, for the same arithmetic.
MAX_BOXCAR = 32767


def reverse_bessel(n):
    """Coefficients of the reverse Bessel polynomial, a[k] for s**k."""
    return [math.factorial(2 * n - k)
            // (2 ** (n - k) * math.factorial(k) * math.factorial(n - k))
            for k in range(n + 1)]


def _value(coeffs, z):
    """The polynomial at `z`, Horner from the top."""
    out = 0j
    for c in reversed(coeffs):
        out = out * z + c
    return out


def roots(coeffs, rounds=500, tol=1e-14):
    """Every root of a real polynomial, by Durand-Kerner."""
    n = len(coeffs) - 1
    if n < 1:
        return []
    lead = float(coeffs[-1])
    monic = [c / lead for c in coeffs]
    guess = [cmath.exp(2j * math.pi * i / n) * (0.4 + 0.9 * i / n)
             for i in range(n)]

    for _ in range(rounds):
        moved = 0.0
        for i in range(n):
            denominator = 1.0 + 0j
            for j in range(n):
                if i != j:
                    denominator *= (guess[i] - guess[j])
            if denominator == 0:
                continue
            step = _value(monic, guess[i]) / denominator
            guess[i] -= step
            moved = max(moved, abs(step))
        if moved < tol:
            break
    return guess


def _magnitude(poles, omega):
    """|H(j omega)| for the all-pole prototype with unit DC gain."""
    out = 1.0
    for p in poles:
        out *= abs(p) / abs(1j * omega - p)
    return out


def prototype(order):
    """The analog Bessel poles, scaled so -3 dB lands at omega = 1."""
    if order < 1:
        raise ValueError('order is at least 1, not %r' % (order,))
    poles = roots(reverse_bessel(order))

    low, high = 1e-6, 1e3
    for _ in range(200):
        mid = 0.5 * (low + high)
        if _magnitude(poles, mid) > 0.5 ** 0.5:
            low = mid
        else:
            high = mid
    cut = 0.5 * (low + high)
    return [p / cut for p in poles]


def _pair(poles):
    """Conjugate pairs first, then any real pole. One section each."""
    left = sorted(poles, key=lambda p: (round(p.imag, 9), p.real))
    out, used = [], [False] * len(left)
    for i, p in enumerate(left):
        if used[i]:
            continue
        if abs(p.imag) < 1e-9:
            used[i] = True
            out.append((p,))
            continue
        for j in range(i + 1, len(left)):
            if not used[j] and abs(left[j] - p.conjugate()) < 1e-6:
                used[i] = used[j] = True
                out.append((p, left[j]))
                break
    return out


def sections(fs, fc, order):
    """Biquads for a Bessel low-pass at `fc`, sampled at `fs`."""
    if not 0.0 < fc < fs / 2.0:
        raise ValueError('a cutoff of %g Hz is not below the Nyquist of a '
                         '%g Hz stream' % (fc, fs))
    warped = 2.0 * fs * math.tan(math.pi * fc / fs)
    c = 2.0 * fs
    out = []

    for group in _pair(prototype(order)):
        if len(group) == 1:
            p = group[0].real * warped
            a0 = c - p
            out.append((-p / a0, -p / a0, 0.0, (-p - c) / a0, 0.0))
            continue
        p = group[0] * warped
        squared = abs(p) ** 2
        linear = -2.0 * p.real
        a0 = c * c + linear * c + squared
        out.append((squared / a0, 2.0 * squared / a0, squared / a0,
                    2.0 * (squared - c * c) / a0,
                    (c * c - linear * c + squared) / a0))
    return out


def response(design_sections, f, fs):
    """The cascade's complex response at `f`, sampled at `fs`."""
    z = cmath.exp(-2j * math.pi * f / fs)
    out = 1.0 + 0j
    for b0, b1, b2, a1, a2 in design_sections:
        out *= ((b0 + b1 * z + b2 * z * z) /
                (1.0 + a1 * z + a2 * z * z))
    return out


def boxcar_gain(f, length, fs):
    """|H| of an accumulate-and-dump of `length`, at rate `fs`."""
    if length <= 1:
        return 1.0
    x = math.pi * f / fs
    if abs(math.sin(x)) < 1e-15:
        return 1.0
    return abs(math.sin(length * x) / (length * math.sin(x)))


def _fold(f, rate):
    """Where `f` lands after sampling at `rate`, in [0, rate/2]."""
    folded = math.fmod(abs(f), rate)
    return rate - folded if folded > rate / 2.0 else folded


def chain_gain(chain, f):
    """|H| from the converter's input to the record, at input frequency `f`."""
    fs, mid = chain['fs'], chain['mid_rate']
    gain = boxcar_gain(f, chain['boxcar'], fs)
    return gain * abs(response(chain['sections'], _fold(f, mid), mid))


def worst_alias_db(chain, harmonics=64, steps=41):
    """The loudest thing that can fold into the passband, in dB."""
    out_rate, fc = chain['out_rate'], chain['cutoff']
    worst = 0.0
    for step in range(steps):
        u = fc * step / max(1, steps - 1)
        for k in range(1, harmonics + 1):
            for f in (k * out_rate - u, k * out_rate + u):
                if f > chain['fs'] / 2.0:
                    continue
                worst = max(worst, chain_gain(chain, f))
    return 20.0 * math.log10(worst) if worst > 0 else -math.inf


def group_delay_ripple(chain, points=32):
    """Peak-to-peak group delay across the passband, in output samples."""
    fc, mid = chain['cutoff'], chain['mid_rate']
    delays = []
    step = fc / (points * 8.0)
    for i in range(1, points + 1):
        f = fc * i / points
        before = cmath.phase(response(chain['sections'], f - step, mid))
        after = cmath.phase(response(chain['sections'], f + step, mid))
        slip = after - before
        while slip > math.pi:
            slip -= 2.0 * math.pi
        while slip < -math.pi:
            slip += 2.0 * math.pi
        delays.append(-slip / (2.0 * math.pi * 2.0 * step))
    return (max(delays) - min(delays)) * chain['out_rate']


def _split(ratio, want_mid):
    """(boxcar, decimate) whose product is `ratio`, decimate near `want_mid`.
    """
    best = None
    for decimate in range(1, ratio + 1):
        if ratio % decimate:
            continue
        boxcar = ratio // decimate
        if boxcar > MAX_BOXCAR:
            continue
        score = abs(math.log(decimate / float(want_mid)))
        if best is None or score < best[0]:
            best = (score, boxcar, decimate)
    if best is None:
        raise ValueError('no boxcar under %d divides a ratio of %d - ask for '
                         'a higher output rate' % (MAX_BOXCAR, ratio))
    return best[1], best[2]


def design(fs, out_rate, order=4, cutoff=None, headroom=8):
    """The whole chain, and what it is worth."""
    if order < 1 or order > 2 * MAX_SECTIONS:
        raise ValueError('order is 1..%d - the board runs %d biquads'
                         % (2 * MAX_SECTIONS, MAX_SECTIONS))
    if out_rate <= 0 or fs <= 0 or out_rate > fs:
        raise ValueError('an output rate of %g Hz off a %g Hz converter is '
                         'not a decimation' % (out_rate, fs))

    ratio = int(round(fs / float(out_rate)))
    boxcar, decimate = _split(max(1, ratio), headroom)
    mid_rate = fs / float(boxcar)
    # THE ACHIEVED RATE, NOT THE ASKED ONE.
    achieved = fs / float(boxcar * decimate)
    edge = float(cutoff) if cutoff else 0.2 * achieved

    chain = {
        'fs': float(fs),
        'out_rate': achieved,
        'mid_rate': mid_rate,
        'boxcar': boxcar,
        'decimate': decimate,
        'order': order,
        'cutoff': edge,
        'sections': sections(mid_rate, edge, order),
    }
    chain['worst_alias_db'] = worst_alias_db(chain)
    chain['group_delay_samples'] = group_delay_ripple(chain)
    chain['passband_db'] = 20.0 * math.log10(max(1e-30, chain_gain(chain, edge)))
    return chain


def for_link(fs, max_rate_hz, order=4, margin=0.8, **kw):
    """The chain for a link that carries `max_rate_hz` records a second."""
    if max_rate_hz <= 0:
        raise ValueError('a link that carries no records carries no '
                         'measurement either')
    return design(fs, float(max_rate_hz) * margin, order=order, **kw)


def ladder(fs, max_rate_hz, rungs=4, order=4, margin=0.8, step=2.0):
    """A ladder of whole chains, each `step` times slower than the last."""
    out = []
    for n in range(rungs):
        rate = float(max_rate_hz) * margin / (step ** n)
        if rate * (step if n else 1.0) < 1.0:
            break
        try:
            out.append(design(fs, rate, order=order))
        except ValueError:
            break              # the ratio ran out of factors: stop here
    if not out:
        raise ValueError('no rung fits a %g Hz link off a %g Hz converter'
                         % (max_rate_hz, fs))
    return out


def flat(sections_list):
    """The sections as one list of floats, b0 b1 b2 a1 a2 - the order the
    harness and the wire both take them in.
    """
    return [value for section in sections_list for value in section]
