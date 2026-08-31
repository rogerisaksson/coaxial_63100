"""The anti-alias chain the board runs, designed here.

The converter is three orders of magnitude faster than the link. Closing
that gap by throwing samples away folds everything above half the output
rate back into the answer as something that was never there, and no field in
a record says so. This designs what stops it, and - the part that matters -
REPORTS WHAT IT ACTUALLY ATTENUATES rather than asserting it is enough.

    chain = design(fs=3.75e6, out_rate=1000.0, order=4)
    chain['boxcar'], chain['decimate']      # what the board runs
    chain['sections']                        # biquads, b0 b1 b2 a1 a2
    chain['worst_alias_db']                  # the honest number

**Bessel, and the trade is deliberate.** Its group delay is maximally flat,
so a current waveform arrives its own shape rather than smeared - which is
what a drive wants from a measurement. It pays with a gentle rolloff: at the
same order it attenuates a stopband far less than a Butterworth. Both
numbers come out of `design`, and nothing here picks for the operator.

Two stages, because one cannot be both cheap and sharp. A boxcar costs one
add per sample and is the only thing that can run at the converter's rate;
biquads cost ~10 cycles each and run on what the boxcar left. The host is
the only place that knows the output rate, so it is the only place that can
choose a cutoff against the sampling theorem.

No numpy: this tree's host runs on pyserial and the standard library, so the
roots come from Durand-Kerner and the -3 dB point from a bisection. Degree 8
is the ceiling and both converge in a few dozen iterations.
"""
import cmath
import math

#: Biquads the board will run. `Filter/Inc/filter.h`'s FILTER_MAX_SECTIONS,
#: and the one place a host may not simply ask for more.
MAX_SECTIONS = 4

#: A boxcar sums into an int32 against a 65535 code, so it cannot hold more
#: than this - `LIVE_MAX_ADDITIONS` in board_limits.h, the same bound the
#: daq accumulator saturates at, for the same arithmetic.
MAX_BOXCAR = 32767


def reverse_bessel(n):
    """Coefficients of the reverse Bessel polynomial, a[k] for s**k.

    a[k] = (2n-k)! / (2**(n-k) * k! * (n-k)!). Exact in integers, so the
    poles below start from something with no rounding in it yet.
    """
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
    """Every root of a real polynomial, by Durand-Kerner.

    Started off a spiral rather than the unit circle: equally spaced points
    on a circle are a fixed point of the iteration for some polynomials and
    it never moves off them.
    """
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
    """The analog Bessel poles, scaled so -3 dB lands at omega = 1.

    Bessel's own normalisation is unit group delay, not unit cutoff; an
    anti-alias filter is specified by where it stops passing, so the poles
    are divided by the -3 dB frequency found here. Bisection rather than a
    table: a table is a second answer that goes stale at the order nobody
    tabulated.
    """
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
    """Biquads for a Bessel low-pass at `fc`, sampled at `fs`.

    Bilinear with the cutoff pre-warped, so the digital -3 dB lands where it
    was asked for rather than where the transform's compression put it -
    which at fc/fs of a tenth is already 3 % out.
    """
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
    """|H| of an accumulate-and-dump of `length`, at rate `fs`.

    The Dirichlet kernel, not a guess: nulls at multiples of fs/length and a
    first sidelobe 13 dB down, which is exactly why it is a first stage and
    never the whole filter.
    """
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
    """|H| from the converter's input to the record, at input frequency `f`.

    Every stage in the order the board runs them: the boxcar at the full
    rate, then the biquads at what its dump left, then the decimation - and
    the folds each one brings, which is the whole question being asked.
    """
    fs, mid = chain['fs'], chain['mid_rate']
    gain = boxcar_gain(f, chain['boxcar'], fs)
    return gain * abs(response(chain['sections'], _fold(f, mid), mid))


def worst_alias_db(chain, harmonics=64, steps=41):
    """The loudest thing that can fold into the passband, in dB.

    For every output frequency the passband keeps, the input frequencies
    that land on it are `k * out_rate +/- u`. This walks them and reports the
    worst gain any of them still has - so a design is judged by what it
    fails to stop, not by the shape of its passband.
    """
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
    """Peak-to-peak group delay across the passband, in output samples.

    The reason a Bessel is here at all. Measured off the phase rather than
    asserted, so a design that was asked for an order it does not have still
    reports what it really does.
    """
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

    The boxcar takes as much of the thinning as it can: it is the stage that
    runs at the converter's rate, and every factor left to it is a factor the
    biquads do not pay for. What `decimate` keeps is the headroom the shaped
    filter needs above the output rate.
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
    """The whole chain, and what it is worth.

    fs        what the converter is running at
    out_rate  records a second the link is to carry
    order     Bessel order, 1..8 - two per biquad, four biquads
    cutoff    passband edge in Hz. Default a FIFTH of `out_rate`, not the
              half the sampling theorem allows nor the 0.4 a sharper filter
              would take: what folds onto the passband edge is
              `out_rate - cutoff`, and a Bessel has barely started there.
              Measured on this design at a 10 kHz mid rate, cutoff 400 Hz:

                  order   at 1.5*fc   at 4*fc
                      2     -6.4 dB   -21.8 dB
                      4     -7.5 dB   -37.2 dB
                      6     -7.4 dB   -48.9 dB
                      8     -7.2 dB   -57.4 dB

              Order steepens the far stopband and leaves the knee where it
              was - that is Bessel, and it is why the fold has to be put
              four octaves out rather than one. At 0.4 * out_rate the whole
              chain stopped -7.6 dB of what folds; at 0.2 it stops -35 dB.
    headroom  how far above `out_rate` the biquads run. The boxcar takes the
              rest, being the cheap stage.

    Returns the design the board loads and the analysis a caller should read
    before believing it: `worst_alias_db` is what still folds in, and
    `group_delay_samples` is what the Bessel bought.
    """
    if order < 1 or order > 2 * MAX_SECTIONS:
        raise ValueError('order is 1..%d - the board runs %d biquads'
                         % (2 * MAX_SECTIONS, MAX_SECTIONS))
    if out_rate <= 0 or fs <= 0 or out_rate > fs:
        raise ValueError('an output rate of %g Hz off a %g Hz converter is '
                         'not a decimation' % (out_rate, fs))

    ratio = int(round(fs / float(out_rate)))
    boxcar, decimate = _split(max(1, ratio), headroom)
    mid_rate = fs / float(boxcar)
    edge = float(cutoff) if cutoff else 0.2 * out_rate

    chain = {
        'fs': float(fs),
        'out_rate': fs / float(boxcar * decimate),
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
    """The chain for a link that carries `max_rate_hz` records a second.

    MORE CHANNELS IS A LONGER RECORD IS FEWER RECORDS A SECOND IS A LOWER
    CUTOFF. The sampling theorem does not care how many channels were
    wanted: whatever the link carries is the output rate, and the
    passband is a fifth of it. The board reports what it can carry for
    the stride it actually has - `state()['max_rate_hz']` - so this is
    where that number becomes a filter.

    `margin` keeps the task under the ceiling rather than on it: a ring
    produced at exactly the drain rate overflows on the first slow read.
    """
    if max_rate_hz <= 0:
        raise ValueError('a link that carries no records carries no '
                         'measurement either')
    return design(fs, float(max_rate_hz) * margin, order=order, **kw)


def ladder(fs, max_rate_hz, rungs=4, order=4, margin=0.8, step=2.0):
    """A ladder of whole chains, each `step` times slower than the last.

    The board climbs it when its ring fills and comes back down when the
    link has caught up. Every rung is a COMPLETE design - boxcar,
    coefficients, decimation - because decimating harder without
    redesigning is exactly how a fold gets in, and the board cannot
    design anything: it chooses between designs sent to it.

    Rung 0 is what the link carries today. Four rungs at a factor of two
    cover 8x, which is a fibre against a bottle.
    """
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
    harness and the wire both take them in."""
    return [value for section in sections_list for value in section]
