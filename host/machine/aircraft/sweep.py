"""The rotorcraft's sweep past the spectator before their figures, a line the course's giver flies.

    card = sweep.opened(aerobatics.CARD) + course.CARD    # the QUAD page's: lift, sweep, figures

Out from home down the runway's line (`airfield.placed`) and back past the spectator beside its
middle, fast and low, over home and round to it again (`course.flown`).
"""
import functools
import math

from machine.aircraft import airfield, course

#: The line ACROSS_M off the runway's, clear of the height pole: out to OUT_M at HIGH_M, round
#: TURN_M toward the spectator and back past them at LOW_M over home to BACK_M, round and home
#: again, no faster than SPEED_M_S - at 18 m/s the quad ran 3-5 m past its turns, on turns of
#: 0.25 m it lost the line 47 m out. The field's solids stand off it: the trees 4.8 m and more,
#: the pole 2.3 m. Low over the deck: at 12 m/s the helicopter's tilted rotor 0.54 m under its
#: middle, its tips 0.3 m over the runway at LOW_M.
ACROSS_M, OUT_M, BACK_M, TURN_M, HIGH_M, LOW_M, SPEED_M_S = -2.0, -30.0, 14.0, 1.5, 1.2, 0.85, 12.0


def smoothed(out, smooth=4.8):
    """`out` (`course.laid`) with its points, tangents, bends and curve smoothed over `smooth`
    samples, its conservative `swings` kept."""
    at, step, n = out['at'], out['step'], len(out['at'])
    hw = max(1, int(round(2.5 * smooth)))
    wt = [math.exp(-0.5 * (m / smooth) ** 2) for m in range(-hw, hw + 1)]
    ws = sum(wt)
    wt = [w / ws for w in wt]
    at1 = [tuple(sum(w * at[(j + m) % n][c] for m, w in zip(range(-hw, hw + 1), wt))
                 for c in range(3)) for j in range(n)]
    at2 = [tuple(sum(w * at1[(j + m) % n][c] for m, w in zip(range(-hw, hw + 1), wt))
                 for c in range(3)) for j in range(n)]
    at = [tuple(at1[j][c] + 0.55 * (at1[j][c] - at2[j][c]) for c in range(3)) for j in range(n)]
    way = []
    for j in range(n):
        d = [b - a for a, b in zip(at[j - 1], at[(j + 1) % n])]
        size = math.sqrt(sum(x * x for x in d)) or 1.0
        way.append(tuple(x / size for x in d))
    bends = [(math.dist(way[j - 2], way[(j + 2) % n]) / (4.0 * step),
              (way[(j + 2) % n][1] - way[j - 2][1]) / (4.0 * step),
              ((math.atan2(way[(j + 2) % n][0], way[(j + 2) % n][2])
                - math.atan2(way[j - 2][0], way[j - 2][2]) + math.pi) % math.tau - math.pi)
              / (4.0 * step)) for j in range(n)]
    curve = [tuple((b - a) / (4.0 * step) for a, b in zip(way[j - 2], way[(j + 2) % n]))
             for j in range(n)]
    swings = [max(out['swings'][j], math.dist(curve[(j + 2) % n], curve[j - 2]) / (4.0 * step))
              for j in range(n)]
    out.update(at=at, way=way, bends=bends, curve=curve, swings=swings)
    return out


@functools.lru_cache(maxsize=None)
def track():
    """The sweep's line, laid as the course's (`course.laid`): no gates."""
    c, r = ACROSS_M, TURN_M

    def straight(a0, a1, across, high):
        n = max(1, round(abs(a1 - a0) / course.DS))
        return [(a0 + (a1 - a0) * k / n, across, high) for k in range(n)]

    def bend(along, way, h0, h1):
        return [(along + way * r * math.sin(t), c - r - way * r * math.cos(t), h0 + (h1 - h0) * t / math.pi)
                for t in (k * math.pi / 24.0 for k in range(24))]
    points = (straight(0.0, OUT_M, c, HIGH_M) + bend(OUT_M, -1.0, HIGH_M, LOW_M)
              + straight(OUT_M, BACK_M, c - 2.0 * r, LOW_M) + bend(BACK_M, 1.0, LOW_M, HIGH_M)
              + straight(BACK_M, 0.0, c, HIGH_M))
    return smoothed(course.laid([(x, h, z) for a, across, h in points for x, z in [airfield.placed(a, across)]], []))


def row(route, now):
    """The sweep's row for now (`aerobatics.fly`'s giver): its line flown once (`course.flown`),
    `swept` at its end."""
    return course.flown(route, track(), 1, 'swept', SPEED_M_S)


def opened(card):
    """`card` swept past the spectator (`row`) after its lift."""
    lift = [figure[0] for figure in card].index('lift') + 1
    return card[:lift] + (('sweep', row, 0.0, 0.0, 'swept'),) + card[lift:]
