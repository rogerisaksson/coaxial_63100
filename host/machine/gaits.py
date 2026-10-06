"""The gynoid's gaits: a gait a row of `machine.going`'s setpoints, her way between two a mix.

    going.ask = gaits.between(0.3)              # -1 her stand, 0 the walk's row, 1 the run's
    k, held = gaits.toward(k, held, 1.0, dt)    # asked the run: the row she goes on, a pass on
    k, held, now = gaits.passed(k, held, 1.0, now, (('leaning', 0.5),), dt)    # and her accents
    going.ask = gaits.mannered(gaits.between(k), now)                      # the row in them
"""
import math
import re

#: A gait a row of setpoints: speed, m/s; step, the seconds from a landing to the other foot's;
#: stand, a foot's stance; the bounce - `up` m over its landing and rising `rise` m/s, `bounce` s
#: on; land, the foot's pitch as it comes down, deg, the heel up; knee, deg, as it lands; lean,
#: her trunk ahead of plumb, deg; fold, the swinging knee's, deg; track, m, her feet out from
#: her capture point; off, the heel's rise as she leaves, deg; list, the pelvis's roll over the
#: standing leg, deg; under, m ahead of the ankle, the sole's point put under her; folded, the
#: share of its swing the knee's fold takes; reach, m ahead of its hip the free foot may wait;
#: elbow, her elbow's bend, deg, and play, deg more a deg its shoulder reaches ahead; turn, the
#: pelvis's with the leg that lands, deg; strut, the knee, deg, the pelvis is never over what
#: a standing leg reaches on (`machine.strut`): 6, the form's all but straight; standing,
#: weigh, how far toward her left foot her weight is, -1 her right to 1, and hang, deg the
#: pelvis rolls up over that leg a unit of it.
#: The run's is `machine.runner`'s: asked 1.5 m/s, 1.44-1.47 at 508-522 J/m drawn (2026-10-05).
RUN = {'speed': 1.5, 'step': 0.40, 'stand': 0.30, 'up': 0.0, 'rise': 0.49, 'bounce': 0.30,
       'land': 14.0, 'knee': 22.0, 'lean': 6.0, 'fold': 55.0, 'track': 0.035, 'off': 50.0,
       'list': 0.0, 'under': 0.117, 'folded': 1.0, 'reach': 0.36, 'elbow': 80.0, 'play': 0.0,
       'turn': 0.0, 'strut': 6.0, 'weigh': 0.0, 'hang': 0.0}
#: The walk's, a woman's as near as found (2026-10-06): searched from the walk as built's time
#: on a walk's price and its widths off `tools.sim.normal.BAND`, 1 920 rows, then on its J/m,
#: a stop and a passage to her jog and back too, 960. 0.85 m/s at 502 J/m, 0.20-0.26 off the
#: band - the heel 22 deg up as its toes leave where 28 -, `looks.FORM` but her head 42 mm
#: aside and bobbing 33; her ways 77 of 104, the page's presses 66 of 72.
WALK = {'speed': 0.87, 'step': 0.522, 'stand': 0.651, 'up': 0.104, 'rise': 0.0, 'bounce': 0.107,
        'land': -4.9, 'knee': 11.8, 'lean': 4.9, 'fold': 45.2, 'track': 0.044, 'off': 2.1,
        'list': 3.1, 'under': 0.036, 'folded': 0.63, 'reach': 0.333, 'elbow': 30.0, 'play': 0.9,
        'turn': 1.1, 'strut': 6.0, 'weigh': 0.0, 'hang': 0.0}
#: Her jog, the run's row at the walk's speed: her way from the walk to the run passes it. On
#: its speed alone the run's row goes 0.54 m/s at 936 J/m asked 0.6, 0.74 at 704 asked 0.8,
#: 0.99 at 602, 1.19 at 547 (2026-10-05). Before it a row found between, at 1.42 m/s, passed
#: walk to run and back on 2 timings of 10.
JOG = dict(RUN, speed=0.8)
#: Between her walk and her jog, the walk's shape on the jog's time: a gait, where their mix
#: is none. From her jog, each of the walk's shares taken alone and held (2026-10-05): its
#: time, up; its bounce, its landing or its swing, down in 2 s - its bounce and its landing
#: together, up, with its swing too, up; all of it at once, down.
#: Her quick step, the walk's row as first found - 2 240 rows on a walk's price before her
#: stand was in the law: a foot always down, 0.53 s a step, the knee folding 10 deg, 0.81 m/s
#: at 392 J/m. On stilts as her walk (the user, 2026-10-05), 4.1 off a woman's band: her way
#: between her walk and EASE since - on the walk's own shape every passage to her jog was down
#: in a step.
QUICK = {'speed': 0.76, 'step': 0.528, 'stand': 0.581, 'up': 0.10, 'rise': 0.0, 'bounce': 0.253,
         'land': -12.9, 'knee': 19.4, 'lean': 5.1, 'fold': 10.0, 'track': 0.048, 'off': 1.7,
         'list': 0.0, 'under': 0.037, 'folded': 0.65, 'reach': 0.248, 'elbow': 30.0, 'play': 0.9,
         'turn': 0.0, 'strut': 6.0, 'weigh': 0.0, 'hang': 0.0}
EASE = dict(QUICK, stand=JOG['stand'], step=JOG['step'])
#: Standing: the walk's row at no speed.
STAND = dict(WALK, speed=0.0)

#: Her way's rows that are gaits, by `between`'s k: her stand, the walk's, her quick step,
#: EASE, her jog, the run's. Between her stand and the walk's, and between her jog and the
#: run's, every row is one - a slower walk, a slower run -; between her quick step and her
#: jog's none but EASE.
WAY = ((-1.0, STAND), (0.0, WALK), (0.125, QUICK), (0.25, EASE), (0.5, JOG), (1.0, RUN))
KNOTS = tuple(k for k, _row in WAY)
#: She goes from row to row this much of her way a second, (under this k, a second), and stays
#: DWELL_S on a gait between before she leaves it. Walk to jog and back through EASE, half a
#: second a move and 2 s on it: up 23 timings of 24 to her jog, 24 of 24 back; a second a move,
#: 2 of 6 to her jog; her walk and her jog mixed straight over 0.6-3 s, back to her walk 12 of
#: 20, and over 2-6 s through EASE with no stay, 12 of 24 back and 8 of 24 on. Her jog to the
#: run over 3 s, her leaps grew, 0.43 to 0.54 s a step, and she was down; over 5 and over 10 s
#: she ran 1.5 m/s (2026-10-05).
RATES, DWELL_S = ((0.0, 1.0), (0.5, 0.5), (1.0, 0.1)), 2.0

#: A walk's ways - a landing kept clear of the standing foot, a foot due by where she is, her
#: manners - are a row's the more both feet bear a step, in full from BESIDE_S of it.
BESIDE_S = 0.05

#: Her manners (the user, 2026-10-05: a middle layer that blends, concepts as tuples): a
#: manner a row's setpoints moved, each this far at an amount of 1 - or other manners,
#: ((manner, amount), ..). Asked as such tuples, their amounts 0 to 1 and summed, each coming
#: on and going over MANNER_S. Alone on the walk's row, in `tools.sim.normal`'s words and at
#: J/m (2026-10-06): leaning 0.50 at 559, her trunk 16 deg - at 12 deg more she fell from her
#: stand, and with her knees bent 18 was up at 17 and at 20: bent legs carry a lean -;
#: crouched 0.58, Groucho's, at 409; wide 0.68, her feet 0.36 legs apart, at 495; tripping
#: 0.36, a step 0.42 s, at 666; catwalk swaying 0.40, the pelvis turning 29 deg and rolling
#: 11.5, at 570 - its list 6 deg and its turn 16 fell from her stand. Standing, hip left:
#: 77 % of her on her left sole, the pelvis rolled 5.5 deg up over it, her right knee 29 deg
#: where 10.
MANNERS = {'leaning': {'lean': 9.0}, 'crouched': {'strut': 18.0, 'knee': 14.0},
           'wide': {'track': 0.036}, 'tripping': {'step': -0.1, 'stand': -0.1},
           'catwalk': {'list': 1.9, 'turn': 12.9},
           'hip left': {'weigh': 0.8, 'hang': 7.0}, 'hip right': {'weigh': -0.8, 'hang': 7.0},
           'into the wind': (('leaning', 1.0), ('crouched', 0.8), ('tripping', 0.4))}
MANNER_S = 2.0
#: Her stand's own setpoints: an accent that moves them is a pose's, let go before she goes
#: on - walked off from her weight on one leg she was down in 1.2 s (2026-10-06).
POSES = frozenset(('weigh', 'hang'))


def derive():
    """EASE and STAND anew from the walk's row and her jog's as they are: a knob moved those."""
    STAND.update(dict(WALK, speed=0.0))


def share(row):
    """How much of a walk `row` is, 0 to 1: the s both feet bear a step, of BESIDE_S."""
    return min(1.0, max(0.0, (row['stand'] - row['step']) / BESIDE_S))


def told(text):
    """((manner, amount), ..) of 'leaning:0.5 into the wind:1': a manner's name, its amount."""
    return tuple((name.strip(), float(amount))
                 for name, amount in re.findall(r"([A-Za-z' ]+):\s*([-+.\d]+)", text))


def posed(amounts):
    """How much of a pose's accent she is in, of `amounts` {manner: amount}."""
    return max((v for name, v in amounts.items() if POSES & set(MANNERS[name])), default=0.0)


def passed(k, held, want, now, asked, dt):
    """(k, held, {manner: amount}) `dt` s on: her way toward the row `want` (`toward`), her
    accents toward `asked` (`manners`) - a pose's let go before she leaves her stand, and
    hers again as she stands."""
    now = manners(now, asked, dt, going=want > -1.0 or k > -1.0)
    return toward(k, held, -1.0 if k <= -1.0 and posed(now) else want, dt) + (now,)


def plain(asked, k=1.0, into=None):
    """{manner: amount} of `asked` ((manner, amount), ..), each manner made of others as
    those."""
    into = {} if into is None else into
    for name, amount in asked:
        if isinstance(MANNERS[name], tuple):
            plain(MANNERS[name], k * amount, into)
        else:
            into[name] = into.get(name, 0.0) + k * amount
    return into


def manners(now, asked, dt, going=False):
    """{manner: amount} `dt` s on from `now`, asked `asked` ((manner, amount), ..): each
    amount, 0 to 1, a full one in MANNER_S; `going`, a pose's is none."""
    want, step, out = plain(asked), dt / MANNER_S, {}
    for name in set(now) | set(want):
        at, to = now.get(name, 0.0), max(0.0, min(1.0, want.get(name, 0.0)))
        if going and POSES & set(MANNERS[name]):
            to = 0.0
        at += max(-step, min(step, to - at))
        if at > 0.0:
            out[name] = at
    return out


def mannered(row, amounts):
    """`row` in her manners `amounts` {manner: amount}: its setpoints moved as MANNERS has
    them, the more a walk's the row is."""
    if not amounts:
        return row
    k, out = share(row), dict(row)
    for name, amount in amounts.items():
        for key, at_one in MANNERS[name].items():
            out[key] += k * amount * at_one
    return out


def mix(a, b, k):
    """The row `k` of the way from `a` to `b`."""
    return {name: a[name] + (b[name] - a[name]) * k for name in a}


def between(k):
    """The row `k` of her way (WAY): -1 her stand, 0 the walk's, 0.5 her jog, 1 the run's."""
    k = max(WAY[0][0], min(WAY[-1][0], k))
    (k0, a), (k1, b) = next(pair for pair in zip(WAY, WAY[1:]) if k <= pair[1][0])
    return mix(a, b, (k - k0) / (k1 - k0))


def toward(k, held, want, dt):
    """(k, held) `dt` s on, asked the row `want`: `k` moved toward it at RATES' pace for where
    it is, no further than the next of KNOTS, and off one she walks or jogs on only when she
    has been `held` s on it, DWELL_S at least."""
    want = max(-1.0, min(1.0, want))
    if want == k or (k in KNOTS and 0.0 <= k <= 0.5 and held < DWELL_S):
        return k, held + dt
    up = want > k
    at = k if up else k - 1e-9
    rate = next(r for under, r in RATES if at < under or under >= 1.0)
    stop = (min([want] + [knot for knot in KNOTS if knot > k]) if up else
            max([want] + [knot for knot in KNOTS if knot < k]))
    if abs(stop - k) <= rate * dt:
        return stop, 0.0
    return k + math.copysign(rate * dt, stop - k), 0.0
