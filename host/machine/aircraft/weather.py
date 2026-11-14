"""The air a craft flies through: a kind of weather, its wind swinging, its gusts, its eddies.

    air = weather.air(seed, 'gusty')              # still before its first second, that kind kept
    weather.blown(air, dt)                        # the air on: its wind, its gusts, its eddies
    wind = weather.wind_at(air['wind'], h)        # its wind where the frame is

What the frame is flown through, none of it told to its law (`AIR`).
"""
import math
import random

from machine.aircraft import compile as aircraft_compile

_W = aircraft_compile.weather()

#: The air's kinds (`world.toml` `[weather]`): mean m/s, veer deg, veer s, gust m/s, every s, eddy m/s rms.
AIR = _W['air']
TOUR, KIND_S, EASE_S, TURN_DEG = tuple(AIR), _W['kind_s'], _W['ease_s'], _W['turn_deg']
GUST_S, GUST_SHARE, GUST_DEG = _W['gust_s'], _W['gust_share'], _W['gust_deg']
EDDY_S, EDDY_DISC_S, EDDY_DISC, EDDY_UP = _W['eddy_s'], _W['eddy_disc_s'], _W['eddy_disc'], _W['eddy_up']
REF_M, SHEAR, SHEAR_M, SHEAR_MOST, RISE_M = _W['ref_m'], _W['shear'], _W['shear_m'], _W['shear_most'], _W['rise_m']
DISCS = _W['discs']


def air(seed=3, kind=None):
    """Air before its first second, still: the 'kind' it is in - toured from TOUR's first, or
    `kind` 'kept' -, how long 'for', its sizes 'come' so far (the wind's mean, its way's swing,
    its gusts', its eddies'); the 'way' its wind blows, rad from z toward x, the way it comes
    round 'to' and its 'swing', eased 'from' (its sizes, its way) where its kind turned; its
    'gust' (s into it, of, its share, rad off the way) and the seconds to the 'next'; the
    frame's 'eddy' and each disc's, m/s, and the first stages they follow, 'stirs'; what they
    make: its 'wind' at REF_M, m/s in the world, of it the 'steady' - its mean along its way -,
    the gust in it 'blows', m/s."""
    return {'kind': kind or TOUR[0], 'kept': kind is not None, 'for': 0.0, 'come': [0.0] * 4,
            'from': ([0.0] * 4, 0.0), 'way': 0.0, 'to': 0.0, 'swing': 0.0, 'gust': None,
            'next': 0.0, 'blows': 0.0, 'eddy': [0.0] * 3, 'eddies': [[0.0] * 3 for _ in range(DISCS)],
            'stirs': [[0.0] * 3 for _ in range(DISCS + 1)],
            'dice': random.Random(seed), 'wind': (0.0, 0.0, 0.0), 'steady': (0.0, 0.0, 0.0)}


def blown(air, dt):
    """The air `dt` s on, its wind at REF_M: its kind's turn over, TOUR's next and a new way;
    its sizes eased to its kind's and its way round, swung; a gust begun where one is due,
    risen and fallen; its eddies turned over, each as far as its seconds let it."""
    dice = air['dice']
    air['for'] += dt
    if not air['kept'] and air['for'] >= KIND_S:
        air['kind'], air['for'] = TOUR[(TOUR.index(air['kind']) + 1) % len(TOUR)], 0.0
        air['from'] = (list(air['come']), air['way'])
        air['to'] += math.radians(dice.uniform(-TURN_DEG, TURN_DEG))
    mean, veer, veer_s, gust, every, rough = AIR[air['kind']]
    ease = min(1.0, air['for'] / EASE_S)
    ease *= ease * (3.0 - 2.0 * ease)
    was, way = air['from']
    mean, veer, gust, rough = air['come'] = [then + (now - then) * ease for then, now in zip(
        was, (mean, veer, gust, rough))]
    air['way'] = way + (air['to'] - way) * ease
    air['swing'] += math.tau * dt / veer_s if veer_s else 0.0
    way = air['way'] + math.radians(veer) * math.sin(air['swing'])
    if air['gust'] is None:
        air['next'] -= dt
        if every and air['next'] <= 0.0:
            air['gust'] = (0.0, dice.uniform(*GUST_S), dice.uniform(*GUST_SHARE),
                           math.radians(dice.uniform(-GUST_DEG, GUST_DEG)))
            air['next'] = every * dice.uniform(0.5, 1.5)
    blows, off = 0.0, 0.0
    if air['gust'] is not None:
        into, of, share, off = air['gust']
        blows = 0.5 * share * gust * (1.0 - math.cos(math.tau * into / of))
        air['gust'] = (into + dt, of, share, off) if into + dt < of else None
    for eddy, stir, size, over in [(air['eddy'], air['stirs'][0], rough, EDDY_S)] + [
            (own, stir, EDDY_DISC * rough, EDDY_DISC_S)
            for own, stir in zip(air['eddies'], air['stirs'][1:])]:
        keep = math.exp(-dt / over)
        kick = math.sqrt(2.0) * size * math.sqrt(1.0 - keep * keep)
        for k in range(3):
            stir[k] = stir[k] * keep + (EDDY_UP if k == 1 else 1.0) * kick * dice.gauss(0.0, 1.0)
            eddy[k] = eddy[k] * keep + stir[k] * (1.0 - keep)
    air['blows'], air['steady'] = blows, (mean * math.sin(way), 0.0, mean * math.cos(way))
    steady = air['steady']
    air['wind'] = (steady[0] + blows * math.sin(way + off) + air['eddy'][0], air['eddy'][1],
                   steady[2] + blows * math.cos(way + off) + air['eddy'][2])
    return air['wind']


def kept(air):
    """The air's kind stepped by hand: TOUR's first, kept, then the next; after the last, the
    tour again."""
    k = TOUR.index(air['kind']) + 1 if air['kept'] else 0
    air['kind'], air['kept'], air['for'] = TOUR[k % len(TOUR)], k < len(TOUR), 0.0
    air['from'] = (list(air['come']), air['way'])


def airspeeds(frame, air):
    """(the frame's speed as its flight controller has it - over the ground -, its speed
    through `air`), m/s: what its boards' observers are told, what their truths fly in."""
    wind = wind_at(air['wind'], frame['h'], air['steady']) if air else (0.0, 0.0, 0.0)
    return (math.sqrt(sum(float(v) ** 2 for v in frame['vel'])),
            math.sqrt(sum((float(v) - u) ** 2 for v, u in zip(frame['vel'], wind))))


def wind_at(wind, h, steady=(0.0, 0.0, 0.0)):
    """The air's speed `wind`, m/s at REF_M, where the frame is `h` m up: sheared along the
    floor, what of it is not `steady` - its gusts, its eddies, its rise and fall - none at the
    floor."""
    low, near = min(SHEAR_MOST, (max(h, SHEAR_M) / REF_M) ** SHEAR), min(1.0, max(0.0, h) / RISE_M)
    return (low * (steady[0] + near * (wind[0] - steady[0])), low * near * wind[1],
            low * (steady[2] + near * (wind[2] - steady[2])))
