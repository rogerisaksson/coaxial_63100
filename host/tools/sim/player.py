"""A player's hands on her: the page's keys and their holds, the plays of them, the shoves."""

#: A player's walk (`se2`'s), m/s; a stop.
WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)
#: A player's twists, a hold each, s: the page's keys send none outside them
#: (`humanoid_keys.twist`); an agent sends the same.
KEYS = {'w': WALK, 's': (-0.35, 0.0, 0.0), 'q': (0.0, 0.5, 0.0), 'e': (0.0, -0.5, 0.0),
        'wa': (0.69, 0.0, 0.7), 'wd': (0.69, 0.0, -0.7), 'a': (0.0, 0.0, 0.7), 'W': (1.4, 0.0, 0.0), '-': STOP}
PLAYS = {'start_stop': 'w3 -3 w3 -3', 'turns': 'w2 wa2 wd2 -3', 'strafes': 'q2 e2 -3', 'reverse': 'w2 s2 w2 -3',
         'sprint': 'w2 W3 w2 -3', 'spin': 'a3 -3', 'weave': 'w1 wa1 w1 wd1 w1 -3', 'about': 'a2 wa1 w2 -3',
         'walk': 'w10 -2', 'moves': 'w2 wa2 q2 s2 W2 -3', 'stutter': 'w3 -0.25 w2 -0.2 w3 -3'}


def played(keys):
    """(commands, seconds) of a player's keys, 'w3 -2': each a twist held its seconds, from 1 s."""
    at, plan = 1.0, []
    for word in keys.split():
        plan.append((at, {'twist': KEYS[word.rstrip('0123456789.')]}))
        at += float(word[len(word.rstrip('0123456789.')):])
    return plan, at + 1.0


#: Shoves: N for SHOVE_S, the way they push (deg about up from ahead, 90 to her left), standing or
#: walking, at one of MOMENTS - a quarter of her stride apart, the foot that swings decides; 80 N
#: held, 200 N held or fallen with her head slower than `scenarios.FALL_M_S` onto the floor.
FORCES_N, SHOVE_S, MOMENTS = (80, 200), 0.12, (5.0, 5.15, 5.30, 5.45)
SHOVES = {'shove_%d_%03d_%s_%d' % (n, way, gait, k): (n, way, gait, at)
          for n in FORCES_N for way in range(0, 360, 45) for gait in ('stand', 'walk')
          for k, at in enumerate(MOMENTS)}
