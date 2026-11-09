"""A player's hands on her: the page's keys and their holds, the plays of them, the shoves."""
from machine.law import se2

#: A player's walk (`se2`'s, her build's), m/s; a stop.
WALK, STOP = (se2.WALK_SPEED, 0.0, 0.0), (0.0, 0.0, 0.0)
#: A player's twists, a hold each, s: the page's keys send none outside them, her build's speeds
#: (`humanoid_keys.twist`, its turn to the hundredth); an agent sends the same.
_YAW = round(se2.YAW_RATE, 2)
KEYS = {'w': WALK, 's': (-se2.BACK_SPEED, 0.0, 0.0), 'q': (0.0, se2.STRAFE_SPEED, 0.0), 'e': (0.0, -se2.STRAFE_SPEED, 0.0),
        'wa': (se2.WALK_SPEED, 0.0, _YAW), 'wd': (se2.WALK_SPEED, 0.0, -_YAW), 'a': (0.0, 0.0, _YAW),
        'W': (se2.RUN_SPEED, 0.0, 0.0), '-': STOP}
PLAYS = {'start_stop': 'w3 -3 w3 -3', 'turns': 'w2 wa2 wd2 -3', 'strafes': 'q2 e2 -3', 'reverse': 'w2 s2 w2 -3',
         'sprint': 'w2 W3 w2 -3', 'spin': 'a3 -3', 'weave': 'w1 wa1 w1 wd1 w1 -3', 'about': 'a2 wa1 w2 -3',
         'walk': 'w10 -2', 'moves': 'w2 wa2 q2 s2 W2 -3', 'stutter': 'w3 -0.25 w2 -0.2 w3 -3',
         'run_leap': 'W2 h3 -3', 'walk_leap': 'w3 h3 -3'}


def played(keys):
    """(commands, seconds) of a player's keys, 'w3 -2': each a twist held its seconds, from 1 s; 'h' Space,
    a hop said at its moment over the twist held ('W2 h W3 -3': her run's leap)."""
    at, plan = 1.0, []
    for word in keys.split():
        key, held = word.rstrip('0123456789.'), word[len(word.rstrip('0123456789.')):]
        plan.append((at, {'say': 'hop'} if key == 'h' else {'twist': KEYS[key]}))
        at += float(held or 0.0)
    return plan, at + 1.0


#: Shoves: N for SHOVE_S, the way they push (deg about up from ahead, 90 to her left), standing or
#: walking, at one of MOMENTS - a quarter of her stride apart, the foot that swings decides; 80 N
#: held, 200 N held or fallen with her head slower than `scenarios.FALL_M_S` onto the floor.
FORCES_N, SHOVE_S, MOMENTS = (80, 200), 0.12, (5.0, 5.15, 5.30, 5.45)
SHOVES = {'shove_%d_%03d_%s_%d' % (n, way, gait, k): (n, way, gait, at)
          for n in FORCES_N for way in range(0, 360, 45) for gait in ('stand', 'walk')
          for k, at in enumerate(MOMENTS)}
