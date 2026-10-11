# Findings: turns on the one law

The law's heading turned, walking and standing (`machine/going.py`,
`hold.staying`; test_room.py); the room's words that use it: [room](room.md).

## Turned at a rate

2026-10-11, the heading turned at a rate from 3 s into the walk's row or the
stand (before: 0.1 rad/s walking fell at 11 s, no turn standing):

| Turned | Outcome |
| --- | --- |
| walking 0.3 rad/s | 20 s on, 292 deg, landing knee 11.9 deg (straight 10.7), 0.73 m/s (0.85) |
| walking 0.5 rad/s | fell at 8 s, 147 deg |
| standing 0.5 rad/s | 258 deg in 9 s, 16 steps round, within 0.1 m |
| standing 1.0 rad/s | fell at 4.5 s |

- A standing leg keeps the yaw it landed with; the heading is where the
  next lands and what the pelvis turns to. Standing, a foot TWIST_RAD (20
  deg) off it steps round (`hold.staying`), under the hips' yaw stops of 25.
- Two frame bugs: the pace was two positions projected on two headings
  (3.6 m out it read 0.1 m/s at 0.65, the steps shrank, a fall at 8.5 s);
  `runner.pitched` took the lean along the world's z (a foot facing 90 deg
  read its roll as its pitch, a fall at 11 s). Both along the heading now.
- The straight walk is untouched (gait 99 of 99).
