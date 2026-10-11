# Findings: turns on the one law

The law's heading turned, walking and standing (`machine/going.py`,
`hold.staying`; test_room.py); the room's words that use it: [room](room.md).

## Turned at a rate

The heading turned at a rate from 3 s into the walk's row or the
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

## A walk begun after a turn

A walk asked the pass she faces her way after a turn on the spot, her feet up
to 0.12 m apart along it (24 turns of 0-120 deg): 18 of 24 walked on, the
rest went backward and were down in 2-3 s.

| Cause | Now | Starts |
| --- | --- | --- |
| the rear foot swings first and lands by the step asked, 0.05-0.11 m past what her capture point reaches: she cannot pass over it | a walk's landing no further than she passes over it (`hold.within`): the pendulum's, its centre of pressure going from the standing foot to the landing one over the time both bear, her capture point 0.03 m behind the landing foot's point at most | 22 of 24, none down |
| on both feet with a speed asked the pelvis rose to the walk's height, a heel rose to reach it and its ball pushed her back: 0.9 m/s backward in 0.8 s, both feet down | on both feet she is no higher than both reach flat while she is not going - by her speed, not the speed asked | 24 of 24 |

- Walked, halted, turned, walked again, 16 turns of 20-90 deg: 15 walk on,
  one slow; the halt carries her 0.49 m.
- The rows as before: the walk 0.85 m/s at 498-501 J/m, the stroll 0.53 at
  662-701; test_gynoid_going.py 64 of 64.
- A landing is told from her capture point as she goes now: told at its
  furthest, the standing sole's rear edge its centre of pressure, 21 of 24;
  at its measured rate 22, two down forward; held a fixed 28 mm past it the
  stroll stalled at 0.11 m/s; at a margin of 0.05 m 10 of 12.
- Tried on which foot leaves and when, and out: a foot waiting till her
  capture point is over the other, 5 of 12 and 4 with the flat cap - it kept
  the turn's own steps down; the foremost foot she passes over staying, 7 of
  12; the foot landed last staying only if she passes over it, 14 of 24, and
  18 with the flat cap.
- Open: stopped by a thing while a speed is asked she stands on both feet,
  her capture point behind them, and goes over backward in 1.2 s.
