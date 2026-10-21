# Findings: stunts

The mocap takes as her scenarios (`tools/sim/stunts.py`, `scenarios.STUNTS`),
each judged against its take; the scenarios' bench: [scenarios](scenarios.md).

## The takes as scenarios

Each take under stunts/ beside the repo a scenario (`scenarios.STUNTS`), her
rows measured as the take is (`tools/sim/stunts.py`, MOVES: lengths in legs,
times in sqrt(leg/g), speeds in sqrt(g leg)) over the take's seconds from the
telling, judged within a sole's length (0.22 legs), her list (8.6 deg), her
arm's cone (20 deg) and one natural time (0.28 s); her head over the whole
run no lower than the human's, a fall's head under 1 m/s. The takes (a leg
0.89 m, a tick 0.30 s), kept in tests/takes/stunts.json:

| take | the move |
| --- | --- |
| Stand To Sit | drops 0.52 in 2.4 ticks, hips 0.48 back, trunk 43 deg forward, knees 95, feet down |
| Sitting on Bed | drops 0.46 in 3.2, hips 0.41 back, seated 9 deg off plumb, knees 99 |
| Sit To Stand | rises 0.52 in 2.8, trunk 53 forward |
| Lying Down from Sitting | hips 1.07 back in 6.9, lying 94 deg |
| Situp To Stand | rises 0.98 in 3.5 from the head at 0.05 |
| Crouching Down, Crouch Up | drops 0.66 in 3.7 (knees 140), rises 0.58 in 2.3 |
| Kneeling Down, Kneeling Up | drops 0.73 in 2.4 (knees 155), rises 0.66 in 3.9 |
| Standing Jump | hips up 0.55, feet 0.91, in place |
| Jumping Up, Jump Down, Jumping Down | 0.70 up or down, 1.4-2.6 ahead |
| Running Jump | feet 1.08 up, 3.96 ahead at 1.59 |
| Jogging, Running Forward | 0.74, 1.64; backwards 0.49, 0.93 |
| Moonwalk | 0.39 backwards, the feet 0.06 up |
| Hip Hop Dancing | bounces 0.38, the arms 80 deg, in place for 34 s |
| Standing Yawn | the arms 123 deg, 13 deg back |
| Turning in Bed | rolled 88 deg, lying |
| Backflip, Run To Flip | pitched 720, 367 deg; feet 1.87, 2.59 up; hips 0.65, 1.08 |
| Falling on Butt, Tripping Forward, Slipping Backwards | the head to 0.27, 0.04, 0.04 |
| Ascending Stairs, Walking Up The Stairs | 0.55 up over 0.57 ahead in 1.2-1.4 s |
| Edge Tripping Backwards | caught: ends level, the head 0.94 |

- Her first run: 2 of 31 met - the slick patch and the ice, crossed standing
  -; 6 refused for want of a text (backflip twice, kneel twice, yawn,
  turn_over); 23 short. Shoved back 200 N standing she fell onto her head,
  0.02 legs at 1.6 m/s, where the human sits down to 0.27; shoved ahead
  walking, head 0.11 at 1.1 m/s (the human's face to 0.04); backed off a
  step's edge, head 0.02 at 1.8 m/s where the human catches himself at 0.94.
  A crouch drops her 0.11 legs where 0.66, knees 54 deg where 140; a sit's
  hips 0.20 back where 0.48, its trunk 25 deg forward where 43; on the bed
  0.15 back where 0.41 and her head later to 0.69; the chair's hop down
  leaves her on it (ended 0.00 where -0.69), the hop up falls, head 4.3 m/s;
  a standing jump lifts her soles 0.10 where 0.91 and her hips 0.01 where
  0.55; W runs her at 0.34 sqrt(g leg) where a jog is 0.74 and a run 1.64, S
  backs her at 0.07 where 0.49; the dance bounces 0.05 where 0.38, arms 27
  deg where 80; rising from the chair she is 0.19 up where 0.52 by the
  take's end; lying from sitting her head went to 0.17 where 0.60 (the
  roll); the stairs fell; the floor rise 0.17 up where 0.98.
