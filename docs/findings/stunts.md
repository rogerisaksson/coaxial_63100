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
- A posture's take told as its own rows (`stunts.modes`, `stunts.wire`; the
  take read into her modes as `modal.reading` reads hers, off the take's
  standing frames, its centre of mass by her figure's segment shares on its
  bones; `{'take': key}` in STUNTS). Streamed whole, the human's stance
  felled her: his feet widened and shifted through a crouch (head 0.02 legs),
  his pelvis's 20 deg list or his heels' rise in it (0.04, 0.24), his arms'
  swing (0.05, hands' jerk 29 000 m/s^3). The axial, sagittal and pitch modes
  alone: the crouch 0.62 legs down where 0.66, knees 128 where 140, the kneel
  0.69 where 0.73, knees 144 where 155, both met (far 0.0; the sagittal from
  his hips instead of his centre of mass put her capture point at her heels,
  hands 8-12 000); the sit's hips 0.06-0.56 back where 0.48 run to run, its
  trunk 45 deg forward where 43. 4 of 31 met, 235 widths off in all. Her
  hands' jerk, the spasm, where her walk's is 1 400-1 800: the sit 20 800,
  rising from it 19 800, the dance 20 200, turning in bed 16 000, the bed's
  edge 12 700, rising from a crouch or a kneel 8 800-8 900, the runs
  770-1 400. The dance's bounce of 0.38 legs streamed as axial felled her
  (0.98 down, 1.9 m/s); told to lean back 13 deg standing she leans 3.7.
- The trunk's pitch zeroed at her stand (`fr[PITCH0]`) instead of the take's
  standing frame: the rig stands with its neck 3-6 deg behind its hips, and
  zeroed there she leant 5 deg past the human and a crouch toppled her
  (forward 106-130 deg at every wobble). Off her stand the crouch stands 0.58
  legs down, knees 126, forward 45.6 where the take's 44.3, her hands' jerk
  2 300 (5 400); the yawn's lean back 13.6 where 13.5. Not kept: the step's
  upper rows at W_TURN - the sit's hands 20 800 to 4 800-6 700, the crouch's
  5 400 to 900, but the walk 2.75 off a woman's (2.55), the pelvis rolling 3
  deg where 5; the arms' rows alone at W_TURN - the walk as it was, but the
  crouch's spine folded for the lever instead and she fell. 3 of 31 met, 255
  widths off: the crouch within every width but its descent's time, 0.53 s
  from a tenth to nine tenths of the way where the human's 1.03 - his hips
  curve back and then down, hers drop straight.
