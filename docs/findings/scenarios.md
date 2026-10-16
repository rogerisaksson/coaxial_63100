# Findings: scenarios

Her scenarios as a player commands them (`tools/sim/scenarios.py`): each scene placed,
her worker's session driven, the world's truth the referee. The board's findings:
[FINDINGS](../FINDINGS.md).

## The bench before her laws for them

Stairs of 0.15 x 0.30 m, ramps of 10 deg; walked at 0.69 m/s from 1 s, then stopped;
the stunts as their entries said. 6 of 19 met.

| scenario | stood | fell at s | went m | rose m |
| --- | --- | --- | --- | --- |
| wall, tree | yes | - | -0.02, 0.20 | 0 |
| sill 35 mm | yes | - | 3.29 | 0 |
| rug | no | 6.03 | 2.82 | -0.64 |
| hole 10 cm | no | 3.67 | 1.63 | -0.73 |
| slick patch, snow, asphalt | yes | - | 2.01, 2.80, 2.50 | 0 |
| ice | no | 11.24 | 1.42 | -0.74 |
| stairs up | no | 3.00 | 0.57 | -0.45 |
| stairs down | no | 6.87 | 2.75 | -0.77 |
| ramp up | no | 11.10 | 1.64 | -0.24 |
| ramp down | yes | - | 3.38 | -0.10 |
| across a ramp | no | 4.90 | 0.81 | -0.83 |
| chair, bed | yes | - | 0.04 | 0 |
| mattress | yes | - | 0.04 | 0 |
| hop down, up | down: yes; up: no, 2.19 | | 0.01, -0.78 | 0, -0.78 |

- `sit_chair`, `lie_bed` and `lie_floor` crouch her 8 cm and hold her there: her
  pelvis 0.898 -> 0.817 m, a seat at 0.45 m never touched; on the mattress she
  stands. The tilt was all their checks read.
- `hop_down_chair` from the seat leaves her standing on it; `hop_up_chair` falls
  at 2.19 s with the chair no longer moved under her mid-hop.

## A touch, and the shove as one

- A shove is a hand on her: the edge feels what the world applies to her bodies
  as touch. A touch above her soles not within 50 deg of her going stops it -
  she brakes, steps, stands -, one along it she walks out; she goes on as told
  1.5 s after it is gone, and her going toward what she felt is cut for 3 s.
- 80 N for 0.12 s, eight ways, four moments a quarter stride apart: standing
  30 -> 32 of 32 held, walking at 0.69 m/s 19 -> 24 of 32 (aside 1 -> 5 of 8).
  Braked on every touch, a shove from behind fell her as the brake's stand came
  on a timer with her capture point 0.34 m ahead.
- On the open floor, the page's room taken away (its far wall and door felled her
  at 3.2 m on snow): 14 of 26 met, ice among them, and all seven of a player's
  sequences. Laid turned 2e-4 rad either way, 12 hold every time
  (`test_scenarios`); the wall's stop and a spin in place do not.

## A stop

The whole bench, each scenario laid 0 and 2e-4 rad either way, the 80 N polar,
the kept replays at theirs: 40 of 78, standing 31 and walking 24 of 32, replays
5 of 9.

- A swing begun is finished (`stride.beat`): 41 of 78, the spin in place standing
  where it fell at 7.94 s; nothing else moved, her walk the same to the digit.
- The kept stop after a walk: her left toes met the room's wall 31 mm up as she
  stopped at it, read as a trip; the stumble - 1.15 Hz, lift 2.6, drive 0.32 and
  more for 0.95 s - stepped her high against it, down at 8.79 s. At the bench's
  wall five trips came in its brake and its stand, the last at 1.58 m/s.
- Trips on a flat floor: turning, toes scuffed it at 0.6-6.6 mm, touched along
  it at 1.00-1.02 of their load (her soles' friction, 1.0) where their corners
  read 17-35 mm; edges met read 12.5-54 mm and 1.35-58. The stumble those scuffs
  fired carried the turns' play, a hop after a stop and the sill through their
  stops; without it all three fell. The stumble and the parry were her only
  stepping out of a rhythm, fired by chance.
- The brake said 0.85 Hz and lift 0.7 where her walk is 0.94 and 1.05: kept as
  it was, the stops after a walk and a turn stood at every wobble (replays 9 of
  9), walking shoves 27.
- Over a swing her walk holds her capture point's lead on her centre of mass,
  0.19-0.20 m start to end at 0.69 m/s, where a linear inverted pendulum on her
  sole foresees 0.35-0.36: her rhythm hurried by it acted on her plain walk in 4
  to 7 % of its passes and moved it, 2.57 -> 3.58 off a woman's.

The laws now: her capture point past her soles - flat, past her ZMP's margin -
steps her (`stride.beat`'s need), the stumble and the parry gone; a row that halts
her rhythm waits while she steps (`law.held_back`); an edge met raises her swings
by its height, read at her toes' tip (`touch`); her heading goes with her pelvis
only in a turn asked. 45 of 78 - fifteen scenarios at every wobble, the hole
among them for the first time -, standing 32 and walking 23 of 32, replays 9 of
9; her walk 51.0 s, 0.69 m/s, 412 J/m, 2.61 off a woman's.

- Walking shoves by way: forward-aside 1 of 8 where 5, aside and back-aside 8
  of 8 where 5. Her heading after her going too, walking 18 and the hole, the
  wall and the tree lost (the rug met); after her pelvis, the sill fell: a yaw of
  0.22 rad off the floor turned her way and she crabbed into APART_M.
- Stepping through a hop's load she left the floor rolling and landed at 60 deg:
  the need fires on a drift of 0.19 m/s after a stop; held back, the hop waits.

## The terrain's scenes

- The base plane, the hole's bottom 10 cm down, lay over every descent: the stairs
  down and the slope down were a 10 cm dip, the slope down's goal (0.45 m lower)
  out of reach; the stairs' landing (1.5 m) and the slope's top (1.2 m) ended in
  a drop before a 7-10 s walk did. The bench's floors now go on 6 m (`UPPER_M`),
  a descent's on the base plane laid at its foot; the page's room keeps a
  house's landing.
- On them, at every wobble: stairs up down at 2.95-2.98 s, down at 3.24-3.45,
  the slope up at 6.36-7.19, down at 3.99, across it at 3.30-3.43, the rug at
  5.91-5.96.

## The floor under a sole

- Her law read a pitched sole as on an edge, so a sole flat on a ramp stood on its
  heel's line. Each sole's floor is now its own pitch while its insole's heel cell
  and ball cell both press, as far as alike; under half `EDGE_RAD` it is level.
  The rug held at 13 of 13 wobbles where 0; level floors unchanged to the digit.
- One centre of pressure per insole took a hole's edge under a sole's middle for
  a floor 10 deg up; two cells tell them apart. Read whole, a level floor was
  0.003-0.006 rad heel up (her soft soles' balls pressed in deeper) and her
  pelvis rolled 4.96 deg where 5.05.
- `f * (R.T @ v)` for `(f * R.T) @ v` in the edge moved her roll by 0.015 deg
  and flipped the turns' play at one held wobble: a scenario's three wobbles
  are a margin's sample; thirteen (-6e-4 to 6e-4 rad) read the hole 9 of 13.
- On the 10 deg ramp each sole's floor reads -0.14 to -0.17 rad (-0.175); she
  speeds up it, 0.28 -> 1.07 m/s, her height measured from one floor for both
  soles, the lower sole's heel lifted further each step.
