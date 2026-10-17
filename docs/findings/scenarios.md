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

## One command

The page's keys send a twist and verbs, as an agent does (`humanoid_keys.twist`):
W A S D a way from the view, her heading turned to it at 1.5 rad/s a radian off,
to 0.7; walked at 0.69 m/s within a quarter turn of it, 1.4 on Shift within 0.2
rad; Q E a sidestep at 0.5. Every twist they send lies within the bench's plays
(`test_views_humanoid`).

- Its turnabout - in place 2 s, an arc 1 s, a walk 2 s, a stop (`play_about`) -
  held 5 of 13 wobbles, down 6.93-7.80 s: on her right sole, her centre of mass
  0.18 -> 0.73 m across from it, the left swing landing at 6.9 s. In place then
  a walk, in place then an arc, an arc then a walk, an arc of 3 s, each with its
  stop: 13 of 13.

## A swing's own clock

Each swing's share runs ahead of its rhythm's (`stride.hurried`, `modal.LEAD`) as
far as lands it before her capture point is a leg's reach (0.35 m) across past
the soles that stood, on the swing's own side; her rhythm's angle untouched. Her
plain walk the same bit for bit (her pose hashed each second for 20 s).

- The bench: 50 of 81 scenes, walking 80 N shoves 23 -> 26 of 32 (forward-aside
  1 -> 3 of 8), standing 32, replays 9 of 9; `play_about` 5 -> 8 of 13
  wobbles, the turns and the weave 13 of 13.
- Measured along her way too, the capture point runs past the standing sole every
  step of a walk: her plain walk hurried from 10 s on. With no sole standing it
  read 1000 m past them and a swing slammed down at 3 times its pace: a kept
  replay fell at 12.78 s.
- Either side or its own, `play_about` the same: its left lands at 6.75 s (6.9
  unhurried), the capture point still 5 cm past it.

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
- The stairs up: both feet stay at the first riser - the right's swing bumps it
  and lands at its foot, the left lands on its nosing and slides off - while her
  body runs on 0.28 m ahead of both; she pitches onto the stairs at 2.95 s.
  The step leg joining her stance steps her height reading 0.20 -> 0.61 units,
  asked 10 m/s^2 up in a pass; a sole lifting off while still stood reads her
  legs shortening 5 units a second. Each read otherwise - legs by their load,
  to her floor, her rate from her centre of mass - she still fell at 2.87-3.0 s
  and the slope earlier; each sole landed past the edge it met, the sill fell
  at 6.8 s.
- A swing past its rhythm's end and not down checking her going as far as it is
  late: up the ramp 0 -> 5 of 13 wobbles, but her stop at the tree, stepping in
  place, moved at one (12 of 13). Only past 70 ms of lateness - her plain walk's
  and turns' swings 7 ms late at most, stepping in place at the tree 68, over
  the ramp's crest 106 at the median -: the tree 13, the ramp 2: not kept.

## A fall met low

- A 200 N shove from standing, to her right: her head met the floor at 3.90 m/s.
  Said a crouch of 2 axial units 0.15 s after it, 0.32-0.67 m/s; at 0.25 s,
  1.16; at 0.35, 3.29; 3.5 units down, 2.29. Her capture point is then 0.17 m
  past her standing sole (80 N: at most 0.05) and 0.79 m by 0.7 s.
- Foreseen in her law - past the soles, cosh(omega t) to a landing, against a
  leg's reach - her knees leading, her trunk tucked: standing 200 N 14 -> 11 of
  32 met (soft falls turned to 1.05-1.5 m/s), walking 80 N 28 -> 26. Her step
  had caught one with her capture point 0.4 m from her centre of mass, past the
  0.35 m assumed: a foresight early enough to help a 200 N fall reads
  recoverable shoves as falls.

## Support, whatever bears her

- Past her modes' 1 onto a chair: 447 N on its seat at 1.83 s, her soles
  unload, her law reads no stance and lets her fall freely: down at 3.06 s.
- A segment the skin feels pushed up is support (`machine.borne`): in her
  support region, grounding her, her soles standing while it bears. Its sensed
  force fed to her whole-body step a pass late chattered: 695, 250, 88, 539, 131
  N on the seat 30 ms apart, her soles 0 to 286 N; unfed, seat and soles shared
  her weight, 100 N each.
- Her free soles were held in her pelvis's frame, as in flight: pitched forward
  0.5 rad on a seat they rose 9 cm. Something bearing her and no sole standing,
  they are held up from the floor in her way's frame.
- Her soles unloaded, the step took her base as free: her trunk pitched from
  0.95 to 1.42 units and she slid off the seat. Each bearing segment a contact of
  the step - three unknowns at the touch's centre, its point still, its push in
  its cone -: seated 1.95 to 8 s, the seat 190-220 N, each sole 30-50 N.
- On a mattress's soft edge the step planned 311 N where it bore 107, and she
  rolled back; a contact's push no more than 30 N over what it bears now: she
  reclines 0.44 rad on the edge 5.8 s and rises. Reclined 1 rad, her pelvis
  pitched with her trunk - one pitch mode for both -, her thighs rose with it and
  she slid off.
- Rising: the old thrust's 6000 N/m threw her soles 15 cm up; a lean of 1 s
  before the thrust overshot her trunk to 1.5 units and her capture point stepped
  her out of a squat; a lean of 0.5 s under a thrust blended over 1.5 s, she
  rises at 0.13 m/s.
- Not kept: standing soles pressed 20 N while something bears her, a contact's
  friction half a sole's - the chair fell rising at 11.9 s.
- The referee: 'sat' and 'lay' count what bears her off her soles, on her soles
  her soles' own loads; the bed side-on, its edge 0.1 m behind her (lengthwise,
  its headboard stood in her).
- The bench: scenes 62 of 87 (chair and bed up), the 80 N polar and the replays
  as they were; 200 N as it was, standing 14 of 32 met, walking 21. Chair 13 of
  13 wobbles; bed 7: at -5e-4 she sat over her right sole on the mattress's edge,
  2 and 219 N, 22 N on the edge, and a sole lifting stepped her. Seated deeper
  and slower, 3 of 13.
- Rising off the bed at -2e-4 her pelvis's geom was over the mattress's front
  corner - 2.1 cm from its edge where the chair has her 6.7 -, the contact's
  normal 23 deg ahead (0.92 up, 0.40 on). As the sit-up's blend ended the step
  planned 145 N up of the 358 asked, a contact let go in its last round (wrench.md);
  the corner took 241-309 N along its normal and threw her on at 4-6.7 m/s^2
  against -1 asked, a 0.27 m step; the other sole, the step's counterweight on
  one (W_COM 1000, W_SWING 30), rose to 0.34 m and never landed.
  `wrench.pushed` takes every prop's normal as the world's up.
- The step held to its band: the bed 10 -> 11 of 13 wobbles, its two falls
  lying back (5.15, 6.24 s) gone. She sits deeper on it, her centre of mass 14
  cm behind her soles' middle as the sit-up ends, and a rise waits while her
  capture point is off her soles and something bears her (161 passes at wobble
  0). The bench as it was but the bed, up at -2e-4, down at 2e-4: there the bed
  bears half her weight 0.51 s of the 1 asked. At -5e-4 her soles' loads swap
  side to side every 50 ms rising, the mattress's share flickering 0.17-1.0,
  and she goes back over her heels (12.75 s).
- Not kept: her height held while something bears her - raised to as she is by
  the share off her soles -: the bed 9 -> 6 of 13, falling as she lay back where
  the soft mattress had her deeper than asked.

## Jumping

- A hop with no twist throws her straight up (`flight.shape_hop`'s forward speed
  is a twist's): from a chair's seat she lands on it again; up onto one she falls
  at 2.5 s.

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
- CI installed MuJoCo 3.16.0 against 3.14.0 here: the hole fell at two held
  wobbles and the tree ended at 0.18-0.19 m/s (0.01-0.05 here). The engine is
  pinned to the one measured on; the hole, 9 of 13 wobbles, left the held set.

## The floor ahead

- Up the 10 deg ramp each swing came down early, 0.34, 0.36, 0.19 s where 0.40,
  0.05 to 0.09 m behind her centre of mass (0.10 ahead on a level floor), her
  speed 0.39 -> 1.74 m/s, down at 5.1 s: her swings and her stance read one
  floor height for both soles. Read as a plane through the floor's height at its
  point, pitched as her standing soles stand on it (`modal.under`), they land
  0.04 to 0.28 m ahead and she climbs 0.33 m; her level walk the same to the
  digit, the bench unmoved.
- The ramp's top and the stairs' landing were grown 6 m long past their compiled
  bounds (`floor._sized`); MuJoCo culled every contact 0.85 m from the top's
  middle and she walked off the ramp through it.
- At the top a swing ended in the air on the ramp's plane run on, 7 cm over the
  top, and stood at 0 N by its height while the other was let go: a sole stands
  by its height alone only over the floor felt. Down at 6.5-9.0 s where 5.1.
- Not kept: a sole her law has flat turning its floor at 2 rad/s while one cell
  alone presses - level ground has such windows at every toe-off, up to 0.153 s,
  and her walk went 385 J/m with her knee 20 deg behind plumb; the floor bounded
  by a sole on one cell, at least its pitch toes down on its heel and toes up on
  its ball - the ramp 3 of 3 and 9 of 13, but at the hole's lip a sole on one
  cell is on no plane and the hole fell 3 of 3; the same with the bearing point
  within 20 mm of the sole's end (the ramp's 12-16 mm, the lip's 29 and more) -
  ramp 1 of 3, hole 1 of 3; her height lowered with a swing past its rhythm's
  end - down the ramp she fell at 5.0 s where 3.5, and a standing 80 N shove was
  lost.
- Left: on the ramp her speed 0.6-1.3 m/s where 0.69 is asked; past its top the
  plane run on over the level; down the ramp a swing hangs 6 cm over the floor,
  down the stairs 27 cm, and reaches on only 11 cm (`stride.REACH_S`).
