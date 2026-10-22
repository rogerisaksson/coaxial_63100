# Findings: scenarios

Her scenarios as a player commands them (`tools/sim/scenarios.py`): each scene
placed, her worker's session driven, the world's truth the referee. What bears
her off her soles: [support](support.md). The board's findings:
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
- Trips on a flat floor: turning, toes scuffed it at 0.6-6.6 mm where their
  corners read 17-35 mm; the stumble those scuffs fired carried the turns' play,
  a hop after a stop and the sill through their stops, by chance.
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
  held 5 of 13 wobbles. In place then a walk, in place then an arc, an arc
  then a walk, an arc of 3 s, each with its stop: 13 of 13.

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

## Jumping

- A hop with no twist throws her straight up (`flight.shape_hop`'s forward speed
  is a twist's): from a chair's seat she lands on it again; up onto one she falls
  at 2.5 s.
- `hop_down_chair` and `hop_up_chair` reach no hop: their phases are not the
  `load`, `throw`, `land` `flight.shape_hop` keys on, and name no targets; on the
  loop she stands on the seat. Not kept: a hop said on the seat (down at 3.06 s),
  with a 0.3 m/s twist (3.50); walked off it, 0.15 m/s for 0.3 s, 6 of 13 - the
  rest stop stepping in place on the seat, creep 0.07 m/s to its edge and walk off
  it at 5.1 s.
- On her loop alone, `flight.shape_hop` left out, `hop` crouches 12 cm, throws
  at 1.24 m/s up, flies 0.15 s and lands standing at 967 N (shaped, 18.7 cm
  over her stand where 2.2). The chair's hop as a loop text - a lean onto her
  toes, a throw with her sagittal shift stiff ahead, a soft landing - flies
  0.2 s and lands on the seat's edge: off a seat 0.22 m to its edge and 0.45 m
  down a throw needs 0.9-1.0 m/s ahead, her ZMP in her soles gives 4.9 m/s^2
  for its 0.15 s, 0.74 m/s. Not kept.

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
- Down the stairs a swing goes 0.65 m, past the first tread, stands at 0 N by
  its height over the floor her standing sole is on, and the other is let go:
  down at 4 g. Down the ramp her soles read its pitch 0.04-0.14 rad where 0.17
  (`se2.floored`: a sole's tilt while its heel and its ball press alike), a
  swing hangs over the floor ahead, and on one sole her centre of mass goes 0.44
  m past it, 0.69 -> 1.6 m/s.
- Not kept, each measured: the world's floor under each sole, a ray (stairs 3.10
  -> 2.93 s, down the ramp 3.82 -> 3.19, down the stairs 3.38); with it each
  swing ending on its landing's floor over the highest between (stairs 3.32,
  down the stairs 3.18, down the ramp 4.09, her walk unmoved); the stairs' gaits
  told (2.95, 3.5); a sole arriving stood by its load alone (down the ramp 4.32,
  across it 5.69, down the stairs 3.20); a 20 cm reach (as before); the floor's
  pitch from her two bearing soles' heights apart (up the ramp 9.17 -> 4.46, the
  hole fell); the ramp's pitch told under each sole (down it to its foot, 0.449
  of 0.45 m, and down there at 4.49 s; up it at the crest, 9.25). Told the ramp
  and her cruise no longer drawn on by her going (`stride.cruises`' AHEAD below
  it) she climbs it at 12 of 13 wobbles - drawn on, her going 1.0-1.35 m/s up it
  lifted her cruise to 0.99 where 0.63 was asked, she ran 1.37 and flew off its
  top -, but snow went 13 -> 9: there her soles slide on ahead, 1.0-1.29 m/s, and
  her centre of mass must follow them - held to what was asked it fell back at
  1.5 m/s. The bound's most held to what was asked of late, the same; neither
  surge is a slide - her estimator's slip 0.009 m/s at snow's, her standing soles
  0.003 at p90 -: both are her swing's, 3.3 m/s up the ramp. A known
  floor alone does not climb: her landings fall on nosings and edges, and her
  capture point runs on.
- The flight told her, her landings onto its treads: [stairs](stairs.md).

## Blind on a flight

Stairs of 0.15 x 0.30 m walked into at 0.69 m/s, nothing told, traced 20 ms a
pass beside the world's floor under each sole (`tools/sim/trace.py`).

- Down: the swing passing the believed floor's level in the air, 15 cm over the
  tread, stands by its height alone, the other is let go and she drops 4 g onto
  it (3.10 s). Held to a load or to lying still there instead, she reached on
  while her centre of mass ran past the standing sole, 0.8 -> 1.5 m/s in 0.3 s,
  and the hole's 10 cm, which that commitment and drop had held, fell: not kept.
- Up: the toes pushed back by the riser's face loaded the sole 85-285 N and it
  stood. An insole reads what presses up through it (`edge.read`'s load as
  the normal share): right, and not kept - the slick patch's 5 mm lip's push
  had stood a sole there and it fell at 3.9-4.4 s, and the snow's far edge
  fell at one wobble of three, a hop off its 4 cm at 1.4 m/s. Pushed back,
  the swing held before its descent with her rhythm and her rows waiting, its
  clearance climbing till the toes were over the edge,
  its heel landing 4 cm past where they met it: the first riser climbed (up 3.0
  -> 4.7 s), but the same hold fired at the tree's and the wall's foot and at
  the snow's 4 cm lip, and with it a give-up at 0.22-0.30 m of raise and a veto
  on a touch above her soles, the wall still fell at 6.3-9.0 s: not kept.
- A flight read from its landings - the floor's rise a metre along from one
  landing to the next, the plane ahead the steeper of it and the soles' pitch -:
  down the ramp -0.23 where -0.18, with a late swing lowering her as far as its
  leg needs she went 3.1 m to its foot (down at 7.63 s where 3.82), but on the
  snow the plane read the standing sole behind 3 cm over its floor and it was
  let go, and a spin in place lowered her and fell: not kept. The floor under a
  standing sole as where it landed, the plane for the swing alone: stairs 2.96,
  across the ramp 4.32: not kept.
- Left: on a tread or a ramp her centre of mass runs on past the standing sole
  while the next swing is late, and the swing goes where her capture point has
  gone - two treads on; nothing told, her first step down a flight is a drop.

## Her errands

- Told to stop, `law.held_back` held the stand for ever: the brake's sway
  kept a swing commanded on two standing soles, her doing flipped between
  `brake` and `#201` 3 781 times in 4 s, `sit_chair` held with it. Holding on
  the sole off the floor, on her rhythm's gain or a control step ahead of now
  each moved her (`walk_turn_right_at_0.69` fell, ice 1.76 m); the hold as it
  was, let go after two of her rhythm's swings (`latent.TURNS`): she stands in
  0.3 s, sits at 3.2 s, rises at 2.7 s, the 12 replays whole. Let go only
  with her capture point inside her soles, hop-after-a-stop fell.
- Her doing read at the world's passes between her law's: a stand her law held
  came and went at every pass, 596 flips in a stop, the page's arrows with it.
  Read at her law's last pass (`se2.doing`), 2.
- A hold let go kept its start: a hop said 2.1 s after a stop, her still
  stepping, was let go at once onto one sole (275/0 N) and fell landing at 3 of
  9 wobbles; each hold counted from its own, 1, after a 38 N push.
- The ice scene's `past 1.8` was that shuffle: told to stop at 1.49 m she
  crept on at 0.045 m/s to 1.88 m; let go she stands at 9.0 s at 1.76 m. Its
  goal is 1.7 m.
- A walk asked of a stand swaying over 0.04 m/s got a stand first and the gait
  1.35 s later (`worker.heard_say`'s `off_c`); at rest she sways 0.05, after
  a stop 0.05-0.16, so every W waited 1.35 s. The sway term gone, init_walk
  comes with the ask, her posture and loads still gating it.
- W held through a scene change: what she felt of the chair (`g['_felt']`,
  3 s) outlived the world laid anew and deflected the walk into a sidestep at
  0.29 m/s. Cleared on the reset, init_walk within 10 ms, 0.49 m/s at 2 s;
  the page tells her the keys' twist again as it sees the new scene.
- On a scene laid anew (`gym._run_scenario`), the pad's cached `setpoint` held
  the prior twist and swallowed the next W asked at once; cleared on scene
  reset, a walk asked at once begins in 1 ms. `arrows.describe` draws the stop
  mark only on `STOP_DOING` and the going arrow only on a commanded twist.
- Told to stand from her walk she goes on 0.70 m and is still 2.7 s later, at
  0.81 m/s felt whatever speed was asked (0.25-0.69): she has the one walk
  (`se2.STOP_M`). A left sidestep moves her at her heading plus a right angle,
  0.12-0.35 m/s, 0.00-0.15 m on after a stop; her back-walk 0.09 rad off dead
  astern, 0.05 m on. The errand from the viewport (`tour.step_errand`: stood
  where her stopping point enters the box before the dock, the look-ahead on
  the prop's normal, a turn, a sidestep, backing onto the edge) docks her at
  the chair with her heels 0.05 m from its edge, seated at 18.8 s, and reclines
  her into bed_rest at 29 s, surety 0.97. Before it: a stop radius of 0.6 m
  from 0.69 m/s overshot the staging point by 0.95 m, and a cone of 0.3 rad
  flipped her between walking and turning in place every half second, 16 s
  round one spot, surety 0.36.
- Asked flat on the bed from recline (pitch -3.0, sagittal -2.5) she lies at
  -84 deg by 6 s and rolls: yaw 3.7 to 157 deg, the bed's share 1.10 to 0.20,
  surety 0.51, the rise stuck in sit_up; sagittal -3.5 with her arms back
  falls off the bed at 6.6 s. Lying flat waits on her trunk's contacts as her
  support (item 1); mocap: lying down from sitting takes 5.7 s, the hips
  0.45 m back, the feet from the floor to the mattress.
- Seated on the bed's edge from the errand (the page's recording, 17:56), her
  soles reach the floor and her support flickers between the mattress and a
  sole - the bed's share 0.96, 0.28, 0.99, 0.29, the loads 0 and 98 N - and the
  whole-body step takes the trunk's moment from her right arm: wound 421 deg
  on the page, -53 to -362 in 6 s here, the chair the same (-186, -310). Her
  shoulders have no range; given a human's (60 behind, 180 overhead) the arm
  pumped within it (+74 to +157 in 0.75 s) while the flicker lasted, and the
  stop met her back-walk's arm swing (65 deg back) and the standing turn at
  the dock: she fell onto the bed, where without it she lay. Not kept. The
  stagger before it: 6 s turning in place for a
  half turn at her YAW_RATE (0.5 rad/s made of 0.7 asked), roll 9.5 deg as it
  begins, and the sidestep at 0.15 m/s.
- The door errand on CI's Linux: aimed at the errand's end a stage past the
  door, a room's height ahead at the least, she closed on the doorway's
  middle by a room's height's worth a room's height walked, met the post at
  x 0.5 for the middle's 0.75 and pushed it 8 s on Windows, 30 s on Linux
  (a probe on the runner, both Pythons). Docked on the line and aimed at
  the middle her stage (1.2 m) ahead at the least she passes at x 0.72,
  through at 7.5 s, over at 8.8 s on both; her stopping distance ahead she
  swung to x 1.18 at the other post. Left red on Linux: the chair's sit by
  its text bears 0.34 of her at 7 s on 3.12 where Windows' 1.02 (the
  perch), the viewport chair misses the seat (3.12) or falls at the dock
  (3.14), the bed's lie sits down before the bed and falls at 22 s where
  Windows finds it at 21.5 s and sits again at 24: the dock's backing stop
  and the seat's perch, item 1's.
- A human's sit (`tools/sim/seats.py`, the takes Stand To Sit and Sitting on
  Bed): 0.6 s down for 0.45-0.52 legs, the hips 0.31-0.47 legs behind the
  ankles, knees 94-98 deg, the trunk 45 deg forward on the way onto a chair
  and 14 back onto a bed's edge, seated within 7 deg of upright, the feet
  never off the floor, the upper arms within 40 deg through it and 10 seated.
  Hers on the page: feet up 0.26 legs, arms 170 deg, hips 0.04 legs back,
  trunk -31 (the lie's recline). The feet: `impedance.ask` lifts her soles by
  what a prop holds her above her legs' asked length. Left down (the spring
  relaxed by it alone) through the viewport's bed errand the right shoulder
  stayed within -53 to +52 deg where -383 to +99, seated 10 deg, soles 98 and
  90 N, the bed 0.34-0.41, surety 0.98; the chair -56 to +131, seated 15; her
  sits then against the band: feet rise 0.00, arms seated 0.3-0.7 deg, descent
  0.54-0.59 s, off it drop 0.38-0.40 legs and knees 101-102 deg (a seat 0.45 m
  high on legs of 0.77 m), hips 0.13-0.21 legs back, the arms' swing down
  54-56 deg. But a seat then bears her with her soles (chair 0.53, bed 0.39),
  not off them as the scenarios judge a sit, a chair's sole loads cross her
  touch 32 times in 8 s, and rising from the bed she fell at swing_out. Not
  kept: the sit is to be laid from the takes whole - the hips back over the
  seat, the weight onto it, the feet down, the rise.

## The takes as scenarios

Each take a scenario judged against its take: [stunts](stunts.md).
