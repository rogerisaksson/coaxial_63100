# Findings: scenarios

Its scenarios as a player commands them (`tools/sim/scenarios.py`): each scene
placed, its worker's session driven, the world's truth the referee. What bears
it off its soles: [support](support.md). The board's findings:
[FINDINGS](../FINDINGS.md).

## The bench before its laws for them

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

- `sit_chair`, `lie_bed` and `lie_floor` crouch its 8 cm and hold it there: its
  pelvis 0.898 -> 0.817 m, a seat at 0.45 m never touched; on the mattress it
  stands. The tilt was all their checks read.
- `hop_down_chair` from the seat leaves it standing on it; `hop_up_chair` falls
  at 2.19 s with the chair no longer moved under its mid-hop.

## A touch, and the shove as one

- A shove is a hand on it: the edge feels what the world applies to its bodies
  as touch. A touch above its soles not within 50 deg of its going stops it -
  it brakes, steps, stands -, one along it it walks out; it goes on as told
  1.5 s after it is gone, and its going toward what it felt is cut for 3 s.
- 80 N for 0.12 s, eight ways, four moments a quarter stride apart: standing
  30 -> 32 of 32 held, walking at 0.69 m/s 19 -> 24 of 32 (aside 1 -> 5 of 8).
  Braked on every touch, a shove from behind fell it as the brake's stand came
  on a timer with its capture point 0.34 m ahead.
- On the open floor, the page's room taken away (its far wall and door felled it
  at 3.2 m on snow): 14 of 26 met, ice among them, and all seven of a player's
  sequences. Laid turned 2e-4 rad either way, 12 hold every time
  (`test_scenarios`); the wall's stop and a spin in place do not.

## A stop

The whole bench, each scenario laid 0 and 2e-4 rad either way, the 80 N polar,
the kept replays at theirs: 40 of 78, standing 31 and walking 24 of 32, replays
5 of 9.

- A swing begun is finished (`stride.beat`): 41 of 78, the spin in place standing
  where it fell at 7.94 s; nothing else moved, its walk the same to the digit.
- Trips on a flat floor: turning, toes scuffed it at 0.6-6.6 mm where their
  corners read 17-35 mm; the stumble those scuffs fired carried the turns' play,
  a hop after a stop and the sill through their stops, by chance.
- The brake said 0.85 Hz and lift 0.7 where its walk is 0.94 and 1.05: kept as
  it was, the stops after a walk and a turn stood at every wobble (replays 9 of
  9), walking shoves 27.
- Over a swing its walk holds its capture point's lead on its centre of mass,
  0.19-0.20 m start to end at 0.69 m/s, where a linear inverted pendulum on its
  sole foresees 0.35-0.36: its rhythm hurried by it acted on its plain walk in 4
  to 7 % of its passes and moved it, 2.57 -> 3.58 off a woman's.

The laws now: its capture point past its soles - flat, past its ZMP's margin -
steps it (`stride.beat`'s need), the stumble and the parry gone; a row that halts
its rhythm waits while it steps (`law.held_back`); an edge met raises its swings
by its height, read at its toes' tip (`touch`); its heading goes with its pelvis
only in a turn asked. 45 of 78 - fifteen scenarios at every wobble, the hole
among them for the first time -, standing 32 and walking 23 of 32, replays 9 of
9; its walk 51.0 s, 0.69 m/s, 412 J/m, 2.61 off a woman's.

- Walking shoves by way: forward-aside 1 of 8 where 5, aside and back-aside 8
  of 8 where 5. Its heading after its going too, walking 18 and the hole, the
  wall and the tree lost (the rug met); after its pelvis, the sill fell: a yaw of
  0.22 rad off the floor turned its way and it crabbed into APART_M.
- Stepping through a hop's load it left the floor rolling and landed at 60 deg:
  the need fires on a drift of 0.19 m/s after a stop; held back, the hop waits.

## One command

The page's keys send a twist and verbs, as an agent does (`humanoid_keys.twist`):
in its own frame, W S on at 0.69 m/s (1.4 on Shift, straight) and back at 0.35,
A D a turn at 0.7 rad/s, in place or on an arc with W S; Q E a sidestep at 0.5,
alone. Every twist they send lies within the bench's plays
(`test_views_humanoid`).

- Its turnabout - in place 2 s, an arc 1 s, a walk 2 s, a stop (`play_about`) -
  held 5 of 13 wobbles. In place then a walk, in place then an arc, an arc
  then a walk, an arc of 3 s, each with its stop: 13 of 13.

## A swing's own clock

Each swing's share runs ahead of its rhythm's (`stride.hurried`, `modal.LEAD`) as
far as lands it before its capture point is a leg's reach (0.35 m) across past
the soles that stood, on the swing's own side; its rhythm's angle untouched. Its
plain walk the same bit for bit (its pose hashed each second for 20 s).

- The bench: 50 of 81 scenes, walking 80 N shoves 23 -> 26 of 32 (forward-aside
  1 -> 3 of 8), standing 32, replays 9 of 9; `play_about` 5 -> 8 of 13
  wobbles, the turns and the weave 13 of 13.
- Measured along its way too, the capture point runs past the standing sole every
  step of a walk: its plain walk hurried from 10 s on. With no sole standing it
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
  and lands at its foot, the left lands on its nosing and slides off - while its
  body runs on 0.28 m ahead of both; it pitches onto the stairs at 2.95 s.
  The step leg joining its stance steps its height reading 0.20 -> 0.61 units,
  asked 10 m/s^2 up in a pass; a sole lifting off while still stood reads its
  legs shortening 5 units a second. Each read otherwise - legs by their load,
  to its floor, its rate from its centre of mass - it still fell at 2.87-3.0 s
  and the slope earlier; each sole landed past the edge it met, the sill fell
  at 6.8 s.
- A swing past its rhythm's end and not down checking its going as far as it is
  late: up the ramp 0 -> 5 of 13 wobbles, but its stop at the tree, stepping in
  place, moved at one (12 of 13). Only past 70 ms of lateness - its plain walk's
  and turns' swings 7 ms late at most, stepping in place at the tree 68, over
  the ramp's crest 106 at the median -: the tree 13, the ramp 2: not kept.

## A fall met low

- A 200 N shove from standing, to its right: its head met the floor at 3.90 m/s.
  Said a crouch of 2 axial units 0.15 s after it, 0.32-0.67 m/s; at 0.25 s,
  1.16; at 0.35, 3.29; 3.5 units down, 2.29. Its capture point is then 0.17 m
  past its standing sole (80 N: at most 0.05) and 0.79 m by 0.7 s.
- Foreseen in its law - past the soles, cosh(omega t) to a landing, against a
  leg's reach - its knees leading, its trunk tucked: standing 200 N 14 -> 11 of
  32 met (soft falls turned to 1.05-1.5 m/s), walking 80 N 28 -> 26. Its step
  had caught one with its capture point 0.4 m from its centre of mass, past the
  0.35 m assumed: a foresight early enough to help a 200 N fall reads
  recoverable shoves as falls.

## Jumping

- A hop with no twist throws it straight up (`flight.shape_hop`'s forward speed
  is a twist's): from a chair's seat it lands on it again; up onto one it falls
  at 2.5 s.
- `hop_down_chair` and `hop_up_chair` reach no hop: their phases are not the
  `load`, `throw`, `land` `flight.shape_hop` keys on, and name no targets; on the
  loop it stands on the seat. Not kept: a hop said on the seat (down at 3.06 s),
  with a 0.3 m/s twist (3.50); walked off it, 0.15 m/s for 0.3 s, 6 of 13 - the
  rest stop stepping in place on the seat, creep 0.07 m/s to its edge and walk off
  it at 5.1 s.
- On its loop alone, `flight.shape_hop` left out, `hop` crouches 12 cm, throws
  at 1.24 m/s up, flies 0.15 s and lands standing at 967 N (shaped, 18.7 cm
  over its stand where 2.2). The chair's hop as a loop text - a lean onto its
  toes, a throw with its sagittal shift stiff ahead, a soft landing - flies
  0.2 s and lands on the seat's edge: off a seat 0.22 m to its edge and 0.45 m
  down a throw needs 0.9-1.0 m/s ahead, its ZMP in its soles gives 4.9 m/s^2
  for its 0.15 s, 0.74 m/s. Not kept.

## The floor under a sole

- Its law read a pitched sole as on an edge, so a sole flat on a ramp stood on its
  heel's line. Each sole's floor is now its own pitch while its insole's heel cell
  and ball cell both press, as far as alike; under half `EDGE_RAD` it is level.
  The rug held at 13 of 13 wobbles where 0; level floors unchanged to the digit.
- One centre of pressure per insole took a hole's edge under a sole's middle for
  a floor 10 deg up; two cells tell them apart. Read whole, a level floor was
  0.003-0.006 rad heel up (its soft soles' balls pressed in deeper) and its
  pelvis rolled 4.96 deg where 5.05.
- `f * (R.T @ v)` for `(f * R.T) @ v` in the edge moved its roll by 0.015 deg
  and flipped the turns' play at one held wobble: a scenario's three wobbles
  are a margin's sample; thirteen (-6e-4 to 6e-4 rad) read the hole 9 of 13.
- On the 10 deg ramp each sole's floor reads -0.14 to -0.17 rad (-0.175); it
  speeds up it, 0.28 -> 1.07 m/s, its height measured from one floor for both
  soles, the lower sole's heel lifted further each step.
- CI installed MuJoCo 3.16.0 against 3.14.0 here: the hole fell at two held
  wobbles and the tree ended at 0.18-0.19 m/s (0.01-0.05 here). The engine is
  pinned to the one measured on; the hole, 9 of 13 wobbles, left the held set.

## The floor ahead

- Up the 10 deg ramp each swing came down early, 0.34, 0.36, 0.19 s where 0.40,
  0.05 to 0.09 m behind its centre of mass (0.10 ahead on a level floor), its
  speed 0.39 -> 1.74 m/s, down at 5.1 s: its swings and its stance read one
  floor height for both soles. Read as a plane through the floor's height at its
  point, pitched as its standing soles stand on it (`modal.under`), they land
  0.04 to 0.28 m ahead and it climbs 0.33 m; its level walk the same to the
  digit, the bench unmoved.
- The ramp's top and the stairs' landing were grown 6 m long past their compiled
  bounds (`floor._sized`); MuJoCo culled every contact 0.85 m from the top's
  middle and it walked off the ramp through it.
- At the top a swing ended in the air on the ramp's plane run on, 7 cm over the
  top, and stood at 0 N by its height while the other was let go: a sole stands
  by its height alone only over the floor felt. Down at 6.5-9.0 s where 5.1.
- Not kept: a sole its law has flat turning its floor at 2 rad/s while one cell
  alone presses - level ground has such windows at every toe-off, up to 0.153 s,
  and its walk went 385 J/m with its knee 20 deg behind plumb; the floor bounded
  by a sole on one cell, at least its pitch toes down on its heel and toes up on
  its ball - the ramp 3 of 3 and 9 of 13, but at the hole's lip a sole on one
  cell is on no plane and the hole fell 3 of 3; the same with the bearing point
  within 20 mm of the sole's end (the ramp's 12-16 mm, the lip's 29 and more) -
  ramp 1 of 3, hole 1 of 3; its height lowered with a swing past its rhythm's
  end - down the ramp it fell at 5.0 s where 3.5, and a standing 80 N shove was
  lost.
- Left: on the ramp its speed 0.6-1.3 m/s where 0.69 is asked; past its top the
  plane run on over the level; down the ramp a swing hangs 6 cm over the floor,
  down the stairs 27 cm, and reaches on only 11 cm (`stride.REACH_S`).
- Down the stairs a swing goes 0.65 m, past the first tread, stands at 0 N by
  its height over the floor its standing sole is on, and the other is let go:
  down at 4 g. Down the ramp its soles read its pitch 0.04-0.14 rad where 0.17
  (`se2.floored`: a sole's tilt while its heel and its ball press alike), a
  swing hangs over the floor ahead, and on one sole its centre of mass goes 0.44
  m past it, 0.69 -> 1.6 m/s.
- Not kept, each measured: the world's floor under each sole, a ray (stairs 3.10
  -> 2.93 s, down the ramp 3.82 -> 3.19, down the stairs 3.38); with it each
  swing ending on its landing's floor over the highest between (stairs 3.32,
  down the stairs 3.18, down the ramp 4.09, its walk unmoved); the stairs' gaits
  told (2.95, 3.5); a sole arriving stood by its load alone (down the ramp 4.32,
  across it 5.69, down the stairs 3.20); a 20 cm reach (as before); the floor's
  pitch from it two bearing soles' heights apart (up the ramp 9.17 -> 4.46, the
  hole fell); the ramp's pitch told under each sole (down it to its foot, 0.449
  of 0.45 m, and down there at 4.49 s; up it at the crest, 9.25). Told the ramp
  and its cruise no longer drawn on by its going (`stride.cruises`' AHEAD below
  it) it climbs it at 12 of 13 wobbles - drawn on, its going 1.0-1.35 m/s up it
  lifted its cruise to 0.99 where 0.63 was asked, it ran 1.37 and flew off its
  top -, but snow went 13 -> 9: there its soles slide on ahead, 1.0-1.29 m/s, and
  its centre of mass must follow them - held to what was asked it fell back at
  1.5 m/s. The bound's most held to what was asked of late, the same; neither
  surge is a slide - its estimator's slip 0.009 m/s at snow's, its standing soles
  0.003 at p90 -: both are its swing's, 3.3 m/s up the ramp. A known
  floor alone does not climb: its landings fall on nosings and edges, and its
  capture point runs on.
- The flight told it, its landings onto its treads: [stairs](stairs.md).

## Blind on a flight

Stairs of 0.15 x 0.30 m walked into at 0.69 m/s, nothing told, traced 20 ms a
pass beside the world's floor under each sole (`tools/sim/trace.py`).

- Down: the swing passing the believed floor's level in the air, 15 cm over the
  tread, stands by its height alone, the other is let go and it drops 4 g onto
  it (3.10 s). Held to a load or to lying still there instead, it reached on
  while its centre of mass ran past the standing sole, 0.8 -> 1.5 m/s in 0.3 s,
  and the hole's 10 cm, which that commitment and drop had held, fell: not kept.
- Up: the toes pushed back by the riser's face loaded the sole 85-285 N and it
  stood. An insole reads what presses up through it (`edge.read`'s load as
  the normal share): right, and not kept - the slick patch's 5 mm lip's push
  had stood a sole there and it fell at 3.9-4.4 s, and the snow's far edge
  fell at one wobble of three, a hop off its 4 cm at 1.4 m/s. Pushed back,
  the swing held before its descent with its rhythm and its rows waiting, its
  clearance climbing till the toes were over the edge,
  its heel landing 4 cm past where they met it: the first riser climbed (up 3.0
  -> 4.7 s), but the same hold fired at the tree's and the wall's foot and at
  the snow's 4 cm lip, and with it a give-up at 0.22-0.30 m of raise and a veto
  on a touch above its soles, the wall still fell at 6.3-9.0 s: not kept.
- A flight read from its landings - the floor's rise a metre along from one
  landing to the next, the plane ahead the steeper of it and the soles' pitch -:
  down the ramp -0.23 where -0.18, with a late swing lowering it as far as its
  leg needs it went 3.1 m to its foot (down at 7.63 s where 3.82), but on the
  snow the plane read the standing sole behind 3 cm over its floor and it was
  let go, and a spin in place lowered it and fell: not kept. The floor under a
  standing sole as where it landed, the plane for the swing alone: stairs 2.96,
  across the ramp 4.32: not kept.
- Left: on a tread or a ramp its centre of mass runs on past the standing sole
  while the next swing is late, and the swing goes where its capture point has
  gone - two treads on; nothing told, its first step down a flight is a drop.

## The takes as scenarios

Each take a scenario judged against its take: [stunts](stunts.md).
