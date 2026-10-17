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

## Jerk

Each segment's peak jerk (`tools/sim/jerk.py`, 20 ms windows) against her steady
walk's - pelvis 927, torso 774, head 1262, hands 2238-2359, feet 4065-4136
m/s^3: a walk's start 3.5-4.3 at her core and head, turns and the weave 6.3-8.2
at her torso and hands, strafes 1.1-1.9, a hop down 11.7.

- A start's spikes sit on its first landings, the trailing sole leaving 9 ms
  after the other lands: its toe's 13-65 N stood it and let it go each law pass,
  her asked acceleration 1.2 m/s^2 across and back every 3 ms. Her asked jerk
  is no higher there than walking (p99 343 against 439 m/s^3); the weave's is
  4 times (1767).
- At the window's switch, a start's and a steady step's are alike: 21 ms after
  the other landed, it bearing about half her load. Out of her support region
  in one pass, her weight swung between the soles every 10 ms and her head
  went 22 m/s^2 across.
- That sole's share let go over her rhythm's double support: starts 4.0 -> 2.5,
  her walk 412 -> 448 J/m, 2.61 -> 2.78 off a woman's. A released sole kept
  down until its swing: `play_about` 11 of 13 wobbles, walking shoves 28, but
  at the tree she stepped in place for good - need 1 each step, no double
  support to settle in: her stop rests on that sole standing again.
- Under it, her step: in double support its striding projection planned her
  weight's split flipping a pass - [0, 379], [382, 0], [0, 384] N - and at
  times [0, 0]. Her soles on their ball and heel, each one's room a line, the
  projection met it through her soles pushed against each other - 926 N along
  one, -906 along the other: in her kinetic energy's metric 1e-7 per N^2
  against 0.03 for a push that moves her -, drove one to pull and let both go:
  11 times in 12 s of her walk, 7 in a start and stop.
- Each standing sole's wrench kept in that metric, W_KEEP 3 of its share of her
  weight: none let go walking; her walk's peaks 553, 590, 597, 1398-1840 m/s^3
  (pelvis, torso, head, hands); a start 1.6 of them, turns 6.5, a spin 5.9, the
  weave 13, the chair 13.4, a hop down 18.7; her walk 408 J/m, 2.55 off a
  woman's (412, 2.61); the bench 62 of 87, 80 N walking 29 (28), 200 N standing
  15 met (14), walking 17 (21). At 10 the tree and the hole fell at two
  wobbles; at 30 2.84 off; at 100 let go 187 times.
- A seat she rose from: a contact let go in a pass, its point still held,
  asked 5134 N of a sole; let go when the step would have it pull, 488 N at
  most - the bed 7 -> 9 of 13 wobbles.
- Not kept: a support's push weighed by its distance from her ZMP - no seat
  used till she is over it, the chair fell at 4.7 s; the blend eased, its rate
  and acceleration nothing at its ends - a spin 4.6 -> 1.7, turns 2.8 -> 8.5,
  her head 35.4 mm aside (35); a swing asked as far as its sole no longer
  bears - 401 J/m, 2.64 off, the weave as it was; a standing leg's servos
  softened by its load - every digit the same.
- Left: a sole let go at 60 N on its toes, the other then 715 N where the step
  planned 192 (turns, the weave); a seat's touch flickering as she rises; a
  hop's landing.

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
