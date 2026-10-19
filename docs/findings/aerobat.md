# Findings: aerobat

The 3D monoplane on two boards (`machine.aerobat`, `machine.wingborne`); the
helicopter's and the ornithopter's, and their motors': [aircraft](aircraft.md).

`machine.aerobat`, `machine.wingborne`: a 1.40 m, 2 kg 3D monoplane (wing
0.392 m^2, 51 g/dm^2; 0.016 m^2 of parasite drag; a 17x8 propeller; a 0.48 m
tail arm with a 0.046 m^2 stabiliser and a 0.035 fin; a fuselage 0.14 m deep,
its form's 0.157 m^2 of side; inertias 0.070/0.046/0.093 kg m^2) on a
taildragger's wheels (0.26 m legs, the prop clear when level, the nose 12 deg
up at rest).

- A 64-inch 3D plane's wing: on 1.30 x 0.26 m (59 g/dm^2) it stalled at 9.4
  m/s and turned on 32 m at 14 m/s; on 1.60 x 0.32 at 7.6 m/s. One size with
  the others on 1.40 x 0.28 m: its opening's 9 m turns asked a CL of 0.95 of
  its 1.1, it ran wide and came to its knife 14.2 s on where 5.1; on 11 m
  turns 5.5 s.
- Its propeller a 1.4 m 3D model's 17x8, fixed, on its speed (static CT 0.08,
  CP 0.04): 477 rad/s and 321 W hovering, 455 and 136 W at 12 m/s, its top 776
  rad/s on a full pack's 63 V, 2.65 g; its thrust none at 1.05 of its pitch's
  speed. Its surfaces in its slipstream, `0.5 rho v^2` along the nose and its
  thrust over its disc (`slipstream`): as quick hovering as on the head
  governed at 365 rad/s, 0.85 of that at 12 m/s, 2.2 at 20. That head on a
  pitch had four 16x14s' thrust, 7.5 g; the 16x14 grown to 20 in, 2.5 g on
  60 A. Idled through a waterfall's top its tail read 5 Pa where a hover's
  97, and it fell 4.5 m onto its back: it is held for 0.7 of its surfaces'
  throw (`AUTHORITY`). Its routine lands in 9 airs of 9 where 7, the course
  in 8 (gate 2 at seed 3) as before.
- Its page on the 17x8, its boards turning it, the air's tour from six seeds,
  400 s each: 47 crashes where the suites' ideal rotors had none - 30
  waterfalls, 15 knife passes, 2 prop-hangs. Its propeller turned within a few
  rad/s of what it was asked: no lag; a tumble lost 2.5 m where 2.2. Gusty,
  its figures higher by `GUST_S` 0.6 s of its law's gusts - its own wind's
  swing about its 5 s mean past 0.3 m/s, its noise in still air 0.10-0.17
  -, its knife pass too; the waterfall one tumble from 6.0 m; its lap at 16
  m/s, which its pack sagged to 48 V holds (at 20 its 17x8's top of 600
  rad/s could not, and it hung nose-high at 4-9 m/s into the pylons): 2
  crashes in the 2 400 s, gates 2 and 6 in gusts.
- The wing bears the weight at 10 deg at 9.3 m/s (`Wing.rotate`). The run:
  on its wheels at 1.2 g, the tail up at 0.4 of that speed, rotated at it
  no further than it rests, the nose 22 deg up once the wheels are off, 1.5
  W of lift; airborne at 0.85 m, the hover's spot laid where a brake at its
  lean ends. It stands 13.1 m short of home on the runway (`RUNWAY_M`).
- The wing's lift across the air, its drag along it (`Wing.lift`, the
  planner's twins `se3mpc` and `mppi_plane.c`): along the back the lift was a
  drag of L sin a, and level at 12 m/s and 9 deg the thrust bore 0.39 of the
  weight; across the air 0.24; on 0.010 m^2 of parasite drag (the fuselage's
  front and the gear; 0.03 was three times a 3D plane's) 0.15. The thrust
  gives along the nose what the wing does not.
- Its own moments trimmed always (`Wing.trimmed`) and known to its spin
  observer (`aerobat.observer`): untrimmed, the stabiliser's 1.9 N m at 9 deg
  held the angle 2.6 deg short and it sank 0.1 m/s; trimmed and left to
  `turn_hat` as well, it held 2.5 deg over and would not come down its final.
  The wing's damping trimmed as it turns: 3.5 N m a rad/s at 17 m/s, it
  rolled onto its side at 15 deg/s.
- The wind read through the wing too (its Jacobian in the wind observer): on
  the drag's alone a hover's push came out 6 m/s of wind in still air, and
  wing-borne none was read; in gusts now 0.5-1 m/s off, a second behind. On
  its wing it flies the air's speed (`Core.plan`): on the ground's 14 m/s a
  7 m/s tail stalled it in the course's climbing turn.
- Wing-borne (nose first at the wing's speed, `Core.wingborne`, on its air
  in its plane of symmetry whatever its pitch, none past 60 deg off the nose,
  and on its horizontal airspeed so a vertical climb stays on the hover's law)
  the nose goes on the air and over it by the angle the wing needs for what
  is asked across the path, solved across the stall blend up to 15 deg, its
  back rolled to that; no thrust pulled back nose first (it coasts to its
  apex). On its air along the nose the share swung 0.4-1.0 with the pitch it
  set, 6-35 deg at 2 Hz, from a hover into its circuit at 8 m/s.
- It speeds up on its propeller's sphere and turns on its wing's
  (`Core.plan`): held to its wing's under the stall's speed it hung
  nose-high at 7 m/s from a hover into its circuit; on its propeller's, 12
  m/s through the air in 1.5 s, its pitch 8-9 deg.
- Flown, not forced: on its wing its height's loop no quicker than
  `PATH_POLE` over its climb's lag (`path_pole`, 1.4 rad/s at 12 m/s). On the
  hover's 6.4 it swung its pull 6.7 m/s^2 at 1.8 Hz in a 1.5 g turn over 10
  cm, its angle 6-26 deg, past the stall; on the hover's past 0.5-1 m off,
  so again off the hoop. On its side the hover's: its tip touched at 0.45 m
  on the wing's. Its push over `PUSH_S`: dropped, a turn's lift 14 % short
  sank it 1.2 m. Its take-off climbs no faster than that loop stops (at 12
  deg it rose 1.3 m over the lap's line into the limbo 30 m on), its heading
  turns no faster than its lean (at the line's 115 deg/s, its nose at 64,
  its pull ahead asked against its nose, it stalled into the house). In the
  air grown and subsided ([air](air.md)), six tours of 400 s: 3 strikes,
  gates in gusts, where the committed law's 7; its circuit's pitch 1.4 deg
  about its 1 s mean where 3.1.
- The opening: the run climbs out on the wing to 3 m; a teardrop at 2 m (45
  deg out at 60 deg/s and 14 m/s, 4-9 deg, a leg of two turn radii, 225 deg
  back) lays it on the runway's line the other way; a line-up at 0.52 m and
  20 m/s, a roll onto its side (`knife-in`), the knife-edge back down the
  runway (`knife_pass`), a pull-out on the wing, the prop-hang. Over the
  runway, 12 to -21 m along it at 19.2 m/s, the nose 11 deg over the path,
  the wings within 16 deg of upright, the lower tip 0.023 m over the runway
  median and 0.018 at its lowest, 1.1 m off the centreline at most. At 12
  m/s on a 0.08 m deep fuselage the nose stood 39 deg over the path and the
  wings leaned back as far; at 70 deg/s the teardrop stalled at 18 deg.
- On its side (`plane`'s `knife`) the attitude is rolled a quarter turn about
  the nose and the side plate bears what the wing did, at the slip
  `tan b = need / (rho S v^2 / m + along)`: 11 deg at 19 m/s. The fin's
  weathervane, 4.5 N m, takes 65 % of the surfaces' boosted reach; left to
  the law's 33 N m/rad (no `Wing.trimmed`) it held the slip 8 deg short and
  the tail sank 3.7 cm in 1.8 s. At 18 m/s it passed their reach and a tip
  struck. Its airspeed latches it wing-borne, the nose's share being cos b.
  The sampling MPC (`plan` 1) flew it nose-high at 7 m/s: the law flies it.
- Nose first it flies a speed, not a place (`Core.plan`): chasing its spot
  along the path it lagged the teardrop's turn by 12 m and the line-up ran to
  18 m/s to catch up. On a spot laid on itself the teardrop holds 13.8-14
  m/s and the final glides on the centreline at 11.5 m/s, where it swung
  35-80 deg at 10-16 m/s.
- Low it lines up on the runway's way nearer its heading (`lined`), a hover
  up it, no faster than its lean turns it at its speed: at 2.4 rad/s the
  line-up stalled, 27 deg at p90. The course's grid heading is a hover's:
  crossing gate 0 at speed it swung the pass 55 deg and a tip struck.
- The pages watched by a spectator beside the field (`show_quad.watch`): the
  eye where it stands, its gaze 0.15 s ahead of the craft filtered over as
  long, its lens framing the craft by its drawn radius a third of the frame
  high over its horizon - the horizon's claim fading from 20 to 45 deg up:
  kept at 60 it held the burn at an eighth of the frame -; the chase on the
  course's laps only. `lined` read any line as the course: the aerobat's
  opening chased its whole routine on the course's 1.5 g sphere with its
  gates up, 33 rows of 33, now 5.
- The helicopter's and the ornithopter's figures at 4 m where at 8, full
  tilt 1.4 s where 2 - the burn from 15 m where 22 -, their sweep past the
  spectator at 12.4 and 11.0 m/s, 1.5 m up; the ornithopter's dive attack's
  tumble 0.8 s, at 1.2 from 4 m its catch dipped to 2 cm (now 1.7 m). The four
  pages watched from one spot by the runway's middle, each craft a quarter of
  the frame: at a third the quad stood too near at its start.
- On the deck, each figure's lowest point - skids, blade and prop tips,
  wings, tail - 0.2-0.5 m over the runway, off the drawn frames and the
  rotors' swept rings: the hover at 0.3 m, the orbit 0.45, the high 0.75. The
  helicopter inverted at 0.2, its disc 0.12-0.13 m over the runway still and
  in six winds; its inverted pirouette climbs to 0.95, where from 0.55 the
  tic-toc after it brought its tips to 0.12. A tumble starts high: the
  ornithopter's wing-rock and slash at 2.4, the slash 0.33 over at its foot;
  the helicopter's chaos fell 1.3 m from 2.4 and struck from 2.0. The quad's
  roll and flip over a 0.7 s toss at 2.0-3.0 m where 3.3-4.9: tossed 0.6 s
  its catch dipped to 0.55, 0.5 s to the floor; spent mid-roll it strikes,
  at 0.7 s as at 1.0. The Extra hangs at 0.9, its tail 0.33 over, its high
  4.5, its knife pass's tip 5 cm; its circuit at 1.8 - at 1.0 it struck in
  the tour's turbulence.
- The air and the ground at the floor (quad.md's air): the helicopter holds
  inverted at 0.2 on 0.21 collective where 0.27, the floor bearing a quarter
  of it. In six winds over 250 s the quad struck none, twice without them,
  both spent: in its 0.45 m orbit in a changing wind, leaning 30 deg at 3
  m/s onto the floor, and in its roll; the helicopter once and the
  ornithopter twice into gate 7. In 36 turbulent seeds two each, three and
  one before the deck; without them the ornithopter struck the floor once.
- The quad's crossings were laid for its own misses: on them the helicopter
  crossed gate 7 0.4-0.5 m right and 0.3-0.45 m low, into it off its pad in
  still air at 2.0 g and in turbulence at 1.75; through the gates' middles at
  1.5 g it held within 0.22 m of gate 7's in six winds. `course.braked` had
  planned any line on the course's bends - the sweep's among them.
- The helicopter and the ornithopter on a line laid for them by flight
  (`course.TWIN_WAYS`) at 2.2 g, their laws on all they have - the governed
  head's 3.6 g; 10 g only at the drive's top, the tips at Mach 1.3: a lap in
  18 s where 24, 14 m/s at the most; none struck in 17 airs, six page winds
  and 36 turbulent seeds, where at 1.5 g on the middles two of 36 each.
  Their laws had been held to the plan's 1.5 g. Planned on all 3.6 g the
  head had nothing left to correct with - its tilt rang 19-89 deg, into gate
  4; through the middles at 2.5 g five gusty airs of nine struck, laid at
  2.5 three. Planned on its own tilt loop (a swing of 0.27 s, the quad's
  0.21) gate 1 stood as far off: not kept.
- The spectator 10 m beside the runway, each craft framed at 0.4 of the
  view's half, 0.3 at the least under the horizon's claim: from 14 m at 0.25
  and 0.12 the four drew 33-52 cells of a 100 x 36 page, the helicopter 33;
  now 58-98, the helicopter 58. At 0.2 the least a snap roll drew 47 cells
  of the page's 50.
- Close to the runway, a spectator 14 m beside it: the card within 62 m of
  them where it reached 105, its 3D figures within 23 m, its circuit at 3.5
  m where it flew at 9, the burn from 31 m (44), 92 s (124). The opening's
  turns on 8 m, or its run 6 m, bled it to 4.5 m/s; rolled onto its side
  over 12 m its wings swung 23 deg off upright; its teardrop flown as
  timed turns ran 54 m out and 19 m beside the runway, the prop-hang then
  walking 50 m home in 15.4 s - laid, it hands over 1.5 m from home.
- The airfield (`machine.airfield`): its trees 85 and 100 m off the runway's
  line and 165 m along it, a teardrop's 64 m across at 14 m/s; its gates
  stand only on its course. The course, a lap of 543 m at 14 m/s airspeed on
  27 m turns: an air gate, a limbo bar at 2.8 m, a hoop at 10 m at the far
  turn's top, a slalom of three pylons 4 m off its line, a knife gate twice
  the wings taken on its side 4 m/s faster - one line of arcs a metre apart,
  flown and drawn. Flown by L1, a point 1 s ahead (`2 sin(eta) / T`): calm,
  every gate within 0.4 m, 38.2 s a lap; gusty, within 0.7 m, down 9.5 m
  along its 16 m runway and 5.7 m off its centreline (unsteered). On the
  tangent crabbed over a second (`lined`) it passed the hoop 3.9 m off in
  gusts; a slot of 1.5 m, narrower than the wings, 1.6 m off.
- The tail is destabilising tail first, by the physics: the burn's 12 m/s
  tail slide turned it over and onto the floor at 8.8 g. The aerobat's burn is
  a parachute at 4 m/s (`parachute`, its floor 0.74 m: the Extra's rudder
  hangs 1.0 m under its middle in a prop-hang, the quad's 0.1 m sat it on its
  tail).
- Full tilt to 14.0 m, 18.7 on 7.5 g, its tilt within 1.7 deg; the parachute
  held at the floor's 0.74 m.
- A propeller pulls one way (`f_des >= 0`): the inverted hover, the flat spin
  and the lomcevak are gone from its card; its waterfall's two tumbles fall
  from 5.0 m to 0.56, caught there. Wing-borne is not a dive
  (`DIVE_M_S`): nose first down out of the waterfall the law followed the air
  into the floor.
- The landing: its circuit and its final laid lines flown as the lap is
  (`airfield.CIRCUIT`, `APPROACH`), the course's off its last lap: up the
  runway's line, slowed 1.5 m/s^2 to 12 m/s by its first turn 20 m along,
  round to its near side on 20 m turns 1.8 m up, the final a glide of 0.035
  to 2.5 m past the threshold, 11.5 m/s into its flare at 0.22 m. On 12 m
  turns at 12 m/s - 12.3 deg of the stall's 15, the angle swinging 6-27 deg
  in gusts - every gusty course stalled in its last near turn and hung on
  its propeller down its final at 2-9 m/s; on 20 m, 9.6 deg, no final of 13
  past the stall, the circuit 0.3-1.0 s. At 10.5 m/s turns at 50 deg/s had
  asked CL 1.20 past the stall's 1.10; the far side's gates stood 1.3-2.9 m
  off a 20 m turn.
- The flare: throttle closed, the nose up as `1 / v^2` through the air to 16
  deg (`HARRIER_RAD`), its sink its height over 0.5 s, 0.15 m/s at the
  least; the roll-out tail up, down as it slows, its tyres gripping across
  at 0.8 of the load, 2.35 m/s^2 along. On the ground's speed a 1.1 m/s tail
  wind left its lift 16 % short, a touch at 0.34-0.50 m/s; held off at 5 cm
  to the stall it floated 12-38 m. Still air: a touch at 0.22 m/s and 11.0
  m/s, 0.04 m off the centreline, stopped 8 m along. Gusty, a 1 m/s tail wind
  on the floor: 13 of 13 at 0.11-0.22 m/s, 0.19-0.68 m downwind, stopped
  15-22 m along - past the strip's first end at 16 in 10; its end now 30.
- The wing stalls (`wingborne.STALL_RAD`): the airfoil's lift to 15 deg, a flat
  plate's normal force past 23, blended between, the plate's drag with it;
  at 10 m/s 21.5 N at 15 deg, 14 N at 45, 40 N of drag at 90.
- With the drag of a quad (0.15 m^2) and a 0.17 m^2 wing it could not reach
  the wing's speed: 12 m/s at the thrust's 0.85 g; its main wheels 0.15 m
  ahead of its middle, the elevator's 2.45 N m could not lift the tail
  (0.09 now). Its rudder, the ornithopter's and the helicopter's hold at
  2-3x their loads.
- On its instruments the knife pass at 0.52 m struck the runway in two draws
  of eight - a pull's tilt left its height 2 cm off -; at 0.545 m none of
  ten, its tip 1.8-4.6 cm over. On the 1.4 m wing at 0.47 m its tip 3.3-4.3
  cm over, its wings 0.94 upright at the least; at 0.45 it struck. Full
  tilt's 0.2 deg of tilt on its 147 N left it 0.6 m off its spot, 0.15 m/s
  off in speed, and the burn's coast -
  no thrust, and hovering, a row's 5 m/s short of wing-borne at 9-17 m/s -
  leant 28 deg after it against 1.6 on the truth.
- Its figures one into the next, a stage change's most angular acceleration
  in still air: wing-borne carried whole over 0.4-0.6 of its share (snapped
  at half) and the wing's attitude eased in with it, the teardrop's leg
  133 -> 42 rad/s^2 and a turn's 96 -> 43; the yaw lined up low eased in as
  a row's turn and slide go, the run-in 159 -> 30; the plan turning the
  heading as the law does (it turned to the quad course's gate), the final
  184 -> 48, the pull-out 102 -> 13, the line-up 109 -> 76; the run's pitch
  by its speed, the lift 72 -> 49. The turn-out's roll at its stop is 108,
  the stunts' entries 150-276, the wheels' contacts 179.
- Its course on all its propeller has, 2.65 g: on 1.5 its climb to the hoop had
  0.5 and it struck gate 2 in a gusty air of eight; on all of it none in
  nine. The quad's on 3.6 g of its 11.1, its line laid for that: on 5 and 6
  g its laps 12.1 and 11.3 s where 13.9, its ideal rotors into gate 0 in 4
  and 6 of 6, its boards none; on 7 g into gates 0, 13 and 14.
- Its turns on the wing's lift short of the stall, not the propeller's sphere
  of 1.5 g, which held it to 1.1 g across: the course at 20 m/s on 12 m
  turns (3.4 g, 7.6 g there to the stall), a 180 deg turn 1.9 s where it was
  6 at 14 m/s on 27 m; every gate within 0.44 m, the pylons 3.5-4.4 m off,
  its knife gate at a belly of 0.00. From 20 m/s its final needs the 90 m
  the lap now starts short of the runway: from 56 m it floated 18 m past the
  strip, at 9.5 m/s it ballooned to 5.5 m; it stops 3.2 m short of the
  middle. The routine's teardrops at 18 m/s and 130 deg/s (4.2 g), its
  circuit at 100 deg/s; the turn-out's roll at its stop is 166 rad/s^2.
- Its opening a laid line (`airfield.opening`): 12 m on from the lift, right
  a quarter turn straight into a left three quarters round on 11 m turns, onto
  the runway's line and down it on its side at 19 m/s, its lower tip 4-6 cm
  over the runway, its top to the spectator (0.90 toward them abreast, -0.50
  the other way round); a teardrop to the far side and up the line to the
  prop-hang. Flown as timed turns it ended 22 m beside the runway at 6 m/s.
- On its side the span stays up, the wing bearing what is asked across: the
  gusty course's knife gate at a belly of 0.00 on both laps, where the span
  rolled to every ask rocked it 0.07-0.86 and passed the gate at 0.29.
- Its page in six winds over 250 s: 16 crashes with the air and the ground
  at the floor, 20 without, 21 before the deck - on the runway, its knife
  pass 15-25 s into turbulence among them, into the house 22 m beside it and
  a car, into gates 2 and 3.
