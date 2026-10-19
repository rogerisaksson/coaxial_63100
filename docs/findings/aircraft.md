# Findings: aircraft

The helicopter, the ornithopter and the aerobat on two boards each, one core
under three names (`machine.helicopter`, `machine.ornithopter`,
`machine.aerobat`, `machine.wingborne`); the quad's own: [quad](quad.md).

## Helicopter and ornithopter

`machine.helicopter`, `machine.ornithopter`: one core, a swashplate's or four
wings' names on it; their routines on ideal rotors at 120 Hz, a line a stage
(hover, pirouette, orbit, spiral, flip, inverted, tic-toc, roll, full tilt,
burn, land).

| | Helicopter | Ornithopter |
| --- | --- | --- |
| hover: collective, lean | 0.34, 1 deg (the tail's thrust) | 0.34, 0 |
| pirouette | 195 deg/s, tail 0.27 | 195, aft 0.27 |
| orbit, spiral: bank, L/W | 32-36 deg, 1.19-1.45 | 32-36, 1.19-1.47 |
| flip, inverted hover | 247 deg/s, 2.0 L/W; -0.97 held | 247, 2.0; -0.98 held |
| tic-toc / wing rock | 602 deg/s, 2.9 g, L/W -3.2..3.2, tail at its stop | 648, 2.9 g, -3.3..3.2 |
| piro-flip / slash: drift | 2.4 m off its spot, caught at 1.95 L/W | 2.4 m, 6.1 m low, 3.4 L/W |
| full tilt | 703 rad/s, 11.1 L/W, 9.6 g, 26 m/s, 46 m | 703, 10.6, 9.1 g, 25 m/s, 44 m |
| burn | from 61 m to 0.13, 3.9 g | from 55 m, 3.8 g |
| landing | -0.06 m/s, 1.6 g for a step, one skid 53 deg/s | -0.07 m/s, 0.6 g |

- The main rotor at 703 rad/s is 6 700 rpm: the 0.5 m blades' tips at 351
  m/s, Mach 1.03. A helicopter governs its head and pulls by collective, 2.9 times
  its weight at the governor's 360 rad/s. The ornithopter's beat at the
  governor is 29 Hz (gear 2), 56 in full tilt; its wings were drawn at the
  rotor's angle, twice the HUD's beat.
- `quad.Sky.state` read every craft's height from the quad's rest on its skids,
  0.135 m: the ornithopter's and the aerobat's is 0.150, their hover 1.485 m
  and their burn's stop 85 mm; each sky carries its own `rest`.
- The HUD's rotor rows at 40 cells: `CONV 93%` was `CONV …` behind `SOA`, the
  title `POWER OF ALL FOUR  last 30 s` cut at `last 3`.
- Governed: the head at 360 rad/s, 403 boosted in the figures, the pull planned
  on it (`PULL_MAX`, 3.6 g). Full tilt 10.2 m/s and 16.4 m, 2.9 L/W, 1.8 g;
  the burn from 20.9 m on no collective, held at 0.13; the figures to the
  digit as before. The tail a 0.15 m rotor at 6e-5 N s^2 geared to the head
  (0.88): 0.73 of its pitch in the piro-flip where it stood at its stop. The
  ornithopter's aft wings alike (0.94, k_yaw 3e-5): 0.91 in the slash.
- The ornithopter's beat through a 6:1 crank: 9.7 Hz at the governor, 11
  boosted (the BionicOpter's 15-20 Hz at 175 g, by the sixth root of the mass
  10). Its pairs' lift pulses 0.3 of its mean twice a beat in antiphase: a
  pitching moment of 0.4 N m at 19 Hz, 0.07 deg of rock, 10 deg/s in the
  hover - under a dot.
- Pulled against its axis - a burn's brake - a lean leant the frame the wrong
  way: the horizontal ask turned with the thrust's sign; the helicopter's burn
  leans 17-19 deg where 20, no longer against itself.
- The arrows are the force from the hub, inverted too, drawn over whatever
  covers them; `NEG_ARROW_INK` where a thrust pulls against its axis.
- The laps. Planned on `PULL` of the governed pull - the quad's share of the
  drive's whole - the helicopter lapped the quad's line in 39 s at 4.3 m/s;
  on all it has (3.2 g) it struck gate 4, on 2.6 gate 7, on 2.0 g lapped in
  18 s whole, on 1.5 in 23 (`COURSE_SPHERE`, the page 19.5 s at 8.6 m/s, 13.2
  at most); the ornithopter on 2.0 and 1.7 struck gate 7, on 1.5 lapped in
  24 s whole. The head idles up (the stunt boost) where the plan leans past
  half of gravity, as in the figures.
- On their instruments (`machine.senses`) both lap the course and land. The
  ornithopter's dive turns its lean over the top - 1 m/s^2 asked up, its
  lean 80 deg each way -, its cyclic end to end in 30 ms: 1.37 a pass on the
  truth, 1.50 on the instruments, the check 1.5.

## Drawn

Four meshes of lofts, surfaces, drums, tubes and boxes, each judged by PNG
at 150 x 62 cells alone and at 80 x 30 on its page: the aerobat an Extra
300's and the helicopter a T-REX 700's, measured off models
(`machine.aerobat.form`, `machine.helicopter.form`); the quad a Hunter
long-range quad's, measured off a model - two plates on flat arms, the pack
under them, the camera ahead, a GPS mast, an aerial; the ornithopter
designed, no model - a ribbed glass head, a segmented abdomen, veined
dragonfly wings, a V-tail, insect legs. Skin, plate and accent paints:
carbon's halftone leaves a cloud of dots, so the bodies are light and the
fittings dark.

Each laid on what the sky flies and stands on. The T-REX at 0.831 a unit,
its tail hub at the boom's 0.85 m, its disc 1 cm over the sky's 0.22 m; its
tail rotor 0.11 m (0.15 at the 1.0 m rotor, its tip 7 mm over the ground); its
skids at the sky's feet 0.17 m out, the model's 0.086, the bows spread to
them.
The Hunter at 0.0216 m a cm, its motors' 19.6 cm reach at the rotors'; its
legs on the sky's feet at 0.7 of the arms (the rails before stood 0.1 m out,
0.11 off them); its two-bladed props on its planform at 16 in, the aerobat's
on the same. The 63100 stacks coaxial: under each motor, the helicopter's
main under its mast, its tail's out from the boom's side.

The Extra at the 1.4 m span, off a model 21.9 units across: fuselage 1.27 m
to the spinner's tip, the stabiliser's quarter chord 0.67 m behind the
centre of gravity at 27 % of the mean chord (the flight model's arm 0.48),
the tail wheel 0.81 m. Its surfaces on hinge lines: the ailerons 28 % of the
chord from 0.10 to 0.64 m out, the hinge swept 5 deg with the trailing edge;
the elevator's halves behind 0.71 m; the
rudder full height behind the fuselage's end. The flight model keeps its
aerodynamics; its contacts are the drawing's - the wheels at 0.093 and
-0.819 m, 0.147 out; the fuselage four boxes along its sections, one box at
the belly's depth to the tail having struck the runway rolling at 5.6 s. The
low hover's floor 0.55 -> 0.74 m: at 0.55 the rudder's foot went 13 cm into
the runway, at 0.74 it holds 5.8 cm over it. The main legs 6.7 cm longer
than the model's, the sky standing it 12 deg nose-up on them. The lettering
read mirrored from above until its columns ran tip-in on +x.

## Aerobat

`machine.aerobat`, `machine.wingborne`: a 1.40 m, 2 kg 3D monoplane (wing
0.392 m^2, 51 g/dm^2; 0.016 m^2 of parasite drag; a 20 in propeller; a 0.48 m
tail arm with a 0.046 m^2 stabiliser and a 0.035 fin; a fuselage 0.14 m deep,
its form's 0.157 m^2 of side; inertias 0.070/0.046/0.093 kg m^2) on a
taildragger's wheels (0.26 m legs, the prop clear when level, the nose 12 deg
up at rest).

- A 64-inch 3D plane's wing: on 1.30 x 0.26 m (59 g/dm^2) it stalled at 9.4
  m/s and turned on 32 m at 14 m/s; on 1.60 x 0.32 at 7.6 m/s. One size with
  the others on 1.40 x 0.28 m: its opening's 9 m turns asked a CL of 0.95 of
  its 1.1, it ran wide and came to its knife 14.2 s on where 5.1; on 11 m
  turns 5.5 s. The 20 in
  propeller and its contra stage: thrust and the surfaces in their wash by
  the diameter^4 of the 16 in's, drag and the stage's torque by ^5, the
  boards' loads with them - 9.7 g at the governor's boost, 1.2 kW a drive at
  full collective, the loads 1.3 hovering, 3.8 at the most.
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
- Wing-borne (nose first at the wing's speed, `Core.wingborne`, gated on
  horizontal airspeed `min(along, vh)` so a vertical climb stays on the
  hover's law) the nose goes on the air and over it by the angle the wing
  needs for what is asked across the path, solved across the stall blend up
  to 15 deg, its back rolled to that; no thrust pulled back nose first (it
  coasts to its apex).
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
- The helicopter and the ornithopter fly the course through its gates'
  middles (`course.MIDDLES`): its crossings were laid for the quad's own
  misses, and on them the helicopter crossed gate 7 0.4-0.5 m right and
  0.3-0.45 m low, into it off its pad in still air at 2.0 g and in
  turbulence at 1.75. On the middles at 1.5 g, within 0.22 m of gate 7's
  middle in six winds. `course.braked` had planned any line on the
  course's bends - the sweep's among them.
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
- Full tilt 12.4 m/s and 18.7 m, 3.07 L/W, 2.0 g; the apex 3.2 deg (under 10
  deg); the parachute held at 0.58.
- A propeller pulls one way (`f_des >= 0`): the inverted hover, the flat spin
  and the lomcevak are gone from its card; the waterfall's inverted half
  falls through, 8.0 to 4.9 m, caught at 3.6. Wing-borne is not a dive
  (`DIVE_M_S`): nose first down out of the waterfall the law followed the air
  into the floor.
- The landing a rectangle back onto its runway: climb-out, four 90 deg
  turns at 50 deg/s and 12 m/s, 8 deg, the final flown wing-borne (`w =
  1.0`) at 11.5 m/s down to 0.38 m over the runway. At 50 deg/s and 10.5 m/s
  the turns asked CL 1.20 past the stall's 1.10 and flew at 25 deg on the
  propeller. The flare: throttle closed, the nose pitched up as `1 / v_fwd^2`
  to 16 deg (`HARRIER_RAD`) as airspeed bleeds off in ground effect.
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

## Motors

Shaft power is `k_drag w^3` at the governed top (`GOV_RAD_S * STUNT_BOOST`),
the quad's at hover (211 rad/s a rotor, 4.9 N). Cans drawn to that load,
diameter x length; the Coaxial 63100 board (68 mm) behind each stator, on
the helicopter's frame faces (the frames 64 mm apart, the board too wide).

| Craft | W | Can mm | Board |
| --- | --- | --- | --- |
| quad, a rotor | 39 | 42 x 27 | under the can on the arm |
| helicopter, main | 271 | 42 x 48 | frame's left face |
| helicopter, tail | - | 28 x 20 | frame's right face |
| ornithopter, a pair | 304 | 42 x 40 | behind it in the thorax |
| aerobat, each | 292 | 42 x 40 | behind it in the cowl |

Layers (S on a page): shell, frame (the quad's two plates, the heli's
frames and tray, the orni's spine, the aerobat's longerons, firewall, spar
and ribs), pack (a 6S brick, 140 x 45 x 40 mm, or the heli's tray lit),
bare (shafts, mast, swashplate, cranks, spars, every blade and surface).

- Drawn to their pictures: the ornithopter's pod dusky (58, 54, 48) with
  one canopy ahead and wings 16 mm in chord; the quad a slate hump
  (86, 92, 100) on round arms, its two skid rails 100 mm out at SKID_M;
  the Extra the quad's skin, its cowl, fin, rudder and spats amber, black
  lettering on top; the Goblin's canopy a loft over the front frame, lime
  above its floor, its blades lime past 0.8 of their reach.
