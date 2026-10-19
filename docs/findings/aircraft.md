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
  truth, 1.50 on the instruments; 1.32-1.64 in ten gusty seeds, 0.80-1.90
  with the air and the ground at the floor, the check 2.0.
- The two-drive crafts' lines (`course.track(course.TWIN_WAYS)` and `sweep.track`)
  are smoothed by a zero-phase unsharp-masked Gaussian kernel (`smooth=4.8` in
  `course.laid`) with a 0.10 s lead (`course.twin`) matching the group-delay
  difference between the 2nd-order critically damped ($\omega_n = 10$ rad/s)
  $\text{SO}(3)$ reference governor (`helicopter.kit._smooth_target`) and the
  quad's tilt filter. With the ornithopter's intra-stroke wing-beat pitch rock
  rate compensated from `sky.phase`, the cyclic 1 s RMS on `course.TWIN_CARD` is
  0.0985 (still) and 0.0947 (gusty) for the helicopter, and 0.0975 (still) and
  0.0951 (gusty) for the ornithopter, with 17.3-17.5 s laps and no gate struck;
  each page draws the line its lap's giver flies (`quad.flight.ways`).

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

Its own page: [aerobat](aerobat.md).

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
