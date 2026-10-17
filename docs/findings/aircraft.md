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

## Aerobat

`machine.aerobat`, `machine.wingborne`: a 1.30 m, 2 kg 3D monoplane (wing
0.338 m^2, 0.03 m^2 of parasite drag, a 0.55 m tail arm with a 0.06 m^2
stabiliser and a 0.045 fin, inertias 0.09/0.06/0.12 kg m^2) on a taildragger's
wheels (0.25 m legs, the prop clear when level, the nose 12 deg up at rest).

- The wing bears the weight at 10 deg at 11.5 m/s (`Wing.rotate`). The run:
  on its wheels at 1.2 g, the tail up at 0.4 of that speed, rotated at it
  no further than it rests, the nose 22 deg up once the wheels are off, 1.5
  W of lift; airborne at 0.85 m, the hover's spot laid where a brake at its
  lean ends. It stands 17.9 m short of home on the first gate's heading
  (`RUNWAY_M`): rolls 2.5 s, lifts at home +1, climbs to 1.9 m at 13.9 m/s,
  comes to its prop-hang 10 m on and walks home.
- Wing-borne (nose first at the wing's speed, `Core.wingborne`) the nose
  goes on the air and over it by the angle the wing needs for what is asked
  across the path, no more than 15 deg, its back rolled to that; its push
  observer frozen (what it learnt there was the wing's own 1.5 g, turning
  with the nose faster than it was learnt: push_hat +-9 m/s^2, leans of 46-64
  deg asked, a 45 deg tumble at the top of full tilt); no thrust pulled back
  nose first (it coasts to its apex).
- The tail is destabilising tail first, by the physics: the burn's 12 m/s
  tail slide turned it over and onto the floor at 8.8 g. The aerobat's burn is
  a parachute at 4 m/s (`parachute`, its floor 0.55 m: its tail hangs 0.665
  under its middle in a prop-hang, the quad's 0.1 m sat it on its tail).
- Full tilt 12.4 m/s and 18.7 m, 3.07 L/W, 2.0 g; the apex 17-21 deg, 623
  deg/s for a moment; the parachute held at 0.58.
- A propeller pulls one way (`f_des >= 0`): the inverted hover, the flat spin
  and the lomcevak are gone from its card; the waterfall's inverted half
  falls through, 8.0 to 4.9 m, caught at 3.6. Wing-borne is not a dive
  (`DIVE_M_S`): nose first down out of the waterfall the law followed the air
  into the floor.
- The landing a circuit: climb-out at 3 m and 8 m/s in a harrier (tilt 45-68
  deg, the hover's law on a spot walking at 10.5), a 180 deg base turn,
  final to 0.35 m, the flare: throttle closed, the nose 40 deg up until the
  tail wheel is 0.22 m from down, then as it rests; 2 m/s at the touch, 0.9
  g, rolled out on the wheels' friction (0.15, their contact softened from
  0.01 to 0.03 s: on 0.2 and 0.01 the run chattered at 5-9 g). With the
  throttle held through the flare the wing floated it up at 1.4 m/s: no
  stall in it. Let down in the hover instead it sat on its tail at 0.36 and
  came over at 16 g, a strike.
- The runway lies 45 deg off the first gate's heading: along it the run went
  through gate 1 (its post 0.4 m off the line, the wing 0.65), across it
  into the house; on its wheels the law keeps the runway's heading, in the
  air it turns to the gates'. On the course it struck gate 11 before a lap:
  the page flies its routine alone (docs/TODO.md).
- With the drag of a quad (0.15 m^2) and a 0.17 m^2 wing it could not reach
  the wing's speed: 12 m/s at the thrust's 0.85 g; its main wheels 0.15 m
  ahead of its middle, the elevator's 2.45 N m could not lift the tail
  (0.09 now). Its rudder, the ornithopter's and the helicopter's hold at
  2-3x their loads.
