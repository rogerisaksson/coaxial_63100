# Findings: one stack for every drive

One motor, one gearbox and one inverter: the candidates, the motors as bought,
and what the stacks hold (`machine/drives.py`, `docs/DIMENSIONS.md`). The
drives' own sizing: [drives](drives.md); each drive's demand on her loop:
[demand](demand.md).

## Load band

The soles' load through a sensor's band (`physics.LOAD_S` 20 ms, the lab's):
standing still the raw contact solver flickered 0-950 N, a foot at 0 N 104
passes a second; at 20 ms the feet stand on 159 and 160 N, sd 0.02, the walk's
touchdown 769 -> 253 N, its power 683 -> 560 W.

## Candidates

One box and one inverter by smoke:

| Candidate | kg (drives) | Part types |
| --- | --- | --- |
| two frame classes (68 x 30 at KV 110, 60 x 16 at KV 90) | 32.3 (16.2) | 23 |
| frame B's lamination at two stack lengths: 60 x 16, and 60 x 30 for the hips, knees, ankles, spine; KV 90; box B and the 50 A inverter on every drive | 28.5 (12.4) | 21 |
| the long stack on every drive | 30.1 (14.0) | 20 |
| the short stack on every drive | 27.7 (11.6) | 20 |
| 60 x 20 mm at KV 90, the 64 mm box at 1:30, the 70 mm 50 A inverter on every drive | 28.4 (12.3) | 20 |

- Fit (`tools/sim/fit.py` on each): the two lengths stand 12 and 5 mm past
  the skin at the ankle and the knee against 21 and 14; none past
  the shell but the elbow's 5 mm; the ankle's two drives clear of
  each other. On 60 x 20 mm: 9 and 3 mm past the skin at the ankle and the knee,
  6 mm past the shell at the elbow, 2 at the hip roll.

## Motors as bought

Complete units off the shelf, no frameless kit; two motors
(motors cost); one board and one gearbox (variants of those are the cost).
The 60 x 20 mm frame at KV 90 by a law fitted on T-Motor's pages is
no part: at its 4.1 N m its rotor was 0.75e-4 kg m^2 where every outrunner
found with that torque has 1.6-2.6e-4.

| Motor | mm | g | KV | mohm | N m, 180 s or rated | Peak | USD |
| --- | --- | --- | --- | --- | --- | --- | --- |
| T-Motor U8 II Lite | 87 x 27 | 253 | 85, 100 | 134-141 at 100 | 3.0 at 31 A | - | 300 |
| T-Motor U8 Lite | 87 x 27 | 243 | 85 | 225 | 2.1 at 19.1 A | - | - |
| CubeMars R80 | 87 x 27.5 | 354 | 110 | 125 | 1.3 | 4.0 | 259 |
| CubeMars RO80, a kit | 93 x 26 | 352 | 105 | 120 | 1.3 at 15 A | 4.0 at 50 A | 115 |
| EaglePower LA8308 | 92 x 28.5 | 336 | 90 | 186 | 2.3 at 22 A | - | 45-90 |
| T-Motor MN6007 II | 67 x 26 | 159 | 160 | 178 | 1.2 at 23.7 A | - | 130 |
| CubeMars R60 | 69 x 26 | 248 | 115 | 300 | 0.8 | 2.4 at 40 A | 178 |

- The U8 II Lite at KV 100 on the 15 drives whose joint asks over 50 N m (the
  legs, the trunk); the MN6007 II on the arms' and the head's 6: USD 5 280 of
  motors where 21 U8s are 6 300; the LA8308 in the U8's place 1 680 and 1.2
  kg more.
- No T-Motor page gives a peak: the U8's 4 N m is CubeMars' for the same
  36N42P stator, the MN6007's 1.9 the RO60's by its stack.
- KV 100: the knee asks 860-1007 deg/s and at 1:30 has 960 at 48 V, at KV 85
  816. A joint's torque times its speed over the pack's volts is its amps
  whatever the winding; 42 poles at 5 000 rpm are 1.75 kHz under 50 kHz of
  PWM, no room for a higher ratio.
- On them (`docs/DIMENSIONS.md`): 108 N m and 0.177 kg m^2 at a hip where 124
  and 0.071, holding 74 N m where 66 (Km 0.26 against the law's 0.23); 51 N m
  at an arm's joint; 30.8 kg where 28.4, 14.7 of it drives.
- The 63100 on every drive: 100 mm behind the U8's 87, 33 wider
  than the MN6007. Past the clothes standing (`tools/sim/fit.py`): the
  elbow's stack 20 mm, the shoulder's 15, the hip yaw's 6, where the 70 mm
  stacks had the elbow's 6 and nothing else. The spine's, the neck's and the
  head's lie in the back on its middle line, flush (a hand's breadth off it a
  100 mm disc is 17-20 mm out of the shell); the head's under the neck's: in
  the skull, 0.2 kg more of head.
- Their rotors: fed forward whole by the board (`physics.ROTOR_FF` 1), a
  standing start's first 50 ms had the knees and hips at their clamps; at a
  quarter of it (`physics.ROTOR_FF` 0.25), no saturation on start.

## Heat and power

- In still air (`drives.COOLING` 1.0 where the oil assumed is
  0.3; the boards as built, 10 min of heat a run, the windings the hottest
  node):
  + Air-cooled: a board's back on an aluminium holder over a third
    of it, on its motor or gearbox, its only cooling (`drives.COOLING` 1,
    `thermal_app_t`'s `joint` on the humanoid's worlds). The U8 holds 42 N m
    indefinitely where 74 in oil, the MN6007 22 where 35.
  + Two categories would serve if split: the four hips and knees, and the four
    ankle drives whose rotors stand 74-93 times the foot's inertia through
    their rods; the rest fits the U8 and MN6007 at 1:30. The drives stay as
    bought ([demand](demand.md)).

## The library's variants

The parts bought but not built in are in the library beside hers
(`machine/library`): the U8 Lite, the R80, the RO80, the LA8308 and the R60
as their pages have them, their rotors and windings' heat estimated from the
U8 II's; the roller wave at 84 and 50 mm; a 50 A board on a 70 mm disc. A
build names them (`machine/robots`); `COAXIAL_ROBOT` runs one, its text and
manifest with it. `gynoid_mesomorph_medium_r80`: her figure on the R80 and
the R60, the knee 1056 deg/s where 960; she walks 51 s at 0.69 m/s, 2.50
off a woman's band, 484 J/m where 408 on the U8 (KV 110 for 100: more amps
a N m, the copper's).
