# Findings: QUAD

The quad on four boards: frame and air (`machine.quad`), routine and
course, the page's flight, the tuner. Flown since from its cards' rows on a
core in C ([streams](streams.md)): "the law" here is the one before it. The
board's findings: [FINDINGS](../FINDINGS.md).

## Hover and landing

On four stand-ins:

| Fault | Fix |
| --- | --- |
| the third flight never left its hover: hover share 0.42 over a room of 0.4 since the dry loss's refit | the room is the throttle's point less a flight's 0.3 |
| the burn stopped 0.44 m up, crept 3.1 s to its mark | the burn flown down a speed for each height over its mark: within 2 cm 1.6-1.8 s on |
| the lift trailed its ramp 0.4 m | one altitude loop, 5 rad/s about a height and its rate |
| the landing dropped its last 3 cm | the thrust run down as the propellers do |
| the fall's and the landed spool's rotors, braked to idle at the clamp, spent 0.16-0.25 of the envelope | - |

Four flights on end: the lift at 1.50 m, down still, the share flat on the
floor.

## Routine

`machine.aerobatics`' card: a
pirouette, an orbit and a corkscrew to 8 m, a roll and a flip over a toss,
the way back down, full tilt, the burn. 59.9 s on ideal rotors, 0.5 g at most
outside its turns, the pole 2.7 m off; on four stand-ins no board throttling,
SOA 0.50 at most in its figures.

| Fault | Fix |
| --- | --- |
| heading loop at 16/8 on the discs' drag: rotors 1 470-2 080 rpm at the clamp in a 180 deg/s pirouette | 4/4, the turn fed forward; 120 deg/s: 16 A |
| spot loop at 4/3.2: an orbit's bank rang 19-31 deg on 14 of margin | 1/1.6, the air fed forward: 24-25 |
| a roll on rotors run down as their propellers do went 1.6 m aside whichever way the thrust was cut | the collective bears the weight over a turn begun and ended alike |
| thrust slewed 15 N/s swung a hover 0.4 m | - |
| a page pass 14 ms, one in twenty 50, the rows asked by the wall's clock | the flight's clock is its passes' sum |

## Pack

15 cells: 63 V full, 0.04 ohm, 1.0 A h, a racer's.

- The routine takes 12 % of it, a course's flight 20; full tilt 129 A, the
  bus to 56.3 V ([envelope](envelope.md), what the rotors pull). Spent at a
  fifth it comes down from anywhere at 2 m/s and is changed on the floor in
  3 s, the observers kept.
- At 63 V the gate stage's dump is the envelope. On the bench's still air,
  8.33 K/W: a hover at 0.65-0.72, every board throttled by the corkscrew. The
  record says a propeller's wash, 2.5 K/W (an assumption), the stand-in's
  truth laid on it: 0.52-0.60 in the figures, the hottest node 77-116 C.
- A row's pull is the action space's for what its boards hold and its pack
  has ([envelope](envelope.md), the shell): what the frame is sped up on
  and a fall's stop is planned on.
- A row's height told from an eased one by what it moved a pass: a 50 ms
  pass took the landed row for 9.6 m/s and launched the frame 3 m. A row
  brings its height's rate and pull.
- A rotor's speed asked 700 rad/s^2 at most: stepped, a pass was the whole
  clamp and 0.2 of an envelope for a read.

## Course

`machine.course`, flown after the routine has warmed the boards:
14 gates of 3.4 m on a line of 141 m (an alley of trees, a
house's ridge, round a mast, between two cars, a slalom).

- Ideal rotors: lap 21.6 s, 10.7 m/s and 69 deg at most, every gate within
  0.43 m of its middle on passes of 10-50 ms. Four stand-ins: 28 s,
  0.20-0.28 m over three flights, envelopes 0.54-0.82, the rotors asked 58 %
  of their pull at the mean through a lap (never all, never none), none
  throttling.
- What it took of the law:
  + A row's own pull (a bend's, asked 0.2 s ahead).
  + The nose free of its heading: a turn of the frame is the discs' drag's,
    10 N of thrust apart a N m. Held on the heading the rotors were at their
    clamp 12 % of a lap and two boards throttled at 0.94; free 3 % and 0.82.
  + The discs leaning against what is asked up, not all of gravity (at a
    crest a bend had 0.6 of its pull).
  + The tilt loop on the discs' axis alone (a nose 66 deg behind its heading
    turned the tilt's torque round and the frame went over), its miss
    answered as 0.6 rad at most: whole, a lean turned back asked more than
    the rotors slew and the page's frame swung 75, 87, 121 deg over, 4 m
    lost.
- The lap's pull is the envelopes' share of its lean: planned on the law's
  reach it never eased.
- A flight begins on 0.7 of the pull: an idle board is at 0.52-0.56, not
  cool; waited for all of it the page stood 120 s on the floor.
- The stand-ins' rotors and heat are stepped the pass's seconds
  (`SimulatedDrive.paced`, `fast_forward`): on the wall's clock a pass 0.25
  s late turned the rotors those seconds under a setpoint meant for 50 ms,
  and under the gate's load a gate was passed 2.7 m off; paced, 0.22 m alone
  and 0.25 m beside fourteen busy processes, envelopes 0.74 and 0.79.
- A pack gives the routine 21 % of itself and the course 27 %: spent on a lap
  of every fourth flight, ended at that lap's finish. Spent on a flight's way
  down, the card went on to the next flight's, that flight never flown, and
  the finish's gate stayed lit through the next lift: a flight's own way
  down, the first gate.
- A starved page's every pass is its longest, 50 ms; stepped whole, the frame
  rang on its rotors: the corkscrew at 30 A where 24 at 45 ms, the
  envelopes' share 0.22, `home` never held (41 s on it with the feed every
  60 ms, the pack at 67 %), and CI read full tilt's kilowatt as 814 W off a
  frame. A pass is its steps, 25 ms at most (`flight.passed`): the same
  feed, the corkscrew 26.2 A on 0.75 of the pull and over, `home` 3.0 s;
  full tilt's kilowatt is 0.15 s of its two, read a step.

## Air

`machine.quad`: `AIR`, `blown`.

- One kind of weather at a time: calm, constant, gusty, changing, turbulent,
  20 s each or one kept (W). A wind of 0.5-4 m/s at 5 m, sheared along the
  floor; 1 - cos gusts of 2.4-4 m/s over it; eddies of 0.1-1.2 m/s rms, the
  frame's and each disc's. The frame's drag is against it; a disc's thrust
  rises with air rising.
- The law is told none of it and learns the wind from its drag in 0.12 s:
  a hover at 5 m in 3.9 m/s stood 0.68 m off its spot unlearnt, never
  `held`; 0.03 m learnt; in gusts to 6.8 m/s 0.23 m at most.
- Its push learnt and not the wind: a lap in a constant wind passed a gate
  0.65 m off where 0.37 in still air (the push turns with the frame's own
  speed through a bend); the wind itself: 0.36-0.44.
- A bend planned on what the wind leaves of its grip: gusty 0.78 -> 0.50
  m, turbulent 0.94 -> 0.58, a lap 1 s slower.
- On the page's four boards the routine's fall went 3.5 m downwind and was
  back on its spot by its stop; laps 29.0 and 25.7 s, a gate 0.23 m off at
  most, boards to 0.76 of their envelopes, none throttling.
- The pointer shows the wind's direction as the view has it, its size and
  kind (`terminal/views/quad/wind.py`).

## Solids

`course.solids` in `quad.mjcf`: a tree its trunk and cone, a
house its walls and roof, a car, the mast, a gate's bars (128 things); the
frame strikes with its discs and hub; a pass 0.12 ms dearer.

- The first gate's lower bar stood 0.3 m over the spot, in the lift's way,
  struck 8 cm up: the floor is its lower edge.
- The gates stand for the course's flights alone, as drawn: full tilt goes up
  through the first one's top bar.
- Struck, a flight is over (`flight.CRASHED`): its rotors cut (asked to a
  stand through their loops, sensorless, they hunted at 17 A), the wreck left
  4 s, the frame on its spot again, its flight begun over.
- Through the tour's air nothing is struck, the routine and two laps: ideal
  rotors a gate 0.50 m off at most, the page's four boards 0.27.

## Shorter laps and WEP

The page's four boards, still air and six winds:

| Bend grip, lean | Swing | Throttle room | Laps s | Worst wind s | Gate off m | Envelopes |
| --- | --- | --- | --- | --- | --- | --- |
| 0.4, 20 m/s^2 | - | 0.08 | 26.4, 25.4 | 28.2 | - | - |
| 0.7, 23 m/s^2 | 0.6 s fastest | 0.04 | 21.8, 20.6 | 22.3 | 0.53 | 0.71-0.74 |

- The bound is how fast a lean turns, the rotors' spool: the tilt loop at
  twice its gain rang; the spot's at four times struck a gate; a lean of 26
  without the swing's limit struck the slalom's gate four flights of four.
- War emergency power was a rule: a thing in the way of the frame's ghost,
  flown 0.5 s on, and the law short of pull - all the rotors gave for 0.6 s,
  5 s of it a flight. It is the space's outer shell
  ([envelope](envelope.md)).

## Tuner

`tools/sim/quad_race.py` scores a candidate flown, not planned.

- A line searched for its planned lap (each gate crossed 0.25 m off its
  middle and turned 25 deg at most) was planned 20.2 -> 17.8 s and flown
  19.1 and 18.3 where 20.8 and 19.4, but 0.9-1.05 m off its gates and into
  one. Turned alone: planned 18.6, flown 5 % shorter, 0.65-0.97 m off.
- Its scoreboard: the card on ideal rotors and on four boards, still air and
  three winds, 8 flights in 15 s on the relay: laps 20.8/19.4 and 21.8/20.8 s
  as built, a gate 0.52 m off at most. A candidate is its constants by name
  and its line's crossings (`course.WAYS`), searched by `cmaes`.

## Size

`quad.sized`: shape and propeller tip speed kept; lengths by
size, mass by its cube, the rotors' pull by its square, their spool (the
frame's clock) by the size: speeds unchanged, pulls 1/size. The law's gains,
the routine's rows and the course (gates, plan) scale by size and clock from
tables of their units (`_UNITS`); a lap's lean is a share of the frame's
whole pull, a crest's fall of gravity.

| Size | Laps s |
| --- | --- |
| x0.75 | 16.4, 15.6 |
| x1 | 20.8, 19.5 |
| x1.5 | 30.2, 28.1 |
| x2 | 40.1, 37.4 |

Ideal rotors, still air, the course as much larger; by their clocks
20.0-21.9 and 18.7-20.8 s; a gate 0.68-0.92 of the frame's reach off its
middle. Gravity does not scale: x0.5, 71 m/s^2 along
the floor, came into the climb 1.6 m/s fast, went 0.7 m over and struck the
second gate's top bar.

## Plan

- Circle plan (the first course; the sphere's now):
  laps 20.8 and 19.5 s -> 14.9 and 14.4 on ideal rotors, 15.3 and 15.2 on
  four boards. What holds: the frame's place found between the line's
  samples (`course.nearest`); by a sample's own tangent the asked speed's
  change ran 4.3 m/s^2 rms a pass, 3.0 between. The asked speed eased to the
  plan's over SOFT_S, the plan looked up as much further: stepped, 14 m/s^2
  in a pass, it stood past its finish. Braked for its finish as for a bend
  it stood 0.36 m past the first gate (`course.STOP`).
- The line rises through its gates (`course.SLOPE`). Level
  through each, a climb is an S: second to third gate, 3.5 m in 12.5, 10
  m/s^2 up and down at 10 m/s, the frame 1.9 m under its line. Crossed at
  the slope from the gate before to the one after: 0.26 m; a lap 17.8 s
  where 19.3 on the level line's plan, a gate 0.65 m off where 0.48; half its
  size, struck on every plan before, flies 9.5 and 8.8 s. Searched on that
  line, 80 generations of 24 on ideal rotors: 16.1 and 15.3 s, a gate 0.45 m
  off; four boards 16.4 and 15.9 at 0.67-0.74 of their envelopes, stuffy
  18.2 and 17.1, the pack at 60 % 16.4 and 15.9; 33.3 as the tuner counts it
  where 38.5, 33.2 in four winds it never flew. A point round that line on
  the plan's shares: 14.0 s, 15.1 with the swing's limit.

### Jerk

The flight looked jerky, as of line segments, not one fast and smooth move.
Ideal rotors, the pull smoothed over 0.1 s:

| Plan | Jerk m/s^3 rms | Disc lean rate deg/s rms | Speed-up/slow-down turns a lap |
| --- | --- | --- | --- |
| as the day began | 48-52 | 137-146 | 27-29 |
| tuned on the rising line | 43-46 | 123-133 | 20-23 |
| closed C2 cubic spline through the gates' middles, by chord length (16.4, 15.5 s) | 43-44 | - | 20-26 |
| monotone cubic heights | 47-50 | - | - |

The line's joints are not the cause: the spline flies 0.04 m about the frame
where 0.5. The tuner's crossings lie 8.0 deg rms off the way from the gate
before to the one after, 13.0 off the gates' own headings: it had turned the
line toward a spline's. The plan jerks: a burst of speed and a brake a gate.

- Monotone slopes through the gates (Fritsch-Carlson), plan and
  crossings searched again on them, 45 generations of 20: 16.3 and 15.5 s
  where the secant's 16.1 and 15.3, a gate 0.38 m off where 0.45; 31.9 as the
  tuner counts it where 31.6; it had stretched them a quarter, toward the
  secant's. The secant's line goes 0.03 m under its lowest gate and 0.07
  over its highest: kept.

### Pull turn rate

From a hover, asked a speed rising 20 m/s^2 along its nose
then falling as fast, every 0.9 s: the frame is 5-12 m/s under its ask when
the next turn comes (half a second of its pull a turn) and 1-2 m off its
height; the tilt's miss answered to 1.2 rad where 0.6: 2.4-7.5; its loop at
(140, 20) and 1.5: 1.8-5.6. The rotors never above 97 % of their top in it.
On the laps they give 37 % of their thrust, none ever at its top.

## Thermal limits

- On the FETs' whole SOA (FINDINGS): one course on four boards no
  longer meets its envelopes - all of their pull, 14.8 and 14.3 s, stuffy
  alike. The flight's own cut keeps the kelvins it was measured in: SPEND
  0.3 of a 100 K span is 0.2 of the junction's 150.
- WEP past the SOA: thermal op 14 holds a board's thermal derate at one, 2 s
  at most, the trip on its record's ceiling standing (MINOR 25). No flight
  takes it: each board's hold is its bound.
- A search with the floor in its cost, eight winds and two sizes on ideal
  rotors: 12.5 and 11.9 s in four winds it never flew, and on four boards
  into the floor every flight, 0.2 s into the second lap (the V into the
  first gate, 2.6 m to 1.7 and up again, level through it at 13 m/s). Ideal
  rotors 25 % quicker to speed struck there too: a knife's edge, not a
  spool. WEP came 0.33 s before the floor, the climb held to its pace's 12
  m/s^2. A search flies the boards as well.
- The boards' observers in flight (four boards, 4 min):

| Network | Air scale | Room C (25) | State |
| --- | --- | --- | --- |
| bench's, flight untold | 0.59-1.33 | 22-30 | STABLE -> UNCERTAIN at 46 s for good |
| airstream (op 15), the wash with rotor speed, flight untold | 0.73-1.05 | - | back at 49 s |
| told the ground speed (op 16, held 1 s) | 0.999-1.002, 0.05 K | - | STABLE at 27 s, kept |
| 4-6 m/s gust, told the ground speed alone | 0.97-0.99, 0.09 K, margin 0.91 at least | - | kept |

## Propellers

- Flown on the 24 V link's 310 rad/s top on the 63 V pack: rotors at
  0.54-0.71 of it between the gates, the frame at 13.0-13.8 m/s on the
  straights, the plan's 13 m/s cap 58-100 % of each; the 20x10's pitch speed
  there 12.5 m/s, the 63100 at 34 % of its 12 000 rpm no-load on its 30 A.
- 16x14s (CT 0.133, CP 0.077, the 20x10's scaled to its pitch), on a stand-in
  at 63 V: 703 rad/s at the 60 A clamp, 11.1 times the frame's weight; 739
  at WEP's 90 A, 66 A of it, the link's ceiling; 691 on a sagged 55 V; pitch
  speed 40 m/s.
- The thrust now falls with the air through each disc, to none at 1.05 of its
  pitch speed. The pack a racer's, 0.04 ohm: at 0.12 the four's 8 kW took
  the bus to half.

## Larger course

`machine.grounds`: 176 m, 16 gates of 2.6-3.4 m, two slaloms, a
9.4 m climb, a 10 m dive, a hall's windows, a 3 m alley. Searched, crossings
off their middles:

| Search | Laps s | On boards s | Tips | SOA |
| --- | --- | --- | --- | --- |
| a small frame's tip at 30 s a metre | 17.9, 17.0 | - | 0.26 m past its margin | - |
| at 150 s a metre, the straights on THROTTLE of the pull | 19.6, 19.2 | 19.5, 19.0 (two winds unflown alike) | all inside | 0.65-0.75 |
| THROTTLE 0.6 | 17 | - | 0.4 m out | 0.94-0.96 |

The narrow gates bind before the motors. A knife-edge, 91 deg for 70 ms, out
of the alley; the page's course on boards passed a gate 1 cm inside its
room, once 1 out.

## Raw

A dive pushed inverted, no level flight kept.

- The race paced at 12 m/s and 12 m/s^2, its discs against 0.5 g at least,
  a crest no faster than 0.511 g let it fall: never past -8.8 m/s^2 down.
- On a sphere about gravity, its discs free, it turned over to 125 deg for a
  0.4 m miss in a bend and fell into the floor; as far down as its line asks
  and 0.26 g past it: whole.
- Rotors asked through the air along their discs: the law learnt their loss
  as a wind against it, until it learnt on the thrust it believes. A dive's
  drag, 9 m/s^2 at 20 m/s, held it 2.5 m over its line until the climb took
  it.
- Rotors lagging 80 ms past their spool struck where the boards, a pass,
  flew whole.
- Searched over boards, winds, sizes and heat: 13.8 and 14.0 s, 18.8 m/s,
  turned over 159 deg, -32.5 m/s^2; on boards 14.0-16.2 s, SOA 0.83-1.00,
  WEP 0.6-1.8 s. Gate 8 crossed 0.11 m higher: the boards' tips 0.10-0.15 m
  inside in four winds, the page's 0.14 past its room's; run again, a stuffy
  room struck gate 6.
- A flight on stand-ins its own: their front end drew its noise from the
  module's random and their heat ran on the wall's clock until a flight's
  first pass. Run twice: SOA 0.83 then 0.91, a stuffy room whole or into gate
  6. Seeded, on the flight's clock from their arming: identical to the digit.
- The sphere eased at a share of what the envelopes left came down slowest
  where they left least: a stuffy room's spent boards asked twice their pull
  for 2 s, into a house. At a share of all of it every flight whole.
- Gate 0 crossed 0.3 m higher, 0.2 under its middle: the boards' tips
  0.07-0.15 m inside in four winds, a stuffy room and a pack at 60 %, every
  flight whole, where a wind's passed 0.02 out; 0.3 or 0.4 under struck or
  passed 0.6 out: the line's shape at the finish carries round.
- The page in the race's steps: stepped as the wall's passes came, a loaded
  page drew its air's eddies on other steps, gate 0 crossed 0.97 m low, 0.15
  past its room, one flight in nine. In 20 ms steps, the rest owed, the
  observers read on the flight's clock: one flight whatever the wall, 14.24
  and 14.98 s, every tip 0.25 m inside.

## Thrusts shared as her torques

The collective and the torques shared over the rotors by her whole-body
law's levels under bounds (`machine.qp.stack`) flew the race 0.3 s a lap
slower on the boards than shared by hand, a strike more in 12 flights. A row
of K_DRAG / K_THRUST, 0.04, fell under the stack's tie-break and the
heading's torque came 15 % short: rows are laid at unit scale. The share is
the core's active set now (aero_share.c).

## On the board's law

Four stand-ins on `drive.c` and their own loops (`ctrl_loop.c`, 1 kHz); the
race, 12 flights (`quad_race.py --suite all`):

| Rotors | Boards laps, s | A gate's frame past its margin, m | SOA | Pull at the least | Struck |
| --- | --- | --- | --- | --- | --- |
| the closed form, the flight's loop a pass | 14.0-14.3, 14.6-15.6 | -0.13 to -0.15 | 0.83-1.00 | 35-48 % | stuffy gate 1 |
| the law, the boards' loops | 13.8-14.3, 14.3-15.3 | -0.14 to -0.17, air 5 +0.03 | 0.78-0.94 | 23-60 % | none |

- The flight's own speed loop, a pass of 20 ms a step: 38 A of braking held a
  pass took a rotor from 197 rad/s through its crossover to 37, the estimate
  lost it on the torque's turn and two boards tripped in the burn, 2.7 m up.
  At 5 ms it held. The loop is the board's.
- A rotor asked under its crossover (66 rad/s) is lost: asked none in a fall
  one went to -5 rad/s and tripped; flown with no least, 6 flights of 6
  struck. Asked its idle at the least and the allocation not told, a gate's
  frame +0.36 m past its margin and the floor struck in a wind; told, the
  row above.
- On I_WEP in their loops the boards stood at 1.00 of their envelopes in
  full tilt: I_MAX kept.
- A stand-in board's pass in one C call (`world_drive_pass`): setpoint
  interpolation, drive execution and state readout unified into a single
  call per rotor pass, removing Python dictionary allocations; the flight
  page runs >20 frames a second.
- The heat's clock in flight (`configure(clock=1)`): flown on the flight's own
  thermal seconds instead of accelerated HASTE=10; boards warmed at rotor idle
  on the floor (100 s) reach CONVERGING before the first flight (hold_amps
  68.6 A), avoiding the UNCERTAIN margin floor. Full tilt holds 58.7 A at
  least where 43.2, legs at 0.81 of envelope where 0.90; race laps 13.9 and
  14.2 s with SOA 0.55-0.62.
- Struck frame crash recovery (`reset`): rotor plant models reinitialized via
  `rotor['rig'].board.drive.model.reset()` before re-spinning to idle; rotor
  restores 0 rad/s without residual negative back-EMF, avoiding secondary floor
  strikes upon restarting the card from rest (`test_struck_it_is_begun_again`).

## The carried camera

A camera under the frame on a stabilised gimbal (`machine.gimbal`), its
picture in the page's corner (`coaxial.graphics.onboard`, C): the page's own
world from the lens, dots over its sky and ground.

- The sight follows the way the frame goes, not its nose: over two laps the
  nose stood over 45 degrees off the way 64 % of the time, 74 at the median,
  up to 180. At rest the nose's, as far as the frame stands upright.
- Level and along its way, a rail's: over two laps the sight trailed its
  bends 12.5 degrees outside the way at the mean, inside it a tenth of
  their time, the gate flown to next in the picture 87 % of the time.
- Given 1.1 of the frame's lean across its way to look into the turn and
  0.35 of it to roll with, 40 degrees at the most, none at rest: 7.7
  degrees inside the way, 86 % of the bends; rolled 9.5 degrees rms in them
  and 17.5 at the most, 4.8 rms on the straights; the gate 91 %.
- A grid's lines end where they would stand under two dots apart: drawn to
  the scenery's far limit they closed to a band three rows thick under the
  horizon.
- 1.0 ms a picture at the median, 36 by 12 cells.
- MuJoCo's renderer on the frame's world, 1.1 ms a shot at 320 x 240: a
  gate's 5 cm bars are under a pixel 10 m off and gone in 36 x 24; the dots
  kept.

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
