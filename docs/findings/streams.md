# Findings: flown from compiled streams

A flight's intent as text (`machine.aerolisp`, `machine.intents`), compiled
to flat payloads; a core in C under them with no branch (`aero/`), flown by
`tools/sim/streams.py` on ideal rotors: a pass 2 ms, a rotor's speed 2 900
rad/s^2 at the most. The quad's own: [quad](quad.md).

## The text

- A stream names what changes and is laid over the one before it. Compiled
  each from nothing tracked, as its author's compiler began, 202 dropped the
  roll and the heading and left the pitch no stiffness.
- A task constraint's height had no place in the payload, its bit alone:
  appended, 124 bytes. Stream 203 alone is its author's 116 bytes and two
  NaN.
- `MAX_SYS_THRUST` is a symbol given with the text, the four's thrust at
  their top a kg: 107.7 m/s^2. Degrees compile to rad.

| Mode | Its target | Its stiffness acts on | Its damping on |
| --- | --- | --- | --- |
| collective | m/s^2 along the rotors' axis | nothing | nothing |
| attitude pitch, roll | the rotors' axis's tilt from up - the nose up, the axis back; the right wing down, the axis right - in the level frame of the heading the core holds | the tilt that takes the axis there | the rate the axis asked itself has not |
| yaw, pitch, roll rate | rad/s about the frame's axis; the yaw's turns the heading held | its miss summed, where no attitude is tracked about that axis | its miss |

204's pitch is damped 100 of its own and 80 of its rate mode's: 1.01 of
critical at 8 000.

## The core

A pass is seven steps in a table: the payload's coming and its blend, the
oscillator, the observers, where the rotors' axis is asked, the torque, the
lift inside its constraints, the share.

- No `if`, `else`, `switch`, `while`, `?`, `&&` or `||` in its sources. Its
  comparisons cast to numbers, the M7's compiler made 40 forward conditional
  branches of them; as masks from a sign bit, none: 9 272 B, no float
  compare, 54 conditional branches each a loop's end, three negations on a
  condition (`copysignf`).
- No sine is taken: the tilt's exponential is its series in the tilt's
  square, the oscillator a point turned by Cayley's map, a cube root three of
  Newton's steps.
- In a vacuum, let go half a degree off, the frame returns as its payload
  says, theta'' = -k theta - d theta':

| k, 1/s^2 | d, 1/s | Damped | Off the closed form at the most, of the miss |
| --- | --- | --- | --- |
| 5 000 | 80 | 0.57 | 0.51 % |
| 1 500 | 25 | 0.32 | 0.26 % |
| 400 | 30 | 0.75 | 0.10 % |
| 8 000 | 180 | 1.01 | 0.54 % |

- The share: 400 wrenches and reaches at random, never past the best of the
  81 sets of rotors held at an end (1.4e-8 of the cost, the weights single
  precision), in 8 passes.
- On the image (device 14, MINOR 34), a frame laid and a pass a request:
  150 442 cycles under Renode, 0.32 ms on the M7 at one a cycle, the same
  within two in each of 25 asks - the power loop's payloads over a frame
  level at rest, tilted and turning, on its back, its rotors' reach shut,
  nothing given. An interrupt's 90 fall in half the passes. Its thrusts are
  the host core's on the reading as the board had it, to the wire's
  millionth of a newton, eight runs of eight: its sources built with no
  multiply fused into an add (`-ffp-contract=off`). Fused - the M7's VFMA,
  none on the host - they stood 1.7e-5 N apart here and 3.6e-4 on CI's
  host, by which passes an interrupt had fallen in; 149 288 cycles then.
  9.3 KB of flash, its device 1.1, 821 B of state.

## The axis asked

The course flown on the law the core replaced, the core asked the same
passes: its tilt's torques against that law's 1.5 N m rms.

| The attitude as | Apart, N m rms | The catch, a nose 80 degrees up |
| --- | --- | --- |
| where up stands in the frame, closed by the least turn's tilt | 0.62: what of that turn is about the rotors' axis no tilt gives - banked 60 degrees a quarter of a miss along the horizon was answered, on its edge none | a wing off stays off |
| the one tilt that takes up there | 0.14 | a wing 3 degrees off asked a roll of 20 about a nose 83 down, of 87 as it came round; a turn of 0.12 s ended 119 degrees off, in the floor |
| the least turn, its part about the rotors' axis taken out as far as up stands off their plane | 0.16 | held, its nose swung 50 degrees either way every 0.35 s, a wing 30 off |
| the rotors' axis in the world, in the level frame of a heading held | 0.14 | caught from every turn tried |

- The heading held is the frame's own as it first stood, its tilt taken
  out, turned on by the yaw rate asked: the axis asked is a place in the
  world whatever the nose does, and a hover is stood from 12 rad/s about the
  rotors' axis a quarter turn off.
- A dropped mode leaves its axis of the frame free: the other's miss is what
  its own turn closes of the offset across it, its sign the axis's own on
  its back.
- The lift over the heading in the share: the heading over it, the hover
  after the catch stood 0.3-1.7 m off its height in 4 turns of 12.
- On a lap the raw force asked, the loops' own on it, turns 7.2-9.5 rad/s
  rms and 55-72 rad/s at the most; the frame 4.6 rad/s, its axis 29
  degrees rms off that force. A raw turn fed forward struck every flight.
  Slewed on its braking curve inside the reach (8.0 rad/s, 200 rad/s^2,
  slope 10 1/s) with its turn rate in the payload fed forward onto pitch and
  roll rates: the slewed force turns 4.0 rad/s rms (peak 7.9 rad/s bounded
  within reach), the frame 4.5 rad/s, and the axis miss on a lap drops to
  10.2 degrees rms (34.3 peak where 65); 12 flights of 12 whole in the race
  suite.


## The rotors' reach

A rotor's speed 2 900 rad/s^2 either way is 0.27 N a pass in a hover: the
tilt's torque changes 1 800 rad/s^3 at the most on half of it. 5 000 on the
miss is rendered whole to 0.19 degrees of it.

| As flown | 202's blend to 45 degrees | The turn and the catch |
| --- | --- | --- |
| the payload's stiffness and damping, the share alone bounding them | over, every rotor at an end every pass | - |
| past the reach the miss closed at the fastest rate the rotors stop from, (j e^2)^(1/3), that rate come to at sqrt(2 j x) | nose 44 degrees down, wings 32 either way of 35 | the roll and the heading left none of the share by a rate asked 314 rad/s; into the floor |
| each row asked no further than the reach takes it alone | as above | the lift's all on its back: 0.16 m from 2.9 |
| the lift as far as the axis points where asked | as above | caught 80 degrees off; the hover after swung 40-70 degrees for good, 5-8 rad/s about its axis |
| its own inertia's turn about the tilt's axes carried round with the frame's heading, not taken out | as above | stood again from turns of 0.05 to 0.12 s |

- On slewed rotors 45 degrees at 5 000: there in 0.18 s, a bounded jerk's
  0.24 s on half the reach, 0.85 degrees past.
- A tilt's torque the frame's turn about the rotors' axis carries round is
  asked no larger than the reach follows it, j over that rate.
- The lift's row weighs as far as the axis points where asked: at its own
  weight a hover asked from a quarter turn off, 12 rad/s about the axis, was
  given the heading's torque the wrong way round (0.29 N m of -7.6 asked)
  and struck the floor; now it keeps its height.

## Upsets

A hover asked 5 m up, the rotors' axis off up and the frame turning about
it; when it stands, s:

| Off, degrees | 0 rad/s | 6 | 9 | 12 |
| --- | --- | --- | --- | --- |
| 30 | 0.22 | 1.10 | 1.66 | 2.59 |
| 90 | 0.31 | 1.34 | 2.20 | 3.47 |
| 150 | 0.41 | 1.01 | 1.66 | 2.76, 4.2 m at the lowest |

The heading's torque is the propellers' drag's, 0.6 N m in a hover: under a
tilt's asks it is what they leave.

## The air

A hover's stream 5 m up through the tour's five kinds of air, 100 s, no
place asked:

| The lean the air may ask | Speed along the floor at the most, m/s | Tilt at the most | Height off |
| --- | --- | --- | --- |
| 0.6 of the lift | 0.35 | 7.6 degrees | 11 mm |
| none | 3.26 | 0.03 degrees | 9 mm |

- A push of 1.5 m/s^2 across in no air, untold: leant 8.69 degrees of 8.69,
  0.32 m/s gathered while it was learnt, the height's rate 1.5 mm/s.
- The air is held as a wind: the frame's drag on its speed through it and
  what its rotors lose to it along their axis the frame's own sizes, a push
  the quick one, laid to the wind by damped least squares at 5 rad/s. A lap
  in 3.3 m/s rms: held within 0.12, 0.07 and 0.20 m/s rms (x, up, z); a
  hover in 4 m/s 0.01 m off its spot, in gusts to 7 m/s 0.11.
- Held as a drag's size on the ground's speed and a push, a crosswind's
  drag trailed a bend: four boards in two winds of six crossed the dive's
  foot 0.7-0.8 m wide, a gate's frame 0.17-0.27 m past its margin. As a
  wind: inside it in fourteen winds, six of them alike at a learning rate
  of 2 to 20 rad/s.
- A force that is no wind's, 3 N on a hover in still air, is held as the
  wind that makes it, 5.7 m/s: 0.10 m off its spot at the most, its height
  within 2 cm.
- What the rotors lose is made up along their own axis, whatever they are
  asked. Along the axis asked, by how far the frame pointed there: a frame
  leant against a wind sank 1.8 m/s in 6 s, and a dive's thrust was short -
  the course's laps 14.6 and 14.7 s where 14.3 and 14.5.
- 202's dive ends 15.3 m/s ahead of a vacuum's 17.0.
- It keeps a vacuum: no place is held (8.9 m gone in 100 s; 177 with no
  lean), and a speed is kept against the drag as far as the lean reaches -
  9.6 m/s after the power loop, leant 23 degrees.

## The power loop

`intents.POWER_LOOP` from the floor, each stream its seconds - the catch
till the rotors' axis has stood within 10 degrees of the one asked, every
rate under 3 rad/s, for 50 ms -, the hover asked again:

| Stream | Seconds | Ends |
| --- | --- | --- |
| 201 hover | 3.0 | 1.500 m, level, still; 1.50 m at the most on the way |
| 202 dive, weave | 1.5 | its axis 48 degrees ahead at the mean, 36 and 39 either side twice a second, 15.3 m/s ahead, 3.32 m up |
| 203 turn over the nose | 0.08 | every rotor at an end of its reach; 92 N given of 4 asked: the torque first, no rotor under its idle |
| 204 catch | 0.59 | over to 176 degrees, rates to 42 rad/s; its axis 81.0 degrees off up, 16 m/s along the floor stopped to 4.0 |
| 201 again | 4.0 | 1.500 m; 1.08 m at the lowest |

- A turn of 0.03 s is caught in 0.36 s, of 0.05 in 0.32, of 0.12 - rates to
  85 rad/s - in 0.47; twelve from 0.03 to 0.12 s in 0.29-0.66, nothing
  struck, 0.65 m at the lowest.
- Flown its injector's 0.45 s whatever the frame did, the catch ended on its
  tilt or 40-150 degrees off by where the turn had it: 80 rad/s is stopped
  and brought round in 0.6 s.
- The axis asked is a place on a sphere and the frame is free about it: the
  catch stands with its nose 41 and its wing 47 degrees up.
- A stream says nothing of when: 204 comes at its injector's time, and a
  turn asked at 314 rad/s is over in 20 ms in a vacuum.

## A card's rows

A card flown a pass a row (`machine.aerokin`, `streams.asked`, `flew`): the
row's pull in a vacuum as a collective and its rotors' axis's tilt in the
heading the core holds, the turn the law's heading gives that axis as the
rates about the wing and the nose, the tilt's damping on what the frame
turns short of them. The page's flight and the race fly it
(`flight.step`, `quad_race.py`); the law they flew before is gone, its
numbers beside:

| Card | Its rows' payloads | The law before |
| --- | --- | --- |
| the routine, ideal rotors | 59.3 s; a roll and a flip to 179 degrees, each caught 0.5 off, 0.09 m from its spot, the axis 22 and 14 off at the most on the way; the burn stopped at 0.12 m | 59.8 s; its catches to 176 |
| the course, ideal rotors | laps 14.3 and 14.5 s, 16.7 m/s at the most | 14.2 and 14.5 s |
| the race, 12 flights (`--suite all`) | 29.1 as it counts; every flight whole, a gate's frame 0.04-0.19 m inside its margin, 0.40-0.45 m about the frame | 29.2; 0.03-0.17 m inside, 0.37-0.44 |
| the race in fourteen winds, 34 flights | all whole; in the winds laps 14.3 and 14.5-14.8 s, a gate's frame 0.09-0.17 m inside | three winds: 14.3 and 15.1-15.2 s |

- The law before stepped its rotors a pass of 20 ms at a time, a pass's
  whole change of speed at its start: twice the torque's impulse while they
  slew. On the core's passes of 2 ms a tilt of 40 degrees came 24 ms later,
  ideal rotors and stand-in boards alike, and the lap's seam - 2.4 m down in
  6 - was crossed 0.51 m high. The tilt's stiffness 300 where 201: 0.06 m
  low.
- The line's crossings were searched on that law's misses of its line, 0.4
  m at three gates. Laid again by flight - each moved by what the frame
  crossed its gate off that law's own crossing - they stood 0.01 m rms off
  in three flights.
- A bend's pull, up and down as across, is asked ahead by what the frame
  trails one (`course.trailed`): its tilt's damping over its stiffness, 57
  ms, its rotors' lag, 20, and the pass its row is come to over - half of
  it while a row was stepped in whole. Searched then it was 89 ms, 28.88
  with a gate's frame 0.05 m inside its margin: derived, 28.87 and 0.11,
  fourteen winds whole and 0.07 inside at the least.
- A bend's pull goes from one side to the other in half a turn of the
  tilt's own loop at the fastest (`course.swing`), 0.208 s: where its step
  stands at the far side. Searched it was 0.36 s. Gridded, 28.09 at 0.18
  with every flight whole, and at 0.15 a gate's frame 0.17 m past its
  margin, 33.07. On 0.208 with the crossings laid again by flight (0.01 m
  rms off their marks): 28.25, a gate's frame 0.08-0.17 m inside; fourteen
  winds whole, 0.03 inside at the least, the boards' laps 13.9 and
  14.1-14.3 s where 14.3 and 14.5-14.6.
- The sphere stays searched: on 0.40 of the clamp's pull a gate's frame is
  0.07 m past its margin, on 0.45 0.31, and on 0.55 three flights of twelve
  are not whole.
- No clause in a pass above the core: a card's step is a table laid once
  and a move by masks (`aerobatics.laid`, `fly`), the lap's plan a table a
  sphere computed beforehand and read where the frame is
  (`course.braked`), 33 functions held to it (`test_quad_streams.py`). Run
  back 40 m a pass it was 357 us of this host; read, 64, the 1 441 rows of
  two laps the same to the bit and the race flight for flight. A row left
  is the one flown last: a lap's end is told a pass later, 0.02 s on the
  race's count.
- The plan is told no wind and the frame's drag is not fed forward: the air
  is the core's alone.
- The routine's fall is paced, 10 m/s (`aerobatics.STOP`): let fall at any
  pace the rotors stood at their idle with the drag made good, a wind took
  the frame 15 m off its spot and the burn found it leant 60-100 degrees -
  on the floor at 11 m/s, seven winds of seven. Paced: stopped at
  0.100-0.101 m in eight winds, 4.4 m off its spot at the most on the way.

## On four boards

The page's four stand-in boards (`terminal.views.quad.flight.flown`), the
core's pass each of the world's 2 ms inside the flight's 20: a board's reach
is the speed its loop is asked, its loop's slew either way, up no faster than
the current it holds gives; its rotor's thrust as given is its estimate's.

| Stream | Ends | Boards |
| --- | --- | --- |
| 201 | 1.500 m | 52 A at the most as it lifts, SOA 0.35 |
| 202 | 15.4 m/s ahead, 3.33 m | 58 A, SOA 0.64 |
| 203 | every rotor at an end | 70 A, SOA 0.82 |
| 204 | over to 171 degrees, caught in 0.50 s, 79.8 off | 67 A, SOA 0.84, 53 A held at the least, the pack to 58.6 V |
| 201 again | 1.500 m | SOA 0.84, 51 A held |

- No drive tripped, no node past its throttle's point: asked all the rotors
  give, each board holds its own clamp and its reach says so.
- A rotor's copper round the course, a tick of its loop at a time: 27.5 A
  rms where 19.8 holds its speeds - of its heat 12 % a ripple between two
  asks, 23 % the core's passes within a row, 13 % the rows'. An ask written
  whole a pass, two ticks of the loop, is a spool to the loop's
  feed-forward and the tick after its end: 6.3 A apart at 500 Hz on 300
  rad/s^2, 29 on 1 400. As rows a tick long along its line
  (`flight.asked`): no ripple, 26.6 A rms, the least a board held 54.8 A
  where 52.0; full tilt's four 2.2-3.7 A apart at the most where 4.4-5.9;
  the race 28.24, whole. The loop ticked once a pass did the same, and a
  wreck's restart ran two rotors of four backwards.
- A row stepped in whole each 20 ms is a staircase to the core, its rotors
  at their slew after each step: 23 % of the copper's heat. Come to over
  its pass (`aerokin.step`'s `blend_duration_ms`, the core's own blend):
  22.8 A rms a rotor where 26.6, 5 % of its heat the passes' within a row
  and 20 the rows', the least a board held 65.6 A where 54.8, the pack at
  82 % after two laps where 80. The race 28.24, 28.23 in fourteen winds,
  34 flights of 34 whole, a gate's frame 0.01 m inside its margin at the
  least where 0.03, the crossings 0.02 m rms off their marks; the boards'
  rows 90-92 % of the clamp's pull at the least in every wind where 72-85
  (the law before 89-91), 78 in the stuffy room where 63, SOA 0.70-0.83
  where 0.80-0.88, laps 13.9 and 14.1-14.3 s.
- Four boards a pass of 2 ms: 0.20 s of this host a flight's second (0.28
  with a board's whole state read and its propeller's torque laid a pass;
  the law before, a pass of 20 ms, 0.12).
- On the page the flight's thread gives way at each of 40 000 calls a
  second: at Python's 5 ms a turn it waited on the drawing's, a flight's
  second took 1.02 of the wall's, its clock 21 % behind. At 0.2 ms: 0.51,
  none behind, 16 frames a second where the law before drew 17.8.
- `course.BRAKE` and `course.GO` searched on the core (`quad_race.py --search`):
  0.735 and 0.708 (cost 28.06 where 28.24), every flight whole (100 %),
  a gate's frame -0.03 m past its margin; boards' laps 13.8 and 14.1 s
  (verified on unseen seeds 6, 7, 8: 28.05, laps 13.8 and 14.0-14.1 s),
  against 14.3-14.8 s.
