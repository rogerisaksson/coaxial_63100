# Findings: flown from compiled streams

A flight's intent as text (`machine.aerolisp`, `machine.intents`), compiled
to flat payloads; a core in C under them with no branch (`aero/`), flown by
`tools/sim/streams.py` on ideal rotors: a pass 2 ms, a rotor's speed 2 900
rad/s^2 at the most and 20 ms behind its ask. The quad's own:
[quad](quad.md).

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
  branches of them; as masks from a sign bit, none: 8 708 B, no float
  compare, 48 conditional branches each a loop's end, three negations on a
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

## The axis asked

The course's own flight on the law (`machine.flying`), the core asked the
same passes: its tilt's torques against the law's 1.5 N m rms.

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
| 30 | 0.22 | 1.10 | 1.66 | 2.64 |
| 90 | 0.31 | 1.35 | 2.24 | 3.53 |
| 150 | 0.37 | 1.12 | 1.79 | 2.76, 4.2 m at the lowest |

The heading's torque is the propellers' drag's, 0.6 N m in a hover: under a
tilt's asks it is what they leave.

## The air

A hover's stream 5 m up through the tour's five kinds of air, 100 s, no
place asked:

| The lean the air may ask | Speed along the floor at the most, m/s | Tilt at the most | Height off |
| --- | --- | --- | --- |
| 0.6 of the lift | 0.35 | 7.7 degrees | 12 mm |
| none | 3.26 | 0.03 degrees | 9 mm |

- A push of 1.5 m/s^2 across, untold: leant 8.69 degrees of 8.69, 0.32 m/s
  gathered while it was learnt, the height's rate 1.5 mm/s.
- The air held as a push alone trails a bend and a tilt: the drag turns
  with the speed, what the rotors lose with their axis. Held as what it is
  made of - a drag's size on the speed squared, the rotors' loss on the air
  along their axis and their thrusts' roots, a push beside them, what the
  speed is off by shared between the three by least squares -, 202's dive
  ends 15.5 m/s ahead of a vacuum's 17.0; with no observer of the speed 9.7.
- It keeps a vacuum: no place is held (7.7 m gone in 100 s; 177 with no
  lean), and a speed is kept against the drag as far as the lean reaches -
  7.0 m/s after the power loop, leant 13 degrees.

## The power loop

`intents.POWER_LOOP` from the floor, each stream its seconds - the catch
till the rotors' axis has stood within 10 degrees of the one asked, every
rate under 3 rad/s, for 50 ms -, the hover asked again:

| Stream | Seconds | Ends |
| --- | --- | --- |
| 201 hover | 3.0 | 1.500 m, level, still; 1.50 m at the most on the way |
| 202 dive, weave | 1.5 | its axis 45 degrees ahead and 6 more into its own wind, 37 either side twice a second, 15.5 m/s ahead, 4.30 m up |
| 203 turn over the nose | 0.08 | every rotor at an end of its reach; 90 N given of 4 asked: the torque first, no rotor under its idle |
| 204 catch | 0.51 | over to 172 degrees, rates to 42 rad/s; its axis 80.4 degrees off up, 16 m/s along the floor stopped to 1.8 |
| 201 again | 4.0 | 1.500 m, still; 1.50 m at the lowest |

- A turn of 0.03 s is caught in 0.38 s, of 0.05 in 0.32, of 0.12 - rates to
  81 rad/s - in 0.58; twelve from 0.03 to 0.12 s in 0.32-0.62, each stood
  at 1.500 m after.
- Flown its injector's 0.45 s whatever the frame did, the catch ended on its
  tilt or 40-150 degrees off by where the turn had it: 80 rad/s is stopped
  and brought round in 0.6 s.
- The axis asked is a place on a sphere and the frame is free about it: the
  catch stands with its nose 41 and its wing 47 degrees up.
- A stream says nothing of when: 204 comes at its injector's time, and a
  turn asked at 314 rad/s is over in 20 ms in a vacuum.

## A card's rows

A card flown a pass a row (`machine.aerokin`, `streams.routine`): the row's
pull in a vacuum as a collective and its rotors' axis's tilt in the heading
the core holds, the turn the law's heading gives that axis as the rates
about the wing and the nose, the tilt's damping on what the frame turns
short of them. Ideal rotors, a pass 20 ms, the core's ten in it; beside it
the law the page flies (`machine.flying`).

| Card | Its rows' payloads | The law |
| --- | --- | --- |
| the routine | 59.1 s; a roll and a flip to 179 degrees, each caught 0.4 off, 0.08 m from its spot, the axis 21 and 14 off at the most on the way; the burn stopped at 0.12 m | 59.8 s; its catches to 176 |
| the course | laps 14.9 and 14.8 s, 15.5 m/s at the most, the nearest tip 0.33 m from a gate's frame | 14.2 and 14.4 s, 16.5 m/s |
| the course, the tour's air of three seeds | 14.8-14.9 and 15.1-15.5 s, tips 0.26-0.33 m | - |

- The law's drag is fed forward at the speed asked; here the air is the
  observers' alone, the frame flown as in none.

## On four boards

The page's four stand-in boards (`terminal.views.quad.flight.flown`), the
core's pass each of the world's 2 ms inside the flight's 20: a board's reach
is the speed its loop is asked, its loop's slew either way, up no faster than
the current it holds gives; its rotor's thrust as given is its estimate's.

| Stream | Ends | Boards |
| --- | --- | --- |
| 201 | 1.500 m | 52 A at the most as it lifts, SOA 0.35 |
| 202 | its axis 51 degrees ahead, 15.2 m/s ahead, 3.98 m | 57 A, SOA 0.65 |
| 203 | every rotor at an end | 60 A, SOA 0.80 |
| 204 | over to 170 degrees, caught in 0.48 s, 79.9 off | 61 A, SOA 0.88, 47 A held at the least, the pack to 60.3 V |
| 201 again | 1.501 m, still | SOA 0.89, 45 A held |

- No drive tripped, no node past its throttle's point: asked all the rotors
  give, each board holds its own clamp and its reach says so.
- Four boards a pass of 2 ms: 0.31 s of this host a flight's second (0.11 at
  20 ms), 53 us a board's step and state.
