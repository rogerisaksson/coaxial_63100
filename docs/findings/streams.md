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
| attitude pitch, roll | the nose's and the wing's rise: where up stands in the frame | the turn that takes up there | the rate the target itself has not |
| yaw, pitch, roll rate | rad/s about the frame's axis | its miss summed, where no attitude is tracked about that axis | its miss |

204's pitch is damped 100 of its own and 80 of its rate mode's: 1.01 of
critical at 8 000.

## The core

A pass is seven steps in a table: the payload's coming and its blend, the
oscillator, the observers, where up is asked, the torque, the lift inside its
constraints, the share.

- No `if`, `else`, `switch`, `while`, `?`, `&&` or `||` in its sources. Its
  comparisons cast to numbers, the M7's compiler made 40 forward conditional
  branches of them; as masks from a sign bit, none: 7 268 B, no float
  compare, 42 conditional branches each a loop's end, three negations on a
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
| 30 | 0.22 | 1.10 | 1.66 | 2.63 |
| 90 | 0.31 | 1.39 | 2.50 | never: 15.8 rad/s about its axis after 4 s, its height kept |
| 150 | 0.38 | 1.21 | 1.83 | 2.68, 4.2 m at the lowest |

The heading's torque is the propellers' drag's, 0.6 N m in a hover: under a
tilt's asks it is what they leave.

## The air

A hover's stream 5 m up through the tour's five kinds of air, 100 s, no
place asked:

| The lean the air may ask | Speed along the floor at the most, m/s | Tilt at the most | Height off |
| --- | --- | --- | --- |
| 0.6 of the lift | 0.30 | 7.8 degrees | 12 mm |
| none | 3.26 | 0.03 degrees | - |

- The speed's observer at 10 rad/s; at 3 the speeds were 0.51-0.83 m/s a
  kind, at 30 0.24-0.59. The rates' at 20, 40 or 80: the same.
- A push of 1.5 m/s^2 across, untold: leant 8.69 degrees of 8.69, 0.32 m/s
  gathered while it was learnt, the height's rate 1.5 mm/s. The push taken
  in the frame as it stands, not as it is asked: 8.42 degrees and 0.6 m/s
  up after 3 s.
- It keeps a vacuum: no place is held (30 m gone in 100 s), and a speed is
  kept against the drag as far as the lean reaches - 11 m/s after the power
  loop, leant 31 degrees.

## The power loop

`intents.POWER_LOOP` from the floor, each stream its seconds, the hover
asked again:

| Stream | Seconds | Ends |
| --- | --- | --- |
| 201 hover | 3.0 | 1.500 m, level, still; 1.50 m at the most on the way |
| 202 dive, weave | 1.5 | nose 44 degrees down, wings 32 either way twice a second, 10.3 m/s ahead, 2.67 m up |
| 203 turn over the nose | 0.08 | every rotor at an end of its reach; 46 N given of 4 asked: the torque first, no rotor under its idle |
| 204 catch | 0.45 | over to 164 degrees, caught 79.4 off of 80, rates to 25 rad/s |
| 201 again | 4.0 | 1.501 m, still; 1.49 m at the lowest |

- A turn of 0.05 s: caught 78.6 off, 1.09 m at the lowest; 0.12 s: 79.3,
  rates to 41 rad/s, 0.60 m.
- The tilt asked is a point on a sphere and the heading is free: the catch
  stands on it with its wing 21 degrees down, closed about the rotors' axis
  by the drag's torque alone.
- A stream says nothing of when: 204 comes at its injector's time, and a
  turn asked at 314 rad/s is over in 20 ms in a vacuum.

## On four boards

The page's four stand-in boards (`terminal.views.quad.flight.flown`), the
core's pass each of the world's 2 ms inside the flight's 20: a board's reach
is the speed its loop is asked, its loop's slew either way, up no faster than
the current it holds gives; its rotor's thrust as given is its estimate's.

| Stream | Ends | Boards |
| --- | --- | --- |
| 201 | 1.500 m | 52 A at the most as it lifts, SOA 0.35 |
| 202 | nose 47 degrees down, 10.4 m/s ahead, 2.70 m | 54 A, SOA 0.57 |
| 203 | every rotor at an end | 60 A, SOA 0.67 |
| 204 | over to 161 degrees, caught 78.5 off | 61 A, SOA 0.89, 46 A held at the least, the pack to 59.8 V |
| 201 again | 1.500 m, still | SOA 0.89, 45 A held |

- No drive tripped, no node past its throttle's point: asked all the rotors
  give, each board holds its own clamp and its reach says so.
- Four boards a pass of 2 ms: 0.31 s of this host a flight's second (0.11 at
  20 ms), 53 us a board's step and state.
