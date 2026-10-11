# Findings: flight

Her soles off the floor on the whole-body stack ([wbc](wbc.md)): released,
asked to their landing, taken again.

`machine/flight.py` stands between a law's ask and the stack's step;
`tools/sim/hop.py` throws her; tests/test_flight.py holds it.

- The stack and the loop both solve a step with no sole standing: no force,
  her centre of mass at -9.81 m/s^2, the drives at 0 N m. What they are asked
  was missing: asked to stand on both soles 0.2 m over the floor the stack
  lays 153 N a sole on nothing.
- Released: every sole meant to stand, together, once all have borne under
  15 N for 4 ms; one alone only as lost, 50 ms with the stack laying over
  30 N on it. Taken again bearing 20 N, while she rises no faster than
  0.1 m/s. Released one at a time as each rose, or one taken again as it
  brushed the floor 2 ms into her rise, the other sole was left the whole
  throw - 648 N for 50 ms - and she rolled over, 19 deg.
- A released sole is asked to its landing: under her capture point, drawn in
  to her leg's reach (`reach.land`), the knee 25 deg. The stack brakes a knee
  going out 9.2 rad/s to rest in 40 ms at its clamp, 108 N m.
- With no sole standing no turn is asked. Asked - even as they were, their
  rates alone bled - the stack spun her legs to hold her trunk still: the
  knees out at 3 rad/s with no torque in them, on their stops at the
  landing, 1.9 kN, down.
- The throw. Her height asked down alone, the posture held her knees: 10 mm
  of 60; crouched by the posture too (hip, knee and ankle by their terms),
  her pelvis straight down over her ankles, her centre of mass lay 26 mm
  behind her soles' middle. Her trunk asked upright through the throw, the
  stack put her weight on her heels' edges - 16 mm past the soles' margin -
  to turn her legs under it: 4.7 kg m^2/s about her at lift-off, 14.9 deg
  over at the landing; her trunk unasked, 5.3 deg.

| Let down, thrown, for | On | Flown | Under her soles | Struck | Tilt | Stood |
| --- | --- | --- | --- | --- | --- | --- |
| 0.10 m, 10 m/s^2, 0.15 s | the python stack | 0.149 s | 21.2 mm | 1 898 N | 7.4 deg | yes |
| 0.10 m, 14, 0.15 | python | 0.165 | 26.6 | 2 095 | 5.3 | yes |
| 0.12 m, 18, 0.2 | python | 0.217 | 46.2 | 2 637 | 7.9 | yes |
| 0.10 m, 14, 0.15, three in a row | python | 0.504 in all | 27.6 | 2 150 | 12.1 | yes |
| 0.10 m, 14, 0.15, her soles never released | python | 0.088 | 8.7 | 515 | 4.5 | yes |
| 0.10 m, 10, 0.15 | the loop in C | 0.154 | 21.8 | 1 792 | 10.6 | yes |
| 0.10 m, 14, 0.15 | C | 0.177 | 31.4 | 1 851 | 12.8 | yes |
| 0.10 m, 10, 0.15, three in a row | C | 0.465 in all | 23.1 | 1 890 | 12.1 | yes |
| 0.12 m, 18, 0.2 | C | 0.236 | 55.7 | 2 592 | 25 | no, at the landing |
| 0.10 m, 14, 0.15, three in a row | C | - | 36.6 | 2 362 | 25 | no, the second |

- The loop in C, two changes. A turn of all zeros is unasked. The soles'
  margins are laid for standing soles only: laid for a sole in the air too,
  their weight set the level's scale and the levels below came in as strong
  as a swinging sole's own ask - with no sole standing it met 33 of 115
  m/s^2 where the python stack met 110; now 110, 0.006 m/s^2 apart. Its walk
  (8 s at 0.5 m/s: 2.47 m, 0.41 m/s, 259 J/m, 15 steps, tilt 1.8) and four
  shoves (38 N from her left, 60 from ahead and behind, 80 from her right)
  are as before to the digit.
- Open. The strike, 1.8-2.6 kN at 0.8 m/s: not her height's law's (at a
  quarter of it 1 735 N; the torque as asked under it 1 414), nor the sole's
  speed onto the floor (eased onto the floor it left over the last 30 mm,
  2 244 N). Standing on one sole the loop meets a third of a swinging sole's
  ask where the python stack meets most (115 m/s^2 asked: 13 against 109).
  A hop on one sole is untried.
- tests/test_wbc_core.py stood in no suite list: the loop's suite ran in no
  gate. It is in the default set now, 21 of 21.
