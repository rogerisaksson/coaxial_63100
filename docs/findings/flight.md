# Findings: flight

Her soles off the floor on the whole-body stack ([wbc](wbc.md)): which
stand, the others asked to their landing.

`machine/flight.py` stands between a law's ask and the stack's step;
`tools/sim/hop.py` throws her; tests/test_flight.py holds it.

- The stack and the loop both solve a step with no sole standing: no force,
  her centre of mass at -9.81 m/s^2, the drives at 0 N m. What they are asked
  was missing: asked to stand on both soles 0.2 m over the floor the stack
  lays 153 N a sole on nothing.
- Which soles stand is told from one pass, nothing kept
  (`flight.standing`): a sole stands that bears 15 N; one that bears
  nothing stands within 2 mm of the floor beside one that bears, or with
  her centre of mass under 0.1 m/s. What fell on the way:
  + By its load alone: the first sole to lighten was lifted and the other
    left the whole throw - 648 N for 50 ms -, 19 deg over.
  + Within 2 mm alone: the stack pressed a sole 1 mm over the floor down
    onto it as she left, 648 N: 23 deg over on the loop in C, down at the
    second of three hops on the python stack.
  + All soles or none while she rises over 0.1 m/s: the one she stood on
    after a landing flipped at that speed for 0.2 s, 21.2 deg over three
    hops.
  + An unloaded sole stood near the floor as she falls too: three hops
    stood at 2 mm and fell at 0.5; with her at rest the numbers are the
    same at 0.5, 1 and 2 mm.
  + Her centre of pressure brought 25 mm inside the standing soles' box:
    three hops 13.1 deg over against 9.8. Not there.
- Her soles' loads cross 15 N twice in the 3 ms they leave the floor in:
  no sole stands for 1 ms, both for 2, none from there. She flies while
  neither bears 15 N; `tools/sim/hop.py` counts a flight of 20 ms or
  more.
- A sole that does not stand is asked to its landing: under her capture
  point, drawn in to her leg's reach (`reach.land`), the knee 25 deg. The
  stack brakes a knee going out 9.2 rad/s to rest in 40 ms at its clamp,
  108 N m.
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
  over at the landing; her trunk unasked, 5.4 deg.

| Let down, thrown, for | On | Flown | Under her soles | Struck | Tilt | Stood |
| --- | --- | --- | --- | --- | --- | --- |
| 0.10 m, 10 m/s^2, 0.15 s | the python stack | 0.149 s | 21.5 mm | 1 945 N | 7.1 deg | yes |
| 0.10 m, 14, 0.15 | python | 0.165 | 26.8 | 2 101 | 5.4 | yes |
| 0.12 m, 18, 0.2 | python | 0.217 | 46.3 | 2 639 | 8.0 | yes |
| 0.10 m, 14, 0.15, three in a row | python | 0.508 in all | 28.2 | 2 223 | 11.5 | yes |
| 0.10 m, 14, 0.15, her soles never released | python | 0.102 | 8.7 | 515 | 4.5 | yes |
| 0.10 m, 10, 0.15 | the loop in C | 0.156 | 23.4 | 1 961 | 10.3 | yes |
| 0.10 m, 14, 0.15 | C | 0.175 | 30.8 | 2 018 | 13.6 | yes |
| 0.10 m, 10, 0.15, three in a row | C | 0.474 in all | 24.5 | 2 012 | 10.3 | yes |
| 0.12 m, 18, 0.2 | C | 0.236 | 56.2 | 2 603 | 25 | no, at the landing |
| 0.10 m, 14, 0.15, three in a row | C | - | 37.7 | 2 038 | 25 | no, the second |

- The loop in C, two changes. A turn of all zeros is unasked. The soles'
  margins are laid for standing soles only: laid for a sole in the air too,
  their weight set the level's scale and the levels below came in as strong
  as a swinging sole's own ask - with no sole standing it met 33 of 115
  m/s^2 where the python stack met 110; now 110, 0.006 m/s^2 apart. Its walk
  (8 s at 0.5 m/s: 2.47 m, 0.41 m/s, 259 J/m, 15 steps, tilt 1.8) and four
  shoves (38 N from her left, 60 from ahead and behind, 80 from her right)
  are as before to the digit.
- Open. The strike, 1.9-2.6 kN at 0.8 m/s: not her height's law's (at a
  quarter of it 1 735 N; the torque as asked under it 1 414), nor the sole's
  speed onto the floor (eased onto the floor it left over the last 30 mm,
  2 244 N). Standing on one sole the loop meets 15 of 115 m/s^2 of a sole
  asked back where the python stack meets 97: her form's weight on the
  turns' level, which her walk's 259 J/m is ([wbc](wbc.md), The margins
  soft).
  A hop on one sole is untried. Her throw is asked as an acceleration for a
  time: it is asked 30 ms yet with her soles off the floor; a spring to her
  legs' length ends there by itself.
- tests/test_wbc_core.py stood in no suite list: the loop's suite ran in no
  gate. It is in the default set now, 21 of 21.
