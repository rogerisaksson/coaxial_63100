# Findings: wrench

Her whole body's step without an active set (`machine/wrench.py`), in the
stack's place ([wbc](wbc.md)); `tools/sim/wbc.py --wrench` and
`tools/sim/hop.py --wrench` run it, tests/test_wrench.py holds it.

- The loop in C spends 6.5 us of a 1 570 us tick on her body's terms - the
  pose, M, the bias, two Jacobians (this host, -O2): 0.4 %. The rest is five
  dense levels over 71 variables, each with its active set.
- The step has 21 unknowns: six a sole - the wrench it takes of the floor
  where it stands, its own acceleration where it does not - and her upper
  body's nine accelerations. Every joint follows from them: a leg through
  its sole by its six joints' damped inverse, the pelvis by the six base
  rows of her dynamics. One weighted least squares over the 21, four times:
  each wrench clipped into its sole's bounds (pressing, its friction's
  pyramid, its centre of pressure 3 mm inside its edges), what was clipped
  pinned for the next; her height's band the same way. The drives' clamps
  clip the torques after it. No other iteration, no branch on a leg.

| Measure | The step | The python stack | The loop in C |
| --- | --- | --- | --- |
| A step standing | its torques within 0.16 N m of the stack's | - | - |
| Standing 10 s | tilt 0.3 deg, creep 0.62 mm/s, 0.05 of a clamp | the same | - |
| Shoved from 8 ways, stood of 8 at 38 / 60 / 80 / 100 / 120 N | 8 / 6 / 4 / 2 / 0 | 8 / 7 / 4 / 4 / 0 | 8 / 7 / 5 / 4 / 0 |
| WEP over those polars | 0 / 4.0 / 3.9 / 4.6 / 5.0 s | 0 / 2.4 / 11.6 / 10.1 / 7.3 | 0 / 1.4 / 5.4 / 10.3 / 3.3 |
| The walk as built before her loop, 8 s at 0.5 m/s asked | 16 steps, 1.90 m, 0.32 m/s, 1 400 J/m, tilt 2.5 deg | 16, 1.99 m, 0.33, 761, 1.3 | 15, 2.47 m, 0.41, 259, 1.8 |
| Hops at 10, 14, 18 m/s^2, under her soles | 14.4, 17.9, 41.6 mm | 16.3, 21.5, 46.4 | 16.3, 21.5, 47.7 |
| Their tilt, and three in a row's | 9.3, 2.5, 11.4, 5.5 deg | 6.1, 4.9, 6.1, 5.7 | 5.5, 4.6, 15.2, 6.6 |
| About her at lift-off, 14 m/s^2 | 0.3 kg m^2/s | 3.1 | 2.7 |
| A pass in python | 1.1-1.3 ms | 2.2-2.6 ms | - |

Shoves at her stand, no step taken, the PD under each, the gearboxes with
their play; the step in C stands the python step's 8 / 6 / 4 / 2 / 0.

- What each change bought:
  + Solves in a row - her legs slaved to the pelvis, the pelvis on its asks,
    what that needs of the floor spread over her soles, the pelvis again on
    what they give: the stand as the stack's; 38 N felled her 3 times of 8
    and the walk went down at its second step, what the floor could not
    give landing on the pelvis alone, over 200 rad/s^2.
  + The soles' wrenches the unknowns of one least squares: 38 N 8 of 8, 60
    N 4, three hops of four.
  + A sole in the air an acceleration in its wrench's place: every hop
    stood. Asked exactly, a swinging sole's ask moved the pelvis 340
    rad/s^2 in a pass.
  + Her upper body's accelerations unknowns too: the walk 7 steps where 4.
  + A swinging sole as heavy as the turns: the walk whole; at a third of
    them down at its 7th step. Her standing knees' posture at 3: 60 N 7-8
    of 8 where 6, a hop 2.5 deg over where 5.0.
  + A swinging knee's fold asked as on the stack and a knee sprung back
    from straight: the walk 4 steps where 2; a swung leg straight at the
    knee had been asked 17 000 rad/s^2.
  + A leg's inverse damped 0.02: 60 N 6 of 8 where 4 at 1e-3; at 0.05 a
    hop left the floor 8 mm, at 0.1 not at all.
- A clipped moment is pinned in all three of its world parts, one of them
  moved about the sole's own axes: on its ball a sole's room across goes
  with its room along.
- Striding (`ask['strides']`), what the floor cannot give is not traded
  again: the rows' answer with the floor unbounded is brought to the faces
  her soles are past - a part its share of the sole's push, about the
  sole's own axes, no part pinned as a value - by the least change of
  `full' M full`, a swinging sole's ask held (`_taken`); her height's
  band before the faces. Her walk under it in
  [orthokinematics](orthokinematics.md). Python's alone yet.
- Those faces with the rows traded, standing: shoved, 7, 5, 3, 2 and 0 of
  8 where 8, 6, 4, 2 and 0; the rounds stand as they were.
- Her height follows her standing knees' posture inside the band, as on
  the stack: asked alone her knees went straight in 0.35 s, 6 to 0 deg, and
  the leg's inverse with them.
- An answer past the band after the rounds - a contact let go in the last,
  its solve never held to it - is held to its edge in up to 2 rounds more,
  not striding (`PAST`, `WBC_PAST`): on the bed 417 of 4046 passes ended
  past it, 2.16 m/s^2 at the median and 4.34 at p90, then 0.01 and 0.22;
  the chair 3 to 0; her walk and her stand none of 4667.
- A rounds' solve with no inverse is solved again ridged on its free
  unknowns (`RIDGE`, 1e-12 of its largest diagonal): falling on a told ramp,
  two bearing points 16 mm apart left it rank 21 of 27, cond 2.9e19, their
  split said by no row, and the step raised. `wbc_wrench.c` has no contacts.
- Open. 80 N and over: 4, 3 and 1 of 8 where the stack 7, 8 and 6. Her
  walk's 1 400 J/m: the clamps clip after the solve. The step in C and its
  tick on the M7. Stops but the knees'.

## In C

`wbc/src/wbc_wrench.c` and `wbc_cone.c`, the same rows on her chain in C;
`tools.cores.wbc.wrench_step`, `tools/sim/wbc.py --core --wrench`;
tests/test_wbc_core.py holds it to the step.

| | wbc_wrench.c | The loop in C (wbc_stack.c) |
| --- | --- | --- |
| A tick on this host, -O2 | 16 us standing, 23 a sole swung, 24 striding | 1 570 us |
| Her body's terms of it | 7 us | 7 us |
| A tick on the Release image (Renode), device 13 ops 1, 2 and 0 | 0.25 M instructions, 0.52 ms on the M7; a sole swung 0.34 M, striding 0.35, setting out 0.44 M, 0.93 ms | 24.85 M, 52.3 ms |
| On the Debug image | 0.31 M; 0.43, 0.43, 0.54 M | 26.29 M |
| Its state | 43 KB | 471 KB |
| Its stack on the M7 | 2.9 KB | 8.4 KB |
| Beside the python step, 120 states | 9e-5 N m with two soles standing, 1e-10 with one or none | - |
| Beside it striding, her walk's 4 951 solves | 7e-10 N m on one sole, 6e-4 on two | - |
| Her walk on it, 40 s | 0.69 m/s, 301 J/m, 2.46 off, her head nodding 2 deg: the python step's 300 and 3 | - |
| A hop on it | 17.9 mm, 2.4 deg: the python step's | - |

- On the image its torques and her soles' loads are the host's within 2e-3
  (test_emulator.py). Of her stand's 250 k instructions on the Release
  image the maps are 43 - the passive joints' rows 20 of them -, her
  frames 42, her bias 36, the rows 35, her mass matrix 32, the rounds 18,
  the base's rows 16, out 11, the base's solve 11, her centre 6 (marks in
  a scratch image).
- Nothing clipped, the rounds end at their first solve. A sole swung and
  the other's moment clipped, all four run: 109 k, the tick 0.34 M, 0.72
  ms - the DOD's 1 ms is the longest tick's, not her stand's, and a
  board's own cycles, not an instruction a cycle: unmeasured.
- Striding (`wbc_cone.c`) the rounds end at their first solve too. Where
  it clipped, her kinetic energy's metric is formed - M T and M E over
  what T and E reach, her accelerations' full map never -, factored, and
  the faces met through its inverse's columns a met face has, each solved
  once: 65 k instructions the metric, 10 its factor, 22 the faces, the
  tick 0.35 M, 0.73 ms. Setting out, 0.15 s, the four rounds run beside
  it: 0.44 M, 0.93 ms, the step's longest tick (device 13 op 2).
- The Debug image formerly ticked 0.32 and 0.43 M: `board_mem.c`'s memset and memcpy
  were -O0 there, 1.5 instructions a byte, and a tick zeroes and copies
  some 40 KB. With `-Og` debug flags and `board_mem.c` compiled under `-O2`,
  the debug flash footprint dropped from 268 KB (91 %) to 220 KB (75 %) and
  ITCMRAM from 36 KB to 33 KB.
- Her body's terms formed 6x6s: a congruence a link for the composite
  inertias, an adjoint a link, a Jacobian by adjoints up the chain and
  every column turned into the world. A link as its mass, its centre and
  its inertia about it, composite about the pelvis's origin in the
  world's axes, a Jacobian's columns from the hinges' axes: her mass
  matrix 45 k instructions where 248, a tick 0.39 M where 0.70 (the Debug
  image), her body 31 KB where 45, the model 8 KB of flash less; MuJoCo's
  as before (test_wbc_core.py).
- Her accelerations over the unknowns, 39 x 21, were built whole and read
  in 11 rows. A row is taken through the pelvis's solve (`over`); the
  unknowns a dof's row reaches and the dofs the pelvis leads are tables
  laid once; her momentum is the pelvis's rows of M u; an unknown asked
  alone goes onto the diagonal: 0.32 M where 0.39 (the Debug image), the
  step's state 25 KB where 32.
- A pass of her latent loop on this host with it: 1.0 ms where 1.3, solved
  every third pass; 2.1 where 2.9 every pass. Her sensing, her modes and her
  world are the rest. Her stand's loop, a pass: 0.41 ms where 1.1.
- For the two to give one answer the python step changed, her measures
  kept (her walk 330 J/m and 2.60 off where 329 and 2.59, her hops to 0.1
  deg):
  + The joints no drive holds - her toes, her hair, her hems - by their own
    rows of her dynamics under all the rest. From their bias alone the
    answer depended on the pelvis's coordinates: 0.03 N m and 2 rad/s^2
    apart between MuJoCo's and the chain's.
  + A clipped moment is its value and her height's band's edge is met, the
    rest solved round them. As rows of 1e5 they put 1e7 on the normal
    matrix's diagonal, and what two standing soles share between them came
    0.1 N m apart at a pass in twenty, 0.6 at most, from one solver to
    another (LU, Cholesky, QR). Now 4e-4.
  + A clipped force stays a row of 1e5 of its weight's share, 327 a
    newton. Pinned as its value her soles pressed nothing through a throw
    (clipped at nought on the first solve, before the band): 1.6 mm. The
    band decided first, her hops went 11.6 deg over where 2.5, 2.2 kg
    m^2/s about her at lift-off where 0.3, and her walk struck 459 N.
