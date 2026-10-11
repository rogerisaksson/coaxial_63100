# Findings: wrench

Her whole body's step without an active set (`machine/wrench.py`), in the
stack's place ([wbc](wbc.md)); `tools/sim/wbc.py --wrench` and
`tools/sim/hop.py --wrench` run it, tests/test_wrench.py holds it.

- The loop in C spends 14.5 us of a 1 587 us tick on her body's terms - the
  pose, M, the bias, two Jacobians (this host, -O2): 0.9 %. The rest is five
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
| Shoved from 8 ways, stood of 8 at 38 / 60 / 80 / 100 / 120 N | 8 / 7 / 4 / 3 / 1 | 8 / 8 / 7 / 8 / 6 | 8 / 8 / 7 / 2 / 0 |
| WEP over those polars | 0 / 4.8 / 11.3 / 15.0 / 11.4 s | 0 / 6.2 / 12.6 / 18.5 / 20.1 | 0 / 0 / 0.7 / 1.4 / 1.1 |
| The walk, 8 s at 0.5 m/s asked | 16 steps, 1.90 m, 0.32 m/s, 1 400 J/m, tilt 2.5 deg | 16, 1.99 m, 0.33, 761, 1.3 | 15, 2.47 m, 0.41, 259, 1.8 |
| Hops at 10, 14, 18 m/s^2, under her soles | 14.4, 17.9, 41.6 mm | 16.3, 21.5, 46.4 | 16.3, 21.5, 47.7 |
| Their tilt, and three in a row's | 9.3, 2.5, 11.4, 5.5 deg | 6.1, 4.9, 6.1, 5.7 | 5.5, 4.6, 15.2, 6.6 |
| About her at lift-off, 14 m/s^2 | 0.3 kg m^2/s | 3.1 | 2.7 |
| A pass in python | 1.1-1.3 ms | 2.2-2.6 ms | - |

Shoves with the MPC's steps, the PD under each, the gearboxes with their play.

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
- Her height follows her standing knees' posture inside the band, as on
  the stack: asked alone her knees went straight in 0.35 s, 6 to 0 deg, and
  the leg's inverse with them.
- Open. 80 N and over: 4, 3 and 1 of 8 where the stack 7, 8 and 6. Her
  walk's 1 400 J/m: the clamps clip after the solve. The step in C and its
  tick on the M7. Stops but the knees'.
