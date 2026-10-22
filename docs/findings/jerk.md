# Findings: jerk

Each segment's peak jerk through her transitions (`tools/sim/jerk.py`, the
referee's 20 ms windows; the scenes in [scenarios](scenarios.md)) against her
steady walk's - pelvis 553, torso 590, head 597, hands 1398-1840, feet
4109-4159 m/s^3: back 1.3, a start and stop 1.4, a sprint 1.8, moves 2.6,
strafes 2.9, turns 3.8, a spin 4.4, a hop down 11.1, the chair 11.6, the weave
17.9, a hand the worst in most.

- A start's spikes sit on its first landings, the trailing sole leaving 9 ms
  after the other lands: its toe's 13-65 N stood it and let it go each law pass,
  her asked acceleration 1.2 m/s^2 across and back every 3 ms. Her asked jerk
  is no higher there than walking (p99 343 against 439 m/s^3); the weave's is
  4 times (1767).
- At the window's switch, a start's and a steady step's are alike: 21 ms after
  the other landed, it bearing about half her load. Out of her support region
  in one pass, her weight swung between the soles every 10 ms and her head
  went 22 m/s^2 across.
- That sole's share let go over her rhythm's double support: starts 4.0 -> 2.5,
  her walk 412 -> 448 J/m, 2.61 -> 2.78 off a woman's. A released sole kept
  down until its swing: `play_about` 11 of 13 wobbles, walking shoves 28, but
  at the tree she stepped in place for good - need 1 each step, no double
  support to settle in: her stop rests on that sole standing again.
- Under it, her step: in double support its striding projection planned her
  weight's split flipping a pass - [0, 379], [382, 0], [0, 384] N - and at
  times [0, 0]. Her soles on their ball and heel, each one's room a line, the
  projection met it through her soles pushed against each other - 926 N along
  one, -906 along the other: in her kinetic energy's metric 1e-7 per N^2
  against 0.03 for a push that moves her -, drove one to pull and let both go:
  11 times in 12 s of her walk, 7 in a start and stop.
- Each standing sole's wrench kept in that metric, W_KEEP 3 of its share of her
  weight: none let go walking; her walk's peaks 553, 590, 597, 1398-1840 m/s^3
  (pelvis, torso, head, hands); a start 1.6 of them, turns 6.5, a spin 5.9, the
  weave 13, the chair 13.4, a hop down 18.7; her walk 408 J/m, 2.55 off a
  woman's (412, 2.61); the bench 62 of 87, 80 N walking 29 (28), 200 N standing
  15 met (14), walking 17 (21). At 10 the tree and the hole fell at two
  wobbles; at 30 2.84 off; at 100 let go 187 times.
- A seat she rose from: a contact let go in a pass, its point still held,
  asked 5134 N of a sole; let go when the step would have it pull, 488 N at
  most - the bed 7 -> 9 of 13 wobbles.
- Not kept: a support's push weighed by its distance from her ZMP - no seat
  used till she is over it, the chair fell at 4.7 s; the blend eased, its rate
  and acceleration nothing at its ends - a spin 4.6 -> 1.7, turns 2.8 -> 8.5,
  her head 35.4 mm aside (35); a swing asked as far as its sole no longer
  bears - 401 J/m, 2.64 off, the weave as it was; a standing leg's servos
  softened by its load - every digit the same.
- Left: a sole let go at 60 N on its toes, the other then 715 N where the step
  planned 192 (turns, the weave); a seat's touch flickering as she rises; a
  hop's landing.
- The let-go itself: held still standing, the trailing sole is asked about 35
  m/s^2 its first pass swinging - its toes' need (half a sole and its toes by
  its tilt) whole at once, the clearance's cubic rise -, its hip 8 -> -158 N m,
  the other hip -1 -> 115, her centre of mass -3.0 -> +2.3 m/s^2 up. Not kept:
  the rise quintic and the toes' need eased in with it - 780 m/s^2 its third
  pass, 600 J/m; the toes' need eased into its place alone - 100 m/s^2 its
  second pass, 2.92 off.
- In the weave a sole let go bearing 63 N jumped her upper body's feed 133 N m
  in the pass, the swing's first ask 89 m/s^2 - the steady walk's let-go 56 N,
  13 N m, 35 m/s^2. Before it the right sole, in its stance phase on its toes
  and read by its corners 24-37 mm over the floor, bore 30-39 N and nothing a
  pass about and stood and was let go each. Its lowest point read with its toes'
  tip: let-gos 35 -> 60, her walk 2.52 -> 2.60 off: not kept.
- A sole's push capped in her step over its pre-swing, nothing left as its swing
  begins: her step planned 0 N on it the pass before every let-go of her walk
  and of the weave - and planned about that already -, and it bore 47-58 N as it
  was let go, its upper body's jump the same (12-16 N m walking, 130 N m in the
  weave). Its leg holds it to the floor through her drives' tracking and their
  observer, not her step's push: not kept.
- The swing along and across on the least-jerk path from the sole's
  acceleration, none if it stood: the let-go's upper-body jump 12.6 -> 7.2 N m
  walking and 133 -> 59 in the weave, its first ask 35 -> 25 m/s^2; but her walk
  2.52 -> 3.52 off, her knee landing at 42 deg - the old spring was the limb's
  own stiffness through the whole swing, and the path's horizon held to it
  lagged. The old spring's ask changing no faster than 4 000 m/s^3 (her steady
  walk's feet): the jump 7.8 N m walking and 113 in the weave, her walk 2.56
  off, every segment's jerk peak and her hips at their clamps 3.3 % of passes as
  before: not kept.
- The weave's worst, a hand's 18 181 m/s^3 at 4.49 s: the trailing sole let go
  as the swing goes 3.3 times its rhythm (`stride.hurried`), its clearance 0 to
  12 cm in 24 ms - 328 m/s^2 up asked, 170 along; drives at up to 16.8 times
  their clamps, clipped after the solve, the other sole 764 N where 166
  planned. The chair's rise (a hand's 20 161): the standing rounds' split
  flipping a pass, [343, 1], [56, 211], [532, 0] N, drives to 385 times, as
  the seat's contacts come and go under it - [68, 69], [0, 96], [0, 0], [53],
  [63, 3] N planned on them 3 ms apart. A hop's
  drop 11.5 times; the spin 2.5; turns' worst under its clamps (0.1).
- Not kept: the drives' limits in the step - a torque past its limit pinned as
  a row in the standing rounds and a face in the striding projection: the
  weave's other sole 235-322 N where 764, but her walk's own peaks rose (head
  597 -> 3046, a hand 1840 -> 11002) and moves, reverse and the weave fell, the
  pins on and off a pass; the clearance on a quintic and the swing's spring
  eased in from its release - her walk's peaks 0.85-0.9 of before, the weave's
  hand 13 -> 8.5, but a stop's head 836 -> 6276 m/s^3 and her walk 3.25 off,
  on stilts.
- A row's blend is a straight line (`latent.step`): its rate steps at both
  ends and the damper's ask with it - at the sit's command the trunk's turn
  asked 0 -> 9.2 rad/s^2 in a pass, both soles' centres of pressure -11 -> 83
  mm, on their toes. Over five wobbles each peak is one event (turns 6.3-6.6
  at 5.54 s, the weave 12.7-13.0 at 4.50); at wobble 0:

  | blend | turns | spin | weave | chair | hop down |
  | --- | --- | --- | --- | --- | --- |
  | straight, its rate asked (as it is) | 6.5 | 5.9 | 12.9 | 14.4 | 18.7 |
  | straight, its rate not | 8.2 | 6.0 | 12.8 | 5.1 | 15.9 |
  | smoothstep, its rate asked | 12.3 | 1.5 | 10.6 | 14.1 | 16.1 |
  | quintic, its rate and acceleration | 12.0 | 1.7 | 12.1 | 31.4 | 18.2 |
  | quintic, neither | 12.7 | 1.8 | 13.0 | 13.1 | 18.8 |

  Its rate not asked: the bench 63 -> 65 of 93 scenes (the slick patch 3 of
  3), walking shoves 29 -> 30, her walk the same to the digit, but
  `walk_turn_right_at_0.69` 12 of 13 wobbles - stepping in place after its
  push, a landing at 912 N bounced both soles off the floor. Not kept; the
  turns' peak is its let-go, moved by each.
- Her head and arms are her step's reaction masses for her pelvis's and
  trunk's turns (W_TURN 30, W_POSTURE 1; at W_TURN 0 the neck's worst error
  1.82 -> 0.16 rad). Seated, the neck 1.74 rad off its posture for 6 s, its
  drive asked 2.1-3.9 times its clamp, 40 % of the chair scene's passes; at a
  hop's takeoff 2.11 rad at 33 rad/s. Her walk asks no upper drive past 0.3 of
  its clamp. Not kept: the neck and head held as turns - the shoulders took it,
  3.3-3.9 % of the passes past their clamps, 69.6 times at worst, a hand's
  peak on the chair 22 091 -> 37 818 m/s^3.
  Seated it is her pelvis's turn: without her trunk's the neck still 1.73 rad
  off; without her pelvis's she fell before she sat.
- Seated, only her thighs bear, their touch points side by side (`borne`: a
  point each): the seat gives her step no pitch moment, her turns' asks met by
  the plan's opposite, -3 to -15 rad/s^2 asked, +2 to +5 planned.
- Kept: an ask her drives cannot carry shrunk as one toward asking nothing
  (her weight borne, every acceleration zero: the same rows solved again,
  `wrench.shrunk`, wbc_shrunk.c) as far as brings every torque within its
  clamp, in place of the clip after the solve - the largest s in [0, 1] along
  u0 + s (u1 - u0) with every drive within its limit, closed form a drive; her
  `over` and WEP's grant the ask's, before it. Her walk the same (0.70 m/s,
  408 J/m, 2.51 off; 12 of 16 001 solves shrunk, to 0.21 at the least); the 22
  held scenes 66 of 66 at three wobbles; a hop down's hand 18.7 -> 11.1, the
  chair's 9.3 -> 5.7, turns 6.5 -> 3.8, a spin 5.9 -> 4.4, a start 1.6 -> 1.4,
  the chair's head 11.6 as it was; the weave's hand 13.0 -> 17.9: at 4.48 s
  its hurried swing asks the left hip 10.7 times its clamp and her balance's
  ask goes with it, 0.09-0.5 of itself over 60 ms (its 99th percentiles
  856/1353 -> 810/1259). A hop down's shrink is the neck's 19 of 19 passes,
  the head her turns' reaction mass, 5 of them to 0. A pass 1.14 -> 1.70 ms
  in python, the C tick 25 -> 35 us a sole swung, standing the same.
- Not kept: a contact's hold in the step grown with its felt force, nothing at
  5 N to whole at a tenth of her weight, in place of the switch at 5 N: the
  chair's 99th percentiles the same (pelvis 291, head 1109 m/s^3), the bed's
  up (634 -> 893, 962 -> 1468); her walk the same.
- The chair's worst head jerk (6945 m/s^3 at 1.10 s, the sit's first passes)
  sits on the neck at 1.04-1.14 of its clamp, the head the step's reaction
  mass; the law's setpoints and feeds are smooth through it, her soles' loads
  swing 125/90 -> 91/138 N a pass. Not kept: each drive drawn to its clamp in
  the rounds by a row whose weight grows with the overshoot (nothing at the
  clamp, whole a tenth past it): her head nodded 8 deg walking where 2, the
  weave's 99th percentiles 856/1353 -> 1155/2263, the turns' 391/430 ->
  446/648, the chair's the same.
- Her head at a stop (the stop ending her walk, a reverse, a sprint): her neck
  nods 101-105 deg in 0.15 s, its drive at 25 N m, at its WEP's 51.3
  chattering 50 either way at its end. At her brake's last landing her
  capture point is inside her soles, her step's strides 1 to 0 in a law pass,
  its rounds' answer her neck at 268 rad/s^2 where its projection's was -0.3;
  her turns' rows asked 4-7 rad/s^2 then. Not it: the law's 3 ms hold (solved
  every pass, the same peaks), her drawing (her head within 1 mm and 0.65 deg
  of MuJoCo's), her floor's boxes (the slab's top a plane, bit for bit).
- Her friction by heading: walking straight 45, -45 and -135 deg her head's
  peaks are 2 499, 2 276 and 2 252 m/s^3 where 0, 90 and 180 deg give 585, 597
  and 576 - her step's friction pyramid the world's x and z. Along each sole's
  floor axes 577, 578 and 593; that table's chair missed its goal.
