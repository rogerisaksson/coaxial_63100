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
- Who pays at a stop: her rounds solve round what her soles' bounds clip and
  trade it among the rows, the cheapest her neck (W_POSTURE among W_TURN and
  W_COM). At the reverse's stop the rows ask the landed sole to pull, -71 to
  -224 N in 0.1 s - her knees' row the most (-210 -> -44 N without it), her
  turns' and her posture's next (-90, -91), her settle, contact damping,
  momentum and height none -; the clip lets it go and the moment left is her
  neck's at 268 rad/s^2, her arms' 15-38, her spine's 0. On the python twin,
  whose errands are the C's to the digit (her neck's widest nod walk, reverse,
  sprint, spin, weave, deg; a hand's peak walking, m/s^3; errands of 19):

  | what her soles cannot give | neck | hand | errands |
  | --- | --- | --- | --- |
  | traded among the rows (as it is) | 101/105/102/52/62 | 3 679 | 19 |
  | the rows', her strides handed over at omega (C) | 22/92/53/-/- | - | 18 |
  | her inertia's while her soles alone bear her (C) | 0/15/0/0/17 | - | 14 |
  | her neck and head at W_TURN | 0/3/1/0/3 | 13 332 | - |
  | her upper body kept as first asked, soles alone | 0/1/0/0/16 | 2 334 | 16 |
  | that, a prop's bearing held over 1/omega | 0/1/0/0/16 | 2 334 | 16 |
  | her neck and head alone kept | 0/2/1/0/16 | 13 521 | 15 |

  Handed over, her backing to the bed drifts 0.75 m aside in 2.75 s; her
  inertia's, W from a stop takes 1.02-1.03 s (1.0) and a chair's seat bears
  0.35 of her. Her upper body kept, her walk as it was (ortho 50 of 50):
  backing, her yaw yields where her arms bore it - her heading -2.35 -> -1.82
  rad, 0.22 m/s for 0.15, into the chair's side -; lying back, the bed's
  touch lost two passes and her soles took 225/116 N; kept with a seat
  bearing her, its two points side by side giving no pitch, she slid off it.
- The stop's nod bisected (`play_walk` on her law in C, her head against her
  torso): 0.1 deg at 1c05786f, 101.2 at 1f1d039d, the branchless control pass -
  its neck cut from her step only under 0.05 of her rhythm's gain, a brake's
  1.000 all through. Before it `no_stride` let her neck go in a brake by name.
- Her neck let go through a brake (`motion.control`: her rhythm told to run,
  held to her landings' footprint, told no drive) and a sole asked up off the
  floor only once the other bears as much of her (`poise.stands`,
  wbc_stride.c): her head's widest nod, walk 101 -> 1.3 deg, sprint 102 -> 1.4,
  the reverse's stop 105 -> none (36.7 mid-gait, as before), start and stop
  6.6 -> 4.1, a spin 53 -> 34, bed 29 -> 0.3, bed_rise 29 -> 7; her walk the
  same (0.70 m/s, 409 J/m, 2.55 off). Without the guard play_about's stop let
  go a sole bearing 196 N of her, the other 43, and she fell at 8.8 s. Not kept:
  let go wherever her rhythm was told no drive - a hop's stand sank her in a
  replay; her head's yaw let go with it - the chair's dock put her hand on the
  loft, 0 of 5 starts.
- Her friction by heading: walking straight 45, -45 and -135 deg her head's
  peaks are 2 499, 2 276 and 2 252 m/s^3 where 0, 90 and 180 deg give 585, 597
  and 576 - her step's friction pyramid the world's x and z. Along each sole's
  floor axes 577, 578 and 593; that table's chair missed its goal.

## The anti-spasm insert

Her errands said from every spot (`tools/sim/house.py --said`, 112 runs a
verb; a spasm an arm past 87 deg/s, her neck past 37 - her walk's), traced pass
by pass, against the insert's five roots (`machine.supervisor`,
anti_spasm_supervisor.lisp):

- A regime switch at her brake's last landing (sit_chair from -0.1, -0.4, 0):
  the sole lands at 6.312 s and her step's strides go 1.00 -> 0.00 in one law
  pass; it bears 599.5 N, the left shoulder's torque 1.2 -> -54.6 N m, its
  clamp, the arm at -1 208 deg/s 100 ms on: the standing rounds trade what her
  soles cannot give onto it (W_POSTURE 1).
- A contact let go and back with the switch, reclining on the bed's edge
  (lie_bed from -0.1, 0.8, pi/2): the bed bears 0.60 of her on two points of
  ~90 N; a sole lets go while the other stands bearing 0.0 N, strides 0.23 ->
  1.00 in a pass, the bed's points [99.7, 82.1] -> [248.9] -> [73.9, 37.4] N;
  her neck 5.2 -> 50.5 N m (its WEP, 51.3), 1 282 deg/s in 60 ms.
- A limit cycle seated (-0.1, -0.4, 0): the seat's force flips each 6 ms -
  116, 33, 151, 33, 52, 37, 56 N -, her step's scale 0.44 to 0.93 and back,
  neck and head at +-51.3 N m pass by pass; the seat let go, her neck slams to
  -120.8 deg at 2 114 deg/s. Neck, head, shoulders and ankles have no stop in
  MuJoCo, their setpoints a room (`sprung._Q_LIM`: the neck -20 to 26 deg).
- Standing at a corner of her way: the left shoulder at its 58 N m clamp for
  0.3 s, the arm wheeled -8 -> -232 deg (its room ends at -65).
- No count to hunt, nothing ringing: her encoders are MuJoCo's exact angles
  (an A1335's 12 bits through 30:1, 5.1e-5 rad), her gearboxes rigid
  (`physics.BOXED` 0; boxed, a rotor on its wind-up rings at 14.8 Hz in the
  ankle to 41.4 in the shoulder). Her law's 3 ms passes step her drives' feed:
  her walk to 237 kN m/s a drive, the insert's 850 N m/s cuts it 280-fold.
- Her tank, a reaction mass's energy in its own inertia past a budget handing
  its drive to a damper that meets the clamp at the budget's rate (outside the
  tree first): budgeted by her lodge walk's 99.9th percentile (neck 101,
  shoulder 326 deg/s), the chair's 120 runs fell 39 -> 26 (24 saved, 12 new),
  its neck where both held 618 -> 178 deg/s at the median, 1 940 -> 203 at
  p90, 2 877 -> 334 at most, the arms at most 932 -> 394; the bed's 96 fell 5
  -> 7, its neck 1 259 -> 123, the arms 1 105 -> 395. Re-laid with the tank on
  that budget moved (the head 0.32 -> 0.05 J: strike shakes and the room's far
  wall made it). Across a joint's whole room (neck 169, shoulder 790 deg/s)
  the arms were never held: 50 chair runs fell 15 -> 13, the neck 742 -> 294.
  Her walk's own, its rate over a sole's settling: her head 4.5 deg/s, her walk
  holding it still.
- Outside the tree, two and three spots: a Schmitt latch on each borne segment
  (on at 15 N, off under 5, 45 ms dwell), its point held while latched, fell at
  the brake's spot, seated, at 24.0 s; her step's strides carried on a
  critically damped second order at 10 /s took the brake's arm swing at 6.43 s
  and left one at the corner's stand, 1 098 deg/s, the seated neck 1 966 ->
  1 354; her neck, head and shoulders held (`motion.control`'s w_up) past
  their rooms or their energy: 2 of 3 spots fell.
- The insert as her master bus (`wbc_bus.c`, her drives' 1 kHz pass), her walk
  on her loop, 51 s, each stage alone against none (0.70 m/s, 408-410 J/m):
  gate 411, low-pass 410, jerk 409, bridge 417, handover 409 J/m; all but the
  slew 0.72 m/s, 416 J/m. The tank alone drained to its 0.5 J floor in 2.6 s
  through one Q 0.5 band-pass biquad: her stride's 1.8 Hz leaked at -12 dB, her
  walk's own 4.5 W; a 6 Hz high-pass then a 35 Hz low-pass hold it. Its
  preload told to her drives' observer as given torque was a bias: 455 J/m.
- The slew at 850 N m/s felled her alone at 11.9-13.1 s, on her law's command
  or on the drive's feed; her law asks her hips 10 kN m/s at p99, 19 at most,
  and a let-go 114-117 N m in a pass; the ducked, narrowed walk fell at 118.6
  deg. Each drive's own instead: its peak x 2 pi x 2.5 kHz.
- Her damped least squares on the insert's raised cosine: her standing leg's
  sigma_min is 0.023 at the median, under 0.045 90 % of stance. On both legs
  0.61 m/s, 483 J/m, 2.23 off a woman's (as built 0.70, 408, 2.52), its last
  bits stable; on the swung leg alone 0.68-0.69, 416-420, 2.68-2.85; its
  larger with 0.02 there 0.68, 420, 2.95. On the swung leg it acts in its
  swing's first 20 ms alone: the trailing knee leaves at 13-16 deg (sigma_min
  0.035-0.044, lambda to 0.063), at landing 26 (0.07). With the bus she
  walked on stilts (2.87 off, knee at lift 18.5) till her heel's cap was a
  woman's at lift, 2.58 ([feet](feet.md)).
- The buffers before her stops: 0.70 m/s, 407-408 J/m, 2.51-2.52 off; the
  ducked walk bit for bit with and without them.
- Her strides handed over through three 85 ms lags each way, on ice her parry
  waited - a stride's rise 0.31 -> 1.0 handed 0.95 -> 0.64 - she tilted 9-27
  deg and fell at 4.54 s (slip, every wobble); at a third of it each, 4.56. A
  rise at once, a fall lagged: slip, chair, sprint, rug, snow, stutter, moves
  and hole stand, her walk 0.70 m/s, 407 J/m, 2.61 off.
- The jerk at 12 000 rad/s^3 on her drives' observers' ask alone: they pushed
  against the rest, her arm yielded 0.24 rad at a jamb where 0.55; her
  drives' torque following it, her law's 3 ms asks ramped out: on stilts, 465
  J/m, 2.80 off, her leaps and viewport errands red. Each drive's own instead:
  its slew over its rotor's inertia.
- The bridge's 1.2 N m preload as a bias in the command's sign - one
  reversible drive a joint, no antagonist to pull against - with its window's
  65 % slew cut sat her on one sole in the chair (144 and 0 N, hips 13.3 and
  -12.3 N m) and she fell rising at 10.69 s; either alone she rose. Gone: the
  window's cut alone.
- The soft clips' knee at zero (1.5 lim (u - u^3/3)) took 5 % of a hop's push at
  its drive's own jerk: her leap's charge lengthened it 0.10 and 0.08 m where
  0.15, a jamb yielded her arm 0.24 rad. Their knee 6 dB under the ceiling:
  0.18 and 0.24 m, 0.55 rad.
- The bridge's window on every change inside it held her lumbar's torque near
  zero at 35 %: the viewport's chair fell at 4.73 s on every nudge, on the
  lumbar alone at 5.71. On a reversal alone - the gap a transmission crosses -
  she sat.
- The tank tripped at its floor too: a hop's push drained its 4 J in 30 ms, every
  drive rolled back and she rose 669 mm off one sole. Tripped by the detector
  alone, as the text has it - parasitic oscillation in its band - she jumped
  as before.
- Her knees' stop's damper on the bus's gated, low-passed rate braked a jump's
  knee lock late: she rose 410 mm (gate) and stepped on 0.84 m after landing,
  0.41 (low-pass); on the encoder's rate itself, every stage on, 331 mm and
  -0.06 m.
- Each alone - the D low-pass, the reversal's bridge, the handover with its
  rise at once - felled her walking a right turn (replay walk_turn_right_at_0.69,
  8.4-8.9 s at every nudge; the base stands to 13.5 s): with the handover her
  law stood her at 7.58 s while her step still strode (0.96-0.88), both soles
  let go, she landed on one at 360 N and tilted 17 deg. Out of the bus, open.
- The insert's comparators on what bears her off her soles, gating which
  borne segments her step plans on (on 0.18 of her weight, off 0.06, 45 ms
  dwell): the chair's pelvis bounced off the seat - its share 0.32, 0, 0.47,
  0, 0.42, 0 -, each landing held out of her step up to 45 ms while it bore
  0.3-0.47 of her, and her rise fell at 9.26 s; every manifold latched, she
  rose. A debounce on what the step plans on hides a force: hysteresis is a
  mode's, her stance's.
- The buffers at their width alone stop an approach of 1.9 rad/s at most
  (0.06 rad, 140 /s^2, zeta 1): in the standing jump's crouch her soles'
  centres of pressure sat on their heels and her step threw her head for her
  trunk's lean (her neck 55-73 rad/s^2, its posture's row the cheapest), her
  neck met its buffer at 13 rad/s, ran 0.36 rad past its stop and she landed
  0.35 m on. Engaged as far again as the spring takes to stop the approach
  (v / (e w), 0.4 rad at 13 rad/s): 0.044 rad past.
- Her swung knee's torque steps - -36 to +39 N m at its lift, -40 to 0 at
  its rise's end, the clearance's cubic stepping its sole 37 m/s^2 - count
  three hot half cycles and trip its tank: at a 0.6 rad heel's cap with the
  knee's lock buffered, 28 trips in 12 bed errands and, wobbled 3 poses x 8
  headings, 13 docked, 7 fell. At a woman's heel (36.2 deg) the lock
  buffered, 12 of 12; the lock left to the drive pass's knee spring alone, 9
  of 11 shoves from behind, weaves and chairs fell. The clearance on a
  quintic, C2: 2.80 off a woman's, 441 J/m; on a quartic, C2 at its top:
  3.22, 455 J/m (as built 2.52, 409); neither kept.
- After the run leap her law asks her down 4-6 m/s^2 and on 2-6 m/s^2 on one
  sole on its ball (5.86-6.26 s); her step plans that sole 243 -> 0 N as
  asked. Unbuffered, her knee hits its lock at 7.8 rad/s and she lands on
  both (the base the same); its lock's buffer reached ahead, she sank to
  0.47 m and fell, 4 starts in 5.
- Her drives damp a step of their rates once the D channel settles: a pass
  after it 0.3 % of the damping and the preload's sign, +2.5 W; at the gate's
  attack and the low-pass's 2 % time (26 ms), -6.64 W where -6.55 bare.
