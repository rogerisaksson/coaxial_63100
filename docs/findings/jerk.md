# Findings: jerk

Each segment's peak jerk through her transitions (`tools/sim/jerk.py`, the
referee's 20 ms windows; the scenes in [scenarios](scenarios.md)) against her
steady walk's - pelvis 553, torso 590, head 597, hands 1398-1840, feet
4109-4159 m/s^3: back 1.3, a start and stop 1.6, a sprint 1.7, moves 2.6,
strafes 2.9, a spin 5.9, turns 6.5, the weave 13.0, the chair 14.4, a hop down
18.7, a hand the worst in most.

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
  flipping a pass, [343, 1], [56, 211], [532, 0] N, drives to 385 times. A hop's
  drop 11.5 times; the spin 2.5; turns' worst under its clamps (0.1).
- Not kept: the drives' limits in the step - a torque past its limit pinned as
  a row in the standing rounds and a face in the striding projection: the
  weave's other sole 235-322 N where 764, but her walk's own peaks rose (head
  597 -> 3046, a hand 1840 -> 11002) and moves, reverse and the weave fell, the
  pins on and off a pass; the clearance on a quintic and the swing's spring
  eased in from its release - her walk's peaks 0.85-0.9 of before, the weave's
  hand 13 -> 8.5, but a stop's head 836 -> 6276 m/s^3 and her walk 3.25 off,
  on stilts.
