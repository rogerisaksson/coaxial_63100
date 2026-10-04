# Findings: her standing

The gynoid standing: the rigs under her, the one law in the capture point's
plane, the push polar. Kept up walking she is in [balance](balance.md); the
board's own are in [FINDINGS](../FINDINGS.md).

- Standing (`machine.stand`, the user, 2026-10-04): the director holds the
  arrival's stand `stand_s` (its keyframes lifted `up` onto two bricks, the
  left `stagger` ahead), the floor rigged under her (`machine.floor`: two
  bricks, a balance board hinged in the slab's gap, stiff at BOARD_K or
  free), and what befalls her standing (`events.STANDING`, the scoreboard's
  'stand' suite: a nudge or a shove from her side or along her way, a brick
  taken away, the feet abreast or staggered, a nudge on the board stiff and
  free). Standing on the box soles alone (no step), 12 s: the nudges 100 %,
  the stiff board 100 %, the free rocker 65 and 100 % (rocking sideways,
  ahead), the shoves 67 and 50, the bricks 50 and 41 (2026-10-04).
- The arrival's pelvis target no further than `arrival.PULL_M` (5 cm) from
  the pelvis: a get-up handed over with her centre of mass 12.7 cm ahead
  of the squat's target sent it 19 cm off, the stance legs straightened
  toward it and flung her to 1.04 m (both get-ups after P's shove down in
  3 tries); clamped, both walk again at 21.8 and 22.2 s on the first try,
  and the rises 70.5 -> 91.8 % (10 cm 81.9) (2026-10-04).
- The standing step: out of the support - a point under each sole's centre,
  one once a foot has hung unloaded 0.1 s - the lighter foot is put down
  past the capture point as it will be at the landing, as the arrival's
  keyframes (0.1 s up, 0.12 down, the swing foot never a stance leg and
  pinned where it lands, the pelvis lowered to reach). Taken by load, a
  shove's first 30 ms unloaded the far foot to 31 N and she stepped it to
  its own side, a no-op; the walker begun from the stand instead marched
  on at a 5 % stride and fell every time (391 and 48 %). A 120 N shove runs
  the capture point 16 cm out in 0.25 s and the cross-over cannot reach;
  the brick's step down lands (the left to 0.18 m, 191 N, the pelvis 6 cm
  lower) and she leans back off both feet 1 s on. The scoreboard with the
  step at 3 cm 58.5 % (the nudges along 51), without 64; the reflex stays
  behind the sole's edge until it earns its place (2026-10-04).
- The brick's step down, three ways (2026-10-04): stood again over 0.6 s
  instead of 0.3, or the pelvis target let 10 cm out, the second step a
  31 cm cross-over out of reach, down at 6.1-6.7 s; the pelvis lowered
  for the brick's 6 cm only once stood again (before, dropped on the
  brick's leg as the other reached down, the landing bounced 0-600 N),
  the landing holds at 150-250 N a foot but she leans back off both,
  0.17 -> 0.36 m/s, down at 6.75; steps of 10 cm at most 0.15 s apart
  (`stand.STEP_MAX_M`), four of them, the body backward-left at 1 m/s by
  the fourth, down at 6.6. Standing on the lower foot after the step, her
  weight runs back whatever the stood target: not understood yet.
- The free rocker felled every rise with the pelvis target held within 5
  cm (`arrival.PULL_M`): the pull by stage - 5 cm in the squat and the
  look, where the get-up's hand-over flung her, `PULL_UP_M` from the push
  on. At 0.15 she rises on the rocker and stands through the nudge: the
  stand suite 65.2 -> 74.7 % (the rocker 27 -> 62, rocking ahead 21 ->
  85), the look suite 69.9 -> 64.2 (the rises 76, 100, 99 -> 90, 75, 71 at
  0.6-0.9); both get-ups after P's shove walk again. Standing first:
  0.15 (2026-10-04).
- The 15 cm pull through the push, the rise and the stand alone, the first
  strides at 5 cm as the squat: the rises 76, 100, 99 % again (91.8) with
  the stand suite's 74.7 kept (2026-10-04).
- The brick, read closer (2026-10-04): after the step down her loads bounce
  0-1100 N a foot for 0.4 s (stood 4 cm lower the same); the second step
  takes the brick's foot 10 cm back onto the brick's rear edge - the
  controller knows no brick - where, pinned, it bears 0-8 N, and she goes
  on one leg to a third step and a fall at 6.7-6.9 s.
- Point feet (`figure.POINT_FEET`, a sphere under each sole's centre): from
  the squat she falls in the rise at 1.77 s - the arrival's feedback is
  the ankle's, nothing on a point; stilts need a step to rise (2026-10-04).
- Shoved standing from her side (`look.py --stand shove`, the shove's
  newtons a knob): 60, 80 and 100 N for 0.12 s all fell her at 6.2-6.5 s,
  two or three steps taken; 38 N she holds without a step. Past saving
  standing lies between 38 and 60 N (2026-10-04).
- A pinned foot's press (`arrival.PRESS_M` 0.01, the lab, 2026-10-04): a
  foot bearing nothing is solved from the pelvis as it is, put exactly on
  the floor and bears nothing - after the step the stepped foot hung 2 mm
  up at 0 N while the capture point passed over it, and she fell at 6.2 s.
  Reaching 1 cm under for what it does not bear: the 60 N standing shove
  stood from either side, one step, the landing a 1.3 kN spike and a 1.4 cm
  hop; 100 N fell from either side after two cross-over steps. The stand
  suite 74.9 % and 207 against the built 74.7 and 215 (its shove 120 N);
  the whole scoreboard 721.4 and 65.8 % against 729 and 65.7: built.
- The 100 N standing shove (the lab, 2026-10-04): the cross-over's foot
  never came down - the step cap scaled its CROSS_M ahead to 10 cm, the
  foot hung 2.7 cm up 12 cm short of its mark at 0 N. Clipped before the
  clearances it lands 24 cm ahead, and she still falls from either side in
  two steps, at the cap 0.1 or 0.2, DWELL_S 0.15-0.4: not taken.
- The brick taken away (the lab, 2026-10-04): the freed foot steps 11 cm
  out and 6 cm down at 0.6 m/s, lands at 1.0-1.35 kN, she bounces off both
  feet and goes over after a second step; slowed to 0.3 or 0.2 m/s the
  capture point has run 25-50 cm before it lands. Dropped straight down
  where it hangs (its own point still holding the capture point between
  the two) it lands at 1.2-1.3 kN and bears, and the body tips on toward
  the dropped side faster than the second step at any cap 0.1-0.3, gain
  0.1-0.8, dwell 0.25-0.4: not taken either. The drop's momentum, not the
  step, is the gap.
- One law for standing (`machine.dcm`, 2026-10-04; the user: every
  special case a sign the problem wants a more general form): the capture
  point in the sole's hold - an ellipse 4 cm across and 8 along about the
  support's point nearest it -, a step due only outside it, landing on the
  ray from the measured centre of pressure through the capture point as it
  will be at STEP_S 0.22, STANCE_M 2 cm beyond; the foot that steps the
  hung one, else the farther from the landing; a foot hangs bearing
  nothing 0.15 s where the centre of mass asks 45 % of it; any foot
  reaches 1 cm under for what it does not bear. The stand suite 76.5 % and
  177.9 against the nine-constant reflex's 74.7 and 215.2: the nudges, the
  boards and the staggered rocker 100 %, the free rocker 73, the shoves and
  the bricks 45-49; the nudges and the 60 N shoves stand with no step at
  all. On the way: a landing 8 cm beyond a round 4 cm hold returned every
  step as the next; a step called inside the hold by the time to leave
  fired on 2 cm with the ankle already bringing it back; run from the
  hold's edge rather than the centre of pressure every step on a 120 N
  push landed short; a foot hung by its load alone made the foot a push
  unloads no support, and she stepped inward for nothing.
- The push polar (`events.PUSH_DEG`, a push's way an angle in her plane;
  `gait_montecarlo --suite stand --grid events.PUSH_DEG=0,45
  events.SHOVES.shove=60,80,100,120`, 48 pushes over 8 ways, 2026-10-04),
  the one law as first built: 20 of 48 stood - 60 N 11 of 12, 80 N 9 of 12
  (none of 3 along her way), 100 and 120 N none of 24. She stood what the
  hold alone holds - 6.2 cm of capture point at 60 N, 8.3 at 80, the sole
  8.6 along and 11 across from her middle - and no push that asks a step.
- Why no step stood, traced a row every 25 ms (2026-10-04): on sprung toes
  the stepping foot was pinned as it left the floor, the ankle 1-2 cm up
  and the toes still pressing 60-157 N, and never went to its landing -
  pinned now only coming down (a keyframe's `land`); pinned at its first
  touch, toes first and the ankle 1.2 cm up, it levelled 2 mm over the
  floor at 0 N - it seeks the floor from there until it bears
  (`bearing.SEEK_M_S`); her standing legs are straight - the hip 0.769 m
  over the ankle of a 0.770 m reach, 3.8 cm of it along the floor, 17.8 at
  2 cm down, 24.8 at 4, 30.0 at 6 - so over the front foot after a step the
  rear leg hung 2.4 cm off the floor and she stood on one foot's toes: the
  pelvis no higher than both legs reach (`bearing.height`); called 30 ms
  into a 120 ms push the landing fell 5 cm short - aimed anew in flight
  (`stand.retarget`); the hold as an ellipse called a second step with the
  capture point on the landed sole - a box; pushed from the front, both
  feet's loads chattering 0-850 N on her heels, neither leg held the
  pelvis and it pitched 54-65 deg back in 0.15 s under a plumb torso.
- The landing's three forms on the polar (2026-10-04). On the ray from the
  centre of pressure through the capture point, the steps' mechanics
  mended: 80-120 N 14 of 36, a push from behind stood at 80, 100 and 120 N
  on one step; the pelvis height's variants beside it - the stepping foot
  in the reach limit or not, the target held within 1 cm of the pelvis or
  not - 11-14 of 36, chance. The support carried past the capture point
  along the way it left, the feet kept apart (a half-plane): wrong - on
  point feet the capture point is held only on the line between them -
  and every push with a side to it fell. On the ray from the standing
  foot's point (`dcm.landing`; the foot bearing less steps, the capture
  point running from the other alone, so a push from behind lands the foot
  in its own lane): 25 of 48 - 60 N 12 of 12, 80 N 10, 100 N 3 (from
  behind 1 of 1, from the front 2 of 2), 120 N none - and the bricks 55-60
  % from 48-51, the boards and the nudges 100, the rockers 63 and 90.
  Still down: 120 N any way, and 100 N with a side to it - the capture
  point passes the loaded foot, the free foot's landing lies across it
  (none, by `dcm.landing`), and the loaded one lifted from under her
  drops her, the pelvis rolling 27 deg in 0.2 s.
- The trunk leant by how far the capture point stands out of the hold, as
  an offset on the spine's pitch and roll setpoints (the lab, 12 probes,
  2026-10-04): 300 and 600 deg a metre, either sign - 100 N from a side
  fell every time, 100 N along her way stood without it and fell with it
  3 of 4. Not taken: a setpoint is not a moment; the centroidal moment
  wants the spine's torque, or the pelvis's own tilt, in the law.
- The grinder's rounds on the stand suite, the day's build (`gym.py`,
  2026-10-04; each a best of 8-10 beside the built 77.5 %): `dcm.HOLD_M`
  along, `STANCE_M`, `STEP_S` 155.4 and 79.1 % at the built values;
  `dcm.CLEAR_M` along with `drives.TOE_K`, `TOE_LOSS` 132.3 and 81.2 % at
  the toes' spring 42 N m/rad - stiffer toes hold her standing where the
  walk wants them at 10.
- Walking on the standing law (the lab, 2026-10-04): her weight asked ahead
  of her feet's middle, ramped 5 cm/s, for the law to step her on. She
  pivots over her forefeet as one piece - straight legs, the heels up, the
  feet's loads to 38 and 0 N by 4.5 s - and the step called at 8 cm comes
  0.15 s before she is down. Not a walk: a step planned before the hold is
  lost wants a reference for the capture point, not a lean.
- Each foot bears its share (the lab, `bearing.shared`, 2026-10-04):
  standing to stay every leg a stance leg, its foot let down 0.3 m/s a
  share of her weight it lacks and drawn up as fast a share too many, the
  share what her centre of mass asks of it between the feet, all of it
  while the other steps. The polar 32 of 48 against 25 - 100 N 9 of 12
  against 3, from her sides 3 of 3 where none stood, 120 N 3 against 0, 60
  N 9 against 12 - and the rigs: the bricks 96 and 63 % against 60 and 55,
  the rockers 100 and 93 against 63 and 90. From behind it falls at 60,
  100 and 120 N: tipped over the standing foot's toes its load reads 0,
  the leg is let down for its lack and throws her on, the foot 20 cm in
  the air. From a side the same throw is the push-off of a side step: the
  loaded foot lands 31 cm out, the other follows, three more steps and she
  stands, 5 cm down on knees bent 40 deg. Two variants by probe: the
  shares on both feet standing only, a step as before - the rockers and a
  brick stand, 80 N from a side falls on five steps; a foot let down only
  where it has lost its floor - a nudge falls.
- The share law baked (`bearing.shared`, 2026-10-04): the stand suite 99.3
  and 89.2 % against 237.9 and 77.5 - the 120 N shoves 66 and 67 % from 50
  and 58, the bricks 96 and 63 from 60 and 55, the rockers 100 and 100
  from 63 and 90 -, the scoreboard 538.0 and 82.8 % against 641.7 and 78.6,
  the rises, the walks and the events as they were. Its forms on the polar,
  six of 48 pushes each: as baked 32; the standing foot's let-down held
  still through a step 30; the share what the capture point asks 24 and 23;
  a step run as before the law, the shares on both feet standing only, 18
  and 20 - the rockers 100 % under every one, the bricks 66-96. By probe
  her quiet standing chatters under it, a foot at 0 N one row in two; the
  lack filtered over 50 ms the feet bear 161 and 162 N (its polar next).
