# Her errands from every spot

`tools/sim/house.py`: the lodge's free floor at a 0.6 m grid, 0.45 m clear of
its furniture, four headings, the chair said as F says it (`gym.offered`,
`gym.chose`); of 120 the chair is offered at 58.

| her way | ok | down before seated | down seated | down rising |
| --- | --- | --- | --- | --- |
| straight at the chair's normal | 4 | 36 (16 into the chair) | 12 | 6 |
| round the furniture, still at each turn | 21 | 14 | 7 | 16 |
| the same, measured again (6 short) | 21 | 9 | 7 | 15 |
| walked through where a curve fits her arc (5 short) | 23 | 10 | 7 | 13 |

- Her way (`route`): A* on her map's cells, the furniture grown by her
  radius and a cell; she sets off within 26 deg of her leg (her walk's arc,
  1 m round, a cell off it), stands still at each corner and plans the rest
  from there. Turned mid-rhythm her feet met and she went 0.4 m back into
  the chair. Docked 2.9 cm along its edge and 0.054 rad askew at the median,
  in 19.4 s.
- Walked through (`route.planned`): a convex corridor round each of A*'s
  segments, carried past its corners by her arc's tangent and a stride, its
  half-planes at the nearest blocked cells (a cell's face where the whole
  segment lies beyond it); in them a clamped cubic B-spline, a knot a
  stride, its acceleration within `WALK_SPEED * YAW_RATE`, its last 1.23 m
  straight; a solve 0.31 ms median where one fits, 12.6 where none, a plan
  1.1 ms median, 29.9 most. Bent past her arc or none found, the corner
  stays a stop: a curve fits from 2-3 of the 27 spots whose way turns (the
  chair's 20, the bed's 7), the chair's slot from above tighter than her
  arc. Held by her heading, a 0.5 rad step turned her path's chord 0.32 rad
  in 1 s; on her arc's twist her path turned 0.33-0.40 rad/s, a median 0.17
  rad behind her heading; walking a curve she kept within 0.05-0.33 m of
  it, the chair from (-1.9, -1.0) seated and risen where she fell, docked
  in 17.7 s where 27.3. Off the bed's curve her stop drifted 0.8 rad off her
  heading at 0.87 m/s and threw her, the bed's falls 4 -> 5.
- Said from every spot (`--said`, 28 spots x 4 headings, the lamp's post
  kept clear): which spot falls is chaotic in her start - a heading 0.4
  mrad off 3pi/2 sat where its exact value fell -, a change in her loop
  moves 110 of 112 rows, and the chair's tally spreads about 5 of 112; a
  difference is judged with each heading also +-2e-4 rad (336 runs).
- The dock's turns fell her on the loft (x >= 2.0, its top 0.75 m, 0.28 m
  beside the seat, a ground of hers to the referee): her right fingers on
  its edge, turning at 0.1-0.2 m/s 0.13-0.21 m past the dock's line her
  stop left her at. A half turn in place on the open floor creeps her
  centre 0.45-0.58 m in 9-9.5 s, her hands 0.65-0.74 m from where she
  stood; at half her yaw rate 0.65-0.80 m in 13-14 s - about 0.05 m/s
  turning with her, not the spin's cruise (zeroed, 0.43-0.54 m).
- Her turn in place a spin of its own (`spin_left`, `spin_right`: her
  reverse gait's stand, the turn's stepping and stance width; a twist under
  0.05 m/s ahead walks her by it, `se2.GAIT_BOXES`): a half turn creeps her
  0.18-0.19 m, her hands 0.34-0.38 m from where she stood; its stance width
  her reverse gait's, 0.33-0.36 m. Wobbled from every spot (336 runs a verb)
  the chair 228 where 215, 84 down where 101, its turns' falls 20 -> 2; the
  bed 296 where 294, short 1 where 14, 39 down where 28 - corner stops.
- Her sidestep entered on her predicted stop, as it is left: entered on where
  she stood, it flipped in and out every pass at 0.13 m off the line (the
  dock's tolerance 0.125), her dock 4 s longer.
- Her stage before the chair: the middle of the free run on its normal, 0.5
  m where the table leaves 1.03 m; her back-walk's 0.05 m on and her
  sidestep's 0.15 counted in her stops.
- Its sit fired while her rhythm still ran lowered her on one sole: fired
  once it is still, the falls seated 17 -> 7.
- Laid at its dock, the sit and rise hold 8 of 50 within 6 cm, 5 cm further
  and 0.1 rad (4 before), 12 of 21 within 3 cm and 0.03 rad; 5 cm further she
  never sits. The rise's lean tips her pelvis 22-25
  deg ahead and her trunk 9-27 back, her head 71-98 back (her neck then let
  go while she sat; braced seated as walking since, seat findings), her
  centre of mass 10 cm further behind her soles; held, her head follows her
  trunk and her trunk still goes back. The suite's lift-off at 0.82 of her on
  her soles is waited for (`ortho.SOLES`).
- Her pelvis's turn kept sprung while a seat bears her
  (`impedance.SEAT_HOLDS`): let go, sat 3 deg askew she turned 0.05 rad in
  0.1 s and lifted a sole.
- The bed by its texts from every spot: 10 of 52, 16 down. Its lie let her
  arms go: lying back her shoulders turned 260-500 deg, the step's
  reaction masses. Held as the suite holds them (`arms_held`), lying back
  -22 to 14 deg, the bed bearing 0.72 of her: 17 of 52, 4 down, none
  lying. Left: her rise from lying, 29 of 52 still lying 12 s on - sat up
  by the text her back stays on the mattress, her step holding each point
  it bears still in its three axes, away from it too.
- The viewport's bed errand on the spin: lain, her pelvis 0.33 m onto the
  mattress past its edge, both soles at 0 N; F's rise plants soles that
  cannot reach the floor and holds in `sit_up` at surety 0 for 7 s. Lying
  back she slides off its edge 12 of 336 wobbled runs, her pelvis's
  contact coming and going, her feet on the floor.
- The sit begun only docked within the dock's tolerance and her list, her
  sidestep onto the line: wobbled on 99e87de4 the chair 79/237/20
  (fell/ok/short) where 77/232/27, the bed 21/315/0 as it was.
- A corner turned on the spin, replanned a stride before it instead of
  braked to a stand: the chair 291/33/12 where 83/233/20, the bed 43/286/7
  where 23/313/0; she spins in from her walking speed.
- Her seat seen from her estimated soles against the room's props: her
  root's estimate drifted (-0.029, +0.104, -0.022) m over an errand, her
  soles the same to 1 mm, the seat seen 2.2 cm deeper than it stood; a
  thigh met its edge, her pelvis 2 cm over the seat, and her trunk's
  straightening tipped her off. Laid at the errand's own start (0.133,
  -0.025, -0.015) she sat. Her soles about her root placed where the
  props are read: the chair 46/268/22 where 57/265/14, its sit's falls 16
  where 27, the bed 18/318/0 as it was.
- Lying back on the bed a sole left the floor, read as a stride, and her
  neck's own hold let go for 0.36 s: it flexed 64 deg at 1034 deg/s, her
  head to 84 deg. A sole up while the bed bears her is no stride: the bed
  wobbled with its rise, neck median 932 -> 152 deg/s (recline 296 runs ->
  42), head 898 -> 354, fell 55 -> 41, held 315 -> 320, risen 97 -> 93;
  the chair the same to the bit. Held through a sit too, she fell off the
  seat in the chair scenario (6.85 s): the sit's coupling stays.
- Her stop braked over 0.5 s: her plan reached 0 m/s while she walked on at
  0.6-0.85, and one landing took 0.85 to 0.10 m/s in 0.2 s - her trunk 13
  deg ahead, both arms 27 deg (atan of the 5 m/s^2) at 280-344 deg/s. Braked
  over a stride of her walk's 0.9 Hz, scaled by her speed, no shorter than
  her stand's 0.5 s: eight stop phases her trunk 1.8 deg at most, arms 118
  deg/s, 0.81-0.93 m to a stand (STOP_M 0.87). Chair wobbled with its rise
  149/171 fell/risen where 165/141, its arms braking 60 runs where 232; bed
  33/69 where 41/93. A fixed 13 deg lean back in the brake: the chair 207
  falls; the arms held stiff: not kept; a stride's brake on a turn in place:
  her settle and turn fell 41 more.
- The rise's preload on the suite's exponential-fast curve, paced to her
  springs, took 575 ms with its bend at the start; on a minimum-jerk
  deceleration it keeps the suite's 350 ms (manifold_handover.lisp). Chair
  wobbled with its rise: fell 149 -> 114, risen 171 -> 206, held 268 both;
  trunk p90 698 -> 631 deg/s, arms 478 -> 424; the bed the same to the bit.
  The chair a hair off rises 4 of 12 offsets (0-1 mm) on either curve.
- She sat on the chair's edge and slid off it: the descent ran on her
  stand's sagittal spring (300, 3.1 rad/s) though paced to her rest's (1500,
  6.9): her centre of mass ended 2.1 cm behind its 3.48 cm, her hips 2.8 cm
  further back than planned, a thigh on the seat's edge 8.5 mm before her
  buttocks met it, and she rolled 30 deg off it. Its targets held at her
  rest springs: 3 mm behind, both buttocks bear (maneuver._envelope). Chair
  wobbled with its rise: fell 114 -> 56 (seated 20 -> 5, rise 62 -> 19),
  held 268 -> 286, ok 206 -> 267, trunk p90 631 -> 456 deg/s. Landing her
  thighs clear of the edge instead: the seat bore 0.37 of her (DODs red);
  her trunk 9 deg back for the seat's share: 12 sits of 58 fell.
- The referee felled her on the loft beside the chair, 0.75 m up: 25 of
  the 33 errands that fell on their way, her right hand 17 of them, and
  her law let go as she stood. Her ground counts under her knees' height
  over her floor (`worker._is_fallen`, `wbc_pass_paired`): chair wobbled
  with its rise fell 56 -> 28, held 286 -> 306, ok 267 -> 292; the bed
  the same to the bit. Of the 33, 9 were falls: hands on slab_a 7,
  stair4 1, pelvis 1.
- The bed's rise stops in sit_up because her lie rolls flat: reclined 0.5
  rad (the text's -1.0) on a bed bearing her hips 0.15 m behind her ankles,
  a sole lets go and her trunk goes on to 1.6-1.75 rad, her pelvis 0.14-0.3
  m into the bed. Flat, her upper body pulls 6.05 kg m about her hips
  against her hanging legs' 2.84: sit_up asks +1.0 and she stays at -3.24
  for 5 s. Reclined 0.38 rad, her centre of mass where the pose has it
  (`seated.com`), she rose 108 of 336 (69), stopped 161 (234) and fell 67
  (33): lain so a knee kicks to 151 deg and her pitch rocks -0.6 to -1.7,
  sit_up threw her trunk 3 rad forward. Not kept; neither its height held
  at the sit's (rose 3 of 24, rolling), nor the suite's patches: its sit
  lands her pelvis on the bed's edge (22 of 24 fell), its wake from the
  text's lie felled 15 of 24.
- The chair's rise throws her head: sat, her neck 3-30 deg and her head
  2-4; the lean's first 120 ms take them to -49 and -88 deg at up to 1 322
  deg/s (the head's drive held at its -0.85 rad stop), the thrust her neck
  85 deg back at 1 477. Her neck braced through the sit too, 58 errands
  fell 10 where 2 - 6 in the thrust - and its median 1 356 where 1 187:
  the lean's and thrust's centre of mass is asked of her head.
- Lain on the bed, what holds her rise there: her ZMP held to her support
  (`stance_hold`), her centre of mass is asked 0 across (W_COM) and she
  rises none of the way; let go in its plant and sit_up she rose 14 of 24
  (7) and fell 9 (4), her neck at 842 deg/s (194); paced as the suite's
  rise (latched, 1.65 s up, the chair's lean, hand-over and drive), 8 and 12.
  Lying, her soles aloft are asked 0.13 m from her centre of mass (the
  text's sagittal -1.5): her legs fold till a hip is at -2.22 rad past its
  -2.00, and its stop shrinks her whole ask to 0.05-0.13 (`scale`); asked
  out at the floor (-6) her legs hang, hips +0.3 to +0.6, and no sit-up comes.
  Her back's hold let go once its row would lift it (its residual up, in
  `_rounds`): the paced rise stood 2 of 4 in a probe, but in her step bed 6
  of 24 (8), chair 49 of 58 (53), its trunk p90 453 -> 266. Her arms counted
  as supports (`touch.ABOVE` leaves them out): 0 of 4 stood.
- The bed's rise with her ZMP let go, 24 errands: latched and 1.65 s up 12
  rose, 12 fell; then the chair's lean, hand-over and drive 6 and 10, 8
  waiting on her soles; her back's hold one-sided only while her ZMP is let
  go 11 and 10, the chair 48 of 58 and 7 down (53, 2) - its rise lets her
  ZMP go too, and its seat left her in the lean. Not kept.
- The chair's rise with her momentum's row off: her neck 896-1273 deg/s
  (955-1632), her head 1146-1421 (1146-1524) - not what throws them; her
  neck was let go while she sat and the lean whipped her head (braced seated
  since, seat findings). Her
  sit's depth on its seat (pelvis 3.4-13 cm behind her feet) does not follow
  her dock: every sit met it at 0.190 m and began 0.129-0.142.
- A rise counted up as she stood counted falls a second after it: up held
  REST_S (`house._goal`), main from 120 spots and headings, the chair 91
  ok and 21 down - 15 after standing, her centre of mass going on at
  0.3-0.44 m/s past her soles into a 0.41 m stagger -, the bed 27, 9, and 84
  stuck in sit_up.
- Lain on the bed her surety field read her tilt past her asked pelvis
  pitch, 0.87 rad, as a topple: surety 0.03, and its brace stood in for her
  step's answer through the rise - her trunk asked 2.3 rad up moved none:
  the stalls in sit_up. Counted from lying flat while borne
  (`surety_field`): the bed 74 ok, 34 down, 12 stuck; the chair 89 and 24
  (91, 21), wobbled 360 runs 262 and 77 (262, 74). Not counted at all
  while borne, 68 and 39: past flat, the brace's tuck holds her on the
  bed; unbraced the lie's recline rolled her back over it, her trunk to 3
  rad.
- Her arms' reach as the chair's counterweight, both 0.45 rad ahead as the
  suite's descent has them: 58 errands 41 ok and 13 down; let hang again as
  the seat settles her, 49 and 5 (53 and 2 by the old count). Not kept.
- Her arms as supports (`borne`, a bearing arm's hold let go in her step):
  58 chair errands 42 and 13 down where 46 and 9, the bed 8 and 4 where 7
  and 4; on the surety's fix the chair 39 and 17, the bed 6 and 16 where 13
  and 8 - a hand that met the seat or bed as she rose was held there.
  Bearing only while her reach is asked back, asked back to push her up: 3
  of 24. Her arms' swing rolling her onto her side first: 2 of 24. Not kept.
- Her ZMP let go while the bed bears her (texts), on the surety's fix: the
  rise's plant and sit_up 10 of 24 (13), the lie's recline and rest 52 of
  120 and 56 down (68, 39), its rest alone 41 and 63; her soles aloft at
  rest on the floor while anything bears her (`swing.rested`) 45 and 52.
  Not kept.
- The suite's bed played (`patch-supine-mattress-sleep` less its gate,
  `patch-wake-and-stand`): its sit lands her hips on the mattress's edge,
  0.13 m behind her soles, the soft edge bears 0.22 of her then none and
  she tips forward off her feet - 0 of 23; docked at the edge the bed's
  side takes her calves, 17-59 N, and she never comes down - 0 of 24. Sat
  up, the chair's lean, thrust and stand from the bed's edge: 44 of 120, 69
  down - her soles never take 0.82 of her, her hips 0.3-0.5 m onto it.
