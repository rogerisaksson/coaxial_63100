# Findings: her going on one law

The gynoid's stand, walk and run as the setpoints of one law
(`machine/going.py` over `gaits.py`, `hold.py` and `strut.py`,
`tools/sim/go.py`; the user's, 2026-10-05): what the law is, what a walk
and a stand asked of it, what it walks and runs at. The walk as built is
in [walk](walk.md), the run in [run](run.md), the page's standing law in
[standing](standing.md). On the page under J, S and F its rows;
`tools/sim/ways.py` is its spread of timings.

- The law (2026-10-05). A limb bears or is free. What bears carries the
  pelvis: its height the bounce's - from where and how fast she came down
  on the foot to a height and a rise asked - and never over what the
  standing leg reaches on a knee of 6 deg; along the floor she goes where
  the pendulum takes her. What is free goes to its next contact: ahead by
  half her way over a support and by what she goes over the speed asked,
  across at her capture point and a track out, clear of the floor and of
  the standing foot. A foot leaves a stance's time after it landed, the
  next due a step's after it: the stance the longer, a walk; the shorter,
  a run. A gait is a row of 16 numbers - no stage, no table of a plan -
  and the law 370 lines, where the walk's modules are 1 877 with 111
  dated notes of a fall and the runner 312.
- The words (the user's, 2026-10-05): the balance point is the capture
  point against what the bearing limbs hold; interference is whatever
  bears or touches, wanted or not, and the force through it; clearance
  the gap of whatever is to touch nothing. Three levels, each a setpoint
  to the one below and 3-10 times slower: a limb's effort every pass,
  the balance point's margin a contact's change, the margins themselves
  over seconds - a fall the last one's rare evidence, never a price.
- The pendulum's time, sqrt(h/g), is 0.30 s: an error in the capture
  point grows 28-fold in a second, a reflex plans a step and no further.
  Across, a foot's gain is that number - the runner's 0.5 Tc + 0.14 s was
  0.29 already; over a walk's 0.5 s on a foot the gain past which no
  width holds is 0.446 s.
- The run's row is the runner (2026-10-05): asked 1.5 m/s she holds
  1.46-1.47, 516 J/m drawn (297 of work), 12 s.
- What a walk asked of the law, each from a trace (2026-10-05). A foot
  bearing 250-500 N mid-swing was no landing till its swing was done:
  what bears is a contact. Clearance as a knee's fold slewed 230 deg/s
  lost to the knee's own 340 unfolding, the heel on the floor at 0.7 of
  its swing; as a projection - the sole raised to its clearance, the
  foot as it was - none; the toes counted as it came down lifted a
  forefoot landing 1.6 cm, the run's knee 28 deg where 22 and down at
  0.8 s. Her first steps 6-7 cm apart, the free foot struck the standing
  one, 180-475 N on both soles' sensors; a landing 11 cm across from a
  foot still down then: none - held so in the run, 7 cm apart, down in 3
  s. A bounce ended 10 cm up threw her off the floor, the knee 37 to 6
  deg in 0.1 s; ended on the strut as it will be then: none. Both feet
  down, the height asked fell 1.4 cm to the new leg's reach: both let
  her go onto a straight leg, 531 N, 0.92 to 0.5 m/s, and she could not
  pass over the foot (the pendulum's 0.53 m/s at 0.16 m behind it);
  carried by the leg that reaches, 110/72 to 288/210 N over 0.12 s, the
  pelvis 0.892-0.897 m. Held over the strut's flat-footed arc for her
  next landing, her standing heel risen as its leg's length asks, the
  hip was asked 8 deg off where it was: 0.96 to 0.66 m/s in 0.1 s; the
  arc ridden down, her knee landing bent, she walks. The free foot sent
  0.48 m ahead to wait, the run's reach, pitched her trunk back 5 deg
  and its standing hip braked her; 0.25 m: none.
- A walk found (2026-10-05; CMA-ES through the relay, a row's cost what
  she did not walk of her way - a fall ends the trial, no more): 1 280
  rows, 583 walked their 8 s. The best 20 s at 0.66 m/s, 346 J/m drawn
  (119 of work) - no foot down a tenth of a second a step, the knee 14
  deg behind plumb: the scoreboard's price 50.5, six conditions broken.
- A walk found on her form (2026-10-05; a row priced as the scoreboard
  prices a walk, `looks.priced`, a foot always down, the strut the
  form's 6 deg): 2 240 rows, 1 357 walked their 10 s. The best 30 s, 67
  steps, 0.99 m/s at 360 J/m drawn (144 of work) where the walk as built
  draws 623 at 0.96; the knee landing at 17.8 deg (26.4) and 10.6 behind
  plumb (9.6, the form's 10), the leg 14.5 deg behind plumb, the toes
  back 2.1 mm at lift (2), her head's bob 34 mm (30), a strike 483 N
  (450), the pelvis's roll over the stance leg 0.6 deg (3): its price
  26.3. Asked 0.76 m/s she walks 0.99, the speed's integral at its 0.15
  m; rolled 3 deg over the stance leg, 0.64 m/s at 610 J/m. Found on one
  walk, its price is chance: its numbers to three digits, the row in
  `gaits.WALK`, walk 0.95 m/s at 366 J/m priced 92, her toes 8.6 mm back
  at their lift where 1.9; to four digits another way, 0.87 m/s and a
  stumble, 336.
- From the walk's row to the run's and back, the rows mixed in a
  straight line over 4 s (2026-10-05): across both ways on her
  setpoints alone, and down 3.6 s into the run, 1.9 s into the walk; the
  row half-way is no gait, down at 2.8 s begun on it.
- A row between found (2026-10-05; a row's cost what of two passages
  she was not up for, walk to run in 3 s and back): 432 rows, 14 up
  through both. On the best she walks 0.90 m/s, runs 1.45 and walks 0.95
  again, 22 s, the passages a second long, begun at 4.0 and at 4.5 s; on
  eight other timings down 3-6 s after she is across - into the run at
  1.56-1.93 m/s where 1.5 is asked. Held 5 s on the way, the row itself
  goes 0.75 m/s at 627 J/m, both feet down every step.
- Her standing legs pushing her toward the speed asked, the pelvis asked
  0.05 m ahead of where it is a m/s she is slow (2026-10-05): asked 0.76
  she walks 0.84 at 379 J/m where 0.94 at 367; at 0.2 the run is down at
  8.6 s; her passages no better, 1 of 10. Left out.
- In main (2026-10-05; her boards as built): the walk's row 12 s at 0.94
  m/s, 367 J/m drawn, 26 steps and both feet down at each; the run's
  1.46 m/s at 517. `go.py --search walk|between` is the search.
- What it draws (2026-10-05): the walk's row at 0.96 m/s 352 W meaned,
  233 the median, 1.19 kW at 95 % and 1.93 at most - the walk as built
  peaks 4.1 kW at 0.65 m/s; the run's at 1.5 m/s 773 W, 705, 2.2 kW and
  4.0.
- Her stand on it, what her legs do (2026-10-05; `hold.py`). Her
  standing legs lean her toward where her capture point is asked - over
  the middle of her feet as she stands, ahead by the speed asked over
  the pendulum's rate as she goes - and a foot leaves only as a step is
  wanted: she is asked on, or her capture point is out of her feet's
  hold (`dcm.due`). Standing: 10 s on no step with the pelvis asked 0.2
  m a m of it or more, down at 3.3 s with none; asked onto her feet's
  line where it is nearest, nothing held her along it, down at 5.1 s.
  Her feet across from her centre of mass's capture point, not her
  hip's: before, they fell short of it and walked off after it.
- Across, her walk is the pendulum's (2026-10-05; `tools/sim/go.py`'s
  steps traced): her steps 0.167 m wide (sd 6 mm), her capture point
  0.037 m in from the standing ankle as the other foot leaves (sd 1 mm)
  and 0.127 as it lands 0.40 s on - 3.3 a second from a centre of
  pressure 4 mm in from the ankle. Of 192 rows about the walk's 133
  fell, most within 4 s, all across: a foot down at its 11 cm clearance
  left her capture point 0.09-0.11 m in from it, the next step 0.33-0.64
  m wide on a swing of 0.5 s, the one after at the clearance again -
  0.105, 0.332, 0.099, 0.641 m and down; slowed to 0.38 m/s, a swing
  took 1.0 s.
- A free foot due by her capture point across, and a foot's leave waited
  for (2026-10-05). A foot is due, too, as her capture point has run as
  far across from the standing one as a step's swing takes it from
  `track`, e^(omega t); both feet down, her legs bring it to `track` in
  from the foot that stays - across with both down alone: held over one
  foot it stayed there, the free foot came down 11 cm from it and she
  was off across at 1.5 m/s - and the other leaves as it is no further
  in than that and 2 cm. Stood, asked on over 0.1-3 s, walked at
  0.64-0.77 m/s, asked to a stand and stood, 20 s: 10 timings of 10
  where 0 of 4 before, down in her stop at 15-17 s. Slower walks hold:
  asked 0.19, 0.38 and 0.57 m/s she goes 0.13, 0.36 and 0.49, 20 s from
  a stand and from her walk. Of 32 rows about the walk's, up 14 s on
  three placings: 9 on the law as it was, 14 with her stand's hold, 19
  with these.
- Shoved on it (2026-10-05; `go.py --shove`, the page's 38 and 120 N for
  0.12 s from eight ways). Standing, nudged: no step, 8 of 8, her speed
  0.10 m/s at most and still in 1.7 s. Shoved: down on all 8 within 2 s,
  0-4 steps - a step hardly going takes its swing's 0.48 s. Walking:
  nudged up 7 of 8, shoved 5 of 8. As the standing reflex has it
  (`machine.stand`: the foot that can land where her capture point goes,
  on the ray through it, due as it has run): down on all 8 still, and
  after a walk she stood on 2 timings of 10, 7-16 steps each - a foot
  down on the ray leaves her capture point 2 cm from it, on the edge of
  her hold, and steps again. Not in the law.
- One rule for a walk's steps and a run's (2026-10-05). A foot leaves
  `both` s after the other landed and, `both` under 0, by as much before
  that one is due: the run's stance of `stand` s is the same rule. The
  free foot is due the other's `step` on, and by where she is the more
  both feet bear a step, in full from 0.05 s of it. The walk's row and
  the run's go as before, 0.78 m/s at 397 J/m and 1.44 at 522.
- Her jog (2026-10-05): the run's row at the walk's speed, the row her
  way from walk to run passes. On its speed alone the run's row holds
  0.54 m/s asked 0.6 (936 J/m), 0.74 asked 0.8 (704), 0.99 (602), 1.19
  (547), 1.46 (521) - with her speed's gain twice a walk's where she
  flies; on a walk's, asked 1.0 she ran 0.7 for 3 s, then away past 1.47
  and down at 7.5 s. From her walk to her jog over a second: up 8
  timings of 8; back to her walk 6 of 11, her steps 0.13-0.48 m long by
  turns and a foot caught 0.13 s into its swing; on to the run 2 of 4.
  Out of her walk her jog bounces 11 cm, the pelvis 0.826-0.940 m and
  the knee to 79 deg, leaving the floor at 0.72-0.86 m/s where 0.49 is
  asked, a step 0.43-0.54 s; asked faster she leaps further and is down.
  A stance ended as she rises as asked: down at 1.0 s at any speed.
- Tried on her stand and left out (2026-10-05). A swing's clock, a foot
  due a step after the other landed at most: her walk settled at
  0.59-0.61 m/s for 0.76, its knee landing at 28 deg. Her calves as on
  the page (`arrival.CALF_DEG_M`), standing on both feet: a nudge's 0.08
  m/s where 0.10, still in 1.1 s where 1.7 - and stood again after a
  walk on 8 timings of 10, down 17-18 s in; on with every step hardly
  going, 6 of 10.
- Between two gaits their mix is none, a third is (2026-10-05). From her
  jog, each of the walk's shares taken alone and held: its time, up; its
  bounce, its landing or its swing, down in 2 s; its bounce and landing
  together, up, with its swing too, up; all at once, down - a stance
  that still flies ended falling, -0.5 to -0.7 m/s as the foot left, and
  she came down on the next at 1 m/s. `gaits.EASE` is the walk's shape
  on the jog's time, a knot of her way between them. Through it half a
  second a move and 2 s on it: to her jog up 23 timings of 24, back to
  her walk 24 of 24; a second a move, 2 of 6 to her jog; her walk and
  her jog mixed straight over 0.6-3 s, back 12 of 20; through EASE with
  no stay over 2-6 s, 12 of 24 back and 8 of 24 on. The row taken up
  only as a foot lands: every way worse, to her jog 2 of 8. The bounce
  lasting the stance where she flies: back 5 of 16. Her rise as she
  leaves fed back a step after another, 0.1-0.3 of it: the run's row
  down at 1.2 m/s.
- Her jog to the run (2026-10-05). Asked faster over 3 s her bounce
  grows - she comes down 0.57, 0.61, 0.68, 0.74 m/s, leaves rising 0.71,
  0.83, 0.96 where 0.49 is asked, the knee to 75 deg - and she is down;
  over 5 and over 10 s she runs 1.5 m/s, coming down 0.23 m/s at speed
  where 0.54 jogging. `gaits.toward` is how fast a row asked is
  followed: a second from her stand to her walk, half a second between
  the knots about EASE and 2 s on each, 5 s from her jog to the run.
- Her ways asked, over a spread (2026-10-05; `tools/sim/ways.py`, 92
  trials). Stood, walked, stood again 10 of 10; to her jog and to a
  stand again 8 of 8; to the run - 1.5 m/s - and to a stand again 11 of
  12; turned back on her way 6 of 6; the run held a minute 2 of 2;
  slower walks 6 of 6: 43 ways of 44. With her standing legs' hold at
  0.4 of the page's, 27 of 38: stood again after a walk her centre of
  mass crept 6 cm back and 3 across in 1.6 s, a step fell due and she
  was down 5 s on. Nudged (38 N): standing 8 of 8, walking 7, running 4.
  Shoved (120 N): standing 0 of 8, walking 4, running 0.
- Under the director (2026-10-05; `machine/pace.py`). Standing settled
  in the arrival's stand, a pace asked, the law takes her where she is -
  its legs standing where hers are, its setpoints eased from the
  director's over 0.3 s: from the squat she rose, stood, walked 0.82
  m/s, jogged, ran 1.5 m/s from 36 to 44 s and was back at her jog at
  50. Down, the director's fall and get-up are hers; risen, she stands
  for the law again.
- The walk's row had two walks (2026-10-05). Asked 0.76 m/s she walked
  0.81, her knee landing at 18 deg, a step 0.49 s - her feet placed at
  the row's reach, 0.33 m ahead - or 0.64 m/s, the knee landing at 28
  deg, a step 0.54 s, her feet 0.28 m ahead: slow, the law placed them
  nearer to speed her, and nearer under the same pelvis the leg came
  down bent and the step short. Which, her start decided: her feet 0.19
  m apart where 0.17, or the director's stand under her, the second -
  its jog then 1.1-1.2 m/s for 0.8 and the page's presses up 7 timings
  of 12 where the tool's world had 11. Asked 0.80 or more there is one,
  0.82 m/s. One at 0.76 from every start within 4 s, the knee 18 deg,
  with a walk's step never shorter than the speed asked has it and her
  standing leg leaning her 0.5 of her speed's error over the pendulum's
  rate where 0.2 (`hold.LEAN_K`) - 0.2 still where she flies: at 0.5
  there her ways were up 47 of 104 where 80. The page's presses under
  the director then: up 20 timings of 24, down as the run speeds up.
- Tried on her way between gaits and left out (2026-10-05). A flight no
  longer than her rise carries her (0.10 s at the run's 0.49 m/s, none
  on the row between): her ways asked up 23 of 50 where 53 of 56. The
  free sole kept clear to 0.9 or 0.95 of its swing: one walk in both
  worlds, its knee landing at 34-37 deg.
- On the page (2026-10-05; `terminal/views/humanoid_keys.py`). J hands
  her going to the law at its walk - landed anew, risen, standing, then
  on - and S and F step the row asked: her stand, two slow walks, her
  walk, her jog, two faster, the run; J again, the walk as built, S and
  F its cadence. The law's was the page's own for an evening: on stilts,
  her arms bent and not swinging at her sides (the user) - the walk as
  built's again till the law's is a woman's. On the walk as built the
  keys moved its cadence 0.60-0.90 strides/s, 0.39-0.72 m/s: the meter
  two cells down and one up (the user); to 1.0 since 2026-10-06 - 0.90
  m/s, four minutes on at it with her hottest drive at 0.35 of its span,
  1.0-1.08 m/s at 1.05, down in 2 s at 1.10. On the law, driven as the
  keys drive it: walked 0.6-0.8 m/s, her jog, the run 1.3-1.7 m/s for 10
  s, stood again at 60 s; walking 200 s her hips' laminate 60 C, 0.37 of
  its span spent, nothing derated. The floor's events hand her to the
  walk as built; its style does nothing on the law.
- Its walk beside a woman's (2026-10-05; `tools/sim/normal.py`,
  [normal](normal.md)): 4.1 off her band where the walk as built 2.0 - the
  swinging knee 26 deg (52-78), 12 as its toes leave (30-60), the heel
  1.5 deg up then (28-62), the pelvis rolling 1.9 and turning 0.9 deg
  (5-15, 4-22), her feet 0.29 legs apart (0.02-0.22), its step 0.81 for
  its time (1.1-1.8). Her arms were the runner's, elbows 80 deg and
  still: the row's now (`elbow`, `play`; 24 deg bent, 16 of play,
  `machine.free`), the walk 394 J/m where 424. Tried, each alone on the
  row, 16 s from a stand: the knee's fold 30 and 45 deg - swinging 41
  and 56, the toes 40 and 81 mm back as they lift, 0.70 and 0.63 m/s for
  0.76; a step of 0.60 s, both feet down 0.10-0.12 s of it - the toes
  not back, 0.86-0.90 m/s, her head's bob 36-52 mm, the knee 15-17 deg
  behind plumb; the pelvis rolled 4 deg over the standing leg, none as a
  foot lands - rolling 10 deg, her feet 26 cm apart; the heel risen 35
  deg through the s both feet bear - the knee 54 deg at its lift and 53
  swinging, 0.90 m/s at 591 J/m, her ways up 28 of 104 where 77; with
  the fold 25 or the longer step as well, down in her first steps - a
  step of 0.6 s took the pelvis 6-8 cm down a flat foot's arc and the
  next leg's bounce threw her up off the floor. In, and of no weight on
  the row as it is: the pelvis kept up where the free foot's landing
  meets the floor, the standing heel rising as that asks; the heel's
  rise through the double support, the foot about to leave's alone; the
  pelvis's roll none at a landing and the spine against it.
- A woman's walk on the law (2026-10-06; `gaits.WALK`, `QUICK`). On the
  walk as built's time - a step 0.59 s, both feet down 0.16-0.20 s of
  it, the free foot's reach 0.32 m - the standing heel rises 28-38 deg
  by its strut's need as it leaves, the toes not back, the strike
  340-400 N, her feet 12-14 cm apart: 0.78 m/s at 471-514 J/m, priced 22
  where 60-105. With the knee's fold 40 deg, the thigh coming on half of
  it, the pelvis listed 4 and turned 4 deg (a row's `turn`, the waist
  against it): 615 J/m, 1.30 off a woman's band. Searched from there
  (CMA-ES, 24 knobs, 1 920 rows, a walk's price and 15 a width off the
  band): 0.34-0.51 off it, `looks.FORM` met, at 690-776 J/m - the
  ankles' work 101 J/m where 31, the knees' 100 where 64, the hips' 83
  where 59 -, its stops 3 of 10: the search's constants were every
  row's, and her stand's. Searched again on the row's own setpoints and
  the walk's own ways, its J/m by 25, a stop and a passage to her jog
  and back among its trials (960 rows): a step 0.52 s, 0.13 s of it on
  both feet, the fold 45 deg with the thigh coming on 0.32 of it, the
  pelvis listing 3 deg and turning 1, the feet's track 0.044 m. In: 0.81
  m/s at 574 J/m, 0.34-0.41 off the band - the heel 20 deg up as its
  toes leave where 28, the knee 27-28 where 30 -, `looks.FORM` but her
  head 38 mm aside and the standing hip 2.9 deg up where 3; in words
  stiff 0.31, shuffling 0.24, no stilts. The row first found is her
  quick step, a knot between her walk and EASE: on the walk's own shape
  every passage to her jog was down in a step. Her ways 68 of 104 where
  77 - nudged walking up 4 of 8 where 7, shoved 0 where 4, to her jog
  and back 6 of 8 where 8 -; the page's presses under the director 23 of
  24 where 20.
- What felled her on the way, and what was done (2026-10-06, each by
  trace). Both feet down and the rear one kept for her capture point
  while her hips went on at 0.8 m/s: carried past the lead foot on the
  rear toes, the lead sole at 5-100 N - a going foot leaves on its time
  (`hold.WAITS_M_S`). Her capture point over or outside the standing
  ankle through a single support at a track of 0.035 m, the next foot
  landing inside it: leaned back in as it nears that ankle's line
  (`hold.IN_M`). A stop on a long step, her feet 0.18 m staggered -
  crept back, stepped about: the rear one brought up beside the other
  (`hold.CLOSE_M`). Standing after a stop, the pelvis 3 mm over what her
  legs reach flat: a heel rose to reach her, its ball pushed her back,
  and the further back the more - standing she is no higher than both
  reach flat. A first step laid for the speed asked, 0.25 m ahead at 0.2
  m/s, she could not pass over: laid for her pace and 0.36 m/s more
  (`going.GAINS_M_S`), a free foot down 1.5 steps after the other at the
  latest (`LATE`). The fold alone, the toes 40-81 mm back: the thigh
  comes on with it (`free.LEAD`). Tried and out: the heel risen through
  the double support while that leg still bears her - stuck, and a foot
  merely standing rose on its heel as the row turned; given to the
  load's passing it is in and does nothing at this row's 2 deg. The free
  foot's toes pinned through the first of its swing: her parry's steps
  are done in a tenth of a second. The pelvis neither listing nor
  turning as she stands: down in her first steps, a searched row leaning
  on its list from the first.
- The strut a row's setpoint (2026-10-06; `strut`, deg: the standing
  knee the pelvis is never over, 6 the form's). Bent more, the walk's
  row draws less: 528 J/m at 10 deg - 0.25 off a woman's band where
  0.41, the knee 29-30 deg as its toes leave -, 503 at 14, 502 at 18,
  500 at 24, where 574 at 6: the straight leg's cost is how it is
  driven, the item's energy, not its shape. Asked 0.6 m/s on the walk's
  step time she goes 0.59 at 848 J/m, a step 0.39 legs, her walk ratio
  0.73 where 1.1-1.8, the heel 7 deg up as its toes leave - stiff 0.61,
  shuffling 0.67, tripping 0.53, a shuffle on stilts: a woman's step
  keeps its ratio to its time at any pace, the law's slow walks keep the
  walk's time.
- What her walk on the law draws, by joint and by where in a stride
  (2026-10-06; her draw the work her drives do, braking giving nothing
  back, their copper's heat and the boards' own 53 W). At 0.84 m/s
  570-582 J/m: 279 of work, 229 copper, 63 the boards'; 196 braked - the
  knees 85 worked and 80 braked, the ankles 108 and 74, the hips 69 and
  32. Half of it in three passages of a stride: a landing's first 0.1 s,
  90 J/m; the other foot's landing, 49; the foot's leaving, 135 - there
  the knee asked 23 deg more in a pass and the ankle 13, its toes' point
  counted under the floor as the heel is up and raised at once
  (`free.lifted`). A walk's foot eased up to its clearance over the
  first fifth of its swing: 502 J/m (four starts, 496-516), as the page
  runs her 485 and 0.20 off a woman's band where 0.34, a sole's strike
  352 N where 428, her head bobbing 33 mm where 30. A run's is drawn up
  at once: eased over any of its swing she was down on her way back to a
  stand, 1-2 timings of 12 where 11-12. A foot on its heel has its
  heel's rise only as its toes come down (`strut.rocker`): taken about
  its ball at once, the knee was asked 8 and 21 deg within 20 ms of a
  landing - 8 J/m. Her ways 69 of 104 where 68, the page's keys 12 of 12
  where 9, its presses under the director 21 of 24 where 23. Tried and
  out: the lead leg a strut from its landing, its knee from the bend it
  landed with to the row's - 531 J/m where 506; what stands eased over a
  foot's landing or leaving - 514 where 512; the fold a smooth bump over
  the whole swing - 446, the knee 17.5 deg as its toes leave where 27,
  stiff 0.46: out till the knee bends before the toes leave; the free
  foot's ankle by its own way, not level - 576; the leaving knee bent as
  its load passes - 10 deg 527, 30 deg down in her first steps: its foot
  bears 345 N again as it pushes off, the load passing twice.
