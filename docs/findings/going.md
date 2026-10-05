# Findings: her going on one law

The gynoid's stand, walk and run as the setpoints of one law
(`machine/going.py` over `gaits.py`, `hold.py` and `strut.py`,
`tools/sim/go.py`; the user's, 2026-10-05): what the law is, what a walk
and a stand asked of it, what it walks and runs at. The walk as built is
in [walk](walk.md), the run in [run](run.md), the page's standing law in
[standing](standing.md). Nothing of it is on the page; `tools/sim/ways.py`
is its spread of timings.

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
