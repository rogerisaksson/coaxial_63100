# Findings: one going law

Stand, walk and run as the setpoints of one law (`machine/going.py` over
`gaits.py`, `hold.py` and `strut.py`; `tools/sim/go.py`; 2026-10-05): the
law, what a walk and a stand asked of it, what it walks and runs at. The walk
as built: [walk](walk.md); the run: [run](run.md); the page's standing law:
[standing](standing.md). On the page under J, S and F its rows;
`tools/sim/ways.py` is its spread of timings.

## The law

2026-10-05:

- A limb bears or is free. What bears carries the pelvis: its height the
  bounce's (from where and how fast the body came down on the foot to a
  height and a rise asked), never over what the standing leg reaches on a
  knee of 6 deg; along the floor the body goes where the pendulum takes it.
- What is free goes to its next contact: ahead by half the way over a support
  and by the speed over the asked, across at the capture point and a track
  out, clear of the floor and of the standing foot.
- A foot leaves a stance's time after it landed, the next due a step's after
  it: the stance the longer, a walk; the shorter, a run.
- A gait is a row of 16 numbers, no stage, no plan table; the law 370 lines,
  where the walk's modules are 1 877 with 111 dated notes of a fall, the
  runner 312.
- Terms: the balance point is the capture point against what the bearing
  limbs hold; interference is whatever bears or touches, wanted or not, and
  the force through it; clearance the gap of whatever must touch nothing.
  Three levels, each a setpoint to the one below and 3-10 times slower: a
  limb's effort every pass; the balance point's margin a contact's change;
  the margins themselves over seconds, a fall the last one's rare evidence,
  never a price.
- The pendulum's time sqrt(h/g) is 0.30 s: a capture point error grows
  28-fold in a second; a reflex plans a step and no further. Across, a foot's
  gain is that number (the runner's 0.5 Tc + 0.14 s was 0.29 already); over a
  walk's 0.5 s on a foot the gain past which no width holds is 0.446 s.
- The run's row is the runner: asked 1.5 m/s it holds 1.46-1.47, 516 J/m
  drawn (297 of work), 12 s.

## What a walk asked of the law

Each from a trace (2026-10-05):

| Fault | Fix |
| --- | --- |
| a foot bearing 250-500 N mid-swing was no landing until its swing was done | what bears is a contact |
| clearance as a knee's fold slewed 230 deg/s lost to the knee's own 340 unfolding: the heel on the floor at 0.7 of its swing | clearance as a projection: the sole raised to its clearance, the foot as it was |
| the toes counted as the foot came down lifted a forefoot landing 1.6 cm; the run's knee 28 deg where 22, down at 0.8 s | - |
| first steps 6-7 cm apart: the free foot struck the standing one, 180-475 N on both soles' sensors | a landing 11 cm across from a foot still down; the run 7 cm apart, down in 3 s |
| a bounce ended 10 cm up threw the body off the floor, the knee 37 -> 6 deg in 0.1 s | ended on the strut as it will be then |
| both feet down, the height asked fell 1.4 cm to the new leg's reach: both released the body onto a straight leg, 531 N, 0.92 -> 0.5 m/s; it could not pass over the foot (the pendulum's 0.53 m/s at 0.16 m behind it) | carried by the leg that reaches: 110/72 -> 288/210 N over 0.12 s, the pelvis 0.892-0.897 m |
| held over the strut's flat-footed arc for the next landing, the standing heel risen as its leg's length asks: the hip asked 8 deg off, 0.96 -> 0.66 m/s in 0.1 s | the arc ridden down, the knee landing bent: walks |
| the free foot sent 0.48 m ahead to wait (the run's reach) pitched the trunk back 5 deg, its standing hip braked | 0.25 m |

## Walk searches

CMA-ES through the relay (2026-10-05):

| Search | Rows | Walked | Best |
| --- | --- | --- | --- |
| cost: what of its way it did not walk; a fall ends the trial | 1 280 | 583 their 8 s | 20 s at 0.66 m/s, 346 J/m drawn (119 of work); no foot down a tenth of a second a step, the knee 14 deg behind plumb; scoreboard price 50.5, six conditions broken |
| priced as the scoreboard prices a walk (`looks.priced`), a foot always down, the strut the form's 6 deg | 2 240 | 1 357 their 10 s | 30 s, 67 steps, 0.99 m/s at 360 J/m (144 of work) where the walk as built draws 623 at 0.96; price 26.3 |

- The second's form against the walk as built (in brackets): knee landing
  17.8 deg (26.4) and 10.6 behind plumb (9.6, the form's 10); the leg 14.5
  deg behind plumb; toes back 2.1 mm at lift (2); head bob 34 mm (30); strike
  483 N (450); pelvis roll over the stance leg 0.6 deg (3).
- Asked 0.76 m/s it walks 0.99, the speed's integral at its 0.15 m; rolled 3
  deg over the stance leg, 0.64 m/s at 610 J/m.
- Found on one walk, its price is chance: its numbers to three digits, the
  row in `gaits.WALK`, walk 0.95 m/s at 366 J/m priced 92, toes 8.6 mm back
  at lift where 1.9; to four digits another way, 0.87 m/s and a stumble, 336.

## Walk to run

- The rows mixed in a straight line over 4 s: across both ways on the
  setpoints alone, and down 3.6 s into the run, 1.9 s into the walk; the row
  half-way is no gait, down at 2.8 s begun on it (2026-10-05).
- A row between found (cost: what of two passages it was not up for, walk to
  run in 3 s and back): 432 rows, 14 up through both. On the best: walks
  0.90 m/s, runs 1.45, walks 0.95 again, 22 s, the passages a second long,
  begun at 4.0 and 4.5 s; on eight other timings down 3-6 s after crossing,
  into the run at 1.56-1.93 m/s where 1.5 is asked. Held 5 s on the way, the
  row itself goes 0.75 m/s at 627 J/m, both feet down every step.
- Left out: the standing legs pushing toward the asked speed, the pelvis
  asked 0.05 m ahead a m/s slow. Asked 0.76: 0.84 at 379 J/m where 0.94 at
  367; at 0.2 the run down at 8.6 s; passages no better, 1 of 10.

## In main

2026-10-05, the boards as built:

| Row | m/s | J/m | Mean W | Median W | 95 % kW | Max kW |
| --- | --- | --- | --- | --- | --- | --- |
| walk, 12 s, 26 steps, both feet down at each | 0.94-0.96 | 367 | 352 | 233 | 1.19 | 1.93 |
| run | 1.46-1.5 | 517 | 773 | 705 | 2.2 | 4.0 |

The walk as built peaks 4.1 kW at 0.65 m/s. `go.py --search walk|between` is
the search.

## Stand on the law

`hold.py` (2026-10-05):

- The standing legs lean the body toward where its capture point is asked:
  over the middle of the feet standing, ahead by the asked speed over the
  pendulum's rate going. A foot leaves only when a step is wanted: the body
  is asked on, or its capture point is outside the feet's hold (`dcm.due`).
- Standing: 10 s with no step with the pelvis asked 0.2 m per m of it or
  more; down at 3.3 s with none; asked onto the feet's line where nearest,
  nothing held it along it, down at 5.1 s.
- The feet placed across from the CoM's capture point, not the hip's: before,
  they fell short of it and walked off after it.

## Across

- The walk across is the pendulum's (`tools/sim/go.py`'s steps traced): steps
  0.167 m wide (sd 6 mm); the capture point 0.037 m in from the standing
  ankle as the other foot leaves (sd 1 mm) and 0.127 as it lands 0.40 s on:
  3.3 a second from a CoP 4 mm in from the ankle. Of 192 rows about the
  walk's, 133 fell, most within 4 s, all across: a foot down at its 11 cm
  clearance left the capture point 0.09-0.11 m in from it, the next step
  0.33-0.64 m wide on a swing of 0.5 s, the one after at the clearance again
  (0.105, 0.332, 0.099, 0.641 m and down); slowed to 0.38 m/s, a swing took
  1.0 s.
- Fix: a free foot is also due when the capture point has run as far across
  from the standing one as a step's swing takes it from `track`, e^(omega
  t); both feet down, the legs bring it to `track` in from the foot that
  stays (across with both down alone: held over one foot it stayed there,
  the free foot came down 11 cm from it and the body went off across at 1.5
  m/s); the other leaves when it is no further in than that and 2 cm.
- Result: stood, asked on over 0.1-3 s, walked at 0.64-0.77 m/s, asked to a
  stand and stood, 20 s: 10 timings of 10 where 0 of 4 before, down in the
  stop at 15-17 s. Slower walks hold: asked 0.19, 0.38 and 0.57 m/s it goes
  0.13, 0.36 and 0.49, 20 s from a stand and from the walk. Of 32 rows about
  the walk's, up 14 s on three placings: 9 on the law as it was, 14 with the
  stand's hold, 19 with these.

## Shoves

`go.py --shove`, the page's 38 and 120 N for 0.12 s from eight ways
(2026-10-05):

| State | Nudged 38 N | Shoved 120 N |
| --- | --- | --- |
| standing | 8 of 8, no step; speed 0.10 m/s at most, still in 1.7 s | 0 of 8 within 2 s, 0-4 steps (a step barely moving takes its swing's 0.48 s) |
| walking | 7 of 8 | 5 of 8 |

As the standing reflex has it (`machine.stand`: the foot that can land where
the capture point goes, on the ray through it, due as it has run): down on
all 8 still, and after a walk stood on 2 timings of 10, 7-16 steps each; a
foot down on the ray leaves the capture point 2 cm from it, on the edge of
its hold, and steps again. Not in the law.

## Steps and the jog

- One rule for a walk's steps and a run's (2026-10-05): a foot leaves `both`
  s after the other landed and, `both` under 0, by as much before that one
  is due; the run's stance of `stand` s is the same rule. The free foot is
  due the other's `step` on, and by where the body is the more both feet
  bear a step, in full from 0.05 s of it. The walk's and run's rows as
  before: 0.78 m/s at 397 J/m and 1.44 at 522.
- The jog: the run's row at the walk's speed, the row between walk and run.
  On its speed alone the run's row holds:

| Asked m/s | 0.6 | 0.8 | 1.0 | 1.2 | 1.5 |
| --- | --- | --- | --- | --- | --- |
| held m/s | 0.54 | 0.74 | 0.99 | 1.19 | 1.46 |
| J/m | 936 | 704 | 602 | 547 | 521 |

  With its speed's gain twice a walk's where it flies; on a walk's, asked 1.0
  it ran 0.7 for 3 s, then away past 1.47 and down at 7.5 s.

- From the walk to the jog over a second: up 8 timings of 8; back to the walk
  6 of 11, the steps 0.13-0.48 m by turns, a foot caught 0.13 s into its
  swing; on to the run 2 of 4. Out of the walk the jog bounces 11 cm (the
  pelvis 0.826-0.940 m, the knee to 79 deg), leaving the floor at 0.72-0.86
  m/s where 0.49 is asked, a step 0.43-0.54 s; asked faster it leaps further
  and is down. A stance ended on rising as asked: down at 1.0 s at any speed.
- Tried on the stand and left out: a swing's clock, a foot due a step after
  the other landed at most (the walk settled at 0.59-0.61 m/s for 0.76, the
  knee landing at 28 deg); the calves as on the page (`arrival.CALF_DEG_M`),
  standing on both feet (a nudge's 0.08 m/s where 0.10, still in 1.1 s where
  1.7; stood again after a walk on 8 timings of 10, down 17-18 s in; on with
  every step barely moving, 6 of 10).

## Between two gaits

Their mix is no gait; a third row is (2026-10-05). From the jog, each of the
walk's shares taken alone and held:

| Taken | Result |
| --- | --- |
| its time | up |
| its bounce, its landing or its swing | down in 2 s |
| bounce and landing together | up; with its swing too, up |
| all at once | down: a stance that still flies ended falling, -0.5 to -0.7 m/s as the foot left, the next landing at 1 m/s |

`gaits.EASE` is the walk's shape on the jog's time, a knot between them:

| Passage | Result |
| --- | --- |
| through EASE, half a second a move, 2 s on it | to the jog 23 timings of 24, back to the walk 24 of 24 |
| a second a move | 2 of 6 to the jog |
| walk and jog mixed straight over 0.6-3 s | back 12 of 20 |
| through EASE with no stay over 2-6 s | 12 of 24 back, 8 of 24 on |
| the row taken up only as a foot lands | every way worse; to the jog 2 of 8 |
| the bounce lasting the stance where it flies | back 5 of 16 |
| the rise at leaving fed back a step after another, 0.1-0.3 of it | the run's row down at 1.2 m/s |

- Jog to run: asked faster over 3 s the bounce grows (down at 0.57, 0.61,
  0.68, 0.74 m/s; leaving rising 0.71, 0.83, 0.96 where 0.49 is asked; the
  knee to 75 deg) and it falls; over 5 and over 10 s it runs 1.5 m/s, coming
  down 0.23 m/s at speed where 0.54 jogging. `gaits.toward` is how fast an
  asked row is followed: a second from the stand to the walk; half a second
  between the knots about EASE and 2 s on each; 5 s from the jog to the run.
- Tried on the way between gaits and left out: a flight no longer than the
  rise carries (0.10 s at the run's 0.49 m/s, none on the row between): ways
  up 23 of 50 where 53 of 56. The free sole kept clear to 0.9 or 0.95 of its
  swing: one walk in both worlds, its knee landing at 34-37 deg.

## Ways over a spread

`tools/sim/ways.py`, 92 trials (2026-10-05):

| Way | Up |
| --- | --- |
| stood, walked, stood again | 10 of 10 |
| to the jog and to a stand again | 8 of 8 |
| to the run (1.5 m/s) and to a stand again | 11 of 12 |
| turned back on its way | 6 of 6 |
| the run held a minute | 2 of 2 |
| slower walks | 6 of 6 |
| total | 43 of 44 |

- With the standing legs' hold at 0.4 of the page's: 27 of 38; stood again
  after a walk the CoM crept 6 cm back and 3 across in 1.6 s, a step fell due
  and it was down 5 s on.
- Nudged (38 N): standing 8 of 8, walking 7, running 4. Shoved (120 N):
  standing 0 of 8, walking 4, running 0.

## Under the director

`machine/pace.py` (2026-10-05): standing settled in the arrival's stand, a
pace asked, the law takes the body where it is, its legs standing where the
body's are, its setpoints eased from the director's over 0.3 s. From the
squat: rose, stood, walked 0.82 m/s, jogged, ran 1.5 m/s from 36 to 44 s,
back at the jog at 50. Down, the director's fall and get-up take over; risen,
the body stands for the law again.

- The walk's row had two walks (2026-10-05). Asked 0.76 m/s: 0.81, the knee
  landing at 18 deg, a step 0.49 s, the feet placed at the row's reach 0.33
  m ahead; or 0.64 m/s, the knee landing at 28 deg, a step 0.54 s, the feet
  0.28 m ahead (slow, the law placed them nearer to speed up, and nearer
  under the same pelvis the leg came down bent and the step short). The start
  decided: the feet 0.19 m apart where 0.17, or the director's stand under
  it, gave the second; its jog then 1.1-1.2 m/s for 0.8, the page's presses
  up 7 timings of 12 where the tool's world had 11. Asked 0.80 or more there
  is one, 0.82 m/s.
- Fix: one walk at 0.76 from every start within 4 s, the knee 18 deg, a
  walk's step never shorter than the asked speed has it, the standing leg
  leaning the body 0.5 of its speed's error over the pendulum's rate where
  0.2 (`hold.LEAN_K`), 0.2 still where it flies (at 0.5 there the ways were
  up 47 of 104 where 80). The page's presses under the director: up 20
  timings of 24, down as the run speeds up.

## On the page

`terminal/views/humanoid_keys.py` (2026-10-05):

- J hands the going to the law at its walk (landed anew, risen, standing,
  then on); S and F step the asked row: the stand, two slow walks, the walk,
  the jog, two faster, the run. J again: the walk as built, S and F its
  cadence.
- The law's walk was the page's own for an evening: on stilts, the arms bent
  and not swinging at the sides. The walk as built's again until the law's
  is a woman's.
- On the walk as built the keys moved its cadence 0.60-0.90 strides/s,
  0.39-0.72 m/s (the meter two cells down and one up); to 1.0 since
  2026-10-06: 0.90 m/s, four minutes on at it with the hottest drive at 0.35
  of its span; 1.0-1.08 m/s at 1.05; down in 2 s at 1.10.
- On the law, driven as the keys drive it: walked 0.6-0.8 m/s, the jog, the
  run 1.3-1.7 m/s for 10 s, stood again at 60 s; walking 200 s the hips'
  laminate 60 C, 0.37 of its span spent, nothing derated. The floor's events
  hand the body to the walk as built; its style does nothing on the law.

## Against a woman's walk

[normal](normal.md), `tools/sim/normal.py` (2026-10-05): 4.1 off the band
where the walk as built 2.0. Swinging knee 26 deg (52-78); 12 as its toes
leave (30-60); the heel 1.5 deg up then (28-62); the pelvis rolling 1.9 and
turning 0.9 deg (5-15, 4-22); the feet 0.29 legs apart (0.02-0.22); the step
0.81 for its time (1.1-1.8). The arms were the runner's, elbows 80 deg and
still; now the row's (`elbow`, `play`; 24 deg bent, 16 of play,
`machine.free`): 394 J/m where 424.

Tried, each alone on the row, 16 s from a stand:

| Change | Result |
| --- | --- |
| the knee's fold 30 and 45 deg | swinging 41 and 56; toes 40 and 81 mm back at lift; 0.70 and 0.63 m/s for 0.76 |
| a step of 0.60 s, both feet down 0.10-0.12 s of it | toes not back; 0.86-0.90 m/s; head bob 36-52 mm; the knee 15-17 deg behind plumb |
| the pelvis rolled 4 deg over the standing leg, none at a landing | rolling 10 deg, the feet 26 cm apart |
| the heel risen 35 deg through the double support | the knee 54 deg at lift and 53 swinging; 0.90 m/s at 591 J/m; ways up 28 of 104 where 77 |
| with the fold 25 or the longer step as well | down in the first steps: a 0.6 s step took the pelvis 6-8 cm down a flat foot's arc and the next leg's bounce threw it off the floor |

In, and of no weight on the row as it is: the pelvis kept up where the free
foot's landing meets the floor, the standing heel rising as that asks; the
heel's rise through the double support, the leaving foot's alone; the
pelvis's roll none at a landing, the spine against it.

## A woman's walk on the law

`gaits.WALK`, `QUICK` (2026-10-06):

- On the walk as built's time (a step 0.59 s, both feet down 0.16-0.20 s of
  it, the free foot's reach 0.32 m): the standing heel rises 28-38 deg by its
  strut's need as it leaves, the toes not back, the strike 340-400 N, the
  feet 12-14 cm apart: 0.78 m/s at 471-514 J/m, priced 22 where 60-105.
- With the knee's fold 40 deg, the thigh coming on half of it, the pelvis
  listed 4 and turned 4 deg (a row's `turn`, the waist against it): 615 J/m,
  1.30 off the band.
- Searched from there (CMA-ES, 24 knobs, 1 920 rows, a walk's price and 15 a
  width off the band): 0.34-0.51 off it, `looks.FORM` met, at 690-776 J/m
  (the ankles' work 101 J/m where 31, the knees' 100 where 64, the hips' 83
  where 59), its stops 3 of 10: the search's constants were every row's,
  and the stand's.
- Searched again on the row's own setpoints and the walk's own ways, its J/m
  by 25, a stop and a passage to the jog and back among its trials (960
  rows): a step 0.52 s, 0.13 s of it on both feet; the fold 45 deg with the
  thigh coming on 0.32 of it; the pelvis listing 3 deg and turning 1; the
  feet's track 0.044 m. In: 0.81 m/s at 574 J/m, 0.34-0.41 off the band (the
  heel 20 deg up as its toes leave where 28, the knee 27-28 where 30);
  `looks.FORM` but the head 38 mm aside and the standing hip 2.9 deg up where
  3; in words stiff 0.31, shuffling 0.24, no stilts.
- The row first found is the quick step, a knot between the walk and EASE: on
  the walk's own shape every passage to the jog was down in a step.
- Ways 68 of 104 where 77 (nudged walking up 4 of 8 where 7; shoved 0 where
  4; to the jog and back 6 of 8 where 8); the page's presses under the
  director 23 of 24 where 20.

## Falls on the way and their fixes

Each by trace (2026-10-06):

| Fault | Fix |
| --- | --- |
| both feet down, the rear kept for the capture point while the hips went on at 0.8 m/s: carried past the lead foot on the rear toes, the lead sole at 5-100 N | a moving foot leaves on its time (`hold.WAITS_M_S`) |
| the capture point over or outside the standing ankle through a single support at a track of 0.035 m, the next foot landing inside it | leaned back in as it nears that ankle's line (`hold.IN_M`) |
| a stop on a long step, the feet 0.18 m staggered: crept back, stepped about | the rear foot brought up beside the other (`hold.CLOSE_M`) |
| standing after a stop, the pelvis 3 mm over what the legs reach flat: a heel rose to reach, its ball pushed the body back, the further back the more | standing no higher than both legs reach flat |
| a first step laid for the asked speed, 0.25 m ahead at 0.2 m/s, could not be passed over | laid for the pace and 0.36 m/s more (`going.GAINS_M_S`); a free foot down 1.5 steps after the other at the latest (`LATE`) |
| the fold alone: the toes 40-81 mm back | the thigh comes on with it (`free.LEAD`) |

Tried and out: the heel risen through the double support while that leg
still bears (stuck, and a merely standing foot rose on its heel as the row
turned; given to the load's passing it is in and does nothing at this row's
2 deg); the free foot's toes pinned through the first of its swing (the
parry's steps are done in a tenth of a second); the pelvis neither listing
nor turning while standing (down in the first steps, a searched row leaning
on its list from the first).

## The strut a row's setpoint

`strut`, deg: the standing knee the pelvis is never over, 6 the form's
(2026-10-06). Bent more, the walk's row draws less:

| strut deg | 6 | 10 | 14 | 18 | 24 |
| --- | --- | --- | --- | --- | --- |
| J/m | 574 | 528 | 503 | 502 | 500 |

At 10 deg 0.25 off the band where 0.41, the knee 29-30 deg as its toes leave.
The straight leg's cost is how it is driven (the energy), not its shape.
Asked 0.6 m/s on the walk's step time: 0.59 at 848 J/m, a step 0.39 legs, the
walk ratio 0.73 where 1.1-1.8, the heel 7 deg up as its toes leave; stiff
0.61, shuffling 0.67, tripping 0.53: a shuffle on stilts. A woman's step
keeps its ratio to its time at any pace; the law's slow walks keep the walk's
time.

## What the walk draws, by joint and phase

2026-10-06; the draw is the drives' work, braking returning nothing, plus
their copper's heat and the boards' own 53 W. At 0.84 m/s, 570-582 J/m: 279
of work, 229 copper, 63 the boards'; 196 braked (the knees 85 worked and 80
braked, the ankles 108 and 74, the hips 69 and 32).

- Half of it in three phases of a stride: a landing's first 0.1 s, 90 J/m;
  the other foot's landing, 49; the foot's leaving, 135. There the knee was
  asked 23 deg more in a pass and the ankle 13, its toes' point counted under
  the floor as the heel is up and raised at once (`free.lifted`).
- A walk's foot eased up to its clearance over the first fifth of its swing:
  502 J/m (four starts, 496-516); as the page runs it 485, 0.20 off the band
  where 0.34; a sole's strike 352 N where 428; the head bobbing 33 mm where
  30. A run's is drawn up at once: eased over any of its swing it was down on
  its way back to a stand, 1-2 timings of 12 where 11-12.
- A foot on its heel has its heel's rise only as its toes come down
  (`strut.rocker`): taken about its ball at once the knee was asked 8 and 21
  deg within 20 ms of a landing: 8 J/m.
- Ways 69 of 104 where 68; the page's keys 12 of 12 where 9; its presses
  under the director 66 of 72 either way.

| Tried and out | J/m |
| --- | --- |
| the lead leg a strut from its landing, its knee from its landing bend to the row's | 531 where 506 |
| what stands eased over a foot's landing or leaving | 514 where 512 |
| the fold a smooth bump over the whole swing (knee 17.5 deg as its toes leave where 27, stiff 0.46): out until the knee bends before the toes leave | 446 |
| the free foot's ankle by its own way, not level | 576 |
| the leaving knee bent as its load passes, 10 deg | 527 |
| the same, 30 deg | down in the first steps: its foot bears 345 N again as it pushes off, the load passing twice |
