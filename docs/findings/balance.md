# Findings: her balance

The gynoid kept up: the capture law and the side step, the floor's events and
shoves, the scoreboard and its searches. The board's own are in
[FINDINGS](../FINDINGS.md).

- The finest of 576 walks (16 cores, cross-entropy, 23 knobs), judged by the
  stance knee past 5 degrees, the soles' 20 ms peak, the pelvis's shake, the
  head's travel and the copper: bend 11.7 -> 5.7 degrees, peak 2.9 -> 2.2
  body weights, head 8.5 -> 6.0 cm, heat 0.47 -> 0.29, work 0.60 -> 0.35 m g
  d. Cost of transport 0.64: the cheapest, knees bent, 0.52; the old walk
  1.07. Straight legs cut the copper by 37 % (2026-09-26).
- Twelve trials a candidate (`tools/sim/gait_montecarlo.py`: three rises,
  three walks, six shoves of 14 N s), 19 s on 16 cores: she holds 85 % of the
  time, every side shove fells her. Shoved toward the standing foot, the catch
  steps the other across it; held off, she falls 1 s later; the standing foot
  lifts on the clock while the other still reaches. The capture point's law
  on the plan's reference fells her walking: the plan's sway is not the
  pendulum's - the DCM error in a steady walk 93 mm rms at liftoff
  (2026-09-26).
- The capture point from the pelvis's sideways speed, not the centre of
  mass's (the swing leg's speed is in that: a wide step put it 6 cm out past
  where it went); its course 0 -> 51 mm out over the swing at 0.65, 0.85 and
  0.9 strides/s; the ankle holds what the course leaves of the sole (at 0.65,
  25 mm off at 0.7 of the swing doubled); the foot 1.1 of what is off further
  out - at 1.3 the step back grew 1.35 a step (`machine.capture`,
  2026-09-26).
- Landings: the height's target from where the body is (from the plan's, 3 cm
  up, both legs threw her 5 cm into the air); the forward target moved no
  faster than 0.3 m/s (a foot 5 cm short snapped the knee straight); the phase
  at most twice its pace (it raced to 3.5 strides/s). The side step: the
  swapped foot down over 0.1 s ahead of the pelvis, the other out over 0.15 s
  to the capture point foreseen once, set down by a sine squared, eased on
  from where it stood; over once it bears her; the walk begun again on it at
  the speed she has, the other to step in beside. The first steps narrow over
  0.6 s: over 2.5 s the third lifted with the capture point 8 cm inside the
  standing foot. Twelve trials: held 73 % - the rises, the walks at 0.85 and
  0.9, the shoves along the line; the side shoves still fell her (the side
  step ends with the capture point 27 cm ahead and no foot there), and the
  walk at 0.65 from its own start pose drifts 7 cm across in its first second
  (2026-09-26).
- What does not help: a toe-off gate on the capture point (the legs driven
  3.5 cm across moved the pelvis 3 mm in 0.2 s; walking on, the frozen plan
  sank her 17 cm); the pelvis driven by the capture point's error (the foot
  slid 20 cm) or by the course's deviation alone (down in 1.4 s): the plan's
  sway servo stays (2026-09-26).
- The director's slip re-anchoring at 4 cm (at 2 cm the feet's slides under
  the ankle's drive at 0.65 re-anchored eight times in 2 s and she fell) and
  the side step's out foot set where the pelvis will be, kept there: twelve
  trials held 87 % - the walk at 0.65 and the shove toward the swinging foot
  at 0.85 hold too. Still felled: the shove toward the standing foot (the side
  step ends with the sagittal capture point 27 cm ahead and no foot there)
  and the shove at 0.9 as the swinging foot lands. Hurrying the phase on a
  catch latched the landing early and 9 cm short; a swinging foot held as
  landed once it bore mid-swing reset the phase at every touch: both out
  again (2026-09-27).
- The side step's foot–foot strikes: the stepping foot's toes struck the
  put-down foot's heel going by (5 cm between 10 cm wide feet, 1800 N), so
  that foot is put down 12 cm across at least and the other goes out before
  on; pitched from its swing, a put-down foot's toes took 1000 N 9 cm up, so
  borne means the ankle within 1 cm of the floor; a height still lowered for
  a catch shortened the trailing leg 3 cm as the step out began, so that leg
  takes the height from the body. A swap asked is a fresh judgement each
  pass (latched, one was done 0.2 s later on the wrong side), none in the
  first 0.15 s of a walk begun again, which blends in over 0.1 s (over 0.3
  the trailing foot stayed down and its share of the load drove the capture
  point on past the standing foot). A catch out of reach is a stomp: the
  foot put down at once where the capture point will be 0.1 s on. Still 87 %:
  the left shove at 0.85 zigzags into a second and third side step; slewing
  the attitude's correction at 0.5 rad/s let the roll grow through the step
  (2026-09-27).
- A search (CMA-ES, 144 candidates) over the torso's counter, the crane
  damping and the capture law's margin and gain: cost 8.2 -> 5.9 (stir 4.4
  -> 1.8 mm), but re-scored with the damping 0.556 -> 0.55 the walk at 0.9
  fell in 4 s, and a 0.1 % change of any knob flipped a shove: landed at 5 s,
  the shove met whatever stride phase the pace had brought her to (0.25 at
  0.85 strides/s, 0.48 at 0.65 and 0.9, 0.07 less with the counter). The
  shove now lands as the phase first crosses 0.30 after 5 s. There the
  counter alone, 3.9 degrees, takes the stir to 2.4 mm, held 86 % against
  84 (3.5-4.5 alike, one corner of margin and gain felled a walk); margin
  43 mm and gain 1.1-1.27 add 0.1 mm and noise; the damping stays off. At
  0.30 the shove toward the swinging foot fells her as it is: the landing
  latched at 0.7 of the swing with the capture point 9 cm out, it ran to 23
  and she toppled over the foot (2026-09-27).
- The floor's events (`physics.World.terrain`) in the shoves' place on the
  scoreboard, a shove hardly ever happening to a walker: a hole 3 cm deep
  and 40 cm long under the left foot's next landing (the floor a slab over
  a plane that deep, cut in two), a sill 4 cm high 15 cm ahead of its toes
  as it lifts, a patch at 0.06 under the landing, a loose rug (0.1 on the
  floor, the sole's own grip on it) its front edge 15 cm short of the
  landing. Her toes skim at 3 cm through the first 0.16 s of a swing: a 2
  cm sill they shoved at with 200 N and went over, at 4 with 350 N, the
  pelvis tipping 8 degrees, and the swing carries them over. The patch at
  0.15 let the stance foot creep 3 mm (the walk asks 0.17 of the floor), at
  0.06 it slides 10 cm back under the push-off and she walks on, tipped 6.
  The rug at 0.3 lay still (the sole's shear 100 N, its hold 165), at 0.1 it
  goes with the push-off and she falls at 7.7 s. The hole fells her at 6.6
  s: the foot finds no floor where the plan lands it, the leg holds it 3 cm
  short, the body falls 14 mm and runs on, a catch lifts the wrong foot. A
  geom moved or grown past its compiled bounds is missed by MuJoCo's
  broadphase (the rug fell through a slab grown 27 m, a box through a sill
  moved 1 m): the slabs are compiled over their whole span and cut, the
  sill and the patch are mocap bodies. Held 93 % (2026-09-27).
- A drive's glitch (`physics.World.glitch`, the peak torque cut to a share
  for a while) on the scoreboard: the left knee's gate dropped for 0.15 s
  at mid-stance ('cut') gives 19 degrees under 500 N, the pelvis 16 mm,
  and takes her weight again, tipped 4; the knee derated to a tenth for 2
  s ('hot', 25 N m) folds 18 -> 64 degrees in 0.3 s and the pelvis sinks
  20 cm, past the height latch's 12: the other leg, reaching from a target
  the body is no longer at, holds its foot in the air and she falls
  backwards at 5.7 s. Walked on straight (the pelvis at the weak leg's
  reach, its step 8 cm short) it fell sooner: a knee already folded cannot
  straighten under her, and the raised target hung the other foot; the cut
  knee, straightened the same way, fell too. The hip held to a quarter for
  a second changed nothing (a stance hip asks under 60 N m). The swinging
  foot lands on the capture point along the walk now as across it (the
  body's speed over the plan's, over omega, 25 cm at most, `landing.FORE_K`):
  the walks' stir 3.70 -> 3.52 mm and 3 % further, the rises held; slowed
  to a stop by the folding knee, the foot had come down where the plan
  had it, ahead of a body going nowhere (2026-09-27).
- A 6 cm sill caught the skimming toes and she fell; a swinging foot
  bearing 80 N is lifted 8 cm more until its heel clears it, the other
  too (`landing.over`): the page's sill, 13 places 3 cm apart, she walks
  on at 4 where 2; the stair, 3 steps where 4 (2026-09-28).
- A 5-step stair of 8 cm, blind, the floor read where each foot bears and
  the step expected again: she walks up all five, and pitches over at a
  landing a step lower (2026-09-28).
- The lace a snag (`World.lace`): 6 of 9 went taut, 5 felled her
  (2026-10-01).
- P's 120 N felled her back to 279ba6f. Shoved 30, 60, 120 N 48 times, HEAD
  held 33, 10, 1, the parry 38, 21, 0 (`landing.PARRY_HURRY`, 2026-10-01).
- The scoreboard's events each laid at three places (`gait_montecarlo`
  SPREAD: 3 cm along the walk, a glitch 0.05 of the stride), a trial its
  spread's mean, 30 runs a candidate: the committed arms and the same moved
  2 % cost 7.63 and 7.10 where one run a trial gave 6.08 and 8.84; the arms
  from the forearm 8.61, the slip at 0.9 strides/s held 55 % (2026-09-28).
- What she trips on (`machine.events`, `tools/sim/look.py --event`): the
  4 cm sill tips her
  6 degrees and she walks on; a lace pulling the lifting foot back 120 N
  for 0.15 s, 7; 250 N for 0.2 s and past it she falls, the director's
  fall declared 0.15 s after it, before the foot is free, the standing
  foot off the floor as she curled. Curled into the squat, the arms out,
  the head met the floor at 0.35, 0.10 and 3.05 m/s after laces of 300,
  400 and 600 N, 1.71 in the hole, 0.32 on the rug; onto all fours, the
  waist turned toward the way she tips as it turns, once in the five, at
  0.5-0.69 m/s, the hands first but for the rug (a hip, falling ahead and
  aside, roll 0.6 -> 36 degrees as she went) (2026-09-28).
- P's shove to her left at phase 0.14 on the merged feet (the box sole, the
  ankles on frame A): the catch overshoots to her right, the pelvis sinks
  0.89 -> 0.62 m over 0.4 s before FALLING_M calls the fall, fallen 0.12 s
  on, the hands down 0.27 s in with the arms 65 % into their catch, the
  head at 1.74 m/s (test_gynoid_falls' crouch, 1 of 16; green before the
  merge; the toes driven in this build). Called 0.14 s earlier on a sink
  past 0.6 m/s, two falls struck, 1.68 and 3.41; the curl over 0.25 s,
  3.22 (2026-10-04).
- The scoreboard at the day's end (`--suite all`, 28 trials, 2026-10-04):
  729 and 65.7 % - the rises 76, 100, 99; the walks 71, 100, 31, 12 at
  0.65-1.0; the events hole 77, sill 100, slip 76, rug 29, the sill at
  0.65 19, the slip at 0.9 18, SOA 77, hot 100, lace 26, nudge 54; the
  shove 27; standing nudges and the stiff board 100, shoves 49, bricks
  49 and 53, the rocker 62 and 85. The fast walks and the catches are
  the gap.
- The catch's gain and hurry on the faults suite (2026-10-04, 33 runs a
  cell): as built (1.108, 0.25) cost 178 and held 54.9 %; the gain 1.0 and
  the hurry 0.15 held 64.8 but cost 375 - the shove's fall landing on a
  gearbox past its rating -; 1.0 and 0.25 442 and 64.6, 1.108 and 0.15
  311 and 60.3. The built values stay; a blind 6-knob search at 8 x 12
  overbooked the relay and was stopped unlogged.
- The grinder's first round (`tools/sim/gym.py`, gemma4:12b proposing,
  2026-10-04): its own pick, capture.MARGIN 0.03-0.06 and GAIN 1.1-1.3 on
  the faults suite, 1 x 10 in 698 s; the best 296.9 and 53.2 % at 0.0319
  and 1.10, the built 178 and 54.9 staying. Its rounds 2 and 3:
  landing.PARRY_M 0.02 and PARRY_HURRY 0.234 on faults 317.3 and 58.5 %;
  stand.GAIN and STEP_MAX_M on faults 178.2 and 54.9, a no-op - standing's
  knobs reach no walk. Its second run, Clef picking the suite: stand.DWELL_S
  0.15-0.6 and HANG_S 0.05-0.3 on the stand suite, 1 x 10 in 273 s, the
  best 178.2 and 76.0 % at 0.368 and 0.15 against the built 215 and 74.7
  (0.4 and 0.1); one round, not baked.
- The day's build (2026-10-04: the toes sprung at `TOE_K` 10, the heel off
  at 0.5 and 40 deg, the one law standing): the scoreboard 641.7 and 78.6
  % over 28 trials against the morning's 729 and 65.7 on driven toes - the
  sill and the slips 100 %, hot 93, the rug and the nudge 77, soa 54, the
  hole 33, the lace 26, the stand suite 77.5, the walks 100, 100, 90 and
  80. Fallen at 10.5 s to P's shove she walks again at 33.2 s. The tests'
  single runs flip with the push-off: at 30 deg the falls suite 44 of 44
  and the faults' walk-on down; at 40 the faults 9 of 9 and her head on
  the floor twice, 4.31 m/s at most.
- The grinder on the day's build (`gym.py`, its knobs by the schema,
  2026-10-04): its first round, the stand suite 1 x 8 over
  `bearing.PRESS_M`, `SEEK_M_S`, `stand.HANG_S`, `HANG_SHARE`, best 146.8
  and 80.9 % at 0.0078, 0.294, 0.130 and 0.453 - the built values' own
  neighbourhood (0.01, 0.3, 0.15, 0.45; built 77.5 %), a best of eight.
- Her arms aimed as she tips (`falls.aimed`, 2026-10-04). P's shove to her
  left at phase 0.14 had put her head on the floor at 4.31 m/s: the catch
  folded her knees, the fall was called at 0.65 m of pelvis with her trunk
  3 deg off plumb, the way read behind her and the arms went back, the
  shoulders to -46 deg; called down 0.07 s on, she pitched onto her face
  0.37 s later. Aimed anew every pass past 12 deg of tip until an arm
  lands: of the 16 crouch falls her head never on the floor in 14, at 0.71
  and 0.87 m/s in two, the peaks' medians 4.1 and 3.7 kN.
- The catch against her own walk (2026-10-05, `capture.CATCH`). At 4 cm,
  from the squat at 0.85 strides/s she parried a plain floor in 2 runs of
  6 a side gain 0.1 % apart, 16 and 10 times, at 1.0 in 5 of 6: her
  start's last strides run 44-74 mm off the course, and early in a fast
  swing her own leg's swing-out - the hip's roll 5-6 deg past its
  setpoint, the ankle 6-7 cm out - jerks the pelvis 12 mm the other way,
  34 mm of landing asked in a clean step, 60 in a parried one. At 7 cm
  none of 12 from the squat; the events held 48 of 90 where 47 (hole 1
  and 0, sill 6 and 7, rug 1 and 0, slip 15 and 16, soa 8 and 7, hot 9,
  lace 0, nudge 8), the five gynoid suites as before.
- Begun standing (2026-10-05). The director starts every walk at 0.85
  strides/s and glides to her pace; the scoreboard set the walker's pace
  over that and 2 walks of 12 fell, 52 parried (the head before the strut:
  none, 222, 67-82 a walk at 0.65): at 1.02 she ran to 1.0-1.1 m/s where
  its plan has 0.85, the catch on her three steps running every 2.6 s,
  down in its third round, 4 runs of 6; at 0.88 her fifth step landed late
  on a knee at 23-26 deg, asked straight in 40 ms under 752 N - 17 N on
  both feet, 3 cm up, 0.56 -> 0.27 m/s - and she fell 2 s on. One knob at
  a time under it, the pre-swing at 26 and 28 1 fell, the let-go's easing
  0.12 1, 0.15 none, 0.18 3, 0.22 7: no order in it. As the director
  starts her all 12 hold on their form. Her standing start stays on an
  edge: from its second step the capture point runs 51-65 mm off the
  course, her weight not over the standing foot as the other lifts, and
  its fourth lands 130 mm wide (119 from the squat); of 6 a side gain
  0.3 % apart 1 fell and 2 parried 3-19 times, and over ramps of 2.5, 3.5,
  4.5 and 6 s (`stance.RAMP_S`) by 3 gains 2 fell, 3.5 and 6 clean.
- Her speed is her own (2026-10-05). The phase follows her
  (`stance.advance`) and nothing holds her to the pace asked: at 1.0
  strides/s she walks 0.96 m/s where the plan has 0.83, 581 W. A foot
  landed further ahead of a body going fast (`landing.FORE_K`) is no
  brake, it lengthens her stride: read against the pace asked instead of
  the phase's rate, 0.65 -> 0.68 m/s at 0.85 strides/s, at a gain of 1.5
  0.83 and 639 J/m. The foot under the hip later in its stance
  (`gait.STANCE_AT` 0.265): 0.78 m/s and 349 W at 1.0.
- The roll's peak against her starts (2026-10-05, 6 side gains 0.3 %
  apart; `gait._roll`'s peak, of the stride). At 0.33, as built, from the
  squat none of 12 starts parries and standing 1 of 6 falls, 2 parry; at
  0.29 from the squat 4 of 12 parry, 13-26 times, standing 3 of 6, one 85
  times; at 0.27 standing none of 18, from the squat 10 of 12, 7-47 times.
  Steady she is on her form at all three. The course learned as her own
  walk's - each leg's, a swing that was no catch taken into it, 0.4 of
  it - changes none of it and is not taken: at 0.65 strides/s 437 J/m
  where 383 and her toes back 4.2 mm where 0.6, the table's 15-38 mm off
  there landing her feet wider.
- Her first step is a fall toward the swinging foot (2026-10-05, begun
  standing, the feet 161 mm apart). As the right foot lifts her centre of
  mass stands 39 mm inside the left ball - the keyframe's 27
  (`arrival.LIFT_IN`), the sole's edge at 45 - and the capture point 35
  mm inside and going on, past the edge 0.1 s later; through the lean her
  weight has come back, 140 N a foot where 230 and 50 after the shift.
  The foot lands 126 mm from the standing one where the walk's track has
  60, and the feet come in step by step on the capture law's asking, the
  walk's plan narrow from the first (`Walker.begin`, no `wide` since
  2026-09-27). Shifted further by the keyframes (`SHIFT_IN` 0.03 where
  0.043, `LIFT_IN` as built and 0.015) she is down in every start of 30,
  standing and from the squat; over 0.9 s (`SHIFT_S`) the standing ones
  hold, 0-10 parries, from the squat still down: a shift on a clock
  overshoots the sole, as on 2026-09-26.
- The lean before the shift (2026-10-05, `arrival.keyframes`): leant on
  both feet, then onto the left, the foot lifted as her weight comes over
  it, the capture point 24-28 mm inside the standing ball at the lift
  where 35. Of 30 starts, standing and from the squat, 6 side gains 0.3 %
  apart: 29 with no parry and 1 down - begun standing, glided to 1.02
  strides/s - where 21, 3 down and 6 parried. The events held 48 of 90 as
  before; shoved down at 1.0 strides/s she is up and walking 22.5 s on.
  With the roll's peak at 0.27 the standing starts all clean, from the
  squat 1 gain of 6 parries, 27-33 times.
