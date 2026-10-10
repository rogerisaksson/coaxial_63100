# Findings: balance

Kept up walking: the capture law and the side step, the floor's events and
shoves, the scoreboard and its searches. The board's findings:
[FINDINGS](../FINDINGS.md).

## Walk search and the capture law

- The finest of 576 walks (16 cores, cross-entropy, 23 knobs), judged by the
  stance knee past 5 deg, the soles' 20 ms peak, the pelvis's shake, the
  head's travel and the copper: bend 11.7 -> 5.7 deg, peak 2.9 -> 2.2 body
  weights, head 8.5 -> 6.0 cm, heat 0.47 -> 0.29, work 0.60 -> 0.35 m g d.
  Cost of transport 0.64 (the cheapest, knees bent, 0.52; the old walk 1.07).
  Straight legs cut the copper by 37 % (2026-09-26).
- Twelve trials a candidate (`tools/sim/gait_montecarlo.py`: three rises,
  three walks, six shoves of 14 N s), 19 s on 16 cores: held 85 % of the
  time; every side shove fells it. Shoved toward the standing foot the catch
  steps the other across it; held off, it falls 1 s later; the standing foot
  lifts on the clock while the other still reaches. The capture point's law on
  the plan's reference fells the walk: the plan's sway is not the pendulum's,
  the DCM error in a steady walk 93 mm rms at liftoff (2026-09-26).
- The capture point from the pelvis's sideways speed, not the CoM's (the swing
  leg's speed is in that: a wide step put it 6 cm out past where it went); its
  course 0 -> 51 mm out over the swing at 0.65, 0.85 and 0.9 strides/s; the
  ankle holds what the course leaves of the sole (at 0.65, 25 mm off at 0.7 of
  the swing doubled); the foot 1.1 of what is off further out; at 1.3 the step
  back grew 1.35 a step (`machine.capture`, 2026-09-26).
- Landings (2026-09-26):

| Change | Fault it removed |
| --- | --- |
| the height's target from where the body is | from the plan's, 3 cm up, both legs threw the body 5 cm into the air |
| the forward target moved no faster than 0.3 m/s | a foot 5 cm short snapped the knee straight |
| the phase at most twice its pace | it raced to 3.5 strides/s |

- The side step: the swapped foot down over 0.1 s ahead of the pelvis, the
  other out over 0.15 s to the capture point foreseen once, set down by a sine
  squared, eased on from where it stood; once over, it bears; the walk begun
  again on it at the speed it has, the other to step in beside. The first
  steps narrow over 0.6 s: over 2.5 s the third lifted with the capture point
  8 cm inside the standing foot. Twelve trials: held 73 % (the rises, the
  walks at 0.85 and 0.9, the shoves along the line); the side shoves still
  felled it (the side step ends with the capture point 27 cm ahead and no foot
  there); the walk at 0.65 from its own start pose drifts 7 cm across in its
  first second.
- No help: a toe-off gate on the capture point (the legs driven 3.5 cm across
  moved the pelvis 3 mm in 0.2 s; walking on, the frozen plan sank it 17 cm);
  the pelvis driven by the capture point's error (the foot slid 20 cm) or by
  the course's deviation alone (down in 1.4 s). The plan's sway servo stays
  (2026-09-26).
- The director's slip re-anchoring at 4 cm (at 2 cm the feet's slides under
  the ankle's drive at 0.65 re-anchored eight times in 2 s and it fell); the
  side step's out foot set where the pelvis will be, kept there: twelve trials
  held 87 %, the walk at 0.65 and the shove toward the swinging foot at 0.85
  holding too. Still felled: the shove toward the standing foot (the side step
  ends with the sagittal capture point 27 cm ahead and no foot there) and the
  shove at 0.9 as the swinging foot lands. Hurrying the phase on a catch
  latched the landing early and 9 cm short; a swinging foot held as landed
  once it bore mid-swing reset the phase at every touch: both out again
  (2026-09-27).
- The side step's foot-to-foot strikes (2026-09-27):

| Fault | Fix |
| --- | --- |
| the stepping foot's toes struck the put-down foot's heel going by (5 cm between 10 cm wide feet, 1800 N) | that foot put down 12 cm across at least, the other out before on |
| pitched from its swing, a put-down foot's toes took 1000 N 9 cm up | borne means the ankle within 1 cm of the floor |
| a height still lowered for a catch shortened the trailing leg 3 cm as the step out began | that leg takes the height from the body |
| a latched swap was done 0.2 s later on the wrong side | a swap asked is a fresh judgement each pass |
| a walk begun again blending over 0.3 s: the trailing foot stayed down and its load share drove the capture point past the standing foot | no swap in its first 0.15 s; the blend over 0.1 s |
| a catch out of reach | a stomp: the foot down at once where the capture point will be 0.1 s on |

  Still 87 %: the left shove at 0.85 zigzags into a second and third side
  step; slewing the attitude's correction at 0.5 rad/s let the roll grow
  through the step.

- A search (CMA-ES, 144 candidates) over the torso's counter, the crane
  damping and the capture law's margin and gain: cost 8.2 -> 5.9 (stir 4.4 ->
  1.8 mm), but re-scored with the damping 0.556 -> 0.55 the walk at 0.9 fell
  in 4 s, and a 0.1 % change of any knob flipped a shove: landed at 5 s, the
  shove met whatever stride phase the pace had reached (0.25 at 0.85
  strides/s, 0.48 at 0.65 and 0.9, 0.07 less with the counter). The shove now
  lands as the phase first crosses 0.30 after 5 s. There the counter alone,
  3.9 deg, takes the stir to 2.4 mm, held 86 % against 84 (3.5-4.5 alike; one
  corner of margin and gain felled a walk); margin 43 mm and gain 1.1-1.27 add
  0.1 mm and noise; the damping stays off. At 0.30 the shove toward the
  swinging foot fells it as it is: the landing latched at 0.7 of the swing
  with the capture point 9 cm out, which ran to 23, and it toppled over the
  foot (2026-09-27).

## Floor events

`physics.World.terrain` in the shoves' place on the scoreboard, a shove
hardly ever happening to a walker (2026-09-27):

| Event | Setup | Result |
| --- | --- | --- |
| hole | 3 cm deep, 40 cm long under the left foot's next landing (the floor a slab over a plane that deep, cut in two) | fells it at 6.6 s: the foot finds no floor where the plan lands it, the leg holds it 3 cm short, the body falls 14 mm and runs on, a catch lifts the wrong foot |
| sill | 4 cm high, 15 cm ahead of its toes as it lifts | the toes skim at 3 cm through the first 0.16 s of a swing: a 2 cm sill shoved at with 200 N and gone over, at 4 with 350 N, the pelvis tipping 8 deg; the swing carries them over |
| patch | friction 0.06 under the landing | at 0.15 the stance foot crept 3 mm (the walk asks 0.17 of the floor); at 0.06 it slides 10 cm back under the push-off, the walk goes on tipped 6 |
| rug | loose, 0.1 on the floor, the sole's own grip on it, its front edge 15 cm short of the landing | at 0.3 it lay still (the sole's shear 100 N, its hold 165); at 0.1 it goes with the push-off, a fall at 7.7 s |

- A geom moved or grown past its compiled bounds is missed by MuJoCo's
  broadphase (the rug fell through a slab grown 27 m, a box through a sill
  moved 1 m): the slabs are compiled over their whole span and cut, the sill
  and the patch are mocap bodies. Held 93 %.
- A drive's glitch (`physics.World.glitch`, the peak torque cut to a share for
  a while) on the scoreboard (2026-09-27):
  + The left knee's gate dropped for 0.15 s at mid-stance ('cut'): 19 deg
    under 500 N, the pelvis 16 mm, then weight taken again, tipped 4.
  + The knee derated to a tenth for 2 s ('hot', 25 N m): folds 18 -> 64 deg in
    0.3 s, the pelvis sinks 20 cm, past the height latch's 12; the other leg,
    reaching from a target the body is no longer at, holds its foot in the air
    and the body falls backwards at 5.7 s. Walked on straight (the pelvis at
    the weak leg's reach, its step 8 cm short) it fell sooner: a knee already
    folded cannot straighten under the load, and the raised target hung the
    other foot; the cut knee, straightened the same way, fell too.
  + The hip held to a quarter for a second changed nothing (a stance hip asks
    under 60 N m).
  + The swinging foot lands on the capture point along the walk as across it
    (the body's speed over the plan's, over omega, 25 cm at most,
    `landing.FORE_K`): the walks' stir 3.70 -> 3.52 mm, 3 % further, the
    rises held. Slowed to a stop by the folding knee, the foot had come down
    where the plan had it, ahead of a body going nowhere.
- A 6 cm sill caught the skimming toes and it fell. A swinging foot bearing
  80 N is lifted 8 cm more until its heel clears it, the other too
  (`landing.over`): the page's sill, 13 places 3 cm apart, walked on at 4
  where 2; the stair, 3 steps where 4 (2026-09-28).
- A 5-step stair of 8 cm, blind, the floor read where each foot bears and the
  step expected again: walks up all five, pitches over at a landing a step
  lower (2026-09-28).
- The lace a snag (`World.lace`): 6 of 9 went taut, 5 felled it (2026-10-01).
- What trips it (`machine.events`, `tools/sim/look.py --event`, 2026-09-28):
  the 4 cm sill tips it 6 deg and it walks on; a lace pulling the lifting foot
  back 120 N for 0.15 s, 7; 250 N for 0.2 s and past, it falls, the
  director's fall declared 0.15 s after, before the foot is free, the standing
  foot off the floor as it curled. Curled into the squat, the arms out, the
  head met the floor at 0.35, 0.10 and 3.05 m/s after laces of 300, 400 and
  600 N, 1.71 in the hole, 0.32 on the rug. Onto all fours, the waist turned
  toward the way it tips as it turns: once in the five, at 0.5-0.69 m/s, the
  hands first but for the rug (a hip, falling ahead and aside, roll 0.6 -> 36
  deg).

## Shoves and the scoreboard

- P's 120 N felled it back to 279ba6f. Shoved 30, 60, 120 N 48 times: HEAD
  held 33, 10, 1; the parry 38, 21, 0 (`landing.PARRY_HURRY`, 2026-10-01).
- The scoreboard's events each laid at three places (`gait_montecarlo` SPREAD:
  3 cm along the walk, a glitch 0.05 of the stride), a trial its spread's
  mean, 30 runs a candidate: the committed arms and the same moved 2 % cost
  7.63 and 7.10 where one run a trial gave 6.08 and 8.84; the arms from the
  forearm 8.61; the slip at 0.9 strides/s held 55 % (2026-09-28).
- P's shove to the left at phase 0.14 on the merged feet (the box sole, the
  ankles on frame A): the catch overshoots to the right, the pelvis sinks 0.89
  -> 0.62 m over 0.4 s before FALLING_M calls the fall, fallen 0.12 s on, the
  hands down 0.27 s in with the arms 65 % into their catch, the head at 1.74
  m/s (test_gynoid_falls' crouch, 1 of 16; green before the merge; the toes
  driven in this build). Called 0.14 s earlier on a sink past 0.6 m/s: two
  falls struck, 1.68 and 3.41; the curl over 0.25 s: 3.22 (2026-10-04).
- The scoreboard at the day's end (`--suite all`, 28 trials, 2026-10-04): 729
  and 65.7 %. Rises 76, 100, 99; walks 71, 100, 31, 12 at 0.65-1.0; events:
  hole 77, sill 100, slip 76, rug 29, the sill at 0.65 19, the slip at 0.9
  18, SOA 77, hot 100, lace 26, nudge 54; shove 27; standing nudges and the
  stiff board 100, shoves 49, bricks 49 and 53, the rocker 62 and 85. The fast
  walks and the catches are the gap.
- The catch's gain and hurry on the faults suite (2026-10-04, 33 runs a
  cell):

| Gain, hurry | Cost | Held % |
| --- | --- | --- |
| 1.108, 0.25 (built, kept) | 178 | 54.9 |
| 1.0, 0.15 | 375 (the shove's fall landing on a gearbox past its rating) | 64.8 |
| 1.0, 0.25 | 442 | 64.6 |
| 1.108, 0.15 | 311 | 60.3 |

  A blind 6-knob search at 8 x 12 overbooked the relay and was stopped
  unlogged.

- The grinder's first round (`tools/sim/gym.py`, gemma4:12b proposing,
  2026-10-04): its own pick, capture.MARGIN 0.03-0.06 and GAIN 1.1-1.3 on the
  faults suite, 1 x 10 in 698 s; best 296.9 and 53.2 % at 0.0319 and 1.10,
  the built 178 and 54.9 staying. Rounds 2 and 3: landing.PARRY_M 0.02 and
  PARRY_HURRY 0.234 on faults 317.3 and 58.5 %; stand.GAIN and STEP_MAX_M on
  faults 178.2 and 54.9, a no-op (standing's knobs reach no walk). Its second
  run, Clef picking the suite: stand.DWELL_S 0.15-0.6 and HANG_S 0.05-0.3 on
  the stand suite, 1 x 10 in 273 s, best 178.2 and 76.0 % at 0.368 and 0.15
  against the built 215 and 74.7 (0.4 and 0.1); one round, not baked.
- The day's build (2026-10-04: the toes sprung at `TOE_K` 10, the heel off at
  0.5 and 40 deg, the one law standing): scoreboard 641.7 and 78.6 % over 28
  trials against the morning's 729 and 65.7 on driven toes (sill and slips
  100 %, hot 93, rug and nudge 77, soa 54, hole 33, lace 26, the stand suite
  77.5, the walks 100, 100, 90 and 80). Felled at 10.5 s by P's shove it walks
  again at 33.2 s. The tests' single runs flip with the push-off: at 30 deg
  the falls suite 44 of 44 and the faults' walk-on down; at 40 the faults 9
  of 9 and the head on the floor twice, 4.31 m/s at most.
- The grinder on the day's build (`gym.py`, its knobs by the schema,
  2026-10-04): the stand suite 1 x 8 over `bearing.PRESS_M`, `SEEK_M_S`,
  `stand.HANG_S`, `HANG_SHARE`; best 146.8 and 80.9 % at 0.0078, 0.294, 0.130
  and 0.453, the built values' own neighbourhood (0.01, 0.3, 0.15, 0.45;
  built 77.5 %), a best of eight.
- The arms aimed as it tips (`falls.aimed`, 2026-10-04). P's shove to the left
  at phase 0.14 had put the head on the floor at 4.31 m/s: the catch folded
  the knees, the fall was called at 0.65 m of pelvis with the trunk 3 deg off
  plumb, the way read behind it, the arms went back (the shoulders to -46
  deg); called down 0.07 s on, it pitched onto its face 0.37 s later. Aimed
  anew every pass past 12 deg of tip until an arm lands: of the 16 crouch
  falls the head never on the floor in 14, at 0.71 and 0.87 m/s in two; the
  peaks' medians 4.1 and 3.7 kN.

## Starts and the catch

- The catch against its own walk (2026-10-05, `capture.CATCH`). At 4 cm, from
  the squat at 0.85 strides/s it parried a plain floor in 2 runs of 6 side
  gains 0.1 % apart, 16 and 10 times; at 1.0 in 5 of 6. The start's last
  strides run 44-74 mm off the course, and early in a fast swing the leg's own
  swing-out (the hip's roll 5-6 deg past its setpoint, the ankle 6-7 cm out)
  jerks the pelvis 12 mm the other way: 34 mm of landing asked in a clean
  step, 60 in a parried one. At 7 cm none of 12 from the squat; the events held
  48 of 90 where 47 (hole 1 and 0, sill 6 and 7, rug 1 and 0, slip 15 and 16,
  soa 8 and 7, hot 9, lace 0, nudge 8); the five gynoid suites as before.
- Begun standing (2026-10-05). The director starts every walk at 0.85
  strides/s and glides to its pace; the scoreboard set the walker's pace over
  that and 2 walks of 12 fell, 52 parried (the head before the strut: none,
  222, 67-82 a walk at 0.65).
  + At 1.02 it ran to 1.0-1.1 m/s where its plan has 0.85, the catch on its
    three steps running every 2.6 s, down in its third round, 4 runs of 6.
  + At 0.88 its fifth step landed late on a knee at 23-26 deg, asked straight
    in 40 ms under 752 N (17 N on both feet, 3 cm up, 0.56 -> 0.27 m/s), a
    fall 2 s on.
  + One knob at a time under it: the pre-swing at 26 and 28, 1 fell; the
    let-go's easing 0.12 1, 0.15 none, 0.18 3, 0.22 7: no order in it.
  + As the director starts it all 12 hold their form. The standing start
    stays on an edge: from its second step the capture point runs 51-65 mm
    off the course, the weight not over the standing foot as the other lifts,
    its fourth lands 130 mm wide (119 from the squat); of 6 side gains 0.3 %
    apart 1 fell and 2 parried 3-19 times; over ramps of 2.5, 3.5, 4.5 and 6 s
    (`stance.RAMP_S`) by 3 gains 2 fell, 3.5 and 6 clean.
- The speed is its own (2026-10-05). The phase follows the body
  (`stance.advance`) and nothing holds it to the asked pace: at 1.0 strides/s
  0.96 m/s where the plan has 0.83, 581 W. A foot landed further ahead of a
  body going fast (`landing.FORE_K`) is no brake, it lengthens the stride:
  read against the asked pace instead of the phase's rate, 0.65 -> 0.68 m/s at
  0.85 strides/s, at a gain of 1.5 0.83 and 639 J/m. The foot under the hip
  later in its stance (`gait.STANCE_AT` 0.265): 0.78 m/s and 349 W at 1.0.
- The roll's peak against the starts (2026-10-05, 6 side gains 0.3 % apart;
  `gait._roll`'s peak, of the stride):

| Peak | From the squat | Standing |
| --- | --- | --- |
| 0.33 (as built) | none of 12 parries | 1 of 6 falls, 2 parry |
| 0.29 | 4 of 12 parry, 13-26 times | 3 of 6, one 85 times |
| 0.27 | 10 of 12, 7-47 times | none of 18 |

  Steady, it is on its form at all three. The course learned as its own
  walk's (each leg's, a swing that was no catch taken into it, 0.4 of it)
  changes none of it and is not taken: at 0.65 strides/s 437 J/m where 383,
  the toes back 4.2 mm where 0.6, the table's 15-38 mm off there landing the
  feet wider.

- The first step is a fall toward the swinging foot (2026-10-05, begun
  standing, the feet 161 mm apart). As the right foot lifts, the CoM stands 39
  mm inside the left ball (the keyframe's 27, `arrival.LIFT_IN`; the sole's
  edge at 45) and the capture point 35 mm inside and going on, past the edge
  0.1 s later; through the lean the weight has come back, 140 N a foot where
  230 and 50 after the shift. The foot lands 126 mm from the standing one where
  the walk's track has 60; the feet come in step by step on the capture law's
  asking, the walk's plan narrow from the first (`Walker.begin`, no `wide`
  since 2026-09-27). Shifted further by the keyframes (`SHIFT_IN` 0.03 where
  0.043, `LIFT_IN` as built and 0.015): down in every start of 30, standing
  and from the squat; over 0.9 s (`SHIFT_S`) the standing ones hold, 0-10
  parries, from the squat still down: a shift on a clock overshoots the sole,
  as on 2026-09-26.
- The lean before the shift (2026-10-05, `arrival.keyframes`): leant on both
  feet, then onto the left, the foot lifted as the weight comes over it, the
  capture point 24-28 mm inside the standing ball at the lift where 35. Of 30
  starts, standing and from the squat, 6 side gains 0.3 % apart: 29 with no
  parry and 1 down (begun standing, glided to 1.02 strides/s) where 21, 3 down
  and 6 parried. The events held 48 of 90 as before; shoved down at 1.0
  strides/s it is up and walking 22.5 s on. With the roll's peak at 0.27 the
  standing starts all clean, from the squat 1 gain of 6 parries, 27-33 times;
  at 0.28-0.30 1 or 2 gains of 6, one down; at 0.31 none. Stood 0.4 s before
  the lean where 0.05 (`STAND_S`): 27 of 30, one gain's standing start parried
  14-22 times; with the peak at 0.29 and 0.27 from the squat 4 gains of 6
  parry.

## Shoves on the one law

- A shove by the stride (2026-10-06; `ways.py --polar`): walking, a shove at
  each of 8 moments of a stride from each of 8 ways, read by the foot that
  stood as it began (what follows a shove is the standing leg's; a trip the
  swinging foot's).
  + 38 N, one foot down: from behind 6 of 6 up, from ahead 6 of 6; over the
    standing foot 2 of 6; to the free side 1 of 6. Traced, 0.17 s into a
    stance: the free foot is aimed across at the capture point as it is, 110
    mm from the standing foot, not as it will be when the foot is down; that
    point runs 38 -> 125 mm in 0.14 s, the aim after it to 169, the foot in to
    134 and out again, down 3 cm short; the steps after are hurried, 0.24 s
    each, a sole struck 740-800 N, the capture point 0.54-0.73 m ahead of the
    standing foot, until a foot is not there.
  + By the moment: the standing foot 0.44 s down, the free one coming down: 15
    of 16 up; 0.16-0.30 s down, the free one just gone: 16 of 32.
  + 70 N: from ahead 8 of 8, from behind 5 of 8, any way with a side to it 4
    of 48. 120 N: none, standing, walking or running.
  + A shove of 120 N for 0.12 s moves the capture point 14 cm, and it runs on
    as e^(3.3 t): seen a tenth of a second later, a step 39 % longer.
  + Way on: detectors at an impulse the IMU has (a shove, a slip, a strike);
    a table under the law's own pace keyed by the shove's way from the
    standing foot, that foot's seconds down, and one foot or both; its answer
    the law's own handles (which foot leaves now, where it lands, how long),
    so the hand-back is the law's next step.

### A free foot not hurried

`going._due` (2026-10-06). The capture point running across, the foot was due
as that point reached where a step's swing takes it: hurried, it came down
short (above). It is due no sooner than what is left of its swing at the
swing's own pace, the row's `step` less the seconds both feet bear: 0.39 s
walking.

| Least whole swing s | 38 N up of 64 | 70 N up of 64 |
| --- | --- | --- |
| 0.30 | 44 | 17 |
| 0.35 | 59 | 28 |
| 0.40 | 64 | 37 |
| 0.45 | 63 | 40 |
| capture point hurrying no foot | 54 | 23 |

- Shoved walking with it, up of 64: 38 N 62 where 44 (with a side to it 46
  of 48 where 28); 70 N 36 where 17 (to the free side 4 of 6, over the
  standing foot 1 of 6); 120 N 5 where none.
- Tried and out: the foot aimed where that point is heading, its rate
  meaned over 30 ms times the seconds to go: half of it 42 of 64 at 38 N,
  all of it the plain walk down.
- Ways 77 of 104 where 69: nudged walking 8 of 8 where 4, turned back 6 of 6
  where 4, nudged running 6 of 8 where 4; the walk's draw and band as they
  were, 501 J/m and 0.20; the page's presses under the director 66 of 72
  either way.
- A recovery is two steps or more of staggering; it must be seen at once
  and a leg put out, which this is, the leg out in full.
- Left, over the standing foot from 70 N, traced: the first step down as
  ever; the second a side step of 62 cm that catches the body, the soles
  bearing 20-150 N for 60 ms after it, the capture point 7 cm outside that
  foot and held there 0.5 s; the other foot in beside it and both down 0.74
  s as that point runs on: a foot leaves only with it over the one that
  stays. Released at once with it past the body, the foot steps out 73 cm
  more and the body goes on sideways: 36 of 64 as before, out. Each
  recovery step a leap; smaller and sooner, or across the standing leg, is
  the reflex's to find (docs/TODO.md).

### Into a steady wind

On the one law (2026-10-06): a force on the trunk against its way, up over 2
s, held 16 s.

| Wind | Plain | Accent `into the wind` |
| --- | --- | --- |
| 10 N | slows 0.85 -> 0.70 m/s, walks on | 0.81 m/s |
| 20 N | down in 3 s | backward at 0.11 m/s, down in 7 |
| 30 N | down in 3-4 s | - |

- A 70 N shove for 0.12 s from ahead it stands 8 of 8: it has nothing for
  the steady force. The standing legs lean it 2.5 cm at most
  (`hold.LEAN_M`), 8.6 N of it, and a walk's steps are not laid back under
  it for a speed it lacks.
- Tried and out: the feet's pattern taken 3 cm back with the accent (0.89
  m/s at 10 N, down at 20); 6 cm (down as it came in); that taken back by
  the speed it is short of, summed a landing (the first steps of a start
  wind it up, down at 4.7 s).
- Traced at 20 N: it lands 0.16-0.19 m behind a foot at 0.36-0.42 m/s and
  does not pass over it; stalled 95 mm behind it, back at 0.5 m/s and down.
  A step is laid for the last step's pace and 0.36 m/s more, which a walk
  gains in still air and not against 0.34 m/s a step of wind; laid for what
  it gained on its last step too, the same: shorter steps, still nothing
  that pushes. The feet's pattern taken back by the wind's worth as it comes
  on, 58 mm: down as soon.
- The accent is the posture; the push against a steady force (a slope's
  too) is the law's to find.
