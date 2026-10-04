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
- P's shove to her left at phase 0.14 on the merged feet (the box sole, the
  ankles on frame A): the catch overshoots to her right, the pelvis sinks
  0.89 -> 0.62 m over 0.4 s before FALLING_M calls the fall, fallen 0.12 s
  on, the hands down 0.27 s in with the arms 65 % into their catch, the
  head at 1.74 m/s (test_gynoid_falls' crouch, 1 of 16; green before the
  merge). Not the toes: driven, the same numbers. Called 0.14 s earlier on
  a sink past 0.6 m/s, two falls struck, 1.68 and 3.41; the curl over 0.25
  s, 3.22 (2026-10-04).
