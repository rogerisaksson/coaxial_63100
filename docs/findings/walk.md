# Findings: her walk

The gynoid walking: her plan, its shape and her look, the start from the
squat. The board's own are in [FINDINGS](../FINDINGS.md).

- The gynoid's walk (`machine.gait`) at 0.85 strides/s: every joint's jerk
  under 4.2 times its rms, the knee's peak 375 deg/s, the head 3.5 mm up and
  down, 5 mm sideways, the pelvis 8 degrees each way. Past 1.0 strides/s the
  legs reach full length early in the swing and the knee snaps (2026-09-25).
- Toe-off with the heel's rise eased to a stop there: the foot stood still,
  the knee went -180 -> +409 deg/s. Rising through it (420 deg/stride) into a
  septic swing: the foot never under 0.93 m/s, the knee flexing throughout,
  the toes 0-3 mm over the floor early in the swing, 23 at mid (2026-09-25).
- Walking free in 3D: a swing leg reached from the pelvis's target put the
  foot 6 cm across the line; reached from the measured pelvis, a stance foot
  held where it landed and taking its weight from 0 over 0.04 of a stride,
  she walks 60 s at 0.6-1.0 strides/s. The sideways speed raw in the
  feedback, 500 Hz, or a sole turning at 0.03 m of torsion: she fell in 2 s
  (2026-09-25).
- Her torso on the pelvis pitched 8 degrees a stride, the head 9 cm fore and
  aft; the spine taking the pelvis's pitch back out: 2.4 and 3 cm. The
  walker, loop and world 0.63 ms a pass: x1.6 real time in her own process
  (2026-09-25).
- Her stance knee at its straightest stood at 27-31 degrees: the pelvis held
  level where the gait drops it on the swing side put the stance hip 2 cm
  low. The pelvis dropped as planned (5 degrees), the spine's roll set to
  take it back out (fed back from the measured roll she fell): 5.5, 9.0,
  12.7 degrees at 0.6, 0.75, 0.9 strides/s, 30 s each on her feet
  (2026-09-25).
- Landed on the ball, heel up, the landing knee stood at 42 degrees and the
  step struck 3.3 body weights: the stumble. On the heel, toes up 9 degrees,
  rolled flat by 0.13 of a stride: 29 -> 10 degrees in the plan (2026-09-26).
- Its sideways gains (STEP_D 0.01) held the walk and lost every start from
  standing: 0.057 back. Started as the right foot lands, on her standing
  stance gliding into the catwalk, she walks on at 0.85 strides/s; at 0.7
  and 0.9 she falls within 2.5 s. Stopping: a stride shortened at the pace
  kept threw her off the floor; carried in phase, slowed, she settles onto
  the front foot and runs on over its ball (2026-09-26).
- The virtual pendulum between her ears (`machine.pendulum`: 1.13 m, her
  weight, a spring of no length of its own): stir 3.2, 4.3, 4.9 mm at 0.65,
  0.85, 0.9 strides/s, 78 % of it on - the head's surge - peaking at each
  landing (2026-09-26).
- 26 knobs +-20 %: all but four within 0.25 mm of 4.2. The torso countering
  the surge twice a stride, 2.5 degrees at 0.125: 2.2 mm; with the head moved
  toward the bob (`SWAY_K` 1): 1.7; by its drift too, down in 2 s. The rise
  flips on 0.5 % of any knob, the committed gait's as much (10 of 24 held):
  one run a candidate scores chance (2026-09-26).
- The pendulum between her ears, read by stride phase at 0.85 strides/s
  (fore, across and up alike, 0.8-0.9 mm each): nearly all of it goes in
  at touchdown. The pelvis fell into it at 0.15 m/s, 11 mm under the plan,
  the landing leg came down bent 23 degrees (the IK's own setpoint) and
  straightened to 7 within 0.16 of a stride, lifting her 26 mm at 0.29 m/s
  to a dead stop at the top: 6 m/s2 up then down at her ears, 3.5 aft as
  the front foot braked her, 4 across, the sole's peak 1.4 kN. A stance
  knee 16 degrees soft at its straightest leaves the leg room: 2.4 -> 2.0
  mm, and the plan's dip eroded over 0.05 of a stride (17 -> 20 mm, nearer
  the body's) 1.7; the peaks 3.7 and 2.5 m/s2. Left: 12 mm of sag at
  touchdown and a sole peak of 1.9 kN - the soles are rigid (2026-09-27).
- The soles given a little, as light sneakers (`machine.physics` SOLE_*):
  the contact's damping 1.5 of critical and its impedance easing in over 5
  mm take a touchdown's peak 1.9 -> 1.5 kN and the stir 1.7 -> 1.4 mm, a
  shove's parry 1.5 -> 1.1 kN. Every softer sole felled the first stride
  from standing, the walks and the shoves unharmed: settling over 0.035 s
  instead of 0.02 the body pitched on twice as fast in the first single
  support (the sole is a lag in the ankle's hold) and the foot landed 0.24
  s early; damped 1.75 or more the stance foot's load flickered to 70 N
  under the rolling foot (a damped contact kicks at its corners' speed) and
  she toppled sideways. MuJoCo's soft contact is a poor foam: its
  softness is a time constant on the whole body's mass, its damping acts on
  every corner's motion (2026-09-27).
- The walk begun from a lean (`machine.arrival`): shifted onto the left
  foot, her weight goes 7 cm ahead of the ankles over 0.6 s on both feet,
  then 5 cm further as the right foot lifts 6 cm over 0.3 s, and the walker
  takes her mid-swing at the phase where the plan has the pelvis where it
  stands over the left ball (`Walker.begin`, as after a side step), the
  foot landing on the walk's own track 4 cm from the standing one. Set down
  first in the walk's landing pose, still, the pelvis tipped back 2 degrees
  as the foot lifted and 5 forward as the walk took her. Handed on still,
  she hung back behind the landed foot and tipped over backwards (the phase
  pulled to a body standing still stands with it); landed on her standing
  stance, 16 cm out, the pelvis could not get over the foot and the next
  went 20 cm out to catch her; leant 10 cm in 0.5 s she was thrown off the
  left foot. The scoreboard as before, the rises held (2026-09-27).
- The look of the walk at 0.85 strides/s, by the trace: the stance knee
  16-29 degrees (the plan's own 16-24: the pelvis rides 10 mm under the
  plan, latched there at each landing as the trailing leg sags 9 mm into
  it, its ankle drooping 2 degrees under the push-off's torque, and raised
  at 0.1 m/s - a rush of 0.22 m/s to a dead stop); the ears 19 mm up and
  down a stride and 33 fore-aft, the torso nodding 7.6 degrees (the
  counter), 66 fore-aft with the torso still: the pelvis's own surge, the
  collision at every touchdown (-0.25 m/s over 0.13 s, +0.2 back before the
  next). The stance knee 8 degrees soft, the heel strike 15 degrees toes
  up (its peak 1.55 -> 1.1 kN), the recovery 0.03 m/s: the knee 9-29, the
  stir 1.4 -> 1.2 mm, held 86 %. The counter off for the eye: 3.5 mm.
  Straighter legs at the strike need the trailing heel up before it (the
  plan's height there is that leg's, flat, 22 cm behind): a heel rising
  in single support ran her ahead of the plan and she sank 35 mm before a
  landing 0.1 stride late (the phase, pulled to the body at 5 a stride,
  lags it); the heel to 50-55 degrees at toe-off felled the walks alike
  (2026-09-27).
- The calf: the heel to 50 degrees at toe-off (`gait.TOE_DEG`, rate -300
  a stride, turning back at 4000 a stride squared), the rise scaled by the
  stride. With the foot landed on the capture point along the walk it
  holds: the walks' stir 3.35 -> 2.80 mm, the ears' up-and-down 18 -> 12 mm
  a stride, the plan's landing knee 29 -> 14 degrees, the hot knee walked
  out (tipped 12), held 90 %. Unscaled, the first short strides' push-off
  hopped her off the front foot (the rises fell); 55 and 60 degrees, an
  earlier heel-off and a slower rise all lose trials. After toe-off the
  toes' tips skim the floor (the sole's peak 1.4-1.9 kN there): curled up
  20 degrees they touched it still, the toe drive too slow (2026-09-27).
- The head's fore-and-aft, 61 mm a stride at the ears: the pelvis surges
  30 (its speed 0.76-0.99 m/s over a 1 m stride, the vault's exchange plus
  the stance leg braking it 2 m/s2 through half the stance and pushing 3-5
  before the landing) and the torso's pitch, 2.2 degrees the spine's
  counter does not take out, doubles it up there. Tried and out: the phase
  run as a pendulum (58-65), a lead on the pitch's rate for the spine (fell
  in 4 s at 0.05-0.15 s either sign), a shorter stride (0.9: the pelvis 38
  at 0.82 m/s), a planned landing dip (the body sags under any plan, the
  knees bent the same and the bob 19-24), the height latch let go of its
  first 15 mm (the first landing hopped), the bob's swing on the landing
  (`PEND_K` 1.5: fell in a second). The stance knee, 6 degrees soft: 4-7 at
  its straightest, 15-20 through mid-stance from the latch's 10-12 mm; at
  4 the walk at 0.9 fell in 0.6 s (2026-09-27).
- The lean before the first step is the body's, not the pelvis's: pushed
  7 cm ahead with the torso plumb (`PLUMB` 1.0) the pelvis went out under a
  vertical trunk and the trunk pitched back half a degree at the push - she
  read as leaning back before she stepped. Now the lean's frame tips the
  pelvis LEAN_DEG 4 forward and the torso with it, the neck keeping the
  head level; the walker takes the lean over at the hand-off and lets it
  out over LEAN_OUT_S 2 s. A lean kept through the walk, 2, 3 or 4 degrees,
  had her fall to the slip and the hot knee and the walk at 0.9: held 76,
  76 and 66 % against 90; let out, held 89.8 %. A stiffer spine steadies
  the torso, not the head: kp 800 -> 1600 -> 2400 N m/rad took the torso's
  pitch 2.1 -> 1.2 -> 0.8 degrees a stride and the ears' fore-and-aft
  58 -> 68 -> 63 mm, the pelvis's surge 24 -> 39 -> 33 - the spine's give
  is the filter between the hips' pulses and the head (2026-09-27).
- The feet 27 -> 24.5 cm: held 81.8 -> 85.3 %, the strike 1.1 -> 1.4-1.6
  kN (2026-09-28).
- After 100 s she sank 172 mm into a crouch: at 80 m she stepped off the
  floor's slab, each landing latching the height lower. The slab runs 10
  km (2026-09-28).
- Her limbs collide (`physics.ME`): floor alone, her sneakers passed 22 mm
  into each other; a swinging foot is kept off the other (`landing.clear`)
  (2026-09-28).
- The swinging hip ran 5 degrees behind its setpoint and caught up into the
  floor, 1.5 kN in 2 ms: the swinging leg's plan leads 20 ms
  (landing.SWING_LEAD_S) (2026-09-28).
- The feet rolled 2 or 3 degrees onto their outer edges took the ankle's
  roll to 3.2 and 2.0 and she caught herself from 10 s (2026-09-28).
- The swing skims (gait.LIFT_M) 25 mm over the floor, 42 at most, where a
  bump lifted the foot 130 mm and she trod the air; levelled at 0.6 of the
  swing the knee bends once, to 55. The heel up 10 degrees as the other
  lands (gait.RISE_DEG): the knee lands at 13 where 33, the head's bob 13
  mm where 23; the look suite's impact 1046-1599 N where 228-1358
  (2026-09-28).
- Turned 9 degrees, dropped 6, the spine taking 0.3 of the hips' offset
  back: hips 72 mm across, shoulders 25 (2026-09-28).
- Toed out with a small swing turn she fell (5/4, 6/6, 8/8 degrees): the
  swing aimed its ankle, the landings recorded its ball, 12 mm wider at 6.
  Aimed at the ball, 8 out and 6 in: 7.4 in stance, 2.5 out swinging. The
  ankle rolls 5.5-5.9 through the loading, the foot flat: tracks at 40-50
  mm take it to 4.8-3.6 but she cannot balance on them (2026-09-28).
- The start as the eye has it: taking her from the lean, the walker's plan
  stood the pelvis up, 7 degrees back in 0.2 s, the stance heel rising - on
  her toes and leaning back; the plan tips it as the lean now, LEAN_DEG 4 ->
  8 to read as one. Risen 1 cm under the stand, she stood on 20-degree knees
  with the hips behind them: risen to the stand's soft knee (4) and sunk 4
  mm as her weight goes left (`arrival.SINK_M`; unsunk the stance knee
  locked at -3), 7-10 through the step. The weight's 6 cm onto the left
  foot rolls the stance hip -4.4 degrees (-7.7 as the right lifts): a lift
  from standing needs her centre of mass over the left sole, and from the
  page's 60 degrees the column's lean reads as leaning back. The head
  nodded 6.1 degrees a step on the neck's 60 N m/rad; 300 at half critical:
  2.2, the pendulum's stir 3.55 -> 3.00 mm, held 89.7 % (2026-09-28).
- Her walk made hers: the arms swung from the forearm, near the body
  (`gait.ARM`: the shoulder 12 degrees a side, the elbow 24 +- 11, the
  wrist 10 +- 7). The hips already swayed 35 mm to the shoulders' 23, the
  pelvis rolling 10 and turning 39 degrees a stride; a deeper hip drop
  (ROLL_DEG 6, 8) swayed the shoulders 28 and 33 mm and nodded the head
  2.9 and 3.6 degrees. The pelvis tipped 6 degrees under an upright torso
  read as leaning back from the rise on: the page's spine at -5.9, the
  torso bent back over the hips; out again. The scoreboard cannot judge a
  look: the committed arms moved 2 % (the shoulder 16.3, the elbow's swing
  9.2) held 80 and 75 %, the slips and the hot knee flipping; every arm
  tried held 84.7-85.5 %, the chosen ones 89.7 (2026-09-28).
- The weight onto the left foot before the first step, partly as the
  right lifts (`arrival.SHIFT_IN` 3.5 -> 5.5 cm inside the left ankle):
  standing, the stance hip rolls -2.5 degrees, not -4.4, the pelvis 33 mm
  across, not 61, her centre of mass 30 mm left at the lift, not 52; -7.8
  through the step as before. Ten and sixteen perturbed starts (timings,
  gains, the lean's and the lift's reach): 3.5 held 10 and 15, 5.5 and 6
  held 10 and 14, 6.5 held 6 of 10 and 7.5 one, the first steps falling at
  9-11 s (2026-09-28).
- Leaning back before the first step, by the page's recordings (R): the
  pelvis sunk for the weight's shift bent the knees 4 -> 12 degrees under a
  plumb torso - the hips 26 mm behind the knees, the hips-to-shoulders line
  -0.5 degrees, the torso 7.2 behind the shins; rising, the torso came up
  first, 18.6 behind them. Now the torso leans as far as the shins rising
  (`arrival.with_shins`, a keyframe at 80 % of the height), stands plumb,
  and leans on the lean's 4 from the shift on as the knees bend 4 -> 8, the
  sink 2 mm (1 and 1.5 mm fell): the hips-to-shoulders line -0.4 standing,
  +3.8 shifting, the torso at most 5.7 (8 from the shift on read as
  unnatural). The seam to the walk: the
  arrival rode the torso on the pelvis and the walker eased its spine in
  from the hand-off's, so the torso swung 10.8 -> 14.0 -> 7.5 degrees as
  the pelvis tipped; both now take the pelvis's tip back out of the spine
  (`walker.PLUMBED` not eased in): the torso 8.7-9.8 through it, the
  largest swing back in 0.3 s -6.6 -> -3.0. Held 89.7 %, 15 of 16
  perturbed starts (2026-09-28).
- The seam's odd lean, by the page's recording: the lean and the step's
  keyframes moved the pelvis 5-9 cm on at the shift's height, so the
  standing knee was planned straight and bent back 1.8 degrees under her
  for 0.35 s while she tipped on over it like a plank (the ankle-shoulders
  line +5 -> +9). Lowered till that knee bends as in the shift
  (`arrival.soft`, SOFT_KNEE 8): at least +6.9 through the seam, bending on
  7 -> 19 as she goes over it; 15 of 16 perturbed starts. Lowered in the
  lean too, the head dipped 5.1 mm there, not 3.3 - the step's alone now.
  The curtsy into the walk (tools/sim/look.py): the head 22 mm down in the
  step and 31 at the first landing whatever the sink (0.5-2 mm) and the
  lean (0-4 degrees; at 0 every start fell) - the walk rides 16-28 mm
  under the stand and sags 7-10 mm under its own target at landings
  (2026-09-28).
- Between the rise and the walk, moment by moment (the page and
  tools/sim/look.py number them: 5 stand, 6 shift, 7 lean, 8 step, 9
  walk; look.py prints each seam 0.3 s either side, the director's asks
  beside): the knees bent 4 -> 9 in 6 and the torso bowed 1 -> 5.7 in 7, a
  curtsy before the step; in 8 the pelvis's target lagged 13-46 mm behind
  her and bent the standing knee 5 -> 16. Now she rises to the shift's
  height (knees 9.4 standing), 6 and 7 still - the torso 1.0, the pelvis
  -0.2 mm, the weight 3 cm on - and in 8 the target not pulled back
  (`fall`), the lean's tilt and 9 cm more: the torso 0.9 -> 4.9 and the
  pelvis +4.2 then -5.8 mm with the lift, the walk's first second -21.5
  where it was -26 to -30. 15 of 16 perturbed starts, held 81.3 %
  (2026-09-28).
- The head in the walk (tools/sim/look.py's walk line, from 2 s in): 53 mm
  fore and aft a stride - the pelvis's surge 31-35 and the torso rocking
  3.3 degrees at 0.6 m - 13 up and down. The ears' pendulum at 0.1: 48.5
  mm, the strike 1613 -> 1115 N, held 87.9 % against 81.3. No better: the
  torso's counter to the surge at any phase (1.5, 3 degrees: 53-74 mm,
  some fell), the pendulum past 0.1 (0.15: 57), the legs' drives 1.5 times
  stiffer (44.9, the strike 1410 N; 2 and 3 fell), the phase pulled less
  (3: 57.6; 2 fell; 1: 46.3, 1628 N); the shoulders over the line (0.5:
  26.7 mm to the hips' 44.9) cost the walk at 0.65 (2026-09-28).
- Her stance leg (`tools/sim/look.py`: the ankle ahead of the hip at the
  landing, the toes behind it and the thigh's angle at the lift, a load held
  0.1 s): as she walked, the thigh left the floor 1.1 degrees ahead of
  upright, the toes 196 mm behind the hip - her feet in front. The toe-off
  at 0.66 and the ball planted at 0.24: 12.2 behind, 297 mm; the stride
  0.85 m: the pelvis's dip 49 -> 34 mm and the head's surge 56 -> 35 mm,
  the thigh 29 ahead at the landing and 6 behind at the lift, held 83.7 %,
  the stir 2.3 mm, the scoreboard's cheapest yet (7.18). Ahead stays ~20
  degrees more than behind whatever the knobs (the stride, the toe-off, the
  stance, the heel's rise, the knee's softness): the landing knee bends
  ~30, the hips held down by the trailing leg in the double support. She
  wears a tee, jeans and sneakers - drawn loose over her, the drums under
  the cloth patched on it in their heat's colour - and the cloth is felt
  in MuJoCo: denim (0.55) on the seat, the thighs and the knees, cotton
  (0.45) on the torso and the upper arms, 4 mm of give (2026-09-28).
- The pelvis rocks about the hips' mid-point, a beam engine's beam (the
  user): in single stance its still point -0.08 of the way to the stance
  hip, that hip 14.3 mm up and down a stance, the swing hip 17.2. Dropped
  HIP_HALF sin|roll| about the stance hip: +0.67, 10.3 and 17.3, but caught
  5 times in the walk's first 3 s from the squat where not at all, the head
  52 -> 90 mm fore and aft, and after A 9 times in 5 s where none. The
  parry's 16 shoves held 1 and 1 of 8, dropped all the way 2 and 4, half way
  3 and 2 - but half way she fell from the squat at 12.1 s. Undone until the
  walk is retuned with it (2026-10-02).
- The parry on her build as modules (test_gynoid_falls' 16 shoves of 38 N,
  held left and right of 8): `landing.PARRY_HURRY` 0.32 1 and 1, 0.28 2 and 2,
  0.25 3 and 3, 0.22 0 and 1, 0.4 1 and 1; split, a shove 0.25 and a catch
  0.32, 1 and 2; a catch 0.30, 2 and 1 - no order: its floor, HEAD's build's
  2 of 8 the test's. A knee derated to nothing walked on at 0.32 and fell at
  0.28 and 0.25. 0.32 kept, the parry to the walk's retune (2026-10-02).
- Retuned for her trunk's roll first and the gimbal's drives on their stages
  (`physics.STAGED`): 9 knobs - arrival's shift, lift and lean, capture's
  margin and gain, the walker's side gains, the parry - 5 generations of 8,
  the median 468 -> 335. The board 578 -> 299, held 76.8 % against main's
  495; the same knobs to 17, 6 and 3 digits 263, 306 and 420 - its chance.
  `landing.PARRY_HURRY` 0.29 (2026-10-03).
