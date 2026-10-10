# Findings: walk

The walk as built: its plan, shape and look, the start from the squat. The
board's findings: [FINDINGS](../FINDINGS.md).

## Plan and shape

- `machine.gait` at 0.85 strides/s: every joint's jerk under 4.2 times its
  rms, the knee's peak 375 deg/s, the head 3.5 mm up and down, 5 mm sideways,
  the pelvis 8 deg each way. Past 1.0 strides/s the legs reach full length
  early in the swing and the knee snaps (2026-09-25).
- Toe-off with the heel's rise eased to a stop there: the foot stood still,
  the knee went -180 -> +409 deg/s. Rising through it (420 deg/stride) into a
  smooth swing: the foot never under 0.93 m/s, the knee flexing throughout,
  the toes 0-3 mm over the floor early in the swing, 23 at mid (2026-09-25).
- Walking free in 3D: a swing leg reached from the pelvis's target put the
  foot 6 cm across the line. Reached from the measured pelvis, a stance foot
  held where it landed and taking its weight from 0 over 0.04 of a stride: 60
  s walked at 0.6-1.0 strides/s. The sideways speed raw in the feedback, 500
  Hz, or a sole turning at 0.03 m of torsion: a fall in 2 s (2026-09-25).
- The torso on the pelvis pitched 8 deg a stride, the head 9 cm fore and aft;
  the spine taking the pelvis's pitch back out: 2.4 and 3 cm. Walker, loop
  and world 0.63 ms a pass: x1.6 real time in its own process (2026-09-25).
- The stance knee at its straightest stood at 27-31 deg: the pelvis held level
  where the gait drops it on the swing side put the stance hip 2 cm low. The
  pelvis dropped as planned (5 deg), the spine's roll set to take it back out
  (fed back from the measured roll it fell): 5.5, 9.0, 12.7 deg at 0.6, 0.75,
  0.9 strides/s, 30 s each standing (2026-09-25).
- Landed on the ball, heel up: the landing knee at 42 deg, the step struck 3.3
  body weights, the stumble. On the heel, toes up 9 deg, rolled flat by 0.13
  of a stride: 29 -> 10 deg in the plan (2026-09-26).
- Its sideways gains (STEP_D 0.01) held the walk and lost every start from
  standing: 0.057 back. Started as the right foot lands, from the standing
  stance gliding into the catwalk: walks on at 0.85 strides/s; at 0.7 and 0.9
  falls within 2.5 s. Stopping: a stride shortened at the pace kept threw the
  body off the floor; carried in phase and slowed, it settles onto the front
  foot and runs on over its ball (2026-09-26).
- The virtual pendulum between the ears (`machine.pendulum`: 1.13 m, the
  body's weight, a spring of no length of its own): stir 3.2, 4.3, 4.9 mm at
  0.65, 0.85, 0.9 strides/s, 78 % of it on (the head's surge), peaking at each
  landing (2026-09-26).
- 26 knobs +-20 %: all but four within 0.25 mm of 4.2. The torso countering
  the surge twice a stride, 2.5 deg at 0.125: 2.2 mm; with the head moved
  toward the bob (`SWAY_K` 1): 1.7; by its drift too: down in 2 s. The rise
  flips on 0.5 % of any knob, the committed gait's as much (10 of 24 held):
  one run a candidate scores chance (2026-09-26).
- The pendulum read by stride phase at 0.85 strides/s (fore, across and up
  alike, 0.8-0.9 mm each): nearly all of it goes in at touchdown. The pelvis
  fell into it at 0.15 m/s, 11 mm under the plan; the landing leg came down
  bent 23 deg (the IK's own setpoint) and straightened to 7 within 0.16 of a
  stride, lifting the body 26 mm at 0.29 m/s to a dead stop at the top: 6
  m/s^2 up then down at the ears, 3.5 aft as the front foot braked, 4 across,
  the sole's peak 1.4 kN. A stance knee 16 deg soft at its straightest leaves
  the leg room: 2.4 -> 2.0 mm; the plan's dip eroded over 0.05 of a stride (17
  -> 20 mm, nearer the body's): 1.7; the peaks 3.7 and 2.5 m/s^2. Left: 12 mm
  of sag at touchdown and a sole peak of 1.9 kN; the soles are rigid
  (2026-09-27).

## Soles

Given a little, as light sneakers (`machine.physics` SOLE_*, 2026-09-27): the
contact's damping 1.5 of critical and its impedance easing in over 5 mm take
a touchdown's peak 1.9 -> 1.5 kN, the stir 1.7 -> 1.4 mm, a shove's parry 1.5
-> 1.1 kN. Every softer sole felled the first stride from standing, the walks
and the shoves unharmed: settling over 0.035 s instead of 0.02 the body
pitched on twice as fast in the first single support (the sole is a lag in
the ankle's hold) and the foot landed 0.24 s early; damped 1.75 or more the
stance foot's load flickered to 70 N under the rolling foot (a damped contact
kicks at its corners' speed) and it toppled sideways. MuJoCo's soft contact
is a poor foam: its softness a time constant on the whole body's mass, its
damping acting on every corner's motion.

## Start from a lean

- `machine.arrival` (2026-09-27): shifted onto the left foot, the weight goes
  7 cm ahead of the ankles over 0.6 s on both feet, then 5 cm further as the
  right foot lifts 6 cm over 0.3 s; the walker takes it mid-swing at the phase
  where the plan has the pelvis where it stands over the left ball
  (`Walker.begin`, as after a side step), the foot landing on the walk's own
  track 4 cm from the standing one. Set down first in the walk's landing
  pose, still, the pelvis tipped back 2 deg as the foot lifted and 5 forward
  as the walk took over. Handed on still, it hung back behind the landed foot
  and tipped over backwards (the phase pulled to a body standing still stands
  with it); landed on its standing stance, 16 cm out, the pelvis could not get
  over the foot and the next went 20 cm out to catch it; leant 10 cm in 0.5 s
  it was thrown off the left foot. The scoreboard as before, the rises held.
- The lean before the first step is the body's, not the pelvis's
  (2026-09-27): pushed 7 cm ahead with the torso plumb (`PLUMB` 1.0) the
  pelvis went out under a vertical trunk and the trunk pitched back half a
  degree at the push: read as leaning back before the step. Now the lean's
  frame tips the pelvis LEAN_DEG 4 forward and the torso with it, the neck
  keeping the head level; the walker takes the lean over at the hand-off and
  lets it out over LEAN_OUT_S 2 s. A lean kept through the walk, 2, 3 or 4
  deg: falls to the slip, the hot knee and the walk at 0.9, held 76, 76 and 66
  % against 90; let out, held 89.8 %. A stiffer spine steadies the torso, not
  the head: kp 800 -> 1600 -> 2400 N m/rad took the torso's pitch 2.1 -> 1.2
  -> 0.8 deg a stride and the ears' fore-and-aft 58 -> 68 -> 63 mm, the
  pelvis's surge 24 -> 39 -> 33: the spine's give is the filter between the
  hips' pulses and the head.
- The start as seen (2026-09-28): taking over from the lean, the walker's plan
  stood the pelvis up, 7 deg back in 0.2 s, the stance heel rising: on the
  toes and leaning back. The plan tips it as the lean now, LEAN_DEG 4 -> 8, to
  read as one. Risen 1 cm under the stand, it stood on 20-deg knees with the
  hips behind them: risen to the stand's soft knee (4) and sunk 4 mm as the
  weight goes left (`arrival.SINK_M`; unsunk the stance knee locked at -3),
  7-10 through the step. The weight's 6 cm onto the left foot rolls the
  stance hip -4.4 deg (-7.7 as the right lifts): a lift from standing needs
  the CoM over the left sole, and from the page's 60 deg the column's lean
  reads as leaning back. The head nodded 6.1 deg a step on the neck's 60 N
  m/rad; 300 at half critical: 2.2, the pendulum's stir 3.55 -> 3.00 mm, held
  89.7 %.
- The weight onto the left foot before the first step, partly as the right
  lifts (`arrival.SHIFT_IN` 3.5 -> 5.5 cm inside the left ankle, 2026-09-28):
  standing, the stance hip rolls -2.5 deg, not -4.4; the pelvis 33 mm across,
  not 61; the CoM 30 mm left at the lift, not 52; -7.8 through the step as
  before. Ten and sixteen perturbed starts (timings, gains, the lean's and the
  lift's reach): 3.5 held 10 and 15; 5.5 and 6 held 10 and 14; 6.5 held 6 of
  10; 7.5 one; the first steps falling at 9-11 s.
- Leaning back before the first step, from the page's recordings (R),
  2026-09-28: the pelvis sunk for the weight's shift bent the knees 4 -> 12
  deg under a plumb torso (the hips 26 mm behind the knees, the
  hips-to-shoulders line -0.5 deg, the torso 7.2 behind the shins); rising,
  the torso came up first, 18.6 behind them. Now the torso leans as far as the
  shins rising (`arrival.with_shins`, a keyframe at 80 % of the height), stands
  plumb, and leans on the lean's 4 from the shift on as the knees bend 4 -> 8,
  the sink 2 mm (1 and 1.5 mm fell): the hips-to-shoulders line -0.4
  standing, +3.8 shifting, the torso at most 5.7 (8 from the shift on read as
  unnatural). The seam to the walk: the arrival rode the torso on the pelvis
  and the walker eased its spine in from the hand-off's, so the torso swung
  10.8 -> 14.0 -> 7.5 deg as the pelvis tipped; both now take the pelvis's tip
  back out of the spine (`walker.PLUMBED` not eased in): the torso 8.7-9.8
  through it, the largest swing back in 0.3 s -6.6 -> -3.0. Held 89.7 %, 15 of
  16 perturbed starts.
- The seam's odd lean, from the page's recording (2026-09-28): the lean and
  the step's keyframes moved the pelvis 5-9 cm on at the shift's height, so
  the standing knee was planned straight and bent back 1.8 deg for 0.35 s
  while the body tipped on over it like a plank (the ankle-shoulders line +5
  -> +9). Lowered until that knee bends as in the shift (`arrival.soft`,
  SOFT_KNEE 8): at least +6.9 through the seam, bending on 7 -> 19 as the body
  goes over it; 15 of 16 perturbed starts. Lowered in the lean too, the head
  dipped 5.1 mm there, not 3.3: the step's alone now. The curtsy into the walk
  (tools/sim/look.py): the head 22 mm down in the step and 31 at the first
  landing whatever the sink (0.5-2 mm) and the lean (0-4 deg; at 0 every start
  fell); the walk rides 16-28 mm under the stand and sags 7-10 mm under its
  own target at landings.
- Between the rise and the walk, moment by moment (the page and
  tools/sim/look.py number them: 5 stand, 6 shift, 7 lean, 8 step, 9 walk;
  look.py prints each seam 0.3 s either side, the director's asks beside),
  2026-09-28: the knees bent 4 -> 9 in 6 and the torso bowed 1 -> 5.7 in 7, a
  curtsy before the step; in 8 the pelvis's target lagged 13-46 mm behind the
  body and bent the standing knee 5 -> 16. Now it rises to the shift's height
  (knees 9.4 standing), 6 and 7 still (the torso 1.0, the pelvis -0.2 mm, the
  weight 3 cm on), and in 8 the target not pulled back (`fall`), the lean's
  tilt and 9 cm more: the torso 0.9 -> 4.9 and the pelvis +4.2 then -5.8 mm
  with the lift; the walk's first second -21.5 where -26 to -30. 15 of 16
  perturbed starts, held 81.3 %.

## Look at 0.85 strides/s

- By the trace (2026-09-27): the stance knee 16-29 deg (the plan's own 16-24:
  the pelvis rides 10 mm under the plan, latched there at each landing as the
  trailing leg sags 9 mm into it, its ankle drooping 2 deg under the
  push-off's torque, and raised at 0.1 m/s: a rush of 0.22 m/s to a dead
  stop). The ears 19 mm up and down a stride and 33 fore-aft, the torso
  nodding 7.6 deg (the counter); 66 fore-aft with the torso still: the
  pelvis's own surge, the collision at every touchdown (-0.25 m/s over 0.13 s,
  +0.2 back before the next).
- The stance knee 8 deg soft, the heel strike 15 deg toes up (its peak 1.55
  -> 1.1 kN), the recovery 0.03 m/s: the knee 9-29, the stir 1.4 -> 1.2 mm,
  held 86 %. The counter off for the eye: 3.5 mm.
- Straighter legs at the strike need the trailing heel up before it (the
  plan's height there is that leg's, flat, 22 cm behind): a heel rising in
  single support ran the body ahead of the plan and it sank 35 mm before a
  landing 0.1 stride late (the phase, pulled to the body at 5 a stride, lags
  it); the heel to 50-55 deg at toe-off felled the walks alike.
- The calf (2026-09-27): the heel to 50 deg at toe-off (`gait.TOE_DEG`, rate
  -300 a stride, turning back at 4000 a stride squared), the rise scaled by
  the stride. With the foot landed on the capture point along the walk it
  holds: the walks' stir 3.35 -> 2.80 mm, the ears' up-and-down 18 -> 12 mm a
  stride, the plan's landing knee 29 -> 14 deg, the hot knee walked out
  (tipped 12), held 90 %. Unscaled, the first short strides' push-off hopped
  off the front foot (the rises fell); 55 and 60 deg, an earlier heel-off and
  a slower rise all lose trials. After toe-off the toes' tips skim the floor
  (the sole's peak 1.4-1.9 kN there): curled up 20 deg they touched it still,
  the toe drive too slow.
- The head's fore-and-aft, 61 mm a stride at the ears (2026-09-27): the pelvis
  surges 30 (its speed 0.76-0.99 m/s over a 1 m stride, the vault's exchange
  plus the stance leg braking it 2 m/s^2 through half the stance and pushing
  3-5 before the landing), and the torso's pitch, 2.2 deg the spine's counter
  does not take out, doubles it up there. The stance knee 6 deg soft: 4-7 at
  its straightest, 15-20 through mid-stance from the latch's 10-12 mm; at 4
  the walk at 0.9 fell in 0.6 s.

| Tried and out | Result |
| --- | --- |
| the phase run as a pendulum | 58-65 mm |
| a lead on the pitch's rate for the spine | fell in 4 s at 0.05-0.15 s either sign |
| a shorter stride (0.9) | the pelvis 38 at 0.82 m/s |
| a planned landing dip | the body sags under any plan, the knees bent the same, the bob 19-24 |
| the height latch let go of its first 15 mm | the first landing hopped |
| the bob's swing on the landing (`PEND_K` 1.5) | fell in a second |

- The head in the walk (tools/sim/look.py's walk line, from 2 s in,
  2026-09-28): 53 mm fore and aft a stride (the pelvis's surge 31-35 and the
  torso rocking 3.3 deg at 0.6 m), 13 up and down. The ears' pendulum at 0.1:
  48.5 mm, the strike 1613 -> 1115 N, held 87.9 % against 81.3.

| No better | Result |
| --- | --- |
| the torso's counter to the surge at any phase (1.5, 3 deg) | 53-74 mm, some fell |
| the pendulum past 0.1 (0.15) | 57 |
| the legs' drives 1.5 times stiffer | 44.9, the strike 1410 N; 2 and 3 fell |
| the phase pulled less (3; 1) | 57.6, 2 fell; 46.3, 1628 N |
| the shoulders over the line (0.5) | 26.7 mm to the hips' 44.9; cost the walk at 0.65 |

## Build changes, 2026-09-28

- The feet 27 -> 24.5 cm: held 81.8 -> 85.3 %, the strike 1.1 -> 1.4-1.6 kN.
- After 100 s it sank 172 mm into a crouch: at 80 m it stepped off the floor's
  slab, each landing latching the height lower. The slab runs 10 km.
- The limbs collide (`physics.ME`): floor alone, the sneakers passed 22 mm
  into each other; a swinging foot is kept off the other (`landing.clear`).
- The swinging hip ran 5 deg behind its setpoint and caught up into the floor,
  1.5 kN in 2 ms: the swinging leg's plan leads 20 ms (`landing.SWING_LEAD_S`).
- The feet rolled 2 or 3 deg onto their outer edges took the ankle's roll to
  3.2 and 2.0; caught from 10 s.
- The swing skims (`gait.LIFT_M`) 25 mm over the floor, 42 at most, where a
  bump lifted the foot 130 mm and it trod the air; levelled at 0.6 of the
  swing the knee bends once, to 55. The heel up 10 deg as the other lands
  (`gait.RISE_DEG`): the knee lands at 13 where 33, the head's bob 13 mm where
  23; the look suite's impact 1046-1599 N where 228-1358.
- Turned 9 deg, dropped 6, the spine taking 0.3 of the hips' offset back: the
  hips 72 mm across, the shoulders 25.
- Toed out with a small swing turn it fell (5/4, 6/6, 8/8 deg): the swing aimed
  its ankle, the landings recorded its ball, 12 mm wider at 6. Aimed at the
  ball, 8 out and 6 in: 7.4 in stance, 2.5 out swinging. The ankle rolls
  5.5-5.9 through the loading, the foot flat: tracks at 40-50 mm take it to
  4.8-3.6, but it cannot balance on them.
- The arms swung from the forearm, near the body (`gait.ARM`: the shoulder 12
  deg a side, the elbow 24 +- 11, the wrist 10 +- 7). The hips already swayed
  35 mm to the shoulders' 23, the pelvis rolling 10 and turning 39 deg a
  stride; a deeper hip drop (ROLL_DEG 6, 8) swayed the shoulders 28 and 33 mm
  and nodded the head 2.9 and 3.6 deg. The pelvis tipped 6 deg under an
  upright torso read as leaning back from the rise on (the page's spine at
  -5.9, the torso bent back over the hips): out again. The scoreboard cannot
  judge a look: the committed arms moved 2 % (the shoulder 16.3, the elbow's
  swing 9.2) held 80 and 75 %, the slips and the hot knee flipping; every arm
  tried held 84.7-85.5 %, the chosen ones 89.7.
- The stance leg (`tools/sim/look.py`: the ankle ahead of the hip at the
  landing, the toes behind it and the thigh's angle at the lift, a load held
  0.1 s): as it walked, the thigh left the floor 1.1 deg ahead of upright, the
  toes 196 mm behind the hip: the feet in front. The toe-off at 0.66 and the
  ball planted at 0.24: 12.2 behind, 297 mm. The stride 0.85 m: the pelvis's
  dip 49 -> 34 mm and the head's surge 56 -> 35 mm, the thigh 29 ahead at the
  landing and 6 behind at the lift, held 83.7 %, stir 2.3 mm, the scoreboard's
  cheapest yet (7.18). Ahead stays ~20 deg more than behind whatever the knobs
  (the stride, the toe-off, the stance, the heel's rise, the knee's softness):
  the landing knee bends ~30, the hips held down by the trailing leg in the
  double support.
- Clothes: a tee, jeans and sneakers, drawn loose over the body, the drums
  under the cloth patched on it in their heat's colour; the cloth felt in
  MuJoCo: denim (0.55) on the seat, the thighs and the knees, cotton (0.45)
  on the torso and the upper arms, 4 mm of give.

## Build as modules

- The pelvis rocks about the hips' mid-point, a beam engine's beam
  (2026-10-02): in single stance its still point -0.08 of the way to the
  stance hip, that hip 14.3 mm up and down a stance, the swing hip 17.2.
  Dropped HIP_HALF sin|roll| about the stance hip: +0.67, 10.3 and 17.3, but
  caught 5 times in the walk's first 3 s from the squat where not at all, the
  head 52 -> 90 mm fore and aft, and after A 9 times in 5 s where none. The
  parry's 16 shoves held 1 and 1 of 8; dropped all the way 2 and 4; half way 3
  and 2, but half way it fell from the squat at 12.1 s. Undone until the walk
  is retuned with it.
- The parry on the build as modules (test_gynoid_falls' 16 shoves of 38 N,
  held left and right of 8, 2026-10-02): `landing.PARRY_HURRY` 0.32 1 and 1,
  0.28 2 and 2, 0.25 3 and 3, 0.22 0 and 1, 0.4 1 and 1; split, a shove 0.25
  and a catch 0.32, 1 and 2; a catch 0.30, 2 and 1: no order, its floor HEAD's
  build's 2 of 8, the test's. A knee derated to nothing walked on at 0.32 and
  fell at 0.28 and 0.25. 0.32 kept, the parry to the walk's retune.
- Retuned for the trunk's roll first and the gimbal's drives on their stages
  (`physics.STAGED`, 2026-10-03): 9 knobs (the arrival's shift, lift and lean;
  capture's margin and gain; the walker's side gains; the parry), 5
  generations of 8, the median 468 -> 335. The board 578 -> 299, held 76.8 %
  against main's 495; the same knobs to 17, 6 and 3 digits 263, 306 and 420:
  its chance. `landing.PARRY_HURRY` 0.29.

## Form

- The walk went Groucho: legs out in front, sneaking (2026-10-05; `look.py`
  from the squat, the left leg through its stance, 8-9 strides):

| Build | Knee at landing | As the leg passes plumb | Going back | Pelvis under its stand |
| --- | --- | --- | --- | --- |
| 2026-10-03 (945a6a8) | 24 deg | 6.8 | 2-8 | 9-29 mm |
| the motors off the feet (e957a3a) | fell at 9.2 s | - | - | - |
| 5d2f296: the heel off at 0.5 where 0.36 | 34 | 20.6 | 16-17 | 22-53 mm; scoreboard 78.6 % held where 65.8 |
| the slip's fix (eb03ee0), `stance.HEEL_REACH` 0.99, stride 0.75 m where 0.85 | - | 16.4 | - | - |
| a woman's catwalk take (`tools/sim/mocap.py`'s, scaled: thigh 0.40, shank 0.41 m; 1.09 m/s on a 1.29 m stride, 0.85 a second) | 1-3 | 2-5 through the stance | the leg 23-25 behind plumb as its ankle moves, the toes not back | 20 mm under its top at each landing, 31 mm a stride |

### Straight leg cost

A straight leg costs more than a bent one, and where (2026-10-05; steady at
0.85 strides/s, 0.63-0.66 m/s, the boards' own 53 W in each): the walk as
built 365 W, 561 J/m (work 151, copper 160).

| Stance knee | W | Hips' and knees' work and copper W |
| --- | --- | --- |
| 6 deg | 402 | 247 |
| 18 | 264 | 133 |
| 30 | 256 | 123 |

- By the left leg's phase, a twentieth each: where it only stands, the knee
  at 1.5 deg, the knee draws 1 W at 6 N m rms; landed bent 21-27 deg and
  straightened under load, hip and knee 620 W over 0.12-0.17; released at 6
  deg into a swing of 57, 895 W over one twentieth; the two legs solved for
  one pelvis in the double support, the hips' roll 73-78 N m rms each. The
  mechanics ask some 10 W.
- A leg carrying as its sole bears of what the two bear (lab): 364 -> 341 W
  as built, the knee at plumb 12.3 -> 3.4 deg, the toes back 2 mm, no parry.
  The power is steady from 2 s into the walk: 368, 372 and 373 W from 2, 4
  and 8 s.

### The stance leg a strut

2026-10-05; `gait._limit`, `stance.rolled`, `stance.carrying`: the hips no
longer planned down to the trailing leg's flat foot (held level before
2026-09-25 the knees stood 33-40 deg, swept 16 at most, the hips up and down
20 mm); the leg held at its soft knee's length, its heel solved to 30
halvings; a leg carrying as its sole bears of what the two bear; the knee
giving up to 24 deg through the double support. Steady
(`tools/sim/armada.py`), new where old:

| Strides/s | J/m | Knee behind plumb deg | Landing deg | Toes back mm | Parries | Head bob mm | Strike N |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.85 | 393 where 561 | 7.1 where 16.7 | 28 where 33 | 0.8 | 0 where 0 | 13 where 22 | 363 where 391 |
| 0.65 | 384 where 885 | 6.8 where 16.5 | 25 where 31 | 0.5 | 0 where 8 | 12 where 25 | 401 where 367 |
| 1.0 | 604 where 608 | 8.1 where 18.3 | 26 where 34 | 0.1 | 0 where 0 | 16 where 28 | 385 where 399 |

- The pre-swing's knee: steady at 1.0 strides/s 18 deg 4 parries, 20-25
  none, 28 3; from the squat 20 one or two as its start ends, 22 two and 7 at
  1.0, 23 one at 0.65 and at 1.0, 24-26 none; under 15 it fell, a strut to
  its lift too. Shoved past saving 16 times: the head at the floor at 1.6-4.0
  m/s once or twice at 23, 25 and 27, at 0.5 at 26, never at 22 and 24; sat
  back limp out of the crouch each time.
- By the stance knee, J/m at 0.85 (the pre-swing at 25): 3 deg 426, 4.5 408,
  6 398, 9 380, 12 371, 18 394; the heel solved to 12 halvings 545, 429, 394
  and 381 at 3, 6, 9 and 12, a knee at 3 deg asked 0.2 deg another way each
  pass and its board reading its rate between two frames.
- Its own recordings of 2026-10-02 and -03 (build/recordings): the knee 2-5
  deg where the foot bears alone behind the plumb line, to -3, landing at
  13-28. The gynoid suites: 28, the falls 44, the faults 9, the stand 6, the
  gait 41, none failed.

### The landing knee by measure

2026-10-05, steady at 0.85 strides/s: 28 deg at touchdown, 6 at 0.19 of the
stride; 23 mm of leg let out under load: 12 the pelvis's roll (its landing
side 2.9 deg low at touchdown, `gait._roll`, its peak at 0.33), 8 the sag
(6-8 mm under its target at 0.39-0.49 of the trailing foot's stance), the
rest the plan's dip.

- The roll's peak at 0.29 and 0.27, steady over 6 side gains: on its form,
  the knee 25, 22 and 21 and 24, 21 and 22 deg at 0.85, 0.65 and 1.0
  strides/s, the head's bob 10 and 8 mm, and the starts parry
  ([balance](balance.md)).
- The swing's reach (`stance.SWING_REACH` 0.985-0.9985): nothing.
- A leg taking the body from where it is and up at 0.12 m/s: the knee back at
  19 deg at 0.65, 383 -> 483-558 J/m: the leg's snap is the load's transfer.
