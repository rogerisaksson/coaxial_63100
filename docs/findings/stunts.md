# Findings: stunts

The mocap takes as her scenarios (`tools/sim/stunts.py`, `scenarios.STUNTS`),
each judged against its take; the scenarios' bench: [scenarios](scenarios.md).

## The takes as scenarios

Each take under stunts/ beside the repo a scenario (`scenarios.STUNTS`), her
rows measured as the take is (`tools/sim/stunts.py`, MOVES: lengths in legs,
times in sqrt(leg/g), speeds in sqrt(g leg)) over the take's seconds from the
telling, judged within a sole's length (0.22 legs), her list (8.6 deg), her
arm's cone (20 deg) and one natural time (0.28 s); her head over the whole
run no lower than the human's, a fall's head under 1 m/s. The takes (a leg
0.89 m, a tick 0.30 s), kept in tests/takes/stunts.json:

| take | the move |
| --- | --- |
| Stand To Sit | drops 0.52 in 2.4 ticks, hips 0.48 back, trunk 43 deg forward, knees 95, feet down |
| Sitting on Bed | drops 0.46 in 3.2, hips 0.41 back, seated 9 deg off plumb, knees 99 |
| Sit To Stand | rises 0.52 in 2.8, trunk 53 forward |
| Lying Down from Sitting | hips 1.07 back in 6.9, lying 94 deg |
| Situp To Stand | rises 0.98 in 3.5 from the head at 0.05 |
| Crouching Down, Crouch Up | drops 0.66 in 3.7 (knees 140), rises 0.58 in 2.3 |
| Kneeling Down, Kneeling Up | drops 0.73 in 2.4 (knees 155), rises 0.66 in 3.9 |
| Standing Jump | hips up 0.55, feet 0.91, in place |
| Jumping Up, Jump Down, Jumping Down | 0.70 up or down, 1.4-2.6 ahead |
| Running Jump | feet 1.08 up, 3.96 ahead at 1.59 |
| Jogging, Running Forward | 0.74, 1.64; backwards 0.49, 0.93 |
| Moonwalk | 0.39 backwards, the feet 0.06 up |
| Hip Hop Dancing | bounces 0.38, the arms 80 deg, in place for 34 s |
| Standing Yawn | the arms 123 deg, 13 deg back |
| Turning in Bed | rolled 88 deg, lying |
| Backflip, Run To Flip | pitched 720, 367 deg; feet 1.87, 2.59 up; hips 0.65, 1.08 |
| Falling on Butt, Tripping Forward, Slipping Backwards | the head to 0.27, 0.04, 0.04 |
| Ascending Stairs, Walking Up The Stairs | 0.55 up over 0.57 ahead in 1.2-1.4 s |
| Edge Tripping Backwards | caught: ends level, the head 0.94 |

- Her first run: 2 of 31 met - the slick patch and the ice, crossed standing
  -; 6 refused for want of a text (backflip twice, kneel twice, yawn,
  turn_over); 23 short. Shoved back 200 N standing she fell onto her head,
  0.02 legs at 1.6 m/s, where the human sits down to 0.27; shoved ahead
  walking, head 0.11 at 1.1 m/s (the human's face to 0.04); backed off a
  step's edge, head 0.02 at 1.8 m/s where the human catches himself at 0.94.
  A crouch drops her 0.11 legs where 0.66, knees 54 deg where 140; a sit's
  hips 0.20 back where 0.48, its trunk 25 deg forward where 43; on the bed
  0.15 back where 0.41 and her head later to 0.69; the chair's hop down
  leaves her on it (ended 0.00 where -0.69), the hop up falls, head 4.3 m/s;
  a standing jump lifts her soles 0.10 where 0.91 and her hips 0.01 where
  0.55; W runs her at 0.34 sqrt(g leg) where a jog is 0.74 and a run 1.64, S
  backs her at 0.07 where 0.49; the dance bounces 0.05 where 0.38, arms 27
  deg where 80; rising from the chair she is 0.19 up where 0.52 by the
  take's end; lying from sitting her head went to 0.17 where 0.60 (the
  roll); the stairs fell; the floor rise 0.17 up where 0.98.
- A posture's take told as its own rows (`stunts.modes`, `stunts.wire`; the
  take read into her modes as `modal.reading` reads hers, off the take's
  standing frames, its centre of mass by her figure's segment shares on its
  bones; `{'take': key}` in STUNTS). Streamed whole, the human's stance
  felled her: his feet widened and shifted through a crouch (head 0.02 legs),
  his pelvis's 20 deg list or his heels' rise in it (0.04, 0.24), his arms'
  swing (0.05, hands' jerk 29 000 m/s^3). The axial, sagittal and pitch modes
  alone: the crouch 0.62 legs down where 0.66, knees 128 where 140, the kneel
  0.69 where 0.73, knees 144 where 155, both met (far 0.0; the sagittal from
  his hips instead of his centre of mass put her capture point at her heels,
  hands 8-12 000); the sit's hips 0.06-0.56 back where 0.48 run to run, its
  trunk 45 deg forward where 43. 4 of 31 met, 235 widths off in all. Her
  hands' jerk, the spasm, where her walk's is 1 400-1 800: the sit 20 800,
  rising from it 19 800, the dance 20 200, turning in bed 16 000, the bed's
  edge 12 700, rising from a crouch or a kneel 8 800-8 900, the runs
  770-1 400. The dance's bounce of 0.38 legs streamed as axial felled her
  (0.98 down, 1.9 m/s); told to lean back 13 deg standing she leans 3.7.
- The trunk's pitch zeroed at her stand (`fr[PITCH0]`) instead of the take's
  standing frame: the rig stands with its neck 3-6 deg behind its hips, and
  zeroed there she leant 5 deg past the human and a crouch toppled her
  (forward 106-130 deg at every wobble). Off her stand the crouch stands 0.58
  legs down, knees 126, forward 45.6 where the take's 44.3, her hands' jerk
  2 300 (5 400); the yawn's lean back 13.6 where 13.5. Not kept: the step's
  upper rows at W_TURN - the sit's hands 20 800 to 4 800-6 700, the crouch's
  5 400 to 900, but the walk 2.75 off a woman's (2.55), the pelvis rolling 3
  deg where 5; the arms' rows alone at W_TURN - the walk as it was, but the
  crouch's spine folded for the lever instead and she fell. 3 of 31 met, 255
  widths off: the crouch within every width but its descent's time, 0.53 s
  from a tenth to nine tenths of the way where the human's 1.03 - his hips
  curve back and then down, hers drop straight.
- Seated, traced at 10 ms: her height lagged the take's braking (4 m/s^2) by
  a/kp = 7 cm and met the seat at 1.2 m/s, a 1.5 kN hit; the seat held her
  hips above her legs' asked length, the springs pulled both soles up 2 cm,
  each was swung and landed as a stand her centre of mass was behind, and
  told her to step - her hips slid 0.31 to 0.07 m off the seat, the chair's
  share 670 to 33 N, the arm 939 deg/s. Three laws changed. The prop counts
  by its share of her weight, not whole at 0.15 of it (that switch made the
  span and the relaxation come and go), and spans her support once it pushes
  as a touch she counts (by its share, half of her on a chair left her centre
  of mass outside and she stepped, a sole 12 cm up till she toppled); the
  soles' lift by the excess is gone. A take's rows, each beginning where the
  last blend ends, blend as a Hermite curve with the row after's rate
  (Catmull-Rom), so her rate and acceleration are the curve's own; curving
  the texts' blends too felled seven held scenes (moves, strafes, spin,
  weave, sill, ice, snow). The bed's sit by the take then: the bed 0.95 of
  her, hands' jerk 17 000 to 1 800, hips 0.13-0.30 legs back, but her feet
  hang - a seat of 0.45 m over shanks of 0.455 with her thighs level - and
  the 5 % of the excess left unrelaxed lifts them; the chair's sit by the
  take perches (hips 0.16, soles 0.75 of her) and fell at one wobble, so the
  held chair keeps its texts (3 of 3). Not kept: the take's legs scaled to the
  seat's height (she hovered 6 cm over it, her hips not back); the seat told
  to her law as a support before it bore her (her hips went back onto the
  chair where the human's go, 0.36 m, and she rolled off it sideways; told
  the whole mattress she lay down). No rise from the bed stands her: the
  text's from the take's lie leaves her on it, the sit-up take's rows are
  held by `risen_back` (a raise waits while the capture point is off her
  soles), the chair's rise by the take fell at 2.7 m/s with her hands at
  18 000. The C mirror's striding solve on two soles, nothing pinned, as a
  stride began at a share of 1e-8: the torques 2.2e-2 N m off the python's;
  the test judges two soles by her accelerations and loads, 3.6e-3 apart.
  The bench then 4 of 31 met (the slick patch, the ice, the kneel, the rise
  from a crouch), 226 widths off from 255: turning in bed -15.5, the dance
  -5.4, the rise from the chair -4.3, lying down -2.8, the bed's sit -2.2;
  the rise from a kneel +2.5 (fell at 3.1 m/s).
- The lie by the takes on the page, measured and not kept: the law back to
  the seat round's, the lisp's lie_bed the page's text, `machine/takes.py`
  telling a take for the bench. Kept: a take's legs hip to ankle, the
  straight leg their zero (hip to sole had the ankle's 0.05 m in it; the
  takes' standing ends' median fell 3.7 cm short of a straight stand); a take
  enters over pi sqrt(leg/g) = 0.88 s; a blend's arriving rate over the next
  row's own span (over this one's, a 0.88 s row after 31 ms ones dipped her
  axial 1.8 to 0.5); a raise waits only told over her soles (the lying take's
  legs straightening behind them froze its rows). At axial 1.8 her hips
  0.284 m behind the soles' middle sit at 0.585 m, the seat; balanced over
  her soles, 0.09 back, she hovers at 0.63 - her law never lets her centre of
  mass behind the heels. Told past her soles as a support she trusts (the
  capture point's box, the parry, her surety) she tipped back from the hover
  at 0.34-0.4 m/s onto the bed; the ZMP's box grown over the told point asked
  the soles a ZMP behind the heels. Landed on target (sagittal -2.52, told
  -2.71) with her legs' spring relaxed by the gap in their reach (as her hips'
  height it drew her 0.15 m forward off the chair), a sole 2 mm up was swung
  5.6 cm by the clearance, her knees bent to the told axial folded the
  hanging legs up 0.27 m, and the pelvis's pitch task on a point contact went
  to the arms - elbows 2.05 times their clamp, the step shrunk to 0.1-0.4,
  hands' jerk 48 000 - and she rolled 57-157 deg or fell at every wobble; the
  pelvis's turn let go to the seat tipped her 82 deg in 0.6 s. The lisp's
  lie_bed since the soles' lift went: a 27 deg recline, the bed 0.39 of her
  and her soles 0.61 (the step splits her weight evenly over three contacts)
  where it bore 1.08 with her soles 44 mm up; reclined 57 deg she rolled 90
  deg at 2e-4 rad. The lying take's hips go back 1.07 legs: a shuffle her
  loop has no mode for. The chair by its texts 5.8 s, the walk 0.70 m/s, 409
  J/m, 2.55 off throughout. The bench after: 5 of 31 met (the slick patch,
  the ice, the kneel, the chair's sit and rise), 214 widths off; the crouch
  takes, half a unit deeper with the straight leg their zero, bottom out at
  the human's full squat (knee 140 deg, drop 0.93 legs where 0.66) and sit
  down at 0.3 m/s; hip_hop, standing_yawn and turning_in_bed (630-1320
  frames) thinned to every second or third for the table's 512 rows
  (wbc_wait.c). The viewport's bed errand, broken since the seat round: at
  the dock her shank brushes the bed's frame, 0.07 of her weight up, and the
  prop counted by its share alone her parry steps her back into it and over
  the edge (2.3 of her weight on the bed, her surety 0, no lie said); the
  count and the span whole from 0.15 of her weight (PROP_ON, the ramp as
  before the round; spanned by the share against her soles she fell the
  same) she stands at the dock and lies, bed_rest at 26.2 s. Her soles'
  lift by the seat's excess kept after all: down they bore 0.61 of her
  reclined on the bed to its 0.39 and the page's rise from the recline fell
  at the click. The chair's 'lean', the one text row begun on as it ends,
  curved as a take's frames are, bent its rise with the arriving rate's
  span and she fell: a row curves only flagged a curve's sample
  (`ortho.CURVE`, a take's). Then the bed holds at 0 and 2e-4 rad (rolled
  8.0 and 0.3 deg, 13 at -2e-4), the chair at 0 and 2e-4 (2.6, 5.1 s); at
  -2e-4 her perch slides off the chair and she falls at 9.06 s, as at the
  base before the takes. Measured, not kept: her soles lifted by the prop's
  share instead of its count (the chair 3 of 3, 4.4-7.5 s, the bed 3 of 3)
  brushed the bed's frame docking and stepped her over it.
- The standing jump laid from what the drives give (`leap.standing_leap`,
  `pushed`, `absorbed`): the crouch his own frames (`takes.frames` to his
  crouch, 0.465 s, by its depth), held one period of the load row's spring
  (2 pi sqrt(m/K), 0.45 s) so the push begins from rest - told a line to the
  crouch she fell 70 ms free and her parry stepped a sole; told while she
  sank, the step caught her up at 14 m/s^2 -; the push a curve of rows 20 ms
  apart from the knees' clamp at 2/3 through the knee's lever at the crouch,
  36.3 axial/s^2 (5.2 m/s^2 net, 466 N) for 0.34 s at full charge, the
  throw's share of it the crouch's share of his (at the whole clamp a tap
  left at 10.5 axial/s of a hold's 12.3; the clamp's share so, a tap's push
  fell short of her weight): take-off 1.5-1.65 m/s, rise 0.43-0.51 legs
  of his 0.55. Four faults of the law on the way, each measured. The latent's
  curve left a row at the Hermite's end slope of the row before though that
  blended linearly (the push's first row from a held crouch at 4.6/s,
  709/s^2) and began each row from the reference at the tick that met it:
  20 ms rows on a 3 ms tick held one tick in seven, the acceleration -228 to
  +98 for its -36; a row's blend begun at its own time instead tipped the
  chair's rise (a 3 ms phase, a knife edge) and was undone. `touch.step`
  overwrote the latent's acceleration with the sway's term: no curve's feed
  reached the impedance; it reaches it only with no polygon held, a throw's
  rows - held, a take's frames 31 ms apart bent 118 sagittal/s^2 and 56
  pitch/s^2 of noise and the sit-down take fell seated at 6.9 s (3.8
  widths, 0.6 now; the standing jump 3.9 -> 3.0). The whole-body step's
  knee rows damped against
  still, so its height band pinned the push at 3 m/s^2 under its ask with
  the knees at 0.2 of their clamp; they damp against the told height's rate
  where the rows let the polygon go (`modal.BEND_RATE`, the ask's
  `knee_rate`) - under a held polygon the seats' texts, laid against that
  damping, sat her onto the bed at 1.86 of her weight or had it bear 0.00,
  and at her own rate past AHEAD a stride's instant throw flew 0.20 s for
  0.23. The surety read a mode's phase (out[57], the ZMP's slot in the
  table) for the polygon the rows hold, a sixth of every rhythm sure of the
  air: the held polygon is read beside it, the phase read kept - without it
  the bed bears 0.00 of her. The landing, measured to its law: told his
  pre-folded squat she had 0.07 m to brake 1.6 m/s in and sank to the
  floor; long-legged on a spring till the clock's touchdown she met the floor
  0.1 s early at the apex (973 N) and bounced to a second fall; a parabola
  from the push's end speed yanked her down at 23 m/s^2 where she met the
  floor slower; his own rise told as rows carries his rebound (a second
  squat at 1.3-1.5 s) and she jumped again off the landing; the crouch's
  frames quickened by their depth dropped a tap 0.2 m off her soles. Kept:
  legs long and soft in flight (no height spring), the absorption a damper
  told the bottom the drives brake her to from straight legs (`bottom`: the
  clamp's work along the squat against her landing speed, the whole crouch
  for a hold's 12.3 axial/s, 0.1 of it for a tap's 5), rows that lower her
  waiting for a floor under her (`waits.landed_rows`), her rest over his
  rise's seconds: she stands, 2.9 widths off (rise 0.49 of his 0.55 from the
  crouch, travel -0.16, head 1.07, flight 0.04 of his 0.91 - the tuck is
  beyond her drives), the same in two runs, but on one leg: the push's
  last rows loaded one sole (0/497 N), she landed on the other (1 449/0
  N) and stood on it - her capture point 0.14 m past her toes at the
  push's end had her parry step a sole. The parry steps her only while
  the rows hold the polygon (`a[NEED]` by out[HELD]; let go through the
  absorption too, no step saved a landing on its tip and she fell: the
  absorption holds it again): off and down on both soles, 3.9 widths
  (travel 0.42 legs in the bench's scene, 0.04 m in the page's), her soles
  11 mm clear; a tap hops 0.07 m from its dip and stays on its spot; her
  apex over her stand is 0.015 m, the legs 0.5 axial short of straight as
  she leaves. A sole let go is out of the parry's support under any sway
  (`lifted`, `1 - still`): the sole a stand's sway hung 2.4 mm up
  ([feet](feet.md), test_sysid_live's three reds) is parried in place, 15 mm
  up and down at 172 N, her capture point 2.5 cm past the one sole left
  while the other lies under it. Measured, not kept: counting it unless her
  drive strides left a sole still in the air at a sprint's end in the
  support and she fell (sill, hole, snow, sprint, turns); counting it
  within its sole's give (5 mm) of the floor let a sliding sole on snow
  count (snow +-2e-4 fell at 9.35 and 7.61 s), the run's leap held 0.55 m
  for a tap's 0.73, 1.5 kg on her back read 0.00 kg off her insoles; the
  swing's clearance scaled by its reach over a sole's half-length (where it
  lies, none) still lifted it 10.8 mm and landed her weight on it at 265 N,
  the parry's landing 5 cm off. Open: a let-go sole counting by its grip.
  Measured, not kept: the push integrated with the lever's
  force along it (the knee's lever growing as she rises, 18 axial/s at
  take-off for the parabola's 12.3) flew her 0.21 s with 20 mm clear and
  stood 3.5 widths, but a tap's late burst left its legs 0.5 behind and
  never off the floor; on the motor's torque-speed line through the lever
  (10.5 axial/s, no lever for a straight leg's turn) she fell at 7.3 and
  the hold flew 0.13 s: the stack's lag at the push's end, not the
  drives', caps the apex. Her walk unmoved: 2.53 off a
  woman's at 0.70 m/s and 409 J/m, 2.55 at the commit before.
