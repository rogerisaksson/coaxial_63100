# Findings: standing

The rigs, the one law in the capture point's plane, the push polar. Kept up
walking: [balance](balance.md); the board's findings:
[FINDINGS](../FINDINGS.md).

## Rigs and the stand suite

`machine.stand` (2026-10-04): the director holds the arrival's stand
`stand_s` (its keyframes lifted `up` onto two bricks, the left `stagger`
ahead), the floor rigged under it (`machine.floor`: two bricks, a balance
board hinged in the slab's gap, stiff at BOARD_K or free), and the events
standing (`events.STANDING`, the scoreboard's 'stand' suite: a nudge or a
shove from the side or along the way, a brick taken away, the feet abreast or
staggered, a nudge on the board stiff and free).

- On the box soles alone (no step), 12 s: nudges 100 %, the stiff board 100
  %, the free rocker 65 and 100 % (rocking sideways, ahead), the shoves 67
  and 50, the bricks 50 and 41.
- The arrival's pelvis target no further than `arrival.PULL_M` (5 cm) from
  the pelvis: a get-up handed over with the CoM 12.7 cm ahead of the squat's
  target sent it 19 cm off, the stance legs straightened toward it and flung
  the body to 1.04 m (both get-ups after P's shove down in 3 tries). Clamped:
  both walk again at 21.8 and 22.2 s on the first try; rises 70.5 -> 91.8 %
  (10 cm: 81.9).
- The free rocker felled every rise with the target held within 5 cm. The
  pull by stage: 5 cm in the squat and the look, where the get-up's hand-over
  flung the body; `PULL_UP_M` from the push on. At 0.15 it rises on the
  rocker and stands through the nudge: stand suite 65.2 -> 74.7 % (the rocker
  27 -> 62, rocking ahead 21 -> 85); look suite 69.9 -> 64.2 (rises 76, 100,
  99 -> 90, 75, 71 at 0.6-0.9); both get-ups after P's shove walk again.
  Standing first: 0.15.
- The 15 cm pull through the push, the rise and the stand alone, the first
  strides at 5 cm as the squat: rises 76, 100, 99 % again (91.8), the stand
  suite's 74.7 kept.
- Point feet (`figure.POINT_FEET`, a sphere under each sole's centre): from
  the squat it falls in the rise at 1.77 s; the arrival's feedback is the
  ankle's, nothing on a point. Stilts need a step to rise.
- Shoved standing from the side (`look.py --stand shove`, the shove's
  newtons a knob): 60, 80 and 100 N for 0.12 s all felled it at 6.2-6.5 s,
  two or three steps taken; 38 N held without a step. Past saving standing
  lies between 38 and 60 N.

## The standing step

2026-10-04:

- Out of the support (a point under each sole's centre; one once a foot has
  hung unloaded 0.1 s) the lighter foot is put down past the capture point
  as it will be at the landing, as the arrival's keyframes (0.1 s up, 0.12
  down, the swing foot never a stance leg and pinned where it lands, the
  pelvis lowered to reach).
- Taken by load: a shove's first 30 ms unloaded the far foot to 31 N and it
  stepped to its own side, a no-op. The walker begun from the stand instead
  marched on at a 5 % stride and fell every time (391 and 48 %).
- A 120 N shove runs the capture point 16 cm out in 0.25 s; the cross-over
  cannot reach.
- The scoreboard with the step at 3 cm 58.5 % (the nudges along 51), without
  64: the reflex stays behind the sole's edge until it earns its place.
- A pinned foot's press (`arrival.PRESS_M` 0.01, the lab): a foot bearing
  nothing is solved from the pelvis as it is, put exactly on the floor, and
  bears nothing: after the step the stepped foot hung 2 mm up at 0 N while
  the capture point passed over it; fell at 6.2 s. Reaching 1 cm under for
  what it does not bear: the 60 N standing shove stood from either side, one
  step, the landing a 1.3 kN spike and a 1.4 cm hop; 100 N fell from either
  side after two cross-over steps. Stand suite 74.9 % and 207 against the
  built 74.7 and 215 (its shove 120 N); whole scoreboard 721.4 and 65.8 %
  against 729 and 65.7: built.
- The 100 N standing shove (the lab): the cross-over's foot never came down;
  the step cap scaled its CROSS_M ahead to 10 cm, the foot hung 2.7 cm up 12
  cm short of its mark at 0 N. Clipped before the clearances it lands 24 cm
  ahead and still falls from either side in two steps, at the cap 0.1 or
  0.2, DWELL_S 0.15-0.4: not taken.

## Bricks

2026-10-04:

- The brick's step down lands (the left to 0.18 m, 191 N, the pelvis 6 cm
  lower) and the body leans back off both feet 1 s on.

| Variant | Result |
| --- | --- |
| stood again over 0.6 s instead of 0.3, or the pelvis target let 10 cm out | the second step a 31 cm cross-over out of reach; down at 6.1-6.7 s |
| the pelvis lowered for the brick's 6 cm only once stood again (before: dropped on the brick's leg as the other reached down, the landing bounced 0-600 N) | the landing holds at 150-250 N a foot but leans back off both, 0.17 -> 0.36 m/s, down at 6.75 |
| steps of 10 cm at most, 0.15 s apart (`stand.STEP_MAX_M`) | four of them, the body backward-left at 1 m/s by the fourth, down at 6.6 |

- Standing on the lower foot after the step, the weight runs back whatever
  the stood target: not understood.
- Read closer: after the step down the loads bounce 0-1100 N a foot for 0.4 s
  (stood 4 cm lower the same); the second step takes the brick's foot 10 cm
  back onto the brick's rear edge (the controller knows no brick) where,
  pinned, it bears 0-8 N, and the body goes on one leg to a third step and a
  fall at 6.7-6.9 s.
- The brick taken away (the lab): the freed foot steps 11 cm out and 6 cm
  down at 0.6 m/s, lands at 1.0-1.35 kN, the body bounces off both feet and
  goes over after a second step; slowed to 0.3 or 0.2 m/s the capture point
  has run 25-50 cm before it lands. Dropped straight down where it hangs (its
  own point still holding the capture point between the two): lands at
  1.2-1.3 kN and bears, and the body tips toward the dropped side faster
  than the second step at any cap 0.1-0.3, gain 0.1-0.8, dwell 0.25-0.4: not
  taken. The drop's momentum, not the step, is the gap.

## One law for standing

`machine.dcm` (2026-10-04): every special case a sign the problem needs a
more general form.

- The capture point in the sole's hold: an ellipse 4 cm across and 8 along
  about the support's point nearest it. A step is due only outside it,
  landing on the ray from the measured CoP through the capture point as it
  will be at STEP_S 0.22, STANCE_M 2 cm beyond. The foot that steps: the hung
  one, else the farther from the landing. A foot hangs bearing nothing 0.15 s
  where the CoM asks 45 % of it; any foot reaches 1 cm under for what it
  does not bear.
- Stand suite 76.5 % and 177.9 against the nine-constant reflex's 74.7 and
  215.2: the nudges, the boards and the staggered rocker 100 %, the free
  rocker 73, the shoves and the bricks 45-49; the nudges and the 60 N shoves
  stand with no step at all.

| Tried on the way | Fault |
| --- | --- |
| a landing 8 cm beyond a round 4 cm hold | returned every step as the next |
| a step called inside the hold by the time to leave | fired on 2 cm with the ankle already bringing it back |
| run from the hold's edge rather than the CoP | every step on a 120 N push landed short |
| a foot hung by its load alone | a foot a push unloads was no support; it stepped inward for nothing |

## Push polar

`events.PUSH_DEG`, a push's way an angle in its plane; `gait_montecarlo
--suite stand --grid events.PUSH_DEG=0,45
events.SHOVES.shove=60,80,100,120`, 48 pushes over 8 ways (2026-10-04).

- The one law as first built: 20 of 48 stood (60 N 11 of 12; 80 N 9 of 12,
  none of 3 along its way; 100 and 120 N none of 24). It stood what the hold
  alone holds (6.2 cm of capture point at 60 N, 8.3 at 80; the sole 8.6
  along and 11 across from its middle) and no push that asks a step.
- Why no step stood, traced a row every 25 ms:

| Fault | Fix |
| --- | --- |
| on sprung toes the stepping foot was pinned as it left the floor (the ankle 1-2 cm up, the toes still pressing 60-157 N) and never went to its landing | pinned only coming down (a keyframe's `land`) |
| pinned at its first touch, toes first and the ankle 1.2 cm up, it levelled 2 mm over the floor at 0 N | it seeks the floor from there until it bears (`bearing.SEEK_M_S`) |
| straight standing legs (the hip 0.769 m over the ankle of a 0.770 m reach; 3.8 cm of it along the floor, 17.8 at 2 cm down, 24.8 at 4, 30.0 at 6): over the front foot after a step the rear leg hung 2.4 cm off the floor, the body on one foot's toes | the pelvis no higher than both legs reach (`bearing.height`) |
| called 30 ms into a 120 ms push the landing fell 5 cm short | aimed anew in flight (`stand.retarget`) |
| the hold as an ellipse called a second step with the capture point on the landed sole | a box |
| pushed from the front, both feet's loads chattering 0-850 N on the heels: neither leg held the pelvis, which pitched 54-65 deg back in 0.15 s under a plumb torso | - |

- The landing's three forms:

| Form | Stood | Note |
| --- | --- | --- |
| on the ray from the CoP through the capture point, the steps' mechanics mended | 80-120 N 14 of 36 | from behind stood at 80, 100 and 120 N on one step; the pelvis height's variants (the stepping foot in the reach limit or not, the target within 1 cm of the pelvis or not) 11-14 of 36: chance |
| the support carried past the capture point along the way it left, the feet kept apart (a half-plane) | every push with a side to it fell | wrong: on point feet the capture point is held only on the line between them |
| on the ray from the standing foot's point (`dcm.landing`; the foot bearing less steps, the capture point running from the other alone, so a push from behind lands the foot in its own lane) | 25 of 48: 60 N 12 of 12, 80 N 10, 100 N 3 (from behind 1 of 1, from the front 2 of 2), 120 N none | the bricks 55-60 % from 48-51; boards and nudges 100; rockers 63 and 90 |

  Still down: 120 N any way, and 100 N with a side to it: the capture point
  passes the loaded foot, the free foot's landing lies across it (none, by
  `dcm.landing`), and the loaded one lifted from under the body drops it, the
  pelvis rolling 27 deg in 0.2 s.

- The trunk leant by how far the capture point stands out of the hold, as an
  offset on the spine's pitch and roll setpoints (the lab, 12 probes): 300
  and 600 deg a metre, either sign; 100 N from a side fell every time, 100 N
  along the way stood without it and fell with it 3 of 4. Not taken: a
  setpoint is not a moment; the centroidal moment needs the spine's torque,
  or the pelvis's own tilt, in the law.
- The grinder's rounds on the stand suite, the day's build (`gym.py`; each a
  best of 8-10 beside the built 77.5 %): `dcm.HOLD_M` along, `STANCE_M`,
  `STEP_S` 155.4 and 79.1 % at the built values; `dcm.CLEAR_M` along with
  `drives.TOE_K`, `TOE_LOSS` 132.3 and 81.2 % at the toes' spring 42 N
  m/rad: stiffer toes hold the stand where the walk wants them at 10.
- Walking on the standing law (the lab): the weight asked ahead of the feet's
  middle, ramped 5 cm/s, for the law to step the body on. It pivots over its
  forefeet as one piece (straight legs, the heels up, the feet's loads to 38
  and 0 N by 4.5 s); the step called at 8 cm comes 0.15 s before it is down.
  Not a walk: a step planned before the hold is lost needs a reference for
  the capture point, not a lean.

## Shares

- Each foot bears its share (the lab, `bearing.shared`): standing to stay,
  every leg a stance leg, its foot let down 0.3 m/s a share of the weight it
  lacks and drawn up as fast a share too many; the share what the CoM asks of
  it between the feet, all of it while the other steps.
  + Polar 32 of 48 against 25: 100 N 9 of 12 against 3, from the sides 3 of
    3 where none stood; 120 N 3 against 0; 60 N 9 against 12. Rigs: bricks
    96 and 63 % against 60 and 55; rockers 100 and 93 against 63 and 90.
  + From behind it falls at 60, 100 and 120 N: tipped over the standing
    foot's toes its load reads 0, the leg is let down for its lack and throws
    the body on, the foot 20 cm in the air. From a side the same throw is the
    push-off of a side step: the loaded foot lands 31 cm out, the other
    follows, three more steps and it stands, 5 cm down on knees bent 40 deg.
  + Two variants by probe: the shares on both feet standing only, a step as
    before (the rockers and a brick stand, 80 N from a side falls on five
    steps); a foot let down only where it has lost its floor (a nudge falls).
- The share law baked (`bearing.shared`): stand suite 99.3 and 89.2 % against
  237.9 and 77.5 (the 120 N shoves 66 and 67 % from 50 and 58; the bricks 96
  and 63 from 60 and 55; the rockers 100 and 100 from 63 and 90); scoreboard
  538.0 and 82.8 % against 641.7 and 78.6; the rises, walks and events as
  they were.
  + Its forms on the polar, six of 48 pushes each: as baked 32; the standing
    foot's let-down held still through a step 30; the share what the capture
    point asks 24 and 23; a step run as before the law, the shares on both
    feet standing only, 18 and 20. The rockers 100 % under every one, the
    bricks 66-96.
  + By probe its quiet standing chatters under it, a foot at 0 N one row in
    two; the lack filtered over 50 ms the feet bear 161 and 162 N.
- The loads as a sensor has them:
  + The lack filtered over 50 and 150 ms (the lab's `SHARE_S`): polar 26 and
    25 of 48 against 32 (60 N 12 and 11 of 12 against 9; the bricks 100 and
    87 % against 96 and 63; 80-120 N 14 of 36 against 23): not taken.
  + The soles' load itself through a band (`physics.LOAD_S`): standing still
    its sd 131 and 98 N -> 13 at 10 ms and 0.02 at 20; no pass at 0 N where
    104 and 88 a second. Polar 24 and 26 of 48 at 10 and 20 ms: 60 N 12 of
    12, from the sides at 80-100 N 1 and 0 of 6. The foot that stepped was
    the lighter that pass, and the loaded leg's side step that stood those
    pushes was called only where its load flickered to 0 N; read calm, the
    lighter foot's landing lies across the other and no step is taken.
  + The stepping foot one that can land where the capture point goes
    (`stand.needed`): polar 29, 30, 32 and 30 of 48 at 0, 5, 10 and 20 ms;
    at 20 ms 60 N 12, 80 N 11, 100 N 7, 120 N none; the bricks 100 and 84 %;
    the rockers 100.
  + Baked at 20 ms: scoreboard 229.0 and 86.4 % against 538.0 and 82.8 (408.6
    and 85.2 at 5 ms); every rise and walk 100 %; the soa event 100 from 54,
    the nudge 100 from 77, the rug 54 from 77, the 120 N shoves 53 and 48
    from 66 and 67.
  + By probe at 100 N: from behind it stands at 20 ms with the share's rate
    at 0.1 m/s and falls at 0.3; from a side it falls at 0.1 and stands at
    0.3; at 5 ms and 0.3 both stand. The 120 N pushes are for the rate and
    the band to find together.

## Calves

`arrival.CALF_DEG_M` (2026-10-05): nobody stands frozen; calves, thighs and
toes balance in the small, the toes of little use in a shoe. Untouched, the
capture point moved 0.0 mm a second. Nudged on the trunk for 0.1 s, its
swing past its rest and back, and when it is still:

| Nudge | Pelvis alone | Both ankles asked 96 deg a m of it to half a degree |
| --- | --- | --- |
| 40 N from behind | +29/-12 mm, 0.54 s | +26/-6, 0.50 |
| 80 N from behind | +72/-42, 1.36 | +70/-31, 0.94 |
| 40 N from the front | +13/-36, 0.61 | +9/-31, 0.53 |
| 80 N from the front | a step | a step |

The stand's scoreboard 89.0 % where 87.7 (30 runs); the staggered bricks 100
where 85.5; the stir on the nudges 0.03 mm where 0.05, on the free rocker 0.09
where 0.32. To 1 deg 87.3 %; at 150 deg/m 87.6; the pelvis's own damping at
0.35-0.45 s (`arrival.COM_D`) fell to 80 N from the front.
