# Findings: demand

Each drive's demand on her loop against its parts - the scenes, the chair's
stunts; the drives themselves in [drives](drives.md).

`tools/sim/demand.py`: each drive's amps, speed and heat on her loop as the
page runs it, a share of its parts' limits - its board's amps (48.4 A a U8,
its winding's top), what it holds for ever, its gearbox's momentary torque,
its no-load speed at 48 V, a node's span to its ceiling. Worst of the sides.

| Scene | Drive | Peak, of its amps | Passes past 2/3, % | rms, of what it holds | Gearbox | Speed | Heat |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Walk, 51 s | hip | 1.00 | 2.66 | 0.59 | 0.84 | 0.36 | 0.32 |
| | knee | 1.00 | 0.53 | 0.40 | 0.84 | 0.56 | 0.19 |
| | spine | 1.00 | 0.05 | 0.14 | 0.84 | 0.08 | 0.15 |
| | spine roll | 0.77 | 0.02 | 0.07 | 0.65 | 0.12 | 0.14 |
| | hip roll | 0.72 | 0.01 | 0.28 | 0.61 | 0.09 | 0.16 |
| | ankle pair | 0.70 | 0.01 | 0.39 | 0.59 | 0.40 | 0.18 |
| Hop | spine | 1.00 | 0.98 | 0.29 | 0.84 | 0.18 | 0.14 |
| | hip | 1.00 | 0.77 | 0.27 | 0.84 | 0.53 | 0.14 |
| | knee | 0.99 | 0.70 | 0.35 | 0.84 | 0.59 | 0.14 |
| | shoulder | 0.89 | 0.10 | 0.12 | 0.36 | 0.09 | 0.11 |
| | neck | 0.74 | 0.05 | 0.11 | 0.30 | 0.13 | 0.11 |
| 38 N, 8 ways, stood 8 | knee | 0.12 | 0 | 0.08 | 0.10 | 0.01 | 0.11 |
| 80 N, 8 ways, stood 5 | ankle pair | 0.22 | 0 | 0.14 | 0.18 | 0.02 | 0.11 |

- Her rms, her speeds and her heat are inside 2/3 everywhere; her peaks are
  not: walking and hopping her hips, knees and spine are at their clamps.
- The chair's stunts against her knees (108 N m at the board's amps, 128 the
  gearbox's momentary; 31.1 kg, thigh 0.39 and shank 0.38 m, two legs, the hip
  over the ankle): down off the seat (0.45 m and a 0.05 m hop, 3.1 m/s), a
  constant force over its best absorption (0.30 m) peaks at 125 N m a knee at
  its deepest, 108 deg; held at 108 N m throughout, its force falling as the
  knee bends, the knees take 219 J a radian, 344 over 20-110 deg against the
  275 the drop brings - met only at their peak, none of the 50 % margin. Up
  onto it (2.2 m/s at take-off from a 0.17 m push begun at 90 deg) 101 N m at
  the push's start, 0.93. Speed binds neither: the board's amps hold to
  0.93 of a knee's 960 deg/s.
- The peaks are the loop's own torques: past 2/3, its feed alone is past it
  in every pass, the PD under it never half the torque.
- The performance build (`machine/humanoids/gynoid_mesomorph_medium_performance.toml`:
  the U12 II KV120 - 7.9 N m at 48 V, 4.5 kW, 778 g - on box A for the
  spine, hips, knees and ankles, 3.7 kg more; her text extended): her hop
  asks its hips 144 A, 1.44 of their amps, the knees 114 A, 1.15, where hers
  asks 76 A, 1.58, and the spine 1.11; the box 0.93 of its momentary where
  1.33; her standing jump rises 0.03 legs on it for 0.01, 0.01 of flight
  either way, the human's 0.55 and 0.91. The hop's ask is the law's, past
  any drive: a jump needs its throw laid from what the drives give.
- Walking, a hip's are its leg's lift: the pass its sole goes from standing
  to swinging the sole is asked 33 m/s^2 at once, bearing 50 N still; the
  hip gives -108 N m for 15 ms, the knee 80, then 10 to 20 as before.
  16 ms a step, each step.
- On the bench's scenarios (`demand.py --scenarios`) the hips, knees and
  spine reach 1.45-1.58 of their board's amps and 1.2-1.34 of their gearbox's
  momentary torque: a twist from a key grants a held drive 1.45x its clamp,
  past its parts' peak, where her text's walk gets its peak. Her rms, speeds
  and heat stay under 2/3 but in the turns' play (knee rms 0.85, heat 0.87).
- The turns need it: in a right turn the inside sole's heel is asked up
  0.3 rad by the other's landing (0.1 straight), falls behind the ask, and its
  load goes 265 -> 34 N; she saturates her hips 0.43 s with both soles bare.
  Without the grant the turns' play and the weave fall at every wobble; granted
  to every driver, her walk's head goes 38 mm aside (35) and strikes 455 N.
- What unloads it: the turn reversed (left to right), her centre of mass goes
  0.145 -> 0.410 m across from the new inside foot in 0.54 s, its hip's roll
  0.09 -> 0.50 rad, its yaw held near 0 (posed where it is: no change); the
  outside foot lands at 5.94 s, after the stance has let go. Its drives are
  under half their clamps until then: a step landed too late, not a weak leg.
- Her rhythm hurried by her capture point's run sideways (off the standing
  sole's edge, as e^(omega t), due before a stance's width) never acts on her
  plain walk (0 of 6 666 passes); in the reversed turn it quickened every mode
  with the swing, her pelvis's turn among them, and she spun 1.9 rad and fell.
  A swing's timing needs a clock of its own, apart from her rhythm's angle.
- With each swing's own clock the turns hold without the grant (13 of 13 wobbles),
  the turnabout too and better (13, against 7 granted: saturation damps what her
  law over-asks there); the weave falls at 4 of 13 without it, at 1 granted only
  while a swing leads or her capture point is past a lone standing sole's own
  side - never in her plain walk, 0 of 6 666 passes. Granted by the gait under
  way, whoever said it: the bench as before, walking 80 N shoves 28 of 32.
- The standing jump's push at 2/3 of the knees' clamp (108 N m) through the
  knee's lever to the leg's line at the human's crouch (0.31 m): 466 N, 5.2
  m/s^2 net for 0.34 s, take-off 1.74 m/s ideal and 1.5-1.65 realized; his
  1.66 (11.7 axial/s). The landing's absorption at the same clamp: 1.9 axial
  units of squat for 12.3 axial/s, the whole crouch (stunts.md).
